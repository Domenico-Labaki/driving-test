// ============================================================================
// audio_controller.v
// Full WM8731 audio codec driver for the Altera DE2 board.
//
// Two sub-systems:
//   1. I2C (SDAT/SCLK) config at power-on -- sends WM8731 register writes
//      to turn on the DAC path, set sample rate, unmute, etc.
//   2. I2S transmit: generates BCK (bit clock), DACLRCK (word select) and
//      DACDAT from an internal tone generator. Also drives XCK (master clock)
//      from the incoming 18.432 MHz clock (we use CLOCK_50/2 ~ close enough
//      for USB-speed operation; DE2 also exposes a dedicated AUD_XCK pin).
//
// Simplifications:
//   - 48 kHz sample rate, 16-bit, I2S (left-justified with 1-cycle delay)
//   - Mono: same sample on both channels
//   - Tone generator: square wave at collision (220 Hz), success (880 Hz),
//     tick (440 Hz during normal play). No sound during IDLE.
//
// Ports match DE2 reference names exactly.
// ============================================================================
module audio_controller (
    input  wire        clk_50,               // 50 MHz master
    input  wire        rst,
    input  wire [2:0]  game_state,
    input  wire        collision_event,      // single-tick pulse on collision edge
    input  wire        pass,

    // WM8731 I2C pins
    output reg         I2C_SCLK,
    inout  wire        I2C_SDAT,

    // WM8731 I2S pins
    output wire        AUD_XCK,              // master clock to codec
    output reg         AUD_BCLK,             // bit clock
    output reg         AUD_DACLRCK,          // L/R word select
    output reg         AUD_DACDAT            // serial audio data
);

    // ------------------------------------------------------------------------
    //  Game state codes (must match game_fsm.v)
    // ------------------------------------------------------------------------
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_PLAYING  = 3'd1;
    localparam [2:0] S_COLLIDED = 3'd2;
    localparam [2:0] S_AT_STOP  = 3'd3;
    localparam [2:0] S_PARKING  = 3'd4;
    localparam [2:0] S_DONE     = 3'd5;

    // ------------------------------------------------------------------------
    //  AUD_XCK: divide 50 MHz by 2 -> 25 MHz master clock.
    //  WM8731 can accept up to 18.432 MHz XCK for 48 kHz operation, but
    //  with USB-mode config the codec has an internal PLL and accepts
    //  12 MHz reliably. For robustness we produce 12.5 MHz by dividing by 4.
    // ------------------------------------------------------------------------
    reg [1:0] xck_div;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) xck_div <= 2'd0;
        else     xck_div <= xck_div + 2'd1;
    end
    assign AUD_XCK = xck_div[1];  // 50/4 = 12.5 MHz

    // ------------------------------------------------------------------------
    //  I2C controller for WM8731 register configuration.
    //  Device address: 0x34 (write).  Each register write is 3 bytes:
    //    byte0 = 0x34 (addr + W)
    //    byte1 = {reg_addr[6:0], data[8]}
    //    byte2 = data[7:0]
    // ------------------------------------------------------------------------

    // Register list (WM8731 datasheet):
    //   R6  0x0000 power up all (except OUT off)   -> 0x0C 0x00
    //   R0  left line in:  0x17                    -> 0x00 0x17
    //   R1  right line in: 0x17                    -> 0x02 0x17
    //   R2  left hp:       0x79 (0 dB)             -> 0x04 0x79
    //   R3  right hp:      0x79                    -> 0x06 0x79
    //   R4  analogue path: 0x10 (DAC -> LHPOUT)    -> 0x08 0x10
    //   R5  digital path:  0x00 (DAC unmuted)      -> 0x0A 0x00
    //   R7  digital format: 0x02 (I2S, 16-bit)     -> 0x0E 0x02
    //   R8  sampling ctrl: 0x00 (USB mode, 48k)    -> 0x10 0x00
    //   R9  active: 0x01                           -> 0x12 0x01

    localparam integer N_CMD = 10;
    reg [15:0] cmd_rom [0:N_CMD-1];
    initial begin
        cmd_rom[0] = 16'h0C_00;   // R6 power
        cmd_rom[1] = 16'h00_17;   // R0 L line in
        cmd_rom[2] = 16'h02_17;   // R1 R line in
        cmd_rom[3] = 16'h04_79;   // R2 L HP
        cmd_rom[4] = 16'h06_79;   // R3 R HP
        cmd_rom[5] = 16'h08_10;   // R4 analogue path
        cmd_rom[6] = 16'h0A_00;   // R5 digital path
        cmd_rom[7] = 16'h0E_02;   // R7 digital format (I2S, 16b)
        cmd_rom[8] = 16'h10_00;   // R8 sampling (USB, 48 kHz)
        cmd_rom[9] = 16'h12_01;   // R9 active
    end

    // I2C clock period @ ~100 kHz from 50 MHz = 500 ticks
    localparam integer I2C_DIV = 250;  // half-period
    reg [8:0] i2c_tick;
    reg       i2c_phase;     // toggles every I2C_DIV ticks
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin i2c_tick <= 0; i2c_phase <= 0; end
        else if (i2c_tick == I2C_DIV-1) begin
            i2c_tick  <= 0;
            i2c_phase <= ~i2c_phase;
        end
        else i2c_tick <= i2c_tick + 1;
    end
    wire i2c_tick_edge = (i2c_tick == I2C_DIV-1);

    // SDAT is open-drain: we only drive 0, else tri-state (release to pull-up)
    reg sdat_drive_low;
    assign I2C_SDAT = sdat_drive_low ? 1'b0 : 1'bz;

    // I2C controller FSM
    localparam [3:0] I_IDLE  = 4'd0,
                     I_START = 4'd1,
                     I_BYTE  = 4'd2,
                     I_ACK   = 4'd3,
                     I_STOP1 = 4'd4,
                     I_STOP2 = 4'd5,
                     I_NEXT  = 4'd6,
                     I_DONE  = 4'd7,
                     I_WAIT  = 4'd8;

    reg [3:0]  istate;
    reg [3:0]  cmd_idx;
    reg [1:0]  byte_idx;    // 0..2 (addr, reg, data)
    reg [3:0]  bit_idx;     // 0..8 (8 data + 1 ack slot)
    reg [7:0]  shift;

    // Pre-power-up wait ~100 ms so codec is stable
    reg [23:0] boot_cnt;

    // For loading shift register with the next byte
    wire [7:0] cur_byte =
        (byte_idx == 2'd0) ? 8'h34 :                       // codec I2C addr
        (byte_idx == 2'd1) ? cmd_rom[cmd_idx][15:8] :
                             cmd_rom[cmd_idx][7:0];

    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            istate         <= I_IDLE;
            cmd_idx        <= 0;
            byte_idx       <= 0;
            bit_idx        <= 0;
            shift          <= 0;
            I2C_SCLK       <= 1'b1;
            sdat_drive_low <= 1'b0;
            boot_cnt       <= 0;
        end
        else begin
            case (istate)
                I_IDLE: begin
                    I2C_SCLK       <= 1'b1;
                    sdat_drive_low <= 1'b0;
                    if (boot_cnt == 24'd5_000_000) begin   // 100 ms
                        istate  <= I_START;
                        cmd_idx <= 0;
                    end
                    else boot_cnt <= boot_cnt + 1;
                end

                I_START: begin
                    // SDA falls while SCL is high
                    if (i2c_tick_edge) begin
                        sdat_drive_low <= 1'b1;  // pull SDA low
                        I2C_SCLK       <= 1'b1;
                        byte_idx       <= 0;
                        bit_idx        <= 0;
                        shift          <= 8'h34;  // first byte: addr+W
                        istate         <= I_BYTE;
                    end
                end

                I_BYTE: begin
                    // Transmit 8 bits MSB first. SCL low -> set data; SCL high -> sampled.
                    if (i2c_tick_edge) begin
                        if (!i2c_phase) begin
                            // Just about to go SCL low: update data
                            I2C_SCLK       <= 1'b0;
                            sdat_drive_low <= ~shift[7];  // drive low when data bit is 0
                        end
                        else begin
                            // SCL going high: data sampled
                            I2C_SCLK <= 1'b1;
                            if (bit_idx == 4'd7) begin
                                bit_idx <= 0;
                                istate  <= I_ACK;
                            end
                            else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx + 1;
                            end
                        end
                    end
                end

                I_ACK: begin
                    if (i2c_tick_edge) begin
                        if (!i2c_phase) begin
                            // Release SDA for slave ACK
                            I2C_SCLK       <= 1'b0;
                            sdat_drive_low <= 1'b0;
                        end
                        else begin
                            I2C_SCLK <= 1'b1;
                            // Move to next byte or stop
                            if (byte_idx == 2'd2) begin
                                istate <= I_STOP1;
                            end
                            else begin
                                byte_idx <= byte_idx + 1;
                                shift    <= (byte_idx == 2'd0) ?
                                              cmd_rom[cmd_idx][15:8] :
                                              cmd_rom[cmd_idx][7:0];
                                istate <= I_BYTE;
                            end
                        end
                    end
                end

                I_STOP1: begin
                    if (i2c_tick_edge) begin
                        if (!i2c_phase) begin
                            I2C_SCLK       <= 1'b0;
                            sdat_drive_low <= 1'b1;   // SDA low
                        end
                        else begin
                            I2C_SCLK <= 1'b1;
                            istate   <= I_STOP2;
                        end
                    end
                end
                I_STOP2: begin
                    if (i2c_tick_edge) begin
                        sdat_drive_low <= 1'b0;       // SDA releases high = STOP
                        istate         <= I_NEXT;
                    end
                end

                I_NEXT: begin
                    if (cmd_idx == N_CMD - 1) istate <= I_DONE;
                    else begin
                        cmd_idx <= cmd_idx + 1;
                        istate  <= I_WAIT;
                        boot_cnt <= 0;
                    end
                end

                I_WAIT: begin
                    // Inter-command delay ~1 ms
                    if (boot_cnt == 24'd50_000) istate <= I_START;
                    else                         boot_cnt <= boot_cnt + 1;
                end

                I_DONE: begin
                    // Codec is configured. Stay here forever (unless reset).
                    I2C_SCLK       <= 1'b1;
                    sdat_drive_low <= 1'b0;
                end

                default: istate <= I_IDLE;
            endcase
        end
    end

    wire i2c_done = (istate == I_DONE);

    // ------------------------------------------------------------------------
    //  I2S audio transmit.
    //  Sample rate: XCK / 256 = 12.5MHz / 256 ~= 48.828 kHz (close to 48k)
    //  BCLK: XCK / 4 = 3.125 MHz (64 BCLK per LRCLK period at 48.8 kHz)
    // ------------------------------------------------------------------------

    // BCLK: divide clk_50 by 16 -> 3.125 MHz. Toggle on half-period (8 cycles).
    reg [3:0] bclk_cnt;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            bclk_cnt <= 0;
            AUD_BCLK <= 0;
        end
        else begin
            if (bclk_cnt == 4'd7) begin
                bclk_cnt <= 0;
                AUD_BCLK <= ~AUD_BCLK;
            end
            else bclk_cnt <= bclk_cnt + 1;
        end
    end

    // Detect BCLK falling edge (in clk_50 domain) for shifting data
    reg bclk_prev;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) bclk_prev <= 0;
        else     bclk_prev <= AUD_BCLK;
    end
    wire bclk_falling = bclk_prev & ~AUD_BCLK;

    // LRCLK toggles every 32 BCLK cycles (16-bit per channel)
    reg [4:0] lrclk_cnt;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            lrclk_cnt   <= 0;
            AUD_DACLRCK <= 0;
        end
        else if (bclk_falling) begin
            if (lrclk_cnt == 5'd31) begin
                lrclk_cnt   <= 0;
                AUD_DACLRCK <= ~AUD_DACLRCK;
            end
            else lrclk_cnt <= lrclk_cnt + 1;
        end
    end

    // Detect LRCLK edge (in clk_50 domain) to latch a new sample
    reg lrclk_prev;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) lrclk_prev <= 0;
        else     lrclk_prev <= AUD_DACLRCK;
    end
    wire lrclk_edge = lrclk_prev ^ AUD_DACLRCK;

    // ------------------------------------------------------------------------
    //  Tone generator: square wave whose frequency depends on game event.
    //  Sample is a 16-bit signed square wave.
    //  Amplitude 8000 (~quarter of full scale) to avoid clipping.
    // ------------------------------------------------------------------------

    // Tone selector
    // Divider-value table: sample_rate / (2*freq)
    //   collision 220 Hz -> 48828 / 440   ~= 111
    //   tick      440 Hz -> 48828 / 880   ~= 55
    //   success   880 Hz -> 48828 / 1760  ~= 28
    reg [7:0] tone_div;
    always @(*) begin
        case (game_state)
            S_COLLIDED: tone_div = 8'd111;
            S_DONE:     tone_div = pass ? 8'd28 : 8'd111;
            S_AT_STOP:  tone_div = 8'd55;
            S_PARKING:  tone_div = 8'd55;
            S_PLAYING:  tone_div = 8'd0;   // silent during normal play
            default:    tone_div = 8'd0;
        endcase
    end

    reg [7:0]  tone_cnt;
    reg        tone_bit;
    reg signed [15:0] sample;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            tone_cnt <= 0;
            tone_bit <= 0;
            sample   <= 16'sd0;
        end
        else if (lrclk_edge) begin
            if (tone_div == 0) begin
                sample   <= 16'sd0;     // silence
                tone_cnt <= 0;
                tone_bit <= 0;
            end
            else if (tone_cnt >= tone_div) begin
                tone_cnt <= 0;
                tone_bit <= ~tone_bit;
                sample   <= tone_bit ? 16'sd8000 : -16'sd8000;
            end
            else begin
                tone_cnt <= tone_cnt + 1;
            end
        end
    end

    // Suppress unused warning on collision_event (reserved for one-shot FX)
    wire _unused_ok = &{1'b0, collision_event, 1'b0};

    // ------------------------------------------------------------------------
    //  Shift register for serialising sample onto AUD_DACDAT (MSB first).
    //  I2S format with 1-cycle delay: data changes on BCLK falling edge.
    // ------------------------------------------------------------------------
    reg [15:0] sh;
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            sh          <= 16'd0;
            AUD_DACDAT  <= 1'b0;
        end
        else if (!i2c_done) begin
            AUD_DACDAT <= 1'b0;
        end
        else if (lrclk_edge) begin
            sh <= sample;         // load new sample at each LRCLK edge
        end
        else if (bclk_falling) begin
            AUD_DACDAT <= sh[15];
            sh         <= {sh[14:0], 1'b0};
        end
    end

endmodule
