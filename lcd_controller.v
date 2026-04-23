// ============================================================================
// lcd_controller.v
// Drives the DE2 on-board 16x2 HD44780 character LCD.
// Implements:
//   - Power-on init sequence (function set, display on, entry mode, clear)
//   - Per-state message (two 16-char lines)
//   - Refresh whenever game_state changes
//
// Runs on clk_50 (50 MHz) so we can count wall-clock time accurately.
// HD44780 timing requirements:
//   - tAS (RS/RW setup to EN rise)    >= 40 ns
//   - PWEH (EN high pulse)            >= 230 ns
//   - tAH (data hold after EN fall)   >= 10 ns
//   - per-command wait                >= 40 us (2 ms for clear/home)
// At 50 MHz (20 ns period), we use generous counts.
// ============================================================================
module lcd_controller (
    input  wire        clk_50,
    input  wire        rst,
    input  wire [2:0]  game_state,
    output reg  [7:0]  LCD_DATA,
    output reg         LCD_EN,
    output reg         LCD_RS,
    output reg         LCD_RW,
    output wire        LCD_ON,
    output wire        LCD_BLON
);

    // Power & backlight always on
    assign LCD_ON   = 1'b1;
    assign LCD_BLON = 1'b1;

    // Game-state encoding (must match game_fsm.v)
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_PLAYING  = 3'd1;
    localparam [2:0] S_COLLIDED = 3'd2;
    localparam [2:0] S_AT_STOP  = 3'd3;
    localparam [2:0] S_PARKING  = 3'd4;
    localparam [2:0] S_DONE     = 3'd5;

    // --- Timing constants @ 50 MHz ---
    localparam integer T_15MS   = 32'd750_000;  // 15 ms power-on wait
    localparam integer T_5MS    = 32'd250_000;
    localparam integer T_2MS    = 32'd100_000;  // clear/home delay
    localparam integer T_50US   = 32'd2_500;    // generic command delay
    localparam integer T_EN_HI  = 32'd30;       // ~600 ns EN pulse high
    localparam integer T_EN_LO  = 32'd30;

    // --- Driver FSM states ---
    localparam [4:0] D_POWER_WAIT = 5'd0;
    localparam [4:0] D_INIT_LOAD  = 5'd1;
    localparam [4:0] D_INIT_RS    = 5'd2;
    localparam [4:0] D_INIT_ENHI  = 5'd3;
    localparam [4:0] D_INIT_ENLO  = 5'd4;
    localparam [4:0] D_INIT_WAIT  = 5'd5;
    localparam [4:0] D_INIT_NEXT  = 5'd6;
    localparam [4:0] D_MSG_LOAD   = 5'd7;
    localparam [4:0] D_MSG_RS     = 5'd8;
    localparam [4:0] D_MSG_ENHI   = 5'd9;
    localparam [4:0] D_MSG_ENLO   = 5'd10;
    localparam [4:0] D_MSG_WAIT   = 5'd11;
    localparam [4:0] D_MSG_NEXT   = 5'd12;
    localparam [4:0] D_HOLD       = 5'd13;

    reg [4:0]  dstate;
    reg [31:0] timer;

    // --- Init command sequence ---
    // 0x38 Function Set: 8-bit, 2 line, 5x8 font
    // 0x0C Display ON, cursor OFF, blink OFF
    // 0x06 Entry mode: increment, no shift
    // 0x01 Clear display
    // 0x80 Set DDRAM addr 0 (line 1)
    localparam integer INIT_LEN = 5;
    reg [2:0] init_idx;
    reg [7:0] init_cmd;
    always @(*) begin
        case (init_idx)
            3'd0: init_cmd = 8'h38;
            3'd1: init_cmd = 8'h0C;
            3'd2: init_cmd = 8'h06;
            3'd3: init_cmd = 8'h01;
            3'd4: init_cmd = 8'h80;
            default: init_cmd = 8'h00;
        endcase
    end

    // --- Message ROM ---
    // 32 characters per state: 16 for line 1, 16 for line 2.
    // Index 0..15 = line1, 17..32 = line2 (index 16 = set DDRAM 0xC0 cmd)
    // We store 32 chars and issue a 0xC0 command between them.
    reg [5:0]  msg_idx;          // 0..33 (32 chars + 2 cmd slots)

    // Per-state message text (each 32 chars, padded with spaces)
    //         "0123456789ABCDEF"
    // IDLE :  "Press SW17 Start"   " GEL372 - Ready "
    // PLAY :  "Driving...      "   " Follow the road"
    // COLL :  "!! Collision !! "   " Off the track! "
    // STOP :  "STOP zone       "   " Come to a halt "
    // PARK :  "Parking zone    "   " Brake to stop  "
    // DONE :  " *** PASS ***   "   " Press KEY3 rst "  (or FAIL)
    //
    // We store the two 16-char rows and index into them.

    // Pass signal isn't directly visible -- we approximate with game_state only.
    // (Pass/fail distinction at S_DONE is optional; we show PASS if arrived here.)

    // Helper: returns the ASCII char at position (line 0..1, col 0..15) for state
    function [7:0] ascii_of;
        input [2:0] st;
        input       line;  // 0 or 1
        input [3:0] col;
        reg [127:0] row0, row1;  // 16 chars * 8 bits = 128
        begin
            case (st)
                S_IDLE: begin
                    row0 = "Press SW17 Start";
                    row1 = " GEL372 - Ready ";
                end
                S_PLAYING: begin
                    row0 = "Driving...      ";
                    row1 = " Follow the road";
                end
                S_COLLIDED: begin
                    row0 = "!! Collision !! ";
                    row1 = " Off the track! ";
                end
                S_AT_STOP: begin
                    row0 = "STOP zone       ";
                    row1 = " Come to a halt ";
                end
                S_PARKING: begin
                    row0 = "Parking zone    ";
                    row1 = " Brake to stop  ";
                end
                S_DONE: begin
                    row0 = " *** PASS ***   ";
                    row1 = " Press KEY3 rst ";
                end
                default: begin
                    row0 = "                ";
                    row1 = "                ";
                end
            endcase
            // Extract byte: MSB is leftmost char. char index 'col' from left.
            // For a 128-bit packed string, leftmost char lives at bits [127:120].
            // So char at col i = bits [127 - 8*i -: 8].
            if (line == 1'b0)
                ascii_of = row0[127 - 8*col -: 8];
            else
                ascii_of = row1[127 - 8*col -: 8];
        end
    endfunction

    // --- State tracker for re-issue on state change ---
    reg [2:0] last_state;

    // --- Main FSM ---
    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            dstate     <= D_POWER_WAIT;
            timer      <= 32'd0;
            init_idx   <= 3'd0;
            msg_idx    <= 6'd0;
            LCD_EN     <= 1'b0;
            LCD_RS     <= 1'b0;
            LCD_RW     <= 1'b0;
            LCD_DATA   <= 8'h00;
            last_state <= 3'd7;  // impossible state -> forces initial refresh
        end
        else begin
            case (dstate)

                // --- 1. Wait 15 ms after power-on ---
                D_POWER_WAIT: begin
                    LCD_EN <= 1'b0;
                    if (timer == T_15MS) begin
                        timer    <= 0;
                        init_idx <= 0;
                        dstate   <= D_INIT_LOAD;
                    end
                    else timer <= timer + 1;
                end

                // --- 2. Send init commands ---
                D_INIT_LOAD: begin
                    LCD_RS   <= 1'b0;  // command
                    LCD_RW   <= 1'b0;  // write
                    LCD_DATA <= init_cmd;
                    dstate   <= D_INIT_RS;
                    timer    <= 0;
                end
                D_INIT_RS: begin
                    // Small setup time before EN rises
                    if (timer == 5) begin timer <= 0; dstate <= D_INIT_ENHI; end
                    else timer <= timer + 1;
                end
                D_INIT_ENHI: begin
                    LCD_EN <= 1'b1;
                    if (timer == T_EN_HI) begin timer <= 0; dstate <= D_INIT_ENLO; end
                    else timer <= timer + 1;
                end
                D_INIT_ENLO: begin
                    LCD_EN <= 1'b0;
                    if (timer == T_EN_LO) begin timer <= 0; dstate <= D_INIT_WAIT; end
                    else timer <= timer + 1;
                end
                D_INIT_WAIT: begin
                    // Clear cmd needs 2 ms; others 50 us.
                    if (init_cmd == 8'h01) begin
                        if (timer == T_2MS) begin timer <= 0; dstate <= D_INIT_NEXT; end
                        else timer <= timer + 1;
                    end
                    else begin
                        if (timer == T_50US) begin timer <= 0; dstate <= D_INIT_NEXT; end
                        else timer <= timer + 1;
                    end
                end
                D_INIT_NEXT: begin
                    if (init_idx == INIT_LEN - 1) begin
                        // Init done, begin first message
                        msg_idx    <= 6'd0;
                        last_state <= game_state;
                        dstate     <= D_MSG_LOAD;
                    end
                    else begin
                        init_idx <= init_idx + 1;
                        dstate   <= D_INIT_LOAD;
                    end
                end

                // --- 3. Send message bytes ---
                D_MSG_LOAD: begin
                    // msg_idx:
                    //   0..15 : line 1 chars (col 0..15)
                    //   16    : set DDRAM 0xC0 (line 2 start)
                    //   17..32: line 2 chars (col 0..15)
                    //   33    : return -> hold
                    LCD_RS   <= (msg_idx < 6'd16) ? 1'b1 :
                                (msg_idx == 6'd16) ? 1'b0 :
                                (msg_idx < 6'd33) ? 1'b1 : 1'b0;
                    LCD_RW   <= 1'b0;
                    LCD_DATA <= (msg_idx < 6'd16) ? ascii_of(game_state, 1'b0, msg_idx[3:0]) :
                                (msg_idx == 6'd16) ? 8'hC0 :
                                (msg_idx < 6'd33) ? ascii_of(game_state, 1'b1, (msg_idx - 6'd17)) : 8'h20;
                    timer  <= 0;
                    if (msg_idx >= 6'd33) dstate <= D_HOLD;
                    else                  dstate <= D_MSG_RS;
                end
                D_MSG_RS: begin
                    if (timer == 5) begin timer <= 0; dstate <= D_MSG_ENHI; end
                    else timer <= timer + 1;
                end
                D_MSG_ENHI: begin
                    LCD_EN <= 1'b1;
                    if (timer == T_EN_HI) begin timer <= 0; dstate <= D_MSG_ENLO; end
                    else timer <= timer + 1;
                end
                D_MSG_ENLO: begin
                    LCD_EN <= 1'b0;
                    if (timer == T_EN_LO) begin timer <= 0; dstate <= D_MSG_WAIT; end
                    else timer <= timer + 1;
                end
                D_MSG_WAIT: begin
                    if (timer == T_50US) begin timer <= 0; dstate <= D_MSG_NEXT; end
                    else timer <= timer + 1;
                end
                D_MSG_NEXT: begin
                    msg_idx <= msg_idx + 1;
                    dstate  <= D_MSG_LOAD;
                end

                // --- 4. Hold until state changes, then refresh ---
                D_HOLD: begin
                    LCD_EN <= 1'b0;
                    if (game_state != last_state) begin
                        // Send clear + home then rewrite
                        last_state <= game_state;
                        init_idx   <= 3'd3;          // 0x01 clear
                        dstate     <= D_INIT_LOAD;
                    end
                end

                default: dstate <= D_POWER_WAIT;
            endcase
        end
    end

endmodule
