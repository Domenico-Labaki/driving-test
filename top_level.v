// ============================================================================
// top_level.v
// Master wiring file for the GEL372 FPGA Driving Test Simulator.
// Port names match the Altera DE2 board reference so the .qsf file
// applies cleanly.
// ============================================================================
module top_level (
    // Clock
    input  wire         CLOCK_50,

    // User inputs
    input  wire [3:0]   KEY,
    input  wire [17:0]  SW,

    // VGA (ADV7123 DAC: 10-bit per channel)
    output wire [9:0]   VGA_R,
    output wire [9:0]   VGA_G,
    output wire [9:0]   VGA_B,
    output wire         VGA_HS,
    output wire         VGA_VS,
    output wire         VGA_CLK,
    output wire         VGA_BLANK,
    output wire         VGA_SYNC,

    // Seven-seg displays
    output wire [6:0]   HEX0,
    output wire [6:0]   HEX1,
    output wire [6:0]   HEX2,
    output wire [6:0]   HEX3,
    output wire [6:0]   HEX4,
    output wire [6:0]   HEX5,

    // LCD
    output wire [7:0]   LCD_DATA,
    output wire         LCD_EN,
    output wire         LCD_RS,
    output wire         LCD_RW,
    output wire         LCD_ON,
    output wire         LCD_BLON,

    // LEDs
    output wire [17:0]  LEDR,
    output wire [8:0]   LEDG,

    // Audio codec (WM8731)
    inout  wire         I2C_SDAT,
    output wire         I2C_SCLK,
    output wire         AUD_XCK,
    output wire         AUD_BCLK,
    output wire         AUD_DACLRCK,
    output wire         AUD_DACDAT
);

    // ------------------------------------------------------------------------
    // Synchronised active-high reset: KEY[3] is active-low (pressed = 0)
    // ------------------------------------------------------------------------
    wire rst_raw = ~KEY[3];
    reg [1:0] rst_sync;
    always @(posedge CLOCK_50) rst_sync <= {rst_sync[0], rst_raw};
    wire rst = rst_sync[1];

    // ------------------------------------------------------------------------
    // Internal wires
    // ------------------------------------------------------------------------
    wire        clk_25, clk_game;
    wire        steer_left, steer_right, accel, brake;
    wire [9:0]  veh_x;
    wire [8:0]  veh_y;
    wire [2:0]  speed;
    wire [1:0]  direction;
    wire        collision_detected;
    wire        at_start_line;
    wire        in_stop_zone;
    wire        in_parking_zone;
    wire [2:0]  game_state;
    wire        pass;
    wire [2:0]  lives;
    wire [5:0]  countdown_sec;

    // Collision rising-edge pulse (1 tick of clk_game)
    reg col_prev_r;
    always @(posedge clk_game or posedge rst) begin
        if (rst) col_prev_r <= 0;
        else     col_prev_r <= collision_detected;
    end
    wire collision_event = collision_detected & ~col_prev_r;

    // PLAYING-state strobe for vehicle_controller
    localparam [2:0] S_PLAYING  = 3'd1;
    localparam [2:0] S_COLLIDED = 3'd2;
    localparam [2:0] S_AT_STOP  = 3'd3;
    localparam [2:0] S_PARKING  = 3'd4;

    wire game_active = (game_state == S_PLAYING) ||
                       (game_state == S_AT_STOP) ||
                       (game_state == S_PARKING);
    wire freeze      = (game_state == S_COLLIDED);

    // ------------------------------------------------------------------------
    // ADV7123 DAC control signals
    // ------------------------------------------------------------------------
    assign VGA_CLK     = clk_25;   // DAC samples on rising edge of pixel clock
    assign VGA_BLANK = 1'b1;     // Never blank (controller handles it via RGB=0)
    assign VGA_SYNC  = 1'b0;     // Sync-on-green disabled

    // ------------------------------------------------------------------------
    // 4-bit -> 10-bit colour expansion
    // Replicate the 4 MSBs into bits [9:6] and pad [5:0] by repeating,
    // so that 4'hF -> 10'h3FF (full white) and 4'h0 -> 10'h000 (black).
    // ------------------------------------------------------------------------
    wire [3:0] vga_r4, vga_g4, vga_b4;

    assign VGA_R = {vga_r4, vga_r4, 2'b00};
    assign VGA_G = {vga_g4, vga_g4, 2'b00};
    assign VGA_B = {vga_b4, vga_b4, 2'b00};

    // ------------------------------------------------------------------------
    // Module instantiations
    // ------------------------------------------------------------------------
    clock_divider u_clk (
        .clk_50   (CLOCK_50),
        .rst      (rst),
        .clk_25   (clk_25),
        .clk_game (clk_game)
    );

    input_handler u_inp (
        .clk         (clk_game),
        .rst         (rst),
        .KEY_raw     (KEY),
        .SW_raw      (SW[1:0]),
        .steer_left  (steer_left),
        .steer_right (steer_right),
        .accel       (accel),
        .brake       (brake)
    );

    game_fsm u_fsm (
        .clk                (clk_game),
        .rst                (rst),
        .SW_en              (SW[17]),
        .collision_detected (collision_detected),
        .at_start_line      (at_start_line),
        .in_stop_zone       (in_stop_zone),
        .in_parking_zone    (in_parking_zone),
        .speed              (speed),
        .game_state         (game_state),
        .pass               (pass),
        .lives              (lives),
        .countdown_sec      (countdown_sec)
    );

    vehicle_controller u_veh (
        .clk_game    (clk_game),
        .rst         (rst),
        .game_active (game_active),
        .freeze      (freeze),
        .accel       (accel),
        .brake       (brake),
        .steer_left  (steer_left),
        .steer_right (steer_right),
        .veh_x       (veh_x),
        .veh_y       (veh_y),
        .speed       (speed),
        .direction   (direction)
    );

    collision_detector u_col (
        .veh_x              (veh_x),
        .veh_y              (veh_y),
        .collision_detected (collision_detected),
        .at_start_line      (at_start_line),
        .in_stop_zone       (in_stop_zone),
        .in_parking_zone    (in_parking_zone)
    );

    vga_controller u_vga (
        .clk_25     (clk_25),
        .rst        (rst),
        .veh_x      (veh_x),
        .veh_y      (veh_y),
        .direction  (direction),
        .game_state (game_state),
        .VGA_HS     (VGA_HS),
        .VGA_VS     (VGA_VS),
        .VGA_R      (vga_r4),
        .VGA_G      (vga_g4),
        .VGA_B      (vga_b4)
    );

    score_display u_seg (
        .clk         (clk_game),
        .rst         (rst),
        .speed       (speed),
        .countdown_sec (countdown_sec),
        .HEX0        (HEX0),
        .HEX1        (HEX1),
        .HEX2        (HEX2),
        .HEX3        (HEX3),
        .HEX4        (HEX4),
        .HEX5        (HEX5)
    );

    lcd_controller u_lcd (
        .clk_50     (CLOCK_50),
        .rst        (rst),
        .game_state (game_state),
        .LCD_DATA   (LCD_DATA),
        .LCD_EN     (LCD_EN),
        .LCD_RS     (LCD_RS),
        .LCD_RW     (LCD_RW),
        .LCD_ON     (LCD_ON),
        .LCD_BLON   (LCD_BLON)
    );

    led_controller u_led (
        .clk                (clk_game),
        .rst                (rst),
        .collision_detected (collision_detected),
        .in_parking_zone    (in_parking_zone),
        .in_stop_zone       (in_stop_zone),
        .game_state         (game_state),
        .speed              (speed),
        .pass               (pass),
        .LEDR               (LEDR),
        .LEDG               (LEDG)
    );

    audio_controller u_aud (
        .clk_50          (CLOCK_50),
        .rst             (rst),
        .game_state      (game_state),
        .collision_event (collision_event),
        .pass            (pass),
        .I2C_SCLK        (I2C_SCLK),
        .I2C_SDAT        (I2C_SDAT),
        .AUD_XCK         (AUD_XCK),
        .AUD_BCLK        (AUD_BCLK),
        .AUD_DACLRCK     (AUD_DACLRCK),
        .AUD_DACDAT      (AUD_DACDAT)
    );

    // Suppress unused warning on lives
    wire _u_lives = |lives;

endmodule
