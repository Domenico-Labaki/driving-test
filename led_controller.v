// ============================================================================
// led_controller.v
// Reactive feedback on LEDR (red) and LEDG (green).
//   - LEDR[17:0] : all on during collision; flashes during COLLIDED state
//   - LEDG[0]    : on when in parking zone
//   - LEDG[7:1]  : speed bar (thermometer, 7 levels)
//   - LEDG[8]    : on when PASS at S_DONE
// ============================================================================
module led_controller (
    input  wire        clk,
    input  wire        rst,
    input  wire        collision_detected,
    input  wire        in_parking_zone,
    input  wire        in_stop_zone,
    input  wire [2:0]  game_state,
    input  wire [2:0]  speed,
    input  wire        pass,
    output reg  [17:0] LEDR,
    output reg  [8:0]  LEDG
);

    localparam [2:0] S_COLLIDED = 3'd2;
    localparam [2:0] S_DONE     = 3'd5;
    localparam [2:0] S_GAME_OVER= 3'd6;

    // Blink counter for flashing during COLLIDED
    reg [4:0] blink_cnt;
    always @(posedge clk or posedge rst) begin
        if (rst) blink_cnt <= 0;
        else     blink_cnt <= blink_cnt + 1;
    end
    wire blink = blink_cnt[4];  // ~2 Hz at 60 Hz game clock

    // Thermometer-code the speed bar
    wire [6:0] speed_kmh = {4'd0, speed} * 7'd10;  // 00,10,...,70
    reg [6:0] speed_bar;
    always @(*) begin
        if (speed_kmh == 0)         speed_bar = 7'b0000000;
        else if (speed_kmh <= 10)   speed_bar = 7'b0000001;
        else if (speed_kmh <= 20)   speed_bar = 7'b0000011;
        else if (speed_kmh <= 30)   speed_bar = 7'b0000111;
        else if (speed_kmh <= 40)   speed_bar = 7'b0001111;
        else if (speed_kmh <= 50)   speed_bar = 7'b0011111;
        else if (speed_kmh <= 60)   speed_bar = 7'b0111111;
        else                        speed_bar = 7'b1111111;
    end

    always @(*) begin
        LEDR = 18'h0;
        LEDG = 9'h0;

        // Red indicators
        if (game_state == S_COLLIDED) LEDR = blink ? 18'h3FFFF : 18'h0;
        else if (game_state == S_GAME_OVER) LEDR = 18'h3FFFF;
        else if (collision_detected)  LEDR = 18'h3FFFF;
        else if (in_stop_zone)        LEDR = 18'h00FF0;   // center band while at STOP

        // Green indicators
        LEDG[7:1] = speed_bar;
        if (in_parking_zone)                LEDG[0] = 1'b1;
        if ((game_state == S_DONE) && pass) LEDG    = 9'h1FF;
    end

endmodule
