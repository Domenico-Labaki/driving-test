// ============================================================================
// vga_controller.v
// Standard 640x480 @ 60 Hz VGA timing from a 25 MHz pixel clock.
// Draws the track (same rectangles used by collision_detector), the STOP
// and PARK zones, the movable car, and a set of cones on the boundary.
//
// Sync polarity for 640x480@60Hz (VESA): HS and VS are both NEGATIVE
// polarity (active-low), so VGA_HS=0 during the sync pulse.
//
// Outputs 4-bit RGB. Top-level expands to 10-bit for the ADV7123 DAC.
// ============================================================================
module vga_controller (
    input  wire       clk_25,
    input  wire       rst,
    input  wire [9:0] veh_x,
    input  wire [8:0] veh_y,
    input  wire [1:0] direction,
    input  wire [2:0] game_state,
    output reg        VGA_HS,
    output reg        VGA_VS,
    output reg  [3:0] VGA_R,
    output reg  [3:0] VGA_G,
    output reg  [3:0] VGA_B
);

    // --- 640x480 @ 60 Hz timing parameters ---
    localparam H_ACTIVE = 10'd640;
    localparam H_FP     = 10'd16;
    localparam H_SYNC   = 10'd96;
    localparam H_BP     = 10'd48;
    localparam H_TOTAL  = 10'd800;

    localparam V_ACTIVE = 10'd480;
    localparam V_FP     = 10'd10;
    localparam V_SYNC   = 10'd2;
    localparam V_BP     = 10'd33;
    localparam V_TOTAL  = 10'd525;

    reg [9:0] h_cnt;
    reg [9:0] v_cnt;

    // --- Horizontal / vertical counters ---
    always @(posedge clk_25 or posedge rst) begin
        if (rst) begin
            h_cnt <= 10'd0;
            v_cnt <= 10'd0;
        end
        else begin
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 10'd0;
                if (v_cnt == V_TOTAL - 1) v_cnt <= 10'd0;
                else                      v_cnt <= v_cnt + 10'd1;
            end
            else h_cnt <= h_cnt + 10'd1;
        end
    end

    // --- Sync pulses (active-low for 640x480@60) ---
    always @(posedge clk_25 or posedge rst) begin
        if (rst) begin
            VGA_HS <= 1'b1;
            VGA_VS <= 1'b1;
        end
        else begin
            VGA_HS <= ~((h_cnt >= (H_ACTIVE + H_FP)) &&
                       (h_cnt <  (H_ACTIVE + H_FP + H_SYNC)));
            VGA_VS <= ~((v_cnt >= (V_ACTIVE + V_FP)) &&
                       (v_cnt <  (V_ACTIVE + V_FP + V_SYNC)));
        end
    end

    // --- In active display area? ---
    wire in_active = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
    wire [9:0] px = h_cnt;
    wire [9:0] py = v_cnt;

    // --- Track rectangles (MUST match collision_detector.v) ---
    wire in_A = (px >= 20 ) && (px < 360) && (py >= 40 ) && (py < 100);
    wire in_B = (px >= 240) && (px < 310) && (py >= 80 ) && (py < 230);
    wire in_C = (px >= 120) && (px < 520) && (py >= 210) && (py < 270);
    wire in_D = (px >= 460) && (px < 540) && (py >= 120) && (py < 430);
    wire in_E = (px >= 200) && (px < 540) && (py >= 370) && (py < 430);
    wire in_F = (px >= 60 ) && (px < 310) && (py >= 330) && (py < 430);
    wire in_G = (px >= 20 ) && (px < 280) && (py >= 400) && (py < 470);
    wire on_road = in_A | in_B | in_C | in_D | in_E | in_F | in_G;

    // Zones
    wire in_stop_px = (px >= 280) && (px < 360) && (py >= 40) && (py < 100);
    wire in_park_px = (px >= 20 ) && (px < 120) && (py >= 420) && (py < 470);

    // --- Car sprite (20x12 rectangle) with direction indicator ---
    wire in_car = (px >= veh_x) && (px < veh_x + 10'd20) &&
                  (py >= {1'b0, veh_y}) && (py < {1'b0, veh_y} + 10'd12);

    // "Nose" = small 4x4 region at the front of the car (depends on facing)
    wire in_nose_up    = (direction == 2'd0) && (px >= veh_x + 10'd8 ) && (px < veh_x + 10'd12) &&
                         (py >= {1'b0,veh_y}) && (py < {1'b0,veh_y} + 10'd3);
    wire in_nose_right = (direction == 2'd1) && (px >= veh_x + 10'd17) && (px < veh_x + 10'd20) &&
                         (py >= {1'b0,veh_y} + 10'd4) && (py < {1'b0,veh_y} + 10'd8);
    wire in_nose_down  = (direction == 2'd2) && (px >= veh_x + 10'd8 ) && (px < veh_x + 10'd12) &&
                         (py >= {1'b0,veh_y} + 10'd9) && (py < {1'b0,veh_y} + 10'd12);
    wire in_nose_left  = (direction == 2'd3) && (px >= veh_x)           && (px < veh_x + 10'd3) &&
                         (py >= {1'b0,veh_y} + 10'd4) && (py < {1'b0,veh_y} + 10'd8);
    wire in_nose = in_nose_up | in_nose_right | in_nose_down | in_nose_left;

    // --- Cone markers: small 6x6 squares at key boundary points ---
    function cone_at;
        input [9:0] cx;
        input [9:0] cy;
        input [9:0] h;
        input [9:0] v;
        begin
            cone_at = (h >= cx-3) && (h < cx+3) && (v >= cy-3) && (v < cy+3);
        end
    endfunction

    wire c1  = cone_at(10'd150, 10'd110, px, py);
    wire c2  = cone_at(10'd220, 10'd110, px, py);
    wire c3  = cone_at(10'd100, 10'd320, px, py);
    wire c4  = cone_at(10'd180, 10'd320, px, py);
    wire c5  = cone_at(10'd260, 10'd320, px, py);
    wire c6  = cone_at(10'd340, 10'd210, px, py);
    wire c7  = cone_at(10'd420, 10'd210, px, py);
    wire c8  = cone_at(10'd560, 10'd200, px, py);
    wire c9  = cone_at(10'd560, 10'd380, px, py);
    wire c10 = cone_at(10'd180, 10'd440, px, py);
    wire on_cone = c1|c2|c3|c4|c5|c6|c7|c8|c9|c10;

    // --- White centre-line markings ---
    wire dash_A = in_A & (py == 70) & (px[4:0] < 5'd20);
    wire dash_G = in_G & (py == 440) & (px[4:0] < 5'd20);

    // --- Color mux with priority ---
    // Priority: cone > nose > car > dash lines > zones > road > grass
    // During blanking interval RGB is forced to 0.
    always @(*) begin
        // Default = grass (off-road)
        VGA_R = 4'h2; VGA_G = 4'h8; VGA_B = 4'h2;

        if (on_cone) begin
            VGA_R = 4'hF; VGA_G = 4'h6; VGA_B = 4'h0;  // orange
        end
        else if (in_nose) begin
            VGA_R = 4'hF; VGA_G = 4'hF; VGA_B = 4'hF;  // white nose
        end
        else if (in_car) begin
            if (game_state == 3'd2) begin                // S_COLLIDED: red
                VGA_R = 4'hF; VGA_G = 4'h0; VGA_B = 4'h0;
            end
            else if (game_state == 3'd5) begin           // S_DONE: yellow
                VGA_R = 4'hF; VGA_G = 4'hF; VGA_B = 4'h0;
            end
            else begin                                   // normal: blue
                VGA_R = 4'h1; VGA_G = 4'h3; VGA_B = 4'hF;
            end
        end
        else if (dash_A || dash_G) begin
            VGA_R = 4'hF; VGA_G = 4'hF; VGA_B = 4'hF;  // white dashes
        end
        else if (in_stop_px) begin
            VGA_R = 4'hF; VGA_G = 4'h4; VGA_B = 4'h4;  // red STOP zone
        end
        else if (in_park_px) begin
            VGA_R = 4'hE; VGA_G = 4'hE; VGA_B = 4'h2;  // yellow PARK zone
        end
        else if (on_road) begin
            VGA_R = 4'h3; VGA_G = 4'h3; VGA_B = 4'h3;  // dark gray road
        end

        // Force black during blanking interval
        if (!in_active) begin
            VGA_R = 4'h0; VGA_G = 4'h0; VGA_B = 4'h0;
        end
    end

endmodule
