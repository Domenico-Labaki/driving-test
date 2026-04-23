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
    input  wire [2:0] direction,
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

    // --- Track rectangles (must stay aligned with collision_detector.v) ---
    function is_road;
        input [9:0] x;
        input [9:0] y;
        begin
            is_road =
                // A: start corridor
                ((x >= 10'd20 ) && (x < 10'd430) && (y >= 10'd40 ) && (y < 10'd100)) ||
                // B: left chicane region
                ((x >= 10'd95 ) && (x < 10'd290) && (y >= 10'd90 ) && (y < 10'd250)) ||
                // C: loop entry connector
                ((x >= 10'd260) && (x < 10'd340) && (y >= 10'd120) && (y < 10'd190)) ||
                // D/E/F/G: main right loop
                ((x >= 10'd300) && (x < 10'd570) && (y >= 10'd140) && (y < 10'd190)) ||
                ((x >= 10'd530) && (x < 10'd600) && (y >= 10'd140) && (y < 10'd430)) ||
                ((x >= 10'd300) && (x < 10'd570) && (y >= 10'd380) && (y < 10'd430)) ||
                ((x >= 10'd300) && (x < 10'd350) && (y >= 10'd140) && (y < 10'd430)) ||
                // H/I/J: left bay
                ((x >= 10'd140) && (x < 10'd300) && (y >= 10'd300) && (y < 10'd350)) ||
                ((x >= 10'd140) && (x < 10'd180) && (y >= 10'd300) && (y < 10'd430)) ||
                ((x >= 10'd250) && (x < 10'd300) && (y >= 10'd300) && (y < 10'd430)) ||
                // K: finish corridor
                ((x >= 10'd20 ) && (x < 10'd620) && (y >= 10'd400) && (y < 10'd470));
        end
    endfunction

    wire on_road = is_road(px, py);

    // Zones
    wire in_stop_px = (px >= 330) && (px < 420) && (py >= 40) && (py < 100);
    wire in_park_px = (px >= 20 ) && (px < 120) && (py >= 400) && (py < 470);

    // Road-edge extraction to draw white lane markings around the asphalt.
    wire [9:0] px_m1 = (px == 0)       ? 10'd0   : (px - 10'd1);
    wire [9:0] px_p1 = (px == 10'd639) ? 10'd639 : (px + 10'd1);
    wire [9:0] py_m1 = (py == 0)       ? 10'd0   : (py - 10'd1);
    wire [9:0] py_p1 = (py == 10'd479) ? 10'd479 : (py + 10'd1);

    wire road_n = is_road(px,    py_m1);
    wire road_s = is_road(px,    py_p1);
    wire road_w = is_road(px_m1, py);
    wire road_e = is_road(px_p1, py);
    wire road_edge = on_road && !(road_n && road_s && road_w && road_e);

    // Start / finish markings
    wire start_line  = (px >= 10'd75) && (px < 10'd82) &&
                       (py >= 10'd40) && (py < 10'd100);
    wire finish_line = in_park_px && (px[3] ^ py[3]);

    // --- Car sprite (20x12 rectangle) with direction indicator ---
    wire in_car = (px >= veh_x) && (px < veh_x + 10'd20) &&
                  (py >= {1'b0, veh_y}) && (py < {1'b0, veh_y} + 10'd12);

    // "Nose" = small region at the front of the car (depends on 8-way heading)
    wire in_nose_up         = (direction == 3'd0) && (px >= veh_x + 10'd8 ) && (px < veh_x + 10'd12) &&
                              (py >= {1'b0,veh_y}) && (py < {1'b0,veh_y} + 10'd3);
    wire in_nose_up_right   = (direction == 3'd1) && (px >= veh_x + 10'd17) && (px < veh_x + 10'd20) &&
                              (py >= {1'b0,veh_y}) && (py < {1'b0,veh_y} + 10'd3);
    wire in_nose_right      = (direction == 3'd2) && (px >= veh_x + 10'd17) && (px < veh_x + 10'd20) &&
                              (py >= {1'b0,veh_y} + 10'd4) && (py < {1'b0,veh_y} + 10'd8);
    wire in_nose_down_right = (direction == 3'd3) && (px >= veh_x + 10'd17) && (px < veh_x + 10'd20) &&
                              (py >= {1'b0,veh_y} + 10'd9) && (py < {1'b0,veh_y} + 10'd12);
    wire in_nose_down       = (direction == 3'd4) && (px >= veh_x + 10'd8 ) && (px < veh_x + 10'd12) &&
                              (py >= {1'b0,veh_y} + 10'd9) && (py < {1'b0,veh_y} + 10'd12);
    wire in_nose_down_left  = (direction == 3'd5) && (px >= veh_x) && (px < veh_x + 10'd3) &&
                              (py >= {1'b0,veh_y} + 10'd9) && (py < {1'b0,veh_y} + 10'd12);
    wire in_nose_left       = (direction == 3'd6) && (px >= veh_x) && (px < veh_x + 10'd3) &&
                              (py >= {1'b0,veh_y} + 10'd4) && (py < {1'b0,veh_y} + 10'd8);
    wire in_nose_up_left    = (direction == 3'd7) && (px >= veh_x) && (px < veh_x + 10'd3) &&
                              (py >= {1'b0,veh_y}) && (py < {1'b0,veh_y} + 10'd3);
    wire in_nose = in_nose_up | in_nose_up_right | in_nose_right | in_nose_down_right |
                   in_nose_down | in_nose_down_left | in_nose_left | in_nose_up_left;

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

    wire c1  = cone_at(10'd170, 10'd105, px, py);
    wire c2  = cone_at(10'd110, 10'd170, px, py);
    wire c3  = cone_at(10'd150, 10'd235, px, py);
    wire c4  = cone_at(10'd255, 10'd155, px, py);
    wire c5  = cone_at(10'd315, 10'd300, px, py);
    wire c6  = cone_at(10'd315, 10'd385, px, py);
    wire c7  = cone_at(10'd560, 10'd155, px, py);
    wire c8  = cone_at(10'd560, 10'd385, px, py);
    wire c9  = cone_at(10'd175, 10'd300, px, py);
    wire c10 = cone_at(10'd175, 10'd385, px, py);
    wire on_cone = c1|c2|c3|c4|c5|c6|c7|c8|c9|c10;

    // --- White centre-line markings ---
    wire stop_dashes = in_stop_px && (px >= 10'd340) && (px < 10'd410) && (py[3:0] < 4'd3);

    // --- Color mux with priority ---
    // Priority: cone > nose > car > dash lines > zones > road > grass
    // During blanking interval RGB is forced to 0.
    always @(*) begin
        // Default = asphalt background
        VGA_R = 4'h4; VGA_G = 4'h4; VGA_B = 4'h4;

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
        else if (start_line || finish_line || road_edge || stop_dashes) begin
            VGA_R = 4'hF; VGA_G = 4'hF; VGA_B = 4'hF;  // white line markings
        end
        else if (in_stop_px) begin
            VGA_R = 4'h6; VGA_G = 4'h6; VGA_B = 4'h6;  // keep STOP area darker under dashes
        end
        else if (in_park_px) begin
            VGA_R = 4'h5; VGA_G = 4'h5; VGA_B = 4'h5;  // finish lane tint
        end
        else if (on_road) begin
            VGA_R = 4'h3; VGA_G = 4'h3; VGA_B = 4'h3;  // track asphalt
        end

        // Force black during blanking interval
        if (!in_active) begin
            VGA_R = 4'h0; VGA_G = 4'h0; VGA_B = 4'h0;
        end
    end

endmodule
