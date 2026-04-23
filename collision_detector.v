// ============================================================================
// collision_detector.v
// Defines the track as a UNION of allowed rectangular road segments
// (taken from the circuit.jpeg layout). A collision fires when the
// vehicle's bounding box is NOT contained within any allowed segment.
//
// Coordinate system: 640 (wide) x 480 (tall), origin top-left.
//
// Track segments (approximate layout of circuit.jpeg):
//   A: START corridor  -- horizontal top strip (x=20..280, y=40..90)
//   B: Diagonal link   -- we approximate as a small vertical segment
//                        (x=240..300, y=80..220)
//   C: Middle crossbar -- horizontal (x=120..520, y=210..260)
//   D: Right loop      -- vertical right column (x=470..530, y=120..420)
//   E: Right U-turn    -- horizontal bottom link (x=200..530, y=370..420)
//   F: Slalom lane     -- horizontal (x=60..300, y=340..390)
//   G: FINISH corridor -- horizontal bottom strip (x=20..260, y=420..470)
//
// Special zones:
//   STOP zone    -- top-center (x=280..360, y=40..90)   (overlap with A)
//   PARK zone    -- bottom-left of FINISH (x=20..120, y=420..470)
// ============================================================================
module collision_detector (
    input  wire [9:0] veh_x,
    input  wire [8:0] veh_y,
    output wire       collision_detected,
    output wire       in_stop_zone,
    output wire       in_parking_zone
);

    // --- Car bounding box ---
    localparam [9:0] CAR_W = 10'd20;
    localparam [8:0] CAR_H = 9'd12;

    wire [9:0] x0 = veh_x;
    wire [9:0] x1 = veh_x + CAR_W;
    wire [8:0] y0 = veh_y;
    wire [8:0] y1 = veh_y + CAR_H;

    // Helper: car fully inside a rectangle [rx0,rx1] x [ry0,ry1]
    // (using continuous assigns for simplicity)
    wire in_A = (x0 >= 10'd20 ) && (x1 <= 10'd360) && (y0 >= 9'd40 ) && (y1 <= 9'd100);
    wire in_B = (x0 >= 10'd240) && (x1 <= 10'd310) && (y0 >= 9'd80 ) && (y1 <= 9'd230);
    wire in_C = (x0 >= 10'd120) && (x1 <= 10'd520) && (y0 >= 9'd210) && (y1 <= 9'd270);
    wire in_D = (x0 >= 10'd460) && (x1 <= 10'd540) && (y0 >= 9'd120) && (y1 <= 9'd430);
    wire in_E = (x0 >= 10'd200) && (x1 <= 10'd540) && (y0 >= 9'd370) && (y1 <= 9'd430);
    wire in_F = (x0 >= 10'd60 ) && (x1 <= 10'd310) && (y0 >= 9'd330) && (y1 <= 9'd430);
    wire in_G = (x0 >= 10'd20 ) && (x1 <= 10'd280) && (y0 >= 9'd400) && (y1 <= 9'd470);

    wire on_road = in_A | in_B | in_C | in_D | in_E | in_F | in_G;

    assign collision_detected = ~on_road;

    // STOP zone: top strip near center
    assign in_stop_zone =
        (x0 >= 10'd280) && (x1 <= 10'd360) &&
        (y0 >= 9'd40 ) && (y1 <= 9'd100);

    // PARKING/FINISH zone: bottom-left
    assign in_parking_zone =
        (x0 >= 10'd20 ) && (x1 <= 10'd120) &&
        (y0 >= 9'd420) && (y1 <= 9'd470);

endmodule
