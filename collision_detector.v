// ============================================================================
// collision_detector.v
// Defines the track as a UNION of allowed rectangular road segments
// (taken from the circuit.jpeg layout). A collision fires when the
// vehicle's bounding box is NOT contained within any allowed segment.
//
// Coordinate system: 640 (wide) x 480 (tall), origin top-left.
//
// Track segments (mapped to the provided reference image):
//   A: START corridor  -- top horizontal strip
//   B: Left chicane    -- broad diagonal connector region
//   C: Loop entry      -- connector into center-right loop
//   D/E/F/G: Main loop -- rectangular loop on right
//   H/I/J: Left bay    -- parking approach notch
//   K: FINISH lane     -- bottom horizontal strip
//
// Special zones:
//   STOP zone    -- top-center (x=280..360, y=40..90)   (overlap with A)
//   PARK zone    -- bottom-left of FINISH (x=20..120, y=420..470)
// ============================================================================
module collision_detector (
    input  wire [9:0] veh_x,
    input  wire [8:0] veh_y,
    output wire       collision_detected,
    output wire       at_start_line,
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

    // Car must be fully contained in one allowed segment
    wire in_A = (x0 >= 10'd20 ) && (x1 <= 10'd430) && (y0 >= 9'd40 ) && (y1 <= 9'd100);
    wire in_B = (x0 >= 10'd95 ) && (x1 <= 10'd290) && (y0 >= 9'd90 ) && (y1 <= 9'd250);
    wire in_C = (x0 >= 10'd260) && (x1 <= 10'd340) && (y0 >= 9'd120) && (y1 <= 9'd190);
    wire in_D = (x0 >= 10'd300) && (x1 <= 10'd570) && (y0 >= 9'd140) && (y1 <= 9'd190);
    wire in_E = (x0 >= 10'd530) && (x1 <= 10'd600) && (y0 >= 9'd140) && (y1 <= 9'd430);
    wire in_F = (x0 >= 10'd300) && (x1 <= 10'd570) && (y0 >= 9'd380) && (y1 <= 9'd430);
    wire in_G = (x0 >= 10'd300) && (x1 <= 10'd350) && (y0 >= 9'd140) && (y1 <= 9'd430);
    wire in_H = (x0 >= 10'd140) && (x1 <= 10'd300) && (y0 >= 9'd300) && (y1 <= 9'd350);
    wire in_I = (x0 >= 10'd140) && (x1 <= 10'd180) && (y0 >= 9'd300) && (y1 <= 9'd430);
    wire in_J = (x0 >= 10'd250) && (x1 <= 10'd300) && (y0 >= 9'd300) && (y1 <= 9'd430);
    wire in_K = (x0 >= 10'd20 ) && (x1 <= 10'd620) && (y0 >= 9'd400) && (y1 <= 9'd470);

    wire on_road = in_A | in_B | in_C | in_D | in_E | in_F | in_G |
                   in_H | in_I | in_J | in_K;

    assign collision_detected = ~on_road;

    // Start line: thin strip near the initial car position in segment A
    assign at_start_line =
        (x0 >= 10'd55) && (x1 <= 10'd90) &&
        (y0 >= 9'd40 ) && (y1 <= 9'd100);

    // STOP zone: top strip near the right side, matching the visual marker
    assign in_stop_zone =
        (x0 >= 10'd330) && (x1 <= 10'd420) &&
        (y0 >= 9'd40 ) && (y1 <= 9'd100);

    // PARKING/FINISH zone: bottom-left
    assign in_parking_zone =
        (x0 >= 10'd20 ) && (x1 <= 10'd120) &&
        (y0 >= 9'd400) && (y1 <= 9'd470);

endmodule
