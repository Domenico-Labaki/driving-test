// ============================================================================
// vehicle_controller.v
// Tracks the vehicle's X/Y position, speed, and facing direction.
// Position is clamped to the 640x480 VGA screen; speed is clamped 0..7.
// Direction is encoded as 2 bits: 0=UP, 1=RIGHT, 2=DOWN, 3=LEFT.
// ============================================================================
module vehicle_controller (
    input  wire       clk_game,
    input  wire       rst,
    input  wire       game_active,    // only update when high
    input  wire       freeze,         // during collision penalty
    input  wire       accel,
    input  wire       brake,
    input  wire       steer_left,
    input  wire       steer_right,
    output reg  [9:0] veh_x,          // 0..639
    output reg  [8:0] veh_y,          // 0..479
    output reg  [2:0] speed,          // 0..7
    output reg  [1:0] direction       // 0=UP,1=RIGHT,2=DOWN,3=LEFT
);

    // Track geometry must match collision_detector.v and vga_controller.v.
    localparam [9:0] CAR_W = 10'd20;
    localparam [8:0] CAR_H = 9'd12;

    // --- Start pose: on the START strip, facing right ---
    localparam [9:0] START_X   = 10'd60;
    localparam [8:0] START_Y   = 9'd60;
    localparam [1:0] DIR_UP    = 2'd0;
    localparam [1:0] DIR_RIGHT = 2'd1;
    localparam [1:0] DIR_DOWN  = 2'd2;
    localparam [1:0] DIR_LEFT  = 2'd3;

    localparam [2:0] MAX_SPEED = 3'd7;

    // --- Steering edge detectors so holding a KEY doesn't spin the car ---
    reg steer_left_prev, steer_right_prev;
    wire steer_l_edge = steer_left  & ~steer_left_prev;
    wire steer_r_edge = steer_right & ~steer_right_prev;

    // --- Throttle sub-dividers so speed doesn't saturate in one tick ---
    reg [4:0] throttle_cnt;

    reg [2:0] move_cnt;

    function on_road_bbox;
        input [9:0] x;
        input [8:0] y;
        reg [9:0] x1;
        reg [8:0] y1;
        begin
            x1 = x + CAR_W;
            y1 = y + CAR_H;

            on_road_bbox =
                ((x >= 10'd20 ) && (x1 <= 10'd430) && (y >= 9'd40 ) && (y1 <= 9'd100)) ||
                ((x >= 10'd95 ) && (x1 <= 10'd290) && (y >= 9'd90 ) && (y1 <= 9'd250)) ||
                ((x >= 10'd260) && (x1 <= 10'd340) && (y >= 9'd120) && (y1 <= 9'd190)) ||
                ((x >= 10'd300) && (x1 <= 10'd570) && (y >= 9'd140) && (y1 <= 9'd190)) ||
                ((x >= 10'd530) && (x1 <= 10'd600) && (y >= 9'd140) && (y1 <= 9'd430)) ||
                ((x >= 10'd300) && (x1 <= 10'd570) && (y >= 9'd380) && (y1 <= 9'd430)) ||
                ((x >= 10'd300) && (x1 <= 10'd350) && (y >= 9'd140) && (y1 <= 9'd430)) ||
                ((x >= 10'd140) && (x1 <= 10'd300) && (y >= 9'd300) && (y1 <= 9'd350)) ||
                ((x >= 10'd140) && (x1 <= 10'd180) && (y >= 9'd300) && (y1 <= 9'd430)) ||
                ((x >= 10'd250) && (x1 <= 10'd300) && (y >= 9'd300) && (y1 <= 9'd430)) ||
                ((x >= 10'd20 ) && (x1 <= 10'd620) && (y >= 9'd400) && (y1 <= 9'd470));
        end
    endfunction

    always @(posedge clk_game or posedge rst) begin
        if (rst) begin
            veh_x            <= START_X;
            veh_y            <= START_Y;
            speed            <= 3'd0;
            direction        <= DIR_RIGHT;
            steer_left_prev  <= 1'b0;
            steer_right_prev <= 1'b0;
            throttle_cnt     <= 5'd0;
            move_cnt         <= 3'd0;
        end
        else begin
            steer_left_prev  <= steer_left;
            steer_right_prev <= steer_right;

            if (!game_active) begin
                // When IDLE or DONE: hold the start pose if speed is 0,
                // otherwise allow natural stop via braking elsewhere.
                if (speed == 0) begin
                    veh_x     <= START_X;
                    veh_y     <= START_Y;
                    direction <= DIR_RIGHT;
                end
            end
            else if (freeze) begin
                // During collision penalty: force speed to 0, keep position
                speed <= 3'd0;
            end
            else begin
                // --- Speed update (every 8 ticks ~= 7.5 Hz adjustments) ---
                throttle_cnt <= throttle_cnt + 1;
                if (throttle_cnt == 5'd24) begin
                    throttle_cnt <= 0;
                    if (brake) begin
                        if (speed != 0) speed <= speed - 1;
                    end
                    else if (accel) begin
                        if (speed != MAX_SPEED) speed <= speed + 1;
                    end
                    // else coast: hold speed
                end

                // --- Direction update on button EDGE (only if moving) ---
                if (speed != 0) begin
                    if (steer_l_edge)      direction <= direction - 2'd1; // turn left (CCW)
                    else if (steer_r_edge) direction <= direction + 2'd1; // turn right (CW)
                end

                // --- Position update every 4 ticks (~15 Hz) ---
                move_cnt <= move_cnt + 1;
                if (move_cnt == 3'd3) begin
                    move_cnt <= 3'd0;

                    // Candidate move in the current heading.
                    case (direction)
                        DIR_UP: begin
                            if (speed != 0 && veh_y > speed && on_road_bbox(veh_x, veh_y - {1'b0, speed}))
                                veh_y <= veh_y - {1'b0, speed};
                            else if (veh_y + speed < 9'd480-12 && on_road_bbox(veh_x, veh_y + {1'b0, speed})) begin
                                veh_y <= veh_y + {1'b0, speed};
                                direction <= DIR_DOWN;
                            end
                        end
                        DIR_DOWN: begin
                            if (speed != 0 && veh_y + speed < 9'd480-12 && on_road_bbox(veh_x, veh_y + {1'b0, speed}))
                                veh_y <= veh_y + {1'b0, speed};
                            else if (veh_y > speed && on_road_bbox(veh_x, veh_y - {1'b0, speed})) begin
                                veh_y <= veh_y - {1'b0, speed};
                                direction <= DIR_UP;
                            end
                        end
                        DIR_RIGHT: begin
                            if (speed != 0 && veh_x + speed < 10'd640-20 && on_road_bbox(veh_x + {1'b0, speed}, veh_y))
                                veh_x <= veh_x + {1'b0, speed};
                            else if (veh_x > speed && on_road_bbox(veh_x - {1'b0, speed}, veh_y)) begin
                                veh_x <= veh_x - {1'b0, speed};
                                direction <= DIR_LEFT;
                            end
                        end
                        DIR_LEFT: begin
                            if (speed != 0 && veh_x > speed && on_road_bbox(veh_x - {1'b0, speed}, veh_y))
                                veh_x <= veh_x - {1'b0, speed};
                            else if (veh_x + speed < 10'd640-20 && on_road_bbox(veh_x + {1'b0, speed}, veh_y)) begin
                                veh_x <= veh_x + {1'b0, speed};
                                direction <= DIR_RIGHT;
                            end
                        end
                    endcase
                end
            end
        end
    end

endmodule
