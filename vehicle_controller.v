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
    reg [3:0] throttle_cnt;

    always @(posedge clk_game or posedge rst) begin
        if (rst) begin
            veh_x            <= START_X;
            veh_y            <= START_Y;
            speed            <= 3'd0;
            direction        <= DIR_RIGHT;
            steer_left_prev  <= 1'b0;
            steer_right_prev <= 1'b0;
            throttle_cnt     <= 4'd0;
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
                if (throttle_cnt == 4'd7) begin
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

                // --- Position update ---
                case (direction)
                    DIR_UP:    if (veh_y > speed)           veh_y <= veh_y - {6'd0, speed};
                               else                         veh_y <= 9'd0;
                    DIR_DOWN:  if (veh_y + speed < 9'd480-12)
                                                            veh_y <= veh_y + {6'd0, speed};
                               else                         veh_y <= 9'd480-12;
                    DIR_RIGHT: if (veh_x + speed < 10'd640-20)
                                                            veh_x <= veh_x + {7'd0, speed};
                               else                         veh_x <= 10'd640-20;
                    DIR_LEFT:  if (veh_x > speed)           veh_x <= veh_x - {7'd0, speed};
                               else                         veh_x <= 10'd0;
                endcase
            end
        end
    end

endmodule
