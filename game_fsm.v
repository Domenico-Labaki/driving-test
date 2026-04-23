// ============================================================================
// game_fsm.v
// Master finite-state machine for the driving-test game.
// States:
//   IDLE       -> waiting for SW_en (SW[17]) to start the game
//   PLAYING    -> normal game loop; vehicle moves, timer runs
//   COLLIDED   -> short penalty freeze after a crash; loses one life
//   AT_STOP    -> vehicle entered STOP zone; must halt (speed==0)
//   PARKING    -> vehicle reached parking/FINISH area
//   DONE       -> pass/fail displayed; wait for reset
// ============================================================================
module game_fsm (
    input  wire       clk,                // clk_game tick
    input  wire       rst,                // active-high
    input  wire       SW_en,              // SW[17]
    input  wire       collision_detected,
    input  wire       in_stop_zone,
    input  wire       in_parking_zone,
    input  wire [2:0] speed,              // from vehicle_controller
    output reg  [2:0] game_state,
    output reg        pass,
    output reg  [2:0] lives,
    output reg [15:0] elapsed_sec
);

    // --- State encoding ---
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_PLAYING  = 3'd1;
    localparam [2:0] S_COLLIDED = 3'd2;
    localparam [2:0] S_AT_STOP  = 3'd3;
    localparam [2:0] S_PARKING  = 3'd4;
    localparam [2:0] S_DONE     = 3'd5;

    // Parameters
    localparam integer PENALTY_TICKS = 60;    // ~1 s at 60 Hz
    localparam integer STOP_HOLD     = 60;    // must be stopped this long
    localparam [2:0]   LIVES_INIT    = 3'd3;

    // Internal counters
    reg [7:0]  penalty_cnt;
    reg [7:0]  stop_cnt;
    reg [5:0]  second_cnt;    // counts 60 game ticks to make 1 second
    reg        collision_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            game_state     <= S_IDLE;
            pass           <= 1'b0;
            lives          <= LIVES_INIT;
            elapsed_sec    <= 16'd0;
            penalty_cnt    <= 0;
            stop_cnt       <= 0;
            second_cnt     <= 0;
            collision_prev <= 0;
        end
        else begin
            collision_prev <= collision_detected;

            // --- Seconds counter runs only while playing ---
            if (game_state == S_PLAYING || game_state == S_AT_STOP) begin
                if (second_cnt == 6'd59) begin
                    second_cnt  <= 0;
                    elapsed_sec <= elapsed_sec + 16'd1;
                end
                else second_cnt <= second_cnt + 1;
            end

            // --- State transitions ---
            case (game_state)
                S_IDLE: begin
                    pass        <= 1'b0;
                    lives       <= LIVES_INIT;
                    elapsed_sec <= 16'd0;
                    second_cnt  <= 0;
                    if (SW_en) game_state <= S_PLAYING;
                end

                S_PLAYING: begin
                    // rising edge of collision
                    if (collision_detected && !collision_prev) begin
                        game_state  <= S_COLLIDED;
                        penalty_cnt <= 0;
                        if (lives != 0) lives <= lives - 1;
                    end
                    else if (in_stop_zone) begin
                        game_state <= S_AT_STOP;
                        stop_cnt   <= 0;
                    end
                    else if (in_parking_zone) begin
                        game_state <= S_PARKING;
                    end
                end

                S_COLLIDED: begin
                    if (lives == 0) begin
                        pass       <= 1'b0;
                        game_state <= S_DONE;
                    end
                    else if (penalty_cnt == PENALTY_TICKS) begin
                        game_state <= S_PLAYING;
                    end
                    else penalty_cnt <= penalty_cnt + 1;
                end

                S_AT_STOP: begin
                    // Must come to a full stop within STOP zone, then leave
                    if (collision_detected && !collision_prev) begin
                        game_state  <= S_COLLIDED;
                        penalty_cnt <= 0;
                        if (lives != 0) lives <= lives - 1;
                    end
                    else if (speed == 3'd0) begin
                        if (stop_cnt == STOP_HOLD) game_state <= S_PLAYING;
                        else stop_cnt <= stop_cnt + 1;
                    end
                    else begin
                        stop_cnt <= 0;
                        // If vehicle leaves the stop zone WITHOUT ever stopping,
                        // that will naturally move in_stop_zone to 0 and we fall
                        // through to a collision branch in the next tick if off-road.
                        if (!in_stop_zone) game_state <= S_PLAYING;
                    end
                end

                S_PARKING: begin
                    // Vehicle parked successfully: must be stopped inside the zone
                    if (speed == 3'd0 && in_parking_zone) begin
                        pass       <= 1'b1;
                        game_state <= S_DONE;
                    end
                    else if (!in_parking_zone) begin
                        // Rolled out; back to playing
                        game_state <= S_PLAYING;
                    end
                end

                S_DONE: begin
                    // Held in DONE until external reset (KEY[3])
                end

                default: game_state <= S_IDLE;
            endcase
        end
    end

endmodule
