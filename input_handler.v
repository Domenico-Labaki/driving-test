// ============================================================================
// input_handler.v
// Debounces the noisy mechanical inputs from the DE2 board.
//   - KEY[0] (active-low) -> steer_right
//   - KEY[1] (active-low) -> steer_left
//   - KEY[2] (active-low) -> accel
//   - SW[0]  (active-high) -> brake
//
// Strategy: for each input, require the signal to be stable for
// DEBOUNCE_CYCLES consecutive clk edges before propagating the change.
// ============================================================================
module input_handler (
    input  wire       clk,          // clk_game (~60 Hz) is fine for debouncing
    input  wire       rst,
    input  wire [3:0] KEY_raw,      // active-low on DE2
    input  wire [1:0] SW_raw,
    output reg        steer_left,
    output reg        steer_right,
    output reg        accel,
    output reg        brake
);

    // At 60 Hz, 4 cycles = ~66 ms — comfortably longer than any bounce.
    localparam integer DEBOUNCE_CYCLES = 4;

    // Helper registers: counter of stable cycles + current stable value
    reg [2:0] cnt_l,  cnt_r,  cnt_a,  cnt_b;
    reg       curr_l, curr_r, curr_a, curr_b;

    wire want_r = ~KEY_raw[0];   // invert active-low KEY
    wire want_l = ~KEY_raw[1];
    wire want_a = ~KEY_raw[2];
    wire want_b =  SW_raw[0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt_l <= 0; cnt_r <= 0; cnt_a <= 0; cnt_b <= 0;
            curr_l <= 0; curr_r <= 0; curr_a <= 0; curr_b <= 0;
            steer_left <= 0; steer_right <= 0; accel <= 0; brake <= 0;
        end
        else begin
            // steer_left
            if (want_l == curr_l) cnt_l <= 0;
            else if (cnt_l == DEBOUNCE_CYCLES-1) begin curr_l <= want_l; cnt_l <= 0; end
            else cnt_l <= cnt_l + 1;

            // steer_right
            if (want_r == curr_r) cnt_r <= 0;
            else if (cnt_r == DEBOUNCE_CYCLES-1) begin curr_r <= want_r; cnt_r <= 0; end
            else cnt_r <= cnt_r + 1;

            // accel
            if (want_a == curr_a) cnt_a <= 0;
            else if (cnt_a == DEBOUNCE_CYCLES-1) begin curr_a <= want_a; cnt_a <= 0; end
            else cnt_a <= cnt_a + 1;

            // brake
            if (want_b == curr_b) cnt_b <= 0;
            else if (cnt_b == DEBOUNCE_CYCLES-1) begin curr_b <= want_b; cnt_b <= 0; end
            else cnt_b <= cnt_b + 1;

            // Drive outputs from stable state
            steer_left  <= curr_l;
            steer_right <= curr_r;
            accel       <= curr_a;
            brake       <= curr_b;
        end
    end

endmodule
