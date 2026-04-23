// ============================================================================
// score_display.v
// Drives HEX0..HEX5 on the DE2 board (active-low segments).
//   HEX1 HEX0 = speed   (tens, units)
//   HEX5 HEX4 HEX3 HEX2 = MM MM SS SS  (elapsed minutes:seconds)
// ============================================================================
module score_display (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  speed,
    input  wire [15:0] elapsed_sec,
    output reg  [6:0]  HEX0,
    output reg  [6:0]  HEX1,
    output reg  [6:0]  HEX2,
    output reg  [6:0]  HEX3,
    output reg  [6:0]  HEX4,
    output reg  [6:0]  HEX5
);

    // BCD -> 7-seg (active-low: a=bit0 ... g=bit6, 0=lit)
    function [6:0] seg7;
        input [3:0] bcd;
        begin
            case (bcd)
                4'd0: seg7 = 7'b1000000;
                4'd1: seg7 = 7'b1111001;
                4'd2: seg7 = 7'b0100100;
                4'd3: seg7 = 7'b0110000;
                4'd4: seg7 = 7'b0011001;
                4'd5: seg7 = 7'b0010010;
                4'd6: seg7 = 7'b0000010;
                4'd7: seg7 = 7'b1111000;
                4'd8: seg7 = 7'b0000000;
                4'd9: seg7 = 7'b0010000;
                default: seg7 = 7'b1111111; // blank
            endcase
        end
    endfunction

    // Elapsed time split
    wire [15:0] total = elapsed_sec;
    // Modulo/divide synthesize to small combinational logic for 16-bit values.
    // Cap minutes at 99 to fit two digits.
    wire [15:0] total_capped = (total > 16'd5999) ? 16'd5999 : total;
    wire [15:0] minutes = total_capped / 16'd60;
    wire [15:0] seconds = total_capped - (minutes * 16'd60);

    wire [3:0] min_tens  = minutes[7:0] / 8'd10;
    wire [3:0] min_units = minutes[7:0] - (min_tens * 4'd10);
    wire [3:0] sec_tens  = seconds[7:0] / 8'd10;
    wire [3:0] sec_units = seconds[7:0] - (sec_tens * 4'd10);

    // Speed digits (speed is 0..7 so tens = 0)
    wire [3:0] spd_tens  = 4'd0;
    wire [3:0] spd_units = {1'b0, speed};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            HEX0 <= 7'b1111111; HEX1 <= 7'b1111111;
            HEX2 <= 7'b1111111; HEX3 <= 7'b1111111;
            HEX4 <= 7'b1111111; HEX5 <= 7'b1111111;
        end
        else begin
            HEX0 <= seg7(spd_units);
            HEX1 <= seg7(spd_tens);
            HEX2 <= seg7(sec_units);
            HEX3 <= seg7(sec_tens);
            HEX4 <= seg7(min_units);
            HEX5 <= seg7(min_tens);
        end
    end

endmodule
