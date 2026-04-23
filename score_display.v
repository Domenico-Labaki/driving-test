// ============================================================================
// score_display.v
// Drives HEX0..HEX5 on the DE2 board (active-low segments).
//   HEX1 HEX0 = speed in km/h-style units (00..70)
//   HEX5 HEX4 = countdown units/tens (00..60)
// ============================================================================
module score_display (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  speed,
    input  wire [5:0]  countdown_sec,
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

    wire [3:0] cd_tens  = countdown_sec / 6'd10;
    wire [3:0] cd_units = countdown_sec - (cd_tens * 4'd10);

    // Convert internal 0..7 speed steps to a km/h-style display value.
    wire [6:0] speed_kmh = {4'd0, speed} * 7'd10;  // 00,10,...,70
    wire [3:0] spd_tens  = speed_kmh / 7'd10;
    wire [3:0] spd_units = speed_kmh - (spd_tens * 4'd10);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            HEX0 <= 7'b1111111; HEX1 <= 7'b1111111;
            HEX2 <= 7'b1111111; HEX3 <= 7'b1111111;
            HEX4 <= 7'b1111111; HEX5 <= 7'b1111111;
        end
        else begin
            HEX0 <= seg7(spd_units);
            HEX1 <= seg7(spd_tens);
            HEX2 <= seg7(4'd0);           // hundreds (always 0)
            HEX3 <= seg7(4'd0);
            HEX4 <= seg7(cd_units);
            HEX5 <= seg7(cd_tens);
        end
    end

endmodule
