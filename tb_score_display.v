// tb_score_display.v
`timescale 1ns/1ps

module tb_score_display;
    reg clk = 0, rst = 1;
    reg [2:0]  speed = 0;
    reg [15:0] elapsed_sec = 0;
    wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    score_display dut (
        .clk(clk), .rst(rst),
        .speed(speed), .elapsed_sec(elapsed_sec),
        .HEX0(HEX0), .HEX1(HEX1),
        .HEX2(HEX2), .HEX3(HEX3),
        .HEX4(HEX4), .HEX5(HEX5)
    );

    always #10 clk = ~clk;

    initial begin
        $display("tb_score_display: start");
        #100 rst = 0;
        speed = 5; elapsed_sec = 75;
        repeat (3) @(posedge clk);
        // Expected: speed=05  time=01:15
        // HEX0 = '5' = 7'b0010010
        // HEX1 = '0' = 7'b1000000
        // HEX2 = '5' = 7'b0010010
        // HEX3 = '1' = 7'b1111001
        // HEX4 = '1' = 7'b1111001
        // HEX5 = '0' = 7'b1000000
        $display("speed=5 -> HEX1 HEX0 = %b %b (expect 1000000 0010010)", HEX1, HEX0);
        $display("time=75 -> HEX5..HEX2 = %b %b : %b %b (expect 1000000 1111001 0010010 1111001 -> 01:15)",
                 HEX5, HEX4, HEX3, HEX2);
        $stop;
    end
endmodule
