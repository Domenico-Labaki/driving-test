// tb_score_display.v
`timescale 1ns/1ps

module tb_score_display;
    reg clk = 0, rst = 1;
    reg [2:0]  speed = 0;
    reg [5:0] countdown_sec = 0;
    wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    score_display dut (
        .clk(clk), .rst(rst),
        .speed(speed), .countdown_sec(countdown_sec),
        .HEX0(HEX0), .HEX1(HEX1),
        .HEX2(HEX2), .HEX3(HEX3),
        .HEX4(HEX4), .HEX5(HEX5)
    );

    always #10 clk = ~clk;

    initial begin
        $display("tb_score_display: start");
        #100 rst = 0;
        speed = 5; countdown_sec = 37;
        repeat (3) @(posedge clk);
        // Expected: speed=05  countdown=37
        // HEX0 = '5' = 7'b0010010
        // HEX1 = '0' = 7'b1000000
        // HEX2 = '0' = 7'b1000000
        // HEX3 = '0' = 7'b1000000
        // HEX4 = '3' = 7'b0110000
        // HEX5 = '7' = 7'b1111000
        $display("speed=5 -> HEX1 HEX0 = %b %b (expect 1000000 0010010)", HEX1, HEX0);
        $display("countdown=37 -> HEX5..HEX2 = %b %b : %b %b (expect 1111000 0110000 1000000 1000000)",
                 HEX5, HEX4, HEX3, HEX2);
        $stop;
    end
endmodule
