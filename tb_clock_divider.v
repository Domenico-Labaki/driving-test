// tb_clock_divider.v
`timescale 1ns/1ps

module tb_clock_divider;
    reg clk_50 = 0;
    reg rst   = 1;
    wire clk_25, clk_game;

    clock_divider dut (.clk_50(clk_50), .rst(rst), .clk_25(clk_25), .clk_game(clk_game));

    always #10 clk_50 = ~clk_50;   // 50 MHz

    integer edges25 = 0;
    always @(posedge clk_25) edges25 = edges25 + 1;

    initial begin
        $display("tb_clock_divider: start");
        #100 rst = 0;
        // Run for 2000 ns = 100 clk_50 cycles -> should see ~50 clk_25 rising edges
        #2000;
        $display("clk_25 rising edges in 2 us: %0d (expected ~50)", edges25);
        if (edges25 < 40 || edges25 > 60) $display("FAIL: clk_25 divide ratio off");
        else                              $display("PASS: clk_25 divide ratio OK");
        $stop;
    end
endmodule
