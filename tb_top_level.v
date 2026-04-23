// tb_top_level.v
// Integration testbench. Drives CLOCK_50, asserts SW[17] to start,
// pokes buttons, and observes that state transitions happen.
// Note: real clk_game period is 8.33 ms -> full scenario would take seconds.
// For fast sim we force-override internal clk_game (not possible cleanly in
// pure Verilog-2001 without hierarchical refs). Instead, we just verify that
// reset-out-of behaviour is correct and VGA sync signals are toggling.
`timescale 1ns/1ps

module tb_top_level;
    reg CLOCK_50 = 0;
    reg [3:0] KEY = 4'b1111;    // all released
    reg [17:0] SW = 18'd0;

    wire [3:0] VGA_R, VGA_G, VGA_B;
    wire VGA_HS, VGA_VS;
    wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
    wire [7:0] LCD_DATA;
    wire LCD_EN, LCD_RS, LCD_RW, LCD_ON, LCD_BLON;
    wire [17:0] LEDR;
    wire [8:0]  LEDG;
    wire I2C_SDAT;   // tri-state
    pullup (I2C_SDAT);
    wire I2C_SCLK, AUD_XCK, AUD_BCLK, AUD_DACLRCK, AUD_DACDAT;

    top_level dut (
        .CLOCK_50(CLOCK_50),
        .KEY(KEY), .SW(SW),
        .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B),
        .VGA_HS(VGA_HS), .VGA_VS(VGA_VS),
        .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2),
        .HEX3(HEX3), .HEX4(HEX4), .HEX5(HEX5),
        .LCD_DATA(LCD_DATA), .LCD_EN(LCD_EN), .LCD_RS(LCD_RS),
        .LCD_RW(LCD_RW), .LCD_ON(LCD_ON), .LCD_BLON(LCD_BLON),
        .LEDR(LEDR), .LEDG(LEDG),
        .I2C_SDAT(I2C_SDAT), .I2C_SCLK(I2C_SCLK),
        .AUD_XCK(AUD_XCK), .AUD_BCLK(AUD_BCLK),
        .AUD_DACLRCK(AUD_DACLRCK), .AUD_DACDAT(AUD_DACDAT)
    );

    always #10 CLOCK_50 = ~CLOCK_50;  // 50 MHz

    // Simple VGA sync heartbeat check
    integer hs_edges = 0;
    always @(posedge VGA_HS) hs_edges = hs_edges + 1;

    initial begin
        $display("tb_top_level: start");
        // Reset
        KEY[3] = 0;
        #200;
        KEY[3] = 1;
        #1000;

        // Start game
        SW[17] = 1;
        $display("Asserted SW[17]");

        // Run for a while
        #50000;
        $display("After 50us: HS edges=%0d (expect >0)", hs_edges);
        $display("LEDR=%h LEDG=%h", LEDR, LEDG);

        $stop;
    end
endmodule
