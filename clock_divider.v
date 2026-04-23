module clock_divider (
    input  wire clk_50,
    input  wire rst,
    output wire clk_25,
    output reg  clk_game
);

    // --- 25 MHz via PLL (proper clock network routing) ---
    wire pll_locked;

    altpll #(
        .intended_device_family ("Cyclone II"),
        .lpm_type               ("altpll"),
        .pll_type               ("AUTO"),
        .operation_mode         ("NORMAL"),
        .inclk0_input_frequency (20000),   // 50 MHz = 20 ns period
        .clk0_divide_by         (2),
        .clk0_multiply_by       (1),
        .clk0_phase_shift       ("0"),
        .port_clkena0           ("PORT_UNUSED"),
        .compensate_clock       ("CLK0")
    ) u_pll (
        .inclk  ({1'b0, clk_50}),
        .clk    (clk_25),          // clk[0] = 25 MHz
        .locked (pll_locked),
        .activeclock (),
        .areset (1'b0),
        .clkbad (),
        .clkena ({6{1'b1}}),
        .clkloss (),
        .clkswitch (1'b0),
        .configupdate (1'b0),
        .enable0 (),
        .enable1 (),
        .extclk (),
        .extclkena ({4{1'b1}}),
        .fbin (1'b1),
        .fbmimicbidir (),
        .fbout (),
        .fref (),
        .icdrclk (),
        .pfdena (1'b1),
        .phasecounterselect ({4{1'b1}}),
        .phasedone (),
        .phasestep (1'b1),
        .phaseupdown (1'b1),
        .pllena (1'b1),
        .scanaclr (1'b0),
        .scanclk (1'b0),
        .scanclkena (1'b1),
        .scandata (1'b0),
        .scandataout (),
        .scandone (),
        .scanread (1'b0),
        .scanwrite (1'b0),
        .sclkout0 (),
        .sclkout1 (),
        .vcooverrange (),
        .vcounderrange ()
    );

    // --- ~60 Hz game tick (fabric toggle is fine for slow logic) ---
    localparam integer GAME_DIV = 20'd416_666;
    reg [19:0] cnt;

    always @(posedge clk_50 or posedge rst) begin
        if (rst) begin
            cnt      <= 20'd0;
            clk_game <= 1'b0;
        end
        else if (cnt == GAME_DIV) begin
            cnt      <= 20'd0;
            clk_game <= ~clk_game;
        end
        else cnt <= cnt + 20'd1;
    end

endmodule