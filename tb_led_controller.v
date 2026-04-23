// tb_led_controller.v
`timescale 1ns/1ps

module tb_led_controller;
    reg clk = 0, rst = 1;
    reg collision_detected = 0, in_parking_zone = 0, in_stop_zone = 0;
    reg [2:0] game_state = 0, speed = 0;
    reg pass = 0;
    wire [17:0] LEDR;
    wire [8:0]  LEDG;

    led_controller dut (
        .clk(clk), .rst(rst),
        .collision_detected(collision_detected),
        .in_parking_zone(in_parking_zone),
        .in_stop_zone(in_stop_zone),
        .game_state(game_state),
        .speed(speed), .pass(pass),
        .LEDR(LEDR), .LEDG(LEDG)
    );
    always #10 clk = ~clk;

    initial begin
        $display("tb_led_controller: start");
        #100 rst = 0; #40;

        collision_detected = 1; #20;
        $display("Collision: LEDR=%h (expect 3FFFF)", LEDR);

        collision_detected = 0; in_parking_zone = 1; speed = 3; #20;
        $display("Park+speed=3: LEDG=%b (expect bit0=1, bits 1..3 on)", LEDG);

        game_state = 3'd5; pass = 1; #20;
        $display("DONE+pass: LEDG=%h (expect 1FF)", LEDG);

        $stop;
    end
endmodule
