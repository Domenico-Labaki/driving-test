// tb_game_fsm.v
`timescale 1ns/1ps

module tb_game_fsm;
    reg clk = 0, rst = 1;
    reg SW_en = 0;
    reg collision_detected = 0;
    reg at_start_line = 0;
    reg in_stop_zone = 0, in_parking_zone = 0;
    reg [2:0] speed = 0;
    wire [2:0] game_state;
    wire pass;
    wire [2:0] lives;
    wire [5:0] countdown_sec;

    game_fsm dut (
        .clk(clk), .rst(rst), .SW_en(SW_en),
        .collision_detected(collision_detected),
        .at_start_line(at_start_line),
        .in_stop_zone(in_stop_zone), .in_parking_zone(in_parking_zone),
        .speed(speed),
        .game_state(game_state), .pass(pass),
        .lives(lives), .countdown_sec(countdown_sec)
    );

    always #10 clk = ~clk;

    initial begin
        $display("tb_game_fsm: start");
        #100 rst = 0;
        #40;
        $display("State after reset = %0d (expect 0=IDLE), lives=%0d", game_state, lives);

        // Start game
        SW_en = 1; #40;
        at_start_line = 1; #20; at_start_line = 0; #20;
        $display("State after SW_en = %0d (expect 1=PLAYING)", game_state);

        // Collision
        collision_detected = 1; #20; collision_detected = 0; #40;
        $display("State after collision = %0d (expect 2=COLLIDED), lives=%0d (expect 2)",
                 game_state, lives);

        // Wait for penalty to expire (60 ticks @ 20ns clk period -> 60*20=1200)
        #1400;
        $display("State after penalty = %0d (expect 1=PLAYING)", game_state);

        // Reach parking zone and stop
        in_parking_zone = 1; #40;
        $display("State when in parking = %0d (expect 4=PARKING)", game_state);
        speed = 0; #60;
        $display("State after stopping in park = %0d (expect 5=DONE), pass=%b (expect 1)",
                 game_state, pass);

        $stop;
    end
endmodule
