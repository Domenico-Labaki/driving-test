// tb_vehicle_controller.v
`timescale 1ns/1ps

module tb_vehicle_controller;
    reg clk_game = 0, rst = 1;
    reg game_active = 0, freeze = 0;
    reg accel = 0, brake = 0, steer_left = 0, steer_right = 0;
    wire [9:0] veh_x;
    wire [8:0] veh_y;
    wire [2:0] speed;
    wire [2:0] direction;

    vehicle_controller dut (
        .clk_game(clk_game), .rst(rst),
        .game_active(game_active), .freeze(freeze),
        .accel(accel), .brake(brake),
        .steer_left(steer_left), .steer_right(steer_right),
        .veh_x(veh_x), .veh_y(veh_y),
        .speed(speed), .direction(direction)
    );

    always #10 clk_game = ~clk_game;

    initial begin
        $display("tb_vehicle_controller: start");
        #100 rst = 0;
        #40;
        $display("Start pos x=%0d y=%0d dir=%0d (expect 60,60,2)",
                 veh_x, veh_y, direction);

        // Activate game, accel for a while
        game_active = 1; accel = 1;
        repeat (200) @(posedge clk_game);
        $display("After accel, speed=%0d x=%0d (should be >0 and >60)", speed, veh_x);

        // Brake to stop
        accel = 0; brake = 1;
        repeat (200) @(posedge clk_game);
        $display("After brake, speed=%0d (expect 0)", speed);

        // Steer right from stop should NOT change direction
        brake = 0; steer_right = 1;
        repeat (5) @(posedge clk_game);
        $display("Direction after steer at speed=0: %0d (expect unchanged=2)", direction);
        steer_right = 0;

        // Accel then steer
        accel = 1;
        repeat (50) @(posedge clk_game);
        steer_right = 1; @(posedge clk_game); steer_right = 0;
        repeat (5) @(posedge clk_game);
        $display("Direction after steer right at speed>0: %0d (expect 3=DOWN-RIGHT)", direction);

        $stop;
    end
endmodule
