// tb_collision_detector.v
`timescale 1ns/1ps

module tb_collision_detector;
    reg [9:0] veh_x = 60;
    reg [8:0] veh_y = 60;
    wire collision_detected, in_stop_zone, in_parking_zone;

    collision_detector dut (
        .veh_x(veh_x), .veh_y(veh_y),
        .collision_detected(collision_detected),
        .in_stop_zone(in_stop_zone), .in_parking_zone(in_parking_zone)
    );

    initial begin
        $display("tb_collision_detector: start");
        #10 $display("Start pos (60,60): collision=%b stop=%b park=%b (expect 0,0,0)",
                     collision_detected, in_stop_zone, in_parking_zone);

        // Off-grass
        veh_x = 10; veh_y = 10; #10;
        $display("Grass (10,10): collision=%b (expect 1)", collision_detected);

        // Stop zone
        veh_x = 290; veh_y = 60; #10;
        $display("Stop (290,60): collision=%b stop=%b (expect 0,1)",
                 collision_detected, in_stop_zone);

        // Park zone
        veh_x = 40; veh_y = 430; #10;
        $display("Park (40,430): collision=%b park=%b (expect 0,1)",
                 collision_detected, in_parking_zone);

        // Middle road (segment C)
        veh_x = 300; veh_y = 230; #10;
        $display("Mid road (300,230): collision=%b (expect 0)", collision_detected);

        $stop;
    end
endmodule
