// tb_input_handler.v
`timescale 1ns/1ps

module tb_input_handler;
    reg clk = 0, rst = 1;
    reg [3:0] KEY_raw = 4'b1111;   // all released (active-low)
    reg [1:0] SW_raw  = 2'b00;
    wire steer_left, steer_right, accel, brake;

    input_handler dut (
        .clk(clk), .rst(rst),
        .KEY_raw(KEY_raw), .SW_raw(SW_raw),
        .steer_left(steer_left), .steer_right(steer_right),
        .accel(accel), .brake(brake)
    );

    always #10 clk = ~clk;   // 50 MHz

    initial begin
        $display("tb_input_handler: start");
        #100 rst = 0;
        #100;
        // Simulate bouncing on KEY[0]
        KEY_raw[0] = 0; #40; KEY_raw[0] = 1; #40; KEY_raw[0] = 0; #40; KEY_raw[0] = 1; #40;
        KEY_raw[0] = 0; // stable pressed
        repeat (10) @(posedge clk);
        $display("steer_left after stable press = %b (expect 1)", steer_left);

        // Release and wait
        KEY_raw[0] = 1;
        repeat (10) @(posedge clk);
        $display("steer_left after release = %b (expect 0)", steer_left);

        // KEY2-driven accel (active-low)
        KEY_raw[2] = 0; repeat (10) @(posedge clk);
        $display("accel after KEY[2] press = %b (expect 1)", accel);

        // SW0-driven brake (active-high)
        SW_raw[0] = 1; repeat (10) @(posedge clk);
        $display("brake after SW[0]=1 = %b (expect 1)", brake);

        $stop;
    end
endmodule
