`timescale 1ns/1ps
module tb_output_age;
    reg clk_50m=0;
    always #10 clk_50m=~clk_50m;
    reg rst_n=0;
    wire adc_clk,adc_oe_n,uart_tx,pwm_a,ain1,ain2,pwm_b,bin1,bin2;
    wire [1:0]led;
    j280_measure_top dut(.clk_50m(clk_50m),.rst_n(rst_n),
        .adc_data(10'd100),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.uart_tx(uart_tx),
        .pwm_a(pwm_a),.ain1(ain1),.ain2(ain2),.pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));
    initial begin
        #65 rst_n=1;
        #100;
        // Accelerate only the microsecond enable to cover 65 ms efficiently.
        force dut.us_divider=6'd49;
        repeat(65540) @(negedge clk_50m);
        if (dut.pwm_age !== 16'hffff) $fatal(1,"output age did not saturate");
        @(negedge clk_50m);
        force dut.tick_us=32'hfffffff0;
        @(negedge clk_50m);
        release dut.tick_us;
        repeat(40) @(negedge clk_50m);
        if (dut.pwm_age !== 16'hffff) $fatal(1,"timestamp wrap reset unchanged output age");
        release dut.us_divider;
        $display("PASS tb_output_age"); $finish;
    end
    initial begin #2000000; $fatal(1,"output age test timeout"); end
endmodule
