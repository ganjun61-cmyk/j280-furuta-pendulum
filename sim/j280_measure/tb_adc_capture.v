`timescale 1ns/1ps
module tb_adc_capture;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg [9:0] adc_data = 0;
    reg adc_otr = 0;
    wire adc_clk, adc_oe_n;
    wire [9:0] raw;
    wire valid, sample_strobe;
    wire [15:0] age_us;
    j280_adc_capture dut(.clk(clk), .rst_n(rst_n),
        .adc_data(adc_data), .adc_otr(adc_otr),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n),
        .raw(raw), .valid(valid), .sample_strobe(sample_strobe), .age_us(age_us));

    integer captures = 0;
    time last_rise = 0;
    always @(posedge adc_clk) begin
        if (last_rise && $time-last_rise != 200) $fatal(1,"ADC period is not 200 ns");
        last_rise = $time;
        // Model a parallel data output settling 25 ns after ADC rising clock.
        #25 adc_data = 10'd341;
    end
    always @(posedge clk) if (sample_strobe) captures = captures+1;
    initial begin
        #63 rst_n = 1;
        #700;
        if (valid) $fatal(1,"ADC became valid before pipeline warmup");
        #2200;
        if (!valid || raw !== 10'd341) $fatal(1,"raw ADC sample missing or scaled");
        if (captures < 3 || adc_oe_n !== 0) $fatal(1,"ADC event/enable failure");
        if (age_us < 1 || age_us > 2) $fatal(1,"ADC pipeline age estimate wrong");
        adc_otr = 1;
        #500;
        if (valid) $fatal(1,"overrange not invalidated");
        adc_otr = 0;
        #500;
        if (!valid) $fatal(1,"overrange recovery failed");
        rst_n = 0;
        #40;
        if (valid || raw !== 0 || adc_clk !== 0) $fatal(1,"ADC reset failed");
        $display("PASS tb_adc_capture");
        $finish;
    end
    initial begin #10000; $fatal(1,"ADC test timeout"); end
endmodule
