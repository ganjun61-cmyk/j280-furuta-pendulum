`timescale 1ns/1ps
module tb_measure_top;
    reg clk_50m = 0;
    always #10 clk_50m = ~clk_50m;
    reg rst_n = 0;
    reg [9:0] adc_data = 10'd513;
    reg adc_otr = 0;
    reg enc1_a = 0, enc1_b = 0;
    wire adc_clk, adc_oe_n, uart_tx;
    wire pwm_a, ain1, ain2, pwm_b, bin1, bin2;
    wire [1:0] led;
    j280_measure_top dut(.clk_50m(clk_50m),.rst_n(rst_n),.adc_data(adc_data),
        .adc_otr(adc_otr),.enc1_a(enc1_a),.enc1_b(enc1_b),.adc_clk(adc_clk),
        .adc_oe_n(adc_oe_n),.uart_tx(uart_tx),.pwm_a(pwm_a),.ain1(ain1),
        .ain2(ain2),.pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));
    integer stream;
    integer byte_count = 0;
    integer i;
    reg [7:0] value;
    reg [1023:0] output_path;
    // Observe the real UART pin rather than accessing the sender payload.
    initial begin
        if (!$value$plusargs("OUT=%s",output_path)) output_path = "top-uart.bin";
        stream = $fopen(output_path,"wb");
        if (!stream) $fatal(1,"cannot open UART fixture");
        forever begin
            @(negedge uart_tx);
            #(217*20/2);
            if (uart_tx !== 0) $fatal(1,"invalid UART start bit");
            for (i=0;i<8;i=i+1) begin
                #(217*20);
                value[i] = uart_tx;
            end
            #(217*20);
            if (uart_tx !== 1) $fatal(1,"invalid UART stop bit");
            $fwrite(stream,"%c",value);
            byte_count = byte_count+1;
        end
    end
    always @(negedge clk_50m) begin
        if ({pwm_a,ain1,ain2,pwm_b,bin1,bin2} !== 6'b0)
            $fatal(1,"motor pin became active");
    end
    initial begin
        #65 rst_n = 1;
        #500000;
        enc1_a=1; #200; enc1_b=1; #200;
        enc1_a=0; #200; enc1_b=0;
        #3699400; // after two emitted frames, introduce sensor fault
        adc_otr=1; adc_data=10'd1023;
        #4000000;
        adc_otr=0; adc_data=10'd17;
        #3600000;
        rst_n=0;
        #500;
        if (uart_tx !== 1 || adc_clk !== 0) $fatal(1,"top reset failed");
        $fclose(stream);
        if (byte_count < 160 || byte_count % 32 != 0) $fatal(1,"frame count/truncation");
        $display("PASS tb_measure_top bytes=%0d",byte_count);
        $finish;
    end
    initial begin #15000000; $fatal(1,"top test timeout"); end
endmodule
