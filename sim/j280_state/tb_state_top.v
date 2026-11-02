`timescale 1ns/1ps
// Real-timing top run: records UART pin bytes to wire.bin. Scenario:
// 0-20 ms static; 20.25-60 ms encoder +1 count / 500 us and ADC +1 code / 1 ms;
// 60.75-64.5 ms ADC over-range (sensor invalid); then static until the end.
module tb_state_top;
    localparam integer BIT_NS = 217 * 20;
    reg clk_50m = 0;
    always #10 clk_50m = ~clk_50m;
    reg rst_n = 0;
    reg [9:0] adc_data = 10'd500;
    reg adc_otr = 0;
    reg enc1_a = 0, enc1_b = 0;
    wire adc_clk, adc_oe_n, uart_tx;
    wire pwm_a, ain1, ain2, pwm_b, bin1, bin2;
    wire [1:0] led;
    j280_state_top dut(.clk_50m(clk_50m),.rst_n(rst_n),.adc_data(adc_data),
        .adc_otr(adc_otr),.enc1_a(enc1_a),.enc1_b(enc1_b),.adc_clk(adc_clk),
        .adc_oe_n(adc_oe_n),.uart_tx(uart_tx),.pwm_a(pwm_a),.ain1(ain1),
        .ain2(ain2),.pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));
    integer stream, i, step, byte_count = 0;
    reg [7:0] value;
    reg led_seen = 0;

    initial begin
        stream = $fopen("wire.bin","wb");
        if (!stream) begin $display("FAIL cannot open wire.bin"); $finish; end
        forever begin
            @(negedge uart_tx);
            #(BIT_NS/2);
            if (uart_tx !== 0) begin $display("FAIL invalid UART start bit"); $finish; end
            for (i=0;i<8;i=i+1) begin
                #(BIT_NS);
                value[i] = uart_tx;
            end
            #(BIT_NS);
            if (uart_tx !== 1) begin $display("FAIL invalid UART stop bit"); $finish; end
            $fwrite(stream,"%c",value);
            byte_count = byte_count+1;
        end
    end
    always @(negedge clk_50m) begin
        if ({pwm_a,ain1,ain2,pwm_b,bin1,bin2} !== 6'b0) begin
            $display("FAIL motor pin became active"); $finish;
        end
        if (led[0] === 1'b1) led_seen = 1;
    end
    initial begin
        #65 rst_n = 1;
        #(20250000-65);
        for (step = 0; step < 80; step = step + 1) begin
            case (step % 4)
                0: enc1_a = 1;
                1: enc1_b = 1;
                2: enc1_a = 0;
                3: enc1_b = 0;
            endcase
            if (step % 2 == 1) adc_data = adc_data + 10'd1;
            #500000;
        end
        #500000;
        adc_otr = 1; adc_data = 10'd1023;
        #3750000;
        adc_otr = 0; adc_data = 10'd540;
        #25000000;
        wait (uart_tx === 1'b1 && dut.tx_busy === 1'b0);
        #(BIT_NS);
        $fclose(stream);
        if (byte_count % 128 != 0) begin $display("FAIL truncated frame bytes=%0d", byte_count); $finish; end
        if (!led_seen) begin $display("FAIL LED0 never showed all-valid"); $finish; end
        rst_n = 0;
        #100;
        if (uart_tx !== 1 || adc_clk !== 0 || led[0] !== 0) begin $display("FAIL top reset"); $finish; end
        $display("PASS tb_state_top bytes=%0d", byte_count);
        $finish;
    end
    initial begin #200000000; $display("FAIL top test timeout"); $finish; end
endmodule
