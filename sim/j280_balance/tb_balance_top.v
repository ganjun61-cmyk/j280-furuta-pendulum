`timescale 1ns/1ps
// Real 50 MHz / 230400 baud. Only capture hold/debounce are small test values.
module tb_balance_top;
    localparam integer BIT_NS=217*20;
    reg clk_50m=0,rst_n=0,uart_rx=1;
    always #10 clk_50m=~clk_50m;
    reg [9:0] adc_data=735;
    reg adc_otr=0,enc1_a=0,enc1_b=0,key_sw1=1,key_sw2=1,dip_sw1=1;
    wire adc_clk,adc_oe_n,uart_tx,pwm_a,ain1,ain2,pwm_b,bin1,bin2;
    wire [3:0] led;
    j280_balance_top #(.DEBOUNCE_CLOCKS(20),.MIN_HOLD_MS(2)) dut(
        .clk_50m(clk_50m),.rst_n(rst_n),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .adc_data(adc_data),.adc_otr(adc_otr),.enc1_a(enc1_a),.enc1_b(enc1_b),
        .key_sw1(key_sw1),.key_sw2(key_sw2),.dip_sw1(dip_sw1),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.pwm_a(pwm_a),.ain1(ain1),.ain2(ain2),
        .pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));
    reg [1535:0] commands[0:1];
    reg [7:0] octet;
    integer wire_file,pins,snapshots,events,i,b,bit_number,cycle=0,bytes=0;
    integer scenario=0;
    reg [2:0] previous=3'bxxx;
    task send_command;
        input integer index;
        begin
            for(i=0;i<192;i=i+1) begin
                uart_rx=0; #BIT_NS;
                for(b=0;b<8;b=b+1) begin uart_rx=commands[index][8*i+b]; #BIT_NS; end
                uart_rx=1; #BIT_NS;
            end
        end
    endtask
    always @(posedge clk_50m) begin
        cycle=cycle+1;
        if(dut.snapshot_valid)
            $fdisplay(snapshots,"%0d,%0d,%0d",cycle,dut.snapshot_tick,dut.raw_sequence-1);
    end
    always @(negedge clk_50m) begin
        if({pwm_a,ain1,ain2} !== 3'b000) $fatal(1,"A channel active");
        if(bin1 && bin2) $fatal(1,"invalid B direction");
        if(!bin1 && !bin2 && pwm_b !== 1) $fatal(1,"B inactive not Stop");
        if({pwm_b,bin1,bin2} !== previous) begin
            $fdisplay(pins,"%0d,%0d,%0d,%0d",cycle,pwm_b,bin1,bin2);
            previous={pwm_b,bin1,bin2};
        end
    end
    initial begin
        wire_file=$fopen("wire.bin","wb");
        forever begin
            @(negedge uart_tx); #(BIT_NS/2);
            if(uart_tx !== 0) $fatal(1,"UART start");
            for(bit_number=0;bit_number<8;bit_number=bit_number+1) begin
                #BIT_NS; octet[bit_number]=uart_tx;
            end
            #BIT_NS;
            if(uart_tx !== 1) $fatal(1,"UART stop");
            $fwrite(wire_file,"%c",octet); bytes=bytes+1;
        end
    end
    initial begin
        if(!$value$plusargs("CASE=%d",scenario)) scenario=0;
        $readmemh("commands.hex",commands);
        pins=$fopen("pins.csv","w");snapshots=$fopen("snapshots.csv","w");events=$fopen("events.csv","w");
        #95; rst_n=1;
        #1000000; send_command(0); // invalid last coefficient must not arm
        #600000; dip_sw1=0; #100000; key_sw1=0; #2000; key_sw1=1;
        #300000; dip_sw1=1; #1000000; send_command(1);
        #1000000; dip_sw1=0; #1000000; key_sw1=0; #2000; key_sw1=1;
        // Drive ADC through two small signs, leaving hardware angle trip separate.
        #16000000; adc_data=737;
        #6000000;
        if(scenario==1) adc_data=780;
        else adc_data=733;
        #10000000;
        if(scenario==0) begin
            key_sw2=0; $fdisplay(events,"%0d,stop_key",cycle);
            #200; if({pwm_b,bin1,bin2} !== 3'b100) $fatal(1,"local stop latency");
            #1800; key_sw2=1;
        end
        #16000000;
        wait(!dut.tx_busy); #10000;
        if(bytes<6*128 || bytes%128) $fatal(1,"telemetry frame count");
        if({pwm_b,bin1,bin2} !== 3'b100) $fatal(1,"final B not Stop");
        $fclose(wire_file);$fclose(pins);$fclose(snapshots);$fclose(events);
        $display("PASS tb_balance_top CASE=%0d bytes=%0d",scenario,bytes);$finish;
    end
    initial begin #100000000; $fatal(1,"watchdog"); end
endmodule
