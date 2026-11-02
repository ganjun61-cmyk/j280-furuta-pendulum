`timescale 1ns/1ps
// Full physical clock/baud simulation. Python supplies complete CRC command
// frames and a scenario; all motor checks below inspect the actual top pins.
module tb_motor_test_top;
    parameter integer DEBOUNCE_CLOCKS = 1000000;
    localparam integer BIT_NS = 217 * 20;
    reg clk_50m = 0;
    always #10 clk_50m = ~clk_50m;
    reg rst_n = 0, uart_rx = 1;
    reg [9:0] adc_data = 220;
    reg adc_otr = 0, enc1_a = 0, enc1_b = 0;
    reg key_sw1 = 1, key_sw2 = 1, dip_sw1 = 1;
    wire adc_clk, adc_oe_n, uart_tx;
    wire pwm_a, ain1, ain2, pwm_b, bin1, bin2;
    wire [3:0] led;
    j280_motor_test_top #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS)) dut (
        .clk_50m(clk_50m),.rst_n(rst_n),.adc_data(adc_data),.adc_otr(adc_otr),
        .enc1_a(enc1_a),.enc1_b(enc1_b),.uart_rx(uart_rx),.key_sw1(key_sw1),
        .key_sw2(key_sw2),.dip_sw1(dip_sw1),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),
        .uart_tx(uart_tx),.pwm_a(pwm_a),.ain1(ain1),.ain2(ain2),
        .pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));

    reg [159:0] commands [0:3];
    integer script, wire_file, snapshots, pins, rc, i, bit_number;
    integer cycle = 0, byte_count = 0;
    reg [15:0] op;
    reg [31:0] arg;
    reg [7:0] received;
    reg [2:0] last_pins = 3'bxxx;
    reg [7:0] expected_mode;

    task send_command;
        input integer command_index;
        integer octet, bit_index;
        begin
            for (octet=0; octet<20; octet=octet+1) begin
                uart_rx = 0;
                #BIT_NS;
                for (bit_index=0; bit_index<8; bit_index=bit_index+1) begin
                    uart_rx = commands[command_index][octet*8+bit_index];
                    #BIT_NS;
                end
                uart_rx = 1;
                #BIT_NS;
            end
        end
    endtask

    initial begin
        wire_file = $fopen("wire.bin", "wb");
        forever begin
            @(negedge uart_tx);
            #(BIT_NS/2);
            if (uart_tx !== 0) $fatal(1,"FAIL UART start");
            for (bit_number=0;bit_number<8;bit_number=bit_number+1) begin
                #BIT_NS;
                received[bit_number] = uart_tx;
            end
            #BIT_NS;
            if (uart_tx !== 1) $fatal(1,"FAIL UART stop");
            $fwrite(wire_file,"%c",received);
            byte_count = byte_count + 1;
        end
    end

    // This monitor samples the source signals BEFORE the snapshot edge's NBA
    // updates. The reference comes from acquisition/output inputs, not payload.
    always @(posedge clk_50m) begin
        cycle = cycle + 1;
        if (dut.reset_ok && dut.sample_divider == 99999) begin
            expected_mode = {5'd0,!(bin1 || bin2),bin2,bin1};
            $fdisplay(snapshots,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                dut.frame_sequence,dut.tick_next[31:0],dut.raw,dut.position,
                dut.applied_compare,expected_mode,dut.state,dut.config_id,dut.config_crc);
        end
    end
    always @(negedge clk_50m) begin
        // The motor on XH1 is TB6612 channel B; channel A (XH2) stays 0.
        if ({pwm_a,ain1,ain2} !== 3'b000) $fatal(1,"FAIL A channel active");
        if (bin1 && bin2) $fatal(1,"FAIL invalid drive direction");
        if (!bin1 && !bin2 && pwm_b !== 1) $fatal(1,"FAIL idle Stop combination");
        if (dut.tx_overflow) $fatal(1,"FAIL top telemetry overrun");
        if ({pwm_b,bin1,bin2} !== last_pins) begin
            $fdisplay(pins,"%0d %0d %0d %0d",cycle,pwm_b,bin1,bin2);
            last_pins = {pwm_b,bin1,bin2};
        end
    end

    initial begin
        script=$fopen("scenario.txt","r");
        snapshots=$fopen("snapshots.csv","w");
        pins=$fopen("pins.txt","w");
        if (!script || !snapshots || !pins) $fatal(1,"FAIL opening files");
        $readmemh("commands.hex",commands);
        #65 rst_n=1;
        while (!$feof(script)) begin
            rc=$fscanf(script,"%s",op);
            if (rc == 1) begin
                if (op == "E") begin
                    wait (!dut.tx_busy && !dut.snapshot_valid);
                    repeat(2) @(negedge clk_50m);
                    if (byte_count == 0 || byte_count % 40) $fatal(1,"FAIL truncated UART");
                    $fclose(wire_file); $fclose(snapshots); $fclose(pins); $fclose(script);
                    $display("PASS tb_motor_test_top bytes=%0d",byte_count);
                    $finish;
                end else begin
                    rc=$fscanf(script," %d",arg);
                    if (rc != 1) $fatal(1,"FAIL script argument");
                    case(op)
                        "W": #(arg);
                        "C": send_command(arg);
                        "D": dip_sw1=arg[0];
                        "S1": key_sw1=arg[0];
                        "S2": key_sw2=arg[0];
                        "O": adc_otr=arg[0];
                        "A": adc_data=arg[9:0];
                        default: $fatal(1,"FAIL unknown operation");
                    endcase
                end
            end
        end
        $fatal(1,"FAIL missing scenario end");
    end
    initial begin #500000000; $fatal(1,"FAIL top watchdog"); end
endmodule
