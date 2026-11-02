`timescale 1ns/1ps
// Real 50 MHz / 230400 baud. Only debounce/hold/settle are small test values.
// The ADC follows scripted pendulum paths (no plant): CASE 0 rises to upright
// and is caught, steps the arm reference with SW4/SW3, turns it with DIP 2,
// then SW2 stops; CASE 1 passes the dead zone (OTR) before the catch, then
// exceeds the exit angle; CASE 2 loses the ADC mid-range (blind entry); CASE 3
// sends nothing over UART: DIP 1 loads the built-in frames (standalone), SW1
// swings up, BALANCE outlasts the keepalive window, SW2 stops.
module tb_swing_top;
    localparam integer BIT_NS=217*20;
    localparam integer SAMPLE_NS=2000000;
    reg clk_50m=0,rst_n=0,uart_rx=1;
    always #10 clk_50m=~clk_50m;
    reg [9:0] adc_data=221;
    reg adc_otr=0,enc1_a=0,enc1_b=0,key_sw1=1,key_sw2=1,dip_sw1=1,key_sw3=1,key_sw4=1,dip_sw2=1;
    wire adc_clk,adc_oe_n,uart_tx,pwm_a,ain1,ain2,pwm_b,bin1,bin2;
    wire [3:0] led;
    j280_swing_top #(.DEBOUNCE_CLOCKS(20),.MIN_HOLD_MS(2),.SETTLE_MS(60)) dut(
        .clk_50m(clk_50m),.rst_n(rst_n),.uart_rx(uart_rx),.uart_tx(uart_tx),
        .adc_data(adc_data),.adc_otr(adc_otr),.enc1_a(enc1_a),.enc1_b(enc1_b),
        .key_sw1(key_sw1),.key_sw2(key_sw2),.dip_sw1(dip_sw1),
        .key_sw3(key_sw3),.key_sw4(key_sw4),.dip_sw2(dip_sw2),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.pwm_a(pwm_a),.ain1(ain1),.ain2(ain2),
        .pwm_b(pwm_b),.bin1(bin1),.bin2(bin2),.led(led));
    reg [1535:0] commands[0:4];   // bad CONFIG, CONFIG, SWING, GAINS2, keepalive
    reg [7:0] octet;
    integer wire_file,pins,snapshots,events,i,b,bit_number,cycle=0,bytes=0,code;
    integer scenario=0;
    reg signed [31:0] anchor;
    reg [2:0] previous=3'bxxx;
    reg [7:0] previous_state=0;
    reg previous_moving=0, previous_rotating=0;
    reg ka_on=0;
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
    // Host keepalive every 100 ms once a host session is running (CONFIG keepalive 200 ms).
    always begin
        #100000000;
        if(ka_on) begin send_command(4); $fdisplay(events,"%0d,keepalive",cycle); end
    end
    task ramp;
        input integer from, to, step;
        begin
            code=from;
            while((step>0 && code<to) || (step<0 && code>to)) begin
                adc_data=code; #SAMPLE_NS;
                code=code+step;
            end
            adc_data=to;
        end
    endtask
    task press;
        input integer which;
        begin
            if(which==3) key_sw3=0; else key_sw4=0;
            #2000;
            if(which==3) key_sw3=1; else key_sw4=1;
            #2000;
            $fdisplay(events,"%0d,sw%0d",cycle,which);
        end
    endtask
    // Minimal motor: one quadrature count per 2 ms in the driven direction
    // (BIN2 = increasing counts), so the guard's arm-stall check sees motion.
    reg [1:0] enc_phase=0;
    integer enc_count=0;
    always begin
        #(SAMPLE_NS);
        if(bin2) begin enc_phase=enc_phase+2'd1; enc_count=enc_count+1; end
        else if(bin1) begin enc_phase=enc_phase-2'd1; enc_count=enc_count-1; end
        {enc1_a,enc1_b}=(enc_phase==2'd0)?2'b00:(enc_phase==2'd1)?2'b10:(enc_phase==2'd2)?2'b11:2'b01;
    end
    always @(posedge clk_50m) begin
        cycle=cycle+1;
        if(dut.snapshot_valid)
            $fdisplay(snapshots,"%0d,%0d,%0d",cycle,dut.snapshot_tick,dut.raw_sequence-1);
        if(dut.state!==previous_state) begin
            $fdisplay(events,"%0d,state,%0d,%0d",cycle,dut.state,dut.stop_reason);
            previous_state=dut.state;
        end
        if(dut.catch_pulse) $fdisplay(events,"%0d,catch,%0d,%0d,%0d,%0d",cycle,
            dut.preset0,dut.preset1,dut.preset2,dut.preset3);
        if(dut.traj_moving!==previous_moving || dut.traj_rotating!==previous_rotating) begin
            $fdisplay(events,"%0d,traj,%0d,%0d,%0d",cycle,dut.traj_moving,dut.traj_rotating,dut.traj_target);
            previous_moving=dut.traj_moving; previous_rotating=dut.traj_rotating;
        end
    end
    always @(negedge clk_50m) begin
        if({pwm_a,ain1,ain2} !== 3'b000) $fatal(1,"A channel active");
        if(bin1 && bin2) $fatal(1,"invalid B direction");
        if(!bin1 && !bin2 && pwm_b !== 1) $fatal(1,"B inactive not Stop");
        if((bin1 || bin2) && dut.state!=8'd3 && dut.state!=8'd4) $fatal(1,"drive outside SWING/BALANCE");
        if({pwm_b,bin1,bin2} !== previous) begin
            $fdisplay(pins,"%0d,%0d,%0d,%0d",cycle,pwm_b,bin1,bin2);
            previous={pwm_b,bin1,bin2};
        end
        if(dut.state!=8'd3 && (dut.ref_r!=={dut.theta_ref,16'd0} || dut.ref_u_ff!==0))
            $fatal(1,"reference not at the anchor outside BALANCE");
        // Replica strobes must be bit-identical every clock (timing-only copies).
        if(dut.g_est_valid_b!==dut.g_est_valid || dut.g_est_valid_c!==dut.g_est_valid ||
           dut.g_core_valid_b!==dut.g_core_valid) $fatal(1,"replica strobe differs");
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
        if(scenario==3) begin
            // DIP 1 on loads the built-in frames at once; ARMED also waits for a
            // fresh estimator (a few 2 ms snapshots after reset).
            #1000000; dip_sw1=0; #100000; dip_sw1=1; #300000; dip_sw1=0; #20000000;
            if(dut.state!==8'd1 || !dut.swing_loaded || !dut.standalone || dut.config_id!==16'd6001)
                $fatal(1,"built-in frames did not arm");
        end else begin
            // A rejected CONFIG must not arm, and with DIP 1 kept off it must not
            // start the built-in (standalone) load either.
            #1000000; send_command(0); // invalid last coefficient must not arm
            #600000; key_sw1=0; #2000; key_sw1=1;
            #400000;
            if(dut.state!==8'd0 || dut.config_loaded || dut.standalone)
                $fatal(1,"rejected config armed or loaded");
            #1000000; send_command(1);
            #1000000; send_command(2);
            #1000000; send_command(3);  // GAINS2
            #1000000; dip_sw1=0; #1000000;
            if(dut.state!==8'd1 || !dut.swing_loaded || dut.standalone) $fatal(1,"not ARMED with swing block");
            ka_on=(scenario==0);
        end
        key_sw1=0; #2000; key_sw1=1; $fdisplay(events,"%0d,sw1",cycle);
        #(SAMPLE_NS);
        if(dut.state!==8'd4) $fatal(1,"SW1 did not start SWING from hanging");
        #(10*SAMPLE_NS);
        if(enc_count<5 || dut.position!==enc_count) $fatal(1,"kick did not drive the counts upward");
        if(scenario==0 || scenario==3) ramp(221,737,20);
        else if(scenario==1) begin
            ramp(221,1,-20);
            adc_otr=1; adc_data=0; $fdisplay(events,"%0d,otr_on",cycle);
            #(10*SAMPLE_NS);
            adc_otr=0; adc_data=1010; $fdisplay(events,"%0d,otr_off",cycle);
            #(SAMPLE_NS);
            ramp(1010,737,-20);
        end else begin
            ramp(221,501,20);
            adc_otr=1; $fdisplay(events,"%0d,otr_on",cycle);
        end
        #(15*SAMPLE_NS);
        if(scenario==2) begin
            if(dut.state!==8'd255 || dut.stop_reason!==8'd26) $fatal(1,"mid-range blind did not fault");
        end else begin
            if(dut.state!==8'd3) $fatal(1,"no catch into BALANCE");
            if(scenario==0) begin
                anchor=dut.theta_ref;
                // Before the settle time the keys are ignored.
                if(dut.state_elapsed_ms>=60) $fatal(1,"bench too slow for the settle check");
                press(4); #(3*SAMPLE_NS);
                if(dut.traj_target!==anchor) $fatal(1,"step accepted before settle");
            end
            adc_data=735; #(5*SAMPLE_NS);
            adc_data=739; #(5*SAMPLE_NS);
            if(scenario==0) begin
                #(15*SAMPLE_NS);
                if(dut.state!==8'd3 || dut.state_elapsed_ms<60) $fatal(1,"not settled in BALANCE");
                press(4); #(2*SAMPLE_NS);
                if(dut.traj_target!==anchor+8 || !dut.traj_moving) $fatal(1,"SW4 did not step the reference");
                #(30*SAMPLE_NS);
                if(dut.traj_moving || dut.ref_r!=={anchor+32'sd8,16'd0}) $fatal(1,"step did not land");
                press(3); #(2*SAMPLE_NS); press(3); #(40*SAMPLE_NS);
                if(dut.ref_r!=={anchor-32'sd8,16'd0}) $fatal(1,"two SW3 steps did not land");
                dip_sw2=0; $fdisplay(events,"%0d,dip2_on",cycle);
                #(30*SAMPLE_NS);
                if(!dut.traj_rotating) $fatal(1,"DIP 2 did not start turning");
                if(dut.ref_r>={anchor-32'sd8,16'd0}) $fatal(1,"turn not in the last step direction");
                dip_sw2=1; $fdisplay(events,"%0d,dip2_off",cycle);
                #(40*SAMPLE_NS);
                if(dut.traj_rotating || dut.traj_moving) $fatal(1,"turn did not stop");
                if(dut.ref_r[15:0]!==16'd0 || ((dut.ref_r>>>16)-anchor)%8!=0) $fatal(1,"turn did not stop on the grid");
                key_sw2=0; $fdisplay(events,"%0d,stop_key",cycle);
                #200; if({pwm_b,bin1,bin2} !== 3'b100) $fatal(1,"local stop latency");
                #1800; key_sw2=1;
            end else if(scenario==3) begin
                // The built-in keepalive is 20 ms: 30 ms more without any UART frame.
                #(15*SAMPLE_NS);
                if(dut.state!==8'd3 || dut.state_elapsed_ms<40)
                    $fatal(1,"standalone BALANCE did not outlast the keepalive window");
                key_sw2=0; $fdisplay(events,"%0d,stop_key",cycle);
                #2000; key_sw2=1;
            end else begin
                adc_data=818; $fdisplay(events,"%0d,angle",cycle);
            end
        end
        #(8*SAMPLE_NS);
        wait(!dut.tx_busy); #10000;
        if(bytes<6*128 || bytes%128) $fatal(1,"telemetry frame count");
        if({pwm_b,bin1,bin2} !== 3'b100) $fatal(1,"final B not Stop");
        $fclose(wire_file);$fclose(pins);$fclose(snapshots);$fclose(events);
        $display("PASS tb_swing_top CASE=%0d bytes=%0d",scenario,bytes);$finish;
    end
    initial begin #900000000; $fatal(1,"watchdog"); end
endmodule
