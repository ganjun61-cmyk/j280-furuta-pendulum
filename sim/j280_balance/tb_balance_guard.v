`timescale 1ns/1ps

// File-driven self checks use only the public port contract. Clock scaling is
// 20 clocks/ms, independent 500 Hz snapshots are 40 clocks apart. No force is
// used. Hierarchical reads only cross-check the timing-optimized decision
// signals against the spec formulas; behavioral assertions use public ports.
module tb_balance_guard;
    parameter integer MIN_HOLD_MS = 2;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, cmd_valid=0, cmd_error=0;
    reg [1503:0] cmd_payload=0;
    reg [15:0] cmd_crc=0;
    reg start_event=0, stop_key=0, arm_on_raw=0, arm_on_stable=0, keys_released=1;
    reg sample_valid=0;
    reg [63:0] sample_tick_us=0;
    reg [9:0] adc_raw=500;
    reg [15:0] adc_age_us=0, sensor_fault_bits=0;
    reg signed [31:0] encoder_position=0;
    reg est_valid=0;
    reg [2:0] est_quality=7;
    reg signed [31:0] est_encoder_velocity_q16=0, est_adc_velocity_q16=0;
    reg core_valid=0, core_fault=0, core_overrun=0, pwm_reject=0;
    reg signed [31:0] core_theta_velocity_q16=0, core_alpha_velocity_q16=0;
    wire [7:0] state, stop_reason, rx_status;
    wire config_loaded, output_allow, takeover_pulse, est_fresh, est_ok, core_fresh;
    wire [15:0] config_id, config_crc, max_compare, comp_compare, blend_compare;
    wire [1023:0] coefficients;
    wire signed [31:0] comp_gain_q20, upright_zero_q16, theta_ref, safety_position_delta;
    wire [31:0] state_elapsed_ms;
    j280_balance_guard #(.CLOCKS_PER_MS(20), .STALE_CLOCKS(100),
        .ARM_TIMEOUT_MS(300), .MIN_HOLD_MS(MIN_HOLD_MS)) dut (
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_payload(cmd_payload),.cmd_crc(cmd_crc),
        .cmd_error(cmd_error),.start_event(start_event),.stop_key(stop_key),
        .arm_on_raw(arm_on_raw),.arm_on_stable(arm_on_stable),.keys_released(keys_released),
        .sample_valid(sample_valid),.sample_tick_us(sample_tick_us),.adc_raw(adc_raw),
        .adc_age_us(adc_age_us),.sensor_fault_bits(sensor_fault_bits),.encoder_position(encoder_position),
        .est_valid(est_valid),.est_quality(est_quality),.est_encoder_velocity_q16(est_encoder_velocity_q16),
        .est_adc_velocity_q16(est_adc_velocity_q16),.core_valid(core_valid),.core_fault(core_fault),
        .core_overrun(core_overrun),.core_theta_velocity_q16(core_theta_velocity_q16),
        .core_alpha_velocity_q16(core_alpha_velocity_q16),.pwm_reject(pwm_reject),
        .state(state),.stop_reason(stop_reason),.rx_status(rx_status),.config_loaded(config_loaded),
        .config_id(config_id),.config_crc(config_crc),.coefficients(coefficients),.max_compare(max_compare),
        .comp_compare(comp_compare),.blend_compare(blend_compare),.comp_gain_q20(comp_gain_q20),
        .upright_zero_q16(upright_zero_q16),.theta_ref(theta_ref),.safety_position_delta(safety_position_delta),
        .output_allow(output_allow),.takeover_pulse(takeover_pulse),.est_fresh(est_fresh),
        .est_ok(est_ok),.core_fresh(core_fresh),.state_elapsed_ms(state_elapsed_ms));

    integer fd, rc, line=0, cycle=0, phase=0, n, e_state, e_reason, e_loaded, e_rx;
    integer e_id, e_crc, e_max, e_comp, e_blend, e_gain, e_zero, a, b, c;
    reg [2047:0] vector_path;
    reg [255:0] op, name;
    reg signed [63:0] value;
    reg [63:0] tick_value, tick_clock=0;
    reg [1023:0] expected_coefficients;
    reg samples_on=1, est_on=1, core_on=1, bad_tick=0, abort_now=0;
    reg force_sample=0, force_est=0, force_core=0;
    reg [7:0] previous_state=0;
    reg previous_loaded=0;
    wire [1135:0] public_parameters={coefficients,max_compare,comp_compare,blend_compare,
                                    comp_gain_q20,upright_zero_q16};
    reg [1135:0] previous_parameters=0;
    reg signed [32:0] alpha_ref;
    reg [32:0] alpha_abs_ref;

    task fail;
        input [511:0] message;
        begin
            $display("FAIL line=%0d cycle=%0d %0s state=%0d reason=%0d loaded=%0d rx=%02x", 
                     line, cycle, message, state, stop_reason, config_loaded, rx_status);
            $finish;
        end
    endtask

    task step;
        input integer clocks;
        integer i;
        begin
            for(i=0;i<clocks;i=i+1) begin
                sample_valid=force_sample || (samples_on && phase==39) || bad_tick;
                est_valid=force_est || (est_on && phase==39);
                core_valid=force_core || (core_on && phase==3 && state==3);
                if(force_sample) begin sample_tick_us=tick_value; tick_clock=tick_value; end
                else if(bad_tick) sample_tick_us=0;
                else if(sample_valid) begin
                    tick_clock=tick_clock+64'd2000;
                    sample_tick_us=tick_clock;
                end
                if(abort_now) begin cmd_valid=1; cmd_payload=1504'h0304; end
                // Fast decision paths must equal the priority-encoded reason.
                #0;
                if(dut.any_trip !== (dut.trip!=0)) fail("any_trip differs from trip!=0");
                if(dut.fault_trip !== (dut.trip>=8'd16)) fail("fault_trip differs from trip>=16");
                if(dut.travel_bad !== ((dut.travel_delta[32] ? -dut.travel_delta : dut.travel_delta)
                                       > {17'd0,dut.travel_limit}))
                    fail("bounded travel compare differs from |delta|>limit");
                if(config_loaded && dut.speed_bad !== (
                        dut.abs32(dut.ev_now)>{dut.speed_limit,16'd0} ||
                        dut.abs32(dut.av_now)>{dut.adc_speed_limit,16'd0} ||
                        ((state==3) && (dut.abs32(dut.cv_now)>{dut.speed_limit,16'd0} ||
                                        dut.abs32(dut.ca_now)>{dut.adc_speed_limit,16'd0}))))
                    fail("bounded speed compare differs from |v|>limit");
                // Reference |alpha| from the public zero and ADC input, as in the spec.
                alpha_ref=$signed({7'd0,adc_raw,16'd0})-$signed({upright_zero_q16[31],upright_zero_q16});
                alpha_abs_ref=alpha_ref[32] ? -alpha_ref : alpha_ref;
                if(config_loaded && dut.enter_ok !== (alpha_abs_ref<={1'b0,dut.enter_code,16'd0}))
                    fail("bounded entry compare differs from |alpha|<=enter");
                if(config_loaded && dut.angle_bad !== ((state==3) && alpha_abs_ref>{1'b0,dut.exit_code,16'd0}))
                    fail("bounded exit compare differs from |alpha|>exit");
                @(posedge clk); #1;
                cycle=cycle+1;
                if(output_allow !== (state==3)) fail("allow must equal BALANCE after every condition clock");
                if(config_loaded !== (config_id!=0)) fail("loaded iff id nonzero");
                if(!config_loaded && config_crc!==0) fail("unloaded CRC must be zero");
                if((state==1 || state==2 || state==3) && !config_loaded) fail("active state without config");
                if(state==255 && config_loaded) fail("FAULT must consume config");
                if(rx_status[3:0]>7) fail("rx code outside protocol domain");
                if(rst_n && takeover_pulse !== (previous_state==2 && state==3)) fail("takeover must pulse exactly once");
                if(rst_n && public_parameters!==previous_parameters && !(config_loaded && !previous_loaded))
                    fail("public parameters changed outside atomic commit");
                previous_state=state;
                previous_loaded=config_loaded; previous_parameters=public_parameters;
                @(negedge clk);
                phase=(phase+1)%40;
                force_sample=0; force_est=0; force_core=0;
            end
            sample_valid=0; est_valid=0; core_valid=0;
        end
    endtask

    initial begin
        if(!$value$plusargs("VECTORS=%s",vector_path)) fail("missing vector file");
        fd=$fopen(vector_path,"r");
        if(!fd) fail("cannot open vector file");
        @(negedge clk);
        step(2); rst_n=1;
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%s",op); line=line+1;
            if(rc==1) case(op)
                "W": begin rc=$fscanf(fd,"%d",n); step(n); end
                "C": begin
                    rc=$fscanf(fd,"%h %h",cmd_payload,cmd_crc);
                    cmd_valid=1; step(1); cmd_valid=0;
                end
                "U": begin rc=$fscanf(fd,"%h %h",cmd_payload,cmd_crc); end
                "S": begin rc=$fscanf(fd,"%d",tick_value); force_sample=1; step(1); end
                "N": begin tick_value=tick_clock+64'd2000; force_sample=1; step(1); end
                "J": begin
                    rc=$fscanf(fd,"%s",name);
                    force_est=(name=="est" || name=="all");
                    force_core=(name=="core" || name=="all"); step(1);
                    if(abort_now) begin cmd_valid=0; abort_now=0; end
                end
                "I": begin
                    rc=$fscanf(fd,"%s %d",name,value);
                    case(name)
                        "raw": arm_on_raw=value[0];
                        "stable": arm_on_stable=value[0];
                        "released": keys_released=value[0];
                        "start": start_event=value[0];
                        "stop": stop_key=value[0];
                        "adc": adc_raw=value[9:0];
                        "age": adc_age_us=value[15:0];
                        "faults": sensor_fault_bits=value[15:0];
                        "quality": est_quality=value[2:0];
                        "position": encoder_position=value[31:0];
                        "ev": est_encoder_velocity_q16=value[31:0];
                        "av": est_adc_velocity_q16=value[31:0];
                        "cv": core_theta_velocity_q16=value[31:0];
                        "ca": core_alpha_velocity_q16=value[31:0];
                        "numeric": core_fault=value[0];
                        "overrun": core_overrun=value[0];
                        "reject": pwm_reject=value[0];
                        "error": cmd_error=value[0];
                        "samples_on": samples_on=value[0];
                        "est_on": est_on=value[0];
                        "core_on": core_on=value[0];
                        "bad_tick": bad_tick=value[0];
                        "abort": abort_now=value[0];
                        "reset": begin rst_n=value[0]; #1; previous_state=state; end
                        default: fail("unknown input name");
                    endcase
                end
                "E": begin
                    rc=$fscanf(fd,"%d %d %d %d",e_state,e_reason,e_loaded,e_rx);
                    if(state!==e_state[7:0] || stop_reason!==e_reason[7:0] || config_loaded!==e_loaded[0] ||
                       (e_rx>=0 && rx_status[3:0]!==e_rx[3:0])) fail("state/reason/loaded/rx mismatch");
                end
                "P": begin
                    rc=$fscanf(fd,"%d %d %d %d %d %d %d",e_id,e_crc,e_max,e_comp,e_blend,e_gain,e_zero);
                    if(config_id!==e_id[15:0] || config_crc!==e_crc[15:0] || max_compare!==e_max[15:0] ||
                       comp_compare!==e_comp[15:0] || blend_compare!==e_blend[15:0] ||
                       comp_gain_q20!==e_gain || upright_zero_q16!==e_zero) fail("atomic parameter mismatch");
                end
                "K": begin
                    rc=$fscanf(fd,"%h",expected_coefficients);
                    if(coefficients!==expected_coefficients) fail("full coefficient payload mismatch");
                end
                "T": begin
                    rc=$fscanf(fd,"%d %d",a,b);
                    if(theta_ref!==a || safety_position_delta!==b) fail("reference/delta mismatch");
                end
                "F": begin
                    rc=$fscanf(fd,"%d %d %d",a,b,c);
                    if(est_fresh!==a[0] || est_ok!==b[0] || core_fresh!==c[0]) fail("freshness mismatch");
                end
                "A": begin
                    rc=$fscanf(fd,"%d",a);
                    if(output_allow!==a[0]) fail("registered output_allow mismatch");
                end
                "H": begin
                    rc=$fscanf(fd,"%d",a);
                    if(rx_status[7:4]!==a[3:0]) fail("CRC-correct frame counter mismatch");
                end
                "Q": begin $display("PASS tb_balance_guard cycles=%0d checks=%0d",cycle,line); $finish; end
                default: fail("unknown vector opcode");
            endcase
        end
        fail("vectors ended without Q");
    end
    initial begin #10000000; fail("watchdog"); end
endmodule
