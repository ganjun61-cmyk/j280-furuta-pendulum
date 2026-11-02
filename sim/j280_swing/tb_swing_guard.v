`timescale 1ns/1ps

// v5/v6 guard bench: the v4 guard bench (same opcodes and per-clock checks) plus
// swing-core inputs; v6 adds the trajectory travel delta, built-in frames (B),
// trajectory field / standalone checks (V). Clock scaling 20 clocks/ms, 500 Hz snapshots 40 clocks
// apart; swing results arrive at phase 5 while in SWING (swing_on).
module tb_swing_guard;
    parameter integer MIN_HOLD_MS = 2;
    parameter integer VERSION = 5;
    parameter integer EXIT_MAX = 90;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, cmd_valid=0, cmd_error=0;
    reg [1503:0] cmd_payload=0;
    reg [15:0] cmd_crc=0;
    wire cmd_control_ok=cmd_payload[7:0]==VERSION && cmd_payload[1503:16]==1488'd0;
    wire cmd_is_config=cmd_payload[15:8]==8'd1, cmd_is_swing=cmd_payload[15:8]==8'd4;
    wire cmd_is_gains2=cmd_payload[15:8]==8'd5;
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
    reg [15:0] table_crc=16'h8f37;
    reg swing_valid=0, swing_blind_mid=0, swing_catch=0, swing_overrun=0;
    reg [15:0] swing_blind=0;
    reg signed [31:0] swing_theta_speed_q16=0;
    reg signed [15:0] swing_command=0;
    reg signed [31:0] track_delta=0;
    reg [1503:0] builtin_config=0, builtin_swing=0;
    reg [15:0] builtin_crc=0;
    reg cmd_builtin=0, inject_on=1, injecting=0;
    reg [1:0] inj_served=0;
    integer inj_wait=0;
    wire [159:0] traj_cfg;
    wire [1:0] builtin_req;
    wire standalone;
    reg [159:0] expected_traj;
    reg [32:0] track_abs;
    wire [7:0] state, stop_reason, rx_status;
    wire config_loaded, output_allow, takeover_pulse, est_fresh, est_ok, core_fresh;
    wire [15:0] config_id, config_crc, max_compare, comp_compare, blend_compare;
    wire [1023:0] coefficients;
    wire signed [31:0] comp_gain_q20, upright_zero_q16, theta_ref, safety_position_delta;
    wire [31:0] state_elapsed_ms;
    wire swing_loaded, swing_start, catch_pulse, is_balance, is_swing;
    wire [3:0] lash_half;
    wire [7:0] gap_boost, gap_threshold;
    wire [383:0] coefficients2;
    wire gains2_loaded, gains2_active;
    reg [383:0] expected_coefficients2;
    wire [575:0] swing_cfg;
    j280_swing_guard #(.CLOCKS_PER_MS(20), .STALE_CLOCKS(100),
        .ARM_TIMEOUT_MS(300), .MIN_HOLD_MS(MIN_HOLD_MS), .VERSION(VERSION), .EXIT_MAX(EXIT_MAX),
        .BUILTIN_TIMEOUT(2000)) dut (
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_payload(cmd_payload),.cmd_crc(cmd_crc),
        .cmd_control_ok(cmd_control_ok),.cmd_is_config(cmd_is_config),.cmd_is_swing(cmd_is_swing),
        .cmd_is_gains2(cmd_is_gains2),.cmd_builtin(cmd_builtin),.cmd_error(cmd_error),.start_event(start_event),.stop_key(stop_key),
        .arm_on_raw(arm_on_raw),.arm_on_stable(arm_on_stable),.keys_released(keys_released),
        .sample_valid(sample_valid),.sample_tick_us(sample_tick_us),.adc_raw(adc_raw),
        .adc_age_us(adc_age_us),.sensor_fault_bits(sensor_fault_bits),.encoder_position(encoder_position),
        .est_valid(est_valid),.est_valid2(est_valid),.est_valid3(est_valid),
        .est_quality(est_quality),.est_encoder_velocity_q16(est_encoder_velocity_q16),
        .est_adc_velocity_q16(est_adc_velocity_q16),.core_valid(core_valid),.core_valid2(core_valid),
        .core_fault(core_fault),
        .core_overrun(core_overrun),.core_theta_velocity_q16(core_theta_velocity_q16),
        .core_alpha_velocity_q16(core_alpha_velocity_q16),.pwm_reject(pwm_reject),
        .table_crc(table_crc),.swing_valid(swing_valid),.swing_blind_mid(swing_blind_mid),
        .swing_catch(swing_catch),.swing_overrun(swing_overrun),.swing_blind(swing_blind),
        .swing_theta_speed_q16(swing_theta_speed_q16),.swing_command(swing_command),
        .track_delta(track_delta),.traj_cfg(traj_cfg),.standalone(standalone),.builtin_req(builtin_req),
        .state(state),.stop_reason(stop_reason),.rx_status(rx_status),.config_loaded(config_loaded),
        .config_id(config_id),.config_crc(config_crc),.coefficients(coefficients),.max_compare(max_compare),
        .comp_compare(comp_compare),.blend_compare(blend_compare),.comp_gain_q20(comp_gain_q20),
        .upright_zero_q16(upright_zero_q16),.theta_ref(theta_ref),.safety_position_delta(safety_position_delta),
        .output_allow(output_allow),.takeover_pulse(takeover_pulse),.est_fresh(est_fresh),
        .est_ok(est_ok),.core_fresh(core_fresh),.state_elapsed_ms(state_elapsed_ms),
        .swing_loaded(swing_loaded),.swing_cfg(swing_cfg),.swing_start(swing_start),.catch_pulse(catch_pulse),
        .is_balance(is_balance),.is_swing(is_swing),.lash_half(lash_half),.gap_boost(gap_boost),
        .gap_threshold(gap_threshold),.coefficients2(coefficients2),.gains2_loaded(gains2_loaded),
        .gains2_active(gains2_active));

    integer fd, rc, line=0, cycle=0, phase=0, n, e_state, e_reason, e_loaded, e_rx;
    integer e_id, e_crc, e_max, e_comp, e_blend, e_gain, e_zero, a, b, c;
    reg [2047:0] vector_path;
    reg [255:0] op, name;
    reg signed [63:0] value;
    reg [63:0] tick_value, tick_clock=0;
    reg [1023:0] expected_coefficients;
    reg [575:0] expected_swing;
    reg samples_on=1, est_on=1, core_on=1, swing_on=1, bad_tick=0, abort_now=0;
    reg force_sample=0, force_est=0, force_core=0, force_swing=0;
    reg [7:0] previous_state=0;
    reg previous_loaded=0;
    wire [1135:0] public_parameters={coefficients,max_compare,comp_compare,blend_compare,
                                    comp_gain_q20,upright_zero_q16};
    reg [1135:0] previous_parameters=0;
    reg signed [32:0] alpha_ref;
    reg [32:0] alpha_abs_ref;
    reg signed [33:0] reference34, limit34;

    task fail;
        input [511:0] message;
        begin
            $display("FAIL line=%0d cycle=%0d %0s state=%0d reason=%0d loaded=%0d swing=%0d rx=%02x",
                     line, cycle, message, state, stop_reason, config_loaded, swing_loaded, rx_status);
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
                swing_valid=force_swing || (swing_on && phase==5 && state==4);
                if(force_sample) begin sample_tick_us=tick_value; tick_clock=tick_value; end
                else if(bad_tick) sample_tick_us=0;
                else if(sample_valid) begin
                    tick_clock=tick_clock+64'd2000;
                    sample_tick_us=tick_clock;
                end
                if(abort_now) begin cmd_valid=1; cmd_payload={1488'd0,8'd3,VERSION[7:0]}; end
                // Built-in injector model: a requested frame arrives 3 clocks later as a
                // CRC-correct command marked cmd_builtin (once per request value).
                if(builtin_req==2'd0) begin inj_served=0; inj_wait=0; end
                else if(inject_on && builtin_req!=inj_served && !cmd_valid) begin
                    if(inj_wait==3) begin
                        cmd_valid=1; cmd_builtin=1; injecting=1; inj_served=builtin_req; inj_wait=0;
                        cmd_payload=(builtin_req==2'd1) ? builtin_config : builtin_swing;
                        cmd_crc=(builtin_req==2'd1) ? builtin_crc : 16'h0;
                    end else inj_wait=inj_wait+1;
                end
                #0;
                if(dut.any_trip !== (dut.trip!=0)) fail("any_trip differs from trip!=0");
                if(dut.fault_trip !== (dut.trip>=8'd16)) fail("fault_trip differs from trip>=16");
                // Limit implied by the latched bounds (CONFIG or SWING travel).
                reference34=$signed({{2{dut.safety_reference[31]}},dut.safety_reference});
                limit34=dut.safety_high-reference34;
                if(config_loaded && dut.safety_low !== reference34-limit34)
                    fail("travel bounds not symmetric about the reference");
                track_abs=track_delta[31] ? -$signed({track_delta[31],track_delta}) : {1'b0,track_delta};
                if(config_loaded && dut.travel_bad !== ((VERSION==6 && state==3) ?
                        (track_abs>{17'd0,dut.track_limit}) :
                        ((dut.travel_delta[32] ? -dut.travel_delta : dut.travel_delta) > limit34)))
                    fail("bounded travel compare differs from |delta|>limit");
                if(standalone && !(config_loaded || dut.pending || dut.builtin_stage!=0))
                    fail("standalone outside a built-in load or session");
                if(builtin_req!=0 && !(state==0 && dut.builtin_stage!=0)) fail("built-in request outside a SAFE load");
                // The ahead-of-time deadline equals the direct compare on every good capture snapshot.
                if(state==2 && dut.qualifying && sample_valid && !dut.tick_bad &&
                   dut.deadline_reached !== (!dut.qualify_deadline[64] && sample_tick_us>=dut.qualify_deadline[63:0]))
                    fail("ahead-of-time capture deadline differs from the direct compare");
                if(standalone && config_loaded && !dut.ka_fresh) fail("standalone must not need keepalive");
                if(VERSION!=6 && (standalone || traj_cfg!==0)) fail("v6 outputs active below VERSION 6");
                if(config_loaded && dut.speed_bad !== (
                        dut.abs32(dut.ev_now)>{dut.speed_limit,16'd0} ||
                        dut.abs32(dut.av_now)>{dut.adc_speed_limit,16'd0} ||
                        ((state==3) && (dut.abs32(dut.cv_now)>{dut.speed_limit,16'd0} ||
                                        dut.abs32(dut.ca_now)>{dut.adc_speed_limit,16'd0})) ||
                        ((state==4) && dut.abs32(dut.sv_now)>{dut.speed_limit,16'd0})))
                    fail("bounded speed compare differs from |v|>limit");
                alpha_ref=$signed({7'd0,adc_raw,16'd0})-$signed({upright_zero_q16[31],upright_zero_q16});
                alpha_abs_ref=alpha_ref[32] ? -alpha_ref : alpha_ref;
                if(config_loaded && dut.enter_ok !== (alpha_abs_ref<={1'b0,dut.enter_code,16'd0}))
                    fail("bounded entry compare differs from |alpha|<=enter");
                if(config_loaded && dut.angle_bad !== ((state==3) && alpha_abs_ref>{1'b0,dut.exit_code,16'd0}))
                    fail("bounded exit compare differs from |alpha|>exit");
                @(posedge clk); #1;
                cycle=cycle+1;
                if(output_allow !== (state==3 || state==4)) fail("allow must equal BALANCE/SWING");
                if({dut.is_armed,dut.is_capture,dut.is_balance,dut.is_swing} !==
                   {state==1,state==2,state==3,state==4}) fail("one-hot state copy differs from state");
                if(dut.is_safe !== (state==0)) fail("is_safe differs from state==SAFE");
                if(config_loaded !== (config_id!=0)) fail("loaded iff id nonzero");
                if(!config_loaded && config_crc!==0) fail("unloaded CRC must be zero");
                if(swing_loaded && !config_loaded) fail("swing block without config");
                if((state==1 || state==2 || state==3 || state==4) && !config_loaded) fail("active state without config");
                if(state==4 && !swing_loaded) fail("SWING without swing block");
                if(gains2_loaded && !config_loaded) fail("gains2 block without config");
                if(gains2_active && !(state==3 && gains2_loaded)) fail("gains2 active outside BALANCE");
                // Exact switch time: active once BALANCE has lasted beyond switch_ms, never before it.
                if(state==3 && gains2_loaded && state_elapsed_ms>{16'd0,dut.g2_switch_ms} && !gains2_active)
                    fail("gains2 not active after switch_ms");
                if(gains2_active && state_elapsed_ms<{16'd0,dut.g2_switch_ms}) fail("gains2 active before switch_ms");
                if(state==255 && (config_loaded || swing_loaded)) fail("FAULT must consume config");
                if(state!=0 && (dut.pending || dut.swing_pending || dut.g2_pending))
                    fail("a command block is pending outside SAFE");
                if(rx_status[3:0]>7) fail("rx code outside protocol domain");
                if(rst_n && takeover_pulse !== (previous_state==2 && state==3)) fail("takeover must pulse exactly once");
                if(rst_n && swing_start !== (previous_state==1 && state==4)) fail("swing_start must pulse on SWING entry");
                if(rst_n && catch_pulse !== (previous_state==4 && state==3)) fail("catch must pulse on SWING->BALANCE");
                if(rst_n && public_parameters!==previous_parameters && !(config_loaded && !previous_loaded))
                    fail("public parameters changed outside atomic commit");
                previous_state=state;
                previous_loaded=config_loaded; previous_parameters=public_parameters;
                @(negedge clk);
                if(injecting) begin cmd_valid=0; cmd_builtin=0; injecting=0; end
                phase=(phase+1)%40;
                force_sample=0; force_est=0; force_core=0; force_swing=0;
            end
            sample_valid=0; est_valid=0; core_valid=0; swing_valid=0;
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
                    force_core=(name=="core" || name=="all");
                    force_swing=(name=="swing" || name=="all"); step(1);
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
                        "sv": swing_theta_speed_q16=value[31:0];
                        "command": swing_command=value[15:0];
                        "blind": swing_blind=value[15:0];
                        "mid": swing_blind_mid=value[0];
                        "catch": swing_catch=value[0];
                        "swing_overrun": swing_overrun=value[0];
                        "table": table_crc=value[15:0];
                        "numeric": core_fault=value[0];
                        "overrun": core_overrun=value[0];
                        "reject": pwm_reject=value[0];
                        "error": cmd_error=value[0];
                        "samples_on": samples_on=value[0];
                        "est_on": est_on=value[0];
                        "core_on": core_on=value[0];
                        "swing_on": swing_on=value[0];
                        "bad_tick": bad_tick=value[0];
                        "abort": abort_now=value[0];
                        "track": track_delta=value[31:0];
                        "inject": inject_on=value[0];
                        "reset": begin rst_n=value[0]; #1; previous_state=state; end
                        default: fail("unknown input name");
                    endcase
                end
                "E": begin
                    rc=$fscanf(fd,"%d %d %d %d",e_state,e_reason,e_loaded,e_rx);
                    if(state!==e_state[7:0] || stop_reason!==e_reason[7:0] || config_loaded!==e_loaded[0] ||
                       (e_rx>=0 && rx_status[3:0]!==e_rx[3:0])) fail("state/reason/loaded/rx mismatch");
                end
                "G": begin
                    rc=$fscanf(fd,"%d",a);
                    if(swing_loaded!==a[0]) fail("swing_loaded mismatch");
                end
                "L": begin
                    rc=$fscanf(fd,"%d %d %d",a,b,c);
                    if(lash_half!==a[3:0] || gap_boost!==b[7:0] || gap_threshold!==c[7:0])
                        fail("backlash field mismatch");
                end
                "X": begin
                    rc=$fscanf(fd,"%d %d",a,b);
                    if(gains2_loaded!==a[0] || gains2_active!==b[0]) fail("gains2 flag mismatch");
                end
                "Y": begin
                    rc=$fscanf(fd,"%h",expected_coefficients2);
                    if(coefficients2!==expected_coefficients2) fail("gains2 coefficient payload mismatch");
                end
                "Z": begin
                    rc=$fscanf(fd,"%h",expected_swing);
                    if(swing_cfg!==expected_swing) fail("swing block payload mismatch");
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
                "B": begin rc=$fscanf(fd,"%h %h %h",builtin_config,builtin_swing,builtin_crc); end
                "V": begin
                    rc=$fscanf(fd,"%h %d",expected_traj,a);
                    if(traj_cfg!==expected_traj || standalone!==a[0]) fail("trajectory fields/standalone mismatch");
                end
                "Q": begin $display("PASS tb_swing_guard cycles=%0d checks=%0d",cycle,line); $finish; end
                default: fail("unknown vector opcode");
            endcase
        end
        fail("vectors ended without Q");
    end
    initial begin #20000000; fail("watchdog"); end
endmodule
