`timescale 1ns/1ps
// v6: automatic swing-up, balance, and arm position control while balancing.
// Local start only, active motor on XH1/B. Sampling/control 500 Hz,
// self-describing v6 telemetry 100 Hz (v5 layout; the v4 constant words carry
// the trajectory). SW3/SW4 step the arm reference by one grid step, DIP 2
// turns the arm at constant speed; DIP 1 on with no CONFIG injects the built-in
// command frames into the command parser (standalone, no host needed).
module j280_swing_phase_top #(
    parameter integer DEBOUNCE_CLOCKS=1000000,
    parameter integer ARM_ACTIVE_LOW=1,
    parameter integer MIN_HOLD_MS=200,
    parameter integer SETTLE_MS=3000
)(
    input wire clk_50m,rst_n,
    input wire [9:0] adc_data,
    input wire adc_otr,enc1_a,enc1_b,uart_rx,key_sw1,key_sw2,dip_sw1,
    input wire key_sw3,key_sw4,dip_sw2,
    input wire [3:0] key_row,
    output wire [3:0] key_col,
    output wire adc_clk,adc_oe_n,uart_tx,pwm_a,ain1,ain2,pwm_b,bin1,bin2,
    output wire [3:0] led
);
    localparam [7:0] BALANCE=8'd3,SWING=8'd4,FAULT=8'd255;
    assign {pwm_a,ain1,ain2}=3'b000;
    (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) reset_pipe<=0;
        else reset_pipe<={reset_pipe[0],1'b1};
    wire reset_ok=reset_pipe[1];

    wire [9:0] raw;
    wire adc_valid,adc_strobe;
    wire [15:0] adc_age;
    j280_adc_capture adc(.clk(clk_50m),.rst_n(reset_ok),.adc_data(adc_data),
        .adc_otr(adc_otr),.adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.raw(raw),
        .valid(adc_valid),.sample_strobe(adc_strobe),.age_us(adc_age));
    wire signed [31:0] position;
    wire [31:0] illegal_count;
    wire encoder_ready;
    // 2 us per-line filter: PWM edges in the shared XH1 cable caused illegal
    // A/B transitions during 40% balance drive (balance-451/461/462).
    j280_encoder_filtered #(.FILTER_CLOCKS(100)) encoder(.clk(clk_50m),.rst_n(reset_ok),.enc_a(enc1_a),.enc_b(enc1_b),
        .position(position),.illegal_count(illegal_count),.ready(encoder_ready));

    wire [7:0] rx_byte;
    wire rx_valid,rx_error,cmd_valid,cmd_error;
    wire [1503:0] cmd_payload;
    wire [15:0] cmd_crc;
    j280_uart_rx rx(.clk(clk_50m),.rst_n(reset_ok),.rx(uart_rx),.data(rx_byte),
        .valid(rx_valid),.frame_error(rx_error));
    // Built-in frame injector (standalone). On builtin_req 1/2, once the host
    // UART has been quiet for 1.2 ms (longer than the parser's inter-byte
    // timeout, so it is hunting for a frame), the ROM frame is fed to the
    // parser one byte every 4 clocks; host bytes are dropped meanwhile and the
    // resulting command is marked cmd_builtin.
    wire [1:0] builtin_req;
    reg [1:0] inj_served;
    reg inj_active,inj_valid;
    reg [8:0] inj_addr;
    reg [7:0] inj_byte,inj_count;
    reg [1:0] inj_phase;
    reg [3:0] inj_tail;
    reg [16:0] uart_quiet;
    wire [7:0] rom_byte;
    j280_builtin builtin(.clk(clk_50m),.addr(inj_addr),.data(rom_byte));
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin
            inj_served<=0; inj_active<=0; inj_valid<=0; inj_addr<=0; inj_byte<=0; inj_count<=0;
            inj_phase<=0; inj_tail<=0; uart_quiet<=0;
        end else begin
            inj_valid<=0;
            if(rx_valid || rx_error) uart_quiet<=0;
            else if(uart_quiet!=17'h1ffff) uart_quiet<=uart_quiet+17'd1;
            if(builtin_req==2'd0) inj_served<=0;
            if(!inj_active) begin
                if(builtin_req!=2'd0 && builtin_req!=inj_served && uart_quiet>=17'd60000) begin
                    inj_active<=1; inj_served<=builtin_req;
                    inj_addr<={builtin_req[1],8'd0}; inj_count<=0; inj_phase<=0; inj_tail<=0;
                end
            end else if(inj_tail!=4'd0) begin
                if(inj_tail==4'd15) inj_active<=0;
                inj_tail<=inj_tail+4'd1;
            end else begin
                inj_phase<=inj_phase+2'd1;
                if(inj_phase==2'd2) begin inj_valid<=1; inj_byte<=rom_byte; end
                if(inj_phase==2'd3) begin
                    if(inj_count==8'd191) inj_tail<=4'd1;
                    else begin inj_addr<=inj_addr+9'd1; inj_count<=inj_count+8'd1; end
                end
            end
        end
    j280_balance_cmd_rx #(.VERSION(8'd6)) parser(.clk(clk_50m),.rst_n(reset_ok),
        .byte_valid(inj_active ? inj_valid : rx_valid),.byte_data(inj_active ? inj_byte : rx_byte),
        .byte_error(!inj_active && rx_error),.frame_valid(cmd_valid),
        .frame_payload(cmd_payload),.frame_crc(cmd_crc),.frame_error(cmd_error));
    // One register stage on the command bus. The control-frame check (version
    // and 1488 reserved bits zero) is computed before it, so the guard's
    // ABORT/CONFIG precedence on the commit clock is a single gate.
    reg g_cmd_valid,g_cmd_error,g_cmd_control_ok,g_cmd_is_config,g_cmd_is_swing,g_cmd_is_gains2,g_cmd_builtin;
    reg [1503:0] g_cmd_payload;
    reg [15:0] g_cmd_crc;
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin
            g_cmd_valid<=0; g_cmd_error<=0; g_cmd_control_ok<=0; g_cmd_payload<=0; g_cmd_crc<=0;
            g_cmd_is_config<=0; g_cmd_is_swing<=0; g_cmd_is_gains2<=0; g_cmd_builtin<=0;
        end else begin
            g_cmd_builtin<=inj_active;
            g_cmd_valid<=cmd_valid; g_cmd_error<=cmd_error; g_cmd_payload<=cmd_payload; g_cmd_crc<=cmd_crc;
            g_cmd_control_ok<=cmd_payload[7:0]==8'd6 && cmd_payload[1503:16]==1488'd0;
            g_cmd_is_config<=cmd_payload[15:8]==8'd1;
            g_cmd_is_swing<=cmd_payload[15:8]==8'd4;
            g_cmd_is_gains2<=cmd_payload[15:8]==8'd5;
        end
    wire start_event,stop_key,arm_raw,arm_stable,keys_released;
    wire [2:0] input_levels;
    j280_local_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS),.ARM_ACTIVE_LOW(ARM_ACTIVE_LOW)) local_inputs(
        .clk(clk_50m),.rst_n(reset_ok),.sw1_n(key_sw1),.sw2_n(key_sw2),.dip1(dip_sw1),
        .start_event(start_event),.stop_pressed(stop_key),.arm_on_raw(arm_raw),
        .arm_on_stable(arm_stable),.keys_released(keys_released),.levels(input_levels));
    wire sw3_event,sw4_event,dip2_on;
    wire [2:0] aux_levels;
    j280_aux_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS),.DIP_ACTIVE_LOW(ARM_ACTIVE_LOW)) aux_inputs(
        .clk(clk_50m),.rst_n(reset_ok),.sw3_n(key_sw3),.sw4_n(key_sw4),.dip2(dip_sw2),
        .sw3_event(sw3_event),.sw4_event(sw4_event),.dip2_on(dip2_on),.levels(aux_levels));

    reg [5:0] us_divider;
    reg [63:0] tick_us;
    wire [63:0] tick_next=(us_divider==6'd49)?tick_us+64'd1:tick_us;
    reg [16:0] sample_divider;
    reg [31:0] raw_sequence,telemetry_sequence;
    reg [2:0] telemetry_phase;
    reg adc_new,snapshot_valid,packet_valid;
    reg [223:0] snapshot;
    reg [63:0] snapshot_tick;
    reg [9:0] snapshot_adc;
    reg snapshot_adc_valid;
    reg signed [31:0] snapshot_position;
    reg [991:0] packet_payload;
    wire tx_busy,tx_overflow;
    wire [15:0] faults={13'd0,tx_overflow,(|illegal_count),!adc_valid};
    wire [7:0] raw_flags={4'd0,tx_overflow,1'b0,adc_new,adc_valid};
    wire est_valid;
    wire [2:0] est_quality;
    wire signed [31:0] est_enc_velocity,est_adc_velocity;
    j280_raw_estimator estimator(.clk(clk_50m),.rst_n(reset_ok),
        .sample_valid(snapshot_valid),.sample_payload(snapshot),.sample_tick_us(snapshot_tick),
        .sample_ready(),.result_valid(est_valid),.result_payload(),.result_tick_us(),.segment_id(),
        .encoder_position(),.encoder_velocity_q16(est_enc_velocity),.adc_velocity_q16(est_adc_velocity),
        .encoder_span_us(),.adc_span_us(),.adc_position_time_us(),.encoder_midpoint_x2_us(),
        .adc_midpoint_x2_us(),.encoder_age_x2_us(),.adc_age_x2_us(),.quality(est_quality),.reason());

    wire [7:0] state,stop_reason,rx_status;
    wire config_loaded,output_allow,takeover_pulse,est_fresh,est_ok,core_fresh;
    wire swing_loaded,swing_start,catch_pulse,is_balance,is_swing;
    wire [3:0] lash_half;
    wire [7:0] gap_boost,gap_threshold;
    wire [575:0] swing_cfg;
    wire [15:0] table_crc;
    wire [15:0] config_id,config_crc,max_compare,comp_compare,blend_compare;
    wire [1023:0] coefficients;
    wire signed [31:0] comp_gain_q20,upright_zero_q16,theta_ref,safety_position_delta;
    wire [31:0] state_elapsed_ms;
    wire [159:0] traj_cfg;
    wire standalone;
    wire core_valid,core_fault,core_overrun,core_busy,core_ready;
    wire signed [31:0] xhat0,xhat1,xhat2,xhat3,u_linear,u_compensated,u_model;
    wire signed [15:0] command;
    wire [15:0] core_latency;
    // The arm sample, one clock after the snapshot (the v5 tracker's slot).
    reg tracked_valid;
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) tracked_valid<=0;
        else tracked_valid<=snapshot_valid;
    wire signed [31:0] tracked_arm=snapshot_position;

    // Arm reference trajectory. It rests on the anchor (theta_ref) outside
    // BALANCE and on the takeover/catch clock, where the published outputs are
    // bypassed because they lag reset_now by one clock.
    wire traj_reset=!is_balance || takeover_pulse || catch_pulse;
    wire settled=is_balance && state_elapsed_ms>=SETTLE_MS;
    wire traj_done,traj_moving,traj_rotating;
    wire signed [47:0] traj_r,traj_theta_ref;
    wire signed [31:0] traj_alpha_ref,traj_u_ff,traj_v,traj_target;
    wire signed [15:0] traj_index;
    // One register stage on the CONFIG-time trajectory parameters: they change
    // only at a CONFIG commit in SAFE, where the trajectory is held at its anchor.
    reg [159:0] traj_cfg_q;
    reg [15:0] max_compare_q;
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin traj_cfg_q<=0; max_compare_q<=0; end
        else begin traj_cfg_q<=traj_cfg; max_compare_q<=max_compare; end
    assign key_col=4'b0000;  // keyboard scanner removed from the lean phase build
    j280_traj traj(.clk(clk_50m),.rst_n(reset_ok),.reset_now(traj_reset),.anchor(theta_ref),
        .start(tracked_valid && !traj_reset),.ccw_event(sw3_event),.cw_event(sw4_event),
        .rotate(dip2_on),.settled(settled),.arm(tracked_arm),
        .key_go(1'b0),.key_delta(16'sd0),.key_speed(16'd0),
        .step_counts(traj_cfg_q[10:0]),.move_speed(traj_cfg_q[31:16]),.rotate_speed(traj_cfg_q[47:32]),
        .accel(traj_cfg_q[59:48]),.kv(traj_cfg_q[79:64]),.ka(traj_cfg_q[95:80]),.ki(traj_cfg_q[111:96]),
        .lean_arm(traj_cfg_q[125:112]),.lean_pend(traj_cfg_q[141:128]),.i_window(traj_cfg_q[154:144]),
        .max_compare(max_compare_q),.done(traj_done),.r_out(traj_r),.theta_ref_out(traj_theta_ref),
        .alpha_ref_out(traj_alpha_ref),.u_ff_out(traj_u_ff),.v_out(traj_v),.target_out(traj_target),
        .index_out(traj_index),.moving_out(traj_moving),.rotating_out(traj_rotating));
    wire signed [47:0] anchor_q16={theta_ref,16'd0};
    wire signed [47:0] ref_r=traj_reset ? anchor_q16 : traj_r;
    wire signed [47:0] ref_theta=traj_reset ? anchor_q16 : traj_theta_ref;
    wire signed [31:0] ref_alpha=traj_reset ? 32'sd0 : traj_alpha_ref;

    wire signed [63:0] theta_measure_wide=$signed({tracked_arm,16'd0})-$signed({{16{ref_theta[47]}},ref_theta});
    wire measurement_range_fault=(theta_measure_wide>64'sd2147483647 ||
                                  theta_measure_wide< -64'sd2147483648);
    wire signed [31:0] theta_measure=theta_measure_wide[31:0];
    wire signed [63:0] raw_delta_wide=
        $signed({{32{snapshot_position[31]}},snapshot_position})-
        $signed({{32{theta_ref[31]}},theta_ref});
    wire signed [31:0] theta_counts=(raw_delta_wide>64'sd2147483647) ? 32'sh7fffffff :
        (raw_delta_wide< -64'sd2147483648) ? 32'sh80000000 : raw_delta_wide[31:0];
    // Exact mod-1040 arm phase (one-time remainder at takeover, then encoder
    // deltas), so the one-revolution torque table is aligned at multi-turn
    // positions. The alpha zero is untouched.
    wire [10:0] phase_phase;
    j280_phase_track phase_track(.clk(clk_50m),.rst_n(reset_ok),
        .position(snapshot_position),.sample(snapshot_valid),.phase(phase_phase));
    wire signed [23:0] phase_ff_q16;
    j280_phase_lut phase_lut(.clk(clk_50m),.rst_n(reset_ok),
        .phase(phase_phase),.ff_q16(phase_ff_q16));
    wire signed [31:0] alpha_measure=$signed({6'd0,snapshot_adc,16'd0})-upright_zero_q16-ref_alpha;
    // Phase-dependent torque feedforward (Q16 compare), clamped with the
    // trajectory feedforward so the core's |ff| limit cannot trip.
    // Engage the phase torque feedforward only after the balance has settled
    // (3 s), so the catch/takeover transient sees the unmodified controller.
    wire signed [23:0] phase_ff_eff = settled ? phase_ff_q16 : 24'sd0;
    wire signed [32:0] ff_sum={traj_u_ff[31],traj_u_ff}+{{9{phase_ff_eff[23]}},phase_ff_eff};
    wire signed [31:0] ref_u_ff=traj_reset ? 32'sd0 :
        (ff_sum>33'sd73728000) ? 32'sd73728000 :
        (ff_sum< -33'sd73728000) ? -32'sd73728000 : ff_sum[31:0];

    j280_swing_table_id table_id(.crc(table_crc));
    wire swing_busy,swing_valid,swing_overrun,swing_good,swing_mid,swing_catch;
    wire signed [15:0] swing_command;
    wire [15:0] swing_blind;
    wire signed [31:0] swing_a,swing_ad,swing_energy,swing_ref,swing_speed;
    wire signed [31:0] preset0,preset1,preset2,preset3;
    j280_swing_core swing(.clk(clk_50m),.rst_n(reset_ok),.start(swing_start),
        .sample_valid(snapshot_valid && is_swing),.code(snapshot_adc),.adc_valid(snapshot_adc_valid),
        .theta(theta_counts),.cfg(swing_cfg),.max_compare(max_compare),.zero_q16(upright_zero_q16),
        .busy(swing_busy),.result_valid(swing_valid),.overrun(swing_overrun),.command(swing_command),
        .good(swing_good),.blind_mid(swing_mid),.catch_ok(swing_catch),.blind(swing_blind),
        .est_a(swing_a),.est_ad(swing_ad),.energy(swing_energy),.ref_q16(swing_ref),
        .theta_speed(swing_speed),.preset_x0(preset0),.preset_x1(preset1),
        .preset_x2(preset2),.preset_x3(preset3));

    wire core_clear=!is_balance;
    wire core_start=(traj_done && is_balance) || takeover_pulse;
    // Algorithm 18/19: 20 Hz SVF on the pendulum-angle measurement. Its rate is
    // averaged with the core's Kalman alpha rate; algorithm 19 also averages a
    // 2 Hz low-passed Kalman arm rate for slow arm damping.
    // Lean phase build: alg19 is the only mode; constants let synthesis fold
    // the unused filter/deadband/GAINS2 paths.
    wire alpha_rate_mix=1'b1;
    wire arm_rate_mix=1'b1;
    wire signed [31:0] alpha_svf_rate;
    wire signed [31:0] alpha_svf_arm_lp;
    wire alpha_svf_busy;
    j280_alpha_svf alpha_svf(.clk(clk_50m),.rst_n(reset_ok),.clear(core_clear),
        .start(core_start),.alpha_q16(alpha_measure),.arm_rate_q16(xhat2),
        .rate_q16(alpha_svf_rate),.arm_rate_lp_q16(alpha_svf_arm_lp),
        .busy(alpha_svf_busy));
    // GAINS2 replaces the observer and feedback gains (coefficients 20..31); the
    // core latches its coefficients at each sample start.
    wire [1023:0] core_coefficients=coefficients;
    j280_balance_core core(.clk(clk_50m),.rst_n(reset_ok),.clear(core_clear),.sample_valid(core_start),
        .theta_q16(theta_measure),.alpha_q16(alpha_measure),.coefficients(core_coefficients),
        .max_compare(max_compare),.comp_compare(comp_compare),.blend_compare(blend_compare),
        .comp_gain_q20(comp_gain_q20),.feedforward_q16(ref_u_ff),.filter_mode(4'd15),
        .alpha_rate_ext_q16(alpha_svf_rate),.alpha_rate_mix(alpha_rate_mix),
        .arm_rate_ext_q16(alpha_svf_arm_lp),.arm_rate_mix(arm_rate_mix),
        .preset_valid(catch_pulse),.preset_x0(preset0),.preset_x1(preset1),
        .preset_x2(preset2),.preset_x3(preset3),.busy(core_busy),.sample_ready(core_ready),
        .result_valid(core_valid),.fault(core_fault),.overrun(core_overrun),
        .xhat0(xhat0),.xhat1(xhat1),.xhat2(xhat2),.xhat3(xhat3),
        .u_linear_q16(u_linear),.u_compensated_q16(u_compensated),.u_model_q16(u_model),
        .signed_command(command),.latency_cycles(core_latency));
    // SWING drives the bridge from the swing core; the last swing command is
    // held after a catch until the first balance result. One register stage
    // between the core and the bridge (the v5 gap-boost slot).
    wire swinging=is_swing;
    reg bal_valid;
    reg signed [15:0] bal_command;
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin bal_valid<=0; bal_command<=0; end
        else begin
            bal_valid<=core_valid && !core_fault && !measurement_range_fault;
            if(core_valid) bal_command<=command;
        end
    wire pin_pwm,pin_in1,pin_in2,pwm_active,pwm_reject;
    wire [15:0] applied_compare;
    wire [1:0] applied_direction;
    j280_balance_pwm #(.MAX_COMPARE(1125)) pwm(.clk(clk_50m),.rst_n(reset_ok),.allow(output_allow),
        .update(swinging ? (swing_valid && !swing_overrun) : bal_valid),
        .signed_command(swinging ? swing_command : bal_command),
        .pwm(pin_pwm),.in1(pin_in1),.in2(pin_in2),.active(pwm_active),
        .applied_compare(applied_compare),.applied_direction(applied_direction),.reject(pwm_reject));
    assign pwm_b=reset_ok?pin_pwm:1'b1;
    assign bin1=reset_ok && pin_in1;
    assign bin2=reset_ok && pin_in2;
    // One register stage on the far-away guard inputs. Each strobe moves with
    // its data; the guard responds 20 ns later.
    reg signed [31:0] g_position,g_est_ev,g_est_av,g_core_tv,g_core_av,g_swing_speed,g_track;
    reg g_est_valid,g_est_valid_b,g_est_valid_c,g_core_valid,g_core_valid_b;
    reg g_core_fault,g_core_overrun,g_pwm_reject;
    reg g_swing_valid,g_swing_mid,g_swing_catch,g_swing_overrun;
    reg [2:0] g_est_quality;
    reg [15:0] g_faults,g_adc_age,g_swing_blind;
    reg signed [15:0] g_swing_command;
    wire signed [32:0] track_wide=$signed({position[31],position})-$signed({ref_r[47],ref_r[47:16]});
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin
            g_position<=0; g_est_ev<=0; g_est_av<=0; g_core_tv<=0; g_core_av<=0;
            g_est_valid<=0; g_est_valid_b<=0; g_est_valid_c<=0;
            g_core_valid<=0; g_core_valid_b<=0; g_core_fault<=0; g_core_overrun<=0;
            g_pwm_reject<=0; g_est_quality<=0; g_faults<=0; g_adc_age<=16'hffff;
            g_swing_speed<=0; g_swing_valid<=0; g_swing_mid<=0; g_swing_catch<=0;
            g_swing_overrun<=0; g_swing_blind<=0; g_swing_command<=0; g_track<=0;
        end else begin
            g_position<=position; g_est_ev<=est_enc_velocity; g_est_av<=est_adc_velocity;
            g_core_tv<=xhat2; g_core_av<=xhat3;
            // Three/four identical registers per strobe: same value every clock,
            // separate nets for the wide copies (timing only).
            g_est_valid<=est_valid; g_est_valid_b<=est_valid; g_est_valid_c<=est_valid;
            g_core_valid<=core_valid; g_core_valid_b<=core_valid;
            g_core_fault<=core_fault || measurement_range_fault; g_core_overrun<=core_overrun;
            g_pwm_reject<=pwm_reject; g_est_quality<=est_quality; g_faults<=faults; g_adc_age<=adc_age;
            g_swing_speed<=swing_speed; g_swing_valid<=swing_valid; g_swing_mid<=swing_mid;
            g_swing_catch<=swing_catch; g_swing_overrun<=swing_overrun; g_swing_blind<=swing_blind;
            g_swing_command<=swing_command;
            g_track<=(track_wide>33'sd2147483647) ? 32'sh7fffffff :
                     (track_wide< -33'sd2147483648) ? 32'sh80000000 : track_wide[31:0];
        end
    j280_swing_guard #(.MIN_HOLD_MS(MIN_HOLD_MS),.VERSION(8'd6),.MAX_ALGORITHM(16'd19)) guard(.clk(clk_50m),.rst_n(reset_ok),
        .cmd_valid(g_cmd_valid),.cmd_payload(g_cmd_payload),.cmd_crc(g_cmd_crc),
        .cmd_control_ok(g_cmd_control_ok),.cmd_is_config(g_cmd_is_config),.cmd_is_swing(g_cmd_is_swing),
        .cmd_is_gains2(1'b0),.cmd_builtin(g_cmd_builtin),.cmd_error(g_cmd_error),
        .start_event(start_event),.stop_key(stop_key),.arm_on_raw(arm_raw),
        .arm_on_stable(arm_stable),.keys_released(keys_released),
        .sample_valid(snapshot_valid),.sample_tick_us(snapshot_tick),.adc_raw(snapshot_adc),
        .adc_age_us(g_adc_age),.sensor_fault_bits(g_faults),.encoder_position(g_position),
        .est_valid(g_est_valid),.est_valid2(g_est_valid_b),.est_valid3(g_est_valid_c),
        .est_quality(g_est_quality),.est_encoder_velocity_q16(g_est_ev),
        .est_adc_velocity_q16(g_est_av),.core_valid(g_core_valid),.core_valid2(g_core_valid_b),
        .core_fault(g_core_fault),.core_overrun(g_core_overrun),
        .core_theta_velocity_q16(g_core_tv),.core_alpha_velocity_q16(g_core_av),.pwm_reject(g_pwm_reject),
        .table_crc(table_crc),.swing_valid(g_swing_valid),.swing_blind_mid(g_swing_mid),
        .swing_catch(g_swing_catch),.swing_overrun(g_swing_overrun),.swing_blind(g_swing_blind),
        .swing_theta_speed_q16(g_swing_speed),.swing_command(g_swing_command),
        .track_delta(g_track),
        .state(state),.stop_reason(stop_reason),.rx_status(rx_status),.config_loaded(config_loaded),
        .config_id(config_id),.config_crc(config_crc),.coefficients(coefficients),
        .max_compare(max_compare),.comp_compare(comp_compare),.blend_compare(blend_compare),
        .comp_gain_q20(comp_gain_q20),.upright_zero_q16(upright_zero_q16),.theta_ref(theta_ref),
        .safety_position_delta(safety_position_delta),.output_allow(output_allow),
        .takeover_pulse(takeover_pulse),.est_fresh(est_fresh),.est_ok(est_ok),
        .core_fresh(core_fresh),.state_elapsed_ms(state_elapsed_ms),
        .swing_loaded(swing_loaded),.swing_cfg(swing_cfg),.swing_start(swing_start),
        .catch_pulse(catch_pulse),.is_balance(is_balance),.is_swing(is_swing),
        .lash_half(lash_half),.gap_boost(gap_boost),.gap_threshold(gap_threshold),
        .coefficients2(),.gains2_loaded(),.gains2_active(),
        .traj_cfg(traj_cfg),.filter_mode(),.arm_rate_mix_sel(),
        .standalone(standalone),.builtin_req(builtin_req));

    reg [2:0] quality_last;
    reg [127:0] xhat_last;
    reg signed [31:0] linear_last,compensated_last,model_last;
    reg [31:0] controller_age_us,latency_last;
    reg have_result;
    wire core_ok=have_result && core_fresh && !core_fault;
    wire [7:0] flags={(state==FAULT),core_ok,arm_stable,config_loaded,tx_overflow,
                     pwm_active,adc_new,adc_valid};
    wire [7:0] motor_mode={5'd0,!pwm_active,pin_in2,pin_in1};
    wire [7:0] swing_flags={1'b0,1'b0,1'b0,1'b0,
                            swing_mid,swing_catch,swing_good,swing_loaded};
    wire traj_live=!traj_reset;
    // Status bits 4..6 are the auxiliary inputs as the person at the board sees
    // them (SW3/SW4 pressed, DIP 2 on); bit 7 stays zero.
    wire [15:0] traj_status={traj_live ? traj_index[7:0] : 8'd0,1'b0,dip2_on,
                             !aux_levels[1],!aux_levels[0],settled,standalone,
                             traj_live && traj_rotating,traj_live && traj_moving};
    reg [991:0] telemetry_now;
    always @* begin
        telemetry_now=992'd0;
        telemetry_now[0+:8]=8'd6;
        telemetry_now[8+:8]=flags;
        telemetry_now[16+:32]=telemetry_sequence;
        telemetry_now[48+:64]=tick_next;
        telemetry_now[112+:32]=raw_sequence;
        telemetry_now[144+:16]={6'd0,raw};
        telemetry_now[160+:16]=adc_age;
        telemetry_now[176+:32]=position;
        telemetry_now[208+:32]=ref_r[47:16];
        telemetry_now[240+:32]=upright_zero_q16;
        telemetry_now[272+:16]=config_id;
        telemetry_now[288+:16]=config_crc;
        telemetry_now[304+:8]=state;
        telemetry_now[312+:8]=stop_reason;
        telemetry_now[320+:8]={5'd0,input_levels};
        telemetry_now[328+:8]=motor_mode;
        telemetry_now[336+:16]=applied_compare;
        telemetry_now[352+:16]=traj_status;
        telemetry_now[368+:32]=linear_last;
        telemetry_now[400+:32]=compensated_last;
        telemetry_now[432+:128]=xhat_last;
        telemetry_now[560+:32]=controller_age_us;
        telemetry_now[592+:16]=faults;
        telemetry_now[608+:8]={5'd0,quality_last};
        telemetry_now[616+:8]=rx_status;
        telemetry_now[624+:32]=latency_last;
        telemetry_now[656+:32]=traj_live ? traj_v : 32'sd0;
        telemetry_now[688+:32]=ref_u_ff;
        telemetry_now[720+:32]=state_elapsed_ms;
        telemetry_now[752+:32]=safety_position_delta;
        telemetry_now[784+:32]=model_last;
        telemetry_now[816+:32]=swing_a;
        telemetry_now[848+:32]=swing_ad;
        telemetry_now[880+:32]=swing_energy;
        telemetry_now[912+:32]=swing_ref;
        telemetry_now[944+:16]=swing_command;
        telemetry_now[960+:16]=swing_blind;
        telemetry_now[976+:8]=swing_flags;
    end
    always @(posedge clk_50m or negedge reset_ok) begin
        if(!reset_ok) begin
            us_divider<=0; tick_us<=0; sample_divider<=0; raw_sequence<=0;
            telemetry_sequence<=0; telemetry_phase<=0; adc_new<=0;
            snapshot_valid<=0; snapshot<=0; snapshot_tick<=0; snapshot_adc<=0; snapshot_adc_valid<=0;
            snapshot_position<=0; packet_valid<=0; packet_payload<=0;
            quality_last<=0; xhat_last<=0; linear_last<=0; compensated_last<=0; model_last<=0;
            controller_age_us<=32'hffffffff; latency_last<=0; have_result<=0;
        end else begin
            if(us_divider==6'd49) begin
                us_divider<=0;
                if(controller_age_us!=32'hffffffff) controller_age_us<=controller_age_us+32'd1;
            end else us_divider<=us_divider+6'd1;
            tick_us<=tick_next;
            snapshot_valid<=0; packet_valid<=0;
            if(est_valid) quality_last<=est_quality;
            if(takeover_pulse || catch_pulse) begin have_result<=0; controller_age_us<=32'hffffffff; end
            if(core_valid) begin
                latency_last<={16'd0,core_latency};
                if(!core_fault) begin
                    xhat_last<={xhat3,xhat2,xhat1,xhat0};
                    linear_last<=u_linear; compensated_last<=u_compensated; model_last<=u_model;
                    controller_age_us<=0; have_result<=1;
                end
            end
            if(sample_divider==17'd99999) begin
                sample_divider<=0;
                snapshot<={16'd0,faults,8'd2,8'd0,16'd0,16'd0,position,adc_age,
                           6'd0,raw,tick_next[31:0],raw_sequence,raw_flags,8'd2};
                snapshot_tick<=tick_next; snapshot_adc<=raw; snapshot_adc_valid<=adc_valid;
                snapshot_position<=position;
                snapshot_valid<=1; raw_sequence<=raw_sequence+32'd1; adc_new<=0;
                if(telemetry_phase==3'd0) begin
                    packet_payload<=telemetry_now; packet_valid<=1;
                    telemetry_sequence<=telemetry_sequence+32'd1;
                end
                if(telemetry_phase==3'd4) telemetry_phase<=0;
                else telemetry_phase<=telemetry_phase+3'd1;
            end else sample_divider<=sample_divider+17'd1;
            if(adc_strobe) adc_new<=1;
        end
    end
    j280_state_packet_tx #(.PAYLOAD_BYTES(124)) tx(.clk(clk_50m),.rst_n(reset_ok),
        .packet_valid(packet_valid),.payload(packet_payload),.uart_tx(uart_tx),
        .overflow(tx_overflow),.busy(tx_busy));
    // LEDs: outside BALANCE as v5 (drive, state blink, swing/standalone, FAULT).
    // In BALANCE one LED marks the target step (index mod 4); it blinks while
    // the reference moves and a lit LED runs around while turning.
    wire [3:0] step_led=4'd1<<traj_index[1:0];
    wire [3:0] turn_led=4'd1<<ref_r[23:22];
    wire [3:0] balance_led=traj_rotating ? turn_led : (traj_moving ? (tick_us[16] ? step_led : 4'd0) : step_led);
    wire [3:0] other_led={(state==FAULT),
                          (swing_loaded|standalone) ? tick_us[18] : tick_us[20],
                          (state==SWING)?tick_us[15]:(state==8'd2)?tick_us[16]:(state==8'd1)?tick_us[17]:
                          (config_loaded?tick_us[19]:1'b0),
                          pwm_active};
    assign led=(state==BALANCE) ? balance_led : other_led;
endmodule
