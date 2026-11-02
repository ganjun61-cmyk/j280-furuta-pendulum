`timescale 1ns/1ps

// v5/v6 one-shot safety supervisor: the v4 balance guard plus automatic swing-up.
// v6 (VERSION=6): CONFIG bytes 168..187 carry the arm trajectory fields instead
// of the v5 backlash fields; run_ms 0 means no time limit; with DIP 1 switched on
// and no CONFIG, the guard asks the top (builtin_req) to inject the built-in
// CONFIG, then SWING, frame into the command parser; they arrive as ordinary
// CRC-checked commands marked cmd_builtin and pass the same checks (standalone:
// no host keepalive); in BALANCE the arm travel is bounded about the moving
// reference (track_delta) instead of the CONFIG-time position.
// cmd_valid means a CRC-correct frame (all frames refresh KA). No host command
// starts motion. With a SWING block loaded, SW1 in ARMED starts SWING only when
// the pendulum hangs inside the configured window; SWING hands over to BALANCE
// on a swing-core catch. In SWING the ADC-invalid bit and estimator quality are
// replaced by the swing core's bounded blind count; every other v4 protection
// stays. All stop decisions and output_allow are registered.
module j280_swing_guard #(
    parameter integer CLOCKS_PER_MS = 50000,
    parameter integer STALE_CLOCKS = 250000,
    parameter integer ARM_TIMEOUT_MS = 60000,
    parameter integer MIN_HOLD_MS = 200,
    // Wire version and CONFIG exit_code ceiling (tests also run the v4 suite with 4/57).
    parameter [7:0] VERSION = 8'd5,
    parameter [15:0] EXIT_MAX = 16'd90,
    // Highest accepted CONFIG algorithm_id. The lean build raises it to 15 for
    // the pendulum-side DOB; the full build keeps 14 unchanged.
    parameter [15:0] MAX_ALGORITHM = 16'd14,
    // Arm stall: |swing command| >= STALL_COMPARE with no encoder change over the
    // core's 5-sample window, for more than STALL_SAMPLES consecutive results.
    // 150 ms: the swinging pendulum's reaction can pin the arm for ~100 ms at
    // full drive (swing-651), which is not an encoder failure.
    parameter [15:0] STALL_COMPARE = 16'd500,
    parameter [6:0] STALL_SAMPLES = 7'd75,
    // SW1 starts SWING only within this many counts of the CONFIG-time arm position.
    parameter [15:0] START_WINDOW = 16'd32,
    // A requested built-in frame that has not arrived within this many clocks
    // abandons the standalone load (DIP 1 off/on retries).
    parameter integer BUILTIN_TIMEOUT = 5000000
) (
    input wire clk, rst_n,
    input wire cmd_valid,
    input wire [1503:0] cmd_payload,
    input wire [15:0] cmd_crc,
    // Must equal cmd_payload[7:0]==VERSION && cmd_payload[1503:16]==0 for the
    // same frame; the top computes it ahead of its command register (timing).
    input wire cmd_control_ok,
    // Must equal cmd_payload[15:8]==1/4/5 for the same frame (registered in the
    // top ahead of the command register, timing).
    input wire cmd_is_config, cmd_is_swing, cmd_is_gains2,
    // The frame came from the built-in injector (the host UART is cut off while it runs).
    input wire cmd_builtin,
    input wire cmd_error,
    input wire start_event, stop_key, arm_on_raw, arm_on_stable, keys_released,
    input wire sample_valid,
    input wire [63:0] sample_tick_us,
    input wire [9:0] adc_raw,
    input wire [15:0] adc_age_us, sensor_fault_bits,
    input wire signed [31:0] encoder_position,
    input wire est_valid,
    // Registered copies of the same strobes (top: identical registers), so the
    // wide copies and the age counters do not share one high-fanout net.
    input wire est_valid2, est_valid3,
    input wire [2:0] est_quality,
    input wire signed [31:0] est_encoder_velocity_q16, est_adc_velocity_q16,
    input wire core_valid, core_valid2, core_fault, core_overrun,
    input wire signed [31:0] core_theta_velocity_q16, core_alpha_velocity_q16,
    input wire pwm_reject,
    input wire [15:0] table_crc,
    input wire swing_valid, swing_blind_mid, swing_catch, swing_overrun,
    input wire [15:0] swing_blind,
    input wire signed [31:0] swing_theta_speed_q16,
    input wire signed [15:0] swing_command,
    // v6: encoder position minus the trajectory reference (whole counts).
    input wire signed [31:0] track_delta,
    output reg [7:0] state, stop_reason, rx_status,
    output reg config_loaded,
    // Masked by config_loaded, so a stop clears one bit instead of 32 (timing).
    output wire [15:0] config_id, config_crc,
    output reg [1023:0] coefficients,
    output reg [15:0] max_compare, comp_compare, blend_compare,
    output reg signed [31:0] comp_gain_q20, upright_zero_q16, theta_ref,
    output wire signed [31:0] safety_position_delta,
    output reg output_allow, takeover_pulse,
    output wire est_fresh, est_ok, core_fresh,
    output reg [31:0] state_elapsed_ms,
    output reg swing_loaded,
    output reg [575:0] swing_cfg,
    output reg swing_start, catch_pulse,
    // Registered copies of state==BALANCE / state==SWING for the top's wide loads.
    output reg is_balance, is_swing,
    // v5 backlash compensation (CONFIG payload bytes 168..173), latched at commit.
    output reg [3:0] lash_half,
    output reg [7:0] gap_boost, gap_threshold,
    // v5 GAINS2: a second observer + feedback gain set (coefficients 20..31; the
    // model part 0..19 stays from CONFIG). The top feeds it to the balance core
    // once gains2_active (BALANCE for switch_ms). Cleared by any stop.
    output reg [383:0] coefficients2,
    output reg gains2_loaded, gains2_active,
    // v6 trajectory fields (CONFIG bytes 168..187, ten 16-bit words), latched at commit.
    output reg [159:0] traj_cfg,
    // v6 controller mode from CONFIG algorithm_id: 1 bypass, 2..7 filter
    // presets, 8..11 arm-error dead zones (4/6/8/10 counts).
    output reg [3:0] filter_mode,
    // mode 16: the top blends 75% motor angle with 25% backlash-tracked load angle.
    output reg blend_mode,
    // Algorithm 19: also mix the 2 Hz low-passed arm rate into the arm-rate term.
    output reg arm_rate_mix_sel,
    output reg standalone,
    // 1: inject the built-in CONFIG frame, 2: the built-in SWING frame, 0: none.
    output reg [1:0] builtin_req
);
    localparam [7:0] SAFE=0, ARMED=1, CAPTURE=2, BALANCE=3, SWING=4, FAULT=255;
    localparam [7:0] R_COMPLETE=1, R_STOP=2, R_OFF=3, R_ABORT=4,
        R_ARM_TIMEOUT=5, R_CAPTURE_TIMEOUT=6, R_SWING_TIMEOUT=7, R_SENSOR=16, R_STALE=17,
        R_TRAVEL=18, R_SPEED=19, R_KA=20, R_ANGLE=21, R_NUMERIC=22,
        R_OVERRUN=23, R_PWM=24, R_BLIND=25, R_BLIND_ENTRY=26, R_STALL=27;
    localparam [31:0] MS_LAST=CLOCKS_PER_MS-1, STALE=STALE_CLOCKS;
    // v5 frames carry the backlash fields and accept GAINS2; VERSION=4 keeps
    // the v4 contract (all CONFIG bytes 168..187 zero, command 5 unknown).
    localparam V6=(VERSION==8'd6);
    localparam EXT=(VERSION==8'd5) || V6;     // GAINS2 accepted
    localparam LASH=(VERSION==8'd5);          // backlash fields

    // Keep the complete pending frame immutable until all checks pass. CONFIG
    // and SWING share the buffer: SWING is accepted only with no CONFIG pending.
    reg pending, pending_bad, swing_pending, swing_stage;
    reg [5:0] g2_index;
    reg [1503:0] pending_payload;
    reg [15:0] pending_crc;
    reg [5:0] check_index;
    wire [7:0] command=cmd_payload[15:8];
    wire new_config=cmd_valid && cmd_is_config;
    wire control_ok=cmd_control_ok;
    wire host_abort=cmd_valid && command==8'd3 && control_ok;

    wire [15:0] p_id=pending_payload[16+:16], p_algorithm=pending_payload[32+:16];
    wire [15:0] p_run=pending_payload[48+:16], p_ka=pending_payload[64+:16];
    wire [15:0] p_travel=pending_payload[80+:16], p_speed=pending_payload[96+:16];
    wire [15:0] p_adc_speed=pending_payload[112+:16], p_max=pending_payload[128+:16];
    wire [15:0] p_hold=pending_payload[144+:16], p_wait=pending_payload[160+:16];
    wire signed [31:0] p_zero=pending_payload[176+:32];
    wire [15:0] p_enter=pending_payload[208+:16], p_exit=pending_payload[224+:16];
    wire [15:0] p_comp=pending_payload[240+:16], p_blend=pending_payload[256+:16];
    wire signed [31:0] p_gain=pending_payload[272+:32];
    // Range checks in parallel on the immutable pending frame; the serial index
    // only picks one result bit (timing: no 32-to-1 word mux on 1024 bits).
    wire [31:0] coef_bad;
    wire [11:0] gain2_bad;
    genvar gi;
    generate
        for(gi=0;gi<32;gi=gi+1) begin : config_range
            wire [31:0] c=pending_payload[304+gi*32+:32];
            assign coef_bad[gi]=(c[31] ? (32'd0-c) : c)>32'd134217728;
        end
        for(gi=0;gi<12;gi=gi+1) begin : gains2_range
            wire [31:0] c=pending_payload[48+gi*32+:32];
            assign gain2_bad[gi]=(c[31] ? (32'd0-c) : c)>32'd134217728;
        end
    endgenerate
    wire [15:0] c_lash=pending_payload[1344+:16], c_boost=pending_payload[1360+:16];
    wire [15:0] c_threshold=pending_payload[1376+:16];
    // v6 trajectory fields (scripts/j280_traj.py FIELDS order).
    wire [15:0] t_step=pending_payload[1344+:16], t_move=pending_payload[1360+:16];
    wire [15:0] t_rot=pending_payload[1376+:16], t_accel=pending_payload[1392+:16];
    wire [15:0] t_lean_arm=pending_payload[1456+:16], t_lean_pend=pending_payload[1472+:16];
    wire [15:0] t_iwin=pending_payload[1488+:16];
    wire traj_ok=t_step<=16'd1040 && t_accel<=16'd4095 &&
        (t_step==16'd0 || (t_accel>=16'd1 && t_accel<=t_move && (t_rot==16'd0 || t_rot>=t_accel))) &&
        t_lean_arm<=16'd16383 && t_lean_pend<=16'd16383 && t_iwin<=16'd1040;
    wire signed [63:0] gain64={{32{p_gain[31]}},p_gain};
    wire signed [63:0] blend64={48'd0,p_blend}, comp64={48'd0,p_comp};
    wire signed [63:0] slope_error=gain64*blend64-(comp64<<<20);
    wire [63:0] slope_abs=slope_error[63] ? (64'd0-slope_error) : slope_error;
    wire [63:0] slope_tolerance=({48'd0,p_blend}+64'd1)>>1;
    // The scalar check is split in two halves, each registered a clock apart
    // during the coefficient scan (the payload is immutable then); the commit
    // clock ANDs the two bits. Timing: no 20-term compare feeding one register.
    // v6 reuses algorithm_id as the controller mode (1 bypass, 2..11 presets).
    wire algorithm_ok=V6 ? (p_algorithm>=16'd1 && p_algorithm<=MAX_ALGORITHM) : p_algorithm==16'd1;
    wire scalar_lo=pending_payload[7:0]==VERSION && p_id!=0 && algorithm_ok &&
        (V6 || p_run>=1) && p_run<=30000 && p_ka>=1 && p_ka<=500 &&
        p_travel>=1 && p_travel<=300 && p_speed>=1 && p_speed<=10000 &&
        p_adc_speed>=1 && p_adc_speed<=30000 && p_max>=1 && p_max<=1125 &&
        p_hold>=MIN_HOLD_MS && p_hold<=1000 &&
        ({1'b0,p_hold}+17'd2)<={1'b0,p_wait} && p_wait<=10000;
    wire scalar_hi=p_zero>=32'sd0 && p_zero<=32'sd67043328 &&
        p_enter<=9 && p_exit>=1 && p_exit<=EXIT_MAX && p_exit>p_enter &&
        p_comp<=250 && p_comp<=p_max && p_blend>=25 && p_blend<=250 &&
        p_gain>=32'sd0 && p_gain<=32'sd10485760 && slope_abs<=slope_tolerance &&
        pending_payload[1328+:16]==16'd2000 &&
        (V6 ? traj_ok :
         LASH ? (c_lash<=16'd15 && c_boost<=16'd250 && c_threshold<=16'd250 &&
                 pending_payload[1503:1392]==112'd0)
              : pending_payload[1503:1344]==160'd0);

    // SWING block (scripts/j280_swing_math.py FIELDS), checked on the same buffer.
    wire [15:0] s_id=pending_payload[16+:16], s_ms=pending_payload[32+:16];
    wire [15:0] s_travel=pending_payload[48+:16], s_blind=pending_payload[64+:16];
    wire [15:0] s_step=pending_payload[80+:16], s_edge_low=pending_payload[96+:16];
    wire [15:0] s_edge_high=pending_payload[112+:16], s_hang=pending_payload[128+:16];
    wire [15:0] s_hang_window=pending_payload[144+:16], s_kick_n=pending_payload[160+:16];
    wire signed [15:0] s_kick=pending_payload[176+:16];
    wire [15:0] s_lead=pending_payload[192+:16], s_crc=pending_payload[208+:16];
    wire signed [31:0] s_catch_a=pending_payload[224+:32], s_catch_w=pending_payload[256+:32];
    wire signed [31:0] s_g=pending_payload[288+:32], s_gl=pending_payload[320+:32];
    wire signed [31:0] s_kv=pending_payload[352+:32], s_kdrive=pending_payload[384+:32];
    wire signed [31:0] s_margin=pending_payload[416+:32], s_kscale=pending_payload[448+:32];
    wire signed [31:0] s_amp=pending_payload[480+:32], s_kp=pending_payload[512+:32];
    wire signed [31:0] s_kd=pending_payload[544+:32];
    wire [16:0] s_hang_low={1'b0,s_hang}-{1'b0,s_hang_window};
    wire [16:0] s_hang_high={1'b0,s_hang}+{1'b0,s_hang_window};
    function in_range;
        input signed [31:0] value, low, high;
        begin in_range=value>=low && value<=high; end
    endfunction
    wire swing_ok=pending_payload[7:0]==VERSION && pending_payload[15:8]==8'd4 &&
        s_id==config_id && s_ms>=500 && s_ms<=10000 && s_travel>=1 && s_travel<=260 &&
        s_blind>=1 && s_blind<=100 && s_step>=1 && s_step<=64 && s_edge_low<=200 &&
        s_edge_high>=800 && s_edge_high<=1023 && s_hang<=1023 && s_hang_window<=60 &&
        s_kick_n<=250 && s_kick>=-16'sd1000 && s_kick<=16'sd1000 && s_lead<=100 &&
        s_crc==table_crc &&
        in_range(s_catch_a,32'sd1,32'sd357913941) && in_range(s_catch_w,32'sd1,32'sd268435456) &&
        in_range(s_g,32'sd0,32'sd1048576) && in_range(s_gl,32'sd0,32'sd134217728) &&
        in_range(s_kv,32'sd0,32'sd67108864) && in_range(s_kdrive,32'sd0,32'sd1073741824) &&
        in_range(s_margin,-32'sd1048576,32'sd1048576) && in_range(s_kscale,32'sd0,32'sd1048576) &&
        in_range(s_amp,32'sd0,32'sd33554432) && in_range(s_kp,32'sd0,32'sd16777216) &&
        in_range(s_kd,32'sd0,32'sd134217728) &&
        s_edge_low<s_edge_high && s_hang>s_hang_window &&
        {1'b0,s_edge_low}<s_hang_low && s_hang_high<{1'b0,s_edge_high} &&
        pending_payload[1503:576]==928'd0;

    // GAINS2 (command 5): config_id, switch_ms, 12 coefficients, rest zero.
    wire [15:0] g_id=pending_payload[16+:16], g_switch=pending_payload[32+:16];
    wire gains2_ok=EXT && pending_payload[7:0]==VERSION && pending_payload[15:8]==8'd5 &&
        g_id==config_id && g_switch>=16'd1 && g_switch<=16'd30000 &&
        pending_payload[1503:432]==1072'd0;

    // pending_payload is immutable during checks, so registered results at
    // commit describe the same frame.
    reg scalar_lo_q, scalar_hi_q, swing_ok_q, gains2_ok_q;
    reg g2_pending, g2_bad;
    reg [15:0] g2_switch_ms;
    reg [15:0] run_ms, keepalive_ms, travel_limit, speed_limit, adc_speed_limit;
    reg [15:0] hold_ms, wait_ms, enter_code, exit_code;
    reg [15:0] swing_ms, blind_limit;
    reg [9:0] hang_low, hang_high;
    reg signed [31:0] safety_reference;
    reg signed [33:0] safety_high, safety_low;
    reg [15:0] track_limit;
    reg [1:0] builtin_stage;
    reg builtin_tried, builtin_cfg_req;
    reg [31:0] builtin_clocks;
    // tick_expect >= qualify_deadline, registered: the next good snapshot carries
    // exactly tick_expect (any other tick is a tick fault, which takes priority).
    reg deadline_reached;
    // Angle windows as Q16 bounds around the upright zero, latched at commit.
    // Held as whole ADC codes (timing): adc*65536 <= H  <=>  adc <= floor(H/65536)
    // and adc*65536 >= L <=> adc >= ceil(L/65536), so 12-bit compares give the
    // same result as the Q16 window on every clock. |bounds| < 1200 codes.
    reg signed [11:0] enter_hi_code, enter_lo_code, exit_hi_code, exit_lo_code;
    // Signed Q16 speed bounds, latched at commit: |v|>L == v>L || v<-L (L<2^31).
    reg signed [32:0] enc_speed_high, enc_speed_low, adc_speed_high, adc_speed_low;
    reg arm_seen_off;
    reg [31:0] est_age, core_age, swing_age, ka_div, state_div;
    reg [6:0] stall_count;
    reg start_centered;
    reg [15:0] ka_ms;
    reg [2:0] last_quality;
    reg signed [31:0] last_ev, last_av, last_cv, last_ca, last_sv;
    // Next expected snapshot time and capture deadline, each with its 64-bit
    // carry, latched when the reference time is taken.
    reg [64:0] tick_expect, qualify_deadline;
    reg tick_seen, qualifying;
    reg [3:0] frame_count;

    function [31:0] abs32;
        input [31:0] value;
        begin abs32=value[31] ? (32'd0-value) : value; end
    endfunction

    // One-hot copies of the state, written with state on every transition, so
    // the trip logic does not share the top's high-fanout state decodes (timing
    // only: each flag equals its state==X compare on every clock).
    reg is_armed, is_capture, is_safe;
    reg [15:0] config_id_q, config_crc_q;
    assign config_id=config_loaded ? config_id_q : 16'd0;
    assign config_crc=config_loaded ? config_crc_q : 16'd0;
    wire active=is_armed || is_capture || is_balance || is_swing;
    wire balancing=is_balance;
    wire swinging=is_swing;
    wire [2:0] quality_now=est_valid2 ? est_quality : last_quality;
    wire [31:0] ev_now=est_valid2 ? est_encoder_velocity_q16 : last_ev;
    wire [31:0] av_now=est_valid2 ? est_adc_velocity_q16 : last_av;
    wire [31:0] cv_now=core_valid ? core_theta_velocity_q16 : last_cv;
    wire [31:0] ca_now=core_valid ? core_alpha_velocity_q16 : last_ca;
    wire [31:0] sv_now=swing_valid ? swing_theta_speed_q16 : last_sv;
    assign est_fresh=est_valid || est_age<STALE;
    assign est_ok=est_fresh && quality_now==3'd7;
    assign core_fresh=balancing && (core_valid || core_age<STALE);
    wire swing_fresh=swing_valid || swing_age<STALE;
    wire ka_fresh=config_loaded && (standalone || cmd_valid || ka_ms<keepalive_ms);

    // Both operands are sign extended BEFORE subtraction.
    wire signed [32:0] travel_delta=$signed({encoder_position[31],encoder_position})-
                                  $signed({safety_reference[31],safety_reference});
    assign safety_position_delta=(travel_delta>33'sd2147483647) ? 32'sh7fffffff :
        (travel_delta< -33'sd2147483648) ? 32'sh80000000 : travel_delta[31:0];
    wire signed [33:0] position34={{2{encoder_position[31]}},encoder_position};
    wire track_bad=track_delta>$signed({16'd0,track_limit}) || track_delta< -$signed({16'd0,track_limit});
    wire travel_bad=(V6 && balancing) ? track_bad : (position34>safety_high || position34<safety_low);
    function over;
        input [31:0] value;
        input signed [32:0] high, low;
        begin over=$signed({value[31],value})>high || $signed({value[31],value})<low; end
    endfunction
    wire speed_bad_comb=over(ev_now,enc_speed_high,enc_speed_low) ||
        over(av_now,adc_speed_high,adc_speed_low) ||
        (balancing && (over(cv_now,enc_speed_high,enc_speed_low) ||
                       over(ca_now,adc_speed_high,adc_speed_low))) ||
        (swinging && over(sv_now,enc_speed_high,enc_speed_low));
    // One register stage: the speed comparators otherwise feed the stop/state
    // fan-out and set the slowest clk_50m path. A 20 ns later stop is harmless.
    reg speed_bad_q;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) speed_bad_q<=0;
        else speed_bad_q<=speed_bad_comb;
    wire speed_bad=V6 ? speed_bad_q : speed_bad_comb;
    wire signed [11:0] adc12=$signed({2'd0,adc_raw});
    wire enter_ok=adc12<=enter_hi_code && adc12>=enter_lo_code;
    wire exit_ok=adc12<=exit_hi_code && adc12>=exit_lo_code;
    wire signed [33:0] p_zero34={{2{p_zero[31]}},p_zero};
    wire signed [33:0] enter_h34=p_zero34+$signed({2'd0,p_enter,16'd0});
    wire signed [33:0] enter_l34=p_zero34-$signed({2'd0,p_enter,16'd0})+34'sd65535;
    wire signed [33:0] exit_h34=p_zero34+$signed({2'd0,p_exit,16'd0});
    wire signed [33:0] exit_l34=p_zero34-$signed({2'd0,p_exit,16'd0})+34'sd65535;
    wire angle_bad=balancing && !exit_ok;
    wire hang_ok=adc_raw>=hang_low && adc_raw<=hang_high;
    // SWING tolerates the ADC-invalid bit and estimator quality loss; the
    // swing core's consecutive not-good count and entry position bound them.
    wire sensor_bad=adc_age_us>16'd100 || sensor_fault_bits[15:1]!=0 ||
        (!swinging && (sensor_fault_bits[0] || quality_now!=3'd7));
    wire blind_entry_bad=swinging && swing_valid && swing_blind_mid;
    wire blind_bad=swinging && swing_valid && swing_blind>blind_limit;
    wire tick_bad=sample_valid && tick_seen &&
        (tick_expect[64] || sample_tick_us!=tick_expect[63:0]);
    wire stale_bad=!est_fresh || tick_bad || (balancing && !core_fresh) || (swinging && !swing_fresh);
    wire overrun_bad=core_overrun || (swinging && swing_overrun);
    wire [15:0] swing_command_abs=swing_command[15] ? (16'd0-swing_command) : swing_command;
    wire stall_sample=swing_command_abs>=STALL_COMPARE && swing_theta_speed_q16==32'sd0;
    wire stall_bad=swinging && stall_count>STALL_SAMPLES;
    wire [63:0] hold_us={48'd0,hold_ms}*64'd1000;
    wire capture_ready=sample_valid && enter_ok && qualifying && deadline_reached;
    wire catch_ready=swinging && swing_valid && swing_catch && quality_now==3'd7 &&
        !sensor_fault_bits[0] && exit_ok;

    // Ordered chain: local stop, raw DIP, sensor, blind entry, blind length,
    // stale, travel, speed, arm stall, angle, KA, numeric, overrun, PWM, ABORT,
    // then normal timeouts/completion.
    wire [7:0] trip=stop_key ? R_STOP : !arm_on_raw ? R_OFF :
        sensor_bad ? R_SENSOR : blind_entry_bad ? R_BLIND_ENTRY : blind_bad ? R_BLIND :
        stale_bad ? R_STALE : travel_bad ? R_TRAVEL :
        speed_bad ? R_SPEED : stall_bad ? R_STALL : angle_bad ? R_ANGLE : !ka_fresh ? R_KA :
        (core_fault && !core_overrun) ? R_NUMERIC : overrun_bad ? R_OVERRUN :
        pwm_reject ? R_PWM : host_abort ? R_ABORT :
        (is_armed && state_elapsed_ms>=ARM_TIMEOUT_MS) ? R_ARM_TIMEOUT :
        (is_capture && state_elapsed_ms>={16'd0,wait_ms}) ? R_CAPTURE_TIMEOUT :
        (swinging && state_elapsed_ms>={16'd0,swing_ms}) ? R_SWING_TIMEOUT :
        (balancing && run_ms!=16'd0 && state_elapsed_ms>={16'd0,run_ms}) ? R_COMPLETE : 8'd0;
    wire fault_class=sensor_bad || blind_entry_bad || blind_bad || stale_bad || travel_bad ||
        speed_bad || stall_bad || angle_bad || !ka_fresh || core_fault || overrun_bad || pwm_reject;
    wire any_trip=stop_key || !arm_on_raw || fault_class || host_abort ||
        (is_armed && state_elapsed_ms>=ARM_TIMEOUT_MS) ||
        (is_capture && state_elapsed_ms>={16'd0,wait_ms}) ||
        (swinging && state_elapsed_ms>={16'd0,swing_ms}) ||
        (balancing && run_ms!=16'd0 && state_elapsed_ms>={16'd0,run_ms});
    wire fault_trip=!stop_key && arm_on_raw && fault_class;
    // SWING and GAINS2 share the acceptance rule; one enable loads the buffer.
    wire swing_accept=is_safe && config_loaded && !pending && !swing_pending && !g2_pending;
    wire load_pending=cmd_valid && ((cmd_is_config && is_safe) || (cmd_is_swing && swing_accept) ||
                                    (cmd_is_gains2 && EXT && swing_accept));
    // Standalone: DIP 1 on in SAFE with nothing loaded loads the built-in CONFIG,
    // then (once it commits) the built-in SWING; a failed attempt waits for DIP off.
    // The start request is registered (timing); a host frame on that clock wins
    // and the request is simply re-evaluated.
    wire builtin_cfg_want=V6 && is_safe && !config_loaded && !pending && !swing_pending && !g2_pending &&
        builtin_stage==2'd0 && !builtin_tried && arm_seen_off && arm_on_raw &&
        arm_on_stable && keys_released && !stop_key;
    wire builtin_cfg_go=builtin_cfg_req && builtin_cfg_want && !cmd_valid;
    wire arm_ready=config_loaded && builtin_stage==2'd0 && !pending && !swing_pending && !g2_pending && arm_seen_off && arm_on_raw &&
        arm_on_stable && keys_released && !stop_key && !sensor_bad && est_ok && ka_fresh;

    task clear_config;
        begin
            config_loaded<=0; pending<=0;
            swing_loaded<=0; swing_pending<=0;
            gains2_loaded<=0; g2_pending<=0;
            filter_mode<=4'd0; blend_mode<=0; arm_rate_mix_sel<=0;
            standalone<=0; builtin_stage<=0; builtin_req<=0;
        end
    endtask
    // A trip happens only outside SAFE, where no CONFIG/SWING/GAINS2 is pending
    // (all are accepted only in SAFE and ARMED needs none pending), so a trip only
    // has to drop the loaded flags (timing: fewer endpoints on the trip net).
    task clear_session;
        begin
            config_loaded<=0; swing_loaded<=0; gains2_loaded<=0; standalone<=0;
        end
    endtask
    task enter_state;
        input [7:0] next_state;
        begin
            state<=next_state; state_div<=0; state_elapsed_ms<=0;
            is_armed<=next_state==ARMED; is_capture<=next_state==CAPTURE;
            is_balance<=next_state==BALANCE; is_swing<=next_state==SWING;
            is_safe<=next_state==SAFE;
            gains2_active<=0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=SAFE; stop_reason<=0; rx_status<=0; config_loaded<=0;
            is_armed<=0; is_capture<=0; is_balance<=0; is_swing<=0;
            config_id_q<=0; config_crc_q<=0; coefficients<=0; max_compare<=0;
            comp_compare<=0; blend_compare<=0; comp_gain_q20<=0;
            upright_zero_q16<=0; theta_ref<=0; output_allow<=0;
            takeover_pulse<=0; state_elapsed_ms<=0;
            pending<=0; pending_bad<=0; pending_payload<=0; pending_crc<=0; check_index<=0;
            swing_pending<=0; swing_stage<=0; swing_loaded<=0; swing_cfg<=0;
            swing_start<=0; catch_pulse<=0; swing_ms<=0; blind_limit<=0; hang_low<=0; hang_high<=0;
            run_ms<=0; keepalive_ms<=0; travel_limit<=0; speed_limit<=0; adc_speed_limit<=0;
            hold_ms<=0; wait_ms<=0; enter_code<=0; exit_code<=0;
            safety_reference<=0; arm_seen_off<=0; est_age<=STALE; core_age<=STALE; swing_age<=STALE;
            last_quality<=0; last_ev<=0; last_av<=0; last_cv<=0; last_ca<=0; last_sv<=0;
            stall_count<=0; start_centered<=0;
            ka_div<=0; ka_ms<=16'hffff; state_div<=0; frame_count<=0;
            tick_expect<=65'd2000; qualify_deadline<=0; tick_seen<=0; qualifying<=0;
            scalar_lo_q<=0; scalar_hi_q<=0; swing_ok_q<=0; safety_high<=0; safety_low<=0;
            enter_hi_code<=0; enter_lo_code<=0; exit_hi_code<=0; exit_lo_code<=0; is_safe<=1;
            enc_speed_high<=0; enc_speed_low<=0; adc_speed_high<=0; adc_speed_low<=0;
            lash_half<=0; gap_boost<=0; gap_threshold<=0;
            coefficients2<=0; gains2_loaded<=0; gains2_active<=0;
            filter_mode<=4'd0; blend_mode<=0; arm_rate_mix_sel<=0;
            gains2_ok_q<=0; g2_pending<=0; g2_bad<=0; g2_index<=0; g2_switch_ms<=0;
            traj_cfg<=0; standalone<=0; track_limit<=0; builtin_stage<=0; builtin_tried<=0;
            builtin_cfg_req<=0; deadline_reached<=0; builtin_req<=0; builtin_clocks<=0;
        end else begin
            takeover_pulse<=0; swing_start<=0; catch_pulse<=0;
            builtin_cfg_req<=builtin_cfg_want && !builtin_cfg_go;
            if(builtin_stage==2'd0) builtin_clocks<=0;
            else if(builtin_clocks!=32'hffffffff) builtin_clocks<=builtin_clocks+32'd1;
            deadline_reached<=!qualify_deadline[64] && !tick_expect[64] &&
                              tick_expect[63:0]>=qualify_deadline[63:0];
            if(check_index==6'd30) scalar_lo_q<=scalar_lo;
            if(check_index==6'd31) scalar_hi_q<=scalar_hi;
            swing_ok_q<=swing_ok;
            gains2_ok_q<=gains2_ok;
            if(est_valid) begin
                est_age<=0; last_quality<=est_quality;
            end else if(est_age<STALE) est_age<=est_age+32'd1;
            if(est_valid3) begin
                last_ev<=est_encoder_velocity_q16; last_av<=est_adc_velocity_q16;
            end
            if(balancing && core_valid) core_age<=0;
            else if(core_age<STALE) core_age<=core_age+32'd1;
            // Zero outside BALANCE keeps the stale-value guard without putting
            // the long catch/takeover priority chain on these clock enables.
            if(!balancing) begin
                last_cv<=0; last_ca<=0;
            end else if(core_valid2) begin
                last_cv<=core_theta_velocity_q16; last_ca<=core_alpha_velocity_q16;
            end
            if(swinging && swing_valid) begin
                swing_age<=0; last_sv<=swing_theta_speed_q16;
            end else if(swing_age<STALE) swing_age<=swing_age+32'd1;
            if(!swinging) stall_count<=0;
            else if(swing_valid)
                stall_count<=!stall_sample ? 7'd0 : (stall_count==7'd127 ? stall_count : stall_count+7'd1);
            // Registered: a still arm is within the window on consecutive clocks.
            start_centered<=travel_delta>=-$signed({17'd0,START_WINDOW}) &&
                            travel_delta<=$signed({17'd0,START_WINDOW});
            if(cmd_valid) begin ka_div<=0; ka_ms<=0; end
            else if(ka_ms!=16'hffff) begin
                if(ka_div==MS_LAST) begin ka_div<=0; ka_ms<=ka_ms+16'd1; end
                else ka_div<=ka_div+32'd1;
            end
            if(state_div==MS_LAST) begin
                state_div<=0;
                if(state_elapsed_ms!=32'hffffffff) state_elapsed_ms<=state_elapsed_ms+32'd1;
            end else state_div<=state_div+32'd1;
            if(!arm_on_stable) begin arm_seen_off<=1; builtin_tried<=0; end
            if(active && sample_valid) begin
                tick_expect<={1'b0,sample_tick_us}+65'd2000; tick_seen<=1;
            end
            // The deadline is read only while qualifying, which is set on this
            // same clock; loading it also on a trip clock is unobservable (timing).
            if(is_capture && sample_valid && enter_ok && !qualifying)
                qualify_deadline<={1'b0,sample_tick_us}+{1'b0,hold_us};

            // CONFIG: 32 serial coefficient checks, then one scalar/commit clock.
            // New CONFIG and valid ABORT have precedence even on commit clock.
            if(pending && !new_config && !host_abort) begin
                if(check_index<6'd32) begin
                    if(coef_bad[check_index[4:0]]) pending_bad<=1;
                    check_index<=check_index+6'd1;
                end else begin
                    pending<=0;
                    if(!pending_bad && scalar_lo_q && scalar_hi_q) begin
                        config_loaded<=1; config_id_q<=p_id; config_crc_q<=pending_crc;
                        coefficients<=pending_payload[304+:1024];
                        max_compare<=p_max; comp_compare<=p_comp; blend_compare<=p_blend;
                        comp_gain_q20<=p_gain; upright_zero_q16<=p_zero;
                        run_ms<=p_run; keepalive_ms<=p_ka; travel_limit<=p_travel;
                        speed_limit<=p_speed; adc_speed_limit<=p_adc_speed;
                        hold_ms<=p_hold; wait_ms<=p_wait; enter_code<=p_enter; exit_code<=p_exit;
                        lash_half<=LASH ? c_lash[3:0] : 4'd0;
                        gap_boost<=LASH ? c_boost[7:0] : 8'd0;
                        gap_threshold<=LASH ? c_threshold[7:0] : 8'd0;
                        traj_cfg<=V6 ? pending_payload[1344+:160] : 160'd0;
                        filter_mode<=V6 ? ((p_algorithm==16'd1) ? 4'd0 :
                                           (p_algorithm==16'd16) ? 4'd14 :
                                           (p_algorithm==16'd18) ||
                                           (p_algorithm==16'd19) ? 4'd15 : p_algorithm[3:0]) : 4'd0;
                        blend_mode<=V6 && (p_algorithm==16'd16);
                        arm_rate_mix_sel<=V6 && (p_algorithm==16'd19);
                        track_limit<=p_travel;
                        safety_reference<=encoder_position;
                        // Arithmetic >>>16: floor for the upper bound; +65535 first makes it ceil.
                        enter_hi_code<=enter_h34[27:16]; enter_lo_code<=enter_l34[27:16];
                        exit_hi_code<=exit_h34[27:16]; exit_lo_code<=exit_l34[27:16];
                        enc_speed_high<=$signed({1'b0,p_speed,16'd0});
                        enc_speed_low<=-$signed({1'b0,p_speed,16'd0});
                        adc_speed_high<=$signed({1'b0,p_adc_speed,16'd0});
                        adc_speed_low<=-$signed({1'b0,p_adc_speed,16'd0});
                        safety_high<=position34+$signed({18'd0,p_travel});
                        safety_low<=position34-$signed({18'd0,p_travel});
                        rx_status[3:0]<=4'd1;
                        if(builtin_stage==2'd1) begin builtin_stage<=2'd2; builtin_req<=2'd2; builtin_clocks<=0; end
                    end else begin
                        rx_status[3:0]<=4'd5;
                        if(standalone) begin standalone<=0; builtin_stage<=0; builtin_req<=0; end
                    end
                end
            end
            // SWING: one clock for the registered check, then commit. The arm
            // travel bounds are re-latched about the CONFIG-time reference.
            if(swing_pending && !new_config && !host_abort) begin
                if(!swing_stage) swing_stage<=1;
                else begin
                    swing_pending<=0;
                    if(swing_ok_q) begin
                        swing_loaded<=1; swing_cfg<=pending_payload[575:0];
                        swing_ms<=s_ms; blind_limit<=s_blind;
                        hang_low<=s_hang_low[9:0]; hang_high<=s_hang_high[9:0];
                        safety_high<=$signed({{2{safety_reference[31]}},safety_reference})+$signed({18'd0,s_travel});
                        safety_low<=$signed({{2{safety_reference[31]}},safety_reference})-$signed({18'd0,s_travel});
                        track_limit<=s_travel;
                        rx_status[3:0]<=4'd1;
                    end else begin
                        rx_status[3:0]<=4'd5;
                        if(standalone) begin standalone<=0; config_loaded<=0; end
                    end
                    if(builtin_stage==2'd2) builtin_stage<=0;
                end
            end
            // GAINS2: 12 serial coefficient checks, then the registered scalar check.
            if(g2_pending && !new_config && !host_abort) begin
                if(g2_index<6'd12) begin
                    if(gain2_bad[g2_index[3:0]]) g2_bad<=1;
                    g2_index<=g2_index+6'd1;
                end else begin
                    g2_pending<=0;
                    if(!g2_bad && gains2_ok_q) begin
                        gains2_loaded<=1; coefficients2<=pending_payload[48+:384];
                        g2_switch_ms<=g_switch; rx_status[3:0]<=4'd1;
                    end else rx_status[3:0]<=4'd5;
                end
            end
            if(load_pending) pending_payload<=cmd_payload;
            if(builtin_cfg_go) begin
                standalone<=1; builtin_stage<=2'd1; builtin_req<=2'd1; builtin_tried<=1;
            end
            if(cmd_valid) begin
                frame_count<=frame_count+4'd1; rx_status[7:4]<=frame_count+4'd1;
                case(command)
                    8'd1: begin
                        if(!is_safe) rx_status[3:0]<=4'd4;
                        else begin
                            clear_config;
                            pending<=1; pending_crc<=cmd_crc;
                            check_index<=0; pending_bad<=0; rx_status[3:0]<=0;
                        end
                    end
                    8'd2: rx_status[3:0]<=control_ok ? 4'd2 : 4'd5;
                    8'd3: begin
                        rx_status[3:0]<=control_ok ? 4'd3 : 4'd5;
                        if(host_abort && is_safe) clear_config;
                    end
                    8'd4: begin
                        if(!swing_accept) rx_status[3:0]<=4'd4;
                        else begin
                            swing_loaded<=0; swing_pending<=1; swing_stage<=0;
                            // Back to the CONFIG travel until this block commits.
                            safety_high<=$signed({{2{safety_reference[31]}},safety_reference})+$signed({18'd0,travel_limit});
                            safety_low<=$signed({{2{safety_reference[31]}},safety_reference})-$signed({18'd0,travel_limit});
                            track_limit<=travel_limit;
                            rx_status[3:0]<=0;
                        end
                    end
                    8'd5: begin
                        if(!EXT) rx_status[3:0]<=4'd6;
                        else if(!swing_accept) rx_status[3:0]<=4'd4;
                        else begin
                            gains2_loaded<=0; g2_pending<=1; g2_index<=0; g2_bad<=0;
                            rx_status[3:0]<=0;
                        end
                    end
                    default: rx_status[3:0]<=4'd6;
                endcase
            end
            if(cmd_error) rx_status[3:0]<=4'd7;
            // The injected frames take the ordinary command path above; keep the
            // standalone load going (a CONFIG clears it) and stop requesting.
            if(cmd_valid && cmd_builtin) begin
                if(cmd_is_config && builtin_stage==2'd1) begin
                    standalone<=1; builtin_stage<=2'd1; builtin_req<=0; builtin_clocks<=0;
                end
                if(cmd_is_swing && builtin_stage==2'd2) begin builtin_req<=0; builtin_clocks<=0; end
            end
            if(builtin_stage!=2'd0 && builtin_clocks>=BUILTIN_TIMEOUT && is_safe) clear_config;

            case(state)
                SAFE: begin
                    output_allow<=0; qualifying<=0; tick_seen<=0;
                    if(arm_ready && !cmd_valid) begin
                        enter_state(ARMED); arm_seen_off<=0;
                        tick_seen<=sample_valid;
                        if(sample_valid) tick_expect<={1'b0,sample_tick_us}+65'd2000;
                    end
                end
                ARMED, CAPTURE, BALANCE, SWING: begin
                    if(any_trip) begin
                        stop_reason<=trip; output_allow<=0; clear_session;
                        qualifying<=0; tick_seen<=0;
                        // Lost host keepalive is a latched protection fault;
                        // only ordinary arm/capture/swing/run deadlines return SAFE.
                        if(fault_trip) enter_state(FAULT);
                        else enter_state(SAFE);
                    end else if(is_armed && start_event) begin
                        if(!swing_loaded) begin
                            enter_state(CAPTURE); qualifying<=0;
                        end else if(hang_ok && start_centered && est_ok && !sensor_fault_bits[0]) begin
                            enter_state(SWING); output_allow<=1; swing_start<=1;
                            theta_ref<=encoder_position; swing_age<=0; last_sv<=0;
                        end
                    end else if(is_capture && sample_valid) begin
                        if(!enter_ok) qualifying<=0;
                        else if(!qualifying) begin
                            qualifying<=1;
                        end
                        else if(capture_ready) begin
                            enter_state(BALANCE); output_allow<=1; takeover_pulse<=1;
                            theta_ref<=encoder_position; core_age<=0;
                        end
                    end else if(catch_ready) begin
                        // theta_ref stays at the swing start; the core takes the preset.
                        enter_state(BALANCE); catch_pulse<=1;
                        core_age<=0;
                    end else if(is_balance && gains2_loaded && !gains2_active &&
                                state_elapsed_ms>={16'd0,g2_switch_ms}) begin
                        gains2_active<=1;
                    end
                end
                default: begin
                    output_allow<=0; state<=FAULT; clear_config;
                    is_armed<=0; is_capture<=0; is_balance<=0; is_swing<=0; gains2_active<=0; is_safe<=0;
                end
            endcase
        end
    end
endmodule
