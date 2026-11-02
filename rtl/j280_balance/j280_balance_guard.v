`timescale 1ns/1ps

// One-shot safety supervisor. cmd_valid means a CRC-correct frame, including
// unknown/rejected commands (all of them refresh KA). No host command starts
// motion. All stop decisions and output_allow are registered at the condition
// clock; the PWM module must use allow as its immediate combinational Stop gate.
module j280_balance_guard #(
    parameter integer CLOCKS_PER_MS = 50000,
    parameter integer STALE_CLOCKS = 250000,
    parameter integer ARM_TIMEOUT_MS = 60000,
    parameter integer MIN_HOLD_MS = 200
) (
    input wire clk, rst_n,
    input wire cmd_valid,
    input wire [1503:0] cmd_payload,
    input wire [15:0] cmd_crc,
    input wire cmd_error,
    input wire start_event, stop_key, arm_on_raw, arm_on_stable, keys_released,
    input wire sample_valid,
    input wire [63:0] sample_tick_us,
    input wire [9:0] adc_raw,
    input wire [15:0] adc_age_us, sensor_fault_bits,
    input wire signed [31:0] encoder_position,
    input wire est_valid,
    input wire [2:0] est_quality,
    input wire signed [31:0] est_encoder_velocity_q16, est_adc_velocity_q16,
    input wire core_valid, core_fault, core_overrun,
    input wire signed [31:0] core_theta_velocity_q16, core_alpha_velocity_q16,
    input wire pwm_reject,
    output reg [7:0] state, stop_reason, rx_status,
    output reg config_loaded,
    output reg [15:0] config_id, config_crc,
    output reg [1023:0] coefficients,
    output reg [15:0] max_compare, comp_compare, blend_compare,
    output reg signed [31:0] comp_gain_q20, upright_zero_q16, theta_ref,
    output wire signed [31:0] safety_position_delta,
    output reg output_allow, takeover_pulse,
    output wire est_fresh, est_ok, core_fresh,
    output reg [31:0] state_elapsed_ms
);
    localparam [7:0] SAFE=0, ARMED=1, CAPTURE=2, BALANCE=3, FAULT=255;
    localparam [7:0] R_COMPLETE=1, R_STOP=2, R_OFF=3, R_ABORT=4,
        R_ARM_TIMEOUT=5, R_CAPTURE_TIMEOUT=6, R_SENSOR=16, R_STALE=17,
        R_TRAVEL=18, R_SPEED=19, R_KA=20, R_ANGLE=21, R_NUMERIC=22,
        R_OVERRUN=23, R_PWM=24;
    localparam [31:0] MS_LAST=CLOCKS_PER_MS-1, STALE=STALE_CLOCKS;

    // Keep the complete pending frame immutable until all 32 coefficients and
    // the scalar checks pass. There is no public partially loaded parameter.
    reg pending, pending_bad;
    reg [1503:0] pending_payload;
    reg [15:0] pending_crc;
    reg [5:0] check_index;
    wire [7:0] command=cmd_payload[15:8];
    wire new_config=cmd_valid && command==8'd1;
    wire control_ok=cmd_payload[7:0]==8'd4 && cmd_payload[1503:16]==1488'd0;
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
    wire signed [31:0] checked_coefficient=pending_payload[304+check_index*32+:32];
    wire signed [63:0] gain64={{32{p_gain[31]}},p_gain};
    wire signed [63:0] blend64={48'd0,p_blend}, comp64={48'd0,p_comp};
    wire signed [63:0] slope_error=gain64*blend64-(comp64<<<20);
    wire [63:0] slope_abs=slope_error[63] ? (64'd0-slope_error) : slope_error;
    wire [63:0] slope_tolerance=({48'd0,p_blend}+64'd1)>>1;
    wire scalar_ok=pending_payload[7:0]==8'd4 && p_id!=0 && p_algorithm==1 &&
        p_run>=1 && p_run<=30000 && p_ka>=1 && p_ka<=500 &&
        p_travel>=1 && p_travel<=240 && p_speed>=1 && p_speed<=10000 &&
        p_adc_speed>=1 && p_adc_speed<=30000 && p_max>=1 && p_max<=1125 &&
        p_hold>=MIN_HOLD_MS && p_hold<=1000 &&
        ({1'b0,p_hold}+17'd2)<={1'b0,p_wait} && p_wait<=10000 &&
        p_zero>=32'sd0 && p_zero<=32'sd67043328 &&
        p_enter<=9 && p_exit>=1 && p_exit<=57 && p_exit>p_enter &&
        p_comp<=250 && p_comp<=p_max && p_blend>=25 && p_blend<=250 &&
        p_gain>=32'sd0 && p_gain<=32'sd10485760 && slope_abs<=slope_tolerance &&
        pending_payload[1328+:16]==16'd2000 && pending_payload[1503:1344]==160'd0;

    // pending_payload is immutable for the 32 coefficient-check clocks, so the
    // registered scalar result at commit describes the same frame.
    reg scalar_ok_q;
    reg [15:0] run_ms, keepalive_ms, travel_limit, speed_limit, adc_speed_limit;
    reg [15:0] hold_ms, wait_ms, enter_code, exit_code;
    reg signed [31:0] safety_reference;
    reg signed [33:0] safety_high, safety_low;
    // Angle windows as Q16 bounds around the upright zero, latched at commit.
    reg signed [33:0] enter_high, enter_low, exit_high, exit_low;
    // Signed Q16 speed bounds, latched at commit: |v|>L == v>L || v<-L (L<2^31).
    reg signed [32:0] enc_speed_high, enc_speed_low, adc_speed_high, adc_speed_low;
    reg arm_seen_off;
    reg [31:0] est_age, core_age, ka_div, state_div;
    reg [15:0] ka_ms;
    reg [2:0] last_quality;
    reg signed [31:0] last_ev, last_av, last_cv, last_ca;
    // Next expected snapshot time and capture deadline, each with its 64-bit
    // carry, latched when the reference time is taken (timing only; the
    // predicates below equal the spec's subtract-and-compare forms exactly).
    reg [64:0] tick_expect, qualify_deadline;
    reg tick_seen, qualifying;
    reg [3:0] frame_count;

    function [31:0] abs32;
        input [31:0] value;
        begin abs32=value[31] ? (32'd0-value) : value; end
    endfunction

    wire active=state==ARMED || state==CAPTURE || state==BALANCE;
    wire balancing=state==BALANCE;
    wire [2:0] quality_now=est_valid ? est_quality : last_quality;
    wire [31:0] ev_now=est_valid ? est_encoder_velocity_q16 : last_ev;
    wire [31:0] av_now=est_valid ? est_adc_velocity_q16 : last_av;
    wire [31:0] cv_now=core_valid ? core_theta_velocity_q16 : last_cv;
    wire [31:0] ca_now=core_valid ? core_alpha_velocity_q16 : last_ca;
    assign est_fresh=est_valid || est_age<STALE;
    assign est_ok=est_fresh && quality_now==3'd7;
    assign core_fresh=balancing && (core_valid || core_age<STALE);
    wire ka_fresh=config_loaded && (cmd_valid || ka_ms<keepalive_ms);

    // Both operands are sign extended BEFORE subtraction. Comparing a wrapped
    // int32 delta would turn a full-range encoder jump into a harmless +/-1.
    wire signed [32:0] travel_delta=$signed({encoder_position[31],encoder_position})-
                                  $signed({safety_reference[31],safety_reference});
    // Saturate only the 32-bit diagnostic; the protection always uses 33 bits.
    assign safety_position_delta=(travel_delta>33'sd2147483647) ? 32'sh7fffffff :
        (travel_delta< -33'sd2147483648) ? 32'sh80000000 : travel_delta[31:0];
    // |pos-ref|>limit as two parallel compares against bounds latched at commit.
    wire signed [33:0] position34={{2{encoder_position[31]}},encoder_position};
    wire travel_bad=position34>safety_high || position34<safety_low;
    function over;
        input [31:0] value;
        input signed [32:0] high, low;
        begin over=$signed({value[31],value})>high || $signed({value[31],value})<low; end
    endfunction
    wire speed_bad=over(ev_now,enc_speed_high,enc_speed_low) ||
        over(av_now,adc_speed_high,adc_speed_low) ||
        (balancing && (over(cv_now,enc_speed_high,enc_speed_low) ||
                       over(ca_now,adc_speed_high,adc_speed_low)));
    // |adc<<16 - zero| <= enter<<16 (and > exit<<16) as compares against bounds.
    wire signed [33:0] adc34=$signed({8'd0,adc_raw,16'd0});
    wire enter_ok=adc34<=enter_high && adc34>=enter_low;
    wire angle_bad=balancing && (adc34>exit_high || adc34<exit_low);
    // The exact interface has no adc_valid pin. Top supplies ADC-invalid,
    // encoder-invalid and TX-overflow in sensor_fault_bits, independently of
    // sample/estimator strobes. ADC age is checked on every condition clock.
    wire sensor_bad=adc_age_us>16'd100 || sensor_fault_bits!=0 || quality_now!=3'd7;
    // == (tick<=last || tick-last!=2000): a carry means last+2000 wrapped.
    wire tick_bad=sample_valid && tick_seen &&
        (tick_expect[64] || sample_tick_us!=tick_expect[63:0]);
    wire stale_bad=!est_fresh || tick_bad || (balancing && !core_fresh);
    wire [63:0] hold_us={48'd0,hold_ms}*64'd1000;
    // == (tick>=start && tick-start>=hold): unreachable if start+hold wrapped.
    wire capture_ready=sample_valid && enter_ok && qualifying &&
        !qualify_deadline[64] && sample_tick_us>=qualify_deadline[63:0];

    // Preserve this ordered chain: local stop, raw DIP, sensor, stale, travel,
    // speed, angle, KA, numeric, overrun, PWM, ABORT, normal timeouts/completion.
    // core_fault+core_overrun is specifically an overrun, never numeric_range.
    wire [7:0] trip=stop_key ? R_STOP : !arm_on_raw ? R_OFF :
        sensor_bad ? R_SENSOR : stale_bad ? R_STALE : travel_bad ? R_TRAVEL :
        speed_bad ? R_SPEED : angle_bad ? R_ANGLE : !ka_fresh ? R_KA :
        (core_fault && !core_overrun) ? R_NUMERIC : core_overrun ? R_OVERRUN :
        pwm_reject ? R_PWM : host_abort ? R_ABORT :
        (state==ARMED && state_elapsed_ms>=ARM_TIMEOUT_MS) ? R_ARM_TIMEOUT :
        (state==CAPTURE && state_elapsed_ms>={16'd0,wait_ms}) ? R_CAPTURE_TIMEOUT :
        (balancing && state_elapsed_ms>={16'd0,run_ms}) ? R_COMPLETE : 8'd0;
    // Same predicates as trip!=0 and trip>=R_SENSOR, without the priority chain,
    // so state/output decisions do not wait for the reason encoder.
    wire fault_class=sensor_bad || stale_bad || travel_bad || speed_bad || angle_bad ||
        !ka_fresh || core_fault || core_overrun || pwm_reject;
    wire any_trip=stop_key || !arm_on_raw || fault_class || host_abort ||
        (state==ARMED && state_elapsed_ms>=ARM_TIMEOUT_MS) ||
        (state==CAPTURE && state_elapsed_ms>={16'd0,wait_ms}) ||
        (balancing && state_elapsed_ms>={16'd0,run_ms});
    wire fault_trip=!stop_key && arm_on_raw && fault_class;
    wire arm_ready=config_loaded && !pending && arm_seen_off && arm_on_raw &&
        arm_on_stable && keys_released && !stop_key && !sensor_bad && est_ok && ka_fresh;

    task clear_config;
        begin
            config_loaded<=0; config_id<=0; config_crc<=0; pending<=0;
        end
    endtask
    task enter_state;
        input [7:0] next_state;
        begin state<=next_state; state_div<=0; state_elapsed_ms<=0; end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=SAFE; stop_reason<=0; rx_status<=0; config_loaded<=0;
            config_id<=0; config_crc<=0; coefficients<=0; max_compare<=0;
            comp_compare<=0; blend_compare<=0; comp_gain_q20<=0;
            upright_zero_q16<=0; theta_ref<=0; output_allow<=0;
            takeover_pulse<=0; state_elapsed_ms<=0;
            pending<=0; pending_bad<=0; pending_payload<=0; pending_crc<=0; check_index<=0;
            run_ms<=0; keepalive_ms<=0; travel_limit<=0; speed_limit<=0; adc_speed_limit<=0;
            hold_ms<=0; wait_ms<=0; enter_code<=0; exit_code<=0;
            safety_reference<=0; arm_seen_off<=0; est_age<=STALE; core_age<=STALE;
            last_quality<=0; last_ev<=0; last_av<=0; last_cv<=0; last_ca<=0;
            ka_div<=0; ka_ms<=16'hffff; state_div<=0; frame_count<=0;
            tick_expect<=65'd2000; qualify_deadline<=0; tick_seen<=0; qualifying<=0;
            scalar_ok_q<=0; safety_high<=0; safety_low<=0;
            enter_high<=0; enter_low<=0; exit_high<=0; exit_low<=0;
            enc_speed_high<=0; enc_speed_low<=0; adc_speed_high<=0; adc_speed_low<=0;
        end else begin
            takeover_pulse<=0;
            scalar_ok_q<=scalar_ok;
            if(est_valid) begin
                est_age<=0; last_quality<=est_quality;
                last_ev<=est_encoder_velocity_q16; last_av<=est_adc_velocity_q16;
            end else if(est_age<STALE) est_age<=est_age+32'd1;
            if(balancing && core_valid) begin
                core_age<=0; last_cv<=core_theta_velocity_q16; last_ca<=core_alpha_velocity_q16;
            end else if(core_age<STALE) core_age<=core_age+32'd1;
            if(cmd_valid) begin ka_div<=0; ka_ms<=0; end
            else if(ka_ms!=16'hffff) begin
                if(ka_div==MS_LAST) begin ka_div<=0; ka_ms<=ka_ms+16'd1; end
                else ka_div<=ka_div+32'd1;
            end
            if(state_div==MS_LAST) begin
                state_div<=0;
                if(state_elapsed_ms!=32'hffffffff) state_elapsed_ms<=state_elapsed_ms+32'd1;
            end else state_div<=state_div+32'd1;
            if(!arm_on_stable) arm_seen_off<=1;
            if(active && sample_valid) begin
                tick_expect<={1'b0,sample_tick_us}+65'd2000; tick_seen<=1;
            end

            // 32 serial coefficient checks, then one scalar/commit clock.
            // New CONFIG and valid ABORT have precedence even on commit clock.
            if(pending && !new_config && !host_abort) begin
                if(check_index<6'd32) begin
                    if(abs32(checked_coefficient)>32'd134217728) pending_bad<=1;
                    check_index<=check_index+6'd1;
                end else begin
                    pending<=0;
                    if(!pending_bad && scalar_ok_q) begin
                        config_loaded<=1; config_id<=p_id; config_crc<=pending_crc;
                        coefficients<=pending_payload[304+:1024];
                        max_compare<=p_max; comp_compare<=p_comp; blend_compare<=p_blend;
                        comp_gain_q20<=p_gain; upright_zero_q16<=p_zero;
                        run_ms<=p_run; keepalive_ms<=p_ka; travel_limit<=p_travel;
                        speed_limit<=p_speed; adc_speed_limit<=p_adc_speed;
                        hold_ms<=p_hold; wait_ms<=p_wait; enter_code<=p_enter; exit_code<=p_exit;
                        safety_reference<=encoder_position;
                        enter_high<=$signed({{2{p_zero[31]}},p_zero})+$signed({2'd0,p_enter,16'd0});
                        enter_low<=$signed({{2{p_zero[31]}},p_zero})-$signed({2'd0,p_enter,16'd0});
                        exit_high<=$signed({{2{p_zero[31]}},p_zero})+$signed({2'd0,p_exit,16'd0});
                        exit_low<=$signed({{2{p_zero[31]}},p_zero})-$signed({2'd0,p_exit,16'd0});
                        enc_speed_high<=$signed({1'b0,p_speed,16'd0});
                        enc_speed_low<=-$signed({1'b0,p_speed,16'd0});
                        adc_speed_high<=$signed({1'b0,p_adc_speed,16'd0});
                        adc_speed_low<=-$signed({1'b0,p_adc_speed,16'd0});
                        safety_high<=position34+$signed({18'd0,p_travel});
                        safety_low<=position34-$signed({18'd0,p_travel});
                        rx_status[3:0]<=4'd1;
                    end else rx_status[3:0]<=4'd5;
                end
            end
            if(cmd_valid) begin
                frame_count<=frame_count+4'd1; rx_status[7:4]<=frame_count+4'd1;
                case(command)
                    8'd1: begin
                        if(state!=SAFE) rx_status[3:0]<=4'd4;
                        else begin
                            clear_config;
                            pending<=1; pending_payload<=cmd_payload; pending_crc<=cmd_crc;
                            check_index<=0; pending_bad<=0; rx_status[3:0]<=0;
                        end
                    end
                    8'd2: rx_status[3:0]<=control_ok ? 4'd2 : 4'd5;
                    8'd3: begin
                        rx_status[3:0]<=control_ok ? 4'd3 : 4'd5;
                        if(host_abort && state==SAFE) clear_config;
                    end
                    default: rx_status[3:0]<=4'd6;
                endcase
            end
            if(cmd_error) rx_status[3:0]<=4'd7;

            case(state)
                SAFE: begin
                    output_allow<=0; qualifying<=0; tick_seen<=0;
                    if(arm_ready && !new_config && !host_abort) begin
                        enter_state(ARMED); arm_seen_off<=0;
                        tick_seen<=sample_valid;
                        if(sample_valid) tick_expect<={1'b0,sample_tick_us}+65'd2000;
                    end
                end
                ARMED, CAPTURE, BALANCE: begin
                    if(any_trip) begin
                        stop_reason<=trip; output_allow<=0; clear_config;
                        qualifying<=0; tick_seen<=0; core_age<=STALE;
                        // Lost host keepalive is a latched protection fault;
                        // only ordinary arm/capture/run deadlines return SAFE.
                        if(fault_trip) enter_state(FAULT);
                        else enter_state(SAFE);
                    end else if(state==ARMED && start_event) begin
                        enter_state(CAPTURE); qualifying<=0;
                    end else if(state==CAPTURE && sample_valid) begin
                        if(!enter_ok) qualifying<=0;
                        else if(!qualifying) begin
                            qualifying<=1; qualify_deadline<={1'b0,sample_tick_us}+{1'b0,hold_us};
                        end
                        else if(capture_ready) begin
                            enter_state(BALANCE); output_allow<=1; takeover_pulse<=1;
                            theta_ref<=encoder_position; core_age<=0; last_cv<=0; last_ca<=0;
                        end
                    end
                end
                default: begin output_allow<=0; state<=FAULT; clear_config; end
            endcase
        end
    end
endmodule
