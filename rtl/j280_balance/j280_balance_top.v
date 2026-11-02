`timescale 1ns/1ps
// Manually captured upright balance: local start only, active motor on XH1/B.
// Sampling/control remain 500 Hz; self-describing v4 telemetry is 100 Hz.
module j280_balance_top #(
    parameter integer DEBOUNCE_CLOCKS=1000000,
    parameter integer ARM_ACTIVE_LOW=1,
    parameter integer MIN_HOLD_MS=200
)(
    input wire clk_50m,rst_n,
    input wire [9:0] adc_data,
    input wire adc_otr,enc1_a,enc1_b,uart_rx,key_sw1,key_sw2,dip_sw1,
    output wire adc_clk,adc_oe_n,uart_tx,pwm_a,ain1,ain2,pwm_b,bin1,bin2,
    output wire [3:0] led
);
    localparam [7:0] BALANCE=8'd3,FAULT=8'd255;
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
    j280_balance_cmd_rx parser(.clk(clk_50m),.rst_n(reset_ok),.byte_valid(rx_valid),
        .byte_data(rx_byte),.byte_error(rx_error),.frame_valid(cmd_valid),
        .frame_payload(cmd_payload),.frame_crc(cmd_crc),.frame_error(cmd_error));
    wire start_event,stop_key,arm_raw,arm_stable,keys_released;
    wire [2:0] input_levels;
    j280_local_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS),.ARM_ACTIVE_LOW(ARM_ACTIVE_LOW)) local_inputs(
        .clk(clk_50m),.rst_n(reset_ok),.sw1_n(key_sw1),.sw2_n(key_sw2),.dip1(dip_sw1),
        .start_event(start_event),.stop_pressed(stop_key),.arm_on_raw(arm_raw),
        .arm_on_stable(arm_stable),.keys_released(keys_released),.levels(input_levels));

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
    wire [15:0] config_id,config_crc,max_compare,comp_compare,blend_compare;
    wire [1023:0] coefficients;
    wire signed [31:0] comp_gain_q20,upright_zero_q16,theta_ref,safety_position_delta;
    wire [31:0] state_elapsed_ms;
    wire core_valid,core_fault,core_overrun,core_busy,core_ready;
    wire signed [31:0] xhat0,xhat1,xhat2,xhat3,u_linear,u_compensated,u_model;
    wire signed [15:0] command;
    wire [15:0] core_latency;
    wire signed [63:0] theta_delta_wide=
        $signed({{32{snapshot_position[31]}},snapshot_position})-
        $signed({{32{theta_ref[31]}},theta_ref});
    wire signed [63:0] theta_measure_wide=theta_delta_wide<<<16;
    wire measurement_range_fault=(theta_measure_wide>64'sd2147483647 ||
                                  theta_measure_wide< -64'sd2147483648);
    wire signed [31:0] theta_measure=theta_measure_wide[31:0];
    wire signed [31:0] alpha_measure=$signed({6'd0,snapshot_adc,16'd0})-upright_zero_q16;
    wire core_clear=(state!=BALANCE);
    wire core_start=(snapshot_valid && state==BALANCE) || takeover_pulse;
    j280_balance_core core(.clk(clk_50m),.rst_n(reset_ok),.clear(core_clear),.sample_valid(core_start),
        .theta_q16(theta_measure),.alpha_q16(alpha_measure),.coefficients(coefficients),
        .max_compare(max_compare),.comp_compare(comp_compare),.blend_compare(blend_compare),
        .comp_gain_q20(comp_gain_q20),.feedforward_q16(32'sd0),.filter_mode(4'd0),.preset_valid(1'b0),.preset_x0(32'sd0),.preset_x1(32'sd0),
        .preset_x2(32'sd0),.preset_x3(32'sd0),.busy(core_busy),.sample_ready(core_ready),
        .result_valid(core_valid),.fault(core_fault),.overrun(core_overrun),
        .xhat0(xhat0),.xhat1(xhat1),.xhat2(xhat2),.xhat3(xhat3),
        .u_linear_q16(u_linear),.u_compensated_q16(u_compensated),.u_model_q16(u_model),
        .signed_command(command),.latency_cycles(core_latency));
    wire pin_pwm,pin_in1,pin_in2,pwm_active,pwm_reject;
    wire [15:0] applied_compare;
    wire [1:0] applied_direction;
    j280_balance_pwm #(.MAX_COMPARE(1125)) pwm(.clk(clk_50m),.rst_n(reset_ok),.allow(output_allow),
        .update(core_valid && !core_fault && !measurement_range_fault),.signed_command(command),
        .pwm(pin_pwm),.in1(pin_in1),.in2(pin_in2),.active(pwm_active),
        .applied_compare(applied_compare),.applied_direction(applied_direction),.reject(pwm_reject));
    assign pwm_b=reset_ok?pin_pwm:1'b1;
    assign bin1=reset_ok && pin_in1;
    assign bin2=reset_ok && pin_in2;
    // One register stage on the far-away guard inputs (encoder, estimator, core,
    // PWM, sensor flags). Each strobe moves with its data; the guard responds
    // 20 ns later, which only shortens the routed path into its stop fan-out.
    reg signed [31:0] g_position,g_est_ev,g_est_av,g_core_tv,g_core_av;
    reg g_est_valid,g_core_valid,g_core_fault,g_core_overrun,g_pwm_reject;
    reg [2:0] g_est_quality;
    reg [15:0] g_faults,g_adc_age;
    always @(posedge clk_50m or negedge reset_ok)
        if(!reset_ok) begin
            g_position<=0; g_est_ev<=0; g_est_av<=0; g_core_tv<=0; g_core_av<=0;
            g_est_valid<=0; g_core_valid<=0; g_core_fault<=0; g_core_overrun<=0;
            g_pwm_reject<=0; g_est_quality<=0; g_faults<=0; g_adc_age<=16'hffff;
        end else begin
            g_position<=position; g_est_ev<=est_enc_velocity; g_est_av<=est_adc_velocity;
            g_core_tv<=xhat2; g_core_av<=xhat3; g_est_valid<=est_valid; g_core_valid<=core_valid;
            g_core_fault<=core_fault || measurement_range_fault; g_core_overrun<=core_overrun;
            g_pwm_reject<=pwm_reject; g_est_quality<=est_quality; g_faults<=faults; g_adc_age<=adc_age;
        end
    j280_balance_guard #(.MIN_HOLD_MS(MIN_HOLD_MS)) guard(.clk(clk_50m),.rst_n(reset_ok),
        .cmd_valid(cmd_valid),.cmd_payload(cmd_payload),.cmd_crc(cmd_crc),.cmd_error(cmd_error),
        .start_event(start_event),.stop_key(stop_key),.arm_on_raw(arm_raw),
        .arm_on_stable(arm_stable),.keys_released(keys_released),
        .sample_valid(snapshot_valid),.sample_tick_us(snapshot_tick),.adc_raw(snapshot_adc),
        .adc_age_us(g_adc_age),.sensor_fault_bits(g_faults),.encoder_position(g_position),
        .est_valid(g_est_valid),.est_quality(g_est_quality),.est_encoder_velocity_q16(g_est_ev),
        .est_adc_velocity_q16(g_est_av),.core_valid(g_core_valid),
        .core_fault(g_core_fault),.core_overrun(g_core_overrun),
        .core_theta_velocity_q16(g_core_tv),.core_alpha_velocity_q16(g_core_av),.pwm_reject(g_pwm_reject),
        .state(state),.stop_reason(stop_reason),.rx_status(rx_status),.config_loaded(config_loaded),
        .config_id(config_id),.config_crc(config_crc),.coefficients(coefficients),
        .max_compare(max_compare),.comp_compare(comp_compare),.blend_compare(blend_compare),
        .comp_gain_q20(comp_gain_q20),.upright_zero_q16(upright_zero_q16),.theta_ref(theta_ref),
        .safety_position_delta(safety_position_delta),.output_allow(output_allow),
        .takeover_pulse(takeover_pulse),.est_fresh(est_fresh),.est_ok(est_ok),
        .core_fresh(core_fresh),.state_elapsed_ms(state_elapsed_ms));

    reg [2:0] quality_last;
    reg [127:0] xhat_last;
    reg signed [31:0] linear_last,compensated_last,model_last;
    reg [31:0] controller_age_us,latency_last;
    reg have_result;
    wire core_ok=have_result && core_fresh && !core_fault;
    wire [7:0] flags={(state==FAULT),core_ok,arm_stable,config_loaded,tx_overflow,
                     pwm_active,adc_new,adc_valid};
    wire [7:0] motor_mode={5'd0,!pwm_active,pin_in2,pin_in1};
    reg [991:0] telemetry_now;
    always @* begin
        telemetry_now=992'd0;
        telemetry_now[0+:8]=8'd4;
        telemetry_now[8+:8]=flags;
        telemetry_now[16+:32]=telemetry_sequence;
        telemetry_now[48+:64]=tick_next;
        telemetry_now[112+:32]=raw_sequence;
        telemetry_now[144+:16]={6'd0,raw};
        telemetry_now[160+:16]=adc_age;
        telemetry_now[176+:32]=position;
        telemetry_now[208+:32]=theta_ref;
        telemetry_now[240+:32]=upright_zero_q16;
        telemetry_now[272+:16]=config_id;
        telemetry_now[288+:16]=config_crc;
        telemetry_now[304+:8]=state;
        telemetry_now[312+:8]=stop_reason;
        telemetry_now[320+:8]={5'd0,input_levels};
        telemetry_now[328+:8]=motor_mode;
        telemetry_now[336+:16]=applied_compare;
        telemetry_now[352+:16]=16'd2500;
        telemetry_now[368+:32]=linear_last;
        telemetry_now[400+:32]=compensated_last;
        telemetry_now[432+:128]=xhat_last;
        telemetry_now[560+:32]=controller_age_us;
        telemetry_now[592+:16]=faults;
        telemetry_now[608+:8]={5'd0,quality_last};
        telemetry_now[616+:8]=rx_status;
        telemetry_now[624+:32]=latency_last;
        telemetry_now[656+:32]=32'd2000;
        telemetry_now[688+:32]=32'd10000;
        telemetry_now[720+:32]=state_elapsed_ms;
        telemetry_now[752+:32]=safety_position_delta;
        telemetry_now[784+:32]=model_last;
    end
    always @(posedge clk_50m or negedge reset_ok) begin
        if(!reset_ok) begin
            us_divider<=0; tick_us<=0; sample_divider<=0; raw_sequence<=0;
            telemetry_sequence<=0; telemetry_phase<=0; adc_new<=0;
            snapshot_valid<=0; snapshot<=0; snapshot_tick<=0; snapshot_adc<=0;
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
            if(takeover_pulse) begin have_result<=0; controller_age_us<=32'hffffffff; end
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
                snapshot_tick<=tick_next; snapshot_adc<=raw; snapshot_position<=position;
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
    assign led[0]=pwm_active;
    assign led[1]=(state==8'd3)?1'b1:(state==8'd2)?tick_us[16]:
                  (state==8'd1)?tick_us[17]:(config_loaded?tick_us[19]:1'b0);
    assign led[2]=tick_us[20];
    assign led[3]=(state==FAULT);
endmodule
