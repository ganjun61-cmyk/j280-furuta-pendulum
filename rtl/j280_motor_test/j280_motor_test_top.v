`timescale 1ns/1ps
// Bounded motor test v3: passive v2 sensing and raw estimator, one host-loaded
// constant-duty pulse on TB6612 channel B started only by the local SW1 key,
// guarded by local stop/arm inputs, sensor validity, travel, speed, keepalive
// and time. 40-byte telemetry every 2 ms snapshot. Channel A is always 0.
// The XH1 motor lines are wired to U4 BO1/BO2 in the baseboard schematic,
// although the user manual labels them channel A; XH2 (channel A) is unused.
module j280_motor_test_top #(
    parameter integer DEBOUNCE_CLOCKS = 1000000,
    parameter integer ARM_ACTIVE_LOW = 1
) (
    input wire clk_50m,
    input wire rst_n,
    input wire [9:0] adc_data,
    input wire adc_otr,
    input wire enc1_a, enc1_b,
    input wire uart_rx,
    input wire key_sw1, key_sw2, dip_sw1,
    output wire adc_clk, adc_oe_n,
    output wire uart_tx,
    output wire pwm_a, ain1, ain2, pwm_b, bin1, bin2,
    output wire [3:0] led
);
    localparam integer CLOCKS_PER_MS = 50000;
    localparam integer PWM_PERIOD = 2500;
    localparam integer PERIODS_PER_MS = CLOCKS_PER_MS / PWM_PERIOD;
    localparam integer MAX_COMPARE = 1000;
    localparam integer MAX_PULSE_MS = 2000;
    localparam [16:0] SAMPLE_CLOCKS = 17'd100000;

    assign {pwm_a,ain1,ain2} = 3'b000;
    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_pipe;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) reset_pipe <= 0;
        else reset_pipe <= {reset_pipe[0],1'b1};
    end
    wire reset_ok = reset_pipe[1];

    wire [9:0] raw;
    wire adc_valid, adc_strobe;
    wire [15:0] adc_age;
    j280_adc_capture adc_sampler(.clk(clk_50m),.rst_n(reset_ok),
        .adc_data(adc_data),.adc_otr(adc_otr),.adc_clk(adc_clk),
        .adc_oe_n(adc_oe_n),.raw(raw),.valid(adc_valid),
        .sample_strobe(adc_strobe),.age_us(adc_age));
    wire signed [31:0] position;
    wire [31:0] illegal_count;
    wire encoder_ready;
    j280_encoder encoder(.clk(clk_50m),.rst_n(reset_ok),
        .enc_a(enc1_a),.enc_b(enc1_b),.position(position),
        .illegal_count(illegal_count),.ready(encoder_ready));

    wire [7:0] rx_byte;
    wire rx_byte_valid, rx_byte_error, cmd_valid, cmd_error;
    wire [127:0] cmd_payload;
    wire [15:0] cmd_crc;
    j280_uart_rx receiver(.clk(clk_50m),.rst_n(reset_ok),.rx(uart_rx),.data(rx_byte),
        .valid(rx_byte_valid),.frame_error(rx_byte_error));
    j280_cmd_rx parser(.clk(clk_50m),.rst_n(reset_ok),.byte_valid(rx_byte_valid),
        .byte_data(rx_byte),.byte_error(rx_byte_error),.frame_valid(cmd_valid),
        .frame_payload(cmd_payload),.frame_crc(cmd_crc),.frame_error(cmd_error));

    wire start_event, stop_pressed, arm_on_raw, arm_on_stable, keys_released;
    wire [2:0] input_levels;
    j280_local_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS),.ARM_ACTIVE_LOW(ARM_ACTIVE_LOW)) inputs(
        .clk(clk_50m),.rst_n(reset_ok),.sw1_n(key_sw1),.sw2_n(key_sw2),.dip1(dip_sw1),
        .start_event(start_event),.stop_pressed(stop_pressed),.arm_on_raw(arm_on_raw),
        .arm_on_stable(arm_on_stable),.keys_released(keys_released),.levels(input_levels));

    reg [5:0] us_divider;
    reg [63:0] tick_us;
    reg [15:0] output_age_us;
    wire [63:0] tick_next = (us_divider==49) ? tick_us+64'd1 : tick_us;
    reg [16:0] sample_divider;
    reg [31:0] frame_sequence;
    reg adc_new;
    reg snapshot_valid;
    reg [223:0] snapshot;
    reg [63:0] snapshot_tick;
    reg [287:0] telemetry;
    wire tx_busy, tx_overflow;
    wire [7:0] flags_v2 = {4'b0,tx_overflow,1'b0,adc_new,adc_valid};
    wire [15:0] faults = {13'b0,tx_overflow,(|illegal_count),!adc_valid};

    wire est_valid;
    wire [2:0] est_quality;
    wire signed [31:0] est_enc_velocity;
    j280_raw_estimator #(.WINDOW_US(10000),.PERIOD_US(2000),.MAX_ADC_AGE_US(100),
        .MAX_ADC_STEP(64),.MAX_ENCODER_STEP(64),.HISTORY_DEPTH(8)) estimator (
        .clk(clk_50m),.rst_n(reset_ok),.sample_valid(snapshot_valid),
        .sample_payload(snapshot),.sample_tick_us(snapshot_tick),
        .sample_ready(),.result_valid(est_valid),.result_payload(),.result_tick_us(),
        .segment_id(),.encoder_position(),.encoder_velocity_q16(est_enc_velocity),
        .adc_velocity_q16(),.encoder_span_us(),.adc_span_us(),.adc_position_time_us(),
        .encoder_midpoint_x2_us(),.adc_midpoint_x2_us(),.encoder_age_x2_us(),
        .adc_age_x2_us(),.quality(est_quality),.reason());

    wire [7:0] state, stop_reason, rx_status;
    wire config_loaded, pwm_start, output_allow, est_fresh, est_ok;
    wire [15:0] config_id, config_crc, drive_compare_cfg;
    wire [1:0] drive_direction;
    wire [31:0] drive_periods;
    wire pwm_done, pwm_reject, driving, pin_pwm, pin_in1, pin_in2;
    wire [15:0] applied_compare;
    j280_motor_guard #(.CLOCKS_PER_MS(CLOCKS_PER_MS),.PERIODS_PER_MS(PERIODS_PER_MS),
        .MAX_COMPARE(MAX_COMPARE),.MAX_PULSE_MS(MAX_PULSE_MS),.MAX_TRAVEL_COUNT(200),
        .MAX_SPEED_COUNT_S(10000),.MAX_KEEPALIVE_MS(500),.ARM_TIMEOUT_MS(60000),
        .RUN_GRACE_MS(10),.STALE_CLOCKS(250000)) guard (
        .clk(clk_50m),.rst_n(reset_ok),.cmd_valid(cmd_valid),.cmd_payload(cmd_payload),
        .cmd_crc(cmd_crc),.cmd_error(cmd_error),.start_event(start_event),
        .stop_key(stop_pressed),.arm_on_raw(arm_on_raw),.arm_on_stable(arm_on_stable),
        .keys_released(keys_released),.est_valid(est_valid),.est_quality(est_quality),
        .est_velocity_q16(est_enc_velocity),.encoder_position(position),
        .pwm_done(pwm_done),.pwm_reject(pwm_reject),.state(state),.stop_reason(stop_reason),
        .config_loaded(config_loaded),.config_id(config_id),.config_crc(config_crc),
        .drive_direction(drive_direction),.drive_compare(drive_compare_cfg),
        .drive_periods(drive_periods),.pwm_start(pwm_start),.output_allow(output_allow),
        .rx_status(rx_status),.est_fresh(est_fresh),.est_ok(est_ok));
    j280_pwm_pulse #(.PERIOD(PWM_PERIOD),.MAX_COMPARE(MAX_COMPARE),
        .MAX_PERIODS(MAX_PULSE_MS*PERIODS_PER_MS)) pulse (
        .clk(clk_50m),.rst_n(reset_ok),.allow(output_allow),.start(pwm_start),
        .direction(drive_direction),.compare(drive_compare_cfg),.periods(drive_periods),
        .pwm(pin_pwm),.in1(pin_in1),.in2(pin_in2),.driving(driving),
        .drive_compare(applied_compare),.done(pwm_done),.reject(pwm_reject));
    // Held in reset the pulse module drives the same idle Stop combination.
    assign pwm_b = reset_ok ? pin_pwm : 1'b1;
    assign bin1 = reset_ok & pin_in1;
    assign bin2 = reset_ok & pin_in2;

    reg [2:0] est_quality_last;
    reg [18:0] previous_output;
    wire [18:0] output_now = {driving, pin_in1, pin_in2, applied_compare};
    wire output_changed = (output_now != previous_output);
    wire [15:0] output_age_next = output_changed ? 16'd0 :
                                  ((us_divider==49) && (output_age_us!=16'hffff))
                                  ? output_age_us+16'd1 : output_age_us;
    wire [7:0] flags_v3 = {1'b0, est_ok, arm_on_stable, config_loaded, tx_overflow,
                           driving, adc_new, adc_valid};
    wire [7:0] motor_mode = {5'd0, !driving, pin_in2, pin_in1};
    wire [7:0] estimator_byte = {4'd0, est_fresh, est_quality_last};

    always @(posedge clk_50m or negedge reset_ok) begin
        if (!reset_ok) begin
            us_divider<=0; tick_us<=0; output_age_us<=0;
            sample_divider<=0; frame_sequence<=0;
            adc_new<=0; snapshot_valid<=0; snapshot<=0; snapshot_tick<=0;
            telemetry<=0; est_quality_last<=0; previous_output<=0;
        end else begin
            if (us_divider==49) us_divider<=0;
            else us_divider<=us_divider+1'b1;
            tick_us<=tick_next;
            output_age_us<=output_age_next;
            previous_output<=output_now;
            if (est_valid) est_quality_last<=est_quality;
            snapshot_valid<=0;
            if (sample_divider==SAMPLE_CLOCKS-17'd1) begin
                sample_divider<=0;
                snapshot<={output_age_next,faults,8'd2,8'd0,16'd0,16'd0,
                           position,adc_age,{6'd0,raw},tick_next[31:0],frame_sequence,
                           flags_v2,8'd2};
                telemetry<={config_crc,config_id,{5'd0,input_levels},rx_status,
                            estimator_byte,stop_reason,output_age_next,faults,state,motor_mode,
                            16'd2500,applied_compare,position,adc_age,{6'd0,raw},
                            tick_next[31:0],frame_sequence,flags_v3,8'd3};
                snapshot_tick<=tick_next;
                frame_sequence<=frame_sequence+1'b1;
                snapshot_valid<=1;
                adc_new<=0;
            end else sample_divider<=sample_divider+1'b1;
            if (adc_strobe) adc_new<=1;
        end
    end

    j280_state_packet_tx #(.PAYLOAD_BYTES(36)) transmitter(.clk(clk_50m),.rst_n(reset_ok),
        .packet_valid(snapshot_valid),.payload(telemetry),.uart_tx(uart_tx),
        .overflow(tx_overflow),.busy(tx_busy));

    assign led[0] = driving;
    assign led[1] = (state == 8'd2) ? 1'b1 :
                    (state == 8'd1) ? tick_us[17] :
                    (state == 8'd0 && config_loaded) ? tick_us[19] : 1'b0;
    assign led[2] = tick_us[20];
    assign led[3] = (state == 8'd255);
endmodule
