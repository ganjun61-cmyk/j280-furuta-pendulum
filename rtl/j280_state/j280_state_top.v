`timescale 1ns/1ps
// Passive state telemetry v2: 500 Hz snapshots and raw-quantity estimates,
// 100 Hz 128-byte UART packets. No motor enable, no UART RX, no controller.
module j280_state_top (
    input wire clk_50m,
    input wire rst_n,
    input wire [9:0] adc_data,
    input wire adc_otr,
    input wire enc1_a, enc1_b,
    output wire adc_clk, adc_oe_n,
    output wire uart_tx,
    output wire pwm_a, ain1, ain2, pwm_b, bin1, bin2,
    output wire [1:0] led
);
    localparam [16:0] SAMPLE_CLOCKS = 17'd100000;
    localparam [15:0] WINDOW_US = 16'd10000;
    localparam [15:0] PERIOD_US = 16'd2000;
    localparam [15:0] MAX_ADC_AGE_US = 16'd100;
    localparam [15:0] MAX_ADC_STEP = 16'd64;
    localparam [15:0] MAX_ENCODER_STEP = 16'd64;
    localparam [15:0] TELEMETRY_PERIOD_US = 16'd10000;
    localparam [7:0] LAST_PHASE = 8'd4;

    assign {pwm_a,ain1,ain2,pwm_b,bin1,bin2} = 6'b000000;
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
    wire tx_busy, tx_overflow;
    wire [7:0] flags = {4'b0,tx_overflow,1'b0,adc_new,adc_valid};
    wire [15:0] faults = {13'b0,tx_overflow,(|illegal_count),!adc_valid};
    wire [15:0] output_age_next = ((us_divider==49) && (output_age_us!=16'hffff))
                                  ? output_age_us+16'd1 : output_age_us;

    always @(posedge clk_50m or negedge reset_ok) begin
        if (!reset_ok) begin
            us_divider<=0; tick_us<=0; output_age_us<=0;
            sample_divider<=0; frame_sequence<=0;
            adc_new<=0; snapshot_valid<=0; snapshot<=0; snapshot_tick<=0;
        end else begin
            if (us_divider==49) us_divider<=0;
            else us_divider<=us_divider+1'b1;
            tick_us<=tick_next;
            output_age_us<=output_age_next;
            snapshot_valid<=0;
            if (sample_divider==SAMPLE_CLOCKS-17'd1) begin
                sample_divider<=0;
                snapshot<={output_age_next,faults,8'd2,8'd0,16'd0,16'd0,
                           position,adc_age,{6'd0,raw},tick_next[31:0],frame_sequence,
                           flags,8'd2};
                snapshot_tick<=tick_next;
                frame_sequence<=frame_sequence+1'b1;
                snapshot_valid<=1;
                adc_new<=0;
            end else sample_divider<=sample_divider+1'b1;
            // As in v1, a strobe coinciding with the snapshot edge is counted
            // toward the next snapshot (conservative; raw already holds it).
            if (adc_strobe) adc_new<=1;
        end
    end

    wire result_valid;
    wire [223:0] result_payload;
    wire [63:0] result_tick_us, adc_position_time_us;
    wire [31:0] segment_id, encoder_span_us, adc_span_us;
    wire signed [63:0] encoder_position;
    wire signed [31:0] encoder_velocity_q16, adc_velocity_q16;
    wire [63:0] encoder_midpoint_x2_us, adc_midpoint_x2_us;
    wire [31:0] encoder_age_x2_us, adc_age_x2_us;
    wire [2:0] quality;
    wire [7:0] reason;
    wire estimator_ready;
    j280_raw_estimator #(.WINDOW_US(WINDOW_US),.PERIOD_US(PERIOD_US),
        .MAX_ADC_AGE_US(MAX_ADC_AGE_US),.MAX_ADC_STEP(MAX_ADC_STEP),
        .MAX_ENCODER_STEP(MAX_ENCODER_STEP),.HISTORY_DEPTH(8)) estimator (
        .clk(clk_50m),.rst_n(reset_ok),.sample_valid(snapshot_valid),
        .sample_payload(snapshot),.sample_tick_us(snapshot_tick),
        .sample_ready(estimator_ready),.result_valid(result_valid),
        .result_payload(result_payload),.result_tick_us(result_tick_us),
        .segment_id(segment_id),.encoder_position(encoder_position),
        .encoder_velocity_q16(encoder_velocity_q16),.adc_velocity_q16(adc_velocity_q16),
        .encoder_span_us(encoder_span_us),.adc_span_us(adc_span_us),
        .adc_position_time_us(adc_position_time_us),
        .encoder_midpoint_x2_us(encoder_midpoint_x2_us),
        .adc_midpoint_x2_us(adc_midpoint_x2_us),
        .encoder_age_x2_us(encoder_age_x2_us),.adc_age_x2_us(adc_age_x2_us),
        .quality(quality),.reason(reason));

    // Estimator outputs hold until the next result (2 ms), so the packet
    // sender's single-cycle latch takes one atomic snapshot.
    reg [7:0] telemetry_phase;
    reg [31:0] packet_seq;
    reg packet_valid;
    reg [2:0] last_quality;
    wire [991:0] packet = {
        64'd0, TELEMETRY_PERIOD_US, packet_seq,
        MAX_ENCODER_STEP, MAX_ADC_STEP, MAX_ADC_AGE_US,
        PERIOD_US, WINDOW_US, 16'd1, reason, {5'd0, quality},
        adc_age_x2_us, encoder_age_x2_us, adc_midpoint_x2_us, encoder_midpoint_x2_us,
        adc_position_time_us, adc_span_us, encoder_span_us,
        adc_velocity_q16, encoder_velocity_q16, encoder_position,
        result_tick_us, segment_id, result_payload};

    always @(posedge clk_50m or negedge reset_ok) begin
        if (!reset_ok) begin
            telemetry_phase <= 8'd0; packet_seq <= 32'd0;
            packet_valid <= 1'b0; last_quality <= 3'd0;
        end else begin
            if (packet_valid) packet_seq <= packet_seq + 32'd1;
            packet_valid <= 1'b0;
            if (result_valid) begin
                last_quality <= quality;
                packet_valid <= (telemetry_phase == 8'd0);
                telemetry_phase <= (telemetry_phase == LAST_PHASE) ? 8'd0 : telemetry_phase + 8'd1;
            end
        end
    end

    j280_state_packet_tx transmitter(.clk(clk_50m),.rst_n(reset_ok),
        .packet_valid(packet_valid),.payload(packet),.uart_tx(uart_tx),
        .overflow(tx_overflow),.busy(tx_busy));
    assign led[0] = (last_quality == 3'd7);
    assign led[1] = tick_us[19];
endmodule
