`timescale 1ns/1ps

// Passive raw-quantity window estimator for one 28-byte snapshot (v2 layout).
// Positions plus encoder/ADC window velocities in signed Q16.16, with explicit
// validity; no control or motor outputs. Every accepted snapshot produces one
// result exactly RESULT_LATENCY clocks later, independent of the data path.
// sample_ready is always high: a submission while busy cancels the previous
// computation and is itself reported as input_overrun (reason 11).
module j280_raw_estimator #(
    parameter integer WINDOW_US = 10000,
    parameter integer PERIOD_US = 2000,
    parameter integer MAX_ADC_AGE_US = 100,
    parameter integer MAX_ADC_STEP = 64,
    parameter integer MAX_ENCODER_STEP = 64,
    parameter integer HISTORY_DEPTH = 8
) (
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire [223:0] sample_payload,
    input wire [63:0] sample_tick_us,
    output wire sample_ready,
    output reg result_valid,
    output reg [223:0] result_payload,
    output reg [63:0] result_tick_us,
    output reg [31:0] segment_id,
    output reg signed [63:0] encoder_position,
    output reg signed [31:0] encoder_velocity_q16,
    output reg signed [31:0] adc_velocity_q16,
    output reg [31:0] encoder_span_us,
    output reg [31:0] adc_span_us,
    output reg [63:0] adc_position_time_us,
    output reg [63:0] encoder_midpoint_x2_us,
    output reg [63:0] adc_midpoint_x2_us,
    output reg [31:0] encoder_age_x2_us,
    output reg [31:0] adc_age_x2_us,
    output reg [2:0] quality,
    output reg [7:0] reason
);
    function integer clog2;
        input integer value;
        integer remaining;
        begin
            remaining = value - 1;
            clog2 = 0;
            while (remaining > 0) begin
                clog2 = clog2 + 1;
                remaining = remaining >> 1;
            end
            if (clog2 < 1) clog2 = 1;
        end
    endfunction

    localparam integer SAFE_PERIOD = (PERIOD_US > 0) ? PERIOD_US : 1;
    localparam CONFIG_OK = (PERIOD_US > 0) && (WINDOW_US > 0) &&
                           (WINDOW_US % SAFE_PERIOD == 0) &&
                           (HISTORY_DEPTH >= WINDOW_US / SAFE_PERIOD + 2) &&
                           (MAX_ADC_AGE_US >= 0) && (MAX_ADC_AGE_US <= 65535) &&
                           (MAX_ADC_AGE_US < PERIOD_US) &&
                           (MAX_ADC_STEP >= 0) && (MAX_ADC_STEP <= 1023) &&
                           (MAX_ENCODER_STEP >= 0);
    localparam integer DEPTH = (HISTORY_DEPTH > 1) ? HISTORY_DEPTH : 2;
    localparam integer IDX_W = clog2(DEPTH);
    localparam [IDX_W-1:0] LAST_IDX = DEPTH - 1;
    localparam [IDX_W:0] DEPTH_COUNT = DEPTH;
    localparam integer RESULT_LATENCY = 256;
    localparam [8:0] EMIT_COUNT = RESULT_LATENCY - 1;
    localparam [63:0] WINDOW64 = CONFIG_OK ? WINDOW_US : 0;
    localparam [63:0] PERIOD64 = CONFIG_OK ? PERIOD_US : 0;
    localparam [15:0] AGE_LIMIT = CONFIG_OK ? MAX_ADC_AGE_US : 0;
    localparam [16:0] ADC_STEP_LIMIT = CONFIG_OK ? MAX_ADC_STEP : 0;
    localparam [32:0] ENC_STEP_LIMIT = CONFIG_OK ? MAX_ENCODER_STEP : 0;
    localparam [63:0] MAX_TIME = 64'h000f_ffff_ffff_ffff;

    localparam [3:0] S_IDLE = 4'd0, S_VAL1 = 4'd1, S_VAL2 = 4'd2, S_SCAN = 4'd3,
                     S_PREP1 = 4'd4, S_PREP2 = 4'd5, S_PREP3 = 4'd6, S_DIV = 4'd7,
                     S_HOLD = 4'd8;

    assign sample_ready = 1'b1;

    // Accepted snapshot context.
    reg [3:0] state;
    reg [8:0] lat_count;
    reg [223:0] s_payload;
    reg [63:0] s_tick;
    reg s_overrun;

    wire [7:0] f_version = s_payload[7:0];
    wire [7:0] f_flags = s_payload[15:8];
    wire [31:0] f_seq = s_payload[47:16];
    wire [31:0] f_low = s_payload[79:48];
    wire [15:0] f_adc = s_payload[95:80];
    wire [15:0] f_age = s_payload[111:96];
    wire [31:0] f_enc = s_payload[143:112];
    wire [7:0] f_state = s_payload[191:184];
    wire [15:0] f_fault = s_payload[207:192];

    // Committed segment history; the newest entry is also kept in prev_*.
    reg [31:0] seg_count;
    reg [IDX_W:0] hist_count;
    reg [IDX_W-1:0] head;
    reg [31:0] prev_seq;
    reg [31:0] prev_enc;
    reg [63:0] prev_tick;
    reg [63:0] prev_atime;
    reg [9:0] prev_adc;
    reg signed [63:0] prev_pos;
    reg [63:0] hist_tick [0:DEPTH-1];
    reg [63:0] hist_atime [0:DEPTH-1];
    reg [63:0] hist_pos [0:DEPTH-1];
    reg [9:0] hist_adc [0:DEPTH-1];
    wire prev_valid = (hist_count != 0);

    // Stage 1 checks.
    reg v_hdr_bad, v_fault, v_age_bad, v_range_bad, v_src_bad, v_seq_gap, v_tick_gap;
    reg [63:0] v_atime;
    reg [31:0] v_delta;
    reg [32:0] v_delta_abs;
    reg [16:0] v_adc_abs;

    // Stage 2 decision.
    reg r_reject;
    reg [7:0] r_reason;
    reg [7:0] r_boundary;
    reg r_new_seg;
    reg [31:0] r_seg_next;
    reg signed [63:0] r_pos;
    reg [63:0] thr_e, thr_a;
    reg thr_e_ok, thr_a_ok;

    // Window endpoints.
    reg [IDX_W-1:0] scan_idx;
    reg [IDX_W:0] scan_k;
    reg enc_found, adc_found;
    reg [63:0] enc_start_t, adc_start_t;
    reg signed [63:0] enc_start_pos;
    reg [9:0] adc_start_v;

    // Per-channel arithmetic (ch 0 = encoder, 1 = ADC).
    reg ch;
    reg numeric_fail;
    reg [63:0] p_span, p_mid;
    reg [64:0] p_diff;
    reg [64:0] p_abs;
    reg p_neg;
    reg [65:0] p_age2;
    reg p_bad;
    reg [47:0] p_product;
    reg div_start;
    reg [63:0] e_mid, a_mid;
    reg [31:0] e_span, a_span, e_age2, a_age2;
    reg signed [31:0] e_q, a_q;

    wire div_done, div_overflow;
    wire signed [31:0] div_quotient;
    j280_signed_divider #(.NUM_WIDTH(64), .DEN_WIDTH(32)) divider (
        .clk(clk), .rst_n(rst_n), .clear(sample_valid), .start(div_start),
        .negative(p_neg), .magnitude({p_product, 16'd0}), .divisor(p_span[31:0]),
        .done(div_done), .quotient(div_quotient), .overflow(div_overflow));

    wire emit_now = (state == S_HOLD) && (lat_count >= EMIT_COUNT);
    wire emit_bad = r_reject || numeric_fail;
    wire commit_we = emit_now && !emit_bad;
    wire [IDX_W-1:0] write_idx = r_new_seg ? {IDX_W{1'b0}} : head;
    wire [IDX_W-1:0] head_after = (write_idx == LAST_IDX) ? {IDX_W{1'b0}} : write_idx + 1'b1;

    wire [31:0] delta_now = f_enc - prev_enc;
    wire [16:0] adc_delta_now = {7'd0, f_adc[9:0]} - {7'd0, prev_adc};
    wire [63:0] cur_t = ch ? v_atime : s_tick;
    wire [63:0] start_t = ch ? adc_start_t : enc_start_t;
    wire [64:0] diff_now = ch ? ({55'd0, f_adc[9:0]} - {55'd0, adc_start_v})
                              : ({r_pos[63], r_pos} - {enc_start_pos[63], enc_start_pos});
    wire [63:0] scan_tick = hist_tick[scan_idx];
    wire [63:0] scan_atime = hist_atime[scan_idx];

    always @(posedge clk) begin
        if (commit_we) begin
            hist_tick[write_idx] <= s_tick;
            hist_atime[write_idx] <= v_atime;
            hist_pos[write_idx] <= r_pos;
            hist_adc[write_idx] <= f_adc[9:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; lat_count <= 9'd0;
            s_payload <= 224'd0; s_tick <= 64'd0; s_overrun <= 1'b0;
            seg_count <= 32'd0; hist_count <= 0; head <= 0;
            prev_seq <= 32'd0; prev_enc <= 32'd0; prev_tick <= 64'd0;
            prev_atime <= 64'd0; prev_adc <= 10'd0; prev_pos <= 64'sd0;
            v_hdr_bad <= 1'b0; v_fault <= 1'b0; v_age_bad <= 1'b0; v_range_bad <= 1'b0;
            v_src_bad <= 1'b0; v_seq_gap <= 1'b0; v_tick_gap <= 1'b0;
            v_atime <= 64'd0; v_delta <= 32'd0; v_delta_abs <= 33'd0; v_adc_abs <= 17'd0;
            r_reject <= 1'b0; r_reason <= 8'd0; r_boundary <= 8'd0; r_new_seg <= 1'b0;
            r_seg_next <= 32'd0; r_pos <= 64'sd0;
            thr_e <= 64'd0; thr_a <= 64'd0; thr_e_ok <= 1'b0; thr_a_ok <= 1'b0;
            scan_idx <= 0; scan_k <= 0; enc_found <= 1'b0; adc_found <= 1'b0;
            enc_start_t <= 64'd0; adc_start_t <= 64'd0; enc_start_pos <= 64'sd0; adc_start_v <= 10'd0;
            ch <= 1'b0; numeric_fail <= 1'b0;
            p_span <= 64'd0; p_mid <= 64'd0; p_diff <= 65'd0; p_abs <= 65'd0; p_neg <= 1'b0;
            p_age2 <= 66'd0; p_bad <= 1'b0; p_product <= 48'd0; div_start <= 1'b0;
            e_mid <= 64'd0; a_mid <= 64'd0; e_span <= 32'd0; a_span <= 32'd0;
            e_age2 <= 32'd0; a_age2 <= 32'd0; e_q <= 32'sd0; a_q <= 32'sd0;
            result_valid <= 1'b0; result_payload <= 224'd0; result_tick_us <= 64'd0;
            segment_id <= 32'd0; encoder_position <= 64'sd0;
            encoder_velocity_q16 <= 32'sd0; adc_velocity_q16 <= 32'sd0;
            encoder_span_us <= 32'd0; adc_span_us <= 32'd0; adc_position_time_us <= 64'd0;
            encoder_midpoint_x2_us <= 64'd0; adc_midpoint_x2_us <= 64'd0;
            encoder_age_x2_us <= 32'd0; adc_age_x2_us <= 32'd0;
            quality <= 3'd0; reason <= 8'd0;
        end else begin
            result_valid <= 1'b0;
            div_start <= 1'b0;
            if (state != S_IDLE && lat_count != 9'h1ff)
                lat_count <= lat_count + 9'd1;

            case (state)
                S_VAL1: begin
                    v_hdr_bad <= (f_flags != 8'd3) || (f_version != 8'd2) || (f_state != 8'd2);
                    v_fault <= (f_fault != 16'd0);
                    v_age_bad <= (f_age > AGE_LIMIT);
                    v_range_bad <= (f_adc > 16'd1023) || (s_tick > MAX_TIME) || (f_low != s_tick[31:0]);
                    v_src_bad <= (s_tick < {48'd0, f_age});
                    v_atime <= s_tick - {48'd0, f_age};
                    v_seq_gap <= prev_valid && (f_seq != prev_seq + 32'd1);
                    v_tick_gap <= prev_valid && (s_tick != prev_tick + PERIOD64);
                    v_delta <= delta_now;
                    v_delta_abs <= delta_now[31] ? (33'd0 - {1'b1, delta_now}) : {1'b0, delta_now};
                    v_adc_abs <= adc_delta_now[16] ? (17'd0 - adc_delta_now) : adc_delta_now;
                    state <= S_VAL2;
                end
                S_VAL2: begin
                    r_reject <= 1'b1;
                    r_new_seg <= !prev_valid || v_seq_gap || v_tick_gap;
                    r_boundary <= !prev_valid ? 8'd12 : v_seq_gap ? 8'd5 : v_tick_gap ? 8'd6 : 8'd0;
                    r_seg_next <= seg_count;
                    r_pos <= 64'sd0;
                    thr_e <= s_tick - WINDOW64;
                    thr_a <= v_atime - WINDOW64;
                    thr_e_ok <= (s_tick >= WINDOW64);
                    thr_a_ok <= (v_atime >= WINDOW64);
                    scan_idx <= (head == {IDX_W{1'b0}}) ? LAST_IDX : head - 1'b1;
                    scan_k <= 0;
                    state <= S_HOLD;
                    if (s_overrun) r_reason <= 8'd11;
                    else if (!CONFIG_OK) r_reason <= 8'd10;
                    else if (v_hdr_bad) r_reason <= 8'd2;
                    else if (v_fault) r_reason <= 8'd3;
                    else if (v_age_bad) r_reason <= 8'd4;
                    else if (v_range_bad) r_reason <= 8'd10;
                    else if (v_src_bad) r_reason <= 8'd9;
                    else if (prev_valid && !v_seq_gap && !v_tick_gap && (v_delta_abs > ENC_STEP_LIMIT))
                        r_reason <= 8'd7;
                    else if (prev_valid && !v_seq_gap && !v_tick_gap && (v_atime <= prev_atime))
                        r_reason <= 8'd9;
                    else if (prev_valid && !v_seq_gap && !v_tick_gap && (v_adc_abs > ADC_STEP_LIMIT))
                        r_reason <= 8'd8;
                    else begin
                        r_reject <= 1'b0;
                        r_reason <= 8'd0;
                        if (!prev_valid || v_seq_gap || v_tick_gap) begin
                            r_seg_next <= seg_count + 32'd1;
                            r_pos <= {{32{f_enc[31]}}, f_enc};
                        end else begin
                            r_pos <= prev_pos + {{32{v_delta[31]}}, v_delta};
                            state <= S_SCAN;
                        end
                    end
                end
                S_SCAN: begin
                    if (!enc_found && thr_e_ok && (scan_tick <= thr_e)) begin
                        enc_found <= 1'b1;
                        enc_start_t <= scan_tick;
                        enc_start_pos <= hist_pos[scan_idx];
                    end
                    if (!adc_found && thr_a_ok && (scan_atime <= thr_a)) begin
                        adc_found <= 1'b1;
                        adc_start_t <= scan_atime;
                        adc_start_v <= hist_adc[scan_idx];
                    end
                    scan_idx <= (scan_idx == {IDX_W{1'b0}}) ? LAST_IDX : scan_idx - 1'b1;
                    scan_k <= scan_k + 1'b1;
                    if (scan_k + 1'b1 == hist_count) begin
                        ch <= 1'b0;
                        state <= S_PREP1;
                    end
                end
                S_PREP1: begin
                    if (!(ch ? adc_found : enc_found)) begin
                        if (ch) state <= S_HOLD;
                        else ch <= 1'b1;
                    end else begin
                        p_span <= cur_t - start_t;
                        p_mid <= cur_t + start_t;
                        p_diff <= diff_now;
                        state <= S_PREP2;
                    end
                end
                S_PREP2: begin
                    p_age2 <= {1'b0, s_tick, 1'b0} - {2'b00, p_mid};
                    p_neg <= p_diff[64];
                    p_abs <= p_diff[64] ? (65'd0 - p_diff) : p_diff;
                    state <= S_PREP3;
                end
                S_PREP3: begin
                    if ((p_span[63:32] != 0) || p_age2[65] || (p_age2[64:32] != 0) ||
                        (p_abs[64:28] != 0)) begin
                        numeric_fail <= 1'b1;
                        state <= S_HOLD;
                    end else begin
                        p_product <= p_abs[27:0] * 20'd1000000;
                        div_start <= 1'b1;
                        if (ch) begin
                            a_span <= p_span[31:0]; a_mid <= p_mid; a_age2 <= p_age2[31:0];
                        end else begin
                            e_span <= p_span[31:0]; e_mid <= p_mid; e_age2 <= p_age2[31:0];
                        end
                        state <= S_DIV;
                    end
                end
                S_DIV: begin
                    if (div_done) begin
                        if (ch) a_q <= div_quotient;
                        else e_q <= div_quotient;
                        if (div_overflow) begin
                            numeric_fail <= 1'b1;
                            state <= S_HOLD;
                        end else if (ch) begin
                            state <= S_HOLD;
                        end else begin
                            ch <= 1'b1;
                            state <= S_PREP1;
                        end
                    end
                end
                S_HOLD: begin
                    if (emit_now) begin
                        state <= S_IDLE;
                        result_valid <= 1'b1;
                        result_payload <= s_payload;
                        result_tick_us <= s_tick;
                        segment_id <= r_seg_next;
                        if (emit_bad) begin
                            encoder_position <= 64'sd0;
                            adc_position_time_us <= 64'd0;
                            encoder_velocity_q16 <= 32'sd0; adc_velocity_q16 <= 32'sd0;
                            encoder_span_us <= 32'd0; adc_span_us <= 32'd0;
                            encoder_midpoint_x2_us <= 64'd0; adc_midpoint_x2_us <= 64'd0;
                            encoder_age_x2_us <= 32'd0; adc_age_x2_us <= 32'd0;
                            quality <= 3'd0;
                            reason <= numeric_fail ? 8'd10 : r_reason;
                            seg_count <= r_seg_next;
                            hist_count <= 0;
                        end else begin
                            encoder_position <= r_pos;
                            adc_position_time_us <= v_atime;
                            encoder_velocity_q16 <= enc_found ? e_q : 32'sd0;
                            encoder_span_us <= enc_found ? e_span : 32'd0;
                            encoder_midpoint_x2_us <= enc_found ? e_mid : 64'd0;
                            encoder_age_x2_us <= enc_found ? e_age2 : 32'd0;
                            adc_velocity_q16 <= adc_found ? a_q : 32'sd0;
                            adc_span_us <= adc_found ? a_span : 32'd0;
                            adc_midpoint_x2_us <= adc_found ? a_mid : 64'd0;
                            adc_age_x2_us <= adc_found ? a_age2 : 32'd0;
                            quality <= {adc_found, enc_found, 1'b1};
                            reason <= (r_boundary != 8'd0) ? r_boundary :
                                      (enc_found && adc_found) ? 8'd0 : 8'd1;
                            seg_count <= r_seg_next;
                            head <= head_after;
                            if (r_new_seg)
                                hist_count <= {{IDX_W{1'b0}}, 1'b1};
                            else if (hist_count != DEPTH_COUNT)
                                hist_count <= hist_count + {{IDX_W{1'b0}}, 1'b1};
                            prev_seq <= f_seq;
                            prev_enc <= f_enc;
                            prev_tick <= s_tick;
                            prev_atime <= v_atime;
                            prev_adc <= f_adc[9:0];
                            prev_pos <= r_pos;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase

            // A new snapshot always wins; an unfinished one is dropped uncommitted.
            if (sample_valid) begin
                s_overrun <= (state != S_IDLE) && !emit_now;
                s_payload <= sample_payload;
                s_tick <= sample_tick_us;
                lat_count <= 9'd0;
                enc_found <= 1'b0;
                adc_found <= 1'b0;
                numeric_fail <= 1'b0;
                ch <= 1'b0;
                div_start <= 1'b0;
                state <= S_VAL1;
            end
        end
    end
endmodule
