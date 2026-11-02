`timescale 1ns/1ps

// Q16.16 observer and PWM controller, coefficients in signed Q12.20.
// Word zero is coefficients[31:0]. Only the deadline branch publishes state.
// There is one 32x32 signed multiplier with registered operands, a registered
// 64-bit product, and a 72-bit accumulator. Prediction and bias-inclusive correction round separately.
// filter_mode selects an optional low-pass on the alpha-rate state before the
// feedback dot product: 0/1 bypass, 2..5 two cascaded one-pole sections at
// 3/5/8/10 Hz, 6/7 one section at 5/8 Hz. The filter runs only when enabled,
// so the bypass path is bit-exact with the unfiltered controller.
module j280_balance_core #(
    parameter integer RESULT_LATENCY = 256
) (
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire sample_valid,
    input wire signed [31:0] theta_q16,
    input wire signed [31:0] alpha_q16,
    input wire [1023:0] coefficients,
    input wire [15:0] max_compare,
    input wire [15:0] comp_compare,
    input wire [15:0] blend_compare,
    input wire signed [31:0] comp_gain_q20,
    // v6 feedforward (Q16 compare), latched with the sample: it seeds the
    // feedback accumulator (linear = clip(ff - K*xhat)) and is removed from the
    // next observer model input. |ff| > 1000<<16 is a configuration fault.
    input wire signed [31:0] feedforward_q16,
    // Controller mode: 1 bypass, 2..7 alpha-rate filter presets, 8..11 the same
    // controller with a 4/6/8/10-count dead zone on the arm-error feedback.
    input wire [3:0] filter_mode,
    // Algorithm 18: average this 20 Hz SVF rate with the Kalman alpha rate
    // before the K dot product (the observer and DOB stay unchanged).
    input wire signed [31:0] alpha_rate_ext_q16,
    input wire alpha_rate_mix,
    input wire signed [31:0] arm_rate_ext_q16,
    input wire arm_rate_mix,
    // Observer preset (swing-up catch): loads xhat and zeroes the model input
    // while idle; the next sample predicts from it.
    input wire preset_valid,
    input wire signed [31:0] preset_x0,
    input wire signed [31:0] preset_x1,
    input wire signed [31:0] preset_x2,
    input wire signed [31:0] preset_x3,
    output reg busy,
    output wire sample_ready,
    output reg result_valid,
    output reg fault,
    output reg overrun,
    output reg signed [31:0] xhat0,
    output reg signed [31:0] xhat1,
    output reg signed [31:0] xhat2,
    output reg signed [31:0] xhat3,
    output reg signed [31:0] u_linear_q16,
    output reg signed [31:0] u_compensated_q16,
    output reg signed [31:0] u_model_q16,
    output reg signed [15:0] signed_command,
    output reg [15:0] latency_cycles
);
    // Invalid latency parameters cannot create an unbounded or wrapping wait.
    // A deadline shorter than the datapath faults instead of publishing partials.
    localparam integer DEADLINE = (RESULT_LATENCY < 1) ? 1 :
                                  ((RESULT_LATENCY > 500) ? 500 : RESULT_LATENCY);
    localparam [5:0] IDLE = 0, CHECK_MUL = 1, CHECK_SLOPE = 2,
                     MUL = 3, ACCUMULATE = 4, ROUND = 5, ERRORS = 6,
                     COMPENSATE = 7, COMMAND = 8, MODEL = 9,
                     DIV_START = 10, DIV_WAIT = 11, WAIT_COMMIT = 12,
                     CHECK_PRODUCT = 13, PRODUCT = 14, TRUNC = 15,
                     FD1 = 16, FM1 = 17, FP1 = 18, FU1 = 19,
                     FM2 = 20, FP2 = 21, FU2 = 22, DB = 23,
                     DOB_M1 = 24, DOB_M2 = 25, DOB_C1 = 26, DOB_C2 = 27,
                     DOB_M3 = 28, DOB_U1 = 29, DOB_M4 = 30, DOB_K1 = 31,
                     DOB_M5 = 32, DOB_L1 = 33, DOB_C2B = 34;
    localparam [1:0] PREDICT = 0, CORRECT = 1, FEEDBACK = 2, BLEND = 3;
    localparam signed [31:0] MODEL_B_Q16 = 32'd1109222;
    localparam [11:0] DOB_K_Q16 = 12'd3872;
    reg [5:0] stage;
    reg [1:0] phase, row;
    reg [2:0] term;
    reg [15:0] elapsed;
    reg candidate_ready, candidate_fault;
    reg [1023:0] coeff_snapshot;
    reg signed [31:0] measurement0, measurement1;
    reg [15:0] cfg_max, cfg_comp, cfg_blend;
    reg signed [31:0] cfg_gain, cfg_ff;
    reg signed [31:0] previous_model_input;
    reg signed [31:0] pred [0:3];
    reg signed [31:0] candidate [0:3];
    reg signed [31:0] error0, error1;
    reg signed [31:0] linear_work, compensated_work, model_work;
    reg signed [15:0] command_work;
    reg [15:0] command_mag_q;      // magnitude latched one cycle before the divider
    reg command_neg_q;
    reg signed [31:0] mul_a, mul_b;
    // Operand registers keep the operand mux out of the multiplier timing path.
    reg signed [31:0] op_a, op_b;
    reg signed [63:0] product;
    reg signed [71:0] accumulator;
    // Alpha-rate filter: two cascaded y += alpha * (x - y) sections, Q16 alpha.
    reg [15:0] cfg_filter_alpha;
    reg cfg_filter_two;
    reg signed [31:0] filter_y1, filter_y2, filter_d1;
    reg [15:0] filter_alpha_sel;
    reg filter_two_sel;
    reg [4:0] cfg_db_counts;
    reg [4:0] db_counts_sel;
    reg signed [31:0] x0_db;
    // Arm-side disturbance observer: d_hat has the units of the arm
    // acceleration (Q16.16 counts/s^2); dob_v is the previous arm velocity.
    reg [15:0] cfg_dob_alpha;
    reg [17:0] cfg_dob_gain;
    reg cfg_dob_on;
    reg [15:0] dob_alpha_sel;
    reg [17:0] dob_gain_sel;
    reg dob_on_sel;
    reg signed [31:0] dob_d, dob_v, dob_diff;
    reg signed [63:0] dob_model64, dob_acc64, dob_raw64;

    // Preset table, fs = 500 Hz: alpha = round((1 - exp(-2*pi*fc/fs)) * 65536).
    always @* begin
        filter_alpha_sel = 16'd0;
        filter_two_sel = 1'b1;
        db_counts_sel = 5'd0;
        dob_alpha_sel = 16'd0;
        dob_gain_sel = 18'd0;
        dob_on_sel = 1'b0;
        case (filter_mode)
            4'd2: filter_alpha_sel = 16'd2425;   // 3 Hz, two sections
            4'd3: filter_alpha_sel = 16'd3991;   // 5 Hz, two sections
            4'd4: filter_alpha_sel = 16'd6268;   // 8 Hz, two sections
            4'd5: filter_alpha_sel = 16'd7739;   // 10 Hz, two sections
            4'd6: begin filter_alpha_sel = 16'd3991; filter_two_sel = 1'b0; end
            4'd7: begin filter_alpha_sel = 16'd6268; filter_two_sel = 1'b0; end
            4'd8: db_counts_sel = 5'd4;
            4'd9: db_counts_sel = 5'd6;
            4'd10: db_counts_sel = 5'd8;
            4'd11: db_counts_sel = 5'd10;
            4'd12: begin dob_alpha_sel = 16'd3991; dob_gain_sel = 18'd65536; dob_on_sel = 1'b1; end
            4'd13: begin dob_alpha_sel = 16'd7739; dob_gain_sel = 18'd65536; dob_on_sel = 1'b1; end
            4'd14: begin dob_alpha_sel = 16'd7739; dob_gain_sel = 18'd98304; dob_on_sel = 1'b1; end
            4'd15: begin dob_alpha_sel = 16'd7739; dob_gain_sel = 18'd98304; dob_on_sel = 1'b1; end
            default: begin filter_alpha_sel = 16'd0; filter_two_sel = 1'b1; end
        endcase
    end

    wire db_on = cfg_db_counts != 5'd0;
    wire signed [32:0] db_band = {12'd0, cfg_db_counts, 16'd0};
    wire signed [32:0] x0_wide = {candidate[0][31], candidate[0]};
    wire signed [32:0] x0_db_wide = (x0_wide > db_band) ? x0_wide - db_band :
                                    (x0_wide < -db_band) ? x0_wide + db_band : 33'sd0;

    wire filter_on = cfg_filter_alpha != 16'd0;
    // Round to nearest with ties toward +infinity, then saturate the state add.
    wire signed [63:0] filter_inc = (product + 64'sd32768) >>> 16;
    wire signed [33:0] filter_d2_wide = {{2{filter_y1[31]}}, filter_y1} -
                                       {{2{filter_y2[31]}}, filter_y2};

    function signed [31:0] clamp32;
        input signed [33:0] value;
        begin
            if (value > 34'sd2147483647) clamp32 = 32'sh7fffffff;
            else if (value < -34'sd2147483648) clamp32 = 32'sh80000000;
            else clamp32 = value[31:0];
        end
    endfunction

    function signed [31:0] clamp64;
        input signed [63:0] value;
        begin
            if (value > 64'sd2147483647) clamp64 = 32'sh7fffffff;
            else if (value < -64'sd2147483648) clamp64 = 32'sh80000000;
            else clamp64 = value[31:0];
        end
    endfunction

    function signed [31:0] sat_add32;
        input signed [31:0] a, b;
        reg signed [32:0] sum;
        begin
            sum = {a[31], a} + {b[31], b};
            if (sum > 33'sd2147483647) sat_add32 = 32'sh7fffffff;
            else if (sum < -33'sd2147483648) sat_add32 = 32'sh80000000;
            else sat_add32 = sum[31:0];
        end
    endfunction

    function signed [71:0] trunc_q20;
        input signed [71:0] value;
        begin
            trunc_q20 = (value < 0) ? -((-value) >>> 20) : (value >>> 20);
        end
    endfunction

    function signed [31:0] clip_control;
        input signed [71:0] value;
        input [15:0] maximum;
        reg signed [71:0] bound;
        begin
            bound = {40'd0, maximum, 16'd0};
            if (value > bound) clip_control = bound[31:0];
            else if (value < -bound) clip_control = -bound;
            else clip_control = value[31:0];
        end
    endfunction

    // Disturbance-observer arithmetic in native units (Q16.16 counts/s^2).
    wire signed [63:0] dob_x64 = {{32{candidate[2][31]}}, candidate[2]};
    wire signed [63:0] dob_v64 = {{32{dob_v[31]}}, dob_v};
    wire signed [63:0] dob_dv64 = dob_x64 - dob_v64;
    wire signed [63:0] dob_acc_w = (dob_dv64 <<< 9) - (dob_dv64 <<< 3) - (dob_dv64 <<< 2);
    wire signed [63:0] dob_x25 = (dob_x64 <<< 4) + (dob_x64 <<< 3) + dob_x64;
    wire signed [63:0] dob_model_w = -(dob_x25 >>> 1) + (product >>> 16);
    wire signed [31:0] dob_raw32 = clamp64(dob_raw64);
    wire signed [63:0] dob_diff64 = {{32{dob_raw32[31]}}, dob_raw32} -
                                    {{32{dob_d[31]}}, dob_d};
    wire signed [31:0] dob_diff_w = clamp64(dob_diff64);
    wire signed [31:0] dob_shifted = clamp64(product >>> 16);
    wire signed [31:0] dob_d_next = sat_add32(dob_d, dob_shifted);

    wire deadline_now = busy && (elapsed == DEADLINE - 1);
    wire normal_commit = deadline_now && candidate_ready && !candidate_fault && !fault;
    wire reentry = sample_valid && busy && !normal_commit && !fault;
    assign sample_ready = rst_n && !clear && !fault && (!busy || normal_commit);

    wire signed [71:0] rounded = trunc_q20(accumulator);
    // ROUND consumes the truncation registered one clock earlier (timing only).
    reg signed [71:0] rounded_q;
    wire signed [32:0] error0_wide = {measurement0[31], measurement0} -
                                    {pred[0][31], pred[0]};
    wire signed [32:0] error1_wide = {measurement1[31], measurement1} -
                                    {pred[1][31], pred[1]};
    wire [31:0] linear_magnitude = linear_work[31] ? -linear_work : linear_work;
    wire signed [71:0] linear_wide = {{40{linear_work[31]}}, linear_work};
    wire signed [71:0] compensation_offset = {40'd0, cfg_comp, 16'd0};
    wire [15:0] command_magnitude = command_work[15] ? -command_work : command_work;
    wire [16:0] inverse_boundary = {1'b0, cfg_comp} + {1'b0, cfg_blend};
    wire [31:0] outer_model = {command_magnitude - cfg_comp, 16'd0};
    wire signed [71:0] slope_error = {{8{product[63]}}, product} -
                                     {36'd0, cfg_comp, 20'd0};
    wire signed [71:0] slope_tolerance = ({56'd0, cfg_blend} + 72'd1) >> 1;
    wire [127:0] candidate_words = {candidate[3], candidate[2], candidate[1], candidate[0]};

    // Magnitude input and a separate sign reuse the original signed divider.
    wire divider_done, divider_overflow;
    wire signed [31:0] divider_quotient;
    wire divider_start = busy && (stage == DIV_START) && !reentry && !clear && !fault;
    wire [63:0] divider_numerator = {12'd0, command_mag_q, 36'd0};
    wire [31:0] divider_denominator = 32'd1048576 + cfg_gain;
    j280_signed_divider #(.NUM_WIDTH(64), .DEN_WIDTH(32)) inverse_divider (
        .clk(clk), .rst_n(rst_n), .clear(clear || reentry || fault),
        .start(divider_start), .negative(command_neg_q),
        .magnitude(divider_numerator), .divisor(divider_denominator),
        .done(divider_done), .quotient(divider_quotient), .overflow(divider_overflow)
    );

    // Algorithm 18 mixes the Kalman alpha rate with the top's 20 Hz SVF rate.
    wire signed [32:0] rate_mix_sum = {candidate[3][31], candidate[3]} +
                                      {alpha_rate_ext_q16[31], alpha_rate_ext_q16};
    wire signed [31:0] rate_mix = rate_mix_sum[32:1];
    wire signed [32:0] arm_mix_sum = {candidate[2][31], candidate[2]} +
                                     {arm_rate_ext_q16[31], arm_rate_ext_q16};
    wire signed [31:0] arm_mix = arm_mix_sum[32:1];

    // All arithmetic products, including the configuration slope check, share
    // this operand mux. Accumulation always occurs on the following clock.
    always @* begin
        mul_a = 32'sd0;
        mul_b = 32'sd0;
        if (stage == CHECK_MUL) begin
            mul_a = cfg_gain;
            mul_b = {16'd0, cfg_blend};
        end else begin
            case (phase)
                PREDICT: begin
                    if (term == 4) begin
                        mul_a = coeff_snapshot[((16 + row) << 5) +: 32];
                        mul_b = previous_model_input;
                    end else begin
                        mul_a = coeff_snapshot[{row, term[1:0], 5'd0} +: 32];
                        case (term)
                            0: mul_b = xhat0;
                            1: mul_b = xhat1;
                            2: mul_b = xhat2;
                            default: mul_b = xhat3;
                        endcase
                    end
                end
                CORRECT: begin
                    mul_a = coeff_snapshot[((20 + {row, 1'b0} + term) << 5) +: 32];
                    mul_b = term[0] ? error1 : error0;
                end
                FEEDBACK: begin
                    mul_a = coeff_snapshot[((28 + term) << 5) +: 32];
                    if (term == 0 && db_on)
                        mul_b = x0_db;
                    else if (term == 2 && arm_rate_mix)
                        mul_b = arm_mix;
                    else if (term == 3 && alpha_rate_mix)
                        mul_b = rate_mix;
                    else if (term == 3 && filter_on)
                        mul_b = cfg_filter_two ? filter_y2 : filter_y1;
                    else
                        mul_b = candidate_words[{term[1:0], 5'd0} +: 32];
                end
                BLEND: begin
                    mul_a = linear_work;
                    mul_b = cfg_gain;
                end
            endcase
        end
    end

    integer i, c;
    reg bad_configuration;
    always @* begin
        bad_configuration = (max_compare < 1 || max_compare > 1125 ||
                             comp_compare > 250 || comp_compare > max_compare ||
                             blend_compare < 25 || blend_compare > 250 ||
                             comp_gain_q20 < 0 || comp_gain_q20 > 32'sd10485760 ||
                             feedforward_q16 > 32'sd73728000 || feedforward_q16 < -32'sd73728000 ||
                             RESULT_LATENCY < 1 || RESULT_LATENCY > 500);
        for (c = 0; c < 32; c = c + 1) begin
            if ($signed(coefficients[(c << 5) +: 32]) > 32'sd134217728 ||
                $signed(coefficients[(c << 5) +: 32]) < -32'sd134217728)
                bad_configuration = 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; result_valid <= 0; fault <= 0; overrun <= 0;
            xhat0 <= 0; xhat1 <= 0; xhat2 <= 0; xhat3 <= 0;
            u_linear_q16 <= 0; u_compensated_q16 <= 0; u_model_q16 <= 0;
            signed_command <= 0; latency_cycles <= 0;
            previous_model_input <= 0;
            stage <= IDLE; phase <= PREDICT; row <= 0; term <= 0; elapsed <= 0;
            candidate_ready <= 0; candidate_fault <= 0;
            measurement0 <= 0; measurement1 <= 0;
            cfg_max <= 0; cfg_comp <= 0; cfg_blend <= 0; cfg_gain <= 0; cfg_ff <= 0;
            error0 <= 0; error1 <= 0;
            linear_work <= 0; compensated_work <= 0; model_work <= 0; command_work <= 0;
            command_mag_q <= 0; command_neg_q <= 0;
            product <= 0; accumulator <= 0; op_a <= 0; op_b <= 0; rounded_q <= 0;
            coeff_snapshot <= 0;
            cfg_filter_alpha <= 0; cfg_filter_two <= 0;
            filter_y1 <= 0; filter_y2 <= 0; filter_d1 <= 0;
            cfg_db_counts <= 0; x0_db <= 0;
            cfg_dob_alpha <= 0; cfg_dob_gain <= 0; cfg_dob_on <= 0;
            dob_d <= 0; dob_v <= 0; dob_diff <= 0; dob_model64 <= 0; dob_acc64 <= 0;
            dob_raw64 <= 0;
            for (i = 0; i < 4; i = i + 1) begin pred[i] <= 0; candidate[i] <= 0; end
        end else if (clear) begin
            busy <= 0; result_valid <= 0; fault <= 0; overrun <= 0;
            xhat0 <= 0; xhat1 <= 0; xhat2 <= 0; xhat3 <= 0;
            u_linear_q16 <= 0; u_compensated_q16 <= 0; u_model_q16 <= 0;
            signed_command <= 0; latency_cycles <= 0;
            previous_model_input <= 0;
            stage <= IDLE; phase <= PREDICT; row <= 0; term <= 0; elapsed <= 0;
            candidate_ready <= 0; candidate_fault <= 0;
            measurement0 <= 0; measurement1 <= 0;
            cfg_max <= 0; cfg_comp <= 0; cfg_blend <= 0; cfg_gain <= 0; cfg_ff <= 0;
            error0 <= 0; error1 <= 0;
            linear_work <= 0; compensated_work <= 0; model_work <= 0; command_work <= 0;
            command_mag_q <= 0; command_neg_q <= 0;
            product <= 0; accumulator <= 0; op_a <= 0; op_b <= 0; rounded_q <= 0;
            coeff_snapshot <= 0;
            cfg_filter_alpha <= 0; cfg_filter_two <= 0;
            filter_y1 <= 0; filter_y2 <= 0; filter_d1 <= 0;
            cfg_db_counts <= 0; x0_db <= 0;
            cfg_dob_alpha <= 0; cfg_dob_gain <= 0; cfg_dob_on <= 0;
            dob_d <= 0; dob_v <= 0; dob_diff <= 0; dob_model64 <= 0; dob_acc64 <= 0;
            dob_raw64 <= 0;
            for (i = 0; i < 4; i = i + 1) begin pred[i] <= 0; candidate[i] <= 0; end
        end else begin
            result_valid <= 0;
            if (preset_valid && !busy && !fault) begin
                xhat0 <= preset_x0; xhat1 <= preset_x1;
                xhat2 <= preset_x2; xhat3 <= preset_x3;
                previous_model_input <= 0;
                filter_y1 <= preset_x3; filter_y2 <= preset_x3;
                dob_d <= 0; dob_v <= preset_x2;
            end
            if (fault) begin
                busy <= 0;
            end else if (reentry) begin
                // A busy submission is an error even when sample_ready is low.
                // It cancels the candidate before any state can be committed.
                busy <= 0; fault <= 1; overrun <= 1; result_valid <= 1;
                u_linear_q16 <= 0; u_compensated_q16 <= 0; u_model_q16 <= 0;
                signed_command <= 0; latency_cycles <= elapsed + 16'd1;
                candidate_ready <= 0; candidate_fault <= 1; stage <= IDLE;
            end else begin
                // A single product assignment guarantees one shared multiplier
                // for slope validation, observer MACs, feedback and blending.
                if (busy && !deadline_now && (stage == CHECK_MUL || stage == MUL)) begin
                    op_a <= mul_a; op_b <= mul_b;
                end
                if (busy && !deadline_now && (stage == CHECK_PRODUCT || stage == PRODUCT ||
                                              stage == FP1 || stage == FP2 ||
                                              stage == DOB_M2 || stage == DOB_M3 ||
                                              stage == DOB_M4 || stage == DOB_M5))
                    product <= op_a * op_b;
                if (busy) begin
                    elapsed <= elapsed + 16'd1;
                    if (deadline_now) begin
                        busy <= 0; result_valid <= 1; latency_cycles <= DEADLINE;
                        stage <= IDLE;
                        if (normal_commit) begin
                            xhat0 <= candidate[0]; xhat1 <= candidate[1];
                            xhat2 <= candidate[2]; xhat3 <= candidate[3];
                            u_linear_q16 <= linear_work;
                            u_compensated_q16 <= compensated_work;
                            u_model_q16 <= model_work;
                            signed_command <= command_work;
                            previous_model_input <= model_work - cfg_ff;
                        end else begin
                            fault <= 1;
                            u_linear_q16 <= 0; u_compensated_q16 <= 0; u_model_q16 <= 0;
                            signed_command <= 0;
                        end
                    end else begin
                        case (stage)
                            CHECK_MUL: begin
                                stage <= CHECK_PRODUCT;
                            end
                            CHECK_PRODUCT: begin
                                stage <= CHECK_SLOPE;
                            end
                            CHECK_SLOPE: begin
                                if (slope_error > slope_tolerance || slope_error < -slope_tolerance) begin
                                    candidate_fault <= 1; stage <= WAIT_COMMIT;
                                end else begin
                                    phase <= PREDICT; row <= 0; term <= 0;
                                    accumulator <= 0; stage <= MUL;
                                end
                            end
                            MUL: begin
                                stage <= PRODUCT;
                            end
                            PRODUCT: begin
                                stage <= ACCUMULATE;
                            end
                            ACCUMULATE: begin
                                accumulator <= accumulator + {{8{product[63]}}, product};
                                if ((phase == PREDICT && term == 4) ||
                                    (phase == CORRECT && term == 1) ||
                                    (phase == FEEDBACK && term == 3) || phase == BLEND)
                                    stage <= TRUNC;
                                else begin term <= term + 3'd1; stage <= MUL; end
                            end
                            TRUNC: begin
                                rounded_q <= rounded;
                                stage <= ROUND;
                            end
                            ROUND: begin
                                if (phase == PREDICT || phase == CORRECT) begin
                                    if (rounded_q > 72'sd2147483647 || rounded_q < -72'sd2147483648) begin
                                        candidate_fault <= 1; stage <= WAIT_COMMIT;
                                    end else if (phase == PREDICT) begin
                                        pred[row] <= rounded_q[31:0];
                                        if (row == 3) stage <= ERRORS;
                                        else begin
                                            row <= row + 2'd1; term <= 0;
                                            accumulator <= 0; stage <= MUL;
                                        end
                                    end else begin
                                        candidate[row] <= rounded_q[31:0];
                                        term <= 0;
                                        if (row == 3) begin
                                            phase <= FEEDBACK;
                                            if (filter_on) stage <= FD1;
                                            else if (db_on) stage <= DB;
                                            else begin
                                                accumulator <= -({{40{cfg_ff[31]}}, cfg_ff} <<< 20);
                                                stage <= MUL;
                                            end
                                        end else begin
                                            row <= row + 2'd1;
                                            accumulator <= {{40{pred[row+2'd1][31]}}, pred[row+2'd1]} <<< 20;
                                            stage <= MUL;
                                        end
                                    end
                                end else if (phase == FEEDBACK) begin
                                    // The wide feedback can exceed int32 legitimately.
                                    if (cfg_dob_on) stage <= DOB_M1;
                                    else begin
                                        linear_work <= clip_control(-rounded_q, cfg_max);
                                        stage <= COMPENSATE;
                                    end
                                end else begin
                                    compensated_work <= clip_control(linear_wide + rounded_q, cfg_max);
                                    stage <= COMMAND;
                                end
                            end
                            ERRORS: begin
                                if (error0_wide[32] != error0_wide[31] ||
                                    error1_wide[32] != error1_wide[31]) begin
                                    candidate_fault <= 1; stage <= WAIT_COMMIT;
                                end else begin
                                    error0 <= error0_wide[31:0]; error1 <= error1_wide[31:0];
                                    phase <= CORRECT; row <= 0; term <= 0;
                                    accumulator <= {{40{pred[0][31]}}, pred[0]} <<< 20;
                                    stage <= MUL;
                                end
                            end
                            COMPENSATE: begin
                                if (linear_magnitude >= {cfg_blend, 16'd0}) begin
                                    compensated_work <= clip_control(linear_work[31] ?
                                        linear_wide - compensation_offset :
                                        linear_wide + compensation_offset, cfg_max);
                                    stage <= COMMAND;
                                end else begin
                                    phase <= BLEND; term <= 0; accumulator <= 0; stage <= MUL;
                                end
                            end
                            COMMAND: begin
                                command_work <= compensated_work[31] ?
                                    -((-compensated_work) >>> 16) : (compensated_work >>> 16);
                                stage <= MODEL;
                            end
                            MODEL: begin
                                command_mag_q <= command_magnitude;
                                command_neg_q <= command_work[15];
                                if ({1'b0, command_magnitude} >= inverse_boundary) begin
                                    model_work <= command_work[15] ? -outer_model : outer_model;
                                    candidate_ready <= 1; stage <= WAIT_COMMIT;
                                end else stage <= DIV_START;
                            end
                            DIV_START: stage <= DIV_WAIT;
                            DIV_WAIT: begin
                                if (divider_done) begin
                                    model_work <= divider_quotient;
                                    candidate_fault <= divider_overflow;
                                    candidate_ready <= !divider_overflow;
                                    stage <= WAIT_COMMIT;
                                end
                            end
                            FD1: begin
                                filter_d1 <= clamp32({{2{candidate[3][31]}}, candidate[3]} -
                                                     {{2{filter_y1[31]}}, filter_y1});
                                stage <= FM1;
                            end
                            FM1: begin
                                op_a <= filter_d1;
                                op_b <= {16'd0, cfg_filter_alpha};
                                stage <= FP1;
                            end
                            FP1: stage <= FU1;
                            FU1: begin
                                filter_y1 <= sat_add32(filter_y1, filter_inc[31:0]);
                                if (cfg_filter_two) stage <= FM2;
                                else begin
                                    phase <= FEEDBACK; term <= 0;
                                    accumulator <= -({{40{cfg_ff[31]}}, cfg_ff} <<< 20);
                                    stage <= MUL;
                                end
                            end
                            FM2: begin
                                op_a <= clamp32(filter_d2_wide);
                                op_b <= {16'd0, cfg_filter_alpha};
                                stage <= FP2;
                            end
                            FP2: stage <= FU2;
                            FU2: begin
                                filter_y2 <= sat_add32(filter_y2, filter_inc[31:0]);
                                phase <= FEEDBACK; term <= 0;
                                accumulator <= -({{40{cfg_ff[31]}}, cfg_ff} <<< 20);
                                stage <= MUL;
                            end
                            DB: begin
                                x0_db <= x0_db_wide[31:0];
                                term <= 0;
                                accumulator <= -({{40{cfg_ff[31]}}, cfg_ff} <<< 20);
                                stage <= MUL;
                            end
                            DOB_M1: begin
                                op_a <= previous_model_input;
                                op_b <= MODEL_B_Q16;
                                stage <= DOB_M2;
                            end
                            DOB_M2: stage <= DOB_C1;
                            DOB_C1: begin
                                dob_model64 <= dob_model_w;
                                dob_acc64 <= dob_acc_w;
                                stage <= DOB_C2;
                            end
                            DOB_C2: begin
                                dob_raw64 <= dob_acc64 - dob_model64;
                                stage <= DOB_C2B;
                            end
                            DOB_C2B: begin
                                dob_diff <= dob_diff_w;
                                op_a <= dob_diff_w;
                                op_b <= {16'd0, cfg_dob_alpha};
                                stage <= DOB_M3;
                            end
                            DOB_M3: stage <= DOB_U1;
                            DOB_U1: begin
                                dob_d <= dob_d_next;
                                dob_v <= candidate[2];
                                op_a <= dob_d_next;
                                op_b <= {20'd0, DOB_K_Q16};
                                stage <= DOB_M4;
                            end
                            DOB_M4: stage <= DOB_K1;
                            DOB_K1: begin
                                op_a <= dob_shifted;
                                op_b <= {14'd0, cfg_dob_gain};
                                stage <= DOB_M5;
                            end
                            DOB_M5: stage <= DOB_L1;
                            DOB_L1: begin
                                linear_work <= clip_control(-rounded_q -
                                    {{8{product[63]}}, product >>> 16}, cfg_max);
                                stage <= COMPENSATE;
                            end
                            WAIT_COMMIT: begin end
                            default: begin candidate_fault <= 1; stage <= WAIT_COMMIT; end
                        endcase
                    end
                end
                // This branch is deliberately after commit. Nonblocking writes
                // publish the old candidate and snapshot the next input together;
                // its later prediction reads the newly committed xhat/u_prev.
                if (sample_valid && (!busy || normal_commit)) begin
                    busy <= 1; elapsed <= 0;
                    measurement0 <= theta_q16; measurement1 <= alpha_q16;
                    cfg_max <= max_compare; cfg_comp <= comp_compare;
                    cfg_blend <= blend_compare; cfg_gain <= comp_gain_q20; cfg_ff <= feedforward_q16;
                    coeff_snapshot <= coefficients;
                    cfg_filter_alpha <= filter_alpha_sel; cfg_filter_two <= filter_two_sel;
                    cfg_db_counts <= db_counts_sel;
                    cfg_dob_alpha <= dob_alpha_sel;
                    cfg_dob_gain <= dob_gain_sel;
                    cfg_dob_on <= dob_on_sel;
                    candidate_ready <= 0; candidate_fault <= bad_configuration;
                    stage <= bad_configuration ? WAIT_COMMIT : CHECK_MUL;
                end
            end
        end
    end
endmodule
