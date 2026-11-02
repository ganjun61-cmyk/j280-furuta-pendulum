`timescale 1ns/1ps

// PIP-lite balance core. Two 30 Hz second-order state-variable filters on the
// arm and pendulum angles supply filtered positions and rates. The feedback is
// 942's LQR gain with K0 x1.25 and all gains x0.75, plus an arm-angle integral
// with conditional anti-windup. No Kalman observer and no DOB.
//
// Native units: arm counts / counts/s, pendulum ADC codes / codes/s, Q16.16.
// Coefficients are Q20 compare per native unit; filter coefficients are Q16.
// One 32x32 signed multiplier is reused 17 times per 2 ms sample.
module j280_balance_pip_core #(
    parameter integer RESULT_LATENCY = 2048
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
    input wire signed [31:0] feedforward_q16,
    input wire [3:0] filter_mode,
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
    localparam integer DEADLINE = (RESULT_LATENCY < 64) ? 64 :
                                  ((RESULT_LATENCY > 4096) ? 4096 : RESULT_LATENCY);
    localparam [3:0] IDLE = 0, PROD = 1, MUL = 2, ACC = 3, PREP1 = 4, PREP2 = 5,
                     PREP3 = 6, FINAL1 = 7, FINAL1B = 8, FINAL2 = 9,
                     MSET = 10, ACC2 = 11, KFIX = 12;
    // Scaled 942 feedback (Q20): K0 x0.9375, K1..K3 x0.75.
    localparam signed [31:0] K0_Q20 = -32'sd2641474;
    localparam signed [31:0] K1_Q20 = -32'sd18963089;
    localparam signed [31:0] K2_Q20 = -32'sd1671271;
    localparam signed [31:0] K3_Q20 = -32'sd1953204;
    // 30 Hz, zeta=0.8, T=2 ms, discretized; Q16.
    localparam signed [31:0] AF00 = 32'sd61733;
    localparam signed [31:0] AF01 = 32'sd96;
    localparam signed [31:0] AF10 = -32'sd3415249;
    localparam signed [31:0] AF11 = 32'sd32743;
    localparam signed [31:0] BF0  = 32'sd3803;
    localparam signed [31:0] BF1  = 32'sd3415249;

    reg [3:0] stage;
    reg [4:0] idx;
    reg [15:0] elapsed;
    reg signed [31:0] m0, m1;
    reg [15:0] cfg_max, cfg_comp, cfg_blend;
    reg signed [31:0] cfg_gain, cfg_ff;
    reg signed [31:0] af_x1, af_x2, pf_x1, pf_x2;
    reg signed [63:0] acc_a1, acc_a2, acc_p1, acc_p2, acc_k, acc_c;
    reg signed [63:0] acc_sel;
    reg signed [31:0] acc_lo;
    reg acc_carry;
    reg signed [63:0] z_int;
    reg signed [31:0] uint_c_r;
    reg signed [63:0] unsat_r;
    reg z_ok, blend_r;
    reg signed [63:0] comp_sum_r;
    reg signed [31:0] mul_a_q, mul_b_q;
    reg signed [63:0] product;
    reg mul_sign, mul_half;
    reg [5:0] mul_bit;
    reg signed [63:0] mul_acc, mag_shift;
    reg signed [31:0] mag_a, mag_b, mul_lo;
    reg mul_carry;
    reg signed [31:0] raw_q16, linear_q16, linear_mag;
    reg signed [31:0] comp_q16, compensated_q16, model_q16;
    reg signed [15:0] cmd_q16;

    assign sample_ready = rst_n && !clear && !fault && !busy;
    wire signed [63:0] lim64 = $signed({32'd0, cfg_max, 16'd0});
    wire signed [31:0] comp_mag32 = $signed({16'd0, cfg_comp, 16'd0});
    wire mul_bit_on = (mul_bit < 6'd32) && mag_b[mul_bit];
    wire [32:0] mul_low_sum = {1'b0, mul_acc[31:0]} +
                              (mul_bit_on ? {1'b0, mag_shift[31:0]} : 33'd0);
    wire [32:0] mul_high_sum = {1'b0, mul_acc[63:32]} +
                               (mul_bit_on ? {1'b0, mag_shift[63:32]} : 33'd0) +
                               {32'd0, mul_carry};
    wire [32:0] acc_lo_w = {1'b0, acc_sel[31:0]} + {1'b0, product[31:0]};
    wire [32:0] acc_hi_w = {1'b0, acc_sel[63:32]} + {1'b0, product[63:32]} + {32'd0, acc_carry};
    wire signed [63:0] acc_new = {acc_hi_w[31:0], acc_lo};
    // acc_new >>> 20 is a 44-bit signed value; bits above [43] are its sign
    // copies, so a 13-bit fold decides int32 saturation without a 64-bit
    // compare chain (the ACC2 idx15 path is the tightest in the core).
    wire signed [63:0] k_shift = acc_k >>> 20;
    wire k_fits = (k_shift[43:31] == 13'h0000) ||
                  (k_shift[43:31] == 13'h1fff);
    wire signed [31:0] k_sat = k_fits ? k_shift[31:0] :
        (k_shift[43] ? 32'sh80000000 : 32'sh7fffffff);
    wire signed [31:0] uint_c = sat32(((z_int <<< 4) + (z_int <<< 2)) >>> 16);
    wire signed [63:0] unsat_w = {{32{raw_q16[31]}}, raw_q16} +
                                 $signed({{16{uint_c_r[31]}}, uint_c_r, 16'd0}) +
                                 {{32{cfg_ff[31]}}, cfg_ff};
    wire signed [31:0] lin_r_w = clip_control(unsat_r, lim64);
    wire signed [31:0] mag_r_w = lin_r_w[31] ? -lin_r_w : lin_r_w;
    wire blend_r_w = (mag_r_w >= {cfg_blend, 16'd0});
    wire signed [31:0] comp_w = blend_r ?
        (linear_q16[31] ? -comp_mag32 : comp_mag32) : sat32(acc_c >>> 20);
    wire signed [63:0] comp_sum_w = {{32{linear_q16[31]}}, linear_q16} +
                                    {{32{comp_w[31]}}, comp_w};
    wire signed [31:0] comp_clip_w = clip_control(comp_sum_r, lim64);

    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            sat32 = (v > 64'sd2147483647) ? 32'sh7fffffff :
                    (v < -64'sd2147483648) ? 32'sh80000000 : v[31:0];
        end
    endfunction
    function signed [31:0] clip_control;
        input signed [63:0] v;
        input signed [63:0] lim_in;
        begin
            clip_control = (v > lim_in) ? lim_in[31:0] :
                           (v < -lim_in) ? -lim_in[31:0] : v[31:0];
        end
    endfunction

    always @* begin
        case (idx)
            0, 1, 2: acc_sel = acc_a1;
            3, 4, 5: acc_sel = acc_a2;
            6, 7, 8: acc_sel = acc_p1;
            9, 10, 11: acc_sel = acc_p2;
            12, 13, 14, 15: acc_sel = acc_k;
            default: acc_sel = acc_c;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage <= IDLE; idx <= 0; elapsed <= 0;
            busy <= 0; result_valid <= 0; fault <= 0; overrun <= 0;
            m0 <= 0; m1 <= 0; cfg_max <= 0; cfg_comp <= 0; cfg_blend <= 0;
            cfg_gain <= 0; cfg_ff <= 0;
            af_x1 <= 0; af_x2 <= 0; pf_x1 <= 0; pf_x2 <= 0; z_int <= 0;
            uint_c_r <= 0; unsat_r <= 0; z_ok <= 0; blend_r <= 0; comp_sum_r <= 0;
            acc_a1 <= 0; acc_a2 <= 0; acc_p1 <= 0; acc_p2 <= 0; acc_k <= 0; acc_c <= 0;
            mul_a_q <= 0; mul_b_q <= 0; product <= 0;
            mul_sign <= 0; mul_half <= 0; mul_bit <= 0; mul_acc <= 0;
            mag_a <= 0; mag_b <= 0; mag_shift <= 0; mul_lo <= 0; mul_carry <= 0;
            raw_q16 <= 0; linear_q16 <= 0; linear_mag <= 0;
            comp_q16 <= 0; compensated_q16 <= 0; model_q16 <= 0; cmd_q16 <= 0;
            xhat0 <= 0; xhat1 <= 0; xhat2 <= 0; xhat3 <= 0;
            u_linear_q16 <= 0; u_compensated_q16 <= 0; u_model_q16 <= 0;
            signed_command <= 0; latency_cycles <= 0;
        end else begin
            result_valid <= 0;
            if (clear) begin
                stage <= IDLE; idx <= 0; busy <= 0; elapsed <= 0;
                af_x1 <= 0; af_x2 <= 0; pf_x1 <= 0; pf_x2 <= 0; z_int <= 0;
                fault <= 0; overrun <= 0;
            end else begin
                if (preset_valid && !busy) begin
                    af_x1 <= theta_q16; af_x2 <= 0;
                    pf_x1 <= alpha_q16; pf_x2 <= 0;
                    z_int <= 0;
                end
                if (busy && elapsed < DEADLINE) elapsed <= elapsed + 16'd1;
                if (busy && elapsed == DEADLINE) begin
                    fault <= 1; busy <= 0; stage <= IDLE;
                end
                case (stage)
                    IDLE: if (sample_valid) begin
                        if (busy) begin overrun <= 1; fault <= 1; end
                        else begin
                            busy <= 1; elapsed <= 0; idx <= 0;
                            m0 <= theta_q16; m1 <= alpha_q16;
                            cfg_max <= max_compare; cfg_comp <= comp_compare;
                            cfg_blend <= blend_compare; cfg_gain <= comp_gain_q20;
                            cfg_ff <= feedforward_q16;
                            acc_a1 <= 0; acc_a2 <= 0; acc_p1 <= 0; acc_p2 <= 0;
                            acc_k <= 0; acc_c <= 0;
                            stage <= PROD;
                        end
                    end
                    PROD: begin
                        case (idx)
                            0:  begin mul_a_q <= AF00; mul_b_q <= af_x1; end
                            1:  begin mul_a_q <= AF01; mul_b_q <= af_x2; end
                            2:  begin mul_a_q <= BF0;  mul_b_q <= m0;   end
                            3:  begin mul_a_q <= AF10; mul_b_q <= af_x1; end
                            4:  begin mul_a_q <= AF11; mul_b_q <= af_x2; end
                            5:  begin mul_a_q <= BF1;  mul_b_q <= m0;   end
                            6:  begin mul_a_q <= AF00; mul_b_q <= pf_x1; end
                            7:  begin mul_a_q <= AF01; mul_b_q <= pf_x2; end
                            8:  begin mul_a_q <= BF0;  mul_b_q <= m1;   end
                            9:  begin mul_a_q <= AF10; mul_b_q <= pf_x1; end
                            10: begin mul_a_q <= AF11; mul_b_q <= pf_x2; end
                            11: begin mul_a_q <= BF1;  mul_b_q <= m1;   end
                            12: begin mul_a_q <= K0_Q20; mul_b_q <= af_x1; end
                            13: begin mul_a_q <= K1_Q20; mul_b_q <= pf_x1; end
                            14: begin mul_a_q <= K2_Q20; mul_b_q <= af_x2; end
                            15: begin mul_a_q <= K3_Q20; mul_b_q <= pf_x2; end
                            16: begin mul_a_q <= linear_q16; mul_b_q <= cfg_gain; end
                        endcase
                        stage <= MSET;
                    end
                    MSET: begin
                        mul_sign <= mul_a_q[31] ^ mul_b_q[31];
                        mag_a <= mul_a_q[31] ? -mul_a_q : mul_a_q;
                        mag_b <= mul_b_q[31] ? -mul_b_q : mul_b_q;
                        mag_shift <= {32'd0, (mul_a_q[31] ? -mul_a_q : mul_a_q)};
                        mul_acc <= 0; mul_bit <= 0; mul_half <= 0;
                        mul_lo <= 0; mul_carry <= 0; stage <= MUL;
                    end
                    MUL: begin
                        if (mul_bit < 6'd32) begin
                            if (!mul_half) begin
                                mul_lo <= mul_low_sum[31:0];
                                mul_carry <= mul_low_sum[32];
                                mul_half <= 1;
                            end else begin
                                mul_acc <= {mul_high_sum[31:0], mul_lo};
                                mag_shift <= mag_shift << 1;
                                mul_bit <= mul_bit + 6'd1;
                                mul_half <= 0;
                            end
                        end else begin
                            product <= mul_sign ? -mul_acc : mul_acc;
                            stage <= ACC;
                        end
                    end
                    ACC: begin
                        acc_lo <= acc_lo_w[31:0];
                        acc_carry <= acc_lo_w[32];
                        stage <= ACC2;
                    end
                    ACC2: begin
                        case (idx)
                            0, 1, 2: begin acc_a1 <= acc_new; idx <= idx + 1; stage <= PROD; end
                            3, 4: begin acc_a2 <= acc_new; idx <= idx + 1; stage <= PROD; end
                            5: begin
                                acc_a2 <= acc_new;
                                af_x1 <= sat32(acc_a1 >>> 16);
                                af_x2 <= sat32(acc_new >>> 16);
                                acc_a1 <= 0; acc_a2 <= 0;
                                idx <= 6; stage <= PROD;
                            end
                            6, 7, 8: begin acc_p1 <= acc_new; idx <= idx + 1; stage <= PROD; end
                            9, 10: begin acc_p2 <= acc_new; idx <= idx + 1; stage <= PROD; end
                            11: begin
                                acc_p2 <= acc_new;
                                pf_x1 <= sat32(acc_p1 >>> 16);
                                pf_x2 <= sat32(acc_new >>> 16);
                                acc_p1 <= 0; acc_p2 <= 0; acc_k <= 0;
                                idx <= 12; stage <= PROD;
                            end
                            12, 13, 14: begin acc_k <= acc_new; idx <= idx + 1; stage <= PROD; end
                            15: begin
                                acc_k <= acc_new;
                                idx <= 16; stage <= KFIX;
                            end
                            default: begin acc_c <= acc_new; stage <= FINAL1; end
                        endcase
                    end
                    KFIX: begin
                        raw_q16 <= -k_sat;
                        acc_k <= 0;
                        stage <= PREP1;
                    end
                    PREP1: begin
                        uint_c_r <= uint_c;
                        stage <= PREP2;
                    end
                    PREP2: begin
                        unsat_r <= unsat_w;
                        z_ok <= (unsat_w <= lim64 && unsat_w >= -lim64);
                        stage <= PREP3;
                    end
                    PREP3: begin
                        linear_q16 <= lin_r_w;
                        linear_mag <= mag_r_w;
                        blend_r <= blend_r_w;
                        if (z_ok)
                            z_int <= z_int - (af_x1 >>> 16);
                        acc_c <= 0;
                        idx <= 16; stage <= PROD;
                    end
                    FINAL1: begin
                        comp_q16 <= comp_w;
                        comp_sum_r <= comp_sum_w;
                        stage <= FINAL1B;
                    end
                    FINAL1B: begin
                        compensated_q16 <= comp_clip_w;
                        stage <= FINAL2;
                    end
                    FINAL2: begin
                        model_q16 <= compensated_q16 - comp_q16;
                        u_linear_q16 <= linear_q16;
                        u_compensated_q16 <= compensated_q16;
                        u_model_q16 <= compensated_q16 - comp_q16;
                        signed_command <= compensated_q16[31] ?
                            -((-compensated_q16) >>> 16) : (compensated_q16 >>> 16);
                        xhat0 <= af_x1; xhat1 <= pf_x1; xhat2 <= af_x2; xhat3 <= pf_x2;
                        latency_cycles <= elapsed;
                        result_valid <= 1; busy <= 0; stage <= IDLE;
                    end
                    default: stage <= IDLE;
                endcase
            end
        end
    end
endmodule
