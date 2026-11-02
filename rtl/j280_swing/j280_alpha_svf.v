`timescale 1ns/1ps

// 20 Hz second-order state-variable filter on the pendulum angle measurement.
// x1 is the low-passed angle and x2 its rate, in native Q16.16 ADC codes and
// codes/s. The 500 Hz ZOH coefficients are folded into four constants:
//   x1' = x1 + (1809*(u - x1) + 107*x2) >>> 16
//   x2' = (42255*x2 + 1686407*(u - x1)) >>> 16
// rate_q16 is the PREVIOUS x2: the balance core mixes it with its Kalman rate
// for the same sample, and this block updates the states in the background.
// arm_rate_q16 feeds a 2 Hz one-pole low-pass (arm_rate_lp_q16, also the
// pre-update value) for the slow arm-rate damping path. One 32x32
// sign-magnitude multiplier is reused five times per sample.
module j280_alpha_svf (
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire start,
    input wire signed [31:0] alpha_q16,
    input wire signed [31:0] arm_rate_q16,
    output reg signed [31:0] rate_q16,
    output reg signed [31:0] arm_rate_lp_q16,
    output reg busy
);
    localparam signed [31:0] C_X1_U = 32'sd1809;
    localparam signed [31:0] C_X1_X = 32'sd107;
    localparam signed [31:0] C_X2_X = 32'sd42255;
    localparam signed [31:0] C_X2_U = 32'sd1686407;
    localparam signed [31:0] C_ARM_LP = 32'sd1627;   // 2 Hz, Q16 alpha
    localparam [2:0] IDLE = 0, MUL_LOAD = 1, MUL_WAIT = 2, ACC = 3, FINISH = 4;

    reg signed [31:0] x1, x2;
    reg signed [31:0] arm_lp, d_arm_lat;
    reg primed;
    reg [2:0] st, which;
    reg signed [31:0] d_lat, x1_lat, x2_lat;
    reg signed [63:0] acc1, acc2, acc3;

    // Serial sign-magnitude multiplier (same shape as the PIP core's).
    reg signed [31:0] mag_a, mag_b, mul_lo;
    reg signed [63:0] product, mul_acc, mag_shift;
    reg signed [31:0] ld_a, ld_b;
    reg mul_sign, mul_half, mul_carry, mul_done;
    reg [5:0] mul_bit;
    wire mul_bit_on = (mul_bit < 6'd32) && mag_b[mul_bit];
    wire [32:0] mul_low_sum = {1'b0, mul_acc[31:0]} +
                              (mul_bit_on ? {1'b0, mag_shift[31:0]} : 33'd0);
    wire [32:0] mul_high_sum = {1'b0, mul_acc[63:32]} +
                               (mul_bit_on ? {1'b0, mag_shift[63:32]} : 33'd0) +
                               {32'd0, mul_carry};

    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            sat32 = (v > 64'sd2147483647) ? 32'sh7fffffff :
                    (v < -64'sd2147483648) ? 32'sh80000000 : v[31:0];
        end
    endfunction

    wire signed [32:0] d_wide = {alpha_q16[31], alpha_q16} - {x1[31], x1};
    wire signed [31:0] d_sat = (d_wide > 33'sd2147483647) ? 32'sh7fffffff :
                               (d_wide < -33'sd2147483648) ? 32'sh80000000 : d_wide[31:0];
    wire signed [63:0] x1_shift = acc1 >>> 16;
    wire signed [63:0] x2_shift = acc2 >>> 16;
    wire signed [63:0] arm_shift = acc3 >>> 16;
    wire signed [32:0] x1_sum = {x1_lat[31], x1_lat} + {x1_shift[31], x1_shift[31:0]};
    wire signed [32:0] d_arm_wide = {arm_rate_q16[31], arm_rate_q16} - {arm_lp[31], arm_lp};
    wire signed [31:0] d_arm_sat = (d_arm_wide > 33'sd2147483647) ? 32'sh7fffffff :
                                   (d_arm_wide < -33'sd2147483648) ? 32'sh80000000 : d_arm_wide[31:0];
    wire signed [32:0] arm_sum = {arm_lp[31], arm_lp} + {arm_shift[31], arm_shift[31:0]};

    always @* begin
        case (which)
            0: begin ld_a = C_X1_U; ld_b = d_lat; end
            1: begin ld_a = C_X1_X; ld_b = x2_lat; end
            2: begin ld_a = C_X2_X; ld_b = x2_lat; end
            3: begin ld_a = C_X2_U; ld_b = d_lat; end
            default: begin ld_a = C_ARM_LP; ld_b = d_arm_lat; end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x1 <= 0; x2 <= 0; arm_lp <= 0; primed <= 0; rate_q16 <= 0;
            arm_rate_lp_q16 <= 0; busy <= 0;
            st <= IDLE; which <= 0; d_lat <= 0; x1_lat <= 0; x2_lat <= 0; d_arm_lat <= 0;
            acc1 <= 0; acc2 <= 0; acc3 <= 0;
            mag_a <= 0; mag_b <= 0; mul_lo <= 0;
            product <= 0; mul_acc <= 0; mag_shift <= 0;
            mul_sign <= 0; mul_half <= 0; mul_carry <= 0; mul_done <= 0; mul_bit <= 0;
        end else if (clear) begin
            x1 <= 0; x2 <= 0; arm_lp <= 0; primed <= 0; rate_q16 <= 0;
            arm_rate_lp_q16 <= 0; busy <= 0; st <= IDLE;
            which <= 0; acc1 <= 0; acc2 <= 0; acc3 <= 0; mul_done <= 0;
        end else begin
            mul_done <= 0;
            case (st)
                IDLE: if (start) begin
                    if (!primed) begin
                        x1 <= alpha_q16; x2 <= 0; rate_q16 <= 0;
                        arm_lp <= 0; arm_rate_lp_q16 <= 0; primed <= 1;
                    end else begin
                        x1_lat <= x1; x2_lat <= x2;
                        d_lat <= d_sat; rate_q16 <= x2;
                        d_arm_lat <= d_arm_sat; arm_rate_lp_q16 <= arm_lp;
                        acc1 <= 0; acc2 <= 0; acc3 <= 0; which <= 0; busy <= 1;
                        st <= MUL_LOAD;
                    end
                end
                MUL_LOAD: begin
                    mul_sign <= ld_a[31] ^ ld_b[31];
                    mag_a <= ld_a[31] ? -ld_a : ld_a;
                    mag_b <= ld_b[31] ? -ld_b : ld_b;
                    mag_shift <= {32'd0, (ld_a[31] ? -ld_a : ld_a)};
                    mul_acc <= 0; mul_bit <= 0; mul_half <= 0;
                    mul_lo <= 0; mul_carry <= 0;
                    st <= MUL_WAIT;
                end
                MUL_WAIT: begin
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
                        st <= ACC;
                    end
                end
                ACC: begin
                    if (which < 2) acc1 <= acc1 + product;
                    else if (which < 4) acc2 <= acc2 + product;
                    else acc3 <= acc3 + product;
                    if (which == 4) st <= FINISH;
                    else begin which <= which + 3'd1; st <= MUL_LOAD; end
                end
                FINISH: begin
                    x1 <= (x1_sum > 33'sd2147483647) ? 32'sh7fffffff :
                          (x1_sum < -33'sd2147483648) ? 32'sh80000000 : x1_sum[31:0];
                    arm_lp <= (arm_sum > 33'sd2147483647) ? 32'sh7fffffff :
                              (arm_sum < -33'sd2147483648) ? 32'sh80000000 : arm_sum[31:0];
                    x2 <= sat32(x2_shift);
                    busy <= 0; st <= IDLE;
                end
                default: st <= IDLE;
            endcase
        end
    end
endmodule
