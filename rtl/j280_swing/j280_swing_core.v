`timescale 1ns/1ps

// Swing-up law, bit-exact with scripts/j280_swing_math.py step()/initial_state().
// One 32x32 signed multiplier with registered operands and product; every
// shift is arithmetic (floor). About 70 clocks per sample, far below 2 ms.
// cfg is the SWING command payload as latched by the guard (bit 0 = version).
module j280_swing_core (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire sample_valid,
    input wire [9:0] code,
    input wire adc_valid,
    input wire signed [31:0] theta,
    input wire [575:0] cfg,
    input wire [15:0] max_compare,
    input wire signed [31:0] zero_q16,
    output reg busy,
    output reg result_valid,
    output reg overrun,
    output reg signed [15:0] command,
    output reg good,
    output reg blind_mid,
    output reg catch_ok,
    output reg [15:0] blind,
    output reg signed [31:0] est_a,
    output reg signed [31:0] est_ad,
    output reg signed [31:0] energy,
    output reg signed [31:0] ref_q16,
    output reg signed [31:0] theta_speed,
    output reg signed [31:0] preset_x0,
    output reg signed [31:0] preset_x1,
    output reg signed [31:0] preset_x2,
    output reg signed [31:0] preset_x3
);
    wire [15:0] step_code = cfg[80+:16], edge_low = cfg[96+:16], edge_high = cfg[112+:16];
    wire [15:0] kick_samples = cfg[160+:16], lead_samples = cfg[192+:16];
    wire signed [15:0] kick_compare = cfg[176+:16];
    wire signed [31:0] catch_angle = cfg[224+:32], catch_rate = cfg[256+:32];
    wire signed [31:0] gravity = cfg[288+:32], gravity_lead = cfg[320+:32];
    wire signed [31:0] kv = cfg[352+:32], kdrive = cfg[384+:32], margin = cfg[416+:32];
    wire signed [31:0] kscale = cfg[448+:32], amp = cfg[480+:32];
    wire signed [31:0] kp = cfg[512+:32], kd = cfg[544+:32];

    localparam [5:0] IDLE=0, INIT1=1, INIT2=2, L0=3, L1=4, MULW=5, GRAV1=6, GRAV2=7,
        TRIG0=8, TRIG1=9, TRIG2=10, V1=11, V2=12, E1=13, AP1=14, ADP1=15, CP0=16, CP1=17,
        T1=18, T2=19, D1=20, S1=21, S2=22, A1=23, A2=24, R1=25, R2=26, R3=27, U1=28,
        U2=29, C1=30, P0=31, P1=32, P2=33;
    localparam signed [63:0] AD_LIMIT = 64'sd1073741824, V_LIMIT = 64'sd1048576;
    localparam signed [63:0] ONE64 = 64'sd65536;

    reg [5:0] state, ret;
    // Persistent swing state (est_a/est_ad are outputs and state).
    reg [3:0] hist_good;
    reg signed [31:0] hist_ang0, hist_ang1, hist_ang2, hist_ang3;
    reg [9:0] hist_code0, hist_code1, hist_code2, hist_code3;
    reg last_good;
    reg [9:0] last_code;
    reg signed [12:0] th1, th2, th3, th4, th5;
    reg [15:0] n;
    // Per-sample working registers.
    reg [9:0] c_code;
    reg c_valid;
    reg signed [12:0] c_th;
    reg good_r, run4_r, run2_r, mid_r;
    reg [15:0] blind_r;
    reg signed [13:0] rate5_r;
    reg signed [31:0] m_r, a_r, ad_r, sa_r, ca_r, v_r, e_r, ap_r, adp_r, t_r;
    reg signed [31:0] drive_r, scale_r, amps_r, ref_r, err_r, speed_r;
    reg signed [63:0] u1_r, u_r;
    reg signed [15:0] cmd_r;
    reg signed [31:0] op_a, op_b;
    reg signed [63:0] product;

    // ROMs: address is combinational from registers, data one clock later.
    reg [9:0] sine_addr;
    wire [15:0] angle_q;
    wire signed [17:0] sine_q;
    j280_swing_angle_rom angle_rom(.clk(clk), .addr(c_code), .q(angle_q));
    j280_swing_sine_rom sine_rom(.clk(clk), .addr(sine_addr), .q(sine_q));
    function [9:0] sine_index;
        input [31:0] angle;
        reg [31:0] rounded;
        begin rounded = angle + 32'd2097152; sine_index = rounded[31:22]; end
    endfunction
    always @* begin
        case (state)
            L0: sine_addr = sine_index(est_a);
            TRIG0: sine_addr = sine_index(a_r);
            TRIG1: sine_addr = sine_index(a_r + 32'h40000000);
            CP0: sine_addr = sine_index(ap_r + 32'h40000000);
            default: sine_addr = 10'd0;
        endcase
    end

    wire signed [31:0] m_now = {angle_q, 16'd0};
    wire signed [31:0] sine32 = {{14{sine_q[17]}}, sine_q};
    function in_edge;
        input [9:0] c;
        begin in_edge = ({6'd0, c} <= edge_low) || ({6'd0, c} >= edge_high); end
    endfunction
    wire [10:0] step_abs = (c_code >= last_code) ? ({1'b0, c_code} - {1'b0, last_code})
                                                  : ({1'b0, last_code} - {1'b0, c_code});
    // After rejected samples the continuity step widens to step*(1+min(blind,3)).
    wire [17:0] step1 = {2'd0, step_code};
    wire [17:0] step_wide = step1 + ((blind >= 16'd1) ? step1 : 18'd0) +
                            ((blind >= 16'd2) ? step1 : 18'd0) + ((blind >= 16'd3) ? step1 : 18'd0);
    wire good_now = c_valid && (last_good ? ({7'd0, step_abs} <= step1)
                                          : ({7'd0, step_abs} <= step_wide || in_edge(c_code)));
    wire signed [31:0] diff4 = m_now - hist_ang3;
    wire signed [31:0] diff2 = m_now - hist_ang1;
    wire signed [63:0] grav_sum = est_ad + (product >>> 16);
    wire signed [63:0] v_wide = product >>> 32;
    wire signed [63:0] drive_wide = -(product >>> 32);
    wire signed [63:0] scale_wide = product >>> 16;
    wire signed [63:0] u_cmd = u_r >>> 16;
    wire signed [63:0] max64 = {48'd0, max_compare};
    wire signed [16:0] kick17 = kick_compare;
    wire signed [16:0] max17 = {1'b0, max_compare};
    wire signed [32:0] a_abs = a_r[31] ? -$signed({a_r[31], a_r}) : {a_r[31], a_r};
    wire signed [32:0] ad_abs = ad_r[31] ? -$signed({ad_r[31], ad_r}) : {ad_r[31], ad_r};
    wire signed [12:0] th_clip = (theta > 32'sd4095) ? 13'sd4095 :
                                 (theta < -32'sd4095) ? -13'sd4095 : theta[12:0];
    wire signed [13:0] rate_clip = (rate5_r > 14'sd300) ? 14'sd300 :
                                   (rate5_r < -14'sd300) ? -14'sd300 : rate5_r;

    task finish_init;
        begin
            est_a <= m_now; est_ad <= 0; hist_good <= {4{c_valid}};
            hist_ang0 <= m_now; hist_ang1 <= m_now; hist_ang2 <= m_now; hist_ang3 <= m_now;
            hist_code0 <= c_code; hist_code1 <= c_code; hist_code2 <= c_code; hist_code3 <= c_code;
            last_good <= c_valid; last_code <= c_code;
            th1 <= 0; th2 <= 0; th3 <= 0; th4 <= 0; th5 <= 0; n <= 0;
            state <= IDLE; busy <= 0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; ret <= IDLE; busy <= 0; result_valid <= 0; overrun <= 0;
            command <= 0; good <= 0; blind_mid <= 0; catch_ok <= 0; blind <= 0;
            est_a <= 0; est_ad <= 0; energy <= 0; ref_q16 <= 0; theta_speed <= 0;
            preset_x0 <= 0; preset_x1 <= 0; preset_x2 <= 0; preset_x3 <= 0;
            hist_good <= 0; hist_ang0 <= 0; hist_ang1 <= 0; hist_ang2 <= 0; hist_ang3 <= 0;
            hist_code0 <= 0; hist_code1 <= 0; hist_code2 <= 0; hist_code3 <= 0;
            last_good <= 0; last_code <= 0; th1 <= 0; th2 <= 0; th3 <= 0; th4 <= 0; th5 <= 0;
            n <= 0; c_code <= 0; c_valid <= 0; c_th <= 0;
            good_r <= 0; run4_r <= 0; run2_r <= 0; mid_r <= 0; blind_r <= 0; rate5_r <= 0;
            m_r <= 0; a_r <= 0; ad_r <= 0; sa_r <= 0; ca_r <= 0; v_r <= 0; e_r <= 0;
            ap_r <= 0; adp_r <= 0; t_r <= 0; drive_r <= 0; scale_r <= 0; amps_r <= 0;
            ref_r <= 0; err_r <= 0; speed_r <= 0; u1_r <= 0; u_r <= 0; cmd_r <= 0;
            op_a <= 0; op_b <= 0; product <= 0;
        end else begin
            result_valid <= 0;
            if (start) begin
                // Init needs one ROM read of the latest code; no result is published.
                state <= INIT1; busy <= 1; overrun <= 0;
                c_code <= code; c_valid <= adc_valid;
                command <= 0; good <= 0; blind_mid <= 0; catch_ok <= 0; blind <= 0;
                energy <= 0; ref_q16 <= 0; theta_speed <= 0;
                preset_x0 <= 0; preset_x1 <= 0; preset_x2 <= 0; preset_x3 <= 0;
            end else if (sample_valid && (state == INIT1 || state == INIT2)) begin
                // A sample racing the start is dropped; the next one is processed.
                if (state == INIT2) finish_init;
                else state <= INIT2;
            end else if (sample_valid && busy) begin
                overrun <= 1;
            end else if (sample_valid) begin
                state <= L0; busy <= 1;
                c_code <= code; c_valid <= adc_valid; c_th <= th_clip;
            end else begin
                case (state)
                    INIT1: state <= INIT2;
                    INIT2: finish_init;
                    L0: begin
                        good_r <= good_now;
                        mid_r <= !c_valid && !in_edge(last_code);
                        blind_r <= good_now ? 16'd0 : (blind == 16'hffff ? blind : blind + 16'd1);
                        run4_r <= good_now && (&hist_good);
                        run2_r <= good_now && hist_good[0] && hist_good[1];
                        rate5_r <= c_th - th5;
                        state <= L1;
                    end
                    L1: begin
                        m_r <= m_now;
                        if (run4_r) begin
                            ad_r <= diff4 >>> 2; a_r <= m_now; state <= TRIG0;
                        end else if (run2_r) begin
                            ad_r <= diff2 >>> 1; a_r <= m_now; state <= TRIG0;
                        end else begin
                            op_a <= gravity; op_b <= sine32; ret <= GRAV1; state <= MULW;
                        end
                    end
                    MULW: begin product <= op_a * op_b; state <= ret; end
                    GRAV1: begin
                        ad_r <= (grav_sum > AD_LIMIT) ? AD_LIMIT[31:0] :
                                (grav_sum < -AD_LIMIT) ? -AD_LIMIT[31:0] : grav_sum[31:0];
                        state <= GRAV2;
                    end
                    GRAV2: begin a_r <= good_r ? m_r : est_a + ad_r; state <= TRIG0; end
                    TRIG0: state <= TRIG1;
                    TRIG1: begin sa_r <= sine32; state <= TRIG2; end
                    TRIG2: begin
                        ca_r <= sine32; op_a <= ad_r; op_b <= kv; ret <= V1; state <= MULW;
                    end
                    V1: begin
                        v_r <= (v_wide > V_LIMIT) ? V_LIMIT[31:0] :
                               (v_wide < -V_LIMIT) ? -V_LIMIT[31:0] : v_wide[31:0];
                        state <= V2;
                    end
                    V2: begin op_a <= v_r; op_b <= v_r; ret <= E1; state <= MULW; end
                    E1: begin
                        e_r <= (product >>> 17) + ca_r - 32'sd65536;
                        op_a <= {16'd0, lead_samples}; op_b <= ad_r; ret <= AP1; state <= MULW;
                    end
                    AP1: begin
                        ap_r <= a_r + product[31:0];
                        op_a <= gravity_lead; op_b <= sa_r; ret <= ADP1; state <= MULW;
                    end
                    ADP1: begin adp_r <= ad_r + (product >>> 16); state <= CP0; end
                    CP0: state <= CP1;
                    CP1: begin op_a <= adp_r; op_b <= sine32; ret <= T1; state <= MULW; end
                    T1: begin t_r <= product >>> 16; state <= T2; end
                    T2: begin op_a <= t_r; op_b <= kdrive; ret <= D1; state <= MULW; end
                    D1: begin
                        drive_r <= (drive_wide > ONE64) ? 32'sd65536 :
                                   (drive_wide < -ONE64) ? -32'sd65536 : drive_wide[31:0];
                        op_a <= margin - e_r; op_b <= kscale; ret <= S1; state <= MULW;
                    end
                    S1: begin
                        scale_r <= (scale_wide > ONE64) ? 32'sd65536 :
                                   (scale_wide < 0) ? 32'sd0 : scale_wide[31:0];
                        state <= S2;
                    end
                    S2: begin op_a <= amp; op_b <= scale_r; ret <= A1; state <= MULW; end
                    A1: begin amps_r <= product >>> 16; state <= A2; end
                    A2: begin op_a <= amps_r; op_b <= drive_r; ret <= R1; state <= MULW; end
                    R1: begin ref_r <= -(product >>> 16); state <= R2; end
                    R2: begin err_r <= ref_r - {{3{c_th[12]}}, c_th, 16'd0}; state <= R3; end
                    R3: begin op_a <= kp; op_b <= err_r; ret <= U1; state <= MULW; end
                    U1: begin
                        u1_r <= product >>> 16;
                        op_a <= kd; op_b <= {{18{rate5_r[13]}}, rate5_r}; ret <= U2; state <= MULW;
                    end
                    U2: begin u_r <= u1_r - product; state <= C1; end
                    C1: begin
                        if (n < kick_samples)
                            cmd_r <= (kick17 > max17) ? max17[15:0] :
                                     (kick17 < -max17) ? -max17[15:0] : kick_compare;
                        else
                            cmd_r <= (u_cmd > max64) ? max_compare :
                                     (u_cmd < -max64) ? -max_compare : u_cmd[15:0];
                        state <= P0;
                    end
                    P0: begin
                        op_a <= {{18{rate_clip[13]}}, rate_clip}; op_b <= 32'sd6553600;
                        ret <= P1; state <= MULW;
                    end
                    P1: begin
                        speed_r <= product[31:0];
                        op_a <= $signed({22'd0, c_code}) - $signed({22'd0, hist_code3});
                        op_b <= 32'sd8192000; ret <= P2; state <= MULW;
                    end
                    P2: begin
                        result_valid <= 1; busy <= 0; state <= IDLE;
                        command <= cmd_r; good <= good_r; blind_mid <= mid_r; blind <= blind_r;
                        catch_ok <= run4_r && (a_abs < {catch_angle[31], catch_angle}) &&
                                    (ad_abs < {catch_rate[31], catch_rate});
                        energy <= e_r; ref_q16 <= ref_r; theta_speed <= speed_r;
                        preset_x0 <= run4_r ? {{3{c_th[12]}}, c_th, 16'd0} : 32'sd0;
                        preset_x1 <= run4_r ? $signed({6'd0, c_code, 16'd0}) - zero_q16 : 32'sd0;
                        preset_x2 <= run4_r ? speed_r : 32'sd0;
                        preset_x3 <= run4_r ? product[31:0] : 32'sd0;
                        est_a <= a_r; est_ad <= ad_r;
                        hist_good <= {hist_good[2:0], good_r};
                        hist_ang3 <= hist_ang2; hist_ang2 <= hist_ang1; hist_ang1 <= hist_ang0; hist_ang0 <= m_r;
                        hist_code3 <= hist_code2; hist_code2 <= hist_code1; hist_code1 <= hist_code0;
                        hist_code0 <= c_code;
                        last_good <= good_r;
                        if (good_r) last_code <= c_code;
                        th5 <= th4; th4 <= th3; th3 <= th2; th2 <= th1; th1 <= c_th;
                        if (n != 16'hffff) n <= n + 16'd1;
                    end
                    default: begin state <= IDLE; busy <= 0; end
                endcase
            end
        end
    end
endmodule
