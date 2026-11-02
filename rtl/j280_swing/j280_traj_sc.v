`timescale 1ns/1ps
// v6 arm reference trajectory and feedforward; bit-exact to scripts/j280_traj.py.
// Each `start` (one per 2 ms balance sample) runs a ~75-clock sequence with at
// most one add/compare per clock; the three feedforward products share one
// 16-step shift-add multiplier (no DSP/LUT multiplier on the clock path), then
// every output is published on the same clock with `done`.
// `reset_now` holds the state at the anchor; the top bypasses the published
// outputs while it is high (they lag it by a clock).
module j280_traj_sc (
    input wire clk, rst_n,
    input wire reset_now,
    input wire signed [31:0] anchor,
    input wire start,
    input wire ccw_event, cw_event, rotate, settled,
    input wire key_go,
    input wire signed [15:0] key_delta,
    input wire [15:0] key_speed,
    input wire signed [31:0] arm,
    input wire [10:0] step_counts,
    input wire [15:0] move_speed, rotate_speed,
    input wire [11:0] accel,
    input wire [15:0] kv, ka, ki,
    input wire [13:0] lean_arm, lean_pend,
    input wire [10:0] i_window,
    input wire [15:0] max_compare,
    output reg done,
    output reg signed [47:0] r_out, theta_ref_out,
    output reg signed [31:0] alpha_ref_out, u_ff_out, v_out, target_out,
    output reg signed [15:0] index_out,
    output reg moving_out, rotating_out
);
    localparam [5:0] IDLE=0, I1=1, I2=2, I3=3, M1=4, M2=5, M3=6, C1=7, C2=8, C3=9,
                     W1=10, W2=11, W3=12, P1=13, P2=14, ACT=15, R1=16, R2=17, R3=18,
                     F1=19, F2=20, F3=21, F4=22, F5=23, MUL=24, SC1=25, SC2=26;
    localparam signed [21:0] I_MAX=22'sd1048576;
    localparam signed [37:0] UI_MAX=38'sd16384000;   // 250 << 16
    reg [5:0] st;
    reg signed [47:0] r, r_next;
    reg [16:0] m, speed;
    reg [32:0] stop;
    reg neg;                      // motion direction: 0 = +1, 1 = -1
    reg last_neg;
    reg key_mode, key_pending;
    reg signed [15:0] key_delta_lat;
    wire [15:0] move_speed_eff = key_mode ? key_speed : move_speed;
    reg signed [31:0] target;
    reg signed [15:0] index;
    reg rotating;
    reg signed [21:0] i_acc;
    reg ccw_pend, cw_pend, ccw_l, cw_l, rot_l, set_l;
    reg signed [31:0] arm_l;
    reg signed [1:0] s;           // acceleration sign of this sample
    // pipeline registers
    reg signed [32:0] e_wide, g;
    reg [32:0] e_abs;
    reg signed [22:0] i_sum;
    reg int_ok;
    reg signed [48:0] diff, diff_neg, d, pdiff;
    reg [48:0] dabs, ad;
    reg [33:0] lim, c_acc, c_cru;
    reg [17:0] spd_a;
    reg [16:0] spd_m_a;
    reg signed [1:0] sgn;
    reg signed [17:0] v_reg;
    reg signed [33:0] pv, tv;
    reg [27:0] pa;
    reg signed [37:0] pi, ti_raw, ti;
    reg signed [38:0] ta, s1, sum;
    reg signed [31:0] u_clip;
    reg signed [47:0] th_next;
    // Shared sequential multiplier: acc = mcand * mplier (mplier unsigned 16-bit),
    // run for v*kv, accel*ka, i_acc*ki in turn (which = 0, 1, 2).
    reg signed [37:0] mcand, macc;
    reg [15:0] mplier;
    reg [4:0] mcount;
    reg [1:0] which;
    // S-curve 260-count step table (1400 x {pos[15:0],v[31:0],u_ff[31:0],lean[23:0],alpha[23:0]}).
    reg [127:0] sc_table[0:1399];
    initial $readmemh("j280_traj_sc.mem", sc_table);
    reg sc_active, sc_dir;
    reg [10:0] sc_idx;
    reg signed [31:0] sc_base;
    reg [127:0] sc_word;
    wire [15:0] sc_pos = sc_word[127:112];
    wire signed [31:0] sc_vt = sc_word[111:80];
    wire signed [31:0] sc_ut = sc_word[79:48];
    wire signed [23:0] sc_lt = sc_word[47:24];
    wire signed [23:0] sc_at = sc_word[23:0];
    wire signed [47:0] sc_r_next = $signed({sc_base,16'd0}) +
        (sc_dir ? $signed({16'd0,sc_pos,16'd0}) : -$signed({16'd0,sc_pos,16'd0}));
    wire signed [47:0] sc_theta_next = sc_r_next -
        (sc_dir ? $signed({{24{sc_lt[23]}},sc_lt}) : -$signed({{24{sc_lt[23]}},sc_lt}));
    wire signed [31:0] sc_alpha_next = sc_dir ? $signed({{8{sc_at[23]}},sc_at}) : -$signed({{8{sc_at[23]}},sc_at});
    wire signed [31:0] sc_uff_next = sc_dir ? sc_ut : -sc_ut;
    wire signed [31:0] sc_v_next = sc_dir ? sc_vt : -sc_vt;

    wire enabled=(step_counts!=11'd0) && set_l;
    wire signed [31:0] step32={21'd0,step_counts};
    wire signed [47:0] target_q=$signed({target,16'd0});
    wire signed [32:0] t_minus=$signed({target[31],target})-$signed({22'd0,step_counts});
    wire signed [32:0] t_plus=$signed({target[31],target})+$signed({22'd0,step_counts});
    wire signed [48:0] g_q={g,16'd0};
    wire signed [48:0] r49={r[47],r};
    wire [48:0] queue_limit={22'd0,step_counts,16'd0}<<2;
    wire signed [1:0] dir_s=neg ? -2'sd1 : 2'sd1;
    wire signed [38:0] bound={7'd0,max_compare,16'd0};
    wire signed [47:0] lean_a48={18'd0,lean_arm,8'd0};
    wire signed [31:0] lean_p32={10'd0,lean_pend,8'd0};

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            st<=IDLE; done<=0;
            r<=0; r_next<=0; m<=0; speed<=0; stop<=0; neg<=0; last_neg<=0;
            key_mode<=0; key_pending<=0; key_delta_lat<=0;
            target<=0; index<=0; rotating<=0; i_acc<=0;
            ccw_pend<=0; cw_pend<=0; ccw_l<=0; cw_l<=0; rot_l<=0; set_l<=0; arm_l<=0; s<=0;
            e_wide<=0; g<=0; e_abs<=0; i_sum<=0; int_ok<=0; diff<=0; diff_neg<=0; d<=0; pdiff<=0; dabs<=0; ad<=0;
            lim<=0; c_acc<=0; c_cru<=0; spd_a<=0; spd_m_a<=0; sgn<=0; v_reg<=0;
            pv<=0; tv<=0; pa<=0; pi<=0; ti_raw<=0; ti<=0; ta<=0; s1<=0; sum<=0; u_clip<=0; th_next<=0;
            mcand<=0; macc<=0; mplier<=0; mcount<=0; which<=0;
            sc_active<=0; sc_dir<=0; sc_idx<=0; sc_base<=0; sc_word<=0;
            r_out<=0; theta_ref_out<=0; alpha_ref_out<=0; u_ff_out<=0; v_out<=0;
            target_out<=0; index_out<=0; moving_out<=0; rotating_out<=0;
        end else if(reset_now) begin
            st<=IDLE; done<=0;
            r<=$signed({anchor,16'd0}); m<=0; speed<=0; stop<=0; neg<=0; last_neg<=0;
            key_mode<=0; key_pending<=0; key_delta_lat<=0;
            target<=anchor; index<=0; rotating<=0; i_acc<=0;
            ccw_pend<=0; cw_pend<=0; s<=0;
            sc_active<=0; sc_dir<=0; sc_idx<=0; sc_base<=anchor; sc_word<=0;
            r_out<=$signed({anchor,16'd0}); theta_ref_out<=$signed({anchor,16'd0});
            alpha_ref_out<=0; u_ff_out<=0; v_out<=0;
            target_out<=anchor; index_out<=0; moving_out<=0; rotating_out<=0;
        end else begin
            done<=0;
            if(key_go) begin
                key_pending<=1; key_delta_lat<=key_delta;
            end
            if(st==IDLE && start && sc_active) begin
                sc_word<=sc_table[sc_idx];
                st<=SC1;
            end else if(st==IDLE && start) begin
                ccw_l<=ccw_pend || ccw_event; cw_l<=cw_pend || cw_event;
                ccw_pend<=0; cw_pend<=0;
                rot_l<=rotate; set_l<=settled; arm_l<=arm; s<=0;
                if(key_pending) begin
                    target<=$signed(r[47:16])+key_delta_lat;
                    key_mode<=1; key_pending<=0;
                end
                st<=I1;
            end else begin
                if(ccw_event) ccw_pend<=1;
                if(cw_event) cw_pend<=1;
            end
            case(st)
                // Integral of the arm error, while resting after the settle time.
                I1: begin
                    e_wide<=$signed({arm_l[31],arm_l})-$signed({r[47],r[47:16]});
                    int_ok<=ki!=16'd0 && set_l && m==17'd0 && !rotating;
                    st<=I2;
                end
                I2: begin
                    e_abs<=e_wide[32] ? (33'd0-e_wide) : e_wide;
                    i_sum<=$signed({i_acc[21],i_acc})+$signed(e_wide[22:0]);
                    st<=I3;
                end
                I3: begin
                    if(int_ok && e_abs<={22'd0,i_window})
                        i_acc<=(i_sum>$signed({1'b0,I_MAX})) ? I_MAX :
                               (i_sum< -$signed({1'b0,I_MAX})) ? -I_MAX : i_sum[21:0];
                    g<=neg ? t_minus : t_plus;
                    st<=M1;
                end
                // Rotation exit (stop on the grid point ahead) or start.
                M1: begin
                    diff<=g_q-r49;
                    diff_neg<=r49-g_q;
                    lim<={1'b0,stop}+{17'd0,speed};
                    st<=M2;
                end
                M2: begin
                    diff<=neg ? diff_neg : diff;
                    st<=M3;
                end
                M3: begin
                    if(rotating && !rot_l) begin
                        rotating<=0;
                        if(diff<$signed({15'd0,lim})) begin
                            target<=neg ? g[31:0]-step32 : g[31:0]+step32;
                            index<=neg ? index-16'sd2 : index+16'sd2;
                        end else begin
                            target<=g[31:0];
                            index<=neg ? index-16'sd1 : index+16'sd1;
                        end
                    end else if(enabled && rot_l && !rotating && rotate_speed!=16'd0 && m==17'd0 &&
                                r==target_q) begin
                        rotating<=1; neg<=last_neg;
                    end
                    st<=C1;
                end
                // Keys: SW3 then SW4, each within MAX_QUEUE_STEPS steps of the reference.
                C1: begin
                    g<=t_minus;
                    diff<=$signed({t_minus,16'd0})-r49;
                    st<=C2;
                end
                C2: begin
                    dabs<=diff[48] ? (49'd0-diff) : diff;
                    st<=C3;
                end
                C3: begin
                    if(enabled && !rotating && ccw_l && dabs<=queue_limit &&
                       !sc_active && step_counts==11'd260) begin
                        sc_base<=target; sc_dir<=0; sc_idx<=0; sc_active<=1;
                        target<=g[31:0]; index<=index-16'sd1; last_neg<=1;
                    end
                    st<=W1;
                end
                W1: begin
                    g<=t_plus;
                    diff<=$signed({t_plus,16'd0})-r49;
                    st<=W2;
                end
                W2: begin
                    dabs<=diff[48] ? (49'd0-diff) : diff;
                    st<=W3;
                end
                W3: begin
                    if(enabled && !rotating && cw_l && dabs<=queue_limit &&
                       !sc_active && step_counts==11'd260) begin
                        sc_base<=target; sc_dir<=1; sc_idx<=0; sc_active<=1;
                        target<=g[31:0]; index<=index+16'sd1; last_neg<=0;
                    end
                    st<=P1;
                end
                // Profile: accelerate / cruise / decelerate / snap.
                P1: begin
                    d<=$signed({target_q[47],target_q})-r49;
                    c_acc<={1'b0,stop}+{16'd0,speed,1'b0}+{22'd0,accel};
                    c_cru<={1'b0,stop}+{17'd0,speed};
                    spd_a<={1'b0,speed}+{6'd0,accel};
                    spd_m_a<=speed-{5'd0,accel};
                    st<=P2;
                end
                P2: begin
                    ad<=d[48] ? (49'd0-d) : d;
                    sgn<=(d>0) ? 2'sd1 : (d<0) ? -2'sd1 : 2'sd0;
                    st<=ACT;
                end
                ACT: begin
                    if(sc_active) begin end
                    else if(rotating) begin
                        if(spd_a<={2'd0,rotate_speed}) begin
                            stop<=stop+{16'd0,speed}; speed<=spd_a[16:0]; m<=m+17'd1; s<=dir_s;
                        end else if(speed>{1'b0,rotate_speed}) begin
                            m<=m-17'd1; speed<=spd_m_a; stop<=stop-{16'd0,spd_m_a}; s<=-dir_s;
                        end
                    end else begin
                        if(m!=17'd0 && sgn!=dir_s) begin
                            m<=m-17'd1; speed<=spd_m_a; stop<=stop-{16'd0,spd_m_a}; s<=-dir_s;
                        end else if(sgn!=2'sd0) begin
                            if(spd_a<={2'd0,move_speed_eff} && ad>={15'd0,c_acc}) begin
                                neg<=sgn[1];
                                stop<=stop+{16'd0,speed}; speed<=spd_a[16:0]; m<=m+17'd1; s<=sgn;
                            end else if(m!=17'd0 && ad>={15'd0,c_cru} && speed<={1'b0,move_speed_eff}) begin
                            end else if(m!=17'd0) begin
                                m<=m-17'd1; speed<=spd_m_a; stop<=stop-{16'd0,spd_m_a}; s<=-dir_s;
                            end else begin
                                r<=target_q;
                            end
                        end
                    end
                    st<=R1;
                end
                // Advance the reference; while turning, pass grid points.
                R1: begin
                    r_next<=neg ? r-$signed({31'd0,speed}) : r+$signed({31'd0,speed});
                    v_reg<=neg ? -$signed({1'b0,speed}) : $signed({1'b0,speed});
                    g<=neg ? t_minus : t_plus;
                    st<=R2;
                end
                R2: begin
                    pdiff<=neg ? (g_q-$signed({r_next[47],r_next})) : ($signed({r_next[47],r_next})-g_q);
                    st<=R3;
                end
                R3: begin
                    if(rotating && !pdiff[48]) begin
                        target<=g[31:0]; index<=neg ? index-16'sd1 : index+16'sd1;
                    end
                    r<=r_next;
                    macc<=0; mcand<={{20{v_reg[17]}},v_reg}; mplier<=kv; mcount<=0; which<=0;
                    st<=MUL;
                end
                MUL: begin
                    if(mcount!=5'd16) begin
                        if(mplier[0]) macc<=macc+mcand;
                        mcand<=mcand<<<1;
                        mplier<=mplier>>1;
                        mcount<=mcount+5'd1;
                    end else begin
                        macc<=0; mcount<=0;
                        case(which)
                            2'd0: begin
                                pv<=macc[33:0];
                                mcand<={26'd0,accel}; mplier<=ka; which<=2'd1;
                            end
                            2'd1: begin
                                pa<=macc[27:0];
                                mcand<={{16{i_acc[21]}},i_acc}; mplier<=ki; which<=2'd2;
                            end
                            default: begin
                                pi<=macc;
                                st<=F1;
                            end
                        endcase
                    end
                end
                // Feedforward: trunc0(x, k) = (x + (x<0 ? 2^k-1 : 0)) >>> k.
                F1: begin
                    tv<=(pv+(pv[33] ? 34'sd3 : 34'sd0))>>>2;
                    ti_raw<=(pi+(pi[37] ? 38'sd15 : 38'sd0))>>>4;
                    ta<=(s==2'sd1) ? $signed({11'd0,pa}) : (s==-2'sd1) ? -$signed({11'd0,pa}) : 39'sd0;
                    st<=F2;
                end
                F2: begin
                    ti<=(ti_raw>UI_MAX) ? UI_MAX : (ti_raw< -UI_MAX) ? -UI_MAX : ti_raw;
                    s1<=$signed({{5{tv[33]}},tv})+ta;
                    st<=F3;
                end
                F3: begin
                    sum<=s1+$signed({ti[37],ti});
                    th_next<=(s==2'sd1) ? r-lean_a48 : (s==-2'sd1) ? r+lean_a48 : r;
                    st<=F4;
                end
                F4: begin
                    u_clip<=(sum>bound) ? bound[31:0] : (sum< -bound) ? -bound[31:0] : sum[31:0];
                    st<=F5;
                end
                F5: begin
                    done<=1; st<=IDLE;
                    if(m==17'd0 && speed==17'd0) key_mode<=0;
                    r_out<=r; v_out<={{14{v_reg[17]}},v_reg};
                    u_ff_out<=u_clip;
                    theta_ref_out<=th_next;
                    alpha_ref_out<=(s==2'sd1) ? lean_p32 : (s==-2'sd1) ? -lean_p32 : 32'sd0;
                    target_out<=target; index_out<=index;
                    moving_out<=(m!=17'd0)||sc_active||rotating; rotating_out<=rotating;
                end
                SC1: begin
                    r<=sc_r_next; r_out<=sc_r_next; theta_ref_out<=sc_theta_next;
                    alpha_ref_out<=sc_alpha_next; u_ff_out<=sc_uff_next; v_out<=sc_v_next;
                    target_out<=target; index_out<=index; moving_out<=1; rotating_out<=0;
                    if(sc_idx==11'd1399) sc_active<=0;
                    else sc_idx<=sc_idx+11'd1;
                    st<=SC2;
                end
                SC2: begin done<=1; st<=IDLE; end
                default: begin end
            endcase
        end
    end
endmodule
