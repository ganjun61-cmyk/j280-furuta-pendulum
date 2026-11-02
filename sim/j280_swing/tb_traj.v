`timescale 1ns/1ps
// File-driven j280_traj check against scripts/j280_traj.py.
//   P step move rot accel kv ka ki lean_arm lean_pend i_window max_compare
//   R anchor(hex32)                      : reset_now for 3 clocks
//   S ccw cw rot settled arm(hex32) | r(hex48) th(hex48) al(hex32) uff(hex32) v(hex32) tgt(hex32) idx(hex16) mv rt
// Each S row pulses start, checks every output on the done clock, then checks
// that all outputs hold until the next row; key events arrive a few clocks
// before start (latched) and during the busy sequence (pending to the next row).
module tb_traj;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0, reset_now=0, start=0, ccw=0, cw=0, rot=0, settled=0;
    reg signed [31:0] anchor=0, arm=0;
    reg [10:0] step_counts=0, i_window=0;
    reg [15:0] move_speed=0, rotate_speed=0, kv=0, ka=0, ki=0, max_compare=0;
    reg [11:0] accel=0;
    reg [13:0] lean_arm=0, lean_pend=0;
    wire done, moving, rotating;
    wire signed [47:0] r_out, th_out;
    wire signed [31:0] al_out, uff_out, v_out, tgt_out;
    wire signed [15:0] idx_out;
    j280_traj dut(.clk(clk),.rst_n(rst_n),.reset_now(reset_now),.anchor(anchor),.start(start),
        .ccw_event(ccw),.cw_event(cw),.rotate(rot),.settled(settled),.arm(arm),
        .rot_dir_cw(1'b0),.rot_dir_ccw(1'b0),.rot_gear_up(1'b0),.rot_gear_down(1'b0),
        .key_go(1'b0),.key_delta(16'sd0),.key_speed(16'd0),
        .step_counts(step_counts),.move_speed(move_speed),.rotate_speed(rotate_speed),.accel(accel),
        .kv(kv),.ka(ka),.ki(ki),.lean_arm(lean_arm),.lean_pend(lean_pend),.i_window(i_window),
        .max_compare(max_compare),.done(done),.r_out(r_out),.theta_ref_out(th_out),
        .alpha_ref_out(al_out),.u_ff_out(uff_out),.v_out(v_out),.target_out(tgt_out),
        .index_out(idx_out),.moving_out(moving),.rotating_out(rotating));
    integer file, count, row, i_ccw, i_cw, i_rot, i_set, e_mv, e_rt, waitc, p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10;
    reg [31:0] i_arm, e_al, e_uff, e_v, e_tgt, i_anchor;
    reg [47:0] e_r, e_th;
    reg [15:0] e_idx;
    reg [7:0] op;
    reg [255:0] held;
    task fail; input [8*48-1:0] msg; begin
        $display("FAIL %0s row=%0d r=%h th=%h al=%h uff=%h v=%h tgt=%h idx=%h mv=%0d rt=%0d", msg, row,
                 r_out, th_out, al_out, uff_out, v_out, tgt_out, idx_out, moving, rotating);
        $display("     want r=%h th=%h al=%h uff=%h v=%h tgt=%h idx=%h mv=%0d rt=%0d",
                 e_r, e_th, e_al, e_uff, e_v, e_tgt, e_idx, e_mv, e_rt);
        $finish; end endtask
    initial begin
        file=$fopen("vectors.txt","r");
        if(file==0) begin $display("FAIL cannot open vectors"); $finish; end
        repeat(3) @(posedge clk);
        @(negedge clk) rst_n=1;
        row=0;
        while(!$feof(file)) begin
            count=$fscanf(file,"%c",op);
            if(op=="P") begin
                count=$fscanf(file," %d %d %d %d %d %d %d %d %d %d %d\n",p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10);
                if(count!=11) begin $display("FAIL malformed P"); $finish; end
                step_counts=p0; move_speed=p1; rotate_speed=p2; accel=p3; kv=p4; ka=p5; ki=p6;
                lean_arm=p7; lean_pend=p8; i_window=p9; max_compare=p10;
            end else if(op=="R") begin
                count=$fscanf(file," %h\n",i_anchor);
                if(count!=1) begin $display("FAIL malformed R"); $finish; end
                @(negedge clk); anchor=i_anchor; reset_now=1;
                repeat(3) @(negedge clk);
                reset_now=0;
                if(r_out!=={i_anchor,16'd0} || th_out!=={i_anchor,16'd0} || uff_out!==0 || tgt_out!==i_anchor)
                    begin $display("FAIL reset outputs"); $finish; end
            end else if(op=="S") begin
                count=$fscanf(file," %d %d %d %d %h %h %h %h %h %h %h %h %d %d\n",
                    i_ccw,i_cw,i_rot,i_set,i_arm,e_r,e_th,e_al,e_uff,e_v,e_tgt,e_idx,e_mv,e_rt);
                if(count!=14) begin $display("FAIL malformed S row=%0d", row); $finish; end
                row=row+1;
                // A ccw event before start (latched by pending), a cw event on the start clock.
                @(negedge clk); rot=i_rot; settled=i_set; arm=i_arm; ccw=i_ccw;
                @(negedge clk); ccw=0;
                @(negedge clk); cw=i_cw; start=1;
                @(negedge clk); cw=0; start=0;
                held={r_out,th_out,al_out,uff_out,v_out,tgt_out,idx_out,moving,rotating};
                waitc=0;
                while(!done) begin
                    if({r_out,th_out,al_out,uff_out,v_out,tgt_out,idx_out,moving,rotating}!==held)
                        fail("outputs moved before done");
                    @(negedge clk); waitc=waitc+1;
                    if(waitc>200) fail("no done");
                end
                if(r_out!==e_r || th_out!==e_th || al_out!==e_al || uff_out!==e_uff || v_out!==e_v ||
                   tgt_out!==e_tgt || idx_out!==e_idx || moving!==e_mv[0] || rotating!==e_rt[0])
                    fail("mismatch");
                held={r_out,th_out,al_out,uff_out,v_out,tgt_out,idx_out,moving,rotating};
                arm=32'h5a5a5a5a; rot=!rot;
                repeat(5) @(negedge clk);
                if({r_out,th_out,al_out,uff_out,v_out,tgt_out,idx_out,moving,rotating}!==held)
                    fail("outputs not held");
                rot=i_rot;
            end else if(op!="\n" && op!=" " && op!=8'hff) begin
                $display("FAIL unknown op %0d row=%0d", op, row); $finish;
            end
        end
        $fclose(file);
        $display("PASS tb_traj rows=%0d", row);
        $finish;
    end
    initial begin #2000000000; $display("FAIL watchdog"); $finish; end
endmodule
