`timescale 1ns/1ps

// The vector stream supplies complete input snapshots and golden outputs.
// Every intervening cycle must retain the last committed outputs; the bench
// deliberately has no access to the core's internal pipeline or state arrays.
module tb_balance_core;
    parameter integer RESULT_LATENCY = 256;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg clear = 1'b0;
    reg sample_valid = 1'b0;
    reg signed [31:0] theta_q16 = 0, alpha_q16 = 0;
    reg [1023:0] coefficients = 0;
    reg [15:0] max_compare = 1000, comp_compare = 175, blend_compare = 25;
    reg signed [31:0] comp_gain_q20 = 7340032, feedforward_q16 = 0;
    reg [3:0] filter_mode = 4'd0;
    reg preset_valid = 1'b0;
    reg signed [31:0] preset_x0 = 0, preset_x1 = 0, preset_x2 = 0, preset_x3 = 0;
    wire busy, sample_ready, result_valid, fault, overrun;
    wire signed [31:0] xhat0, xhat1, xhat2, xhat3;
    wire signed [31:0] u_linear_q16, u_compensated_q16, u_model_q16;
    wire signed [15:0] signed_command;
    wire [15:0] latency_cycles;

    j280_balance_core #(.RESULT_LATENCY(RESULT_LATENCY)) dut (
        .clk(clk), .rst_n(rst_n), .clear(clear), .sample_valid(sample_valid),
        .theta_q16(theta_q16), .alpha_q16(alpha_q16), .coefficients(coefficients),
        .max_compare(max_compare), .comp_compare(comp_compare),
        .blend_compare(blend_compare), .comp_gain_q20(comp_gain_q20), .feedforward_q16(feedforward_q16),
        .filter_mode(filter_mode),
        .alpha_rate_ext_q16(32'sd0), .alpha_rate_mix(1'b0),
        .arm_rate_ext_q16(32'sd0), .arm_rate_mix(1'b0),
        .preset_valid(preset_valid), .preset_x0(preset_x0), .preset_x1(preset_x1),
        .preset_x2(preset_x2), .preset_x3(preset_x3),
        .busy(busy), .sample_ready(sample_ready), .result_valid(result_valid),
        .fault(fault), .overrun(overrun),
        .xhat0(xhat0), .xhat1(xhat1), .xhat2(xhat2), .xhat3(xhat3),
        .u_linear_q16(u_linear_q16), .u_compensated_q16(u_compensated_q16),
        .u_model_q16(u_model_q16), .signed_command(signed_command),
        .latency_cycles(latency_cycles)
    );

    integer file, count, row, gap, flags, step;
    reg [31:0] in_theta, in_alpha, in_gain;
    reg [1023:0] in_coefficients;
    integer in_max, in_comp, in_blend;
    reg [4:0] expected_status;
    reg [31:0] ex0, ex1, ex2, ex3, ex_linear, ex_comp, ex_model;
    reg [15:0] ex_command, ex_latency;
    reg [31:0] in_p0, in_p1, in_p2, in_p3, in_ff;
    reg [31:0] in_mode;
    wire [255:0] data = {xhat0, xhat1, xhat2, xhat3,
                         u_linear_q16, u_compensated_q16, u_model_q16,
                         signed_command, latency_cycles};
    reg [255:0] held_data;
    reg [2:0] held_status;
    integer clock_index, accepted_cycle;
    reg accepting;

    initial begin
        if (RESULT_LATENCY > 500) begin
            $display("FAIL test latency parameter exceeds 500"); $finish;
        end
        row = 0;
        clock_index = 0; accepted_cycle = -1000;
        repeat (3) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        #1;
        held_data = 0;
        held_status = 0;
        file = $fopen("vectors.txt", "r");
        if (file == 0) begin $display("FAIL cannot open vectors"); $finish; end
        while (!$feof(file)) begin
            count = $fscanf(file, "%d %d %h %h %h %d %d %d %h %h %h %h %h %h %h %h %h %h %d %h %h %h %h %h %d\n",
                            gap, flags, in_theta, in_alpha, in_coefficients,
                            in_max, in_comp, in_blend, in_gain, expected_status,
                            ex0, ex1, ex2, ex3, ex_linear, ex_comp, ex_model,
                            ex_command, ex_latency, in_p0, in_p1, in_p2, in_p3, in_ff, in_mode);
            if (count != 25) begin
                $display("FAIL malformed vector row=%0d fields=%0d", row, count); $finish;
            end
            row = row + 1;
            for (step = 1; step <= gap; step = step + 1) begin
                @(negedge clk);
                rst_n = 1'b1; clear = 1'b0; sample_valid = 1'b0; preset_valid = 1'b0;
                if (step == gap) begin
                    rst_n = !flags[2]; clear = flags[1]; sample_valid = flags[0];
                    preset_valid = flags[3];
                    preset_x0 = in_p0; preset_x1 = in_p1; preset_x2 = in_p2; preset_x3 = in_p3;
                    theta_q16 = in_theta; alpha_q16 = in_alpha;
                    coefficients = in_coefficients;
                    max_compare = in_max; comp_compare = in_comp;
                    blend_compare = in_blend; comp_gain_q20 = in_gain; feedforward_q16 = in_ff;
                    filter_mode = in_mode[3:0];
                end
                #1;
                accepting = rst_n && !clear && sample_valid && sample_ready;
                @(posedge clk); #1;
                clock_index = clock_index + 1;
                if (accepting) accepted_cycle = clock_index;
                if (step < gap) begin
                    if (data !== held_data || {busy, fault, overrun} !== held_status ||
                        result_valid !== 1'b0) begin
                        $display("FAIL early/uncommitted change row=%0d offset=%0d data=%h held=%h status=%b valid=%b",
                                 row, step, data, held_data, {busy,fault,overrun}, result_valid);
                        $finish;
                    end
                    if ((!busy && sample_ready !== !fault) ||
                        (busy && clock_index - accepted_cycle < RESULT_LATENCY - 1 &&
                         sample_ready !== 1'b0)) begin
                        $display("FAIL premature/idle readiness row=%0d offset=%0d ready=%b",
                                 row, step, sample_ready);
                        $finish;
                    end
                end else begin
                    if ({overrun, fault, result_valid, sample_ready, busy} !== expected_status ||
                        data !== {ex0, ex1, ex2, ex3, ex_linear, ex_comp, ex_model,
                                  ex_command, ex_latency}) begin
                        $display("FAIL mismatch row=%0d offset=%0d status=%b expected=%b",
                                 row, step, {overrun,fault,result_valid,sample_ready,busy}, expected_status);
                        $display(" actual   x=%0d,%0d,%0d,%0d u=%0d,%0d,%0d cmd=%0d latency=%0d",
                                 xhat0,xhat1,xhat2,xhat3,u_linear_q16,u_compensated_q16,u_model_q16,
                                 signed_command,latency_cycles);
                        $display(" expected x=%0d,%0d,%0d,%0d u=%0d,%0d,%0d cmd=%0d latency=%0d",
                                 $signed(ex0),$signed(ex1),$signed(ex2),$signed(ex3),
                                 $signed(ex_linear),$signed(ex_comp),$signed(ex_model),
                                 $signed(ex_command),ex_latency);
                        $finish;
                    end
                end
            end
            held_data = data;
            held_status = {busy, fault, overrun};
        end
        $fclose(file);
        $display("PASS tb_balance_core rows=%0d RESULT_LATENCY=%0d", row, RESULT_LATENCY);
        $finish;
    end

    initial begin
        #100000000;
        $display("FAIL global simulation timeout"); $finish;
    end
endmodule
