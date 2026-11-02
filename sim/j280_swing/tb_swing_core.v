`timescale 1ns/1ps
// File-driven: first line cfg/max/zero, then one row per start or sample with
// the golden outputs from scripts/j280_swing_math.py. Outputs must not change
// between results, and every sample must finish within 100 clocks.
module tb_swing_core;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0, start = 0, sample_valid = 0, adc_valid = 0;
    reg [9:0] code = 0;
    reg signed [31:0] theta = 0;
    reg [575:0] cfg = 0;
    reg [15:0] max_compare = 0;
    reg signed [31:0] zero_q16 = 0;
    wire busy, result_valid, overrun, good, blind_mid, catch_ok;
    wire signed [15:0] command;
    wire [15:0] blind;
    wire signed [31:0] est_a, est_ad, energy, ref_q16, theta_speed, x0, x1, x2, x3;
    j280_swing_core dut(.clk(clk), .rst_n(rst_n), .start(start), .sample_valid(sample_valid),
        .code(code), .adc_valid(adc_valid), .theta(theta), .cfg(cfg), .max_compare(max_compare),
        .zero_q16(zero_q16), .busy(busy), .result_valid(result_valid), .overrun(overrun),
        .command(command), .good(good), .blind_mid(blind_mid), .catch_ok(catch_ok), .blind(blind),
        .est_a(est_a), .est_ad(est_ad), .energy(energy), .ref_q16(ref_q16), .theta_speed(theta_speed),
        .preset_x0(x0), .preset_x1(x1), .preset_x2(x2), .preset_x3(x3));

    wire [399:0] observed = {command, good, blind_mid, catch_ok, 5'd0, blind, est_a, est_ad,
                             energy, ref_q16, theta_speed, x0, x1, x2, x3};
    reg [399:0] held;
    integer file, count, row, op, in_code, in_valid, waited, max_wait;
    reg [31:0] in_theta, e_a, e_ad, e_energy, e_ref, e_speed, e0, e1, e2, e3;
    reg [15:0] e_command, e_blind;
    integer e_good, e_mid, e_catch;

    initial begin
        file = $fopen("vectors.txt", "r");
        if (file == 0) begin $display("FAIL cannot open vectors"); $finish; end
        count = $fscanf(file, "%h %d %h\n", cfg, max_compare, zero_q16);
        if (count != 3) begin $display("FAIL malformed header"); $finish; end
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1;
        row = 0; max_wait = 0; held = observed;
        while (!$feof(file)) begin
            count = $fscanf(file, "%d %d %d %h %h %d %d %d %h %h %h %h %h %h %h %h %h %h\n",
                op, in_code, in_valid, in_theta, e_command, e_good, e_mid, e_catch, e_blind,
                e_a, e_ad, e_energy, e_ref, e_speed, e0, e1, e2, e3);
            if (count != 18) begin $display("FAIL malformed row=%0d fields=%0d", row, count); $finish; end
            row = row + 1;
            @(negedge clk);
            code = in_code; adc_valid = in_valid; theta = in_theta;
            if (op == 0) start = 1; else sample_valid = 1;
            @(negedge clk);
            start = 0; sample_valid = 0;
            waited = 1;
            while (busy) begin
                if (op == 1 && observed !== held) begin
                    $display("FAIL output changed before result row=%0d", row); $finish;
                end
                @(negedge clk); waited = waited + 1;
                if (waited > 100) begin $display("FAIL latency row=%0d", row); $finish; end
            end
            if (waited > max_wait) max_wait = waited;
            if (overrun) begin $display("FAIL overrun row=%0d", row); $finish; end
            if (op == 1 && observed !== {e_command, e_good[0], e_mid[0], e_catch[0], 5'd0, e_blind,
                                         e_a, e_ad, e_energy, e_ref, e_speed, e0, e1, e2, e3}) begin
                $display("FAIL mismatch row=%0d", row);
                $display(" actual   cmd=%0d good=%0d mid=%0d catch=%0d blind=%0d a=%0d ad=%0d e=%0d ref=%0d v=%0d x=%0d,%0d,%0d,%0d",
                    command, good, blind_mid, catch_ok, blind, est_a, est_ad, energy, ref_q16, theta_speed, x0, x1, x2, x3);
                $display(" expected cmd=%0d good=%0d mid=%0d catch=%0d blind=%0d a=%0d ad=%0d e=%0d ref=%0d v=%0d x=%0d,%0d,%0d,%0d",
                    $signed(e_command), e_good, e_mid, e_catch, e_blind, $signed(e_a), $signed(e_ad),
                    $signed(e_energy), $signed(e_ref), $signed(e_speed), $signed(e0), $signed(e1),
                    $signed(e2), $signed(e3));
                $finish;
            end
            held = observed;
        end
        $fclose(file);
        $display("PASS tb_swing_core rows=%0d max_clocks=%0d", row, max_wait);
        $finish;
    end
    initial begin #2000000000; $display("FAIL watchdog"); $finish; end
endmodule
