`timescale 1ns/1ps
// Self-checking: glitch rejection, fast real edges, reset with lines high.
module tb_encoder_filtered;
    parameter integer FILTER = 100;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0, a = 0, b = 0;
    wire signed [31:0] position;
    wire [31:0] illegal;
    wire ready;
    j280_encoder_filtered #(.FILTER_CLOCKS(FILTER)) dut(.clk(clk), .rst_n(rst_n), .enc_a(a), .enc_b(b),
        .position(position), .illegal_count(illegal), .ready(ready));
    integer i;

    task step_forward;   // 00 -> 10 -> 11 -> 01 -> 00, quarter period in ns
        input integer quarter;
        begin
            a = 1; #(quarter); b = 1; #(quarter); a = 0; #(quarter); b = 0; #(quarter);
        end
    endtask
    task step_backward;
        input integer quarter;
        begin
            b = 1; #(quarter); a = 1; #(quarter); b = 0; #(quarter); a = 0; #(quarter);
        end
    endtask
    task expect;
        input signed [31:0] pos;
        input [31:0] ill;
        begin
            #(30 * FILTER);
            if (position !== pos || illegal !== ill) begin
                $display("FAIL position=%0d (want %0d) illegal=%0d (want %0d) at %0t", position, pos, illegal, ill, $time);
                $finish;
            end
        end
    endtask

    initial begin
        #95 rst_n = 1;
        wait (ready);
        expect(0, 0);
        // Common-mode glitches on both lines, shorter than the filter: no count, no illegal.
        for (i = 0; i < 200; i = i + 1) begin
            a = 1; b = 1; #(20 * FILTER - 40); a = 0; b = 0; #(500);
        end
        expect(0, 0);
        // Single-line glitch bursts: no count.
        for (i = 0; i < 50; i = i + 1) begin a = 1; #(400); a = 0; #(300); b = 1; #(900); b = 0; #(2000); end
        expect(0, 0);
        // Real motion: 100 us per count (10000 count/s) and slower.
        for (i = 0; i < 20; i = i + 1) step_forward(400000);
        expect(80, 0);
        for (i = 0; i < 30; i = i + 1) step_forward(100000);
        expect(200, 0);
        for (i = 0; i < 60; i = i + 1) step_backward(100000);
        expect(-40, 0);
        // Glitches riding on real motion still do not count or flag.
        for (i = 0; i < 10; i = i + 1) begin
            a = 1; #(12000); b = ~b; #(300); b = ~b; #(13000); b = 1; #(25000); a = 0; #(25000); b = 0; #(25000);
        end
        expect(0, 0);
        // A genuine simultaneous change held longer than the filter is still reported.
        a = 1; b = 1; #(40 * FILTER);
        expect(0, 1);
        a = 0; b = 0; #(40 * FILTER);
        expect(0, 2);
        // Reset with both lines high: ready without a false count.
        rst_n = 0; a = 1; b = 1; #200; rst_n = 1;
        wait (ready);
        expect(0, 0);
        a = 0; #(25000); b = 0; #(25000);   // 11 -> 01 -> 00 is +2
        expect(2, 0);
        $display("PASS tb_encoder_filtered");
        $finish;
    end
    initial begin #200000000; $display("FAIL watchdog"); $finish; end
endmodule
