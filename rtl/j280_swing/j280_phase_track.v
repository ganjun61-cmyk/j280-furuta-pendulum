`timescale 1ns/1ps
// Exact arm-phase tracker: on every 500 Hz snapshot, compute position mod
// 1040 with a 32-step shift/subtract remainder (0.64 us at 50 MHz, far below
// the 2 ms sample period). No delta/prev state, so no phase drift or sample
// update bug; the phase output is exact for each sample.
module j280_phase_track(
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] position,
    input wire sample,
    output reg [10:0] phase
);
    reg busy;
    reg [5:0] step;
    reg [31:0] work, rem;
    reg neg;

    wire [31:0] abs_pos = position[31] ? (~position + 32'd1) : position;
    wire bit_in = work[31];
    wire [32:0] shifted = {rem, bit_in};
    wire ge = shifted >= 33'd1040;
    wire [31:0] rem_next = ge ? (shifted - 33'd1040) : shifted[31:0];
    wire [31:0] phase_next = neg ? (rem_next == 0 ? 32'd0 : 32'd1040 - rem_next)
                                 : rem_next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; step <= 0; work <= 0; rem <= 0; neg <= 0; phase <= 0;
        end else if (!busy && sample) begin
            busy <= 1; step <= 0; work <= abs_pos; rem <= 0; neg <= position[31];
        end else if (busy) begin
            work <= {work[30:0], 1'b0};
            rem <= rem_next;
            if (step == 6'd31) begin
                phase <= phase_next[10:0]; busy <= 0;
            end else begin
                step <= step + 6'd1;
            end
        end
    end
endmodule
