`timescale 1ns/1ps
// One-revolution arm-phase torque feedforward for the settled/rest state.
// 2026-10-28 v2: calibrated from the STATIC 1st harmonic of rest segments,
// not from constant-speed DIP2 rotation. Per-run fits (985/963/971-976,
// 10-26..10-28) give alpha 0.62-0.81 deg and u 0.25-0.47% with the peak at
// phi ~= 300-440 count (mean 365); 0.40% peak is the table amplitude here.
// The old 1.33% table was a constant-speed / motion harmonic and over-
// compensated the stop state by ~3x.
// The phase input is the exact mod-1040 tracker output; 16 bins of 65 counts,
// idx = (phase * 1008) >> 16 (1008/65536 ~= 16/1040). The table is evaluated
// at the 1040-phase bin centers (32, 97, ..., 1007). The measured u harmonic
// is the torque the controller supplies, so the feedforward has the same sign.
module j280_phase_lut(
    input wire clk,
    input wire rst_n,
    input wire [10:0] phase,
    output reg signed [23:0] ff_q16
);
    wire [20:0] phase_ext = {10'd0, phase};
    wire [20:0] prod = (phase_ext << 10) - (phase_ext << 4);
    wire [3:0] idx = prod[19:16];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ff_q16 <= 24'sd0;
        else case (idx)
            4'd0:  ff_q16 <= -24'sd279755;  // phi   32,  -4.27 compare, -0.171%
            4'd1:  ff_q16 <= -24'sd31663;   // phi   97,  -0.48 compare, -0.019%
            4'd2:  ff_q16 <= 24'sd221250;   // phi  162,  +3.38 compare, +0.135%
            4'd3:  ff_q16 <= 24'sd440479;   // phi  227,  +6.72 compare, +0.269%
            4'd4:  ff_q16 <= 24'sd592650;   // phi  292,  +9.04 compare, +0.362%
            4'd5:  ff_q16 <= 24'sd654595;   // phi  357,  +9.99 compare, +0.400%
            4'd6:  ff_q16 <= 24'sd616883;   // phi  422,  +9.41 compare, +0.377%
            4'd7:  ff_q16 <= 24'sd485257;   // phi  487,  +7.40 compare, +0.296%
            4'd8:  ff_q16 <= 24'sd279755;   // phi  552,  +4.27 compare, +0.171%
            4'd9:  ff_q16 <= 24'sd31663;    // phi  617,  +0.48 compare, +0.019%
            4'd10: ff_q16 <= -24'sd221250;  // phi  682,  -3.38 compare, -0.135%
            4'd11: ff_q16 <= -24'sd440479;  // phi  747,  -6.72 compare, -0.269%
            4'd12: ff_q16 <= -24'sd592650;  // phi  812,  -9.04 compare, -0.362%
            4'd13: ff_q16 <= -24'sd654595;  // phi  877,  -9.99 compare, -0.400%
            4'd14: ff_q16 <= -24'sd616883;  // phi  942,  -9.41 compare, -0.377%
            default: ff_q16 <= -24'sd485257; // phi 1007,  -7.40 compare, -0.296%
        endcase
    end
endmodule
