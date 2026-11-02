`timescale 1ns/1ps
// Phase-dependent arm torque bias (2026-10-28 night, v2 stronger).
// The stop-state arm limit cycle is suppressed when the controller keeps u at
// ~7% (gear backlash on one face). v1 (7% cap) helped phi 257/522 but left
// phi 1013 at 9-11 counts, so the worst sector now gets 11-12%. Indexed by
// position[9:0] (mod 1024; ~5.5 deg phase slip, acceptable for a slow bias).
module j280_phase_lut16(
    input wire [9:0] pos,
    output reg signed [25:0] ff_q16
);
    wire [3:0] idx = pos[9:6];
    always @* begin
        case (idx)
            4'd0:  ff_q16 = 26'sd11468800; // phi   32, +175.0 compare, +7.00%
            4'd1:  ff_q16 = 26'sd11468800; // phi   97, +175.0 compare, +7.00%
            4'd2:  ff_q16 = 26'sd0;        // phi  162,   +0.0 compare,  0.00%
            4'd3:  ff_q16 = 26'sd5160960;  // phi  227,  +78.8 compare, +3.15%
            4'd4:  ff_q16 = 26'sd3440640;  // phi  292,  +52.5 compare, +2.10%
            4'd5:  ff_q16 = 26'sd1720320;  // phi  357,  +26.2 compare, +1.05%
            4'd6:  ff_q16 = 26'sd0;        // phi  422,   +0.0 compare,  0.00%
            4'd7:  ff_q16 = 26'sd0;        // phi  487,   +0.0 compare,  0.00%
            4'd8:  ff_q16 = 26'sd1720320;  // phi  552,  +26.2 compare, +1.05%
            4'd9:  ff_q16 = 26'sd5160960;  // phi  617,  +78.8 compare, +3.15%
            4'd10: ff_q16 = 26'sd15564800; // phi  682, +237.5 compare, +9.50%
            4'd11: ff_q16 = 26'sd15564800; // phi  747, +237.5 compare, +9.50%
            4'd12: ff_q16 = 26'sd13107200; // phi  812, +200.0 compare, +8.00%
            4'd13: ff_q16 = 26'sd18022400; // phi  877, +275.0 compare, +11.00%
            4'd14: ff_q16 = 26'sd19660800; // phi  942, +300.0 compare, +12.00%
            default: ff_q16 = 26'sd18022400; // phi 1007, +275.0 compare, +11.00%
        endcase
    end
endmodule
