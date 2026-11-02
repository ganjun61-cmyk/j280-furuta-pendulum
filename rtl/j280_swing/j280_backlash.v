`timescale 1ns/1ps

// Backlash compensation, bit-exact with scripts/j280_backlash.py.
// Tracker: arm estimate follows the motor-side encoder only when the motor
// presses one side of the gap; the balance core measures the estimate.
module j280_backlash_tracker (
    input wire clk,
    input wire rst_n,
    input wire sample,
    input wire balancing,
    input wire signed [31:0] pos,
    input wire [3:0] half,
    output reg signed [31:0] arm,
    output reg signed [4:0] offset,     // pos - arm, one clock after arm (timing)
    output reg valid
);
    reg signed [31:0] pos_q;
    wire signed [33:0] p = {{2{pos[31]}}, pos};
    wire signed [33:0] h = {30'd0, half};
    wire signed [33:0] a = {{2{arm[31]}}, arm};
    wire signed [33:0] lo = p - h, hi = p + h;
    wire signed [33:0] c1 = (a < lo) ? lo : a;
    wire signed [33:0] c2 = (c1 > hi) ? hi : c1;
    wire signed [33:0] sat = (c2 > 34'sd2147483647) ? 34'sd2147483647 :
                             (c2 < -34'sd2147483648) ? -34'sd2147483648 : c2;
    wire signed [32:0] diff = {pos_q[31], pos_q} - {arm[31], arm};
    reg offset_due;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arm <= 0; offset <= 0; valid <= 0; pos_q <= 0; offset_due <= 0;
        end else begin
            valid <= sample;
            offset_due <= sample;
            if (sample) begin
                pos_q <= pos;
                if (!balancing || half == 4'd0) arm <= pos;
                else arm <= sat[31:0];
            end
            if (offset_due) offset <= diff[4:0];
        end
    end
endmodule

// Gap boost: extra drive in the command direction while the motor has not yet
// reached that side of the gap. The core command (and model input) excludes it.
module j280_gap_boost (
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire signed [15:0] command,
    input wire signed [4:0] offset,
    input wire [3:0] half,
    input wire [7:0] gap_boost,
    input wire [7:0] threshold,
    input wire [15:0] max_compare,
    output reg valid,
    output reg signed [15:0] command_out,
    output reg boosted
);
    wire signed [17:0] c = {{2{command[15]}}, command};
    wire [16:0] mag = command[15] ? (17'd0 - {command[15], command}) : {1'b0, command};
    wire [7:0] thr = (threshold == 8'd0) ? 8'd1 : threshold;
    wire enable = (half != 4'd0) && (gap_boost != 8'd0) && (mag >= {9'd0, thr});
    wire signed [5:0] h = {2'd0, half};
    wire up = enable && !command[15] && (command != 16'sd0) && ($signed({offset[4], offset}) < h);
    wire down = enable && command[15] && ($signed({offset[4], offset}) > -h);
    wire signed [17:0] b = {10'd0, gap_boost};
    wire signed [17:0] m = {2'd0, max_compare};
    wire signed [17:0] plus = c + b, minus = c - b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= 0; command_out <= 0; boosted <= 0;
        end else begin
            valid <= valid_in;
            if (valid_in) begin
                boosted <= up || down;
                if (up) command_out <= (plus > m) ? max_compare : plus[15:0];
                else if (down) command_out <= (minus < -m) ? -max_compare : minus[15:0];
                else command_out <= command;
            end
        end
    end
endmodule
