`timescale 1ns/1ps

// One bounded constant-duty pulse for one TB6612 channel.
// Idle pins are IN1=IN2=0, PWM=1 (datasheet "Stop"). A validated start begins
// at the next PWM period boundary and lasts exactly `periods` periods; within
// a period PWM is high for `compare` clocks. allow=0 forces idle pins on the
// next clock edge regardless of the period phase. Pins are registered.
module j280_pwm_pulse #(
    parameter integer PERIOD = 2500,
    parameter integer MAX_COMPARE = 1000,
    parameter integer MAX_PERIODS = 40000
) (
    input wire clk,
    input wire rst_n,
    input wire allow,
    input wire start,
    input wire [1:0] direction,
    input wire [15:0] compare,
    input wire [31:0] periods,
    output reg pwm,
    output reg in1,
    output reg in2,
    output reg driving,
    output reg [15:0] drive_compare,
    output reg done,
    output reg reject
);
    localparam CONFIG_OK = (PERIOD >= 2) && (PERIOD <= 65535) && (MAX_COMPARE >= 1) &&
                           (MAX_COMPARE < PERIOD) && (MAX_PERIODS >= 1);
    localparam [15:0] LAST = PERIOD - 1;
    localparam [15:0] COMPARE_LIMIT = CONFIG_OK ? MAX_COMPARE : 0;
    localparam [31:0] PERIOD_LIMIT = CONFIG_OK ? MAX_PERIODS : 0;

    reg [15:0] count;
    wire wrap = (count == LAST);
    reg pending, active;
    reg [1:0] p_direction, a_direction;
    reg [15:0] p_compare, a_compare;
    reg [31:0] p_periods, remaining;
    wire start_ok = (direction == 2'd1 || direction == 2'd2) && (compare != 16'd0) &&
                    (compare <= COMPARE_LIMIT) && (periods != 32'd0) && (periods <= PERIOD_LIMIT);
    wire drive_next = active && allow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 16'd0; pending <= 1'b0; active <= 1'b0;
            p_direction <= 2'd0; a_direction <= 2'd0; p_compare <= 16'd0; a_compare <= 16'd0;
            p_periods <= 32'd0; remaining <= 32'd0;
            pwm <= 1'b1; in1 <= 1'b0; in2 <= 1'b0; driving <= 1'b0; drive_compare <= 16'd0;
            done <= 1'b0; reject <= 1'b0;
        end else begin
            count <= wrap ? 16'd0 : count + 16'd1;
            done <= 1'b0;
            reject <= 1'b0;
            if (!allow) begin
                pending <= 1'b0;
                active <= 1'b0;
            end else begin
                if (start && !pending && !active) begin
                    if (start_ok) begin
                        pending <= 1'b1;
                        p_direction <= direction; p_compare <= compare; p_periods <= periods;
                    end else reject <= 1'b1;
                end
                if (pending && wrap) begin
                    pending <= 1'b0;
                    active <= 1'b1;
                    a_direction <= p_direction; a_compare <= p_compare; remaining <= p_periods;
                end else if (active && wrap) begin
                    if (remaining == 32'd1) begin
                        active <= 1'b0;
                        done <= 1'b1;
                    end else remaining <= remaining - 32'd1;
                end
            end
            driving <= drive_next;
            drive_compare <= drive_next ? a_compare : 16'd0;
            pwm <= drive_next ? (count < a_compare) : 1'b1;
            in1 <= drive_next && (a_direction == 2'd1);
            in2 <= drive_next && (a_direction == 2'd2);
        end
    end
endmodule
