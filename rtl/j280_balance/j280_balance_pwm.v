`timescale 1ns/1ps

// Continuous signed PWM for the B-stage bridge, with generic top-level pins.
// Positive commands select IN2; negative commands select IN1. Stop is PWM=1
// with both direction pins low. An active zero is a direction-retaining brake.
module j280_balance_pwm #(
    parameter integer PERIOD = 2500,
    parameter integer MAX_COMPARE = 1000
) (
    input wire clk,
    input wire rst_n,
    input wire allow,
    input wire update,
    input wire signed [15:0] signed_command,
    output wire pwm,
    output wire in1,
    output wire in2,
    output wire active,
    output wire [15:0] applied_compare,
    output wire [1:0] applied_direction,
    output reg reject
);
    function integer carrier_width;
        input integer period_value;
        integer remaining;
        begin
            carrier_width = 1;
            remaining = period_value - 1;
            while (remaining > 1) begin
                carrier_width = carrier_width + 1;
                remaining = remaining >> 1;
            end
        end
    endfunction

    // Extend before negating so that -32768 becomes unsigned +32768, never
    // a negative int16 value that could pass the magnitude limit check.
    function [16:0] magnitude;
        input signed [15:0] value;
        reg signed [16:0] extended;
        begin
            extended = {value[15], value};
            if (value[15]) magnitude = -extended;
            else magnitude = extended;
        end
    endfunction

    localparam integer COUNTER_WIDTH = carrier_width(PERIOD);
    localparam [COUNTER_WIDTH-1:0] LAST = PERIOD - 1;
    localparam [16:0] COMPARE_LIMIT = MAX_COMPARE;
    localparam [1:0] POSITIVE = 2'd2;
    localparam [1:0] NEGATIVE = 2'd1;

    reg [COUNTER_WIDTH-1:0] counter;
    reg active_reg;
    reg [15:0] compare_reg;
    reg [1:0] direction_reg;
    reg signed [15:0] pending_command;
    reg pending_valid;
    reg blanking;

    wire boundary = (counter == LAST);
    wire signed [15:0] next_command = update ? signed_command : pending_command;
    wire [16:0] next_magnitude = magnitude(next_command);
    wire [1:0] next_direction = next_command[15] ? NEGATIVE : POSITIVE;

    // Protection bypasses the carrier. The state itself is flushed on the
    // next clock with allow low, so rearming requires a fresh valid update.
    assign active = rst_n && allow && !reject && active_reg;
    assign applied_compare = active ? compare_reg : 16'd0;
    assign applied_direction = active ? direction_reg : 2'd0;
    assign in1 = active && (direction_reg == NEGATIVE);
    assign in2 = active && (direction_reg == POSITIVE);
    assign pwm = active ? (counter < compare_reg) : 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            active_reg <= 0;
            compare_reg <= 0;
            direction_reg <= POSITIVE;
            pending_command <= 0;
            pending_valid <= 0;
            blanking <= 0;
            reject <= 0;
        end else begin
            // Never pause or restart the carrier on allow, update or reject.
            if (boundary) counter <= 0;
            else counter <= counter + 1'b1;

            if (!allow) begin
                active_reg <= 0;
                compare_reg <= 0;
                pending_command <= 0;
                pending_valid <= 0;
                blanking <= 0;
                reject <= 0;
                // Retain the last actually applied nonzero direction; it is
                // hidden while inactive and used for a later zero command.
            end else if (!reject) begin
                if (update && magnitude(signed_command) > COMPARE_LIMIT) begin
                    reject <= 1;
                    active_reg <= 0;
                    compare_reg <= 0;
                    pending_command <= 0;
                    pending_valid <= 0;
                    blanking <= 0;
                end else if (boundary) begin
                    if (update || pending_valid) begin
                        if (active_reg && !blanking && next_magnitude != 0 &&
                            next_direction != direction_reg) begin
                            // Keep the old direction and brake for this entire
                            // period. Save the target even with no later update.
                            compare_reg <= 0;
                            blanking <= 1;
                            pending_command <= next_command;
                            pending_valid <= 1;
                        end else begin
                            // A full blank has elapsed, or no reversal is needed.
                            // A same-edge update always beats the pending target.
                            active_reg <= 1;
                            compare_reg <= next_magnitude[15:0];
                            if (next_magnitude != 0) direction_reg <= next_direction;
                            blanking <= 0;
                            pending_valid <= 0;
                        end
                    end
                end else if (update) begin
                    // Also replace targets during blanking; only its ending
                    // boundary can apply them, so no update shortens the blank.
                    pending_command <= signed_command;
                    pending_valid <= 1;
                end
            end
        end
    end
endmodule
