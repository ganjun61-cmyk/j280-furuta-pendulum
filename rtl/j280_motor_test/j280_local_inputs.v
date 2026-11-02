`timescale 1ns/1ps

// Core-board keys (pressed = low) and DIP switch 1.
// SW2 stop and DIP-off act on the synchronized level without debounce; only
// the actions that permit motion (SW1 start, DIP on) are debounced. The
// debounced states reset to "pressed"/"on" so a key held or a switch left on
// through reset never produces a start event or counts as a fresh off->on.
module j280_local_inputs #(
    parameter integer DEBOUNCE_CLOCKS = 1000000,
    parameter integer ARM_ACTIVE_LOW = 1
) (
    input wire clk,
    input wire rst_n,
    input wire sw1_n,
    input wire sw2_n,
    input wire dip1,
    output reg start_event,
    output wire stop_pressed,
    output wire arm_on_raw,
    output reg arm_on_stable,
    output wire keys_released,
    output wire [2:0] levels
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0] sync;
    reg sw1_pressed_stable;
    reg [31:0] sw1_count, arm_count;
    wire sw1_pressed_raw = !sync[0];
    assign stop_pressed = !sync[1];
    assign arm_on_raw = (ARM_ACTIVE_LOW != 0) ? !sync[2] : sync[2];
    assign keys_released = sync[0] && sync[1];
    assign levels = sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            meta <= 3'b111; sync <= 3'b111;
            sw1_pressed_stable <= 1'b1; arm_on_stable <= 1'b1;
            sw1_count <= 32'd0; arm_count <= 32'd0; start_event <= 1'b0;
        end else begin
            meta <= {dip1, sw2_n, sw1_n};
            sync <= meta;
            start_event <= 1'b0;
            if (sw1_pressed_raw == sw1_pressed_stable) sw1_count <= 32'd0;
            else if (sw1_count == DEBOUNCE_CLOCKS - 1) begin
                sw1_count <= 32'd0;
                sw1_pressed_stable <= sw1_pressed_raw;
                start_event <= sw1_pressed_raw;
            end else sw1_count <= sw1_count + 32'd1;
            if (arm_on_raw == arm_on_stable) arm_count <= 32'd0;
            else if (arm_count == DEBOUNCE_CLOCKS - 1) begin
                arm_count <= 32'd0;
                arm_on_stable <= arm_on_raw;
            end else arm_count <= arm_count + 32'd1;
        end
    end
endmodule
