`timescale 1ns/1ps
// v6 core-board SW3/SW4 (pressed = low) and DIP switch 2, synchronized and
// debounced. Press events fire once per debounced press; the debounced states
// reset to "pressed"/"on", so a key held or a switch left on through reset
// never produces an event or counts as on until it has been released/off.
module j280_aux_inputs #(
    parameter integer DEBOUNCE_CLOCKS = 1000000,
    parameter integer DIP_ACTIVE_LOW = 1
) (
    input wire clk, rst_n,
    input wire sw3_n, sw4_n, dip2,
    output reg sw3_event, sw4_event,
    output reg dip2_on,
    output wire [2:0] levels
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0] sync;
    reg sw3_stable, sw4_stable, dip2_seen_off;
    reg [31:0] c3, c4, cd;
    wire sw3_raw = !sync[0], sw4_raw = !sync[1];
    wire dip2_raw = (DIP_ACTIVE_LOW != 0) ? !sync[2] : sync[2];
    reg dip2_stable;
    assign levels = sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            meta <= 3'b111; sync <= 3'b111;
            sw3_stable <= 1'b1; sw4_stable <= 1'b1; dip2_stable <= 1'b1; dip2_seen_off <= 1'b0;
            c3 <= 0; c4 <= 0; cd <= 0;
            sw3_event <= 0; sw4_event <= 0; dip2_on <= 0;
        end else begin
            meta <= {dip2, sw4_n, sw3_n};
            sync <= meta;
            sw3_event <= 0; sw4_event <= 0;
            if (sw3_raw == sw3_stable) c3 <= 0;
            else if (c3 == DEBOUNCE_CLOCKS - 1) begin
                c3 <= 0; sw3_stable <= sw3_raw; sw3_event <= sw3_raw;
            end else c3 <= c3 + 32'd1;
            if (sw4_raw == sw4_stable) c4 <= 0;
            else if (c4 == DEBOUNCE_CLOCKS - 1) begin
                c4 <= 0; sw4_stable <= sw4_raw; sw4_event <= sw4_raw;
            end else c4 <= c4 + 32'd1;
            if (dip2_raw == dip2_stable) cd <= 0;
            else if (cd == DEBOUNCE_CLOCKS - 1) begin
                cd <= 0; dip2_stable <= dip2_raw;
            end else cd <= cd + 32'd1;
            if (!dip2_stable) dip2_seen_off <= 1'b1;
            // Off acts at once on the synchronized level; on needs a debounced
            // on after an off has been seen since reset.
            dip2_on <= dip2_raw && dip2_stable && dip2_seen_off;
        end
    end
endmodule
