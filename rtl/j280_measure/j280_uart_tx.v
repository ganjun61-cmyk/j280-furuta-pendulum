`timescale 1ns/1ps

// One-byte 8N1 transmitter. Requests received while busy are discarded.
module j280_uart_tx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 230400
) (
    input wire clk,
    input wire rst_n,
    input wire [7:0] data,
    input wire start,
    output reg tx,
    output wire busy
);
    // Round to the nearest whole clock: 50 MHz / 230400 -> 217 clocks.
    localparam integer ROUNDED_TICKS = (CLOCK_HZ + (BAUD / 2)) / BAUD;
    localparam integer BIT_TICKS = (ROUNDED_TICKS < 1) ? 1 : ROUNDED_TICKS;

    function integer counter_width;
        input integer value;
        integer remaining;
        begin
            remaining = value - 1;
            counter_width = 0;
            while (remaining > 0) begin
                counter_width = counter_width + 1;
                remaining = remaining >> 1;
            end
            if (counter_width < 1)
                counter_width = 1;
        end
    endfunction

    localparam integer TICK_WIDTH = counter_width(BIT_TICKS);
    reg [TICK_WIDTH-1:0] tick_count;
    reg [9:0] shift_register;
    reg [3:0] bit_index;
    reg active;
    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx <= 1'b1;
            active <= 1'b0;
            tick_count <= {TICK_WIDTH{1'b0}};
            bit_index <= 4'd0;
            shift_register <= 10'h3ff;
        end else if (!active) begin
            tx <= 1'b1;
            if (start) begin
                shift_register <= {1'b1, data, 1'b0};
                tx <= 1'b0;
                active <= 1'b1;
                tick_count <= {TICK_WIDTH{1'b0}};
                bit_index <= 4'd0;
            end
        end else if (tick_count == BIT_TICKS - 1) begin
            tick_count <= {TICK_WIDTH{1'b0}};
            if (bit_index == 4'd9) begin
                active <= 1'b0;
                tx <= 1'b1;
            end else begin
                shift_register <= {1'b1, shift_register[9:1]};
                tx <= shift_register[1];
                bit_index <= bit_index + 1'b1;
            end
        end else begin
            tick_count <= tick_count + 1'b1;
        end
    end
endmodule
