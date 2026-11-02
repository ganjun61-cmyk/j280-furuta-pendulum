`timescale 1ns/1ps

// One-byte 8N1 receiver, mid-bit sampling after a two-flop synchronizer.
// A start bit shorter than half a bit is ignored. A low stop bit reports
// frame_error and waits for the line to return high before the next byte.
module j280_uart_rx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 230400
) (
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] data,
    output reg valid,
    output reg frame_error
);
    localparam integer ROUNDED_TICKS = (CLOCK_HZ + (BAUD / 2)) / BAUD;
    localparam integer BIT_TICKS = (ROUNDED_TICKS < 2) ? 2 : ROUNDED_TICKS;
    localparam integer HALF_TICKS = BIT_TICKS / 2;
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    (* ASYNC_REG = "TRUE" *) reg rx_meta;
    (* ASYNC_REG = "TRUE" *) reg rx_sync;
    reg [1:0] state;
    reg [15:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    reg wait_high;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1; rx_sync <= 1'b1;
            state <= IDLE; count <= 16'd0; bit_index <= 3'd0; shift <= 8'd0;
            wait_high <= 1'b0; data <= 8'd0; valid <= 1'b0; frame_error <= 1'b0;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            valid <= 1'b0;
            frame_error <= 1'b0;
            case (state)
                IDLE: begin
                    if (wait_high) begin
                        if (rx_sync) wait_high <= 1'b0;
                    end else if (!rx_sync) begin
                        state <= START;
                        count <= 16'd0;
                    end
                end
                START: begin
                    if (count == HALF_TICKS - 1) begin
                        count <= 16'd0;
                        bit_index <= 3'd0;
                        state <= rx_sync ? IDLE : DATA;
                    end else count <= count + 16'd1;
                end
                DATA: begin
                    if (count == BIT_TICKS - 1) begin
                        count <= 16'd0;
                        shift <= {rx_sync, shift[7:1]};
                        if (bit_index == 3'd7) state <= STOP;
                        else bit_index <= bit_index + 3'd1;
                    end else count <= count + 16'd1;
                end
                STOP: begin
                    if (count == BIT_TICKS - 1) begin
                        count <= 16'd0;
                        state <= IDLE;
                        if (rx_sync) begin
                            data <= shift;
                            valid <= 1'b1;
                        end else begin
                            frame_error <= 1'b1;
                            wait_high <= 1'b1;
                        end
                    end else count <= count + 16'd1;
                end
            endcase
        end
    end
endmodule
