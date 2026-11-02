`timescale 1ns/1ps

// A5 5A, 28 immutable payload octets, CRC16-CCITT-FALSE (low byte first).
module j280_frame_tx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 230400
) (
    input wire clk,
    input wire rst_n,
    input wire snapshot_valid,
    input wire [223:0] payload,
    output wire uart_tx,
    output reg overflow,
    output wire busy
);
    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] CRC = 3'd1;
    localparam [2:0] LAUNCH = 3'd2;
    localparam [2:0] WAIT_BUSY = 3'd3;
    localparam [2:0] WAIT_DONE = 3'd4;
    reg [2:0] state;
    reg [223:0] payload_latched;
    reg [15:0] crc;
    reg [4:0] crc_index;
    reg [5:0] byte_index;
    reg [7:0] uart_data;
    reg uart_start;
    wire uart_busy;
    assign busy = (state != IDLE);

    function [15:0] crc16_byte;
        input [15:0] previous_crc;
        input [7:0] value;
        reg [15:0] work;
        integer bit_number;
        begin
            work = previous_crc ^ {value, 8'h00};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                if (work[15])
                    work = (work << 1) ^ 16'h1021;
                else
                    work = work << 1;
            end
            crc16_byte = work;
        end
    endfunction

    j280_uart_tx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) transmitter (
        .clk(clk), .rst_n(rst_n), .data(uart_data), .start(uart_start),
        .tx(uart_tx), .busy(uart_busy));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            payload_latched <= 224'd0;
            crc <= 16'hffff;
            crc_index <= 5'd0;
            byte_index <= 6'd0;
            uart_data <= 8'h00;
            uart_start <= 1'b0;
            overflow <= 1'b0;
        end else begin
            uart_start <= 1'b0;
            if (snapshot_valid && (state != IDLE))
                overflow <= 1'b1;
            case (state)
                IDLE: begin
                    if (snapshot_valid) begin
                        // CRC and UART both consume only this captured copy.
                        payload_latched <= payload;
                        crc <= 16'hffff;
                        crc_index <= 5'd0;
                        byte_index <= 6'd0;
                        state <= CRC;
                    end
                end
                CRC: begin
                    crc <= crc16_byte(crc, payload_latched[crc_index * 8 +: 8]);
                    if (crc_index == 5'd27)
                        state <= LAUNCH;
                    else
                        crc_index <= crc_index + 1'b1;
                end
                LAUNCH: begin
                    if (!uart_busy) begin
                        case (byte_index)
                            6'd0: uart_data <= 8'ha5;
                            6'd1: uart_data <= 8'h5a;
                            6'd30: uart_data <= crc[7:0];
                            6'd31: uart_data <= crc[15:8];
                            default: uart_data <= payload_latched[(byte_index - 6'd2) * 8 +: 8];
                        endcase
                        uart_start <= 1'b1;
                        state <= WAIT_BUSY;
                    end
                end
                // Wait for the transmitter to observe the registered start.
                WAIT_BUSY: begin
                    if (uart_busy)
                        state <= WAIT_DONE;
                end
                WAIT_DONE: begin
                    if (!uart_busy) begin
                        if (byte_index == 6'd31)
                            state <= IDLE;
                        else begin
                            byte_index <= byte_index + 1'b1;
                            state <= LAUNCH;
                        end
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
