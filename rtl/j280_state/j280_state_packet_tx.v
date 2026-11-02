`timescale 1ns/1ps

// A5 5A, PAYLOAD_BYTES immutable payload octets, CRC16-CCITT-FALSE over the
// payload (low byte first). A request while busy is dropped (never queued) and
// sets the sticky overflow flag until reset.
module j280_state_packet_tx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 230400,
    parameter integer PAYLOAD_BYTES = 124
) (
    input wire clk,
    input wire rst_n,
    input wire packet_valid,
    input wire [PAYLOAD_BYTES*8-1:0] payload,
    output wire uart_tx,
    output reg overflow,
    output wire busy
);
    localparam integer FRAME_BYTES = PAYLOAD_BYTES + 4;
    localparam [7:0] LAST_PAYLOAD = PAYLOAD_BYTES - 1;
    localparam [7:0] CRC_LOW = PAYLOAD_BYTES + 2;
    localparam [7:0] CRC_HIGH = PAYLOAD_BYTES + 3;
    localparam [2:0] IDLE = 3'd0, CRC = 3'd1, LAUNCH = 3'd2, WAIT_BUSY = 3'd3, WAIT_DONE = 3'd4;

    reg [2:0] state;
    reg [PAYLOAD_BYTES*8-1:0] payload_latched;
    reg [15:0] crc;
    reg [7:0] crc_index;
    reg [7:0] byte_index;
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

    wire [7:0] payload_index = (byte_index - 8'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            payload_latched <= {PAYLOAD_BYTES*8{1'b0}};
            crc <= 16'hffff;
            crc_index <= 8'd0;
            byte_index <= 8'd0;
            uart_data <= 8'h00;
            uart_start <= 1'b0;
            overflow <= 1'b0;
        end else begin
            uart_start <= 1'b0;
            if (packet_valid && (state != IDLE))
                overflow <= 1'b1;
            case (state)
                IDLE: begin
                    if (packet_valid) begin
                        payload_latched <= payload;
                        crc <= 16'hffff;
                        crc_index <= 8'd0;
                        byte_index <= 8'd0;
                        state <= CRC;
                    end
                end
                CRC: begin
                    crc <= crc16_byte(crc, payload_latched[crc_index * 8 +: 8]);
                    if (crc_index == LAST_PAYLOAD)
                        state <= LAUNCH;
                    else
                        crc_index <= crc_index + 1'b1;
                end
                LAUNCH: begin
                    if (!uart_busy) begin
                        if (byte_index == 8'd0)
                            uart_data <= 8'ha5;
                        else if (byte_index == 8'd1)
                            uart_data <= 8'h5a;
                        else if (byte_index == CRC_LOW)
                            uart_data <= crc[7:0];
                        else if (byte_index == CRC_HIGH)
                            uart_data <= crc[15:8];
                        else
                            uart_data <= payload_latched[payload_index * 8 +: 8];
                        uart_start <= 1'b1;
                        state <= WAIT_BUSY;
                    end
                end
                WAIT_BUSY: begin
                    if (uart_busy)
                        state <= WAIT_DONE;
                end
                WAIT_DONE: begin
                    if (!uart_busy) begin
                        if (byte_index == CRC_HIGH)
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
