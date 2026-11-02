`timescale 1ns/1ps

// 20-byte command frame: A5 5A, 16 payload octets (version, command, fields),
// CRC16-CCITT-FALSE over the payload, low byte first. frame_payload byte 0 is
// frame byte 2 and is only meaningful during the frame_valid cycle.
// Framing errors, inter-byte gaps, CRC and version mismatches drop the frame.
module j280_cmd_rx #(
    parameter integer INTERBYTE_CLOCKS = 50000,
    parameter [7:0] VERSION = 8'd3
) (
    input wire clk,
    input wire rst_n,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire byte_error,
    output reg frame_valid,
    output reg [127:0] frame_payload,
    output reg [15:0] frame_crc,
    output reg frame_error
);
    localparam [1:0] HUNT0 = 2'd0, HUNT1 = 2'd1, BODY = 2'd2, CHECK = 2'd3;

    function [15:0] crc16_byte;
        input [15:0] previous_crc;
        input [7:0] value;
        reg [15:0] work;
        integer bit_number;
        begin
            work = previous_crc ^ {value, 8'h00};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                if (work[15]) work = (work << 1) ^ 16'h1021;
                else work = work << 1;
            end
            crc16_byte = work;
        end
    endfunction

    reg [1:0] state;
    reg [4:0] index;
    reg [15:0] crc;
    reg [31:0] gap;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= HUNT0; index <= 5'd0; crc <= 16'hffff; gap <= 32'd0;
            frame_valid <= 1'b0; frame_payload <= 128'd0; frame_crc <= 16'd0;
            frame_error <= 1'b0;
        end else begin
            frame_valid <= 1'b0;
            frame_error <= 1'b0;
            if (byte_valid || state == HUNT0 || state == CHECK) gap <= 32'd0;
            else if (gap != INTERBYTE_CLOCKS) gap <= gap + 32'd1;

            if (byte_error) begin
                frame_error <= 1'b1;
                state <= HUNT0;
            end else if ((state == HUNT1 || state == BODY) && gap == INTERBYTE_CLOCKS) begin
                frame_error <= 1'b1;
                state <= HUNT0;
            end else begin
                case (state)
                    HUNT0: if (byte_valid && byte_data == 8'ha5) state <= HUNT1;
                    HUNT1: if (byte_valid) begin
                        if (byte_data == 8'h5a) begin
                            state <= BODY; index <= 5'd0; crc <= 16'hffff;
                        end else if (byte_data != 8'ha5) state <= HUNT0;
                    end
                    BODY: if (byte_valid) begin
                        if (index < 5'd16) begin
                            frame_payload[index*8 +: 8] <= byte_data;
                            crc <= crc16_byte(crc, byte_data);
                        end else if (index == 5'd16) frame_crc[7:0] <= byte_data;
                        else frame_crc[15:8] <= byte_data;
                        if (index == 5'd17) state <= CHECK;
                        else index <= index + 5'd1;
                    end
                    CHECK: begin
                        state <= HUNT0;
                        if (crc == frame_crc && frame_payload[7:0] == VERSION) frame_valid <= 1'b1;
                        else frame_error <= 1'b1;
                    end
                endcase
            end
        end
    end
endmodule
