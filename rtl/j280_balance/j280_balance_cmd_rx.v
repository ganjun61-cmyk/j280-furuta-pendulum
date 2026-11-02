`timescale 1ns/1ps
// v4: A5 5A, 188 immutable payload bytes, CCITT-FALSE CRC low byte first.
// Payload byte zero corresponds to wire byte2; only frame_valid permits use.
module j280_balance_cmd_rx #(
    parameter integer INTERBYTE_CLOCKS=50000,
    parameter [7:0] VERSION=8'd4
)(
    input wire clk,rst_n,byte_valid,
    input wire [7:0] byte_data,
    input wire byte_error,
    output reg frame_valid,
    output reg [1503:0] frame_payload,
    output reg [15:0] frame_crc,
    output reg frame_error
);
    localparam [1:0] HUNT0=0,HUNT1=1,BODY=2,CHECK=3;
    reg [1:0] state;
    reg [7:0] index;
    reg [15:0] crc;
    reg [31:0] gap;
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] work;
        integer bit_number;
        begin
            work=previous ^ {value,8'd0};
            for(bit_number=0;bit_number<8;bit_number=bit_number+1)
                if(work[15]) work=(work<<1)^16'h1021;
                else work=work<<1;
            crc_byte=work;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=HUNT0; index<=0; crc<=16'hffff; gap<=0;
            frame_valid<=0; frame_payload<=0; frame_crc<=0; frame_error<=0;
        end else begin
            frame_valid<=0; frame_error<=0;
            if(byte_valid || state==HUNT0 || state==CHECK) gap<=0;
            else if(gap!=INTERBYTE_CLOCKS) gap<=gap+32'd1;
            if(byte_error) begin state<=HUNT0; frame_error<=1; end
            else if((state==HUNT1 || state==BODY) && gap==INTERBYTE_CLOCKS) begin
                state<=HUNT0; frame_error<=1;
            end else case(state)
                HUNT0: if(byte_valid && byte_data==8'ha5) state<=HUNT1;
                HUNT1: if(byte_valid) begin
                    if(byte_data==8'h5a) begin state<=BODY; index<=0; crc<=16'hffff; end
                    else if(byte_data!=8'ha5) state<=HUNT0;
                end
                BODY: if(byte_valid) begin
                    if(index<8'd188) begin
                        frame_payload[index*8+:8]<=byte_data;
                        crc<=crc_byte(crc,byte_data);
                    end else if(index==8'd188) frame_crc[7:0]<=byte_data;
                    else frame_crc[15:8]<=byte_data;
                    if(index==8'd189) state<=CHECK;
                    else index<=index+8'd1;
                end
                CHECK: begin
                    state<=HUNT0;
                    if(crc==frame_crc && frame_payload[7:0]==VERSION) frame_valid<=1;
                    else frame_error<=1;
                end
                default: state<=HUNT0;
            endcase
        end
    end
endmodule
