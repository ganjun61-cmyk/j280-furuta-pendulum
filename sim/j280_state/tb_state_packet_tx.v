`timescale 1ns/1ps
// Sends payload.hex once as a 128-byte frame and records the UART pin bytes to
// wire.bin. Payload mutation and busy requests must not alter or add frames.
module tb_state_packet_tx;
    localparam integer BIT_NS = 100;
    localparam integer FRAME_BYTES = 128;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg packet_valid = 1'b0;
    reg [991:0] golden [0:0];
    reg [991:0] payload = 992'd0;
    wire uart_tx, overflow, busy;
    integer wire_file, index;
    reg [7:0] value;

    j280_state_packet_tx #(.CLOCK_HZ(1000), .BAUD(100)) dut (
        .clk(clk), .rst_n(rst_n), .packet_valid(packet_valid), .payload(payload),
        .uart_tx(uart_tx), .overflow(overflow), .busy(busy));

    task receive_byte;
        output [7:0] data;
        integer bit_number;
        begin
            @(negedge uart_tx);
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0) begin $display("FAIL start bit"); $finish; end
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                #BIT_NS;
                data[bit_number] = uart_tx;
            end
            #BIT_NS;
            if (uart_tx !== 1'b1) begin $display("FAIL stop bit"); $finish; end
            if (busy !== 1'b1) begin $display("FAIL busy dropped before stop bit end"); $finish; end
        end
    endtask

    task request;
        begin
            @(negedge clk);
            packet_valid = 1'b1;
            @(negedge clk);
            packet_valid = 1'b0;
        end
    endtask

    initial begin
        $readmemh("payload.hex", golden);
        wire_file = $fopen("wire.bin", "wb");
        if (wire_file == 0) begin $display("FAIL cannot open wire.bin"); $finish; end
        payload = golden[0];
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        fork
            for (index = 0; index < FRAME_BYTES; index = index + 1) begin
                receive_byte(value);
                $fwrite(wire_file, "%c", value);
            end
            begin
                request;
                if (busy !== 1'b1) begin $display("FAIL idle packet not accepted"); $finish; end
                payload = {992{1'b1}};
                request;
                if (overflow !== 1'b1) begin $display("FAIL request during CRC must overflow"); $finish; end
                repeat (3000) @(negedge clk);
                payload = 992'h1234;
                request;
                if (overflow !== 1'b1) begin $display("FAIL overflow must be sticky"); $finish; end
            end
        join
        $fclose(wire_file);
        wait (busy === 1'b0);
        repeat (400) begin
            @(posedge clk);
            #1;
            if ((uart_tx !== 1'b1) || (busy !== 1'b0) || (overflow !== 1'b1)) begin
                $display("FAIL dropped request was queued or overflow cleared"); $finish;
            end
        end
        rst_n = 1'b0;
        #1;
        if ((uart_tx !== 1'b1) || busy || overflow) begin $display("FAIL reset state"); $finish; end
        $display("PASS tb_state_packet_tx");
        $finish;
    end
    initial begin #20000000; $display("FAIL watchdog"); $finish; end
endmodule
