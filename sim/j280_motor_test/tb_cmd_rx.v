`timescale 1ns/1ps
// Drives uart_rx from a script and logs bytes/frames for Python checks.
// Script lines: "B hh idle_ns" byte, "F hh idle_ns" byte with low stop bit,
// "G low_ns idle_ns" glitch, "E" end. Log: "D hh", "R" (byte frame error),
// "V payload_hex crc_hex", "X" (frame error).
module tb_cmd_rx;
    parameter integer BIT_NS = 4340;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg rx = 1;
    wire [7:0] data;
    wire valid, byte_error, frame_valid, frame_error;
    wire [127:0] payload;
    wire [15:0] crc;
    j280_uart_rx receiver(.clk(clk),.rst_n(rst_n),.rx(rx),.data(data),
        .valid(valid),.frame_error(byte_error));
    j280_cmd_rx parser(.clk(clk),.rst_n(rst_n),.byte_valid(valid),.byte_data(data),
        .byte_error(byte_error),.frame_valid(frame_valid),.frame_payload(payload),
        .frame_crc(crc),.frame_error(frame_error));

    integer script, log, code, i;
    reg [7:0] op;
    reg [31:0] value, idle_ns;

    always @(posedge clk) begin
        if (valid) $fwrite(log, "D %02x\n", data);
        if (byte_error) $fwrite(log, "R\n");
        if (frame_valid) $fwrite(log, "V %032x %04x\n", payload, crc);
        if (frame_error) $fwrite(log, "X\n");
    end

    task send_byte;
        input [7:0] b;
        input stop_bit;
        begin
            rx = 0; #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin rx = b[i]; #(BIT_NS); end
            rx = stop_bit; #(BIT_NS);
            rx = 1;
        end
    endtask

    initial begin
        log = $fopen("rx_log.txt", "w");
        script = $fopen("rx_script.txt", "r");
        if (!log || !script) begin $display("FAIL cannot open files"); $finish; end
        #95 rst_n = 1;
        #1000;
        forever begin
            code = $fscanf(script, " %c", op);
            if (code != 1 || op == "E") begin
                #(20 * BIT_NS);
                $fclose(log);
                $display("PASS tb_cmd_rx");
                $finish;
            end
            if (op == "B" || op == "F") begin
                code = $fscanf(script, " %h %d", value, idle_ns);
                #(idle_ns);
                send_byte(value[7:0], op == "B");
            end else if (op == "G") begin
                code = $fscanf(script, " %d %d", value, idle_ns);
                #(idle_ns);
                rx = 0; #(value); rx = 1;
            end else begin
                $display("FAIL bad script op %c", op); $finish;
            end
        end
    end
    initial begin #2000000000; $display("FAIL watchdog"); $finish; end
endmodule
