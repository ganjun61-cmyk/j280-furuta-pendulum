`timescale 1ns/1ps

module tb_uart_tx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 230400,
    parameter integer DIVISOR = 217
);
    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;
    reg [7:0] data = 8'h00;
    reg start = 1'b0;
    wire tx;
    wire busy;

    j280_uart_tx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) dut (.clk(clk), .rst_n(rst_n), .data(data),
        .start(start), .tx(tx), .busy(busy));

    task send_and_check;
        input [7:0] value;
        reg [9:0] expected;
        integer bit_number;
        integer cycle_number;
        begin
            expected = {1'b1, value, 1'b0};
            @(negedge clk);
            data = value;
            start = 1'b1;
            @(posedge clk);
            #1;
            for (bit_number = 0; bit_number < 10; bit_number = bit_number + 1) begin
                for (cycle_number = 0; cycle_number < DIVISOR; cycle_number = cycle_number + 1) begin
                    if ((bit_number != 0) || (cycle_number != 0)) begin
                        @(posedge clk);
                        #1;
                    end
                    if (tx !== expected[bit_number])
                        $fatal(1, "UART bit %0d cycle %0d expected %b got %b", bit_number, cycle_number, expected[bit_number], tx);
                    if (busy !== 1'b1)
                        $fatal(1, "UART busy must include all ten bit periods");
                    start = 1'b0;
                    // An attempted byte during a transfer must not alter it.
                    if ((bit_number == 3) && (cycle_number == 10)) begin
                        data = ~value;
                        start = 1'b1;
                    end
                end
            end
            @(posedge clk);
            #1;
            if ((busy !== 1'b0) || (tx !== 1'b1))
                $fatal(1, "UART did not become idle after exactly %0d clocks", 10 * DIVISOR);
            repeat (20) begin
                @(posedge clk);
                #1;
                if ((busy !== 1'b0) || (tx !== 1'b1))
                    $fatal(1, "UART queued a start received while busy");
            end
        end
    endtask

    initial begin
        #1;
        if ((tx !== 1'b1) || (busy !== 1'b0))
            $fatal(1, "UART reset must produce an idle high output");
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        send_and_check(8'hA6);
        send_and_check(8'h00);
        send_and_check(8'hFF);

        @(negedge clk);
        data = 8'h55;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        repeat (DIVISOR * 2) @(negedge clk);
        rst_n = 1'b0;
        #1;
        if ((tx !== 1'b1) || (busy !== 1'b0))
            $fatal(1, "UART reset must abort an active byte immediately");
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        send_and_check(8'h3C);
        $display("PASS tb_uart_tx: 8N1, %0d clocks/bit, busy rejection and reset", DIVISOR);
        $finish;
    end

    initial begin
        #500000;
        $fatal(1, "tb_uart_tx timeout");
    end
endmodule
