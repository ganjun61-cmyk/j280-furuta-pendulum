`timescale 1ns/1ps

module tb_frame_tx;
    localparam integer BIT_NS = 100;
    // Bytes 2..29, first byte in the least-significant octet.
    localparam [223:0] GOLDEN_PAYLOAD = 224'hffff1234020308ca000080000000123403ff89abcdef123456780301;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg snapshot_valid = 1'b0;
    reg [223:0] payload = GOLDEN_PAYLOAD;
    wire uart_tx;
    wire overflow;
    wire busy;
    reg [7:0] expected [0:31];
    reg [7:0] received;
    integer byte_number;
    integer out_file = 0;
    reg [2047:0] out_path;

    j280_frame_tx #(.CLOCK_HZ(1000), .BAUD(100)) dut (
        .clk(clk), .rst_n(rst_n), .snapshot_valid(snapshot_valid),
        .payload(payload), .uart_tx(uart_tx), .overflow(overflow), .busy(busy));

    task request_snapshot;
        begin
            @(negedge clk);
            snapshot_valid = 1'b1;
            @(posedge clk);
            #1;
            if (busy !== 1'b1)
                $fatal(1, "Frame sender did not accept an idle snapshot");
            @(negedge clk);
            snapshot_valid = 1'b0;
        end
    endtask

    task receive_byte;
        output [7:0] value;
        integer bit_number;
        begin
            @(negedge uart_tx);
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0)
                $fatal(1, "Frame UART has an invalid start bit");
            if (busy !== 1'b1)
                $fatal(1, "Frame busy dropped during a UART start bit");
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                #BIT_NS;
                value[bit_number] = uart_tx;
                if (busy !== 1'b1)
                    $fatal(1, "Frame busy dropped during a UART data bit");
            end
            #BIT_NS;
            if (uart_tx !== 1'b1)
                $fatal(1, "Frame UART has an invalid stop bit");
            #(BIT_NS / 2 - 1);
            if (busy !== 1'b1)
                $fatal(1, "Frame busy must include the complete final stop bit");
        end
    endtask

    task receive_golden_frame;
        integer index;
        reg [7:0] value;
        begin
            for (index = 0; index < 32; index = index + 1) begin
                receive_byte(value);
                if (value !== expected[index])
                    $fatal(1, "Frame byte %0d expected %02x got %02x", index, expected[index], value);
                if (out_file != 0)
                    $fdisplay(out_file, "%02x", value);
            end
        end
    endtask

    task wait_idle;
        begin
            wait (busy === 1'b0);
            repeat (3) @(negedge clk);
        end
    endtask

    initial begin
        expected[0] = 8'hA5;
        expected[1] = 8'h5A;
        for (byte_number = 0; byte_number < 28; byte_number = byte_number + 1)
            expected[byte_number + 2] = GOLDEN_PAYLOAD[byte_number * 8 +: 8];
        // Independent oracle: Python binascii.crc_hqx(payload, 0xffff) = 0x3711.
        expected[30] = 8'h11;
        expected[31] = 8'h37;
        if ($value$plusargs("OUT=%s", out_path)) begin
            out_file = $fopen(out_path, "w");
            if (out_file == 0)
                $fatal(1, "Cannot open +OUT frame byte file");
        end
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        fork
            receive_golden_frame;
            begin
                @(negedge clk);
                snapshot_valid = 1'b1;
                @(posedge clk);
                #1;
                if (busy !== 1'b1)
                    $fatal(1, "Frame snapshot was not accepted");
                // Change all inputs while CRC is being computed, then keep
                // changing them during the UART transfer.
                payload = {224{1'b1}};
                @(negedge clk);
                snapshot_valid = 1'b0;
                @(negedge clk);
                snapshot_valid = 1'b1;
                @(negedge clk);
                snapshot_valid = 1'b0;
                #1;
                if (overflow !== 1'b1)
                    $fatal(1, "A snapshot during CRC must set overflow");
                repeat (40) @(negedge clk);
                snapshot_valid = 1'b1;
                @(negedge clk);
                snapshot_valid = 1'b0;
                payload = 224'h1234;
                repeat (40) @(negedge clk);
                if (overflow !== 1'b1)
                    $fatal(1, "A busy snapshot must set sticky overflow");
            end
        join
        wait_idle;
        // A dropped request must never emerge as a second frame.
        repeat (400) begin
            @(posedge clk);
            #1;
            if ((uart_tx !== 1'b1) || (busy !== 1'b0) || (overflow !== 1'b1))
                $fatal(1, "Dropped snapshot was queued, or overflow was not sticky");
        end

        payload = GOLDEN_PAYLOAD;
        fork
            begin
                receive_byte(received);
                if (received !== 8'hA5)
                    $fatal(1, "Second accepted frame did not start correctly");
            end
            request_snapshot;
        join
        repeat (15) @(negedge clk);
        rst_n = 1'b0;
        #1;
        if ((uart_tx !== 1'b1) || (busy !== 1'b0) || (overflow !== 1'b0))
            $fatal(1, "Reset must abort frame, clear busy and clear overflow");
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        fork
            receive_golden_frame;
            request_snapshot;
        join
        wait_idle;
        if (overflow !== 1'b0)
            $fatal(1, "Fresh frame after reset spuriously overflowed");
        if (out_file != 0)
            $fclose(out_file);
        $display("PASS tb_frame_tx: fixed CRC, immutable snapshot, drop/overflow and reset recovery");
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "tb_frame_tx timeout");
    end
endmodule
