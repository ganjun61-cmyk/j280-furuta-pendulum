`timescale 1ns/1ps

module tb_encoder;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg enc_a = 1'b1;
    reg enc_b = 1'b1;
    wire signed [31:0] position;
    wire [31:0] illegal_count;
    wire ready;
    j280_encoder dut (.clk(clk), .rst_n(rst_n), .enc_a(enc_a),
        .enc_b(enc_b), .position(position), .illegal_count(illegal_count), .ready(ready));

    task state;
        input [1:0] ab;
        begin
            @(negedge clk);
            enc_a = ab[1];
            enc_b = ab[0];
            repeat (6) @(posedge clk);
            #1;
        end
    endtask

    task check;
        input signed [31:0] expected_position;
        input [31:0] expected_illegal;
        begin
            if ((position !== expected_position) || (illegal_count !== expected_illegal))
                $fatal(1, "Encoder expected position %0d illegal %0d; got position %0d illegal %0d", expected_position, expected_illegal, position, illegal_count);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (8) @(posedge clk);
        #1;
        if (ready !== 1'b1)
            $fatal(1, "Encoder did not finish synchronizer startup");
        check(0, 0);
        // A stable 11 at reset must not become a phantom diagonal edge.
        state(2'b11);
        check(0, 0);

        @(negedge clk);
        rst_n = 1'b0;
        enc_a = 1'b0;
        enc_b = 1'b0;
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (8) @(posedge clk);
        #1;
        check(0, 0);
        state(2'b10); check(1, 0);
        state(2'b11); check(2, 0);
        state(2'b01); check(3, 0);
        state(2'b00); check(4, 0);
        state(2'b01); check(3, 0);
        state(2'b11); check(2, 0);
        state(2'b10); check(1, 0);
        state(2'b00); check(0, 0);
        state(2'b01); check(-1, 0);
        state(2'b11); check(-2, 0);
        state(2'b10); check(-3, 0);
        state(2'b00); check(-4, 0);
        if (!(position < 0))
            $fatal(1, "Position output must have signed semantics");

        state(2'b11); check(-4, 1);
        state(2'b01); check(-3, 1);
        state(2'b10); check(-3, 2);
        state(2'b00); check(-4, 2);

        // Seed public counters to exercise boundaries without billions of edges.
        @(negedge clk);
        dut.position = 32'h7fffffff;
        state(2'b10); check(32'sh80000000, 2);
        state(2'b00); check(32'sh7fffffff, 2);
        @(negedge clk);
        dut.illegal_count = 32'hfffffffe;
        state(2'b11); check(32'sh7fffffff, 32'hffffffff);
        state(2'b00); check(32'sh7fffffff, 32'hffffffff);

        @(negedge clk);
        rst_n = 1'b0;
        #1;
        if ((ready !== 1'b0) || (position !== 0) || (illegal_count !== 0))
            $fatal(1, "Encoder reset did not clear public state");
        $display("PASS tb_encoder: startup, direction, signed wrap, illegal transitions and saturation");
        $finish;
    end

    initial begin
        #20000;
        $fatal(1, "tb_encoder timeout");
    end
endmodule
