`timescale 1ns/1ps
// File-driven: "T pos balancing half arm offset" drives one tracker sample;
// "B command offset half boost threshold max out boosted" drives one boost
// evaluation. Expected values come from scripts/j280_backlash.py. Outputs must
// hold between strobes.
module tb_backlash;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0, sample = 0, balancing = 0, valid_in = 0;
    reg signed [31:0] pos = 0;
    reg [3:0] half = 0, bhalf = 0;
    reg signed [15:0] command = 0;
    reg signed [4:0] boffset = 0;
    reg [7:0] gap_boost = 0, threshold = 0;
    reg [15:0] max_compare = 0;
    wire signed [31:0] arm;
    wire signed [4:0] offset;
    wire tvalid, bvalid, boosted;
    wire signed [15:0] command_out;
    j280_backlash_tracker tracker(.clk(clk), .rst_n(rst_n), .sample(sample), .balancing(balancing),
        .pos(pos), .half(half), .arm(arm), .offset(offset), .valid(tvalid));
    j280_gap_boost gap(.clk(clk), .rst_n(rst_n), .valid_in(valid_in), .command(command), .offset(boffset),
        .half(bhalf), .gap_boost(gap_boost), .threshold(threshold), .max_compare(max_compare),
        .valid(bvalid), .command_out(command_out), .boosted(boosted));
    integer file, count, row, i_bal, i_half, e_off, i_off, i_boost, i_thr, i_max, e_boosted;
    reg [31:0] i_pos, e_arm;
    reg [15:0] i_cmd, e_cmd;
    reg [7:0] op;
    reg [52:0] held;
    initial begin
        file = $fopen("vectors.txt", "r");
        if (file == 0) begin $display("FAIL cannot open vectors"); $finish; end
        repeat (3) @(posedge clk);
        @(negedge clk) rst_n = 1;
        row = 0;
        while (!$feof(file)) begin
            count = $fscanf(file, "%c", op);
            if (op == "T") begin
                count = $fscanf(file, " %h %d %d %h %d\n", i_pos, i_bal, i_half, e_arm, e_off);
                if (count != 5) begin $display("FAIL malformed T row=%0d", row); $finish; end
                row = row + 1;
                held = {arm, offset, command_out};
                @(negedge clk); pos = i_pos; balancing = i_bal; half = i_half; sample = 1;
                @(negedge clk); sample = 0;
                if (!tvalid || arm !== e_arm) begin
                    $display("FAIL tracker row=%0d arm=%0d (want %0d)", row, arm, $signed(e_arm));
                    $finish;
                end
                @(negedge clk);
                if (offset !== e_off[4:0]) begin
                    $display("FAIL tracker row=%0d arm=%0d (want %0d) offset=%0d (want %0d)", row, arm, $signed(e_arm), offset, e_off);
                    $finish;
                end
                if (command_out !== held[15:0]) begin $display("FAIL boost output moved row=%0d", row); $finish; end
                @(negedge clk);
                if (arm !== e_arm || offset !== e_off[4:0]) begin $display("FAIL tracker not held row=%0d", row); $finish; end
            end else if (op == "B") begin
                count = $fscanf(file, " %h %d %d %d %d %d %h %d\n", i_cmd, i_off, i_half, i_boost, i_thr, i_max, e_cmd, e_boosted);
                if (count != 8) begin $display("FAIL malformed B row=%0d", row); $finish; end
                row = row + 1;
                held = {arm, offset, command_out};
                @(negedge clk); command = i_cmd; boffset = i_off; bhalf = i_half; gap_boost = i_boost;
                threshold = i_thr; max_compare = i_max; valid_in = 1;
                @(negedge clk); valid_in = 0;
                if (!bvalid || command_out !== e_cmd || boosted !== e_boosted[0]) begin
                    $display("FAIL boost row=%0d out=%0d (want %0d) boosted=%0d (want %0d)", row, command_out, $signed(e_cmd), boosted, e_boosted);
                    $finish;
                end
                if ({arm, offset} !== held[52:16]) begin $display("FAIL tracker output moved row=%0d", row); $finish; end
                @(negedge clk);
                command = 16'h5a5a; max_compare = 0;
                @(negedge clk);
                if (command_out !== e_cmd) begin $display("FAIL boost not held row=%0d", row); $finish; end
            end else if (op != "\n" && op != " " && op != 8'hff) begin
                $display("FAIL unknown op %0d row=%0d", op, row); $finish;
            end
        end
        $fclose(file);
        $display("PASS tb_backlash rows=%0d", row);
        $finish;
    end
    initial begin #200000000; $display("FAIL watchdog"); $finish; end
endmodule
