`timescale 1ns/1ps

module tb_balance_pwm;
    parameter integer PERIOD = 2500;
    parameter integer MAX_COMPARE = 1000;
    localparam integer MID = MAX_COMPARE / 2;

    reg clk = 0;
    always #10 clk = ~clk; // 50 MHz, including the default 50 us carrier.
    reg rst_n = 1;
    reg allow = 0;
    reg update = 0;
    reg signed [15:0] signed_command = 0;
    wire pwm, in1, in2, active, reject;
    wire [15:0] applied_compare;
    wire [1:0] applied_direction;
    integer phase = 0;
    integer test_case;
    reg old_active;
    reg [15:0] old_compare;
    reg [1:0] old_direction;

    // Count every clock, including clocks crossed by asynchronous stimulus.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phase = 0;
        else if (phase == PERIOD-1) phase = 0;
        else phase = phase + 1;
    end

    j280_balance_pwm #(.PERIOD(PERIOD), .MAX_COMPARE(MAX_COMPARE)) dut (
        .clk(clk), .rst_n(rst_n), .allow(allow), .update(update),
        .signed_command(signed_command), .pwm(pwm), .in1(in1), .in2(in2),
        .active(active), .applied_compare(applied_compare),
        .applied_direction(applied_direction), .reject(reject)
    );

    task fail;
        input [8*160-1:0] message;
        begin
            $display("FAIL tb_balance_pwm CASE=%0d phase=%0d time=%0t: %0s",
                     test_case, phase, $time, message);
            $display("  active=%b compare=%0d direction=%0d pwm=%b in1=%b in2=%b reject=%b",
                     active, applied_compare, applied_direction, pwm, in1, in2, reject);
            $finish;
        end
    endtask

    task check_state;
        input integer expected_active;
        input integer expected_compare;
        input integer expected_direction;
        begin
            if (active !== (expected_active != 0) || applied_compare !== expected_compare ||
                applied_direction !== expected_direction)
                fail("applied state differs from expected active/compare/direction");
            if (in1 !== (expected_direction == 1) || in2 !== (expected_direction == 2))
                fail("generic direction pins disagree with applied direction");
            if (expected_active == 0) begin
                if (pwm !== 1'b1) fail("inactive must be Stop: PWM=1, IN1=IN2=0");
            end else if (pwm !== (phase < expected_compare))
                fail("PWM pulse is not aligned to the free-running carrier");
        end
    endtask

    // Track only elapsed clocks, never DUT internals. Check every sampled cycle,
    // including all alignment waits, for mid-carrier telemetry changes.
    always @(posedge clk) begin
        old_active = active;
        old_compare = applied_compare;
        old_direction = applied_direction;
        #1;
        if (in1 === 1'b1 && in2 === 1'b1) fail("both direction pins asserted");
        if (!rst_n || !allow || reject) check_state(0, 0, 0);
        else begin
            if (phase != 0 && (active !== old_active || applied_compare !== old_compare ||
                              applied_direction !== old_direction))
                fail("applied telemetry changed away from a carrier boundary");
            if (active) begin
                if (applied_compare > MAX_COMPARE ||
                    (applied_direction != 1 && applied_direction != 2))
                    fail("active telemetry outside the contract");
                check_state(1, applied_compare, applied_direction);
            end else check_state(0, 0, 0);
        end
    end

    task tick;
        begin
            @(posedge clk);
            #2; // Sample after nonblocking RTL assignments and the monitor.
        end
    endtask

    task idle;
        input integer cycles;
        integer i;
        begin
            for (i = 0; i < cycles; i = i + 1) begin
                @(negedge clk);
                update = 0;
                tick;
            end
        end
    endtask

    task align_last;
        begin
            while (phase != PERIOD-1) idle(1);
        end
    endtask

    task drive;
        input integer command;
        begin
            @(negedge clk);
            signed_command = command;
            update = 1;
            tick;
            update = 0;
        end
    endtask

    task at_boundary;
        input integer command;
        begin
            align_last;
            drive(command);
            if (phase != 0) fail("test stimulus missed carrier boundary");
        end
    endtask

    task check_period;
        input integer expected_compare;
        input integer expected_direction;
        integer i;
        integer high_clocks;
        begin
            if (phase != 0) fail("full-period check must start at counter zero");
            high_clocks = 0;
            for (i = 0; i < PERIOD; i = i + 1) begin
                check_state(1, expected_compare, expected_direction);
                if (pwm) high_clocks = high_clocks + 1;
                idle(1);
            end
            if (high_clocks != expected_compare) fail("wrong complete-period high width");
        end
    endtask

    task reset_dut;
        begin
            @(negedge clk);
            rst_n = 0;
            allow = 0;
            update = 0;
            #1;
            check_state(0, 0, 0);
            tick;
            if (reject !== 0) fail("reset must clear reject");
            @(negedge clk);
            rst_n = 1;
            #1;
            check_state(0, 0, 0);
        end
    endtask

    task enable_wait;
        begin
            allow = 1;
            #1;
            check_state(0, 0, 0);
            idle(2*PERIOD+3);
            check_state(0, 0, 0);
        end
    endtask

    task widths_and_zero;
        begin
            reset_dut;
            idle(PERIOD+2);
            enable_wait;
            at_boundary(0);
            check_period(0, 2); // First active zero defaults to positive/BIN2.
            at_boundary(1);
            check_period(1, 2);
            check_period(1, 2); // Continuous output without repeated updates.
            at_boundary(MAX_COMPARE);
            check_period(MAX_COMPARE, 2);
            at_boundary(0);
            check_period(0, 2);
            at_boundary(-MID);
            check_period(0, 2); // One full blank before changing direction.
            check_period(MID, 1);
            at_boundary(-MAX_COMPARE);
            check_period(MAX_COMPARE, 1);
            at_boundary(-1);
            check_period(1, 1);
            at_boundary(0);
            check_period(0, 1);
            check_period(0, 1); // Zero retains the last nonzero direction.
            at_boundary(MAX_COMPARE);
            check_period(0, 1);
            check_period(MAX_COMPARE, 2);
        end
    endtask

    task boundary_updates;
        begin
            reset_dut;
            enable_wait;
            // Queue before first activation: Stop persists until the boundary.
            align_last;
            idle(1);
            drive(MAX_COMPARE);
            check_state(0, 0, 0);
            drive(1);
            check_state(0, 0, 0);
            align_last;
            drive(MID); // On-boundary update beats the queued +1.
            check_period(MID, 2);
            drive(MAX_COMPARE);
            drive(1);
            align_last;
            idle(1); // Without a boundary update, the newest pending value wins.
            check_period(1, 2);
            drive(-MAX_COMPARE);
            at_boundary(MID); // Opposite pending must not cause a stale blank.
            check_period(MID, 2);
            drive(-MID);
            at_boundary(0);
            check_period(0, 2);
            // Consecutive asserted update cycles span the boundary.
            while (phase != PERIOD-2) idle(1);
            @(negedge clk);
            update = 1;
            signed_command = MAX_COMPARE;
            tick;
            @(negedge clk);
            signed_command = MID;
            tick;
            check_state(1, MID, 2);
            @(negedge clk);
            signed_command = 1;
            tick;
            update = 0;
            check_state(1, MID, 2);
            align_last;
            idle(1);
            check_period(1, 2);
            signed_command = -MAX_COMPARE; // A changing bus without update is ignored.
            check_period(1, 2);
        end
    endtask

    task reversal_updates;
        begin
            reset_dut;
            enable_wait;
            at_boundary(MID);
            check_period(MID, 2);
            at_boundary(-MAX_COMPARE);
            check_period(0, 2);
            check_period(MAX_COMPARE, 1); // Exactly one period, no extra blank.
            at_boundary(MID);
            check_state(1, 0, 1);
            drive(MAX_COMPARE);
            check_state(1, 0, 1);
            drive(-1);
            check_state(1, 0, 1);
            at_boundary(1); // Newest boundary command also wins when blank ends.
            check_period(1, 2);
            at_boundary(-MAX_COMPARE);
            drive(0); // Zero during blank retains the direction actually applied.
            align_last;
            idle(1);
            check_period(0, 2);
            at_boundary(-MID);
            drive(MAX_COMPARE); // Returning to old sign still cannot bypass blank.
            check_state(1, 0, 2);
            align_last;
            idle(1);
            check_period(MAX_COMPARE, 2);
            at_boundary(-1);
            drive(-MID);
            drive(-MAX_COMPARE); // Latest between-boundary update at blank exit.
            align_last;
            idle(1);
            check_period(MAX_COMPARE, 1);
        end
    endtask

    task stop_and_reset;
        begin
            reset_dut;
            enable_wait;
            at_boundary(MAX_COMPARE);
            drive(-MID); // Pending reversal must be discarded on allow OFF.
            if (pwm !== 1) fail("disable test must interrupt a high PWM pulse");
            #2;
            allow = 0;
            #1;
            check_state(0, 0, 0); // No clock edge or carrier wait is permitted.
            idle(1); // A single sampled OFF must flush both active and pending.
            enable_wait;
            #2;
            allow = 0;
            #1;
            check_state(0, 0, 0);
            idle(1);
            drive(-32768); // Disabled updates, including illegal ones, are ignored.
            if (reject !== 0) fail("disabled command must not latch reject");
            drive(MAX_COMPARE);
            enable_wait;
            at_boundary(-MAX_COMPARE); // Fresh negative activation needs no blank.
            check_period(MAX_COMPARE, 1);
            // allow must not reset or pause the free-running carrier.
            drive(MID);
            #2;
            rst_n = 0; // Keep allow high to test reset independently of allow gating.
            #1;
            check_state(0, 0, 0);
            tick;
            if (reject !== 0) fail("asynchronous reset must clear reject");
            @(negedge clk);
            rst_n = 1;
            update = 0;
            idle(2*PERIOD+3);
            check_state(0, 0, 0);
            at_boundary(0);
            check_period(0, 2);
            at_boundary(-MAX_COMPARE);
            check_period(0, 2);
            check_period(MAX_COMPARE, 1);
            at_boundary(MID); // Disable while a reversal is blanking.
            check_state(1, 0, 1);
            #2;
            allow = 0;
            #1;
            check_state(0, 0, 0);
            idle(1);
            enable_wait;
            at_boundary(0);
            check_period(0, 1); // Last nonzero applied direction survives allow OFF.
            // OFF wins over an illegal update coincident with the boundary.
            align_last;
            @(negedge clk);
            allow = 0;
            update = 1;
            signed_command = -32768;
            #1;
            check_state(0, 0, 0);
            tick;
            update = 0;
            if (reject !== 0) fail("allow OFF must win over illegal boundary update");
            enable_wait;
            at_boundary(0);
            check_period(0, 1);
        end
    endtask

    task reject_bad;
        input integer bad_command;
        begin
            reset_dut;
            enable_wait;
            at_boundary(MID);
            drive(bad_command); // Rejection is on this clock, not next boundary.
            if (reject !== 1) fail("illegal magnitude did not latch reject immediately");
            check_state(0, 0, 0);
            drive(1);
            idle(2*PERIOD);
            if (reject !== 1) fail("reject must stay sticky despite legal updates");
            check_state(0, 0, 0);
            #2;
            allow = 0;
            #1;
            check_state(0, 0, 0);
            idle(1);
            if (reject !== 0) fail("allow OFF must clear reject on the next clock");
            enable_wait;
            at_boundary(MAX_COMPARE);
            check_period(MAX_COMPARE, 2);
        end
    endtask

    task illegal_and_rearm;
        begin
            reject_bad(MAX_COMPARE+1);
            reject_bad(-MAX_COMPARE-1);
            reject_bad(32767);
            reject_bad(-32767);
            reject_bad(-32768); // Detect abs(int16_min) without signed overflow.
            drive(-MID);
            at_boundary(-32768); // Invalid boundary update beats legal pending.
            if (reject !== 1) fail("invalid boundary update must reject pending command");
            check_state(0, 0, 0);
            // Reset can clear a latched reject with allow held high.
            #2;
            rst_n = 0;
            #1;
            if (reject !== 0) fail("reset must asynchronously clear latched reject");
            check_state(0, 0, 0);
            tick;
            @(negedge clk);
            rst_n = 1;
            idle(PERIOD+3);
            check_state(0, 0, 0);
            at_boundary(-MAX_COMPARE);
            check_period(MAX_COMPARE, 1);
            at_boundary(MID);
            drive(-32768); // Rejection also interrupts a reversal blank immediately.
            if (reject !== 1) fail("invalid update during blank must latch reject");
            check_state(0, 0, 0);
        end
    endtask

    initial begin
        if (!$value$plusargs("CASE=%d", test_case)) test_case = 1;
        if (PERIOD < 8 || MAX_COMPARE < 2 || MAX_COMPARE >= PERIOD)
            fail("testbench parameter set unsupported");
        case (test_case)
            1: widths_and_zero;
            2: boundary_updates;
            3: reversal_updates;
            4: stop_and_reset;
            5: illegal_and_rearm;
            default: fail("unknown CASE");
        endcase
        $display("PASS tb_balance_pwm CASE=%0d PERIOD=%0d MAX_COMPARE=%0d",
                 test_case, PERIOD, MAX_COMPARE);
        $finish;
    end

    initial begin
        #1000000000;
        fail("simulation timeout");
    end
endmodule
