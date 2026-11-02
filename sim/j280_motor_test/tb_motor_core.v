`timescale 1ns/1ps
// Guard + pulse generator + local inputs with scaled timing, driven by
// core_script.txt. Every clock on which a monitored output changes is logged
// to core_log.txt for the Python checks; input changes are logged as
// "I cycle op value" (the value is visible to the DUT on the next edge).
module tb_motor_core;
    parameter integer CLOCKS_PER_MS = 200;
    parameter integer PWM_PERIOD = 20;
    parameter integer PERIODS_PER_MS = 10;
    parameter integer MAX_COMPARE = 8;
    parameter integer PWM_MAX_COMPARE = 8;
    parameter integer MAX_PULSE_MS = 20;
    parameter integer ARM_TIMEOUT_MS = 50;
    parameter integer STALE_CLOCKS = 1000;
    parameter integer DEBOUNCE = 20;
    parameter integer EST_PERIOD = 400;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;
    reg sw1_n = 1, sw2_n = 1, dip1 = 1;
    reg cmd_valid = 0, cmd_error = 0;
    reg [127:0] cmd_payload = 0;
    reg [15:0] cmd_crc = 0;
    reg est_valid = 0;
    reg [2:0] est_quality = 3'd7;
    reg signed [31:0] est_velocity = 0;
    reg signed [31:0] position = 0;
    reg est_on = 1;

    wire start_event, stop_pressed, arm_on_raw, arm_on_stable, keys_released;
    wire [2:0] levels;
    wire [7:0] state, stop_reason, rx_status;
    wire config_loaded, pwm_start, output_allow, est_fresh, est_ok;
    wire [15:0] config_id, config_crc, drive_compare, applied_compare;
    wire [1:0] drive_direction;
    wire [31:0] drive_periods;
    wire pwm, in1, in2, driving, done, reject;

    j280_local_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE)) inputs(.clk(clk),.rst_n(rst_n),
        .sw1_n(sw1_n),.sw2_n(sw2_n),.dip1(dip1),.start_event(start_event),
        .stop_pressed(stop_pressed),.arm_on_raw(arm_on_raw),.arm_on_stable(arm_on_stable),
        .keys_released(keys_released),.levels(levels));
    j280_motor_guard #(.CLOCKS_PER_MS(CLOCKS_PER_MS),.PERIODS_PER_MS(PERIODS_PER_MS),
        .MAX_COMPARE(MAX_COMPARE),.MAX_PULSE_MS(MAX_PULSE_MS),.ARM_TIMEOUT_MS(ARM_TIMEOUT_MS),
        .STALE_CLOCKS(STALE_CLOCKS)) guard(.clk(clk),.rst_n(rst_n),
        .cmd_valid(cmd_valid),.cmd_payload(cmd_payload),.cmd_crc(cmd_crc),.cmd_error(cmd_error),
        .start_event(start_event),.stop_key(stop_pressed),.arm_on_raw(arm_on_raw),
        .arm_on_stable(arm_on_stable),.keys_released(keys_released),.est_valid(est_valid),
        .est_quality(est_quality),.est_velocity_q16(est_velocity),.encoder_position(position),
        .pwm_done(done),.pwm_reject(reject),.state(state),.stop_reason(stop_reason),
        .config_loaded(config_loaded),.config_id(config_id),.config_crc(config_crc),
        .drive_direction(drive_direction),.drive_compare(drive_compare),
        .drive_periods(drive_periods),.pwm_start(pwm_start),.output_allow(output_allow),
        .rx_status(rx_status),.est_fresh(est_fresh),.est_ok(est_ok));
    j280_pwm_pulse #(.PERIOD(PWM_PERIOD),.MAX_COMPARE(PWM_MAX_COMPARE),
        .MAX_PERIODS(MAX_PULSE_MS*PERIODS_PER_MS)) pulse(.clk(clk),.rst_n(rst_n),
        .allow(output_allow),.start(pwm_start),.direction(drive_direction),
        .compare(drive_compare),.periods(drive_periods),.pwm(pwm),.in1(in1),.in2(in2),
        .driving(driving),.drive_compare(applied_compare),.done(done),.reject(reject));

    integer cycle = 0, est_count = 0;
    integer script, log, code;
    reg [8*4-1:0] op;
    reg [31:0] a0, a1, a2, a3, a4, a5, a6, a7, a8;
    reg [127:0] raw_payload;
    reg [15:0] raw_crc;
    reg [127:0] snapshot, previous = {128{1'b1}};

    always @(posedge clk) begin
        cycle <= cycle + 1;
        est_valid <= 0;
        if (rst_n && est_on) begin
            if (est_count == EST_PERIOD - 1) begin est_count <= 0; est_valid <= 1; end
            else est_count <= est_count + 1;
        end
    end

    always @(negedge clk) begin
        snapshot = {state, stop_reason, rx_status, config_loaded, config_id, config_crc,
                    pwm, in1, in2, driving, applied_compare, output_allow, est_fresh, est_ok,
                    arm_on_stable};
        if (snapshot !== previous) begin
            $fwrite(log, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                    cycle, state, stop_reason, rx_status, config_loaded, config_id, config_crc,
                    pwm, in1, in2, driving, applied_compare, output_allow, est_fresh, est_ok,
                    arm_on_stable);
            previous = snapshot;
        end
        if (!driving && (pwm !== 1'b1 || in1 !== 1'b0 || in2 !== 1'b0)) begin
            $display("FAIL idle pins at cycle %0d", cycle); $finish;
        end
        if (in1 && in2) begin $display("FAIL both IN high"); $finish; end
    end

    task frame;
        input [7:0] command;
        input [15:0] id;
        input [7:0] direction, reserved;
        input [15:0] duty, pulse_ms, travel, speed, keepalive, crc;
        begin
            @(negedge clk);
            cmd_payload = {keepalive, speed, travel, pulse_ms, duty, reserved, direction, id, command, 8'd3};
            cmd_crc = crc;
            cmd_valid = 1;
            @(negedge clk);
            cmd_valid = 0;
        end
    endtask

    initial begin
        log = $fopen("core_log.txt", "w");
        script = $fopen("core_script.txt", "r");
        if (!log || !script) begin $display("FAIL cannot open files"); $finish; end
        repeat (3) @(negedge clk);
        rst_n = 1;
        forever begin
            code = $fscanf(script, " %s", op);
            if (code != 1 || op == "E") begin
                $fclose(log);
                $display("PASS tb_motor_core");
                $finish;
            end
            case (op)
                "W": begin code = $fscanf(script, " %d", a0); repeat (a0) @(negedge clk); end
                "C": begin
                    code = $fscanf(script, " %d %d %d %d %d %d %d %d %d", a0, a1, a8, a2, a3, a4, a5, a6, a7);
                    frame(8'd1, a0, a1, a8[7:0], a2, a3, a4, a5, a6, a7);
                end
                "M": begin code = $fscanf(script, " %d", a0); frame(a0, 0, 0, 0, 0, 0, 0, 0, 0, 16'h5555); end
                "F": begin
                    code = $fscanf(script, " %h %h", raw_payload, raw_crc);
                    @(negedge clk); cmd_payload = raw_payload; cmd_crc = raw_crc; cmd_valid = 1;
                    @(negedge clk); cmd_valid = 0;
                end
                "X": begin @(negedge clk); cmd_error = 1; @(negedge clk); cmd_error = 0; end
                "S1": begin code = $fscanf(script, " %d", a0); sw1_n = a0; end
                "S2": begin code = $fscanf(script, " %d", a0); sw2_n = a0; end
                "D": begin code = $fscanf(script, " %d", a0); dip1 = a0; end
                "Q": begin code = $fscanf(script, " %d", a0); est_quality = a0; end
                "V": begin code = $fscanf(script, " %d", a0); est_velocity = $signed(a0) <<< 16; end
                "P": begin code = $fscanf(script, " %d", a0); est_on = a0; end
                "N": begin code = $fscanf(script, " %d", a0); position = $signed(a0); end
                "R": begin code = $fscanf(script, " %d", a0); rst_n = a0; end
                default: begin $display("FAIL bad op %0s", op); $finish; end
            endcase
            if (op != "W" && op != "C" && op != "M" && op != "X" && op != "F")
                $fwrite(log, "I %0d %0s %0d\n", cycle, op, $signed(a0));
        end
    end
    initial begin #2000000000; $display("FAIL watchdog"); $finish; end
endmodule
