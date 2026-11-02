`timescale 1ns/1ps

// Bounded motor-test supervisor: SAFE(0) -> ARMED(1) -> RUN(2) -> SAFE, or
// FAULT(255) until reset. The host can only load, keep alive and abort; motion
// starts from the local start key. A loaded configuration is consumed by any
// exit from ARMED/RUN. Stop decisions are registered: output_allow falls on
// the clock edge after the condition is present.
module j280_motor_guard #(
    parameter integer CLOCKS_PER_MS = 50000,
    parameter integer PERIODS_PER_MS = 20,
    parameter integer MAX_COMPARE = 1000,
    parameter integer MAX_PULSE_MS = 2000,
    parameter integer MAX_TRAVEL_COUNT = 200,
    parameter integer MAX_SPEED_COUNT_S = 10000,
    parameter integer MAX_KEEPALIVE_MS = 500,
    parameter integer ARM_TIMEOUT_MS = 60000,
    parameter integer RUN_GRACE_MS = 10,
    parameter integer STALE_CLOCKS = 250000
) (
    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    input wire [127:0] cmd_payload,
    input wire [15:0] cmd_crc,
    input wire cmd_error,
    input wire start_event,
    input wire stop_key,
    input wire arm_on_raw,
    input wire arm_on_stable,
    input wire keys_released,
    input wire est_valid,
    input wire [2:0] est_quality,
    input wire signed [31:0] est_velocity_q16,
    input wire signed [31:0] encoder_position,
    input wire pwm_done,
    input wire pwm_reject,
    output reg [7:0] state,
    output reg [7:0] stop_reason,
    output reg config_loaded,
    output reg [15:0] config_id,
    output reg [15:0] config_crc,
    output reg [1:0] drive_direction,
    output reg [15:0] drive_compare,
    output reg [31:0] drive_periods,
    output reg pwm_start,
    output reg output_allow,
    output reg [7:0] rx_status,
    output wire est_fresh,
    output wire est_ok
);
    localparam [7:0] SAFE = 8'd0, ARMED = 8'd1, RUN = 8'd2, FAULT = 8'd255;
    localparam [7:0] R_COMPLETE = 8'd1, R_STOP_KEY = 8'd2, R_ARM_OFF = 8'd3, R_ABORT = 8'd4,
                     R_ARM_TIMEOUT = 8'd5, R_SENSOR = 8'd16, R_STALE = 8'd17, R_TRAVEL = 8'd18,
                     R_SPEED = 8'd19, R_KEEPALIVE = 8'd20, R_REJECT = 8'd21, R_OVERTIME = 8'd22;
    localparam [3:0] RX_CONFIG = 4'd1, RX_KEEPALIVE = 4'd2, RX_ABORT = 4'd3, RX_STATE = 4'd4,
                     RX_RANGE = 4'd5, RX_COMMAND = 4'd6, RX_FRAME = 4'd7;

    localparam [31:0] MS_LAST = CLOCKS_PER_MS - 1;
    localparam [31:0] STALE = STALE_CLOCKS;

    wire [7:0] c_command = cmd_payload[15:8];
    wire [15:0] c_id = cmd_payload[31:16];
    wire [7:0] c_direction = cmd_payload[39:32];
    wire [7:0] c_reserved = cmd_payload[47:40];
    wire [15:0] c_duty = cmd_payload[63:48];
    wire [15:0] c_pulse = cmd_payload[79:64];
    wire [15:0] c_travel = cmd_payload[95:80];
    wire [15:0] c_speed = cmd_payload[111:96];
    wire [15:0] c_keepalive = cmd_payload[127:112];
    wire c_fields_ok = (c_id != 16'd0) && (c_direction == 8'd1 || c_direction == 8'd2) &&
                       (c_reserved == 8'd0) &&
                       (c_duty != 16'd0) && (c_duty <= MAX_COMPARE) &&
                       (c_pulse != 16'd0) && (c_pulse <= MAX_PULSE_MS) &&
                       (c_travel != 16'd0) && (c_travel <= MAX_TRAVEL_COUNT) &&
                       (c_speed != 16'd0) && (c_speed <= MAX_SPEED_COUNT_S) &&
                       (c_keepalive != 16'd0) && (c_keepalive <= MAX_KEEPALIVE_MS);
    wire control_fields_ok = (cmd_payload[127:16] == 112'd0);
    wire host_abort = cmd_valid && (c_command == 8'd3) && control_fields_ok;

    reg [15:0] travel_limit, speed_limit, keepalive_ms, pulse_ms;
    reg arm_seen_off;
    reg signed [31:0] start_position;
    reg [2:0] last_quality;
    reg signed [31:0] last_velocity;
    reg [31:0] est_age;
    reg [31:0] ka_div, state_div;
    reg [15:0] ka_count;
    reg [31:0] state_ms;
    reg [3:0] frame_count;

    assign est_fresh = (est_age < STALE);
    assign est_ok = est_fresh && (last_quality == 3'd7);
    wire keepalive_fresh = config_loaded && (ka_count < keepalive_ms);
    wire [31:0] travel = encoder_position - start_position;
    wire [31:0] travel_abs = travel[31] ? (32'd0 - travel) : travel;
    wire [31:0] speed_abs = last_velocity[31] ? (32'd0 - last_velocity) : last_velocity;
    wire travel_exceeded = (travel_abs > {16'd0, travel_limit});
    wire speed_exceeded = (speed_abs > {speed_limit, 16'd0});
    wire overtime = (state_ms > {16'd0, pulse_ms} + RUN_GRACE_MS);
    wire arm_timeout = (state_ms >= ARM_TIMEOUT_MS);
    wire running = (state == RUN);
    wire [7:0] trip =
        stop_key ? R_STOP_KEY :
        !arm_on_raw ? R_ARM_OFF :
        (last_quality != 3'd7) ? R_SENSOR :
        !est_fresh ? R_STALE :
        (running && travel_exceeded) ? R_TRAVEL :
        (running && speed_exceeded) ? R_SPEED :
        !keepalive_fresh ? R_KEEPALIVE :
        pwm_reject ? R_REJECT :
        (running && overtime) ? R_OVERTIME :
        host_abort ? R_ABORT :
        ((state == ARMED) && arm_timeout) ? R_ARM_TIMEOUT :
        (running && pwm_done) ? R_COMPLETE : 8'd0;
    wire arm_ready = (state == SAFE) && config_loaded && arm_on_stable && arm_seen_off &&
                     keys_released && est_ok && keepalive_fresh;

    task clear_config;
        begin
            config_loaded <= 1'b0; config_id <= 16'd0; config_crc <= 16'd0;
            drive_direction <= 2'd0; drive_compare <= 16'd0; drive_periods <= 32'd0;
            travel_limit <= 16'd0; speed_limit <= 16'd0; keepalive_ms <= 16'd0; pulse_ms <= 16'd0;
        end
    endtask

    task enter;
        input [7:0] next;
        begin
            state <= next;
            state_div <= 32'd0;
            state_ms <= 32'd0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= SAFE; stop_reason <= 8'd0; pwm_start <= 1'b0; output_allow <= 1'b0;
            rx_status <= 8'd0; frame_count <= 4'd0; arm_seen_off <= 1'b0;
            start_position <= 32'sd0; last_quality <= 3'd0; last_velocity <= 32'sd0;
            est_age <= STALE; ka_div <= 32'd0; ka_count <= 16'hffff;
            state_div <= 32'd0; state_ms <= 32'd0;
            config_loaded <= 1'b0; config_id <= 16'd0; config_crc <= 16'd0;
            drive_direction <= 2'd0; drive_compare <= 16'd0; drive_periods <= 32'd0;
            travel_limit <= 16'd0; speed_limit <= 16'd0; keepalive_ms <= 16'd0; pulse_ms <= 16'd0;
        end else begin
            pwm_start <= 1'b0;

            if (est_valid) begin
                est_age <= 32'd0;
                last_quality <= est_quality;
                last_velocity <= est_velocity_q16;
            end else if (est_age < STALE) est_age <= est_age + 32'd1;

            if (cmd_valid) begin
                ka_div <= 32'd0; ka_count <= 16'd0;
            end else if (ka_count != 16'hffff) begin
                if (ka_div == MS_LAST) begin ka_div <= 32'd0; ka_count <= ka_count + 16'd1; end
                else ka_div <= ka_div + 32'd1;
            end

            if (state_div == MS_LAST) begin
                state_div <= 32'd0;
                if (state_ms != 32'hffffffff) state_ms <= state_ms + 32'd1;
            end else state_div <= state_div + 32'd1;

            if (!arm_on_stable) arm_seen_off <= 1'b1;

            if (cmd_valid) begin
                frame_count <= frame_count + 4'd1;
                rx_status[7:4] <= frame_count + 4'd1;
            end
            if (cmd_error) rx_status[3:0] <= RX_FRAME;
            else if (cmd_valid) begin
                case (c_command)
                    8'd1: begin
                        if (state != SAFE) rx_status[3:0] <= RX_STATE;
                        else if (!c_fields_ok) begin
                            rx_status[3:0] <= RX_RANGE;
                            clear_config;
                        end else begin
                            rx_status[3:0] <= RX_CONFIG;
                            config_loaded <= 1'b1; config_id <= c_id; config_crc <= cmd_crc;
                            drive_direction <= c_direction[1:0]; drive_compare <= c_duty;
                            drive_periods <= c_pulse * PERIODS_PER_MS;
                            pulse_ms <= c_pulse; travel_limit <= c_travel;
                            speed_limit <= c_speed; keepalive_ms <= c_keepalive;
                        end
                    end
                    8'd2: rx_status[3:0] <= control_fields_ok ? RX_KEEPALIVE : RX_RANGE;
                    8'd3: begin
                        rx_status[3:0] <= control_fields_ok ? RX_ABORT : RX_RANGE;
                        if (control_fields_ok && state == SAFE) clear_config;
                    end
                    default: rx_status[3:0] <= RX_COMMAND;
                endcase
            end

            case (state)
                SAFE: begin
                    output_allow <= 1'b0;
                    // A CONFIG accepted/rejected on this edge must take effect
                    // before evaluating arming, rather than using old fields.
                    if (arm_ready && !host_abort && !(cmd_valid && c_command == 8'd1)) begin
                        enter(ARMED);
                        arm_seen_off <= 1'b0;
                    end
                end
                ARMED: begin
                    if (trip != 8'd0) begin
                        output_allow <= 1'b0;
                        stop_reason <= trip;
                        clear_config;
                        enter(trip >= R_SENSOR ? FAULT : SAFE);
                    end else if (start_event) begin
                        enter(RUN);
                        start_position <= encoder_position;
                        output_allow <= 1'b1;
                        pwm_start <= 1'b1;
                    end
                end
                RUN: begin
                    if (trip != 8'd0) begin
                        output_allow <= 1'b0;
                        stop_reason <= trip;
                        clear_config;
                        enter(trip >= R_SENSOR ? FAULT : SAFE);
                    end
                end
                default: begin
                    output_allow <= 1'b0;
                    state <= FAULT;
                end
            endcase
        end
    end
endmodule
