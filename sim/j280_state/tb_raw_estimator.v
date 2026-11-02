`timescale 1ns/1ps
// File-driven core test: 0=submit/wait, 1=submit without waiting,
// 2=reset (also checks all public outputs), 3=drain with no further results.
module tb_raw_estimator;
    parameter WINDOW_US=10000, PERIOD_US=2000, MAX_ADC_AGE_US=100;
    parameter MAX_ADC_STEP=64, MAX_ENCODER_STEP=64, HISTORY_DEPTH=8;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0, sample_valid=0;
    reg [223:0] sample_payload=0;
    reg [63:0] sample_tick_us=0;
    wire sample_ready, result_valid;
    wire [223:0] result_payload;
    wire [63:0] result_tick_us, adc_position_time_us;
    wire [31:0] segment_id, encoder_span_us, adc_span_us;
    wire signed [63:0] encoder_position;
    wire signed [31:0] encoder_velocity_q16, adc_velocity_q16;
    wire [63:0] encoder_midpoint_x2_us, adc_midpoint_x2_us;
    wire [31:0] encoder_age_x2_us, adc_age_x2_us;
    wire [2:0] quality;
    wire [7:0] reason;
    j280_raw_estimator #(.WINDOW_US(WINDOW_US),.PERIOD_US(PERIOD_US),
        .MAX_ADC_AGE_US(MAX_ADC_AGE_US),.MAX_ADC_STEP(MAX_ADC_STEP),
        .MAX_ENCODER_STEP(MAX_ENCODER_STEP),.HISTORY_DEPTH(HISTORY_DEPTH)) dut (
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),
        .sample_payload(sample_payload),.sample_tick_us(sample_tick_us),
        .sample_ready(sample_ready),.result_valid(result_valid),
        .result_payload(result_payload),.result_tick_us(result_tick_us),
        .segment_id(segment_id),.encoder_position(encoder_position),
        .encoder_velocity_q16(encoder_velocity_q16),.adc_velocity_q16(adc_velocity_q16),
        .encoder_span_us(encoder_span_us),.adc_span_us(adc_span_us),
        .adc_position_time_us(adc_position_time_us),
        .encoder_midpoint_x2_us(encoder_midpoint_x2_us),
        .adc_midpoint_x2_us(adc_midpoint_x2_us),
        .encoder_age_x2_us(encoder_age_x2_us),.adc_age_x2_us(adc_age_x2_us),
        .quality(quality),.reason(reason));
    reg [2047:0] vectors_path, results_path;
    integer vectors, results, rc, op, gap, waited, cycle=0;
    reg [63:0] next_tick;
    reg [223:0] next_payload;
    reg last_valid=0;
    always @(posedge clk) begin
        cycle=cycle+1;
        #1;
        if (rst_n && result_valid) begin
            if (last_valid) begin $display("FAIL result_valid wider than one clock"); $finish; end
            $fdisplay(results,"%0d %056h %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                cycle,result_payload,result_tick_us,segment_id,encoder_position,
                encoder_velocity_q16,adc_velocity_q16,encoder_span_us,adc_span_us,
                adc_position_time_us,encoder_midpoint_x2_us,adc_midpoint_x2_us,
                encoder_age_x2_us,adc_age_x2_us,quality,reason);
        end
        last_valid=rst_n && result_valid;
    end
    task check_reset;
        begin
            #1;
            if ({result_valid,result_payload,result_tick_us,segment_id,encoder_position,
                 encoder_velocity_q16,adc_velocity_q16,encoder_span_us,adc_span_us,
                 adc_position_time_us,encoder_midpoint_x2_us,adc_midpoint_x2_us,
                 encoder_age_x2_us,adc_age_x2_us,quality,reason} !== 0 || !sample_ready) begin
                $display("FAIL reset outputs"); $finish;
            end
        end
    endtask
    initial begin
        if (!$value$plusargs("VECTORS=%s",vectors_path) ||
            !$value$plusargs("RESULTS=%s",results_path)) begin
            $display("FAIL missing file arguments"); $finish;
        end
        vectors=$fopen(vectors_path,"r"); results=$fopen(results_path,"w");
        if (!vectors || !results) begin $display("FAIL file open"); $finish; end
        repeat(3) @(negedge clk);
        check_reset;
        rst_n=1;
        while (!$feof(vectors)) begin
            rc=$fscanf(vectors,"%d %d %h %h\n",op,gap,next_tick,next_payload);
            if (rc!=4) begin $display("FAIL vector syntax"); $finish; end
            repeat(gap) @(negedge clk);
            if (op==2) begin
                rst_n=0; sample_valid=0;
                repeat(2) @(negedge clk);
                check_reset;
                rst_n=1;
            end else if (op==3) begin
                repeat(520) begin
                    @(negedge clk);
                    if (result_valid) begin $display("FAIL stale completion"); $finish; end
                end
            end else begin
                if (op==0 && !sample_ready) begin $display("FAIL not ready before submit"); $finish; end
                sample_valid=1; sample_payload=next_payload; sample_tick_us=next_tick;
                @(negedge clk);
                sample_valid=0;
                if (op==0) begin
                    waited=1;
                    while (!result_valid && waited<500) begin
                        @(negedge clk); waited=waited+1;
                    end
                    if (!result_valid) begin $display("FAIL latency >=500"); $finish; end
                    $display("LATENCY %0d",waited);
                end
            end
        end
        repeat(3) @(negedge clk);
        $fclose(vectors); $fclose(results);
        $display("PASS tb_raw_estimator"); $finish;
    end
    initial begin #2000000000; $display("FAIL watchdog"); $finish; end
endmodule
