`timescale 1ns/1ps
// Passive instrumentation only: there is no motor enable or UART RX path.
module j280_measure_top (
    input wire clk_50m,
    input wire rst_n,
    input wire [9:0] adc_data,
    input wire adc_otr,
    input wire enc1_a, enc1_b,
    output wire adc_clk, adc_oe_n,
    output wire uart_tx,
    output wire pwm_a, ain1, ain2, pwm_b, bin1, bin2,
    output wire [1:0] led
);
    assign {pwm_a,ain1,ain2,pwm_b,bin1,bin2} = 6'b000000;
    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_pipe;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) reset_pipe <= 0;
        else reset_pipe <= {reset_pipe[0],1'b1};
    end
    wire reset_ok = reset_pipe[1];
    wire [9:0] raw;
    wire adc_valid, adc_strobe;
    wire [15:0] adc_age;
    j280_adc_capture adc_sampler(.clk(clk_50m),.rst_n(reset_ok),
        .adc_data(adc_data),.adc_otr(adc_otr),.adc_clk(adc_clk),
        .adc_oe_n(adc_oe_n),.raw(raw),.valid(adc_valid),
        .sample_strobe(adc_strobe),.age_us(adc_age));
    wire signed [31:0] position;
    wire [31:0] illegal_count;
    wire encoder_ready;
    j280_encoder encoder(.clk(clk_50m),.rst_n(reset_ok),
        .enc_a(enc1_a),.enc_b(enc1_b),.position(position),
        .illegal_count(illegal_count),.ready(encoder_ready));

    reg [5:0] us_divider;
    reg [31:0] tick_us;
    reg [15:0] output_age_us;
    wire [31:0] tick_next = (us_divider==49) ? tick_us+32'd1 : tick_us;
    reg [16:0] sample_divider;
    reg [31:0] frame_sequence;
    reg adc_new;
    reg snapshot_valid;
    reg [223:0] payload;
    wire tx_busy, tx_overflow;
    wire [7:0] flags = {4'b0,tx_overflow,1'b0,adc_new,adc_valid};
    wire [15:0] faults = {13'b0,tx_overflow,(|illegal_count),!adc_valid};
    wire [15:0] pwm_age = ((us_divider==49) && (output_age_us!=16'hffff))
                         ? output_age_us+16'd1 : output_age_us;

    always @(posedge clk_50m or negedge reset_ok) begin
        if (!reset_ok) begin
            us_divider<=0; tick_us<=0; output_age_us<=0;
            sample_divider<=0; frame_sequence<=0;
            adc_new<=0; snapshot_valid<=0; payload<=0;
        end else begin
            if (us_divider==49) us_divider<=0;
            else us_divider<=us_divider+1'b1;
            tick_us<=tick_next;
            output_age_us<=pwm_age;
            snapshot_valid<=0;
            if (sample_divider==99999) begin
                sample_divider<=0;
                payload<={pwm_age,faults,8'd1,8'd0,16'd0,16'd0,
                          position,adc_age,{6'd0,raw},tick_next,frame_sequence,
                          flags,8'd1};
                frame_sequence<=frame_sequence+1'b1;
                snapshot_valid<=1;
                adc_new<=0;
            end else sample_divider<=sample_divider+1'b1;
            // A sample delivered on this edge belongs to the NEXT snapshot.
            if (adc_strobe) adc_new<=1;
        end
    end
    j280_frame_tx transmitter(.clk(clk_50m),.rst_n(reset_ok),
        .snapshot_valid(snapshot_valid),.payload(payload),.uart_tx(uart_tx),
        .overflow(tx_overflow),.busy(tx_busy));
    assign led[0]=adc_valid && encoder_ready;
    assign led[1]=tick_us[19];
endmodule
