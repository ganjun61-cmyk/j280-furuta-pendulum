`timescale 1ns/1ps

// Serial restoring divider, one quotient bit per clock. Result is the
// magnitude quotient truncated toward zero, then negated when requested;
// overflow is set when that signed value does not fit int32.
module j280_signed_divider #(
    parameter integer NUM_WIDTH = 64,
    parameter integer DEN_WIDTH = 32
) (
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire start,
    input wire negative,
    input wire [NUM_WIDTH-1:0] magnitude,
    input wire [DEN_WIDTH-1:0] divisor,
    output reg done,
    output reg signed [31:0] quotient,
    output reg overflow
);
    reg active;
    reg neg;
    reg [7:0] count;
    reg [NUM_WIDTH-1:0] num;
    reg [NUM_WIDTH-1:0] q;
    reg [DEN_WIDTH-1:0] den;
    reg [DEN_WIDTH-1:0] rem;

    wire [DEN_WIDTH:0] trial = {rem, num[NUM_WIDTH-1]};
    wire [DEN_WIDTH:0] reduced = trial - {1'b0, den};
    wire take = (trial >= {1'b0, den});
    wire [DEN_WIDTH-1:0] rem_next = take ? reduced[DEN_WIDTH-1:0] : trial[DEN_WIDTH-1:0];
    wire [NUM_WIDTH-1:0] q_next = {q[NUM_WIDTH-2:0], take};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0; neg <= 1'b0; count <= 8'd0;
            num <= {NUM_WIDTH{1'b0}}; q <= {NUM_WIDTH{1'b0}};
            den <= {DEN_WIDTH{1'b0}}; rem <= {DEN_WIDTH{1'b0}};
            done <= 1'b0; quotient <= 32'sd0; overflow <= 1'b0;
        end else begin
            done <= 1'b0;
            if (clear) begin
                active <= 1'b0;
            end else if (start) begin
                active <= 1'b1; neg <= negative; count <= 8'd0;
                num <= magnitude; q <= {NUM_WIDTH{1'b0}};
                den <= divisor; rem <= {DEN_WIDTH{1'b0}};
            end else if (active) begin
                rem <= rem_next;
                q <= q_next;
                num <= {num[NUM_WIDTH-2:0], 1'b0};
                count <= count + 8'd1;
                if (count == NUM_WIDTH - 1) begin
                    active <= 1'b0;
                    done <= 1'b1;
                    overflow <= (|q_next[NUM_WIDTH-1:32]) ||
                                (neg ? (q_next[31] && (|q_next[30:0])) : q_next[31]);
                    quotient <= neg ? -q_next[31:0] : q_next[31:0];
                end
            end
        end
    end
endmodule
