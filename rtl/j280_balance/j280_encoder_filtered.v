`timescale 1ns/1ps

// Quadrature counter with a per-line stability filter. Motor PWM edges share
// the XH1 cable and couple common-mode glitches onto A and B; a level change
// is accepted only after it has been stable for FILTER_CLOCKS consecutive
// clocks. Counting and illegal-transition detection then match j280_encoder.
// 00 -> 10 -> 11 -> 01 -> 00 is positive.
module j280_encoder_filtered #(
    parameter integer FILTER_CLOCKS = 100
) (
    input wire clk,
    input wire rst_n,
    input wire enc_a,
    input wire enc_b,
    output reg signed [31:0] position,
    output reg [31:0] illegal_count,
    output reg ready
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] ab_meta;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ab_sync;
    reg [1:0] ab_stable;
    reg [15:0] count_a, count_b, warm;
    reg [1:0] previous_ab;
    // By then a line held constant through reset has passed the filter.
    wire warm_done = (warm == FILTER_CLOCKS + 3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 2'b00; ab_sync <= 2'b00; ab_stable <= 2'b00;
            count_a <= 16'd0; count_b <= 16'd0; warm <= 16'd0;
            previous_ab <= 2'b00; position <= 32'sd0; illegal_count <= 32'd0; ready <= 1'b0;
        end else begin
            ab_meta <= {enc_a, enc_b};
            ab_sync <= ab_meta;
            if (!warm_done) warm <= warm + 16'd1;
            if (ab_sync[1] == ab_stable[1]) count_a <= 16'd0;
            else if (count_a == FILTER_CLOCKS - 1) begin ab_stable[1] <= ab_sync[1]; count_a <= 16'd0; end
            else count_a <= count_a + 16'd1;
            if (ab_sync[0] == ab_stable[0]) count_b <= 16'd0;
            else if (count_b == FILTER_CLOCKS - 1) begin ab_stable[0] <= ab_sync[0]; count_b <= 16'd0; end
            else count_b <= count_b + 16'd1;
            if (!ready) begin
                // Choose the initial state only once nothing is pending, so a
                // static level during reset creates no count.
                if (warm_done && ab_sync == ab_stable) begin
                    previous_ab <= ab_stable;
                    ready <= 1'b1;
                end
            end else begin
                previous_ab <= ab_stable;
                case ({previous_ab, ab_stable})
                    4'b0010, 4'b1011, 4'b1101, 4'b0100:
                        position <= position + 32'sd1;
                    4'b0001, 4'b0111, 4'b1110, 4'b1000:
                        position <= position - 32'sd1;
                    4'b0011, 4'b1100, 4'b0110, 4'b1001: begin
                        if (illegal_count != 32'hffffffff)
                            illegal_count <= illegal_count + 1'b1;
                    end
                    default: begin end
                endcase
            end
        end
    end
endmodule
