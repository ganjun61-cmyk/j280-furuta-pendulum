`timescale 1ns/1ps

// Four counts per A/B cycle; 00 -> 10 -> 11 -> 01 -> 00 is positive.
module j280_encoder (
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
    reg [2:0] warmup;
    reg [1:0] previous_ab;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 2'b00;
            ab_sync <= 2'b00;
            warmup <= 3'b000;
            previous_ab <= 2'b00;
            position <= 32'sd0;
            illegal_count <= 32'd0;
            ready <= 1'b0;
        end else begin
            ab_meta <= {enc_a, enc_b};
            ab_sync <= ab_meta;
            warmup <= {warmup[1:0], 1'b1};
            if (!ready) begin
                // The synchronizer must settle before choosing the initial AB.
                // A static 11 during reset therefore creates no false edge.
                if (warmup[2]) begin
                    previous_ab <= ab_sync;
                    ready <= 1'b1;
                end
            end else begin
                previous_ab <= ab_sync;
                case ({previous_ab, ab_sync})
                    4'b0010, 4'b1011, 4'b1101, 4'b0100:
                        position <= position + 32'sd1;
                    4'b0001, 4'b0111, 4'b1110, 4'b1000:
                        position <= position - 32'sd1;
                    4'b0011, 4'b1100, 4'b0110, 4'b1001: begin
                        // Resynchronize to the new state without inventing a count.
                        if (illegal_count != 32'hffffffff)
                            illegal_count <= illegal_count + 1'b1;
                    end
                    default: begin end
                endcase
            end
        end
    end
endmodule
