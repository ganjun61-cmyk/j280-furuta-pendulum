`timescale 1ns/1ps

// 4x4 membrane keypad scanner and command entry.
// Physical layout (row 0 top, column 0 left):
//   1 2 3 A
//   4 5 6 B
//   7 8 9 C
//   * 0 # D
// Digits build a 0..360 angle. '#' = CW, '*' = CCW, A/B/C = 30/60/90 deg/s,
// D = submit. A submission becomes a relative arm move of angle/360 rev in the
// selected direction, at the selected speed, only while balancing and idle.
//
// Key codes: 0..9 digits, 10 = '#', 11 = '*', 12 = A, 13 = B, 14 = C, 15 = D.
// Columns are driven low one at a time (250 us each, 1 ms frame), rows are
// pulled up externally/in the CST and synchronized here. A key must be stable
// for DEBOUNCE frames (16 ms) before it is accepted.
module j280_keypad #(
    parameter integer SCAN_DIV = 12500,
    parameter integer DEBOUNCE = 16
) (
    input wire clk,
    input wire rst_n,
    input wire [3:0] row_n,
    output reg [3:0] col_n,
    input wire is_balance,
    input wire traj_busy,
    output reg key_go,
    output reg signed [15:0] key_delta,
    output reg [15:0] key_speed,
    output reg [8:0] entry_angle,
    output reg key_dir,
    output reg [1:0] speed_sel,
    output reg [1:0] entry_state,
    output reg [3:0] last_key
);
    localparam [1:0] ST_IDLE = 0, ST_ENTRY = 1, ST_MOVING = 2, ST_ERROR = 3;
    // 30/60/90 deg/s as Q16 counts per 2 ms sample: deg*1040/360/500*65536.
    localparam [15:0] SPEED_30 = 16'd11360;
    localparam [15:0] SPEED_60 = 16'd22720;
    localparam [15:0] SPEED_90 = 16'd34080;

    // ---- matrix scan ----
    reg [13:0] scan_cnt;
    reg [1:0] col_idx;
    reg [3:0] row_s1, row_s2;
    wire [3:0] rows = ~row_s2;
    reg [15:0] matrix, cand_keys, stable_keys;
    reg [4:0] db_cnt;
    reg [9:0] hold_cnt;
    reg hold_pulse;
    reg frame_tick;
    reg key_press;
    reg [3:0] key_code;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= 0; col_idx <= 0; col_n <= 4'b1110;
            row_s1 <= 0; row_s2 <= 0; matrix <= 0; frame_tick <= 0;
        end else begin
            row_s1 <= row_n; row_s2 <= row_s1;
            frame_tick <= 0;
            if (scan_cnt == SCAN_DIV - 1) begin
                scan_cnt <= 0;
                case (col_idx)
                    2'd0: matrix[0+:4] <= rows;
                    2'd1: matrix[4+:4] <= rows;
                    2'd2: matrix[8+:4] <= rows;
                    default: begin matrix[12+:4] <= rows; frame_tick <= 1; end
                endcase
                if (col_idx == 2'd3) begin
                    col_idx <= 0; col_n <= 4'b1110;
                end else begin
                    col_idx <= col_idx + 2'd1;
                    col_n <= ~(4'b0001 << (col_idx + 2'd1));
                end
            end else scan_cnt <= scan_cnt + 14'd1;
        end
    end

    // ---- debounce and key code ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_keys <= 0; stable_keys <= 0; db_cnt <= 0;
            hold_cnt <= 0; hold_pulse <= 0;
            key_press <= 0; key_code <= 0;
        end else begin
            key_press <= 0; hold_pulse <= 0;
            if (frame_tick) begin
                // Hold a digit for ~0.5 s to restart the entry from that digit.
                if (cand_keys != 0 && cand_keys == stable_keys) begin
                    if (hold_cnt != 10'd1023) hold_cnt <= hold_cnt + 10'd1;
                    if (hold_cnt == 10'd499) hold_pulse <= 1;
                end else hold_cnt <= 0;
                if (matrix == cand_keys) begin
                    if (db_cnt != DEBOUNCE - 1) db_cnt <= db_cnt + 5'd1;
                    else begin
                        if (cand_keys != 0 && (cand_keys & (cand_keys - 16'd1)) == 0) begin
                            if (cand_keys != stable_keys) begin
                                key_press <= 1;
                                stable_keys <= cand_keys;
                                case (cand_keys)
                                    16'h0001: key_code <= 4'd1;
                                    16'h0002: key_code <= 4'd4;
                                    16'h0004: key_code <= 4'd7;
                                    16'h0008: key_code <= 4'd11; // *
                                    16'h0010: key_code <= 4'd2;
                                    16'h0020: key_code <= 4'd5;
                                    16'h0040: key_code <= 4'd8;
                                    16'h0080: key_code <= 4'd0;
                                    16'h0100: key_code <= 4'd3;
                                    16'h0200: key_code <= 4'd6;
                                    16'h0400: key_code <= 4'd9;
                                    16'h0800: key_code <= 4'd10; // #
                                    16'h1000: key_code <= 4'd12; // A
                                    16'h2000: key_code <= 4'd13; // B
                                    16'h4000: key_code <= 4'd14; // C
                                    default:  key_code <= 4'd15; // D
                                endcase
                            end
                        end else if (cand_keys == 0) stable_keys <= 0;
                        // multiple keys held: no event and no stable update
                    end
                end else begin
                    cand_keys <= matrix; db_cnt <= 0;
                end
            end
        end
    end

    // ---- entry / command FSM ----
    reg move_seen, entry_fresh;
    reg div_busy, go_pending;
    reg [13:0] div_rem, div_num;
    reg [10:0] div_quot;
    wire [12:0] digit_next = {4'd0, entry_angle} * 10 + {9'd0, key_code};
    wire [13:0] go_num = {5'd0, entry_angle} * 26 + 14'd4;  // round(angle*1040/360)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_go <= 0; key_delta <= 0; key_speed <= SPEED_60;
            entry_angle <= 0; key_dir <= 0; speed_sel <= 1; entry_state <= ST_IDLE;
            last_key <= 0; move_seen <= 0; entry_fresh <= 0;
            div_busy <= 0; go_pending <= 0; div_rem <= 0; div_num <= 0; div_quot <= 0;
        end else begin
            key_go <= 0;
            if (hold_pulse && key_code <= 4'd9 && is_balance) begin
                entry_angle <= {5'd0, key_code};
                entry_state <= ST_ENTRY;
                entry_fresh <= 0;
            end
            if (key_press) begin
                last_key <= key_code;
                if (key_code <= 4'd9) begin
                    entry_fresh <= 0;
                    if (entry_fresh || entry_state == ST_MOVING)
                        entry_angle <= {5'd0, key_code};
                    else if (digit_next <= 13'd360) entry_angle <= digit_next[8:0];
                    else entry_angle <= {5'd0, key_code}; // overflow: restart from this digit
                    entry_state <= ST_ENTRY;
                end else if (key_code == 4'd10 || key_code == 4'd11) begin
                    key_dir <= (key_code == 4'd11);
                end else if (key_code == 4'd12) begin
                    speed_sel <= 0; key_speed <= SPEED_30;
                end else if (key_code == 4'd13) begin
                    speed_sel <= 1; key_speed <= SPEED_60;
                end else if (key_code == 4'd14) begin
                    speed_sel <= 2; key_speed <= SPEED_90;
                end else if (key_code == 4'd15) begin
                    if (is_balance && !traj_busy && entry_angle != 0 && !div_busy && !go_pending) begin
                        go_pending <= 1; entry_state <= ST_MOVING; entry_fresh <= 1;
                    end
                end
            end
            if (go_pending && !div_busy) begin
                go_pending <= 0; div_busy <= 1;
                div_num <= go_num; div_rem <= go_num; div_quot <= 0;
            end else if (div_busy) begin
                if (div_rem >= 14'd9) begin
                    div_rem <= div_rem - 14'd9;
                    div_quot <= div_quot + 11'd1;
                end else begin
                    div_busy <= 0;
                    key_delta <= key_dir ? -$signed({5'd0, div_quot}) :
                                          $signed({5'd0, div_quot});
                    key_go <= 1; move_seen <= 0;
                end
            end
            if (entry_state == ST_MOVING) begin
                if (traj_busy) move_seen <= 1;
                else if (move_seen) entry_state <= ST_IDLE;
            end
        end
    end
endmodule
