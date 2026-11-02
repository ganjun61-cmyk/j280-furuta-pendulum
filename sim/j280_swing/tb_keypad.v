`timescale 1ns/1ps
module tb_keypad;
 reg clk=0,rst_n=0,is_balance=1,traj_busy=0;
 reg [3:0] row_n;
 wire [3:0] col_n;
 wire key_go; wire signed [15:0] key_delta; wire [15:0] key_speed;
 wire [8:0] entry_angle; wire key_dir; wire [1:0] speed_sel,entry_state; wire [3:0] last_key;
 reg pressed=0; reg [1:0] key_row=0,key_col=0;
 j280_keypad #(.SCAN_DIV(125),.DEBOUNCE(4)) dut(.clk(clk),.rst_n(rst_n),
  .row_n(row_n),.col_n(col_n),.is_balance(is_balance),.traj_busy(traj_busy),
  .key_go(key_go),.key_delta(key_delta),.key_speed(key_speed),
  .entry_angle(entry_angle),.key_dir(key_dir),.speed_sel(speed_sel),
  .entry_state(entry_state),.last_key(last_key));
 always #10 clk=~clk;
 always @* begin
  row_n=4'hF;
  if(pressed) begin
   case(col_n)
    4'b1110: if(key_col==0) row_n[key_row]=0;
    4'b1101: if(key_col==1) row_n[key_row]=0;
    4'b1011: if(key_col==2) row_n[key_row]=0;
    4'b0111: if(key_col==3) row_n[key_row]=0;
   endcase
  end
 end
 integer go_count=0; reg signed [15:0] go_delta=0; reg [15:0] go_speed=0;
 always @(posedge clk) if(key_go) begin go_count<=go_count+1; go_delta<=key_delta; go_speed<=key_speed; end
 task press(input [1:0] r, input [1:0] c);
  begin
   @(negedge clk); key_row=r; key_col=c; pressed=1;
   #200000; // 200 us
   pressed=0; #200000;
  end
 endtask
 initial begin
  #100 rst_n=1; #1000;
  press(2'd0,2'd0); // 1
  press(2'd0,2'd1); // 2
  press(2'd3,2'd1); // 0  -> 120
  press(2'd2,2'd3); // C  -> 90 deg/s
  press(2'd3,2'd2); // #  -> CW
  press(2'd3,2'd3); // D  -> submit
  #200000;
  $display("angle=%0d dir=%0d speed_sel=%0d go=%0d delta=%0d speed=%0d state=%0d last=%0d",
    entry_angle,key_dir,speed_sel,go_count,go_delta,go_speed,entry_state,last_key);
  if(go_count==1 && go_delta==16'sd347 && go_speed==16'd34080 && entry_angle==9'd120) $display("PASS");
  else $display("FAIL");
  $finish;
 end
endmodule
