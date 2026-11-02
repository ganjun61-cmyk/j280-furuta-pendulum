`timescale 1ns/1ps
module tb_balance_cmd_rx;
    parameter integer BIT_NS=4340;
    reg clk=0,rst_n=0,rx=1;
    always #10 clk=~clk;
    wire [7:0] data;
    wire byte_valid,byte_error,frame_valid,frame_error;
    wire [1503:0] payload;
    wire [15:0] crc;
    j280_uart_rx uart(.clk(clk),.rst_n(rst_n),.rx(rx),.data(data),
                     .valid(byte_valid),.frame_error(byte_error));
    j280_balance_cmd_rx parser(.clk(clk),.rst_n(rst_n),.byte_valid(byte_valid),
        .byte_data(data),.byte_error(byte_error),.frame_valid(frame_valid),
        .frame_payload(payload),.frame_crc(crc),.frame_error(frame_error));
    integer script,log,rc,i;
    reg [7:0] op;
    reg [31:0] value,idle_ns;
    always @(posedge clk) begin
        if(byte_valid) $fdisplay(log,"D %02x",data);
        if(byte_error) $fdisplay(log,"R");
        if(frame_valid) $fdisplay(log,"V %0376x %04x",payload,crc);
        if(frame_error) $fdisplay(log,"X");
    end
    task send_byte;
        input [7:0] b;
        input stop;
        begin
            rx=0; #BIT_NS;
            for(i=0;i<8;i=i+1) begin rx=b[i]; #BIT_NS; end
            rx=stop; #BIT_NS; rx=1;
        end
    endtask
    initial begin
        log=$fopen("rx_log.txt","w"); script=$fopen("rx_script.txt","r");
        if(!log || !script) $fatal(1,"files");
        #95; rst_n=1; #1000;
        forever begin
            rc=$fscanf(script," %c",op);
            if(rc!=1 || op=="E") begin
                #(20*BIT_NS); $fclose(log);
                $display("PASS tb_balance_cmd_rx"); $finish;
            end
            if(op=="B" || op=="F") begin
                rc=$fscanf(script," %h %d",value,idle_ns);
                #idle_ns; send_byte(value[7:0],op=="B");
            end else if(op=="G") begin
                rc=$fscanf(script," %d %d",value,idle_ns);
                #idle_ns; rx=0; #value; rx=1;
            end else $fatal(1,"script");
        end
    end
    initial begin #300000000; $fatal(1,"watchdog"); end
endmodule
