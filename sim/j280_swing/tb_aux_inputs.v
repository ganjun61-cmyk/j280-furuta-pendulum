`timescale 1ns/1ps
// File-driven j280_aux_inputs check: SW3/SW4 press events and DIP 2 state.
//   W n            : advance n clocks (inputs held)
//   I name value   : set sw3_n/sw4_n/dip2 (1 = released/off), applied before the next clock
//   E name value   : expect an output level (sw3_event/sw4_event/dip2_on) on this clock
//   P n            : run n clocks and count the SW3 and SW4 events
//   C n            : run n clocks and count SW3 events only
//   D n            : run n clocks and count SW4 events only
//   R              : pulse reset low for 3 clocks, then run 2 clocks
//   Q              : finish
module tb_aux_inputs;
    parameter integer DEBOUNCE_CLOCKS = 5;
    parameter integer DIP_ACTIVE_LOW = 1;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0, sw3_n=1, sw4_n=1, dip2=1;
    wire sw3_event, sw4_event, dip2_on;
    wire [2:0] levels;
    j280_aux_inputs #(.DEBOUNCE_CLOCKS(DEBOUNCE_CLOCKS),.DIP_ACTIVE_LOW(DIP_ACTIVE_LOW)) dut(
        .clk(clk),.rst_n(rst_n),.sw3_n(sw3_n),.sw4_n(sw4_n),.dip2(dip2),
        .sw3_event(sw3_event),.sw4_event(sw4_event),.dip2_on(dip2_on),.levels(levels));
    integer fd, rc, line=0, n, value, events, i;
    reg [255:0] op, name;
    reg [2047:0] vector_path;
    task fail;
        input [511:0] message;
        begin
            $display("FAIL line=%0d %0s sw3=%0d sw4=%0d dip2_on=%0d levels=%b",
                     line, message, sw3_event, sw4_event, dip2_on, levels);
            $finish;
        end
    endtask
    initial begin
        if(!$value$plusargs("VECTORS=%s", vector_path)) fail("missing vector file");
        fd=$fopen(vector_path,"r");
        if(!fd) fail("cannot open vector file");
        @(negedge clk);
        repeat(3) @(negedge clk);
        rst_n=1;
        repeat(2) @(negedge clk);
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%s",op); line=line+1;
            if(rc==1) case(op)
                "W": begin rc=$fscanf(fd,"%d",n); for(i=0;i<n;i=i+1) @(negedge clk); end
                "I": begin
                    rc=$fscanf(fd,"%s %d",name,value);
                    case(name)
                        "sw3_n": sw3_n=value[0];
                        "sw4_n": sw4_n=value[0];
                        "dip2": dip2=value[0];
                        default: fail("unknown input name");
                    endcase
                end
                "E": begin
                    rc=$fscanf(fd,"%s %d",name,value);
                    @(negedge clk);
                    case(name)
                        "sw3_event": if(sw3_event!==value[0]) fail("sw3_event mismatch");
                        "sw4_event": if(sw4_event!==value[0]) fail("sw4_event mismatch");
                        "dip2_on": if(dip2_on!==value[0]) fail("dip2_on mismatch");
                        "levels": if(levels!==value[2:0]) fail("levels mismatch");
                        default: fail("unknown expected name");
                    endcase
                end
                "P": begin
                    rc=$fscanf(fd,"%d",n);
                    events=0;
                    for(i=0;i<n;i=i+1) begin
                        @(negedge clk);
                        if(sw3_event) events=events+1;
                        if(sw4_event) events=events+10;
                    end
                    $display("EVENTS %0d",events);
                end
                "C": begin
                    rc=$fscanf(fd,"%d",n);
                    events=0;
                    for(i=0;i<n;i=i+1) begin
                        @(negedge clk);
                        if(sw3_event) events=events+1;
                    end
                    $display("EVENTS %0d",events);
                end
                "D": begin
                    rc=$fscanf(fd,"%d",n);
                    events=0;
                    for(i=0;i<n;i=i+1) begin
                        @(negedge clk);
                        if(sw4_event) events=events+1;
                    end
                    $display("EVENTS %0d",events);
                end
                "R": begin
                    @(negedge clk); rst_n=0;
                    repeat(3) @(negedge clk);
                    rst_n=1;
                    repeat(2) @(negedge clk);
                end
                "Q": begin $display("PASS tb_aux_inputs lines=%0d",line); $finish; end
                default: fail("unknown opcode");
            endcase
        end
        fail("vectors ended without Q");
    end
    initial begin #20000000; fail("watchdog"); end
endmodule
