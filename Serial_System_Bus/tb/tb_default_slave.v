//==========================================================================
// tb_default_slave.v -- self-checking testbench for default_slave
//
// Covers:
//   1. reset behaviour     - ready low, no response
//   2. selected access     - ready exactly one cycle after sel, resp = ERROR
//   2b. never drives the wire - dstream_out stays 0 at all times, so an
//                            unmapped read leaves the shared data wire idle
//                            and the master reassembles zero
//   3. one-cycle ready     - ready does not stick high
//   4. idle                - ready never rises without sel
//   5. back-to-back        - two consecutive selects both answer ERROR, i.e.
//                            the bus keeps making progress through a run of
//                            bad addresses instead of stalling
//==========================================================================
`timescale 1ns/1ps
`include "../rtl/bus_defs.vh"

module tb_default_slave;

    localparam RESP_W = `BUS_RESP_W;

    reg clk = 1'b0;
    reg rst_n;
    reg sel;
    wire              dstream_out;
    wire              ready;
    wire [RESP_W-1:0] resp;

    integer errors = 0;
    integer i;

    always #10 clk = ~clk;

    default_slave #(.RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n), .sel(sel),
        .dstream_out(dstream_out), .ready(ready), .resp(resp)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t ready=%b resp=%b)",
                         name, $time, ready, resp);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_default_slave");
        $display("======================================================");

        rst_n = 1'b0;
        sel   = 1'b0;

        $display("-- 1. reset behaviour --------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(ready === 1'b0, "ready low in reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (2) @(posedge clk); #1;
        chk(ready === 1'b0, "ready low after reset release");

        $display("-- 2. unmapped access answers ERROR ------------------");
        @(posedge clk); sel <= 1'b1;
        @(posedge clk); sel <= 1'b0; #1;
        chk(ready === 1'b1,        "ready one cycle after sel");
        chk(resp  === `RESP_ERROR, "resp = ERROR");
        chk(dstream_out === 1'b0, "never drives the shared data wire");

        $display("-- 3. ready is one cycle wide ------------------------");
        @(posedge clk); #1;
        chk(ready === 1'b0, "ready dropped again");

        $display("-- 4. no response while idle -------------------------");
        // The data wire must stay quiet throughout - the default slave has
        // nothing to send and must not corrupt anyone else's transfer.
        for (i = 0; i < 6; i = i + 1) begin
            @(posedge clk); #1;
            if (ready !== 1'b0) begin
                $display("  ERROR ready asserted with no sel");
                errors = errors + 1;
            end
            if (dstream_out !== 1'b0) begin
                $display("  ERROR default slave drove the data wire");
                errors = errors + 1;
            end
        end
        $display("  ok    quiet for 6 idle cycles");

        $display("-- 5. back-to-back unmapped accesses -----------------");
        @(posedge clk); sel <= 1'b1;
        @(posedge clk); #1;
        chk(ready === 1'b1 && resp === `RESP_ERROR, "first access answered");
        @(posedge clk); sel <= 1'b0; #1;
        chk(ready === 1'b1 && resp === `RESP_ERROR, "second access answered");
        @(posedge clk); #1;
        chk(ready === 1'b0, "quiet afterwards");

        $display("======================================================");
        if (errors == 0) $display(" tb_default_slave: PASSED (0 errors)");
        else             $display(" tb_default_slave: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
