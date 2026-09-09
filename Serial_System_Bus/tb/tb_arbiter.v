//==========================================================================
// tb_arbiter.v -- self-checking testbench for arbiter
//
// The arbiter is exercised with a behavioural stand-in for the rest of the
// bus: the testbench watches `gnt', then pulses bus_ready with a chosen
// response to end the transfer, exactly as bus_mux would.
//
// Covers, in the order the brief lists them:
//   1. reset behaviour   - no grant asserted out of reset, mask clear
//   2. single requester  - grant asserted, HELD until the transfer completes
//   3. both requesters   - master 0 (higher priority) wins, master 1 waits,
//                          and master 1 is granted once master 0 is done
//   4. split scenario    - a SPLIT response masks the granted master, that
//                          master is excluded from arbitration even though
//                          it keeps req high, the other master runs meanwhile,
//                          and split_complete restores it
//   5. no false grant    - grant never goes to a masked master
//   6. ERROR response    - behaves like OKAY as far as arbitration goes
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_arbiter;

    localparam N       = `BUS_N_MASTERS;
    localparam ID_W    = `BUS_ID_W;
    localparam RESP_W  = `BUS_RESP_W;

    reg               clk = 1'b0;
    reg               rst_n;
    reg  [N-1:0]      req;
    reg               bus_ready;
    reg  [RESP_W-1:0] bus_resp;
    reg  [N-1:0]      split_complete;

    wire [N-1:0]      gnt;
    wire              gnt_valid;
    wire [ID_W-1:0]   master_id;
    wire [N-1:0]      split_mask;
    wire              locked;

    integer errors = 0;
    integer i;

    always #10 clk = ~clk;      // 50 MHz

    arbiter #(.N_MASTERS(N), .ID_W(ID_W), .RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n), .req(req),
        .bus_ready(bus_ready), .bus_resp(bus_resp),
        .split_complete(split_complete),
        .gnt(gnt), .gnt_valid(gnt_valid), .master_id(master_id),
        .split_mask(split_mask), .locked(locked)
    );

    //----------------------------------------------------------------------
    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t gnt=%b mask=%b id=%0d lock=%b)",
                         name, $time, gnt, split_mask, master_id, locked);
                errors = errors + 1;
            end
        end
    endtask

    // End the transfer currently on the bus with response `r', modelling
    // what a real master does with its request line at that moment:
    //   OKAY / ERROR - the transaction is over, so req drops
    //   SPLIT        - req stays HIGH; the arbiter's mask is what defers it
    task finish_xfer;
        input [RESP_W-1:0] r;
        begin
            @(posedge clk);
            bus_ready <= 1'b1;
            bus_resp  <= r;
            if (r !== `RESP_SPLIT)
                req[master_id] <= 1'b0;
            @(posedge clk);
            bus_ready <= 1'b0;
            bus_resp  <= `RESP_OKAY;
        end
    endtask

    // Pulse split_complete for master `m' for one cycle.
    task wake;
        input integer m;
        begin
            @(posedge clk);
            split_complete <= (1 << m);
            @(posedge clk);
            split_complete <= {N{1'b0}};
        end
    endtask

    // Wait up to `n' cycles for gnt == expected one-hot; 0 means "no grant".
    task wait_gnt;
        input [N-1:0]     expect_oh;
        input integer     n;
        input [200*8-1:0] name;
        integer           k;
        begin
            k = 0;
            while ((gnt !== expect_oh) && (k < n)) begin
                @(posedge clk); #1;
                k = k + 1;
            end
            chk(gnt === expect_oh, name);
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_arbiter");
        $display("======================================================");

        rst_n          = 1'b0;
        req            = {N{1'b0}};
        bus_ready      = 1'b0;
        bus_resp       = `RESP_OKAY;
        split_complete = {N{1'b0}};

        //------------------------------------------------------------------
        $display("-- 1. reset behaviour --------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(gnt        === {N{1'b0}}, "no grant asserted while in reset");
        chk(split_mask === {N{1'b0}}, "split mask clear in reset");
        chk(gnt_valid  === 1'b0,      "gnt_valid low in reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(gnt === {N{1'b0}}, "no grant asserted after reset release with no req");

        //------------------------------------------------------------------
        $display("-- 2. single requester -------------------------------");
        @(posedge clk); req <= 2'b01;               // only master 0
        wait_gnt(2'b01, 5, "master 0 granted");
        chk(master_id === 1'b0, "master_id tag = 0");
        chk(locked    === 1'b1, "bus locked for the transfer");
        // grant must be HELD while the transfer is in progress
        repeat (4) begin @(posedge clk); #1; end
        chk(gnt === 2'b01, "grant held across the whole transfer");
        finish_xfer(`RESP_OKAY);
        @(posedge clk); req <= 2'b00;
        repeat (2) @(posedge clk); #1;
        chk(gnt === 2'b00, "grant released after completion");
        chk(locked === 1'b0, "lock released after completion");

        //------------------------------------------------------------------
        $display("-- 3. both requesters, fixed priority ----------------");
        @(posedge clk); req <= 2'b11;               // both at the same instant
        wait_gnt(2'b01, 5, "higher-priority master 0 wins");
        repeat (3) begin @(posedge clk); #1; end
        chk(gnt === 2'b01, "master 1 kept waiting while master 0 owns the bus");
        finish_xfer(`RESP_OKAY);
        @(posedge clk); req <= 2'b10;               // master 0 done, master 1 left
        wait_gnt(2'b10, 5, "master 1 granted once master 0 drops");
        chk(master_id === 1'b1, "master_id tag = 1");
        finish_xfer(`RESP_OKAY);
        @(posedge clk); req <= 2'b00;
        repeat (2) @(posedge clk); #1;

        //------------------------------------------------------------------
        $display("-- 4. split scenario ---------------------------------");
        @(posedge clk); req <= 2'b01;               // master 0 only, for now
        wait_gnt(2'b01, 5, "master 0 granted for the transfer that will split");
        finish_xfer(`RESP_SPLIT);
        #1;
        chk(split_mask === 2'b01, "mask[0] set by the SPLIT response");
        chk(gnt        === 2'b00, "grant dropped on SPLIT");

        // master 0 keeps requesting - masking, not withdrawal, defers it
        @(posedge clk); req <= 2'b11;               // master 1 now wants the bus
        wait_gnt(2'b10, 6, "master 1 granted while master 0 is masked");
        chk(split_mask === 2'b01, "master 0 still masked");
        // let master 1 run a whole transfer normally
        finish_xfer(`RESP_OKAY);
        repeat (2) begin @(posedge clk); #1; end
        chk(gnt !== 2'b01, "masked master 0 never granted, though req is high");

        // master 1 goes away; master 0 must STILL not be granted
        @(posedge clk); req <= 2'b01;
        repeat (4) begin @(posedge clk); #1; end
        chk(gnt === 2'b00, "no grant at all while the only requester is masked");

        // slave finishes the deferred transfer
        wake(0);
        #1;
        chk(split_mask === 2'b00, "mask[0] cleared by split_complete[0]");
        wait_gnt(2'b01, 6, "master 0 re-granted to replay the transfer");
        finish_xfer(`RESP_OKAY);
        @(posedge clk); req <= 2'b00;
        repeat (2) @(posedge clk); #1;
        chk(split_mask === 2'b00, "mask stays clear after the replay");

        //------------------------------------------------------------------
        $display("-- 5. lower-priority master can also split -----------");
        @(posedge clk); req <= 2'b10;
        wait_gnt(2'b10, 5, "master 1 granted");
        finish_xfer(`RESP_SPLIT);
        #1;
        chk(split_mask === 2'b10, "mask[1] set, mask[0] untouched");
        @(posedge clk); req <= 2'b11;
        wait_gnt(2'b01, 6, "master 0 runs while master 1 is masked");
        finish_xfer(`RESP_OKAY);
        wake(1);
        #1;
        chk(split_mask === 2'b00, "mask[1] cleared");
        wait_gnt(2'b10, 6, "master 1 re-granted to replay the transfer");
        finish_xfer(`RESP_OKAY);
        repeat (2) @(posedge clk); #1;
        chk(gnt === 2'b00, "bus idle again");

        //------------------------------------------------------------------
        $display("-- 6. ERROR response ends the transfer ---------------");
        @(posedge clk); req <= 2'b01;
        wait_gnt(2'b01, 5, "master 0 granted");
        finish_xfer(`RESP_ERROR);
        @(posedge clk); req <= 2'b00;
        repeat (2) @(posedge clk); #1;
        chk(split_mask === 2'b00, "ERROR does not set the split mask");
        chk(gnt        === 2'b00, "grant released after ERROR");

        $display("======================================================");
        if (errors == 0) $display(" tb_arbiter: PASSED (0 errors)");
        else             $display(" tb_arbiter: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    // Safety net: never let a hung arbiter hang the regression.
    initial begin
        #200000;
        $display(" tb_arbiter: FAILED (timeout)");
        $finish;
    end

endmodule
