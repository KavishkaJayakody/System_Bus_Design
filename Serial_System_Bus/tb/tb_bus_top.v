//==========================================================================
// tb_bus_top.v -- self-checking integration testbench for bus_top
//
// Drives the two master command interfaces directly, exactly as de2_top's
// scenario sequencer does on the board.
//
// Covers, in the order the brief lists them:
//   1. reset            - no grant, no request, clean mask, bus idle
//   2. one master       - write/read to every slave, data integrity,
//                         boundary addresses of all three ranges
//   3. two masters      - both request on the SAME clock; master 0 wins,
//                         master 1 still completes, neither corrupts the
//                         other's data
//   4. split, end to end- master 0 reads the split slave and is deferred,
//                         master 1 runs REAL transfers in the gap, master 0's
//                         re-issued transfer returns the right data, and the
//                         arbiter mask went up and came back down
//   5. unmapped access  - the decode hole and the reserved remote window both
//                         answer ERROR and the bus RECOVERS: the very next
//                         transfer succeeds normally
//   6. split by master 1- the low-priority master can split too (the legacy
//                         design could not - it hung)
//
// Every wait has a cycle budget.  A hang is reported as a FAILURE instead of
// running until the simulator gives up, which is the whole point of test 5.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_bus_top;

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;
    localparam ID_W   = 1;
    localparam SPLAT  = 6;           // slave 0 split latency, in cycles
    localparam TMO    = 200;         // per-transaction cycle budget

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;           // 50 MHz

    reg  [NM-1:0]           cmd_valid, cmd_we;
    reg  [NM*ADDR_W-1:0]    cmd_addr_flat;
    reg  [NM*DATA_W-1:0]    cmd_wdata_flat;
    wire [NM-1:0]           cmd_accept, done, err, mst_busy;
    wire [NM*DATA_W-1:0]    rdata_flat;
    wire [NM*RESP_W-1:0]    resp_flat;
    wire [NM*8-1:0]         split_count_flat;
    reg                     s0_split_en;

    wire [NM-1:0]           gnt;
    wire                    gnt_valid;
    wire [ID_W-1:0]         master_id;
    wire [NM-1:0]           split_mask;
    wire [NS:0]             sel_q;
    wire                    bus_valid, bus_ready, s0_busy;
    wire [ADDR_W-1:0]       bus_addr;
    wire [RESP_W-1:0]       bus_resp;
    wire [DATA_W-1:0]       bus_rdata;

    integer errors = 0;

    bus_top #(
        .N_MASTERS(NM), .ID_W(ID_W), .N_SLAVES(NS),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPLAT)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr_flat(cmd_addr_flat), .cmd_wdata_flat(cmd_wdata_flat),
        .cmd_accept(cmd_accept), .done(done),
        .rdata_flat(rdata_flat), .resp_flat(resp_flat), .err(err),
        .split_count_flat(split_count_flat), .mst_busy(mst_busy),
        .s0_split_en(s0_split_en),
        .gnt(gnt), .gnt_valid(gnt_valid), .master_id(master_id),
        .split_mask(split_mask), .sel_q(sel_q),
        .bus_valid(bus_valid), .bus_addr(bus_addr),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .bus_rdata(bus_rdata),
        .s0_busy(s0_busy)
    );

    //----------------------------------------------------------------------
    // Helpers
    //----------------------------------------------------------------------
    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t gnt=%b mask=%b sel_q=%b)",
                         name, $time, gnt, split_mask, sel_q);
                errors = errors + 1;
            end
        end
    endtask

    function [DATA_W-1:0] mrd;               // last read data of master i
        input integer i;
        begin mrd = rdata_flat[i*DATA_W +: DATA_W]; end
    endfunction

    function [RESP_W-1:0] mresp;             // last response of master i
        input integer i;
        begin mresp = resp_flat[i*RESP_W +: RESP_W]; end
    endfunction

    function [7:0] msplits;                  // splits seen by master i
        input integer i;
        begin msplits = split_count_flat[i*8 +: 8]; end
    endfunction

    // Latency of the last transaction of each master, in clocks.
    integer lat [0:NM-1];
    integer timed_out [0:NM-1];

    //----------------------------------------------------------------------
    // Issue one command on master i and wait for it to finish.  Aborts with
    // an error after TMO cycles rather than hanging the regression - that is
    // how a wedged bus is detected.
    //----------------------------------------------------------------------
    // `automatic': m_run is called from two branches of a fork/join at the
    // same time.  A plain Verilog task has STATIC storage, so the two calls
    // would share `n', `a', `d' and corrupt each other.
    task automatic m_run;
        input integer            i;
        input                    we_i;
        input [ADDR_W-1:0]       a;
        input [DATA_W-1:0]       d;
        integer                  n;
        begin
            timed_out[i] = 0;
            @(posedge clk);
            cmd_valid[i]                 <= 1'b1;
            cmd_we[i]                    <= we_i;
            cmd_addr_flat [i*ADDR_W +: ADDR_W] <= a;
            cmd_wdata_flat[i*DATA_W +: DATA_W] <= d;

            n = 0;
            @(posedge clk);
            while (!cmd_accept[i] && n < TMO) begin @(posedge clk); n = n + 1; end
            cmd_valid[i] <= 1'b0;

            while (!done[i] && n < TMO) begin @(posedge clk); n = n + 1; end
            lat[i] = n;
            if (n >= TMO) begin
                timed_out[i] = 1;
                $display("  ERROR master %0d: transaction to 0x%04h TIMED OUT after %0d clocks - the bus is wedged",
                         i, a, n);
                errors = errors + 1;
            end
            #1;
        end
    endtask

    // Write then read back and compare, on one master.
    task automatic m_check_wr;
        input integer      i;
        input [ADDR_W-1:0] a;
        input [DATA_W-1:0] d;
        input [200*8-1:0]  name;
        begin
            m_run(i, 1'b1, a, d);
            m_run(i, 1'b0, a, {DATA_W{1'b0}});
            if (mrd(i) === d && mresp(i) === `RESP_OKAY)
                $display("  ok    %0s: [0x%04h] = 0x%08h", name, a, d);
            else begin
                $display("  ERROR %0s: [0x%04h] wrote 0x%08h read 0x%08h resp=%b",
                         name, a, d, mrd(i), mresp(i));
                errors = errors + 1;
            end
        end
    endtask

    integer i;

    initial begin
        $display("======================================================");
        $display(" tb_bus_top -- 2 masters, 3 slaves + default slave");
        $display("======================================================");

        rst_n          = 1'b0;
        cmd_valid      = {NM{1'b0}};
        cmd_we         = {NM{1'b0}};
        cmd_addr_flat  = {NM*ADDR_W{1'b0}};
        cmd_wdata_flat = {NM*DATA_W{1'b0}};
        s0_split_en    = 1'b0;

        //==================================================================
        $display("-- 1. reset ------------------------------------------");
        repeat (4) @(posedge clk); #1;
        chk(gnt        === {NM{1'b0}}, "no grant during reset");
        chk(split_mask === {NM{1'b0}}, "split mask clear during reset");
        chk(sel_q      === {(NS+1){1'b0}}, "no slave selected during reset");
        chk(bus_valid  === 1'b0,       "bus idle during reset");
        chk(mst_busy   === {NM{1'b0}}, "both masters idle during reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (4) @(posedge clk); #1;
        chk(gnt       === {NM{1'b0}}, "no grant after reset with no command");
        chk(bus_ready === 1'b0,       "no spurious ready after reset");

        //==================================================================
        $display("-- 2. one master, all three slaves -------------------");
        m_check_wr(0, 16'h0000, 32'h5000_0000, "slave 0 first word");
        m_check_wr(0, 16'h0FFF, 32'h5000_0FFF, "slave 0 last word ");
        m_check_wr(0, 16'h0040, 32'h5000_0040, "slave 0 word 0x40 ");
        m_check_wr(0, 16'h1000, 32'h5100_0000, "slave 1 first word");
        m_check_wr(0, 16'h1FFF, 32'h5100_0FFF, "slave 1 last word ");
        m_check_wr(0, 16'h2000, 32'h5200_0000, "slave 2 first word");
        m_check_wr(0, 16'h27FF, 32'h5200_07FF, "slave 2 last word ");
        $display("  ..    a read of a mapped slave takes %0d clocks", lat[0]);

        // The three slaves must be genuinely independent memories.
        m_run(0, 1'b0, 16'h0000, 32'h0);
        chk(mrd(0) === 32'h5000_0000, "slave 0 word 0 survived the other writes");
        m_run(0, 1'b0, 16'h1000, 32'h0);
        chk(mrd(0) === 32'h5100_0000, "slave 1 word 0 is a different location");
        m_run(0, 1'b0, 16'h2000, 32'h0);
        chk(mrd(0) === 32'h5200_0000, "slave 2 word 0 is a different location");
        chk(split_mask === 2'b00, "no split happened with split_en=0");

        //==================================================================
        $display("-- 3. two masters requesting on the same clock -------");
        // Seed distinct values, then have both masters read at the same time.
        m_run(0, 1'b1, 16'h1100, 32'hAAAA_0001);
        m_run(1, 1'b1, 16'h2100, 32'hBBBB_0002);

        fork
            m_run(0, 1'b0, 16'h1100, 32'h0);
            m_run(1, 1'b0, 16'h2100, 32'h0);
        join
        chk(mrd(0) === 32'hAAAA_0001, "master 0 got ITS data");
        chk(mrd(1) === 32'hBBBB_0002, "master 1 got ITS data");
        chk(!timed_out[0] && !timed_out[1], "both masters completed");
        $display("  ..    contended latencies: m0 %0d clocks, m1 %0d clocks",
                 lat[0], lat[1]);

        // Concurrent writes to the two different slaves must not cross over.
        fork
            m_run(0, 1'b1, 16'h1200, 32'h1111_1111);
            m_run(1, 1'b1, 16'h2200, 32'h2222_2222);
        join
        m_run(0, 1'b0, 16'h1200, 32'h0);
        chk(mrd(0) === 32'h1111_1111, "master 0's concurrent write landed");
        m_run(1, 1'b0, 16'h2200, 32'h0);
        chk(mrd(1) === 32'h2222_2222, "master 1's concurrent write landed");

        //==================================================================
        $display("-- 4. full split transaction, end to end -------------");
        // Seed the split slave while it is not splitting.
        s0_split_en = 1'b0;
        m_run(0, 1'b1, 16'h0010, 32'hC0FF_EE00);
        m_run(1, 1'b1, 16'h2300, 32'h1234_ABCD);

        s0_split_en = 1'b1;
        fork
            // master 0 reads the split-capable slave: this WILL be deferred
            m_run(0, 1'b0, 16'h0010, 32'h0);
            // master 1 does real work in the gap the split opens up
            begin
                m_run(1, 1'b0, 16'h2300, 32'h0);
                m_run(1, 1'b1, 16'h2301, 32'hFEED_0001);
                m_run(1, 1'b0, 16'h2301, 32'h0);
            end
            // watch the mask go up and come back down
            begin : watch
                integer k;
                reg saw_mask, saw_m1_gnt_while_masked;
                saw_mask = 0; saw_m1_gnt_while_masked = 0;
                for (k = 0; k < 300; k = k + 1) begin
                    @(posedge clk); #1;
                    if (split_mask[0]) begin
                        saw_mask = 1;
                        if (gnt[1]) saw_m1_gnt_while_masked = 1;
                    end
                end
                chk(saw_mask, "arbiter masked master 0 during the split");
                chk(saw_m1_gnt_while_masked,
                    "master 1 was granted the bus while master 0 was deferred");
            end
        join

        chk(!timed_out[0],              "master 0's split transaction completed");
        chk(msplits(0) >= 8'd1,         "master 0 recorded at least one SPLIT");
        chk(mrd(0) === 32'hC0FF_EE00,   "the RE-ISSUED transfer returned the right data");
        chk(mresp(0) === `RESP_OKAY,    "master 0's final response is OKAY, never SPLIT");
        chk(mrd(1) === 32'hFEED_0001,   "master 1's work in the gap was correct");
        chk(split_mask === 2'b00,       "mask cleared again afterwards");
        $display("  ..    split read cost master 0 %0d clocks", lat[0]);

        // A write can be split too, and must still take effect exactly once.
        m_run(0, 1'b1, 16'h0011, 32'h7777_7777);
        chk(msplits(0) >= 8'd2, "the write was split as well");
        s0_split_en = 1'b0;
        m_run(0, 1'b0, 16'h0011, 32'h0);
        chk(mrd(0) === 32'h7777_7777, "the split write landed exactly once");

        //==================================================================
        $display("-- 5. unmapped access: ERROR, and the bus RECOVERS ---");
        // The decode hole above slave 2.
        m_run(0, 1'b0, 16'h2800, 32'h0);
        chk(!timed_out[0],           "0x2800 completed instead of hanging the bus");
        chk(mresp(0) === `RESP_ERROR,"0x2800 answered ERROR");
        chk(err[0]   === 1'b1,       "master 0 reports err");
        chk(gnt      === 2'b00,      "the grant was released, bus not held");

        // The very next transfer must work - that is what "recovers" means.
        m_run(0, 1'b0, 16'h1100, 32'h0);
        chk(!timed_out[0],            "the next transfer completed normally");
        chk(mrd(0) === 32'hAAAA_0001, "and returned correct data");
        chk(mresp(0) === `RESP_OKAY,  "and a clean OKAY");

        m_run(0, 1'b1, 16'h2FFF, 32'hDEAD_DEAD);
        chk(mresp(0) === `RESP_ERROR, "an unmapped WRITE also answers ERROR");

        // The window reserved for the phase-2 remote bridge.
        m_run(1, 1'b0, 16'h8000, 32'h0);
        chk(!timed_out[1],            "reserved addr[15]=1 window completed");
        chk(mresp(1) === `RESP_ERROR, "reserved window answers ERROR");
        m_run(1, 1'b0, 16'hFFFF, 32'h0);
        chk(mresp(1) === `RESP_ERROR, "top of memory answers ERROR");
        m_run(1, 1'b0, 16'h2300, 32'h0);
        chk(mrd(1) === 32'h1234_ABCD, "master 1 recovered too");

        // A run of bad addresses back to back must not wedge anything.
        for (i = 0; i < 4; i = i + 1) m_run(0, 1'b0, 16'h2900 + i[15:0], 32'h0);
        chk(!timed_out[0], "four consecutive unmapped accesses all completed");
        m_run(0, 1'b0, 16'h0000, 32'h0);
        chk(mrd(0) === 32'h5000_0000, "bus fully healthy afterwards");

        //==================================================================
        $display("-- 6. the LOW-priority master can split too ----------");
        s0_split_en = 1'b0;
        m_run(1, 1'b1, 16'h0100, 32'h5A5A_5A5A);
        s0_split_en = 1'b1;
        m_run(1, 1'b0, 16'h0100, 32'h0);
        chk(!timed_out[1],             "master 1's split read completed (did not hang)");
        chk(msplits(1) >= 8'd1,        "master 1 recorded a SPLIT");
        chk(mrd(1) === 32'h5A5A_5A5A,  "master 1's re-issued transfer got the data");
        chk(mresp(1) === `RESP_OKAY,   "master 1's final response is OKAY");
        s0_split_en = 1'b0;

        //==================================================================
        $display("======================================================");
        $display(" splits seen: master 0 = %0d, master 1 = %0d",
                 msplits(0), msplits(1));
        if (errors == 0) $display(" tb_bus_top: PASSED (0 errors)");
        else             $display(" tb_bus_top: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #2000000;
        $display(" tb_bus_top: FAILED (global timeout - the bus hung)");
        $finish;
    end

endmodule
