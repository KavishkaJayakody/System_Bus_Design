//==========================================================================
// tb_slave_mem.v -- self-checking testbench for slave_mem
//
// Two DUTs are instantiated from the same module: a plain 2K slave and a 4K
// split-capable slave, so the SPLIT_CAPABLE=0 and =1 builds are both covered.
//
// Covers:
//   1. reset behaviour          - ready low, no spurious response
//   2. write then read back     - data integrity, ready exactly one cycle
//                                 after sel, resp = OKAY
//   3. address independence     - first word, last word and a few interior
//                                 words all hold distinct values (catches the
//                                 classic "upper address bits ignored, so
//                                 offsets alias" bug)
//   4. no response when idle    - ready never rises without sel
//   5. split: SPLIT response, the deferred transfer is NOT performed,
//      split_complete pulses on the right master's bit exactly once, the
//      replay is served with OKAY, and the data is correct
//   6. split of a WRITE          - the write must not take effect until the
//                                 replay
//   7. only one split outstanding - an access from the other master while the
//                                 slave is busy is served normally
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_slave_mem;

    localparam DATA_W    = `BUS_DATA_W;
    localparam RESP_W    = `BUS_RESP_W;
    localparam N         = `BUS_N_MASTERS;
    localparam ID_W      = 1;
    localparam SPL_LAT   = 4;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    integer errors = 0;

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t)", name, $time);
                errors = errors + 1;
            end
        end
    endtask

    //======================================================================
    // DUT A: plain slave, 2K words (slave 2 geometry)
    //======================================================================
    reg                a_sel, a_we;
    reg  [10:0]        a_addr;
    reg  [DATA_W-1:0]  a_wdata;
    wire [DATA_W-1:0]  a_rdata;
    wire               a_ready;
    wire [RESP_W-1:0]  a_resp;
    wire [N-1:0]       a_sc;
    wire               a_busy;

    slave_mem #(
        .DATA_W(DATA_W), .LADDR_W(11), .WORDS(2048), .RESP_W(RESP_W),
        .N_MASTERS(N), .ID_W(ID_W), .SPLIT_CAPABLE(0)
    ) dut_plain (
        .clk(clk), .rst_n(rst_n), .sel(a_sel), .we(a_we), .addr(a_addr),
        .wdata(a_wdata), .master_id(1'b0), .split_en(1'b0),
        .rdata(a_rdata), .ready(a_ready), .resp(a_resp),
        .split_complete(a_sc), .busy(a_busy)
    );

    //======================================================================
    // DUT B: split-capable slave, 4K words (slave 0 geometry)
    //======================================================================
    reg                b_sel, b_we, b_split_en;
    reg  [11:0]        b_addr;
    reg  [DATA_W-1:0]  b_wdata;
    reg  [ID_W-1:0]    b_mid;
    wire [DATA_W-1:0]  b_rdata;
    wire               b_ready;
    wire [RESP_W-1:0]  b_resp;
    wire [N-1:0]       b_sc;
    wire               b_busy;

    slave_mem #(
        .DATA_W(DATA_W), .LADDR_W(12), .WORDS(4096), .RESP_W(RESP_W),
        .N_MASTERS(N), .ID_W(ID_W), .SPLIT_CAPABLE(1), .SPLIT_LATENCY(SPL_LAT)
    ) dut_split (
        .clk(clk), .rst_n(rst_n), .sel(b_sel), .we(b_we), .addr(b_addr),
        .wdata(b_wdata), .master_id(b_mid), .split_en(b_split_en),
        .rdata(b_rdata), .ready(b_ready), .resp(b_resp),
        .split_complete(b_sc), .busy(b_busy)
    );

    // Captured results of the last access on each DUT.
    reg                cap_ready;
    reg  [RESP_W-1:0]  cap_resp;
    reg  [DATA_W-1:0]  cap_rdata;

    // Count split_complete pulses on the split slave.
    integer sc_pulses = 0;
    integer sc_last_id = -1;
    always @(posedge clk) begin
        if (rst_n && |b_sc) begin
            sc_pulses  = sc_pulses + 1;
            sc_last_id = b_sc[0] ? 0 : 1;
        end
    end

    //----------------------------------------------------------------------
    // One access to the plain slave.  Drives sel for exactly one cycle and
    // captures the response that arrives on the following cycle.
    //----------------------------------------------------------------------
    task acc_a;
        input             we_i;
        input [10:0]      addr_i;
        input [DATA_W-1:0] wd;
        begin
            @(posedge clk);
            a_sel <= 1'b1; a_we <= we_i; a_addr <= addr_i; a_wdata <= wd;
            @(posedge clk);
            a_sel <= 1'b0; a_we <= 1'b0;
            #1;
            cap_ready = a_ready; cap_resp = a_resp; cap_rdata = a_rdata;
        end
    endtask

    task acc_b;
        input             we_i;
        input [11:0]      addr_i;
        input [DATA_W-1:0] wd;
        input [ID_W-1:0]  id;
        begin
            @(posedge clk);
            b_sel <= 1'b1; b_we <= we_i; b_addr <= addr_i; b_wdata <= wd;
            b_mid <= id;
            @(posedge clk);
            b_sel <= 1'b0; b_we <= 1'b0;
            #1;
            cap_ready = b_ready; cap_resp = b_resp; cap_rdata = b_rdata;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_slave_mem");
        $display("======================================================");

        rst_n = 1'b0;
        a_sel = 0; a_we = 0; a_addr = 0; a_wdata = 0;
        b_sel = 0; b_we = 0; b_addr = 0; b_wdata = 0; b_mid = 0;
        b_split_en = 1'b0;

        //------------------------------------------------------------------
        $display("-- 1. reset behaviour --------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(a_ready === 1'b0, "plain slave: ready low in reset");
        chk(b_ready === 1'b0, "split slave: ready low in reset");
        chk(b_busy  === 1'b0, "split slave: not busy in reset");
        chk(b_sc    === {N{1'b0}}, "split slave: split_complete clear in reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(a_ready === 1'b0, "plain slave: no ready without sel");
        chk(b_ready === 1'b0, "split slave: no ready without sel");

        //------------------------------------------------------------------
        $display("-- 2. plain slave write / read back ------------------");
        acc_a(1'b1, 11'h123, 32'hDEAD_BEEF);
        chk(cap_ready === 1'b1,        "write completes one cycle after sel");
        chk(cap_resp  === `RESP_OKAY,  "write resp = OKAY");
        chk(a_ready   === 1'b1,        "ready is high now");
        @(posedge clk); #1;
        chk(a_ready === 1'b0,          "ready is only one cycle wide");

        acc_a(1'b0, 11'h123, 32'h0);
        chk(cap_ready === 1'b1,               "read completes one cycle after sel");
        chk(cap_resp  === `RESP_OKAY,         "read resp = OKAY");
        chk(cap_rdata === 32'hDEAD_BEEF,      "read data matches what was written");

        //------------------------------------------------------------------
        $display("-- 3. every word is its own location -----------------");
        acc_a(1'b1, 11'h000, 32'h1111_0000);
        acc_a(1'b1, 11'h001, 32'h1111_0001);
        acc_a(1'b1, 11'h040, 32'h1111_0040);   // +0x40: the classic alias
        acc_a(1'b1, 11'h7FF, 32'h1111_07FF);   // last word of the 2K slave
        acc_a(1'b0, 11'h000, 32'h0);
        chk(cap_rdata === 32'h1111_0000, "word 0x000 intact");
        acc_a(1'b0, 11'h001, 32'h0);
        chk(cap_rdata === 32'h1111_0001, "word 0x001 intact");
        acc_a(1'b0, 11'h040, 32'h0);
        chk(cap_rdata === 32'h1111_0040, "word 0x040 intact (no 0x40 aliasing)");
        acc_a(1'b0, 11'h7FF, 32'h0);
        chk(cap_rdata === 32'h1111_07FF, "last word 0x7FF intact");
        acc_a(1'b0, 11'h123, 32'h0);
        chk(cap_rdata === 32'hDEAD_BEEF, "earlier word 0x123 undisturbed");

        //------------------------------------------------------------------
        $display("-- 4. no response while idle -------------------------");
        repeat (5) begin
            @(posedge clk); #1;
            if (a_ready !== 1'b0) begin
                $display("  ERROR plain slave asserted ready with no sel");
                errors = errors + 1;
            end
        end
        $display("  ok    plain slave stays quiet for 5 idle cycles");

        //------------------------------------------------------------------
        $display("-- 5. split read -------------------------------------");
        // Seed the location with split disabled.
        b_split_en = 1'b0;
        acc_b(1'b1, 12'h010, 32'hCAFE_0010, 1'b0);
        chk(cap_resp === `RESP_OKAY, "seed write served normally (split_en=0)");

        sc_pulses = 0;
        b_split_en = 1'b1;
        acc_b(1'b0, 12'h010, 32'h0, 1'b0);      // master 0 reads -> split
        chk(cap_ready === 1'b1,       "split still completes the bus cycle");
        chk(cap_resp  === `RESP_SPLIT,"fresh access answered with SPLIT");
        chk(b_busy    === 1'b1,       "slave is busy with the deferred transfer");

        // Wait for the wake-up pulse.
        wait (sc_pulses == 1);
        #1;
        chk(sc_last_id == 0,      "split_complete pulsed on master 0's bit");
        repeat (4) @(posedge clk); #1;
        chk(sc_pulses == 1,       "split_complete is exactly one cycle wide");
        chk(b_busy === 1'b0,      "slave no longer busy");

        // The replay.
        acc_b(1'b0, 12'h010, 32'h0, 1'b0);
        chk(cap_resp  === `RESP_OKAY,      "replay served with OKAY, not split again");
        chk(cap_rdata === 32'hCAFE_0010,   "replay returns the correct data");

        //------------------------------------------------------------------
        $display("-- 6. split of a write -------------------------------");
        sc_pulses = 0;
        acc_b(1'b1, 12'h010, 32'h9999_9999, 1'b0);   // fresh -> SPLIT
        chk(cap_resp === `RESP_SPLIT, "fresh write answered with SPLIT");
        wait (sc_pulses == 1); #1;
        b_split_en = 1'b0;
        acc_b(1'b0, 12'h010, 32'h0, 1'b0);           // read it back
        chk(cap_rdata === 32'hCAFE_0010,
            "deferred write did NOT take effect during the split");
        b_split_en = 1'b1;
        // now genuinely replay the write (resume was consumed by the read
        // above, so this one splits again - drive it through)
        sc_pulses = 0;
        acc_b(1'b1, 12'h010, 32'h9999_9999, 1'b0);
        chk(cap_resp === `RESP_SPLIT, "write splits again after a fresh start");
        wait (sc_pulses == 1); #1;
        acc_b(1'b1, 12'h010, 32'h9999_9999, 1'b0);   // replay
        chk(cap_resp === `RESP_OKAY,  "write replay served with OKAY");
        b_split_en = 1'b0;
        acc_b(1'b0, 12'h010, 32'h0, 1'b0);
        chk(cap_rdata === 32'h9999_9999, "replayed write took effect");

        //------------------------------------------------------------------
        $display("-- 7. only one split outstanding ---------------------");
        b_split_en = 1'b1;
        sc_pulses  = 0;
        acc_b(1'b0, 12'h020, 32'h0, 1'b0);           // master 0 -> SPLIT
        chk(cap_resp === `RESP_SPLIT, "master 0 access splits");
        chk(b_busy === 1'b1,          "slave busy");
        acc_b(1'b0, 12'h010, 32'h0, 1'b1);           // master 1 while busy
        chk(cap_resp  === `RESP_OKAY,
            "master 1 served normally while a split is in flight");
        chk(cap_rdata === 32'h9999_9999, "master 1 got real data");
        wait (sc_pulses == 1); #1;
        chk(sc_last_id == 0, "the wake-up still belongs to master 0");
        b_split_en = 1'b0;

        $display("======================================================");
        if (errors == 0) $display(" tb_slave_mem: PASSED (0 errors)");
        else             $display(" tb_slave_mem: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #200000;
        $display(" tb_slave_mem: FAILED (timeout)");
        $finish;
    end

endmodule
