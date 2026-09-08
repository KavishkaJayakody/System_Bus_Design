//==========================================================================
// tb_integration.v -- self-checking integration testbench
//
// Drives the two master command interfaces directly, exactly as de2_top's
// scenario sequencer does on the board.
//
// The design has no integration wrapper - de2_top instantiates `master',
// `system_bus' and `slave' side by side - so this testbench builds the same
// composition itself.  That duplication is the price of having no wrapper:
// keep this wiring and de2_top's in step.
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
//   6. split by master 1- the low-priority master can split too
//   7. serial framing   - the address frame is exactly ADDR_W clocks long,
//                         every time, including on a re-issued transfer
//
// Every wait has a cycle budget.  A hang is reported as a FAILURE instead of
// running until the simulator gives up, which is the whole point of test 5.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_integration;

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;
    localparam ID_W   = 1;
    localparam SPLAT  = 6;           // slave 0 split latency, in cycles
    // A serial transaction is ~21 clocks for a write and ~29 for a read,
    // so the per-transaction budget has to be far larger than it was on
    // the parallel bus.  It is still a budget: a hang is a FAILURE.
    localparam TMO    = 400;
    localparam SPLIT_LATENCY = SPLAT;   // the composition's name for it

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
    wire                    bus_astream, bus_dstream, addr_done;
    wire [ADDR_W-1:0]       bus_addr;
    wire [RESP_W-1:0]       bus_resp;

    integer errors = 0;

    //======================================================================
    // THE SYSTEM UNDER TEST: masters + bus + slaves, wired directly.
    //
    // There is no integration wrapper in the design, so this testbench
    // builds the composition itself.  It is the SAME wiring de2_top has; if
    // one changes, the other must too.
    //
    // Serial wires between the masters and the bus.
    // One bit per master per stream - these are the CANDIDATES; the bus
    // picks one with the grant.
    //======================================================================
    wire [NM-1:0]  m_req;
    wire [NM-1:0]  m_valid;
    wire [NM-1:0]  m_we;
    wire [NM-1:0]  m_astream;
    wire [NM-1:0]  m_dstream;

    //======================================================================
    // Serial wires between the bus and the slaves.
    //======================================================================
    wire                            bus_we;
    wire [NS-1:0]             s_sel;
    wire [NS-1:0]             s_ready;
    wire [NS*RESP_W-1:0]      s_resp_flat;
    wire [NS-1:0]             s_dstream;

    //======================================================================
    // 1. MASTERS
    //======================================================================
    genvar gi;
    generate
    for (gi = 0; gi < NM; gi = gi + 1) begin : g_master
        master #(
            .ADDR_W (ADDR_W),
            .DATA_W (DATA_W),
            .RESP_W (RESP_W)
        ) u_master (
            .clk         (clk),
            .rst_n       (rst_n),
            // parallel, facing the command source
            .cmd_valid   (cmd_valid[gi]),
            .cmd_we      (cmd_we[gi]),
            .cmd_addr    (cmd_addr_flat  [gi*ADDR_W +: ADDR_W]),
            .cmd_wdata   (cmd_wdata_flat [gi*DATA_W +: DATA_W]),
            .cmd_accept  (cmd_accept[gi]),
            .done        (done[gi]),
            .rdata       (rdata_flat     [gi*DATA_W +: DATA_W]),
            .resp        (resp_flat      [gi*RESP_W +: RESP_W]),
            .err         (err[gi]),
            .split_count (split_count_flat[gi*8 +: 8]),
            .busy        (mst_busy[gi]),
            .state       (),                     // waveform only
            // serial, facing the bus
            .bus_req     (m_req[gi]),
            .bus_gnt     (gnt[gi]),
            .m_valid     (m_valid[gi]),
            .m_we        (m_we[gi]),
            .m_astream   (m_astream[gi]),        // ADDRESS, one wire
            .m_dstream   (m_dstream[gi]),        // WRITE DATA, one wire
            .bus_ready   (bus_ready),
            .bus_resp    (bus_resp),
            .bus_dstream (bus_dstream)           // READ DATA, the shared wire
        );
    end
    endgenerate

    //======================================================================
    // 2. THE BUS
    //======================================================================
    wire [NM-1:0] s0_split_complete;

    // Wake-up pulses from every split-capable slave, OR-ed per master.  Only
    // slave 0 can raise one today; a second split-capable slave joins here.
    wire [NM-1:0] s_split_complete = s0_split_complete;

    system_bus #(
        .N_MASTERS (NM),
        .ID_W      (ID_W),
        .N_SLAVES  (NS),
        .ADDR_W    (ADDR_W),
        .RESP_W    (RESP_W)
    ) u_system_bus (
        .clk              (clk),
        .rst_n            (rst_n),

        // master side
        .m_req            (m_req),
        .m_gnt            (gnt),
        .m_valid          (m_valid),
        .m_we             (m_we),
        .m_astream        (m_astream),
        .m_dstream        (m_dstream),
        .bus_ready        (bus_ready),
        .bus_resp         (bus_resp),

        // slave side
        .bus_valid        (bus_valid),
        .bus_we           (bus_we),
        .bus_master_id    (master_id),
        .s_sel            (s_sel),
        .s_ready          (s_ready),
        .s_resp_flat      (s_resp_flat),
        .s_dstream        (s_dstream),
        .s_split_complete (s_split_complete),

        // the two shared wires
        .bus_astream      (bus_astream),
        .bus_dstream      (bus_dstream),

        // status
        .gnt_valid        (gnt_valid),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .bus_addr         (bus_addr),
        .addr_done        (addr_done)
    );

    //======================================================================
    // 3. SLAVES
    //
    // Instantiated one by one rather than in a generate loop, because they
    // differ in size and in whether they can split - and those differences
    // are worth reading at a glance.
    //
    // All three tap the SAME bus_astream and bus_dstream.
    //======================================================================

    // Slave 0 - 4 KB at 0x0000, split capable
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S0_LADDR_W),
        .WORDS         (`S0_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (1),
        .SPLIT_LATENCY (SPLIT_LATENCY)
    ) u_slave0 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S0]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (s0_split_en),
        .dstream_out    (s_dstream[`SEL_S0]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (s0_split_complete),
        .busy           (s0_busy)
    );

    // Slave 1 - 4 KB at 0x1000
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S1_LADDR_W),
        .WORDS         (`S1_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave1 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S1]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S1]),
        .ready          (s_ready[`SEL_S1]),
        .resp           (s_resp_flat[`SEL_S1*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    // Slave 2 - 2 KB at 0x2000
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave2 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S2]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S2]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );


    //----------------------------------------------------------------------
    // Serial framing monitor.  Watches the shared wires the way a scope
    // would: every burst of bus_valid must be exactly ADDR_W clocks, and the
    // address the central deserialiser reassembles must match what was asked
    // for.  A frame that is short or long would still "work" for a while -
    // the receivers keep the last W bits either way - and then fail on some
    // unrelated address, so it is worth checking directly.
    //----------------------------------------------------------------------
    integer frame_len   = 0;
    integer frames_seen = 0;
    integer bad_frames  = 0;
    reg [ADDR_W-1:0] frame_addr = 0;

    always @(posedge clk) if (rst_n) begin
        if (bus_valid) begin
            frame_len = frame_len + 1;
        end else if (frame_len != 0) begin
            frames_seen = frames_seen + 1;
            if (frame_len != ADDR_W) begin
                $display("  ERROR frame was %0d clocks, expected %0d (t=%0t)",
                         frame_len, ADDR_W, $time);
                bad_frames = bad_frames + 1;
            end
            frame_len = 0;
        end
        if (addr_done) frame_addr = bus_addr;
    end

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
        $display(" tb_integration -- 2 masters, 3 slaves + default slave");
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
        m_check_wr(0, 16'h0000, 8'hA0, "slave 0 first word");
        m_check_wr(0, 16'h0FFF, 8'hAF, "slave 0 last word ");
        m_check_wr(0, 16'h0040, 8'hA4, "slave 0 word 0x40 ");
        m_check_wr(0, 16'h1000, 8'hB0, "slave 1 first word");
        m_check_wr(0, 16'h1FFF, 8'hBF, "slave 1 last word ");
        m_check_wr(0, 16'h2000, 8'hC0, "slave 2 first word");
        m_check_wr(0, 16'h27FF, 8'hC7, "slave 2 last word ");
        m_run(0, 1'b1, 16'h1500, 8'h01);
        $display("  ..    WRITE latency %0d clocks", lat[0]);
        m_run(0, 1'b0, 16'h1500, {DATA_W{1'b0}});
        $display("  ..    READ  latency %0d clocks", lat[0]);

        // The three slaves must be genuinely independent memories.
        m_run(0, 1'b0, 16'h0000, {DATA_W{1'b0}});
        chk(mrd(0) === 8'hA0, "slave 0 word 0 survived the other writes");
        m_run(0, 1'b0, 16'h1000, {DATA_W{1'b0}});
        chk(mrd(0) === 8'hB0, "slave 1 word 0 is a different location");
        m_run(0, 1'b0, 16'h2000, {DATA_W{1'b0}});
        chk(mrd(0) === 8'hC0, "slave 2 word 0 is a different location");
        chk(split_mask === 2'b00, "no split happened with split_en=0");

        //==================================================================
        $display("-- 3. two masters requesting on the same clock -------");
        // Seed distinct values, then have both masters read at the same time.
        m_run(0, 1'b1, 16'h1100, 8'h11);
        m_run(1, 1'b1, 16'h2100, 8'h22);

        fork
            m_run(0, 1'b0, 16'h1100, {DATA_W{1'b0}});
            m_run(1, 1'b0, 16'h2100, {DATA_W{1'b0}});
        join
        chk(mrd(0) === 8'h11, "master 0 got ITS data");
        chk(mrd(1) === 8'h22, "master 1 got ITS data");
        chk(!timed_out[0] && !timed_out[1], "both masters completed");
        $display("  ..    contended latencies: m0 %0d clocks, m1 %0d clocks",
                 lat[0], lat[1]);

        // Concurrent writes to the two different slaves must not cross over.
        fork
            m_run(0, 1'b1, 16'h1200, 8'h33);
            m_run(1, 1'b1, 16'h2200, 8'h44);
        join
        m_run(0, 1'b0, 16'h1200, {DATA_W{1'b0}});
        chk(mrd(0) === 8'h33, "master 0's concurrent write landed");
        m_run(1, 1'b0, 16'h2200, {DATA_W{1'b0}});
        chk(mrd(1) === 8'h44, "master 1's concurrent write landed");

        //==================================================================
        $display("-- 4. full split transaction, end to end -------------");
        // Seed the split slave while it is not splitting.
        s0_split_en = 1'b0;
        m_run(0, 1'b1, 16'h0010, 8'hCE);
        m_run(1, 1'b1, 16'h2300, 8'h5A);

        s0_split_en = 1'b1;
        fork
            // master 0 reads the split-capable slave: this WILL be deferred
            m_run(0, 1'b0, 16'h0010, {DATA_W{1'b0}});
            // master 1 does real work in the gap the split opens up
            begin
                m_run(1, 1'b0, 16'h2300, {DATA_W{1'b0}});
                m_run(1, 1'b1, 16'h2301, 8'hFE);
                m_run(1, 1'b0, 16'h2301, {DATA_W{1'b0}});
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
        chk(mrd(0) === 8'hCE,   "the RE-ISSUED transfer returned the right data");
        chk(mresp(0) === `RESP_OKAY,    "master 0's final response is OKAY, never SPLIT");
        chk(mrd(1) === 8'hFE,   "master 1's work in the gap was correct");
        chk(split_mask === 2'b00,       "mask cleared again afterwards");
        $display("  ..    split read cost master 0 %0d clocks", lat[0]);

        // A write can be split too, and must still take effect exactly once.
        m_run(0, 1'b1, 16'h0011, 8'h77);
        chk(msplits(0) >= 8'd2, "the write was split as well");
        s0_split_en = 1'b0;
        m_run(0, 1'b0, 16'h0011, {DATA_W{1'b0}});
        chk(mrd(0) === 8'h77, "the split write landed exactly once");

        //==================================================================
        $display("-- 5. unmapped access: ERROR, and the bus RECOVERS ---");
        // The decode hole above slave 2.
        m_run(0, 1'b0, 16'h2800, {DATA_W{1'b0}});
        chk(!timed_out[0],           "0x2800 completed instead of hanging the bus");
        chk(mresp(0) === `RESP_ERROR,"0x2800 answered ERROR");
        chk(err[0]   === 1'b1,       "master 0 reports err");
        chk(gnt      === 2'b00,      "the grant was released, bus not held");

        // The very next transfer must work - that is what "recovers" means.
        m_run(0, 1'b0, 16'h1100, {DATA_W{1'b0}});
        chk(!timed_out[0],            "the next transfer completed normally");
        chk(mrd(0) === 8'h11, "and returned correct data");
        chk(mresp(0) === `RESP_OKAY,  "and a clean OKAY");

        m_run(0, 1'b1, 16'h2FFF, 8'hDD);
        chk(mresp(0) === `RESP_ERROR, "an unmapped WRITE also answers ERROR");

        // The window reserved for the phase-2 remote bridge.
        m_run(1, 1'b0, 16'h8000, {DATA_W{1'b0}});
        chk(!timed_out[1],            "reserved addr[15]=1 window completed");
        chk(mresp(1) === `RESP_ERROR, "reserved window answers ERROR");
        m_run(1, 1'b0, 16'hFFFF, {DATA_W{1'b0}});
        chk(mresp(1) === `RESP_ERROR, "top of memory answers ERROR");
        m_run(1, 1'b0, 16'h2300, {DATA_W{1'b0}});
        chk(mrd(1) === 8'h5A, "master 1 recovered too");

        // A run of bad addresses back to back must not wedge anything.
        for (i = 0; i < 4; i = i + 1) m_run(0, 1'b0, 16'h2900 + i[15:0], {DATA_W{1'b0}});
        chk(!timed_out[0], "four consecutive unmapped accesses all completed");
        m_run(0, 1'b0, 16'h0000, {DATA_W{1'b0}});
        chk(mrd(0) === 8'hA0, "bus fully healthy afterwards");

        //==================================================================
        $display("-- 6. the LOW-priority master can split too ----------");
        s0_split_en = 1'b0;
        m_run(1, 1'b1, 16'h0100, 8'h9C);
        s0_split_en = 1'b1;
        m_run(1, 1'b0, 16'h0100, {DATA_W{1'b0}});
        chk(!timed_out[1],             "master 1's split read completed (did not hang)");
        chk(msplits(1) >= 8'd1,        "master 1 recorded a SPLIT");
        chk(mrd(1) === 8'h9C,  "master 1's re-issued transfer got the data");
        chk(mresp(1) === `RESP_OKAY,   "master 1's final response is OKAY");
        s0_split_en = 1'b0;


        //==================================================================
        $display("-- 7. serial framing ---------------------------------");
        chk(frames_seen > 0,   "address frames were seen on the wire");
        chk(bad_frames == 0,
            "EVERY frame was exactly ADDR_W clocks long");
        $display("  ..    %0d frames on the wire, all %0d clocks", frames_seen, ADDR_W);

        // Drive a known address and confirm the reassembled value.
        m_run(0, 1'b0, 16'h1ABC, {DATA_W{1'b0}});
        chk(frame_addr === 16'h1ABC,
            "the address reassembled off the single wire matches what was sent");
        m_run(1, 1'b0, 16'h2345, {DATA_W{1'b0}});
        chk(frame_addr === 16'h2345,
            "and again from the other master");

        // A split forces a re-issue: the SECOND frame must be a full,
        // correct frame too, not a resumption of the first.
        s0_split_en = 1'b0;
        m_run(0, 1'b1, 16'h0ABC, 8'h6D);
        bad_frames  = 0;
        frames_seen = 0;
        s0_split_en = 1'b1;
        m_run(0, 1'b0, 16'h0ABC, {DATA_W{1'b0}});
        chk(frames_seen == 2,  "the split transaction put TWO frames on the wire");
        chk(bad_frames == 0,   "the re-issued frame was full length as well");
        chk(frame_addr === 16'h0ABC, "the replay re-sent the SAME address");
        chk(mrd(0) === 8'h6D,  "and returned the right data");
        s0_split_en = 1'b0;

        //==================================================================
        $display("======================================================");
        $display(" splits seen: master 0 = %0d, master 1 = %0d",
                 msplits(0), msplits(1));
        if (errors == 0) $display(" tb_integration: PASSED (0 errors)");
        else             $display(" tb_integration: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #2000000;
        $display(" tb_integration: FAILED (global timeout - the bus hung)");
        $finish;
    end

endmodule
