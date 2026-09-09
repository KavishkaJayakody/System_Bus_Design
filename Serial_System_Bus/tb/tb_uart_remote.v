//==========================================================================
// tb_uart_remote.v -- two complete bus systems, crossed by a UART link
//
// Each `bus_top' is a whole board: 2 masters, the serial bus, 3 memory
// slaves.  Master 0 of each has the UART client/server built in, so either
// board can read and write the other's memory.
//
//    A.ext_tx_serial ---> B.ext_rx_serial
//    B.ext_tx_serial ---> A.ext_rx_serial
//
// `link_up' models the cable: dropping it leaves both RX lines idle high,
// which is what an unplugged connector looks like.
//
// Covers:
//   1. each board's own memory, locally - the two are independent
//   2. a REMOTE WRITE from A landing in B's memory and nowhere else
//   3. a REMOTE READ bringing B's data back to A
//   4. the reverse direction - the block is symmetric, one core serves both
//   5. a remote read of the far board's SPLIT slave, which splits on ITS bus
//      while the near board is simply waiting for a UART reply
//   6. BOTH BOARDS ISSUING AT THE SAME INSTANT - the case the shared
//      transmitter's response-priority rule exists for.  Without it both
//      would sit in C_WAIT waiting for an answer neither was sending.
//   7. an unplugged link timing out with cmd_error instead of hanging
//   8. local traffic still working afterwards - a dead link must not wedge
//      the local bus
//
// Tests 12-17 exist because everything above checks the remote READ path
// with a single byte at a single address, which a stuck bit, a reversed bit
// order or a dropped address bit can all survive:
//   12. ALL 256 byte values round-tripped over the link
//   13. every one of the 14 carried address bits, walked individually, plus
//       a neighbouring-word check for an off-by-one in the offset
//   14. a request frame carrying BOTH tags in its payload, and a response
//       whose data byte is itself a tag - the tag-hunting rule is
//       load-bearing and was previously untested
//   15. back-to-back remote reads, so the parser must resynchronise with no
//       idle time between frames
//   16. a remote read returning 0x00 - the value a broken link would give
//       by default, so it has to be shown to be a real answer
//   17. addr[14] does NOT travel: 0xC000 aliases onto 0x8000.  A documented
//       limit of the agreed wire format, pinned so it cannot change quietly
//
// Local transactions must cost exactly what they cost without the UART
// wrapper, so test 1 also records the local latency for comparison.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_uart_remote;

    localparam NM     = `BUS_N_MASTERS;   // bus masters, incl. each bridge
    // COMMAND ports, one per LOCAL master.  The bridge is bus master NM-1 and
    // has no command port - it is driven by the other board.
    localparam NLM    = NM - 1;
    localparam NS     = `BUS_N_SLAVES;
    localparam ID_W   = `BUS_ID_W;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    localparam CPB = 4;      // short bit period for simulation
    localparam TMO = 3000;   // remote reply timeout, clocks
    localparam SPL = 6;      // slave 0 split latency

    reg clk = 1'b0, rst_n = 1'b0;
    always #10 clk = ~clk;

    reg link_up = 1'b1;      // drop to simulate an unplugged cable

    integer errors = 0;

    //======================================================================
    // Two boards.  Only master 0 of each is driven; master 1 stays idle.
    //======================================================================
    reg  [NLM-1:0]        a_valid, a_we;
    reg  [NLM*ADDR_W-1:0] a_addr;
    reg  [NLM*DATA_W-1:0] a_wdata;
    wire [NLM-1:0]        a_accept, a_done, a_err, a_busy;
    wire [NLM*DATA_W-1:0] a_rdata;
    wire [NLM*RESP_W-1:0] a_resp;
    wire [NLM*8-1:0]      a_splits;
    wire                 a_cmd_error, a_tx, a_rembusy, a_srvbusy;

    reg  [NLM-1:0]        b_valid, b_we;
    reg  [NLM*ADDR_W-1:0] b_addr;
    reg  [NLM*DATA_W-1:0] b_wdata;
    wire [NLM-1:0]        b_accept, b_done, b_err, b_busy;
    wire [NLM*DATA_W-1:0] b_rdata;
    wire [NLM*RESP_W-1:0] b_resp;
    wire [NLM*8-1:0]      b_splits;
    wire                 b_cmd_error, b_tx, b_rembusy, b_srvbusy;

    // The cable.  An idle UART line sits HIGH, so an unplugged input is 1.
    //
    // `loopback' models a jumper from a board's own rm_tx straight back to
    // its own rm_rx - the single-board diagnostic tcl/issp_link_test.tcl
    // relies on, so it is proved here first.
    reg  loopback = 1'b0;
    wire a_rx = loopback ? a_tx : (link_up ? b_tx : 1'b1);
    wire b_rx = link_up ? a_tx : 1'b1;

    reg a_split_en = 1'b0, b_split_en = 1'b0;

    bus_top #(
        .NM(NM), .NS(NS), .ID_W(ID_W),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPL), .CLKS_PER_BIT(CPB), .RESP_TIMEOUT(TMO)
    ) A (
        .clk(clk), .rst_n(rst_n), .rst_m_n(rst_n), .rst_s_n(rst_n),
        .cmd_valid(a_valid), .cmd_we(a_we),
        .cmd_addr_flat(a_addr), .cmd_wdata_flat(a_wdata),
        .cmd_accept(a_accept), .done(a_done),
        .rdata_flat(a_rdata), .resp_flat(a_resp), .err(a_err),
        .split_count_flat(a_splits), .mst_busy(a_busy),
        .split_en(a_split_en),
        .cmd_error(a_cmd_error),
        .rm_rx(a_rx), .rm_tx(a_tx),
        .remote_busy(a_rembusy), .srv_busy(a_srvbusy),
        .gnt(), .gnt_valid(), .split_mask(), .sel_q(), .split_busy(),
        .master_id(), .bus_valid(), .bus_we(), .bus_ready(), .bus_resp(),
        .bus_addr(), .addr_done(), .bus_astream(), .bus_dstream()
    );

    bus_top #(
        .NM(NM), .NS(NS), .ID_W(ID_W),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPL), .CLKS_PER_BIT(CPB), .RESP_TIMEOUT(TMO)
    ) B (
        .clk(clk), .rst_n(rst_n), .rst_m_n(rst_n), .rst_s_n(rst_n),
        .cmd_valid(b_valid), .cmd_we(b_we),
        .cmd_addr_flat(b_addr), .cmd_wdata_flat(b_wdata),
        .cmd_accept(b_accept), .done(b_done),
        .rdata_flat(b_rdata), .resp_flat(b_resp), .err(b_err),
        .split_count_flat(b_splits), .mst_busy(b_busy),
        .split_en(b_split_en),
        .cmd_error(b_cmd_error),
        .rm_rx(b_rx), .rm_tx(b_tx),
        .remote_busy(b_rembusy), .srv_busy(b_srvbusy),
        .gnt(), .gnt_valid(), .split_mask(), .sel_q(), .split_busy(),
        .master_id(), .bus_valid(), .bus_we(), .bus_ready(), .bus_resp(),
        .bus_addr(), .addr_done(), .bus_astream(), .bus_dstream()
    );

    //======================================================================
    // Helpers
    //======================================================================
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

    integer a_lat, b_lat;
    integer i, nbad;

    // One transaction on board A's master 0.  The command port takes a LEVEL
    // valid held until accept, so the driving regs are assigned NON-BLOCKING
    // and `accept' is read at the edge - the same pattern tb_integration
    // uses.  Dropping valid before the core samples it loses the command.
    localparam TMO_CLK = 200000;

    // Per-master completion flag, so a test can assert that a transaction
    // finished rather than silently absorbing a hang.
    reg [NLM-1:0] timed_out;

    // Board A, ANY local master.  `a_cmd' below is this with m = 0, kept
    // because most tests only ever drive master 0.
    task automatic a_cmd_m;
        input integer        m;
        input                we_i;
        input [ADDR_W-1:0]   ad;
        input [DATA_W-1:0]   d;
        integer              n;
        begin
            @(posedge clk);
            a_valid[m]              <= 1'b1;
            a_we[m]                 <= we_i;
            a_addr [m*ADDR_W +: ADDR_W] <= ad;
            a_wdata[m*DATA_W +: DATA_W] <= d;

            n = 0;
            @(posedge clk);
            while (!a_accept[m] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            a_valid[m] <= 1'b0;

            while (!a_done[m] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            timed_out[m] = (n >= TMO_CLK);
            if (timed_out[m]) begin
                $display("  ERROR board A master %0d: 0x%04h TIMED OUT", m, ad);
                errors = errors + 1;
            end
            #1;
        end
    endtask

    task automatic a_cmd;
        input                we_i;
        input [ADDR_W-1:0]   ad;
        input [DATA_W-1:0]   d;
        integer              n;
        begin
            @(posedge clk);
            a_valid[0]              <= 1'b1;
            a_we[0]                 <= we_i;
            a_addr [0 +: ADDR_W]    <= ad;
            a_wdata[0 +: DATA_W]    <= d;

            n = 0;
            @(posedge clk);
            while (!a_accept[0] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            a_valid[0] <= 1'b0;

            while (!a_done[0] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            a_lat = n;
            if (n >= TMO_CLK) begin
                $display("  ERROR board A: transaction to 0x%04h TIMED OUT", ad);
                errors = errors + 1;
            end
            #1;
        end
    endtask

    task automatic b_cmd;
        input                we_i;
        input [ADDR_W-1:0]   ad;
        input [DATA_W-1:0]   d;
        integer              n;
        begin
            @(posedge clk);
            b_valid[0]              <= 1'b1;
            b_we[0]                 <= we_i;
            b_addr [0 +: ADDR_W]    <= ad;
            b_wdata[0 +: DATA_W]    <= d;

            n = 0;
            @(posedge clk);
            while (!b_accept[0] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            b_valid[0] <= 1'b0;

            while (!b_done[0] && n < TMO_CLK) begin @(posedge clk); n = n + 1; end
            b_lat = n;
            if (n >= TMO_CLK) begin
                $display("  ERROR board B: transaction to 0x%04h TIMED OUT", ad);
                errors = errors + 1;
            end
            #1;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_uart_remote -- two boards, one crossed UART link");
        $display("======================================================");

        timed_out = {NLM{1'b0}};
        a_valid = {NLM{1'b0}}; a_we = {NLM{1'b0}};
        a_addr  = {NLM*ADDR_W{1'b0}}; a_wdata = {NLM*DATA_W{1'b0}};
        b_valid = {NLM{1'b0}}; b_we = {NLM{1'b0}};
        b_addr  = {NLM*ADDR_W{1'b0}}; b_wdata = {NLM*DATA_W{1'b0}};

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        //==================================================================
        $display("-- 1. each board's own memory, locally ---------------");
        a_cmd(1'b1, 16'h1ABC, 8'h11);
        $display("  ..    local WRITE latency %0d clocks", a_lat);
        chk(a_lat == 21, "a local write still costs 21 clocks - the wrapper is a pass-through");
        b_cmd(1'b1, 16'h1ABC, 8'h22);
        a_cmd(1'b0, 16'h1ABC, 8'h00);
        $display("  ..    local READ  latency %0d clocks", a_lat);
        chk(a_lat == 30, "a local read still costs 30 clocks");
        chk(a_rdata[0 +: DATA_W] === 8'h11, "A's own 0x1ABC reads back 0x11");
        b_cmd(1'b0, 16'h1ABC, 8'h00);
        chk(b_rdata[0 +: DATA_W] === 8'h22, "B's own 0x1ABC reads back 0x22 - independent");

        //==================================================================
        // The ADDRESS selects the far board: 0x9ABC here is 0x1ABC there.
        // Writes are POSTED, so A retires as soon as the request is on the
        // wire - before B has executed it.  Settle before reading back.
        //==================================================================
        $display("-- 2. A REMOTE WRITE: 0x9ABC here = B's 0x1ABC -------");
        a_cmd(1'b1, 16'h9ABC, 8'h5A);
        chk(!a_cmd_error, "the posted write reported no error");
        chk(a_resp[0 +: RESP_W] === `RESP_OKAY, "and reports OKAY");
        $display("  ..    posted remote write retired in %0d clocks", a_lat);
        chk(a_lat < 400, "it retired on send, without waiting for a reply");
        repeat (600) @(posedge clk);        // let B actually execute it

        $display("-- 3. it landed on B, and only on B ------------------");
        b_cmd(1'b0, 16'h1ABC, 8'h00);
        chk(b_rdata[0 +: DATA_W] === 8'h5A, "B's 0x1ABC now holds A's 0x5A");
        a_cmd(1'b0, 16'h1ABC, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h11, "A's own 0x1ABC is untouched");

        //==================================================================
        $display("-- 4. A REMOTE READ of B's 0x1ABC (via 0x9ABC) -------");
        a_cmd(1'b0, 16'h9ABC, 8'h00);
        chk(!a_cmd_error,                   "the remote read completed");
        chk(a_rdata[0 +: DATA_W] === 8'h5A, "and brought B's data back");
        $display("  ..    remote read took %0d clocks", a_lat);

        //==================================================================
        // Device ids: 0x8xxx -> slave 0 (2K), 0x9xxx -> slave 1 (4K),
        // 0xAxxx -> slave 2 (4K).  One of each, both directions.
        //==================================================================
        $display("-- 5. all three device ids, and the reverse direction -");
        b_cmd(1'b1, 16'h05C3, 8'hD0);       // B's slave 0, id 0
        b_cmd(1'b1, 16'h2567, 8'hD2);       // B's slave 2, id 2
        a_cmd(1'b0, 16'h85C3, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'hD0, "0x85C3 reached B's slave 0 (id 0, 2K)");
        a_cmd(1'b0, 16'hA567, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'hD2, "0xA567 reached B's slave 2 (id 2, 4K)");

        a_cmd(1'b1, 16'h2FFF, 8'hC7);       // seed A's slave 2, last word
        b_cmd(1'b0, 16'hAFFF, 8'h00);       // B reads it remotely
        chk(!b_cmd_error,                   "B's remote read completed");
        chk(b_rdata[0 +: DATA_W] === 8'hC7, "B read A's 0x2FFF - slave 2 really is 4K");

        //==================================================================
        $display("-- 6. remote read of the far board's SPLIT slave -----");
        b_cmd(1'b1, 16'h2A5C, 8'h7E);       // seed B's slave 2 (the splitter)
        b_split_en = 1'b1;                  // make B's slave 2 split
        @(posedge clk); #1;
        a_cmd(1'b0, 16'hAA5C, 8'h00);
        chk(!a_cmd_error,                   "the remote split read completed");
        chk(a_rdata[0 +: DATA_W] === 8'h7E, "and returned the right data");
        chk(b_splits[0 +: 8] > 8'd0,        "B's master 0 really absorbed a SPLIT");
        b_split_en = 1'b0;
        @(posedge clk); #1;

        //==================================================================
        // The case the response-priority rule exists for: if both boards
        // sent a request and then waited, neither would ever answer.
        $display("-- 7. BOTH boards read each other at once ------------");
        a_cmd(1'b1, 16'h2100, 8'hA1);       // seed both sides
        b_cmd(1'b1, 16'h2100, 8'hB2);
        fork
            a_cmd(1'b0, 16'hA100, 8'h00);   // A reads B
            b_cmd(1'b0, 16'hA100, 8'h00);   // B reads A
        join
        chk(!a_cmd_error && !b_cmd_error, "neither side deadlocked - both got an answer");
        chk(a_rdata[0 +: DATA_W] === 8'hB2, "A got B's byte");
        chk(b_rdata[0 +: DATA_W] === 8'hA1, "B got A's byte");

        //==================================================================
        $display("-- 8. unplug the cable -------------------------------");
        link_up = 1'b0;
        @(posedge clk); #1;
        a_cmd(1'b0, 16'h9ABC, 8'h00);
        chk(a_cmd_error,                    "the remote read reported cmd_error");
        chk(a_rdata[0 +: DATA_W] === 8'hFF, "and returned 0xFF, as the spec requires");
        chk(a_resp[0 +: RESP_W] === `RESP_ERROR, "resp = ERROR");
        chk(a_lat < TMO_CLK,                "the master finished, so it was the LINK that timed out");

        //==================================================================
        $display("-- 9. the local bus survived the dead link -----------");
        a_cmd(1'b0, 16'h1ABC, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h11, "A's local read still works");
        chk(!a_err[0],                      "and the local transfer itself is clean");
        // br_error belongs to the BRIDGE now, not to master 0's command
        // stream: it says "the last thing that went over the wire failed"
        // and stays true until another remote transaction is attempted.  A
        // local read is unrelated to the link and no longer clears it - which
        // is the more useful reading, and why the check above moved to err[0].
        chk(a_cmd_error,                    "br_error still reports the dead link");
        a_cmd(1'b1, 16'h1DEF, 8'h3C);
        a_cmd(1'b0, 16'h1DEF, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h3C, "and a fresh local write/read round-trips");

        link_up = 1'b1;
        @(posedge clk); #1;

        //==================================================================
        $display("-- 10. and the link recovers when plugged back in ----");
        a_cmd(1'b0, 16'h9ABC, 8'h00);
        chk(!a_cmd_error,                   "a remote read works again");
        chk(a_rdata[0 +: DATA_W] === 8'h5A, "and returns B's data");

        //==================================================================
        // A jumper from rm_tx back to rm_rx makes the board answer its OWN
        // remote requests out of its OWN memory.  That exercises the entire
        // link path - client, framing, parser, server, bus, response - with
        // no second board, which is what makes it the first thing to try
        // when the real link misbehaves.
        $display("-- 11. LOOPBACK: the board answers itself --------------");
        link_up  = 1'b0;
        loopback = 1'b1;
        @(posedge clk); #1;
        a_cmd(1'b1, 16'h1ABC, 8'hE7);           // seed A's own 0x1ABC
        a_cmd(1'b0, 16'h9ABC, 8'h00);           // remote read -> itself
        chk(!a_cmd_error,                   "the looped-back read completed");
        chk(a_rdata[0 +: DATA_W] === 8'hE7, "0x9ABC returned A's own 0x1ABC");

        a_cmd(1'b1, 16'hA100, 8'h4D);           // posted remote write -> itself
        repeat (600) @(posedge clk);
        a_cmd(1'b0, 16'h2100, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h4D, "a looped-back posted write landed");

        loopback = 1'b0;
        link_up  = 1'b1;
        @(posedge clk); #1;

        //==================================================================
        // The remote READ path is the one the tests above exercise most
        // lightly: each checks a single byte at a single address.  A stuck
        // bit, a reversed bit order or a dropped address bit would survive
        // all of them.  Tests 12-16 close that.
        //==================================================================
        $display("-- 12. EVERY byte value survives a remote read -------");
        // Seed 256 consecutive words of B's slave 1 locally, then read every
        // one of them back over the link.  This is the definitive test of
        // the 8-bit response path: value v lands at 0x1100+v, so a stuck or
        // swapped bit cannot alias one correct answer onto another.
        for (i = 0; i < 256; i = i + 1)
            b_cmd(1'b1, 16'h1100 + i[15:0], i[7:0]);

        nbad = 0;
        for (i = 0; i < 256; i = i + 1) begin
            a_cmd(1'b0, 16'h9100 + i[15:0], 8'h00);
            if (a_cmd_error || a_rdata[0 +: DATA_W] !== i[7:0]) begin
                if (nbad < 4)
                    $display("        value 0x%02h read back as 0x%02h%0s",
                             i[7:0], a_rdata[0 +: DATA_W],
                             a_cmd_error ? "  (cmd_error)" : "");
                nbad = nbad + 1;
            end
        end
        chk(nbad == 0, "all 256 byte values round-tripped over the link");

        //==================================================================
        $display("-- 13. no address bit is lost on the way -------------");
        // Walk every bit of the 14-bit address the link actually carries.
        // Offset bits 0..11 are walked inside slave 1; bits 12..13 are the
        // device field and are walked by the three slave bases.  Each
        // location gets a distinct marker, so a dropped or swapped address
        // bit shows up as one location answering with another's data.
        for (i = 0; i < 12; i = i + 1)
            b_cmd(1'b1, 16'h1000 + (16'd1 << i), 8'h40 + i[7:0]);

        nbad = 0;
        for (i = 0; i < 12; i = i + 1) begin
            a_cmd(1'b0, 16'h9000 + (16'd1 << i), 8'h00);
            if (a_rdata[0 +: DATA_W] !== 8'h40 + i[7:0]) begin
                $display("        offset bit %0d: expected 0x%02h, got 0x%02h",
                         i[7:0], 8'h40 + i[7:0], a_rdata[0 +: DATA_W]);
                nbad = nbad + 1;
            end
        end
        chk(nbad == 0, "all 12 offset bits reached the far board intact");

        // The device field, bits 13:12 of the carried address.
        b_cmd(1'b1, 16'h0044, 8'hE0);       // B slave 0
        b_cmd(1'b1, 16'h1044, 8'hE1);       // B slave 1
        b_cmd(1'b1, 16'h2044, 8'hE2);       // B slave 2
        a_cmd(1'b0, 16'h8044, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'hE0, "device field 00 selected the far slave 0");
        a_cmd(1'b0, 16'h9044, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'hE1, "device field 01 selected the far slave 1");
        a_cmd(1'b0, 16'hA044, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'hE2, "device field 10 selected the far slave 2");

        // Neighbouring words must not alias - the cheapest way to catch an
        // off-by-one in the offset field.
        b_cmd(1'b1, 16'h1500, 8'h71);
        b_cmd(1'b1, 16'h1501, 8'h72);
        a_cmd(1'b0, 16'h9500, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h71, "0x9500 read its own word");
        a_cmd(1'b0, 16'h9501, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h72, "0x9501 read the NEXT word, not the same one");

        //==================================================================
        $display("-- 14. tag bytes inside the payload ------------------");
        // TAG HUNTING is load-bearing: a payload byte may itself be 0xA5 or
        // 0x5A, and the parser must take a fixed number of bytes rather than
        // re-scanning.  A write of 0xA5 to far 0x1016 puts BOTH tags in one
        // request frame - it goes out as A5 5A 40 A5 - and reading it back
        // returns a RESPONSE whose data byte is itself a REQUEST tag.
        a_cmd(1'b1, 16'h9016, 8'hA5);
        repeat (600) @(posedge clk);
        a_cmd(1'b0, 16'h9016, 8'h00);
        chk(!a_cmd_error,                   "a request carrying both tags was parsed");
        chk(a_rdata[0 +: DATA_W] === 8'hA5, "and 0xA5 came back as data, not as a tag");

        a_cmd(1'b1, 16'h901A, 8'h5A);
        repeat (600) @(posedge clk);
        a_cmd(1'b0, 16'h901A, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h5A, "0x5A survives as a data byte too");

        //==================================================================
        $display("-- 15. back-to-back remote reads ---------------------");
        // No pause between transactions: the parser must resynchronise from
        // one frame to the next with no idle time to recover in.
        b_cmd(1'b1, 16'h1600, 8'h81);
        b_cmd(1'b1, 16'h1601, 8'h82);
        b_cmd(1'b1, 16'h1602, 8'h83);
        b_cmd(1'b1, 16'h1603, 8'h84);
        nbad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            a_cmd(1'b0, 16'h9600 + i[15:0], 8'h00);
            if (a_cmd_error || a_rdata[0 +: DATA_W] !== 8'h81 + i[7:0])
                nbad = nbad + 1;
        end
        chk(nbad == 0, "four consecutive remote reads each returned their own byte");

        //==================================================================
        $display("-- 16. a remote read of ZERO is not a failure --------");
        // 0x00 is the value a broken link would plausibly return by
        // default, so it has to be shown to be a real answer: A's own copy
        // of the address holds something else, and cmd_error stays clear.
        b_cmd(1'b1, 16'h1700, 8'h00);       // far side: zero
        a_cmd(1'b1, 16'h1700, 8'hFF);       // near side: not zero
        a_cmd(1'b0, 16'h9700, 8'h00);
        chk(!a_cmd_error,                   "the remote read of 0x00 completed");
        chk(a_rdata[0 +: DATA_W] === 8'h00, "and returned 0x00 from B, not A's 0xFF");

        //==================================================================
        $display("-- 17. the bridge window is exactly the 14 bits it carries");
        // The link command carries 14 address bits and the bridge window is
        // exactly 16K, so every address the window accepts travels intact.
        // 0xC000 and above are OUTSIDE the window and are a decode hole -
        // they no longer alias onto 0x8000, which they did while the remote
        // window was intercepted in the master rather than decoded.
        b_cmd(1'b1, 16'h0055, 8'h3B);
        a_cmd(1'b0, 16'h8055, 8'h00);
        chk(a_rdata[0 +: DATA_W] === 8'h3B, "0x8055 reads B's 0x0055");
        a_cmd(1'b0, 16'hBFFF, 8'h00);
        chk(!a_cmd_error,                   "0xBFFF is the top of the window and still works");
        a_cmd(1'b0, 16'hC055, 8'h00);
        chk(a_resp[0 +: RESP_W] === `RESP_ERROR,
            "0xC055 is ABOVE the window - a decode hole, no longer an alias");

        //==================================================================
        // The point of taking the UART out of master 0 and putting it on the
        // bus as a device: the far board is reachable from ANY master, not
        // just the one that happened to own the wire.  This is impossible in
        // the bus_bridge design and is the reason for the refactor.
        $display("-- 18. EITHER master can reach the far board ---------");
        b_cmd(1'b1, 16'h1C00, 8'h91);
        a_cmd_m(0, 1'b0, 16'h9C00, 8'h00);
        chk(!a_cmd_error,                   "master 0 read the far board");
        chk(a_rdata[0*DATA_W +: DATA_W] === 8'h91, "and got the right byte");

        a_cmd_m(1, 1'b0, 16'h9C00, 8'h00);
        chk(a_rdata[1*DATA_W +: DATA_W] === 8'h91,
            "MASTER 1 read the far board too - it has no UART of its own");

        // and a remote WRITE from master 1
        a_cmd_m(1, 1'b1, 16'h9C01, 8'h92);
        repeat (600) @(posedge clk);
        b_cmd(1'b0, 16'h1C01, 8'h00);
        chk(b_rdata[0 +: DATA_W] === 8'h92, "master 1's remote WRITE landed on B");

        //==================================================================
        $display("-- 19. the bridge is ONE transaction at a time -------");
        // A second master addressing the bridge while a round trip is in
        // flight is answered ERROR, not deferred - there is one set of
        // deferred state and one split_complete to release it with.  It must
        // COMPLETE, though: the bus never hangs.
        link_up = 1'b0;                     // make the round trip take the
        @(posedge clk); #1;                 // full timeout, so it stays busy
        fork
            a_cmd_m(0, 1'b0, 16'h9C00, 8'h00);
            begin
                repeat (40) @(posedge clk);
                a_cmd_m(1, 1'b0, 16'h9C00, 8'h00);
            end
        join
        chk(!timed_out[0] && !timed_out[1],
            "both masters completed - a busy bridge never hangs the bus");
        chk(a_resp[1*RESP_W +: RESP_W] === `RESP_ERROR,
            "the second master was answered ERROR while the bridge was busy");
        link_up = 1'b1;
        @(posedge clk); #1;
        a_cmd_m(0, 1'b0, 16'h9C00, 8'h00);
        chk(a_rdata[0*DATA_W +: DATA_W] === 8'h91,
            "and the bridge is usable again straight afterwards");

        $display("======================================================");
        if (errors == 0) $display(" tb_uart_remote: PASSED (0 errors)");
        else             $display(" tb_uart_remote: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    // Global watchdog: a deadlocked link must fail, not hang the regression.
    initial begin
        #20_000_000;
        $display(" tb_uart_remote: FAILED (timeout)");
        $finish;
    end

endmodule
