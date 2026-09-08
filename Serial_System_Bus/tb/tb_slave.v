//==========================================================================
// tb_slave.v -- self-checking testbench for slave
//
// Two DUTs from the same module: a plain 2K slave and a 4K split-capable
// slave, so both SPLIT_CAPABLE builds are covered.
//
// The testbench drives the wires the way the bus does: `frame' high for
// ADDR_W clocks with the address MSB-first on astream and the write data
// right-aligned on dstream_in, then a one-cycle `sel'.
//
// Covers:
//   1. reset behaviour       - ready low, not busy, wire not driven
//   2. write / read back     - data integrity through the serial path
//   3. response timing       - a write answers at S+1, a read at S+10, and
//                              ready is one cycle wide in both cases
//   4. address extraction    - a LADDR_W-wide deserialiser fed the whole
//                              16-bit frame must end up holding exactly the
//                              low bits: 0x1ABC and 0xFABC must hit the SAME
//                              offset 0xABC, and 0x040 must not alias
//   5. NOT SELECTED          - a slave shifts every frame in whether or not
//                              it is addressed, so it must do nothing at all
//                              without `sel': no ready, no memory change
//   6. split read            - SPLIT at S+1 (fast) while a real read costs
//                              ten cycles, split_complete on the right bit,
//                              replay served with the correct data
//   7. split write           - the deferred write does NOT land until the
//                              replay
//   8. one split outstanding - another master is served normally meanwhile
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_slave;

    localparam ADDR_W  = `BUS_ADDR_W;
    localparam DATA_W  = `BUS_DATA_W;
    localparam RESP_W  = `BUS_RESP_W;
    localparam N       = `BUS_N_MASTERS;
    localparam ID_W    = 1;
    localparam SPL_LAT = 4;

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
    // Shared serial wires
    //======================================================================
    reg              frame, astream, dstream_in, we;
    reg  [ID_W-1:0]  mid;

    // DUT A: plain slave, 2K words (slave 2 geometry)
    reg               a_sel;
    wire              a_dout, a_ready, a_busy;
    wire [RESP_W-1:0] a_resp;
    wire [N-1:0]      a_sc;

    slave #(
        .DATA_W(DATA_W), .LADDR_W(11), .WORDS(2048), .RESP_W(RESP_W),
        .N_MASTERS(N), .ID_W(ID_W), .SPLIT_CAPABLE(0)
    ) dut_plain (
        .clk(clk), .rst_n(rst_n),
        .frame(frame), .astream(astream), .dstream_in(dstream_in),
        .sel(a_sel), .we(we), .master_id(mid), .split_en(1'b0),
        .dstream_out(a_dout), .ready(a_ready), .resp(a_resp),
        .split_complete(a_sc), .busy(a_busy)
    );

    // DUT B: split-capable slave, 4K words (slave 0 geometry)
    reg               b_sel, b_split_en;
    wire              b_dout, b_ready, b_busy;
    wire [RESP_W-1:0] b_resp;
    wire [N-1:0]      b_sc;

    slave #(
        .DATA_W(DATA_W), .LADDR_W(12), .WORDS(4096), .RESP_W(RESP_W),
        .N_MASTERS(N), .ID_W(ID_W), .SPLIT_CAPABLE(1), .SPLIT_LATENCY(SPL_LAT)
    ) dut_split (
        .clk(clk), .rst_n(rst_n),
        .frame(frame), .astream(astream), .dstream_in(dstream_in),
        .sel(b_sel), .we(we), .master_id(mid), .split_en(b_split_en),
        .dstream_out(b_dout), .ready(b_ready), .resp(b_resp),
        .split_complete(b_sc), .busy(b_busy)
    );

    // Captured results of the last access.
    reg  [RESP_W-1:0] cap_resp;
    reg  [DATA_W-1:0] cap_rdata;
    integer           cap_lat;        // clocks from sel to ready

    // Count split_complete pulses.
    integer sc_pulses  = 0;
    integer sc_last_id = -1;
    always @(posedge clk) begin
        if (rst_n && |b_sc) begin
            sc_pulses  = sc_pulses + 1;
            sc_last_id = b_sc[0] ? 0 : 1;
        end
    end

    //----------------------------------------------------------------------
    // Drive one frame on the shared wires.  `do_sel' picks which DUT (if
    // any) gets the select pulse: 0 = none, 1 = plain, 2 = split.
    //----------------------------------------------------------------------
    task automatic ser_access;
        input integer           do_sel;
        input                   we_i;
        input [ADDR_W-1:0]      a;
        input [DATA_W-1:0]      d;
        input [ID_W-1:0]        id;
        integer                 k;
        reg   [DATA_W-1:0]      rx;
        reg                     got;
        begin
            // --- address frame, MSB first, data right-aligned ------------
            for (k = ADDR_W-1; k >= 0; k = k - 1) begin
                @(posedge clk);
                frame      <= 1'b1;
                astream    <= a[k];
                dstream_in <= (k < DATA_W) ? d[k] : 1'b0;
                we         <= we_i;
                mid        <= id;
            end

            // --- one-cycle select ----------------------------------------
            @(posedge clk);
            frame      <= 1'b0;
            astream    <= 1'b0;
            dstream_in <= 1'b0;
            a_sel      <= (do_sel == 1);
            b_sel      <= (do_sel == 2);

            @(posedge clk);
            a_sel <= 1'b0;
            b_sel <= 1'b0;

            // --- collect the reply ---------------------------------------
            // Read data is "the last DATA_W bits on the wire before ready",
            // exactly as master.v takes it.
            rx = {DATA_W{1'b0}};
            got = 1'b0;
            cap_lat = 1;
            for (k = 0; k < 60 && !got; k = k + 1) begin
                #1;
                if ((do_sel == 1 && a_ready) || (do_sel == 2 && b_ready)
                    || (do_sel == 0 && (a_ready || b_ready))) begin
                    got      = 1'b1;
                    cap_resp = (do_sel == 1) ? a_resp : b_resp;
                end else begin
                    rx = {rx[DATA_W-2:0], (do_sel == 1) ? a_dout : b_dout};
                    @(posedge clk);
                    cap_lat = cap_lat + 1;
                end
            end
            cap_rdata = rx;
            if (!got) cap_lat = -1;
        end
    endtask

    integer i;

    initial begin
        $display("======================================================");
        $display(" tb_slave");
        $display("======================================================");

        rst_n = 1'b0;
        frame = 0; astream = 0; dstream_in = 0; we = 0; mid = 0;
        a_sel = 0; b_sel = 0; b_split_en = 0;

        //------------------------------------------------------------------
        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(a_ready === 1'b0, "plain slave: ready low in reset");
        chk(b_ready === 1'b0, "split slave: ready low in reset");
        chk(b_busy  === 1'b0, "split slave: not busy in reset");
        chk(a_dout  === 1'b0, "plain slave: not driving the data wire");
        chk(b_dout  === 1'b0, "split slave: not driving the data wire");
        chk(b_sc    === {N{1'b0}}, "split_complete clear in reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(a_ready === 1'b0, "no ready without an access");

        //------------------------------------------------------------------
        $display("-- 2. write and read back ----------------------------");
        ser_access(1, 1'b1, 16'h2123, 8'hD7, 1'b0);
        chk(cap_resp === `RESP_OKAY, "write answered OKAY");
        chk(cap_lat  == 1,           "a write answers at S+1");

        ser_access(1, 1'b0, 16'h2123, 8'h00, 1'b0);
        chk(cap_resp  === `RESP_OKAY, "read answered OKAY");
        chk(cap_rdata === 8'hD7,      "read data came back through the wire");
        chk(cap_lat   == 10,          "a read answers at S+10 (memory + 8 bits)");

        @(posedge clk); #1;
        chk(a_ready === 1'b0, "ready is only one cycle wide");

        //------------------------------------------------------------------
        $display("-- 3. only the LOW address bits matter ---------------");
        // The slave's deserialiser is LADDR_W wide but is fed all 16 bits.
        ser_access(1, 1'b1, 16'h2000, 8'h10, 1'b0);   // offset 0x000
        ser_access(1, 1'b1, 16'h2001, 8'h11, 1'b0);   // offset 0x001
        ser_access(1, 1'b1, 16'h2040, 8'h12, 1'b0);   // offset 0x040
        ser_access(1, 1'b1, 16'h27FF, 8'h13, 1'b0);   // last word
        ser_access(1, 1'b0, 16'h2000, 8'h00, 1'b0);
        chk(cap_rdata === 8'h10, "offset 0x000 intact");
        ser_access(1, 1'b0, 16'h2001, 8'h00, 1'b0);
        chk(cap_rdata === 8'h11, "offset 0x001 intact");
        ser_access(1, 1'b0, 16'h2040, 8'h00, 1'b0);
        chk(cap_rdata === 8'h12, "offset 0x040 intact (no 0x40 aliasing)");
        ser_access(1, 1'b0, 16'h27FF, 8'h00, 1'b0);
        chk(cap_rdata === 8'h13, "last word 0x7FF intact");

        // Two totally different full addresses with the same low bits must
        // land on the same word - the upper bits are the decoder's business,
        // not the slave's.  This DUT is 11 bits wide (2K), so the addresses
        // have to agree in addr[10:0]:
        //   0x2123 = 0010_0001_0010_0011  ->  010_0001_0010_0011 & 0x7FF = 0x123
        //   0xF923 = 1111_1001_0010_0011  ->                              0x123
        ser_access(1, 1'b1, 16'h2123, 8'h5C, 1'b0);
        ser_access(1, 1'b0, 16'hF923, 8'h00, 1'b0);
        chk(cap_rdata === 8'h5C,
            "0xF923 and 0x2123 share offset 0x123 - upper bits discarded");

        //------------------------------------------------------------------
        $display("-- 4. a frame with NO select must do nothing ---------");
        // Every slave shifts every frame in, addressed or not.  Acting on
        // one without `sel' would corrupt another slave's transfer.
        ser_access(0, 1'b1, 16'h2123, 8'hFF, 1'b0);   // no select at all
        chk(cap_lat == -1, "no slave answered an unselected frame");
        ser_access(1, 1'b0, 16'h2123, 8'h00, 1'b0);
        chk(cap_rdata === 8'h5C,
            "and the unselected write did NOT reach memory");

        //------------------------------------------------------------------
        $display("-- 5. split read -------------------------------------");
        b_split_en = 1'b0;
        ser_access(2, 1'b1, 16'h0010, 8'h3E, 1'b0);
        chk(cap_resp === `RESP_OKAY, "seed write served normally (split_en=0)");

        sc_pulses  = 0;
        b_split_en = 1'b1;
        ser_access(2, 1'b0, 16'h0010, 8'h00, 1'b0);
        chk(cap_resp === `RESP_SPLIT, "fresh access answered with SPLIT");
        chk(cap_lat  == 1,
            "the SPLIT arrives at S+1 - far faster than the 10-cycle read it defers");
        chk(b_busy   === 1'b1, "slave busy with the deferred transfer");

        wait (sc_pulses == 1); #1;
        chk(sc_last_id == 0, "split_complete pulsed on master 0's bit");
        repeat (4) @(posedge clk); #1;
        chk(sc_pulses == 1,  "split_complete is exactly one cycle wide");
        chk(b_busy === 1'b0, "slave no longer busy");

        ser_access(2, 1'b0, 16'h0010, 8'h00, 1'b0);
        chk(cap_resp  === `RESP_OKAY, "replay served with OKAY, not split again");
        chk(cap_rdata === 8'h3E,      "replay returned the correct data");
        chk(cap_lat   == 10,          "and took the full read time");

        //------------------------------------------------------------------
        $display("-- 6. split write ------------------------------------");
        sc_pulses = 0;
        ser_access(2, 1'b1, 16'h0010, 8'h99, 1'b0);
        chk(cap_resp === `RESP_SPLIT, "fresh write answered with SPLIT");
        wait (sc_pulses == 1); #1;
        b_split_en = 1'b0;
        ser_access(2, 1'b0, 16'h0010, 8'h00, 1'b0);
        chk(cap_rdata === 8'h3E,
            "the deferred write did NOT take effect during the split");
        b_split_en = 1'b1;
        sc_pulses = 0;
        ser_access(2, 1'b1, 16'h0010, 8'h99, 1'b0);
        chk(cap_resp === `RESP_SPLIT, "write splits again from a fresh start");
        wait (sc_pulses == 1); #1;
        ser_access(2, 1'b1, 16'h0010, 8'h99, 1'b0);
        chk(cap_resp === `RESP_OKAY, "write replay served with OKAY");
        b_split_en = 1'b0;
        ser_access(2, 1'b0, 16'h0010, 8'h00, 1'b0);
        chk(cap_rdata === 8'h99, "the replayed write took effect");

        //------------------------------------------------------------------
        $display("-- 7. only one split outstanding ---------------------");
        b_split_en = 1'b1;
        sc_pulses  = 0;
        ser_access(2, 1'b0, 16'h0020, 8'h00, 1'b0);   // master 0 -> SPLIT
        chk(cap_resp === `RESP_SPLIT, "master 0's access splits");
        chk(b_busy === 1'b1,          "slave busy");
        ser_access(2, 1'b0, 16'h0010, 8'h00, 1'b1);   // master 1 while busy
        chk(cap_resp  === `RESP_OKAY,
            "master 1 served normally while a split is in flight");
        chk(cap_rdata === 8'h99, "master 1 got real data");
        wait (sc_pulses == 1); #1;
        chk(sc_last_id == 0, "the wake-up still belongs to master 0");
        b_split_en = 1'b0;

        //------------------------------------------------------------------
        $display("-- 8. the data wire is left alone when idle ----------");
        for (i = 0; i < 8; i = i + 1) begin
            @(posedge clk); #1;
            if (a_dout !== 1'b0 || b_dout !== 1'b0) begin
                $display("  ERROR a slave drove the data wire while idle");
                errors = errors + 1;
            end
        end
        $display("  ok    both slaves quiet for 8 idle cycles");

        $display("======================================================");
        if (errors == 0) $display(" tb_slave: PASSED (0 errors)");
        else             $display(" tb_slave: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #500000;
        $display(" tb_slave: FAILED (timeout)");
        $finish;
    end

endmodule
