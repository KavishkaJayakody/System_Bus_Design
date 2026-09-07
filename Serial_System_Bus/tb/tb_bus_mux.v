//==========================================================================
// tb_bus_mux.v -- self-checking testbench for bus_mux
//
// Covers:
//   1. reset behaviour   - sel_q cleared, no ready returned
//   2. forward mux       - the GRANTED master's addr/wdata/we/valid appear on
//                          the bus and the other master's do not, for each
//                          grant in turn
//   3. no grant          - bus_valid low and the bus driven to zero
//   4. return mux        - the reply is steered by the select of the PREVIOUS
//                          cycle, so a slave addressed in cycle T is the one
//                          read back in T+1
//   5. stale select      - changing the live select does not corrupt the
//                          reply that belongs to the earlier address; this is
//                          the whole reason the return select is registered
//   6. idle return       - no ready when nothing was selected last cycle
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_bus_mux;

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    reg  [NM-1:0]              gnt, m_valid, m_we;
    reg  [NM*ADDR_W-1:0]       m_addr_flat;
    reg  [NM*DATA_W-1:0]       m_wdata_flat;
    wire                       bus_valid, bus_we;
    wire [ADDR_W-1:0]          bus_addr;
    wire [DATA_W-1:0]          bus_wdata;

    reg  [NS:0]                sel, s_ready;
    reg  [(NS+1)*RESP_W-1:0]   s_resp_flat;
    reg  [(NS+1)*DATA_W-1:0]   s_rdata_flat;
    wire                       bus_ready;
    wire [RESP_W-1:0]          bus_resp;
    wire [DATA_W-1:0]          bus_rdata;
    wire [NS:0]                sel_q;

    integer errors = 0;

    bus_mux #(
        .N_MASTERS(NM), .N_SLAVES(NS),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .gnt(gnt), .m_valid(m_valid), .m_we(m_we),
        .m_addr_flat(m_addr_flat), .m_wdata_flat(m_wdata_flat),
        .bus_valid(bus_valid), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .sel(sel), .s_ready(s_ready),
        .s_resp_flat(s_resp_flat), .s_rdata_flat(s_rdata_flat),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .bus_rdata(bus_rdata),
        .sel_q(sel_q)
    );

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

    initial begin
        $display("======================================================");
        $display(" tb_bus_mux");
        $display("======================================================");

        rst_n        = 1'b0;
        gnt          = 0;
        m_valid      = 0;
        m_we         = 0;
        // master 0 offers a write, master 1 offers a read - deliberately
        // different in every field so a wrong selection is obvious.
        m_addr_flat  = {16'h2004, 16'h1002};      // {m1, m0}
        m_wdata_flat = {32'hBBBB_BBBB, 32'hAAAA_AAAA};
        sel          = 0;
        s_ready      = 0;
        s_resp_flat  = 0;
        s_rdata_flat = 0;

        $display("-- 1. reset behaviour --------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(sel_q     === 4'b0000, "sel_q cleared in reset");
        chk(bus_ready === 1'b0,    "no ready returned in reset");
        @(posedge clk); rst_n = 1'b1; @(posedge clk); #1;

        $display("-- 2. forward mux follows the grant ------------------");
        gnt = 2'b00; m_valid = 2'b11; m_we = 2'b01; #1;
        chk(bus_valid === 1'b0,          "no grant -> bus_valid low");
        chk(bus_addr  === 16'h0000,      "no grant -> bus_addr driven to 0");
        chk(bus_wdata === 32'h0,         "no grant -> bus_wdata driven to 0");

        gnt = 2'b01; #1;
        chk(bus_valid === 1'b1,          "grant m0 -> bus_valid follows m0");
        chk(bus_we    === 1'b1,          "grant m0 -> bus_we    = m0's write");
        chk(bus_addr  === 16'h1002,      "grant m0 -> bus_addr  = m0's addr");
        chk(bus_wdata === 32'hAAAA_AAAA, "grant m0 -> bus_wdata = m0's data");

        gnt = 2'b10; #1;
        chk(bus_we    === 1'b0,          "grant m1 -> bus_we    = m1's read");
        chk(bus_addr  === 16'h2004,      "grant m1 -> bus_addr  = m1's addr");
        chk(bus_wdata === 32'hBBBB_BBBB, "grant m1 -> bus_wdata = m1's data");

        // m_valid of the non-granted master must be ignored entirely
        m_valid = 2'b01; #1;
        chk(bus_valid === 1'b0,
            "grant m1 but only m0 asserts valid -> bus_valid low");
        m_valid = 2'b11;

        $display("-- 3. return mux is steered by the DELAYED select ----");
        // Give each responder a distinct payload.
        s_ready      = 4'b1111;
        s_resp_flat  = {`RESP_ERROR, `RESP_OKAY, `RESP_OKAY, `RESP_SPLIT};
        s_rdata_flat = {32'hDDDD_DDDD, 32'hCCCC_CCCC, 32'h2222_2222, 32'h1111_1111};

        // cycle T: address slave 1
        @(posedge clk); sel <= 4'b0010;
        @(posedge clk); sel <= 4'b0000;  #1;
        chk(sel_q     === 4'b0010,       "sel_q holds last cycle's select");
        chk(bus_ready === 1'b1,          "slave 1's ready returned");
        chk(bus_rdata === 32'h2222_2222, "slave 1's data returned");
        chk(bus_resp  === `RESP_OKAY,    "slave 1's resp returned");

        // cycle T: address slave 0 (which is answering SPLIT)
        @(posedge clk); sel <= 4'b0001;
        @(posedge clk); sel <= 4'b0000; #1;
        chk(bus_rdata === 32'h1111_1111, "slave 0's data returned");
        chk(bus_resp  === `RESP_SPLIT,   "slave 0's SPLIT returned");

        // cycle T: address the default slave
        @(posedge clk); sel <= 4'b1000;
        @(posedge clk); sel <= 4'b0000; #1;
        chk(bus_rdata === 32'hDDDD_DDDD, "default slave's data returned");
        chk(bus_resp  === `RESP_ERROR,   "default slave's ERROR returned");

        $display("-- 4. a new select does not corrupt the old reply ----");
        // Address slave 2 in cycle T and slave 1 in cycle T+1.  In T+1 the
        // reply belongs to slave 2; a mux driven by the LIVE select would
        // wrongly return slave 1 here.
        @(posedge clk); sel <= 4'b0100;
        @(posedge clk); sel <= 4'b0010; #1;
        chk(bus_rdata === 32'hCCCC_CCCC,
            "T+1 still returns slave 2, the slave addressed in T");
        @(posedge clk); sel <= 4'b0000; #1;
        chk(bus_rdata === 32'h2222_2222,
            "T+2 returns slave 1, the slave addressed in T+1");

        $display("-- 5. no reply when nothing was selected -------------");
        @(posedge clk); #1;
        chk(sel_q     === 4'b0000, "sel_q clear");
        chk(bus_ready === 1'b0,    "bus_ready low");

        $display("======================================================");
        if (errors == 0) $display(" tb_bus_mux: PASSED (0 errors)");
        else             $display(" tb_bus_mux: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
