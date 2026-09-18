//==========================================================================
// tb_bus_mux.v -- self-checking testbench for bus_mux
//
// Covers:
//   1. reset behaviour   - sel_q cleared, no ready returned
//   2. forward mux       - the GRANTED master owns bus_astream and the
//                          control lines; the other master is ignored even
//                          when it is driving
//   3. no grant          - the bus is quiet
//   4. data wire direction - on a write the wire carries the MASTER's bit,
//                          on a read it carries the SELECTED SLAVE's bit,
//                          and the two can never fight because the direction
//                          is just bus_we
//   5. LATCHED return select - the select is captured on the decoder's
//                          one-cycle pulse and HELD.  This is the difference
//                          from the parallel bus: a write answers in 1 cycle
//                          and a read in 10, so a one-cycle-delayed select
//                          would have gone stale long before the read reply
//                          arrived.
//   6. idle return       - no ready before anything has been selected
//==========================================================================
`timescale 1ns/1ps
`include "../rtl/bus_defs.vh"

module tb_bus_mux;

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam RESP_W = `BUS_RESP_W;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    reg  [NM-1:0]              gnt, m_valid, m_we, m_astream, m_dstream;
    wire                       bus_valid, bus_we, bus_astream, bus_dstream;

    reg  [NS:0]                sel, s_ready, s_dstream;
    reg  [(NS+1)*RESP_W-1:0]   s_resp_flat;
    wire                       bus_ready;
    wire [RESP_W-1:0]          bus_resp;
    wire [NS:0]                sel_q;

    integer errors = 0;
    integer k;

    bus_mux #(.N_MASTERS(NM), .N_SLAVES(NS), .RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .gnt(gnt), .m_valid(m_valid), .m_we(m_we),
        .m_astream(m_astream), .m_dstream(m_dstream),
        .bus_valid(bus_valid), .bus_we(bus_we),
        .bus_astream(bus_astream), .bus_dstream(bus_dstream),
        .sel(sel), .s_ready(s_ready), .s_resp_flat(s_resp_flat),
        .s_dstream(s_dstream),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .sel_q(sel_q)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t sel_q=%b)", name, $time, sel_q);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_bus_mux");
        $display("======================================================");

        rst_n       = 1'b0;
        gnt         = 0; m_valid = 0; m_we = 0;
        m_astream   = 2'b10;          // master 0 sends 0, master 1 sends 1
        m_dstream   = 2'b01;          // master 0 sends 1, master 1 sends 0
        sel         = 0; s_ready = 0; s_dstream = 0;
        s_resp_flat = 0;

        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(sel_q     === 4'b0000, "sel_q cleared in reset");
        chk(bus_ready === 1'b0,    "no ready returned in reset");
        @(posedge clk); rst_n = 1'b1; @(posedge clk); #1;

        $display("-- 2. forward mux follows the grant ------------------");
        gnt = 2'b00; m_valid = 2'b11; m_we = 2'b01; #1;
        chk(bus_valid   === 1'b0, "no grant -> bus_valid low");
        chk(bus_astream === 1'b0, "no grant -> address wire driven low");

        gnt = 2'b01; #1;
        chk(bus_valid   === 1'b1, "grant m0 -> bus_valid follows m0");
        chk(bus_we      === 1'b1, "grant m0 -> bus_we = m0's write");
        chk(bus_astream === 1'b0, "grant m0 -> address wire = m0's bit (0)");

        gnt = 2'b10; #1;
        chk(bus_we      === 1'b0, "grant m1 -> bus_we = m1's read");
        chk(bus_astream === 1'b1, "grant m1 -> address wire = m1's bit (1)");

        // the ungranted master must be invisible even while driving
        m_valid = 2'b01; #1;
        chk(bus_valid === 1'b0,
            "grant m1 but only m0 asserts valid -> bus_valid low");
        m_valid = 2'b11;

        $display("-- 3. the data wire's direction is just bus_we -------");
        // Latch a responder so the return side has something selected.
        @(posedge clk); sel <= 4'b0010;          // slave 1
        @(posedge clk); sel <= 4'b0000;
        s_dstream = 4'b0000;                     // every slave sending 0
        @(posedge clk); #1;

        gnt = 2'b01; m_we = 2'b01; #1;           // master 0, writing
        chk(bus_we      === 1'b1, "write: bus_we high");
        chk(bus_dstream === m_dstream[0],
            "write: the data wire carries the MASTER's bit");

        s_dstream = 4'b0010;                     // slave 1 now sending 1
        #1;
        chk(bus_dstream === m_dstream[0],
            "write: a slave driving cannot disturb it");

        m_we = 2'b00; #1;                        // master 0, reading
        chk(bus_we      === 1'b0, "read: bus_we low");
        chk(bus_dstream === 1'b1,
            "read: the data wire carries the SELECTED SLAVE's bit");
        s_dstream = 4'b0000; #1;
        chk(bus_dstream === 1'b0, "read: it follows that slave");

        $display("-- 4. the return select is LATCHED, not delayed ------");
        s_ready     = 4'b0000;
        s_resp_flat = {`RESP_ERROR, `RESP_OKAY, `RESP_OKAY, `RESP_SPLIT};

        // Select slave 2 with a ONE-CYCLE pulse, then leave the bus alone
        // for a long time - as a real read does while the slave shifts data.
        @(posedge clk); sel <= 4'b0100;
        @(posedge clk); sel <= 4'b0000;
        #1;
        chk(sel_q === 4'b0100, "captured on the one-cycle pulse");

        for (k = 0; k < 12; k = k + 1) begin
            @(posedge clk); #1;
            if (sel_q !== 4'b0100) begin
                $display("  ERROR sel_q did not hold (k=%0d, sel_q=%b)", k, sel_q);
                errors = errors + 1;
            end
        end
        $display("  ok    held for 12 cycles with no further pulse");

        // ...and the reply, arriving late, still comes from slave 2.
        s_dstream = 4'b0100;
        s_ready   = 4'b0100;
        #1;
        chk(bus_ready === 1'b1,       "slave 2's late ready is returned");
        chk(bus_resp  === `RESP_OKAY, "slave 2's response is returned");
        s_ready = 4'b0000;

        $display("-- 5. a new selection replaces it --------------------");
        @(posedge clk); sel <= 4'b0001;          // slave 0, which answers SPLIT
        @(posedge clk); sel <= 4'b0000; #1;
        chk(sel_q === 4'b0001, "sel_q moved to slave 0");
        s_ready = 4'b0001; #1;
        chk(bus_resp === `RESP_SPLIT, "slave 0's SPLIT is returned");
        s_ready = 4'b0000;

        @(posedge clk); sel <= 4'b1000;          // the default slave
        @(posedge clk); sel <= 4'b0000; #1;
        s_ready = 4'b1000; #1;
        chk(bus_resp === `RESP_ERROR, "the default slave's ERROR is returned");
        s_ready = 4'b0000;

        $display("-- 6. no reply while nothing is answering ------------");
        @(posedge clk); #1;
        chk(bus_ready === 1'b0, "bus_ready low");

        $display("======================================================");
        if (errors == 0) $display(" tb_bus_mux: PASSED (0 errors)");
        else             $display(" tb_bus_mux: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
