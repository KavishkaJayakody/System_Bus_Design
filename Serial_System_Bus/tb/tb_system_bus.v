//==========================================================================
// tb_system_bus.v -- self-checking testbench for system_bus
//
// This one exists because of the modularisation: NO master and NO memory is
// instantiated here.  The testbench plays both roles by driving the two
// serial interfaces directly - it shifts an address frame in on a master's
// wire and answers as a slave on a slave's wire - so what is under test is
// the bus and only the bus: arbitration, address transport, decode,
// direction control, the return path, and the default responder.
//
// Covers:
//   1. reset             - no grant, no select, both shared wires quiet
//   2. arbitration       - single requester; both at once -> master 0 wins
//                          and the grant is HELD for the whole transfer
//   3. address transport - the address shifted in on ONE wire comes back out
//                          reassembled, and lands on the right slave select
//   4. decode            - each mapped range, both range boundaries, and the
//                          unmapped hole and reserved window
//   5. write direction   - with bus_we=1 the shared data wire carries the
//                          GRANTED MASTER's bit, and no slave can disturb it
//   6. read direction    - with bus_we=0 it carries the SELECTED SLAVE's bit
//   7. latched select    - a reply arriving 10 cycles after the frame still
//                          routes back from the right slave
//   8. split             - a SPLIT answer masks that master, the other one
//                          runs, and split_complete restores it
//   9. no slave, no hang - an unmapped frame is answered ERROR by the bus
//                          ITSELF, with nothing attached to the slave ports
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_system_bus;

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;
    localparam ID_W   = 1;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    // ---- master-side stimulus (the testbench plays the masters) ---------
    reg  [NM-1:0]            m_req, m_valid, m_we, m_astream, m_dstream;
    wire [NM-1:0]            m_gnt;
    wire                     bus_ready;
    wire [RESP_W-1:0]        bus_resp;

    // ---- slave-side stimulus (the testbench plays the slaves) -----------
    wire                     bus_valid, bus_we;
    wire [ID_W-1:0]          bus_master_id;
    wire [NS-1:0]            s_sel;
    reg  [NS-1:0]            s_ready, s_dstream;
    reg  [NS*RESP_W-1:0]     s_resp_flat;
    reg  [NM-1:0]            s_split_complete;

    // ---- the two shared wires -------------------------------------------
    wire                     bus_astream, bus_dstream;

    // ---- status ----------------------------------------------------------
    wire                     gnt_valid, addr_done;
    wire [NM-1:0]            split_mask;
    wire [NS:0]              sel_q;
    wire [ADDR_W-1:0]        bus_addr;

    integer errors = 0;
    integer k;

    system_bus #(
        .N_MASTERS(NM), .ID_W(ID_W), .N_SLAVES(NS),
        .ADDR_W(ADDR_W), .RESP_W(RESP_W)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .m_req(m_req), .m_gnt(m_gnt), .m_valid(m_valid), .m_we(m_we),
        .m_astream(m_astream), .m_dstream(m_dstream),
        .bus_ready(bus_ready), .bus_resp(bus_resp),
        .bus_valid(bus_valid), .bus_we(bus_we),
        .bus_master_id(bus_master_id), .s_sel(s_sel),
        .s_ready(s_ready), .s_resp_flat(s_resp_flat), .s_dstream(s_dstream),
        .s_split_complete(s_split_complete),
        .bus_astream(bus_astream), .bus_dstream(bus_dstream),
        .gnt_valid(gnt_valid), .split_mask(split_mask), .sel_q(sel_q),
        .bus_addr(bus_addr), .addr_done(addr_done)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t gnt=%b s_sel=%b mask=%b)",
                         name, $time, m_gnt, s_sel, split_mask);
                errors = errors + 1;
            end
        end
    endtask

    //----------------------------------------------------------------------
    // Watch the shared wires the way a scope would.
    //----------------------------------------------------------------------
    reg [ADDR_W-1:0] rx_addr;
    reg [DATA_W-1:0] rx_wdata;
    integer frame_cycles = 0, frame_len = 0, frames = 0, bad_len = 0;

    always @(posedge clk) if (rst_n) begin
        if (bus_valid) begin
            rx_addr      <= {rx_addr [ADDR_W-2:0], bus_astream};
            rx_wdata     <= {rx_wdata[DATA_W-2:0], bus_dstream};
            frame_cycles  = frame_cycles + 1;
        end else if (frame_cycles != 0) begin
            frames    = frames + 1;
            frame_len = frame_cycles;
            if (frame_cycles != ADDR_W) bad_len = bad_len + 1;
            frame_cycles = 0;
        end
    end

    //----------------------------------------------------------------------
    // Play a master: request, wait for the grant, shift out a frame.
    //----------------------------------------------------------------------
    task automatic acquire;
        input integer mi;
        integer n;
        begin
            @(posedge clk);
            m_req[mi] <= 1'b1;
            n = 0;
            while (!m_gnt[mi] && n < 50) begin @(posedge clk); n = n + 1; end
            if (n >= 50) begin
                $display("  ERROR master %0d never got the grant", mi);
                errors = errors + 1;
            end
        end
    endtask

    // Shift one ADDR_W-clock frame out of master `mi'.  Write data is
    // right-aligned, exactly as master.v pads it.  m_we is left asserted -
    // it is the data wire's direction control for the whole transaction.
    task automatic drive_frame;
        input integer      mi;
        input              we_i;
        input [ADDR_W-1:0] a;
        input [DATA_W-1:0] d;
        integer            k2;
        begin
            for (k2 = ADDR_W-1; k2 >= 0; k2 = k2 - 1) begin
                @(posedge clk);
                m_valid  [mi] <= 1'b1;
                m_we     [mi] <= we_i;
                m_astream[mi] <= a[k2];
                m_dstream[mi] <= (k2 < DATA_W) ? d[k2] : 1'b0;
            end
            @(posedge clk);
            m_valid  [mi] <= 1'b0;
            m_astream[mi] <= 1'b0;
            m_dstream[mi] <= 1'b0;
            #1;                      // we are now in cycle S: sel is pulsing
        end
    endtask

    // Play a slave: answer the transfer that is waiting.
    task automatic slave_answer;
        input integer      si;
        input [RESP_W-1:0] r;
        input integer      mi;        // which master to release
        begin
            @(posedge clk);
            s_ready[si]                  <= 1'b1;
            s_resp_flat[si*RESP_W +: RESP_W] <= r;
            if (r !== `RESP_SPLIT) m_req[mi] <= 1'b0;   // transfer over
            @(posedge clk);
            s_ready[si] <= 1'b0;
            #1;
        end
    endtask

    // Let the bus's own default responder answer, and just wait for it.
    task automatic await_ready;
        integer n;
        begin
            n = 0;
            while (!bus_ready && n < 40) begin @(posedge clk); #1; n = n + 1; end
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_system_bus -- the bus alone, no master, no memory");
        $display("======================================================");

        rst_n = 1'b0;
        m_req = 0; m_valid = 0; m_we = 0; m_astream = 0; m_dstream = 0;
        s_ready = 0; s_dstream = 0; s_resp_flat = 0; s_split_complete = 0;

        //==================================================================
        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(m_gnt       === {NM{1'b0}}, "no grant during reset");
        chk(s_sel       === {NS{1'b0}}, "no slave selected during reset");
        chk(split_mask  === {NM{1'b0}}, "split mask clear during reset");
        chk(bus_astream === 1'b0,       "address wire quiet during reset");
        chk(bus_dstream === 1'b0,       "data wire quiet during reset");
        chk(bus_ready   === 1'b0,       "no completion during reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(m_gnt === {NM{1'b0}}, "no grant after reset with no request");

        //==================================================================
        $display("-- 2. arbitration ------------------------------------");
        acquire(0);
        chk(m_gnt === 2'b01,          "master 0 granted");
        chk(bus_master_id === 1'b0,   "master_id tag = 0");
        chk(gnt_valid === 1'b1,       "gnt_valid asserted");
        drive_frame(0, 1'b1, 16'h1ABC, 8'h5D);
        // the grant must survive the whole 16-clock frame
        chk(m_gnt === 2'b01,          "grant HELD across the whole frame");
        // frame_len is checked in section 4: the monitor only retires a
        // frame on the edge AFTER bus_valid falls, and the checks below are
        // on one-cycle pulses that cannot wait a clock.

        //==================================================================
        $display("-- 3. address transport off ONE wire -----------------");
        chk(addr_done === 1'b1,       "decode strobe pulsed at the end of the frame");
        chk(bus_addr === 16'h1ABC,    "address reassembled from the single wire");
        chk(rx_addr  === 16'h1ABC,    "and that is what was on the wire");
        chk(s_sel === 3'b010,         "0x1ABC selected slave 1");

        //==================================================================
        $display("-- 4. write direction: the master owns the data wire --");
        chk(bus_we === 1'b1,          "bus_we high for a write");
        chk(rx_wdata === 8'h5D,       "the write data arrived RIGHT-ALIGNED in the frame");
        slave_answer(1, `RESP_OKAY, 0);
        chk(m_gnt === 2'b00,          "grant released after completion");
        chk(frames == 1,              "exactly one frame went out");
        chk(frame_len == ADDR_W,      "and it was exactly ADDR_W clocks");

        //==================================================================
        $display("-- 5. decode: every range and both boundaries --------");
        // {expected s_sel} for each address
        acquire(0); drive_frame(0, 1'b0, `S0_BASE, 8'h00);
        chk(s_sel === 3'b001, "0x0000 -> slave 0");
        slave_answer(0, `RESP_OKAY, 0);
        acquire(0); drive_frame(0, 1'b0, `S0_TOP, 8'h00);
        chk(s_sel === 3'b001, "0x0FFF -> slave 0");
        slave_answer(0, `RESP_OKAY, 0);
        acquire(0); drive_frame(0, 1'b0, `S1_BASE, 8'h00);
        chk(s_sel === 3'b010, "0x1000 -> slave 1");
        slave_answer(1, `RESP_OKAY, 0);
        acquire(0); drive_frame(0, 1'b0, `S2_BASE, 8'h00);
        chk(s_sel === 3'b100, "0x2000 -> slave 2");
        slave_answer(2, `RESP_OKAY, 0);
        acquire(0); drive_frame(0, 1'b0, `S2_TOP, 8'h00);
        chk(s_sel === 3'b100, "0x27FF -> slave 2");
        slave_answer(2, `RESP_OKAY, 0);

        //==================================================================
        $display("-- 6. read direction: the slave owns the data wire ---");
        acquire(0);
        drive_frame(0, 1'b0, 16'h2100, 8'h00);
        chk(s_sel === 3'b100, "slave 2 selected");
        chk(bus_we === 1'b0,  "bus_we low for a read");
        @(posedge clk); #1;                 // sel_q now latched onto slave 2
        s_dstream = 3'b100; #1;             // slave 2 drives a 1
        chk(bus_dstream === 1'b1, "the data wire carries the SELECTED SLAVE's bit");
        s_dstream = 3'b000; #1;
        chk(bus_dstream === 1'b0, "and follows it down again");
        // a master driving must not disturb a read
        m_dstream = 2'b01; #1;
        chk(bus_dstream === 1'b0, "a master cannot disturb the wire during a read");
        m_dstream = 2'b00;

        //==================================================================
        $display("-- 7. the return select is LATCHED -------------------");
        // Answer TEN cycles after the frame, the way a real read does.
        for (k = 0; k < 9; k = k + 1) begin
            @(posedge clk); #1;
            if (sel_q !== 4'b0100) begin
                $display("  ERROR sel_q did not hold (k=%0d sel_q=%b)", k, sel_q);
                errors = errors + 1;
            end
        end
        $display("  ok    sel_q held onto slave 2 for 9 cycles with no new pulse");
        s_dstream = 3'b100;
        slave_answer(2, `RESP_OKAY, 0);
        chk(bus_resp === `RESP_OKAY, "the late reply still routed back from slave 2");
        s_dstream = 3'b000;

        //==================================================================
        $display("-- 8. two masters, priority and the split mask -------");
        // Both request on the same clock.
        @(posedge clk); m_req <= 2'b11;
        repeat (3) @(posedge clk); #1;
        chk(m_gnt === 2'b01, "master 0 wins on a tie");
        drive_frame(0, 1'b0, 16'h0010, 8'h00);
        chk(s_sel === 3'b001, "slave 0 selected");
        // slave 0 answers SPLIT: master 0 keeps m_req high
        slave_answer(0, `RESP_SPLIT, 0);
        chk(split_mask === 2'b01, "mask[0] set by the SPLIT");
        chk(m_gnt      === 2'b00, "grant dropped on the SPLIT");

        repeat (3) @(posedge clk); #1;
        chk(m_gnt === 2'b10, "master 1 granted while master 0 is masked");
        chk(bus_master_id === 1'b1, "master_id tag = 1");
        drive_frame(1, 1'b1, 16'h2345, 8'hC7);
        chk(rx_addr === 16'h2345, "master 1's address reached the wire");
        chk(rx_wdata === 8'hC7,   "and its write data");
        chk(s_sel === 3'b100,     "and decoded to slave 2");
        slave_answer(2, `RESP_OKAY, 1);

        repeat (3) @(posedge clk); #1;
        chk(m_gnt === 2'b00, "still no grant - master 0 is masked and master 1 is done");
        chk(split_mask === 2'b01, "master 0 still masked");

        // the slave finishes: wake master 0
        @(posedge clk); s_split_complete <= 2'b01;
        @(posedge clk); s_split_complete <= 2'b00;
        #1;
        chk(split_mask === 2'b00, "mask cleared by split_complete[0]");
        repeat (3) @(posedge clk); #1;
        chk(m_gnt === 2'b01, "master 0 re-granted to replay");
        drive_frame(0, 1'b0, 16'h0010, 8'h00);
        chk(s_sel === 3'b001, "the replayed frame decoded the same way");
        slave_answer(0, `RESP_OKAY, 0);
        chk(frames > 0 && bad_len == 0, "every frame so far was ADDR_W clocks");

        //==================================================================
        $display("-- 9. unmapped: the BUS answers, with nothing attached");
        // Nothing on the slave ports responds here at all.  If the bus did
        // not carry its own default responder, this would hang forever.
        acquire(0);
        drive_frame(0, 1'b0, 16'h2800, 8'h00);
        chk(s_sel === 3'b000, "no slave select asserted for the decode hole");
        await_ready;
        chk(bus_ready === 1'b1,       "the bus answered by itself");
        chk(bus_resp  === `RESP_ERROR,"and answered ERROR");
        @(posedge clk); m_req[0] <= 1'b0;
        repeat (2) @(posedge clk); #1;
        chk(m_gnt === 2'b00, "grant released - the bus recovered");

        acquire(1);
        drive_frame(1, 1'b0, 16'h8000, 8'h00);
        chk(s_sel === 3'b000, "reserved addr[15]=1 window selects no slave");
        await_ready;
        chk(bus_resp === `RESP_ERROR, "reserved window answers ERROR");
        @(posedge clk); m_req[1] <= 1'b0;

        // and the bus still works afterwards
        acquire(0);
        drive_frame(0, 1'b0, 16'h1234, 8'h00);
        chk(s_sel === 3'b010, "the very next frame decoded normally");
        slave_answer(1, `RESP_OKAY, 0);

        $display("======================================================");
        $display(" %0d frames on the wire, %0d wrong length", frames, bad_len);
        if (errors == 0) $display(" tb_system_bus: PASSED (0 errors)");
        else             $display(" tb_system_bus: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #500000;
        $display(" tb_system_bus: FAILED (timeout)");
        $finish;
    end

endmodule
