//==========================================================================
// tb_master.v -- self-checking testbench for master
//
// The master is exercised against a behavioural stand-in for the rest of the
// bus: a grant model that mirrors the arbiter (including the split mask) and
// a slave model that returns a programmed response one cycle after m_valid.
//
// Covers:
//   1. reset behaviour   - idle, no request, no valid
//   2. write transaction - req, grant, ONE cycle of m_valid, done pulse
//   3. read transaction  - rdata captured, resp = OKAY
//   4. m_valid width     - never asserted for more than one cycle per attempt
//                          (a two-cycle valid would look like two accesses)
//   5. ERROR response    - reported through resp/err, NOT retried
//   6. SPLIT + replay    - bus_req stays HIGH through the wait, m_valid drops,
//                          the identical transfer is re-driven when the grant
//                          returns, split_count increments, and the caller
//                          sees exactly ONE done for the whole thing
//   7. write data not clobbering rdata - a write leaves rdata untouched
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_master;

    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    // command interface
    reg                 cmd_valid, cmd_we;
    reg  [ADDR_W-1:0]   cmd_addr;
    reg  [DATA_W-1:0]   cmd_wdata;
    wire                cmd_accept, done, err, busy;
    wire [DATA_W-1:0]   rdata;
    wire [RESP_W-1:0]   resp;
    wire [7:0]          split_count;
    wire [2:0]          state;

    // bus side
    wire                bus_req, m_valid, m_we;
    wire [ADDR_W-1:0]   m_addr;
    wire [DATA_W-1:0]   m_wdata;
    reg                 bus_gnt, bus_ready;
    reg  [RESP_W-1:0]   bus_resp;
    reg  [DATA_W-1:0]   bus_rdata;

    integer errors = 0;

    master #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr(cmd_addr), .cmd_wdata(cmd_wdata),
        .cmd_accept(cmd_accept), .done(done), .rdata(rdata), .resp(resp),
        .err(err), .split_count(split_count), .busy(busy), .state(state),
        .bus_req(bus_req), .bus_gnt(bus_gnt),
        .m_valid(m_valid), .m_we(m_we), .m_addr(m_addr), .m_wdata(m_wdata),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .bus_rdata(bus_rdata)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t state=%0d req=%b gnt=%b)",
                         name, $time, state, bus_req, bus_gnt);
                errors = errors + 1;
            end
        end
    endtask

    //----------------------------------------------------------------------
    // Bus model
    //----------------------------------------------------------------------
    reg        split_mask;      // mirrors the arbiter's mask for this master
    reg        arm_split;       // next access will be answered with SPLIT
    reg [RESP_W-1:0] next_resp; // response for a non-split access
    reg [DATA_W-1:0] next_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_gnt    <= 1'b0;
            bus_ready  <= 1'b0;
            bus_resp   <= `RESP_OKAY;
            bus_rdata  <= {DATA_W{1'b0}};
            split_mask <= 1'b0;
        end else begin
            // grant: one cycle after an unmasked request, dropped once the
            // transfer completes - same shape as the real arbiter
            if (bus_ready)                   bus_gnt <= 1'b0;
            else if (bus_req && !split_mask) bus_gnt <= 1'b1;
            else                             bus_gnt <= 1'b0;

            // slave: replies one cycle after the access strobe
            bus_ready <= m_valid;
            if (m_valid) begin
                bus_resp  <= arm_split ? `RESP_SPLIT : next_resp;
                bus_rdata <= next_rdata;
            end

            // mask this master when it is split, exactly like the arbiter
            if (bus_ready && bus_resp == `RESP_SPLIT) split_mask <= 1'b1;
        end
    end

    // Track how long m_valid stays high and how many times it is driven.
    integer valid_cycles = 0;
    integer valid_bursts = 0;
    reg     m_valid_d = 1'b0;
    integer done_pulses = 0;
    always @(posedge clk) if (rst_n) begin
        m_valid_d <= m_valid;
        if (m_valid) valid_cycles = valid_cycles + 1;
        if (m_valid && !m_valid_d) valid_bursts = valid_bursts + 1;
        if (done) done_pulses = done_pulses + 1;
    end

    // Issue one command and wait for done.
    task run_cmd;
        input             we_i;
        input [ADDR_W-1:0] a;
        input [DATA_W-1:0] d;
        begin
            @(posedge clk);
            cmd_valid <= 1'b1; cmd_we <= we_i; cmd_addr <= a; cmd_wdata <= d;
            @(posedge clk);
            while (!cmd_accept) @(posedge clk);
            cmd_valid <= 1'b0;
            @(posedge clk);
            while (!done) @(posedge clk);
            #1;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_master");
        $display("======================================================");

        rst_n     = 1'b0;
        cmd_valid = 0; cmd_we = 0; cmd_addr = 0; cmd_wdata = 0;
        arm_split = 0; next_resp = `RESP_OKAY; next_rdata = 32'h0;

        $display("-- 1. reset behaviour --------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(bus_req === 1'b0, "no bus request in reset");
        chk(m_valid === 1'b0, "no transfer strobe in reset");
        chk(busy    === 1'b0, "not busy in reset");
        chk(state   === 3'd0, "FSM in IDLE");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(bus_req === 1'b0, "still idle with no command");

        $display("-- 2. write transaction ------------------------------");
        valid_cycles = 0; valid_bursts = 0; done_pulses = 0;
        next_resp  = `RESP_OKAY;
        next_rdata = 32'hFFFF_FFFF;      // a write must NOT capture this
        run_cmd(1'b1, 16'h1234, 32'hA5A5_A5A5);
        chk(done_pulses == 1,        "exactly one done pulse");
        chk(valid_bursts == 1,       "the transfer was driven exactly once");
        chk(valid_cycles == 1,       "m_valid was high for exactly one cycle");
        chk(resp === `RESP_OKAY,     "resp = OKAY");
        chk(err  === 1'b0,           "err low");
        chk(rdata === 32'h0000_0000, "a write did not overwrite rdata");
        chk(split_count === 8'd0,    "no splits counted");

        $display("-- 3. read transaction -------------------------------");
        valid_cycles = 0; valid_bursts = 0; done_pulses = 0;
        next_rdata = 32'h1234_5678;
        run_cmd(1'b0, 16'h1234, 32'h0);
        chk(done_pulses == 1,          "exactly one done pulse");
        chk(valid_cycles == 1,         "m_valid one cycle");
        chk(rdata === 32'h1234_5678,   "read data captured");
        chk(resp  === `RESP_OKAY,      "resp = OKAY");

        $display("-- 4. ERROR is reported, not retried -----------------");
        valid_cycles = 0; valid_bursts = 0; done_pulses = 0;
        next_resp = `RESP_ERROR;
        run_cmd(1'b0, 16'h2900, 32'h0);          // unmapped-looking address
        chk(done_pulses == 1,      "one done pulse");
        chk(valid_bursts == 1,     "the transfer was NOT retried");
        chk(resp === `RESP_ERROR,  "resp = ERROR");
        chk(err  === 1'b1,         "err asserted");
        chk(busy === 1'b0,         "master returned to IDLE, bus not held");
        next_resp = `RESP_OKAY;

        $display("-- 5. SPLIT and replay -------------------------------");
        valid_cycles = 0; valid_bursts = 0; done_pulses = 0;
        arm_split  = 1'b1;
        next_rdata = 32'hCAFE_F00D;

        // Kick off the command, then watch the split happen.
        @(posedge clk);
        cmd_valid <= 1'b1; cmd_we <= 1'b0;
        cmd_addr  <= 16'h0010; cmd_wdata <= 32'h0;
        @(posedge clk);
        while (!cmd_accept) @(posedge clk);
        cmd_valid <= 1'b0;

        // Wait for the master to register the SPLIT.
        while (split_count == 8'd0) @(posedge clk);
        #1;
        chk(split_count === 8'd1, "split counted");
        chk(state === 3'd4,       "master parked in SPLIT_WAIT");
        chk(bus_req === 1'b1,     "bus_req HELD high while split-deferred");
        chk(done_pulses == 0,     "no done reported for the split itself");
        arm_split = 1'b0;

        // Stay masked for a while; the master must sit still.
        repeat (8) begin
            @(posedge clk); #1;
            if (m_valid !== 1'b0) begin
                $display("  ERROR master drove a transfer while masked");
                errors = errors + 1;
            end
            if (bus_req !== 1'b1) begin
                $display("  ERROR master dropped bus_req while split-deferred");
                errors = errors + 1;
            end
        end
        $display("  ok    quiet but still requesting for 8 cycles");
        chk(valid_bursts == 1, "still only the original attempt so far");

        // Slave finishes: unmask, the master must replay.
        @(posedge clk); split_mask <= 1'b0;
        while (!done) @(posedge clk);
        #1;
        chk(done_pulses == 1,        "exactly ONE done for the whole split+replay");
        chk(valid_bursts == 2,       "the transfer was driven twice: attempt + replay");
        chk(valid_cycles == 2,       "one cycle of m_valid per attempt");
        chk(m_addr === 16'h0010,     "the replay used the SAME address");
        chk(m_we  === 1'b0,          "the replay used the same direction");
        chk(rdata === 32'hCAFE_F00D, "replay data captured");
        chk(resp  === `RESP_OKAY,    "final resp = OKAY, never SPLIT");
        chk(split_count === 8'd1,    "split_count still 1");

        $display("-- 6. normal transaction after a split ---------------");
        valid_bursts = 0; done_pulses = 0;
        next_rdata = 32'h0BAD_0BAD;
        run_cmd(1'b0, 16'h1000, 32'h0);
        chk(done_pulses == 1,        "one done pulse");
        chk(valid_bursts == 1,       "no leftover replay");
        chk(rdata === 32'h0BAD_0BAD, "data captured");

        $display("======================================================");
        if (errors == 0) $display(" tb_master: PASSED (0 errors)");
        else             $display(" tb_master: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #200000;
        $display(" tb_master: FAILED (timeout)");
        $finish;
    end

endmodule
