//==========================================================================
// tb_master.v -- self-checking testbench for master
//
// The master is exercised against a behavioural stand-in for the rest of the
// serial bus: a grant model that mirrors the arbiter (split mask included)
// and a slave model that answers a write in one cycle and shifts a read back
// bit by bit, on the same schedule slave_mem does.
//
// The testbench also DESERIALISES what the master puts on the wires, so the
// checks are on the traffic itself, not just on the command interface.
//
// Covers:
//   1. reset behaviour   - idle, no request, no frame, wire not driven
//   2. write transaction - the frame is exactly ADDR_W clocks, the address
//                          arrives correctly MSB-first, and the write data
//                          arrives RIGHT-ALIGNED in the frame
//   3. read transaction  - the reply is reassembled from the last DATA_W
//                          bits before ready
//   4. frame discipline  - m_valid is ADDR_W clocks per attempt, never more,
//                          never a second frame per attempt
//   5. read etiquette    - the master does NOT drive the shared data wire
//                          during a read; the slave owns it
//   6. ERROR             - reported, and NOT retried
//   7. SPLIT + replay    - bus_req held high, no frame at all while masked,
//                          then a COMPLETE second frame carrying the
//                          identical address and data, and exactly ONE done
//                          for the whole thing
//   8. writes do not clobber rdata
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

    // serial bus side
    wire                bus_req, m_valid, m_we, m_astream, m_dstream;
    reg                 bus_gnt, bus_ready;
    reg  [RESP_W-1:0]   bus_resp;
    wire                bus_dstream;

    integer errors = 0;

    master #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr(cmd_addr), .cmd_wdata(cmd_wdata),
        .cmd_accept(cmd_accept), .done(done), .rdata(rdata), .resp(resp),
        .err(err), .split_count(split_count), .busy(busy), .state(state),
        .bus_req(bus_req), .bus_gnt(bus_gnt),
        .m_valid(m_valid), .m_we(m_we),
        .m_astream(m_astream), .m_dstream(m_dstream),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .bus_dstream(bus_dstream)
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

    //======================================================================
    // Bus model: arbiter + one slave
    //======================================================================
    reg               split_mask;      // mirrors the arbiter's mask
    reg               arm_split;       // next access is answered with SPLIT
    reg [RESP_W-1:0]  next_resp;
    reg [DATA_W-1:0]  next_rdata;

    reg               frame_d;
    wire              frame_end = frame_d & ~m_valid;   // high during S

    reg               run, tx_drive;
    reg [7:0]         cnt;
    reg [DATA_W-1:0]  tx_sr;

    wire tx_out = tx_drive ? tx_sr[DATA_W-1] : 1'b0;

    // The shared wire: the master owns it on a write, the slave on a read -
    // exactly the direction mux bus_mux implements.
    assign bus_dstream = m_we ? m_dstream : tx_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_gnt    <= 1'b0;
            bus_ready  <= 1'b0;
            bus_resp   <= `RESP_OKAY;
            split_mask <= 1'b0;
            frame_d    <= 1'b0;
            run        <= 1'b0;
            cnt        <= 8'd0;
            tx_sr      <= {DATA_W{1'b0}};
            tx_drive   <= 1'b0;
        end else begin
            frame_d   <= m_valid;
            bus_ready <= 1'b0;

            // grant: held for the whole transaction, dropped on completion
            if (bus_ready)                   bus_gnt <= 1'b0;
            else if (bus_req && !split_mask) bus_gnt <= 1'b1;
            else                             bus_gnt <= 1'b0;

            if (frame_end) begin
                if (m_we || arm_split) begin
                    // a write, or a split, answers at S+1
                    bus_ready <= 1'b1;
                    bus_resp  <= arm_split ? `RESP_SPLIT : next_resp;
                    run       <= 1'b0;
                end else begin
                    // a read: shift the data back, then answer
                    run <= 1'b1;
                    cnt <= 8'd0;
                end
            end else if (run) begin
                cnt <= cnt + 8'd1;
                if (cnt == 8'd0) begin
                    tx_sr    <= next_rdata;
                    tx_drive <= 1'b1;
                end else begin
                    tx_sr <= {tx_sr[DATA_W-2:0], 1'b0};
                    if (cnt == DATA_W) begin
                        tx_drive  <= 1'b0;
                        bus_ready <= 1'b1;
                        bus_resp  <= next_resp;
                        run       <= 1'b0;
                    end
                end
            end

            // mask this master when it is split, exactly like the arbiter
            if (bus_ready && bus_resp == `RESP_SPLIT) split_mask <= 1'b1;
        end
    end

    //======================================================================
    // Receivers: deserialise what the master sent, and police the frame.
    //======================================================================
    reg  [ADDR_W-1:0] rx_addr;
    reg  [DATA_W-1:0] rx_wdata;
    reg               mv_d;
    integer           frame_cycles = 0;   // clocks of the frame in progress
    integer           frame_len    = 0;   // length of the last completed one
    integer           frames       = 0;   // completed frames
    integer           bad_len      = 0;
    integer           done_pulses  = 0;
    integer           drove_on_read = 0;

    always @(posedge clk) if (rst_n) begin
        mv_d <= m_valid;
        if (m_valid) begin
            rx_addr      <= {rx_addr [ADDR_W-2:0], m_astream};
            rx_wdata     <= {rx_wdata[DATA_W-2:0], m_dstream};
            frame_cycles  = frame_cycles + 1;
            // On a read the master must leave the data wire alone.
            if (!m_we && m_dstream !== 1'b0) drove_on_read = drove_on_read + 1;
        end else if (frame_cycles != 0) begin
            frames    = frames + 1;
            frame_len = frame_cycles;
            if (frame_cycles != ADDR_W) bad_len = bad_len + 1;
            frame_cycles = 0;
        end
        if (done) done_pulses = done_pulses + 1;
    end

    task automatic run_cmd;
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

    integer k;

    initial begin
        $display("======================================================");
        $display(" tb_master");
        $display("======================================================");

        rst_n     = 1'b0;
        cmd_valid = 0; cmd_we = 0; cmd_addr = 0; cmd_wdata = 0;
        arm_split = 0; next_resp = `RESP_OKAY; next_rdata = 8'h00;

        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(bus_req   === 1'b0, "no bus request in reset");
        chk(m_valid   === 1'b0, "no frame in reset");
        chk(m_astream === 1'b0, "address wire not driven in reset");
        chk(busy      === 1'b0, "not busy in reset");
        chk(state     === 3'd0, "FSM in IDLE");
        @(posedge clk); rst_n = 1'b1;
        repeat (3) @(posedge clk); #1;
        chk(bus_req === 1'b0, "still idle with no command");

        $display("-- 2. write transaction ------------------------------");
        frames = 0; bad_len = 0; done_pulses = 0;
        next_resp  = `RESP_OKAY;
        next_rdata = 8'hFF;                  // a write must NOT capture this
        run_cmd(1'b1, 16'h1ABC, 8'h5D);
        chk(done_pulses == 1,      "exactly one done pulse");
        chk(frames == 1,           "the transfer put exactly ONE frame on the bus");
        chk(frame_len == ADDR_W,   "the frame was exactly ADDR_W clocks");
        chk(rx_addr === 16'h1ABC,  "the address arrived correctly, MSB first");
        chk(rx_wdata === 8'h5D,
            "the write data arrived RIGHT-ALIGNED in the frame");
        chk(resp === `RESP_OKAY,   "resp = OKAY");
        chk(err  === 1'b0,         "err low");
        chk(rdata === 8'h00,       "a write did not overwrite rdata");
        chk(split_count === 8'd0,  "no splits counted");

        $display("-- 3. read transaction -------------------------------");
        frames = 0; bad_len = 0; done_pulses = 0; drove_on_read = 0;
        next_rdata = 8'h9C;
        run_cmd(1'b0, 16'h1ABC, 8'h00);
        chk(done_pulses == 1,     "exactly one done pulse");
        chk(frames == 1,          "one frame");
        chk(rx_addr === 16'h1ABC, "the address arrived correctly");
        chk(rdata === 8'h9C,
            "read data reassembled from the last DATA_W bits before ready");
        chk(resp === `RESP_OKAY,  "resp = OKAY");
        chk(drove_on_read == 0,
            "the master left the shared data wire alone during the read");

        // a few more values, to be sure it is not one lucky pattern
        next_rdata = 8'h00; run_cmd(1'b0, 16'h1000, 8'h00);
        chk(rdata === 8'h00, "read 0x00");
        next_rdata = 8'hFF; run_cmd(1'b0, 16'h1000, 8'h00);
        chk(rdata === 8'hFF, "read 0xFF");
        next_rdata = 8'h80; run_cmd(1'b0, 16'h1000, 8'h00);
        chk(rdata === 8'h80, "read 0x80 (MSB only - catches a bit-order slip)");
        next_rdata = 8'h01; run_cmd(1'b0, 16'h1000, 8'h00);
        chk(rdata === 8'h01, "read 0x01 (LSB only)");

        $display("-- 4. address boundary values ------------------------");
        run_cmd(1'b1, 16'h0000, 8'h11);
        chk(rx_addr === 16'h0000, "address 0x0000 sent correctly");
        run_cmd(1'b1, 16'hFFFF, 8'h22);
        chk(rx_addr === 16'hFFFF, "address 0xFFFF sent correctly");
        run_cmd(1'b1, 16'h8000, 8'h33);
        chk(rx_addr === 16'h8000, "address 0x8000 sent correctly");
        chk(bad_len == 0,         "every frame so far was ADDR_W clocks");

        $display("-- 5. ERROR is reported, not retried -----------------");
        frames = 0; done_pulses = 0;
        next_resp = `RESP_ERROR;
        run_cmd(1'b0, 16'h2900, 8'h00);
        chk(done_pulses == 1,     "one done pulse");
        chk(frames == 1,          "the transfer was NOT retried");
        chk(resp === `RESP_ERROR, "resp = ERROR");
        chk(err  === 1'b1,        "err asserted");
        chk(busy === 1'b0,        "master returned to IDLE, bus not held");
        next_resp = `RESP_OKAY;

        $display("-- 6. SPLIT and replay -------------------------------");
        frames = 0; bad_len = 0; done_pulses = 0;
        arm_split  = 1'b1;
        next_rdata = 8'h6E;

        @(posedge clk);
        cmd_valid <= 1'b1; cmd_we <= 1'b0;
        cmd_addr  <= 16'h0A5C; cmd_wdata <= 8'h00;
        @(posedge clk);
        while (!cmd_accept) @(posedge clk);
        cmd_valid <= 1'b0;

        while (split_count == 8'd0) @(posedge clk);
        #1;
        chk(split_count === 8'd1, "split counted");
        chk(state === 3'd4,       "master parked in SPLIT_WAIT");
        chk(bus_req === 1'b1,     "bus_req HELD high while split-deferred");
        chk(done_pulses == 0,     "no done reported for the split itself");
        chk(frames == 1,          "one frame sent so far");
        arm_split = 1'b0;

        // Stay masked: the master must not put anything on the wires.
        for (k = 0; k < 25; k = k + 1) begin
            @(posedge clk); #1;
            if (m_valid !== 1'b0) begin
                $display("  ERROR master started a frame while masked");
                errors = errors + 1;
            end
            if (bus_req !== 1'b1) begin
                $display("  ERROR master dropped bus_req while split-deferred");
                errors = errors + 1;
            end
        end
        $display("  ok    silent but still requesting for 25 cycles");
        chk(frames == 1, "still only the original frame");

        // The slave finishes: unmask, and the master must re-send.
        @(posedge clk); split_mask <= 1'b0;
        while (!done) @(posedge clk);
        #1;
        chk(done_pulses == 1,     "exactly ONE done for the whole split + replay");
        chk(frames == 2,          "the transfer was sent TWICE: attempt + replay");
        chk(bad_len == 0,         "the replay was a FULL ADDR_W frame, not a resume");
        chk(rx_addr === 16'h0A5C, "the replay re-sent the SAME address");
        chk(rdata === 8'h6E,      "replay data captured");
        chk(resp === `RESP_OKAY,  "final resp = OKAY, never SPLIT");
        chk(split_count === 8'd1, "split_count still 1");

        $display("-- 7. a split WRITE replays its data too -------------");
        frames = 0; done_pulses = 0;
        arm_split = 1'b1;
        @(posedge clk);
        cmd_valid <= 1'b1; cmd_we <= 1'b1;
        cmd_addr  <= 16'h0369; cmd_wdata <= 8'hC7;
        @(posedge clk);
        while (!cmd_accept) @(posedge clk);
        cmd_valid <= 1'b0;
        while (state !== 3'd4) @(posedge clk);
        arm_split = 1'b0;
        @(posedge clk); split_mask <= 1'b0;
        while (!done) @(posedge clk); #1;
        chk(frames == 2,          "two frames");
        chk(rx_addr === 16'h0369, "the replay re-sent the same address");
        chk(rx_wdata === 8'hC7,   "and the same write data");
        chk(done_pulses == 1,     "one done");

        $display("-- 8. normal transaction after a split ---------------");
        frames = 0; done_pulses = 0;
        next_rdata = 8'h4B;
        run_cmd(1'b0, 16'h1000, 8'h00);
        chk(done_pulses == 1, "one done pulse");
        chk(frames == 1,      "no leftover replay");
        chk(rdata === 8'h4B,  "data captured");

        $display("======================================================");
        if (errors == 0) $display(" tb_master: PASSED (0 errors)");
        else             $display(" tb_master: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #500000;
        $display(" tb_master: FAILED (timeout)");
        $finish;
    end

endmodule
