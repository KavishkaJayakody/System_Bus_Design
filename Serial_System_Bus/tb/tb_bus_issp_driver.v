//==========================================================================
// tb_bus_issp_driver.v -- self-checking testbench for bus_issp_driver
//
// The driver is wired to a REAL bus_top, and the testbench drives the ISSP
// source register and reads the probe register through hierarchical
// references - which is exactly what issp_bus_lib.tcl does over JTAG.  The
// helper tasks below mirror that library one for one (arm / flush / await /
// bus_cmd / soft_reset), so a sequence that passes here is a sequence that
// works on the board.
//
// Covers:
//   1. reset             - no command issued, probes clean
//   2. write + read back - through the ISSP command path, data intact
//   3. latency           - a read costs more than a write, and both are far
//                          below the 0xFF saturation that means "hung"
//   4. re-arm            - dropping `go' clears the sticky done so the next
//                          command can be launched
//   5. unmapped address  - ERROR reported and the transaction COMPLETES;
//                          this is the property the console relies on
//   6. split             - with s0_split_en the split count rises and the
//                          transaction still finishes
//   7. frame watchdog    - frame_len reads ADDR_W and frame_bad stays clear
//   8. bus_addr probe    - the address reassembled off the serial wire
//   9. master 1          - the second master works through the same map
//  10. collision         - both masters in flight sets the sticky flag
//  11. soft_rst          - clears the sticky flags
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_bus_issp_driver;

    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    reg clk = 1'b0;
    reg rst_n;
    always #10 clk = ~clk;

    integer errors = 0;

    // ---- driver <-> bus wiring -------------------------------------------
    wire [1:0]          cmd_valid, cmd_we, cmd_accept, done, err, mst_busy;
    wire [2*ADDR_W-1:0] cmd_addr_flat;
    wire [2*DATA_W-1:0] cmd_wdata_flat, rdata_flat;
    wire [2*RESP_W-1:0] resp_flat;
    wire [15:0]         split_count_flat;
    wire [1:0]          gnt, split_mask;
    wire [3:0]          sel_q;
    wire                gnt_valid, bus_valid, bus_astream, bus_dstream;
    wire                addr_done, bus_ready, s0_busy;
    wire [ADDR_W-1:0]   bus_addr;
    wire [RESP_W-1:0]   bus_resp;
    wire                master_id;
    wire                issp_mode, s0_split_en;

    bus_issp_driver #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr_flat(cmd_addr_flat), .cmd_wdata_flat(cmd_wdata_flat),
        .cmd_accept(cmd_accept), .done(done),
        .rdata_flat(rdata_flat), .resp_flat(resp_flat), .err(err),
        .split_count_flat(split_count_flat),
        .gnt(gnt), .split_mask(split_mask), .sel_q(sel_q),
        .s0_busy(s0_busy), .bus_addr(bus_addr), .bus_valid(bus_valid),
        .issp_mode(issp_mode), .s0_split_en(s0_split_en)
    );

    bus_top #(
        .N_MASTERS(2), .ID_W(1), .N_SLAVES(3),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(6)
    ) u_bus (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr_flat(cmd_addr_flat), .cmd_wdata_flat(cmd_wdata_flat),
        .cmd_accept(cmd_accept), .done(done),
        .rdata_flat(rdata_flat), .resp_flat(resp_flat), .err(err),
        .split_count_flat(split_count_flat), .mst_busy(mst_busy),
        .s0_split_en(s0_split_en),
        .gnt(gnt), .gnt_valid(gnt_valid), .master_id(master_id),
        .split_mask(split_mask), .sel_q(sel_q),
        .bus_valid(bus_valid), .bus_astream(bus_astream),
        .bus_dstream(bus_dstream), .bus_addr(bus_addr), .addr_done(addr_done),
        .bus_ready(bus_ready), .bus_resp(bus_resp), .s0_busy(s0_busy)
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

    //======================================================================
    // The Tcl library, in Verilog.  SRC is the shadow copy of the 56-bit
    // source register; src_flush pushes it into the ISSP instance, exactly
    // as write_source_data does over JTAG.
    //======================================================================
    reg [55:0] SRC;
    reg [95:0] PRB;

    task src_flush; begin dut.u_issp.source = SRC; end endtask
    task probe;     begin PRB = dut.u_issp.probe; end endtask

    // Last result of a command.
    reg  [7:0]  r_rdata, r_lat, r_splits;
    reg  [1:0]  r_resp;
    reg         r_err, r_ok;

    task automatic await_done;
        input integer m;
        integer n;
        begin
            r_ok = 1'b0;
            for (n = 0; n < 3000 && !r_ok; n = n + 1) begin
                @(posedge clk); #1;
                probe;
                if (PRB[m*30 + 8]) begin
                    r_ok     = 1'b1;
                    r_rdata  = PRB[m*30      +: 8];
                    r_resp   = PRB[m*30 + 10 +: 2];
                    r_err    = PRB[m*30 + 12];
                    r_lat    = PRB[m*30 + 13 +: 8];
                    r_splits = PRB[m*30 + 21 +: 8];
                end
            end
            if (!r_ok) begin
                probe;
                r_lat = PRB[m*30 + 13 +: 8];
            end
        end
    endtask

    // One transaction, the same five steps issp_bus_lib.tcl performs.
    task automatic bus_cmd;
        input integer      m;
        input              we_i;
        input [ADDR_W-1:0] a;
        input [DATA_W-1:0] d;
        begin
            SRC[m*26 +: 26] = {d, a, we_i, 1'b0};   // arm, go low
            src_flush;
            @(posedge clk);
            SRC[m*26] = 1'b1;                       // rising edge fires it
            src_flush;
            await_done(m);
            SRC[m*26] = 1'b0;                       // release, re-arm
            src_flush;
            @(posedge clk);
            #1;   // settle: return AFTER the edge so probes read new values
        end
    endtask

    task soft_reset;
        begin
            SRC[52] = 1'b1; src_flush; @(posedge clk);
            SRC[52] = 1'b0; src_flush; @(posedge clk);
            #1;
        end
    endtask

    integer wr_lat, rd_lat;

    initial begin
        $display("======================================================");
        $display(" tb_bus_issp_driver -- the JTAG debug path");
        $display("======================================================");

        rst_n = 1'b0;
        SRC   = 56'd0;
        src_flush;

        //==================================================================
        $display("-- 1. reset ------------------------------------------");
        repeat (4) @(posedge clk); #1;
        probe;
        chk(cmd_valid === 2'b00, "no command issued out of reset");
        chk(PRB[8]  === 1'b0,    "m0 done sticky clear");
        chk(PRB[9]  === 1'b0,    "m0 not busy");
        chk(PRB[69] === 1'b0,    "collision clear");
        chk(PRB[91] === 1'b0,    "frame_bad clear");
        chk(issp_mode   === 1'b0, "issp_mode low until the host sets it");
        chk(s0_split_en === 1'b0, "s0_split_en low until the host sets it");
        @(posedge clk); rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // Take the command ports, as bus_connect does.
        SRC[53] = 1'b1; src_flush; @(posedge clk); #1;
        chk(issp_mode === 1'b1, "host took the command ports");

        //==================================================================
        $display("-- 2. write and read back on master 0 ----------------");
        bus_cmd(0, 1'b1, 16'h1ABC, 8'h5D);
        chk(r_ok,                   "write completed");
        chk(r_resp === `RESP_OKAY,  "write resp = OKAY");
        chk(r_err  === 1'b0,        "no error flagged");
        wr_lat = r_lat;

        bus_cmd(0, 1'b0, 16'h1ABC, 8'h00);
        chk(r_ok,                   "read completed");
        chk(r_rdata === 8'h5D,      "read data came back through the ISSP path");
        chk(r_resp  === `RESP_OKAY, "read resp = OKAY");
        rd_lat = r_lat;

        //==================================================================
        $display("-- 3. latency ----------------------------------------");
        $display("  ..    write %0d clks, read %0d clks (as seen from JTAG)",
                 wr_lat, rd_lat);
        chk(rd_lat > wr_lat,   "a read costs more than a write - the data phase");
        chk(rd_lat < 8'hFF,    "nowhere near the 0xFF saturation that means hung");
        chk(wr_lat > 8'd15,    "and both are longer than the 16-clock frame");

        //==================================================================
        $display("-- 4. re-arm between commands ------------------------");
        // done_s must clear when go drops, or the next await returns instantly
        // on the previous command's result.
        probe;
        chk(PRB[8] === 1'b0, "done sticky cleared after go was released");
        bus_cmd(0, 1'b1, 16'h1ABC, 8'hA3);
        bus_cmd(0, 1'b0, 16'h1ABC, 8'h00);
        chk(r_rdata === 8'hA3, "a second command really ran, not a stale result");

        //==================================================================
        $display("-- 5. unmapped address: ERROR, and it COMPLETES ------");
        bus_cmd(0, 1'b0, 16'h2800, 8'h00);
        chk(r_ok,                    "the decode hole completed instead of hanging");
        chk(r_resp === `RESP_ERROR,  "resp = ERROR");
        chk(r_err  === 1'b1,         "sticky error flagged");
        chk(r_lat < 8'hFF,           "latency did not saturate");
        bus_cmd(0, 1'b0, 16'h8000, 8'h00);
        chk(r_resp === `RESP_ERROR,  "reserved addr[15]=1 window answers ERROR too");
        // and the bus is still healthy
        bus_cmd(0, 1'b0, 16'h1ABC, 8'h00);
        chk(r_rdata === 8'hA3,       "the very next command worked - bus recovered");

        //==================================================================
        $display("-- 6. split transaction over JTAG --------------------");
        soft_reset;
        bus_cmd(0, 1'b1, 16'h0A5C, 8'h6D);     // seed with split off
        SRC[54] = 1'b1; src_flush; @(posedge clk); #1;
        chk(s0_split_en === 1'b1, "host enabled slave 0's split");
        bus_cmd(0, 1'b0, 16'h0A5C, 8'h00);
        chk(r_ok,                  "the split read completed");
        chk(r_rdata === 8'h6D,     "and returned the right data after the replay");
        chk(r_splits > 8'd0,       "the split count probe rose");
        $display("  ..    split read took %0d clks vs %0d for a plain read",
                 r_lat, rd_lat);
        chk(r_lat > rd_lat,        "and cost visibly more than a plain read");
        SRC[54] = 1'b0; src_flush; @(posedge clk);

        //==================================================================
        $display("-- 7. serial framing watchdog ------------------------");
        probe;
        chk(PRB[90:86] === 5'd16, "frame_len reads 16 - the address frame is right");
        chk(PRB[91]    === 1'b0,  "frame_bad clear - no frame was ever the wrong length");

        //==================================================================
        $display("-- 8. the reassembled address is visible -------------");
        bus_cmd(0, 1'b0, 16'h2345, 8'h00);
        probe;
        chk(PRB[85:70] === 16'h2345,
            "bus_addr probe shows the address reassembled off the serial wire");

        //==================================================================
        $display("-- 9. master 1 through the same map ------------------");
        bus_cmd(1, 1'b1, 16'h2100, 8'hC7);
        chk(r_ok,               "m1 write completed");
        bus_cmd(1, 1'b0, 16'h2100, 8'h00);
        chk(r_rdata === 8'hC7,  "m1 read data correct");
        chk(r_resp === `RESP_OKAY, "m1 resp = OKAY");

        //==================================================================
        $display("-- 10. collision flag --------------------------------");
        soft_reset;
        probe;
        chk(PRB[69] === 1'b0, "collision clear after soft_rst");
        // Fire both from one source write, as bus_cmd_pair does.
        SRC[0*26 +: 26] = {8'h11, 16'h1200, 1'b1, 1'b0};
        SRC[1*26 +: 26] = {8'h22, 16'h2200, 1'b1, 1'b0};
        src_flush; @(posedge clk);
        SRC[0]  = 1'b1;
        SRC[26] = 1'b1;
        src_flush;
        await_done(0);
        await_done(1);
        probe;
        chk(PRB[69] === 1'b1, "collision set - both masters were in flight together");
        SRC[0] = 1'b0; SRC[26] = 1'b0; src_flush; @(posedge clk);
        bus_cmd(0, 1'b0, 16'h1200, 8'h00);
        chk(r_rdata === 8'h11, "m0's concurrent write landed");
        bus_cmd(1, 1'b0, 16'h2200, 8'h00);
        chk(r_rdata === 8'h22, "m1's concurrent write landed");

        //==================================================================
        $display("-- 11. soft_rst clears the sticky flags --------------");
        bus_cmd(0, 1'b0, 16'h2800, 8'h00);      // set the error flag
        probe;
        chk(PRB[12] === 1'b1, "error sticky set");
        soft_reset;
        probe;
        chk(PRB[12] === 1'b0, "error sticky cleared");
        chk(PRB[69] === 1'b0, "collision cleared");
        chk(PRB[91] === 1'b0, "frame_bad cleared");

        $display("======================================================");
        if (errors == 0) $display(" tb_bus_issp_driver: PASSED (0 errors)");
        else             $display(" tb_bus_issp_driver: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #3000000;
        $display(" tb_bus_issp_driver: FAILED (timeout)");
        $finish;
    end

endmodule
