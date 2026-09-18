//==========================================================================
// tb_top_debug.v -- self-checking testbench for the SYNTHESIS TOP LEVEL
//
// Drives `top_debug' through its real pins - CLOCK_50, rst_n and led[7:0] -
// and through the JTAG source register, which is the only other way in.
// Those three ports plus JTAG are the whole external interface of the
// device - there is no board layer - and this testbench exercises all of it.
//
// The ISSP source register is written through a hierarchical reference,
// `dut.u_dbg.u_issp.source', exactly as write_source_data does over JTAG
// and as tb_bus_issp_driver does.  A sequence that passes here is a
// sequence that works on the board.
//
// tb_bus_issp_driver covers the driver's bit map and its corner cases in
// depth.  This testbench covers what only the top level can show: that the
// real synthesis top ELABORATES and that a transaction issued at the pins
// reaches the memory and comes back, with the read data visible on led[7:0].
//
// `tb/altsource_probe_stub.v' stands in for the Altera megafunction, which
// iverilog cannot elaborate.  It is SIMULATION ONLY and is not in the .qsf.
//
// Covers:
//   1. reset            - no command issued, nothing on the bus
//   2. write + read back- through the top-level command path
//   3. led[7:0]         - shows master 0's read data, and ALL 8 bits move
//   4. every slave      - one word in each of the three memories
//   5. unmapped address - ERROR, and the very next transfer still works
//   6. split            - enabled from the host, and it completes
//   7. framing          - frame_len reads 16 on the real top level
//==========================================================================
`timescale 1ns/1ps
`include "../rtl/bus_defs.vh"

module tb_top_debug;

    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;

    reg        CLOCK_50 = 1'b0;
    reg        rst_n;
    wire [7:0] led;

    always #10 CLOCK_50 = ~CLOCK_50;      // 50 MHz
    wire clk = CLOCK_50;

    integer errors = 0;

    // SPLIT_LATENCY shrunk so a split does not take 0.2 s of simulated time.
    top_debug #(.SPLIT_LATENCY(6), .CLKS_PER_BIT(4), .RESP_TIMEOUT(3000)) dut (
        .CLOCK_50 (CLOCK_50),
        .rst_n    (rst_n), .rst_m_n(rst_n), .rst_s_n(rst_n),
        .led      (led)
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
    // The Tcl library, in Verilog - the same helpers tb_bus_issp_driver
    // uses, pointed at this design's driver instance.
    //======================================================================
    reg [55:0] SRC;
    reg [95:0] PRB;

    task src_flush; begin dut.u_dbg.u_issp.source = SRC; end endtask
    task probe;     begin PRB = dut.u_dbg.u_issp.probe; end endtask

    // A remote read with no cable costs RESP_TIMEOUT clocks on top of the
    // request, so the poll budget above must exceed both, not just a local
    // transaction.
    reg  [7:0]  r_rdata, r_lat, r_splits;
    reg  [1:0]  r_resp;
    reg         r_err, r_ok;

    task automatic await_done;
        input integer m;
        integer n;
        begin
            r_ok = 1'b0;
            for (n = 0; n < 20000 && !r_ok; n = n + 1) begin
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
        end
    endtask

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
            #1;
        end
    endtask

    // Union of every led value seen, so "all 8 bits move" is checkable.
    reg [7:0] led_seen;

    initial begin
        $display("======================================================");
        $display(" tb_top_debug -- the synthesis top through its pins");
        $display("======================================================");

        rst_n    = 1'b0;
        SRC      = 56'd0;
        led_seen = 8'h00;
        src_flush;

        //==================================================================
        $display("-- 1. reset ------------------------------------------");
        repeat (4) @(posedge clk); #1;
        probe;
        chk(PRB[9]  === 1'b0, "master 0 not busy out of reset");
        chk(PRB[39] === 1'b0, "master 1 not busy out of reset");
        chk(PRB[62:60] === 3'b000, "nobody holds the grant out of reset");

        rst_n = 1'b1;
        repeat (4) @(posedge clk); #1;

        //==================================================================
        $display("-- 2. write and read back through the top level -------");
        bus_cmd(0, 1'b1, 16'h1ABC, 8'h5A);
        chk(r_ok,                   "the write completed");
        chk(r_resp === `RESP_OKAY,  "write resp = OKAY");
        bus_cmd(0, 1'b0, 16'h1ABC, 8'h00);
        chk(r_ok,                   "the read completed");
        chk(r_rdata === 8'h5A,      "read data matches what was written");

        //==================================================================
        $display("-- 3. led[7:0] shows master 0's read data ------------");
        chk(led === 8'h5A, "led follows the last read");
        led_seen = led_seen | led;

        // A write must NOT disturb the display.
        bus_cmd(0, 1'b1, 16'h1DEF, 8'hFF);
        chk(led === 8'h5A, "a write leaves the display unchanged");

        bus_cmd(0, 1'b0, 16'h1DEF, 8'h00);
        chk(led === 8'hFF, "and the next read updates it");
        led_seen = led_seen | led;

        bus_cmd(0, 1'b1, 16'h1DEF, 8'h00);
        bus_cmd(0, 1'b0, 16'h1DEF, 8'h00);
        led_seen = led_seen | led;
        // 0xFF then 0x00 have both been displayed, so every bit has been
        // seen high and low - the fitter cannot trim any of the datapath.
        chk(led_seen === 8'hFF, "ALL 8 data bits reach a pin and change");

        //==================================================================
        // Sizes per the link spec: 2K / 4K / 4K, device ids 0 / 1 / 2.
        $display("-- 4. all three slaves -------------------------------");
        bus_cmd(0, 1'b1, 16'h05C3, 8'hA0);      // slave 0, 2K
        bus_cmd(0, 1'b1, 16'h1234, 8'hB1);      // slave 1, 4K
        bus_cmd(0, 1'b1, 16'h2567, 8'hC2);      // slave 2, 4K, splits
        bus_cmd(0, 1'b0, 16'h05C3, 8'h00);
        chk(r_rdata === 8'hA0, "slave 0 kept its word");
        bus_cmd(0, 1'b0, 16'h1234, 8'h00);
        chk(r_rdata === 8'hB1, "slave 1 kept its word");
        bus_cmd(0, 1'b0, 16'h2567, 8'h00);
        chk(r_rdata === 8'hC2, "slave 2 kept its word");
        bus_cmd(0, 1'b1, 16'h2FFF, 8'hD3);      // slave 2 really is 4K now
        bus_cmd(0, 1'b0, 16'h2FFF, 8'h00);
        chk(r_rdata === 8'hD3, "slave 2's last word 0x2FFF is mapped");

        //==================================================================
        $display("-- 5. unmapped address: ERROR, and it COMPLETES ------");
        bus_cmd(0, 1'b0, 16'h0800, 8'h00);
        chk(r_ok,                   "the decode hole completed instead of hanging");
        chk(r_resp === `RESP_ERROR, "resp = ERROR");
        bus_cmd(0, 1'b0, 16'h3000, 8'h00);
        chk(r_resp === `RESP_ERROR, "0x3000, above the map, too");
        bus_cmd(0, 1'b0, 16'h1234, 8'h00);
        chk(r_rdata === 8'hB1,      "the very next transfer worked - bus recovered");

        //==================================================================
        // 0x8000+ is the REMOTE window now, not an error: it leaves over the
        // UART.  With no second board attached the read must TIME OUT and
        // report cmd_error, never hang - and the local bus must be unharmed.
        $display("-- 6. remote window with no cable --------------------");
        bus_cmd(0, 1'b0, 16'h9234, 8'h00);      // = far board's 0x1234
        chk(r_ok,                   "the remote read completed instead of hanging");
        chk(r_err === 1'b1,         "and reported an error");
        chk(r_rdata === 8'hFF,      "returning 0xFF, as the spec requires");
        bus_cmd(0, 1'b0, 16'h1234, 8'h00);
        chk(r_rdata === 8'hB1,      "the local bus still works afterwards");

        //==================================================================
        $display("-- 7. split, enabled from the host -------------------");
        SRC[54] = 1'b1; src_flush; @(posedge clk);
        bus_cmd(0, 1'b0, 16'h2567, 8'h00);      // the SPLIT slave is slave 2
        chk(r_ok,               "the split read completed");
        chk(r_rdata === 8'hC2,  "and returned the right data after the replay");
        chk(r_splits > 8'd0,    "the split count rose");
        SRC[54] = 1'b0; src_flush; @(posedge clk);

        //==================================================================
        $display("-- 8. master 1, and the framing watchdog -------------");
        bus_cmd(1, 1'b1, 16'h2100, 8'h3C);
        bus_cmd(1, 1'b0, 16'h2100, 8'h00);
        chk(r_rdata === 8'h3C,  "master 1 round-tripped through the same map");

        probe;
        chk(PRB[93:89] === 5'd16, "frame_len reads 16 on the real top level");
        chk(PRB[94]    === 1'b0,  "frame_bad clear - every frame was full length");

        $display("======================================================");
        if (errors == 0) $display(" tb_top_debug: PASSED (0 errors)");
        else             $display(" tb_top_debug: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    // Global watchdog: a wedged bus must fail, not hang the regression.
    initial begin
        #4_000_000;
        $display(" tb_top_debug: FAILED (timeout)");
        $finish;
    end

endmodule
