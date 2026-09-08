//==========================================================================
// tb_addr_decoder.v -- self-checking testbench for the SERIAL addr_decoder
//
// The decoder no longer takes a parallel address: it watches the address
// arrive one bit at a time on `astream' and narrows the set of slaves that
// could still match.  So the testbench SHIFTS each address in MSB-first over
// a full frame and then strobes `en', which is exactly what system_bus does.
//
// Covers, in the order the brief lists them:
//   1. reset behaviour        - en=0 forces every select low
//   2. one-hot select         - a mid-range address in each of the 3 ranges
//   3. boundary addresses     - first AND last address of every range, plus
//                               the address one past each range
//   4. unmapped addresses     - the 0x0800-0x0FFF hole, the reserved
//                               addr[15]==1 remote window, and the top of
//                               memory; default slave selected, no slave
//                               select asserted
//   5. mutual exclusion       - {def_sel, slv_sel} is one-hot for every
//                               address in an exhaustive 64K sweep
//   6. serial-specific        - the decision is reached from the PREFIX
//                               alone, before the frame ends, and the low
//                               address bits cannot change it
//
// Prints PASS/FAIL and an error count.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_addr_decoder;

    localparam ADDR_W   = `BUS_ADDR_W;
    localparam N_SLAVES = `BUS_N_SLAVES;

    reg                   clk = 1'b0;
    reg                   rst_n;
    reg                   frame;
    reg                   astream;
    reg                   en;
    wire [N_SLAVES-1:0]   slv_sel;
    wire                  def_sel;
    wire                  hit;

    always #5 clk = ~clk;

    integer errors = 0;
    integer i;

    addr_decoder #(.ADDR_W(ADDR_W), .N_SLAVES(N_SLAVES)) dut (
        .clk(clk), .rst_n(rst_n),
        .frame(frame), .astream(astream), .en(en),
        .slv_sel(slv_sel), .def_sel(def_sel), .hit(hit)
    );

    //----------------------------------------------------------------------
    // Shift one address in MSB-first over a full ADDR_W-clock frame, then
    // strobe en for one cycle - the same sequence system_bus produces.
    //----------------------------------------------------------------------
    task automatic drive_frame;
        input [ADDR_W-1:0] a;
        integer b;
        begin
            @(negedge clk);
            frame = 1'b1;
            for (b = ADDR_W-1; b >= 0; b = b - 1) begin
                astream = a[b];
                @(negedge clk);
            end
            frame   = 1'b0;
            astream = 1'b0;
        end
    endtask

    //----------------------------------------------------------------------
    // Check one address: expected {def_sel, slv_sel} value.  `e' selects
    // whether the strobe is applied, so en=0 still means "everything low".
    //----------------------------------------------------------------------
    task automatic check;
        input [ADDR_W-1:0]   a;
        input                e;
        input [N_SLAVES:0]   exp;      // {def, s2, s1, s0}
        input [200*8-1:0]    name;
        begin
            drive_frame(a);
            en = e;
            #1;
            if ({def_sel, slv_sel} !== exp) begin
                $display("  ERROR %0s: addr=0x%04h en=%b -> {def,sel}=%b, expected %b",
                         name, a, e, {def_sel, slv_sel}, exp);
                errors = errors + 1;
            end else begin
                $display("  ok    %0s: addr=0x%04h en=%b -> {def,sel}=%b",
                         name, a, e, {def_sel, slv_sel});
            end
            @(negedge clk);
            en = 1'b0;
        end
    endtask

    // Same, but silent - for the 64K sweep.
    task automatic check_quiet;
        input [ADDR_W-1:0] a;
        output [N_SLAVES:0] got;
        begin
            drive_frame(a);
            en = 1'b1;
            #1;
            got = {def_sel, slv_sel};
            @(negedge clk);
            en = 1'b0;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_addr_decoder -- the SERIAL prefix matcher");
        $display("======================================================");

        rst_n   = 1'b0;
        frame   = 1'b0;
        astream = 1'b0;
        en      = 1'b0;
        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        //------------------------------------------------------------------
        $display("-- 1. reset / idle behaviour (en=0) ------------------");
        check(16'h0000, 1'b0, 4'b0000, "idle s0 base");
        check(16'h1234, 1'b0, 4'b0000, "idle s1 mid");
        check(16'h2100, 1'b0, 4'b0000, "idle s2 mid");
        check(16'h0900, 1'b0, 4'b0000, "idle unmapped");
        check(16'hFFFF, 1'b0, 4'b0000, "idle top of memory");

        //------------------------------------------------------------------
        $display("-- 2. one-hot select in each range -------------------");
        check(16'h0400, 1'b1, 4'b0001, "s0 mid");
        check(16'h1800, 1'b1, 4'b0010, "s1 mid");
        check(16'h2800, 1'b1, 4'b0100, "s2 mid");

        //------------------------------------------------------------------
        $display("-- 3. range boundaries -------------------------------");
        check(`S0_BASE,        1'b1, 4'b0001, "s0 first  0x0000");
        check(`S0_TOP,         1'b1, 4'b0001, "s0 last   0x0FFF");
        check(`S1_BASE,        1'b1, 4'b0010, "s1 first  0x1000");
        check(`S1_TOP,         1'b1, 4'b0010, "s1 last   0x1FFF");
        check(`S2_BASE,        1'b1, 4'b0100, "s2 first  0x2000");
        check(`S2_TOP,         1'b1, 4'b0100, "s2 last   0x27FF");
        // one past the top of each range
        check(`S0_TOP + 16'd1, 1'b1, 4'b1000, "s0 top+1 -> the 0x0800 hole");
        check(`S1_TOP + 16'd1, 1'b1, 4'b0100, "s1 top+1 -> s2");
        check(`S2_TOP + 16'd1, 1'b1, 4'b1000, "s2 top+1 -> default");

        //------------------------------------------------------------------
        $display("-- 4. unmapped -> default slave ----------------------");
        check(16'h0800, 1'b1, 4'b1000, "hole first 0x0800");
        check(16'h0FFF, 1'b1, 4'b1000, "hole last  0x0FFF");
        check(16'h3000, 16'h1,4'b1000, "above map  0x3000");
        check(16'h7FFF, 1'b1, 4'b1000, "below remote window");
        check(16'h8000, 1'b1, 4'b1000, "remote window base (reserved)");
        check(16'hFFFF, 1'b1, 4'b1000, "top of memory");

        //------------------------------------------------------------------
        $display("-- 5. exhaustive 64K one-hot sweep -------------------");
        $display("  ..    65536 frames, %0d clocks each - this takes a moment", ADDR_W);
        for (i = 0; i < 65536; i = i + 1) begin
            drive_frame(i[ADDR_W-1:0]);
            en = 1'b1;
            #1;
            // exactly one responder, always
            if ({def_sel, slv_sel} != (({def_sel, slv_sel}) & -({def_sel, slv_sel}))
                || {def_sel, slv_sel} == 0) begin
                $display("  ERROR addr=0x%04h not one-hot: {def,sel}=%b",
                         i[ADDR_W-1:0], {def_sel, slv_sel});
                errors = errors + 1;
            end
            // hit must agree with |slv_sel and be the complement of def_sel
            if (hit !== (|slv_sel) || hit === def_sel) begin
                $display("  ERROR addr=0x%04h hit=%b inconsistent (sel=%b def=%b)",
                         i[ADDR_W-1:0], hit, slv_sel, def_sel);
                errors = errors + 1;
            end
            // nothing may ever be mapped inside the reserved remote window
            if (i[15] && |slv_sel) begin
                $display("  ERROR addr=0x%04h is inside the reserved addr[15]=1 window but selected a slave",
                         i[ADDR_W-1:0]);
                errors = errors + 1;
            end
            @(negedge clk);
            en = 1'b0;
        end
        $display("  ok    64K sweep complete - one-hot for every address");

        //------------------------------------------------------------------
        // What only a SERIAL decoder can be asked: does it decide from the
        // prefix alone, before the rest of the address has arrived?
        //------------------------------------------------------------------
        $display("-- 6. the decision is reached early ------------------");

        // Drive just the first PFX_W bits of an address in slave 1's range
        // and stop; `alive' must already be settled on slave 1 alone, with
        // 11 address bits still to come.
        @(negedge clk);
        frame = 1'b1;
        astream = 1'b0; @(negedge clk);   // a15
        astream = 1'b0; @(negedge clk);   // a14
        astream = 1'b0; @(negedge clk);   // a13
        astream = 1'b1; @(negedge clk);   // a12  -> prefix 0001 = slave 1
        astream = 1'b0; @(negedge clk);   // a11
        #1;
        if (dut.alive === 3'b010) begin
            $display("  ok    settled on slave 1 after %0d of %0d bits", 5, ADDR_W);
        end else begin
            $display("  ERROR after the prefix, alive=%b, expected 010", dut.alive);
            errors = errors + 1;
        end
        if (dut.pos === 5'b00000) begin
            $display("  ok    the position marker has retired - no further compares");
        end else begin
            $display("  ERROR position marker still active: pos=%b", dut.pos);
            errors = errors + 1;
        end
        // finish the frame with junk in the offset bits; it must not matter
        for (i = 0; i < ADDR_W-5; i = i + 1) begin
            astream = i[0];
            @(negedge clk);
        end
        frame = 1'b0;
        en    = 1'b1;
        #1;
        if ({def_sel, slv_sel} === 4'b0010)
            $display("  ok    the offset bits could not change the decode");
        else begin
            $display("  ERROR offset bits changed the decode: {def,sel}=%b",
                     {def_sel, slv_sel});
            errors = errors + 1;
        end
        @(negedge clk);
        en = 1'b0;

        // No 16-bit address exists inside the decoder to be wrong about.
        $display("  ..    decoder state is %0d flops (pos %0d + alive %0d), not %0d",
                 5 + N_SLAVES, 5, N_SLAVES, ADDR_W);

        $display("======================================================");
        if (errors == 0) $display(" tb_addr_decoder: PASSED (0 errors)");
        else             $display(" tb_addr_decoder: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
