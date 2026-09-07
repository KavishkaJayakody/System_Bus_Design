//==========================================================================
// tb_addr_decoder.v -- self-checking testbench for addr_decoder
//
// Covers, in the order the brief lists them:
//   1. reset behaviour        - en=0 forces every select low
//   2. one-hot select         - a mid-range address in each of the 3 ranges
//   3. boundary addresses     - first AND last address of every range, plus
//                               the address one past each range
//   4. unmapped addresses     - the 0x2800-0x2FFF hole, the reserved
//                               addr[15]==1 remote window, and the top of
//                               memory; default slave selected, no slave
//                               select asserted
//   5. mutual exclusion       - {def_sel, slv_sel} is one-hot for every
//                               address in an exhaustive 64K sweep
//
// Prints PASS/FAIL and an error count.  Purely combinational DUT, so the
// stimulus is a sequence of blocking assignments with no clock.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_addr_decoder;

    localparam ADDR_W   = `BUS_ADDR_W;
    localparam N_SLAVES = `BUS_N_SLAVES;

    reg                   en;
    reg  [ADDR_W-1:0]     addr;
    wire [N_SLAVES-1:0]   slv_sel;
    wire                  def_sel;
    wire                  hit;

    integer errors = 0;
    integer i;

    addr_decoder #(.ADDR_W(ADDR_W), .N_SLAVES(N_SLAVES)) dut (
        .en(en), .addr(addr), .slv_sel(slv_sel), .def_sel(def_sel), .hit(hit)
    );

    //----------------------------------------------------------------------
    // Check one address: expected {def_sel, slv_sel} value.
    //----------------------------------------------------------------------
    task check;
        input [ADDR_W-1:0]   a;
        input                e;
        input [N_SLAVES:0]   exp;      // {def, s2, s1, s0}
        input [200*8-1:0]    name;
        begin
            en   = e;
            addr = a;
            #1;
            if ({def_sel, slv_sel} !== exp) begin
                $display("  ERROR %0s: addr=0x%04h en=%b -> {def,sel}=%b, expected %b",
                         name, a, e, {def_sel, slv_sel}, exp);
                errors = errors + 1;
            end else begin
                $display("  ok    %0s: addr=0x%04h en=%b -> {def,sel}=%b",
                         name, a, e, {def_sel, slv_sel});
            end
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_addr_decoder");
        $display("======================================================");

        //------------------------------------------------------------------
        $display("-- 1. reset / idle behaviour (en=0) ------------------");
        check(16'h0000, 1'b0, 4'b0000, "idle s0 base");
        check(16'h1234, 1'b0, 4'b0000, "idle s1 mid");
        check(16'h2100, 1'b0, 4'b0000, "idle s2 mid");
        check(16'h2900, 1'b0, 4'b0000, "idle unmapped");
        check(16'hFFFF, 1'b0, 4'b0000, "idle top of memory");

        //------------------------------------------------------------------
        $display("-- 2. one-hot select in each range -------------------");
        check(16'h0800, 1'b1, 4'b0001, "s0 mid");
        check(16'h1800, 1'b1, 4'b0010, "s1 mid");
        check(16'h2400, 1'b1, 4'b0100, "s2 mid");

        //------------------------------------------------------------------
        $display("-- 3. range boundaries -------------------------------");
        check(`S0_BASE,        1'b1, 4'b0001, "s0 first  0x0000");
        check(`S0_TOP,         1'b1, 4'b0001, "s0 last   0x0FFF");
        check(`S1_BASE,        1'b1, 4'b0010, "s1 first  0x1000");
        check(`S1_TOP,         1'b1, 4'b0010, "s1 last   0x1FFF");
        check(`S2_BASE,        1'b1, 4'b0100, "s2 first  0x2000");
        check(`S2_TOP,         1'b1, 4'b0100, "s2 last   0x27FF");
        // one past the top of each range
        check(`S0_TOP + 16'd1, 1'b1, 4'b0010, "s0 top+1 -> s1");
        check(`S1_TOP + 16'd1, 1'b1, 4'b0100, "s1 top+1 -> s2");
        check(`S2_TOP + 16'd1, 1'b1, 4'b1000, "s2 top+1 -> default");

        //------------------------------------------------------------------
        $display("-- 4. unmapped -> default slave ----------------------");
        check(16'h2800, 1'b1, 4'b1000, "hole first 0x2800");
        check(16'h2FFF, 1'b1, 4'b1000, "hole last  0x2FFF");
        check(16'h3000, 16'h1,4'b1000, "above map  0x3000");
        check(16'h7FFF, 1'b1, 4'b1000, "below remote window");
        check(16'h8000, 1'b1, 4'b1000, "remote window base (reserved)");
        check(16'hFFFF, 1'b1, 4'b1000, "top of memory");

        //------------------------------------------------------------------
        $display("-- 5. exhaustive 64K one-hot sweep -------------------");
        en = 1'b1;
        for (i = 0; i < 65536; i = i + 1) begin
            addr = i[ADDR_W-1:0];
            #1;
            // exactly one responder, always
            if ({def_sel, slv_sel} != (({def_sel, slv_sel}) & -({def_sel, slv_sel}))
                || {def_sel, slv_sel} == 0) begin
                $display("  ERROR addr=0x%04h not one-hot: {def,sel}=%b",
                         addr, {def_sel, slv_sel});
                errors = errors + 1;
            end
            // hit must agree with |slv_sel and be the complement of def_sel
            if (hit !== (|slv_sel) || hit === def_sel) begin
                $display("  ERROR addr=0x%04h hit=%b inconsistent (sel=%b def=%b)",
                         addr, hit, slv_sel, def_sel);
                errors = errors + 1;
            end
            // nothing may ever be mapped inside the reserved remote window
            if (addr[15] && |slv_sel) begin
                $display("  ERROR addr=0x%04h is inside the reserved addr[15]=1 window but selected a slave",
                         addr);
                errors = errors + 1;
            end
        end
        $display("  ok    64K sweep complete");

        $display("======================================================");
        if (errors == 0) $display(" tb_addr_decoder: PASSED (0 errors)");
        else             $display(" tb_addr_decoder: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
