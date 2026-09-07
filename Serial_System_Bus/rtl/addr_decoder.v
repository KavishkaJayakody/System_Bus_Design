//==========================================================================
// addr_decoder.v
//
// Purely combinational address decoder for the shared system bus.
//
// Takes the address currently driven on the bus and produces a one-hot slave
// select.  Any address that falls outside the three mapped ranges - including
// the 0x2800-0x2FFF hole above slave 2 and the whole addr[15]==1 remote
// window reserved for phase 2 - selects the default slave instead, which
// answers with ERROR.  Exactly one of {def_sel, slv_sel} is high whenever
// `en' is high, so the bus can never be left without a responder and can
// never hang on an unmapped access.
//
// This module holds no state and has no clock or reset.  `en' stands in for
// reset behaviour: while the bus is idle or held in reset the top level
// drives en=0 and every select output is 0.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// en              in   1          Decode enable.  1 = a transfer is being
//                                 presented on the bus this cycle.  0 forces
//                                 every select low (idle / reset).
// addr            in   ADDR_W     Word address presented by the granted
//                                 master.
// slv_sel         out  N_SLAVES   One-hot select for the mapped slaves,
//                                 bit 0 = slave 0, bit 1 = slave 1,
//                                 bit 2 = slave 2.  All zero when the
//                                 address is unmapped or en=0.
// def_sel         out  1          Default-slave select.  1 when en=1 and the
//                                 address matches no mapped slave.
// hit             out  1          1 when en=1 and the address hit a mapped
//                                 slave.  Purely for status/debug; equals
//                                 |slv_sel.
//==========================================================================
`include "bus_defs.vh"

module addr_decoder #(
    parameter ADDR_W   = `BUS_ADDR_W,
    parameter N_SLAVES = `BUS_N_SLAVES
) (
    input  wire                 en,
    input  wire [ADDR_W-1:0]    addr,
    output wire [N_SLAVES-1:0]  slv_sel,
    output wire                 def_sel,
    output wire                 hit
);

    //----------------------------------------------------------------------
    // Range compares.  Written as prefix matches on the top address bits so
    // they synthesise to a handful of LUTs rather than two magnitude
    // comparators per slave.
    //----------------------------------------------------------------------
    wire hit_s0 = (addr[ADDR_W-1:12] == 4'h0);                  // 0x0000-0x0FFF
    wire hit_s1 = (addr[ADDR_W-1:12] == 4'h1);                  // 0x1000-0x1FFF
    wire hit_s2 = (addr[ADDR_W-1:11] == 5'b00100);              // 0x2000-0x27FF

    assign slv_sel[`SEL_S0] = en & hit_s0;
    assign slv_sel[`SEL_S1] = en & hit_s1;
    assign slv_sel[`SEL_S2] = en & hit_s2;

    assign hit     = en & (hit_s0 | hit_s1 | hit_s2);
    assign def_sel = en & ~(hit_s0 | hit_s1 | hit_s2);

endmodule
