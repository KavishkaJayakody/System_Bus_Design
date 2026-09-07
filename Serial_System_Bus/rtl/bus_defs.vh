//==========================================================================
// bus_defs.vh -- shared constants for the 2-master / 3-slave system bus
//
// Included by every RTL module and testbench.  Module port widths are still
// *parameters* (so a module can be re-instantiated at another size); these
// macros only supply the project-wide defaults and the encodings that must
// agree between modules.
//
// Search path: Quartus gets `rtl' via SEARCH_PATH in the .qsf,
//              Icarus gets it via `-I rtl'.
//==========================================================================
`ifndef BUS_DEFS_VH
`define BUS_DEFS_VH

//--------------------------------------------------------------------------
// Bus geometry
//--------------------------------------------------------------------------
`define BUS_ADDR_W    16      // byte-less word address, 64K word space
`define BUS_DATA_W    32      // one bus word
`define BUS_RESP_W     2      // slave response code, see below
`define BUS_N_MASTERS  2      // arbiter is parameterised; 3 is the phase-2 size
`define BUS_N_SLAVES   3      // mapped slaves, excluding the default slave

//--------------------------------------------------------------------------
// Slave response encoding (driven with `ready')
//   OKAY   transfer completed, rdata valid on a read
//   ERROR  address not mapped / illegal - transfer is over, do not retry
//   SPLIT  slave is busy, it has latched master_id and will pulse
//          split_complete[master_id] later; the master must re-issue
//--------------------------------------------------------------------------
`define RESP_OKAY     2'b00
`define RESP_ERROR    2'b01
`define RESP_SPLIT    2'b10
`define RESP_RSVD     2'b11

//--------------------------------------------------------------------------
// Address map.  addr[15] == 1'b1 is RESERVED for the phase-2 remote window
// and must stay unmapped (it decodes to the default slave => ERROR).
//
//   slave 0   0x0000 - 0x0FFF   4K words, split capable   addr[15:12]==4'h0
//   slave 1   0x1000 - 0x1FFF   4K words                  addr[15:12]==4'h1
//   slave 2   0x2000 - 0x27FF   2K words                  addr[15:11]==5'b00100
//   default   everything else (incl. 0x2800-0x2FFF and all of 0x8000-0xFFFF)
//--------------------------------------------------------------------------
`define S0_BASE       16'h0000
`define S0_TOP        16'h0FFF
`define S1_BASE       16'h1000
`define S1_TOP        16'h1FFF
`define S2_BASE       16'h2000
`define S2_TOP        16'h27FF

`define S0_WORDS      4096
`define S1_WORDS      4096
`define S2_WORDS      2048

`define S0_LADDR_W    12      // $clog2(S0_WORDS)
`define S1_LADDR_W    12
`define S2_LADDR_W    11

// Slave index positions inside the one-hot select vector.  The default slave
// always occupies the top bit, so a vector is {def, s2, s1, s0}.
`define SEL_S0        0
`define SEL_S1        1
`define SEL_S2        2
`define SEL_DEF       3

`endif // BUS_DEFS_VH
