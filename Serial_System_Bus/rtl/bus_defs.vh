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
//
// The address and the data each travel on ONE wire, MSB first, one bit per
// clock.  BUS_ADDR_W and BUS_DATA_W are therefore frame lengths as much as
// widths: an address phase costs BUS_ADDR_W clocks.
//
// BUS_DATA_W must be <= BUS_ADDR_W.  Write data is right-aligned inside the
// address frame (BUS_ADDR_W-BUS_DATA_W leading zeros, then the data), which
// is what lets every receiver simply keep "the last W bits I saw" with no
// bit counter of its own.
//--------------------------------------------------------------------------
`define BUS_ADDR_W    16      // byte-less word address, 64K word space
`define BUS_DATA_W     8      // one bus word.  Serial, so this is also the
                              // number of clocks a data phase costs.
`define BUS_RESP_W     2      // slave response code, see below
// Three requesters: the two local masters, plus the remote bridge's MASTER
// FACE at index 2.  The bridge is a bus device, not part of a master.
`define BUS_N_MASTERS  3
`define BUS_ID_W       2      // ceil(log2(BUS_N_MASTERS))
// Four decoded targets: three memories and the bridge's SLAVE FACE.  The
// default responder is separate and always occupies the top select bit.
`define BUS_N_SLAVES   4

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
// Address map.
//
// The sizes and the device ids are fixed by the BOARD-TO-BOARD LINK SPEC:
// both ends must agree on 2K / 4K / 4K and ids 0/1/2, or a remote access
// lands somewhere different from where the far board thinks it sent it.
// Do not change these without changing the other board too.
//
//   slave 0   0x0000 - 0x07FF   2K words   id 0   addr[15:11]==5'b00000
//   slave 1   0x1000 - 0x1FFF   4K words   id 1   addr[15:12]==4'h1
//   slave 2   0x2000 - 0x2FFF   4K words   id 2   addr[15:12]==4'h2
//                               ^ SPLIT CAPABLE - the spec's "S3 splits on
//                                 read".  The splitter is the THIRD slave.
//   default   everything else: 0x0800-0x0FFF, 0x3000-0x7FFF
//
// 0x8000-0xBFFF is the REMOTE WINDOW, and it IS decoded - it selects the
// BRIDGE, which is an ordinary device on this bus (target 3).  A master
// reaches the other board by addressing it, exactly as it addresses a
// memory; the bridge answers SPLIT, frees the bus for the whole round trip,
// and wakes the master when the far board replies.
//
// far address = local address - 0x8000, and only addr[13:0] travels.
//
//   0x8000-0x87FF -> the far board's slave 0 (2K, id 0)
//   0x9000-0x9FFF -> the far board's slave 1 (4K, id 1)
//   0xA000-0xAFFF -> the far board's slave 2 (4K, id 2)
//
// The window is exactly 16K, which is the 14 address bits the link carries -
// so nothing is silently truncated.  0xC000-0xFFFF is OUTSIDE it and answers
// ERROR from the default responder like any other decode hole.
//--------------------------------------------------------------------------
`define S0_BASE       16'h0000
`define S0_TOP        16'h07FF
`define S1_BASE       16'h1000
`define S1_TOP        16'h1FFF
`define S2_BASE       16'h2000
`define S2_TOP        16'h2FFF

`define S0_WORDS      2048
`define S1_WORDS      4096
`define S2_WORDS      4096

`define S0_LADDR_W    11      // $clog2(S0_WORDS)
`define S1_LADDR_W    12
`define S2_LADDR_W    12

//--------------------------------------------------------------------------
// Target 3: the REMOTE BRIDGE's slave face.  Not a memory - a window onto
// the other board.  16K at 0x8000, so its prefix is just addr[15:14]==2'b10
// and the 14 offset bits it collects are precisely the 14 the link carries.
//--------------------------------------------------------------------------
`define S3_BASE       16'h8000
`define S3_TOP        16'hBFFF
`define S3_WORDS      16384
`define S3_LADDR_W    14

//--------------------------------------------------------------------------
// Remote window, per the link spec.  Local address - REMOTE_BASE = the far
// board's own address, and the low 14 bits of that are what travels on the
// wire as {dev[1:0], offset[11:0]}.
//--------------------------------------------------------------------------
`define REMOTE_BASE   16'h8000
`define REMOTE_BIT    15      // addr[15] set => this transaction is remote
`define LINK_CMD_W    24      // wdata[8] + dev[2] + offset[12] + we + rsvd
`define LINK_REQ_TAG  8'hA5
`define LINK_RESP_TAG 8'h5A

// Target positions inside the one-hot select vector.  The default responder
// always occupies the top bit, so a vector is {def, bridge, s2, s1, s0}.
`define SEL_S0        0
`define SEL_S1        1
`define SEL_S2        2
`define SEL_BR        3
`define SEL_DEF       4

`endif // BUS_DEFS_VH
