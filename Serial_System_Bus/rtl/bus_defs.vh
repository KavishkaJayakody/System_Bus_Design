// Shared constants.  Slave sizes 2K/4K/4K and device ids 0/1/2 are fixed by
// the BOARD-TO-BOARD LINK SPEC - changing them breaks interop silently,
// because a response frame carries data but no status.

`ifndef BUS_DEFS_VH
`define BUS_DEFS_VH

// Write data is right-aligned inside the address frame, which is what lets
// every receiver just keep "the last W bits seen" with no bit counter.
`define BUS_ADDR_W    16      // byte-less word address, 64K word space
`define BUS_DATA_W     8      // one bus word.  Serial, so this is also the
`define BUS_RESP_W     2      // slave response code, see below
`define BUS_N_MASTERS  3
`define BUS_ID_W       2      // ceil(log2(BUS_N_MASTERS))
`define BUS_N_SLAVES   4

// OKAY=00 ERROR=01 SPLIT=10.  ERROR is reported, never retried.  On SPLIT the
// slave has latched master_id and will pulse split_complete[id] later.
`define RESP_OKAY     2'b00
`define RESP_ERROR    2'b01
`define RESP_SPLIT    2'b10
`define RESP_RSVD     2'b11

// 0x0000 2K id0 | 0x1000 4K id1 | 0x2000 4K id2 (splits) | 0x8000 16K bridge.
// The bridge window is exactly the 14 address bits the link carries.
// 0x0800-0x0FFF, 0x3000-0x7FFF and 0xC000-0xFFFF answer ERROR.
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

`define S3_BASE       16'h8000
`define S3_TOP        16'hBFFF
`define S3_WORDS      16384
`define S3_LADDR_W    14

`define REMOTE_BASE   16'h8000
`define REMOTE_BIT    15      // addr[15] set => this transaction is remote
`define LINK_CMD_W    24      // wdata[8] + dev[2] + offset[12] + we + rsvd
`define LINK_REQ_TAG  8'hA5
`define LINK_RESP_TAG 8'h5A

`define SEL_S0        0
`define SEL_S1        1
`define SEL_S2        2
`define SEL_BR        3
`define SEL_DEF       4

`endif // BUS_DEFS_VH
