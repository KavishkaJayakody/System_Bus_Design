//==========================================================================
// addr_decoder.v
//
// SERIAL address decoder for the shared system bus.
//
// It watches the address arrive one bit at a time on `astream' and narrows
// the set of slaves that could still match as it goes.  No 16-bit address is
// ever assembled: the decoder is a progressive prefix matcher, not a
// comparator behind a deserialiser.
//
// Any address outside the four mapped ranges - including the 0x0800-0x0FFF
// hole above slave 0 and everything from 0xC000 up - selects the default
// slave, which answers ERROR.
//
// The fourth target is the REMOTE BRIDGE at 0x8000-0xBFFF.  It is decoded
// exactly like a memory; what sits behind it is the other board rather than
// an array, which the decoder neither knows nor cares about.
//
// Exactly one of {def_sel, slv_sel} is high whenever `en' is high, so the bus
// can never be left without a responder and can never hang on a bad address.
//
//--------------------------------------------------------------------------
// How it works, and why it needs a position marker
//--------------------------------------------------------------------------
// Every OTHER receiver on this bus needs no bit counter, because the address
// goes out MSB-first and write data is right-aligned: a plain shift register
// of width W ends up holding "the last W bits I saw", which is exactly the
// low-order field it wanted.
//
// The decoder is the one receiver that wants the OPPOSITE end.  It needs the
// top few bits - the slave prefix - and those arrive FIRST.  "The last W
// bits" cannot give it those, so it has to know which bit is on the wire.
//
// It knows by the cheapest possible means: a PFX_W-bit one-hot marker that
// starts at the first bit of the frame and shifts once per clock.  After
// PFX_W bits it becomes zero and the decoder stops looking - the remaining
// address bits are the slave's offset and none of the decoder's business.
//
//   frame    ____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|____
//   astream  ----<a15 a14 a13 a12 a11 a10 ....  a1  a0 >-----
//   pos       10000 01000 00100 00010 00001 00000 ...  00000
//   alive[i]  <-- narrowing -->   settled ------------------>
//                                 ^ decided here, 11 clocks early
//   en       ________________________________________|‾‾‾‾|__
//
// Each slave keeps one `alive' bit.  On every marked bit position that the
// slave cares about, a mismatch clears it for the rest of the frame.  A
// slave whose bit is still set when the frame ends matched its whole prefix.
//
// COST: PFX_W + N_SLAVES flip-flops - 8 for this map.  The deserialise-then-
// compare arrangement this replaced needed ADDR_W = 16 just to hold the
// address, and could not decide until the last bit had landed.
//
//--------------------------------------------------------------------------
// The map comes from bus_defs.vh
//--------------------------------------------------------------------------
// Each slave is a power-of-two block, so its range IS a prefix: the top
// (ADDR_W - LADDR_W) bits of its base address.  Both constants below are
// derived, so moving a slave in bus_defs.vh moves the decoder with it -
// unlike the hand-written `4'h0' / `5'b00100' compares this replaced.
//
//   slave 0   base 0x0000, 11 offset bits -> prefix 00000 (5 bits)
//   slave 1   base 0x1000, 12 offset bits -> prefix 0001  (4 bits)
//   slave 2   base 0x2000, 12 offset bits -> prefix 0010  (4 bits)
//   bridge    base 0x8000, 14 offset bits -> prefix 10    (2 bits)
//
// PFX_W is the longest of those prefixes; shorter ones simply have
// don't-care bits at the bottom, which is what CARE encodes.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock.
// rst_n           in   1          Asynchronous active-low reset.
// frame           in   1          bus_valid: high for the whole address
//                                 frame.  Its low phase re-arms the matcher,
//                                 so no separate "start" signal is needed.
// astream         in   1          THE ADDRESS WIRE, MSB first.
// en              in   1          Decode strobe (addr_done, the falling edge
//                                 of frame).  0 forces every select low, so
//                                 the bus is quiet while idle or in reset.
// slv_sel         out  N_SLAVES   One-hot select, bit i = slave i.  All zero
//                                 when the address is unmapped or en=0.
// def_sel         out  1          Default-slave select: en and no match.
// hit             out  1          en and a mapped slave matched. Status only.
//==========================================================================
`include "bus_defs.vh"

module addr_decoder #(
    parameter ADDR_W   = `BUS_ADDR_W,
    parameter N_SLAVES = `BUS_N_SLAVES
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 frame,
    input  wire                 astream,
    input  wire                 en,
    output wire [N_SLAVES-1:0]  slv_sel,
    output wire                 def_sel,
    output wire                 hit
);

    //----------------------------------------------------------------------
    // Prefix lengths, derived from the slave sizes in bus_defs.vh.
    //----------------------------------------------------------------------
    localparam S0_PLEN = ADDR_W - `S0_LADDR_W;      // 5  (2K memory)
    localparam S1_PLEN = ADDR_W - `S1_LADDR_W;      // 4  (4K memory)
    localparam S2_PLEN = ADDR_W - `S2_LADDR_W;      // 4  (4K memory)
    localparam S3_PLEN = ADDR_W - `S3_LADDR_W;      // 2  (16K remote bridge)

    localparam MAX01   = (S0_PLEN > S1_PLEN) ? S0_PLEN : S1_PLEN;
    localparam MAX012  = (MAX01   > S2_PLEN) ? MAX01   : S2_PLEN;
    localparam PFX_W   = (MAX012  > S3_PLEN) ? MAX012  : S3_PLEN;   // 5

    // The expected prefix: the top PFX_W bits of the target's base address.
    localparam [PFX_W-1:0] PFX0 = `S0_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX1 = `S1_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX2 = `S2_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX3 = `S3_BASE >> (ADDR_W - PFX_W);

    // Which of those bits actually matter: the top PLEN of them.  A 4-bit
    // prefix inside a 5-bit field leaves the bottom bit don't-care; the
    // bridge's is only 2 bits, so it leaves the bottom three.
    localparam [PFX_W-1:0] CARE0 = ((1 << S0_PLEN) - 1) << (PFX_W - S0_PLEN);
    localparam [PFX_W-1:0] CARE1 = ((1 << S1_PLEN) - 1) << (PFX_W - S1_PLEN);
    localparam [PFX_W-1:0] CARE2 = ((1 << S2_PLEN) - 1) << (PFX_W - S2_PLEN);
    localparam [PFX_W-1:0] CARE3 = ((1 << S3_PLEN) - 1) << (PFX_W - S3_PLEN);

    localparam [N_SLAVES*PFX_W-1:0] PFX_FLAT  = {PFX3,  PFX2,  PFX1,  PFX0};
    localparam [N_SLAVES*PFX_W-1:0] CARE_FLAT = {CARE3, CARE2, CARE1, CARE0};

    //----------------------------------------------------------------------
    // Position marker and the per-slave "still matching" bits.
    //
    // `frame' low re-arms both, so a frame always starts from a clean state
    // and the matcher needs no reset of its own beyond rst_n.
    //----------------------------------------------------------------------
    localparam [PFX_W-1:0] POS_FIRST = 1 << (PFX_W - 1);

    reg [PFX_W-1:0]   pos;
    reg [N_SLAVES-1:0] alive;

    // A slave dies on this bit if the marker is on a bit it cares about and
    // the wire disagrees with its prefix.
    wire [N_SLAVES-1:0] miss;
    genvar gi;
    generate
        for (gi = 0; gi < N_SLAVES; gi = gi + 1) begin : g_match
            assign miss[gi] = |(pos
                              & CARE_FLAT[gi*PFX_W +: PFX_W]
                              & (PFX_FLAT[gi*PFX_W +: PFX_W]
                                 ^ {PFX_W{astream}}));
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos   <= POS_FIRST;
            alive <= {N_SLAVES{1'b1}};
        end else if (!frame) begin
            // Between frames: re-arm.  `en' is asserted during this phase and
            // samples `alive' combinationally, so the result is still the one
            // this frame produced - it is cleared for the NEXT frame, not
            // this one.
            pos   <= POS_FIRST;
            alive <= {N_SLAVES{1'b1}};
        end else begin
            pos   <= pos >> 1;
            alive <= alive & ~miss;
        end
    end

    //----------------------------------------------------------------------
    // Outputs.  `en' gates everything, exactly as before: one-cycle pulses,
    // and silence while the bus is idle.
    //----------------------------------------------------------------------
    assign slv_sel = {N_SLAVES{en}} & alive;
    assign hit     = en &  (|alive);
    assign def_sel = en & ~(|alive);

endmodule
