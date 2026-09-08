//==========================================================================
// shift_deser.v
//
// Serial in, parallel out.  The other half of the serial bus primitives;
// instantiated in master.v (read data), slave.v (local address and write
// data) and bus_top.v (the full address, for the decoder).
//
// MSB first: after W cycles of `shift', dout holds the W bits received, the
// first one in dout[W-1].
//
// It holds THE LAST W BITS SEEN and nothing else - there is no counter and
// no framing.  That is deliberate, and it is what makes the bus cheap:
//
//   * A slave only needs the low bits of the address.  Give it a
//     W = LADDR_W deserialiser, shift it for the whole 16-clock address
//     frame, and it ends up holding addr[LADDR_W-1:0] with the upper bits
//     harmlessly shifted straight through and discarded.
//
//   * Write data is right-aligned in the frame, so a W = DATA_W deserialiser
//     shifted for the whole frame ends up holding exactly the data.
//
// One frame timer in the master therefore serves every receiver on the bus.
//
//--------------------------------------------------------------------------
// Port    Dir  Width  Meaning
//--------------------------------------------------------------------------
// clk     in   1      Bus clock.
// rst_n   in   1      Asynchronous active-low reset.
// shift   in   1      Sample din this cycle.
// din     in   1      Serial input.
// dout    out  W      The last W bits shifted in, first bit in dout[W-1].
//==========================================================================

module shift_deser #(
    parameter W = 8
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         shift,
    input  wire         din,
    output wire [W-1:0] dout
);

    reg [W-1:0] sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     sr <= {W{1'b0}};
        else if (shift) sr <= {sr[W-2:0], din};
    end

    assign dout = sr;

endmodule
