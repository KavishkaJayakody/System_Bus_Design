//==========================================================================
// shift_ser.v
//
// Parallel in, serial out.  One of the two primitives the serial bus is
// built from; instantiated in master.v for the address and write-data
// streams.
//
// MSB first: after `load', dout presents din[W-1], and each cycle of `shift'
// advances to the next bit down.
//
// There is no bit counter and no `done' output, on purpose.  The register
// shifts ZEROS in behind the data, so once all W bits have been driven the
// output simply goes quiet at 0 and stays there for as long as the frame
// runs.  That is what lets a short field (8 bits of write data) sit inside a
// longer frame (16 clocks of address) without either end counting: the
// sender pads, the receiver keeps the last W bits.  Frame length is timed
// once, by the master FSM, and nowhere else.
//
//--------------------------------------------------------------------------
// Port    Dir  Width  Meaning
//--------------------------------------------------------------------------
// clk     in   1      Bus clock.
// rst_n   in   1      Asynchronous active-low reset.
// load    in   1      Capture din.  Takes priority over shift.  Assert it
//                     the cycle BEFORE the first bit is wanted on dout.
// shift   in   1      Advance one bit.
// din     in   W      Word to send.
// dout    out  1      Serial output, MSB first, 0 once the word is spent.
//==========================================================================

module shift_ser #(
    parameter W = 8
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         load,
    input  wire         shift,
    input  wire [W-1:0] din,
    output wire         dout
);

    reg [W-1:0] sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     sr <= {W{1'b0}};
        else if (load)  sr <= din;
        else if (shift) sr <= {sr[W-2:0], 1'b0};   // zeros in behind
    end

    assign dout = sr[W-1];

endmodule
