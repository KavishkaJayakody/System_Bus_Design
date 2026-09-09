//==========================================================================
// default_slave.v
//
// The responder for every address the decoder does not map: the 0x0800-
// 0x0FFF hole above the 2K slave 0, and 0x3000-0x7FFF above slave 2.
//
// 0x8000-0xBFFF does NOT reach here: it is decoded to the BRIDGE, which is an
// ordinary target on this bus.  What is left over is 0xC000-0xFFFF, above
// the bridge window, and that lands here like any other hole.
//
// It exists purely so the bus cannot hang.  Without it an unmapped access
// would assert no slave select, no slave would ever drive ready, and the
// granted master would sit in its wait state holding the bus forever with
// only a reset to recover it.  Instead the access completes in one cycle
// with resp=ERROR, the arbiter releases the grant, and the master reports
// the failure and moves on.
//
// It needs no deserialisers: it does not care what the address or the data
// were, only that nothing else claimed them.  It never drives the shared
// data wire.  An ERROR is answered in one cycle, long before any data phase,
// so the master's read deserialiser still holds the PREVIOUS read's bits at
// that moment - `master' therefore forces rdata to zero on ERROR rather than
// handing that back.  See master.v.
//
//--------------------------------------------------------------------------
// Port          Dir  Width   Meaning
//--------------------------------------------------------------------------
// clk           in   1       Bus clock.
// rst_n         in   1       Asynchronous active-low reset.
// sel           in   1       One-cycle select: the decoder found no mapped
//                            slave for this address.
// dstream_out   out  1       Always 0 - this slave never sends data.
// ready         out  1       Completion strobe, one cycle after sel.
// resp          out  RESP_W  Always ERROR when ready is high.
//==========================================================================
`include "bus_defs.vh"

module default_slave #(
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sel,
    output wire               dstream_out,
    output reg                ready,
    output reg  [RESP_W-1:0]  resp
);

    assign dstream_out = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            resp  <= `RESP_OKAY;
        end else begin
            ready <= sel;
            resp  <= `RESP_ERROR;
        end
    end

endmodule
