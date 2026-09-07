//==========================================================================
// default_slave.v
//
// The responder for every address the decoder does not map: the 0x2800-
// 0x2FFF hole above slave 2, and the whole addr[15]==1 window reserved for
// the phase-2 remote bridge.
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
// data wire, so a read of an unmapped address returns whatever the wire was
// idling at, which is zero.
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
