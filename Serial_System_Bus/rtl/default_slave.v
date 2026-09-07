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
// only a reset to recover it.  Instead the access completes in the normal
// one cycle with resp=ERROR, the arbiter releases the grant, and the master
// reports the failure and moves on.
//
// Timing matches slave_mem exactly - ready one cycle after sel - so the
// registered return mux does not need a special case.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock.
// rst_n           in   1          Asynchronous active-low reset.
// sel             in   1          Access strobe: the decoder found no mapped
//                                 slave for this address.
// rdata           out  DATA_W     Always zero.  Present so the return mux is
//                                 uniform across all responders.
// ready           out  1          Completion strobe, one cycle after sel.
// resp            out  RESP_W     Always ERROR when ready is high.
//==========================================================================
`include "bus_defs.vh"

module default_slave #(
    parameter DATA_W = `BUS_DATA_W,
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sel,
    output wire [DATA_W-1:0]  rdata,
    output reg                ready,
    output reg  [RESP_W-1:0]  resp
);

    assign rdata = {DATA_W{1'b0}};

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
