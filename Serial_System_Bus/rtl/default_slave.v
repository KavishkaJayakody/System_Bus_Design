// Responder for every address the decoder does not map, so an unmapped access
// completes with ERROR instead of hanging the bus forever.

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
