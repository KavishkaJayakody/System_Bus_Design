// =============================================================================
// slave_simple.sv
// Plain single-cycle read/write memory slave, no split support. Reused for
// both Slave 2 (4K) and Slave 3 (2K) via the SIZE_BYTES parameter.
// =============================================================================
import bus_pkg::*;

module slave_simple #(
  parameter int SIZE_BYTES = 4096
)(
  input  logic                   clk,
  input  logic                   rst_n,

  input  logic                   hsel,
  input  logic [ADDR_WIDTH-1:0]  haddr,
  input  logic [DATA_WIDTH-1:0]  hwdata,
  input  logic                   hwrite,
  input  logic                   hvalid,

  output logic [DATA_WIDTH-1:0]  hrdata,
  output logic [1:0]             hresp
);

  localparam int AW = $clog2(SIZE_BYTES);

  logic [DATA_WIDTH-1:0] mem [0:SIZE_BYTES-1];
  logic [AW-1:0]         idx;

  assign idx    = haddr[AW-1:0];
  assign hresp  = HRESP_OKAY;                 // never splits
  assign hrdata = (hsel && hvalid) ? mem[idx] : '0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // No reset value required for RAM contents; reset only clears
      // control state elsewhere in the system.
    end else if (hsel && hvalid && hwrite) begin
      mem[idx] <= hwdata;
    end
  end

endmodule : slave_simple