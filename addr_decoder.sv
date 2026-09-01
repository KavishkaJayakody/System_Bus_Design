// =============================================================================
// addr_decoder.sv
// Combinational, one-hot address decoder for the 3 slaves.
// Blocks (re-)selection of Slave 1 while it is locked in an active split so
// no other transfer can sneak in and disturb the pending resource.
// =============================================================================
import bus_pkg::*;

module addr_decoder (
  input  logic [ADDR_WIDTH-1:0] haddr,
  input  logic                  hsplit_active,   // Slave 1 currently split-pending
  output logic [NUM_SLAVES-1:0] hsel             // one-hot: [0]=S1 [1]=S2 [2]=S3
);

  always_comb begin
    hsel = '0;
    if (haddr >= S1_BASE && haddr <= S1_TOP) begin
      hsel[0] = ~hsplit_active;
    end else if (haddr >= S2_BASE && haddr <= S2_TOP) begin
      hsel[1] = 1'b1;
    end else if (haddr >= S3_BASE && haddr <= S3_TOP) begin
      hsel[2] = 1'b1;
    end
    // Any address outside the 3 ranges: hsel stays all-zero (no slave responds)
  end

endmodule : addr_decoder