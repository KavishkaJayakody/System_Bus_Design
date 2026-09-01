// =============================================================================
// edge_detect.sv
// 2-stage synchronizer + falling-edge detector for the active-low DE2-115
// push buttons (KEY[]). Produces a clean, single-cycle pulse_out on press.
// =============================================================================
module edge_detect (
  input  logic clk,
  input  logic rst_n,
  input  logic sig_in,      // active-low raw input (idle = 1, pressed = 0)
  output logic pulse_out    // 1-cycle pulse when sig_in falls (button press)
);

  logic sync_ff1, sync_ff2, prev_ff;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sync_ff1 <= 1'b1;
      sync_ff2 <= 1'b1;
      prev_ff  <= 1'b1;
    end else begin
      sync_ff1 <= sig_in;     // 1st synchronizer stage
      sync_ff2 <= sync_ff1;   // 2nd synchronizer stage (metastability-safe sample)
      prev_ff  <= sync_ff2;   // previous sample, one cycle behind sync_ff2
    end
  end

  assign pulse_out = prev_ff & ~sync_ff2;   // was 1 (idle), now 0 (pressed)

endmodule : edge_detect