// =============================================================================
// arbiter.sv
// Fixed-priority arbiter: Master 1 (index 0) is HARDCODED as the highest
// priority requester. Master 2 (index 1) is only granted when Master 1 is
// idle or currently masked out by an in-progress split.
//
// Split handling:
//   - split_pulse : asserted (combinationally, by the top level) the cycle
//                   a SPLIT response comes back for the currently granted
//                   master's transfer targeting the split-capable slave.
//   - On split_pulse: the granted master is masked (its requests are
//                   ignored by the arbiter) and hsplit_notify is pulsed to
//                   it for one cycle so it can back off the bus.
//   - hsplit_done : pulsed by the split-capable slave once the requested
//                   resource is ready. The arbiter unmasks the split
//                   owner and forces a one-cycle "resume" grant to it,
//                   overriding normal priority, so it can collect its
//                   result / completion.
// =============================================================================
import bus_pkg::*;

module arbiter (
  input  logic                    clk,
  input  logic                    rst_n,          // async, active-low

  input  logic [NUM_MASTERS-1:0]  hreq,
  input  logic                    split_pulse,
  input  logic                    hsplit_done,

  output logic [NUM_MASTERS-1:0]  hgrant,
  output logic [NUM_MASTERS-1:0]  hsplit_notify,
  output logic                    hready
);

  logic [NUM_MASTERS-1:0] req_mask_q;     // 1 = master masked (split pending)
  logic [NUM_MASTERS-1:0] split_owner_q;  // which master owns the pending split
  logic                   resume_pending_q;
  logic [NUM_MASTERS-1:0] resume_master_q;

  logic [NUM_MASTERS-1:0] active_req;
  assign active_req = hreq & ~req_mask_q;

  // ---------------------------------------------------------------
  // Combinational grant logic
  // ---------------------------------------------------------------
  always_comb begin
    hgrant = '0;
    if (resume_pending_q) begin
      hgrant = resume_master_q;          // override: finish the split transfer
    end else if (active_req[0]) begin
      hgrant[0] = 1'b1;                  // Master 1 - HARDCODED highest priority
    end else if (active_req[1]) begin
      hgrant[1] = 1'b1;                  // Master 2 - only if Master 1 not requesting
    end
  end

  // This simplified bus has no wait-state slaves other than the split
  // mechanism itself, so HREADY is always asserted.
  assign hready = 1'b1;

  // ---------------------------------------------------------------
  // Sequential state
  // ---------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      req_mask_q       <= '0;
      split_owner_q    <= '0;
      resume_pending_q <= 1'b0;
      resume_master_q  <= '0;
      hsplit_notify    <= '0;
    end else begin
      hsplit_notify <= '0;

      if (resume_pending_q) begin
        resume_pending_q <= 1'b0;        // one-cycle override only
      end

      // A SPLIT response has just come back combinationally for whichever
      // master `hgrant` currently points at (split_pulse and hgrant are
      // both derived from the same-cycle transfer, so hgrant -- not a
      // registered/delayed copy of it -- is the correct thing to sample
      // here alongside split_pulse).
      if (split_pulse) begin
        req_mask_q    <= req_mask_q | hgrant;
        split_owner_q <= split_owner_q | hgrant;
        hsplit_notify <= hgrant;
      end

      // Split-capable slave says the resource is ready
      if (hsplit_done) begin
        req_mask_q       <= req_mask_q & ~split_owner_q;
        resume_pending_q <= 1'b1;
        resume_master_q  <= split_owner_q;
        split_owner_q    <= '0;
      end
    end
  end

endmodule : arbiter