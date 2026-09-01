// =============================================================================
// master.sv
// Generic master interface. Same module is instantiated twice at the top
// level (MASTER_ID=0 for Master 1, MASTER_ID=1 for Master 2); priority
// between them is entirely decided by the arbiter, not by this module.
//
// Bus timing note: this design has no wait-state slaves other than the
// split mechanism itself, so a transfer's address, HGRANT and HRESP/HRDATA
// all settle combinationally within the SAME clock cycle the master is
// granted. The FSM below therefore samples HRESP/HRDATA on the very edge
// that ends the cycle in which HGRANT is seen high while in REQUEST --
// there is no separate "wait one more cycle for the response" state,
// which would otherwise deadlock: once a SPLIT is taken, the arbiter
// masks this master immediately, so it would never see a second HGRANT
// to leave a WAIT_GRANT-style state.
//
// Simple FSM:
//   IDLE    -> req_btn pulse latches address/data/direction, asserts HREQ,
//              moves to REQUEST.
//   REQUEST -> HREQ/HVALID held high until HGRANT arrives. The cycle
//              HGRANT is seen, HRESP is already valid for that same
//              transfer:
//                - HRESP_SPLIT  -> release the bus, set split_pending,
//                                  return to IDLE.
//                - otherwise    -> capture HRDATA, return to IDLE.
//
// While split_pending is set, HREQ is held so the arbiter knows this
// master is still owed a transfer. The arbiter's forced "resume" grant
// asserts HGRANT again independently of state_q; that resume is checked
// first, every cycle, regardless of what state_q is doing.
// =============================================================================
import bus_pkg::*;

module master #(
  parameter int MASTER_ID = 0
)(
  input  logic                   clk,
  input  logic                   rst_n,

  // simple stimulus interface (switches/buttons on DE2-115, or a testbench)
  input  logic                   req_btn,     // 1-cycle pulse: start a transfer
  input  logic [ADDR_WIDTH-1:0]  addr_in,
  input  logic [DATA_WIDTH-1:0]  wdata_in,
  input  logic                   write_in,    // 1 = write, 0 = read

  // AHB-style master port
  output logic                   hreq,
  output logic [ADDR_WIDTH-1:0]  haddr,
  output logic [DATA_WIDTH-1:0]  hwdata,
  output logic                   hwrite,
  output logic                   hvalid,

  input  logic                   hgrant,
  input  logic [DATA_WIDTH-1:0]  hrdata,
  input  logic [1:0]             hresp,
  input  logic                   hsplit_notify,

  // status back to the demo / testbench
  output logic [DATA_WIDTH-1:0]  rdata_out,
  output logic                   busy,
  output logic                   split_pending
);

  typedef enum logic [0:0] {IDLE, REQUEST} state_e;
  state_e state_q;

  logic [ADDR_WIDTH-1:0] addr_q;
  logic [DATA_WIDTH-1:0] wdata_q;
  logic                  write_q;
  logic                  split_pending_q;

  assign hreq   = (state_q == REQUEST) || split_pending_q;
  assign haddr  = addr_q;
  assign hwdata = wdata_q;
  assign hwrite = write_q;
  assign hvalid = (state_q == REQUEST);
  assign busy   = (state_q != IDLE) || split_pending_q;
  assign split_pending = split_pending_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q         <= IDLE;
      addr_q          <= '0;
      wdata_q         <= '0;
      write_q         <= 1'b0;
      split_pending_q <= 1'b0;
      rdata_out       <= '0;
    end else if (split_pending_q && hgrant) begin
      // Forced resume grant from the arbiter: the split resource is ready
      // and this is the cycle its result/completion is delivered.
      rdata_out       <= hrdata;
      split_pending_q <= 1'b0;
      // state_q is already IDLE here (we left it there when the split
      // was first taken), nothing else to update.
    end else begin
      unique case (state_q)
        IDLE: begin
          if (req_btn) begin
            addr_q  <= addr_in;
            wdata_q <= wdata_in;
            write_q <= write_in;
            state_q <= REQUEST;
          end
        end

        REQUEST: begin
          if (hgrant) begin
            // Granted this cycle == the transfer itself; HRESP/HRDATA
            // are already valid for it right now.
            if (hresp == HRESP_SPLIT) begin
              split_pending_q <= 1'b1;
              state_q         <= IDLE;
            end else begin
              rdata_out <= hrdata;
              state_q   <= IDLE;
            end
          end
          // else: not granted yet, keep HREQ/HVALID asserted and wait
        end

        default: state_q <= IDLE;
      endcase
    end
  end

endmodule : master