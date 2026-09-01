// =============================================================================
// master1_stimulus.sv
// Hardcoded instruction sequencer for Master 1. Stands in for the UART
// command source that will replace it later -- it drives the exact same
// req_pulse/addr/wdata/write interface a UART parser would, so swapping
// this module out later is a drop-in change.
//
// Self-paced: issues one step, waits for `busy` to rise then fall again
// (i.e. the transaction -- including a full split round-trip, if any --
// has completed), waits IDLE_GAP cycles, then issues the next step. Loops
// forever.
//
// Hardcoded program (all addresses target Slave 1, 0x0000-0x0FFF):
//   Step 0: WRITE 0x0010 <= 0xA5                (baseline data)
//   Step 1: READ  0x0010                        (normal read-back)
//   Step 2: WRITE 0x0030 <= 0x77                (baseline data for split test)
//   Step 3: READ  0x0030, force_split asserted  (drives the split scenario)
// =============================================================================
import bus_pkg::*;

module master1_stimulus #(
  parameter int IDLE_GAP = 6     // cycles to wait after a step fully completes
)(
  input  logic                   clk,
  input  logic                   rst_n,
  input  logic                   busy,          // from this master's master.sv instance

  output logic                   req_pulse,
  output logic [ADDR_WIDTH-1:0]  addr_out,
  output logic [DATA_WIDTH-1:0]  wdata_out,
  output logic                   write_out,
  output logic                   force_split_out
);

  localparam int NUM_STEPS = 4;

  localparam logic [ADDR_WIDTH-1:0] ADDR_TBL  [0:NUM_STEPS-1] = '{16'h0010, 16'h0010, 16'h0030, 16'h0030};
  localparam logic [DATA_WIDTH-1:0] DATA_TBL  [0:NUM_STEPS-1] = '{8'hA5,    8'h00,    8'h77,    8'h00   };
  localparam logic                  WRITE_TBL [0:NUM_STEPS-1] = '{1'b1,     1'b0,     1'b1,     1'b0    };
  localparam logic                  SPLIT_TBL [0:NUM_STEPS-1] = '{1'b0,     1'b0,     1'b0,     1'b1    };

  typedef enum logic [1:0] {ST_GAP, ST_ISSUE, ST_BUSY_WAIT} state_e;
  state_e state_q;

  localparam int GAP_W = (IDLE_GAP <= 1) ? 1 : $clog2(IDLE_GAP + 1);
  localparam int STEP_W = (NUM_STEPS <= 1) ? 1 : $clog2(NUM_STEPS);

  logic [GAP_W-1:0]  gap_cnt_q;
  logic [STEP_W-1:0] step_q;
  logic              seen_busy_q;

  assign force_split_out = SPLIT_TBL[step_q] && (state_q != ST_GAP);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q     <= ST_GAP;
      gap_cnt_q   <= GAP_W'(IDLE_GAP);
      step_q      <= '0;
      seen_busy_q <= 1'b0;
      req_pulse   <= 1'b0;
      addr_out    <= '0;
      wdata_out   <= '0;
      write_out   <= 1'b0;
    end else begin
      req_pulse <= 1'b0;   // default: one-shot pulse

      unique case (state_q)
        ST_GAP: begin
          if (gap_cnt_q == 0) begin
            state_q <= ST_ISSUE;
          end else begin
            gap_cnt_q <= gap_cnt_q - 1'b1;
          end
        end

        ST_ISSUE: begin
          req_pulse   <= 1'b1;
          addr_out    <= ADDR_TBL[step_q];
          wdata_out   <= DATA_TBL[step_q];
          write_out   <= WRITE_TBL[step_q];
          seen_busy_q <= 1'b0;
          state_q     <= ST_BUSY_WAIT;
        end

        ST_BUSY_WAIT: begin
          if (busy) seen_busy_q <= 1'b1;
          if (seen_busy_q && !busy) begin
            step_q    <= (step_q == NUM_STEPS - 1) ? '0 : step_q + 1'b1;
            gap_cnt_q <= GAP_W'(IDLE_GAP);
            state_q   <= ST_GAP;
          end
        end

        default: state_q <= ST_GAP;
      endcase
    end
  end

endmodule : master1_stimulus