// =============================================================================
// master2_stimulus.sv
// Hardcoded instruction sequencer for Master 2. Same role/interface as
// master1_stimulus.sv, targeting Slave 2 with a simple write/read pair and
// no split step. Its IDLE_GAP differs slightly from Master 1's on purpose
// so the two masters' requests will periodically land in the same cycle,
// exercising the arbiter's priority logic (Master 1 wins) live on the
// board without needing any manual input.
//
// Hardcoded program (targets Slave 2, 0x1000-0x1FFF):
//   Step 0: WRITE 0x1000 <= 0x3C
//   Step 1: READ  0x1000
// =============================================================================
import bus_pkg::*;

module master2_stimulus #(
  parameter int IDLE_GAP = 5     // deliberately different from Master 1's gap
)(
  input  logic                   clk,
  input  logic                   rst_n,
  input  logic                   busy,

  output logic                   req_pulse,
  output logic [ADDR_WIDTH-1:0]  addr_out,
  output logic [DATA_WIDTH-1:0]  wdata_out,
  output logic                   write_out
);

  localparam int NUM_STEPS = 2;

  localparam logic [ADDR_WIDTH-1:0] ADDR_TBL  [0:NUM_STEPS-1] = '{16'h1000, 16'h1000};
  localparam logic [DATA_WIDTH-1:0] DATA_TBL  [0:NUM_STEPS-1] = '{8'h3C,    8'h00   };
  localparam logic                  WRITE_TBL [0:NUM_STEPS-1] = '{1'b1,     1'b0    };

  typedef enum logic [1:0] {ST_GAP, ST_ISSUE, ST_BUSY_WAIT} state_e;
  state_e state_q;

  localparam int GAP_W  = (IDLE_GAP <= 1) ? 1 : $clog2(IDLE_GAP + 1);
  localparam int STEP_W = (NUM_STEPS <= 1) ? 1 : $clog2(NUM_STEPS);

  logic [GAP_W-1:0]  gap_cnt_q;
  logic [STEP_W-1:0] step_q;
  logic              seen_busy_q;

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
      req_pulse <= 1'b0;

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

endmodule : master2_stimulus