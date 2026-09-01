// =============================================================================
// slave1_split.sv
// Slave 1: 4KB memory, split-transaction capable.
//
// Behaviour:
//   - Normal access (force_split = 0): single-cycle read/write, like the
//     other slaves.
//   - When force_split = 1 at the moment of a valid access, the slave:
//       1. Returns HRESP = SPLIT this cycle (does NOT service the access).
//       2. Latches the pending address/data/direction internally.
//       3. Asserts hsplit_active so the address decoder blocks new
//          transfers to this slave.
//       4. After SPLIT_DELAY cycles (modelling the resource becoming
//          free), services the latched access internally and pulses
//          hsplit_done for exactly one cycle.
//       5. For a pending READ, also drives the fetched byte on HRDATA
//          for one extra cycle (the "resume" grant cycle) so the
//          arbiter's forced resume grant can deliver it to the master.
//
// force_split is wired to a DE2-115 switch in the top level so a live
// split scenario can be demonstrated on hardware, in addition to being
// driven from a testbench.
// =============================================================================
import bus_pkg::*;

module slave1_split #(
  parameter int SPLIT_DELAY = 4     // cycles until the split resource is "ready"
)(
  input  logic                   clk,
  input  logic                   rst_n,

  input  logic                   hsel,
  input  logic [ADDR_WIDTH-1:0]  haddr,
  input  logic [DATA_WIDTH-1:0]  hwdata,
  input  logic                   hwrite,
  input  logic                   hvalid,
  input  logic                   force_split,

  output logic [DATA_WIDTH-1:0]  hrdata,
  output logic [1:0]             hresp,
  output logic                   hsplit_done,
  output logic                   hsplit_active
);

  localparam int CNT_W = (SPLIT_DELAY <= 1) ? 1 : $clog2(SPLIT_DELAY + 1);

  logic [DATA_WIDTH-1:0] mem [0:4095];
  logic [11:0]            addr_idx;

  logic                   splitting_q;
  logic                   delivering_q;      // 1-cycle: deliver pend_rdata_q on resume
  logic [CNT_W-1:0]       delay_cnt_q;
  logic [11:0]            pend_addr_q;
  logic                   pend_write_q;
  logic [DATA_WIDTH-1:0]  pend_wdata_q;
  logic [DATA_WIDTH-1:0]  pend_rdata_q;

  assign addr_idx      = haddr[11:0];
  assign hsplit_active = splitting_q;

  // ---- combinational response/read-data ----
  always_comb begin
    hresp  = HRESP_OKAY;
    hrdata = '0;
    if (delivering_q) begin
      hrdata = pend_rdata_q;                       // deliver the split result
    end else if (hsel && hvalid && !splitting_q) begin
      if (force_split) begin
        hresp = HRESP_SPLIT;
      end else begin
        hrdata = mem[addr_idx];
      end
    end
  end

  // ---- sequential: memory + split state machine ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      splitting_q  <= 1'b0;
      delivering_q <= 1'b0;
      delay_cnt_q  <= '0;
      pend_addr_q  <= '0;
      pend_write_q <= 1'b0;
      pend_wdata_q <= '0;
      pend_rdata_q <= '0;
      hsplit_done  <= 1'b0;
    end else begin
      hsplit_done  <= 1'b0;
      delivering_q <= 1'b0;               // one-shot pulse by default

      if (hsel && hvalid && !splitting_q && force_split) begin
        // New access must be split: latch it and start the countdown
        splitting_q  <= 1'b1;
        delay_cnt_q  <= SPLIT_DELAY[CNT_W-1:0];
        pend_addr_q  <= addr_idx;
        pend_write_q <= hwrite;
        pend_wdata_q <= hwdata;
      end else if (splitting_q) begin
        if (delay_cnt_q == 0) begin
          // Resource ready: service the latched access now
          if (pend_write_q) begin
            mem[pend_addr_q] <= pend_wdata_q;
          end else begin
            pend_rdata_q <= mem[pend_addr_q];
            delivering_q <= 1'b1;
          end
          hsplit_done <= 1'b1;
          splitting_q <= 1'b0;
        end else begin
          delay_cnt_q <= delay_cnt_q - 1'b1;
        end
      end else if (hsel && hvalid && hwrite && !force_split) begin
        mem[addr_idx] <= hwdata;
      end
    end
  end

endmodule : slave1_split