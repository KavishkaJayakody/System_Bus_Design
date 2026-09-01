// =============================================================================
// top_level.sv
// Top-level integration for the DE2-115 board.
//
// Both masters are now driven by hardcoded instruction sequencers
// (master1_stimulus / master2_stimulus) instead of switches and push
// buttons -- this is a placeholder for the UART command source that will
// replace them later. A UART parser can be dropped in later by simply
// replacing these two instances with something that drives the same
// req_pulse/addr/wdata/write interface into the master.sv instances.
//
// The only physical control left is KEY[0], the system reset -- this is
// a board-level control rather than "an instruction to a master", so it
// has been kept as a normal push-button reset for now.
//
// LEDR/LEDG remain as a status display for bring-up / debugging while the
// UART link doesn't exist yet:
//   LEDR[7:0]  : Master 1's last read data
//   LEDR[8]    : Master 1 busy
//   LEDR[9]    : Master 2 busy
//   LEDR[10]   : Master 1 split-pending
//   LEDR[11]   : Master 2 split-pending
//   LEDR[12]   : Slave 1 hsplit_active (locked in a split)
//   LEDR[13]   : HGRANT[0] (Master 1 granted)
//   LEDR[14]   : HGRANT[1] (Master 2 granted)
//   LEDG[7:0]  : Master 2's last read data
// =============================================================================
import bus_pkg::*;

module top_level (
  input  logic        CLOCK_50,
  input  logic [3:0]  KEY,        // only KEY[0] (reset) is used
  output logic [17:0] LEDR,
  output logic [8:0]  LEDG
);

  logic clk;
  logic rst_n;
  assign clk   = CLOCK_50;
  assign rst_n = KEY[0];          // async active-low reset

  // ---------------------------------------------------------------
  // Master <-> arbiter / mux signals
  // ---------------------------------------------------------------
  logic [NUM_MASTERS-1:0]                 hreq, hgrant, hsplit_notify;
  logic [NUM_MASTERS-1:0][ADDR_WIDTH-1:0] m_haddr;
  logic [NUM_MASTERS-1:0][DATA_WIDTH-1:0] m_hwdata;
  logic [NUM_MASTERS-1:0]                 m_hwrite;
  logic [NUM_MASTERS-1:0]                 m_hvalid;

  logic [DATA_WIDTH-1:0] m1_rdata, m2_rdata;
  logic                  m1_busy,  m2_busy;
  logic                  m1_split_pend, m2_split_pend;

  logic [DATA_WIDTH-1:0] mux_hrdata;
  logic [1:0]            mux_hresp;

  // ---------------------------------------------------------------
  // Hardcoded instruction sequencers (stand-in for the future UART source)
  // ---------------------------------------------------------------
  logic                  m1_req_pulse, m2_req_pulse;
  logic [ADDR_WIDTH-1:0] m1_addr,      m2_addr;
  logic [DATA_WIDTH-1:0] m1_wdata,     m2_wdata;
  logic                  m1_write,     m2_write;
  logic                  force_split;

  master1_stimulus u_m1_stim (
    .clk(clk), .rst_n(rst_n), .busy(m1_busy),
    .req_pulse(m1_req_pulse), .addr_out(m1_addr), .wdata_out(m1_wdata), .write_out(m1_write),
    .force_split_out(force_split)
  );

  master2_stimulus u_m2_stim (
    .clk(clk), .rst_n(rst_n), .busy(m2_busy),
    .req_pulse(m2_req_pulse), .addr_out(m2_addr), .wdata_out(m2_wdata), .write_out(m2_write)
  );

  master #(.MASTER_ID(0)) u_master1 (
    .clk(clk), .rst_n(rst_n),
    .req_btn(m1_req_pulse), .addr_in(m1_addr), .wdata_in(m1_wdata), .write_in(m1_write),
    .hreq(hreq[0]), .haddr(m_haddr[0]), .hwdata(m_hwdata[0]), .hwrite(m_hwrite[0]), .hvalid(m_hvalid[0]),
    .hgrant(hgrant[0]), .hrdata(mux_hrdata), .hresp(mux_hresp), .hsplit_notify(hsplit_notify[0]),
    .rdata_out(m1_rdata), .busy(m1_busy), .split_pending(m1_split_pend)
  );

  master #(.MASTER_ID(1)) u_master2 (
    .clk(clk), .rst_n(rst_n),
    .req_btn(m2_req_pulse), .addr_in(m2_addr), .wdata_in(m2_wdata), .write_in(m2_write),
    .hreq(hreq[1]), .haddr(m_haddr[1]), .hwdata(m_hwdata[1]), .hwrite(m_hwrite[1]), .hvalid(m_hvalid[1]),
    .hgrant(hgrant[1]), .hrdata(mux_hrdata), .hresp(mux_hresp), .hsplit_notify(hsplit_notify[1]),
    .rdata_out(m2_rdata), .busy(m2_busy), .split_pending(m2_split_pend)
  );

  // ---------------------------------------------------------------
  // Arbiter
  // ---------------------------------------------------------------
  logic split_pulse;
  logic hsplit_done;
  logic hsplit_active;
  logic hready_unused;

  arbiter u_arbiter (
    .clk(clk), .rst_n(rst_n),
    .hreq(hreq),
    .split_pulse(split_pulse),
    .hsplit_done(hsplit_done),
    .hgrant(hgrant),
    .hsplit_notify(hsplit_notify),
    .hready(hready_unused)
  );

  // ---------------------------------------------------------------
  // Address decoder
  // ---------------------------------------------------------------
  logic [NUM_SLAVES-1:0] hsel;
  logic [ADDR_WIDTH-1:0] s_haddr;

  addr_decoder u_decoder (
    .haddr(s_haddr),
    .hsplit_active(hsplit_active),
    .hsel(hsel)
  );

  // ---------------------------------------------------------------
  // Shared multiplexer block
  // ---------------------------------------------------------------
  logic [DATA_WIDTH-1:0] s_hwdata;
  logic                  s_hwrite, s_hvalid;

  logic [NUM_SLAVES-1:0][DATA_WIDTH-1:0] s_hrdata_arr;
  logic [NUM_SLAVES-1:0][1:0]            s_hresp_arr;

  bus_mux u_mux (
    .hgrant(hgrant),
    .m_haddr(m_haddr), .m_hwdata(m_hwdata), .m_hwrite(m_hwrite), .m_hvalid(m_hvalid),
    .s_haddr(s_haddr), .s_hwdata(s_hwdata), .s_hwrite(s_hwrite), .s_hvalid(s_hvalid),
    .s_hrdata(s_hrdata_arr), .s_hresp(s_hresp_arr), .hsel(hsel),
    .m_hrdata(mux_hrdata), .m_hresp(mux_hresp)
  );

  // ---------------------------------------------------------------
  // Slaves
  // ---------------------------------------------------------------
  slave1_split #(.SPLIT_DELAY(4)) u_slave1 (
    .clk(clk), .rst_n(rst_n),
    .hsel(hsel[0]), .haddr(s_haddr), .hwdata(s_hwdata), .hwrite(s_hwrite), .hvalid(s_hvalid),
    .force_split(force_split),
    .hrdata(s_hrdata_arr[0]), .hresp(s_hresp_arr[0]),
    .hsplit_done(hsplit_done), .hsplit_active(hsplit_active)
  );

  slave_simple #(.SIZE_BYTES(4096)) u_slave2 (
    .clk(clk), .rst_n(rst_n),
    .hsel(hsel[1]), .haddr(s_haddr), .hwdata(s_hwdata), .hwrite(s_hwrite), .hvalid(s_hvalid),
    .hrdata(s_hrdata_arr[1]), .hresp(s_hresp_arr[1])
  );

  slave_simple #(.SIZE_BYTES(2048)) u_slave3 (
    .clk(clk), .rst_n(rst_n),
    .hsel(hsel[2]), .haddr(s_haddr), .hwdata(s_hwdata), .hwrite(s_hwrite), .hvalid(s_hvalid),
    .hrdata(s_hrdata_arr[2]), .hresp(s_hresp_arr[2])
  );

  // SPLIT detected: currently selected slave is Slave 1 and it just
  // returned HRESP_SPLIT on a valid transfer.
  assign split_pulse = s_hvalid && hsel[0] && (s_hresp_arr[0] == HRESP_SPLIT);

  // ---------------------------------------------------------------
  // Status LEDs
  // ---------------------------------------------------------------
  assign LEDR[7:0]   = m1_rdata;
  assign LEDR[8]     = m1_busy;
  assign LEDR[9]     = m2_busy;
  assign LEDR[10]    = m1_split_pend;
  assign LEDR[11]    = m2_split_pend;
  assign LEDR[12]    = hsplit_active;
  assign LEDR[13]    = hgrant[0];
  assign LEDR[14]    = hgrant[1];
  assign LEDR[17:15] = 3'b000;

  assign LEDG[7:0]   = m2_rdata;
  assign LEDG[8]     = force_split;

endmodule : top_level