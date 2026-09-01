// =============================================================================
// bus_mux.sv
// Single shared multiplexer block:
//   - Master -> bus: drives the granted master's HADDR/HWDATA/HWRITE/HVALID
//                    onto the shared lines going to the slaves.
//   - Slave  -> master: routes the selected slave's HRDATA/HRESP back onto
//                    the shared return lines seen by the masters.
// =============================================================================
import bus_pkg::*;

module bus_mux (
  input  logic [NUM_MASTERS-1:0]                  hgrant,
  input  logic [NUM_MASTERS-1:0][ADDR_WIDTH-1:0]  m_haddr,
  input  logic [NUM_MASTERS-1:0][DATA_WIDTH-1:0]  m_hwdata,
  input  logic [NUM_MASTERS-1:0]                  m_hwrite,
  input  logic [NUM_MASTERS-1:0]                  m_hvalid,

  output logic [ADDR_WIDTH-1:0]                   s_haddr,
  output logic [DATA_WIDTH-1:0]                   s_hwdata,
  output logic                                    s_hwrite,
  output logic                                    s_hvalid,

  input  logic [NUM_SLAVES-1:0][DATA_WIDTH-1:0]   s_hrdata,
  input  logic [NUM_SLAVES-1:0][1:0]              s_hresp,
  input  logic [NUM_SLAVES-1:0]                   hsel,

  output logic [DATA_WIDTH-1:0]                   m_hrdata,
  output logic [1:0]                              m_hresp
);

  // ---- Master -> shared bus ----
  always_comb begin
    s_haddr  = '0;
    s_hwdata = '0;
    s_hwrite = 1'b0;
    s_hvalid = 1'b0;
    for (int i = 0; i < NUM_MASTERS; i++) begin
      if (hgrant[i]) begin
        s_haddr  = m_haddr[i];
        s_hwdata = m_hwdata[i];
        s_hwrite = m_hwrite[i];
        s_hvalid = m_hvalid[i];
      end
    end
  end

  // ---- Selected slave -> shared return bus ----
  always_comb begin
    m_hrdata = '0;
    m_hresp  = HRESP_OKAY;
    for (int j = 0; j < NUM_SLAVES; j++) begin
      if (hsel[j]) begin
        m_hrdata = s_hrdata[j];
        m_hresp  = s_hresp[j];
      end
    end
  end

endmodule : bus_mux