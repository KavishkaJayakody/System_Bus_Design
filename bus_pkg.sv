// =============================================================================
// bus_pkg.sv
// Shared parameters, types and address map for the mini-AHB-style bus.
// =============================================================================
package bus_pkg;

  parameter int ADDR_WIDTH  = 16;
  parameter int DATA_WIDTH  = 8;
  parameter int NUM_MASTERS = 2;
  parameter int NUM_SLAVES  = 3;

  // HRESP encoding (only OKAY / SPLIT are used; ERROR reserved for future use)
  typedef enum logic [1:0] {
    HRESP_OKAY  = 2'b00,
    HRESP_ERROR = 2'b01,
    HRESP_SPLIT = 2'b10
  } hresp_e;

  // ---------------------------------------------------------------------
  // Address map (byte addresses). HADDR[1:0] is hardwired to 2'b00, so all
  // transfers are naturally word-aligned; the ranges below still work
  // because every base sits on a power-of-two boundary >= its size.
  // ---------------------------------------------------------------------
  parameter logic [ADDR_WIDTH-1:0] S1_BASE = 16'h0000; // Slave 1: 4K, split-capable
  parameter logic [ADDR_WIDTH-1:0] S1_TOP  = 16'h0FFF;
  parameter logic [ADDR_WIDTH-1:0] S2_BASE = 16'h1000; // Slave 2: 4K
  parameter logic [ADDR_WIDTH-1:0] S2_TOP  = 16'h1FFF;
  parameter logic [ADDR_WIDTH-1:0] S3_BASE = 16'h2000; // Slave 3: 2K
  parameter logic [ADDR_WIDTH-1:0] S3_TOP  = 16'h27FF;

endpackage : bus_pkg