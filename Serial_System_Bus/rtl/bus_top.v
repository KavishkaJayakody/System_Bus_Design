//==========================================================================
// bus_top.v
//
// Integration of the complete shared bus: N_MASTERS masters, the arbiter,
// the address decoder, both multiplexers, three memory slaves and the
// default slave.  Board-independent - de2_top wraps this with clocking,
// switches and displays, and the testbenches drive these same command ports
// directly.
//
//   master[i] --+                                    +-- slave_mem 0 (4K, split)
//               |   arbiter --> gnt                  |
//               +-> bus_mux (fwd) --> addr_decoder --+-- slave_mem 1 (4K)
//                        ^                           |
//                        |                           +-- slave_mem 2 (2K)
//                   bus_mux (ret) <- sel_q -----------+
//                                                     +-- default_slave
//
// Everything below runs on one clock with one asynchronous active-low reset.
// There is no clock gating and no second domain.
//
// The master command interfaces are flattened vectors (Verilog-2001 has no
// arrays of ports): master i occupies bits [i*W +: W].
//
//--------------------------------------------------------------------------
// Port              Dir  Width              Meaning
//--------------------------------------------------------------------------
// clk               in   1                  Bus clock.
// rst_n             in   1                  Asynchronous active-low reset.
// cmd_valid         in   N_MASTERS          Per-master: a command is offered.
// cmd_we            in   N_MASTERS          Per-master: 1 = write.
// cmd_addr_flat     in   N_MASTERS*ADDR_W   Per-master command address.
// cmd_wdata_flat    in   N_MASTERS*DATA_W   Per-master command write data.
// cmd_accept        out  N_MASTERS          Per-master: command latched.
// done              out  N_MASTERS          Per-master: transaction finished.
// rdata_flat        out  N_MASTERS*DATA_W   Per-master last read data.
// resp_flat         out  N_MASTERS*RESP_W   Per-master last response.
// err               out  N_MASTERS          Per-master last response = ERROR.
// split_count_flat  out  N_MASTERS*8        Per-master count of splits seen.
// mst_busy          out  N_MASTERS          Per-master FSM not idle.
// s0_split_en       in   1                  1 = slave 0 behaves as busy and
//                                           splits fresh accesses.
// gnt               out  N_MASTERS          One-hot grant (status).
// gnt_valid         out  1                  A master owns the bus (status).
// master_id         out  ID_W               Granted master index (status).
// split_mask        out  N_MASTERS          Arbiter split mask (status).
// sel_q             out  N_SLAVES+1         Registered responder select,
//                                           {default,s2,s1,s0} (status).
// bus_valid         out  1                  Forward transfer strobe (status).
// bus_addr          out  ADDR_W             Address on the bus (status).
// bus_ready         out  1                  Return completion strobe (status).
// bus_resp          out  RESP_W             Return response (status).
// bus_rdata         out  DATA_W             Return read data (status).
// s0_busy           out  1                  Slave 0 has a split in flight.
//==========================================================================
`include "bus_defs.vh"

module bus_top #(
    parameter N_MASTERS     = `BUS_N_MASTERS,
    parameter ID_W          = 1,               // ceil(log2(N_MASTERS))
    parameter N_SLAVES      = `BUS_N_SLAVES,
    parameter ADDR_W        = `BUS_ADDR_W,
    parameter DATA_W        = `BUS_DATA_W,
    parameter RESP_W        = `BUS_RESP_W,
    parameter SPLIT_LATENCY = 4
) (
    input  wire                            clk,
    input  wire                            rst_n,

    // ---- master command interfaces --------------------------------------
    input  wire [N_MASTERS-1:0]            cmd_valid,
    input  wire [N_MASTERS-1:0]            cmd_we,
    input  wire [N_MASTERS*ADDR_W-1:0]     cmd_addr_flat,
    input  wire [N_MASTERS*DATA_W-1:0]     cmd_wdata_flat,
    output wire [N_MASTERS-1:0]            cmd_accept,
    output wire [N_MASTERS-1:0]            done,
    output wire [N_MASTERS*DATA_W-1:0]     rdata_flat,
    output wire [N_MASTERS*RESP_W-1:0]     resp_flat,
    output wire [N_MASTERS-1:0]            err,
    output wire [N_MASTERS*8-1:0]          split_count_flat,
    output wire [N_MASTERS-1:0]            mst_busy,

    // ---- slave 0 split control ------------------------------------------
    input  wire                            s0_split_en,

    // ---- observability ---------------------------------------------------
    output wire [N_MASTERS-1:0]            gnt,
    output wire                            gnt_valid,
    output wire [ID_W-1:0]                 master_id,
    output wire [N_MASTERS-1:0]            split_mask,
    output wire [N_SLAVES:0]               sel_q,
    output wire                            bus_valid,
    output wire [ADDR_W-1:0]               bus_addr,
    output wire                            bus_ready,
    output wire [RESP_W-1:0]               bus_resp,
    output wire [DATA_W-1:0]               bus_rdata,
    output wire                            s0_busy
);

    //----------------------------------------------------------------------
    // Master <-> forward-mux wiring
    //----------------------------------------------------------------------
    wire [N_MASTERS-1:0]         m_req;
    wire [N_MASTERS-1:0]         m_valid;
    wire [N_MASTERS-1:0]         m_we;
    wire [N_MASTERS*ADDR_W-1:0]  m_addr_flat;
    wire [N_MASTERS*DATA_W-1:0]  m_wdata_flat;

    wire                         bus_we;
    wire [DATA_W-1:0]            bus_wdata;

    //----------------------------------------------------------------------
    // Masters
    //----------------------------------------------------------------------
    genvar gi;
    generate
    for (gi = 0; gi < N_MASTERS; gi = gi + 1) begin : g_master
        master #(
            .ADDR_W (ADDR_W),
            .DATA_W (DATA_W),
            .RESP_W (RESP_W)
        ) u_master (
            .clk         (clk),
            .rst_n       (rst_n),
            .cmd_valid   (cmd_valid[gi]),
            .cmd_we      (cmd_we[gi]),
            .cmd_addr    (cmd_addr_flat  [gi*ADDR_W +: ADDR_W]),
            .cmd_wdata   (cmd_wdata_flat [gi*DATA_W +: DATA_W]),
            .cmd_accept  (cmd_accept[gi]),
            .done        (done[gi]),
            .rdata       (rdata_flat     [gi*DATA_W +: DATA_W]),
            .resp        (resp_flat      [gi*RESP_W +: RESP_W]),
            .err         (err[gi]),
            .split_count (split_count_flat[gi*8 +: 8]),
            .busy        (mst_busy[gi]),
            .state       (),                     // waveform only
            .bus_req     (m_req[gi]),
            .bus_gnt     (gnt[gi]),
            .m_valid     (m_valid[gi]),
            .m_we        (m_we[gi]),
            .m_addr      (m_addr_flat    [gi*ADDR_W +: ADDR_W]),
            .m_wdata     (m_wdata_flat   [gi*DATA_W +: DATA_W]),
            .bus_ready   (bus_ready),
            .bus_resp    (bus_resp),
            .bus_rdata   (bus_rdata)
        );
    end
    endgenerate

    //----------------------------------------------------------------------
    // Arbiter.  split_complete is the OR of every split-capable slave's
    // wake-up vector; only slave 0 can raise one today.
    //----------------------------------------------------------------------
    wire [N_MASTERS-1:0] s0_split_complete;
    wire [N_MASTERS-1:0] split_complete = s0_split_complete;

    arbiter #(
        .N_MASTERS (N_MASTERS),
        .ID_W      (ID_W),
        .RESP_W    (RESP_W)
    ) u_arbiter (
        .clk            (clk),
        .rst_n          (rst_n),
        .req            (m_req),
        .bus_ready      (bus_ready),
        .bus_resp       (bus_resp),
        .split_complete (split_complete),
        .gnt            (gnt),
        .gnt_valid      (gnt_valid),
        .master_id      (master_id),
        .split_mask     (split_mask),
        .locked         ()
    );

    //----------------------------------------------------------------------
    // Forward / return multiplexers
    //----------------------------------------------------------------------
    wire [N_SLAVES-1:0]              slv_sel;
    wire                             def_sel;
    wire [N_SLAVES:0]                sel = {def_sel, slv_sel};

    wire [N_SLAVES:0]                s_ready;
    wire [(N_SLAVES+1)*RESP_W-1:0]   s_resp_flat;
    wire [(N_SLAVES+1)*DATA_W-1:0]   s_rdata_flat;

    bus_mux #(
        .N_MASTERS (N_MASTERS),
        .N_SLAVES  (N_SLAVES),
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W),
        .RESP_W    (RESP_W)
    ) u_bus_mux (
        .clk          (clk),
        .rst_n        (rst_n),
        .gnt          (gnt),
        .m_valid      (m_valid),
        .m_we         (m_we),
        .m_addr_flat  (m_addr_flat),
        .m_wdata_flat (m_wdata_flat),
        .bus_valid    (bus_valid),
        .bus_we       (bus_we),
        .bus_addr     (bus_addr),
        .bus_wdata    (bus_wdata),
        .sel          (sel),
        .s_ready      (s_ready),
        .s_resp_flat  (s_resp_flat),
        .s_rdata_flat (s_rdata_flat),
        .bus_ready    (bus_ready),
        .bus_resp     (bus_resp),
        .bus_rdata    (bus_rdata),
        .sel_q        (sel_q)
    );

    //----------------------------------------------------------------------
    // Address decoder.  Enabled only while a transfer is actually being
    // driven, so an idle bus selects nothing at all.
    //----------------------------------------------------------------------
    addr_decoder #(
        .ADDR_W   (ADDR_W),
        .N_SLAVES (N_SLAVES)
    ) u_decoder (
        .en      (bus_valid),
        .addr    (bus_addr),
        .slv_sel (slv_sel),
        .def_sel (def_sel),
        .hit     ()
    );

    //----------------------------------------------------------------------
    // Slave 0 - 4K words at 0x0000, split capable
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S0_LADDR_W),
        .WORDS         (`S0_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (1),
        .SPLIT_LATENCY (SPLIT_LATENCY)
    ) u_slave0 (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (slv_sel[`SEL_S0]),
        .we             (bus_we),
        .addr           (bus_addr[`S0_LADDR_W-1:0]),
        .wdata          (bus_wdata),
        .master_id      (master_id),
        .split_en       (s0_split_en),
        .rdata          (s_rdata_flat[`SEL_S0*DATA_W +: DATA_W]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (s0_split_complete),
        .busy           (s0_busy)
    );

    //----------------------------------------------------------------------
    // Slave 1 - 4K words at 0x1000
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S1_LADDR_W),
        .WORDS         (`S1_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave1 (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (slv_sel[`SEL_S1]),
        .we             (bus_we),
        .addr           (bus_addr[`S1_LADDR_W-1:0]),
        .wdata          (bus_wdata),
        .master_id      (master_id),
        .split_en       (1'b0),
        .rdata          (s_rdata_flat[`SEL_S1*DATA_W +: DATA_W]),
        .ready          (s_ready[`SEL_S1]),
        .resp           (s_resp_flat[`SEL_S1*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    //----------------------------------------------------------------------
    // Slave 2 - 2K words at 0x2000
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave2 (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (slv_sel[`SEL_S2]),
        .we             (bus_we),
        .addr           (bus_addr[`S2_LADDR_W-1:0]),
        .wdata          (bus_wdata),
        .master_id      (master_id),
        .split_en       (1'b0),
        .rdata          (s_rdata_flat[`SEL_S2*DATA_W +: DATA_W]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    //----------------------------------------------------------------------
    // Default slave - everything unmapped, answers ERROR so the bus never
    // stalls on a bad address.
    //----------------------------------------------------------------------
    default_slave #(
        .DATA_W (DATA_W),
        .RESP_W (RESP_W)
    ) u_default (
        .clk   (clk),
        .rst_n (rst_n),
        .sel   (def_sel),
        .rdata (s_rdata_flat[`SEL_DEF*DATA_W +: DATA_W]),
        .ready (s_ready[`SEL_DEF]),
        .resp  (s_resp_flat[`SEL_DEF*RESP_W +: RESP_W])
    );

endmodule
