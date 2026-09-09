// THE SYSTEM: 2 local masters + system_bus + 3 memory slaves + the remote
// bridge.  Instantiation and wiring ONLY - no logic, so bus behaviour cannot
// hide here.  Three bus masters but only NLM = NM-1 command ports; the
// bridge's master face is driven by the far board.

`include "bus_defs.vh"

module bus_top #(
    parameter NM            = `BUS_N_MASTERS,   // bus masters, incl. the bridge
    // Locally commanded masters, DERIVED from NM: the last bus master is the
    // bridge and has no command port.  It sizes the ports below, so it has to
    // be declared here - a localparam in the body is declared too late for a
    // port width, which iverilog tolerates and ModelSim rejects outright.
    parameter NLM           = NM - 1,
    parameter NS            = `BUS_N_SLAVES,    // decoded targets, incl. the bridge
    parameter ID_W          = `BUS_ID_W,
    parameter ADDR_W        = `BUS_ADDR_W,
    parameter DATA_W        = `BUS_DATA_W,
    parameter RESP_W        = `BUS_RESP_W,
    parameter SPLIT_LATENCY = 10_000_000,
    parameter CLKS_PER_BIT  = 434,
    parameter RESP_TIMEOUT  = 500000,
    // DEBUG ONLY: 0 removes the last parallel address anywhere in the design.
    parameter OBSERVE_ADDR  = 1
) (
    input  wire                    clk,
    // THREE RESET DOMAINS, one per KEY.  rst_n also covers the bridge: it is
    // the link, not a memory, and it has a master face as well as a slave one.
    input  wire                    rst_n,     // KEY[0] - system_bus + bridge
    input  wire                    rst_m_n,   // KEY[1] - the local masters
    input  wire                    rst_s_n,   // KEY[2] - the memory slaves

    input  wire [NLM-1:0]          cmd_valid,
    input  wire [NLM-1:0]          cmd_we,
    input  wire [NLM*ADDR_W-1:0]   cmd_addr_flat,
    input  wire [NLM*DATA_W-1:0]   cmd_wdata_flat,
    output wire [NLM-1:0]          cmd_accept,
    output wire [NLM-1:0]          done,
    output wire [NLM*DATA_W-1:0]   rdata_flat,
    output wire [NLM*RESP_W-1:0]   resp_flat,
    output wire [NLM-1:0]          err,
    output wire [NLM*8-1:0]        split_count_flat,
    output wire [NLM-1:0]          mst_busy,

    input  wire                    split_en,

    output wire                    cmd_error,      // remote read timed out
    input  wire                    rm_rx,
    output wire                    rm_tx,
    output wire                    remote_busy,
    output wire                    srv_busy,
    output wire [7:0]              dbg_rx_last,
    output wire [7:0]              dbg_rx_count,
    output wire [7:0]              dbg_tx_count,
    output wire [1:0]              dbg_rx_state,
    output wire                    dbg_req_seen,
    output wire                    dbg_resp_seen,
    output wire                    dbg_rx_active,
    output wire                    dbg_req_overrun,

    output wire [NM-1:0]           gnt,
    output wire                    gnt_valid,
    output wire [NM-1:0]           split_mask,
    output wire [NS:0]             sel_q,
    output wire                    split_busy,
    output wire [ID_W-1:0]         master_id,
    output wire                    bus_valid,
    output wire                    bus_we,
    output wire                    bus_ready,
    output wire [RESP_W-1:0]       bus_resp,
    output wire [ADDR_W-1:0]       bus_addr,
    output wire                    addr_done,
    output wire                    bus_astream,
    output wire                    bus_dstream
);

    wire [NM-1:0]  m_req;
    wire [NM-1:0]  m_valid;
    wire [NM-1:0]  m_we;
    wire [NM-1:0]  m_astream;
    wire [NM-1:0]  m_dstream;

    wire [NS-1:0]          s_sel;
    wire [NS-1:0]          s_ready;
    wire [NS*RESP_W-1:0]   s_resp_flat;
    wire [NS-1:0]          s_dstream;

    genvar gi;
    generate
    for (gi = 0; gi < NLM; gi = gi + 1) begin : g_master
        master #(
            .ADDR_W (ADDR_W),
            .DATA_W (DATA_W),
            .RESP_W (RESP_W)
        ) u_master (
            .clk         (clk),
            .rst_n       (rst_m_n),
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
            .m_astream   (m_astream[gi]),        // ADDRESS, one wire
            .m_dstream   (m_dstream[gi]),        // WRITE DATA, one wire
            .bus_ready   (bus_ready),
            .bus_resp    (bus_resp),
            .bus_dstream (bus_dstream)           // READ DATA, the shared wire
        );
    end
    endgenerate

    wire [NM-1:0] s2_split_complete;
    wire [NM-1:0] br_split_complete;

    wire [NM-1:0] s_split_complete = s2_split_complete | br_split_complete;

    system_bus #(
        .N_MASTERS    (NM),
        .ID_W         (ID_W),
        .N_SLAVES     (NS),
        .ADDR_W       (ADDR_W),
        .RESP_W       (RESP_W),
        .OBSERVE_ADDR (OBSERVE_ADDR)
    ) u_system_bus (
        .clk              (clk),
        .rst_n            (rst_n),

        .m_req            (m_req),
        .m_gnt            (gnt),
        .m_valid          (m_valid),
        .m_we             (m_we),
        .m_astream        (m_astream),
        .m_dstream        (m_dstream),
        .bus_ready        (bus_ready),
        .bus_resp         (bus_resp),

        .bus_valid        (bus_valid),
        .bus_we           (bus_we),
        .bus_master_id    (master_id),
        .s_sel            (s_sel),
        .s_ready          (s_ready),
        .s_resp_flat      (s_resp_flat),
        .s_dstream        (s_dstream),
        .s_split_complete (s_split_complete),

        .bus_astream      (bus_astream),
        .bus_dstream      (bus_dstream),

        .gnt_valid        (gnt_valid),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .bus_addr         (bus_addr),
        .addr_done        (addr_done)
    );

    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S0_LADDR_W),
        .WORDS         (`S0_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave0 (
        .clk            (clk),
        .rst_n          (rst_s_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S0]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S0]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S1_LADDR_W),
        .WORDS         (`S1_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave1 (
        .clk            (clk),
        .rst_n          (rst_s_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S1]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S1]),
        .ready          (s_ready[`SEL_S1]),
        .resp           (s_resp_flat[`SEL_S1*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (1),
        .SPLIT_LATENCY (SPLIT_LATENCY)
    ) u_slave2 (
        .clk            (clk),
        .rst_n          (rst_s_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S2]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (split_en),
        .dstream_out    (s_dstream[`SEL_S2]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (s2_split_complete),
        .busy           (split_busy)
    );

    bus_bridge #(
        .ADDR_W       (ADDR_W),
        .DATA_W       (DATA_W),
        .RESP_W       (RESP_W),
        .N_MASTERS    (NM),
        .ID_W         (ID_W),
        .LADDR_W      (`S3_LADDR_W),
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .RESP_TIMEOUT (RESP_TIMEOUT)
    ) u_bridge (
        .clk             (clk),
        .rst_n           (rst_n),

        .frame           (bus_valid),
        .astream         (bus_astream),
        .dstream_in      (bus_dstream),
        .sel             (s_sel[`SEL_BR]),
        .we              (bus_we),
        .master_id       (master_id),
        .dstream_out     (s_dstream[`SEL_BR]),
        .ready           (s_ready[`SEL_BR]),
        .resp            (s_resp_flat[`SEL_BR*RESP_W +: RESP_W]),
        .split_complete  (br_split_complete),
        .busy            (),

        // master face - bus master NM-1, the LOWEST arbiter priority, so
        // remote traffic can never out-rank the local masters
        .bus_req         (m_req[NM-1]),
        .bus_gnt         (gnt[NM-1]),
        .m_valid         (m_valid[NM-1]),
        .m_we            (m_we[NM-1]),
        .m_astream       (m_astream[NM-1]),
        .m_dstream       (m_dstream[NM-1]),
        .bus_ready       (bus_ready),
        .bus_resp        (bus_resp),
        .bus_dstream     (bus_dstream),

        .rm_rx           (rm_rx),
        .rm_tx           (rm_tx),

        .br_error        (cmd_error),
        .remote_busy     (remote_busy),
        .srv_busy        (srv_busy),
        .dbg_rx_last     (dbg_rx_last),
        .dbg_rx_count    (dbg_rx_count),
        .dbg_tx_count    (dbg_tx_count),
        .dbg_rx_state    (dbg_rx_state),
        .dbg_req_seen    (dbg_req_seen),
        .dbg_resp_seen   (dbg_resp_seen),
        .dbg_rx_active   (dbg_rx_active),
        .dbg_req_overrun (dbg_req_overrun)
    );

endmodule
