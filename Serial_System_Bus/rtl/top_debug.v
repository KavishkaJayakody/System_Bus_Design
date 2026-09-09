// Synthesis top: the bus system plus its JTAG In-System Sources and Probes
// front-end.  Wires, two instances and one assign - no logic.
// MUST be TOP_LEVEL_ENTITY, or there is no ISSP in the bitstream at all.

`include "bus_defs.vh"

module top_debug #(
    parameter SPLIT_LATENCY = 10_000_000,
    parameter CLKS_PER_BIT  = 434,
    parameter RESP_TIMEOUT  = 500000
) (
    input  wire       CLOCK_50,
    // Three reset buttons, one per domain.  KEY is active low with a pull-up.
    input  wire       rst_n,       // KEY[0] M23 - system_bus + the bridge
    input  wire       rst_m_n,     // KEY[1] M21 - the two local masters
    input  wire       rst_s_n,     // KEY[2] N21 - the three memory slaves
    input  wire       rm_rx,
    output wire       rm_tx,
    output wire [7:0] led
);

    localparam NM     = `BUS_N_MASTERS;   // bus masters, incl. the bridge
    localparam NLM    = NM - 1;           // masters with a command port
    localparam NS     = `BUS_N_SLAVES;    // decoded targets, incl. the bridge
    localparam ID_W   = 1;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    wire clk = CLOCK_50;

    wire [NLM-1:0]        cmd_valid, cmd_we, cmd_accept, done, err, mst_busy;
    wire [NLM*ADDR_W-1:0] cmd_addr_flat;
    wire [NLM*DATA_W-1:0] cmd_wdata_flat, rdata_flat;
    wire [NLM*RESP_W-1:0] resp_flat;
    wire [NLM*8-1:0]      split_count_flat;
    wire                 split_en;
    wire                 cmd_error, remote_busy, srv_busy;
    wire [7:0]           dbg_rx_last, dbg_rx_count, dbg_tx_count;
    wire [1:0]           dbg_rx_state;
    wire                 dbg_req_seen, dbg_resp_seen, dbg_rx_active;
    wire                 dbg_req_overrun;

    wire [NM-1:0]     gnt, split_mask;
    wire              gnt_valid;
    wire [ID_W-1:0]   master_id;
    wire [NS:0]       sel_q;
    wire              bus_valid, bus_we, bus_ready, split_busy;
    wire              bus_astream, bus_dstream, addr_done;
    wire [ADDR_W-1:0] bus_addr;
    wire [RESP_W-1:0] bus_resp;

    bus_issp_driver #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)
    ) u_dbg (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid        (cmd_valid),
        .cmd_we           (cmd_we),
        .cmd_addr_flat    (cmd_addr_flat),
        .cmd_wdata_flat   (cmd_wdata_flat),
        .cmd_accept       (cmd_accept),
        .done             (done),
        .rdata_flat       (rdata_flat),
        .resp_flat        (resp_flat),
        .err              (err),
        .split_count_flat (split_count_flat),
        .gnt              (gnt),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .split_busy       (split_busy),
        .bus_addr         (bus_addr),
        .bus_valid        (bus_valid),
        .cmd_error        (cmd_error),
        .remote_busy      (remote_busy),
        .srv_busy         (srv_busy),
        .dbg_rx_last      (dbg_rx_last),
        .dbg_rx_count     (dbg_rx_count),
        .dbg_tx_count     (dbg_tx_count),
        .dbg_rx_state     (dbg_rx_state),
        .dbg_req_seen     (dbg_req_seen),
        .dbg_resp_seen    (dbg_resp_seen),
        .dbg_rx_active    (dbg_rx_active),
        .dbg_req_overrun  (dbg_req_overrun),
        .issp_mode        (),
        .split_en         (split_en)
    );

    bus_top #(
        .NM(NM), .NS(NS), .ID_W(ID_W),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPLIT_LATENCY),
        .CLKS_PER_BIT(CLKS_PER_BIT), .RESP_TIMEOUT(RESP_TIMEOUT)
    ) u_bus (
        .clk              (clk),
        .rst_n            (rst_n),
        .rst_m_n          (rst_m_n),
        .rst_s_n          (rst_s_n),

        .cmd_valid        (cmd_valid),
        .cmd_we           (cmd_we),
        .cmd_addr_flat    (cmd_addr_flat),
        .cmd_wdata_flat   (cmd_wdata_flat),
        .cmd_accept       (cmd_accept),
        .done             (done),
        .rdata_flat       (rdata_flat),
        .resp_flat        (resp_flat),
        .err              (err),
        .split_count_flat (split_count_flat),
        .mst_busy         (mst_busy),

        .split_en         (split_en),

        .cmd_error        (cmd_error),
        .rm_rx            (rm_rx),
        .rm_tx            (rm_tx),
        .remote_busy      (remote_busy),
        .srv_busy         (srv_busy),
        .dbg_rx_last      (dbg_rx_last),
        .dbg_rx_count     (dbg_rx_count),
        .dbg_tx_count     (dbg_tx_count),
        .dbg_rx_state     (dbg_rx_state),
        .dbg_req_seen     (dbg_req_seen),
        .dbg_resp_seen    (dbg_resp_seen),
        .dbg_rx_active    (dbg_rx_active),
        .dbg_req_overrun  (dbg_req_overrun),

        .gnt              (gnt),
        .gnt_valid        (gnt_valid),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .split_busy       (split_busy),
        .master_id        (master_id),
        .bus_valid        (bus_valid),
        .bus_we           (bus_we),
        .bus_ready        (bus_ready),
        .bus_resp         (bus_resp),
        .bus_addr         (bus_addr),
        .addr_done        (addr_done),
        .bus_astream      (bus_astream),
        .bus_dstream      (bus_dstream)
    );

    assign led = rdata_flat[0*DATA_W +: DATA_W];

endmodule
