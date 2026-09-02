`timescale 1ns / 1ps

module top_bus_system #(
    parameter CLKS_PER_BIT = 434,  // 50 MHz / 115200 baud
    parameter RESP_TIMEOUT = 500000 // remote reply timeout, clocks
)(
    input  wire        clk,
    input  wire        rst_n,

    // Master 0 Interface
    input  wire        m0_cmd_start,
    input  wire        m0_cmd_we,
    input  wire        m0_cmd_remote,   // 1 = run this on the OTHER board
    output wire        m0_cmd_error,    // remote transaction timed out
    input  wire [13:0] m0_cmd_addr,
    input  wire [7:0]  m0_cmd_wdata,
    output wire [7:0]  m0_cmd_rdata,
    output wire        m0_cmd_done,

    // Master 1 Interface
    input  wire        m1_cmd_start,
    input  wire        m1_cmd_we,
    input  wire [13:0] m1_cmd_addr,
    input  wire [7:0]  m1_cmd_wdata,
    output wire [7:0]  m1_cmd_rdata,
    output wire        m1_cmd_done,

    // UART link to the other board. Both directions belong to Master 0 now:
    // it is the client for our remote transactions and the server for theirs.
    input  wire        ext_rx_serial,
    output wire        ext_tx_serial
);

    // Master-Interconnect Signals
    wire req0, req1, gnt0, gnt1;
    wire m0_split_notify, m1_split_notify;
    wire [13:0] m0_addr, m1_addr;
    wire [7:0]  m0_wdata, m1_wdata;
    wire        m0_we, m1_we;
    wire        m0_valid, m1_valid;

    // Shared Interconnect Output Signals
    wire [13:0] bus_addr;
    wire [7:0]  bus_wdata;
    wire        bus_we;
    wire [7:0]  bus_rdata;
    wire        bus_ready;

    // Decoder Selects
    wire sel_s0, sel_s1, sel_s2, sel_bridge;

    // Slave Response Signals
    wire [7:0] s0_rdata, s1_rdata, s2_rdata, sb_rdata;
    wire       s0_ready, s1_ready, s2_ready, sb_ready;
    wire       s0_split_req, s0_split_done;

    // ------------------------------------------------------------------------
    // Master Nodes
    // ------------------------------------------------------------------------
    master_node_uart #(
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .RESP_TIMEOUT (RESP_TIMEOUT)
    ) m0_inst (
        .clk       (clk),            .rst_n    (rst_n),
        .cmd_start (m0_cmd_start),   .cmd_we   (m0_cmd_we),
        .cmd_remote(m0_cmd_remote),
        .cmd_addr  (m0_cmd_addr),    .cmd_wdata(m0_cmd_wdata),
        .cmd_rdata (m0_cmd_rdata),   .cmd_done (m0_cmd_done),
        .cmd_error (m0_cmd_error),
        .bus_req   (req0),           .bus_gnt  (gnt0),
        .bus_split (m0_split_notify),
        .m_addr    (m0_addr),        .m_wdata  (m0_wdata),
        .m_we      (m0_we),          .m_valid  (m0_valid),
        .bus_rdata (bus_rdata),      .bus_ready(bus_ready),
        .uart_rx_serial(ext_rx_serial),
        .uart_tx_serial(ext_tx_serial),
        .remote_busy(), .srv_busy()
    );

    master_node m1_inst (
        .clk       (clk),            .rst_n    (rst_n),
        .cmd_start (m1_cmd_start),   .cmd_we   (m1_cmd_we), 
        .cmd_addr  (m1_cmd_addr),    .cmd_wdata(m1_cmd_wdata),
        .cmd_rdata (m1_cmd_rdata),   .cmd_done (m1_cmd_done),
        .bus_req   (req1),           .bus_gnt  (gnt1), 
        .bus_split (m1_split_notify),
        .m_addr    (m1_addr),        .m_wdata  (m1_wdata), 
        .m_we      (m1_we),          .m_valid  (m1_valid),
        .bus_rdata (bus_rdata),      .bus_ready(bus_ready)
    );

    // ------------------------------------------------------------------------
    // System Bus Interconnect
    // ------------------------------------------------------------------------
    bus_interconnect interconnect_inst (
        .clk             (clk),
        .rst_n           (rst_n),

        // Master 0 Interface
        .req0            (req0),
        .gnt0            (gnt0),
        .m0_split_notify (m0_split_notify),
        .m0_addr         (m0_addr),
        .m0_wdata        (m0_wdata),
        .m0_we           (m0_we),
        .m0_valid        (m0_valid),

        // Master 1 Interface
        .req1            (req1),
        .gnt1            (gnt1),
        .m1_split_notify (m1_split_notify),
        .m1_addr         (m1_addr),
        .m1_wdata        (m1_wdata),
        .m1_we           (m1_we),
        .m1_valid        (m1_valid),

        // Bus Out to Slaves
        .bus_addr        (bus_addr),
        .bus_wdata       (bus_wdata),
        .bus_we          (bus_we),

        // Decoded Selects
        .sel_s0          (sel_s0),
        .sel_s1          (sel_s1),
        .sel_s2          (sel_s2),
        .sel_bridge      (sel_bridge),

        // Split Handshakes
        .s0_split_req    (s0_split_req),
        .s0_split_done   (s0_split_done),

        // Slave Data Inputs
        .s0_rdata        (s0_rdata),
        .s0_ready        (s0_ready),
        .s1_rdata        (s1_rdata),
        .s1_ready        (s1_ready),
        .s2_rdata        (s2_rdata),
        .s2_ready        (s2_ready),
        .sb_rdata        (sb_rdata),
        .sb_ready        (sb_ready),

        // Bus Responses back to Masters
        .bus_rdata       (bus_rdata),
        .bus_ready       (bus_ready)
    );

    // ------------------------------------------------------------------------
    // Slave Nodes
    // ------------------------------------------------------------------------
    // Slave 0 (4K Split RAM)
    slave_0_split_4k s0_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .sel        (sel_s0),
        .we         (bus_we),
        .addr       (bus_addr[11:0]),
        .din        (bus_wdata),
        .dout       (s0_rdata),
        .ready      (s0_ready),
        .split_req  (s0_split_req),
        .split_done (s0_split_done)
    );

    // Slave 1 (4K Fast RAM)
    slave_fast_ram #(.ADDR_WIDTH(12)) s1_inst (
        .clk   (clk),
        .rst_n (rst_n),
        .sel   (sel_s1),
        .we    (bus_we),
        .addr  (bus_addr[11:0]),
        .din   (bus_wdata),
        .dout  (s1_rdata),
        .ready (s1_ready)
    );

    // Slave 2 (2K Fast RAM)
    slave_fast_ram #(.ADDR_WIDTH(11)) s2_inst (
        .clk   (clk),
        .rst_n (rst_n),
        .sel   (sel_s2),
        .we    (bus_we),
        .addr  (bus_addr[10:0]),
        .din   (bus_wdata),
        .dout  (s2_rdata),
        .ready (s2_ready)
    );

    // External Bridge
    // The UART now lives inside Master 0, so this decoder slot is unused.
    // Acknowledge with zero rather than leaving it dangling: an unanswered
    // select would hang the bus exactly like the 0x2800-0x2FFF gap.
    reg sb_ready_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sb_ready_r <= 1'b0;
        else        sb_ready_r <= sel_bridge;
    end
    assign sb_rdata = 8'h00;
    assign sb_ready = sb_ready_r;
endmodule