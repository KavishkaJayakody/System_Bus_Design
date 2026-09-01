`timescale 1ns / 1ps

module top_bus_system (
    input  wire        clk,
    input  wire        rst_n,

    // Master 0 Interface
    input  wire        m0_cmd_start,
    input  wire        m0_cmd_we,
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

    // External Bridge Pins for DE0 GPIOs
    input  wire        ext_rx_serial,
    output wire        ext_tx_serial
);

    // Instantiate Sub-system Interconnect
    bus_interconnect interconnect_inst (
        .clk           (clk),
        .rst_n         (rst_n),

        // Master 0 Wires
        .m0_cmd_start (m0_cmd_start),
        .m0_cmd_we    (m0_cmd_we),
        .m0_cmd_addr  (m0_cmd_addr),
        .m0_cmd_wdata (m0_cmd_wdata),
        .m0_cmd_rdata (m0_cmd_rdata),
        .m0_cmd_done  (m0_cmd_done),

        // Master 1 Wires
        .m1_cmd_start (m1_cmd_start),
        .m1_cmd_we    (m1_cmd_we),
        .m1_cmd_addr  (m1_cmd_addr),
        .m1_cmd_wdata (m1_cmd_wdata),
        .m1_cmd_rdata (m1_cmd_rdata),
        .m1_cmd_done  (m1_cmd_done),

        // Serial Bridge Wires
        .ext_rx_serial (ext_rx_serial),
        .ext_tx_serial (ext_tx_serial)
    );

endmodule