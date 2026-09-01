`timescale 1ns / 1ps
//
// top_debug
//
// Synthesis top level: the system bus plus its JTAG In-System Sources and
// Probes driver. Only clk, rst_n and the two cross-FPGA bridge pins leave the
// device; both master command ports are driven over JTAG by bus_issp_driver.
//
module top_debug #(
    parameter CLKS_PER_BIT = 434   // 50 MHz / 115200 baud
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ext_rx_serial,
    output wire       ext_tx_serial,

    // DE0-Nano LED[7:0] - active high. Shows the data byte from the last
    // Master 0 READ to complete; writes leave the display unchanged.
    output wire [7:0] led
);

    // JTAG side of the Master 0 command path
    wire        j0_start, j0_we;
    wire [13:0] j0_addr;
    wire [7:0]  j0_wdata;

    // Post-injector side actually presented to Master 0
    wire        m0_start, m0_we, m0_done;
    wire [13:0] m0_addr;
    wire [7:0]  m0_wdata, m0_rdata;
    wire        uart_cmd_stb;
    wire [1:0]  uart_rx_byte_idx;

    wire        m1_start, m1_we, m1_done;
    wire [13:0] m1_addr;
    wire [7:0]  m1_wdata, m1_rdata;

    bus_issp_driver u_dbg (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(j0_start), .m0_cmd_we(j0_we),
        .m0_cmd_addr(j0_addr),   .m0_cmd_wdata(j0_wdata),
        .m0_cmd_rdata(m0_rdata), .m0_cmd_done(m0_done),
        .m1_cmd_start(m1_start), .m1_cmd_we(m1_we),
        .m1_cmd_addr(m1_addr),   .m1_cmd_wdata(m1_wdata),
        .m1_cmd_rdata(m1_rdata), .m1_cmd_done(m1_done)
    );

    // UART command receiver. Passes the JTAG command through until three
    // bytes arrive over UART, then injects that 24-bit command for one clock.
    uart_cmd_inject #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rxcmd (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(ext_rx_serial),
        .jtag_start(j0_start), .jtag_we(j0_we),
        .jtag_addr(j0_addr),   .jtag_wdata(j0_wdata),
        .m0_start(m0_start),   .m0_we(m0_we),
        .m0_addr(m0_addr),     .m0_wdata(m0_wdata),
        .uart_cmd_stb(uart_cmd_stb),
        .rx_byte_idx(uart_rx_byte_idx)
    );

    top_bus_system #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_bus (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(m0_start), .m0_cmd_we(m0_we),
        .m0_cmd_addr(m0_addr),   .m0_cmd_wdata(m0_wdata),
        .m0_cmd_rdata(m0_rdata), .m0_cmd_done(m0_done),
        .m1_cmd_start(m1_start), .m1_cmd_we(m1_we),
        .m1_cmd_addr(m1_addr),   .m1_cmd_wdata(m1_wdata),
        .m1_cmd_rdata(m1_rdata), .m1_cmd_done(m1_done),
        .ext_tx_serial(ext_tx_serial)
    );

    // master_node holds cmd_rdata until the next read completes, so this is
    // a stable display with no extra latch needed.
    assign led = m0_rdata;

endmodule
