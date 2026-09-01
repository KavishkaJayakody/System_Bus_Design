`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/01/2026 02:01:52 AM
// Design Name: 
// Module Name: top_bus_system
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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

    // Master-Arbiter Wires
    wire req0, req1, gnt0, gnt1;
    wire m0_split_notify, m1_split_notify;

    // Master-Bus MUX Wires
    wire [13:0] m0_addr, m1_addr, bus_addr;
    wire [7:0]  m0_wdata, m1_wdata, bus_wdata;
    wire        m0_we, m1_we, bus_we;
    wire        m0_valid, m1_valid, bus_valid;

    // Decoder & Slave Selects
    wire sel_s0, sel_s1, sel_s2, sel_bridge, decode_err;

    // Slave Responses
    wire [7:0] s0_rdata, s1_rdata, s2_rdata, sb_rdata;
    wire       s0_ready, s1_ready, s2_ready, sb_ready;
    wire       s0_split_req, s0_split_done;

    reg [7:0]  bus_rdata;
    reg        bus_ready;

    // 1. Central Arbiter
    arbiter_2m_split arbiter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .req0(req0),
        .req1(req1),
        .gnt0(gnt0),
        .gnt1(gnt1),
        .s0_split_req(s0_split_req),
        .s0_split_done(s0_split_done),
        .m0_split_notify(m0_split_notify),
        .m1_split_notify(m1_split_notify)
    );

    // 2. Master Nodes
    master_node m0_inst (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(m0_cmd_start), .cmd_we(m0_cmd_we), .cmd_addr(m0_cmd_addr), .cmd_wdata(m0_cmd_wdata),
        .cmd_rdata(m0_cmd_rdata), .cmd_done(m0_cmd_done),
        .bus_req(req0), .bus_gnt(gnt0), .bus_split(m0_split_notify),
        .m_addr(m0_addr), .m_wdata(m0_wdata), .m_we(m0_we), .m_valid(m0_valid),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready)
    );

    master_node m1_inst (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(m1_cmd_start), .cmd_we(m1_cmd_we), .cmd_addr(m1_cmd_addr), .cmd_wdata(m1_cmd_wdata),
        .cmd_rdata(m1_cmd_rdata), .cmd_done(m1_cmd_done),
        .bus_req(req1), .bus_gnt(gnt1), .bus_split(m1_split_notify),
        .m_addr(m1_addr), .m_wdata(m1_wdata), .m_we(m1_we), .m_valid(m1_valid),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready)
    );

    // 3. Active Master MUX
    assign bus_addr  = gnt0 ? m0_addr  : (gnt1 ? m1_addr  : 14'h0000);
    assign bus_wdata = gnt0 ? m0_wdata : (gnt1 ? m1_wdata : 8'h00);
    assign bus_we    = gnt0 ? m0_we    : (gnt1 ? m1_we    : 1'b0);
    assign bus_valid = gnt0 ? m0_valid : (gnt1 ? m1_valid : 1'b0);

    // 4. Address Decoder
    address_decoder decoder_inst (
        .addr(bus_addr),
        .bus_valid(bus_valid),
        .sel_s0(sel_s0),
        .sel_s1(sel_s1),
        .sel_s2(sel_s2),
        .sel_bridge(sel_bridge),
        .decode_err(decode_err)
    );

    // 5. Slave 0 (4K Split)
    slave_0_split_4k s0_inst (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_s0), .we(bus_we), .addr(bus_addr[11:0]), .din(bus_wdata),
        .dout(s0_rdata), .ready(s0_ready),
        .split_req(s0_split_req), .split_done(s0_split_done)
    );

    // 6. Slave 1 (4K Fast RAM)
    slave_fast_ram #(.ADDR_WIDTH(12)) s1_inst (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_s1), .we(bus_we), .addr(bus_addr[11:0]), .din(bus_wdata),
        .dout(s1_rdata), .ready(s1_ready)
    );

    // 7. Slave 2 (2K Fast RAM)
    slave_fast_ram #(.ADDR_WIDTH(11)) s2_inst (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_s2), .we(bus_we), .addr(bus_addr[10:0]), .din(bus_wdata),
        .dout(s2_rdata), .ready(s2_ready)
    );

    // 8. Cross-FPGA Dummy Bridge
    cross_fpga_bridge_dummy bridge_inst (
        .clk(clk), .rst_n(rst_n),
        .sel(sel_bridge), .we(bus_we), .addr(bus_addr[11:0]), .din(bus_wdata),
        .dout(sb_rdata), .ready(sb_ready),
        .ext_rx_serial(ext_rx_serial), .ext_tx_serial(ext_tx_serial)
    );

    // 9. Response Return MUX
    always @(*) begin
        bus_rdata = 8'h00;
        bus_ready = 1'b0;

        if (sel_s0) begin
            bus_rdata = s0_rdata;
            bus_ready = s0_ready;
        end else if (sel_s1) begin
            bus_rdata = s1_rdata;
            bus_ready = s1_ready;
        end else if (sel_s2) begin
            bus_rdata = s2_rdata;
            bus_ready = s2_ready;
        end else if (sel_bridge) begin
            bus_rdata = sb_rdata;
            bus_ready = sb_ready;
        end
    end

endmodule