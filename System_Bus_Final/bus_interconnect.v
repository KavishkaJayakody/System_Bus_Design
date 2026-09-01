`timescale 1ns / 1ps

module bus_interconnect (
    input  wire        clk,
    input  wire        rst_n,

    // Master 0 Interface Signals
    input  wire        req0,
    output wire        gnt0,
    output wire        m0_split_notify,
    input  wire [13:0] m0_addr,
    input  wire [7:0]  m0_wdata,
    input  wire        m0_we,
    input  wire        m0_valid,

    // Master 1 Interface Signals
    input  wire        req1,
    output wire        gnt1,
    output wire        m1_split_notify,
    input  wire [13:0] m1_addr,
    input  wire [7:0]  m1_wdata,
    input  wire        m1_we,
    input  wire        m1_valid,

    // Shared Bus Signals to Slaves
    output wire [13:0] bus_addr,
    output wire [7:0]  bus_wdata,
    output wire        bus_we,

    // Slave Select Signals
    output wire        sel_s0,
    output wire        sel_s1,
    output wire        sel_s2,
    output wire        sel_bridge,

    // Slave Control & Split Handshake Signals
    input  wire        s0_split_req,
    input  wire        s0_split_done,

    // Slave Data Inputs
    input  wire [7:0]  s0_rdata,
    input  wire        s0_ready,
    input  wire [7:0]  s1_rdata,
    input  wire        s1_ready,
    input  wire [7:0]  s2_rdata,
    input  wire        s2_ready,
    input  wire [7:0]  sb_rdata,
    input  wire        sb_ready,

    // Multiplexed Bus Responses back to Masters
    output reg  [7:0]  bus_rdata,
    output reg         bus_ready
);

    wire bus_valid;
    wire decode_err;

    // ------------------------------------------------------------------------
    // 1. Central Arbiter Module
    // ------------------------------------------------------------------------
    arbiter_2m_split arbiter_inst (
        .clk             (clk),
        .rst_n           (rst_n),
        .req0            (req0),
        .req1            (req1),
        .gnt0            (gnt0),
        .gnt1            (gnt1),
        .s0_split_req    (s0_split_req),
        .s0_split_done   (s0_split_done),
        .m0_split_notify (m0_split_notify),
        .m1_split_notify (m1_split_notify)
    );

    // ------------------------------------------------------------------------
    // 2. Active Master Multiplexer
    // ------------------------------------------------------------------------
    assign bus_addr  = gnt0 ? m0_addr  : (gnt1 ? m1_addr  : 14'h0000);
    assign bus_wdata = gnt0 ? m0_wdata : (gnt1 ? m1_wdata : 8'h00);
    assign bus_we    = gnt0 ? m0_we    : (gnt1 ? m1_we    : 1'b0);
    assign bus_valid = gnt0 ? m0_valid : (gnt1 ? m1_valid : 1'b0);

    // ------------------------------------------------------------------------
    // 3. Address Decoder
    // ------------------------------------------------------------------------
    address_decoder decoder_inst (
        .addr       (bus_addr),
        .bus_valid  (bus_valid),
        .sel_s0     (sel_s0),
        .sel_s1     (sel_s1),
        .sel_s2     (sel_s2),
        .sel_bridge (sel_bridge),
        .decode_err (decode_err)
    );

    // ------------------------------------------------------------------------
    // 4. Response Return Multiplexer
    // ------------------------------------------------------------------------
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