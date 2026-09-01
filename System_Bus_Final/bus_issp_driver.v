`timescale 1ns / 1ps
//
// bus_issp_driver
//
// JTAG In-System Sources and Probes front-end for top_bus_system.
// Lets you drive both master command ports and observe the results from the
// Quartus "In-System Sources and Probes Editor" with no pins and no testbench.
//
// Instance ID: "BUS0"
//
// SOURCE map (50 bits, driven by the host):
//   src[0]     m0_go        level; a 0->1 edge launches one Master 0 command
//   src[1]     m0_cmd_we    1 = write, 0 = read
//   src[15:2]  m0_cmd_addr  14-bit bus address
//   src[23:16] m0_cmd_wdata 8-bit write data
//   src[24]    m1_go        level; a 0->1 edge launches one Master 1 command
//   src[25]    m1_cmd_we
//   src[39:26] m1_cmd_addr
//   src[47:40] m1_cmd_wdata
//   src[48]    soft_rst     clears the sticky done/collision flags
//   src[49]    reserved
//
// PROBE map (38 bits, read back by the host):
//   prb[7:0]   m0_rl        last read data captured for Master 0
//   prb[8]     m0_done_s    sticky "command finished" (clears when m0_go drops)
//   prb[9]     m0_busy      command in flight
//   prb[17:10] m1_rl
//   prb[18]    m1_done_s
//   prb[19]    m1_busy
//   prb[27:20] m0_lat       cycles from launch to done, saturating at 0xFF
//   prb[35:28] m1_lat
//   prb[36]    collision    sticky: both masters were in flight at once
//   prb[37]    reserved
//
// A latency reading of 0xFF with busy still high means the transaction never
// completed - e.g. an address in the 0x2800-0x2FFF decode gap, which asserts
// no slave select and therefore never returns bus_ready.
//
module bus_issp_driver (
    input  wire        clk,
    input  wire        rst_n,

    output reg         m0_cmd_start,
    output wire        m0_cmd_we,
    output wire [13:0] m0_cmd_addr,
    output wire [7:0]  m0_cmd_wdata,
    input  wire [7:0]  m0_cmd_rdata,
    input  wire        m0_cmd_done,

    output reg         m1_cmd_start,
    output wire        m1_cmd_we,
    output wire [13:0] m1_cmd_addr,
    output wire [7:0]  m1_cmd_wdata,
    input  wire [7:0]  m1_cmd_rdata,
    input  wire        m1_cmd_done
);

    wire [49:0] src;
    wire [37:0] prb;

    altsource_probe #(
        .sld_auto_instance_index ("YES"),
        .instance_id             ("BUS0"),
        .source_width            (50),
        .probe_width             (38),
        .source_initial_value    ("0"),
        .enable_metastability    ("YES")
    ) u_issp (
        .source     (src),
        .probe      (prb),
        .source_clk (clk),
        .source_ena (1'b1)
    );

    // Command fields. Bit 0 of each 24-bit slice is the 'go' level, so the
    // command payload is the upper 23 bits of the slice.
    assign {m0_cmd_wdata, m0_cmd_addr, m0_cmd_we} = src[23:1];
    assign {m1_cmd_wdata, m1_cmd_addr, m1_cmd_we} = src[47:25];

    wire m0_go    = src[0];
    wire m1_go    = src[24];
    wire soft_rst = src[48];

    reg        m0_go_d,   m1_go_d;
    reg        m0_busy,   m1_busy;
    reg        m0_done_s, m1_done_s;
    reg        collision;
    reg  [7:0] m0_rl,  m1_rl;
    reg  [7:0] m0_lat, m1_lat;

    wire m0_rise = m0_go & ~m0_go_d;
    wire m1_rise = m1_go & ~m1_go_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {m0_go_d,   m1_go_d}      <= 2'b0;
            {m0_busy,   m1_busy}      <= 2'b0;
            {m0_done_s, m1_done_s}    <= 2'b0;
            collision                 <= 1'b0;
            {m0_cmd_start, m1_cmd_start} <= 2'b0;
            {m0_rl,  m1_rl}           <= 16'b0;
            {m0_lat, m1_lat}          <= 16'b0;
        end else begin
            m0_go_d <= m0_go;
            m1_go_d <= m1_go;

            // cmd_start is a single-cycle pulse
            m0_cmd_start <= 1'b0;
            m1_cmd_start <= 1'b0;

            // ---- Master 0 ----
            if (m0_rise && !m0_busy) begin
                m0_cmd_start <= 1'b1;
                m0_busy      <= 1'b1;
                m0_done_s    <= 1'b0;
                m0_lat       <= 8'd0;
            end else if (m0_busy) begin
                if (m0_lat != 8'hFF) m0_lat <= m0_lat + 8'd1;
                if (m0_cmd_done) begin
                    m0_busy   <= 1'b0;
                    m0_done_s <= 1'b1;
                    m0_rl     <= m0_cmd_rdata;
                end
            end

            // ---- Master 1 ----
            if (m1_rise && !m1_busy) begin
                m1_cmd_start <= 1'b1;
                m1_busy      <= 1'b1;
                m1_done_s    <= 1'b0;
                m1_lat       <= 8'd0;
            end else if (m1_busy) begin
                if (m1_lat != 8'hFF) m1_lat <= m1_lat + 8'd1;
                if (m1_cmd_done) begin
                    m1_busy   <= 1'b0;
                    m1_done_s <= 1'b1;
                    m1_rl     <= m1_cmd_rdata;
                end
            end

            // ---- Sticky status ----
            if (m0_busy && m1_busy) collision <= 1'b1;

            if (!m0_go) m0_done_s <= 1'b0;
            if (!m1_go) m1_done_s <= 1'b0;

            if (soft_rst) begin
                m0_done_s <= 1'b0;
                m1_done_s <= 1'b0;
                collision <= 1'b0;
            end
        end
    end

    assign prb = {1'b0, collision,
                  m1_lat, m0_lat,
                  m1_busy, m1_done_s, m1_rl,
                  m0_busy, m0_done_s, m0_rl};

endmodule
