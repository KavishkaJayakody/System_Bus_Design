// JTAG In-System Sources and Probes front-end, instance "SBUS".  It drives the
// masters' normal PARALLEL command ports - not a back door onto the wires.
//
// THIS BIT MAP LIVES IN THREE PLACES THAT MUST AGREE: here, the `assign prb'
// at the bottom of this file, and tcl/issp_bus_lib.tcl.
//
// SOURCE 56b, per master base m*26:
//   [b+0] go (a 0->1 edge fires ONE command; deliberately NOT in the payload
//            concat - widening over it aliases we onto go and kills reads)
//   [b+1] we   [b+17:b+2] addr   [b+25:b+18] wdata
//   m0 = src[25:0]  m1 = src[51:26]
//   [52] soft_rst  [53] issp_mode (unconnected)  [54] split_en  [55] spare
//
// PROBE 128b, per master base m*30:
//   [b+7:b+0] rdata  [b+8] done_s  [b+9] busy  [b+11:b+10] resp
//   [b+12] err_s  [b+20:b+13] lat  [b+28:b+21] splits  [b+29] br_error (m0)
//   m0 = prb[29:0]  m1 = prb[59:30]
//
//   [62:60] gnt (m0,m1,BRIDGE)   [65:63] split_mask
//   [70:66] sel_q {def,BR,s2,s1,s0}   [71] split_busy   [72] collision
//   [88:73] bus_addr   [93:89] frame_len (must read 16)   [94] frame_bad
//   [95] remote_busy   [96] srv_busy   [97] rx_active   [98] req_overrun
//   [106:99] rx_last  [114:107] rx_count  [122:115] tx_count
//   [124:123] rx_state  [125] req_seen  [126] resp_seen  [127] spare

`include "bus_defs.vh"

module bus_issp_driver #(
    parameter ADDR_W = `BUS_ADDR_W,
    parameter DATA_W = `BUS_DATA_W,
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire                  clk,
    input  wire                  rst_n,

    output reg  [1:0]            cmd_valid,
    output wire [1:0]            cmd_we,
    output wire [2*ADDR_W-1:0]   cmd_addr_flat,
    output wire [2*DATA_W-1:0]   cmd_wdata_flat,
    input  wire [1:0]            cmd_accept,
    input  wire [1:0]            done,
    input  wire [2*DATA_W-1:0]   rdata_flat,
    input  wire [2*RESP_W-1:0]   resp_flat,
    input  wire [1:0]            err,
    input  wire [15:0]           split_count_flat,

    input  wire [2:0]            gnt,          // 3 masters: m0, m1, bridge
    input  wire [2:0]            split_mask,
    input  wire [4:0]            sel_q,        // {def, bridge, s2, s1, s0}
    input  wire                  split_busy,
    input  wire [ADDR_W-1:0]     bus_addr,
    input  wire                  bus_valid,

    input  wire                  cmd_error,      // sticky-ish: last remote failed
    input  wire                  remote_busy,
    input  wire                  srv_busy,
    input  wire [7:0]            dbg_rx_last,
    input  wire [7:0]            dbg_rx_count,
    input  wire [7:0]            dbg_tx_count,
    input  wire [1:0]            dbg_rx_state,
    input  wire                  dbg_req_seen,
    input  wire                  dbg_resp_seen,
    input  wire                  dbg_rx_active,
    input  wire                  dbg_req_overrun,

    output wire                  issp_mode,
    output wire                  split_en
);

    wire [55:0] src;
    wire [127:0] prb;

    altsource_probe #(
        .sld_auto_instance_index ("YES"),
        .instance_id             ("SBUS"),
        .source_width            (56),
        .probe_width             (128),
        .source_initial_value    ("0"),
        .enable_metastability    ("YES")
    ) u_issp (
        .source     (src),
        .probe      (prb),
        .source_clk (clk),
        .source_ena (1'b1)
    );

    // deliberately NOT part of the payload - widening the concat over it
    assign cmd_addr_flat [0*ADDR_W +: ADDR_W] = src[17:2];
    assign cmd_wdata_flat[0*DATA_W +: DATA_W] = src[25:18];
    assign cmd_we[0]                          = src[1];

    assign cmd_addr_flat [1*ADDR_W +: ADDR_W] = src[43:28];
    assign cmd_wdata_flat[1*DATA_W +: DATA_W] = src[51:44];
    assign cmd_we[1]                          = src[27];

    wire [1:0] go       = {src[26], src[0]};
    wire       soft_rst = src[52];
    assign     issp_mode   = src[53];
    assign     split_en    = src[54];

    reg  [1:0]  go_d, busy, done_s, err_s;
    reg         rem_err_s;          // sticky: a REMOTE cmd timed out
    reg  [7:0]  rl   [0:1];
    reg  [1:0]  rp   [0:1];
    reg  [7:0]  lat  [0:1];
    reg         collision;

    wire [1:0] go_rise = go & ~go_d;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            go_d      <= 2'b00;
            busy      <= 2'b00;
            done_s    <= 2'b00;
            err_s     <= 2'b00;
            rem_err_s <= 1'b0;
            cmd_valid <= 2'b00;
            collision <= 1'b0;
            for (i = 0; i < 2; i = i + 1) begin
                rl[i]  <= 8'h00;
                rp[i]  <= 2'b00;
                lat[i] <= 8'h00;
            end
        end else begin
            go_d <= go;

            for (i = 0; i < 2; i = i + 1) begin
                if (cmd_valid[i] && cmd_accept[i])
                    cmd_valid[i] <= 1'b0;

                if (go_rise[i] && !busy[i]) begin
                    cmd_valid[i] <= 1'b1;
                    busy[i]      <= 1'b1;
                    done_s[i]    <= 1'b0;
                    lat[i]       <= 8'd0;
                end else if (busy[i]) begin
                    if (lat[i] != 8'hFF) lat[i] <= lat[i] + 8'd1;
                    if (done[i]) begin
                        busy[i]   <= 1'b0;
                        done_s[i] <= 1'b1;
                        rl[i]     <= rdata_flat[i*DATA_W +: DATA_W];
                        rp[i]     <= resp_flat [i*RESP_W +: RESP_W];
                        err_s[i]  <= err[i];
                        // ERROR: "the far board never answered" is a different
                        if (i == 0) rem_err_s <= cmd_error;
                    end
                end

                if (!go[i]) done_s[i] <= 1'b0;
            end

            if (busy[0] && busy[1]) collision <= 1'b1;

            if (soft_rst) begin
                done_s    <= 2'b00;
                err_s     <= 2'b00;
                collision <= 1'b0;
            end
        end
    end

    reg [4:0] fcnt, frame_len;
    reg       frame_bad;
    reg       bus_valid_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fcnt        <= 5'd0;
            frame_len   <= 5'd0;
            frame_bad   <= 1'b0;
            bus_valid_d <= 1'b0;
        end else begin
            bus_valid_d <= bus_valid;

            if (bus_valid) begin
                if (fcnt != 5'd31) fcnt <= fcnt + 5'd1;
            end else if (bus_valid_d) begin
                frame_len <= fcnt;
                if (fcnt != ADDR_W[4:0]) frame_bad <= 1'b1;
                fcnt <= 5'd0;
            end else begin
                fcnt <= 5'd0;
            end

            if (soft_rst) frame_bad <= 1'b0;
        end
    end

    assign prb = { 1'b0,                                   // [127] spare
                   dbg_resp_seen,                          // [126]
                   dbg_req_seen,                           // [125]
                   dbg_rx_state,                           // [124:123]
                   dbg_tx_count,                           // [122:115]
                   dbg_rx_count,                           // [114:107]
                   dbg_rx_last,                            // [106:99]
                   dbg_req_overrun,                        // [98]
                   dbg_rx_active,                          // [97]
                   srv_busy,                               // [96]
                   remote_busy,                            // [95]
                   frame_bad,                              // [94]
                   frame_len,                              // [93:89]
                   bus_addr,                               // [88:73]
                   collision,                              // [72]
                   split_busy,                             // [71]
                   sel_q,                                  // [70:66]
                   split_mask,                             // [65:63]
                   gnt,                                    // [62:60]
                   1'b0, split_count_flat[15:8],           // [59:51] m1
                   lat[1], err_s[1], rp[1], busy[1], done_s[1], rl[1],
                   rem_err_s, split_count_flat[7:0],       // [29:21] m0
                   lat[0], err_s[0], rp[0], busy[0], done_s[0], rl[0] };

endmodule
