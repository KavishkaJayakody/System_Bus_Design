//==========================================================================
// bus_issp_driver.v
//
// JTAG In-System Sources and Probes front-end for the SERIAL system bus.
// Drives either master's command port and reports what the bus did, with no
// pins, no switches and no testbench - everything goes over the USB-Blaster.
//
// Instance ID: "SBUS"
//
// The command ports it drives are PARALLEL (a whole address, a whole data
// byte): serialisation happens inside `master'.  So this module debugs the
// bus through its normal front door - it is not a back door onto the wires.
// What it adds beyond the LEDs is the bus-side observability at the bottom
// of the probe map, including the reassembled address and the frame length,
// which are the two things that say whether SERIAL transport is working.
//
//--------------------------------------------------------------------------
// SOURCE map (56 bits, driven by the host)
//--------------------------------------------------------------------------
//   Per master, base = m*26:
//     src[b+0]      go        level; a 0->1 edge launches ONE command
//     src[b+1]      we        1 = write, 0 = read
//     src[b+17:b+2] addr      16-bit word address
//     src[b+25:b+18] wdata    8-bit write data
//   so  m0 = src[25:0]   m1 = src[51:26]
//
//     src[52]  soft_rst     clears the sticky done/collision/error/frame flags
//     src[53]  issp_mode    1 = ISSP owns the master command ports
//                           (0 = the on-board scenario sequencer owns them)
//     src[54]  split_en     1 = the split-capable slave (slave 2)
//                           answers SPLIT
//     src[55]  spare        (was cmd_remote; the ADDRESS selects the far
//                           board now - anything at 0x8000+ goes over the
//                           link, so no command bit is needed)
//
//--------------------------------------------------------------------------
// PROBE map (128 bits, read back by the host)
//--------------------------------------------------------------------------
//   Per master, base = m*30:
//     prb[b+7:b+0]    rdata     last read data
//     prb[b+8]        done_s    sticky "finished"; clears when go drops
//     prb[b+9]        busy      command in flight
//     prb[b+11:b+10]  resp      00 OKAY, 01 ERROR, 10 SPLIT
//     prb[b+12]       err_s     sticky: last transaction answered ERROR
//     prb[b+20:b+13]  lat       clocks from launch to done, saturating at 0xFF
//     prb[b+28:b+21]  splits    that master's split_count
//     prb[b+29]       cmd_error sticky: a REMOTE transaction timed out.
//                               Master 0 only; prb[59] reads 0 always.
//   so  m0 = prb[29:0]   m1 = prb[59:30]
//
//     prb[62:60]  gnt         one-hot grant, 3 masters: m0, m1, BRIDGE
//     prb[65:63]  split_mask  a lit bit = that master is split-deferred
//     prb[70:66]  sel_q       latched responder {def, BRIDGE, s2, s1, s0}
//     prb[71]     split_busy  the split memory has one in flight
//     prb[72]     collision   sticky: both masters were in flight at once
//     prb[88:73]  bus_addr    the address REASSEMBLED off the serial wire
//     prb[93:89]  frame_len   clocks the last address frame was high
//     prb[94]     frame_bad   sticky: some frame was not ADDR_W clocks
//     prb[95]     remote_busy a remote round trip is outstanding
//     prb[96]     srv_busy    serving the other board's request right now
//     prb[97]     rx_active   the link's RX line is not idle-high
//     prb[98]     req_overrun sticky: an incoming REQUEST was overwritten
//                             before the server could run it - the far board
//                             sent faster than this side drained
//
//   LINK DIAGNOSTICS - what you read when the far board says nothing:
//     prb[106:99]   rx_last     last byte the UART framed
//     prb[114:107]  rx_count    bytes received since reset (wraps)
//     prb[122:115]  tx_count    bytes sent since reset (wraps)
//     prb[124:123]  rx_state    0=hunting for a tag, 1=in REQ, 2=in RESP
//     prb[125]      req_seen    sticky: a whole REQUEST was parsed
//     prb[126]      resp_seen   sticky: a whole RESPONSE was parsed
//     prb[127]      spare
//
//   rx_count == 0        nothing is arriving: cable, ground or baud
//   rx_count > 0 but
//     resp_seen == 0     bytes arrive but never parse: baud slightly off,
//                        byte order, or a tag/format disagreement
//   req_overrun == 1     requests ARE parsing but one was thrown away: this
//                        side was blocked longer than a frame takes to
//                        arrive (a long split, or our own client queued
//                        behind the transmitter).  The link has no flow
//                        control, so this is reported, not prevented.
//
// A latency of 0xFF with busy still high means the transaction never
// completed.  On this bus that should be impossible - every unmapped address
// is answered ERROR by the default responder - so if you ever see it, the
// bus is genuinely wedged and that is a defect worth chasing.
//
// frame_len must read 16 (ADDR_W).  If frame_bad is set, the serial framing
// on the real device is wrong and nothing downstream can be trusted.
//==========================================================================
`include "bus_defs.vh"

module bus_issp_driver #(
    parameter ADDR_W = `BUS_ADDR_W,
    parameter DATA_W = `BUS_DATA_W,
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- master command ports (parallel; `master' does the serialising) --
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

    // ---- bus observability ----------------------------------------------
    input  wire [2:0]            gnt,          // 3 masters: m0, m1, bridge
    input  wire [2:0]            split_mask,
    input  wire [4:0]            sel_q,        // {def, bridge, s2, s1, s0}
    input  wire                  split_busy,
    input  wire [ADDR_W-1:0]     bus_addr,
    input  wire                  bus_valid,

    // ---- the UART link to the other board (master 0 only) ---------------
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

    // ---- control back out to the board ----------------------------------
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

    //----------------------------------------------------------------------
    // Command fields.  Bit 0 of each 26-bit slice is the `go' level and is
    // deliberately NOT part of the payload - widening the concat over it
    // would alias `we' onto `go' and make reads impossible.  (That exact bug
    // cost time on the earlier parallel design's driver.)
    //----------------------------------------------------------------------
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

    //----------------------------------------------------------------------
    // Per-master launch, completion capture and latency.
    //----------------------------------------------------------------------
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
                // `master' takes a LEVEL and answers with cmd_accept - unlike
                // the parallel design's one-cycle start pulse - so hold
                // cmd_valid until it is taken.
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
                        // The link timeout is reported separately from a bus
                        // ERROR: "the far board never answered" is a different
                        // fault from "that address is not mapped".
                        if (i == 0) rem_err_s <= cmd_error;
                    end
                end

                // Releasing `go' re-arms that master for the next command.
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

    //----------------------------------------------------------------------
    // Serial framing watchdog.
    //
    // The one thing the LEDs cannot show: how long the address frame
    // actually was on the real device.  Count bus_valid, latch the length
    // when it falls, and stick a flag if it was ever not ADDR_W.
    //----------------------------------------------------------------------
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

    //----------------------------------------------------------------------
    // Probe assembly.  Keep this in step with the header comment and with
    // tcl/issp_bus_lib.tcl - three places, one bit map.
    //----------------------------------------------------------------------
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
