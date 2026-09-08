//==========================================================================
// master_uart.v -- `master' with the board-to-board UART link built in
//
// A transaction is LOCAL or REMOTE, and the ADDRESS decides:
//
//     addr[15] == 0    local   - goes out on this board's serial bus
//     addr[15] == 1    remote  - carried over the UART to the other board
//                                and executed on ITS bus
//
//     far address = local address - 0x8000
//     0x9ABC here == 0x1ABC over there
//
// There is no "remote" command bit; the window is the selector. Reads bring
// the data back. WRITES ARE POSTED - the far side sends nothing and this
// master retires as soon as the request is on the wire.
//
// The block is SYMMETRIC: it is also the server for the other board. A
// request arriving on the UART is run on the local bus through the same
// master core and, if it was a read, the result is sent back. One core
// serves both, and `srv_active' muxes the command inputs. Nothing on the
// receiving side knows the request came from a UART - it goes through the
// normal arbiter and address decoder, so it can reach any slave at any
// offset.
//
//--------------------------------------------------------------------------
// Wire format (8N1, 115200 baud, CLKS_PER_BIT = 434 at 50 MHz)
//--------------------------------------------------------------------------
//   REQUEST  (4 bytes)          RESPONSE (2 bytes, READS ONLY)
//     0xA5                        0x5A
//     cmd[7:0]                    data byte
//     cmd[15:8]
//     cmd[23:16]
//
// Byte order is LITTLE-ENDIAN: the low byte of the command goes first.
//
//   bit  23 ........ 16 | 15  14 | 13 .......... 2 |  1  |  0
//       +---------------+--------+-----------------+-----+-----+
//       |    wdata[7:0] |  dev   |   offset[11:0]  | we  |  0  |
//       +---------------+--------+-----------------+-----+-----+
//                  00=slave0  01=slave1  10=slave2    1=write  reserved
//
// `dev' and `offset' together are simply the far address's low 14 bits, so
// the whole command is {wdata, addr[13:0], we, 1'b0} on the way out and
// {2'b00, cmd[15:2]} on the way in. The receiving side's top two address
// bits are always 00, which is what makes 14 bits lossless - and it means a
// request can never decode back into the receiver's own remote window. The
// link is LOOP-FREE by construction; there is no hop count.
//
// Fourteen bits is lossless only for the addresses the far board can decode,
// which is what the map guarantees - but note it means ONLY addr[13:0] ever
// leaves. The usable remote window is 0x8000-0xBFFF; addr[14] is dropped, so
// 0xC000 and above alias back onto it (0xC000 goes where 0x8000 goes). There
// is no check for this: the whole point of the window is that the address IS
// the routing, so there is no spare encoding left to answer with.
//
//--------------------------------------------------------------------------
// Things that are load-bearing
//--------------------------------------------------------------------------
// * RESPONSES TAKE PRIORITY over requests in the shared transmitter. If both
//   boards fire at once and requests won, both would sit waiting for an
//   answer neither is sending, and the link would deadlock.
//
// * WRITES ARE POSTED. No response frame is sent for a write, so a 0x5A
//   frame is never ambiguous - exactly one arrives per read.
//
// * TAG HUNTING. A payload byte may itself equal 0xA5 or 0x5A. Hunt for a
//   tag, then take exactly 3 more bytes (request) or exactly 1 (response) -
//   never re-scan the payload for tags.
//
// * RESP_TIMEOUT (10 ms) completes a remote READ with 0xFF and `cmd_error'
//   if no answer arrives, so an unplugged cable reports a failure instead of
//   hanging. The counter is 32 bits: a counter narrower than its own
//   parameter truncates silently (design_notes §4).
//
// * THE LOCAL PATH IS A PURE PASS-THROUGH. `cmd_accept', `done', `rdata' and
//   `resp' come combinationally from the core for a local transaction, so a
//   local transfer still costs 21 clocks for a write and 30 for a read.
//
// * The core takes a LEVEL `cmd_valid' and answers `cmd_accept', so a local
//   command that arrives while the server is busy simply waits, held by its
//   own valid, instead of being dropped.
//
// * The server only takes the core when the core is IDLE and no local
//   command is being offered on the same cycle. Otherwise a local transfer
//   could complete while `srv_active' was high and its `done' be swallowed.
//
// * THE REQUEST HOLD IS ONE DEEP AND THE LINK HAS NO FLOW CONTROL. Writes
//   are posted and the wire format carries no ACK, so there is nothing to
//   push back with: if the far board sends faster than this side drains, a
//   request IS lost. That is a property of the agreed protocol, not a bug to
//   code around - so it is made VISIBLE instead, on `dbg_req_overrun'. The
//   one case that is a bug, a request arriving on the same cycle the server
//   takes the previous one, is handled: see `srv_take'.
//
//--------------------------------------------------------------------------
// Port             Dir  Width     Meaning
//--------------------------------------------------------------------------
// Everything `master' has, unchanged, plus:
//
// cmd_error        out  1         the last remote read timed out
// rm_rx            in   1         from the other board's rm_tx
// rm_tx            out  1         to the other board's rm_rx
// remote_busy      out  1         a remote read is outstanding
// srv_busy         out  1         serving the other board right now
// dbg_req_overrun  out  1         sticky: an incoming REQUEST was overwritten
//                                 before the server could run it
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module master_uart #(
    parameter ADDR_W       = `BUS_ADDR_W,
    parameter DATA_W       = `BUS_DATA_W,
    parameter RESP_W       = `BUS_RESP_W,
    parameter CLKS_PER_BIT = 434,        // 50 MHz / 115200
    parameter RESP_TIMEOUT = 500000      // ~10 ms at 50 MHz
) (
    input  wire                clk,
    input  wire                rst_n,

    // ---- command interface (parallel), as `master' plus remote ----------
    input  wire                cmd_valid,
    input  wire                cmd_we,
    input  wire [ADDR_W-1:0]   cmd_addr,
    input  wire [DATA_W-1:0]   cmd_wdata,
    output wire                cmd_accept,
    output wire                done,
    output wire [DATA_W-1:0]   rdata,
    output wire [RESP_W-1:0]   resp,
    output wire                err,
    output wire                cmd_error,
    output wire [7:0]          split_count,
    output wire                busy,

    // ---- serial bus, identical to `master' ------------------------------
    output wire                bus_req,
    input  wire                bus_gnt,
    output wire                m_valid,
    output wire                m_we,
    output wire                m_astream,
    output wire                m_dstream,
    input  wire                bus_ready,
    input  wire [RESP_W-1:0]   bus_resp,
    input  wire                bus_dstream,

    // ---- UART pins to the other board -----------------------------------
    input  wire                rm_rx,
    output wire                rm_tx,

    // ---- status ----------------------------------------------------------
    output wire                remote_busy,
    output wire                srv_busy,

    // ---- LINK DIAGNOSTICS ------------------------------------------------
    // You cannot debug a serial link you cannot see.  These say whether
    // anything is arriving at all, what it was, and whether the parser ever
    // made sense of it - which is what separates "no cable / wrong baud"
    // from "cable fine, protocol disagreement".
    output wire [7:0]          dbg_rx_last,     // last byte the UART framed
    output wire [7:0]          dbg_rx_count,    // bytes received, wraps
    output wire [7:0]          dbg_tx_count,    // bytes sent, wraps
    output wire [1:0]          dbg_rx_state,    // 0=hunting 1=in REQ 2=in RESP
    output wire                dbg_req_seen,    // sticky: parsed a REQUEST
    output wire                dbg_resp_seen,   // sticky: parsed a RESPONSE
    output wire                dbg_rx_active,   // the RX line is not idle-high
    output wire                dbg_req_overrun  // sticky: a REQUEST was
                                                // overwritten before the
                                                // server could run it
);

    localparam [7:0] REQ_TAG  = `LINK_REQ_TAG;
    localparam [7:0] RESP_TAG = `LINK_RESP_TAG;

    // The command carried on the wire, fixed by the link spec at 24 bits:
    // {wdata[7:0], dev[1:0], offset[11:0], we, reserved} - and dev+offset is
    // just the far address's low 14 bits.
    localparam CMD_W  = `LINK_CMD_W;             // 24
    localparam FAR_W  = CMD_W - DATA_W - 2;      // 14 address bits on the wire

    // addr[15] set = this transaction belongs to the other board.
    wire cmd_is_remote = cmd_addr[`REMOTE_BIT];

    //======================================================================
    // UART primitives
    //======================================================================
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(rm_rx),
        .rx_data(rx_data), .rx_valid(rx_valid)
    );

    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_data),
        .tx_serial(rm_tx), .tx_busy(tx_busy)
    );

    //======================================================================
    // Byte-stream sender: shifts 1..5 bytes out, least significant first
    //======================================================================
    localparam TXS_IDLE = 2'd0;
    localparam TXS_SEND = 2'd1;
    localparam TXS_WAIT = 2'd2;

    reg [1:0]  txs;
    reg [39:0] txsr;             // tag + up to 4 payload bytes
    reg [2:0]  txcnt;
    reg        tx_go;
    reg [39:0] tx_payload;
    reg [2:0]  tx_len;
    wire       tx_seq_idle = (txs == TXS_IDLE) && !tx_go;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txs      <= TXS_IDLE;
            txsr     <= 40'h0;
            txcnt    <= 3'd0;
            tx_start <= 1'b0;
            tx_data  <= 8'h00;
        end else begin
            case (txs)
                TXS_IDLE: begin
                    tx_start <= 1'b0;
                    if (tx_go) begin
                        txsr  <= tx_payload;
                        txcnt <= tx_len;
                        txs   <= TXS_SEND;
                    end
                end
                TXS_SEND: begin
                    tx_data  <= txsr[7:0];
                    tx_start <= 1'b1;
                    txsr     <= {8'h00, txsr[39:8]};
                    txcnt    <= txcnt - 3'd1;
                    txs      <= TXS_WAIT;
                end
                TXS_WAIT: begin
                    tx_start <= 1'b0;
                    // uart_tx raises tx_busy on the edge it accepts tx_start,
                    // so both low means the byte has actually gone.
                    if (!tx_start && !tx_busy)
                        txs <= (txcnt == 3'd0) ? TXS_IDLE : TXS_SEND;
                end
                default: txs <= TXS_IDLE;
            endcase
        end
    end

    //======================================================================
    // Receive parser: splits the byte stream into requests and responses
    //======================================================================
    localparam R_TAG  = 2'd0;
    localparam R_REQ  = 2'd1;
    localparam R_RESP = 2'd2;

    reg [1:0]  rxs;
    reg [1:0]  rxn;
    reg [CMD_W-1:0] rx_cmd;
    reg [7:0]  resp_data;
    reg        req_valid;
    reg        resp_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxs        <= R_TAG;
            rxn        <= 2'd0;
            rx_cmd     <= {CMD_W{1'b0}};
            resp_data  <= 8'h00;
            req_valid  <= 1'b0;
            resp_valid <= 1'b0;
        end else begin
            req_valid  <= 1'b0;
            resp_valid <= 1'b0;

            if (rx_valid) begin
                case (rxs)
                    R_TAG: begin
                        rxn <= 2'd0;
                        if      (rx_data == REQ_TAG)  rxs <= R_REQ;
                        else if (rx_data == RESP_TAG) rxs <= R_RESP;
                        // anything else: stay hunting for a tag
                    end
                    // Exactly three more bytes, then stop.  A payload byte
                    // may itself be 0xA5 or 0x5A; re-scanning it for tags
                    // would desynchronise the stream.
                    R_REQ: begin
                        case (rxn)
                            2'd0: rx_cmd[7:0]   <= rx_data;
                            2'd1: rx_cmd[15:8]  <= rx_data;
                            2'd2: rx_cmd[23:16] <= rx_data;
                            default: ;
                        endcase
                        if (rxn == 2'd2) begin
                            req_valid <= 1'b1;
                            rxs       <= R_TAG;
                        end else
                            rxn <= rxn + 2'd1;
                    end
                    R_RESP: begin
                        resp_data  <= rx_data;
                        resp_valid <= 1'b1;
                        rxs        <= R_TAG;
                    end
                    default: rxs <= R_TAG;
                endcase
            end
        end
    end

    //======================================================================
    // Link diagnostics
    //
    // Deliberately dumb and free-running: they are what you read when the
    // far board is silent and you need to know whose fault it is.  Nothing
    // here affects a transfer.
    //======================================================================
    reg [7:0] rx_last, rx_count, tx_count;
    reg       req_seen, resp_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_last   <= 8'h00;
            rx_count  <= 8'd0;
            tx_count  <= 8'd0;
            req_seen  <= 1'b0;
            resp_seen <= 1'b0;
        end else begin
            if (rx_valid) begin
                rx_last  <= rx_data;
                rx_count <= rx_count + 8'd1;
            end
            // one count per byte handed to the transmitter
            if (txs == TXS_SEND) tx_count <= tx_count + 8'd1;
            if (req_valid)  req_seen  <= 1'b1;
            if (resp_valid) resp_seen <= 1'b1;
        end
    end

    assign dbg_rx_last   = rx_last;
    assign dbg_rx_count  = rx_count;
    assign dbg_tx_count  = tx_count;
    assign dbg_rx_state  = rxs;
    assign dbg_req_seen  = req_seen;
    assign dbg_resp_seen = resp_seen;
    assign dbg_rx_active = ~rm_rx;      // an idle 8N1 line sits HIGH
    assign dbg_req_overrun = req_overrun;

    //======================================================================
    // The master core.  EVERY local bus access goes through this, whether it
    // came from our own command port or from the other board.
    //======================================================================
    reg               srv_active;
    reg               srv_valid;
    reg  [ADDR_W-1:0] srv_addr;
    reg  [DATA_W-1:0] srv_wdata;
    reg               srv_we;

    // A local command reaches the core only when we are not serving, and
    // only when it is not marked remote.
    wire              core_valid = srv_active ? srv_valid
                                              : (cmd_valid && !cmd_is_remote);
    wire              core_we    = srv_active ? srv_we    : cmd_we;
    wire [ADDR_W-1:0] core_addr  = srv_active ? srv_addr  : cmd_addr;
    wire [DATA_W-1:0] core_wdata = srv_active ? srv_wdata : cmd_wdata;

    wire              core_accept, core_done, core_busy;
    wire [DATA_W-1:0] core_rdata;
    wire [RESP_W-1:0] core_resp;

    master #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RESP_W (RESP_W)
    ) u_core (
        .clk         (clk),
        .rst_n       (rst_n),
        .cmd_valid   (core_valid),
        .cmd_we      (core_we),
        .cmd_addr    (core_addr),
        .cmd_wdata   (core_wdata),
        .cmd_accept  (core_accept),
        .done        (core_done),
        .rdata       (core_rdata),
        .resp        (core_resp),
        .err         (),                       // recomputed below
        .split_count (split_count),
        .busy        (core_busy),
        .state       (),                       // waveform only
        .bus_req     (bus_req),
        .bus_gnt     (bus_gnt),
        .m_valid     (m_valid),
        .m_we        (m_we),
        .m_astream   (m_astream),
        .m_dstream   (m_dstream),
        .bus_ready   (bus_ready),
        .bus_resp    (bus_resp),
        .bus_dstream (bus_dstream)
    );

    //======================================================================
    // Client and server
    //======================================================================
    localparam C_IDLE = 2'd0;
    localparam C_SEND = 2'd1;
    localparam C_WAIT = 2'd2;

    localparam S_IDLE = 2'd0;
    localparam S_EXEC = 2'd1;
    localparam S_RESP = 2'd2;

    reg [1:0]        cs, ss;
    reg [CMD_W-1:0]  out_cmd;
    reg [31:0]       to_cnt;               // 32 bits: RESP_TIMEOUT must fit
    reg [DATA_W-1:0] srv_result;
    reg              req_hold;
    reg [CMD_W-1:0]  req_cmd_hold;
    reg              req_overrun;

    // Remote result, and which kind of transaction produced the last one.
    reg [DATA_W-1:0] rem_rdata;
    reg [RESP_W-1:0] rem_resp;
    reg              rem_done;
    reg              rem_err;
    reg              rem_we;     // the outstanding remote command's direction
    reg              last_was_remote;

    assign remote_busy = (cs != C_IDLE);
    assign srv_busy    = srv_active;
    assign cmd_error   = rem_err;

    // The client accepts its own command; the core accepts local ones.  Both
    // are one-cycle pulses, so the outside sees one uniform handshake.
    wire rem_accept = cmd_valid && cmd_is_remote && (cs == C_IDLE);
    assign cmd_accept = cmd_is_remote ? rem_accept
                                   : (srv_active ? 1'b0 : core_accept);

    // A completion belongs to us unless the core is running the other
    // board's transaction, in which case the server consumes it.
    assign done  = rem_done | (core_done && !srv_active);
    assign rdata = last_was_remote ? rem_rdata : core_rdata;
    assign resp  = last_was_remote ? rem_resp  : core_resp;
    assign err   = (resp == `RESP_ERROR);
    assign busy  = core_busy | remote_busy | srv_active;

    // The core is free to be taken by the server only when it is idle and no
    // local command is being offered this cycle.  Otherwise a local transfer
    // could start now and complete while srv_active was high, and its `done'
    // would be swallowed.
    wire core_free = !core_busy && !(cmd_valid && !cmd_is_remote);

    // The cycle the server actually picks the held request up.  Naming it
    // here instead of burying the condition in the S_IDLE branch is what lets
    // the receive-side latch below tell "consumed" from "overwritten".
    wire srv_take = (ss == S_IDLE) && req_hold && core_free && (cs != C_SEND);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs              <= C_IDLE;
            ss              <= S_IDLE;
            srv_active      <= 1'b0;
            srv_valid       <= 1'b0;
            srv_addr        <= {ADDR_W{1'b0}};
            srv_wdata       <= {DATA_W{1'b0}};
            srv_we          <= 1'b0;
            srv_result      <= {DATA_W{1'b0}};
            req_hold        <= 1'b0;
            req_cmd_hold    <= {CMD_W{1'b0}};
            req_overrun     <= 1'b0;
            out_cmd         <= {CMD_W{1'b0}};
            to_cnt          <= 32'd0;
            tx_go           <= 1'b0;
            tx_payload      <= 40'h0;
            tx_len          <= 3'd0;
            rem_rdata       <= {DATA_W{1'b0}};
            rem_resp        <= `RESP_OKAY;
            rem_done        <= 1'b0;
            rem_err         <= 1'b0;
            rem_we          <= 1'b0;
            last_was_remote <= 1'b0;
        end else begin
            rem_done <= 1'b0;
            tx_go    <= 1'b0;

            // Latch an incoming request.  Only one may be outstanding, and
            // the link has NO FLOW CONTROL to push back with - writes are
            // posted and the wire format has no ACK - so two things matter:
            //
            //  * A request arriving on the VERY CYCLE the server takes the
            //    previous one must survive.  Clearing req_hold inside the
            //    S_IDLE branch below would win the last-assignment race and
            //    drop it, so the flag is cleared HERE instead, and only when
            //    nothing new arrived on the same cycle.
            //
            //  * A request that genuinely overwrites an unconsumed one is
            //    gone and cannot be recovered, so it is FLAGGED.  That
            //    happens when the server is blocked for longer than a frame
            //    takes to arrive - the core parked in a long split, or our
            //    own client queued behind the transmitter.  A silent drop is
            //    exactly the failure this link's diagnostics exist to catch.
            if (req_valid) begin
                req_cmd_hold <= rx_cmd;
                req_hold     <= 1'b1;
                if (req_hold && !srv_take) req_overrun <= 1'b1;
            end else if (srv_take) begin
                req_hold <= 1'b0;
            end

            // Remember which path the current transaction took, so `rdata'
            // and `resp' mux to the right source when it completes.
            //
            // `cmd_error' describes the LAST transaction, not the session, so
            // accepting a local command clears it - the same semantics
            // System_Bus_Final's master_node_uart has.  The ISSP driver keeps
            // its own sticky copy for the host to poll.
            if (cmd_accept) begin
                last_was_remote <= cmd_is_remote;
                if (!cmd_is_remote) rem_err <= 1'b0;
            end

            //--------------------------------------------------------------
            // Server: the other board's transactions, run on our bus
            //--------------------------------------------------------------
            case (ss)
                S_IDLE: begin
                    // req_hold is cleared by the receive-side latch above, so
                    // that a request landing on this same cycle is not lost.
                    if (srv_take) begin
                        srv_active <= 1'b1;
                        srv_we     <= req_cmd_hold[1];
                        // {dev, offset} is the far address's low 14 bits; the
                        // top two are always 00 here, which is what stops a
                        // request decoding back into our own remote window.
                        srv_addr   <= {{(ADDR_W-FAR_W){1'b0}},
                                       req_cmd_hold[FAR_W+1:2]};
                        srv_wdata  <= req_cmd_hold[CMD_W-1 -: DATA_W];
                        srv_valid  <= 1'b1;
                        ss         <= S_EXEC;
                    end
                end
                S_EXEC: begin
                    // The core takes a LEVEL valid; drop it once accepted.
                    if (core_accept) srv_valid <= 1'b0;
                    if (core_done) begin
                        srv_result <= core_rdata;
                        // WRITES ARE POSTED: the far side expects nothing
                        // back, so a write retires the server immediately and
                        // only a read goes on to send a RESPONSE.
                        if (srv_we) begin
                            srv_active <= 1'b0;
                            ss         <= S_IDLE;
                        end else begin
                            ss <= S_RESP;
                        end
                    end
                end
                S_RESP: begin
                    if (tx_seq_idle) begin
                        tx_payload <= {24'h0, srv_result, RESP_TAG};
                        tx_len     <= 3'd2;
                        tx_go      <= 1'b1;
                        srv_active <= 1'b0;
                        ss         <= S_IDLE;
                    end
                end
                default: ss <= S_IDLE;
            endcase

            //--------------------------------------------------------------
            // Client: our own remote transactions
            //--------------------------------------------------------------
            case (cs)
                C_IDLE: begin
                    if (rem_accept) begin
                        // {wdata, dev, offset, we, 0}.  dev+offset is the far
                        // address's low 14 bits, and the far address is the
                        // local one minus 0x8000 - which, since addr[15] is
                        // the only bit above them that is set, is just
                        // cmd_addr[13:0] taken as-is.
                        out_cmd <= {cmd_wdata, cmd_addr[FAR_W-1:0], cmd_we, 1'b0};
                        rem_we  <= cmd_we;
                        rem_err <= 1'b0;
                        cs      <= C_SEND;
                    end
                end
                C_SEND: begin
                    // Responses take priority: never start a request while
                    // the server owes the other board an answer.
                    if (tx_seq_idle && ss != S_RESP) begin
                        tx_payload <= {{(40-8-CMD_W){1'b0}}, out_cmd, REQ_TAG};
                        tx_len     <= 3'd4;      // tag + 3 payload bytes
                        tx_go      <= 1'b1;
                        to_cnt     <= 32'd0;
                        if (rem_we) begin
                            // WRITES ARE POSTED.  Nothing comes back, so the
                            // request being on the wire IS the completion.
                            rem_rdata <= {DATA_W{1'b0}};
                            rem_resp  <= `RESP_OKAY;
                            rem_done  <= 1'b1;
                            rem_err   <= 1'b0;
                            cs        <= C_IDLE;
                        end else begin
                            cs <= C_WAIT;
                        end
                    end
                end
                // Only a READ ever reaches here - writes are posted.
                C_WAIT: begin
                    if (resp_valid) begin
                        rem_rdata <= resp_data;
                        rem_resp  <= `RESP_OKAY;
                        rem_done  <= 1'b1;
                        rem_err   <= 1'b0;
                        cs        <= C_IDLE;
                    end else if (to_cnt >= RESP_TIMEOUT) begin
                        // Link dead or unplugged: complete with an error
                        // rather than hang the master forever.
                        rem_rdata <= {DATA_W{1'b1}};
                        rem_resp  <= `RESP_ERROR;
                        rem_done  <= 1'b1;
                        rem_err   <= 1'b1;
                        cs        <= C_IDLE;
                    end else
                        to_cnt <= to_cnt + 32'd1;
                end
                default: cs <= C_IDLE;
            endcase
        end
    end

endmodule
