//==========================================================================
// bus_bridge.v -- the board-to-board link, as a DEVICE ON THE BUS
//
// There is exactly one UART in the design and it lives in here.  No master
// owns it.  The bridge is a device with TWO FACES, which is what lets either
// local master reach the far board and lets the far board reach all three
// local memories:
//
//   SLAVE FACE   decoded target 3, 0x8000-0xBFFF.  A local master reaches
//                the other board by addressing it exactly as it addresses a
//                memory.  Serialisation, decode and arbitration are the
//                bus's normal ones; nothing about the transaction is special
//                until it gets in here.
//
//   MASTER FACE  arbiter index N_MASTERS-1, the LOWEST priority.  Requests
//                arriving from the far board are issued on this bus through
//                an ordinary `master' core, so remote traffic can never
//                out-rank the local masters.
//
// far address = local address - 0x8000, and the window is exactly 16K -
// which is exactly the 14 address bits the link carries, so nothing is
// silently truncated on the way out.
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
// `dev' and `offset' together are simply addr[13:0], which is what the slave
// face collects off the serial wire in one 14-bit deserialiser.
//
//--------------------------------------------------------------------------
// A REMOTE TRANSACTION IS A SPLIT TRANSACTION
//--------------------------------------------------------------------------
// A round trip costs 347 us for the request and another 174 us for a reply -
// tens of thousands of clocks.  Holding the bus for that would be absurd, so
// the bridge answers SPLIT on the first access, exactly as the split-capable
// memory does:
//
//   S      master addresses the bridge -> SPLIT.  The arbiter masks that
//          master and RELEASES THE BUS.  The other master runs at full speed.
//   ...    the bridge sends the request and, on a read, waits for the reply
//   T      split_complete[id] pulses, the arbiter unmasks the master
//   T+n    the master re-issues the identical frame and the bridge serves it:
//          a write answers OKAY at S+1, a read shifts 8 bits out and answers
//          at S+10 - the same shape as a memory read, so nothing upstream
//          needs to know this one went over a wire.
//
// WRITES SPLIT TOO, and complete once the request bytes are on the wire.
// That is what makes the link safe to drive hard: the master cannot issue a
// second remote write until the first has actually gone, so the far board's
// single-byte receiver can never be overrun by us.  It costs one replay and
// buys back-pressure the wire format itself does not provide.
//
// ONE TRANSACTION AT A TIME.  A second master addressing the bridge while a
// round trip is in flight is answered ERROR, not deferred: there is one set
// of deferred state and one split_complete to release it with, so queueing a
// second would need a second of each.  ERROR completes, the master reports
// it, and the bus does not hang - the same discipline as the default slave.
//
//--------------------------------------------------------------------------
// Things that are load-bearing
//--------------------------------------------------------------------------
// * RESPONSES TAKE PRIORITY over requests in the shared transmitter.  If both
//   boards issue at the same instant and each preferred its own request, both
//   would wait for an answer neither is sending.
//
// * TAG HUNTING.  Hunt a tag, then take exactly 3 more bytes (request) or 1
//   (response).  Never re-scan the payload - a payload byte may be 0xA5/0x5A.
//
// * THE LINK IS LOOP-FREE BY CONSTRUCTION.  The server issues
//   {2'b00, addr[13:0]}, so a received request can only ever reach device ids
//   0-2 - the three real memories.  It can never address the bridge itself,
//   so nothing can be forwarded back out.  No hop count is needed.
//
// * RESP_TIMEOUT (32 bits) completes a remote READ with 0xFF and latches
//   br_error rather than hanging.  A counter narrower than its own parameter
//   truncates silently.
//
// * THE UART IS REACHED THROUGH A NARROW BYTE INTERFACE - a byte plus enable
//   and busy going out, a byte plus ready coming in.  Anything honouring that
//   can replace it; the framing, both faces and the split handling stay.
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module bus_bridge #(
    parameter ADDR_W       = `BUS_ADDR_W,
    parameter DATA_W       = `BUS_DATA_W,
    parameter RESP_W       = `BUS_RESP_W,
    parameter N_MASTERS    = `BUS_N_MASTERS,
    parameter ID_W         = `BUS_ID_W,
    parameter LADDR_W      = `S3_LADDR_W,     // 14 - the window's own offset
    parameter CLKS_PER_BIT = 434,             // 50 MHz / 115200
    parameter RESP_TIMEOUT = 500000           // ~10 ms at 50 MHz
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- SLAVE FACE: identical in shape to `slave' ----------------------
    input  wire                  frame,
    input  wire                  astream,
    input  wire                  dstream_in,
    input  wire                  sel,
    input  wire                  we,
    input  wire [ID_W-1:0]       master_id,
    output wire                  dstream_out,
    output reg                   ready,
    output reg  [RESP_W-1:0]     resp,
    output wire [N_MASTERS-1:0]  split_complete,
    output wire                  busy,

    // ---- MASTER FACE: identical in shape to `master' --------------------
    output wire                  bus_req,
    input  wire                  bus_gnt,
    output wire                  m_valid,
    output wire                  m_we,
    output wire                  m_astream,
    output wire                  m_dstream,
    input  wire                  bus_ready,
    input  wire [RESP_W-1:0]     bus_resp,
    input  wire                  bus_dstream,

    // ---- the wire -------------------------------------------------------
    input  wire                  rm_rx,
    output wire                  rm_tx,

    // ---- status ---------------------------------------------------------
    output wire                  br_error,        // last remote read timed out
    output wire                  remote_busy,     // a round trip is in flight
    output wire                  srv_busy,        // serving the far board now

    // ---- LINK DIAGNOSTICS -----------------------------------------------
    // You cannot debug a serial link you cannot see.
    output wire [7:0]            dbg_rx_last,
    output wire [7:0]            dbg_rx_count,
    output wire [7:0]            dbg_tx_count,
    output wire [1:0]            dbg_rx_state,
    output wire                  dbg_req_seen,
    output wire                  dbg_resp_seen,
    output wire                  dbg_rx_active,
    output wire                  dbg_req_overrun
);

    localparam [7:0] REQ_TAG  = `LINK_REQ_TAG;
    localparam [7:0] RESP_TAG = `LINK_RESP_TAG;
    localparam CMD_W = `LINK_CMD_W;                  // 24
    localparam FAR_W = CMD_W - DATA_W - 2;           // 14

    function integer clogb2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clogb2 = 1; v > 1; clogb2 = clogb2 + 1)
                v = v >> 1;
        end
    endfunction
    localparam BCNT_W = clogb2(DATA_W);

    //======================================================================
    // The wire, and the narrow byte interface onto it
    //======================================================================
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(rm_rx), .rx_data(rx_data), .rx_valid(rx_valid)
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
    // Byte-stream sender: 1..5 bytes out, least significant first
    //======================================================================
    localparam TXS_IDLE = 2'd0;
    localparam TXS_SEND = 2'd1;
    localparam TXS_WAIT = 2'd2;

    reg [1:0]  txs;
    reg [39:0] txsr;
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

    reg [1:0]       rxs;
    reg [1:0]       rxn;
    reg [CMD_W-1:0] rx_cmd;
    reg [7:0]       resp_data;
    reg             req_valid;
    reg             resp_valid;

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
    // Link diagnostics.  Free-running; nothing here affects a transfer.
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

    //======================================================================
    // SLAVE FACE - deserialisers.  Every target on this bus shifts every
    // frame in speculatively; selection arrives afterwards.
    //
    // LADDR_W is 14, so this holds addr[13:0] - which is precisely the field
    // the link carries.  No arithmetic is needed to turn a local address
    // into a far one; the window's base simply is not in these bits.
    //======================================================================
    wire [LADDR_W-1:0] laddr;
    wire [DATA_W-1:0]  wdata_des;

    shift_deser #(.W(LADDR_W)) u_addr_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(astream), .dout(laddr)
    );

    shift_deser #(.W(DATA_W)) u_wdata_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(dstream_in), .dout(wdata_des)
    );

    //======================================================================
    // CLIENT: a local master's transaction, carried to the far board
    //======================================================================
    localparam B_IDLE = 2'd0;   // no round trip in flight
    localparam B_SEND = 2'd1;   // request built, waiting for the transmitter
    localparam B_WAIT = 2'd2;   // read: waiting for the far board's answer
    localparam B_RDY  = 2'd3;   // answer in hand, waiting for the replay

    reg [1:0]            bs;
    reg [ID_W-1:0]       own_id;      // which master we deferred
    reg [LADDR_W-1:0]    own_addr;
    reg [DATA_W-1:0]     own_wdata;
    reg                  own_we;
    reg [DATA_W-1:0]     rd_byte;     // what the far board sent back
    reg [31:0]           to_cnt;      // 32 bits: RESP_TIMEOUT must fit
    reg                  err_r;
    reg [N_MASTERS-1:0]  sc_r;

    // The replay: the same master coming back for the answer we are holding.
    wire is_replay = (bs == B_RDY) && (master_id == own_id);

    // A fresh access we can take on.
    wire take_new  = sel && (bs == B_IDLE);
    // Anything else addressed at us while busy - answered ERROR, never left
    // hanging.  The owner cannot be here: the arbiter has it masked.
    wire reject    = sel && (bs != B_IDLE) && !is_replay;
    // The replay actually being served.
    wire serve     = sel && is_replay;

    assign split_complete = sc_r;
    assign busy           = (bs != B_IDLE);
    assign remote_busy    = (bs != B_IDLE);
    assign br_error       = err_r;

    //----------------------------------------------------------------------
    // SERVER: the far board's transaction, issued on OUR bus
    //======================================================================
    localparam S_IDLE = 2'd0;
    localparam S_EXEC = 2'd1;
    localparam S_RESP = 2'd2;

    reg [1:0]        ss;
    reg              srv_valid;
    reg [ADDR_W-1:0] srv_addr;
    reg [DATA_W-1:0] srv_wdata;
    reg              srv_we;
    reg [DATA_W-1:0] srv_result;
    reg              req_hold;
    reg [CMD_W-1:0]  req_cmd_hold;
    reg              req_overrun;

    wire              core_accept, core_done, core_busy;
    wire [DATA_W-1:0] core_rdata;

    assign srv_busy        = (ss != S_IDLE);
    assign dbg_req_overrun = req_overrun;

    // The master face is an ordinary `master'.  It is driven ONLY by the
    // server - a local command never enters here, which is the whole point of
    // taking the UART out of the master.
    master #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .RESP_W (RESP_W)
    ) u_core (
        .clk         (clk),
        .rst_n       (rst_n),
        .cmd_valid   (srv_valid),
        .cmd_we      (srv_we),
        .cmd_addr    (srv_addr),
        .cmd_wdata   (srv_wdata),
        .cmd_accept  (core_accept),
        .done        (core_done),
        .rdata       (core_rdata),
        .resp        (),
        .err         (),
        .split_count (),
        .busy        (core_busy),
        .state       (),
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

    // The cycle the server picks the held request up.  Naming it here is what
    // lets the receive latch below tell "consumed" from "overwritten".
    wire srv_take = (ss == S_IDLE) && req_hold && !core_busy && (bs != B_SEND);

    //======================================================================
    // The two faces, and the transmitter they share
    //======================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs           <= B_IDLE;
            own_id       <= {ID_W{1'b0}};
            own_addr     <= {LADDR_W{1'b0}};
            own_wdata    <= {DATA_W{1'b0}};
            own_we       <= 1'b0;
            rd_byte      <= {DATA_W{1'b0}};
            to_cnt       <= 32'd0;
            err_r        <= 1'b0;
            sc_r         <= {N_MASTERS{1'b0}};
            ss           <= S_IDLE;
            srv_valid    <= 1'b0;
            srv_addr     <= {ADDR_W{1'b0}};
            srv_wdata    <= {DATA_W{1'b0}};
            srv_we       <= 1'b0;
            srv_result   <= {DATA_W{1'b0}};
            req_hold     <= 1'b0;
            req_cmd_hold <= {CMD_W{1'b0}};
            req_overrun  <= 1'b0;
            tx_go        <= 1'b0;
            tx_payload   <= 40'h0;
            tx_len       <= 3'd0;
        end else begin
            sc_r  <= {N_MASTERS{1'b0}};      // default: one-cycle pulse
            tx_go <= 1'b0;

            //--------------------------------------------------------------
            // Latch an incoming request.  The link has NO FLOW CONTROL to
            // push back with, so a request that overwrites an unconsumed one
            // is gone and is FLAGGED rather than hidden.  A request arriving
            // on the very cycle the server takes the previous one must
            // survive, so req_hold is cleared HERE and only when nothing new
            // arrived on the same cycle.
            //--------------------------------------------------------------
            if (req_valid) begin
                req_cmd_hold <= rx_cmd;
                req_hold     <= 1'b1;
                if (req_hold && !srv_take) req_overrun <= 1'b1;
            end else if (srv_take) begin
                req_hold <= 1'b0;
            end

            //--------------------------------------------------------------
            // SERVER - runs first, so RESPONSES BEAT REQUESTS for the
            // transmitter.  The client below checks `ss != S_RESP' against
            // the registered value, so the two can never both claim it.
            //--------------------------------------------------------------
            case (ss)
                S_IDLE: begin
                    if (srv_take) begin
                        srv_we    <= req_cmd_hold[1];
                        // {2'b00, addr[13:0]}: the top two bits are ALWAYS
                        // zero, which is what stops a received request from
                        // decoding back into our own bridge window.  The
                        // link is loop-free by construction.
                        srv_addr  <= {{(ADDR_W-FAR_W){1'b0}},
                                      req_cmd_hold[FAR_W+1:2]};
                        srv_wdata <= req_cmd_hold[CMD_W-1 -: DATA_W];
                        srv_valid <= 1'b1;
                        ss        <= S_EXEC;
                    end
                end
                S_EXEC: begin
                    if (core_accept) srv_valid <= 1'b0;
                    if (core_done) begin
                        srv_result <= core_rdata;
                        // WRITES ARE POSTED: the far side expects nothing
                        // back, so only a read goes on to send a RESPONSE.
                        ss <= srv_we ? S_IDLE : S_RESP;
                    end
                end
                S_RESP: begin
                    if (tx_seq_idle) begin
                        tx_payload <= {24'h0, srv_result, RESP_TAG};
                        tx_len     <= 3'd2;
                        tx_go      <= 1'b1;
                        ss         <= S_IDLE;
                    end
                end
                default: ss <= S_IDLE;
            endcase

            //--------------------------------------------------------------
            // CLIENT
            //--------------------------------------------------------------
            case (bs)
                B_IDLE: begin
                    if (sel) begin
                        own_id    <= master_id;
                        own_addr  <= laddr;
                        own_wdata <= wdata_des;
                        own_we    <= we;
                        err_r     <= 1'b0;
                        to_cnt    <= 32'd0;
                        bs        <= B_SEND;
                    end
                end

                B_SEND: begin
                    // Responses take priority: never start a request while
                    // the server owes the other board an answer.
                    if (tx_seq_idle && ss != S_RESP) begin
                        tx_payload <= {{(40-8-CMD_W){1'b0}},
                                       own_wdata, own_addr, own_we, 1'b0,
                                       REQ_TAG};
                        tx_len     <= 3'd4;      // tag + 3 payload bytes
                        tx_go      <= 1'b1;
                        to_cnt     <= 32'd0;
                        if (own_we) begin
                            // The request is on the wire; that IS the
                            // completion for a posted write.  Wake the
                            // master so it can replay and retire.
                            sc_r[own_id] <= 1'b1;
                            bs           <= B_RDY;
                        end else begin
                            bs <= B_WAIT;
                        end
                    end
                end

                // Only a READ reaches here - writes are posted.
                B_WAIT: begin
                    if (resp_valid) begin
                        rd_byte      <= resp_data;
                        sc_r[own_id] <= 1'b1;
                        bs           <= B_RDY;
                    end else if (to_cnt >= RESP_TIMEOUT) begin
                        // Link dead or unplugged: complete with an error
                        // rather than hang the master forever.
                        rd_byte      <= {DATA_W{1'b1}};
                        err_r        <= 1'b1;
                        sc_r[own_id] <= 1'b1;
                        bs           <= B_RDY;
                    end else
                        to_cnt <= to_cnt + 32'd1;
                end

                B_RDY: begin
                    // The masked master has been released; when it re-issues
                    // the identical frame we serve it and are free again.
                    if (serve) bs <= B_IDLE;
                end

                default: bs <= B_IDLE;
            endcase
        end
    end

    //======================================================================
    // Read data phase, shaped exactly like a memory slave's so that nothing
    // upstream can tell this word came off a wire:
    //   S     `serve' - the replay is selected
    //   S+1   the byte is loaded into the output register
    //   S+2 .. S+9   DATA_W bits out on dstream_out, MSB first
    //   S+10  ready + OKAY
    //======================================================================
    localparam R_IDLE_D  = 2'd0;
    localparam R_LOAD    = 2'd1;
    localparam R_SHIFT   = 2'd2;

    reg [1:0]        rs;
    reg [BCNT_W-1:0] rcnt;
    reg [DATA_W-1:0] rd_sr;

    wire serve_read  = serve && !own_we;
    wire serve_write = serve &&  own_we;
    wire rd_last     = (rs == R_SHIFT) && (rcnt == (DATA_W-1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rs   <= R_IDLE_D;
            rcnt <= {BCNT_W{1'b0}};
        end else begin
            case (rs)
                R_IDLE_D: if (serve_read) rs <= R_LOAD;
                R_LOAD:   begin rs <= R_SHIFT; rcnt <= {BCNT_W{1'b0}}; end
                R_SHIFT:  begin
                    rcnt <= rcnt + 1'b1;
                    if (rd_last) rs <= R_IDLE_D;
                end
                default: rs <= R_IDLE_D;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             rd_sr <= {DATA_W{1'b0}};
        else if (rs == R_LOAD)  rd_sr <= rd_byte;
        else if (rs == R_SHIFT) rd_sr <= {rd_sr[DATA_W-2:0], 1'b0};
    end

    // Quiet unless actually sending, so an idle bus does not look like
    // traffic in a waveform.
    assign dstream_out = (rs == R_SHIFT) ? rd_sr[DATA_W-1] : 1'b0;

    //======================================================================
    // Handshake.  A fresh access is deferred (SPLIT), a rejected one is
    // ERROR, a replayed write answers OKAY at once and a replayed read when
    // its last bit has gone.  Exactly one of these can be true at a time.
    //
    // A ROUND TRIP THAT TIMED OUT COMPLETES WITH ERROR, not OKAY.  The data
    // is 0xFF either way, but a transaction the far board never answered has
    // failed and the master must be able to see that on `resp' alone - the
    // same channel an unmapped address uses.  `br_error' is the finer-grained
    // report that separates "the link died" from "that address is not
    // mapped"; it does not replace saying so on the bus.
    //
    // err_r is cleared when a fresh access is taken on, so it can only ever
    // describe the transaction being completed here.
    //======================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            resp  <= `RESP_OKAY;
        end else begin
            ready <= take_new || reject || serve_write || rd_last;
            resp  <= take_new         ? `RESP_SPLIT
                   : (reject || err_r) ? `RESP_ERROR
                                       : `RESP_OKAY;
        end
    end

endmodule
