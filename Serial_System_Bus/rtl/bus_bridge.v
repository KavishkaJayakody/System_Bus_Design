// The board-to-board link, as a DEVICE ON THE BUS - not part of any master.
// Two faces:
//   slave face   target 3 at 0x8000-0xBFFF; either master reaches the far
//                board by addressing it like a memory
//   master face  bus master N_MASTERS-1, LOWEST priority, so remote traffic
//                cannot out-rank local traffic
//
// far address = local - 0x8000, and only addr[13:0] travels.
//
// Wire format (8N1, 115200):
//   REQUEST   A5 b0 b1 b2     RESPONSE  5A data   (READS ONLY)
//   cmd = {wdata[7:0], addr[13:0], we, 1'b0}, little-endian
//
// A remote transaction is a SPLIT transaction: the round trip is ~347us each
// way, so the bridge defers the master and frees the bus.

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

    output wire                  bus_req,
    input  wire                  bus_gnt,
    output wire                  m_valid,
    output wire                  m_we,
    output wire                  m_astream,
    output wire                  m_dstream,
    input  wire                  bus_ready,
    input  wire [RESP_W-1:0]     bus_resp,
    input  wire                  bus_dstream,

    input  wire                  rm_rx,
    output wire                  rm_tx,

    output wire                  br_error,        // last remote read timed out
    output wire                  remote_busy,     // a round trip is in flight
    output wire                  srv_busy,        // serving the far board now

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
                    if (!tx_start && !tx_busy)
                        txs <= (txcnt == 3'd0) ? TXS_IDLE : TXS_SEND;
                end
                default: txs <= TXS_IDLE;
            endcase
        end
    end

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
                    // may itself be 0xA5 or 0x5A; re-scanning it for tags
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

    wire is_replay = (bs == B_RDY) && (master_id == own_id);

    wire take_new  = sel && (bs == B_IDLE);
    // Busy: a second master is answered ERROR, not deferred - there is one set
    // of deferred state and one split_complete.  It completes; nothing hangs.
    wire reject    = sel && (bs != B_IDLE) && !is_replay;
    wire serve     = sel && is_replay;

    assign split_complete = sc_r;
    assign busy           = (bs != B_IDLE);
    assign remote_busy    = (bs != B_IDLE);
    assign br_error       = err_r;

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

    // The master face is driven ONLY by the server - no local command enters
    // here.  That is the whole point of taking the UART out of master 0.
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

    wire srv_take = (ss == S_IDLE) && req_hold && !core_busy && (bs != B_SEND);

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

            if (req_valid) begin
                req_cmd_hold <= rx_cmd;
                req_hold     <= 1'b1;
                if (req_hold && !srv_take) req_overrun <= 1'b1;
            end else if (srv_take) begin
                req_hold <= 1'b0;
            end

            // Server runs first, so RESPONSES BEAT REQUESTS for the
            // transmitter; the client's `ss != S_RESP' uses the registered
            // value, so the two can never both claim it.
            case (ss)
                S_IDLE: begin
                    if (srv_take) begin
                        srv_we    <= req_cmd_hold[1];
                        // {2'b00, addr[13:0]}: the top bits are always zero,
                        // so a received request can never decode back into
                        // our own bridge window.  Loop-free by construction.
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
                        // WRITES ARE POSTED - only a read sends a RESPONSE.
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
                    // Never start a request while the server owes an answer.
                    if (tx_seq_idle && ss != S_RESP) begin
                        tx_payload <= {{(40-8-CMD_W){1'b0}},
                                       own_wdata, own_addr, own_we, 1'b0,
                                       REQ_TAG};
                        tx_len     <= 3'd4;      // tag + 3 payload bytes
                        tx_go      <= 1'b1;
                        to_cnt     <= 32'd0;
                        if (own_we) begin
                            sc_r[own_id] <= 1'b1;
                            bs           <= B_RDY;
                        end else begin
                            bs <= B_WAIT;
                        end
                    end
                end

                B_WAIT: begin
                    if (resp_valid) begin
                        rd_byte      <= resp_data;
                        sc_r[own_id] <= 1'b1;
                        bs           <= B_RDY;
                    end else if (to_cnt >= RESP_TIMEOUT) begin
                        rd_byte      <= {DATA_W{1'b1}};
                        err_r        <= 1'b1;
                        sc_r[own_id] <= 1'b1;
                        bs           <= B_RDY;
                    end else
                        to_cnt <= to_cnt + 32'd1;
                end

                B_RDY: begin
                    if (serve) bs <= B_IDLE;
                end

                default: bs <= B_IDLE;
            endcase
        end
    end

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

    assign dstream_out = (rs == R_SHIFT) ? rd_sr[DATA_W-1] : 1'b0;

    // A timed-out round trip completes with ERROR, not OKAY: the data is 0xFF
    // either way, but the master must see the failure on `resp' alone.
    // err_r is cleared when a fresh access starts, so it only ever describes
    // the transaction being completed here.
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
