`timescale 1ns / 1ps
//
// master_node_uart -- Master 0 with a UART link built in.
//
// A transaction is either LOCAL (cmd_remote = 0), which behaves exactly as
// master_node always did, or REMOTE (cmd_remote = 1), which is carried over
// UART to the other board's master and executed on ITS bus. Reads bring the
// data back, writes bring an acknowledgement, so cmd_done means the far side
// really did it.
//
// The block is symmetric: it is also the SERVER for the other board. A request
// arriving on the UART is executed on the local bus through the same
// master_node core and the result is sent back.
//
//   client:  cmd_remote -> send REQ -> wait RESP -> cmd_rdata / cmd_done
//   server:  REQ arrives -> run on local bus -> send RESP
//
// Wire format (8N1), tagged so requests and responses cannot be confused:
//   REQUEST   0xA5, b0, b1, b2      b2:b1:b0 = the 24-bit command
//   RESPONSE  0x5A, data            read data, or 0x00 acknowledging a write
//
// 24-bit command layout (same field order as one ISSP source slice):
//   cmd[0] reserved   cmd[1] we   cmd[15:2] addr   cmd[23:16] wdata
//
// If no response arrives within RESP_TIMEOUT clocks the transaction is
// completed anyway with cmd_error set, so a dead or unplugged link reports a
// failure instead of hanging the bus forever.
//
module master_node_uart #(
    parameter CLKS_PER_BIT  = 434,      // 50 MHz / 115200
    parameter RESP_TIMEOUT  = 500000    // ~10 ms at 50 MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // Command interface
    input  wire        cmd_start,
    input  wire        cmd_we,
    input  wire        cmd_remote,      // 1 = perform this on the OTHER board
    input  wire [13:0] cmd_addr,
    input  wire [7:0]  cmd_wdata,
    output reg  [7:0]  cmd_rdata,
    output reg         cmd_done,
    output reg         cmd_error,       // remote transaction timed out

    // Shared-bus interface (identical to master_node)
    output wire        bus_req,
    input  wire        bus_gnt,
    input  wire        bus_split,
    output wire [13:0] m_addr,
    output wire [7:0]  m_wdata,
    output wire        m_we,
    output wire        m_valid,
    input  wire [7:0]  bus_rdata,
    input  wire        bus_ready,

    // UART pins to the other board
    input  wire        uart_rx_serial,
    output wire        uart_tx_serial,

    // Status
    output wire        remote_busy,     // a remote transaction is outstanding
    output wire        srv_busy         // serving the other board right now
);

    localparam [7:0] REQ_TAG  = 8'hA5;
    localparam [7:0] RESP_TAG = 8'h5A;

    // ------------------------------------------------------------------
    // UART primitives
    // ------------------------------------------------------------------
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) rx_inst (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(uart_rx_serial),
        .rx_data(rx_data), .rx_valid(rx_valid)
    );

    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) tx_inst (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_data),
        .tx_serial(uart_tx_serial), .tx_busy(tx_busy)
    );

    // ------------------------------------------------------------------
    // Byte-stream sender: shifts 1-4 bytes out, LSB byte first
    // ------------------------------------------------------------------
    localparam TXS_IDLE = 2'd0;
    localparam TXS_SEND = 2'd1;
    localparam TXS_WAIT = 2'd2;

    reg [1:0]  txs;
    reg [31:0] txsr;
    reg [2:0]  txcnt;
    reg        tx_go;
    reg [31:0] tx_payload;
    reg [2:0]  tx_len;
    wire       tx_seq_idle = (txs == TXS_IDLE) && !tx_go;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txs <= TXS_IDLE; txsr <= 32'h0; txcnt <= 3'd0;
            tx_start <= 1'b0; tx_data <= 8'h00;
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
                    txsr     <= {8'h00, txsr[31:8]};
                    txcnt    <= txcnt - 3'd1;
                    txs      <= TXS_WAIT;
                end
                TXS_WAIT: begin
                    tx_start <= 1'b0;
                    // uart_tx raises tx_busy on the edge it accepts tx_start,
                    // so both low means the byte has gone.
                    if (!tx_start && !tx_busy)
                        txs <= (txcnt == 3'd0) ? TXS_IDLE : TXS_SEND;
                end
                default: txs <= TXS_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Receive parser: splits the stream into requests and responses
    // ------------------------------------------------------------------
    localparam R_TAG  = 2'd0;
    localparam R_REQ  = 2'd1;
    localparam R_RESP = 2'd2;

    reg [1:0]  rxs;
    reg [1:0]  rxn;
    reg [23:0] rx_cmd;
    reg [7:0]  resp_data;
    reg        req_valid;
    reg        resp_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxs <= R_TAG; rxn <= 2'd0; rx_cmd <= 24'h0;
            resp_data <= 8'h00; req_valid <= 1'b0; resp_valid <= 1'b0;
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

    // ------------------------------------------------------------------
    // master_node core -- every LOCAL bus access goes through this, whether
    // it came from our own command port or from the other board.
    // ------------------------------------------------------------------
    reg         srv_active;
    reg         srv_start;
    reg  [13:0] srv_addr;
    reg  [7:0]  srv_wdata;
    reg         srv_we;

    wire        core_start = srv_active ? srv_start
                                        : (cmd_start && !cmd_remote);
    wire        core_we    = srv_active ? srv_we    : cmd_we;
    wire [13:0] core_addr  = srv_active ? srv_addr  : cmd_addr;
    wire [7:0]  core_wdata = srv_active ? srv_wdata : cmd_wdata;

    wire [7:0]  core_rdata;
    wire        core_done;

    master_node core (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(core_start), .cmd_we(core_we),
        .cmd_addr(core_addr),   .cmd_wdata(core_wdata),
        .cmd_rdata(core_rdata), .cmd_done(core_done),
        .bus_req(bus_req), .bus_gnt(bus_gnt), .bus_split(bus_split),
        .m_addr(m_addr), .m_wdata(m_wdata), .m_we(m_we), .m_valid(m_valid),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready)
    );

    // ------------------------------------------------------------------
    // Client: our own remote transactions
    // ------------------------------------------------------------------
    localparam C_IDLE = 2'd0;
    localparam C_SEND = 2'd1;
    localparam C_WAIT = 2'd2;

    reg [1:0]  cs;
    reg [23:0] out_cmd;
    reg [23:0] to_cnt;

    assign remote_busy = (cs != C_IDLE);
    assign srv_busy    = srv_active;

    // ------------------------------------------------------------------
    // Server: the other board's transactions, run on our bus
    // ------------------------------------------------------------------
    localparam S_IDLE = 2'd0;
    localparam S_EXEC = 2'd1;
    localparam S_RESP = 2'd2;

    reg [1:0] ss;
    reg [7:0] srv_result;
    reg       req_hold;
    reg [23:0] req_cmd_hold;

    // Single shared transmitter: responses take priority so two boards that
    // issue remote commands at the same instant still service each other
    // instead of both sitting in C_WAIT.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs         <= C_IDLE;
            ss         <= S_IDLE;
            srv_active <= 1'b0;
            srv_start  <= 1'b0;
            srv_addr   <= 14'h0;
            srv_wdata  <= 8'h00;
            srv_we     <= 1'b0;
            srv_result <= 8'h00;
            req_hold   <= 1'b0;
            req_cmd_hold <= 24'h0;
            out_cmd    <= 24'h0;
            to_cnt     <= 24'd0;
            tx_go      <= 1'b0;
            tx_payload <= 32'h0;
            tx_len     <= 3'd0;
            cmd_rdata  <= 8'h00;
            cmd_done   <= 1'b0;
            cmd_error  <= 1'b0;
        end else begin
            cmd_done  <= 1'b0;
            tx_go     <= 1'b0;
            srv_start <= 1'b0;

            // Latch an incoming request; only one may be outstanding.
            if (req_valid) begin
                req_hold     <= 1'b1;
                req_cmd_hold <= rx_cmd;
            end

            // ---- local (non-remote) transactions retire straight through ----
            if (!srv_active && core_done) begin
                cmd_rdata <= core_rdata;
                cmd_done  <= 1'b1;
                cmd_error <= 1'b0;
            end

            // ---------------- server ----------------
            case (ss)
                S_IDLE: begin
                    // Do not steal the core while a local transaction is live.
                    if (req_hold && cs != C_SEND && !core_start) begin
                        req_hold   <= 1'b0;
                        srv_active <= 1'b1;
                        srv_we     <= req_cmd_hold[1];
                        srv_addr   <= req_cmd_hold[15:2];
                        srv_wdata  <= req_cmd_hold[23:16];
                        srv_start  <= 1'b1;
                        ss         <= S_EXEC;
                    end
                end
                S_EXEC: begin
                    if (core_done) begin
                        srv_result <= core_we ? 8'h00 : core_rdata;
                        ss         <= S_RESP;
                    end
                end
                S_RESP: begin
                    if (tx_seq_idle) begin
                        tx_payload <= {16'h0000, srv_result, RESP_TAG};
                        tx_len     <= 3'd2;
                        tx_go      <= 1'b1;
                        srv_active <= 1'b0;
                        ss         <= S_IDLE;
                    end
                end
                default: ss <= S_IDLE;
            endcase

            // ---------------- client ----------------
            case (cs)
                C_IDLE: begin
                    if (cmd_start && cmd_remote) begin
                        out_cmd   <= {cmd_wdata, cmd_addr, cmd_we, 1'b0};
                        cmd_error <= 1'b0;
                        cs        <= C_SEND;
                    end
                end
                C_SEND: begin
                    if (tx_seq_idle && ss != S_RESP) begin
                        tx_payload <= {out_cmd, REQ_TAG};
                        tx_len     <= 3'd4;
                        tx_go      <= 1'b1;
                        to_cnt     <= 24'd0;
                        cs         <= C_WAIT;
                    end
                end
                C_WAIT: begin
                    if (resp_valid) begin
                        cmd_rdata <= resp_data;
                        cmd_done  <= 1'b1;
                        cmd_error <= 1'b0;
                        cs        <= C_IDLE;
                    end else if (to_cnt >= RESP_TIMEOUT) begin
                        cmd_rdata <= 8'hFF;
                        cmd_done  <= 1'b1;
                        cmd_error <= 1'b1;   // link dead / no answer
                        cs        <= C_IDLE;
                    end else
                        to_cnt <= to_cnt + 24'd1;
                end
                default: cs <= C_IDLE;
            endcase
        end
    end

endmodule
