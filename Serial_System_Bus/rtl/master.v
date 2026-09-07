//==========================================================================
// master.v
//
// Bus master transaction driver.  Takes one read or write command at a time
// from a command interface and runs it to completion on the shared bus,
// including replaying it after a SPLIT response.
//
// FSM
//   IDLE     waiting for a command.  Latches addr/we/wdata when cmd_valid.
//   REQ      bus_req asserted, waiting for the arbiter's grant.
//   XFER     grant held; drives m_valid for exactly ONE cycle together with
//            the latched address and data.  One cycle, because the slaves
//            treat every cycle of sel as a new access - holding m_valid
//            would start a second transfer.
//   WAIT     grant still held, m_valid low, waiting for bus_ready.
//              resp = OKAY or ERROR -> DONE
//              resp = SPLIT         -> SPLIT_WAIT
//   SPLIT_W  the deferred state.  bus_req stays HIGH; the arbiter has masked
//            this master, so the request is simply not eligible until the
//            slave pulses split_complete.  When the grant comes back the
//            same latched transfer is re-issued from XFER - the replay.
//   DONE     one-cycle done pulse, then IDLE.
//
// Holding bus_req through SPLIT_W (rather than dropping it and re-requesting)
// is what makes the split invisible to the command interface: the caller
// sees one command and one done, however many times the transfer was
// actually driven onto the bus.
//
// ERROR is reported, not retried - an unmapped address will never become
// mapped, so retrying would hang the bus forever.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock.
// rst_n           in   1          Asynchronous active-low reset.
// cmd_valid       in   1          A command is offered.  Hold high until
//                                 cmd_accept.
// cmd_we          in   1          1 = write, 0 = read.
// cmd_addr        in   ADDR_W     Full bus word address.
// cmd_wdata       in   DATA_W     Write data (ignored on a read).
// cmd_accept      out  1          One-cycle pulse, the command was latched.
// done            out  1          One-cycle pulse, the transaction finished.
//                                 rdata/resp/err are valid on this cycle and
//                                 hold until the next transaction finishes.
// rdata           out  DATA_W     Data returned by the last READ.  Not
//                                 updated by writes, so a write cannot
//                                 clobber the value on the display.
// resp            out  RESP_W     Final response of the last transaction:
//                                 OKAY or ERROR.  Never SPLIT - a split is
//                                 absorbed internally by the replay.
// err             out  1          resp == ERROR, i.e. the last transaction
//                                 hit an unmapped address.
// split_count     out  8          Saturating count of SPLIT responses seen.
//                                 Status/debug and testbench checking.
// busy            out  1          FSM is not in IDLE.
// state           out  3          Current FSM state, for waveforms/SignalTap.
// bus_req         out  1          Bus request to the arbiter.
// bus_gnt         in   1          This master's bit of the arbiter grant.
// m_valid         out  1          Transfer strobe onto the forward mux.
// m_we            out  1          Write enable onto the forward mux.
// m_addr          out  ADDR_W     Address onto the forward mux.
// m_wdata         out  DATA_W     Write data onto the forward mux.
// bus_ready       in   1          Completion strobe from the return mux.
// bus_resp        in   RESP_W     Response from the return mux.
// bus_rdata       in   DATA_W     Read data from the return mux.
//==========================================================================
`include "bus_defs.vh"

module master #(
    parameter ADDR_W = `BUS_ADDR_W,
    parameter DATA_W = `BUS_DATA_W,
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire                clk,
    input  wire                rst_n,

    // ---- command interface --------------------------------------------
    input  wire                cmd_valid,
    input  wire                cmd_we,
    input  wire [ADDR_W-1:0]   cmd_addr,
    input  wire [DATA_W-1:0]   cmd_wdata,
    output reg                 cmd_accept,
    output reg                 done,
    output reg  [DATA_W-1:0]   rdata,
    output reg  [RESP_W-1:0]   resp,
    output wire                err,
    output reg  [7:0]          split_count,
    output wire                busy,
    output wire [2:0]          state,

    // ---- bus ------------------------------------------------------------
    output reg                 bus_req,
    input  wire                bus_gnt,
    output reg                 m_valid,
    output wire                m_we,
    output wire [ADDR_W-1:0]   m_addr,
    output wire [DATA_W-1:0]   m_wdata,
    input  wire                bus_ready,
    input  wire [RESP_W-1:0]   bus_resp,
    input  wire [DATA_W-1:0]   bus_rdata
);

    localparam ST_IDLE    = 3'd0;
    localparam ST_REQ     = 3'd1;
    localparam ST_XFER    = 3'd2;
    localparam ST_WAIT    = 3'd3;
    localparam ST_SPLIT_W = 3'd4;
    localparam ST_DONE    = 3'd5;

    reg [2:0]         cs, ns;
    reg               r_we;
    reg [ADDR_W-1:0]  r_addr;
    reg [DATA_W-1:0]  r_wdata;

    assign state   = cs;
    assign busy    = (cs != ST_IDLE);
    assign m_we    = r_we;
    assign m_addr  = r_addr;
    assign m_wdata = r_wdata;
    assign err     = (resp == `RESP_ERROR);

    wire split_now = bus_ready && (bus_resp == `RESP_SPLIT);

    //----------------------------------------------------------------------
    // Next state and Mealy outputs.  Every output is defaulted at the top of
    // the block, so no latch can be inferred.
    //----------------------------------------------------------------------
    always @* begin
        ns         = cs;
        cmd_accept = 1'b0;
        bus_req    = 1'b0;
        m_valid    = 1'b0;
        done       = 1'b0;

        case (cs)
            ST_IDLE: begin
                if (cmd_valid) begin
                    cmd_accept = 1'b1;
                    ns         = ST_REQ;
                end
            end

            ST_REQ: begin
                bus_req = 1'b1;
                if (bus_gnt) ns = ST_XFER;
            end

            ST_XFER: begin
                bus_req = 1'b1;
                m_valid = 1'b1;          // exactly one cycle
                ns      = ST_WAIT;
            end

            ST_WAIT: begin
                bus_req = 1'b1;
                if (bus_ready)
                    ns = split_now ? ST_SPLIT_W : ST_DONE;
            end

            ST_SPLIT_W: begin
                bus_req = 1'b1;          // held: the arbiter mask, not the
                                         // request, is what defers us
                if (bus_gnt) ns = ST_XFER;
            end

            ST_DONE: begin
                done = 1'b1;
                ns   = ST_IDLE;
            end

            default: ns = ST_IDLE;
        endcase
    end

    //----------------------------------------------------------------------
    // State and data registers.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs          <= ST_IDLE;
            r_we        <= 1'b0;
            r_addr      <= {ADDR_W{1'b0}};
            r_wdata     <= {DATA_W{1'b0}};
            rdata       <= {DATA_W{1'b0}};
            resp        <= `RESP_OKAY;
            split_count <= 8'd0;
        end else begin
            cs <= ns;

            // Latch the command as it is accepted.
            if (cs == ST_IDLE && cmd_valid) begin
                r_we    <= cmd_we;
                r_addr  <= cmd_addr;
                r_wdata <= cmd_wdata;
            end

            if (cs == ST_WAIT && bus_ready) begin
                if (split_now) begin
                    // Saturating, so a long hardware run cannot wrap it back
                    // to zero and hide that splits happened.
                    if (split_count != 8'hFF)
                        split_count <= split_count + 8'd1;
                end else begin
                    resp <= bus_resp;
                    // Reads only: a write would otherwise overwrite the
                    // displayed value with whatever the slave echoed.
                    if (!r_we)
                        rdata <= bus_rdata;
                end
            end
        end
    end

endmodule
