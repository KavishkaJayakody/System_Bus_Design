//==========================================================================
// master.v
//
// Bus master transaction driver for the SERIAL bus.  Takes one read or write
// command at a time on a parallel command interface, shifts it out onto the
// two-wire bus, and runs it to completion including replaying it after a
// SPLIT response.
//
// The serialisation is entirely internal: whoever issues a command still
// hands over a whole address and a whole data word and gets a whole word
// back.  Only the wires between here and the slaves are one bit wide.
//
// FSM
//   IDLE     waiting for a command.  Latches addr/we/wdata when cmd_valid.
//   REQ      bus_req asserted, waiting for the grant.  Holds `load' on both
//            serialisers the whole time, so they are primed the instant the
//            grant lands.
//   ASHIFT   the address phase: ADDR_W clocks with bus_valid high, driving
//            the address MSB-first on m_astream and - on a write - the data
//            right-aligned on m_dstream.  This is the ONLY frame timer on
//            the bus; every receiver just keeps the last W bits it saw.
//   WAIT     grant still held, frame over, shifting m_dstream IN in case
//            this is a read, waiting for bus_ready.
//              resp = OKAY or ERROR -> DONE
//              resp = SPLIT         -> SPLIT_WAIT
//   SPLIT_W  the deferred state.  bus_req stays HIGH; the arbiter has masked
//            this master.  `load' is held again here, so when the grant
//            comes back the serialisers re-present the SAME address and data
//            and the whole frame is re-transmitted - a genuine re-issue, not
//            a resumption.
//   DONE     one-cycle done pulse, then IDLE.
//
// Read data is "the last DATA_W bits seen on m_dstream before bus_ready".
// That contract costs no extra wire: the deserialiser free-runs through
// WAIT, and whatever the slave shifted out most recently is what is sitting
// in it when the completion strobe arrives.
//
// ERROR is reported, not retried - an unmapped address will never become
// mapped, so retrying would hang the bus.
//
//--------------------------------------------------------------------------
// Port            Dir  Width   Meaning
//--------------------------------------------------------------------------
// clk             in   1       Bus clock.
// rst_n           in   1       Asynchronous active-low reset.
// cmd_valid       in   1       A command is offered.  Hold until cmd_accept.
// cmd_we          in   1       1 = write, 0 = read.
// cmd_addr        in   ADDR_W  Full bus word address, parallel.
// cmd_wdata       in   DATA_W  Write data, parallel (ignored on a read).
// cmd_accept      out  1       One-cycle pulse, the command was latched.
// done            out  1       One-cycle pulse, the transaction finished.
// rdata           out  DATA_W  Data returned by the last READ, reassembled.
//                              Not updated by writes.
// resp            out  RESP_W  Final response: OKAY or ERROR.  Never SPLIT -
//                              a split is absorbed internally by the replay.
// err             out  1       resp == ERROR.
// split_count     out  8       Saturating count of SPLIT responses seen.
// busy            out  1       FSM is not in IDLE.
// state           out  3       Current FSM state, for waveforms/SignalTap.
// bus_req         out  1       Bus request to the arbiter.
// bus_gnt         in   1       This master's bit of the arbiter grant.
// m_valid         out  1       Address-phase marker.  HIGH FOR ADDR_W CLOCKS,
//                              not one - this is the frame, and every
//                              receiver on the bus shifts while it is high.
// m_we            out  1       Write enable, held stable for the whole
//                              transaction (the data wire's direction mux
//                              depends on it).
// m_astream       out  1       Serial address out, MSB first.
// m_dstream       out  1       Serial write data out, right-aligned in the
//                              frame.  Driven to 0 on a read so the wire is
//                              free for the slave.
// bus_ready       in   1       Completion strobe from the return path.
// bus_resp        in   RESP_W  Response accompanying it.
// bus_dstream     in   1       The shared serial data wire, sampled for read
//                              data.
//==========================================================================
`include "bus_defs.vh"

module master #(
    parameter ADDR_W = `BUS_ADDR_W,
    parameter DATA_W = `BUS_DATA_W,     // must be <= ADDR_W
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire                clk,
    input  wire                rst_n,

    // ---- command interface (parallel) -----------------------------------
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

    // ---- serial bus -----------------------------------------------------
    output reg                 bus_req,
    input  wire                bus_gnt,
    output reg                 m_valid,
    output wire                m_we,
    output wire                m_astream,
    output wire                m_dstream,
    input  wire                bus_ready,
    input  wire [RESP_W-1:0]   bus_resp,
    input  wire                bus_dstream
);

    localparam ST_IDLE    = 3'd0;
    localparam ST_REQ     = 3'd1;
    localparam ST_ASHIFT  = 3'd2;
    localparam ST_WAIT    = 3'd3;
    localparam ST_SPLIT_W = 3'd4;
    localparam ST_DONE    = 3'd5;

    // Bits needed to count one address frame.  A constant function keeps
    // this tied to ADDR_W instead of being a hand-set parameter that can
    // silently go out of step with it.
    function integer clogb2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clogb2 = 1; v > 1; clogb2 = clogb2 + 1)
                v = v >> 1;
        end
    endfunction
    localparam CNT_W = clogb2(ADDR_W);

    reg [2:0]         cs, ns;
    reg               r_we;
    reg [ADDR_W-1:0]  r_addr;
    reg [DATA_W-1:0]  r_wdata;
    reg [CNT_W-1:0]   bitcnt;
    // How long the wait state lasted.  A responder that answered before it
    // could have sent DATA_W bits did not send any - see the rdata capture.
    reg [CNT_W-1:0]   wcnt;

    reg               ser_load, ser_shift, rd_shift;

    assign state = cs;
    assign busy  = (cs != ST_IDLE);
    assign m_we  = r_we;
    assign err   = (resp == `RESP_ERROR);

    wire split_now = bus_ready && (bus_resp == `RESP_SPLIT);

    //----------------------------------------------------------------------
    // Outgoing streams.
    //
    // The data serialiser is ADDR_W wide, not DATA_W, and is loaded with the
    // data zero-extended.  That is what right-aligns the write data in the
    // frame: it emits ADDR_W-DATA_W zeros first and the data last, so a
    // DATA_W-wide receiver shifted for the whole frame ends up holding
    // exactly the data with no bit counter of its own.
    //----------------------------------------------------------------------
    wire [ADDR_W-1:0] wdata_padded = r_wdata;      // zero-extended
    wire              dstream_raw;

    shift_ser #(.W(ADDR_W)) u_addr_ser (
        .clk(clk), .rst_n(rst_n),
        .load(ser_load), .shift(ser_shift),
        .din(r_addr), .dout(m_astream)
    );

    shift_ser #(.W(ADDR_W)) u_wdata_ser (
        .clk(clk), .rst_n(rst_n),
        .load(ser_load), .shift(ser_shift),
        .din(wdata_padded), .dout(dstream_raw)
    );

    // Leave the shared data wire alone on a read - the slave owns it then.
    assign m_dstream = r_we ? dstream_raw : 1'b0;

    //----------------------------------------------------------------------
    // Incoming read data.  Free-runs through WAIT; whatever is in it when
    // bus_ready arrives is the answer.
    //----------------------------------------------------------------------
    wire [DATA_W-1:0] rdata_ser;

    shift_deser #(.W(DATA_W)) u_rdata_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(rd_shift), .din(bus_dstream), .dout(rdata_ser)
    );

    //----------------------------------------------------------------------
    // Next state and Mealy outputs.  Every output defaulted at the top, so
    // no latch can be inferred.
    //----------------------------------------------------------------------
    always @* begin
        ns         = cs;
        cmd_accept = 1'b0;
        bus_req    = 1'b0;
        m_valid    = 1'b0;
        done       = 1'b0;
        ser_load   = 1'b0;
        ser_shift  = 1'b0;
        rd_shift   = 1'b0;

        case (cs)
            ST_IDLE: begin
                if (cmd_valid) begin
                    cmd_accept = 1'b1;
                    ns         = ST_REQ;
                end
            end

            ST_REQ: begin
                bus_req  = 1'b1;
                ser_load = 1'b1;              // primed, ready for the grant
                if (bus_gnt) ns = ST_ASHIFT;
            end

            ST_ASHIFT: begin
                bus_req   = 1'b1;
                m_valid   = 1'b1;             // the frame
                ser_shift = 1'b1;
                if (bitcnt == (ADDR_W-1)) ns = ST_WAIT;
            end

            ST_WAIT: begin
                bus_req  = 1'b1;
                rd_shift = 1'b1;              // collect whatever comes back
                if (bus_ready)
                    ns = split_now ? ST_SPLIT_W : ST_DONE;
            end

            ST_SPLIT_W: begin
                bus_req  = 1'b1;              // held: the arbiter mask, not
                                              // the request, is what defers us
                ser_load = 1'b1;              // reload for the replay
                if (bus_gnt) ns = ST_ASHIFT;
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
            bitcnt      <= {CNT_W{1'b0}};
            wcnt        <= {CNT_W{1'b0}};
        end else begin
            cs <= ns;

            // One bit per clock of the address frame, and parked at 0
            // everywhere else so every frame starts from the same place -
            // including a replay.
            if (cs == ST_ASHIFT) bitcnt <= bitcnt + 1'b1;
            else                 bitcnt <= {CNT_W{1'b0}};

            // Saturating, so a long wait cannot wrap back under DATA_W.
            if (cs != ST_WAIT)             wcnt <= {CNT_W{1'b0}};
            else if (wcnt != {CNT_W{1'b1}}) wcnt <= wcnt + 1'b1;

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
                    // displayed value with whatever was last on the wire.
                    //
                    // An ERROR from the default responder is answered in ONE
                    // cycle, long before any data phase, so the deserialiser
                    // still holds the PREVIOUS read's bits shifted along by
                    // the clocks of WAIT.  Returning that would leak the last
                    // transfer's data and put convincing rubbish on led[7:0].
                    //
                    // But not every ERROR is like that.  The remote bridge
                    // shifts a real byte out and THEN reports ERROR when the
                    // far board never answered - 0xFF, which the link spec
                    // requires the host to see.  So the test is not "was it
                    // an ERROR" but "could a byte have arrived at all": a
                    // responder that answered in fewer than DATA_W clocks
                    // cannot have sent one.
                    if (!r_we)
                        rdata <= (bus_resp == `RESP_ERROR && wcnt < DATA_W)
                                     ? {DATA_W{1'b0}} : rdata_ser;
                end
            end
        end
    end

endmodule
