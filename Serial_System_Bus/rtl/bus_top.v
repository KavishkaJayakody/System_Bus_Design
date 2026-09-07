//==========================================================================
// bus_top.v
//
// Integration of the complete SERIAL shared bus: N_MASTERS masters, the
// arbiter, the address decoder, the two-wire data path, three memory slaves
// and the default slave.  Board-independent - de2_top wraps this with
// clocking, switches and displays, and the testbenches drive these same
// command ports directly.
//
//   master[i] --+                              +-- slave_mem 0 (4K, split)
//               |  arbiter --> gnt             |
//               +->bus_mux --> bus_astream ----+-- slave_mem 1 (4K)
//                             bus_dstream -----+-- slave_mem 2 (2K)
//                                  |           |
//                                  |           +-- default_slave
//                                  v
//                          shift_deser(16) --> addr_decoder --> sel
//
// THE ADDRESS ARRIVES ONE BIT AT A TIME, so the decoder cannot decide
// anything until the frame is over.  The central deserialiser collects the
// whole address while bus_valid is high; `addr_done' - the falling edge of
// bus_valid - then enables the (still purely combinational) decoder for
// exactly one cycle, producing the one-cycle `sel' pulse the slaves act on.
//
// Deriving addr_done from bus_valid rather than adding an "end of frame"
// wire keeps the shared bus at eight wires and means the frame length lives
// in exactly one place: the master's counter.
//
// Everything below runs on one clock with one asynchronous active-low reset.
// No clock gating, no second domain.
//
// The master command interfaces are flattened vectors (Verilog-2001 has no
// arrays of ports): master i occupies bits [i*W +: W].
//
//--------------------------------------------------------------------------
// Port              Dir  Width              Meaning
//--------------------------------------------------------------------------
// clk               in   1                  Bus clock.
// rst_n             in   1                  Asynchronous active-low reset.
// cmd_valid         in   N_MASTERS          Per-master: a command is offered.
// cmd_we            in   N_MASTERS          Per-master: 1 = write.
// cmd_addr_flat     in   N_MASTERS*ADDR_W   Per-master command address.
// cmd_wdata_flat    in   N_MASTERS*DATA_W   Per-master command write data.
// cmd_accept        out  N_MASTERS          Per-master: command latched.
// done              out  N_MASTERS          Per-master: transaction finished.
// rdata_flat        out  N_MASTERS*DATA_W   Per-master last read data.
// resp_flat         out  N_MASTERS*RESP_W   Per-master last response.
// err               out  N_MASTERS          Per-master last response = ERROR.
// split_count_flat  out  N_MASTERS*8        Per-master count of splits seen.
// mst_busy          out  N_MASTERS          Per-master FSM not idle.
// s0_split_en       in   1                  1 = slave 0 behaves as busy.
// gnt               out  N_MASTERS          One-hot grant (status).
// gnt_valid         out  1                  A master owns the bus (status).
// master_id         out  ID_W               Granted master index (status).
// split_mask        out  N_MASTERS          Arbiter split mask (status).
// sel_q             out  N_SLAVES+1         Latched responder select (status).
// bus_valid         out  1                  Address frame active (status).
// bus_astream       out  1                  The shared address wire (status).
// bus_dstream       out  1                  The shared data wire (status).
// bus_addr          out  ADDR_W             The reassembled address, valid
//                                           from addr_done onwards (status,
//                                           and what the board displays).
// addr_done         out  1                  One-cycle end-of-frame / decode
//                                           strobe (status).
// bus_ready         out  1                  Return completion strobe (status).
// bus_resp          out  RESP_W             Return response (status).
// s0_busy           out  1                  Slave 0 has a split in flight.
//==========================================================================
`include "bus_defs.vh"

module bus_top #(
    parameter N_MASTERS     = `BUS_N_MASTERS,
    parameter ID_W          = 1,               // ceil(log2(N_MASTERS))
    parameter N_SLAVES      = `BUS_N_SLAVES,
    parameter ADDR_W        = `BUS_ADDR_W,
    parameter DATA_W        = `BUS_DATA_W,
    parameter RESP_W        = `BUS_RESP_W,
    parameter SPLIT_LATENCY = 4
) (
    input  wire                            clk,
    input  wire                            rst_n,

    // ---- master command interfaces (parallel) ---------------------------
    input  wire [N_MASTERS-1:0]            cmd_valid,
    input  wire [N_MASTERS-1:0]            cmd_we,
    input  wire [N_MASTERS*ADDR_W-1:0]     cmd_addr_flat,
    input  wire [N_MASTERS*DATA_W-1:0]     cmd_wdata_flat,
    output wire [N_MASTERS-1:0]            cmd_accept,
    output wire [N_MASTERS-1:0]            done,
    output wire [N_MASTERS*DATA_W-1:0]     rdata_flat,
    output wire [N_MASTERS*RESP_W-1:0]     resp_flat,
    output wire [N_MASTERS-1:0]            err,
    output wire [N_MASTERS*8-1:0]          split_count_flat,
    output wire [N_MASTERS-1:0]            mst_busy,

    // ---- slave 0 split control ------------------------------------------
    input  wire                            s0_split_en,

    // ---- observability ---------------------------------------------------
    output wire [N_MASTERS-1:0]            gnt,
    output wire                            gnt_valid,
    output wire [ID_W-1:0]                 master_id,
    output wire [N_MASTERS-1:0]            split_mask,
    output wire [N_SLAVES:0]               sel_q,
    output wire                            bus_valid,
    output wire                            bus_astream,
    output wire                            bus_dstream,
    output wire [ADDR_W-1:0]               bus_addr,
    output wire                            addr_done,
    output wire                            bus_ready,
    output wire [RESP_W-1:0]               bus_resp,
    output wire                            s0_busy
);

    //----------------------------------------------------------------------
    // Master <-> forward-mux wiring.  One bit per master per stream.
    //----------------------------------------------------------------------
    wire [N_MASTERS-1:0]  m_req;
    wire [N_MASTERS-1:0]  m_valid;
    wire [N_MASTERS-1:0]  m_we;
    wire [N_MASTERS-1:0]  m_astream;
    wire [N_MASTERS-1:0]  m_dstream;

    wire                  bus_we;

    //----------------------------------------------------------------------
    // Masters
    //----------------------------------------------------------------------
    genvar gi;
    generate
    for (gi = 0; gi < N_MASTERS; gi = gi + 1) begin : g_master
        master #(
            .ADDR_W (ADDR_W),
            .DATA_W (DATA_W),
            .RESP_W (RESP_W)
        ) u_master (
            .clk         (clk),
            .rst_n       (rst_n),
            .cmd_valid   (cmd_valid[gi]),
            .cmd_we      (cmd_we[gi]),
            .cmd_addr    (cmd_addr_flat  [gi*ADDR_W +: ADDR_W]),
            .cmd_wdata   (cmd_wdata_flat [gi*DATA_W +: DATA_W]),
            .cmd_accept  (cmd_accept[gi]),
            .done        (done[gi]),
            .rdata       (rdata_flat     [gi*DATA_W +: DATA_W]),
            .resp        (resp_flat      [gi*RESP_W +: RESP_W]),
            .err         (err[gi]),
            .split_count (split_count_flat[gi*8 +: 8]),
            .busy        (mst_busy[gi]),
            .state       (),                     // waveform only
            .bus_req     (m_req[gi]),
            .bus_gnt     (gnt[gi]),
            .m_valid     (m_valid[gi]),
            .m_we        (m_we[gi]),
            .m_astream   (m_astream[gi]),
            .m_dstream   (m_dstream[gi]),
            .bus_ready   (bus_ready),
            .bus_resp    (bus_resp),
            .bus_dstream (bus_dstream)
        );
    end
    endgenerate

    //----------------------------------------------------------------------
    // Arbiter.  Unchanged from the parallel bus - it only ever looked at
    // req, ready and resp, none of which were serialised.
    //----------------------------------------------------------------------
    wire [N_MASTERS-1:0] s0_split_complete;
    wire [N_MASTERS-1:0] split_complete = s0_split_complete;

    arbiter #(
        .N_MASTERS (N_MASTERS),
        .ID_W      (ID_W),
        .RESP_W    (RESP_W)
    ) u_arbiter (
        .clk            (clk),
        .rst_n          (rst_n),
        .req            (m_req),
        .bus_ready      (bus_ready),
        .bus_resp       (bus_resp),
        .split_complete (split_complete),
        .gnt            (gnt),
        .gnt_valid      (gnt_valid),
        .master_id      (master_id),
        .split_mask     (split_mask),
        .locked         ()
    );

    //----------------------------------------------------------------------
    // The two-wire data path
    //----------------------------------------------------------------------
    wire [N_SLAVES-1:0]              slv_sel;
    wire                             def_sel;
    wire [N_SLAVES:0]                sel = {def_sel, slv_sel};

    wire [N_SLAVES:0]                s_ready;
    wire [(N_SLAVES+1)*RESP_W-1:0]   s_resp_flat;
    wire [N_SLAVES:0]                s_dstream;

    bus_mux #(
        .N_MASTERS (N_MASTERS),
        .N_SLAVES  (N_SLAVES),
        .RESP_W    (RESP_W)
    ) u_bus_mux (
        .clk         (clk),
        .rst_n       (rst_n),
        .gnt         (gnt),
        .m_valid     (m_valid),
        .m_we        (m_we),
        .m_astream   (m_astream),
        .m_dstream   (m_dstream),
        .bus_valid   (bus_valid),
        .bus_we      (bus_we),
        .bus_astream (bus_astream),
        .bus_dstream (bus_dstream),
        .sel         (sel),
        .s_ready     (s_ready),
        .s_resp_flat (s_resp_flat),
        .s_dstream   (s_dstream),
        .bus_ready   (bus_ready),
        .bus_resp    (bus_resp),
        .sel_q       (sel_q)
    );

    //----------------------------------------------------------------------
    // Central address deserialiser and the end-of-frame strobe.
    //
    // The decoder is combinational and needs the whole address at once, so
    // one deserialiser here collects the frame off the shared wire.  The
    // slaves each collect their own low bits off the same wire in parallel -
    // nothing is broadcast back out in parallel form.
    //----------------------------------------------------------------------
    shift_deser #(.W(ADDR_W)) u_addr_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(bus_valid), .din(bus_astream), .dout(bus_addr)
    );

    reg bus_valid_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bus_valid_d <= 1'b0;
        else        bus_valid_d <= bus_valid;
    end

    // Falling edge of the frame: the address is complete this cycle.
    assign addr_done = bus_valid_d & ~bus_valid;

    //----------------------------------------------------------------------
    // Address decoder.  Still purely combinational; it is simply enabled one
    // cycle per frame instead of one cycle per parallel access.
    //----------------------------------------------------------------------
    addr_decoder #(
        .ADDR_W   (ADDR_W),
        .N_SLAVES (N_SLAVES)
    ) u_decoder (
        .en      (addr_done),
        .addr    (bus_addr),
        .slv_sel (slv_sel),
        .def_sel (def_sel),
        .hit     ()
    );

    //----------------------------------------------------------------------
    // Slave 0 - 4K words at 0x0000, split capable
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S0_LADDR_W),
        .WORDS         (`S0_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (1),
        .SPLIT_LATENCY (SPLIT_LATENCY)
    ) u_slave0 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (slv_sel[`SEL_S0]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (s0_split_en),
        .dstream_out    (s_dstream[`SEL_S0]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (s0_split_complete),
        .busy           (s0_busy)
    );

    //----------------------------------------------------------------------
    // Slave 1 - 4K words at 0x1000
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S1_LADDR_W),
        .WORDS         (`S1_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave1 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (slv_sel[`SEL_S1]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S1]),
        .ready          (s_ready[`SEL_S1]),
        .resp           (s_resp_flat[`SEL_S1*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    //----------------------------------------------------------------------
    // Slave 2 - 2K words at 0x2000
    //----------------------------------------------------------------------
    slave_mem #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (N_MASTERS),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave2 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (slv_sel[`SEL_S2]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S2]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    //----------------------------------------------------------------------
    // Default slave - everything unmapped, answers ERROR so the bus never
    // stalls on a bad address.
    //----------------------------------------------------------------------
    default_slave #(
        .RESP_W (RESP_W)
    ) u_default (
        .clk         (clk),
        .rst_n       (rst_n),
        .sel         (def_sel),
        .dstream_out (s_dstream[`SEL_DEF]),
        .ready       (s_ready[`SEL_DEF]),
        .resp        (s_resp_flat[`SEL_DEF*RESP_W +: RESP_W])
    );

endmodule
