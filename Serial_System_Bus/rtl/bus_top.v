//==========================================================================
// bus_top.v
//
// The system: masters, the bus, slaves.  Three kinds of module, wired
// together by serial wires and nothing else.
//
//   master  x2  ---+
//                  |   bus_astream   ONE wire, the address
//   system_bus  ---+   bus_dstream   ONE wire, the data
//                  |
//   slave   x3  ---+
//
// This module is only integration - it contains no logic of its own beyond
// wiring and one OR gate.  Everything a bus does lives in `system_bus';
// everything a peripheral does lives in `master' and `slave'.  Board
// concerns (clocking, switches, displays) live one level up in `de2_top',
// and the testbenches drive the command ports here directly.
//
// Splitting it this way is what makes the bus testable on its own:
// `tb_system_bus' drives the master-side and slave-side interfaces below
// with no real master or memory attached at all.
//
// The master command interfaces are flattened vectors (Verilog-2001 has no
// arrays of ports): master i occupies bits [i*W +: W].  Those are PARALLEL -
// a whole address and a whole data word - because they face the sequencer,
// not the bus.  The serialisation boundary is inside `master'.
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
// bus_astream       out  1                  THE address wire (status).
// bus_dstream       out  1                  THE data wire (status).
// bus_addr          out  ADDR_W             Reassembled address (status).
// addr_done         out  1                  Decode strobe (status).
// bus_ready         out  1                  Completion strobe (status).
// bus_resp          out  RESP_W             Response (status).
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

    //======================================================================
    // Serial wires between the masters and the bus.
    // One bit per master per stream - these are the CANDIDATES; the bus
    // picks one with the grant.
    //======================================================================
    wire [N_MASTERS-1:0]  m_req;
    wire [N_MASTERS-1:0]  m_valid;
    wire [N_MASTERS-1:0]  m_we;
    wire [N_MASTERS-1:0]  m_astream;
    wire [N_MASTERS-1:0]  m_dstream;

    //======================================================================
    // Serial wires between the bus and the slaves.
    //======================================================================
    wire                            bus_we;
    wire [N_SLAVES-1:0]             s_sel;
    wire [N_SLAVES-1:0]             s_ready;
    wire [N_SLAVES*RESP_W-1:0]      s_resp_flat;
    wire [N_SLAVES-1:0]             s_dstream;

    //======================================================================
    // 1. MASTERS
    //======================================================================
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
            // parallel, facing the command source
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
            // serial, facing the bus
            .bus_req     (m_req[gi]),
            .bus_gnt     (gnt[gi]),
            .m_valid     (m_valid[gi]),
            .m_we        (m_we[gi]),
            .m_astream   (m_astream[gi]),        // ADDRESS, one wire
            .m_dstream   (m_dstream[gi]),        // WRITE DATA, one wire
            .bus_ready   (bus_ready),
            .bus_resp    (bus_resp),
            .bus_dstream (bus_dstream)           // READ DATA, the shared wire
        );
    end
    endgenerate

    //======================================================================
    // 2. THE BUS
    //======================================================================
    wire [N_MASTERS-1:0] s0_split_complete;

    // Wake-up pulses from every split-capable slave, OR-ed per master.  Only
    // slave 0 can raise one today; a second split-capable slave joins here.
    wire [N_MASTERS-1:0] s_split_complete = s0_split_complete;

    system_bus #(
        .N_MASTERS (N_MASTERS),
        .ID_W      (ID_W),
        .N_SLAVES  (N_SLAVES),
        .ADDR_W    (ADDR_W),
        .RESP_W    (RESP_W)
    ) u_system_bus (
        .clk              (clk),
        .rst_n            (rst_n),

        // master side
        .m_req            (m_req),
        .m_gnt            (gnt),
        .m_valid          (m_valid),
        .m_we             (m_we),
        .m_astream        (m_astream),
        .m_dstream        (m_dstream),
        .bus_ready        (bus_ready),
        .bus_resp         (bus_resp),

        // slave side
        .bus_valid        (bus_valid),
        .bus_we           (bus_we),
        .bus_master_id    (master_id),
        .s_sel            (s_sel),
        .s_ready          (s_ready),
        .s_resp_flat      (s_resp_flat),
        .s_dstream        (s_dstream),
        .s_split_complete (s_split_complete),

        // the two shared wires
        .bus_astream      (bus_astream),
        .bus_dstream      (bus_dstream),

        // status
        .gnt_valid        (gnt_valid),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .bus_addr         (bus_addr),
        .addr_done        (addr_done)
    );

    //======================================================================
    // 3. SLAVES
    //
    // Instantiated one by one rather than in a generate loop, because they
    // differ in size and in whether they can split - and those differences
    // are worth reading at a glance.
    //
    // All three tap the SAME bus_astream and bus_dstream.
    //======================================================================

    // Slave 0 - 4 KB at 0x0000, split capable
    slave #(
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
        .sel            (s_sel[`SEL_S0]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (s0_split_en),
        .dstream_out    (s_dstream[`SEL_S0]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (s0_split_complete),
        .busy           (s0_busy)
    );

    // Slave 1 - 4 KB at 0x1000
    slave #(
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
        .sel            (s_sel[`SEL_S1]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S1]),
        .ready          (s_ready[`SEL_S1]),
        .resp           (s_resp_flat[`SEL_S1*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    // Slave 2 - 2 KB at 0x2000
    slave #(
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
        .sel            (s_sel[`SEL_S2]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S2]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

endmodule
