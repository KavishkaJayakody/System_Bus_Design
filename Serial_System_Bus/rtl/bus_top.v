//==========================================================================
// bus_top.v
//
// THE COMPLETE BUS SYSTEM: 2 masters + system_bus + 3 memory slaves, wired
// together and nothing else.  No board, no switches, no JTAG, no displays.
//
// This is the serial design's equivalent of `top_bus_system' in the parallel
// System_Bus_Final project, and it plays the same role: the synthesis top
// holds only the debug front-end and instantiates this for the system.
//
//   top_debug  ->  bus_issp_driver  +  bus_top
//
// WHAT THIS MODULE IS FOR
//
// The composition below used to be written out three times - in the board
// top level and in both integration testbenches - because there was no
// wrapper to hold it.  Three copies of the same wiring is three chances to
// change one and forget the others, at which point the testbenches quietly
// stop testing what the board builds.  It is written once here now, and
// top_debug, tb_integration and tb_bus_issp_driver all instantiate it.
//
// WHAT IT DOES NOT CHANGE
//
// The three-kinds-of-module split is untouched and still the rule:
// `system_bus' is the bus and contains no master and no memory; `master' and
// `slave' are peripherals and contain no bus logic.  This module contains no
// logic AT ALL - it is instantiation and wiring only, so it cannot become a
// place where bus behaviour hides.  `tb_system_bus' still tests `system_bus'
// on its own, with nothing attached at either end.
//
// THE INTERFACE
//
// Facing out, the master command ports are PARALLEL - a whole address and a
// whole data byte, packed one slice per master.  Serialisation happens
// inside `master'.  The JTAG host drives this module through those ports,
// which is the bus's front door and not a back way onto the wires.
//
// Everything from `gnt' down is OBSERVATION ONLY: the ISSP driver and the
// testbenches read it, and nothing that reads it can affect a transfer.  The
// two shared serial wires are brought out for the same reason -
// `tb_integration' watches the framing on them directly.
//
//--------------------------------------------------------------------------
// Port              Dir  Width       Meaning
//--------------------------------------------------------------------------
// clk               in   1           Bus clock, one domain for everything.
// rst_n             in   1           Asynchronous active-low reset.
//
// cmd_valid         in   NM          LEVEL, held until cmd_accept.
// cmd_we            in   NM          1 = write, 0 = read.
// cmd_addr_flat     in   NM*ADDR_W   Command addresses, packed.
// cmd_wdata_flat    in   NM*DATA_W   Command write data, packed.
// cmd_accept        out  NM          The master took the command, one clock.
// done              out  NM          Transaction finished, one clock.
// rdata_flat        out  NM*DATA_W   Last read data per master, packed.
// resp_flat         out  NM*RESP_W   Final response per master, packed.
// err               out  NM          That transaction answered ERROR.
// split_count_flat  out  NM*8        Splits absorbed per master, packed.
// mst_busy          out  NM          Master has a command in flight.
//
// split_en          in   1           Make the split-capable slave (slave 2)
//                                    answer SPLIT - the "slave is busy"
//                                    model.  See design_notes §1.
//
// gnt               out  NM          One-hot grant.
// gnt_valid         out  1           Somebody owns the bus.
// split_mask        out  NM          Lit bit = that master is split-deferred.
// sel_q             out  NS+1        Latched responder {default, s2, s1, s0}.
// split_busy        out  1           The split slave has a deferred transfer.
// master_id         out  ID_W        Tag of the granted master.
// bus_valid         out  1           Frame marker, high for ADDR_W clocks.
// bus_we            out  1           1 = write; also the data direction.
// bus_ready         out  1           Completion strobe.
// bus_resp          out  RESP_W      OKAY / ERROR / SPLIT.
// bus_addr          out  ADDR_W      DEBUG ONLY: the address reassembled off
//                                    the wire, for the JTAG probe.  Zero when
//                                    OBSERVE_ADDR=0.  No datapath reads it.
// addr_done         out  1           Decode strobe, the frame's falling edge.
// bus_astream       out  1           THE SHARED ADDRESS WIRE.
// bus_dstream       out  1           THE SHARED DATA WIRE.
//==========================================================================
`include "bus_defs.vh"

module bus_top #(
    parameter NM            = `BUS_N_MASTERS,
    parameter NS            = `BUS_N_SLAVES,
    parameter ID_W          = 1,
    parameter ADDR_W        = `BUS_ADDR_W,
    parameter DATA_W        = `BUS_DATA_W,
    parameter RESP_W        = `BUS_RESP_W,
    // Slave 0 stays "busy" for this many clocks per split.  The board build
    // uses ~0.2 s so a split is visible from the host; testbenches shrink it.
    parameter SPLIT_LATENCY = 10_000_000,
    // UART link to the other board.  50 MHz / 115200 baud = 434.
    parameter CLKS_PER_BIT  = 434,
    // Remote reply timeout, ~10 ms at 50 MHz.  A dead link reports an error
    // instead of hanging.
    parameter RESP_TIMEOUT  = 500000,
    // Keep the 16-bit address reassembly that feeds the JTAG probe.  It is
    // DEBUG ONLY - the decoder is serial and does not use it - so 0 removes
    // it and with it the last parallel address anywhere in the design.
    parameter OBSERVE_ADDR  = 1
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- master command ports (parallel) ---------------------------------
    input  wire [NM-1:0]           cmd_valid,
    input  wire [NM-1:0]           cmd_we,
    input  wire [NM*ADDR_W-1:0]    cmd_addr_flat,
    input  wire [NM*DATA_W-1:0]    cmd_wdata_flat,
    output wire [NM-1:0]           cmd_accept,
    output wire [NM-1:0]           done,
    output wire [NM*DATA_W-1:0]    rdata_flat,
    output wire [NM*RESP_W-1:0]    resp_flat,
    output wire [NM-1:0]           err,
    output wire [NM*8-1:0]         split_count_flat,
    output wire [NM-1:0]           mst_busy,

    // ---- the "slave is busy" model, on the split-capable slave -----------
    input  wire                    split_en,

    // ---- board-to-board link over UART (master 0 only) -------------------
    // There is no "remote" input: addr[15] selects the far board.
    output wire                    cmd_error,      // remote read timed out
    input  wire                    rm_rx,
    output wire                    rm_tx,
    output wire                    remote_busy,
    output wire                    srv_busy,
    // link diagnostics - observation only, see master_uart.v
    output wire [7:0]              dbg_rx_last,
    output wire [7:0]              dbg_rx_count,
    output wire [7:0]              dbg_tx_count,
    output wire [1:0]              dbg_rx_state,
    output wire                    dbg_req_seen,
    output wire                    dbg_resp_seen,
    output wire                    dbg_rx_active,
    output wire                    dbg_req_overrun,

    // ---- observation only ------------------------------------------------
    output wire [NM-1:0]           gnt,
    output wire                    gnt_valid,
    output wire [NM-1:0]           split_mask,
    output wire [NS:0]             sel_q,
    output wire                    split_busy,
    output wire [ID_W-1:0]         master_id,
    output wire                    bus_valid,
    output wire                    bus_we,
    output wire                    bus_ready,
    output wire [RESP_W-1:0]       bus_resp,
    output wire [ADDR_W-1:0]       bus_addr,
    output wire                    addr_done,
    output wire                    bus_astream,
    output wire                    bus_dstream
);

    //======================================================================
    // Serial wires between the masters and the bus.
    // One bit per master per stream - these are the CANDIDATES; the bus
    // picks one with the grant.
    //======================================================================
    wire [NM-1:0]  m_req;
    wire [NM-1:0]  m_valid;
    wire [NM-1:0]  m_we;
    wire [NM-1:0]  m_astream;
    wire [NM-1:0]  m_dstream;

    //======================================================================
    // Wires between the bus and the slaves.
    //======================================================================
    wire [NS-1:0]          s_sel;
    wire [NS-1:0]          s_ready;
    wire [NS*RESP_W-1:0]   s_resp_flat;
    wire [NS-1:0]          s_dstream;

    //======================================================================
    // 1. MASTERS
    //
    // Parallel command in, SERIAL onto the bus: cmd_addr goes out one bit
    // at a time on m_astream, cmd_wdata on m_dstream.
    //
    // Master 0 is a `master_uart' - the ordinary master core plus a UART
    // client and server for reaching the OTHER board.  Master 1 is a plain
    // `master', local only.  That asymmetry is the same one System_Bus_Final
    // has, and it is why these are instantiated one by one instead of in a
    // generate loop.
    //
    // For a LOCAL transaction the wrapper is a pass-through: master 0 costs
    // exactly what master 1 costs.
    //======================================================================

    // Master 0 - with the UART link
    master_uart #(
        .ADDR_W       (ADDR_W),
        .DATA_W       (DATA_W),
        .RESP_W       (RESP_W),
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .RESP_TIMEOUT (RESP_TIMEOUT)
    ) u_master0 (
        .clk         (clk),
        .rst_n       (rst_n),
        // parallel, facing the command source
        .cmd_valid   (cmd_valid[0]),
        .cmd_we      (cmd_we[0]),
        .cmd_addr    (cmd_addr_flat  [0*ADDR_W +: ADDR_W]),
        .cmd_wdata   (cmd_wdata_flat [0*DATA_W +: DATA_W]),
        .cmd_accept  (cmd_accept[0]),
        .done        (done[0]),
        .rdata       (rdata_flat     [0*DATA_W +: DATA_W]),
        .resp        (resp_flat      [0*RESP_W +: RESP_W]),
        .err         (err[0]),
        .cmd_error   (cmd_error),
        .split_count (split_count_flat[0*8 +: 8]),
        .busy        (mst_busy[0]),
        // serial, facing the bus
        .bus_req     (m_req[0]),
        .bus_gnt     (gnt[0]),
        .m_valid     (m_valid[0]),
        .m_we        (m_we[0]),
        .m_astream   (m_astream[0]),          // ADDRESS, one wire
        .m_dstream   (m_dstream[0]),          // WRITE DATA, one wire
        .bus_ready   (bus_ready),
        .bus_resp    (bus_resp),
        .bus_dstream (bus_dstream),           // READ DATA, the shared wire
        // the link to the other board
        .rm_rx          (rm_rx),
        .rm_tx          (rm_tx),
        .remote_busy    (remote_busy),
        .srv_busy       (srv_busy),
        .dbg_rx_last    (dbg_rx_last),
        .dbg_rx_count   (dbg_rx_count),
        .dbg_tx_count   (dbg_tx_count),
        .dbg_rx_state   (dbg_rx_state),
        .dbg_req_seen   (dbg_req_seen),
        .dbg_resp_seen  (dbg_resp_seen),
        .dbg_rx_active  (dbg_rx_active),
        .dbg_req_overrun(dbg_req_overrun)
    );

    // Masters 1..NM-1 - local only
    genvar gi;
    generate
    for (gi = 1; gi < NM; gi = gi + 1) begin : g_master
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
    // 2. THE BUS - no master and no memory inside it
    //======================================================================
    wire [NM-1:0] s2_split_complete;

    // Wake-up pulses from every split-capable slave, OR-ed per master.  Only
    // slave 2 can raise one today; a second split-capable slave joins here.
    wire [NM-1:0] s_split_complete = s2_split_complete;

    system_bus #(
        .N_MASTERS (NM),
        .ID_W      (ID_W),
        .N_SLAVES  (NS),
        .ADDR_W    (ADDR_W),
        .RESP_W    (RESP_W),
        .OBSERVE_ADDR (OBSERVE_ADDR)
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

    // Slave 0 - 2 KB at 0x0000.  Device id 0 on the board-to-board link.
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S0_LADDR_W),
        .WORDS         (`S0_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (0)
    ) u_slave0 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S0]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (1'b0),
        .dstream_out    (s_dstream[`SEL_S0]),
        .ready          (s_ready[`SEL_S0]),
        .resp           (s_resp_flat[`SEL_S0*RESP_W +: RESP_W]),
        .split_complete (),
        .busy           ()
    );

    // Slave 1 - 4 KB at 0x1000
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S1_LADDR_W),
        .WORDS         (`S1_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
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

    // Slave 2 - 4 KB at 0x2000, SPLIT CAPABLE.  Device id 2 on the link, and
    // the spec's "S3 splits on read": the splitter is the THIRD slave.
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
        .ID_W          (ID_W),
        .SPLIT_CAPABLE (1),
        .SPLIT_LATENCY (SPLIT_LATENCY)
    ) u_slave2 (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame          (bus_valid),
        .astream        (bus_astream),
        .dstream_in     (bus_dstream),
        .sel            (s_sel[`SEL_S2]),
        .we             (bus_we),
        .master_id      (master_id),
        .split_en       (split_en),
        .dstream_out    (s_dstream[`SEL_S2]),
        .ready          (s_ready[`SEL_S2]),
        .resp           (s_resp_flat[`SEL_S2*RESP_W +: RESP_W]),
        .split_complete (s2_split_complete),
        .busy           (split_busy)
    );

endmodule
