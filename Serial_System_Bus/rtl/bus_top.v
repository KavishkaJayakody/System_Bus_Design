//==========================================================================
// bus_top.v
//
// THE COMPLETE BUS SYSTEM: 2 local masters + system_bus + 3 memory slaves +
// the remote BRIDGE, wired together and nothing else.  No board, no switches,
// no JTAG, no displays.
//
// The bridge is a DEVICE ON THE BUS, not part of a master.  It has two faces:
// a slave face at 0x8000-0xBFFF that either local master can address, and a
// master face at arbiter index 2 - the lowest priority - through which the
// far board reaches all three local memories.  So this system has THREE bus
// masters but only TWO command ports.
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
// cmd_valid         in   NLM         LEVEL, held until cmd_accept.
// cmd_we            in   NLM         1 = write, 0 = read.
// cmd_addr_flat     in   NLM*ADDR_W  Command addresses, packed.
// cmd_wdata_flat    in   NLM*DATA_W  Command write data, packed.
// cmd_accept        out  NLM         The master took the command, one clock.
// done              out  NLM         Transaction finished, one clock.
// rdata_flat        out  NLM*DATA_W  Last read data per master, packed.
// resp_flat         out  NLM*RESP_W  Final response per master, packed.
// err               out  NLM         That transaction answered ERROR.
// split_count_flat  out  NLM*8       Splits absorbed per master, packed.
// mst_busy          out  NLM         Master has a command in flight.
//
// NLM = NM-1: the bridge's master face is a bus master with no command port.
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
    parameter NM            = `BUS_N_MASTERS,   // bus masters, incl. the bridge
    parameter NS            = `BUS_N_SLAVES,    // decoded targets, incl. the bridge
    parameter ID_W          = `BUS_ID_W,
    parameter ADDR_W        = `BUS_ADDR_W,
    parameter DATA_W        = `BUS_DATA_W,
    parameter RESP_W        = `BUS_RESP_W,
    // The split-capable memory stays "busy" for this many clocks per split.
    // The board build uses ~0.2 s so a split is visible from the host.
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
    // NLM of them, one per LOCAL master.  The bridge's master face is bus
    // master NM-1 and is driven by the far board, not from here.
    input  wire [NLM-1:0]          cmd_valid,
    input  wire [NLM-1:0]          cmd_we,
    input  wire [NLM*ADDR_W-1:0]   cmd_addr_flat,
    input  wire [NLM*DATA_W-1:0]   cmd_wdata_flat,
    output wire [NLM-1:0]          cmd_accept,
    output wire [NLM-1:0]          done,
    output wire [NLM*DATA_W-1:0]   rdata_flat,
    output wire [NLM*RESP_W-1:0]   resp_flat,
    output wire [NLM-1:0]          err,
    output wire [NLM*8-1:0]        split_count_flat,
    output wire [NLM-1:0]          mst_busy,

    // ---- the "slave is busy" model, on the split-capable memory ----------
    input  wire                    split_en,

    // ---- board-to-board link (THE BRIDGE) --------------------------------
    // There is no "remote" input: the ADDRESS selects the far board, and
    // 0x8000-0xBFFF is decoded to the bridge like any other target.
    output wire                    cmd_error,      // remote read timed out
    input  wire                    rm_rx,
    output wire                    rm_tx,
    output wire                    remote_busy,
    output wire                    srv_busy,
    // link diagnostics - observation only, see bus_bridge.v
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

    // Locally commanded masters.  The last bus master is the bridge.
    localparam NLM = NM - 1;

    //======================================================================
    // Serial wires between the masters and the bus.  One bit per master per
    // stream - these are the CANDIDATES; the bus picks one with the grant.
    //======================================================================
    wire [NM-1:0]  m_req;
    wire [NM-1:0]  m_valid;
    wire [NM-1:0]  m_we;
    wire [NM-1:0]  m_astream;
    wire [NM-1:0]  m_dstream;

    //======================================================================
    // Wires between the bus and the decoded targets.
    //======================================================================
    wire [NS-1:0]          s_sel;
    wire [NS-1:0]          s_ready;
    wire [NS*RESP_W-1:0]   s_resp_flat;
    wire [NS-1:0]          s_dstream;

    //======================================================================
    // 1. LOCAL MASTERS
    //
    // Parallel command in, SERIAL onto the bus.  Both are plain `master's -
    // there is no UART inside either of them any more.  A master reaches the
    // other board by addressing the bridge, exactly as it addresses memory.
    //======================================================================
    genvar gi;
    generate
    for (gi = 0; gi < NLM; gi = gi + 1) begin : g_master
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
    wire [NM-1:0] br_split_complete;

    // Wake-up pulses from every split-capable target, OR-ed per master: the
    // split memory and the bridge both defer transfers and both release them.
    wire [NM-1:0] s_split_complete = s2_split_complete | br_split_complete;

    system_bus #(
        .N_MASTERS    (NM),
        .ID_W         (ID_W),
        .N_SLAVES     (NS),
        .ADDR_W       (ADDR_W),
        .RESP_W       (RESP_W),
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

        // target side
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
    // 3. MEMORY SLAVES
    //
    // Instantiated one by one rather than in a generate loop, because they
    // differ in size and in whether they can split - and those differences
    // are worth reading at a glance.  All three tap the SAME two wires.
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
    // the spec's "S3 splits on read": the splitter is the THIRD memory.
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

    //======================================================================
    // 4. THE BRIDGE - target 3 on the slave side, bus master NM-1 on the
    //    other.  The only UART in the design lives in here.
    //======================================================================
    bus_bridge #(
        .ADDR_W       (ADDR_W),
        .DATA_W       (DATA_W),
        .RESP_W       (RESP_W),
        .N_MASTERS    (NM),
        .ID_W         (ID_W),
        .LADDR_W      (`S3_LADDR_W),
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .RESP_TIMEOUT (RESP_TIMEOUT)
    ) u_bridge (
        .clk             (clk),
        .rst_n           (rst_n),

        // slave face - addressed at 0x8000-0xBFFF like any other target
        .frame           (bus_valid),
        .astream         (bus_astream),
        .dstream_in      (bus_dstream),
        .sel             (s_sel[`SEL_BR]),
        .we              (bus_we),
        .master_id       (master_id),
        .dstream_out     (s_dstream[`SEL_BR]),
        .ready           (s_ready[`SEL_BR]),
        .resp            (s_resp_flat[`SEL_BR*RESP_W +: RESP_W]),
        .split_complete  (br_split_complete),
        .busy            (),

        // master face - bus master NM-1, the LOWEST arbiter priority, so
        // remote traffic can never out-rank the local masters
        .bus_req         (m_req[NM-1]),
        .bus_gnt         (gnt[NM-1]),
        .m_valid         (m_valid[NM-1]),
        .m_we            (m_we[NM-1]),
        .m_astream       (m_astream[NM-1]),
        .m_dstream       (m_dstream[NM-1]),
        .bus_ready       (bus_ready),
        .bus_resp        (bus_resp),
        .bus_dstream     (bus_dstream),

        // the wire
        .rm_rx           (rm_rx),
        .rm_tx           (rm_tx),

        // status
        .br_error        (cmd_error),
        .remote_busy     (remote_busy),
        .srv_busy        (srv_busy),
        .dbg_rx_last     (dbg_rx_last),
        .dbg_rx_count    (dbg_rx_count),
        .dbg_tx_count    (dbg_tx_count),
        .dbg_rx_state    (dbg_rx_state),
        .dbg_req_seen    (dbg_req_seen),
        .dbg_resp_seen   (dbg_resp_seen),
        .dbg_rx_active   (dbg_rx_active),
        .dbg_req_overrun (dbg_req_overrun)
    );

endmodule
