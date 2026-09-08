//==========================================================================
// de2_top.v
//
// DE2-115 board wrapper for the 2-master / 3-slave shared bus.
// Synthesis top level.  Device EP4CE115F29C7 (Cyclone IV E), taken from the
// Quartus project, not guessed.
//
// Everything below runs in ONE clock domain at 50 MHz.  The brief asked for
// a PLL or clock divider to slow the demo down; a divided clock would be a
// gated/derived clock and would break the project's own "one clock domain,
// no gated clocks" rule, so the design instead runs the bus at full speed
// and throttles the SCENARIO SEQUENCER with a slow clock-enable tick.  Same
// visible effect, one clock, timing analysable.
//
//--------------------------------------------------------------------------
// Board controls
//--------------------------------------------------------------------------
//   KEY[0]      reset (active low, debounced, async assert / sync release)
//   KEY[1]      single step - run exactly one more bus transaction
//
//   SW[1:0]     scenario select
//                 00  one master alone   (master 0 works slaves 1 and 2)
//                 01  two masters        (m0 -> slave 1, m1 -> slave 2)
//                 10  split transaction  (m0 -> slave 0, m1 -> slave 2)
//                 11  unmapped recovery  (m0 hits 0x2800 and 0x8000)
//   SW[14]      display master: 0 = master 0, 1 = master 1
//   SW[15]      0 = free run at 50 MHz, 1 = slow (one transaction per tick)
//   SW[16]      slave 0 split enable - the "slave is busy" model
//               (OR-ed with the JTAG host's own split-enable bit)
//   SW[17]      run.  0 = stopped, use KEY[1] to step one transaction
//
//--------------------------------------------------------------------------
// JTAG debug
//--------------------------------------------------------------------------
//   An In-System Sources & Probes instance, ID "SBUS", is built into every
//   bitstream (see bus_issp_driver.v for the bit map).  It is inert until a
//   host sets its issp_mode bit, at which point it takes the masters'
//   command ports away from the on-board sequencers and drives the bus over
//   JTAG instead.  The mux is unconditional, so the switches cannot fight it
//   and SW[17] can stay wherever it is:
//
//       cd Serial_System_Bus && quartus_stp -t tcl/issp_console.tcl
//
//--------------------------------------------------------------------------
// Displays
//--------------------------------------------------------------------------
//   HEX7..HEX4  address of the selected master's last command
//   HEX3..HEX2  that master's SPLIT count
//   HEX1..HEX0  that master's last READ data.  Writes never update it, so
//               the value stays readable.  All 8 data bits reach a pin,
//               which matters: the fitter trims memory bits that cannot
//               reach an output, so an unobservable datapath is a
//               partly-unbuilt one.
//
//   LEDR[1:0]   arbiter grant, one-hot {m1, m0}
//   LEDR[3:2]   arbiter split mask {m1, m0}   <- lit while a master is split
//   LEDR[7:4]   responding slave, {default, s2, s1, s0}
//   LEDR[9:8]   last bus response  00 OKAY, 01 ERROR, 10 SPLIT
//   LEDR[10]    slave 0 busy with a deferred transfer
//   LEDR[11]    a master owns the bus
//   LEDR[13:12] sticky ERROR seen by {m1, m0}
//   LEDR[15:14] master busy {m1, m0}
//   LEDR[16]    slave 0 split enable (echo of SW[16])
//   LEDR[17]    run enable (echo of SW[17])
//
//   LEDG[0]     master 0 completed a transaction (stretched so it is visible)
//   LEDG[1]     master 1 completed a transaction
//   LEDG[2]     slave 0 busy
//   LEDG[3]     bus granted
//   LEDG[7:4]   master 0 split count, low nibble
//   LEDG[8]     any master has seen an ERROR since reset
//
//--------------------------------------------------------------------------
// Port        Dir  Width  DE2-115 net
//--------------------------------------------------------------------------
// CLOCK_50    in   1      50 MHz dedicated clock input, pin Y2
// KEY         in   4      Pushbuttons, active low, KEY[0]=M23 KEY[1]=M21
// SW          in   18     Slide switches
// LEDR        out  18     Red LEDs, active high
// LEDG        out  9      Green LEDs, active high
// HEX0..HEX7  out  7 each Seven-segment digits, active LOW segments
//==========================================================================
`include "bus_defs.vh"

module de2_top #(
    // Slave 0 stays "busy" for this many clocks per split.  ~0.2 s at 50 MHz
    // so the mask LED is actually visible; testbenches override it.
    parameter SPLIT_LATENCY = 10_000_000,
    // Slow-mode tick: one transaction every 2**TICK_W clocks (~0.17 s).
    parameter TICK_W        = 23,
    // Button debounce interval, 2**DEB_W clocks (~1.3 ms).  Testbenches
    // shrink this so a simulated key press does not take 65536 cycles.
    parameter DEB_W         = 16
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [17:0] SW,
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG,
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5,
    output wire [6:0]  HEX6,
    output wire [6:0]  HEX7
);

    localparam NM     = `BUS_N_MASTERS;      // 2 masters on this board
    localparam NS     = `BUS_N_SLAVES;
    localparam ID_W   = 1;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    wire clk = CLOCK_50;

    //======================================================================
    // Reset and buttons
    //======================================================================
    wire rst_n;
    reset_ctrl #(.CNT_W(DEB_W)) u_reset (
        .clk(clk), .key_n(KEY[0]), .rst_n(rst_n)
    );

    wire step_pulse;
    debouncer  #(.CNT_W(DEB_W)) u_step_key (
        .clk(clk), .rst_n(rst_n), .key_n(KEY[1]),
        .level(), .pulse(step_pulse)
    );

    //======================================================================
    // Switch synchronisers.  The slide switches are asynchronous to
    // CLOCK_50; two flops each keeps metastability out of the control logic.
    //======================================================================
    reg [17:0] sw_meta, sw_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sw_meta <= 18'd0;
            sw_sync <= 18'd0;
        end else begin
            sw_meta <= SW;
            sw_sync <= sw_meta;
        end
    end

    wire [1:0] scenario    = sw_sync[1:0];
    wire       disp_sel    = sw_sync[14];
    wire       slow_mode   = sw_sync[15];
    wire       sw_split_en = sw_sync[16];
    wire       run_sw      = sw_sync[17];

    //======================================================================
    // Slow tick for the scenario sequencer.  The BUS is not slowed down -
    // only the rate at which new transactions are launched.
    //======================================================================
    reg [TICK_W-1:0] tick_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tick_cnt <= {TICK_W{1'b0}};
        else        tick_cnt <= tick_cnt + 1'b1;
    end
    wire tick = (tick_cnt == {TICK_W{1'b0}});

    // Continuous run, slow run, or one transaction per KEY[1] press.
    wire run_req = (run_sw ? (slow_mode ? tick : 1'b1) : 1'b0) | step_pulse;

    //======================================================================
    // Master command interfaces
    //======================================================================
    // The masters' command ports have TWO possible owners: the on-board
    // scenario sequencers, and the JTAG debug front-end.  `issp_mode' (a
    // source bit) decides.  Out of reset it is low, so the board demo works
    // with nothing connected.
    wire [NM-1:0]        cmd_valid, cmd_we, cmd_accept, done, err, mst_busy;
    wire [NM*ADDR_W-1:0] cmd_addr_flat;
    wire [NM*DATA_W-1:0] cmd_wdata_flat, rdata_flat;
    wire [NM*RESP_W-1:0] resp_flat;
    wire [NM*8-1:0]      split_count_flat;

    wire [NM-1:0]        prog_valid, prog_we;
    wire [NM*ADDR_W-1:0] prog_addr_flat;
    wire [NM*DATA_W-1:0] prog_wdata_flat;

    wire [NM-1:0]        issp_valid, issp_we;
    wire [NM*ADDR_W-1:0] issp_addr_flat;
    wire [NM*DATA_W-1:0] issp_wdata_flat;
    wire                 issp_mode, issp_split_en;

    wire [1:0]  m0_step,  m1_step;
    wire [15:0] m0_pass,  m1_pass;
    wire [15:0] m0_xacts, m1_xacts;

    master_prog #(.MID(0), .ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_prog0 (
        .clk(clk), .rst_n(rst_n),
        .run_req(run_req), .scenario(scenario),
        .cmd_valid (prog_valid[0]),
        .cmd_we    (prog_we[0]),
        .cmd_addr  (prog_addr_flat [0*ADDR_W +: ADDR_W]),
        .cmd_wdata (prog_wdata_flat[0*DATA_W +: DATA_W]),
        // Gated so a JTAG-issued transaction cannot advance the sequencer.
        .cmd_accept(cmd_accept[0] & ~issp_mode),
        .done      (done[0]      & ~issp_mode),
        .step(m0_step), .pass_count(m0_pass), .xact_count(m0_xacts)
    );

    master_prog #(.MID(1), .ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_prog1 (
        .clk(clk), .rst_n(rst_n),
        .run_req(run_req), .scenario(scenario),
        .cmd_valid (prog_valid[1]),
        .cmd_we    (prog_we[1]),
        .cmd_addr  (prog_addr_flat [1*ADDR_W +: ADDR_W]),
        .cmd_wdata (prog_wdata_flat[1*DATA_W +: DATA_W]),
        // Gated so a JTAG-issued transaction cannot advance the sequencer.
        .cmd_accept(cmd_accept[1] & ~issp_mode),
        .done      (done[1]      & ~issp_mode),
        .step(m1_step), .pass_count(m1_pass), .xact_count(m1_xacts)
    );

    //======================================================================
    // Command-port owner
    //======================================================================
    assign cmd_valid      = issp_mode ? issp_valid      : prog_valid;
    assign cmd_we         = issp_mode ? issp_we         : prog_we;
    assign cmd_addr_flat  = issp_mode ? issp_addr_flat  : prog_addr_flat;
    assign cmd_wdata_flat = issp_mode ? issp_wdata_flat : prog_wdata_flat;

    // Either the switch or the JTAG host can make slave 0 busy.
    wire s0_split_en = sw_split_en | issp_split_en;

    //======================================================================
    // JTAG debug front-end (In-System Sources & Probes, instance "SBUS")
    //
    // Present in every build.  It costs a couple of hundred LEs and stays
    // out of the way until a host sets issp_mode, so there is no separate
    // "debug" bitstream to keep in step with this one.
    //======================================================================
    bus_issp_driver #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)
    ) u_issp (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid        (issp_valid),
        .cmd_we           (issp_we),
        .cmd_addr_flat    (issp_addr_flat),
        .cmd_wdata_flat   (issp_wdata_flat),
        .cmd_accept       (cmd_accept),
        .done             (done),
        .rdata_flat       (rdata_flat),
        .resp_flat        (resp_flat),
        .err              (err),
        .split_count_flat (split_count_flat),
        .gnt              (gnt),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .s0_busy          (s0_busy),
        .bus_addr         (bus_addr),
        .bus_valid        (bus_valid),
        .issp_mode        (issp_mode),
        .s0_split_en      (issp_split_en)
    );

    //======================================================================
    // The bus
    //======================================================================
    wire [NM-1:0]     gnt, split_mask;
    wire              gnt_valid;
    wire [ID_W-1:0]   master_id;
    wire [NS:0]       sel_q;
    wire              bus_valid, bus_ready, s0_busy;
    wire              bus_astream, bus_dstream, addr_done;
    wire [ADDR_W-1:0] bus_addr;
    wire [RESP_W-1:0] bus_resp;

    //======================================================================
    // THE SYSTEM: masters + bus + slaves, wired directly.
    //
    // There is no integration wrapper - `master', `system_bus' and `slave'
    // are instantiated here side by side.  NOTE: the two integration
    // testbenches build the same composition themselves, so if the wiring
    // below changes, tb_integration.v and tb_bus_issp_driver.v change too.
    //
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
    // Serial wires between the bus and the slaves.
    //======================================================================
    wire                            bus_we;
    wire [NS-1:0]             s_sel;
    wire [NS-1:0]             s_ready;
    wire [NS*RESP_W-1:0]      s_resp_flat;
    wire [NS-1:0]             s_dstream;

    //======================================================================
    // 1. MASTERS
    //======================================================================
    genvar gi;
    generate
    for (gi = 0; gi < NM; gi = gi + 1) begin : g_master
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
    wire [NM-1:0] s0_split_complete;

    // Wake-up pulses from every split-capable slave, OR-ed per master.  Only
    // slave 0 can raise one today; a second split-capable slave joins here.
    wire [NM-1:0] s_split_complete = s0_split_complete;

    system_bus #(
        .N_MASTERS (NM),
        .ID_W      (ID_W),
        .N_SLAVES  (NS),
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
        .N_MASTERS     (NM),
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

    // Slave 2 - 2 KB at 0x2000
    slave #(
        .DATA_W        (DATA_W),
        .LADDR_W       (`S2_LADDR_W),
        .WORDS         (`S2_WORDS),
        .RESP_W        (RESP_W),
        .N_MASTERS     (NM),
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

    //======================================================================
    // Status capture for the displays
    //======================================================================
    reg [ADDR_W-1:0] last_addr0, last_addr1;
    reg [RESP_W-1:0] last_resp;
    reg [NM-1:0]     err_sticky;
    reg [19:0]       act0, act1;          // done-pulse stretchers

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_addr0 <= {ADDR_W{1'b0}};
            last_addr1 <= {ADDR_W{1'b0}};
            last_resp  <= `RESP_OKAY;
            err_sticky <= {NM{1'b0}};
            act0       <= 20'd0;
            act1       <= 20'd0;
        end else begin
            if (cmd_accept[0]) last_addr0 <= cmd_addr_flat[0*ADDR_W +: ADDR_W];
            if (cmd_accept[1]) last_addr1 <= cmd_addr_flat[1*ADDR_W +: ADDR_W];

            if (bus_ready) last_resp <= bus_resp;

            // Sticky until the next reset, so a single ERROR is not missed.
            if (done[0] && err[0]) err_sticky[0] <= 1'b1;
            if (done[1] && err[1]) err_sticky[1] <= 1'b1;

            // Stretch each done pulse to ~20 ms so the eye can catch it.
            if (done[0])     act0 <= 20'hFFFFF;
            else if (|act0)  act0 <= act0 - 1'b1;
            if (done[1])     act1 <= 20'hFFFFF;
            else if (|act1)  act1 <= act1 - 1'b1;
        end
    end

    //======================================================================
    // LEDs
    //======================================================================
    assign LEDR[1:0]   = gnt;
    assign LEDR[3:2]   = split_mask;
    assign LEDR[7:4]   = sel_q;
    assign LEDR[9:8]   = last_resp;
    assign LEDR[10]    = s0_busy;
    assign LEDR[11]    = gnt_valid;
    assign LEDR[13:12] = err_sticky;
    assign LEDR[15:14] = mst_busy;
    assign LEDR[16]    = s0_split_en;   // either source
    assign LEDR[17]    = run_sw;

    assign LEDG[0]     = |act0;
    assign LEDG[1]     = |act1;
    assign LEDG[2]     = s0_busy;
    assign LEDG[3]     = gnt_valid;
    assign LEDG[7:4]   = split_count_flat[0*8 +: 4];
    assign LEDG[8]     = |err_sticky;

    //======================================================================
    // Seven-segment displays
    //======================================================================
    wire [ADDR_W-1:0] disp_addr  = disp_sel ? last_addr1 : last_addr0;
    wire [DATA_W-1:0] disp_rdata = disp_sel ? rdata_flat[1*DATA_W +: DATA_W]
                                            : rdata_flat[0*DATA_W +: DATA_W];
    wire [7:0]        disp_splits = disp_sel ? split_count_flat[1*8 +: 8]
                                             : split_count_flat[0*8 +: 8];

    seg7_hex u_h7 (.val(disp_addr [15:12]), .blank(1'b0), .seg(HEX7));
    seg7_hex u_h6 (.val(disp_addr [11: 8]), .blank(1'b0), .seg(HEX6));
    seg7_hex u_h5 (.val(disp_addr [ 7: 4]), .blank(1'b0), .seg(HEX5));
    seg7_hex u_h4 (.val(disp_addr [ 3: 0]), .blank(1'b0), .seg(HEX4));
    seg7_hex u_h3 (.val(disp_splits[7:4]), .blank(1'b0), .seg(HEX3));
    seg7_hex u_h2 (.val(disp_splits[3:0]), .blank(1'b0), .seg(HEX2));
    seg7_hex u_h1 (.val(disp_rdata [7:4]), .blank(1'b0), .seg(HEX1));
    seg7_hex u_h0 (.val(disp_rdata [3:0]), .blank(1'b0), .seg(HEX0));

    //----------------------------------------------------------------------
    // KEY[3:2] and SW[13:2] are brought out to their board pins but are not
    // used by this design, and master_id / bus_valid / bus_rdata / the
    // sequencer counters exist for SignalTap and waveforms only.  Quartus
    // reports these as "input pins that do not drive logic" - that warning
    // is expected here, not a defect.
    //----------------------------------------------------------------------

endmodule
