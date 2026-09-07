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
//   SW[17]      run.  0 = stopped, use KEY[1] to step one transaction
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
    wire       s0_split_en = sw_sync[16];
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
    wire [NM-1:0]        cmd_valid, cmd_we, cmd_accept, done, err, mst_busy;
    wire [NM*ADDR_W-1:0] cmd_addr_flat;
    wire [NM*DATA_W-1:0] cmd_wdata_flat, rdata_flat;
    wire [NM*RESP_W-1:0] resp_flat;
    wire [NM*8-1:0]      split_count_flat;

    wire [1:0]  m0_step,  m1_step;
    wire [15:0] m0_pass,  m1_pass;
    wire [15:0] m0_xacts, m1_xacts;

    master_prog #(.MID(0), .ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_prog0 (
        .clk(clk), .rst_n(rst_n),
        .run_req(run_req), .scenario(scenario),
        .cmd_valid (cmd_valid[0]),
        .cmd_we    (cmd_we[0]),
        .cmd_addr  (cmd_addr_flat [0*ADDR_W +: ADDR_W]),
        .cmd_wdata (cmd_wdata_flat[0*DATA_W +: DATA_W]),
        .cmd_accept(cmd_accept[0]),
        .done      (done[0]),
        .step(m0_step), .pass_count(m0_pass), .xact_count(m0_xacts)
    );

    master_prog #(.MID(1), .ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_prog1 (
        .clk(clk), .rst_n(rst_n),
        .run_req(run_req), .scenario(scenario),
        .cmd_valid (cmd_valid[1]),
        .cmd_we    (cmd_we[1]),
        .cmd_addr  (cmd_addr_flat [1*ADDR_W +: ADDR_W]),
        .cmd_wdata (cmd_wdata_flat[1*DATA_W +: DATA_W]),
        .cmd_accept(cmd_accept[1]),
        .done      (done[1]),
        .step(m1_step), .pass_count(m1_pass), .xact_count(m1_xacts)
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

    bus_top #(
        .N_MASTERS(NM), .ID_W(ID_W), .N_SLAVES(NS),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPLIT_LATENCY)
    ) u_bus (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_we(cmd_we),
        .cmd_addr_flat(cmd_addr_flat), .cmd_wdata_flat(cmd_wdata_flat),
        .cmd_accept(cmd_accept), .done(done),
        .rdata_flat(rdata_flat), .resp_flat(resp_flat), .err(err),
        .split_count_flat(split_count_flat), .mst_busy(mst_busy),
        .s0_split_en(s0_split_en),
        .gnt(gnt), .gnt_valid(gnt_valid), .master_id(master_id),
        .split_mask(split_mask), .sel_q(sel_q),
        .bus_valid(bus_valid), .bus_astream(bus_astream),
        .bus_dstream(bus_dstream), .bus_addr(bus_addr), .addr_done(addr_done),
        .bus_ready(bus_ready), .bus_resp(bus_resp),
        .s0_busy(s0_busy)
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
    assign LEDR[16]    = s0_split_en;
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
