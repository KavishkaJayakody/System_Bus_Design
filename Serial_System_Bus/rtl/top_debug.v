//==========================================================================
// top_debug.v
//
// Synthesis top level for the SERIAL system bus: the bus system plus its
// JTAG In-System Sources and Probes driver, and nothing else.
//
// Only CLOCK_50, rst_n, led[7:0] and the two UART bridge pins leave the
// device.  Both master command ports are driven over JTAG by
// `bus_issp_driver', so there are no switches to set, no scenario to select
// and no display to read - the host issues the transaction and reads the
// answer back.
//
// The bridge pins carry REMOTE transactions to a second board: cross them
// over (each board's rm_tx to the other's rm_rx, plus a COMMON GROUND) and
// an address with bit 15 set is executed on the far board instead of this
// one - far address = local address - 0x8000.  See master_uart.v.
//
// This is the same shape as `top_debug' in the parallel System_Bus_Final
// project, and for the same reason: the debug front-end sits IN FRONT of the
// system, driving it through its normal parallel command ports, so a JTAG
// transaction takes exactly the path any other transaction takes.  There is
// no back door onto the serial wires.
//
//   top_debug
//    +- bus_issp_driver   the JTAG front-end, ISSP instance "SBUS"
//    +- bus_top           THE SYSTEM: master x2 + system_bus + slave x3
//
// This file, like `bus_top', contains no logic - wires, two instances and
// one assign.  Everything that decides anything is inside those two.
//
//--------------------------------------------------------------------------
// Driving it
//--------------------------------------------------------------------------
//   cd Serial_System_Bus
//   quartus_stp -t tcl/issp_console.tcl     interactive
//   quartus_stp -t tcl/issp_bus_test.tcl    scripted, exit 0 = pass
//
// `quartus_stp' is the only interpreter that works - the JTAG and ISSP Tcl
// packages are absent from `quartus_sh' and from the Quartus GUI console.
// Close the In-System Sources & Probes Editor tab first; an open editor
// holds the JTAG session.
//
// top_debug MUST be the TOP_LEVEL_ENTITY.  The driver is instantiated here;
// with anything else as top there is no ISSP in the bitstream at all.
//
//--------------------------------------------------------------------------
// Port        Dir  Width  DE2-115 net
//--------------------------------------------------------------------------
// CLOCK_50    in   1      50 MHz dedicated clock input, pin Y2
// rm_rx       in   1      GPIO[0], JP5 pin AB22 - UART RX from the other
//                         board's rm_tx.
//                         Leave it unconnected and a remote READ times out
//                         with cmd_error after 10 ms; nothing hangs.
// rm_tx       out  1      GPIO[1], JP5 pin AC15 - UART TX to the other
//                         board's rm_rx.  Crossed over, COMMON GROUND.
//                         The spec's D3/C3 are the far board's pins and
//                         do not exist on this device; only the baud
//                         rate and frame format must match.
// rst_n       in   1      KEY[0], pin M23, active low with a pull-up.  The
//                         only recovery from a wedged bus - though on this
//                         design nothing should wedge it, since every
//                         unmapped address is answered ERROR by the default
//                         responder inside system_bus.
// led         out  8      LEDR[7:0], active high.  Master 0's last READ
//                         data.  `master' holds cmd_rdata until the next
//                         read completes, so it is stable with no latch
//                         here - and all 8 bits reach a pin, which is what
//                         stops the fitter trimming the memories.
//==========================================================================
`include "bus_defs.vh"

module top_debug #(
    // Slave 0 stays "busy" for this many clocks per split.  ~0.2 s at 50 MHz
    // so a split is visible from the host; testbenches override it.
    parameter SPLIT_LATENCY = 10_000_000,
    // UART link to the other board.  50 MHz / 115200 baud = 434, and a
    // remote reply that does not arrive within RESP_TIMEOUT clocks (~10 ms)
    // completes the transaction with cmd_error instead of hanging.
    parameter CLKS_PER_BIT  = 434,
    parameter RESP_TIMEOUT  = 500000
) (
    input  wire       CLOCK_50,
    input  wire       rst_n,
    input  wire       rm_rx,
    output wire       rm_tx,
    output wire [7:0] led
);

    localparam NM     = `BUS_N_MASTERS;
    localparam NS     = `BUS_N_SLAVES;
    localparam ID_W   = 1;
    localparam ADDR_W = `BUS_ADDR_W;
    localparam DATA_W = `BUS_DATA_W;
    localparam RESP_W = `BUS_RESP_W;

    wire clk = CLOCK_50;

    //======================================================================
    // Command ports, driver -> system.  Parallel: `master' serialises.
    //======================================================================
    wire [NM-1:0]        cmd_valid, cmd_we, cmd_accept, done, err, mst_busy;
    wire [NM*ADDR_W-1:0] cmd_addr_flat;
    wire [NM*DATA_W-1:0] cmd_wdata_flat, rdata_flat;
    wire [NM*RESP_W-1:0] resp_flat;
    wire [NM*8-1:0]      split_count_flat;
    wire                 split_en;
    wire                 cmd_error, remote_busy, srv_busy;
    wire [7:0]           dbg_rx_last, dbg_rx_count, dbg_tx_count;
    wire [1:0]           dbg_rx_state;
    wire                 dbg_req_seen, dbg_resp_seen, dbg_rx_active;
    wire                 dbg_req_overrun;

    //======================================================================
    // Observation, system -> driver.  None of it can affect a transfer.
    //======================================================================
    wire [NM-1:0]     gnt, split_mask;
    wire              gnt_valid;
    wire [ID_W-1:0]   master_id;
    wire [NS:0]       sel_q;
    wire              bus_valid, bus_we, bus_ready, split_busy;
    wire              bus_astream, bus_dstream, addr_done;
    wire [ADDR_W-1:0] bus_addr;
    wire [RESP_W-1:0] bus_resp;

    //======================================================================
    // The JTAG debug front-end, IN FRONT of the system
    //======================================================================
    bus_issp_driver #(
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W)
    ) u_dbg (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid        (cmd_valid),
        .cmd_we           (cmd_we),
        .cmd_addr_flat    (cmd_addr_flat),
        .cmd_wdata_flat   (cmd_wdata_flat),
        .cmd_accept       (cmd_accept),
        .done             (done),
        .rdata_flat       (rdata_flat),
        .resp_flat        (resp_flat),
        .err              (err),
        .split_count_flat (split_count_flat),
        .gnt              (gnt),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .split_busy       (split_busy),
        .bus_addr         (bus_addr),
        .bus_valid        (bus_valid),
        // the UART link, master 0 only
        .cmd_error        (cmd_error),
        .remote_busy      (remote_busy),
        .srv_busy         (srv_busy),
        .dbg_rx_last      (dbg_rx_last),
        .dbg_rx_count     (dbg_rx_count),
        .dbg_tx_count     (dbg_tx_count),
        .dbg_rx_state     (dbg_rx_state),
        .dbg_req_seen     (dbg_req_seen),
        .dbg_resp_seen    (dbg_resp_seen),
        .dbg_rx_active    (dbg_rx_active),
        .dbg_req_overrun  (dbg_req_overrun),
        // The driver is the ONLY command source now, so issp_mode has
        // nothing left to arbitrate and is left open.  It stays on the
        // driver because the bit map is shared with tcl/issp_bus_lib.tcl.
        .issp_mode        (),
        .split_en         (split_en)
    );

    //======================================================================
    // THE SYSTEM: master x2 + system_bus + slave x3
    //======================================================================
    bus_top #(
        .NM(NM), .NS(NS), .ID_W(ID_W),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W), .RESP_W(RESP_W),
        .SPLIT_LATENCY(SPLIT_LATENCY),
        .CLKS_PER_BIT(CLKS_PER_BIT), .RESP_TIMEOUT(RESP_TIMEOUT)
    ) u_bus (
        .clk              (clk),
        .rst_n            (rst_n),

        .cmd_valid        (cmd_valid),
        .cmd_we           (cmd_we),
        .cmd_addr_flat    (cmd_addr_flat),
        .cmd_wdata_flat   (cmd_wdata_flat),
        .cmd_accept       (cmd_accept),
        .done             (done),
        .rdata_flat       (rdata_flat),
        .resp_flat        (resp_flat),
        .err              (err),
        .split_count_flat (split_count_flat),
        .mst_busy         (mst_busy),

        .split_en         (split_en),

        .cmd_error        (cmd_error),
        .rm_rx            (rm_rx),
        .rm_tx            (rm_tx),
        .remote_busy      (remote_busy),
        .srv_busy         (srv_busy),
        .dbg_rx_last      (dbg_rx_last),
        .dbg_rx_count     (dbg_rx_count),
        .dbg_tx_count     (dbg_tx_count),
        .dbg_rx_state     (dbg_rx_state),
        .dbg_req_seen     (dbg_req_seen),
        .dbg_resp_seen    (dbg_resp_seen),
        .dbg_rx_active    (dbg_rx_active),
        .dbg_req_overrun  (dbg_req_overrun),

        .gnt              (gnt),
        .gnt_valid        (gnt_valid),
        .split_mask       (split_mask),
        .sel_q            (sel_q),
        .split_busy       (split_busy),
        .master_id        (master_id),
        .bus_valid        (bus_valid),
        .bus_we           (bus_we),
        .bus_ready        (bus_ready),
        .bus_resp         (bus_resp),
        .bus_addr         (bus_addr),
        .addr_done        (addr_done),
        .bus_astream      (bus_astream),
        .bus_dstream      (bus_dstream)
    );

    // `master' captures rdata on reads only, so this is stable between
    // reads with no extra latch here.
    assign led = rdata_flat[0*DATA_W +: DATA_W];

endmodule
