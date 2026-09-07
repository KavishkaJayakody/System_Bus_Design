//==========================================================================
// slave_mem.v
//
// Word-addressed memory slave behind the bus ready/response handshake.
// One module covers all three slaves; WORDS/LADDR_W set the size and
// SPLIT_CAPABLE adds the split machinery for slave 0.
//
// Timing (this is the whole slave contract):
//
//   cycle T    sel=1, we/addr/wdata/master_id valid.  The slave registers
//              the access.  It does NOT drive ready in this cycle.
//   cycle T+1  ready=1 together with resp and, on a read, rdata.
//
// One cycle of latency, because the memory read is synchronous - which is
// what lets Quartus put the array in M9K block RAM instead of LUTs.  The
// return mux in bus_mux is steered by a registered select for exactly this
// reason.
//
// SPLIT (SPLIT_CAPABLE=1 only)
//   While split_en is high a fresh access is answered with resp=SPLIT
//   instead of being performed: the slave latches the requesting master_id,
//   counts SPLIT_LATENCY cycles, then pulses split_complete[that id] for one
//   cycle.  The arbiter unmasks that master, the master replays the
//   identical transfer, and this second attempt is recognised (resume_pend
//   with a matching master id) and served normally with OKAY.
//
//   The deferred transfer is NOT performed during the split - no write
//   happens, no rdata is captured.  The master replays the whole transfer,
//   so the slave only has to remember *who* it deferred, not what.
//
//   Exactly one split may be outstanding.  An access arriving while the
//   slave is busy, or while a resume is still pending for another master, is
//   served normally with OKAY.  That is a deliberate simplification: it
//   keeps the deferred-transfer state single-entry and makes it impossible
//   for two masters to be masked with only one split_complete to wake them.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock.
// rst_n           in   1          Asynchronous active-low reset.
// sel             in   1          Access strobe: this slave is selected and
//                                 the master is driving a transfer.  Already
//                                 the AND of the decoder select and
//                                 bus_valid.  One cycle per transfer.
// we              in   1          1 = write wdata, 0 = read into rdata.
// addr            in   LADDR_W    Word address inside this slave, i.e. the
//                                 low bits of the bus address.
// wdata           in   DATA_W     Write data.
// master_id       in   ID_W       Tag of the master owning the bus, from the
//                                 arbiter.  Only used when SPLIT_CAPABLE.
// split_en        in   1          1 = model the slave as busy, so fresh
//                                 accesses are answered with SPLIT.  Tied
//                                 low on the non-split slaves.
// rdata           out  DATA_W     Read data, valid with ready when resp=OKAY
//                                 on a read.
// ready           out  1          Completion strobe, one cycle, asserted the
//                                 cycle after sel.
// resp            out  RESP_W     OKAY or SPLIT, valid with ready.
// split_complete  out  N_MASTERS  One-cycle pulse on the bit of the master
//                                 whose deferred transfer may now be
//                                 replayed.  Always 0 when SPLIT_CAPABLE=0.
// busy            out  1          A split is in flight.  Status/debug only.
//==========================================================================
`include "bus_defs.vh"

module slave_mem #(
    parameter DATA_W        = `BUS_DATA_W,
    parameter LADDR_W       = 12,
    parameter WORDS         = 4096,
    parameter RESP_W        = `BUS_RESP_W,
    parameter N_MASTERS     = `BUS_N_MASTERS,
    parameter ID_W          = 1,
    parameter SPLIT_CAPABLE = 0,
    parameter SPLIT_LATENCY = 4,     // cycles the slave stays "busy"
    // Width of the busy counter.  32 bits so any SPLIT_LATENCY an integer
    // parameter can express fits without being silently truncated - the
    // board uses 10,000,000 clocks (~0.2 s) so the mask LED is visible,
    // which a narrower counter would have quietly wrapped.
    parameter CNT_W         = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  sel,
    input  wire                  we,
    input  wire [LADDR_W-1:0]    addr,
    input  wire [DATA_W-1:0]     wdata,
    input  wire [ID_W-1:0]       master_id,
    input  wire                  split_en,
    output reg  [DATA_W-1:0]     rdata,
    output reg                   ready,
    output reg  [RESP_W-1:0]     resp,
    output wire [N_MASTERS-1:0]  split_complete,
    output wire                  busy
);

    //----------------------------------------------------------------------
    // Split control.  Generated away entirely on the plain slaves.
    //----------------------------------------------------------------------
    wire do_split;      // this cycle's access is being deferred

    generate
    if (SPLIT_CAPABLE) begin : g_split

        reg                  busy_r;
        reg  [CNT_W-1:0]     cnt_r;
        reg  [ID_W-1:0]      split_id_r;
        reg                  resume_pend_r;
        reg  [ID_W-1:0]      resume_id_r;
        reg  [N_MASTERS-1:0] sc_r;

        // Sized copy of the latency, so the load below is not a 32-bit
        // integer being silently truncated into a narrower counter.
        localparam [CNT_W-1:0] LAT_C = SPLIT_LATENCY;

        // The replay of a deferred transfer: same master, resume still owed.
        wire is_resume = resume_pend_r && (master_id == resume_id_r);

        // Defer only a fresh access, only while modelled busy, and only when
        // no other split is already in flight or awaiting its replay.
        assign do_split = sel && split_en && !busy_r && !resume_pend_r;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                busy_r        <= 1'b0;
                cnt_r         <= {CNT_W{1'b0}};
                split_id_r    <= {ID_W{1'b0}};
                resume_pend_r <= 1'b0;
                resume_id_r   <= {ID_W{1'b0}};
                sc_r          <= {N_MASTERS{1'b0}};
            end else begin
                sc_r <= {N_MASTERS{1'b0}};      // default: one-cycle pulse

                if (do_split) begin
                    busy_r     <= 1'b1;
                    cnt_r      <= LAT_C;
                    split_id_r <= master_id;
                end else if (busy_r) begin
                    if (cnt_r == {CNT_W{1'b0}}) begin
                        busy_r           <= 1'b0;
                        sc_r[split_id_r] <= 1'b1;   // wake that master
                        resume_pend_r    <= 1'b1;
                        resume_id_r      <= split_id_r;
                    end else begin
                        cnt_r <= cnt_r - 1'b1;
                    end
                end

                // The replay consumes the outstanding resume.  This cannot
                // collide with the set above: is_resume requires
                // resume_pend_r=1, the set only happens when it was 0.
                if (sel && is_resume)
                    resume_pend_r <= 1'b0;
            end
        end

        assign split_complete = sc_r;
        assign busy           = busy_r;

    end else begin : g_nosplit

        assign do_split       = 1'b0;
        assign split_complete = {N_MASTERS{1'b0}};
        assign busy           = 1'b0;

    end
    endgenerate

    // An access that is actually performed (not deferred).
    wire serve = sel && !do_split;

    //----------------------------------------------------------------------
    // Handshake registers.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            resp  <= `RESP_OKAY;
        end else begin
            ready <= sel;
            resp  <= do_split ? `RESP_SPLIT : `RESP_OKAY;
        end
    end

    //----------------------------------------------------------------------
    // Memory array and its output register.
    //
    // DELIBERATE EXCEPTION to the "every sequential block has an async reset"
    // rule: this block is clock-only.  Giving a 4096-word array an
    // asynchronous reset forces Quartus to abandon M9K inference and build
    // the memory out of LUTs and flip-flops - 4096*32 registers, which does
    // not fit and would not close timing.  The array is therefore not reset
    // and holds no defined value at power-up; every test writes a location
    // before it reads it.  rdata sits in the same block so it can be the M9K
    // output register.
    //----------------------------------------------------------------------
    reg [DATA_W-1:0] mem [0:WORDS-1];

    always @(posedge clk) begin
        if (serve && we)
            mem[addr] <= wdata;
        if (serve && !we)
            rdata <= mem[addr];
    end

endmodule
