//==========================================================================
// slave.v
//
// Word-addressed memory slave on the SERIAL bus.  One module covers all
// three slaves; WORDS/LADDR_W set the size and SPLIT_CAPABLE adds the split
// machinery for slave 0.
//
// Every slave taps the same two wires and shifts them in unconditionally
// while `frame' is high - none of them knows yet whether it is the one being
// addressed, because the decoder cannot decide until the last address bit
// has arrived.  Selection comes afterwards, as a one-cycle `sel' pulse.
// Shifting speculatively costs nothing: a slave that was not selected simply
// never acts on what it collected.
//
// Deserialiser widths do all the field extraction for free:
//   * the address deserialiser is LADDR_W wide, not ADDR_W, so after the
//     whole frame it holds addr[LADDR_W-1:0] - this slave's offset - with
//     the upper address bits shifted straight through and discarded
//   * the write-data deserialiser is DATA_W wide, and the master
//     right-aligns the data in the frame, so it ends up holding exactly the
//     data
// Neither needs a bit counter.
//
// TIMING
//
//   cycle S     `sel' pulses.  laddr and wdata are already assembled.
//               write -> the word is committed here
//               read  -> the memory read starts here
//   S+1         write: ready + OKAY.  Split: ready + SPLIT.
//   S+2 .. S+9  read: DATA_W bits shifted out on dstream_out, MSB first
//   S+10        read: ready + OKAY
//
// The read data phase starts at S+2, not S+1, and the extra cycle is
// deliberate.  `mem_q' has to be a plain register loaded straight from the
// array so Quartus can absorb it as the M9K output register; making it the
// shift register instead would force an asynchronous array read and drop the
// whole memory into LUTs.  So mem_q lands at S+1 and is copied into the
// output shift register for S+2.
//
// SPLIT (SPLIT_CAPABLE=1 only)
//   While split_en is high a fresh access is answered with SPLIT instead of
//   being performed: the slave latches the requesting master_id, counts
//   SPLIT_LATENCY cycles, then pulses split_complete[that id].  The arbiter
//   unmasks that master, it re-transmits the identical frame, and the second
//   attempt is recognised (resume_pend with a matching master id) and served
//   normally.  The deferred transfer is NOT performed - no write lands, no
//   read happens - so the slave only has to remember WHO it deferred.
//
//   Note the split answer costs one cycle while a real read costs ten.  On a
//   serial bus that asymmetry is the whole point of splitting: the slave gets
//   out of the way fast and the bus goes to somebody else.
//
//   Exactly one split may be outstanding.  An access arriving while the
//   slave is busy, or while a resume is still owed, is served normally.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock.
// rst_n           in   1          Asynchronous active-low reset.
// frame           in   1          Address-phase marker (bus_valid).  High for
//                                 ADDR_W clocks; the deserialisers run while
//                                 it is high.
// astream         in   1          Shared serial address wire.
// dstream_in      in   1          Shared serial data wire, as an input.
// sel             in   1          One-cycle select from the decoder, the
//                                 cycle after the frame ends.
// we              in   1          1 = write.  Held stable for the whole
//                                 transaction by the granted master.
// master_id       in   ID_W       Tag of the master owning the bus.  Only
//                                 used when SPLIT_CAPABLE.
// split_en        in   1          1 = model the slave as busy.  Tied low on
//                                 the non-split slaves.
// dstream_out     out  1          This slave's drive onto the shared data
//                                 wire.  0 unless it is actually shifting
//                                 read data out.
// ready           out  1          Completion strobe, one cycle.
// resp            out  RESP_W     OKAY or SPLIT, valid with ready.
// split_complete  out  N_MASTERS  One-cycle pulse on the bit of the master
//                                 whose deferred transfer may be replayed.
// busy            out  1          A split is in flight.
//==========================================================================
`include "bus_defs.vh"

module slave #(
    parameter DATA_W        = `BUS_DATA_W,
    parameter LADDR_W       = 12,
    parameter WORDS         = 4096,
    parameter RESP_W        = `BUS_RESP_W,
    parameter N_MASTERS     = `BUS_N_MASTERS,
    parameter ID_W          = 1,
    parameter SPLIT_CAPABLE = 0,
    parameter SPLIT_LATENCY = 4,
    // Width of the busy counter.  32 bits so any SPLIT_LATENCY an integer
    // parameter can express fits without being silently truncated - the
    // board uses 10,000,000 clocks (~0.2 s) so the mask LED is visible.
    parameter CNT_W         = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  frame,
    input  wire                  astream,
    input  wire                  dstream_in,
    input  wire                  sel,
    input  wire                  we,
    input  wire [ID_W-1:0]       master_id,
    input  wire                  split_en,
    output wire                  dstream_out,
    output reg                   ready,
    output reg  [RESP_W-1:0]     resp,
    output wire [N_MASTERS-1:0]  split_complete,
    output wire                  busy
);

    function integer clogb2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clogb2 = 1; v > 1; clogb2 = clogb2 + 1)
                v = v >> 1;
        end
    endfunction
    localparam BCNT_W = clogb2(DATA_W);

    //----------------------------------------------------------------------
    // Deserialise the frame.  Runs on every slave, selected or not.
    //----------------------------------------------------------------------
    wire [LADDR_W-1:0] laddr;
    wire [DATA_W-1:0]  wdata_des;

    shift_deser #(.W(LADDR_W)) u_addr_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(astream), .dout(laddr)
    );

    shift_deser #(.W(DATA_W)) u_wdata_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(dstream_in), .dout(wdata_des)
    );

    //----------------------------------------------------------------------
    // Split control.  Generated away entirely on the plain slaves.
    //----------------------------------------------------------------------
    wire do_split;

    generate
    if (SPLIT_CAPABLE) begin : g_split

        reg                  busy_r;
        reg  [CNT_W-1:0]     cnt_r;
        reg  [ID_W-1:0]      split_id_r;
        reg                  resume_pend_r;
        reg  [ID_W-1:0]      resume_id_r;
        reg  [N_MASTERS-1:0] sc_r;

        localparam [CNT_W-1:0] LAT_C = SPLIT_LATENCY;

        wire is_resume = resume_pend_r && (master_id == resume_id_r);

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
                        sc_r[split_id_r] <= 1'b1;
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
    wire serve     = sel && !do_split;
    wire mem_write = serve &&  we;
    wire mem_read  = serve && !we;

    //----------------------------------------------------------------------
    // Read data phase.
    //   R_IDLE  nothing going on
    //   R_MEMQ  one cycle: mem_q holds the word, load the output register
    //   R_SHIFT DATA_W cycles: drive the wire, MSB first
    //----------------------------------------------------------------------
    localparam R_IDLE  = 2'd0;
    localparam R_MEMQ  = 2'd1;
    localparam R_SHIFT = 2'd2;

    reg [1:0]        rs;
    reg [BCNT_W-1:0] rcnt;

    wire rd_last = (rs == R_SHIFT) && (rcnt == (DATA_W-1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rs   <= R_IDLE;
            rcnt <= {BCNT_W{1'b0}};
        end else begin
            case (rs)
                R_IDLE:  if (mem_read) rs <= R_MEMQ;
                R_MEMQ:  begin rs <= R_SHIFT; rcnt <= {BCNT_W{1'b0}}; end
                R_SHIFT: begin
                    rcnt <= rcnt + 1'b1;
                    if (rd_last) rs <= R_IDLE;
                end
                default: rs <= R_IDLE;
            endcase
        end
    end

    //----------------------------------------------------------------------
    // Handshake.  A write or a split answers in one cycle; a read answers
    // when the last bit has gone out.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            resp  <= `RESP_OKAY;
        end else begin
            ready <= (sel && (do_split || we)) || rd_last;
            resp  <= do_split ? `RESP_SPLIT : `RESP_OKAY;
        end
    end

    //----------------------------------------------------------------------
    // Memory array and its output register.
    //
    // DELIBERATE EXCEPTION to the "every sequential block has an async reset"
    // rule: this block is clock-only.  Giving a 4096-word array an
    // asynchronous reset stops Quartus inferring M9K block RAM and makes it
    // build the memory out of LUTs and flip-flops instead, which does not fit
    // and would not close timing.  The array holds no defined value at
    // power-up; every test writes a location before reading it.
    //
    // Read and write enables are mutually exclusive, so the array is never
    // read and written in the same cycle - a read-during-write would make
    // Quartus infer a RAM whose result it documents as undefined.
    //----------------------------------------------------------------------
    reg [DATA_W-1:0] mem [0:WORDS-1];
    reg [DATA_W-1:0] mem_q;

    always @(posedge clk) begin
        if (mem_write) mem[laddr] <= wdata_des;
        if (mem_read)  mem_q      <= mem[laddr];
    end

    //----------------------------------------------------------------------
    // Output shift register.  Separate from mem_q so mem_q can stay a plain
    // register and be absorbed as the M9K output register - see the header.
    //----------------------------------------------------------------------
    reg [DATA_W-1:0] rd_sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              rd_sr <= {DATA_W{1'b0}};
        else if (rs == R_MEMQ)   rd_sr <= mem_q;
        else if (rs == R_SHIFT)  rd_sr <= {rd_sr[DATA_W-2:0], 1'b0};
    end

    // Quiet unless actually sending, so the shared wire stays readable in a
    // waveform and an idle bus does not look like traffic.
    assign dstream_out = (rs == R_SHIFT) ? rd_sr[DATA_W-1] : 1'b0;

endmodule
