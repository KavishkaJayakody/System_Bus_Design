//==========================================================================
// arbiter.v
//
// Fixed-priority bus arbiter with transfer lock and an AHB-style split mask.
//
// Parameterised for N_MASTERS requesters.  It is instantiated with 2 today;
// phase 2 adds the remote bridge as a third requester and only N_MASTERS /
// ID_W change.  Nothing below assumes two masters.
//
// Priority is by index: master 0 is highest, master N-1 lowest.
//
// Lock:  once a master is granted the grant is held until that transfer
//        completes (bus_ready), so a transfer is never torn in half.  The
//        arbiter re-arbitrates on the cycle after completion, which is what
//        stops master 0 from owning the bus across a whole burst.
//
// Split: if the completing response is SPLIT the arbiter sets mask[id] for
//        the granted master and releases the bus.  A masked master is removed
//        from arbitration entirely, so the other masters run at full speed
//        while it waits.  When the slave pulses split_complete[id] the mask
//        bit clears and that master re-arbitrates and re-issues its transfer.
//
// The masked master keeps its `req' asserted the whole time; masking, not
// request withdrawal, is what removes it from the contest.  That keeps the
// replay decision inside the master FSM where it belongs.
//
//--------------------------------------------------------------------------
// Port            Dir  Width      Meaning
//--------------------------------------------------------------------------
// clk             in   1          Bus clock, single domain.
// rst_n           in   1          Asynchronous active-low reset.
// req             in   N_MASTERS  Per-master bus request, active high, held
//                                 until the master's transfer completes.
// bus_ready       in   1          Completion strobe from the selected slave,
//                                 returned through bus_mux.  Qualified here
//                                 by `locked', so a stray ready while idle is
//                                 ignored.
// bus_resp        in   RESP_W     Response accompanying bus_ready
//                                 (OKAY / ERROR / SPLIT).
// split_complete  in   N_MASTERS  Per-master pulse from a split-capable slave
//                                 saying that master's deferred data is ready.
//                                 One cycle wide.
// gnt             out  N_MASTERS  One-hot grant, registered.  All zero when
//                                 no master owns the bus.
// gnt_valid       out  1          |gnt - a master currently owns the bus.
// master_id       out  ID_W       Index of the granted master.  Driven onto
//                                 the bus as the master tag so a split-capable
//                                 slave can remember whose transfer it
//                                 deferred.  Only meaningful when gnt_valid.
// split_mask      out  N_MASTERS  Current mask register, exported for LEDs
//                                 and for the testbench to check directly.
// locked          out  1          A transfer is in progress on the bus.
//==========================================================================
`include "bus_defs.vh"

module arbiter #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    // ID_W must equal ceil(log2(N_MASTERS)).  It is a parameter rather than a
    // $clog2 call to stay inside Verilog-2001.  N_MASTERS=3 => ID_W=2.
    parameter ID_W      = `BUS_ID_W,
    parameter RESP_W    = `BUS_RESP_W
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [N_MASTERS-1:0]    req,
    input  wire                    bus_ready,
    input  wire [RESP_W-1:0]       bus_resp,
    input  wire [N_MASTERS-1:0]    split_complete,
    output reg  [N_MASTERS-1:0]    gnt,
    output wire                    gnt_valid,
    output reg  [ID_W-1:0]         master_id,
    output reg  [N_MASTERS-1:0]    split_mask,
    output reg                     locked
);

    integer i;

    //----------------------------------------------------------------------
    // Eligibility: requesting AND not currently split-masked.
    //----------------------------------------------------------------------
    wire [N_MASTERS-1:0] elig = req & ~split_mask;

    //----------------------------------------------------------------------
    // Fixed-priority pick, lowest index wins.  The loop walks from the
    // lowest-priority index downwards so the last (highest priority) hit is
    // the one that survives.  Every output is given a default first, so this
    // block cannot infer a latch.
    //----------------------------------------------------------------------
    reg                  pick_val;
    reg [ID_W-1:0]       pick_id;
    reg [N_MASTERS-1:0]  pick_onehot;

    always @* begin
        pick_val    = 1'b0;
        pick_id     = {ID_W{1'b0}};
        pick_onehot = {N_MASTERS{1'b0}};
        for (i = N_MASTERS-1; i >= 0; i = i - 1) begin
            if (elig[i]) begin
                pick_val       = 1'b1;
                pick_id        = i[ID_W-1:0];
                pick_onehot    = {N_MASTERS{1'b0}};
                pick_onehot[i] = 1'b1;
            end
        end
    end

    //----------------------------------------------------------------------
    // A completion belongs to the arbiter only while a grant is outstanding.
    //----------------------------------------------------------------------
    wire xfer_done  = locked & bus_ready;
    wire xfer_split = xfer_done & (bus_resp == `RESP_SPLIT);

    //----------------------------------------------------------------------
    // Grant / lock / mask registers.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gnt        <= {N_MASTERS{1'b0}};
            master_id  <= {ID_W{1'b0}};
            split_mask <= {N_MASTERS{1'b0}};
            locked     <= 1'b0;
        end else begin
            //--------------------------------------------------------------
            // Split mask, one bit per master.  Set beats clear on the same
            // bit; that combination cannot occur in practice because a slave
            // only pulses split_complete at least one cycle after it issued
            // the SPLIT response that sets the bit.
            //
            // The master being deferred is identified by `gnt', the one-hot
            // grant, NOT by comparing master_id against the loop index.  That
            // comparison used to read `master_id == i[ID_W-1:0]', which
            // TRUNCATES the index: with ID_W too small for N_MASTERS, i=2
            // narrows to 0 and splitting master 0 silently masks master 2 as
            // well, permanently - it is never granted again and its requests
            // simply vanish.  gnt is one-hot by construction and cannot
            // alias, so the mask is now correct for any N_MASTERS/ID_W pair.
            //--------------------------------------------------------------
            for (i = 0; i < N_MASTERS; i = i + 1) begin
                if (xfer_split && gnt[i])
                    split_mask[i] <= 1'b1;
                else if (split_complete[i])
                    split_mask[i] <= 1'b0;
            end

            //--------------------------------------------------------------
            // Grant.  Hold while locked; release the cycle the transfer
            // completes and re-arbitrate on the cycle after that.
            //--------------------------------------------------------------
            if (!locked) begin
                if (pick_val) begin
                    gnt       <= pick_onehot;
                    master_id <= pick_id;
                    locked    <= 1'b1;
                end else begin
                    gnt <= {N_MASTERS{1'b0}};
                end
            end else if (bus_ready) begin
                gnt    <= {N_MASTERS{1'b0}};
                locked <= 1'b0;
            end
        end
    end

    assign gnt_valid = |gnt;

endmodule
