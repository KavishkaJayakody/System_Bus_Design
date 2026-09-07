//==========================================================================
// master_prog.v
//
// Synthesisable scenario sequencer: the thing that gives a master something
// to do on the board.  One instance per master; MID selects which half of
// the program table this instance runs.
//
// It is deliberately NOT part of bus_top - the testbenches drive the same
// command interface directly, so what runs on the board and what runs in
// simulation exercise identical RTL.
//
// Each scenario is a 4-step program that loops forever.  A step is issued
// only when a run request arrives (continuous run, a slow tick, or a single
// KEY press), so the whole thing can be watched one transaction at a time.
//
// Program table (addresses chosen to hit specific slaves):
//
//   sc  master 0                          master 1
//   --  --------------------------------  --------------------------------
//   0   W/R 0x1234 (s1), W/R 0x2567 (s2)  idle - the "one master" scenario
//   1   W/R 0x1ABC, W/R 0x1DEF   (s1)     W/R 0x2345, W/R 0x2678   (s2)
//   2   W/R 0x0A5C, W/R 0x0369   (s0)     W/R 0x21A7, W/R 0x24E3   (s2)
//   3   R 0x2ABC (hole), R 0x1234 (ok),   R 0x2345 x4  (s2)
//       R 0x8DEF (reserved), R 0x2567
//
//   The addresses deliberately use varied nibbles so every seven-segment
//   digit actually changes; with round numbers most segments sit stuck and
//   the fitter (correctly) reports the pins as tied.
//
//   Scenario 1 is the contention demo, 2 is the split demo (turn slave 0's
//   split on and master 1 keeps working through master 0's stall), and 3 is
//   the unmapped-address recovery demo - two ERROR responses interleaved
//   with two good transfers that must still succeed.
//
// Write data is {tag, pass_count} where tag identifies the master.  The pass
// counter increments once per completed loop, so the value read back and
// shown on the display changes every pass and a stalled bus is obvious.
//
//--------------------------------------------------------------------------
// Port         Dir  Width   Meaning
//--------------------------------------------------------------------------
// clk          in   1       Bus clock.
// rst_n        in   1       Asynchronous active-low reset.
// run_req      in   1       Run request.  Held high = run back to back; a
//                           one-clock pulse = run exactly one more step.
//                           Latched internally, so a single-cycle tick is
//                           never missed.
// scenario     in   2       Which program to run, from SW[1:0].
// cmd_valid    out  1       To master: a command is offered.
// cmd_we       out  1       To master: 1 = write.
// cmd_addr     out  ADDR_W  To master: full bus address.
// cmd_wdata    out  DATA_W  To master: write data.
// cmd_accept   in   1       From master: command latched.
// done         in   1       From master: transaction finished.
// step         out  2       Index of the step being run (display/debug).
// pass_count   out  16      Completed loops of the program.
// xact_count   out  16      Completed transactions.
//==========================================================================

module master_prog #(
    parameter MID    = 0,
    parameter ADDR_W = 16,
    parameter DATA_W = 32       // must be >= 32; the pattern uses bits 31:0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               run_req,
    input  wire [1:0]         scenario,
    output reg                cmd_valid,
    output reg                cmd_we,
    output reg [ADDR_W-1:0]   cmd_addr,
    output reg [DATA_W-1:0]   cmd_wdata,
    input  wire               cmd_accept,
    input  wire               done,
    output reg [1:0]          step,
    output reg [15:0]         pass_count,
    output reg [15:0]         xact_count
);

    localparam S_IDLE  = 2'd0;
    localparam S_ISSUE = 2'd1;
    localparam S_WAIT  = 2'd2;

    reg [1:0] st;
    reg       pending;

    //----------------------------------------------------------------------
    // Program lookup.  MID is a parameter, so only one half of this table
    // survives in each instance.  Defaults first: no latch.
    //----------------------------------------------------------------------
    reg               p_active;
    reg               p_we;
    reg [ADDR_W-1:0]  p_addr;

    always @* begin
        p_active = 1'b1;
        p_we     = 1'b0;
        p_addr   = {ADDR_W{1'b0}};

        if (MID == 0) begin
            case ({scenario, step})
                // scenario 0 - one master alone, slaves 1 and 2
                4'b00_00: begin p_we = 1'b1; p_addr = 16'h1234; end
                4'b00_01: begin p_we = 1'b0; p_addr = 16'h1234; end
                4'b00_10: begin p_we = 1'b1; p_addr = 16'h2567; end
                4'b00_11: begin p_we = 1'b0; p_addr = 16'h2567; end
                // scenario 1 - contention: master 0 works slave 1
                4'b01_00: begin p_we = 1'b1; p_addr = 16'h1ABC; end
                4'b01_01: begin p_we = 1'b0; p_addr = 16'h1ABC; end
                4'b01_10: begin p_we = 1'b1; p_addr = 16'h1DEF; end
                4'b01_11: begin p_we = 1'b0; p_addr = 16'h1DEF; end
                // scenario 2 - split: master 0 works the split-capable slave
                4'b10_00: begin p_we = 1'b1; p_addr = 16'h0A5C; end
                4'b10_01: begin p_we = 1'b0; p_addr = 16'h0A5C; end
                4'b10_10: begin p_we = 1'b1; p_addr = 16'h0369; end
                4'b10_11: begin p_we = 1'b0; p_addr = 16'h0369; end
                // scenario 3 - unmapped recovery: bad, good, bad, good
                4'b11_00: begin p_we = 1'b0; p_addr = 16'h2ABC; end  // hole
                4'b11_01: begin p_we = 1'b0; p_addr = 16'h1234; end  // ok
                4'b11_10: begin p_we = 1'b0; p_addr = 16'h8DEF; end  // reserved
                4'b11_11: begin p_we = 1'b0; p_addr = 16'h2567; end  // ok
                default:  begin p_active = 1'b0; end
            endcase
        end else begin
            case ({scenario, step})
                // scenario 0 - master 1 stays off the bus entirely
                4'b00_00,
                4'b00_01,
                4'b00_10,
                4'b00_11: begin p_active = 1'b0; end
                // scenario 1 - contention: master 1 works slave 2
                4'b01_00: begin p_we = 1'b1; p_addr = 16'h2345; end
                4'b01_01: begin p_we = 1'b0; p_addr = 16'h2345; end
                4'b01_10: begin p_we = 1'b1; p_addr = 16'h2678; end
                4'b01_11: begin p_we = 1'b0; p_addr = 16'h2678; end
                // scenario 2 - master 1 keeps working through master 0's split
                4'b10_00: begin p_we = 1'b1; p_addr = 16'h21A7; end
                4'b10_01: begin p_we = 1'b0; p_addr = 16'h21A7; end
                4'b10_10: begin p_we = 1'b1; p_addr = 16'h24E3; end
                4'b10_11: begin p_we = 1'b0; p_addr = 16'h24E3; end
                // scenario 3 - master 1 just keeps reading a good address
                4'b11_00,
                4'b11_01,
                4'b11_10,
                4'b11_11: begin p_we = 1'b0; p_addr = 16'h2345; end
                default:  begin p_active = 1'b0; end
            endcase
        end
    end

    //----------------------------------------------------------------------
    // Write pattern.  Low half = pass_count, high half = a per-master tag
    // XORed with the transaction count.
    //
    // The XOR is not decoration: with a plain constant in the high half the
    // two byte lanes carried identical values, the fitter spotted it and
    // merged them, and the memories were built 24 bits wide instead of 32.
    // Every bit of the pattern has to vary independently for the whole
    // datapath to actually be implemented.
    //----------------------------------------------------------------------
    wire [15:0] tag = (MID != 0) ? 16'hB1B1 : 16'hA0A0;

    reg [DATA_W-1:0] pattern;
    always @* begin
        pattern         = {DATA_W{1'b0}};
        pattern[15:0]   = pass_count;
        pattern[31:16]  = tag ^ xact_count;
    end

    //----------------------------------------------------------------------
    // Sequencer
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            step       <= 2'd0;
            pending    <= 1'b0;
            cmd_valid  <= 1'b0;
            cmd_we     <= 1'b0;
            cmd_addr   <= {ADDR_W{1'b0}};
            cmd_wdata  <= {DATA_W{1'b0}};
            pass_count <= 16'd0;
            xact_count <= 16'd0;
        end else begin
            // Latch the run request so a one-clock tick is never missed.
            if (run_req)
                pending <= 1'b1;

            case (st)
                S_IDLE: begin
                    if (pending) begin
                        if (p_active) begin
                            cmd_valid <= 1'b1;
                            cmd_we    <= p_we;
                            cmd_addr  <= p_addr;
                            cmd_wdata <= pattern;
                            st        <= S_ISSUE;
                        end else begin
                            // This master does nothing in this step: skip it
                            // so the two masters stay in step with each other.
                            step    <= step + 2'd1;
                            pending <= 1'b0;
                            if (step == 2'd3)
                                pass_count <= pass_count + 16'd1;
                        end
                    end
                end

                S_ISSUE: begin
                    if (cmd_accept) begin
                        cmd_valid <= 1'b0;
                        st        <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (done) begin
                        st         <= S_IDLE;
                        pending    <= 1'b0;
                        step       <= step + 2'd1;
                        xact_count <= xact_count + 16'd1;
                        if (step == 2'd3)
                            pass_count <= pass_count + 16'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
