//==========================================================================
// bus_mux.v
//
// The data path of the serial shared bus: two one-bit wires and the small
// amount of muxing that decides who is driving them.
//
// FPGAs have no internal tristate buffers, so a "shared wire" is a mux, not
// a wired-OR.  These are genuinely single nets though - every master and
// every slave taps the same bus_astream and the same bus_dstream.
//
//   bus_astream  driven by the GRANTED master, always.
//                Received by all three slaves and by the central address
//                deserialiser that feeds the decoder.
//
//   bus_dstream  half duplex, and it never needs a turnaround because the
//                two directions cannot collide by construction:
//                  write -> only the master sends (during the frame)
//                  read  -> only the slave sends (after the frame)
//                so the direction mux is just bus_we.
//
// THE RETURN SELECT IS LATCHED, NOT DELAYED BY ONE CYCLE.  On the parallel
// bus a reply always came back exactly one cycle after the address, so a
// single register was enough.  Here a write answers in 1 cycle, a split in
// 1, and a read in 10 - the reply is no longer at a fixed offset.  So the
// select is captured when the decoder pulses it and HELD until the next
// transfer selects something else.  Only one transfer is ever outstanding
// (the arbiter locks the bus), so holding is safe.
//
//--------------------------------------------------------------------------
// Port            Dir  Width               Meaning
//--------------------------------------------------------------------------
// clk             in   1                   Bus clock.
// rst_n           in   1                   Asynchronous active-low reset.
// gnt             in   N_MASTERS           One-hot grant from the arbiter.
// m_valid         in   N_MASTERS           Per-master frame marker.
// m_we            in   N_MASTERS           Per-master write enable.
// m_astream       in   N_MASTERS           Per-master serial address out.
// m_dstream       in   N_MASTERS           Per-master serial write data out.
// bus_valid       out  1                   Frame marker of the granted
//                                          master.  HIGH FOR ADDR_W CLOCKS.
// bus_we          out  1                   1 = write.  Also the direction
//                                          control for the data wire.
// bus_astream     out  1                   THE shared address wire.
// bus_dstream     out  1                   THE shared data wire.
// sel             in   N_SLAVES+1          One-cycle one-hot select from the
//                                          decoder, {default,s2,s1,s0}.
// s_ready         in   N_SLAVES+1          Per-slave completion strobe.
// s_resp_flat     in   (N_SLAVES+1)*RESP_W Per-slave response code.
// s_dstream       in   N_SLAVES+1          Per-slave serial read data out.
// bus_ready       out  1                   Completion strobe to the masters
//                                          and the arbiter.
// bus_resp        out  RESP_W              Response returned with it.
// sel_q           out  N_SLAVES+1          The latched responder select,
//                                          exported for the LEDs and the
//                                          testbenches.
//==========================================================================
`include "bus_defs.vh"

module bus_mux #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    parameter N_SLAVES  = `BUS_N_SLAVES,
    parameter RESP_W    = `BUS_RESP_W
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // ---- forward path -------------------------------------------------
    input  wire [N_MASTERS-1:0]              gnt,
    input  wire [N_MASTERS-1:0]              m_valid,
    input  wire [N_MASTERS-1:0]              m_we,
    input  wire [N_MASTERS-1:0]              m_astream,
    input  wire [N_MASTERS-1:0]              m_dstream,
    output reg                               bus_valid,
    output reg                               bus_we,
    output reg                               bus_astream,
    output wire                              bus_dstream,

    // ---- return path --------------------------------------------------
    input  wire [N_SLAVES:0]                 sel,
    input  wire [N_SLAVES:0]                 s_ready,
    input  wire [(N_SLAVES+1)*RESP_W-1:0]    s_resp_flat,
    input  wire [N_SLAVES:0]                 s_dstream,
    output reg                               bus_ready,
    output reg  [RESP_W-1:0]                 bus_resp,
    output reg  [N_SLAVES:0]                 sel_q
);

    integer i;

    //----------------------------------------------------------------------
    // Forward mux: the granted master owns the address wire and the control
    // lines.  Defaults first, so no latch is inferred when nobody is granted.
    //----------------------------------------------------------------------
    reg m_dstream_sel;

    always @* begin
        bus_valid     = 1'b0;
        bus_we        = 1'b0;
        bus_astream   = 1'b0;
        m_dstream_sel = 1'b0;
        for (i = 0; i < N_MASTERS; i = i + 1) begin
            if (gnt[i]) begin
                bus_valid     = m_valid[i];
                bus_we        = m_we[i];
                bus_astream   = m_astream[i];
                m_dstream_sel = m_dstream[i];
            end
        end
    end

    //----------------------------------------------------------------------
    // Latched responder select.  Captured on the decoder's one-cycle pulse
    // and held for however long that responder takes to answer.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)   sel_q <= {(N_SLAVES+1){1'b0}};
        else if (|sel) sel_q <= sel;
    end

    //----------------------------------------------------------------------
    // Return mux: the selected slave's completion and response.
    //----------------------------------------------------------------------
    reg s_dstream_sel;

    always @* begin
        bus_ready     = 1'b0;
        bus_resp      = `RESP_OKAY;
        s_dstream_sel = 1'b0;
        for (i = 0; i <= N_SLAVES; i = i + 1) begin
            if (sel_q[i]) begin
                bus_ready     = s_ready[i];
                bus_resp      = s_resp_flat[i*RESP_W +: RESP_W];
                s_dstream_sel = s_dstream[i];
            end
        end
    end

    //----------------------------------------------------------------------
    // The one shared data wire.  Direction is simply bus_we: on a write only
    // the master ever sends, on a read only the slave ever does.
    //----------------------------------------------------------------------
    assign bus_dstream = bus_we ? m_dstream_sel : s_dstream_sel;

endmodule
