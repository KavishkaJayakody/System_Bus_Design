//==========================================================================
// bus_mux.v
//
// The two data-path multiplexers of the shared bus.
//
//  * Forward  (master -> slave):  combinational, selected by the arbiter's
//    one-hot grant.  Only the granted master's address/data/control reach
//    the slaves; every other master is ignored, which is what makes this a
//    single-master-at-a-time shared bus rather than a crossbar.
//
//  * Return   (slave -> master):  selected by a REGISTERED copy of the
//    decoder select.  The slaves use synchronous read, so a slave that was
//    addressed in cycle T returns its data and its ready/response in T+1.
//    Selecting the return path with sel_q (the T select, delayed by one
//    cycle) lines the mux up with the data.  Using the live select here
//    would return the wrong slave's data whenever the address changed.
//
// The returned {ready, resp, rdata} is broadcast to all masters; each master
// only looks at it while it holds the grant, so no return-path arbitration
// is needed.
//
// The one-hot select vectors are {default, slave2, slave1, slave0}, i.e. the
// default slave sits in the top bit (index N_SLAVES).
//
//--------------------------------------------------------------------------
// Port            Dir  Width               Meaning
//--------------------------------------------------------------------------
// clk             in   1                   Bus clock.
// rst_n           in   1                   Asynchronous active-low reset.
// gnt             in   N_MASTERS           One-hot grant from the arbiter.
// m_valid         in   N_MASTERS           Per-master transfer strobe.
// m_we            in   N_MASTERS           Per-master write enable.
// m_addr_flat     in   N_MASTERS*ADDR_W    Per-master address, master i in
//                                          bits [i*ADDR_W +: ADDR_W].
// m_wdata_flat    in   N_MASTERS*DATA_W    Per-master write data, same
//                                          packing.
// bus_valid       out  1                   Transfer strobe of the granted
//                                          master, one cycle per transfer.
// bus_we          out  1                   1 = write, 0 = read.
// bus_addr        out  ADDR_W              Address of the granted master.
// bus_wdata       out  DATA_W              Write data of the granted master.
// sel             in   N_SLAVES+1          Live one-hot select from the
//                                          address decoder, already gated
//                                          with bus_valid by the top level.
// s_ready         in   N_SLAVES+1          Per-slave completion strobe.
// s_resp_flat     in   (N_SLAVES+1)*RESP_W Per-slave response code.
// s_rdata_flat    in   (N_SLAVES+1)*DATA_W Per-slave read data.
// bus_ready       out  1                   Completion strobe returned to the
//                                          masters and the arbiter.
// bus_resp        out  RESP_W              Response returned with it.
// bus_rdata       out  DATA_W              Read data returned with it.
// sel_q           out  N_SLAVES+1          The registered select, exported
//                                          so the top level can light an LED
//                                          with the responding slave and so
//                                          the testbench can check it.
//==========================================================================
`include "bus_defs.vh"

module bus_mux #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    parameter N_SLAVES  = `BUS_N_SLAVES,
    parameter ADDR_W    = `BUS_ADDR_W,
    parameter DATA_W    = `BUS_DATA_W,
    parameter RESP_W    = `BUS_RESP_W
) (
    input  wire                              clk,
    input  wire                              rst_n,

    // ---- forward path -------------------------------------------------
    input  wire [N_MASTERS-1:0]              gnt,
    input  wire [N_MASTERS-1:0]              m_valid,
    input  wire [N_MASTERS-1:0]              m_we,
    input  wire [N_MASTERS*ADDR_W-1:0]       m_addr_flat,
    input  wire [N_MASTERS*DATA_W-1:0]       m_wdata_flat,
    output reg                               bus_valid,
    output reg                               bus_we,
    output reg  [ADDR_W-1:0]                 bus_addr,
    output reg  [DATA_W-1:0]                 bus_wdata,

    // ---- return path --------------------------------------------------
    input  wire [N_SLAVES:0]                 sel,
    input  wire [N_SLAVES:0]                 s_ready,
    input  wire [(N_SLAVES+1)*RESP_W-1:0]    s_resp_flat,
    input  wire [(N_SLAVES+1)*DATA_W-1:0]    s_rdata_flat,
    output reg                               bus_ready,
    output reg  [RESP_W-1:0]                 bus_resp,
    output reg  [DATA_W-1:0]                 bus_rdata,
    output reg  [N_SLAVES:0]                 sel_q
);

    integer i;

    //----------------------------------------------------------------------
    // Forward mux.  Defaults first, so no latch is inferred when no master
    // holds the grant.
    //----------------------------------------------------------------------
    always @* begin
        bus_valid = 1'b0;
        bus_we    = 1'b0;
        bus_addr  = {ADDR_W{1'b0}};
        bus_wdata = {DATA_W{1'b0}};
        for (i = 0; i < N_MASTERS; i = i + 1) begin
            if (gnt[i]) begin
                bus_valid = m_valid[i];
                bus_we    = m_we[i];
                bus_addr  = m_addr_flat[i*ADDR_W +: ADDR_W];
                bus_wdata = m_wdata_flat[i*DATA_W +: DATA_W];
            end
        end
    end

    //----------------------------------------------------------------------
    // One-cycle delayed copy of the decoder select, used to steer the
    // return path onto the slave that was addressed last cycle.
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sel_q <= {(N_SLAVES+1){1'b0}};
        else        sel_q <= sel;
    end

    //----------------------------------------------------------------------
    // Return mux.
    //----------------------------------------------------------------------
    always @* begin
        bus_ready = 1'b0;
        bus_resp  = `RESP_OKAY;
        bus_rdata = {DATA_W{1'b0}};
        for (i = 0; i <= N_SLAVES; i = i + 1) begin
            if (sel_q[i]) begin
                bus_ready = s_ready[i];
                bus_resp  = s_resp_flat[i*RESP_W +: RESP_W];
                bus_rdata = s_rdata_flat[i*DATA_W +: DATA_W];
            end
        end
    end

endmodule
