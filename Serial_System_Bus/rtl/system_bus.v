//==========================================================================
// system_bus.v
//
// THE BUS.  Arbitration, address decoding, the two shared serial wires and
// the responder for unmapped addresses - and nothing else.  No master, no
// memory.  Masters and slaves are instantiated OUTSIDE this module and reach
// it only through the serial interfaces below.
//
//   master[0] --+                                +-- slave[0]
//   master[1] --+---->  system_bus  <------------+-- slave[1]
//               |    bus_astream (1 wire)        +-- slave[2]
//               |    bus_dstream (1 wire)
//               +-- arbiter, addr_decoder, bus_mux, default responder
//
// TWO WIRES CARRY THE PAYLOAD
//
//   bus_astream   the address.  ONE wire.  Driven by the granted master,
//                 received by every slave and by the decoder's deserialiser.
//   bus_dstream   the data.  ONE wire, half duplex.  Driven by the granted
//                 master on a write and by the selected slave on a read, and
//                 received by everything.  It never needs a turnaround
//                 because the two directions cannot overlap: a write only
//                 ever sends during the frame, a read only ever after it.
//                 The direction control is therefore just bus_we.
//
// The per-endpoint `m_astream', `m_dstream' and `s_dstream' inputs are the
// CANDIDATES - one wire from each master or slave into the mux.  Exactly one
// of them reaches the shared wire, chosen by the grant or by the latched
// select.  There are no tristates anywhere: an FPGA has no internal tristate
// buffers, so a shared wire is a mux.
//
// WHAT IS AND IS NOT IN HERE
//   in:  arbiter (priority, bus lock, split mask), addr_decoder, bus_mux,
//        the central address deserialiser, and the default responder that
//        answers ERROR for unmapped addresses.
//   out: masters and memory slaves - they are peripherals, not bus.
//
// The default responder lives here on purpose.  It holds no state a
// peripheral would have; it exists so an unmapped address cannot leave the
// bus without a responder and hang it.  That is a property of the bus, not
// of anything hanging off it.
//
// One clock domain, one asynchronous active-low reset.
//
//--------------------------------------------------------------------------
// MASTER-SIDE INTERFACE (serial)
//--------------------------------------------------------------------------
// Port            Dir  Width               Meaning
//--------------------------------------------------------------------------
// m_req           in   N_MASTERS           Per-master bus request, held until
//                                          that master's transfer completes.
// m_gnt           out  N_MASTERS           One-hot grant, registered.
// m_valid         in   N_MASTERS           Per-master address-frame marker.
//                                          The granted master's is passed on
//                                          as bus_valid; the rest are ignored.
// m_we            in   N_MASTERS           Per-master direction, 1 = write.
// m_astream       in   N_MASTERS           ADDRESS candidate, ONE WIRE from
//                                          each master.
// m_dstream       in   N_MASTERS           WRITE DATA candidate, ONE WIRE
//                                          from each master.
// bus_ready       out  1                   Completion strobe, broadcast to
//                                          every master.
// bus_resp        out  RESP_W              Response with it: OKAY/ERROR/SPLIT.
// bus_dstream     out  1                   THE data wire.  Read data reaches
//                                          the masters on this, the same net
//                                          the slaves see.
//
//--------------------------------------------------------------------------
// SLAVE-SIDE INTERFACE (serial)
//--------------------------------------------------------------------------
// bus_valid       out  1                   Address-frame marker, broadcast.
//                                          High for ADDR_W clocks; every
//                                          slave shifts while it is high.
// bus_astream     out  1                   THE address wire, broadcast.
// bus_we          out  1                   Direction, broadcast and held
//                                          stable for the whole transaction.
// bus_master_id   out  ID_W                Tag of the granted master, so a
//                                          split-capable slave can remember
//                                          whose transfer it deferred.
// s_sel           out  N_SLAVES            One-hot select, one wire per
//                                          slave, pulsed for ONE cycle the
//                                          cycle after the frame ends.
// s_ready         in   N_SLAVES            Per-slave completion strobe.
// s_resp_flat     in   N_SLAVES*RESP_W     Per-slave response code.
// s_dstream       in   N_SLAVES            READ DATA candidate, ONE WIRE
//                                          from each slave.
// s_split_complete in  N_MASTERS           Wake-up pulses from the split-
//                                          capable slaves, already OR-ed
//                                          together by the integrator.
//
//--------------------------------------------------------------------------
// STATUS (for LEDs, SignalTap and the testbenches)
//--------------------------------------------------------------------------
// gnt_valid       out  1                   A master owns the bus.
// split_mask      out  N_MASTERS           Arbiter split mask.
// sel_q           out  N_SLAVES+1          Latched responder select,
//                                          {default, s2, s1, s0}.
// bus_addr        out  ADDR_W              DEBUG ONLY.  The address
//                                          reassembled off the wire, valid
//                                          from addr_done on, for the JTAG
//                                          probe.  Nothing in the datapath
//                                          reads it; zero if OBSERVE_ADDR=0.
// addr_done       out  1                   End-of-frame / decode strobe.
//==========================================================================
`include "bus_defs.vh"

module system_bus #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    parameter ID_W      = 1,                  // ceil(log2(N_MASTERS))
    parameter N_SLAVES  = `BUS_N_SLAVES,
    parameter ADDR_W    = `BUS_ADDR_W,
    parameter RESP_W    = `BUS_RESP_W,
    // Keep the 16-bit address reassembly that feeds the JTAG probe.  It is
    // observation only - the decoder is serial and does not use it - so 0
    // removes it and every parallel address with it.
    parameter OBSERVE_ADDR = 1
) (
    input  wire                            clk,
    input  wire                            rst_n,

    // ---- master side -----------------------------------------------------
    input  wire [N_MASTERS-1:0]            m_req,
    output wire [N_MASTERS-1:0]            m_gnt,
    input  wire [N_MASTERS-1:0]            m_valid,
    input  wire [N_MASTERS-1:0]            m_we,
    input  wire [N_MASTERS-1:0]            m_astream,
    input  wire [N_MASTERS-1:0]            m_dstream,
    output wire                            bus_ready,
    output wire [RESP_W-1:0]               bus_resp,

    // ---- slave side ------------------------------------------------------
    output wire                            bus_valid,
    output wire                            bus_we,
    output wire [ID_W-1:0]                 bus_master_id,
    output wire [N_SLAVES-1:0]             s_sel,
    input  wire [N_SLAVES-1:0]             s_ready,
    input  wire [N_SLAVES*RESP_W-1:0]      s_resp_flat,
    input  wire [N_SLAVES-1:0]             s_dstream,
    input  wire [N_MASTERS-1:0]            s_split_complete,

    // ---- THE TWO SHARED WIRES -------------------------------------------
    output wire                            bus_astream,
    output wire                            bus_dstream,

    // ---- status ----------------------------------------------------------
    output wire                            gnt_valid,
    output wire [N_MASTERS-1:0]            split_mask,
    output wire [N_SLAVES:0]               sel_q,
    output wire [ADDR_W-1:0]               bus_addr,
    output wire                            addr_done
);

    //----------------------------------------------------------------------
    // Arbiter.  Fixed priority, bus lock, split mask.  It never sees the
    // serial wires at all - only req, ready and resp - which is why it did
    // not change when the bus was serialised.
    //----------------------------------------------------------------------
    arbiter #(
        .N_MASTERS (N_MASTERS),
        .ID_W      (ID_W),
        .RESP_W    (RESP_W)
    ) u_arbiter (
        .clk            (clk),
        .rst_n          (rst_n),
        .req            (m_req),
        .bus_ready      (bus_ready),
        .bus_resp       (bus_resp),
        .split_complete (s_split_complete),
        .gnt            (m_gnt),
        .gnt_valid      (gnt_valid),
        .master_id      (bus_master_id),
        .split_mask     (split_mask),
        .locked         ()
    );

    //----------------------------------------------------------------------
    // Responder vectors.  The default responder occupies the top bit, so a
    // vector is {default, s2, s1, s0}.  Slaves supply the low bits from
    // outside; the default responder below supplies the top one.
    //----------------------------------------------------------------------
    wire                             def_sel;
    wire [N_SLAVES-1:0]              slv_sel;

    wire                             def_ready;
    wire [RESP_W-1:0]                def_resp;
    wire                             def_dstream;

    wire [N_SLAVES:0]                mux_sel     = {def_sel,     slv_sel};
    wire [N_SLAVES:0]                mux_ready   = {def_ready,   s_ready};
    wire [N_SLAVES:0]                mux_dstream = {def_dstream, s_dstream};
    wire [(N_SLAVES+1)*RESP_W-1:0]   mux_resp    = {def_resp,    s_resp_flat};

    assign s_sel = slv_sel;

    //----------------------------------------------------------------------
    // The data path: who drives the two shared wires.
    //----------------------------------------------------------------------
    bus_mux #(
        .N_MASTERS (N_MASTERS),
        .N_SLAVES  (N_SLAVES),
        .RESP_W    (RESP_W)
    ) u_bus_mux (
        .clk         (clk),
        .rst_n       (rst_n),
        .gnt         (m_gnt),
        .m_valid     (m_valid),
        .m_we        (m_we),
        .m_astream   (m_astream),
        .m_dstream   (m_dstream),
        .bus_valid   (bus_valid),
        .bus_we      (bus_we),
        .bus_astream (bus_astream),
        .bus_dstream (bus_dstream),
        .sel         (mux_sel),
        .s_ready     (mux_ready),
        .s_resp_flat (mux_resp),
        .s_dstream   (mux_dstream),
        .bus_ready   (bus_ready),
        .bus_resp    (bus_resp),
        .sel_q       (sel_q)
    );

    //----------------------------------------------------------------------
    // End-of-frame strobe.
    //----------------------------------------------------------------------
    reg bus_valid_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bus_valid_d <= 1'b0;
        else        bus_valid_d <= bus_valid;
    end

    // Falling edge of the frame: the address is complete this cycle.
    // Deriving it here rather than adding an "end of frame" wire keeps the
    // shared bus at eight wires and leaves the frame length defined in
    // exactly one place - the master's counter.
    assign addr_done = bus_valid_d & ~bus_valid;

    //----------------------------------------------------------------------
    // Address decoder.  SERIAL: it watches the address arrive bit by bit on
    // the shared wire and narrows which slave can still match, so no 16-bit
    // address is ever assembled in the datapath.  It settles after the
    // 5-bit prefix - eleven clocks before the frame ends - and `addr_done'
    // merely strobes the answer out.
    //
    // Nothing here is broadcast back out in parallel form: slaves collect
    // their own low offset bits off the same single wire.
    //----------------------------------------------------------------------
    addr_decoder #(
        .ADDR_W   (ADDR_W),
        .N_SLAVES (N_SLAVES)
    ) u_decoder (
        .clk     (clk),
        .rst_n   (rst_n),
        .frame   (bus_valid),
        .astream (bus_astream),
        .en      (addr_done),
        .slv_sel (slv_sel),
        .def_sel (def_sel),
        .hit     ()
    );

    //----------------------------------------------------------------------
    // Address observation for the JTAG probe - DEBUG ONLY.
    //
    // Nothing in the datapath reads this.  It exists so a host can compare
    // the address the bus reassembled off the single wire with the one it
    // sent, which is the first thing worth looking at when serial framing
    // misbehaves on silicon.  Set OBSERVE_ADDR = 0 and it disappears, along
    // with the last parallel address anywhere in the design; the bus works
    // identically without it and only the probe goes dark.
    //----------------------------------------------------------------------
    generate
        if (OBSERVE_ADDR) begin : g_obs_addr
            shift_deser #(.W(ADDR_W)) u_addr_deser (
                .clk(clk), .rst_n(rst_n),
                .shift(bus_valid), .din(bus_astream), .dout(bus_addr)
            );
        end else begin : g_no_obs_addr
            assign bus_addr = {ADDR_W{1'b0}};
        end
    endgenerate

    //----------------------------------------------------------------------
    // Default responder - the reason an unmapped address cannot hang the bus.
    //----------------------------------------------------------------------
    default_slave #(
        .RESP_W (RESP_W)
    ) u_default (
        .clk         (clk),
        .rst_n       (rst_n),
        .sel         (def_sel),
        .dstream_out (def_dstream),
        .ready       (def_ready),
        .resp        (def_resp)
    );

endmodule
