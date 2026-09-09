// THE BUS: arbitration, serial address decode, the two shared wires and the
// responder for unmapped addresses.  No master and no memory in here - they
// are instantiated outside and reach it only through the serial ports.

`include "bus_defs.vh"

module system_bus #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    parameter ID_W      = `BUS_ID_W,                  // ceil(log2(N_MASTERS))
    parameter N_SLAVES  = `BUS_N_SLAVES,
    parameter ADDR_W    = `BUS_ADDR_W,
    parameter RESP_W    = `BUS_RESP_W,
    parameter OBSERVE_ADDR = 1
) (
    input  wire                            clk,
    input  wire                            rst_n,

    input  wire [N_MASTERS-1:0]            m_req,
    output wire [N_MASTERS-1:0]            m_gnt,
    input  wire [N_MASTERS-1:0]            m_valid,
    input  wire [N_MASTERS-1:0]            m_we,
    input  wire [N_MASTERS-1:0]            m_astream,
    input  wire [N_MASTERS-1:0]            m_dstream,
    output wire                            bus_ready,
    output wire [RESP_W-1:0]               bus_resp,

    output wire                            bus_valid,
    output wire                            bus_we,
    output wire [ID_W-1:0]                 bus_master_id,
    output wire [N_SLAVES-1:0]             s_sel,
    input  wire [N_SLAVES-1:0]             s_ready,
    input  wire [N_SLAVES*RESP_W-1:0]      s_resp_flat,
    input  wire [N_SLAVES-1:0]             s_dstream,
    input  wire [N_MASTERS-1:0]            s_split_complete,

    output wire                            bus_astream,
    output wire                            bus_dstream,

    output wire                            gnt_valid,
    output wire [N_MASTERS-1:0]            split_mask,
    output wire [N_SLAVES:0]               sel_q,
    output wire [ADDR_W-1:0]               bus_addr,
    output wire                            addr_done
);

    // The arbiter never sees the serial wires - only req, ready and resp.
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

    reg bus_valid_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bus_valid_d <= 1'b0;
        else        bus_valid_d <= bus_valid;
    end

    assign addr_done = bus_valid_d & ~bus_valid;

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

    // Address observation for the JTAG probe - DEBUG ONLY.
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
