// The datapath: who drives the two shared serial wires.  An FPGA has no
// internal tristates, so a shared wire is a mux.

`include "bus_defs.vh"

module bus_mux #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    parameter N_SLAVES  = `BUS_N_SLAVES,
    parameter RESP_W    = `BUS_RESP_W
) (
    input  wire                              clk,
    input  wire                              rst_n,

    input  wire [N_MASTERS-1:0]              gnt,
    input  wire [N_MASTERS-1:0]              m_valid,
    input  wire [N_MASTERS-1:0]              m_we,
    input  wire [N_MASTERS-1:0]              m_astream,
    input  wire [N_MASTERS-1:0]              m_dstream,
    output reg                               bus_valid,
    output reg                               bus_we,
    output reg                               bus_astream,
    output wire                              bus_dstream,

    input  wire [N_SLAVES:0]                 sel,
    input  wire [N_SLAVES:0]                 s_ready,
    input  wire [(N_SLAVES+1)*RESP_W-1:0]    s_resp_flat,
    input  wire [N_SLAVES:0]                 s_dstream,
    output reg                               bus_ready,
    output reg  [RESP_W-1:0]                 bus_resp,
    output reg  [N_SLAVES:0]                 sel_q
);

    integer i;

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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)   sel_q <= {(N_SLAVES+1){1'b0}};
        else if (|sel) sel_q <= sel;
    end

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

    assign bus_dstream = bus_we ? m_dstream_sel : s_dstream_sel;

endmodule
