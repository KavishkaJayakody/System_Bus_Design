// Serial address decoder: a progressive prefix matcher, not a comparator
// behind a deserialiser.  A one-hot marker walks the first PFX_W bits and one
// `alive' bit per target is cleared on a mismatch, so the answer is settled
// 11 clocks before the frame ends.  Prefixes are DERIVED from the bases in
// bus_defs.vh - do not hand-write range literals back in.

`include "bus_defs.vh"

module addr_decoder #(
    parameter ADDR_W   = `BUS_ADDR_W,
    parameter N_SLAVES = `BUS_N_SLAVES
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 frame,
    input  wire                 astream,
    input  wire                 en,
    output wire [N_SLAVES-1:0]  slv_sel,
    output wire                 def_sel,
    output wire                 hit
);

    localparam S0_PLEN = ADDR_W - `S0_LADDR_W;      // 5  (2K memory)
    localparam S1_PLEN = ADDR_W - `S1_LADDR_W;      // 4  (4K memory)
    localparam S2_PLEN = ADDR_W - `S2_LADDR_W;      // 4  (4K memory)
    localparam S3_PLEN = ADDR_W - `S3_LADDR_W;      // 2  (16K remote bridge)

    localparam MAX01   = (S0_PLEN > S1_PLEN) ? S0_PLEN : S1_PLEN;
    localparam MAX012  = (MAX01   > S2_PLEN) ? MAX01   : S2_PLEN;
    localparam PFX_W   = (MAX012  > S3_PLEN) ? MAX012  : S3_PLEN;   // 5

    localparam [PFX_W-1:0] PFX0 = `S0_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX1 = `S1_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX2 = `S2_BASE >> (ADDR_W - PFX_W);
    localparam [PFX_W-1:0] PFX3 = `S3_BASE >> (ADDR_W - PFX_W);

    localparam [PFX_W-1:0] CARE0 = ((1 << S0_PLEN) - 1) << (PFX_W - S0_PLEN);
    localparam [PFX_W-1:0] CARE1 = ((1 << S1_PLEN) - 1) << (PFX_W - S1_PLEN);
    localparam [PFX_W-1:0] CARE2 = ((1 << S2_PLEN) - 1) << (PFX_W - S2_PLEN);
    localparam [PFX_W-1:0] CARE3 = ((1 << S3_PLEN) - 1) << (PFX_W - S3_PLEN);

    localparam [N_SLAVES*PFX_W-1:0] PFX_FLAT  = {PFX3,  PFX2,  PFX1,  PFX0};
    localparam [N_SLAVES*PFX_W-1:0] CARE_FLAT = {CARE3, CARE2, CARE1, CARE0};

    localparam [PFX_W-1:0] POS_FIRST = 1 << (PFX_W - 1);

    reg [PFX_W-1:0]   pos;
    reg [N_SLAVES-1:0] alive;

    wire [N_SLAVES-1:0] miss;
    genvar gi;
    generate
        for (gi = 0; gi < N_SLAVES; gi = gi + 1) begin : g_match
            assign miss[gi] = |(pos
                              & CARE_FLAT[gi*PFX_W +: PFX_W]
                              & (PFX_FLAT[gi*PFX_W +: PFX_W]
                                 ^ {PFX_W{astream}}));
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos   <= POS_FIRST;
            alive <= {N_SLAVES{1'b1}};
        end else if (!frame) begin
            pos   <= POS_FIRST;
            alive <= {N_SLAVES{1'b1}};
        end else begin
            pos   <= pos >> 1;
            alive <= alive & ~miss;
        end
    end

    assign slv_sel = {N_SLAVES{en}} & alive;
    assign hit     = en &  (|alive);
    assign def_sel = en & ~(|alive);

endmodule
