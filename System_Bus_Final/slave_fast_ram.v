module slave_fast_ram #(
    parameter ADDR_WIDTH = 12 // kept for port compatibility (12 for S1, 11 for S2)
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  sel,
    input  wire                  we,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [7:0]            din,
    output reg  [7:0]            dout,
    output reg                   ready
);

    localparam IDX_BITS = 6; // 64 words

    (* ramstyle = "M9K" *) reg [7:0] mem [0:(1<<IDX_BITS)-1];

    // MSB 6 bits of whatever address width this instance was given
    wire [IDX_BITS-1:0] mem_idx = addr[ADDR_WIDTH-1 -: IDX_BITS];

    integer i;
    initial begin
        // Deterministic sim contents only — this is an 'initial' block,
        // not a reset, so it does NOT force LUT/FF-based memory.
        for (i = 0; i < (1 << IDX_BITS); i = i + 1)
            mem[i] = 8'h00;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout  <= 8'h00;
            ready <= 1'b0;
            // 'mem' is deliberately NOT reset here.
        end else begin
            ready <= 1'b0;

            if (sel) begin
                if (we) begin
                    mem[mem_idx] <= din;
                    dout         <= din;
                end else begin
                    dout <= mem[mem_idx];
                end
                ready <= 1'b1;
            end
        end
    end

endmodule