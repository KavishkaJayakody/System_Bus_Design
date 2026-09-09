// Serial in, parallel out, MSB first.  Holds THE LAST W BITS SEEN - no
// counter, no framing.  That is what lets each receiver pick its own width.

module shift_deser #(
    parameter W = 8
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         shift,
    input  wire         din,
    output wire [W-1:0] dout
);

    reg [W-1:0] sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     sr <= {W{1'b0}};
        else if (shift) sr <= {sr[W-2:0], din};
    end

    assign dout = sr;

endmodule
