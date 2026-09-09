// Parallel in, serial out, MSB first.  Shifts zeros in behind the data, so a
// short field sits inside a longer frame with no bit counter at either end.

module shift_ser #(
    parameter W = 8
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         load,
    input  wire         shift,
    input  wire [W-1:0] din,
    output wire         dout
);

    reg [W-1:0] sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     sr <= {W{1'b0}};
        else if (load)  sr <= din;
        else if (shift) sr <= {sr[W-2:0], 1'b0};   // zeros in behind
    end

    assign dout = sr[W-1];

endmodule
