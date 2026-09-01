`timescale 1ns / 1ps

module slave_fast_ram #(
    parameter ADDR_WIDTH = 12 // 12 for 4K (S1), 11 for 2K (S2)
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

    reg [7:0] mem [(1 << ADDR_WIDTH)-1:0];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < (1 << ADDR_WIDTH); i = i + 1)
                mem[i] <= 8'h00;
            dout  <= 8'h00;
            ready <= 1'b0;
        end else begin
            ready <= 1'b0;

            if (sel) begin
                if (we) begin
                    mem[addr] <= din;
                    dout      <= din;   // immediately reflect newly written value
                end else begin
                    dout <= mem[addr];  // normal read path
                end
                ready <= 1'b1;
            end
        end
    end

endmodule