`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/31/2026 08:53:32 PM
// Design Name: 
// Module Name: ram_4k
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

module ram_4k #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 12 // 2^12 = 4096 depth (4K entries)
)(
    input  wire                   clk,
    input  wire                   we,      // Write Enable: 1 = Write, 0 = Read
    input  wire [ADDR_WIDTH-1:0]  addr,    // Address bus (0 to 4095)
    input  wire [DATA_WIDTH-1:0]  din,     // Data input
    output reg  [DATA_WIDTH-1:0]  dout     // Data output (synchronous read)
);

    // 4096-entry memory array
    reg [DATA_WIDTH-1:0] mem [(1 << ADDR_WIDTH)-1:0];

    // Synchronous Read/Write
    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din;
        end
        // Synchronous read (registers the output)
        dout <= mem[addr];
    end

endmodule