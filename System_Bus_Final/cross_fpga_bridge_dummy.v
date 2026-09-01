`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/01/2026 02:00:21 AM
// Design Name: 
// Module Name: cross_fpga_bridge_dummy
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

module cross_fpga_bridge_dummy (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sel,
    input  wire        we,
    input  wire [11:0] addr,
    input  wire [7:0]  din,
    output reg  [7:0]  dout,
    output reg         ready,
    
    // Physical cross-FPGA pins (to be wired to GPIO headers on DE0)
    input  wire        ext_rx_serial,
    output wire        ext_tx_serial
);

    // Dummy loopback register for initial local verification
    reg [7:0] dummy_reg;
    assign ext_tx_serial = 1'b1; // Idle high

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dummy_reg <= 8'hBE;
            dout      <= 8'h00;
            ready     <= 1'b0;
        end else begin
            ready <= 1'b0;
            if (sel) begin
                if (we) dummy_reg <= din;
                dout  <= dummy_reg;
                ready <= 1'b1;
            end
        end
    end

endmodule