`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/01/2026 01:58:32 AM
// Design Name: 
// Module Name: address_decoder
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

module address_decoder (
    input  wire [13:0] addr,
    input  wire        bus_valid,
    output reg         sel_s0,      // 0x0000 - 0x0FFF (4K Split)
    output reg         sel_s1,      // 0x1000 - 0x1FFF (4K Fast)
    output reg         sel_s2,      // 0x2000 - 0x27FF (2K Fast)
    output reg         sel_bridge,  // 0x3000 - 0x3FFF (External Bridge)
    output reg         decode_err
);

    always @(*) begin
        sel_s0     = 1'b0;
        sel_s1     = 1'b0;
        sel_s2     = 1'b0;
        sel_bridge = 1'b0;
        decode_err = 1'b0;

        if (bus_valid) begin
            if (addr[13:12] == 2'b00) begin
                sel_s0 = 1'b1; // 0x0000 - 0x0FFF
            end else if (addr[13:12] == 2'b01) begin
                sel_s1 = 1'b1; // 0x1000 - 0x1FFF
            end else if (addr[13:11] == 3'b100) begin
                sel_s2 = 1'b1; // 0x2000 - 0x27FF
            end else if (addr[13:12] == 2'b11) begin
                sel_bridge = 1'b1; // 0x3000 - 0x3FFF
            end else begin
                decode_err = 1'b1;
            end
        end
    end

endmodule