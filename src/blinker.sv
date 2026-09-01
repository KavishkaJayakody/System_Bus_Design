`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/31/2026 05:43:01 PM
// Design Name: 
// Module Name: blinker
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

module blinker #(
    parameter COUNT_MAX = 50_000_000 // 0.5s toggle at 100 MHz (hardware default)
)(
    input  wire clk,
    input  wire rst,
    output reg  led
);

    reg [$clog2(COUNT_MAX)-1:0] counter;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
            led     <= 1'b0;
        end else begin
            if (counter == COUNT_MAX - 1) begin
                counter <= 0;
                led     <= ~led;
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule