`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/31/2026 08:54:21 PM
// Design Name: 
// Module Name: tb_ram_4k
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

module tb_ram_4k;

    parameter DATA_WIDTH = 8;
    parameter ADDR_WIDTH = 12;
    localparam MEM_DEPTH = (1 << ADDR_WIDTH); // 4096

    reg                   clk;
    reg                   we;
    reg  [ADDR_WIDTH-1:0] addr;
    reg  [DATA_WIDTH-1:0] din;
    wire [DATA_WIDTH-1:0] dout;

    integer i;
    integer error_count = 0;

    // Instantiate RAM Module
    ram_4k #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) uut (
        .clk(clk),
        .we(we),
        .addr(addr),
        .din(din),
        .dout(dout)
    );

    // 100 MHz Clock Generation (10 ns period)
    always #5 clk = ~clk;

    // Task: Write single word
    task write_word(input [ADDR_WIDTH-1:0] waddr, input [DATA_WIDTH-1:0] wdata);
        begin
            @(posedge clk);
            we   <= 1'b1;
            addr <= waddr;
            din  <= wdata;
        end
    endtask

    // Task: Read and check single word
    task read_and_check(input [ADDR_WIDTH-1:0] raddr, input [DATA_WIDTH-1:0] expected_data);
        begin
            @(posedge clk);
            we   <= 1'b0;
            addr <= raddr;

            // Wait 1 cycle for synchronous read data to latch onto dout
            @(posedge clk);
            #1; // Small delta delay to sample after clock edge
            if (dout !== expected_data) begin
                $display("[ERROR] Addr: 0x%03h | Expected: 0x%02h | Got: 0x%02h", raddr, expected_data, dout);
                error_count = error_count + 1;
            end else begin
                $display("[PASS]  Addr: 0x%03h | Data: 0x%02h", raddr, dout);
            end
        end
    endtask

    initial begin
        // Initialize signals
        clk  = 0;
        we   = 0;
        addr = 0;
        din  = 0;

        #20; // Initial settling delay

        $display("----------------------------------------------");
        $display("Starting 4K RAM Write/Read Verification...");
        $display("----------------------------------------------");

        // --- STEP 1: Write to first 16 locations + Boundary Limits ---
        for (i = 0; i < 16; i = i + 1) begin
            write_word(i, i ^ 8'hA5); // Write test pattern
        end
        write_word(MEM_DEPTH - 1, 8'hFF); // Test top boundary address (4095)

        // Disable write before reading
        @(posedge clk);
        we <= 1'b0;

        #20;

        // --- STEP 2: Read back and verify ---
        $display("Verifying Data...");
        for (i = 0; i < 16; i = i + 1) begin
            read_and_check(i, i ^ 8'hA5);
        end
        read_and_check(MEM_DEPTH - 1, 8'hFF); // Verify top boundary address

        // --- STEP 3: Summary Report ---
        $display("----------------------------------------------");
        if (error_count == 0) begin
            $display(">> SIMULATION SUCCESSFUL: All checks passed! <<");
        end else begin
            $display(">> SIMULATION FAILED: %0d mismatch(es) found! <<", error_count);
        end
        $display("----------------------------------------------");

        #50;
        $finish;
    end

endmodule
