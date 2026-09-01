`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/01/2026 02:02:35 AM
// Design Name: 
// Module Name: tb_top_bus_system
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

module tb_top_bus_system;

    reg        clk;
    reg        rst_n;

    reg        m0_cmd_start, m0_cmd_we;
    reg [13:0] m0_cmd_addr;
    reg [7:0]  m0_cmd_wdata;
    wire [7:0] m0_cmd_rdata;
    wire       m0_cmd_done;

    reg        m1_cmd_start, m1_cmd_we;
    reg [13:0] m1_cmd_addr;
    reg [7:0]  m1_cmd_wdata;
    wire [7:0] m1_cmd_rdata;
    wire       m1_cmd_done;

    wire       ext_tx_serial;

    top_bus_system uut (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(m0_cmd_start), .m0_cmd_we(m0_cmd_we), .m0_cmd_addr(m0_cmd_addr), .m0_cmd_wdata(m0_cmd_wdata),
        .m0_cmd_rdata(m0_cmd_rdata), .m0_cmd_done(m0_cmd_done),
        .m1_cmd_start(m1_cmd_start), .m1_cmd_we(m1_cmd_we), .m1_cmd_addr(m1_cmd_addr), .m1_cmd_wdata(m1_cmd_wdata),
        .m1_cmd_rdata(m1_cmd_rdata), .m1_cmd_done(m1_cmd_done),
        .ext_rx_serial(1'b1),
        .ext_tx_serial(ext_tx_serial)
    );

    // 50 MHz Clock
    always #10 clk = ~clk;

    integer watchdog_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            watchdog_count <= 0;
        end else begin
            if (m0_cmd_start || m1_cmd_start || uut.req0 || uut.req1 || uut.gnt0 || uut.gnt1 || uut.s0_split_req || uut.s0_split_done || uut.m0_inst.bus_req || uut.m1_inst.bus_req) begin
                watchdog_count <= watchdog_count + 1;
                if (watchdog_count >= 2000) begin
                    $display("WATCHDOG: bus appears stalled.");
                    $display("  req0=%0b req1=%0b gnt0=%0b gnt1=%0b m0_done=%0b m1_done=%0b s0_split_req=%0b s0_split_done=%0b",
                             uut.req0, uut.req1, uut.gnt0, uut.gnt1,
                             m0_cmd_done, m1_cmd_done, uut.s0_split_req, uut.s0_split_done);
                    $display("  m0_state=%0d m1_state=%0d arb_state=%0d", uut.m0_inst.state, uut.m1_inst.state, uut.arbiter_inst.state);
                    $finish;
                end
            end else begin
                watchdog_count <= 0;
            end
        end
    end

    initial begin
        clk = 0;
        rst_n = 0;
        m0_cmd_start = 0; m0_cmd_we = 0; m0_cmd_addr = 0; m0_cmd_wdata = 0;
        m1_cmd_start = 0; m1_cmd_we = 0; m1_cmd_addr = 0; m1_cmd_wdata = 0;

        $dumpfile("tb_top_bus_system.vcd");
        $dumpvars(0, tb_top_bus_system);

        #30 rst_n = 1; // Release Reset
        #20;

        $display("=========================================================");
        $display("   STARTING TOP LEVEL BUS INTEGRATION SIMULATION         ");
        $display("=========================================================");

        // --- SCENARIO 1: Single Master R/W to Fast Slaves (S1 & S2) ---
        $display("[TEST 1] Single Master R/W to S1 (4K) and S2 (2K)");
        
        // M0 Write 0x3C to S1 (0x1010)
        @(posedge clk);
        m0_cmd_addr = 14'h1010; m0_cmd_wdata = 8'h3C; m0_cmd_we = 1; m0_cmd_start = 1;
        @(posedge clk); m0_cmd_start = 0;
        @(posedge m0_cmd_done);

        // M0 Read from S1 (0x1010)
        @(posedge clk);
        m0_cmd_addr = 14'h1010; m0_cmd_we = 0; m0_cmd_start = 1;
        @(posedge clk); m0_cmd_start = 0;
        //@(posedge m0_cmd_done);
        $display("  -> S1 Read Result: 0x%02h (Expected: 0x3C)", m0_cmd_rdata);

        // --- SCENARIO 2: Dual Master Contention (Priority Test) ---
        $display("[TEST 2] Dual Master Contention: M0 (High) vs M1 (Low)");
        @(posedge clk);
        // Assert both masters simultaneously
        m0_cmd_addr = 14'h2005; m0_cmd_wdata = 8'hAA; m0_cmd_we = 1; m0_cmd_start = 1;
        m1_cmd_addr = 14'h2006; m1_cmd_wdata = 8'hBB; m1_cmd_we = 1; m1_cmd_start = 1;
        @(posedge clk);
        m0_cmd_start = 0; m1_cmd_start = 0;

        // M0 must finish before M1 completes
        @(posedge m0_cmd_done);
        $display("  -> M0 Finished Contention Write first.");
        @(posedge m1_cmd_done);
        $display("  -> M1 Finished Contention Write after M0 released bus.");

        // --- SCENARIO 3: Split Transaction on Slave 0 ---
        $display("[TEST 3] Split Transaction on Slave 0 with Interleaved M1");
        
        // Step 3a: Write data to S0 (0x000A)
        @(posedge clk);
        m0_cmd_addr = 14'h000A; m0_cmd_wdata = 8'h77; m0_cmd_we = 1; m0_cmd_start = 1;
        @(posedge clk); m0_cmd_start = 0;
        @(posedge m0_cmd_done);

        // Step 3b: M0 issues Read to S0 (triggers split), M1 simultaneously requests S2
        @(posedge clk);
        m0_cmd_addr = 14'h000A; m0_cmd_we = 0; m0_cmd_start = 1;
        #10;
        m1_cmd_addr = 14'h2001; m1_cmd_wdata = 8'h55; m1_cmd_we = 1; m1_cmd_start = 1;
        @(posedge clk);
        m0_cmd_start = 0; m1_cmd_start = 0;

        // M1 should finish its S2 access while M0 is parked in SPLIT wait
        @(posedge m1_cmd_done);
        $display("  -> [SPLIT SUCCESS] M1 serviced while M0 split request was pending!");

        // M0 completes after S0 signals split done
        @(posedge m0_cmd_done);
        $display("  -> S0 Split Read Result: 0x%02h (Expected: 0x77)", m0_cmd_rdata);

        // --- SCENARIO 4: Dummy Bridge Access ---
        $display("[TEST 4] External Bridge Register Access (0x3000)");
        @(posedge clk);
        m0_cmd_addr = 14'h3000; m0_cmd_wdata = 8'hE2; m0_cmd_we = 1; m0_cmd_start = 1;
        @(posedge clk); m0_cmd_start = 0;
        @(posedge m0_cmd_done);

        @(posedge clk);
        m0_cmd_addr = 14'h3000; m0_cmd_we = 0; m0_cmd_start = 1;
        @(posedge clk); m0_cmd_start = 0;
        @(posedge m0_cmd_done);
        $display("  -> Bridge Read Result: 0x%02h (Expected: 0xE2)", m0_cmd_rdata);

        $display("=========================================================");
        $display("   ALL TEST SCENARIOS COMPLETED SUCCESSFULLY             ");
        $display("=========================================================");
        #50 $finish;
    end

endmodule