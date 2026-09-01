`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/01/2026 12:13:42 PM
// Design Name: 
// Module Name: tb_master_node
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


module tb_master_node;

    // 50 MHz Clock: Period = 20 ns (10 ns High / 10 ns Low)
    localparam CLK_PERIOD = 20;

    // Clock and Reset
    reg         clk;
    reg         rst_n;

    // Command Interface (Driver)
    reg         cmd_start;
    reg         cmd_we;
    reg  [13:0] cmd_addr;
    reg  [7:0]  cmd_wdata;
    wire [7:0]  cmd_rdata;
    wire        cmd_done;

    // Arbiter Interface (Emulated)
    wire        bus_req;
    reg         bus_gnt;
    reg         bus_split;

    // Bus Interface (Monitored & Emulated Slave)
    wire [13:0] m_addr;
    wire [7:0]  m_wdata;
    wire        m_we;
    wire        m_valid;
    reg  [7:0]  bus_rdata;
    reg         bus_ready;

    // Test Tracking
    integer error_count = 0;
    integer test_id = 0;

    // Unit Under Test (UUT)
    master_node uut (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_start(cmd_start),
        .cmd_we(cmd_we),
        .cmd_addr(cmd_addr),
        .cmd_wdata(cmd_wdata),
        .cmd_rdata(cmd_rdata),
        .cmd_done(cmd_done),
        .bus_req(bus_req),
        .bus_gnt(bus_gnt),
        .bus_split(bus_split),
        .m_addr(m_addr),
        .m_wdata(m_wdata),
        .m_we(m_we),
        .m_valid(m_valid),
        .bus_rdata(bus_rdata),
        .bus_ready(bus_ready)
    );

    // 50 MHz Clock Generation
    always #(CLK_PERIOD/2) clk = ~clk;

    // Assertion helper
    task check_condition(input condition, input [255:0] err_msg);
        begin
            if (!condition) begin
                $display("[ERROR @ %0t ns] Test %0d: %s", $time, test_id, err_msg);
                error_count = error_count + 1;
            end
        end
    endtask

    // Main Test Sequence
    initial begin
        // Initialize Inputs
        clk       = 0;
        rst_n     = 0;
        cmd_start = 0;
        cmd_we    = 0;
        cmd_addr  = 14'h0000;
        cmd_wdata = 8'h00;
        bus_gnt   = 0;
        bus_split = 0;
        bus_rdata = 8'h00;
        bus_ready = 0;

        $display("=========================================================");
        $display("   STARTING MASTER NODE UNIT VERIFICATION (50 MHz)       ");
        $display("=========================================================");

        // -----------------------------------------------------------
        // TEST 1: Asynchronous Reset Test
        // -----------------------------------------------------------
        test_id = 1;
        $display("[TEST 1] Testing Asynchronous Reset Behavior...");
        #25; // Settle during active reset
        check_condition((uut.state == 3'b000), "State not IDLE after reset");
        check_condition((bus_req == 1'b0),     "bus_req not deasserted on reset");
        check_condition((m_valid == 1'b0),     "m_valid not deasserted on reset");
        check_condition((cmd_done == 1'b0),    "cmd_done not deasserted on reset");

        @(posedge clk);
        #2;
        rst_n = 1; // Release reset synchronously
        $display("  -> Reset verified successfully.");

        // -----------------------------------------------------------
        // TEST 2: Single-Cycle Fast Write (Immediate Grant & Ready)
        // -----------------------------------------------------------
        test_id = 2;
        $display("[TEST 2] Testing Single-Cycle Fast Write Transaction...");
        @(posedge clk);
        #2;
        cmd_addr  = 14'h1A2B;
        cmd_wdata = 8'h5A;
        cmd_we    = 1'b1;
        cmd_start = 1'b1;

        @(posedge clk);
        #2;
        cmd_start = 1'b0; // Single cycle command pulse
        check_condition((bus_req == 1'b1), "bus_req failed to assert upon cmd_start");

        // Emulate immediate Grant from Arbiter
        bus_gnt = 1'b1;

        @(posedge clk);
        #2;
        check_condition((m_valid == 1'b1),      "m_valid failed to assert in DRIVE");
        check_condition((m_addr == 14'h1A2B),   "m_addr mismatch");
        check_condition((m_wdata == 8'h5A),     "m_wdata mismatch");
        check_condition((m_we == 1'b1),         "m_we mismatch");

        // Emulate single-cycle slave Ready
        bus_ready = 1'b1;

        @(posedge clk);
        #2;
        bus_gnt   = 1'b0;
        bus_ready = 1'b0;
        check_condition((cmd_done == 1'b1), "cmd_done did not pulse after bus_ready");
        check_condition((bus_req == 1'b0),  "bus_req was not cleared upon completion");
        check_condition((m_valid == 1'b0),  "m_valid was not cleared upon completion");
        $display("  -> Fast Write verified successfully.");

        // -----------------------------------------------------------
        // TEST 3: Multi-Cycle Delayed Read (Wait States)
        // -----------------------------------------------------------
        test_id = 3;
        $display("[TEST 3] Testing Multi-Cycle Latency Read (WAIT States)...");
        @(posedge clk);
        #2;
        cmd_addr  = 14'h2040;
        cmd_we    = 1'b0;
        cmd_start = 1'b1;

        @(posedge clk);
        #2;
        cmd_start = 0;
        // Hold grant off for 2 cycles to test arbitration wait
        repeat (2) @(posedge clk);
        #2;
        bus_gnt = 1'b1;

        @(posedge clk); // Master enters DRIVE
        #2;
        check_condition((m_valid == 1'b1), "m_valid not asserted in DRIVE");
        check_condition((m_we == 1'b0),    "m_we should be 0 for read");

        // Slave does NOT assert ready immediately -> Master goes to WAIT
        @(posedge clk); // Master enters WAIT
        #2;
        check_condition((uut.state == 3'b011), "Master did not enter WAIT state");

        // Now slave asserts ready with read data
        bus_rdata = 8'hC7;
        bus_ready = 1'b1;

        @(posedge clk);
        #2;
        bus_gnt   = 1'b0;
        bus_ready = 1'b0;
        check_condition((cmd_done == 1'b1),     "cmd_done failed to assert on read completion");
        check_condition((cmd_rdata == 8'hC7),   "cmd_rdata mismatch (expected 0xC7)");
        $display("  -> Multi-cycle Read verified successfully.");

        // -----------------------------------------------------------
        // TEST 4: Split Transaction Handshake
        // -----------------------------------------------------------
        test_id = 4;
        $display("[TEST 4] Testing Split Transaction Back-off and Resume...");
        @(posedge clk);
        #2;
        cmd_addr  = 14'h0050; // Address on Split Slave
        cmd_we    = 1'b0;
        cmd_start = 1'b1;

        @(posedge clk);
        #2;
        cmd_start = 1'b0;
        bus_gnt   = 1'b1;

        @(posedge clk); // Master enters DRIVE
        #2;
        // Slave asserts Split response
        bus_split = 1'b1;

        @(posedge clk); // Master enters SPLIT_W
        #2;
        bus_split = 1'b0;
        bus_gnt   = 1'b0; // Arbiter takes grant away
        check_condition((uut.state == 3'b100), "Master did not enter SPLIT_W state");
        check_condition((m_valid == 1'b0),     "m_valid must drop to 0 during SPLIT");
        check_condition((bus_req == 1'b1),     "Master must keep bus_req asserted in SPLIT_W to re-acquire");

        // Wait 3 idle cycles while slave completes background lookup
        repeat (3) @(posedge clk);

        // Arbiter re-grants bus to complete split transaction
        #2;
        bus_gnt = 1'b1;

        @(posedge clk); // Master re-enters DRIVE
        #2;
        check_condition((m_valid == 1'b1),    "m_valid did not re-assert on re-grant");
        check_condition((m_addr == 14'h0050), "Original address not preserved on resume");

        // Slave delivers data
        bus_rdata = 8'h8E;
        bus_ready = 1'b1;

        @(posedge clk);
        #2;
        bus_gnt   = 1'b0;
        bus_ready = 1'b0;
        check_condition((cmd_done == 1'b1),   "cmd_done failed on split completion");
        check_condition((cmd_rdata == 8'h8E), "cmd_rdata mismatch on split read");
        $display("  -> Split Transaction sequence verified successfully.");

        // -----------------------------------------------------------
        // TEST 5: Consecutive Back-to-Back Transfers
        // -----------------------------------------------------------
        test_id = 5;
        $display("[TEST 5] Testing Consecutive Commands...");
        @(posedge clk);
        #2;
        // Command 1
        cmd_addr  = 14'h1000;
        cmd_wdata = 8'h11;
        cmd_we    = 1'b1;
        cmd_start = 1'b1;
        bus_gnt   = 1'b1;
        @(posedge clk); #2; cmd_start = 0; bus_ready = 1;
        @(posedge clk); #2; bus_ready = 0; bus_gnt = 0;

        // Command 2 immediately on next cycle
        @(posedge clk);
        #2;
        cmd_addr  = 14'h1001;
        cmd_wdata = 8'h22;
        cmd_we    = 1'b1;
        cmd_start = 1'b1;
        bus_gnt   = 1'b1;
        @(posedge clk); #2; cmd_start = 0; bus_ready = 1;
        @(posedge clk); #2; bus_ready = 0; bus_gnt = 0;

        @(posedge clk);
        #2;
        check_condition((uut.state == 3'b000), "Master did not return cleanly to IDLE");
        $display("  -> Consecutive transfers verified successfully.");

        // -----------------------------------------------------------
        // Final Summary
        // -----------------------------------------------------------
        #40;
        $display("=========================================================");
        if (error_count == 0) begin
            $display(">> MASTER NODE TEST SUCCESSFUL: All 5 Test Cases Passed! <<");
        end else begin
            $display(">> MASTER NODE TEST FAILED: %0d error(s) detected! <<", error_count);
        end
        $display("=========================================================");

        #100;
        $finish;
    end

endmodule
