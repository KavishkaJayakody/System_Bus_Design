`timescale 1ns / 1ps

module tb_address_decoder;

    // Simulation Timing: 50 MHz clock (20 ns period)
    localparam CLK_PERIOD = 20;

    // Signals
    reg         clk;
    reg  [13:0] addr;
    reg         bus_valid;
    wire        sel_s0;
    wire        sel_s1;
    wire        sel_s2;
    wire        sel_bridge;
    wire        decode_err;

    // Verification Tracking
    integer error_count = 0;
    integer test_id = 0;

    // Instantiate UUT
    address_decoder uut (
        .addr(addr),
        .bus_valid(bus_valid),
        .sel_s0(sel_s0),
        .sel_s1(sel_s1),
        .sel_s2(sel_s2),
        .sel_bridge(sel_bridge),
        .decode_err(decode_err)
    );

    // 50 MHz Clock Generator
    always #(CLK_PERIOD/2) clk = ~clk;

    // Verification Task
    task verify_decode(
        input [13:0] test_addr,
        input        test_valid,
        input        exp_s0,
        input        exp_s1,
        input        exp_s2,
        input        exp_br,
        input        exp_err,
        input [127:0] desc
    );
        reg [4:0] actual_vector;
        reg [4:0] exp_vector;
        begin
            @(posedge clk);
            addr      <= test_addr;
            bus_valid <= test_valid;
            
            #1; // Delta delay to evaluate combinational outputs
            actual_vector = {sel_s0, sel_s1, sel_s2, sel_bridge, decode_err};
            exp_vector    = {exp_s0, exp_s1, exp_s2, exp_br, exp_err};

            if (actual_vector !== exp_vector) begin
                $display("[ERROR @ %0t ns] Addr: 0x%04h | Valid: %b | Expected: {s0,s1,s2,br,err}=%b | Got: %b | Info: %s",
                         $time, test_addr, test_valid, exp_vector, actual_vector, desc);
                error_count = error_count + 1;
            end else begin
                $display("[PASS]  Addr: 0x%04h | Valid: %b | Out: {s0:%b, s1:%b, s2:%b, br:%b, err:%b} | %s",
                         test_addr, test_valid, sel_s0, sel_s1, sel_s2, sel_bridge, decode_err, desc);
            end
        end
    endtask

    initial begin
        // Initialize Inputs
        clk       = 0;
        addr      = 14'h0000;
        bus_valid = 1'b0;

        #25; // Settle time

        $display("=========================================================");
        $display("   STARTING ADDRESS DECODER UNIT VERIFICATION (50 MHz)   ");
        $display("=========================================================");

        // -----------------------------------------------------------
        // TEST 1: Bus Invalid Checks (bus_valid = 0)
        // -----------------------------------------------------------
        $display("\n[TEST 1] Testing Bus Invalid (bus_valid = 0)...");
        verify_decode(14'h0000, 1'b0, 0, 0, 0, 0, 0, "S0 Range with valid=0");
        verify_decode(14'h1500, 1'b0, 0, 0, 0, 0, 0, "S1 Range with valid=0");
        verify_decode(14'h2200, 1'b0, 0, 0, 0, 0, 0, "S2 Range with valid=0");
        verify_decode(14'h3500, 1'b0, 0, 0, 0, 0, 0, "Bridge Range with valid=0");
        verify_decode(14'h2900, 1'b0, 0, 0, 0, 0, 0, "Unmapped Range with valid=0");

        // -----------------------------------------------------------
        // TEST 2: Slave 0 Space (0x0000 - 0x0FFF, 4KB)
        // -----------------------------------------------------------
        $display("\n[TEST 2] Testing Slave 0 Space (4K Split)...");
        verify_decode(14'h0000, 1'b1, 1, 0, 0, 0, 0, "Slave 0 Start (0x0000)");
        verify_decode(14'h0555, 1'b1, 1, 0, 0, 0, 0, "Slave 0 Mid (0x0555)");
        verify_decode(14'h0FFF, 1'b1, 1, 0, 0, 0, 0, "Slave 0 End (0x0FFF)");

        // -----------------------------------------------------------
        // TEST 3: Slave 1 Space (0x1000 - 0x1FFF, 4KB)
        // -----------------------------------------------------------
        $display("\n[TEST 3] Testing Slave 1 Space (4K Fast)...");
        verify_decode(14'h1000, 1'b1, 0, 1, 0, 0, 0, "Slave 1 Start (0x1000)");
        verify_decode(14'h1A00, 1'b1, 0, 1, 0, 0, 0, "Slave 1 Mid (0x1A00)");
        verify_decode(14'h1FFF, 1'b1, 0, 1, 0, 0, 0, "Slave 1 End (0x1FFF)");

        // -----------------------------------------------------------
        // TEST 4: Slave 2 Space (0x2000 - 0x27FF, 2KB)
        // -----------------------------------------------------------
        $display("\n[TEST 4] Testing Slave 2 Space (2K Fast)...");
        verify_decode(14'h2000, 1'b1, 0, 0, 1, 0, 0, "Slave 2 Start (0x2000)");
        verify_decode(14'h23FE, 1'b1, 0, 0, 1, 0, 0, "Slave 2 Mid (0x23FE)");
        verify_decode(14'h27FF, 1'b1, 0, 0, 1, 0, 0, "Slave 2 End (0x27FF)");

        // -----------------------------------------------------------
        // TEST 5: Unmapped Space / Decode Error (0x2800 - 0x2FFF)
        // -----------------------------------------------------------
        $display("\n[TEST 5] Testing Unmapped Address Range (0x2800 - 0x2FFF)...");
        verify_decode(14'h2800, 1'b1, 0, 0, 0, 0, 1, "Unmapped Gap Start (0x2800)");
        verify_decode(14'h2A50, 1'b1, 0, 0, 0, 0, 1, "Unmapped Gap Mid (0x2A50)");
        verify_decode(14'h2FFF, 1'b1, 0, 0, 0, 0, 1, "Unmapped Gap End (0x2FFF)");

        // -----------------------------------------------------------
        // TEST 6: External Bridge Space (0x3000 - 0x3FFF, 4KB)
        // -----------------------------------------------------------
        $display("\n[TEST 6] Testing External Bridge Space (4K)...");
        verify_decode(14'h3000, 1'b1, 0, 0, 0, 1, 0, "Bridge Start (0x3000)");
        verify_decode(14'h3800, 1'b1, 0, 0, 0, 1, 0, "Bridge Mid (0x3800)");
        verify_decode(14'h3FFF, 1'b1, 0, 0, 0, 1, 0, "Bridge End (0x3FFF)");

        // -----------------------------------------------------------
        // Final Summary
        // -----------------------------------------------------------
        #40;
        $display("\n=========================================================");
        if (error_count == 0) begin
            $display(">> ADDRESS DECODER TEST SUCCESSFUL: All Cases Passed! <<");
        end else begin
            $display(">> ADDRESS DECODER TEST FAILED: %0d error(s) detected! <<", error_count);
        end
        $display("=========================================================");

        #50;
        $finish;
    end

endmodule