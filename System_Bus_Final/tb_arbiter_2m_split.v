`timescale 1ns / 1ps

module tb_arbiter_2m_split;

    localparam CLK_PERIOD = 20; // 50 MHz (10ns High / 10ns Low)

    reg  clk;
    reg  rst_n;
    reg  req0;
    reg  req1;
    reg  s0_split_req;
    reg  s0_split_done;

    wire gnt0;
    wire gnt1;
    wire m0_split_notify;
    wire m1_split_notify;

    integer error_count = 0;
    integer test_id = 0;

    // Unit Under Test
    arbiter_2m_split uut (
        .clk(clk),
        .rst_n(rst_n),
        .req0(req0),
        .req1(req1),
        .gnt0(gnt0),
        .gnt1(gnt1),
        .s0_split_req(s0_split_req),
        .s0_split_done(s0_split_done),
        .m0_split_notify(m0_split_notify),
        .m1_split_notify(m1_split_notify)
    );

    // 50 MHz Clock Generator
    always #(CLK_PERIOD/2) clk = ~clk;

    // Check helper
    task check(input condition, input [255:0] err_msg);
        begin
            if (!condition) begin
                $display("[ERROR @ %0t ns] Test %0d: %s", $time, test_id, err_msg);
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        // Initialize Inputs
        clk           = 0;
        rst_n         = 0;
        req0          = 0;
        req1          = 0;
        s0_split_req  = 0;
        s0_split_done = 0;

        $display("=========================================================");
        $display("   STARTING ARBITER UNIT VERIFICATION (50 MHz)           ");
        $display("=========================================================");

        // -----------------------------------------------------------
        // TEST 1: Asynchronous Reset Test
        // -----------------------------------------------------------
        test_id = 1;
        $display("\n[TEST 1] Testing Asynchronous Reset Behavior...");
        #25;
        check((gnt0 == 1'b0 && gnt1 == 1'b0), "Grants not de-asserted on reset");
        check((m0_split_notify == 1'b0),      "m0_split_notify not 0 on reset");

        @(posedge clk); #2;
        rst_n = 1;
        $display("  -> Reset verified successfully.");

        // -----------------------------------------------------------
        // TEST 2: Single Master Request (M0 solo, then M1 solo)
        // -----------------------------------------------------------
        test_id = 2;
        $display("\n[TEST 2] Testing Single Master Requests (M0 and M1 isolated)...");
        
        // M0 Solo Request
        @(posedge clk); #2;
        req0 = 1;
        @(posedge clk); #2;
        check((gnt0 == 1'b1 && gnt1 == 1'b0), "M0 grant failed to assert");

        @(posedge clk); #2;
        req0 = 0; // Release
        @(posedge clk); #2;
        check((gnt0 == 1'b0 && gnt1 == 1'b0), "Arbiter did not return to IDLE after M0");

        // M1 Solo Request
        req1 = 1;
        @(posedge clk); #2;
        check((gnt1 == 1'b1 && gnt0 == 1'b0), "M1 grant failed to assert");

        @(posedge clk); #2;
        req1 = 0; // Release
        @(posedge clk); #2;
        check((gnt0 == 1'b0 && gnt1 == 1'b0), "Arbiter did not return to IDLE after M1");
        $display("  -> Single master requests verified successfully.");

        // -----------------------------------------------------------
        // TEST 3: Simultaneous Contention (M0 Priority over M1)
        // -----------------------------------------------------------
        test_id = 3;
        $display("\n[TEST 3] Testing Dual Master Contention (Priority Rule)...");
        
        // Assert req0 and req1 simultaneously
        @(posedge clk); #2;
        req0 = 1;
        req1 = 1;

        @(posedge clk); #2;
        check((gnt0 == 1'b1 && gnt1 == 1'b0), "Priority failure: M0 should have been granted over M1");

        // Hold for 2 cycles to verify M0 retains bus while requested
        repeat (2) @(posedge clk); #2;
        check((gnt0 == 1'b1 && gnt1 == 1'b0), "M0 lost grant prematurely during contention");

        // M0 finishes, releases req0; M1 should be granted next
        req0 = 0;
        @(posedge clk); #2;
        check((gnt0 == 1'b0 && gnt1 == 1'b1), "M1 was not granted bus after M0 released");

        // M1 finishes
        req1 = 0;
        @(posedge clk); #2;
        check((gnt0 == 1'b0 && gnt1 == 1'b0), "Bus not idle after both finished");
        $display("  -> Dual master contention and priority verified successfully.");

        // -----------------------------------------------------------
        // TEST 4: Split Transaction Handshake Scenario
        // -----------------------------------------------------------
        test_id = 4;
        $display("\n[TEST 4] Testing Split-Transaction Bus Handover & Re-Grant...");
        
        // Step 4a: M0 requests bus and gets grant
        @(posedge clk); #2;
        req0 = 1;
        @(posedge clk); #2;
        check((gnt0 == 1'b1), "M0 not granted bus for split test");

        // Step 4b: Slave 0 asserts split_req, M1 has pending request
        req1 = 1;
        s0_split_req = 1;

        @(posedge clk); #2;
        check((m0_split_notify == 1'b1), "m0_split_notify failed to pulse on split");
        check((gnt0 == 1'b0),            "M0 grant not revoked during split");
        check((gnt1 == 1'b1),            "M1 failed to acquire bus while M0 was split");

        s0_split_req = 0;

        // Step 4c: M1 executes single-cycle transfer while S0 prepares M0's data
        @(posedge clk); #2;
        req1 = 0; // M1 finishes its access

        // S0 finishes background read
        s0_split_done = 1;

        @(posedge clk); #2;
        s0_split_done = 0;
        check((gnt0 == 1'b1 && gnt1 == 1'b0), "M0 was not re-granted bus after split completion");

        // Step 4d: M0 finishes reading returned split data
        @(posedge clk); #2;
        req0 = 0;

        @(posedge clk); #2;
        check((gnt0 == 1'b0 && gnt1 == 1'b0), "Arbiter failed to return to IDLE after split cycle");
        $display("  -> Split transaction bus release and re-grant verified successfully.");

        // -----------------------------------------------------------
        // Final Summary
        // -----------------------------------------------------------
        #40;
        $display("\n=========================================================");
        if (error_count == 0) begin
            $display(">> ARBITER TEST SUCCESSFUL: All Cases Passed! <<");
        end else begin
            $display(">> ARBITER TEST FAILED: %0d error(s) detected! <<", error_count);
        end
        $display("=========================================================");

        #50;
        $finish;
    end

endmodule