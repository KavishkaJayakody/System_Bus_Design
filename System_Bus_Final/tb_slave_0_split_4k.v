`timescale 1ns / 1ps

module tb_slave_0_split_4k;

    localparam CLK_PERIOD = 20; // 50 MHz (10ns HIGH / 10ns LOW)

    reg         clk;
    reg         rst_n;
    reg         sel;
    reg         we;
    reg  [11:0] addr;
    reg  [7:0]  din;
    wire [7:0]  dout;
    wire        ready;
    wire        split_req;
    wire        split_done;

    integer error_count = 0;
    integer test_id = 0;

    // Unit Under Test
    slave_0_split_4k uut (
        .clk(clk),
        .rst_n(rst_n),
        .sel(sel),
        .we(we),
        .addr(addr),
        .din(din),
        .dout(dout),
        .ready(ready),
        .split_req(split_req),
        .split_done(split_done)
    );

    // 50 MHz Clock Generator
    always #(CLK_PERIOD/2) clk = ~clk;

    // Assertion Helper
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
        clk   = 0;
        rst_n = 0;
        sel   = 0;
        we    = 0;
        addr  = 12'h000;
        din   = 8'h00;

        $display("=========================================================");
        $display("   STARTING SLAVE 0 (4K SPLIT) UNIT TEST (50 MHz)        ");
        $display("=========================================================");

        // -----------------------------------------------------------
        // TEST 1: Asynchronous Reset Test
        // -----------------------------------------------------------
        test_id = 1;
        $display("\n[TEST 1] Testing Asynchronous Reset Behavior...");
        #25;
        check((ready == 1'b0 && split_req == 1'b0 && split_done == 1'b0), "Outputs not 0 on reset");
        
        @(posedge clk);
        #2;
        rst_n = 1;
        $display("  -> Reset verified successfully.");

        // -----------------------------------------------------------
        // TEST 2: Single-Cycle Fast Writes (Multiple Locations)
        // -----------------------------------------------------------
        test_id = 2;
        $display("\n[TEST 2] Testing Single-Cycle Fast Write...");
        
        // Write 0xA5 to 0x00A
        @(posedge clk);
        #2;
        sel = 1; we = 1; addr = 12'h00A; din = 8'hA5;
        
        @(posedge clk);
        #2;
        check((ready == 1'b1), "Ready did not assert on single-cycle write");
        check((split_req == 1'b0), "split_req asserted on write (illegal)");
        
        // Write 0x5B to boundary 0xFFF
        addr = 12'hFFF; din = 8'h5B;
        
        @(posedge clk);
        #2;
        check((ready == 1'b1), "Ready did not assert on boundary write");
        sel = 0; we = 0;
        $display("  -> Fast writes verified successfully.");

        // -----------------------------------------------------------
        // TEST 3: Split-Read Transaction Flow & Latency Countdown
        // -----------------------------------------------------------
        test_id = 3;
        $display("\n[TEST 3] Testing Split Read Cycle & Latency Count...");
        
        // Request Read from 0x00A (Contains 0xA5)
        @(posedge clk);
        #2;
        sel = 1; we = 0; addr = 12'h00A;

        @(posedge clk); // S0 latches request, enters processing_split
        #2;
        check((split_req == 1'b1), "split_req failed to assert on initial read");
        check((ready == 1'b0),     "ready asserted during split initial phase");

        // Master de-asserts select to simulate bus back-off
        sel = 0;

        // Wait for split_done to pulse high
        @(posedge clk); // latency_cnt: 1 -> 2
        #2;
        check((split_done == 1'b0), "split_done asserted prematurely at cycle 1");

        @(posedge clk); // latency_cnt: 2 -> 3 (SPLIT_LATENCY condition met)
        #2;
        check((split_done == 1'b0), "split_done asserted prematurely at cycle 2");

        @(posedge clk); // Registered output asserts split_done = 1
        #2;
        check((split_done == 1'b1), "split_done failed to assert on SPLIT_LATENCY cycle");
        $display("  -> Split background latency verified.");

        // -----------------------------------------------------------
        // TEST 4: Master Returns to Fetch Split Data
        // -----------------------------------------------------------
        test_id = 4;
        $display("\n[TEST 4] Testing Return Read to Retrieve Data...");
        
        // Wait 1 extra idle cycle to emulate arbiter re-grant
        @(posedge clk);
        #2;
        
        // Master re-asserts select with original address
        sel = 1; we = 0; addr = 12'h00A;

        @(posedge clk);
        #2;
        check((ready == 1'b1),        "ready failed to assert on split read return");
        check((dout == 8'hA5),        "dout mismatch on split read (expected 0xA5)");
        check((split_req == 1'b0),    "split_req re-asserted on return fetch (illegal)");
        
        sel = 0;
        $display("  -> Data retrieved correctly: 0x%02h", dout);

        // -----------------------------------------------------------
        // TEST 5: Boundary Split Read at 0xFFF (Contains 0x5B)
        // -----------------------------------------------------------
        test_id = 5;
        $display("\n[TEST 5] Testing Boundary Read at 0xFFF...");
        @(posedge clk);
        #2;
        sel = 1; we = 0; addr = 12'hFFF;
        
        @(posedge clk);
        #2;
        check((split_req == 1'b1), "split_req failed on boundary address");
        sel = 0;

        // Wait for split_done event
        @(posedge split_done);
        @(posedge clk);
        #2;
        sel = 1; we = 0; addr = 12'hFFF;

        @(posedge clk);
        #2;
        check((ready == 1'b1), "ready failed on boundary fetch");
        check((dout == 8'h5B), "dout mismatch on boundary read (expected 0x5B)");
        sel = 0;
        $display("  -> Boundary read verified: 0x%02h", dout);

        // -----------------------------------------------------------
        // Final Summary
        // -----------------------------------------------------------
        #40;
        $display("\n=========================================================");
        if (error_count == 0) begin
            $display(">> SLAVE 0 SPLIT TEST SUCCESSFUL: All Cases Passed! <<");
        end else begin
            $display(">> SLAVE 0 SPLIT TEST FAILED: %0d error(s) detected! <<", error_count);
        end
        $display("=========================================================");

        #50;
        $finish;
    end

endmodule