`timescale 1ns / 1ps

module tb_top_bus_system;

    localparam CLK_PERIOD = 20; // 50 MHz (10 ns HIGH / 10 ns LOW)

    reg         clk;
    reg         rst_n;

    // Master 0 Interface
    reg         m0_cmd_start;
    reg         m0_cmd_we;
    reg  [13:0] m0_cmd_addr;
    reg  [7:0]  m0_cmd_wdata;
    wire [7:0]  m0_cmd_rdata;
    wire        m0_cmd_done;

    // Master 1 Interface
    reg         m1_cmd_start;
    reg         m1_cmd_we;
    reg  [13:0] m1_cmd_addr;
    reg  [7:0]  m1_cmd_wdata;
    wire [7:0]  m1_cmd_rdata;
    wire        m1_cmd_done;

    // Bridge GPIOs
    reg         ext_rx_serial;
    wire        ext_tx_serial;

    integer error_count = 0;
    integer test_id = 0;

    // Top-Level Unit Under Test
    top_bus_system uut (
        .clk(clk),
        .rst_n(rst_n),
        .m0_cmd_start(m0_cmd_start),
        .m0_cmd_we(m0_cmd_we),
        .m0_cmd_addr(m0_cmd_addr),
        .m0_cmd_wdata(m0_cmd_wdata),
        .m0_cmd_rdata(m0_cmd_rdata),
        .m0_cmd_done(m0_cmd_done),
        .m1_cmd_start(m1_cmd_start),
        .m1_cmd_we(m1_cmd_we),
        .m1_cmd_addr(m1_cmd_addr),
        .m1_cmd_wdata(m1_cmd_wdata),
        .m1_cmd_rdata(m1_cmd_rdata),
        .m1_cmd_done(m1_cmd_done),
        .ext_rx_serial(ext_rx_serial),
        .ext_tx_serial(ext_tx_serial)
    );

    // 50 MHz Clock Generator
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Assertion Check Task
    task check(input condition, input [255:0] msg);
        begin
            if (!condition) begin
                $display("  [ERROR @ %0t ns] Test %0d: %s", $time, test_id, msg);
                error_count = error_count + 1;
            end
        end
    endtask

    // Safe Non-Hanging M0 Command Task
    task m0_write(input [13:0] addr, input [7:0] data);
        integer timeout;
        begin
            @(posedge clk); #2;
            m0_cmd_addr  = addr;
            m0_cmd_wdata = data;
            m0_cmd_we    = 1'b1;
            m0_cmd_start = 1'b1;
            @(posedge clk); #2;
            m0_cmd_start = 1'b0;

            timeout = 0;
            while (!m0_cmd_done && timeout < 50) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 50) begin
                $display("  [TIMEOUT] M0 Write to 0x%04h hung!", addr);
                error_count = error_count + 1;
            end
            // Allow 1 clean clock cycle in IDLE state before next command
            @(posedge clk); #2;
        end
    endtask

    task m0_read(input [13:0] addr, output [7:0] rdata);
        integer timeout;
        begin
            @(posedge clk); #2;
            m0_cmd_addr  = addr;
            m0_cmd_we    = 1'b0;
            m0_cmd_start = 1'b1;
            @(posedge clk); #2;
            m0_cmd_start = 1'b0;

            timeout = 0;
            while (!m0_cmd_done && timeout < 50) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 50) begin
                $display("  [TIMEOUT] M0 Read from 0x%04h hung!", addr);
                error_count = error_count + 1;
            end
            rdata = m0_cmd_rdata;
            @(posedge clk); #2;
        end
    endtask

    // Safe Non-Hanging M1 Command Task
    task m1_write(input [13:0] addr, input [7:0] data);
        integer timeout;
        begin
            @(posedge clk); #2;
            m1_cmd_addr  = addr;
            m1_cmd_wdata = data;
            m1_cmd_we    = 1'b1;
            m1_cmd_start = 1'b1;
            @(posedge clk); #2;
            m1_cmd_start = 1'b0;

            timeout = 0;
            while (!m1_cmd_done && timeout < 50) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 50) begin
                $display("  [TIMEOUT] M1 Write to 0x%04h hung!", addr);
                error_count = error_count + 1;
            end
            @(posedge clk); #2;
        end
    endtask

    task m1_read(input [13:0] addr, output [7:0] rdata);
        integer timeout;
        begin
            @(posedge clk); #2;
            m1_cmd_addr  = addr;
            m1_cmd_we    = 1'b0;
            m1_cmd_start = 1'b1;
            @(posedge clk); #2;
            m1_cmd_start = 1'b0;

            timeout = 0;
            while (!m1_cmd_done && timeout < 50) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 50) begin
                $display("  [TIMEOUT] M1 Read from 0x%04h hung!", addr);
                error_count = error_count + 1;
            end
            rdata = m1_cmd_rdata;
            @(posedge clk); #2;
        end
    endtask

    reg [7:0] read_val;

    initial begin
        // Initialize Inputs
        clk           = 0;
        rst_n         = 0;
        m0_cmd_start  = 0;
        m0_cmd_we     = 0;
        m0_cmd_addr   = 14'h0000;
        m0_cmd_wdata  = 8'h00;
        m1_cmd_start  = 0;
        m1_cmd_we     = 0;
        m1_cmd_addr   = 14'h0000;
        m1_cmd_wdata  = 8'h00;
        ext_rx_serial = 1'b1;

        $display("=========================================================");
        $display("   TOP LEVEL VERIFICATION TEST (50 MHz)                  ");
        $display("=========================================================");

        // -----------------------------------------------------------
        // TEST 1: Reset Test
        // -----------------------------------------------------------
        test_id = 1;
        $display("\n[TEST 1] Testing Global Reset...");
        #35;
        check((m0_cmd_done == 0 && m1_cmd_done == 0), "cmd_done asserted during reset");
        check((uut.gnt0 == 0 && uut.gnt1 == 0),       "Grants asserted during reset");
        @(posedge clk); #2;
        rst_n = 1;
        $display("  -> Reset: PASSED");

        // -----------------------------------------------------------
        // TEST 2: Master 0 Read/Write to Fast Slaves
        // -----------------------------------------------------------
        test_id = 2;
        $display("\n[TEST 2] Master 0 Fast Slave Access (S1, S2, Bridge)...");
        // S1 (4K Fast RAM: 0x1000 - 0x1FFF)
        m0_write(14'h1020, 8'hA5);
        m0_read(14'h1020, read_val);
        check((read_val == 8'hA5), "S1 Read mismatch (expected 0xA5)");
        $display("  -> Slave 1 (4K Fast): Read match (0x%02h)", read_val);

        // S2 (2K Fast RAM: 0x2000 - 0x27FF)
        m0_write(14'h2100, 8'h7C);
        m0_read(14'h2100, read_val);
        check((read_val == 8'h7C), "S2 Read mismatch (expected 0x7C)");
        $display("  -> Slave 2 (2K Fast): Read match (0x%02h)", read_val);

        // Bridge Dummy (0x3000 - 0x3FFF)
        m0_write(14'h3010, 8'hBE);
        m0_read(14'h3010, read_val);
        check((read_val == 8'hBE), "Bridge Read mismatch (expected 0xBE)");
        $display("  -> Bridge Dummy: Read match (0x%02h)", read_val);

        // -----------------------------------------------------------
        // TEST 3: Master 1 Standalone Transfers (S1 & S2)
        // -----------------------------------------------------------
        test_id = 3;
        $display("\n[TEST 3] Master 1 Standalone Access to S1 and S2...");
        m1_write(14'h1500, 8'h33);
        m1_read(14'h1500, read_val);
        check((read_val == 8'h33), "M1 S1 Read mismatch (expected 0x33)");
        $display("  -> Master 1 S1: Read match (0x%02h)", read_val);

        m1_write(14'h2200, 8'h44);
        m1_read(14'h2200, read_val);
        check((read_val == 8'h44), "M1 S2 Read mismatch (expected 0x44)");
        $display("  -> Master 1 S2: Read match (0x%02h)", read_val);

        // -----------------------------------------------------------
        // TEST 4: Simultaneous Bus Contention (M0 High vs M1 Low)
        // -----------------------------------------------------------
        test_id = 4;
        $display("\n[TEST 4] Simultaneous Dual-Master Contention...");
        @(posedge clk); #2;
        m0_cmd_addr  = 14'h1050; m0_cmd_wdata = 8'h11; m0_cmd_we = 1'b1; m0_cmd_start = 1'b1;
        m1_cmd_addr  = 14'h2050; m1_cmd_wdata = 8'h22; m1_cmd_we = 1'b1; m1_cmd_start = 1'b1;
        @(posedge clk); #2;
        m0_cmd_start = 1'b0;
        m1_cmd_start = 1'b0;

        @(posedge m0_cmd_done);
        check((m1_cmd_done == 1'b0), "Priority violation: M1 finished before M0!");
        $display("  -> Contention Phase 1: M0 granted and completed first.");

        @(posedge m1_cmd_done);
        $display("  -> Contention Phase 2: M1 completed immediately after.");

        // Read back to ensure no data corruption
        m0_read(14'h1050, read_val); check((read_val == 8'h11), "M0 write corrupted during contention");
        m0_read(14'h2050, read_val); check((read_val == 8'h22), "M1 write corrupted during contention");
        $display("  -> Contention Data Integrity: PASSED");

        // -----------------------------------------------------------
        // TEST 5: Slave 0 Split Read Transaction with Interleaved M1
        // -----------------------------------------------------------
        test_id = 5;
        $display("\n[TEST 5] Slave 0 (4K Split) Read with Interleaved Master 1...");
        
        // 5a. Write known data to S0 (single-cycle fast write)
        m0_write(14'h0040, 8'h99);

        // 5b. Master 0 issues Read to S0 (triggers split back-off)
        @(posedge clk); #2;
        m0_cmd_addr  = 14'h0040;
        m0_cmd_we    = 1'b0;
        m0_cmd_start = 1'b1;
        @(posedge clk); #2;
        m0_cmd_start = 1'b0;

        // Give 1 cycle for M0 to enter DRIVE and receive split back-off
        @(posedge clk); #2;

        // 5c. Master 1 accesses S1 while M0 is waiting for split completion
        m1_cmd_addr  = 14'h1080;
        m1_cmd_wdata = 8'h66;
        m1_cmd_we    = 1'b1;
        m1_cmd_start = 1'b1;
        @(posedge clk); #2;
        m1_cmd_start = 1'b0;

        @(posedge m1_cmd_done);
        check((m0_cmd_done == 1'b0), "M0 finished before S0 latency timer expired!");
        $display("  -> [SPLIT SUCCESS] M1 executed transfer during M0 split back-off!");

        // 5d. Master 0 completes its read once S0 completes in background
        @(posedge m0_cmd_done);
        check((m0_cmd_rdata == 8'h99), "Slave 0 Split Read Data Mismatch (Expected 0x99)");
        $display("  -> Slave 0 Split Read successfully captured: 0x%02h", m0_cmd_rdata);

        // -----------------------------------------------------------
        // Final Summary
        // -----------------------------------------------------------
        #40;
        $display("\n=========================================================");
        if (error_count == 0) begin
            $display(">> TOP LEVEL INTEGRATION SUCCESSFUL: All 5 Tests Passed! <<");
        end else begin
            $display(">> TOP LEVEL INTEGRATION FAILED: %0d error(s) detected! <<", error_count);
        end
        $display("=========================================================");

        #50;
        $finish;
    end

endmodule