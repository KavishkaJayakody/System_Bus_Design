`timescale 1ns / 1ps

module tb_top_bus_system;
    localparam CLK_PERIOD = 20; // 50 MHz Clock (20ns)

    // Address space parameters
    localparam SLAVE0_BASE = 14'h0000; // Split 4K RAM
    localparam SLAVE0_SIZE = 4096;
    localparam SLAVE1_BASE = 14'h1000; // Fast 4K RAM
    localparam SLAVE1_SIZE = 4096;
    localparam SLAVE2_BASE = 14'h2000; // Fast 2K RAM
    localparam SLAVE2_SIZE = 2048;
    localparam BRIDGE_ADDR = 14'h3000; // External bridge register

    // Test Configuration
    localparam NUM_RANDOM_TRIALS = 100; // Number of randomized runs per test case

    reg        clk;
    reg        rst_n;

    // Master 0 Interface
    reg        m0_cmd_start;
    reg        m0_cmd_we;
    reg [13:0] m0_cmd_addr;
    reg [7:0]  m0_cmd_wdata;
    wire [7:0] m0_cmd_rdata;
    wire       m0_cmd_done;

    // Master 1 Interface
    reg        m1_cmd_start;
    reg        m1_cmd_we;
    reg [13:0] m1_cmd_addr;
    reg [7:0]  m1_cmd_wdata;
    wire [7:0] m1_cmd_rdata;
    wire       m1_cmd_done;

    // External Serial Pin (UART transmit, slave 3)
    wire       ext_tx_serial;

    integer error_count = 0;
    integer i;

    // DUT Instantiation
    top_bus_system #(.CLKS_PER_BIT(4)) uut (
        .clk           (clk),
        .rst_n         (rst_n),
        .m0_cmd_start (m0_cmd_start),
        .m0_cmd_we    (m0_cmd_we),
        .m0_cmd_addr  (m0_cmd_addr),
        .m0_cmd_wdata (m0_cmd_wdata),
        .m0_cmd_rdata (m0_cmd_rdata),
        .m0_cmd_done  (m0_cmd_done),
        .m1_cmd_start (m1_cmd_start),
        .m1_cmd_we    (m1_cmd_we),
        .m1_cmd_addr  (m1_cmd_addr),
        .m1_cmd_wdata (m1_cmd_wdata),
        .m1_cmd_rdata (m1_cmd_rdata),
        .m1_cmd_done  (m1_cmd_done),
        .ext_tx_serial (ext_tx_serial)
    );

    // Clock Generator
    always #(CLK_PERIOD/2) clk = ~clk;

    // Synchronous Command Driving Tasks
    task automatic execute_m0(input we, input [13:0] addr, input [7:0] wdata);
        begin
            @(posedge clk);
            m0_cmd_we    <= we;
            m0_cmd_addr  <= addr;
            m0_cmd_wdata <= wdata;
            m0_cmd_start <= 1'b1;

            @(posedge clk);
            m0_cmd_start <= 1'b0;
        end
    endtask

    task automatic execute_m1(input we, input [13:0] addr, input [7:0] wdata);
        begin
            @(posedge clk);
            m1_cmd_we    <= we;
            m1_cmd_addr  <= addr;
            m1_cmd_wdata <= wdata;
            m1_cmd_start <= 1'b1;

            @(posedge clk);
            m1_cmd_start <= 1'b0;
        end
    endtask

    // Main Verification Flow
    initial begin
        clk           = 1'b0;
        rst_n         = 1'b0;
        m0_cmd_start = 1'b0; m0_cmd_we = 1'b0; m0_cmd_addr = 14'h0; m0_cmd_wdata = 8'h0;
        m1_cmd_start = 1'b0; m1_cmd_we = 1'b0; m1_cmd_addr = 14'h0; m1_cmd_wdata = 8'h0;

        $display("=========================================================");
        $display("   STARTING TOP BUS SYSTEM INTEGRATION VERIFICATION      ");
        $display("=========================================================");

        // Power-On Reset Sequence
        #(CLK_PERIOD * 2);
        rst_n = 1'b1;
        #(CLK_PERIOD);

        // ---------------------------------------------------------------
        // TEST 1: Randomized Master 0 Writes & Reads on Fast RAM (Slave 1)
        // ---------------------------------------------------------------
        $display("\n[TEST 1] M0 Fast RAM Write & Read (%0d Randomized Trials)...", NUM_RANDOM_TRIALS);
        for (i = 0; i < NUM_RANDOM_TRIALS; i = i + 1) begin : test1_loop
            reg [13:0] rand_addr;
            reg [7:0]  rand_wdata;

            rand_addr  = SLAVE1_BASE + ($random & (SLAVE1_SIZE - 1));
            rand_wdata = $random & 8'hFF;

            execute_m0(1'b1, rand_addr, rand_wdata);
            @(posedge m0_cmd_done);

            execute_m0(1'b0, rand_addr, 8'h00);
            @(posedge m0_cmd_done);

            if (m0_cmd_rdata !== rand_wdata) begin
                $display("-> ERROR [Test 1 Iter %0d]: M0 addr 0x%04X expected 0x%02X, got 0x%02X",
                         i, rand_addr, rand_wdata, m0_cmd_rdata);
                error_count = error_count + 1;
            end
        end

        // ---------------------------------------------------------------
        // TEST 2: Randomized Master 1 Writes & Reads on Fast RAM (Slave 2)
        // ---------------------------------------------------------------
        $display("\n[TEST 2] M1 Fast RAM Write & Read (%0d Randomized Trials)...", NUM_RANDOM_TRIALS);
        for (i = 0; i < NUM_RANDOM_TRIALS; i = i + 1) begin : test2_loop
            reg [13:0] rand_addr;
            reg [7:0]  rand_wdata;

            rand_addr  = SLAVE2_BASE + ($random & (SLAVE2_SIZE - 1));
            rand_wdata = $random & 8'hFF;

            execute_m1(1'b1, rand_addr, rand_wdata);
            @(posedge m1_cmd_done);

            execute_m1(1'b0, rand_addr, 8'h00);
            @(posedge m1_cmd_done);

            if (m1_cmd_rdata !== rand_wdata) begin
                $display("-> ERROR [Test 2 Iter %0d]: M1 addr 0x%04X expected 0x%02X, got 0x%02X",
                         i, rand_addr, rand_wdata, m1_cmd_rdata);
                error_count = error_count + 1;
            end
        end

        // ---------------------------------------------------------------
        // TEST 3: Randomized UART TX Staging Registers (Slave 3)
        // ---------------------------------------------------------------
        // slave_uart_tx decodes only addr[1:0], so every 4-byte block in
        // 0x3000-0x3FFF aliases onto the same three staging registers. The
        // random upper offset below is deliberate: it checks that aliasing.
        $display("\n[TEST 3] UART TX Staging Registers (%0d Randomized Trials)...", NUM_RANDOM_TRIALS);
        for (i = 0; i < NUM_RANDOM_TRIALS; i = i + 1) begin : test3_loop
            reg [13:0] blk;
            reg [7:0]  b0, b1, b2;

            blk = BRIDGE_ADDR + (($random & 12'hFFF) & ~14'h3);
            b0  = $random & 8'hFF;
            b1  = $random & 8'hFF;
            b2  = $random & 8'hFF;

            execute_m0(1'b1, blk + 14'd0, b0);
            @(posedge m0_cmd_done);
            execute_m0(1'b1, blk + 14'd1, b1);
            @(posedge m0_cmd_done);
            execute_m0(1'b1, blk + 14'd2, b2);
            @(posedge m0_cmd_done);

            execute_m0(1'b0, blk + 14'd0, 8'h00);
            @(posedge m0_cmd_done);
            if (m0_cmd_rdata !== b0) begin
                $display("-> ERROR [Test 3 Iter %0d]: byte0 @ 0x%04X expected 0x%02X, got 0x%02X",
                         i, blk, b0, m0_cmd_rdata);
                error_count = error_count + 1;
            end

            execute_m0(1'b0, blk + 14'd1, 8'h00);
            @(posedge m0_cmd_done);
            if (m0_cmd_rdata !== b1) begin
                $display("-> ERROR [Test 3 Iter %0d]: byte1 @ 0x%04X expected 0x%02X, got 0x%02X",
                         i, blk, b1, m0_cmd_rdata);
                error_count = error_count + 1;
            end

            execute_m0(1'b0, blk + 14'd2, 8'h00);
            @(posedge m0_cmd_done);
            if (m0_cmd_rdata !== b2) begin
                $display("-> ERROR [Test 3 Iter %0d]: byte2 @ 0x%04X expected 0x%02X, got 0x%02X",
                         i, blk, b2, m0_cmd_rdata);
                error_count = error_count + 1;
            end
        end

        // ---------------------------------------------------------------
        // TEST 4: Randomized Split Transaction & Bus Interleaving
        // ---------------------------------------------------------------
        $display("\n[TEST 4] Split RAM Interleaved Operations (%0d Randomized Trials)...", NUM_RANDOM_TRIALS);
        for (i = 0; i < NUM_RANDOM_TRIALS; i = i + 1) begin : test4_loop
            reg [13:0] s0_addr, s1_addr;
            reg [7:0]  s0_wdata, s1_wdata;

            // Generate random target addresses for Slave 0 (Split) and Slave 1 (Fast)
            s0_addr  = SLAVE0_BASE + ($random & (SLAVE0_SIZE - 1));
            s1_addr  = SLAVE1_BASE + ($random & (SLAVE1_SIZE - 1));
            s0_wdata = $random & 8'hFF;
            s1_wdata = $random & 8'hFF;

            // 1. Seed data into Slave 1 (via M1) and Slave 0 (via M0)
            execute_m1(1'b1, s1_addr, s1_wdata);
            @(posedge m1_cmd_done);

            execute_m0(1'b1, s0_addr, s0_wdata);
            @(posedge m0_cmd_done);

            // 2. M0 starts split read on Slave 0
            execute_m0(1'b0, s0_addr, 8'h00);

            // 3. Interleave M1 read to Slave 1 while M0 is stalled in split state
            #(CLK_PERIOD);
            execute_m1(1'b0, s1_addr, 8'h00);

            // 4. Assert parallel completion and match expectations
            fork
                begin
                    @(posedge m0_cmd_done);
                    if (m0_cmd_rdata !== s0_wdata) begin
                        $display("-> ERROR [Test 4 Iter %0d]: M0 Split Read @ 0x%04X expected 0x%02X, got 0x%02X",
                                 i, s0_addr, s0_wdata, m0_cmd_rdata);
                        error_count = error_count + 1;
                    end
                end
                begin
                    @(posedge m1_cmd_done);
                    if (m1_cmd_rdata !== s1_wdata) begin
                        $display("-> ERROR [Test 4 Iter %0d]: M1 Interleaved Read @ 0x%04X expected 0x%02X, got 0x%02X",
                                 i, s1_addr, s1_wdata, m1_cmd_rdata);
                        error_count = error_count + 1;
                    end
                end
            join
        end

        #(CLK_PERIOD * 5);
        $display("\n=========================================================");
        if (error_count == 0)
            $display(">> TOP SYSTEM TEST PASSED: All Interconnect Operations Correct! <<");
        else
            $display(">> TOP SYSTEM TEST FAILED: %0d error(s) detected! <<", error_count);
        $display("=========================================================");
        $finish;
    end

endmodule