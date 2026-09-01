`timescale 1ns / 1ps



module tb_top_bus_system;

    localparam CLK_PERIOD = 20; // 50 MHz Clock (20ns)



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



    // External Serial Pins

    reg        ext_rx_serial;

    wire       ext_tx_serial;



    integer error_count = 0;



    // ------------------------------------------------------------------------

    // DUT Instantiation

    // ------------------------------------------------------------------------

    top_bus_system uut (

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

        .ext_rx_serial (ext_rx_serial),

        .ext_tx_serial (ext_tx_serial)

    );



    // Clock Generator

    always #(CLK_PERIOD/2) clk = ~clk;



    // ------------------------------------------------------------------------

    // Synchronous Command Driving Tasks (Re-entrant automatic scope)

    // ------------------------------------------------------------------------

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



    // ------------------------------------------------------------------------

    // Main Verification Flow

    // ------------------------------------------------------------------------

    initial begin

        // Signal Initialization

        clk           = 1'b0;

        rst_n         = 1'b0;

        m0_cmd_start = 1'b0; m0_cmd_we = 1'b0; m0_cmd_addr = 14'h0; m0_cmd_wdata = 8'h0;

        m1_cmd_start = 1'b0; m1_cmd_we = 1'b0; m1_cmd_addr = 14'h0; m1_cmd_wdata = 8'h0;

        ext_rx_serial = 1'b1;



        $display("=========================================================");

        $display("   STARTING TOP BUS SYSTEM INTEGRATION VERIFICATION      ");

        $display("=========================================================");



        // Power-On Reset Sequence

        #(CLK_PERIOD * 2);

        rst_n = 1'b1;

        #(CLK_PERIOD);



        // ---------------------------------------------------------------

        // TEST 1: Master 0 Write & Read to Fast RAM (Slave 1 @ 0x1000)

        // ---------------------------------------------------------------

        $display("\n[TEST 1] M0 Fast RAM Write & Read (0x1000)...");

        execute_m0(1'b1, 14'h1000, 8'hC5); // Write 0xC5

        @(posedge m0_cmd_done);



        execute_m0(1'b0, 14'h1000, 8'h00); // Read Back

        @(posedge m0_cmd_done);

        

        if (m0_cmd_rdata == 8'hC5)

            $display("-> SUCCESS: M0 read 0x%02X from 0x1000", m0_cmd_rdata);

        else begin

            $display("-> ERROR: M0 expected 0xC5, got 0x%02X", m0_cmd_rdata);

            error_count = error_count + 1;

        end



        // ---------------------------------------------------------------

        // TEST 2: Master 1 Write & Read to Slave 2 (Fast RAM @ 0x2000)

        // ---------------------------------------------------------------

        $display("\n[TEST 2] M1 Fast RAM Write & Read (0x2000)...");

        execute_m1(1'b1, 14'h2000, 8'h3B); // Write 0x3B

        @(posedge m1_cmd_done);



        execute_m1(1'b0, 14'h2000, 8'h00); // Read Back

        @(posedge m1_cmd_done);

        

        if (m1_cmd_rdata == 8'h3B)

            $display("-> SUCCESS: M1 read 0x%02X from 0x2000", m1_cmd_rdata);

        else begin

            $display("-> ERROR: M1 expected 0x3B, got 0x%02X", m1_cmd_rdata);

            error_count = error_count + 1;

        end



        // ---------------------------------------------------------------

        // TEST 3: External Bridge Register Access (0x3000)

        // ---------------------------------------------------------------

        $display("\n[TEST 3] M0 Bridge Reg Read Default (0x3000)...");

        execute_m0(1'b0, 14'h3000, 8'h00);

        @(posedge m0_cmd_done);

        

        if (m0_cmd_rdata == 8'hBE)

            $display("-> SUCCESS: M0 read default 0xBE from Bridge Reg");

        else begin

            $display("-> ERROR: Expected 0xBE, got 0x%02X", m0_cmd_rdata);

            error_count = error_count + 1;

        end



        // ---------------------------------------------------------------

        // TEST 4: Split Transaction & Interleaved Arbitrated Access

        // ---------------------------------------------------------------

        $display("\n[TEST 4] Split Read & Arbitration Interleaving...");

        

        // Step 1: M0 Write 0x7E to Slave 0

        execute_m0(1'b1, 14'h0010, 8'h7E);

        @(posedge m0_cmd_done);



        // Step 2: M0 Starts Split Read

        execute_m0(1'b0, 14'h0010, 8'h00);

        

        // Step 3: Trigger M1 read while M0 is stalled in split state

        #(CLK_PERIOD);

        execute_m1(1'b0, 14'h1000, 8'h00);



        // Step 4: Evaluate Parallel Completion via Fork-Join

        fork

            begin

                @(posedge m0_cmd_done);

                if (m0_cmd_rdata == 8'h7E)

                    $display("-> SUCCESS: M0 Split Read completed with 0x7E");

                else begin

                    $display("-> ERROR: M0 Split Read expected 0x7E, got 0x%02X", m0_cmd_rdata);

                    error_count = error_count + 1;

                end

            end

            begin

                @(posedge m1_cmd_done);

                if (m1_cmd_rdata == 8'hC5)

                    $display("-> SUCCESS: M1 serviced during M0 split delay with 0xC5");

                else begin

                    $display("-> ERROR: M1 interleaved read expected 0xC5, got 0x%02X", m1_cmd_rdata);

                    error_count = error_count + 1;

                end

            end

        join



        #(CLK_PERIOD * 5);

        $display("=========================================================");

        if (error_count == 0)

            $display(">> TOP SYSTEM TEST PASSED: All Interconnect Operations Correct! <<");

        else

            $display(">> TOP SYSTEM TEST FAILED: %0d error(s) detected! <<", error_count);

        $display("=========================================================");

        $finish;

    end



endmodule