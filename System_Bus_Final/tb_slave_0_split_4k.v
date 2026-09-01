`timescale 1ns / 1ps

module tb_slave_0_split_4k;
    localparam CLK_PERIOD = 20; // 50 MHz

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

    always #(CLK_PERIOD/2) clk = ~clk;

    task check(input condition, input [255:0] msg);
        begin
            if (!condition) begin
                $display("[ERROR @ %0t ns] Test %0d: %s", $time, test_id, msg);
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        clk   = 0;
        rst_n = 0;
        sel   = 0;
        we    = 0;
        addr  = 12'h000;
        din   = 8'h00;

        $display("=========================================================");
        $display("   STARTING SLAVE 0 SPLIT RAM UNIT VERIFICATION         ");
        $display("=========================================================");

        // TEST 1: Reset Behavior
        test_id = 1;
        #25;
        check((ready == 1'b0 && split_req == 1'b0 && split_done == 1'b0), "Outputs not zero during reset");
        @(posedge clk); #2;
        rst_n = 1;
        $display("[TEST 1] Reset Behavior: PASSED");

        // TEST 2: Single-Cycle Write Access
        test_id = 2;
        @(posedge clk); #2;
        sel = 1; we = 1; addr = 12'h040; din = 8'h5A;
        @(posedge clk); #2;
        check((ready == 1'b1 && dout == 8'h5A), "Write response/ready failed");
        sel = 0; we = 0;
        $display("[TEST 2] Write Access: PASSED");

        // TEST 3: Split Read Phase 1 - Requesting Split
        test_id = 3;
        @(posedge clk); #2;
        sel = 1; we = 0; addr = 12'h040; // Initiates Split
        @(posedge clk); #2;
        check((split_req == 1'b1), "split_req was not asserted on initial read access");
        
        // Master yields bus on split_req
        sel = 0;
        
        // TEST 4: Wait for Slave internal latency and split_done pulse
        test_id = 4;
        wait(split_done == 1'b1);
        check((split_done == 1'b1), "split_done failed to pulse");
        $display("[TEST 4] Split Internal Processing Complete (split_done asserted)");

        // TEST 5: Split Read Phase 2 - Re-access to fetch data
        test_id = 5;
        @(posedge clk); #2;
        sel = 1; we = 0; addr = 12'h040; // Master re-acquires and requests same address
        @(posedge clk); #2;
        check((ready == 1'b1 && dout == 8'h5A), "Read data mismatch or ready not asserted on fetch");
        sel = 0;
        $display("[TEST 5] Split Data Fetch: PASSED (Data = 0x5A)");

        #50;
        $display("=========================================================");
        if (error_count == 0)
            $display(">> SLAVE 0 SPLIT TEST SUCCESSFUL: All Cases Passed! <<");
        else
            $display(">> SLAVE 0 SPLIT TEST FAILED: %0d error(s)! <<", error_count);
        $display("=========================================================");
        #50;
        $finish;
    end
endmodule