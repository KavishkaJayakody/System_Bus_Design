`timescale 1ns / 1ps

module tb_slave_fast_ram;

    localparam CLK_PERIOD = 20; // 50 MHz

    reg         clk;
    reg         rst_n;
    reg         sel;
    reg         we;
    reg  [11:0] addr;
    reg  [7:0]  din;
    wire [7:0]  dout;
    wire        ready;

    integer error_count = 0;
    integer test_id = 0;

    // Instantiate 4K Instance (Slave 1)
    slave_fast_ram #(.ADDR_WIDTH(12)) uut (
        .clk(clk),
        .rst_n(rst_n),
        .sel(sel),
        .we(we),
        .addr(addr),
        .din(din),
        .dout(dout),
        .ready(ready)
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
        $display("   STARTING FAST RAM UNIT VERIFICATION (50 MHz)          ");
        $display("=========================================================");

        // TEST 1: Reset
        test_id = 1;
        #25;
        check((ready == 1'b0 && dout == 8'h00), "Outputs not 0 on reset");
        @(posedge clk); #2;
        rst_n = 1;
        $display("[TEST 1] Reset Behavior: PASSED");

        // TEST 2: Fast Single-Cycle Write & Read Back (Base Address)
        test_id = 2;
        @(posedge clk); #2;
        sel = 1; we = 1; addr = 12'h000; din = 8'hA1;
        @(posedge clk); #2;
        check((ready == 1'b1), "Ready not asserted on write");
        
        // Read back
        we = 0; addr = 12'h000;
        @(posedge clk); #2;
        check((ready == 1'b1 && dout == 8'hA1), "Read data mismatch at 0x000");
        sel = 0;
        $display("[TEST 2] Base Address Write/Read: PASSED (Data = 0xA1)");

        // TEST 3: Upper Boundary Access (0xFFF for 4K)
        test_id = 3;
        @(posedge clk); #2;
        sel = 1; we = 1; addr = 12'hFFF; din = 8'hD4;
        @(posedge clk); #2;
        we = 0;
        @(posedge clk); #2;
        check((ready == 1'b1 && dout == 8'hD4), "Read data mismatch at boundary 0xFFF");
        sel = 0;
        $display("[TEST 3] Boundary Address Write/Read: PASSED (Data = 0xD4)");

        #40;
        $display("=========================================================");
        if (error_count == 0)
            $display(">> FAST RAM TEST SUCCESSFUL: All Cases Passed! <<");
        else
            $display(">> FAST RAM TEST FAILED: %0d error(s)! <<", error_count);
        $display("=========================================================");
        #50;
        $finish;
    end

endmodule