`timescale 1ns / 1ps
//
// tb_uart_remote -- two complete bus systems wired together by a crossed UART
// link, exercising remote writes and remote reads in both roles.
//
//    A.ext_tx_serial ---> B.ext_rx_serial
//    B.ext_tx_serial ---> A.ext_rx_serial
//
module tb_uart_remote;

    localparam CPB = 4;      // short bit period for simulation
    localparam TMO = 3000;   // remote reply timeout, clocks

    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    reg link_up = 1'b1;      // drop to simulate an unplugged cable

    // ---- board A ----
    reg        a_start=0, a_we=0, a_rem=0; reg [13:0] a_addr=0; reg [7:0] a_wdata=0;
    wire [7:0] a_rdata; wire a_done, a_err, a_tx;
    // ---- board B ----
    reg        b_start=0, b_we=0, b_rem=0; reg [13:0] b_addr=0; reg [7:0] b_wdata=0;
    wire [7:0] b_rdata; wire b_done, b_err, b_tx;

    wire a_rx = link_up ? b_tx : 1'b1;
    wire b_rx = link_up ? a_tx : 1'b1;

    integer errors = 0;

    top_bus_system #(.CLKS_PER_BIT(CPB), .RESP_TIMEOUT(TMO)) A (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(a_start), .m0_cmd_we(a_we), .m0_cmd_remote(a_rem),
        .m0_cmd_addr(a_addr), .m0_cmd_wdata(a_wdata),
        .m0_cmd_rdata(a_rdata), .m0_cmd_done(a_done), .m0_cmd_error(a_err),
        .m1_cmd_start(1'b0), .m1_cmd_we(1'b0), .m1_cmd_addr(14'h0),
        .m1_cmd_wdata(8'h0), .m1_cmd_rdata(), .m1_cmd_done(),
        .ext_rx_serial(a_rx), .ext_tx_serial(a_tx)
    );

    top_bus_system #(.CLKS_PER_BIT(CPB), .RESP_TIMEOUT(TMO)) B (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(b_start), .m0_cmd_we(b_we), .m0_cmd_remote(b_rem),
        .m0_cmd_addr(b_addr), .m0_cmd_wdata(b_wdata),
        .m0_cmd_rdata(b_rdata), .m0_cmd_done(b_done), .m0_cmd_error(b_err),
        .m1_cmd_start(1'b0), .m1_cmd_we(1'b0), .m1_cmd_addr(14'h0),
        .m1_cmd_wdata(8'h0), .m1_cmd_rdata(), .m1_cmd_done(),
        .ext_rx_serial(b_rx), .ext_tx_serial(b_tx)
    );

    integer n;

    task a_cmd(input we, input rem, input [13:0] ad, input [7:0] d);
        begin
            @(posedge clk); #2;
            a_we=we; a_rem=rem; a_addr=ad; a_wdata=d; a_start=1'b1;
            @(posedge clk); #2; a_start=1'b0;
            n=0; while (!a_done && n<200000) begin @(posedge clk); n=n+1; end
            @(posedge clk);
        end
    endtask

    task b_cmd(input we, input rem, input [13:0] ad, input [7:0] d);
        begin
            @(posedge clk); #2;
            b_we=we; b_rem=rem; b_addr=ad; b_wdata=d; b_start=1'b1;
            @(posedge clk); #2; b_start=1'b0;
            n=0; while (!b_done && n<200000) begin @(posedge clk); n=n+1; end
            @(posedge clk);
        end
    endtask

    task expect(input [7:0] got, input [7:0] want, input [8*40:1] what);
        begin
            if (got === want) $display("-> SUCCESS: %0s = 0x%02X", what, got);
            else begin
                $display("-> ERROR:   %0s expected 0x%02X, got 0x%02X", what, want, got);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $display("\n=========================================================");
        $display("   TWO-BOARD REMOTE ACCESS OVER UART");
        $display("=========================================================");
        #40 rst_n = 1; #60;

        $display("\n[1] Seed each board's own 0x1000 locally...");
        a_cmd(1'b1, 1'b0, 14'h1000, 8'h11);
        b_cmd(1'b1, 1'b0, 14'h1000, 8'h22);
        a_cmd(1'b0, 1'b0, 14'h1000, 8'h00); expect(a_rdata, 8'h11, "A local 0x1000");
        b_cmd(1'b0, 1'b0, 14'h1000, 8'h00); expect(b_rdata, 8'h22, "B local 0x1000");

        $display("\n[2] A performs a REMOTE WRITE of 0x5A to B's 0x1000...");
        a_cmd(1'b1, 1'b1, 14'h1000, 8'h5A);
        if (a_err) begin
            $display("-> ERROR:   remote write timed out");
            errors = errors + 1;
        end else
            $display("-> SUCCESS: remote write acknowledged by B");

        $display("\n[3] B reads its own 0x1000 - did A's write land there?");
        b_cmd(1'b0, 1'b0, 14'h1000, 8'h00); expect(b_rdata, 8'h5A, "B local 0x1000");

        $display("\n[4] A's own 0x1000 must be untouched...");
        a_cmd(1'b0, 1'b0, 14'h1000, 8'h00); expect(a_rdata, 8'h11, "A local 0x1000");

        $display("\n[5] A performs a REMOTE READ of B's 0x1000...");
        a_cmd(1'b0, 1'b1, 14'h1000, 8'h00);
        if (a_err) begin
            $display("-> ERROR:   remote read timed out");
            errors = errors + 1;
        end else
            expect(a_rdata, 8'h5A, "A remote read of B 0x1000");

        $display("\n[6] Reverse direction: B remote-reads A's slave 2 (0x2000)...");
        a_cmd(1'b1, 1'b0, 14'h2000, 8'hC7);
        b_cmd(1'b0, 1'b1, 14'h2000, 8'h00);
        if (b_err) begin
            $display("-> ERROR:   reverse remote read timed out");
            errors = errors + 1;
        end else
            expect(b_rdata, 8'hC7, "B remote read of A 0x2000");

        $display("\n[7] A remote-reads B's SPLIT slave (0x0010)...");
        b_cmd(1'b1, 1'b0, 14'h0010, 8'h7E);
        a_cmd(1'b0, 1'b1, 14'h0010, 8'h00);
        if (a_err) begin
            $display("-> ERROR:   remote split read timed out");
            errors = errors + 1;
        end else
            expect(a_rdata, 8'h7E, "A remote read of B split slave");

        $display("\n[8] Unplug the link - a remote command must time out, not hang...");
        link_up = 1'b0;
        a_cmd(1'b0, 1'b1, 14'h1000, 8'h00);
        if (a_err)
            $display("-> SUCCESS: remote read reported cmd_error instead of hanging");
        else begin
            $display("-> ERROR:   expected a timeout, got rdata=0x%02X", a_rdata);
            errors = errors + 1;
        end

        $display("\n[9] Local access still works with the link down...");
        link_up = 1'b1;
        a_cmd(1'b0, 1'b0, 14'h1000, 8'h00); expect(a_rdata, 8'h11, "A local 0x1000");

        $display("\n=========================================================");
        if (errors == 0)
            $display(">> REMOTE ACCESS TEST PASSED: both boards read/write each other <<");
        else
            $display(">> REMOTE ACCESS TEST FAILED: %0d error(s) <<", errors);
        $display("=========================================================\n");
        $finish;
    end

endmodule
