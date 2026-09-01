`timescale 1ns / 1ps
//
// tb_uart_link -- end-to-end check of the UART command path.
//
// ext_tx_serial (slave 3) is looped straight back into uart_cmd_inject's
// receiver, so this one system plays both ends of the link: it stages a
// 24-bit command in slave 3, transmits it, receives it, and injects it into
// its own Master 0. On a real pair of boards the loop is a wire between
// GPIO_0[1] of one and GPIO_0[0] of the other.
//
module tb_uart_link;

    localparam CPB = 4;   // short bit period: keeps the sim to a few thousand cycles

    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    // JTAG-side command (what bus_issp_driver would drive)
    reg        j0_start = 0, j0_we = 0;
    reg [13:0] j0_addr  = 0;
    reg [7:0]  j0_wdata = 0;

    wire        m0_start, m0_we;
    wire [13:0] m0_addr;
    wire [7:0]  m0_wdata, m0_rdata;
    wire        m0_done, uart_cmd_stb;
    wire [1:0]  rx_idx;

    wire serial;               // slave 3 TX, looped back to the receiver
    integer errors = 0;

    uart_cmd_inject #(.CLKS_PER_BIT(CPB)) inject (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(serial),
        .jtag_start(j0_start), .jtag_we(j0_we),
        .jtag_addr(j0_addr),   .jtag_wdata(j0_wdata),
        .m0_start(m0_start),   .m0_we(m0_we),
        .m0_addr(m0_addr),     .m0_wdata(m0_wdata),
        .uart_cmd_stb(uart_cmd_stb), .rx_byte_idx(rx_idx)
    );

    top_bus_system #(.CLKS_PER_BIT(CPB)) bus (
        .clk(clk), .rst_n(rst_n),
        .m0_cmd_start(m0_start), .m0_cmd_we(m0_we),
        .m0_cmd_addr(m0_addr),   .m0_cmd_wdata(m0_wdata),
        .m0_cmd_rdata(m0_rdata), .m0_cmd_done(m0_done),
        .m1_cmd_start(1'b0), .m1_cmd_we(1'b0),
        .m1_cmd_addr(14'h0), .m1_cmd_wdata(8'h0),
        .m1_cmd_rdata(), .m1_cmd_done(),
        .ext_tx_serial(serial)
    );

    // Issue a command from the JTAG side and wait for it to retire.
    task jtag_cmd(input we, input [13:0] a, input [7:0] d);
        begin
            @(posedge clk); #2;
            j0_we = we; j0_addr = a; j0_wdata = d; j0_start = 1'b1;
            @(posedge clk); #2; j0_start = 1'b0;
            @(posedge m0_done); @(posedge clk);
        end
    endtask

    task check(input [7:0] got, input [7:0] want, input [8*32:1] what);
        begin
            if (got === want) $display("-> SUCCESS: %0s = 0x%02X", what, got);
            else begin
                $display("-> ERROR:   %0s expected 0x%02X, got 0x%02X", what, want, got);
                errors = errors + 1;
            end
        end
    endtask

    // Command to send: write 0x5A to 0x1000
    //   cmd[23:16]=wdata  cmd[15:2]=addr  cmd[1]=we  cmd[0]=reserved
    //   (0x5A<<16) | (0x1000<<2) | (1<<1) = 0x5A4002
    localparam [23:0] CMD = 24'h5A4002;

    integer waited;

    initial begin
        $display("\n=========================================================");
        $display("   UART COMMAND LINK - LOOPBACK VERIFICATION");
        $display("=========================================================");
        #40 rst_n = 1; #40;

        $display("\n[1] Seed 0x1000 with 0x00 so the UART write is visible...");
        jtag_cmd(1'b1, 14'h1000, 8'h00);
        jtag_cmd(1'b0, 14'h1000, 8'h00);
        check(m0_rdata, 8'h00, "0x1000 before");

        $display("\n[2] Stage command 0x%06X into slave 3 (write 0x5A -> 0x1000)...", CMD);
        jtag_cmd(1'b1, 14'h3000, CMD[7:0]);
        jtag_cmd(1'b1, 14'h3001, CMD[15:8]);
        jtag_cmd(1'b1, 14'h3002, CMD[23:16]);
        jtag_cmd(1'b0, 14'h3000, 8'h00);  check(m0_rdata, CMD[7:0],   "staged byte0");
        jtag_cmd(1'b0, 14'h3001, 8'h00);  check(m0_rdata, CMD[15:8],  "staged byte1");
        jtag_cmd(1'b0, 14'h3002, 8'h00);  check(m0_rdata, CMD[23:16], "staged byte2");

        $display("\n[3] Trigger transmission (write to 0x3003)...");
        jtag_cmd(1'b1, 14'h3003, 8'h01);

        $display("\n[4] Wait for the looped-back command to be injected...");
        waited = 0;
        while (!uart_cmd_stb && waited < 20000) begin
            @(posedge clk); waited = waited + 1;
        end
        if (waited >= 20000) begin
            $display("-> ERROR:   no command injected (rx_byte_idx=%0d)", rx_idx);
            errors = errors + 1;
        end else begin
            $display("-> SUCCESS: command injected after %0d clks", waited);
            $display("            we=%b addr=0x%04X wdata=0x%02X",
                     m0_we, m0_addr, m0_wdata);
            if (m0_we !== 1'b1 || m0_addr !== 14'h1000 || m0_wdata !== 8'h5A) begin
                $display("-> ERROR:   decoded command does not match 0x%06X", CMD);
                errors = errors + 1;
            end
            @(posedge m0_done); @(posedge clk);
        end

        $display("\n[5] Confirm the UART command actually wrote the bus...");
        jtag_cmd(1'b0, 14'h1000, 8'h00);
        check(m0_rdata, 8'h5A, "0x1000 after UART write");

        $display("\n[6] Injector must be back on the JTAG path...");
        jtag_cmd(1'b1, 14'h1000, 8'h33);
        jtag_cmd(1'b0, 14'h1000, 8'h00);
        check(m0_rdata, 8'h33, "0x1000 after JTAG write");

        $display("\n=========================================================");
        if (errors == 0)
            $display(">> UART LINK TEST PASSED: 24-bit command survived the loop! <<");
        else
            $display(">> UART LINK TEST FAILED: %0d error(s) <<", errors);
        $display("=========================================================\n");
        $finish;
    end

endmodule
