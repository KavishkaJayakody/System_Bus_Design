//==========================================================================
// tb_de2_top.v -- self-checking testbench for the DE2-115 board wrapper
//
// Runs the real board top level, driving the pins the way a person at the
// board would: hold KEY[0], flip switches, press KEY[1].  The debounce and
// split intervals are shrunk by parameter so a simulated key press does not
// take 65536 cycles.
//
// Covers:
//   1. power-on / KEY[0] reset - the design comes up held in reset and
//                                releases cleanly, with nothing driving the
//                                bus
//   2. scenario 00  one master  - master 0 makes progress, master 1 never
//                                 touches the bus
//   3. scenario 01  two masters - both make progress, both get grants
//   4. scenario 10  split       - with SW[16] on, the arbiter mask is seen
//                                 to go up, master 1 keeps completing
//                                 transactions during it, and master 0
//                                 finishes anyway
//   5. scenario 11  unmapped    - the sticky error LED lights AND the design
//                                 keeps running (recovery, not a hang)
//   6. single step              - with run off, one KEY[1] press produces
//                                 exactly one transaction
//   7. displays                 - the HEX digits track the address/data
//==========================================================================
`timescale 1ns/1ps
`include "bus_defs.vh"

module tb_de2_top;

    reg         CLOCK_50 = 1'b0;
    reg  [3:0]  KEY;
    reg  [17:0] SW;
    wire [17:0] LEDR;
    wire [8:0]  LEDG;
    wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;

    always #10 CLOCK_50 = ~CLOCK_50;     // 50 MHz

    integer errors = 0;

    de2_top #(
        .SPLIT_LATENCY (8),      // instead of 10,000,000
        .TICK_W        (4),      // instead of 23
        .DEB_W         (4)       // instead of 16
    ) dut (
        .CLOCK_50(CLOCK_50), .KEY(KEY), .SW(SW),
        .LEDR(LEDR), .LEDG(LEDG),
        .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2), .HEX3(HEX3),
        .HEX4(HEX4), .HEX5(HEX5), .HEX6(HEX6), .HEX7(HEX7)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t LEDR=%b LEDG=%b)",
                         name, $time, LEDR, LEDG);
                errors = errors + 1;
            end
        end
    endtask

    task press_reset;
        begin
            KEY[0] = 1'b0;
            repeat (40) @(posedge CLOCK_50);
            KEY[0] = 1'b1;
            repeat (80) @(posedge CLOCK_50);   // ride out the debounce
        end
    endtask

    task press_step;
        begin
            KEY[1] = 1'b0;
            repeat (40) @(posedge CLOCK_50);
            KEY[1] = 1'b1;
            repeat (40) @(posedge CLOCK_50);
        end
    endtask

    // Convenient handles into the design.
    wire [15:0] m0_xacts = dut.u_prog0.xact_count;
    wire [15:0] m1_xacts = dut.u_prog1.xact_count;

    integer x0, x1;
    integer k;
    reg saw_mask, saw_m1_progress;
    integer m1_at_mask;

    initial begin
        $display("======================================================");
        $display(" tb_de2_top -- DE2-115 board wrapper");
        $display("======================================================");

        KEY = 4'b1111;
        SW  = 18'd0;

        //==================================================================
        $display("-- 1. reset ------------------------------------------");
        KEY[0] = 1'b0;
        repeat (20) @(posedge CLOCK_50); #1;
        chk(dut.rst_n === 1'b0,  "held in reset while KEY[0] is down");
        chk(LEDR[1:0] === 2'b00, "no grant during reset");
        chk(LEDR[3:2] === 2'b00, "split mask clear during reset");
        KEY[0] = 1'b1;
        repeat (60) @(posedge CLOCK_50); #1;
        chk(dut.rst_n === 1'b1,  "reset released after the debounce interval");
        chk(LEDR[1:0] === 2'b00, "still no grant - nothing has been asked for");
        chk(m0_xacts  === 16'd0, "no transactions before run is enabled");

        //==================================================================
        $display("-- 2. scenario 00: one master ------------------------");
        SW[1:0]  = 2'b00;
        SW[17]   = 1'b1;                 // run, full speed
        repeat (400) @(posedge CLOCK_50); #1;
        chk(m0_xacts > 16'd4, "master 0 completed several transactions");
        chk(m1_xacts === 16'd0, "master 1 never issued a transaction");
        chk(LEDG[8] === 1'b0,  "no error flagged");

        //==================================================================
        $display("-- 3. scenario 01: two masters -----------------------");
        SW[17] = 1'b0;
        press_reset;
        SW[1:0] = 2'b01;
        SW[17]  = 1'b1;
        repeat (600) @(posedge CLOCK_50); #1;
        chk(m0_xacts > 16'd4, "master 0 made progress");
        chk(m1_xacts > 16'd4, "master 1 made progress");
        chk(LEDG[8] === 1'b0, "no error flagged");
        // both masters must actually have been granted at some point
        begin : both_granted
            reg g0, g1;
            g0 = 0; g1 = 0;
            for (k = 0; k < 400; k = k + 1) begin
                @(posedge CLOCK_50); #1;
                if (LEDR[0]) g0 = 1;
                if (LEDR[1]) g1 = 1;
            end
            chk(g0, "master 0 was granted the bus");
            chk(g1, "master 1 was granted the bus");
        end

        //==================================================================
        $display("-- 4. scenario 10: split transaction -----------------");
        SW[17] = 1'b0;
        press_reset;
        SW[1:0] = 2'b10;
        SW[16]  = 1'b1;                  // slave 0 split enable
        SW[17]  = 1'b1;

        saw_mask = 0; saw_m1_progress = 0; m1_at_mask = -1;
        for (k = 0; k < 3000; k = k + 1) begin
            @(posedge CLOCK_50); #1;
            if (LEDR[2]) begin           // split_mask[0]
                if (!saw_mask) begin
                    saw_mask   = 1;
                    m1_at_mask = m1_xacts;
                end
                if (m1_at_mask >= 0 && m1_xacts > m1_at_mask)
                    saw_m1_progress = 1;
            end
        end
        chk(saw_mask,        "arbiter masked master 0 on a split");
        chk(saw_m1_progress, "master 1 completed transactions DURING the split");
        chk(m0_xacts > 16'd2,"master 0 still finished its split transactions");
        chk(dut.u_bus.split_count_flat[7:0] > 8'd0,
            "master 0 recorded SPLIT responses");
        chk(LEDG[8] === 1'b0, "no error flagged in the split scenario");
        SW[16] = 1'b0;

        //==================================================================
        $display("-- 5. scenario 11: unmapped address recovery ---------");
        SW[17] = 1'b0;
        press_reset;
        SW[1:0] = 2'b11;
        SW[17]  = 1'b1;
        repeat (200) @(posedge CLOCK_50); #1;
        chk(LEDG[8] === 1'b1,  "sticky ERROR LED lit by the unmapped access");
        x0 = m0_xacts;
        repeat (800) @(posedge CLOCK_50); #1;
        chk(m0_xacts > x0,
            "the design KEPT RUNNING after the error - the bus recovered");
        chk(m1_xacts > 16'd0,  "master 1 unaffected");

        //==================================================================
        $display("-- 6. single step ------------------------------------");
        SW[17] = 1'b0;
        press_reset;
        SW[1:0] = 2'b00;
        SW[17]  = 1'b0;                  // stopped
        repeat (200) @(posedge CLOCK_50); #1;
        chk(m0_xacts === 16'd0, "nothing runs while the run switch is off");

        x0 = m0_xacts;
        press_step;
        repeat (100) @(posedge CLOCK_50); #1;
        chk(m0_xacts == x0 + 1, "one KEY[1] press ran exactly one transaction");
        x0 = m0_xacts;
        repeat (300) @(posedge CLOCK_50); #1;
        chk(m0_xacts == x0, "and then it stopped again");

        //==================================================================
        $display("-- 7. displays ---------------------------------------");
        SW[17] = 1'b1;
        repeat (400) @(posedge CLOCK_50); #1;
        chk(dut.last_addr0 !== 16'h0000 || dut.u_prog0.cmd_addr !== 16'h0000,
            "the address display latched a real command address");
        chk(HEX7 !== 7'h7F && HEX0 !== 7'h7F, "the HEX digits are driven");
        // slave 1's range is 0x1xxx, so the top digit must read 1 or 2
        chk(dut.last_addr0[15:12] === 4'h1 || dut.last_addr0[15:12] === 4'h2,
            "the displayed address is one of the addresses in scenario 00");

        $display("======================================================");
        $display(" m0 transactions=%0d  m1 transactions=%0d",
                 m0_xacts, m1_xacts);
        if (errors == 0) $display(" tb_de2_top: PASSED (0 errors)");
        else             $display(" tb_de2_top: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

    initial begin
        #5000000;
        $display(" tb_de2_top: FAILED (global timeout)");
        $finish;
    end

endmodule
