// =============================================================================
// tb_top_level.sv
// Top-level verification: (a) reset, (b) one master request, (c) two master
// requests, and (d) a split-transaction scenario.
//
// top_level no longer accepts external stimulus (both masters run a fixed,
// hardcoded instruction sequence internally), so this testbench acts as a
// MONITOR: it applies reset, then simply waits for the known points in the
// hardcoded schedule and checks the status LEDs at those points, instead of
// driving KEY/SW as before.
// =============================================================================
module tb_top_level;

  logic        CLOCK_50 = 0;
  logic [3:0]  KEY;
  logic [17:0] LEDR;
  logic [8:0]  LEDG;

  top_level dut (.*);

  always #10 CLOCK_50 = ~CLOCK_50;   // 50 MHz-equivalent test clock

  task automatic check(string name, logic cond);
    if (cond) $display("[PASS] %s", name);
    else      $display("[FAIL] %s", name);
  endtask

  // Wait until `sig` is seen to rise, then fall again (one full busy pulse)
  task automatic wait_busy_cycle(ref logic sig);
    @(posedge sig);
    @(negedge sig);
  endtask

  initial begin
    // ---------------- (a) Reset test ----------------
    KEY = 4'b1110;                 // KEY[0] = 0 -> reset asserted
    repeat (3) @(posedge CLOCK_50);
    check("reset: LEDR status bits clear", LEDR[14:8] == 7'b0);
    KEY[0] = 1'b1;                 // release reset
    repeat (2) @(posedge CLOCK_50);

    // ---------------- (b) One master request ----------------
    // Master 1's step 0 is: WRITE 0xA5 to 0x0010 (no other master active
    // yet in the schedule at this point, so this is effectively a
    // single-master scenario).
    wait_busy_cycle(dut.m1_busy);
    repeat (2) @(posedge CLOCK_50);
    check("single request: Master1 completed step 0 (write) without splitting",
          dut.m1_split_pend == 1'b0);

    // Step 1: READ 0x0010 back -- should return what step 0 wrote (0xA5).
    wait_busy_cycle(dut.m1_busy);
    repeat (2) @(posedge CLOCK_50);
    check("single request: Master1 read-back matches earlier write", LEDR[7:0] == 8'hA5);

    // ---------------- (c) Two master requests (contention) ----------------
    // From here on Master 2 is also cycling its own steps independently;
    // its differing IDLE_GAP versus Master 1's means their requests will
    // periodically land in the same cycle. Sample HGRANT over a window
    // long enough to catch a contention event and confirm Master 1 always
    // wins when both are requesting.
    fork
      begin : contention_watch
        repeat (200) begin
          @(posedge CLOCK_50);
          if (dut.hreq[0] && dut.hreq[1]) begin
            check("contention: Master1 wins fixed priority when both request",
                  dut.hgrant == 2'b01);
            disable contention_watch;
          end
        end
      end
    join

    // ---------------- (d) Split transaction scenario ----------------
    // Master 1's step 3 is: READ 0x0030 with force_split asserted. Wait for
    // that step to actually enter the split-locked state, then watch it
    // resolve and confirm the data delivered matches what step 2 wrote
    // (0x77).
    wait (dut.hsplit_active == 1'b1);
    check("split: Slave1 locked (hsplit_active)", 1'b1);
    check("split: Master1 shows split_pending", dut.m1_split_pend == 1'b1);

    wait (dut.hsplit_active == 1'b0);
    repeat (2) @(posedge CLOCK_50);
    check("split: resolved, Master1 read-back matches pre-split data",
          LEDR[7:0] == 8'h77);

    $display("tb_top_level complete");
    $finish;
  end

  // Safety timeout in case the fixed schedule stalls unexpectedly
  initial begin
    #100000;
    $display("[FAIL] tb_top_level timed out waiting for the hardcoded schedule");
    $finish;
  end

endmodule : tb_top_level