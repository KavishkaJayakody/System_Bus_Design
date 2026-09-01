// =============================================================================
// tb_arbiter.sv
// Arbiter verification: reset test, single-master request, two-master
// (contention) request, and a split-transaction scenario.
// =============================================================================
import bus_pkg::*;

module tb_arbiter;

  logic clk = 0;
  logic rst_n;
  logic [NUM_MASTERS-1:0] hreq;
  logic                   split_pulse;
  logic                   hsplit_done;
  logic [NUM_MASTERS-1:0] hgrant;
  logic [NUM_MASTERS-1:0] hsplit_notify;
  logic                   hready;

  arbiter dut (.*);

  always #5 clk = ~clk;

  task automatic check(string name, logic cond);
    if (cond) $display("[PASS] %s", name);
    else      $display("[FAIL] %s", name);
  endtask

  initial begin
    // ---------------- Reset test ----------------
    rst_n = 0; hreq = '0; split_pulse = 0; hsplit_done = 0;
    @(negedge clk);
    check("reset: hgrant is 0 during reset", hgrant == '0);
    @(posedge clk); #1;
    rst_n = 1;
    @(negedge clk);
    check("reset: hgrant stays 0 with no requests after release", hgrant == '0);

    // ---------------- Single master request (Master 2 alone) --------------
    hreq = 2'b10;
    @(negedge clk);
    check("single request: Master2 alone is granted", hgrant == 2'b10);
    hreq = '0;
    @(negedge clk);

    // ---------------- Two master requests: Master1 must win -----------------
    hreq = 2'b11;
    @(negedge clk);
    check("contention: Master1 wins fixed priority", hgrant == 2'b01);
    // Drop Master1's request; Master2 should now be granted
    hreq = 2'b10;
    @(negedge clk);
    check("contention: Master2 granted once Master1 idle", hgrant == 2'b10);
    hreq = '0;
    @(negedge clk);

    // ---------------- Split transaction scenario ----------------
    // Master1 requests and is granted, then gets a SPLIT response.
    hreq = 2'b01;
    @(negedge clk);
    check("split setup: Master1 granted", hgrant == 2'b01);

    split_pulse = 1'b1;   // Master1's transfer just came back SPLIT
    @(negedge clk);
    split_pulse = 1'b0;
    check("split: notify pulsed to Master1", hsplit_notify == 2'b01);

    // While Master1 is masked, Master2 should be able to use the bus
    hreq = 2'b11;
    @(negedge clk);
    check("split: Master1 masked, Master2 granted despite Master1 request", hgrant == 2'b10);

    // Slave signals the split resource is ready
    hsplit_done = 1'b1;
    @(negedge clk);
    hsplit_done = 1'b0;
    check("split: resume grant forced back to Master1", hgrant == 2'b01);

    @(negedge clk);
    check("split: after resume, normal priority resumes (Master1 still requesting)", hgrant == 2'b01);

    hreq = '0;
    #20;
    $display("tb_arbiter complete");
    $finish;
  end

endmodule : tb_arbiter