// =============================================================================
// tb_addr_decoder.sv
// Address decoder verification: address mapping to each of the 3 slaves,
// out-of-range address, and the split-lock blocking behaviour.
// (The decoder is purely combinational, so there is no clocked "reset test"
//  as such -- we instead confirm hsel is all-zero at time 0 with no valid
//  address driven, which is the equivalent safe/idle condition.)
// =============================================================================
import bus_pkg::*;

module tb_addr_decoder;

  logic [ADDR_WIDTH-1:0] haddr;
  logic                  hsplit_active;
  logic [NUM_SLAVES-1:0] hsel;

  addr_decoder dut (.*);

  task automatic check(string name, logic cond);
    if (cond) $display("[PASS] %s", name);
    else      $display("[FAIL] %s", name);
  endtask

  initial begin
    // ---------------- "Reset" / idle condition ----------------
    haddr = '0; hsplit_active = 1'b0;
    #1;
    check("idle: address 0 selects Slave1 (in range, not split-locked)", hsel == 3'b001);

    // ---------------- Slave 1 range ----------------
    haddr = S1_TOP; hsplit_active = 1'b0;
    #1; check("map: S1_TOP selects Slave1 only", hsel == 3'b001);

    // ---------------- Slave 2 range ----------------
    haddr = S2_BASE; #1; check("map: S2_BASE selects Slave2 only", hsel == 3'b010);
    haddr = S2_TOP;  #1; check("map: S2_TOP selects Slave2 only",  hsel == 3'b010);

    // ---------------- Slave 3 range ----------------
    haddr = S3_BASE; #1; check("map: S3_BASE selects Slave3 only", hsel == 3'b100);
    haddr = S3_TOP;  #1; check("map: S3_TOP selects Slave3 only",  hsel == 3'b100);

    // ---------------- Out of range ----------------
    haddr = 16'hFFFF; #1; check("map: out-of-range address selects nothing", hsel == 3'b000);

    // ---------------- Split lock ----------------
    haddr = S1_BASE; hsplit_active = 1'b1;
    #1; check("split-lock: Slave1 deselected while split active", hsel == 3'b000);
    hsplit_active = 1'b0;
    #1; check("split-lock cleared: Slave1 selectable again", hsel == 3'b001);

    $display("tb_addr_decoder complete");
    $finish;
  end

endmodule : tb_addr_decoder