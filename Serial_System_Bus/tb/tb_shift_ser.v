//==========================================================================
// tb_shift_ser.v -- self-checking testbench for shift_ser
//
// Covers:
//   1. reset behaviour   - output low, nothing held
//   2. MSB-first order   - a known word comes out most significant bit first
//   3. zero padding      - shifting past the end of the word produces 0s,
//                          which is what lets a short field sit inside a
//                          longer frame
//   4. load priority     - load wins over a simultaneous shift
//   5. hold when idle    - the output does not move without shift
//   6. reload            - a second word can follow the first (the replay
//                          after a SPLIT depends on this)
//==========================================================================
`timescale 1ns/1ps

module tb_shift_ser;

    localparam W = 8;

    reg        clk = 1'b0;
    reg        rst_n, load, shift;
    reg  [W-1:0] din;
    wire       dout;

    integer errors = 0;
    integer i;
    reg [15:0] got;

    always #10 clk = ~clk;

    shift_ser #(.W(W)) dut (
        .clk(clk), .rst_n(rst_n), .load(load), .shift(shift),
        .din(din), .dout(dout)
    );

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t)", name, $time);
                errors = errors + 1;
            end
        end
    endtask

    // Load `word', then shift `n' times, collecting what came out.
    task automatic send;
        input [W-1:0]  word;
        input integer  n;
        integer        k;
        begin
            @(posedge clk);
            load <= 1'b1; shift <= 1'b0; din <= word;
            @(posedge clk);
            load <= 1'b0; shift <= 1'b1;
            got = 16'h0;
            for (k = 0; k < n; k = k + 1) begin
                #1;
                got = {got[14:0], dout};       // collect before the next edge
                @(posedge clk);
            end
            shift <= 1'b0;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_shift_ser");
        $display("======================================================");

        rst_n = 1'b0; load = 0; shift = 0; din = 0;

        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(dout === 1'b0, "output low in reset");
        @(posedge clk); rst_n = 1'b1;
        repeat (2) @(posedge clk); #1;
        chk(dout === 1'b0, "output still low after reset with nothing loaded");

        $display("-- 2. MSB first --------------------------------------");
        send(8'b1011_0010, 8);
        chk(got[7:0] === 8'b1011_0010, "0xB2 came out MSB first");
        send(8'h01, 8);
        chk(got[7:0] === 8'h01, "0x01 came out MSB first");
        send(8'h80, 8);
        chk(got[7:0] === 8'h80, "0x80 came out MSB first");
        send(8'hFF, 8);
        chk(got[7:0] === 8'hFF, "0xFF came out MSB first");

        $display("-- 3. zero padding past the end of the word ----------");
        // 16 shifts of an 8-bit word: the data, then 8 zeros.  This is
        // exactly how write data is right-aligned inside the address frame.
        send(8'hA5, 16);
        chk(got === 16'hA500, "8 data bits then 8 zeros - the padding works");

        $display("-- 4. load beats a simultaneous shift ----------------");
        @(posedge clk); din <= 8'hC3; load <= 1'b1; shift <= 1'b1;
        @(posedge clk); load <= 1'b0; #1;
        chk(dout === 1'b1, "loaded 0xC3, MSB presented despite shift being high");
        shift <= 1'b0;

        $display("-- 5. holds while idle -------------------------------");
        repeat (5) begin
            @(posedge clk); #1;
            if (dout !== 1'b1) begin
                $display("  ERROR output moved with shift low");
                errors = errors + 1;
            end
        end
        $display("  ok    output unchanged for 5 idle cycles");

        $display("-- 6. reload for a replay ----------------------------");
        send(8'h5A, 8);
        chk(got[7:0] === 8'h5A, "first word sent");
        send(8'h5A, 8);
        chk(got[7:0] === 8'h5A, "the SAME word sent again after a reload");
        send(8'h3C, 8);
        chk(got[7:0] === 8'h3C, "a different word after that");

        $display("======================================================");
        if (errors == 0) $display(" tb_shift_ser: PASSED (0 errors)");
        else             $display(" tb_shift_ser: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
