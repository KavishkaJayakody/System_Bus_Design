//==========================================================================
// tb_shift_deser.v -- self-checking testbench for shift_deser
//
// Covers:
//   1. reset behaviour  - output cleared
//   2. MSB-first order  - the first bit in lands in dout[W-1]
//   3. "last W bits"    - shifting MORE than W bits leaves the LAST W, which
//                         is the property the whole bus relies on:
//                           * a narrow deserialiser shifted for the full
//                             16-clock address frame ends up holding the low
//                             address bits a slave needs
//                           * a DATA_W deserialiser shifted for the same
//                             frame ends up holding the right-aligned write
//                             data
//   4. hold when idle   - no movement without shift
//==========================================================================
`timescale 1ns/1ps

module tb_shift_deser;

    reg clk = 1'b0;
    reg rst_n, shift, din;

    wire [7:0]  d8;
    wire [11:0] d12;
    wire [15:0] d16;

    integer errors = 0;
    integer i;

    always #10 clk = ~clk;

    // Three widths, all fed the SAME serial stream - exactly how the bus
    // wires them: one address wire into receivers of different widths.
    shift_deser #(.W(8))  dut8  (.clk(clk), .rst_n(rst_n), .shift(shift), .din(din), .dout(d8));
    shift_deser #(.W(12)) dut12 (.clk(clk), .rst_n(rst_n), .shift(shift), .din(din), .dout(d12));
    shift_deser #(.W(16)) dut16 (.clk(clk), .rst_n(rst_n), .shift(shift), .din(din), .dout(d16));

    task chk;
        input             cond;
        input [200*8-1:0] name;
        begin
            if (cond) $display("  ok    %0s", name);
            else begin
                $display("  ERROR %0s   (t=%0t d16=%h d12=%h d8=%h)",
                         name, $time, d16, d12, d8);
                errors = errors + 1;
            end
        end
    endtask

    // Shift `n' bits of `word' in, MSB first (word[n-1] first).
    task automatic feed;
        input [15:0]  word;
        input integer n;
        integer       k;
        begin
            for (k = n-1; k >= 0; k = k - 1) begin
                @(posedge clk);
                shift <= 1'b1;
                din   <= word[k];
            end
            @(posedge clk);
            shift <= 1'b0;
            #1;
        end
    endtask

    initial begin
        $display("======================================================");
        $display(" tb_shift_deser");
        $display("======================================================");

        rst_n = 1'b0; shift = 0; din = 0;

        $display("-- 1. reset ------------------------------------------");
        repeat (3) @(posedge clk); #1;
        chk(d8 === 8'h00 && d12 === 12'h000 && d16 === 16'h0000,
            "all widths cleared in reset");
        @(posedge clk); rst_n = 1'b1;

        $display("-- 2. MSB first --------------------------------------");
        feed(16'h00B2, 8);
        chk(d8 === 8'hB2, "8 bits in -> 0xB2, first bit in the MSB");

        $display("-- 3. a 16-bit address frame into every width --------");
        // This is the real case: bus_addr = 0x1ABC shifted for 16 clocks.
        feed(16'h1ABC, 16);
        chk(d16 === 16'h1ABC, "16-bit receiver holds the whole address");
        chk(d12 === 12'hABC,  "12-bit receiver holds addr[11:0] - a 4K slave's offset");
        chk(d8  === 8'hBC,    "8-bit receiver holds the last 8 bits");

        feed(16'h2345, 16);
        chk(d16 === 16'h2345, "second frame: full address");
        chk(d12 === 12'h345,  "second frame: low 12 bits");

        $display("-- 4. write data right-aligned in the frame ----------");
        // bus_dstream during a write: 8 zeros then the data byte.
        feed(16'h009C, 16);
        chk(d8 === 8'h9C, "a DATA_W receiver picks up the right-aligned byte");
        feed(16'h0000, 16);
        chk(d8 === 8'h00, "and a zero byte too");

        $display("-- 5. holds while idle -------------------------------");
        feed(16'h00FF, 8);
        for (i = 0; i < 5; i = i + 1) begin
            @(posedge clk); #1;
            if (d8 !== 8'hFF) begin
                $display("  ERROR output moved with shift low");
                errors = errors + 1;
            end
        end
        $display("  ok    output unchanged for 5 idle cycles");

        $display("======================================================");
        if (errors == 0) $display(" tb_shift_deser: PASSED (0 errors)");
        else             $display(" tb_shift_deser: FAILED (%0d errors)", errors);
        $display("======================================================");
        $finish;
    end

endmodule
