//==========================================================================
// seg7_hex.v
//
// One hex nibble to one DE2-115 seven-segment digit.
//
// The DE2-115 HEX displays are common anode: a segment lights when its pin
// is driven LOW.  seg[6:0] maps to segments g,f,e,d,c,b,a in that order,
// which is the order the board's pin assignments use.
//
//--------------------------------------------------------------------------
// Port    Dir  Width  Meaning
//--------------------------------------------------------------------------
// val     in   4      Nibble to display, 0-F.
// blank   in   1      1 = all segments off (used to blank a digit).
// seg     out  7      Segment drive, ACTIVE LOW, {g,f,e,d,c,b,a}.
//==========================================================================

module seg7_hex (
    input  wire [3:0] val,
    input  wire       blank,
    output reg  [6:0] seg
);

    always @* begin
        if (blank) begin
            seg = 7'b111_1111;
        end else begin
            case (val)                    //       gfedcba
                4'h0: seg = 7'b100_0000;
                4'h1: seg = 7'b111_1001;
                4'h2: seg = 7'b010_0100;
                4'h3: seg = 7'b011_0000;
                4'h4: seg = 7'b001_1001;
                4'h5: seg = 7'b001_0010;
                4'h6: seg = 7'b000_0010;
                4'h7: seg = 7'b111_1000;
                4'h8: seg = 7'b000_0000;
                4'h9: seg = 7'b001_0000;
                4'hA: seg = 7'b000_1000;
                4'hB: seg = 7'b000_0011;
                4'hC: seg = 7'b100_0110;
                4'hD: seg = 7'b010_0001;
                4'hE: seg = 7'b000_0110;
                4'hF: seg = 7'b000_1110;
                default: seg = 7'b111_1111;
            endcase
        end
    end

endmodule
