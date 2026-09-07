//==========================================================================
// reset_ctrl.v
//
// Turns the DE2-115 KEY[0] pushbutton into the board's single asynchronous
// active-low reset, with the bounce filtered out and the RELEASE edge
// synchronised to the bus clock.
//
// Three jobs in one small block:
//   * 2-flop synchroniser  - KEY[0] is asynchronous to CLOCK_50.
//   * debounce             - the reset is asserted the instant the button
//                            goes down and is only released after the button
//                            has been steady high for 2**CNT_W clocks
//                            (~1.3 ms at 50 MHz with the default 16).  A
//                            bouncing button therefore produces one clean
//                            reset, not a burst of them.
//   * release synchroniser - rst_n falls asynchronously but rises on a clock
//                            edge, so no flip-flop in the design comes out of
//                            reset on a different cycle from its neighbours.
//
// DELIBERATE EXCEPTION to the "every sequential block has an asynchronous
// reset" rule: this IS the reset generator, so it has nothing to be reset
// by.  It relies on Cyclone IV registers powering up cleared, which makes
// rst_n low at configuration and gives a free power-on reset.  In simulation
// the registers start as X and resolve within two clocks of the first edge.
//
//--------------------------------------------------------------------------
// Port     Dir  Width   Meaning
//--------------------------------------------------------------------------
// clk      in   1       Bus clock (CLOCK_50).
// key_n    in   1       Raw KEY[0] pin.  Low = pressed (the DE2-115 keys are
//                       active low with a pull-up).
// rst_n    out  1       System reset, active low.  Asserted while the button
//                       is down and for the debounce interval after it is
//                       released, and after configuration.
//==========================================================================

module reset_ctrl #(
    parameter CNT_W = 16        // debounce interval = 2**CNT_W clocks
) (
    input  wire clk,
    input  wire key_n,
    output reg  rst_n
);

    reg [1:0]        sync;
    reg [CNT_W-1:0]  cnt;

    always @(posedge clk) begin
        sync <= {sync[0], key_n};

        if (!sync[1]) begin
            // button held down: assert reset now, restart the debounce count
            cnt   <= {CNT_W{1'b0}};
            rst_n <= 1'b0;
        end else if (cnt != {CNT_W{1'b1}}) begin
            // released, but not yet steady for long enough
            cnt   <= cnt + 1'b1;
            rst_n <= 1'b0;
        end else begin
            rst_n <= 1'b1;
        end
    end

endmodule
