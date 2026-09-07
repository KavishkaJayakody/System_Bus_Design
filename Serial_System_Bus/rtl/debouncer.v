//==========================================================================
// debouncer.v
//
// Synchronises and debounces one active-low pushbutton and produces both a
// clean level and a one-clock press pulse.
//
// A mechanical key bounces for a few milliseconds.  Fed straight into the
// scenario sequencer's step input, one press would fire dozens of
// transactions.  The level output only changes after the input has been
// stable for 2**CNT_W clocks (~1.3 ms at 50 MHz with the default 16).
//
//--------------------------------------------------------------------------
// Port      Dir  Width  Meaning
//--------------------------------------------------------------------------
// clk       in   1      Bus clock.
// rst_n     in   1      Asynchronous active-low reset.
// key_n     in   1      Raw pushbutton pin, low = pressed.
// level     out  1      Debounced button state, active HIGH (1 = pressed).
// pulse     out  1      One clock wide, on the debounced press edge.
//==========================================================================

module debouncer #(
    parameter CNT_W = 16
) (
    input  wire clk,
    input  wire rst_n,
    input  wire key_n,
    output reg  level,
    output reg  pulse
);

    reg [1:0]       sync;
    reg [CNT_W-1:0] cnt;
    reg             level_d;

    wire pressed = ~sync[1];        // active low pin -> active high level

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync    <= 2'b11;       // idle = not pressed
            cnt     <= {CNT_W{1'b0}};
            level   <= 1'b0;
            level_d <= 1'b0;
            pulse   <= 1'b0;
        end else begin
            sync <= {sync[0], key_n};

            if (pressed == level) begin
                // input agrees with the accepted state: nothing to time
                cnt <= {CNT_W{1'b0}};
            end else if (cnt == {CNT_W{1'b1}}) begin
                // it has disagreed for the whole interval: accept the change
                cnt   <= {CNT_W{1'b0}};
                level <= pressed;
            end else begin
                cnt <= cnt + 1'b1;
            end

            level_d <= level;
            pulse   <= level & ~level_d;      // rising edge of the clean level
        end
    end

endmodule
