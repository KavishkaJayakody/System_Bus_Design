`timescale 1ns / 1ps
//
// uart_tx -- 8N1 UART transmitter.
//
// Assert tx_start for one cycle with tx_data valid. tx_busy stays high until
// the stop bit has been driven. CLKS_PER_BIT = f_clk / baud
// (50 MHz / 115200 = 434).
//
module uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_serial,
    output reg        tx_busy
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shifter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx_serial <= 1'b1;   // line idles high
            tx_busy   <= 1'b0;
            clk_cnt   <= 16'd0;
            bit_idx   <= 3'd0;
            shifter   <= 8'h00;
        end else begin
            case (state)
                IDLE: begin
                    tx_serial <= 1'b1;
                    tx_busy   <= 1'b0;
                    clk_cnt   <= 16'd0;
                    bit_idx   <= 3'd0;
                    if (tx_start) begin
                        shifter   <= tx_data;
                        tx_busy   <= 1'b1;
                        tx_serial <= 1'b0;   // start bit
                        state     <= START;
                    end
                end

                START: begin
                    tx_serial <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt   <= 16'd0;
                        tx_serial <= shifter[0];
                        state     <= DATA;
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            tx_serial <= 1'b1;   // stop bit
                            state     <= STOP;
                        end else begin
                            shifter   <= {1'b0, shifter[7:1]};
                            tx_serial <= shifter[1];
                            bit_idx   <= bit_idx + 3'd1;
                        end
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                STOP: begin
                    tx_serial <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        tx_busy <= 1'b0;
                        state   <= IDLE;
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
