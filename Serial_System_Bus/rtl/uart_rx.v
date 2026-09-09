// 8N1 UART receiver.  Taken unchanged from System_Bus_Final.  Double-flops the
// input and re-checks the start bit at its midpoint.  Do not rewrite.

`timescale 1ns/1ps
// board's oscillator and is asynchronous to this one.  That synchroniser is
module uart_rx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_serial,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg  rx_sync1, rx_sync2;
    wire rx = rx_sync2;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shifter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx_serial;
            rx_sync2 <= rx_sync1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            clk_cnt  <= 16'd0;
            bit_idx  <= 3'd0;
            shifter  <= 8'h00;
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                IDLE: begin
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (!rx) state <= START;   // falling edge = start bit
                end

                START: begin
                    if (clk_cnt == (CLKS_PER_BIT-1)/2) begin
                        if (!rx) begin
                            clk_cnt <= 16'd0;
                            state   <= DATA;
                        end else
                            state <= IDLE;
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt          <= 16'd0;
                        shifter[bit_idx] <= rx;
                        if (bit_idx == 3'd7)
                            state <= STOP;
                        else
                            bit_idx <= bit_idx + 3'd1;
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                STOP: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 16'd0;
                        state   <= IDLE;
                        if (rx) begin          // valid stop bit
                            rx_data  <= shifter;
                            rx_valid <= 1'b1;
                        end
                    end else
                        clk_cnt <= clk_cnt + 16'd1;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
