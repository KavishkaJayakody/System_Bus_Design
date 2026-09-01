`timescale 1ns / 1ps
//
// uart_rx -- 8N1 UART receiver.
//
// rx_valid pulses for one cycle when a byte has been framed. The input is
// double-flopped first: rx_serial arrives from a pin and is asynchronous to
// clk. CLKS_PER_BIT = f_clk / baud (50 MHz / 115200 = 434).
//
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

                // Sample the middle of the start bit; a glitch aborts.
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
