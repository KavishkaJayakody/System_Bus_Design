`timescale 1ns / 1ps
//
// slave_uart_tx -- bus slave 3 (0x3000-0x3FFF). Replaces the old
// cross_fpga_bridge_dummy.
//
// Stage a 24-bit command in three byte registers, then trigger; the block
// serialises all three bytes out of uart_tx to another bus system, whose
// master carries a matching uart_cmd_inject receiver.
//
// Register map (addr[1:0]):
//     0  cmd byte 0   -> becomes cmd[7:0]    at the far end
//     1  cmd byte 1   -> becomes cmd[15:8]
//     2  cmd byte 2   -> becomes cmd[23:16]
//     3  write: any value starts the transfer
//        read:  bit 0 = busy (transfer in progress)
//
// Writes are taken on the first cycle of an access only. The master holds
// m_valid through DRIVE and WAIT, so an ungated trigger would fire repeatedly.
//
module slave_uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sel,
    input  wire        we,
    input  wire [11:0] addr,
    input  wire [7:0]  din,
    output reg  [7:0]  dout,
    output reg         ready,
    output wire        tx_serial
);

    localparam T_IDLE = 2'd0;
    localparam T_SEND = 2'd1;
    localparam T_WAIT = 2'd2;

    reg  [7:0] cmd_byte0, cmd_byte1, cmd_byte2;
    reg        sel_d;
    reg        trig;

    reg  [1:0] tstate;
    reg  [1:0] bsel;
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    wire seq_busy = (tstate != T_IDLE);
    wire sel_start = sel & ~sel_d;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) tx_inst (
        .clk(clk), .rst_n(rst_n),
        .tx_start(tx_start), .tx_data(tx_data),
        .tx_serial(tx_serial), .tx_busy(tx_busy)
    );

    // ---------------- bus side ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_byte0 <= 8'h00;
            cmd_byte1 <= 8'h00;
            cmd_byte2 <= 8'h00;
            dout      <= 8'h00;
            ready     <= 1'b0;
            sel_d     <= 1'b0;
            trig      <= 1'b0;
        end else begin
            sel_d <= sel;
            ready <= 1'b0;
            trig  <= 1'b0;

            if (sel) begin
                ready <= 1'b1;

                if (we) begin
                    if (sel_start) begin
                        case (addr[1:0])
                            2'd0: cmd_byte0 <= din;
                            2'd1: cmd_byte1 <= din;
                            2'd2: cmd_byte2 <= din;
                            2'd3: if (!seq_busy) trig <= 1'b1;
                        endcase
                    end
                    dout <= din;
                end else begin
                    case (addr[1:0])
                        2'd0: dout <= cmd_byte0;
                        2'd1: dout <= cmd_byte1;
                        2'd2: dout <= cmd_byte2;
                        2'd3: dout <= {7'b0000000, seq_busy};
                    endcase
                end
            end
        end
    end

    // ---------------- transmit sequencer ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tstate   <= T_IDLE;
            bsel     <= 2'd0;
            tx_start <= 1'b0;
            tx_data  <= 8'h00;
        end else begin
            case (tstate)
                T_IDLE: begin
                    tx_start <= 1'b0;
                    if (trig) begin
                        bsel   <= 2'd0;
                        tstate <= T_SEND;
                    end
                end

                T_SEND: begin
                    case (bsel)
                        2'd0:    tx_data <= cmd_byte0;
                        2'd1:    tx_data <= cmd_byte1;
                        default: tx_data <= cmd_byte2;
                    endcase
                    tx_start <= 1'b1;
                    tstate   <= T_WAIT;
                end

                T_WAIT: begin
                    tx_start <= 1'b0;
                    // uart_tx raises tx_busy on the edge it accepts tx_start,
                    // so waiting for both low means the byte has gone out.
                    if (!tx_start && !tx_busy) begin
                        if (bsel == 2'd2) begin
                            tstate <= T_IDLE;
                        end else begin
                            bsel   <= bsel + 2'd1;
                            tstate <= T_SEND;
                        end
                    end
                end

                default: tstate <= T_IDLE;
            endcase
        end
    end

endmodule
