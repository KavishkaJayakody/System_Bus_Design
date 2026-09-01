`timescale 1ns / 1ps
//
// uart_cmd_inject -- UART command receiver that sits between the JTAG driver
// and Master 0.
//
// Normally it passes the JTAG (In-System Sources & Probes) command straight
// through. When three bytes have been received over UART they are assembled
// into one 24-bit command; on the next clock that command is issued to Master 0
// instead, and the holding register is cleared the cycle after, so the JTAG
// path resumes immediately.
//
// 24-bit command layout (same field order as the ISSP source slice):
//     cmd[0]      reserved, ignored
//     cmd[1]      we      1 = write, 0 = read
//     cmd[15:2]   addr    14-bit bus address
//     cmd[23:16]  wdata   write data
//
// Bytes arrive least-significant first:
//     byte 0 -> cmd[7:0]    byte 1 -> cmd[15:8]    byte 2 -> cmd[23:16]
//
module uart_cmd_inject #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_serial,

    // Command from the JTAG driver (passed through when idle)
    input  wire        jtag_start,
    input  wire        jtag_we,
    input  wire [13:0] jtag_addr,
    input  wire [7:0]  jtag_wdata,

    // Command presented to Master 0
    output reg         m0_start,
    output reg         m0_we,
    output reg  [13:0] m0_addr,
    output reg  [7:0]  m0_wdata,

    // Status / debug
    output wire        uart_cmd_stb,   // high the cycle a UART command is issued
    output reg  [1:0]  rx_byte_idx     // bytes received so far in this frame
);

    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) rx_inst (
        .clk(clk), .rst_n(rst_n),
        .rx_serial(rx_serial),
        .rx_data(rx_data), .rx_valid(rx_valid)
    );

    reg [23:0] cmd_reg;
    reg        cmd_pending;

    assign uart_cmd_stb = cmd_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_reg     <= 24'h000000;
            cmd_pending <= 1'b0;
            rx_byte_idx <= 2'd0;
        end else begin
            // A pending command lives for exactly one clock: it is issued
            // combinationally below, then the register is cleared.
            if (cmd_pending) begin
                cmd_pending <= 1'b0;
                cmd_reg     <= 24'h000000;
            end

            if (rx_valid) begin
                case (rx_byte_idx)
                    2'd0: cmd_reg[7:0]   <= rx_data;
                    2'd1: cmd_reg[15:8]  <= rx_data;
                    2'd2: cmd_reg[23:16] <= rx_data;
                    default: ;
                endcase

                if (rx_byte_idx == 2'd2) begin
                    rx_byte_idx <= 2'd0;
                    cmd_pending <= 1'b1;   // full 24-bit command assembled
                end else begin
                    rx_byte_idx <= rx_byte_idx + 2'd1;
                end
            end
        end
    end

    // Command mux: UART wins for its single cycle, JTAG otherwise.
    always @(*) begin
        if (cmd_pending) begin
            m0_start = 1'b1;
            m0_we    = cmd_reg[1];
            m0_addr  = cmd_reg[15:2];
            m0_wdata = cmd_reg[23:16];
        end else begin
            m0_start = jtag_start;
            m0_we    = jtag_we;
            m0_addr  = jtag_addr;
            m0_wdata = jtag_wdata;
        end
    end

endmodule
