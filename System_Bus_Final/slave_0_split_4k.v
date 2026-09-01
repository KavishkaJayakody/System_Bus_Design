`timescale 1ns / 1ps

module slave_0_split_4k (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sel,
    input  wire        we,
    input  wire [11:0] addr,
    input  wire [7:0]  din,
    output reg  [7:0]  dout,
    output reg         ready,
    output reg         split_req,
    output reg         split_done
);

    reg [7:0]  mem [0:4095];
    reg [2:0]  latency_cnt;
    reg        processing_split;
    reg        data_ready_for_fetch;
    reg [11:0] saved_raddr;
    reg [7:0]  split_data_reg;

    localparam SPLIT_LATENCY = 3'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout                 <= 8'h00;
            ready                <= 1'b0;
            split_req            <= 1'b0;
            split_done           <= 1'b0;
            processing_split     <= 1'b0;
            data_ready_for_fetch <= 1'b0;
            latency_cnt          <= 3'd0;
            split_data_reg       <= 8'h00;
            saved_raddr          <= 12'h000;
        end else begin
            ready      <= 1'b0;
            split_req  <= 1'b0;
            split_done <= 1'b0;

            // 1. Bus Access Handling
            if (sel) begin
                if (we) begin
                    // Fast Single-Cycle Write
                    mem[addr] <= din;
                    dout      <= din;
                    ready     <= 1'b1;
                end else begin
                    // Read Handling
                    if (data_ready_for_fetch && (addr == saved_raddr)) begin
                        // Master returned to collect completed split data
                        dout                 <= split_data_reg;
                        ready                <= 1'b1;
                        data_ready_for_fetch <= 1'b0;
                    end else if (!processing_split && !data_ready_for_fetch) begin
                        // Initial Read request: trigger split handshake
                        saved_raddr      <= addr;
                        processing_split <= 1'b1;
                        split_req        <= 1'b1;
                        latency_cnt      <= 3'd1;
                    end
                end
            end

            // 2. Background Split Latency Timer
            if (processing_split) begin
                if (latency_cnt == SPLIT_LATENCY) begin
                    split_data_reg       <= mem[saved_raddr];
                    split_done           <= 1'b1;
                    data_ready_for_fetch <= 1'b1;
                    processing_split     <= 1'b0;
                    latency_cnt          <= 3'd0;
                end else begin
                    latency_cnt <= latency_cnt + 1'b1;
                end
            end
        end
    end

endmodule