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

    reg [2:0]  latency_cnt;
    reg        processing_split;
    reg        data_ready_for_fetch;
    reg [11:0] saved_raddr;
    reg [7:0]  split_data_reg;
    reg        just_completed; // blocks a spurious re-trigger caused by
                                // master_node holding sel high for one
                                // extra cycle after it samples 'ready'

    localparam SPLIT_LATENCY = 3'd3;

    // ---- Dedicated M9K block, simple dual-port mode ----
    // Port A: write-only, driven by the live bus address (this module
    //         never performs a direct read on port A -- every read goes
    //         through the split mechanism on port B).
    // Port B: read-only, driven by the internally latched saved_raddr,
    //         fetched once the split latency has elapsed.
    // This maps the two logically-independent accesses the original
    // behavioral code made into mem[] onto the block's two real ports,
    // instead of onto extra LUT-based logic.
    wire [7:0] ram_q_b;

    altsyncram #(
        .operation_mode         ("DUAL_PORT"),
        .width_a                (8),
        .widthad_a              (12),
        .numwords_a             (4096),
        .width_b                (8),
        .widthad_b              (12),
        .numwords_b             (4096),
        .lpm_type               ("altsyncram"),
        .outdata_reg_b          ("UNREGISTERED"),
        .address_aclr_b         ("NONE"),
        .outdata_aclr_b         ("NONE"),
        .indata_aclr_a          ("NONE"),
        .wrcontrol_aclr_a       ("NONE"),
        .ram_block_type         ("M9K"),
        .init_file              ("UNUSED"),
        .intended_device_family ("Cyclone IV E")
    ) s0_ram (
        .clock0     (clk),
        .clock1     (clk),        // DUAL_PORT mode requires clock1 to be
                                   // explicitly connected even when port B
                                   // shares the same clock as port A
        .clocken0   (1'b1),
        .clocken1   (1'b1),
        .aclr0      (1'b0),
        .aclr1      (1'b0),
        // Port A -- write path
        .address_a  (addr),
        .data_a     (din),
        .wren_a     (sel & we),
        .rden_a     (1'b1),
        .q_a        (),
        // Port B -- split-fetch read path
        .address_b  (saved_raddr),
        .data_b     (8'h00),
        .wren_b     (1'b0),
        .q_b        (ram_q_b)
    );

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
            just_completed       <= 1'b0;
        end else begin
            ready          <= 1'b0;
            split_req      <= 1'b0;
            split_done     <= 1'b0;
            just_completed <= 1'b0;

            // 1. Bus Access Handling
            if (sel) begin
                if (we) begin
                    // Fast Single-Cycle Write (handled entirely by port A)
                    dout  <= din;
                    ready <= 1'b1;
                end else begin
                    // Read Handling
                    if (data_ready_for_fetch && (addr == saved_raddr)) begin
                        // Master returned to collect completed split data
                        dout                 <= split_data_reg;
                        ready                <= 1'b1;
                        data_ready_for_fetch <= 1'b0;
                        just_completed       <= 1'b1;
                    end else if (!processing_split && !data_ready_for_fetch && !just_completed) begin
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
                    // saved_raddr has been stable on port B for the whole
                    // latency window, which is well beyond the M9K's
                    // 1-cycle synchronous read latency, so ram_q_b is
                    // guaranteed valid here.
                    split_data_reg       <= ram_q_b;
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