`timescale 1ns / 1ps

module slave_fast_ram #(
    parameter ADDR_WIDTH = 12 // 12 for 4K (S1), 11 for 2K (S2)
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  sel,
    input  wire                  we,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [7:0]            din,
    output reg  [7:0]            dout,
    output reg                   ready
);

    // ---- Dedicated M9K block, single-port mode ----
    // read_during_write_mode_port_a = NEW_DATA means: on a write cycle,
    // q_a comes out equal to the data just written one cycle later; on a
    // read cycle, q_a comes out equal to the stored word one cycle later.
    // That is exactly "dout <= din" on write / "dout <= mem[addr]" on
    // read from the original code, for free, with no extra register
    // stage stacked on top of the block's own synchronous read latency.
    wire [7:0] ram_q;

    altsyncram #(
        .operation_mode                ("SINGLE_PORT"),
        .width_a                       (8),
        .widthad_a                     (ADDR_WIDTH),
        .numwords_a                    (1 << ADDR_WIDTH),
        .lpm_type                      ("altsyncram"),
        .outdata_reg_a                 ("UNREGISTERED"),
        .address_aclr_a                ("NONE"),
        .outdata_aclr_a                ("NONE"),
        .indata_aclr_a                 ("NONE"),
        .wrcontrol_aclr_a              ("NONE"),
        .ram_block_type                ("M9K"),
        .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .init_file                     ("UNUSED"),
        .intended_device_family        ("Cyclone IV E")
    ) ram_inst (
        .clock0     (clk),
        .clocken0   (1'b1),
        .aclr0      (1'b0),
        .address_a  (addr),
        .data_a     (din),
        .wren_a     (sel & we),
        .rden_a     (1'b1),
        .q_a        (ram_q)
    );

    // dout is driven combinationally from the block's own registered
    // output -- kept as a `reg` for interface/port-list compatibility,
    // but this is a combinational always block, not a clocked one, so
    // no second flip-flop stage is added after ram_q.
    always @(*) begin
        dout = ram_q;
    end

    // 'ready' is a pure handshake flag (no memory access of its own),
    // so it keeps its own explicit register exactly as before.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
        end else begin
            ready <= sel;
        end
    end

endmodule