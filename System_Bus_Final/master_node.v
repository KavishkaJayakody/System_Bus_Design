`timescale 1ns / 1ps

module master_node (
    input  wire        clk,
    input  wire        rst_n,
    
    // Command Interface
    input  wire        cmd_start,
    input  wire        cmd_we,
    input  wire [13:0] cmd_addr,
    input  wire [7:0]  cmd_wdata,
    output reg  [7:0]  cmd_rdata,
    output reg         cmd_done,
    
    // Arbiter Interface
    output reg         bus_req,
    input  wire        bus_gnt,
    input  wire        bus_split,
    
    // Shared Bus Interface
    output reg  [13:0] m_addr,
    output reg  [7:0]  m_wdata,
    output reg         m_we,
    output reg         m_valid,
    input  wire [7:0]  bus_rdata,
    input  wire        bus_ready
);

    localparam IDLE    = 3'b000;
    localparam REQ     = 3'b001;
    localparam DRIVE   = 3'b010;
    localparam WAIT    = 3'b011;
    localparam SPLIT_W = 3'b100;
    localparam DONE    = 3'b101;

    reg [2:0]  state;
    reg [13:0] reg_addr;
    reg [7:0]  reg_wdata;
    reg        reg_we;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            bus_req   <= 1'b0;
            m_valid   <= 1'b0;
            m_addr    <= 14'h0000;
            m_wdata   <= 8'h00;
            m_we      <= 1'b0;
            cmd_rdata <= 8'h00;
            cmd_done  <= 1'b0;
            reg_addr  <= 14'h0000;
            reg_wdata <= 8'h00;
            reg_we    <= 1'b0;
        end else begin
            cmd_done <= 1'b0;

            case (state)
                IDLE: begin
                    if (cmd_start) begin
                        reg_addr  <= cmd_addr;
                        reg_wdata <= cmd_wdata;
                        reg_we    <= cmd_we;
                        bus_req   <= 1'b1;
                        state     <= REQ;
                    end
                end

                REQ: begin
                    if (bus_gnt) begin
                        m_addr  <= reg_addr;
                        m_wdata <= reg_wdata;
                        m_we    <= reg_we;
                        m_valid <= 1'b1;
                        state   <= DRIVE;
                    end
                end

                DRIVE: begin
                    if (bus_split) begin
                        m_valid <= 1'b0;
                        bus_req <= 1'b1; // Keep requesting for when data becomes ready
                        state   <= SPLIT_W;
                    end else if (bus_ready) begin
                        m_valid   <= 1'b0;
                        bus_req   <= 1'b0;
                        cmd_rdata <= bus_rdata;
                        cmd_done  <= 1'b1;
                        state     <= DONE;
                    end else begin
                        state <= WAIT;
                    end
                end

                WAIT: begin
                    if (bus_split) begin
                        m_valid <= 1'b0;
                        bus_req <= 1'b1;
                        state   <= SPLIT_W;
                    end else if (bus_ready) begin
                        m_valid   <= 1'b0;
                        bus_req   <= 1'b0;
                        cmd_rdata <= bus_rdata;
                        cmd_done  <= 1'b1;
                        state     <= DONE;
                    end
                end

                SPLIT_W: begin
                    bus_req <= 1'b1;
                    if (bus_gnt) begin
                        m_addr  <= reg_addr;
                        m_wdata <= 8'h00;
                        m_we    <= 1'b0;
                        m_valid <= 1'b1;
                        state   <= DRIVE;
                    end
                end

                DONE: begin
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule