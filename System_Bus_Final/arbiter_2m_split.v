`timescale 1ns / 1ps

module arbiter_2m_split (
    input  wire clk,
    input  wire rst_n,

    // Master Requests and Grants
    input  wire req0,
    input  wire req1,
    output reg  gnt0,
    output reg  gnt1,

    // Slave 0 Split Interface
    input  wire s0_split_req,
    input  wire s0_split_done,
    output reg  m0_split_notify,
    output reg  m1_split_notify
);

    localparam IDLE     = 2'b00;
    localparam GRANT_M0 = 2'b01;
    localparam GRANT_M1 = 2'b10;

    reg [1:0] state, next_state;
    reg       m0_split_pending;
    reg       split_data_ready;

    // 1. Sequential State and Status Tracking
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= IDLE;
            m0_split_pending <= 1'b0;
            split_data_ready <= 1'b0;
        end else begin
            state <= next_state;

            // Set when split occurs
            if (state == GRANT_M0 && s0_split_req) begin
                m0_split_pending <= 1'b1;
            end

            // Latch background completion pulse
            if (s0_split_done) begin
                split_data_ready <= 1'b1;
            end

            // Clear split flags when M0 successfully re-acquires the bus and completes
            if (state == GRANT_M0 && !s0_split_req && split_data_ready) begin
                m0_split_pending <= 1'b0;
                split_data_ready <= 1'b0;
            end
        end
    end

    // 2. Combinational Next-State Logic
    always @(*) begin
        next_state      = state;
        gnt0            = 1'b0;
        gnt1            = 1'b0;
        m0_split_notify = 1'b0;
        m1_split_notify = 1'b0;

        case (state)
            IDLE: begin
                // Priority 1: Service pending split completion
                if (m0_split_pending && (split_data_ready || s0_split_done)) begin
                    next_state = GRANT_M0;
                end
                // Priority 2: Standard Master 0 request
                else if (req0 && !m0_split_pending) begin
                    next_state = GRANT_M0;
                end
                // Priority 3: Master 1 request
                else if (req1) begin
                    next_state = GRANT_M1;
                end
            end

            GRANT_M0: begin
                gnt0 = 1'b1;
                if (s0_split_req) begin
                    m0_split_notify = 1'b1;
                    next_state      = req1 ? GRANT_M1 : IDLE;
                end else if (!req0) begin
                    if (req1)
                        next_state = GRANT_M1;
                    else
                        next_state = IDLE;
                end
            end

            GRANT_M1: begin
                gnt1 = 1'b1;
                if (!req1) begin
                    if (m0_split_pending && (split_data_ready || s0_split_done))
                        next_state = GRANT_M0;
                    else if (req0 && !m0_split_pending)
                        next_state = GRANT_M0;
                    else
                        next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end

endmodule