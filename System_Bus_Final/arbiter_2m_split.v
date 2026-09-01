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
    reg       m0_waiting_for_split;

    // Sequential State & Split History Register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= IDLE;
            m0_waiting_for_split <= 1'b0;
        end else begin
            state <= next_state;

            // Track split status
            if (state == GRANT_M0 && s0_split_req)
                m0_waiting_for_split <= 1'b1;
            else if (state == GRANT_M0 && !s0_split_req && s0_split_done)
                m0_waiting_for_split <= 1'b0;
            else if (state == GRANT_M0 && !s0_split_req && !m0_waiting_for_split)
                m0_waiting_for_split <= 1'b0;
        end
    end

    // Combinational Next-State & Output Logic
    always @(*) begin
        next_state      = state;
        gnt0            = 1'b0;
        gnt1            = 1'b0;
        m0_split_notify = 1'b0;
        m1_split_notify = 1'b0;

        case (state)
            IDLE: begin
                if (m0_waiting_for_split && s0_split_done) begin
                    next_state = GRANT_M0; // Priority re-grant to resume split transaction
                end else if (req0) begin
                    next_state = GRANT_M0; // Master 0 Priority
                end else if (req1) begin
                    next_state = GRANT_M1; // Master 1 Lower Priority
                end
            end

            GRANT_M0: begin
                gnt0 = 1'b1;
                if (s0_split_req) begin
                    m0_split_notify = 1'b1;
                    next_state      = req1 ? GRANT_M1 : IDLE; // Relinquish bus immediately
                end else if (!req0) begin
                    next_state      = req1 ? GRANT_M1 : IDLE;
                end
            end

            GRANT_M1: begin
                gnt1 = 1'b1;
                if (!req1) begin
                    if (m0_waiting_for_split && s0_split_done)
                        next_state = GRANT_M0;
                    else if (req0)
                        next_state = GRANT_M0;
                    else
                        next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end

endmodule