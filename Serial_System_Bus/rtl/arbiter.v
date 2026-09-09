// Fixed-priority arbiter, master 0 highest, with a transfer lock and an
// AHB-style split mask.  Parameterised on N_MASTERS.

`include "bus_defs.vh"

module arbiter #(
    parameter N_MASTERS = `BUS_N_MASTERS,
    // ID_W must equal ceil(log2(N_MASTERS)).  It is a parameter rather than a
    parameter ID_W      = `BUS_ID_W,
    parameter RESP_W    = `BUS_RESP_W
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [N_MASTERS-1:0]    req,
    input  wire                    bus_ready,
    input  wire [RESP_W-1:0]       bus_resp,
    input  wire [N_MASTERS-1:0]    split_complete,
    output reg  [N_MASTERS-1:0]    gnt,
    output wire                    gnt_valid,
    output reg  [ID_W-1:0]         master_id,
    output reg  [N_MASTERS-1:0]    split_mask,
    output reg                     locked
);

    integer i;

    wire [N_MASTERS-1:0] elig = req & ~split_mask;

    // Fixed-priority pick: the loop runs downwards so the lowest index wins.
    reg                  pick_val;
    reg [ID_W-1:0]       pick_id;
    reg [N_MASTERS-1:0]  pick_onehot;

    always @* begin
        pick_val    = 1'b0;
        pick_id     = {ID_W{1'b0}};
        pick_onehot = {N_MASTERS{1'b0}};
        for (i = N_MASTERS-1; i >= 0; i = i - 1) begin
            if (elig[i]) begin
                pick_val       = 1'b1;
                pick_id        = i[ID_W-1:0];
                pick_onehot    = {N_MASTERS{1'b0}};
                pick_onehot[i] = 1'b1;
            end
        end
    end

    wire xfer_done  = locked & bus_ready;
    wire xfer_split = xfer_done & (bus_resp == `RESP_SPLIT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gnt        <= {N_MASTERS{1'b0}};
            master_id  <= {ID_W{1'b0}};
            split_mask <= {N_MASTERS{1'b0}};
            locked     <= 1'b0;
        end else begin
            // Keyed off gnt[i], NOT master_id == i[ID_W-1:0]: that TRUNCATES
            // the index, so with ID_W too small, splitting master 0 also
            // masked master 2 permanently.  gnt is one-hot and cannot alias.
            for (i = 0; i < N_MASTERS; i = i + 1) begin
                if (xfer_split && gnt[i])
                    split_mask[i] <= 1'b1;
                else if (split_complete[i])
                    split_mask[i] <= 1'b0;
            end

            if (!locked) begin
                if (pick_val) begin
                    gnt       <= pick_onehot;
                    master_id <= pick_id;
                    locked    <= 1'b1;
                end else begin
                    gnt <= {N_MASTERS{1'b0}};
                end
            end else if (bus_ready) begin
                gnt    <= {N_MASTERS{1'b0}};
                locked <= 1'b0;
            end
        end
    end

    assign gnt_valid = |gnt;

endmodule
