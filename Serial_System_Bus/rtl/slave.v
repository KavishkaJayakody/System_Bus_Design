// Word-addressed memory slave.  WORDS/LADDR_W set the size; SPLIT_CAPABLE
// adds the split machinery.  Every slave shifts every frame in speculatively -
// the decoder cannot say who is addressed until the prefix has arrived.

`include "bus_defs.vh"

module slave #(
    parameter DATA_W        = `BUS_DATA_W,
    parameter LADDR_W       = 12,
    parameter WORDS         = 4096,
    parameter RESP_W        = `BUS_RESP_W,
    parameter N_MASTERS     = `BUS_N_MASTERS,
    parameter ID_W          = `BUS_ID_W,
    parameter SPLIT_CAPABLE = 0,
    parameter SPLIT_LATENCY = 4,
    // 32 bits: a counter narrower than SPLIT_LATENCY truncates silently.
    parameter CNT_W         = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  frame,
    input  wire                  astream,
    input  wire                  dstream_in,
    input  wire                  sel,
    input  wire                  we,
    input  wire [ID_W-1:0]       master_id,
    input  wire                  split_en,
    output wire                  dstream_out,
    output reg                   ready,
    output reg  [RESP_W-1:0]     resp,
    output wire [N_MASTERS-1:0]  split_complete,
    output wire                  busy
);

    function integer clogb2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clogb2 = 1; v > 1; clogb2 = clogb2 + 1)
                v = v >> 1;
        end
    endfunction
    localparam BCNT_W = clogb2(DATA_W);

    wire [LADDR_W-1:0] laddr;
    wire [DATA_W-1:0]  wdata_des;

    shift_deser #(.W(LADDR_W)) u_addr_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(astream), .dout(laddr)
    );

    shift_deser #(.W(DATA_W)) u_wdata_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(frame), .din(dstream_in), .dout(wdata_des)
    );

    wire do_split;

    generate
    if (SPLIT_CAPABLE) begin : g_split

        reg                  busy_r;
        reg  [CNT_W-1:0]     cnt_r;
        reg  [ID_W-1:0]      split_id_r;
        reg                  resume_pend_r;
        reg  [ID_W-1:0]      resume_id_r;
        reg  [N_MASTERS-1:0] sc_r;

        localparam [CNT_W-1:0] LAT_C = SPLIT_LATENCY;

        wire is_resume = resume_pend_r && (master_id == resume_id_r);

        assign do_split = sel && split_en && !busy_r && !resume_pend_r;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                busy_r        <= 1'b0;
                cnt_r         <= {CNT_W{1'b0}};
                split_id_r    <= {ID_W{1'b0}};
                resume_pend_r <= 1'b0;
                resume_id_r   <= {ID_W{1'b0}};
                sc_r          <= {N_MASTERS{1'b0}};
            end else begin
                sc_r <= {N_MASTERS{1'b0}};      // default: one-cycle pulse

                if (do_split) begin
                    busy_r     <= 1'b1;
                    cnt_r      <= LAT_C;
                    split_id_r <= master_id;
                end else if (busy_r) begin
                    if (cnt_r == {CNT_W{1'b0}}) begin
                        busy_r           <= 1'b0;
                        sc_r[split_id_r] <= 1'b1;
                        resume_pend_r    <= 1'b1;
                        resume_id_r      <= split_id_r;
                    end else begin
                        cnt_r <= cnt_r - 1'b1;
                    end
                end

                if (sel && is_resume)
                    resume_pend_r <= 1'b0;
            end
        end

        assign split_complete = sc_r;
        assign busy           = busy_r;

    end else begin : g_nosplit

        assign do_split       = 1'b0;
        assign split_complete = {N_MASTERS{1'b0}};
        assign busy           = 1'b0;

    end
    endgenerate

    wire serve     = sel && !do_split;
    wire mem_write = serve &&  we;
    wire mem_read  = serve && !we;

    localparam R_IDLE  = 2'd0;
    localparam R_MEMQ  = 2'd1;
    localparam R_SHIFT = 2'd2;

    reg [1:0]        rs;
    reg [BCNT_W-1:0] rcnt;

    wire rd_last = (rs == R_SHIFT) && (rcnt == (DATA_W-1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rs   <= R_IDLE;
            rcnt <= {BCNT_W{1'b0}};
        end else begin
            case (rs)
                R_IDLE:  if (mem_read) rs <= R_MEMQ;
                R_MEMQ:  begin rs <= R_SHIFT; rcnt <= {BCNT_W{1'b0}}; end
                R_SHIFT: begin
                    rcnt <= rcnt + 1'b1;
                    if (rd_last) rs <= R_IDLE;
                end
                default: rs <= R_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            resp  <= `RESP_OKAY;
        end else begin
            ready <= (sel && (do_split || we)) || rd_last;
            resp  <= do_split ? `RESP_SPLIT : `RESP_OKAY;
        end
    end

    // DELIBERATE: no reset here.  An async reset on a 4096-word array stops
    // M9K inference and builds the memory from LUTs instead.  mem_read and
    // mem_write are mutually exclusive - a read-during-write is undefined.
    reg [DATA_W-1:0] mem [0:WORDS-1];
    reg [DATA_W-1:0] mem_q;

    always @(posedge clk) begin
        if (mem_write) mem[laddr] <= wdata_des;
        if (mem_read)  mem_q      <= mem[laddr];
    end

    // Separate from mem_q so mem_q stays a plain register Quartus can absorb
    // as the M9K output register.  Merging them forces an async array read.
    reg [DATA_W-1:0] rd_sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              rd_sr <= {DATA_W{1'b0}};
        else if (rs == R_MEMQ)   rd_sr <= mem_q;
        else if (rs == R_SHIFT)  rd_sr <= {rd_sr[DATA_W-2:0], 1'b0};
    end

    assign dstream_out = (rs == R_SHIFT) ? rd_sr[DATA_W-1] : 1'b0;

endmodule
