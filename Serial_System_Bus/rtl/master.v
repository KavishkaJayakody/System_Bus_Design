// Bus master: takes one parallel command, shifts it onto the two-wire bus, and
// runs it to completion including replaying it after a SPLIT.

`include "bus_defs.vh"

module master #(
    parameter ADDR_W = `BUS_ADDR_W,
    parameter DATA_W = `BUS_DATA_W,     // must be <= ADDR_W
    parameter RESP_W = `BUS_RESP_W
) (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                cmd_valid,
    input  wire                cmd_we,
    input  wire [ADDR_W-1:0]   cmd_addr,
    input  wire [DATA_W-1:0]   cmd_wdata,
    output reg                 cmd_accept,
    output reg                 done,
    output reg  [DATA_W-1:0]   rdata,
    output reg  [RESP_W-1:0]   resp,
    output wire                err,
    output reg  [7:0]          split_count,
    output wire                busy,
    output wire [2:0]          state,

    output reg                 bus_req,
    input  wire                bus_gnt,
    output reg                 m_valid,
    output wire                m_we,
    output wire                m_astream,
    output wire                m_dstream,
    input  wire                bus_ready,
    input  wire [RESP_W-1:0]   bus_resp,
    input  wire                bus_dstream
);

    localparam ST_IDLE    = 3'd0;
    localparam ST_REQ     = 3'd1;
    localparam ST_ASHIFT  = 3'd2;
    localparam ST_WAIT    = 3'd3;
    localparam ST_SPLIT_W = 3'd4;
    localparam ST_DONE    = 3'd5;

    function integer clogb2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clogb2 = 1; v > 1; clogb2 = clogb2 + 1)
                v = v >> 1;
        end
    endfunction
    localparam CNT_W = clogb2(ADDR_W);

    reg [2:0]         cs, ns;
    reg               r_we;
    reg [ADDR_W-1:0]  r_addr;
    reg [DATA_W-1:0]  r_wdata;
    reg [CNT_W-1:0]   bitcnt;
    reg [CNT_W-1:0]   wcnt;

    reg               ser_load, ser_shift, rd_shift;

    assign state = cs;
    assign busy  = (cs != ST_IDLE);
    assign m_we  = r_we;
    assign err   = (resp == `RESP_ERROR);

    wire split_now = bus_ready && (bus_resp == `RESP_SPLIT);

    // data zero-extended.  That is what right-aligns the write data in the
    // The data serialiser is ADDR_W wide and zero-extended: that is what
    // right-aligns the write data, so receivers need no bit counter.
    wire [ADDR_W-1:0] wdata_padded = r_wdata;      // zero-extended
    wire              dstream_raw;

    shift_ser #(.W(ADDR_W)) u_addr_ser (
        .clk(clk), .rst_n(rst_n),
        .load(ser_load), .shift(ser_shift),
        .din(r_addr), .dout(m_astream)
    );

    shift_ser #(.W(ADDR_W)) u_wdata_ser (
        .clk(clk), .rst_n(rst_n),
        .load(ser_load), .shift(ser_shift),
        .din(wdata_padded), .dout(dstream_raw)
    );

    assign m_dstream = r_we ? dstream_raw : 1'b0;

    wire [DATA_W-1:0] rdata_ser;

    shift_deser #(.W(DATA_W)) u_rdata_deser (
        .clk(clk), .rst_n(rst_n),
        .shift(rd_shift), .din(bus_dstream), .dout(rdata_ser)
    );

    always @* begin
        ns         = cs;
        cmd_accept = 1'b0;
        bus_req    = 1'b0;
        m_valid    = 1'b0;
        done       = 1'b0;
        ser_load   = 1'b0;
        ser_shift  = 1'b0;
        rd_shift   = 1'b0;

        case (cs)
            ST_IDLE: begin
                if (cmd_valid) begin
                    cmd_accept = 1'b1;
                    ns         = ST_REQ;
                end
            end

            ST_REQ: begin
                bus_req  = 1'b1;
                ser_load = 1'b1;              // primed, ready for the grant
                if (bus_gnt) ns = ST_ASHIFT;
            end

            ST_ASHIFT: begin
                bus_req   = 1'b1;
                m_valid   = 1'b1;             // the frame
                ser_shift = 1'b1;
                if (bitcnt == (ADDR_W-1)) ns = ST_WAIT;
            end

            ST_WAIT: begin
                bus_req  = 1'b1;
                rd_shift = 1'b1;              // collect whatever comes back
                if (bus_ready)
                    ns = split_now ? ST_SPLIT_W : ST_DONE;
            end

            ST_SPLIT_W: begin
                bus_req  = 1'b1;              // held: the arbiter mask, not
                ser_load = 1'b1;              // reload for the replay
                if (bus_gnt) ns = ST_ASHIFT;
            end

            ST_DONE: begin
                done = 1'b1;
                ns   = ST_IDLE;
            end

            default: ns = ST_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs          <= ST_IDLE;
            r_we        <= 1'b0;
            r_addr      <= {ADDR_W{1'b0}};
            r_wdata     <= {DATA_W{1'b0}};
            rdata       <= {DATA_W{1'b0}};
            resp        <= `RESP_OKAY;
            split_count <= 8'd0;
            bitcnt      <= {CNT_W{1'b0}};
            wcnt        <= {CNT_W{1'b0}};
        end else begin
            cs <= ns;

            if (cs == ST_ASHIFT) bitcnt <= bitcnt + 1'b1;
            else                 bitcnt <= {CNT_W{1'b0}};

            if (cs != ST_WAIT)             wcnt <= {CNT_W{1'b0}};
            else if (wcnt != {CNT_W{1'b1}}) wcnt <= wcnt + 1'b1;

            if (cs == ST_IDLE && cmd_valid) begin
                r_we    <= cmd_we;
                r_addr  <= cmd_addr;
                r_wdata <= cmd_wdata;
            end

            if (cs == ST_WAIT && bus_ready) begin
                if (split_now) begin
                    if (split_count != 8'hFF)
                        split_count <= split_count + 8'd1;
                end else begin
                    resp <= bus_resp;
                    // Not every ERROR is instant: the bridge shifts a real
                    // byte out and THEN reports a timeout (0xFF).  So the
                    // test is "could a byte have arrived", not "was it ERROR".
                    if (!r_we)
                        rdata <= (bus_resp == `RESP_ERROR && wcnt < DATA_W)
                                     ? {DATA_W{1'b0}} : rdata_ser;
                end
            end
        end
    end

endmodule
