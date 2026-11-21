`timescale 1ns/1ns
// Hypercube router: one per cluster, DIMS-dimensional binary hypercube.
//
// Packet (PKT_W bits). The destination cluster sits at bit NODE_LSB:
//   pkt[NODE_LSB +: DIMS] = destination cluster/node
// (the cluster decodes the rest: PE lane and data).
//
// Links are LINK_W bits wide; a packet takes PKT_W / LINK_W beats.
// Routing is e-cube
// (dimension-ordered, lowest differing dimension first), which is
// deadlock-free on a hypercube.
module hypercube_router #(
    parameter integer DIMS     = 3,
    parameter [DIMS-1:0] MY_NODE = 0,
    parameter integer PKT_W    = 24,
    parameter integer LINK_W   = 4,
    parameter integer NODE_LSB = 17
)(
    input  wire              clk,
    input  wire              rst_n,

    // Local cluster injection / ejection
    input  wire [PKT_W-1:0]  local_in_pkt,
    input  wire              local_in_valid,
    output wire              local_in_ready,

    output reg  [PKT_W-1:0]  local_out_pkt,
    output reg               local_out_valid,
    input  wire              local_out_ready,

    // DIMS hypercube links (LINK_W bits each, packed)
    output reg  [LINK_W*DIMS-1:0] dim_tx_data,
    output reg  [DIMS-1:0]   dim_tx_valid,
    input  wire [DIMS-1:0]   dim_tx_ready,

    input  wire [LINK_W*DIMS-1:0] dim_rx_data,
    input  wire [DIMS-1:0]   dim_rx_valid,
    output wire [DIMS-1:0]   dim_rx_ready
);

    localparam integer NSRC  = DIMS + 1;   // DIMS link inputs + local
    localparam integer LOCAL = DIMS;       // index of the local source/port
    localparam integer BEATS = PKT_W / LINK_W;

    // ---------------------------------------------------------
    // 1. Deserializers for incoming links
    // ---------------------------------------------------------
    reg [PKT_W-1:0] rx_shift [0:DIMS-1];
    reg [3:0]       rx_cnt   [0:DIMS-1];
    reg [PKT_W-1:0] rx_buf   [0:DIMS-1];
    reg [DIMS-1:0] rx_buf_vld;
    reg [DIMS-1:0] rx_buf_ack;

    genvar d;
    generate
        for (d = 0; d < DIMS; d = d + 1) begin : gen_rx
            assign dim_rx_ready[d] = !rx_buf_vld[d];

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    rx_shift[d]   <= {PKT_W{1'b0}};
                    rx_cnt[d]     <= 4'd0;
                    rx_buf[d]     <= {PKT_W{1'b0}};
                    rx_buf_vld[d] <= 1'b0;
                end else begin
                    if (rx_buf_ack[d])
                        rx_buf_vld[d] <= 1'b0;

                    if (dim_rx_valid[d] && dim_rx_ready[d]) begin
                        rx_shift[d] <= {dim_rx_data[d*LINK_W +: LINK_W], rx_shift[d][PKT_W-1:LINK_W]};
                        if (rx_cnt[d] == BEATS - 1) begin
                            rx_buf[d]     <= {dim_rx_data[d*LINK_W +: LINK_W], rx_shift[d][PKT_W-1:LINK_W]};
                            rx_buf_vld[d] <= 1'b1;
                            rx_cnt[d]     <= 4'd0;
                        end else begin
                            rx_cnt[d] <= rx_cnt[d] + 4'd1;
                        end
                    end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------
    // 2. Round-robin arbiter over NSRC sources
    // ---------------------------------------------------------
    wire [NSRC-1:0] src_valid = {local_in_valid, rx_buf_vld};

    reg [3:0] rr_ptr;
    reg [3:0] sel_src;
    reg       sel_found;

    integer k;
    reg [4:0] idx;
    always @(*) begin
        sel_found = 1'b0;
        sel_src   = 4'd0;
        for (k = 0; k < NSRC; k = k + 1) begin
            idx = {1'b0, rr_ptr} + k[4:0];
            if (idx >= NSRC) idx = idx - NSRC;
            if (!sel_found && src_valid[idx]) begin
                sel_found = 1'b1;
                sel_src   = idx[3:0];
            end
        end
    end

    reg [PKT_W-1:0] active_pkt;
    always @(*) begin
        active_pkt = local_in_pkt;
        for (k = 0; k < DIMS; k = k + 1)
            if (sel_src == k[3:0]) active_pkt = rx_buf[k];
    end

    // ---------------------------------------------------------
    // 3. E-cube next-hop decode (0..DIMS-1 = link, LOCAL = eject)
    // ---------------------------------------------------------
    wire [DIMS-1:0] dest_node = active_pkt[NODE_LSB +: DIMS];
    wire [DIMS-1:0] diff      = MY_NODE ^ dest_node;

    reg [3:0] target_port;
    always @(*) begin
        target_port = LOCAL;
        for (k = DIMS - 1; k >= 0; k = k - 1)
            if (diff[k]) target_port = k[3:0];   // lowest set bit wins
    end

    // ---------------------------------------------------------
    // 4. Serializers for outgoing links
    // ---------------------------------------------------------
    reg [PKT_W-1:0] tx_shift [0:DIMS-1];
    reg [3:0]       tx_cnt   [0:DIMS-1];
    reg [DIMS-1:0] tx_busy;

    wire port_ready = (target_port == LOCAL) ? (!local_out_valid || local_out_ready)
                                             : !tx_busy[target_port];

    assign local_in_ready = sel_found && (sel_src == LOCAL) && port_ready;

    always @(*) begin
        rx_buf_ack = {DIMS{1'b0}};
        if (sel_found && (sel_src != LOCAL) && port_ready)
            rx_buf_ack[sel_src] = 1'b1;
    end

    // Serializer outputs come straight from the shift register so the
    // receiver always samples the chunk that the transmitter is retiring.
    integer q;
    always @(*) begin
        dim_tx_valid = tx_busy;
        for (q = 0; q < DIMS; q = q + 1)
            dim_tx_data[q*LINK_W +: LINK_W] = tx_shift[q][LINK_W-1:0];
    end

    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr          <= 4'd0;
            local_out_pkt   <= {PKT_W{1'b0}};
            local_out_valid <= 1'b0;
            tx_busy         <= {DIMS{1'b0}};
            for (p = 0; p < DIMS; p = p + 1) begin
                tx_shift[p] <= {PKT_W{1'b0}};
                tx_cnt[p]   <= 4'd0;
            end
        end else begin
            if (local_out_ready)
                local_out_valid <= 1'b0;

            for (p = 0; p < DIMS; p = p + 1) begin
                if (tx_busy[p] && dim_tx_ready[p]) begin
                    tx_shift[p] <= {{LINK_W{1'b0}}, tx_shift[p][PKT_W-1:LINK_W]};
                    if (tx_cnt[p] == BEATS - 1) begin
                        tx_busy[p] <= 1'b0;
                        tx_cnt[p]  <= 4'd0;
                    end else begin
                        tx_cnt[p] <= tx_cnt[p] + 4'd1;
                    end
                end
            end

            if (sel_found && port_ready) begin
                rr_ptr <= (sel_src == NSRC - 1) ? 4'd0 : sel_src + 4'd1;
                if (target_port == LOCAL) begin
                    local_out_pkt   <= active_pkt;
                    local_out_valid <= 1'b1;
                end else begin
                    for (p = 0; p < DIMS; p = p + 1)
                        if (target_port == p[3:0]) begin
                            tx_shift[p] <= active_pkt;
                            tx_busy[p]  <= 1'b1;
                            tx_cnt[p]   <= 4'd0;
                        end
                end
            end
        end
    end

endmodule
