`timescale 1ns/1ns
// DM16 array core: sequencer + clusters + hypercube network + NEWS torus.
// Board independent - the board top supplies the clock, reset and pins.
//
//   PE_ID = {cluster[DIMS-1:0], lane}
//   NEWS torus: ROWS x COLS, row = PE_ID / COLS, col = PE_ID % COLS
module dm16_core #(
    parameter integer DIMS     = 2,          // hypercube dimensions (clusters = 2^DIMS)
    parameter integer LANES    = 2,          // PEs per cluster (2 or 4)
    parameter integer COLS     = 4,          // NEWS torus width
    parameter integer CLK_HZ   = 54_000_000, // for the UART baud divider
    parameter integer BAUD     = 115_200,    // UART rate after reset (the host may change it)
    parameter integer W        = 16,         // PE data width: 16, 32, 64, ... (a power of two)
    parameter integer GOWIN_SP = 1,          // Gowin BSRAM primitives (0: inferred RAM)
    parameter integer NREGS    = 8,          // PE registers r0..r[NREGS-1]: a power of two, 2..1024
    parameter         RF_BRAM  = 1'b0,       // registers in block RAM (1) or flip-flops (0)
    parameter integer MEM_AW   = 10,         // PE memory = 2^MEM_AW words (<= W; <= 13 with GOWIN_SP)
    parameter         HAS_FPU  = 1'b1,       // bf16 FPU lanes (0: FPU instructions are no-ops)
    parameter         HAS_HDMI = 1'b0,       // frame-buffer ops FBX/FBY/FBW (0: no-ops)
    parameter integer FB_W     = 320,        // frame buffer, RGB332 (with HAS_HDMI)
    parameter integer FB_H     = 240,
    parameter integer FB_BITS  = 8           // frame-buffer bits per pixel (reported to the host)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx,
    output wire       uart_tx,

    // Status for board LEDs
    output wire       running,
    output wire [3:0] quarter_active,   // some PE active in each quarter of the array
    output wire       all_active,

    // Frame-buffer write port, to dm16_hdmi (idle without HAS_HDMI)
    output wire        fb_we,
    output wire [16:0] fb_waddr,
    output wire [7:0]  fb_wdata
);

    localparam integer NODES     = 1 << DIMS;
    localparam integer NUM_PE    = NODES * LANES;
    localparam integer ROWS      = NUM_PE / COLS;
    localparam integer LANE_BITS = (LANES <= 2) ? 1 : 2;
    localparam integer PKT_W     = W + 8;   // {valid, 0, dest[5:0], data}
    localparam integer LINK_W    = 4;

    wire [23:0]         current_instr;
    wire                instr_valid;
    wire [NUM_PE-1:0]   all_ctx;
    wire [NUM_PE*W-1:0] all_acc;
    reg  [NODES-1:0]    cluster_any;   // global OR, stage 1
    reg                 any_active;    // global OR, stage 2 (to the sequencer)

    wire        fb_op, fb_busy;    // frame-buffer ops (HDMI option)
    wire [1:0]  fb_sel;
    wire [15:0] fb_field;

    // 1. UART host loader & global SIMD sequencer
    dm16_uart_seq #(
        .CLK_HZ        (CLK_HZ),
        .BAUD          (BAUD),
        .NUM_PE        (NUM_PE),
        .W             (W),
        .DIMS          (DIMS),
        .LANES         (LANES),
        .COLS          (COLS),
        .NREGS         (NREGS),
        .MEM_AW        (MEM_AW),
        .HAS_FPU       (HAS_FPU),
        .RF_BRAM       (RF_BRAM),
        .HAS_HDMI      (HAS_HDMI),
        .FB_BITS       (FB_BITS),
        .FB_W          (FB_W)
    ) u_seq (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .uart_tx       (uart_tx),
        .current_instr (current_instr),
        .instr_valid   (instr_valid),
        .running       (running),
        .all_ctx       (all_ctx),
        .all_acc       (all_acc),
        .any_active    (any_active),
        .fb_op         (fb_op),
        .fb_sel        (fb_sel),
        .fb_field      (fb_field),
        .fb_busy       (fb_busy)
    );

    // Frame-buffer writer (HDMI option)
    generate
        if (HAS_HDMI) begin : gen_fb
            dm16_fb_writer #(
                .NUM_PE (NUM_PE),
                .W      (W),
                .COLS   (COLS),
                .FB_W   (FB_W),
                .FB_H   (FB_H),
                .FB_AW  (17)
            ) u_fbw (
                .clk      (clk),
                .rst_n    (rst_n),
                .op       (fb_op),
                .sel      (fb_sel),
                .field    (fb_field),
                .all_acc  (all_acc),
                .busy     (fb_busy),
                .fb_we    (fb_we),
                .fb_waddr (fb_waddr),
                .fb_wdata (fb_wdata)
            );
        end else begin : gen_no_fb
            assign fb_busy  = 1'b0;
            assign fb_we    = 1'b0;
            assign fb_waddr = 17'd0;
            assign fb_wdata = 8'd0;
        end
    endgenerate

    // Global OR of every PE's context flag (CM-2 style), two register stages:
    // one OR per cluster, then the OR of the clusters.
    integer c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cluster_any <= {NODES{1'b0}};
            any_active  <= 1'b0;
        end else begin
            for (c = 0; c < NODES; c = c + 1) begin
                cluster_any[c] <= |all_ctx[c*LANES +: LANES];
            end
            any_active <= |cluster_any;
        end
    end

    // 2. NEWS toroidal grid
    wire [NUM_PE*W-1:0] news_out_flat;
    wire [NUM_PE*W-1:0] news_n_flat, news_e_flat, news_w_flat, news_s_flat;

    genvar p;
    generate
        for (p = 0; p < NUM_PE; p = p + 1) begin : gen_news_grid
            localparam integer R      = p / COLS;
            localparam integer C      = p % COLS;
            localparam integer R_PREV = (R + ROWS - 1) % ROWS;
            localparam integer R_NEXT = (R + 1) % ROWS;
            localparam integer C_PREV = (C + COLS - 1) % COLS;
            localparam integer C_NEXT = (C + 1) % COLS;

            assign news_n_flat[p*W +: W] = news_out_flat[(R_PREV * COLS + C) * W +: W];
            assign news_s_flat[p*W +: W] = news_out_flat[(R_NEXT * COLS + C) * W +: W];
            assign news_w_flat[p*W +: W] = news_out_flat[(R * COLS + C_PREV) * W +: W];
            assign news_e_flat[p*W +: W] = news_out_flat[(R * COLS + C_NEXT) * W +: W];
        end
    endgenerate

    // 3. Clusters + hypercube router network
    wire [LINK_W*DIMS*NODES-1:0] node_tx_data;
    wire [DIMS*NODES-1:0]        node_tx_valid;
    wire [DIMS*NODES-1:0]        node_tx_ready;
    wire [LINK_W*DIMS*NODES-1:0] node_rx_data;
    wire [DIMS*NODES-1:0]        node_rx_valid;
    wire [DIMS*NODES-1:0]        node_rx_ready;

    genvar n, dim;
    generate
        for (n = 0; n < NODES; n = n + 1) begin : gen_nodes
            wire [PKT_W-1:0] rt_in_pkt, rt_out_pkt;
            wire             rt_in_valid, rt_in_ready, rt_out_valid, rt_out_ready;

            dm16_cluster #(
                .LANES    (LANES),
                .BASE_ID  (n * LANES),
                .W        (W),
                .GOWIN_SP (GOWIN_SP),
                .NREGS    (NREGS),
                .RF_BRAM  (RF_BRAM),
                .MEM_AW   (MEM_AW),
                .HAS_FPU  (HAS_FPU)
            ) u_cluster (
                .clk          (clk),
                .rst_n        (rst_n),
                .instr        (current_instr),
                .instr_valid  (instr_valid),
                .news_n_in    (news_n_flat[n*LANES*W +: LANES*W]),
                .news_e_in    (news_e_flat[n*LANES*W +: LANES*W]),
                .news_w_in    (news_w_flat[n*LANES*W +: LANES*W]),
                .news_s_in    (news_s_flat[n*LANES*W +: LANES*W]),
                .news_out     (news_out_flat[n*LANES*W +: LANES*W]),
                .rt_in_pkt    (rt_in_pkt),
                .rt_in_valid  (rt_in_valid),
                .rt_in_ready  (rt_in_ready),
                .rt_out_pkt   (rt_out_pkt),
                .rt_out_valid (rt_out_valid),
                .rt_out_ready (rt_out_ready),
                .ctx_active   (all_ctx[n*LANES +: LANES]),
                .acc_out      (all_acc[n*LANES*W +: LANES*W])
            );

            hypercube_router #(
                .DIMS     (DIMS),
                .MY_NODE  (n),
                .PKT_W    (PKT_W),
                .LINK_W   (LINK_W),
                .NODE_LSB (W + LANE_BITS)      // dest = {cluster, lane}
            ) u_router (
                .clk             (clk),
                .rst_n           (rst_n),
                .local_in_pkt    (rt_in_pkt),
                .local_in_valid  (rt_in_valid),
                .local_in_ready  (rt_in_ready),
                .local_out_pkt   (rt_out_pkt),
                .local_out_valid (rt_out_valid),
                .local_out_ready (rt_out_ready),
                .dim_tx_data     (node_tx_data [n*LINK_W*DIMS +: LINK_W*DIMS]),
                .dim_tx_valid    (node_tx_valid[n*DIMS        +: DIMS]),
                .dim_tx_ready    (node_tx_ready[n*DIMS        +: DIMS]),
                .dim_rx_data     (node_rx_data [n*LINK_W*DIMS +: LINK_W*DIMS]),
                .dim_rx_valid    (node_rx_valid[n*DIMS        +: DIMS]),
                .dim_rx_ready    (node_rx_ready[n*DIMS        +: DIMS])
            );

            // Hypercube links: neighbour across dimension d is n ^ (1 << d)
            for (dim = 0; dim < DIMS; dim = dim + 1) begin : gen_hc_links
                localparam integer NBR = n ^ (1 << dim);
                assign node_rx_data [n*LINK_W*DIMS + dim*LINK_W +: LINK_W] =
                       node_tx_data [NBR*LINK_W*DIMS + dim*LINK_W +: LINK_W];
                assign node_rx_valid[n*DIMS + dim] = node_tx_valid[NBR*DIMS + dim];
                assign node_tx_ready[n*DIMS + dim] = node_rx_ready[NBR*DIMS + dim];
            end
        end
    endgenerate

    // 4. Status
    localparam integer Q = NUM_PE / 4;
    assign quarter_active[0] = |all_ctx[0*Q +: Q];
    assign quarter_active[1] = |all_ctx[1*Q +: Q];
    assign quarter_active[2] = |all_ctx[2*Q +: Q];
    assign quarter_active[3] = |all_ctx[3*Q +: Q];
    assign all_active        = &all_ctx;

endmodule
