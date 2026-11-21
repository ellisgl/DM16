// DM8 array core: sequencer + clusters + hypercube network + NEWS torus.
// Board independent - the board top supplies the clock, reset and pins.
//
//   PE_ID = {cluster[DIMS-1:0], lane}
//   NEWS torus: ROWS x COLS, row = PE_ID / COLS, col = PE_ID % COLS
module dm8_core #(
    parameter integer DIMS   = 2,          // hypercube dimensions (clusters = 2^DIMS)
    parameter integer LANES  = 2,          // PEs per cluster (2 or 4)
    parameter integer COLS   = 4,          // NEWS torus width
    parameter integer CLK_HZ = 54_000_000  // for the UART baud divider
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx,
    output wire       uart_tx,

    // Status for board LEDs
    output wire       running,
    output wire [3:0] quarter_active,   // some PE active in each quarter of the array
    output wire       all_active
);

    localparam integer NODES     = 1 << DIMS;
    localparam integer NUM_PE    = NODES * LANES;
    localparam integer ROWS      = NUM_PE / COLS;
    localparam integer W         = 16;
    localparam integer LANE_BITS = (LANES <= 2) ? 1 : 2;
    localparam integer PKT_W     = 24;
    localparam integer LINK_W    = 4;

    wire [23:0]         current_instr;
    wire                instr_valid;
    wire [NUM_PE-1:0]   all_ctx;
    wire [NUM_PE*W-1:0] all_acc;

    // 1. UART host loader & global SIMD sequencer
    dm8_uart_seq #(
        .CLK_HZ        (CLK_HZ),
        .NUM_PE        (NUM_PE)
    ) u_seq (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .uart_tx       (uart_tx),
        .current_instr (current_instr),
        .instr_valid   (instr_valid),
        .running       (running),
        .all_ctx       (all_ctx),
        .all_acc       (all_acc)
    );

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

            dm8_cluster #(
                .LANES   (LANES),
                .BASE_ID (n * LANES)
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
                .NODE_LSB (16 + LANE_BITS)     // dest = {cluster, lane}
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
