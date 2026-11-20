module dm8_top (
    input  wire       clk_27m,    // Pin 4 on Tang Nano 20K
    input  wire       rst_btn,    // Pin 88 (Active High button)
    input  wire       uart_rx,    // Pin 70 (from BL616 USB-UART)
    output wire       uart_tx,    // Pin 69 (to BL616 USB-UART)
    output wire [5:0] led         // Pins 15..20 (Active Low)
);

    // Put the 27 MHz pad clock onto the global clock tree. Without this the
    // PnR used generic fabric routing for the clock (warning PR1014), giving
    // uncharacterised skew between sequencer flops and the BRAM ports.
    wire clk;
    BUFG u_clk_bufg (.I(clk_27m), .O(clk));

    wire        rst_n = ~rst_btn;
    wire [23:0] current_instr;
    wire        instr_valid;
    wire        running;
    wire [63:0] all_ctx;
    wire [511:0] all_acc;
    
    // 1. UART Host Loader & Global SIMD Sequencer
    dm8_uart_seq u_seq (
        .clk_27m       (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .uart_tx       (uart_tx),
        .current_instr (current_instr),
        .instr_valid   (instr_valid),
        .running       (running),
        .all_ctx       (all_ctx),
        .all_acc (all_acc)
    );

    // 2. 8x8 2D NEWS Toroidal Grid Interconnect (row = id[5:3], col = id[2:0])
    wire [7:0] pe_news_out [0:63];
    wire [7:0] pe_news_n   [0:63];
    wire [7:0] pe_news_e   [0:63];
    wire [7:0] pe_news_w   [0:63];
    wire [7:0] pe_news_s   [0:63];

    genvar idx_pe;
    generate
        for (idx_pe = 0; idx_pe < 64; idx_pe = idx_pe + 1) begin : gen_news_grid
            localparam [2:0] R = idx_pe[5:3];
            localparam [2:0] C = idx_pe[2:0];
            localparam [2:0] R_PREV = R - 3'd1;
            localparam [2:0] R_NEXT = R + 3'd1;
            localparam [2:0] C_PREV = C - 3'd1;
            localparam [2:0] C_NEXT = C + 3'd1;

            assign pe_news_n[idx_pe] = pe_news_out[(R_PREV * 8) + C];
            assign pe_news_s[idx_pe] = pe_news_out[(R_NEXT * 8) + C];
            assign pe_news_w[idx_pe] = pe_news_out[(R * 8) + C_PREV];
            assign pe_news_e[idx_pe] = pe_news_out[(R * 8) + C_NEXT];
        end
    endgenerate

    // 3. 16 Clusters (64 PEs) + 4D Hypercube Router Network
    wire [7:0] node_tx_data  [0:15];
    wire [3:0] node_tx_valid [0:15];
    wire [3:0] node_tx_ready [0:15];
    wire [7:0] node_rx_data  [0:15];
    wire [3:0] node_rx_valid [0:15];
    wire [3:0] node_rx_ready [0:15];

    genvar n, dim;
    generate
        for (n = 0; n < 16; n = n + 1) begin : gen_nodes
            wire [63:0] cl_tx_pkt, cl_rx_pkt;
            wire [3:0]  cl_tx_vld, cl_tx_rdy, cl_rx_vld, cl_rx_pop;

            wire [31:0] n_in = {pe_news_n[n*4+3], pe_news_n[n*4+2], pe_news_n[n*4+1], pe_news_n[n*4+0]};
            wire [31:0] e_in = {pe_news_e[n*4+3], pe_news_e[n*4+2], pe_news_e[n*4+1], pe_news_e[n*4+0]};
            wire [31:0] w_in = {pe_news_w[n*4+3], pe_news_w[n*4+2], pe_news_w[n*4+1], pe_news_w[n*4+0]};
            wire [31:0] s_in = {pe_news_s[n*4+3], pe_news_s[n*4+2], pe_news_s[n*4+1], pe_news_s[n*4+0]};
            wire [31:0] out_bus;

            assign pe_news_out[n*4+0] = out_bus[7:0];
            assign pe_news_out[n*4+1] = out_bus[15:8];
            assign pe_news_out[n*4+2] = out_bus[23:16];
            assign pe_news_out[n*4+3] = out_bus[31:24];

            dm8_cluster #(
                .NODE_ID(n[3:0])
            ) u_cluster (
                .clk         (clk),
                .rst_n       (rst_n),
                .instr       (current_instr),
                .instr_valid (instr_valid),
                .news_n_in   (n_in),
                .news_e_in   (e_in),
                .news_w_in   (w_in),
                .news_s_in   (s_in),
                .news_out    (out_bus),
                .pe_tx_pkt   (cl_tx_pkt),
                .pe_tx_valid (cl_tx_vld),
                .pe_tx_ready (cl_tx_rdy),
                .pe_rx_pkt   (cl_rx_pkt),
                .pe_rx_valid (cl_rx_vld),
                .pe_rx_pop   (cl_rx_pop),
                .ctx_active  (all_ctx[n*4 +: 4]),
                .acc_out (all_acc[n*32 +: 32])
            );

            // 4-to-1 Local Cluster Injection Arbiter
            wire [15:0] loc_in_pkt = cl_tx_vld[0:0] ? cl_tx_pkt[15:0]  :
                                     cl_tx_vld[1:1] ? cl_tx_pkt[31:16] :
                                     cl_tx_vld[2:2] ? cl_tx_pkt[47:32] : cl_tx_pkt[63:48];
            wire        loc_in_vld = |cl_tx_vld;
            wire        loc_in_rdy;
            assign cl_tx_rdy[0:0] = loc_in_rdy;
            assign cl_tx_rdy[1:1] = loc_in_rdy && !cl_tx_vld[0:0];
            assign cl_tx_rdy[2:2] = loc_in_rdy && !(|cl_tx_vld[1:0]);
            assign cl_tx_rdy[3:3] = loc_in_rdy && !(|cl_tx_vld[2:0]);

            // 1-to-4 Local Cluster Ejection Decoder
            wire [15:0] loc_out_pkt;
            wire        loc_out_vld;
            wire [1:0]  dest_pe = loc_out_pkt[10:9];
            wire [3:0]  pe_mask = (dest_pe == 2'd0) ? 4'b0001 :
                                  (dest_pe == 2'd1) ? 4'b0010 :
                                  (dest_pe == 2'd2) ? 4'b0100 : 4'b1000;
            assign cl_rx_pkt = {4{loc_out_pkt}};
            assign cl_rx_vld = loc_out_vld ? pe_mask : 4'b0000;
            wire loc_out_rdy = |(cl_rx_pop & cl_rx_vld);

            hypercube_router_4d #(
                .MY_NODE(n[3:0])
            ) u_router (
                .clk             (clk),
                .rst_n           (rst_n),
                .local_in_pkt    (loc_in_pkt),
                .local_in_valid  (loc_in_vld),
                .local_in_ready  (loc_in_rdy),
                .local_out_pkt   (loc_out_pkt),
                .local_out_valid (loc_out_vld),
                .local_out_ready (loc_out_rdy),
                .dim_tx_data     (node_tx_data[n]),
                .dim_tx_valid    (node_tx_valid[n]),
                .dim_tx_ready    (node_tx_ready[n]),
                .dim_rx_data     (node_rx_data[n]),
                .dim_rx_valid    (node_rx_valid[n]),
                .dim_rx_ready    (node_rx_ready[n])
            );

            // 4D Hypercube Inter-Node Links: neighbor = n ^ (1, 2, 4, or 8)
            for (dim = 0; dim < 4; dim = dim + 1) begin : gen_hc_links
                localparam integer MASK = (dim == 0) ? 1 : (dim == 1) ? 2 : (dim == 2) ? 4 : 8;
                localparam integer NBR  = n ^ MASK;
                assign node_rx_data[n][dim*2 +: 2] = node_tx_data[NBR][dim*2 +: 2];
                assign node_rx_valid[n][dim]       = node_tx_valid[NBR][dim];
                assign node_tx_ready[n][dim]       = node_rx_ready[NBR][dim];
            end
        end
    endgenerate

    // 4. Onboard Blinkenlights (Active-Low LEDs)
    assign led[0:0] = ~running;           // LED 0: Sequencer Running
    assign led[1:1] = ~(|all_ctx[15:0]);  // LED 1: Quadrant 0 Active
    assign led[2:2] = ~(|all_ctx[31:16]); // LED 2: Quadrant 1 Active
    assign led[3:3] = ~(|all_ctx[47:32]); // LED 3: Quadrant 2 Active
    assign led[4:4] = ~(|all_ctx[63:48]); // LED 4: Quadrant 3 Active
    assign led[5:5] = ~(&all_ctx);        // LED 5: All 64 PEs Active

endmodule
