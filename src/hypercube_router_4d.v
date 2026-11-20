module hypercube_router_4d #(
    parameter [3:0] MY_NODE = 4'd0
)(
    input  wire        clk,
    input  wire        rst_n,

    // Local 4-PE Cluster Injection / Ejection
    input  wire [15:0] local_in_pkt,
    input  wire        local_in_valid,
    output wire        local_in_ready,

    output reg  [15:0] local_out_pkt,
    output reg         local_out_valid,
    input  wire        local_out_ready,

    // 4 Hypercube Dimension Links (4 dims x 2-bit = 8-bit packed bus)
    output reg  [7:0]  dim_tx_data,
    output reg  [3:0]  dim_tx_valid,
    input  wire [3:0]  dim_tx_ready,

    input  wire [7:0]  dim_rx_data,
    input  wire [3:0]  dim_rx_valid,
    output wire [3:0]  dim_rx_ready
);

    // ---------------------------------------------------------
    // 1. 2-Bit Deserializers for Incoming Dimension Links
    // ---------------------------------------------------------
    reg [15:0] rx_shift   [0:3];
    reg [2:0]  rx_cnt     [0:3];
    reg [15:0] rx_buf     [0:3];
    reg [3:0]  rx_buf_vld;
    reg [3:0]  rx_buf_ack;

    genvar d;
    generate
        for (d = 0; d < 4; d = d + 1) begin : gen_rx
            assign dim_rx_ready[d] = !rx_buf_vld[d];

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    rx_shift[d]   <= 16'd0;
                    rx_cnt[d]     <= 3'd0;
                    rx_buf[d]     <= 16'd0;
                    rx_buf_vld[d] <= 1'b0;
                end else begin
                    if (rx_buf_ack[d])
                        rx_buf_vld[d] <= 1'b0;

                    if (dim_rx_valid[d] && dim_rx_ready[d]) begin
                        rx_shift[d] <= {dim_rx_data[d*2 +: 2], rx_shift[d][15:2]};
                        if (rx_cnt[d] == 3'd7) begin
                            rx_buf[d]     <= {dim_rx_data[d*2 +: 2], rx_shift[d][15:2]};
                            rx_buf_vld[d] <= 1'b1;
                            rx_cnt[d]     <= 3'd0;
                        end else begin
                            rx_cnt[d] <= rx_cnt[d] + 3'd1;
                        end
                    end
                end
            end
        end
    endgenerate

    // ---------------------------------------------------------
    // 2. Round-Robin Arbiter across 5 Input Sources (Dims 0..3 + Local)
    // ---------------------------------------------------------
    wire [4:0]  src_valid = {local_in_valid, rx_buf_vld};
    wire [15:0] src_pkt [0:4];
    assign src_pkt[0] = rx_buf[0];
    assign src_pkt[1] = rx_buf[1];
    assign src_pkt[2] = rx_buf[2];
    assign src_pkt[3] = rx_buf[3];
    assign src_pkt[4] = local_in_pkt;

    reg [2:0] rr_ptr;
    reg [2:0] sel_src;
    reg       sel_found;

    integer k;
    reg [3:0] idx;
    always @(*) begin
        sel_found = 1'b0;
        sel_src   = 3'd0;
        for (k = 0; k < 5; k = k + 1) begin
            idx = (({1'b0, rr_ptr} + k[3:0]) % 4'd5);
            if (!sel_found && src_valid[idx]) begin
                sel_found = 1'b1;
                sel_src   = idx[2:0];
            end
        end
    end

    // ---------------------------------------------------------
    // 3. E-Cube Dimension-Ordered Next-Hop Decoder
    // ---------------------------------------------------------
    wire [15:0] active_pkt = src_pkt[sel_src];
    wire [3:0]  dest_node  = active_pkt[14:11];
    wire [3:0]  diff       = MY_NODE ^ dest_node;

    // 0..3 = Outgoing Dimension Port, 4 = Local Eject
    reg [2:0] target_port;
    always @(*) begin
        if (diff == 4'b0000)  target_port = 3'd4; // Arrived at destination node
        else if (diff[0:0])   target_port = 3'd0; // Route along Dim 0
        else if (diff[1:1])   target_port = 3'd1; // Route along Dim 1
        else if (diff[2:2])   target_port = 3'd2; // Route along Dim 2
        else                  target_port = 3'd3; // Route along Dim 3
    end

    // ---------------------------------------------------------
    // 4. 2-Bit Serializers for Outgoing Dimension Links
    // ---------------------------------------------------------
    reg [15:0] tx_shift [0:3];
    reg [2:0]  tx_cnt   [0:3];
    reg [3:0]  tx_busy;

    wire port_ready = (target_port == 3'd4) ? (!local_out_valid || local_out_ready)
                                            : (!tx_busy[target_port[1:0]]);

    assign local_in_ready = sel_found && (sel_src == 3'd4) && port_ready;

    always @(*) begin
        rx_buf_ack = 4'b0000;
        if (sel_found && (sel_src < 3'd4) && port_ready)
            rx_buf_ack[sel_src[1:0]] = 1'b1;
    end

    // Serializer outputs come straight from the shift register so the
    // receiver always samples the chunk that the transmitter is retiring.
    integer q;
    always @(*) begin
        dim_tx_valid = tx_busy;
        for (q = 0; q < 4; q = q + 1)
            dim_tx_data[q*2 +: 2] = tx_shift[q][1:0];
    end

    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr          <= 3'd0;
            local_out_pkt   <= 16'd0;
            local_out_valid <= 1'b0;
            tx_busy         <= 4'b0000;
            for (p = 0; p < 4; p = p + 1) begin
                tx_shift[p] <= 16'd0;
                tx_cnt[p]   <= 3'd0;
            end
        end else begin
            if (local_out_ready)
                local_out_valid <= 1'b0;

            // Advance active 2-bit serializers (data/valid driven combinationally below)
            for (p = 0; p < 4; p = p + 1) begin
                if (tx_busy[p] && dim_tx_ready[p]) begin
                    tx_shift[p] <= {2'b00, tx_shift[p][15:2]};
                    if (tx_cnt[p] == 3'd7) begin
                        tx_busy[p] <= 1'b0;
                        tx_cnt[p]  <= 3'd0;
                    end else begin
                        tx_cnt[p] <= tx_cnt[p] + 3'd1;
                    end
                end
            end

            // Dispatch winning packet to target port
            if (sel_found && port_ready) begin
                rr_ptr <= (sel_src + 3'd1) % 3'd5;
                if (target_port == 3'd4) begin
                    local_out_pkt   <= active_pkt;
                    local_out_valid <= 1'b1;
                end else begin
                    tx_shift[target_port[1:0]] <= active_pkt;
                    tx_busy[target_port[1:0]]  <= 1'b1;
                    tx_cnt[target_port[1:0]]   <= 3'd0;
                end
            end
        end
    end

endmodule
