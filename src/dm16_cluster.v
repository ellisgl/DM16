// One cluster: LANES PEs, each with its own single-port BSRAM (1024 x 16),
// a LANES-to-1 network injection arbiter and a 1-deep receive mailbox per PE.
//
// Packet: {valid, 1'b0, dest[5:0], data[15:0]}
//   dest = {cluster, lane}; the lane is dest[LANE_BITS-1:0] (bits 16 +: LANE_BITS).
module dm16_cluster #(
    parameter integer LANES   = 2,       // PEs in this cluster (2 or 4)
    parameter [5:0]   BASE_ID = 6'd0     // PE_ID of lane 0 (cluster * LANES)
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [23:0]          instr,
    input  wire                 instr_valid,

    // NEWS grid (LANES PEs x 16 bits)
    input  wire [LANES*16-1:0]  news_n_in,
    input  wire [LANES*16-1:0]  news_e_in,
    input  wire [LANES*16-1:0]  news_w_in,
    input  wire [LANES*16-1:0]  news_s_in,
    output wire [LANES*16-1:0]  news_out,

    // Router local port
    output reg  [23:0]          rt_in_pkt,
    output wire                 rt_in_valid,
    input  wire                 rt_in_ready,
    input  wire [23:0]          rt_out_pkt,
    input  wire                 rt_out_valid,
    output wire                 rt_out_ready,

    // Status & host readback
    output wire [LANES-1:0]     ctx_active,
    output wire [LANES*16-1:0]  acc_out
);

    localparam integer LANE_BITS = (LANES <= 2) ? 1 : 2;

    // Instruction broadcast pipeline: one copy per cluster, then one per PE.
    // Every PE sees the sequencer's instruction two clocks later; since
    // everything (memory address, valid, write-back) shifts together, the
    // instruction rate and all relative timing are unchanged. Each register
    // only drives nearby logic, which keeps the 64-way fan-out off the PE
    // paths. syn_preserve stops synthesis merging the copies back together.
    (* syn_preserve = 1 *) reg [23:0] instr_q;
    (* syn_preserve = 1 *) reg        instr_valid_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            instr_q       <= 24'd0;
            instr_valid_q <= 1'b0;
        end else begin
            instr_q       <= instr;
            instr_valid_q <= instr_valid;
        end
    end

    wire [24*LANES-1:0] tx_pkt;
    wire [LANES-1:0]    tx_valid;
    reg  [LANES-1:0]    tx_ready;
    wire [LANES-1:0]    rx_pop;

    // Receive mailboxes
    reg  [15:0]         mbox [0:LANES-1];
    reg  [LANES-1:0]    mbox_full;

    wire [LANE_BITS-1:0] out_lane = rt_out_pkt[16 +: LANE_BITS];
    assign rt_out_ready = !mbox_full[out_lane];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mbox_full <= {LANES{1'b0}};
            for (k = 0; k < LANES; k = k + 1)
                mbox[k] <= 16'd0;
        end else begin
            for (k = 0; k < LANES; k = k + 1)
                if (rx_pop[k]) mbox_full[k] <= 1'b0;
            if (rt_out_valid && rt_out_ready) begin
                mbox[out_lane]      <= rt_out_pkt[15:0];
                mbox_full[out_lane] <= 1'b1;
            end
        end
    end

    // Injection: fixed priority, lowest lane first
    assign rt_in_valid = |tx_valid;
    always @(*) begin
        rt_in_pkt = tx_pkt[23:0];
        tx_ready  = {LANES{1'b0}};
        for (k = LANES - 1; k >= 0; k = k - 1)
            if (tx_valid[k]) rt_in_pkt = tx_pkt[k*24 +: 24];
        for (k = 0; k < LANES; k = k + 1)
            tx_ready[k] = rt_in_ready && ((tx_valid & ((1 << k) - 1)) == 0);
    end

    wire [10*LANES-1:0] mem_addr;
    wire [16*LANES-1:0] mem_wdata;
    wire [LANES-1:0]    mem_we;
    wire [16*LANES-1:0] mem_rdata;

    genvar i;
    generate
        for (i = 0; i < LANES; i = i + 1) begin : gen_pe
            (* syn_preserve = 1 *) reg [23:0] instr_pe;
            (* syn_preserve = 1 *) reg        instr_valid_pe;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    instr_pe       <= 24'd0;
                    instr_valid_pe <= 1'b0;
                end else begin
                    instr_pe       <= instr_q;
                    instr_valid_pe <= instr_valid_q;
                end
            end

            dm16_pe #(
                .PE_ID(BASE_ID + i)
            ) u_pe (
                .clk          (clk),
                .rst_n        (rst_n),
                .instr        (instr_pe),
                .instr_valid  (instr_valid_pe),
                .mem_addr     (mem_addr[i*10 +: 10]),
                .mem_wdata    (mem_wdata[i*16 +: 16]),
                .mem_we       (mem_we[i]),
                .mem_rdata    (mem_rdata[i*16 +: 16]),
                .news_n_in    (news_n_in[i*16 +: 16]),
                .news_e_in    (news_e_in[i*16 +: 16]),
                .news_w_in    (news_w_in[i*16 +: 16]),
                .news_s_in    (news_s_in[i*16 +: 16]),
                .news_out     (news_out[i*16 +: 16]),
                .net_tx_pkt   (tx_pkt[i*24 +: 24]),
                .net_tx_valid (tx_valid[i]),
                .net_tx_ready (tx_ready[i]),
                .net_rx_data  (mbox[i]),
                .net_rx_valid (mbox_full[i]),
                .net_rx_pop   (rx_pop[i]),
                .ctx_active   (ctx_active[i]),
                .acc_out      (acc_out[i*16 +: 16])
            );

            // Single-port BSRAM, 16-bit mode: AD[13:4] = word, AD[1:0] = byte enables
            wire [31:0] do_full;
            assign mem_rdata[i*16 +: 16] = do_full[15:0];

            SP #(
                .READ_MODE  (1'b0),
                .WRITE_MODE (2'b00),
                .BIT_WIDTH  (16),
                .BLK_SEL    (3'b000),
                .RESET_MODE ("SYNC")
            ) u_sp (
                .DO     (do_full),
                .DI     ({16'd0, mem_wdata[i*16 +: 16]}),
                .BLKSEL (3'b000),
                .AD     ({mem_addr[i*10 +: 10], 2'b00, 2'b11}),
                .WRE    (mem_we[i]),
                .CLK    (clk),
                .CE     (1'b1),
                .OCE    (1'b1),
                .RESET  (~rst_n)
            );
        end
    endgenerate

endmodule
