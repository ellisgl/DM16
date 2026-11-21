`timescale 1ns/1ns
// One cluster: LANES PEs, each with its own single-port memory (1024 x W),
// a LANES-to-1 network injection arbiter and a 1-deep receive mailbox per PE.
//
// Packet: {valid, 1'b0, dest[5:0], data[W-1:0]}
//   dest = {cluster, lane}; the lane is dest[LANE_BITS-1:0] (bits W +: LANE_BITS).
//
// Memory: 2^MEM_AW words per PE. GOWIN_SP = 1 uses Gowin SP block RAMs
// (W/16 wide x 2^(MEM_AW-10) banks; MEM_AW <= 13, since the SP's block select
// is 3 bits); GOWIN_SP = 0 infers a plain synchronous RAM for other families.
module dm16_cluster #(
    parameter integer LANES    = 2,      // PEs in this cluster (2 or 4)
    parameter [5:0]   BASE_ID  = 6'd0,   // PE_ID of lane 0 (cluster * LANES)
    parameter integer W        = 16,     // PE data width
    parameter integer GOWIN_SP = 1,
    parameter integer NREGS    = 8,      // PE registers (see dm16_pe)
    parameter         RF_BRAM  = 1'b0,
    parameter integer MEM_AW   = 10,     // PE memory = 2^MEM_AW words
    parameter         HAS_FPU  = 1'b1,   // bf16 lanes in every PE
    parameter integer PKT_W    = W + 8   // derived: do not override
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [23:0]          instr,
    input  wire                 instr_valid,

    // NEWS grid (LANES PEs x W bits)
    input  wire [LANES*W-1:0]   news_n_in,
    input  wire [LANES*W-1:0]   news_e_in,
    input  wire [LANES*W-1:0]   news_w_in,
    input  wire [LANES*W-1:0]   news_s_in,
    output wire [LANES*W-1:0]   news_out,

    // Router local port
    output reg  [PKT_W-1:0]     rt_in_pkt,
    output wire                 rt_in_valid,
    input  wire                 rt_in_ready,
    input  wire [PKT_W-1:0]     rt_out_pkt,
    input  wire                 rt_out_valid,
    output wire                 rt_out_ready,

    // Status & host readback
    output wire [LANES-1:0]     ctx_active,
    output wire [LANES*W-1:0]   acc_out
);

    localparam integer LANE_BITS = (LANES <= 2) ? 1 : 2;

    // Gowin SP banking: 1024 words per SP, up to 8 banks
    localparam integer LOW_AW    = (MEM_AW < 10) ? MEM_AW : 10;
    localparam integer NBANK     = (MEM_AW > 10) ? (1 << (MEM_AW - 10)) : 1;
    localparam integer BANK_BITS = (MEM_AW > 10) ? (MEM_AW - 10) : 1;

    // The SP's 10-bit word address: the low address bits, zero-padded
    function [9:0] word_addr;
        input [MEM_AW-1:0] a;
        begin
            word_addr = 10'd0;
            word_addr[LOW_AW-1:0] = a[LOW_AW-1:0];
        end
    endfunction

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

    wire [PKT_W*LANES-1:0] tx_pkt;
    wire [LANES-1:0]    tx_valid;
    reg  [LANES-1:0]    tx_ready;
    wire [LANES-1:0]    rx_pop;

    // Receive mailboxes
    reg  [W-1:0]        mbox [0:LANES-1];
    reg  [LANES-1:0]    mbox_full;

    wire [LANE_BITS-1:0] out_lane = rt_out_pkt[W +: LANE_BITS];
    assign rt_out_ready = !mbox_full[out_lane];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mbox_full <= {LANES{1'b0}};
            for (k = 0; k < LANES; k = k + 1)
                mbox[k] <= {W{1'b0}};
        end else begin
            for (k = 0; k < LANES; k = k + 1)
                if (rx_pop[k]) mbox_full[k] <= 1'b0;
            if (rt_out_valid && rt_out_ready) begin
                mbox[out_lane]      <= rt_out_pkt[W-1:0];
                mbox_full[out_lane] <= 1'b1;
            end
        end
    end

    // Injection: fixed priority, lowest lane first, into a one-packet
    // buffer in front of the router.
    //
    // The buffer breaks what used to be one long combinational loop in a
    // single clock (PE valid -> arbiter -> router arbitration -> router
    // ready -> arbiter -> PE), the design's slowest path. Now a PE's packet
    // is accepted when the buffer is empty, which comes from a flip-flop,
    // and the router only sees the buffer's flip-flops. The buffer can't
    // fill and drain on the same clock, which keeps the two sides apart:
    // at most one packet every two clocks per cluster, far more than the
    // links carry (a packet takes several clocks per hop).
    reg [PKT_W-1:0] inj_pkt;
    reg             inj_valid;
    reg [PKT_W-1:0] pick_pkt;

    assign rt_in_valid = inj_valid;
    always @(*) rt_in_pkt = inj_pkt;

    always @(*) begin
        pick_pkt = tx_pkt[PKT_W-1:0];
        tx_ready = {LANES{1'b0}};
        for (k = LANES - 1; k >= 0; k = k - 1)
            if (tx_valid[k]) pick_pkt = tx_pkt[k*PKT_W +: PKT_W];
        for (k = 0; k < LANES; k = k + 1)
            tx_ready[k] = !inj_valid && ((tx_valid & ((1 << k) - 1)) == 0);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inj_valid <= 1'b0;
            inj_pkt   <= {PKT_W{1'b0}};
        end else if (inj_valid) begin
            if (rt_in_ready)
                inj_valid <= 1'b0;          // the router took it
        end else if (|tx_valid) begin
            inj_valid <= 1'b1;              // take the highest-priority PE's packet
            inj_pkt   <= pick_pkt;
        end
    end

    wire [MEM_AW*LANES-1:0] mem_addr;
    wire [W*LANES-1:0]  mem_wdata;
    wire [LANES-1:0]    mem_we;
    wire [W*LANES-1:0]  mem_rdata;

    genvar i, j, k2;
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
                .PE_ID   (BASE_ID + i),
                .W       (W),
                .NREGS   (NREGS),
                .RF_BRAM (RF_BRAM),
                .MEM_AW  (MEM_AW),
                .HAS_FPU (HAS_FPU)
            ) u_pe (
                .clk          (clk),
                .rst_n        (rst_n),
                .instr        (instr_pe),
                .instr_valid  (instr_valid_pe),
                .instr_next   (instr_q),          // what instr_pe holds next clock
                .mem_addr     (mem_addr[i*MEM_AW +: MEM_AW]),
                .mem_wdata    (mem_wdata[i*W +: W]),
                .mem_we       (mem_we[i]),
                .mem_rdata    (mem_rdata[i*W +: W]),
                .news_n_in    (news_n_in[i*W +: W]),
                .news_e_in    (news_e_in[i*W +: W]),
                .news_w_in    (news_w_in[i*W +: W]),
                .news_s_in    (news_s_in[i*W +: W]),
                .news_out     (news_out[i*W +: W]),
                .net_tx_pkt   (tx_pkt[i*PKT_W +: PKT_W]),
                .net_tx_valid (tx_valid[i]),
                .net_tx_ready (tx_ready[i]),
                .net_rx_data  (mbox[i]),
                .net_rx_valid (mbox_full[i]),
                .net_rx_pop   (rx_pop[i]),
                .ctx_active   (ctx_active[i]),
                .acc_out      (acc_out[i*W +: W])
            );

            if (GOWIN_SP) begin : g_gowin_mem
                // Gowin SP block RAMs in 16-bit mode (1024 words each):
                // W/16 side by side for the width and NBANK = 2^(MEM_AW-10)
                // banks for the depth, chosen by the SP's own block-select
                // input (BLKSEL vs BLK_SEL), so WRE stays the PE's clean
                // flip-flop signal. AD[13:4] = word, AD[1:0] = byte enables.
                // The read data is picked by the bank of the previous clock's
                // address, matching the RAM's one-clock read.
                reg  [BANK_BITS-1:0] bank_q;
                wire [BANK_BITS-1:0] bank;
                if (MEM_AW > 10) begin : g_banked
                    assign bank = mem_addr[i*MEM_AW + 10 +: BANK_BITS];
                end else begin : g_one_bank
                    assign bank = {BANK_BITS{1'b0}};
                end
                always @(posedge clk) bank_q <= bank;
                for (j = 0; j < W / 16; j = j + 1) begin : g_slice
                    wire [16*NBANK-1:0] do_bank;
                    assign mem_rdata[i*W + j*16 +: 16] = do_bank[bank_q*16 +: 16];
                    for (k2 = 0; k2 < NBANK; k2 = k2 + 1) begin : g_bank
                        localparam [2:0] BANK_ID = k2;
                        wire [31:0] do_full;
                        assign do_bank[k2*16 +: 16] = do_full[15:0];
                        SP #(
                            .READ_MODE  (1'b0),
                            .WRITE_MODE (2'b00),
                            .BIT_WIDTH  (16),
                            .BLK_SEL    (BANK_ID),
                            .RESET_MODE ("SYNC")
                        ) u_sp (
                            .DO     (do_full),
                            .DI     ({16'd0, mem_wdata[i*W + j*16 +: 16]}),
                            .BLKSEL ((NBANK == 1) ? 3'b000 : {{(3-BANK_BITS){1'b0}}, bank}),
                            .AD     ({word_addr(mem_addr[i*MEM_AW +: MEM_AW]), 2'b00, 2'b11}),
                            .WRE    (mem_we[i]),
                            .CLK    (clk),
                            .CE     (1'b1),
                            .OCE    (1'b1),
                            .RESET  (~rst_n)
                        );
                    end
                end
            end else begin : g_infer_mem
                // Plain synchronous single-port RAM, read-first
                reg [W-1:0] ram [0:(1 << MEM_AW) - 1];
                reg [W-1:0] rd_q;
                always @(posedge clk) begin
                    if (mem_we[i])
                        ram[mem_addr[i*MEM_AW +: MEM_AW]] <= mem_wdata[i*W +: W];
                    rd_q <= ram[mem_addr[i*MEM_AW +: MEM_AW]];
                end
                assign mem_rdata[i*W +: W] = rd_q;
            end
        end
    endgenerate

endmodule
