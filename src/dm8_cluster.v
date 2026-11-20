module dm8_cluster #(
    parameter [3:0] NODE_ID = 4'd0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [23:0] instr,
    input  wire        instr_valid,

    // NEWS Grid Wires (4 PEs x 8-bit)
    input  wire [31:0] news_n_in,
    input  wire [31:0] news_e_in,
    input  wire [31:0] news_w_in,
    input  wire [31:0] news_s_in,
    output wire [31:0] news_out,

    // Local Router Interface (4 PEs)
    output wire [63:0] pe_tx_pkt,
    output wire [3:0]  pe_tx_valid,
    input  wire [3:0]  pe_tx_ready,
    input  wire [63:0] pe_rx_pkt,
    input  wire [3:0]  pe_rx_valid,
    output wire [3:0]  pe_rx_pop,

    // Status & Host Readback
    output wire [3:0]  ctx_active,
    output wire [31:0] acc_out
);

    wire [9:0] mem_addr  [0:3];
    wire [7:0] mem_wdata [0:3];
    wire [3:0] mem_we;
    wire [7:0] mem_rdata [0:3];

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_pe
            dm8_pe #(
                .PE_ID({NODE_ID, i[1:0]})
            ) u_pe (
                .clk          (clk),
                .rst_n        (rst_n),
                .instr        (instr),
                .instr_valid  (instr_valid),
                .mem_addr     (mem_addr[i]),
                .mem_wdata    (mem_wdata[i]),
                .mem_we       (mem_we[i]),
                .mem_rdata    (mem_rdata[i]),
                .news_n_in    (news_n_in[i*8 +: 8]),
                .news_e_in    (news_e_in[i*8 +: 8]),
                .news_w_in    (news_w_in[i*8 +: 8]),
                .news_s_in    (news_s_in[i*8 +: 8]),
                .news_out     (news_out[i*8 +: 8]),
                .net_tx_pkt   (pe_tx_pkt[i*16 +: 16]),
                .net_tx_valid (pe_tx_valid[i]),
                .net_tx_ready (pe_tx_ready[i]),
                .net_rx_pkt   (pe_rx_pkt[i*16 +: 16]),
                .net_rx_valid (pe_rx_valid[i]),
                .net_rx_pop   (pe_rx_pop[i]),
                .ctx_active   (ctx_active[i]),
                .acc_out      (acc_out[i*8 +: 8])
            );
        end
    endgenerate

    // 2 Gowin True Dual-Port BRAMs (DPB) for 4 PEs (14-bit address ports)
    genvar b;
    generate
        for (b = 0; b < 2; b = b + 1) begin : gen_dpb
            wire [15:0] doa_unused, dob_unused;

            DPB #(
                .READ_MODE0 (1'b0),
                .READ_MODE1 (1'b0),
                .WRITE_MODE0(2'b00),
                .WRITE_MODE1(2'b00),
                .BIT_WIDTH_0(8),
                .BIT_WIDTH_1(8),
                .BLK_SEL_0  (3'b000),
                .BLK_SEL_1  (3'b000),
                .RESET_MODE ("SYNC")
            ) u_dpb (
                .CLKA   (clk),
                .OCEA   (1'b1),
                .CEA    (1'b1),
                .RESETA (~rst_n),
                .WREA   (mem_we[b*2]),
                .BLKSELA(3'b000),
                .ADA    ({1'b0, mem_addr[b*2], 3'b000}),
                .DIA    ({8'd0, mem_wdata[b*2]}),
                .DOA    ({doa_unused[15:8], mem_rdata[b*2]}),

                .CLKB   (clk),
                .OCEB   (1'b1),
                .CEB    (1'b1),
                .RESETB (~rst_n),
                .WREB   (mem_we[b*2 + 1]),
                .BLKSELB(3'b000),
                .ADB    ({1'b1, mem_addr[b*2 + 1], 3'b000}),
                .DIB    ({8'd0, mem_wdata[b*2 + 1]}),
                .DOB    ({dob_unused[15:8], mem_rdata[b*2 + 1]})
            );
        end
    endgenerate

endmodule
