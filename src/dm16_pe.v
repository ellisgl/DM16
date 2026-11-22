`timescale 1ns/1ns
// DM16 processing element - W-bit datapath (W = 16, 32, 64, ...: a multiple
// of 16) with W/16 bfloat16 FPU lanes. DM16 on the Tang Console uses W = 16.
//
// Registers: acc, b (W bits; at W = 16 each holds an int16 or a bf16),
//            mar (MEM_AW bits), flags C/Z/N, 4-deep context stack, and (with
//            HAS_RF) r1..r[NREGS-1]. r0 is b. Without HAS_RF every r[n] is b.
//            n = imm10 mod NREGS wherever a register is named, so a program
//            that names a register >= NREGS silently gets r[n mod NREGS].
// Memory:    2^MEM_AW W-bit words (1024 on the Tang), word addressed.
//            Direct addresses (imm10) reach only the first 1024 words; the
//            rest of a larger memory is reached through mar.
//
// Instruction (24 bits): opcode[23:20] sub_op[19:16] src_sel[15:14]
//                        flag_ctl[13:10] imm10[9:0]
//
// Operand B (src_sel) for ALU and FPU ops:
//   0 = r[n] (r0 = b)   1 = sign-extended imm10   2 = PE_ID   3 = mem[addr]
//
// Memory addressing (any instruction touching memory):
//   addr = flag_ctl[0] ? mar : imm10;  flag_ctl[1] = post-increment mar
//   (LDX/STX always use mar; mar wraps at 2^MEM_AW. With MEM_AW < 10 an
//   immediate address uses its low MEM_AW bits.)
//
// 0 NOP
// 1 LDI   sub 0: acc = sext(imm10)   1: acc[15:8] = imm8   2: acc[7:0] = imm8
//         (other bits unchanged; wider constants: SHLN 8 then OR imm8)
// 2 LDM   acc = mem[addr]            3 STM  mem[addr] = acc
// 4 LDX   acc = mem[mar]             5 STX  mem[mar]  = acc
// 6 MAR   mar = src_sel==0 ? acc[MEM_AW-1:0] : imm10
// 7 ALU   acc = acc OP opB: 0 ADD 1 ADC 2 SUB 3 AND 4 OR 5 XOR 6 SHL 7 SHR
//                           8 MOV 9 ASR A MUL (low W bits)
//                           B MULH (high W bits, signed; X = unsigned)
//                           C SHLN  D SHRN  E ASRN  F ROL (X = ROR)
//         X is flag_ctl[2]. The barrel shifts move acc by n = opB mod W;
//         C = the last bit shifted out (0 when n = 0; the bit that wrapped
//         round for ROL/ROR). With X set, SHLN/SHRN are funnel shifts that
//         fill from b: SHLN acc = ({acc,b} << n)[2W-1:W],
//                      SHRN acc = ({b,acc} >> n)[W-1:0].
//         MULH: C = 0, Z/N from the high half.
// 8 SWP   sub 0: acc <-> r[n]   1 PUT: r[n] = acc   2 GET: acc = r[n]
//         (flags unchanged; src_sel must be 0; SWP with n = 0 is the
//         original acc <-> b)
// 9 CTX   context control (see cond_eval)
// A SEND  send b; dest = src_sel 0: imm[5:0]  1: acc[5:0]  2: PE_ID ^ imm[5:0]
// B RECV  b = mailbox, C = 1 if a packet was waiting (else C = 0)
// C NEWS  acc = neighbour's acc (sub_op 0 N, 1 E, 2 W, 3 S)
// D FPU   0 FADD acc = acc + opB      4 FI2F acc = bf16(int16 acc)
//         1 FSUB acc = acc - opB      5 FF2I acc = int16(acc), trunc, saturate
//         2 FMUL acc = acc * opB      6 FNEG acc = -acc
//         3 FCMP Z = acc==opB, N = acc<opB, C = unordered   7 FABS acc = |acc|
//         Every op except FCMP works on each 16-bit lane of acc and opB
//         independently (W/16 lanes); FCMP compares lane 0 only.
//         With HAS_FPU = 0 (small FPGAs) every FPU instruction is a no-op,
//         apart from a memory operand's mar post-increment.
//         FADD/FSUB/FMUL/FI2F complete two clocks after they execute; the
//         sequencer adds an idle cycle after them so the next instruction
//         sees the result.
//
// Every instruction must be followed by at least one idle clock (the
// sequencer guarantees it): MUL/MULH results and ALU Z/N flags are written
// in that clock, and operands for the next instruction are read in it.
module dm16_pe #(
    parameter [5:0]   PE_ID   = 6'd0,
    parameter         HAS_RF  = 1'b1,    // registers r1..r[NREGS-1]
    parameter integer W       = 16,      // data width: a power of two, >= 16
    parameter integer NREGS   = 8,       // r0 (= b) .. r[NREGS-1]: a power of two, 2..1024
    parameter         RF_BRAM = 1'b0,    // 1: r1.. in a block RAM; 0: flip-flops
    parameter integer MEM_AW  = 10,      // memory = 2^MEM_AW words; MEM_AW <= W
    parameter         HAS_FPU = 1'b1     // bf16 lanes; 0 makes FPU instructions no-ops
)(
    input  wire        clk,
    input  wire        rst_n,

    // Broadcast SIMD instruction from the global sequencer. instr_next is
    // the instruction that will be on instr one clock later.
    input  wire [23:0] instr,
    input  wire        instr_valid,
    input  wire [23:0] instr_next,

    // Local memory (1024 x W)
    output reg  [MEM_AW-1:0] mem_addr,
    output reg  [W-1:0] mem_wdata,
    output reg          mem_we,
    input  wire [W-1:0] mem_rdata,

    // 2D NEWS nearest-neighbour grid
    input  wire [W-1:0] news_n_in,
    input  wire [W-1:0] news_e_in,
    input  wire [W-1:0] news_w_in,
    input  wire [W-1:0] news_s_in,
    output wire [W-1:0] news_out,

    // Network: {valid, 1'b0, dest[5:0], data[W-1:0]}
    output reg  [W+7:0] net_tx_pkt,
    output reg          net_tx_valid,
    input  wire         net_tx_ready,
    input  wire [W-1:0] net_rx_data,
    input  wire         net_rx_valid,
    output reg          net_rx_pop,

    // Status & host readback
    output wire         ctx_active,
    output wire [W-1:0] acc_out
);

    // Width-derived constants
    localparam integer FPL = W / 16;                 // bf16 lanes
    function integer clog2;
        input integer v;
        integer t;
        begin
            clog2 = 0;
            for (t = v - 1; t > 0; t = t >> 1) clog2 = clog2 + 1;
        end
    endfunction
    localparam integer SHW = clog2(W);               // shift-amount bits
    localparam integer RW  = (NREGS < 2) ? 1 : clog2(NREGS);   // register-number bits

    // ------------------------------------------------------------------
    // Timing scheme
    //
    // The sequencer always leaves at least one idle clock after every
    // instruction a PE executes, and the cluster shows each instruction on
    // instr_next one clock before it reaches instr. The PE uses both:
    //
    //   clock before execute: decode instr_next into one-hot selects, and
    //                         select operand B (r[n], immediate, PE_ID or
    //                         the memory word) into the op_b register.
    //   execute:              acc = flat AND-OR of the selected results.
    //   idle clock after:     MUL/MULH write their (registered) product to
    //                         acc, and ALU ops set Z/N from the new acc.
    //
    // Operand selection one clock early is safe because registers and b are
    // only ever written when an instruction executes, which is at least one
    // clock before the next instruction's early cycle ends.
    // ------------------------------------------------------------------

    // Instruction fields (executing instruction)
    wire [3:0]  opcode   = instr[23:20];
    wire [3:0]  sub_op   = instr[19:16];
    wire [1:0]  src_sel  = instr[15:14];
    wire [3:0]  flag_ctl = instr[13:10];
    wire [9:0]  imm10    = instr[9:0];
    wire [7:0]  imm8     = instr[7:0];
    wire [W-1:0] simm    = {{(W-10){imm10[9]}}, imm10};

    // Instruction fields (next instruction, for early decode)
    wire [3:0]  n_opcode = instr_next[23:20];
    wire [3:0]  n_sub    = instr_next[19:16];
    wire [1:0]  n_src    = instr_next[15:14];
    wire [9:0]  n_imm10  = instr_next[9:0];

    localparam OP_NOP  = 4'h0, OP_LDI  = 4'h1, OP_LDM  = 4'h2, OP_STM  = 4'h3,
               OP_LDX  = 4'h4, OP_STX  = 4'h5, OP_MAR  = 4'h6, OP_ALU  = 4'h7,
               OP_SWP  = 4'h8, OP_CTX  = 4'h9, OP_SEND = 4'hA, OP_RECV = 4'hB,
               OP_NEWS = 4'hC, OP_FPU  = 4'hD;

    localparam F_ADD = 4'h0, F_SUB = 4'h1, F_MUL = 4'h2, F_CMP = 4'h3,
               F_I2F = 4'h4, F_F2I = 4'h5, F_NEG = 4'h6, F_ABS = 4'h7;

    // Registers
    reg [W-1:0] acc;
    reg [W-1:0] b_reg;
    reg [MEM_AW-1:0] mar;
    reg        flag_c, flag_z, flag_n;
    reg [3:0]  ctx_stack;
    wire       ctx  = ctx_stack[0];
    wire       exec = instr_valid && ctx && (opcode != OP_CTX);

    assign ctx_active = ctx;
    assign news_out   = acc;
    assign acc_out    = acc;

    // ------------------------------------------------------------------
    // Memory interface
    //
    // Writes are registered: the BSRAM write-enable, address and data all
    // come straight from flip-flops so WRE can never glitch (a combinational
    // WRE caused spurious partial writes on the Gowin BSRAM). The write
    // lands one clock after the store executes, in the sequencer's idle
    // cycle; any following read samples its address later still.
    // ------------------------------------------------------------------
    wire uses_mem  = (opcode == OP_LDM) || (opcode == OP_STM) ||
                     (opcode == OP_LDX) || (opcode == OP_STX) ||
                     (((opcode == OP_ALU) || (opcode == OP_FPU)) && (src_sel == 2'b11));
    wire use_mar   = (opcode == OP_LDX) || (opcode == OP_STX) || (uses_mem && flag_ctl[0]);
    // An immediate address: imm10, zero-extended or cut to MEM_AW bits
    wire [MEM_AW-1:0] imm_addr;
    generate
        if (MEM_AW > 10) begin : g_imm_ext
            assign imm_addr = {{(MEM_AW-10){1'b0}}, imm10};
        end else begin : g_imm_cut
            assign imm_addr = imm10[MEM_AW-1:0];
        end
    endgenerate
    wire [MEM_AW-1:0] addr_now = use_mar ? mar : imm_addr;
    wire we_now    = exec && ((opcode == OP_STM) || (opcode == OP_STX));

    (* syn_preserve = 1 *) reg        wr_pend;
    (* syn_preserve = 1 *) reg [MEM_AW-1:0] wr_addr_q;
    (* syn_preserve = 1 *) reg [W-1:0] wr_data_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_pend   <= 1'b0;
            wr_addr_q <= {MEM_AW{1'b0}};
            wr_data_q <= {W{1'b0}};
        end else begin
            wr_pend <= we_now;
            if (we_now) begin
                wr_addr_q <= addr_now;
                wr_data_q <= acc;
            end
        end
    end

    always @(*) begin
        mem_we    = wr_pend;
        mem_addr  = wr_pend ? wr_addr_q : addr_now;
        mem_wdata = wr_data_q;
    end

    // Registered memory read data, for LDM/LDX. The sequencer waits two
    // cycles on any read: the BSRAM output is valid in the clock before
    // execute, so mem_q (and op_b, below) capture it then.
    reg [W-1:0] mem_q;
    always @(posedge clk) mem_q <= mem_rdata;

    // ------------------------------------------------------------------
    // Register file and operand B
    //
    // r0 is b_reg; r1..r[NREGS-1] are flip-flops (RF_BRAM = 0) or a block
    // RAM (RF_BRAM = 1). They are read one clock early, for the next
    // instruction (its number is on instr_next), and written when SWP/PUT
    // executes. Operand B (r[n], immediate, PE_ID or the memory word) is
    // likewise selected in that early clock. SWP/GET use src_sel 0 = r[n].
    // ------------------------------------------------------------------
    wire [RW-1:0] r_idx   = imm10[RW-1:0];
    wire [RW-1:0] n_r_idx = n_imm10[RW-1:0];
    wire          r_put   = exec && (opcode == OP_SWP) && (sub_op != 4'd2);   // SWP or PUT
    wire [W-1:0]  n_simm  = {{(W-10){n_imm10[9]}}, n_imm10};
    wire [W-1:0]  op_b;

    generate
        if (HAS_RF && !RF_BRAM) begin : g_rf
            // Flip-flops: an NREGS-way select feeds the op_b register
            reg [W-1:0] rf [0:NREGS-1];
            reg [W-1:0] op_b_q;
            always @(posedge clk)
                if (r_put && (r_idx != {RW{1'b0}}))
                    rf[r_idx] <= acc;
            wire [W-1:0] n_r_val = (n_r_idx == {RW{1'b0}}) ? b_reg : rf[n_r_idx];
            always @(posedge clk) begin
                case (n_src)
                    2'b00: op_b_q <= n_r_val;
                    2'b01: op_b_q <= n_simm;
                    2'b10: op_b_q <= {{(W-6){1'b0}}, PE_ID};
                    2'b11: op_b_q <= mem_rdata;
                endcase
            end
            assign op_b = op_b_q;
        end else if (HAS_RF) begin : g_rf
            // Block RAM with a clocked read: the read address (from
            // instr_next) is sampled at the same edge that loads the rest of
            // operand B, so the data is ready when the instruction executes.
            // The write is registered, so the RAM's write enable comes
            // straight from a flip-flop (Gowin BSRAMs make spurious writes on
            // a glitchy WRE); it lands in the idle clock after SWP/PUT, at
            // the same edge as the next instruction's read, so a write and a
            // read of the same register on that edge are bypassed.
            // Without the attribute Gowin synthesis builds small memories
            // from flip-flops (a 32 x 16 file became 496 flip-flops per PE)
            (* syn_ramstyle = "block_ram" *) reg [W-1:0] rf [0:NREGS-1];
            reg [W-1:0]  rf_q, op_o, byp_d;
            reg          sel_rf, byp;
            reg          rf_we_q;
            reg [RW-1:0] rf_wa_q;
            reg [W-1:0]  rf_wd_q;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) rf_we_q <= 1'b0;
                else        rf_we_q <= r_put && (r_idx != {RW{1'b0}});
            end
            always @(posedge clk) begin
                rf_wa_q <= r_idx;
                rf_wd_q <= acc;
            end
            always @(posedge clk) begin
                if (rf_we_q)
                    rf[rf_wa_q] <= rf_wd_q;
                rf_q <= rf[n_r_idx];
            end
            always @(posedge clk) begin
                sel_rf <= (n_src == 2'b00) && (n_r_idx != {RW{1'b0}});
                byp    <= rf_we_q && (rf_wa_q == n_r_idx);
                byp_d  <= rf_wd_q;
                case (n_src)
                    2'b00: op_o <= b_reg;
                    2'b01: op_o <= n_simm;
                    2'b10: op_o <= {{(W-6){1'b0}}, PE_ID};
                    2'b11: op_o <= mem_rdata;
                endcase
            end
            assign op_b = !sel_rf ? op_o : (byp ? byp_d : rf_q);
        end else begin : g_no_rf   // every r[n] is b
            reg [W-1:0] op_b_q;
            always @(posedge clk) begin
                case (n_src)
                    2'b00: op_b_q <= b_reg;
                    2'b01: op_b_q <= n_simm;
                    2'b10: op_b_q <= {{(W-6){1'b0}}, PE_ID};
                    2'b11: op_b_q <= mem_rdata;
                endcase
            end
            assign op_b = op_b_q;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Early decode: one-hot selects for the instruction about to execute
    // ------------------------------------------------------------------
    wire n_alu = (n_opcode == OP_ALU);
    wire n_fpu = (n_opcode == OP_FPU) && HAS_FPU;

    reg d_add, d_and, d_or, d_xor, d_mov, d_shl1, d_shr1, d_asr1, d_bsh, d_mul;
    reg d_sub, d_adc, d_mulh, d_x;
    reg d_shln, d_shrn, d_asrn, d_rot;
    reg d_ldi, d_ldi_hi, d_ldi_lo, d_ldm, d_get;
    reg d_news_n, d_news_e, d_news_w, d_news_s;
    reg d_f2i, d_fneg, d_fabs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {d_add, d_and, d_or, d_xor, d_mov, d_shl1, d_shr1, d_asr1, d_bsh, d_mul} <= 10'd0;
            {d_sub, d_adc, d_mulh, d_x, d_shln, d_shrn, d_asrn, d_rot} <= 8'd0;
            {d_ldi, d_ldi_hi, d_ldi_lo, d_ldm, d_get} <= 5'd0;
            {d_news_n, d_news_e, d_news_w, d_news_s, d_f2i, d_fneg, d_fabs} <= 7'd0;
        end else begin
            d_add    <= n_alu && (n_sub <= 4'h2);                  // ADD ADC SUB
            d_sub    <= (n_sub == 4'h2);
            d_adc    <= (n_sub == 4'h1);
            d_and    <= n_alu && (n_sub == 4'h3);
            d_or     <= n_alu && (n_sub == 4'h4);
            d_xor    <= n_alu && (n_sub == 4'h5);
            d_shl1   <= n_alu && (n_sub == 4'h6);
            d_shr1   <= n_alu && (n_sub == 4'h7);
            d_mov    <= n_alu && (n_sub == 4'h8);
            d_asr1   <= n_alu && (n_sub == 4'h9);
            d_mul    <= n_alu && ((n_sub == 4'hA) || (n_sub == 4'hB));
            d_mulh   <= (n_sub == 4'hB);
            d_bsh    <= n_alu && (n_sub >= 4'hC);
            d_shln   <= (n_sub == 4'hC);
            d_shrn   <= (n_sub == 4'hD);
            d_asrn   <= (n_sub == 4'hE);
            d_rot    <= (n_sub == 4'hF);
            d_x      <= instr_next[12];                            // flag_ctl[2]
            d_ldi    <= (n_opcode == OP_LDI) && (n_sub[1:0] != 2'd1) && (n_sub[1:0] != 2'd2);
            d_ldi_hi <= (n_opcode == OP_LDI) && (n_sub[1:0] == 2'd1);
            d_ldi_lo <= (n_opcode == OP_LDI) && (n_sub[1:0] == 2'd2);
            d_ldm    <= (n_opcode == OP_LDM) || (n_opcode == OP_LDX);
            d_get    <= (n_opcode == OP_SWP) && (n_sub != 4'd1);   // SWP, GET
            d_news_n <= (n_opcode == OP_NEWS) && (n_sub[1:0] == 2'd0);
            d_news_e <= (n_opcode == OP_NEWS) && (n_sub[1:0] == 2'd1);
            d_news_w <= (n_opcode == OP_NEWS) && (n_sub[1:0] == 2'd2);
            d_news_s <= (n_opcode == OP_NEWS) && (n_sub[1:0] == 2'd3);
            d_f2i    <= n_fpu && (n_sub == F_F2I);
            d_fneg   <= n_fpu && (n_sub == F_NEG);
            d_fabs   <= n_fpu && (n_sub == F_ABS);
        end
    end

    // ------------------------------------------------------------------
    // Integer ALU
    // ------------------------------------------------------------------
    // ADD / ADC / SUB share one adder; for SUB, C is the borrow.
    wire [W:0]  sum   = {1'b0, acc} + {1'b0, op_b ^ {W{d_sub}}} +
                        {{W{1'b0}}, d_sub | (d_adc & flag_c)};
    wire        add_c = sum[W] ^ d_sub;

    // (W+1) x (W+1) signed multiply covers both signed and unsigned W-bit
    // operands (at W = 16 it fits one 18 x 18 DSP); the low half is the
    // same either way. The product is registered and written to acc in the
    // idle clock.
    wire signed [W:0]     mul_a    = {~d_x & acc[W-1],  acc};
    wire signed [W:0]     mul_b    = {~d_x & op_b[W-1], op_b};
    wire signed [2*W+1:0] mul_full = mul_a * mul_b;
    reg         [2*W-1:0] mul_q;
    always @(posedge clk) mul_q <= mul_full[2*W-1:0];

    // Barrel shifter: one (2W+1)-bit right shift by k = 0..W of {hi, lo, 0}
    // serves every shift and rotate.
    //   right by n:  hi = fill, lo = acc,  k = n      result = sh[W:1], C = sh[0]
    //   left by n:   hi = acc,  lo = fill, k = W - n  result = sh[W:1], C = sh[W+1]
    wire [SHW-1:0] sh_n    = op_b[SHW-1:0];
    wire           sh_left = d_shln || (d_rot && !d_x);
    wire [W-1:0]   sh_hi   = d_shrn ? (d_x ? b_reg : {W{1'b0}}) :
                             d_asrn ? {W{acc[W-1]}} : acc;
    wire [W-1:0]   sh_lo   = d_shln ? (d_x ? b_reg : {W{1'b0}}) : acc;
    wire [SHW:0]   sh_k    = sh_left ? (W - {1'b0, sh_n}) : {1'b0, sh_n};
    wire [2*W:0]   sh_out  = {sh_hi, sh_lo, 1'b0} >> sh_k;
    wire [W-1:0]   sh_res  = sh_out[W:1];
    wire           sh_c    = (sh_n == {SHW{1'b0}}) ? 1'b0 :
                             d_rot   ? (d_x ? sh_res[W-1] : sh_res[0]) :
                             sh_left ? sh_out[W+1] : sh_out[0];

    // ------------------------------------------------------------------
    // Context condition
    // ------------------------------------------------------------------
    reg cond_eval;
    always @(*) begin
        case (sub_op)
            4'h0: cond_eval = 1'b1;                       // ALWAYS
            4'h1: cond_eval = 1'b0;                       // NEVER
            4'h2: cond_eval = flag_z;                     // IF ZERO
            4'h3: cond_eval = !flag_z;                    // IF NOT ZERO
            4'h4: cond_eval = flag_c;                     // IF CARRY
            4'h5: cond_eval = flag_n;                     // IF NEGATIVE
            4'h6: cond_eval = ({2'd0, PE_ID} == imm8);    // IF PE_ID == IMM
            default: cond_eval = ctx;
        endcase
    end

    // ------------------------------------------------------------------
    // bfloat16 unit: acc OP op_b
    // ------------------------------------------------------------------
    wire fp_arith = (opcode == OP_FPU) &&
                    ((sub_op == F_ADD) || (sub_op == F_SUB) || (sub_op == F_MUL) || (sub_op == F_I2F));
    wire fp_start = exec && fp_arith && HAS_FPU;
    wire [1:0] fp_op = (sub_op == F_ADD) ? 2'd0 :
                       (sub_op == F_SUB) ? 2'd1 :
                       (sub_op == F_MUL) ? 2'd2 : 2'd3;
    wire [W-1:0] fp_result;
    wire [W-1:0] f2i;
    reg          fp_wb1, fp_wb2;

    // bf16 -> int16, truncate toward zero, saturate
    function [15:0] bf16_to_int16;
        input [15:0] x;
        reg   [15:0] mag;
        begin
            if (x[14:7] < 8'd127)
                bf16_to_int16 = 16'd0;
            else if (x[14:7] >= 8'd142)
                bf16_to_int16 = x[15] ? 16'h8000 : 16'h7FFF;
            else begin
                mag           = {1'b1, x[6:0], 8'd0} >> (8'd142 - x[14:7]);
                bf16_to_int16 = x[15] ? (~mag + 16'd1) : mag;
            end
        end
    endfunction

    genvar l;
    generate
        for (l = 0; l < (HAS_FPU ? FPL : 0); l = l + 1) begin : g_fpl
            dm16_bf16 u_fpu (
                .clk    (clk),
                .rst_n  (rst_n),
                .start  (fp_start),
                .op     (fp_op),
                .a      (acc [l*16 +: 16]),
                .b      (op_b[l*16 +: 16]),
                .result (fp_result[l*16 +: 16])
            );
            assign f2i[l*16 +: 16] = bf16_to_int16(acc[l*16 +: 16]);
        end
        if (!HAS_FPU) begin : g_no_fpu      // never selected: FPU ops are no-ops
            assign fp_result = {W{1'b0}};
            assign f2i       = {W{1'b0}};
        end
    endgenerate

    // Compare lane 0 of acc vs op_b (subnormals = zero, +0 == -0)
    wire        x_nan = (acc[14:7]  == 8'hFF) && (acc[6:0]  != 7'd0);
    wire        y_nan = (op_b[14:7] == 8'hFF) && (op_b[6:0] != 7'd0);
    wire [15:0] x_c   = (acc[14:7]  == 8'd0) ? 16'd0 : acc[15:0];
    wire [15:0] y_c   = (op_b[14:7] == 8'd0) ? 16'd0 : op_b[15:0];
    wire [15:0] x_key = x_c[15] ? ~x_c : (x_c | 16'h8000);
    wire [15:0] y_key = y_c[15] ? ~y_c : (y_c | 16'h8000);

    // SEND destination
    wire [5:0] send_dest = (src_sel == 2'b01) ? acc[5:0] :
                           (src_sel == 2'b10) ? (PE_ID ^ imm10[5:0]) : imm10[5:0];

    // ------------------------------------------------------------------
    // acc input: a flat AND-OR of every source, with one-hot selects
    // ------------------------------------------------------------------
    reg          mul_wb, mulh_wb, zn_pend;
    wire [W-1:0] mul_res  = mulh_wb ? mul_q[2*W-1:W] : mul_q[W-1:0];
    wire [W-1:0] sign_bits = {FPL{16'h8000}};        // bf16 sign of every lane

    // LDI hi/lo set bits 15:8 / 7:0 and leave the rest of acc alone
    wire [W-1:0] ldi_hi_val, ldi_lo_val;
    generate
        if (W > 16) begin : g_ldi_wide
            assign ldi_hi_val = {acc[W-1:16], imm8, acc[7:0]};
            assign ldi_lo_val = {acc[W-1:8], imm8};
        end else begin : g_ldi_16
            assign ldi_hi_val = {imm8, acc[7:0]};
            assign ldi_lo_val = {acc[15:8], imm8};
        end
    endgenerate

    wire [W-1:0] acc_exec =
          ({W{d_add}}    & sum[W-1:0])
        | ({W{d_and}}    & (acc & op_b))
        | ({W{d_or}}     & (acc | op_b))
        | ({W{d_xor}}    & (acc ^ op_b))
        | ({W{d_mov}}    & op_b)
        | ({W{d_shl1}}   & {acc[W-2:0], 1'b0})
        | ({W{d_shr1}}   & {1'b0, acc[W-1:1]})
        | ({W{d_asr1}}   & {acc[W-1], acc[W-1:1]})
        | ({W{d_bsh}}    & sh_res)
        | ({W{d_ldi}}    & simm)
        | ({W{d_ldi_hi}} & ldi_hi_val)
        | ({W{d_ldi_lo}} & ldi_lo_val)
        | ({W{d_ldm}}    & mem_q)
        | ({W{d_get}}    & op_b)
        | ({W{d_news_n}} & news_n_in)
        | ({W{d_news_e}} & news_e_in)
        | ({W{d_news_w}} & news_w_in)
        | ({W{d_news_s}} & news_s_in)
        | ({W{d_f2i}}    & f2i)
        | ({W{d_fneg}}   & (acc ^ sign_bits))
        | ({W{d_fabs}}   & (acc & ~sign_bits));

    wire acc_exec_we = d_add | d_and | d_or | d_xor | d_mov | d_shl1 | d_shr1 | d_asr1 |
                       d_bsh | d_ldi | d_ldi_hi | d_ldi_lo | d_ldm | d_get |
                       d_news_n | d_news_e | d_news_w | d_news_s | d_f2i | d_fneg | d_fabs;

    // Carry from the ALU ops that set it (logic ops and MOV clear it)
    wire alu_c = (d_add & add_c) | (d_bsh & sh_c) | (d_shl1 & acc[W-1]) |
                 ((d_shr1 | d_asr1) & acc[0]);
    wire is_alu_flags = d_add | d_and | d_or | d_xor | d_mov | d_shl1 | d_shr1 | d_asr1 | d_bsh;

    // ------------------------------------------------------------------
    // State update
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc          <= {W{1'b0}};
            b_reg        <= {W{1'b0}};
            mar          <= {MEM_AW{1'b0}};
            flag_c       <= 1'b0;
            flag_z       <= 1'b0;
            flag_n       <= 1'b0;
            fp_wb1       <= 1'b0;
            fp_wb2       <= 1'b0;
            mul_wb       <= 1'b0;
            mulh_wb      <= 1'b0;
            zn_pend      <= 1'b0;
            ctx_stack    <= 4'b1111;
            net_tx_pkt   <= {(W+8){1'b0}};
            net_tx_valid <= 1'b0;
            net_rx_pop   <= 1'b0;
        end else begin
            if (net_tx_valid && net_tx_ready)
                net_tx_valid <= 1'b0;

            net_rx_pop <= 1'b0;

            // FPU write-back, two clocks after the arithmetic op executed
            // (the sequencer adds a second idle clock after these)
            fp_wb1 <= fp_start;
            fp_wb2 <= fp_wb1;
            if (fp_wb2)
                acc <= fp_result;

            // MUL/MULH write-back and ALU Z/N, in the idle clock
            mul_wb  <= exec && d_mul;
            mulh_wb <= d_mulh;
            zn_pend <= exec && is_alu_flags;
            if (mul_wb) begin
                acc    <= mul_res;
                flag_z <= (mul_res == {W{1'b0}});
                flag_n <= mul_res[W-1];
            end
            if (zn_pend) begin
                flag_z <= (acc == {W{1'b0}});
                flag_n <= acc[W-1];
            end

            if (exec) begin
                if (acc_exec_we)
                    acc <= acc_exec;
                if (is_alu_flags)
                    flag_c <= alu_c;
                if (d_mul)
                    flag_c <= 1'b0;
                if (uses_mem && flag_ctl[1])
                    mar <= mar + 1'b1;
            end

            if (instr_valid && (opcode == OP_CTX)) begin
                case (flag_ctl[1:0])
                    2'b00: ctx_stack[0] <= cond_eval;                     // SET
                    2'b01: ctx_stack    <= {ctx_stack[2:0], cond_eval};   // PUSH
                    2'b10: ctx_stack    <= {1'b1, ctx_stack[3:1]};        // POP
                    2'b11: ctx_stack    <= ~ctx_stack;                    // INVERT
                endcase
            end

            if (exec) begin
                case (opcode)
                    OP_MAR: mar <= (src_sel == 2'b00) ? acc[MEM_AW-1:0] : imm_addr;

                    OP_SWP:             // r1.. are written by g_rf
                        if ((sub_op != 4'd2) && (r_idx == {RW{1'b0}} || !HAS_RF))
                            b_reg <= acc;                           // SWP, PUT

                    OP_SEND: begin
                        net_tx_pkt   <= {1'b1, 1'b0, send_dest, b_reg};
                        net_tx_valid <= 1'b1;
                    end

                    OP_RECV: begin
                        if (net_rx_valid) begin
                            b_reg      <= net_rx_data;
                            net_rx_pop <= 1'b1;
                            flag_c     <= 1'b1;
                        end else begin
                            flag_c     <= 1'b0;
                        end
                    end

                    OP_FPU:
                        if ((sub_op == F_CMP) && HAS_FPU) begin
                            flag_c <= x_nan | y_nan;
                            flag_z <= !(x_nan | y_nan) && (x_key == y_key);
                            flag_n <= !(x_nan | y_nan) && (x_key <  y_key);
                        end

                    default: ;
                endcase
            end
        end
    end

endmodule
