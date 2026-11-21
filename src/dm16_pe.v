// DM16 processing element - 16-bit datapath with a built-in bfloat16 FPU.
//
// Registers: acc, b (16 bits; each holds an int16 or a bf16), mar (10 bits),
//            flags C/Z/N, 4-deep context stack, and (with HAS_RF) r1..r7.
//            r0 is b. Without HAS_RF every r[n] is b.
//            n = imm10[2:0] wherever a register is named.
// Memory:    1024 x 16-bit words (2 KB), word addressed.
//
// Instruction (24 bits): opcode[23:20] sub_op[19:16] src_sel[15:14]
//                        flag_ctl[13:10] imm10[9:0]
//
// Operand B (src_sel) for ALU and FPU ops:
//   0 = r[n] (r0 = b)   1 = sign-extended imm10   2 = PE_ID   3 = mem[addr]
//
// Memory addressing (any instruction touching memory):
//   addr = flag_ctl[0] ? mar : imm10;  flag_ctl[1] = post-increment mar
//   (LDX/STX always use mar.)
//
// 0 NOP
// 1 LDI   sub 0: acc = sext(imm10)   1: acc[15:8] = imm8   2: acc[7:0] = imm8
// 2 LDM   acc = mem[addr]            3 STM  mem[addr] = acc
// 4 LDX   acc = mem[mar]             5 STX  mem[mar]  = acc
// 6 MAR   mar = src_sel==0 ? acc[9:0] : imm10
// 7 ALU   acc = acc OP opB: 0 ADD 1 ADC 2 SUB 3 AND 4 OR 5 XOR 6 SHL 7 SHR
//                           8 MOV 9 ASR A MUL (low 16 bits)
//                           B MULH (high 16 bits, signed; X = unsigned)
//                           C SHLN  D SHRN  E ASRN  F ROL (X = ROR)
//         X is flag_ctl[2]. The barrel shifts move acc by n = opB[3:0];
//         C = the last bit shifted out (0 when n = 0; the bit that wrapped
//         round for ROL/ROR). With X set, SHLN/SHRN are funnel shifts that
//         fill from b: SHLN acc = ({acc,b} << n)[31:16],
//                      SHRN acc = ({b,acc} >> n)[15:0].
//         MULH: C = 0, Z/N from the high half.
// 8 SWP   sub 0: acc <-> r[n]   1 PUT: r[n] = acc   2 GET: acc = r[n]
//         (flags unchanged; SWP with n = 0 is the original acc <-> b)
// 9 CTX   context control (see cond_eval)
// A SEND  send b; dest = src_sel 0: imm[5:0]  1: acc[5:0]  2: PE_ID ^ imm[5:0]
// B RECV  b = mailbox, C = 1 if a packet was waiting (else C = 0)
// C NEWS  acc = neighbour's acc (sub_op 0 N, 1 E, 2 W, 3 S)
// D FPU   0 FADD acc = acc + opB      4 FI2F acc = bf16(int16 acc)
//         1 FSUB acc = acc - opB      5 FF2I acc = int16(acc), trunc, saturate
//         2 FMUL acc = acc * opB      6 FNEG acc = -acc
//         3 FCMP Z = acc==opB, N = acc<opB, C = unordered   7 FABS acc = |acc|
//         FADD/FSUB/FMUL/FI2F complete two clocks after they execute; the
//         sequencer adds an idle cycle after them so the next instruction
//         sees the result.
module dm16_pe #(
    parameter [5:0] PE_ID  = 6'd0,
    parameter       HAS_RF = 1'b1      // r1..r7 (flip-flops)
)(
    input  wire        clk,
    input  wire        rst_n,

    // Broadcast SIMD instruction from the global sequencer
    input  wire [23:0] instr,
    input  wire        instr_valid,

    // Local memory (1024 x 16)
    output reg  [9:0]  mem_addr,
    output reg  [15:0] mem_wdata,
    output reg         mem_we,
    input  wire [15:0] mem_rdata,

    // 2D NEWS nearest-neighbour grid
    input  wire [15:0] news_n_in,
    input  wire [15:0] news_e_in,
    input  wire [15:0] news_w_in,
    input  wire [15:0] news_s_in,
    output wire [15:0] news_out,

    // Network: {valid, 1'b0, dest[5:0], data[15:0]}
    output reg  [23:0] net_tx_pkt,
    output reg         net_tx_valid,
    input  wire        net_tx_ready,
    input  wire [15:0] net_rx_data,
    input  wire        net_rx_valid,
    output reg         net_rx_pop,

    // Status & host readback
    output wire        ctx_active,
    output wire [15:0] acc_out
);

    // Instruction fields
    wire [3:0]  opcode   = instr[23:20];
    wire [3:0]  sub_op   = instr[19:16];
    wire [1:0]  src_sel  = instr[15:14];
    wire [3:0]  flag_ctl = instr[13:10];
    wire [9:0]  imm10    = instr[9:0];
    wire [7:0]  imm8     = instr[7:0];
    wire [15:0] simm     = {{6{imm10[9]}}, imm10};

    localparam OP_NOP  = 4'h0, OP_LDI  = 4'h1, OP_LDM  = 4'h2, OP_STM  = 4'h3,
               OP_LDX  = 4'h4, OP_STX  = 4'h5, OP_MAR  = 4'h6, OP_ALU  = 4'h7,
               OP_SWP  = 4'h8, OP_CTX  = 4'h9, OP_SEND = 4'hA, OP_RECV = 4'hB,
               OP_NEWS = 4'hC, OP_FPU  = 4'hD;

    localparam F_ADD = 4'h0, F_SUB = 4'h1, F_MUL = 4'h2, F_CMP = 4'h3,
               F_I2F = 4'h4, F_F2I = 4'h5, F_NEG = 4'h6, F_ABS = 4'h7;

    // Registers
    reg [15:0] acc;
    reg [15:0] b_reg;
    reg [9:0]  mar;
    reg        flag_c, flag_z, flag_n;
    reg [3:0]  ctx_stack;
    wire       ctx = ctx_stack[0];

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
    wire [9:0] addr_now = use_mar ? mar : imm10;
    wire we_now    = instr_valid && ctx && ((opcode == OP_STM) || (opcode == OP_STX));

    (* syn_preserve = 1 *) reg        wr_pend;
    (* syn_preserve = 1 *) reg [9:0]  wr_addr_q;
    (* syn_preserve = 1 *) reg [15:0] wr_data_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_pend   <= 1'b0;
            wr_addr_q <= 10'd0;
            wr_data_q <= 16'd0;
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

    // Registered memory read data. The sequencer waits two cycles on any
    // read, so mem_q holds the word by the time the instruction executes,
    // and the BSRAM clock-to-out delay stays off the ALU/FPU paths.
    reg [15:0] mem_q;
    always @(posedge clk) mem_q <= mem_rdata;

    // ------------------------------------------------------------------
    // Register file: r0 is b_reg; r1..r7 are flip-flops (this device has
    // no distributed RAM; entry 0 of rf is never used).
    // ------------------------------------------------------------------
    wire [2:0]  r_idx = imm10[2:0];
    wire [15:0] r_val;
    wire        r_put = instr_valid && ctx && (opcode == OP_SWP) &&
                        (sub_op != 4'd2);       // SWP or PUT

    generate
        if (HAS_RF) begin : g_rf
            reg [15:0] rf [0:7];
            always @(posedge clk)
                if (r_put && (r_idx != 3'd0))
                    rf[r_idx] <= acc;
            assign r_val = (r_idx == 3'd0) ? b_reg : rf[r_idx];
        end else begin : g_no_rf   // every r[n] is b
            assign r_val = b_reg;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Operand B
    // ------------------------------------------------------------------
    reg [15:0] op_b;
    always @(*) begin
        case (src_sel)
            2'b00: op_b = r_val;
            2'b01: op_b = simm;
            2'b10: op_b = {10'd0, PE_ID};
            2'b11: op_b = mem_q;
        endcase
    end

    // ------------------------------------------------------------------
    // Integer ALU
    // ------------------------------------------------------------------
    wire        alu_x    = flag_ctl[2];

    // 17 x 17 signed multiply covers both signed and unsigned 16-bit
    // operands (fits one 18 x 18 DSP); the low half is the same either way.
    wire signed [16:0] mul_a    = {~alu_x & acc[15],  acc};
    wire signed [16:0] mul_b    = {~alu_x & op_b[15], op_b};
    wire signed [33:0] mul_full = mul_a * mul_b;

    // Barrel shifter: one 33-bit right shift by k = 0..16 of {hi, lo, 0}
    // serves every shift and rotate.
    //   right by n:  hi = fill, lo = acc,  k = n       result = sh[16:1], C = sh[0]
    //   left by n:   hi = acc,  lo = fill, k = 16 - n  result = sh[16:1], C = sh[17]
    wire [3:0]  sh_n    = op_b[3:0];
    wire        sh_left = (sub_op == 4'hC) || ((sub_op == 4'hF) && !alu_x);
    reg  [15:0] sh_hi, sh_lo;
    always @(*) begin
        case (sub_op)
            4'hC:    begin sh_hi = acc;                        sh_lo = alu_x ? b_reg : 16'd0; end // SHLN
            4'hD:    begin sh_hi = alu_x ? b_reg : 16'd0;      sh_lo = acc;                   end // SHRN
            4'hE:    begin sh_hi = {16{acc[15]}};              sh_lo = acc;                   end // ASRN
            default: begin sh_hi = acc;                        sh_lo = acc;                   end // ROL/ROR
        endcase
    end
    wire [4:0]  sh_k   = sh_left ? (5'd16 - {1'b0, sh_n}) : {1'b0, sh_n};
    wire [32:0] sh_out = {sh_hi, sh_lo, 1'b0} >> sh_k;
    wire [15:0] sh_res = sh_out[16:1];
    reg         sh_c;
    always @(*) begin
        if (sh_n == 4'd0)       sh_c = 1'b0;
        else if (sub_op == 4'hF) sh_c = alu_x ? sh_res[15] : sh_res[0];
        else if (sh_left)        sh_c = sh_out[17];
        else                     sh_c = sh_out[0];
    end

    reg  [16:0] alu_res;
    always @(*) begin
        alu_res = {1'b0, acc};
        case (sub_op)
            4'h0: alu_res = {1'b0, acc} + {1'b0, op_b};                    // ADD
            4'h1: alu_res = {1'b0, acc} + {1'b0, op_b} + {16'd0, flag_c};  // ADC
            4'h2: alu_res = {1'b0, acc} - {1'b0, op_b};                    // SUB (C = borrow)
            4'h3: alu_res = {1'b0, acc & op_b};                            // AND
            4'h4: alu_res = {1'b0, acc | op_b};                            // OR
            4'h5: alu_res = {1'b0, acc ^ op_b};                            // XOR
            4'h6: alu_res = {acc[15], acc[14:0], 1'b0};                    // SHL
            4'h7: alu_res = {acc[0], 1'b0, acc[15:1]};                     // SHR
            4'h8: alu_res = {1'b0, op_b};                                  // MOV
            4'h9: alu_res = {acc[0], acc[15], acc[15:1]};                  // ASR
            4'hA: alu_res = {1'b0, mul_full[15:0]};                        // MUL
            4'hB: alu_res = {1'b0, mul_full[31:16]};                       // MULH
            4'hC, 4'hD, 4'hE, 4'hF:
                  alu_res = {sh_c, sh_res};                                // SHLN/SHRN/ASRN/ROL
            default: alu_res = {1'b0, acc};
        endcase
    end

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
    wire fp_start = instr_valid && ctx && fp_arith;
    wire [1:0] fp_op = (sub_op == F_ADD) ? 2'd0 :
                       (sub_op == F_SUB) ? 2'd1 :
                       (sub_op == F_MUL) ? 2'd2 : 2'd3;
    wire [15:0] fp_result;
    reg         fp_wb1, fp_wb2;

    dm16_bf16 u_fpu (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (fp_start),
        .op     (fp_op),
        .a      (acc),
        .b      (op_b),
        .result (fp_result)
    );

    // bf16 -> int16, truncate toward zero, saturate
    wire [7:0]  acc_e = acc[14:7];
    reg  [15:0] f2i, f2i_mag;
    always @(*) begin
        f2i_mag = 16'd0;
        if (acc_e < 8'd127)
            f2i = 16'd0;
        else if (acc_e >= 8'd142)
            f2i = acc[15] ? 16'h8000 : 16'h7FFF;
        else begin
            f2i_mag = {1'b1, acc[6:0], 8'd0} >> (8'd142 - acc_e);
            f2i     = acc[15] ? (~f2i_mag + 16'd1) : f2i_mag;
        end
    end

    // Compare acc vs op_b (subnormals = zero, +0 == -0)
    wire        x_nan = (acc[14:7]  == 8'hFF) && (acc[6:0]  != 7'd0);
    wire        y_nan = (op_b[14:7] == 8'hFF) && (op_b[6:0] != 7'd0);
    wire [15:0] x_c   = (acc[14:7]  == 8'd0) ? 16'd0 : acc;
    wire [15:0] y_c   = (op_b[14:7] == 8'd0) ? 16'd0 : op_b;
    wire [15:0] x_key = x_c[15] ? ~x_c : (x_c | 16'h8000);
    wire [15:0] y_key = y_c[15] ? ~y_c : (y_c | 16'h8000);

    // SEND destination
    wire [5:0] send_dest = (src_sel == 2'b01) ? acc[5:0] :
                           (src_sel == 2'b10) ? (PE_ID ^ imm10[5:0]) : imm10[5:0];

    // ------------------------------------------------------------------
    // State update
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc          <= 16'd0;
            b_reg        <= 16'd0;
            mar          <= 10'd0;
            flag_c       <= 1'b0;
            flag_z       <= 1'b0;
            flag_n       <= 1'b0;
            fp_wb1       <= 1'b0;
            fp_wb2       <= 1'b0;
            ctx_stack    <= 4'b1111;
            net_tx_pkt   <= 24'd0;
            net_tx_valid <= 1'b0;
            net_rx_pop   <= 1'b0;
        end else begin
            if (net_tx_valid && net_tx_ready)
                net_tx_valid <= 1'b0;

            net_rx_pop <= 1'b0;

            // FPU write-back, two clocks after the arithmetic op executed
            fp_wb1 <= fp_start;
            fp_wb2 <= fp_wb1;
            if (fp_wb2)
                acc <= fp_result;

            if (instr_valid) begin
                if (opcode == OP_CTX) begin
                    case (flag_ctl[1:0])
                        2'b00: ctx_stack[0] <= cond_eval;                     // SET
                        2'b01: ctx_stack    <= {ctx_stack[2:0], cond_eval};   // PUSH
                        2'b10: ctx_stack    <= {1'b1, ctx_stack[3:1]};        // POP
                        2'b11: ctx_stack    <= ~ctx_stack;                    // INVERT
                    endcase
                end else if (ctx) begin
                    if (uses_mem && flag_ctl[1])
                        mar <= mar + 10'd1;

                    case (opcode)
                        OP_LDI: case (sub_op[1:0])
                                    2'd1:    acc[15:8] <= imm8;
                                    2'd2:    acc[7:0]  <= imm8;
                                    default: acc       <= simm;
                                endcase

                        OP_LDM, OP_LDX: acc <= mem_q;

                        OP_MAR: mar <= (src_sel == 2'b00) ? acc[9:0] : imm10;

                        OP_ALU: begin
                            acc    <= alu_res[15:0];
                            flag_c <= alu_res[16];
                            flag_z <= (alu_res[15:0] == 16'd0);
                            flag_n <= alu_res[15];
                        end

                        OP_SWP: begin       // r1..r7 are written by g_rf
                            if (sub_op != 4'd1)
                                acc <= r_val;                           // SWP, GET
                            if ((sub_op != 4'd2) && (r_idx == 3'd0 || !HAS_RF))
                                b_reg <= acc;                           // SWP, PUT
                        end

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

                        OP_NEWS: case (sub_op[1:0])
                                     2'b00: acc <= news_n_in;
                                     2'b01: acc <= news_e_in;
                                     2'b10: acc <= news_w_in;
                                     2'b11: acc <= news_s_in;
                                 endcase

                        OP_FPU: case (sub_op)
                                    F_CMP: begin
                                        flag_c <= x_nan | y_nan;
                                        flag_z <= !(x_nan | y_nan) && (x_key == y_key);
                                        flag_n <= !(x_nan | y_nan) && (x_key <  y_key);
                                    end
                                    F_F2I: acc <= f2i;
                                    F_NEG: acc <= acc ^ 16'h8000;
                                    F_ABS: acc <= acc & 16'h7FFF;
                                    default: ;   // ADD/SUB/MUL/I2F via fp_wb
                                endcase

                        default: ;
                    endcase
                end
            end
        end
    end

endmodule
