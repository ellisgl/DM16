module dm8_pe #(
    parameter [5:0] PE_ID = 6'd0
)(
    input  wire        clk,
    input  wire        rst_n,

    // Broadcast SIMD Microinstruction from Global Sequencer
    input  wire [23:0] instr,
    input  wire        instr_valid,

    // Local Dual-Port BRAM Interface (1024 x 8-bit)
    output reg  [9:0]  mem_addr,
    output reg  [7:0]  mem_wdata,
    output reg         mem_we,
    input  wire [7:0]  mem_rdata,

    // 2D NEWS Nearest-Neighbor Grid Inputs/Outputs
    input  wire [7:0]  news_n_in,
    input  wire [7:0]  news_e_in,
    input  wire [7:0]  news_w_in,
    input  wire [7:0]  news_s_in,
    output wire [7:0]  news_out,

    // Hypercube Router Interface
    output reg  [15:0] net_tx_pkt,
    output reg         net_tx_valid,
    input  wire        net_tx_ready,
    input  wire [15:0] net_rx_pkt,
    input  wire        net_rx_valid,
    output reg         net_rx_pop,

    // Status & Host Readback Outputs
    output wire        ctx_active,
    output wire [7:0]  acc_out
);

    // Instruction Field Decoding
    wire [3:0] opcode   = instr[23:20];
    wire [3:0] sub_op   = instr[19:16];
    wire [1:0] src_sel  = instr[15:14];
    wire [3:0] flag_ctl = instr[13:10];
    wire [9:0] imm10    = instr[9:0];
    wire [7:0] imm8     = instr[7:0];

    // Opcodes
    localparam OP_NOP     = 4'h0,
               OP_LDI     = 4'h1,
               OP_LDM     = 4'h2,
               OP_STM     = 4'h3,
               OP_LDX     = 4'h4,
               OP_STX     = 4'h5,
               OP_MOV_MAR = 4'h6,
               OP_ALU     = 4'h7,
               OP_SWP     = 4'h8,
               OP_SET_CTX = 4'h9,
               OP_SEND    = 4'hA,
               OP_RECV    = 4'hB,
               OP_NEWS    = 4'hC;

    // Core Registers
    reg [7:0] acc;
    reg [7:0] b_reg;
    reg [9:0] mar;
    reg       flag_c, flag_z, flag_n;

    // 4-bit Context Stack (bit 0 is current active context)
    reg [3:0] ctx_stack;
    wire      ctx = ctx_stack[0:0];

    assign ctx_active = ctx;
    assign news_out   = acc;
    assign acc_out    = acc;

    // ALU Operand B Selection
    reg [7:0] op_b;
    always @(*) begin
        case (src_sel)
            2'b00: op_b = b_reg;
            2'b01: op_b = imm8;
            2'b10: op_b = {2'b00, PE_ID};
            2'b11: op_b = mem_rdata;
        endcase
    end

    // Combinational 8-bit ALU
    reg [8:0] alu_res;
    always @(*) begin
        alu_res = {1'b0, acc};
        case (sub_op)
            4'h0: alu_res = {1'b0, acc} + {1'b0, op_b};                  // ADD
            4'h1: alu_res = {1'b0, acc} + {1'b0, op_b} + {8'd0, flag_c}; // ADC
            4'h2: alu_res = {1'b0, acc} - {1'b0, op_b};                  // SUB
            4'h3: alu_res = {1'b0, acc & op_b};                          // AND
            4'h4: alu_res = {1'b0, acc | op_b};                          // OR
            4'h5: alu_res = {1'b0, acc ^ op_b};                          // XOR
            4'h6: alu_res = {acc[7:7], acc[6:0], 1'b0};                  // SHL
            4'h7: alu_res = {acc[0:0], 1'b0, acc[7:1]};                  // SHR
            4'h8: alu_res = {1'b0, op_b};                                // MOV
            default: alu_res = {1'b0, acc};
        endcase
    end

    // Memory Address & Write-Enable Combinational Logic
    always @(*) begin
        mem_addr  = (opcode == OP_LDX || opcode == OP_STX) ? mar : imm10;
        mem_wdata = acc;
        mem_we    = instr_valid && ctx && (opcode == OP_STM || opcode == OP_STX);
    end

    // Context Condition Evaluator (Unconditional)
    reg cond_eval;
    always @(*) begin
        case (sub_op)
            4'h0: cond_eval = 1'b1;                 // ALWAYS ON
            4'h1: cond_eval = 1'b0;                 // ALWAYS OFF
            4'h2: cond_eval = flag_z;               // IF ZERO
            4'h3: cond_eval = !flag_z;              // IF NOT ZERO
            4'h4: cond_eval = flag_c;               // IF CARRY
            4'h5: cond_eval = flag_n;               // IF NEGATIVE
            4'h6: cond_eval = (PE_ID == imm8[5:0]); // IF PE_ID == IMM
            default: cond_eval = ctx;
        endcase
    end

    // Sequential State Update
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc          <= 8'd0;
            b_reg        <= 8'd0;
            mar          <= 10'd0;
            flag_c       <= 1'b0;
            flag_z       <= 1'b0;
            flag_n       <= 1'b0;
            ctx_stack    <= 4'b1111;
            net_tx_pkt   <= 16'd0;
            net_tx_valid <= 1'b0;
            net_rx_pop   <= 1'b0;
        end else begin
            // Hold net_tx_valid until the router accepts the packet
            if (net_tx_valid && net_tx_ready)
                net_tx_valid <= 1'b0;

            net_rx_pop <= 1'b0;

            if (instr_valid) begin
                if (opcode == OP_SET_CTX) begin
                    case (flag_ctl[1:0])
                        2'b00: ctx_stack[0:0] <= cond_eval;                   // SET
                        2'b01: ctx_stack      <= {ctx_stack[2:0], cond_eval}; // PUSH
                        2'b10: ctx_stack      <= {1'b1, ctx_stack[3:1]};      // POP
                        2'b11: ctx_stack      <= ~ctx_stack;                  // INVERT
                    endcase
                end else if (ctx) begin
                    case (opcode)
                        OP_LDI: acc <= imm8;

                        OP_LDM,
                        OP_LDX: acc <= mem_rdata;

                        OP_MOV_MAR: begin
                            mar <= (src_sel == 2'b00) ? {b_reg[1:0], acc} : imm10;
                        end

                        OP_ALU: begin
                            acc    <= alu_res[7:0];
                            flag_c <= alu_res[8:8];
                            flag_z <= (alu_res[7:0] == 8'd0);
                            flag_n <= alu_res[7:7];
                        end

                        OP_SWP: begin
                            acc   <= b_reg;
                            b_reg <= acc;
                        end

                        OP_SEND: begin
                            net_tx_pkt   <= {1'b1, imm8[5:0], 1'b0, b_reg};
                            net_tx_valid <= 1'b1;
                        end

                        OP_RECV: begin
                            if (net_rx_valid) begin
                                b_reg      <= net_rx_pkt[7:0];
                                net_rx_pop <= 1'b1;
                                flag_c     <= 1'b1;
                            end else begin
                                flag_c     <= 1'b0;
                            end
                        end

                        OP_NEWS: begin
                            case (sub_op[1:0])
                                2'b00: acc <= news_n_in;
                                2'b01: acc <= news_e_in;
                                2'b10: acc <= news_w_in;
                                2'b11: acc <= news_s_in;
                            endcase
                        end

                        default: ;
                    endcase
                end
            end
        end
    end

endmodule