`timescale 1ns/1ns

module dm16_uart_seq #(
    parameter integer CLK_HZ = 54_000_000,
    parameter integer BAUD   = 115_200,
    parameter integer NUM_PE = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         uart_rx,
    output reg          uart_tx,

    // Sequencer broadcast to 64-PE array
    output reg  [23:0]  current_instr,
    output reg          instr_valid,
    output reg          running,

    // Live status from 64-PE array
    input  wire [NUM_PE-1:0]   all_ctx,
    input  wire [NUM_PE*16-1:0] all_acc,
    input  wire                 any_active   // registered global OR of all_ctx
);

    // =========================================================================
    // 1. UART RX & TX
    // =========================================================================
    // Clocks per bit start at CLK_HZ / BAUD (115200 after every reset) and
    // can be changed at run time by the host with command 0x08 (see below),
    // e.g. 75 MHz / 3 Mbaud = 25.
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;
    reg [9:0] clks_per_bit;

    reg [9:0] rx_clk_cnt;
    reg [2:0] rx_bit_idx;
    reg [7:0] rx_shift;
    reg [7:0] rx_byte;
    reg       rx_valid;
    reg [1:0] rx_state;
    reg       rx_sync1, rx_sync2;

    always @(posedge clk) begin
        rx_sync1 <= uart_rx;
        rx_sync2 <= rx_sync1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state   <= 0;
            rx_clk_cnt <= 0;
            rx_bit_idx <= 0;
            rx_valid   <= 0;
        end else begin
            rx_valid <= 0;
            case (rx_state)
                0: if (!rx_sync2) begin
                    rx_state   <= 1;
                    rx_clk_cnt <= 0;
                end
                1: if (rx_clk_cnt == {1'b0, clks_per_bit[9:1]}) begin
                    if (!rx_sync2) begin
                        rx_clk_cnt <= 0;
                        rx_bit_idx <= 0;
                        rx_state   <= 2;
                    end else rx_state <= 0;
                end else rx_clk_cnt <= rx_clk_cnt + 10'd1;
                2: if (rx_clk_cnt == clks_per_bit - 10'd1) begin
                    rx_clk_cnt           <= 0;
                    rx_shift[rx_bit_idx] <= rx_sync2;
                    if (rx_bit_idx == 7) rx_state <= 3;
                    else rx_bit_idx <= rx_bit_idx + 3'd1;
                end else rx_clk_cnt <= rx_clk_cnt + 10'd1;
                3: if (rx_clk_cnt == clks_per_bit - 10'd1) begin
                    rx_byte  <= rx_shift;
                    rx_valid <= 1;
                    rx_state <= 0;
                end else rx_clk_cnt <= rx_clk_cnt + 10'd1;
            endcase
        end
    end

    reg [9:0] tx_clk_cnt;
    reg [2:0] tx_bit_idx;
    reg [7:0] tx_shift;
    reg       tx_busy;
    reg [1:0] tx_state;

    // Single-driver handshake signals from Host Loader FSM
    reg       tx_start;
    reg [7:0] tx_data_in;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx    <= 1;
            tx_busy    <= 0;
            tx_state   <= 0;
            tx_clk_cnt <= 0;
            tx_bit_idx <= 0;
            tx_shift   <= 0;
        end else if (tx_start && !tx_busy) begin
            tx_shift   <= tx_data_in;
            tx_busy    <= 1;
            tx_state   <= 1;
            tx_clk_cnt <= 0;
            tx_bit_idx <= 0;
        end else if (tx_busy) begin
            case (tx_state)
                1: begin
                    uart_tx <= 0;
                    if (tx_clk_cnt == clks_per_bit - 10'd1) begin
                        tx_clk_cnt <= 0;
                        tx_state   <= 2;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
                2: begin
                    uart_tx <= tx_shift[tx_bit_idx];
                    if (tx_clk_cnt == clks_per_bit - 10'd1) begin
                        tx_clk_cnt <= 0;
                        if (tx_bit_idx == 7) tx_state <= 3;
                        else tx_bit_idx <= tx_bit_idx + 3'd1;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
                3: begin
                    uart_tx <= 1;
                    if (tx_clk_cnt == clks_per_bit - 10'd1) begin
                        tx_busy  <= 0;
                        tx_state <= 0;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
            endcase
        end
    end

    // =========================================================================
    // 2. Host Loader FSM & 4096 x 24-bit Instruction Memory
    // =========================================================================
    //   0x01 a_hi a_lo i2 i1 i0   write imem[a] (12-bit address)
    //   0x02 e_hi e_lo            run from 0 until pc == e (or HALT)
    //   0x03                      single step
    //   0x04                      halt, pc = 0
    //   0x05                      read the context mask (NUM_PE/8 bytes)
    //   0x06 i2 i1 i0             execute one instruction immediately
    //   0x07                      read every PE's ACC (2 * NUM_PE bytes)
    //   0x08 d_hi d_lo            set UART clocks per bit
    //   0x09                      read status byte {6'b0, any_active, running}
    //   0x0A a_hi a_lo n_hi n_lo  bulk read: n words from address a of every PE
    //                             (enables all PEs, then for each word injects
    //                             LDM and streams the ACC dump: n * 2 * NUM_PE
    //                             bytes; clobbers ACC; the program must be idle)
    localparam integer PC_W = 12;
    reg [23:0]     imem [0:(1 << PC_W) - 1];
    reg [PC_W-1:0] pc;
    reg [PC_W-1:0] end_pc;
    reg        single_step;
    reg [4:0]  cmd_state;
    reg [PC_W-1:0] wr_addr;
    reg [23:0] wr_data;
    reg [7:0]  dump_idx;

    reg [23:0] inj_instr;
    reg        inj_valid;
    reg        inj_active;

    // Bulk read (0x0A)
    localparam [23:0] I_CTX_ALWAYS = 24'h900000;
    localparam [23:0] I_LDM        = 24'h200000;    // | address
    reg [9:0]  br_addr;
    reg [10:0] br_left;
    reg        br_ctx_done;
    reg [5:0]  br_wait;

    // A multi-byte command that stalls for 2^17 clocks (2.6 ms at 50 MHz)
    // is abandoned, so line noise or bytes sent at the wrong baud rate
    // can't leave the command decoder waiting for data forever.
    reg [16:0] cmd_idle;
    // States that wait for argument bytes from the host
    wire       cmd_in_args = ((cmd_state >= 5'd1)  && (cmd_state <= 5'd9))  ||
                             ((cmd_state >= 5'd12) && (cmd_state <= 5'd14)) ||
                             ((cmd_state >= 5'd16) && (cmd_state <= 5'd19));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_state   <= 0;
            running     <= 0;
            single_step <= 0;
            end_pc      <= 0;
            dump_idx    <= 0;
            tx_start    <= 0;
            tx_data_in  <= 0;
            inj_instr   <= 0;
            inj_valid   <= 0;
            clks_per_bit <= CLKS_PER_BIT;
            cmd_idle    <= 0;
            br_addr     <= 0;
            br_left     <= 0;
            br_ctx_done <= 0;
            br_wait     <= 0;
        end else begin
            single_step <= 0;
            tx_start    <= 0;
            inj_valid   <= 0;

            if (rx_valid || !cmd_in_args)
                cmd_idle <= 0;
            else if (cmd_idle == {17{1'b1}})
                cmd_state <= 4'd0;
            else
                cmd_idle <= cmd_idle + 17'd1;

            if (running && (pc == end_pc))
                running <= 0;

            // State 10: Stream the CTX mask (NUM_PE/8 bytes)
            if (cmd_state == 4'd10 && !tx_busy && !tx_start) begin
                if (dump_idx < NUM_PE / 8) begin
                    tx_data_in <= all_ctx[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 8'd1;
                end else cmd_state <= 4'd0;
            end

            // State 15: Send the status byte
            if (cmd_state == 4'd15 && !tx_busy && !tx_start) begin
                tx_data_in <= {6'd0, any_active, running};
                tx_start   <= 1'b1;
                cmd_state  <= 4'd0;
            end

            // State 11: Stream every PE's 16-bit ACC, little-endian (2*NUM_PE bytes)
            if (cmd_state == 4'd11 && !tx_busy && !tx_start) begin
                if (dump_idx < 2 * NUM_PE) begin
                    tx_data_in <= all_acc[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 8'd1;
                end else cmd_state <= 4'd0;
            end

            // States 20-22: bulk read. Inject an instruction (CTX ALWAYS once,
            // then LDM per word), give it 64 clocks to reach every ACC (it
            // needs about 10), then stream the ACC dump as in state 11.
            if (cmd_state == 5'd20) begin
                inj_instr <= br_ctx_done ? (I_LDM | {14'd0, br_addr}) : I_CTX_ALWAYS;
                inj_valid <= 1'b1;
                br_wait   <= 6'd0;
                cmd_state <= 5'd21;
            end
            if (cmd_state == 5'd21) begin
                br_wait <= br_wait + 6'd1;
                if (br_wait == 6'd63) begin
                    if (!br_ctx_done) begin
                        br_ctx_done <= 1'b1;
                        cmd_state   <= 5'd20;
                    end else begin
                        dump_idx    <= 8'd0;
                        cmd_state   <= 5'd22;
                    end
                end
            end
            if (cmd_state == 5'd22 && !tx_busy && !tx_start) begin
                if (dump_idx < 2 * NUM_PE) begin
                    tx_data_in <= all_acc[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 8'd1;
                end else begin
                    br_addr   <= br_addr + 10'd1;
                    br_left   <= br_left - 11'd1;
                    cmd_state <= (br_left == 11'd1) ? 5'd0 : 5'd20;
                end
            end

            if (rx_valid) begin
                case (cmd_state)
                    4'd0: case (rx_byte)
                        8'h01: cmd_state   <= 4'd1; // WRITE_IMEM
                        8'h02: cmd_state   <= 4'd6; // RUN
                        8'h03: single_step <= 1'b1; // STEP
                        8'h04: running     <= 1'b0; // HALT/RESET
                        8'h05: begin dump_idx <= 8'd0; cmd_state <= 4'd10; end // READ_CTX
                        8'h06: cmd_state   <= 4'd8; // INJECT_INSTR (3 bytes)
                        8'h07: begin dump_idx <= 8'd0; cmd_state <= 4'd11; end // READ_ACC_GRID
                        8'h08: cmd_state   <= 4'd13; // SET_BAUD (2 bytes: clocks per bit, big-endian)
                        8'h09: cmd_state   <= 4'd15; // READ_STATUS
                        8'h0A: cmd_state   <= 5'd16; // BULK_READ (4 bytes)
                    endcase
                    4'd1: begin wr_addr[11:8]  <= rx_byte[3:0]; cmd_state <= 4'd2; end
                    4'd2: begin wr_addr[7:0]   <= rx_byte;      cmd_state <= 4'd3; end
                    4'd3: begin wr_data[23:16] <= rx_byte;      cmd_state <= 4'd4; end
                    4'd4: begin wr_data[15:8]  <= rx_byte;      cmd_state <= 4'd5; end
                    4'd5: begin
                        imem[wr_addr] <= (wr_data & 24'hFFFF00) | rx_byte;
                        cmd_state     <= 4'd0;
                    end
                    4'd6: begin end_pc[11:8] <= rx_byte[3:0]; cmd_state <= 4'd7; end
                    4'd7: begin
                        end_pc[7:0] <= rx_byte;
                        running     <= 1'b1;
                        cmd_state   <= 4'd0;
                    end
                    4'd8: begin wr_data[23:16] <= rx_byte; cmd_state <= 4'd9; end
                    4'd9: begin wr_data[15:8]  <= rx_byte; cmd_state <= 4'd12; end
                    4'd13: begin wr_data[15:8] <= rx_byte; cmd_state <= 4'd14; end
                    4'd14: begin
                        // Takes effect from the next byte; values below 8 are ignored
                        if ({wr_data[9:8], rx_byte} >= 10'd8)
                            clks_per_bit <= {wr_data[9:8], rx_byte};
                        cmd_state <= 4'd0;
                    end
                    4'd12: begin
                        inj_instr <= (wr_data & 24'hFFFF00) | rx_byte;
                        inj_valid <= 1'b1;
                        cmd_state <= 4'd0;
                    end
                    5'd16: begin br_addr[9:8] <= rx_byte[1:0]; cmd_state <= 5'd17; end
                    5'd17: begin br_addr[7:0] <= rx_byte;      cmd_state <= 5'd18; end
                    5'd18: begin br_left[10:8] <= rx_byte[2:0]; cmd_state <= 5'd19; end
                    5'd19: begin
                        br_left[7:0] <= rx_byte;
                        br_ctx_done  <= 1'b0;
                        cmd_state    <= ({br_left[10:8], rx_byte} == 11'd0) ? 5'd0 : 5'd20;
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // 3. Global SIMD Sequencer
    // =========================================================================
    // Opcode 0xF is sequencer control - executed here, never sent to the PEs:
    //   F0 LOOP  n     run the body up to the matching ENDL n times
    //                  (n = instr[15:0], 0 counts as 1; nests 4 deep)
    //   F1 ENDL
    //   F2 JMP   addr  addr = instr[11:0]
    //   F3 BANY  addr  branch if any PE is active (global OR of contexts)
    //   F4 BNONE addr  branch if no PE is active
    //   F5 HALT
    // A branch waits GOR_WAIT clocks before sampling any_active, which covers
    // the instruction broadcast pipeline (cluster + PE registers), the PE
    // update and the two-stage OR tree, so it always sees the result of the
    // preceding SET_CTX.
    localparam integer GOR_WAIT = 8;
    localparam [3:0] C_LOOP = 4'h0, C_ENDL = 4'h1, C_JMP = 4'h2,
                     C_BANY = 4'h3, C_BNONE = 4'h4, C_HALT = 4'h5;

    reg mem_wait, mem_wait2, fp_wait;
    reg [23:0] fetched_instr;
    always @(posedge clk) begin
        fetched_instr <= imem[pc];
    end

    reg imem_wait;
    // Instructions that read PE memory get two extra cycles: one for the
    // BSRAM read, one for the PE's mem_q register (which keeps the BSRAM
    // clock-to-out delay off the ALU/FPU paths):
    // LDM, LDX, and ALU/FPU ops with a memory operand (src_sel = 3).
    function is_read;
        input [23:0] ins;
        is_read = (ins[23:20] == 4'h2) || (ins[23:20] == 4'h4) ||
                  (((ins[23:20] == 4'h7) || (ins[23:20] == 4'hD)) && (ins[15:14] == 2'b11));
    endfunction
    wire is_mem_read = is_read(fetched_instr);
    wire is_inj_read = is_read(inj_instr);

    // FADD/FSUB/FMUL/FI2F write their result two clocks after executing,
    // so they get one extra idle cycle before the next instruction.
    function is_fp_slow;
        input [23:0] ins;
        is_fp_slow = (ins[23:20] == 4'hD) &&
                     ((ins[19:16] == 4'h0) || (ins[19:16] == 4'h1) ||
                      (ins[19:16] == 4'h2) || (ins[19:16] == 4'h4));
    endfunction

    wire            is_ctl   = (fetched_instr[23:20] == 4'hF);
    wire [3:0]      ctl_op   = fetched_instr[19:16];
    wire [PC_W-1:0] ctl_tgt  = fetched_instr[PC_W-1:0];
    wire [15:0]     ctl_cnt  = fetched_instr[15:0];

    // Hardware loop stack
    reg [PC_W-1:0] lp_start [0:3];
    reg [15:0]     lp_cnt   [0:3];
    reg [2:0]      lp_sp;
    wire [1:0]     lp_top = lp_sp[1:0] - 2'd1;

    reg [3:0]      gor_cnt;
    reg            gor_armed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc            <= 0;
            current_instr <= 0;
            instr_valid   <= 0;
            mem_wait      <= 0;
            mem_wait2     <= 0;
            fp_wait       <= 0;
            imem_wait     <= 1'b0;
            inj_active    <= 0;
            lp_sp         <= 0;
            gor_cnt       <= 0;
            gor_armed     <= 0;
        end else begin
            instr_valid <= 0;

            if (rx_valid && (cmd_state == 4'd0) && (rx_byte == 8'h04)) begin
                pc         <= 0;
                mem_wait   <= 0;
                mem_wait2  <= 0;
                fp_wait    <= 0;
                inj_active <= 0;
                imem_wait  <= 1'b0;
                lp_sp      <= 0;
                gor_armed  <= 0;
            end else if (mem_wait2) begin
                mem_wait2 <= 1'b0;
                mem_wait  <= 1'b1;
            end else if (mem_wait) begin
                instr_valid <= 1'b1;
                mem_wait    <= 1'b0;
                fp_wait     <= is_fp_slow(current_instr);
                imem_wait   <= 1'b1;        // the PEs need an idle clock after every instruction
                if (!inj_active)
                    pc <= pc + 1'b1;
                inj_active  <= 1'b0;
            end else if (imem_wait) begin
                // Wait 1 clock for synchronous BSRAM fetched_instr[pc] to update
                imem_wait <= 1'b0;
            end else if (fp_wait) begin
                fp_wait <= 1'b0;
            end else if (inj_valid) begin
                // Injected control instructions are ignored
                if (inj_instr[23:20] != 4'hF) begin
                    current_instr <= inj_instr;
                    if (is_inj_read) begin
                        mem_wait2  <= 1'b1;
                        inj_active <= 1'b1;
                    end else begin
                        instr_valid <= 1'b1;
                        imem_wait   <= 1'b1;    // idle clock after every instruction
                        fp_wait     <= is_fp_slow(inj_instr);
                    end
                end
            end else if ((running && pc < end_pc) || single_step) begin
                if (is_ctl) begin
                    imem_wait <= 1'b1;
                    case (ctl_op)
                        C_LOOP: begin
                            if (lp_sp != 3'd4) begin
                                lp_start[lp_sp[1:0]] <= pc + 1'b1;
                                lp_cnt[lp_sp[1:0]]   <= (ctl_cnt == 16'd0) ? 16'd1 : ctl_cnt;
                                lp_sp                <= lp_sp + 3'd1;
                            end
                            pc <= pc + 1'b1;
                        end
                        C_ENDL: begin
                            if (lp_sp != 3'd0 && lp_cnt[lp_top] > 16'd1) begin
                                lp_cnt[lp_top] <= lp_cnt[lp_top] - 16'd1;
                                pc             <= lp_start[lp_top];
                            end else begin
                                if (lp_sp != 3'd0) lp_sp <= lp_sp - 3'd1;
                                pc <= pc + 1'b1;
                            end
                        end
                        C_JMP:  pc <= ctl_tgt;
                        C_BANY, C_BNONE: begin
                            if (!gor_armed) begin
                                gor_armed <= 1'b1;
                                gor_cnt   <= GOR_WAIT;
                                imem_wait <= 1'b0;
                            end else if (gor_cnt != 4'd0) begin
                                gor_cnt   <= gor_cnt - 4'd1;
                                imem_wait <= 1'b0;
                            end else begin
                                gor_armed <= 1'b0;
                                if ((ctl_op == C_BANY) ? any_active : !any_active)
                                    pc <= ctl_tgt;
                                else
                                    pc <= pc + 1'b1;
                            end
                        end
                        C_HALT: pc <= end_pc;   // the loader stops when pc == end_pc
                        default: pc <= pc + 1'b1;
                    endcase
                end else begin
                    current_instr <= fetched_instr;
                    if (is_mem_read) begin
                        mem_wait2 <= 1'b1;
                    end else begin
                        instr_valid <= 1'b1;
                        pc          <= pc + 1'b1;
                        imem_wait   <= 1'b1;
                        fp_wait     <= is_fp_slow(fetched_instr);
                    end
                end
            end
        end
    end

endmodule
