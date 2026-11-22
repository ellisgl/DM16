`timescale 1ns/1ns

module dm16_uart_seq #(
    parameter integer CLK_HZ  = 54_000_000,
    parameter integer BAUD    = 115_200,
    parameter integer NUM_PE  = 8,
    parameter integer W       = 16,     // PE data width (ACC dumps send W/8 bytes per PE)
    // The rest only describe the array, for the configuration readback (0x0B)
    parameter integer DIMS    = 4,
    parameter integer LANES   = 4,
    parameter integer COLS    = 8,
    parameter integer NREGS   = 8,
    parameter integer MEM_AW  = 10,
    parameter         HAS_FPU = 1'b1,
    parameter         RF_BRAM = 1'b0
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                uart_rx,
    output reg                 uart_tx,

    // Sequencer broadcast to 64-PE array
    output reg  [23:0]         current_instr,
    output reg                 instr_valid,
    output reg                 running,

    // Live status from 64-PE array
    input  wire [NUM_PE-1:0]   all_ctx,
    input  wire [NUM_PE*W-1:0] all_acc,
    input  wire                any_active   // registered global OR of all_ctx
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
                    end else begin
                        rx_state <= 0;
                    end
                end else begin
                    rx_clk_cnt <= rx_clk_cnt + 10'd1;
                end
                2: if (rx_clk_cnt == clks_per_bit - 10'd1) begin
                    rx_clk_cnt           <= 0;
                    rx_shift[rx_bit_idx] <= rx_sync2;
                    if (rx_bit_idx == 7) begin
                        rx_state <= 3;
                    end else begin 
                        rx_bit_idx <= rx_bit_idx + 3'd1;
                    end
                end else begin
                    rx_clk_cnt <= rx_clk_cnt + 10'd1;
                end
                3: if (rx_clk_cnt == clks_per_bit - 10'd1) begin
                    rx_byte  <= rx_shift;
                    rx_valid <= 1;
                    rx_state <= 0;
                end else begin
                    rx_clk_cnt <= rx_clk_cnt + 10'd1;
                end
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
                    end else begin
                        tx_clk_cnt <= tx_clk_cnt + 10'd1;
                    end
                end
                2: begin
                    uart_tx <= tx_shift[tx_bit_idx];
                    if (tx_clk_cnt == clks_per_bit - 10'd1) begin
                        tx_clk_cnt <= 0;
                        if (tx_bit_idx == 7) begin
                            tx_state <= 3;
                        end
                        else begin
                            tx_bit_idx <= tx_bit_idx + 3'd1;
                        end
                    end else begin
                        tx_clk_cnt <= tx_clk_cnt + 10'd1;
                    end
                end
                3: begin
                    uart_tx <= 1;
                    if (tx_clk_cnt == clks_per_bit - 10'd1) begin
                        tx_busy  <= 0;
                        tx_state <= 0;
                    end else begin
                        tx_clk_cnt <= tx_clk_cnt + 10'd1;
                    end
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
    //   0x05                      read the context mask (ceil(NUM_PE/8) bytes)
    //   0x06 i2 i1 i0             execute one instruction immediately
    //   0x07                      read every PE's ACC (W/8 * NUM_PE bytes)
    //   0x08 d_hi d_lo            set UART clocks per bit
    //   0x09                      read status byte {6'b0, any_active, running}
    //   0x0B                      read the configuration: 12 bytes
    //                             'D', 16 (format), W, DIMS, LANES, COLS,
    //                             log2(NREGS), MEM_AW, flags {RF_BRAM, HAS_FPU},
    //                             CLK_HZ / 1000 as 24 bits little-endian
    //   0x0A a_hi a_lo n_hi n_lo  bulk read: n words from address a of every PE
    //                             (enables all PEs, then for each word loads it
    //                             into ACC and streams the ACC dump: n * W/8 *
    //                             NUM_PE bytes; the program must be idle).
    //                             Within the first 1024 words it injects LDM a+k
    //                             and clobbers only ACC. Beyond them, where an
    //                             immediate address can't reach, it sets mar = a
    //                             (via LDI hi/lo and MAR) and injects LDX with
    //                             post-increment, so it clobbers ACC and MAR.
    localparam integer PC_W = 12;
    reg [23:0]     imem [0:(1 << PC_W) - 1];
    reg [PC_W-1:0] pc;
    reg [PC_W-1:0] end_pc;
    reg            single_step;
    reg [4:0]      cmd_state;
    reg [PC_W-1:0] wr_addr;
    reg [23:0]     wr_data;
    localparam integer ACC_BYTES = NUM_PE * W / 8;
    localparam integer CTX_BYTES = (NUM_PE + 7) / 8;
    // Zero-padded to whole bytes. The "+ 1" keeps the replication count
    // non-zero (illegal in Verilog-2001) when NUM_PE is a multiple of 8; the
    // extra top bit is dropped by the assignment.
    wire [CTX_BYTES*8-1:0] ctx_padded = {{(CTX_BYTES*8 - NUM_PE + 1){1'b0}}, all_ctx};

    // Configuration readback (0x0B)
    function integer log2i;
        input integer v;
        integer t;
        begin
            log2i = 0;
            for (t = v; t > 1; t = t >> 1) begin
                log2i = log2i + 1;
            end
        end
    endfunction

    localparam [23:0] CFG_KHZ   = CLK_HZ / 1000;
    localparam [7:0]  CFG_W     = W;
    localparam [7:0]  CFG_DIMS  = DIMS;
    localparam [7:0]  CFG_LANES = LANES;
    localparam [7:0]  CFG_COLS  = COLS;
    localparam [7:0]  CFG_RBITS = log2i(NREGS);
    localparam [7:0]  CFG_AW    = MEM_AW;
    localparam [7:0]  CFG_FLAGS = (RF_BRAM ? 8'd2 : 8'd0) | (HAS_FPU ? 8'd1 : 8'd0);
    localparam [95:0] CFG_BYTES = {
        CFG_KHZ, CFG_FLAGS, CFG_AW, CFG_RBITS, CFG_COLS, CFG_LANES, CFG_DIMS, CFG_W,
        8'd16, 8'h44 };                    // byte 0 = 'D' (sent first)
    reg [15:0] dump_idx;

    reg [23:0] inj_instr;
    reg        inj_valid;
    reg        inj_active;

    // Bulk read (0x0A): instructions it injects
    localparam [23:0] I_CTX_ALWAYS = 24'h900000;
    localparam [23:0] I_LDI_0      = 24'h100000;    // acc = 0
    localparam [23:0] I_LDI_HI     = 24'h110000;    // | byte: acc[15:8] = byte
    localparam [23:0] I_LDI_LO     = 24'h120000;    // | byte: acc[7:0]  = byte
    localparam [23:0] I_MAR_ACC    = 24'h600000;    // mar = acc
    localparam [23:0] I_LDX_INC    = 24'h400800;    // acc = mem[mar++]
    localparam [23:0] I_LDM        = 24'h200000;    // | address: acc = mem[address]
    // Phases: 0 CTX ALWAYS; 1-4 set mar (only when br_mar); 5 one word each
    reg [15:0] br_addr;
    reg [15:0] br_left;
    reg [2:0]  br_phase;
    reg        br_mar;
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
            cmd_state    <= 0;
            running      <= 0;
            single_step  <= 0;
            end_pc       <= 0;
            dump_idx     <= 0;
            tx_start     <= 0;
            tx_data_in   <= 0;
            inj_instr    <= 0;
            inj_valid    <= 0;
            clks_per_bit <= CLKS_PER_BIT;
            cmd_idle     <= 0;
            br_addr      <= 0;
            br_left      <= 0;
            br_phase     <= 0;
            br_mar       <= 0;
            br_wait      <= 0;
        end else begin
            single_step <= 0;
            tx_start    <= 0;
            inj_valid   <= 0;

            if (rx_valid || !cmd_in_args) begin
                cmd_idle <= 0;
            end else if (cmd_idle == {17{1'b1}}) begin
                cmd_state <= 4'd0;
            end else begin
                cmd_idle <= cmd_idle + 17'd1;
            end

            if (running && (pc == end_pc)) begin
                running <= 0;
            end
        
            // State 10: Stream the CTX mask (CTX_BYTES bytes, zero-padded)
            if (cmd_state == 4'd10 && !tx_busy && !tx_start) begin
                if (dump_idx < CTX_BYTES) begin
                    tx_data_in <= ctx_padded[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 16'd1;
                end else begin
                    cmd_state <= 4'd0;
                end
            end

            // State 23: Stream the configuration (12 bytes)
            if (cmd_state == 5'd23 && !tx_busy && !tx_start) begin
                if (dump_idx < 16'd12) begin
                    tx_data_in <= CFG_BYTES[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 16'd1;
                end else begin
                    cmd_state <= 4'd0;
                end
            end

            // State 15: Send the status byte
            if (cmd_state == 4'd15 && !tx_busy && !tx_start) begin
                tx_data_in <= {6'd0, any_active, running};
                tx_start   <= 1'b1;
                cmd_state  <= 4'd0;
            end

            // State 11: Stream every PE's W-bit ACC, little-endian (ACC_BYTES bytes)
            if (cmd_state == 4'd11 && !tx_busy && !tx_start) begin
                if (dump_idx < ACC_BYTES) begin
                    tx_data_in <= all_acc[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 16'd1;
                end else begin
                    cmd_state <= 4'd0;
                end
            end

            // States 20-22: bulk read. Inject one instruction (see the phases
            // above), give it 64 clocks to reach every ACC (it needs about
            // 10), and after each word's load stream the ACC dump as in
            // state 11.
            if (cmd_state == 5'd20) begin
                case (br_phase)
                    3'd0:    inj_instr <= I_CTX_ALWAYS;
                    3'd1:    inj_instr <= I_LDI_0;
                    3'd2:    inj_instr <= I_LDI_HI | {16'd0, br_addr[15:8]};
                    3'd3:    inj_instr <= I_LDI_LO | {16'd0, br_addr[7:0]};
                    3'd4:    inj_instr <= I_MAR_ACC;
                    default: inj_instr <= br_mar ? I_LDX_INC : (I_LDM | {14'd0, br_addr[9:0]});
                endcase
                inj_valid <= 1'b1;
                br_wait   <= 6'd0;
                cmd_state <= 5'd21;
            end
            if (cmd_state == 5'd21) begin
                br_wait <= br_wait + 6'd1;
                if (br_wait == 6'd63) begin
                    if (br_phase != 3'd5) begin
                        br_phase  <= (br_phase == 3'd0 && !br_mar) ? 3'd5 : br_phase + 3'd1;
                        cmd_state <= 5'd20;
                    end else begin
                        dump_idx  <= 16'd0;
                        cmd_state <= 5'd22;
                    end
                end
            end
            if (cmd_state == 5'd22 && !tx_busy && !tx_start) begin
                if (dump_idx < ACC_BYTES) begin
                    tx_data_in <= all_acc[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 16'd1;
                end else begin
                    br_addr   <= br_addr + 16'd1;
                    br_left   <= br_left - 16'd1;
                    cmd_state <= (br_left == 16'd1) ? 5'd0 : 5'd20;
                end
            end

            if (rx_valid) begin
                case (cmd_state)
                    4'd0: case (rx_byte)
                        8'h01: cmd_state      <= 4'd1; // WRITE_IMEM
                        8'h02: cmd_state      <= 4'd6; // RUN
                        8'h03: single_step    <= 1'b1; // STEP
                        8'h04: running        <= 1'b0; // HALT/RESET
                        8'h05: begin 
                            // READ_CTX
                            dump_idx <= 16'd0;
                            cmd_state <= 4'd10;
                        end 
                        8'h06: cmd_state      <= 4'd8; // INJECT_INSTR (3 bytes)
                        8'h07: begin
                            // READ_ACC_GRID
                            dump_idx  <= 16'd0;
                            cmd_state <= 4'd11;
                        end
                        8'h08: cmd_state      <= 4'd13; // SET_BAUD (2 bytes: clocks per bit, big-endian)
                        8'h09: cmd_state      <= 4'd15; // READ_STATUS
                        8'h0A: cmd_state      <= 5'd16; // BULK_READ (4 bytes)
                        8'h0B: begin
                            // READ_CONFIG
                            dump_idx  <= 16'd0;
                            cmd_state <= 5'd23;
                        end
                    endcase
                    4'd1: begin
                        wr_addr[11:8] <= rx_byte[3:0];
                        cmd_state     <= 4'd2;
                    end
                    4'd2: begin
                        wr_addr[7:0] <= rx_byte;
                        cmd_state    <= 4'd3;
                    end
                    4'd3: begin
                        wr_data[23:16] <= rx_byte;
                        cmd_state <= 4'd4;
                    end
                    4'd4: begin
                        wr_data[15:8] <= rx_byte;
                        cmd_state     <= 4'd5;
                    end
                    4'd5: begin
                        imem[wr_addr] <= (wr_data & 24'hFFFF00) | rx_byte;
                        cmd_state     <= 4'd0;
                    end
                    4'd6: begin
                        end_pc[11:8] <= rx_byte[3:0];
                        cmd_state    <= 4'd7;
                    end
                    4'd7: begin
                        end_pc[7:0] <= rx_byte;
                        running     <= 1'b1;
                        cmd_state   <= 4'd0;
                    end
                    4'd8: begin
                        wr_data[23:16] <= rx_byte;
                        cmd_state <= 4'd9;
                    end
                    4'd9: begin 
                        wr_data[15:8]  <= rx_byte;
                        cmd_state <= 4'd12;
                    end
                    4'd13: begin
                        wr_data[15:8] <= rx_byte;
                        cmd_state <= 4'd14;
                    end
                    4'd14: begin
                        // Takes effect from the next byte; values below 8 are ignored
                        if ({wr_data[9:8], rx_byte} >= 10'd8) begin
                            clks_per_bit <= {wr_data[9:8], rx_byte};
                        end
                        cmd_state <= 4'd0;
                    end
                    4'd12: begin
                        inj_instr <= (wr_data & 24'hFFFF00) | rx_byte;
                        inj_valid <= 1'b1;
                        cmd_state <= 4'd0;
                    end
                    5'd16: begin
                        br_addr[15:8] <= rx_byte;
                        cmd_state     <= 5'd17;
                    end
                    5'd17: begin
                        br_addr[7:0]  <= rx_byte;
                        cmd_state <= 5'd18;
                    end
                    5'd18: begin
                        br_left[15:8] <= rx_byte;
                        cmd_state <= 5'd19;
                    end
                    5'd19: begin
                        br_left[7:0] <= rx_byte;
                        br_phase     <= 3'd0;
                        // past the first 1024 words: read through mar
                        br_mar       <= ({1'b0, br_addr} + {1'b0, br_left[15:8], rx_byte}) > 17'd1024;
                        cmd_state    <= ({br_left[15:8], rx_byte} == 16'd0) ? 5'd0 : 5'd20;
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
                                if (lp_sp != 3'd0) begin
                                    lp_sp <= lp_sp - 3'd1;
                                end
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
                                if ((ctl_op == C_BANY) ? any_active : !any_active) begin
                                    pc <= ctl_tgt;
                                end else begin
                                    pc <= pc + 1'b1;
                                end
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
