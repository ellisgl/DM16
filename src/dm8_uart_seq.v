module dm8_uart_seq #(
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
    input  wire [NUM_PE*16-1:0] all_acc
);

    // =========================================================================
    // 1. UART RX & TX (CLKS_PER_BIT derived from CLK_HZ / BAUD)
    // =========================================================================
    // e.g. 54 MHz / 115200 = 468.75 -> 469 clocks per bit (0.05% error)
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;

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
                1: if (rx_clk_cnt == (CLKS_PER_BIT / 2)) begin
                    if (!rx_sync2) begin
                        rx_clk_cnt <= 0;
                        rx_bit_idx <= 0;
                        rx_state   <= 2;
                    end else rx_state <= 0;
                end else rx_clk_cnt <= rx_clk_cnt + 10'd1;
                2: if (rx_clk_cnt == CLKS_PER_BIT - 1) begin
                    rx_clk_cnt           <= 0;
                    rx_shift[rx_bit_idx] <= rx_sync2;
                    if (rx_bit_idx == 7) rx_state <= 3;
                    else rx_bit_idx <= rx_bit_idx + 3'd1;
                end else rx_clk_cnt <= rx_clk_cnt + 10'd1;
                3: if (rx_clk_cnt == CLKS_PER_BIT - 1) begin
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
                    if (tx_clk_cnt == CLKS_PER_BIT - 1) begin
                        tx_clk_cnt <= 0;
                        tx_state   <= 2;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
                2: begin
                    uart_tx <= tx_shift[tx_bit_idx];
                    if (tx_clk_cnt == CLKS_PER_BIT - 1) begin
                        tx_clk_cnt <= 0;
                        if (tx_bit_idx == 7) tx_state <= 3;
                        else tx_bit_idx <= tx_bit_idx + 3'd1;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
                3: begin
                    uart_tx <= 1;
                    if (tx_clk_cnt == CLKS_PER_BIT - 1) begin
                        tx_busy  <= 0;
                        tx_state <= 0;
                    end else tx_clk_cnt <= tx_clk_cnt + 10'd1;
                end
            endcase
        end
    end

    // =========================================================================
    // 2. Host Loader FSM & 512 x 24-bit Instruction Memory
    // =========================================================================
    reg [23:0] imem [0:511];
    reg [8:0]  pc;
    reg [8:0]  end_pc;
    reg        single_step;
    reg [3:0]  cmd_state;
    reg [8:0]  wr_addr;
    reg [23:0] wr_data;
    reg [7:0]  dump_idx;

    reg [23:0] inj_instr;
    reg        inj_valid;
    reg        inj_active;

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
        end else begin
            single_step <= 0;
            tx_start    <= 0;
            inj_valid   <= 0;

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

            // State 11: Stream every PE's 16-bit ACC, little-endian (2*NUM_PE bytes)
            if (cmd_state == 4'd11 && !tx_busy && !tx_start) begin
                if (dump_idx < 2 * NUM_PE) begin
                    tx_data_in <= all_acc[dump_idx*8 +: 8];
                    tx_start   <= 1'b1;
                    dump_idx   <= dump_idx + 8'd1;
                end else cmd_state <= 4'd0;
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
                    endcase
                    4'd1: begin wr_addr[8:8]   <= rx_byte[0:0]; cmd_state <= 4'd2; end
                    4'd2: begin wr_addr[7:0]   <= rx_byte;      cmd_state <= 4'd3; end
                    4'd3: begin wr_data[23:16] <= rx_byte;      cmd_state <= 4'd4; end
                    4'd4: begin wr_data[15:8]  <= rx_byte;      cmd_state <= 4'd5; end
                    4'd5: begin
                        imem[wr_addr] <= (wr_data & 24'hFFFF00) | rx_byte;
                        cmd_state     <= 4'd0;
                    end
                    4'd6: begin end_pc[8:8] <= rx_byte[0:0]; cmd_state <= 4'd7; end
                    4'd7: begin
                        end_pc[7:0] <= rx_byte;
                        running     <= 1'b1;
                        cmd_state   <= 4'd0;
                    end
                    4'd8: begin wr_data[23:16] <= rx_byte; cmd_state <= 4'd9; end
                    4'd9: begin wr_data[15:8]  <= rx_byte; cmd_state <= 4'd12; end
                    4'd12: begin
                        inj_instr <= (wr_data & 24'hFFFF00) | rx_byte;
                        inj_valid <= 1'b1;
                        cmd_state <= 4'd0;
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // 3. Global SIMD Sequencer (with automatic 2-cycle BRAM Read Wait State)
    // =========================================================================
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
        end else begin
            instr_valid <= 0;

            //if (rx_valid && rx_byte == 8'h04) begin
            if (rx_valid && (cmd_state == 4'd0) && (rx_byte == 8'h04)) begin
                pc         <= 0;
                mem_wait   <= 0;
                mem_wait2  <= 0;
                fp_wait    <= 0;
                inj_active <= 0;
                imem_wait  <= 1'b0;
            end else if (mem_wait2) begin
                mem_wait2 <= 1'b0;
                mem_wait  <= 1'b1;
            end else if (mem_wait) begin
                instr_valid <= 1'b1;
                mem_wait    <= 1'b0;
                fp_wait     <= is_fp_slow(current_instr);
                if (!inj_active) begin
                    pc <= pc + 9'd1;
                    imem_wait <= 1'b1;
                end
                inj_active  <= 1'b0;
            end else if (imem_wait) begin
                // Wait 1 clock for synchronous BSRAM fetched_instr[pc] to update
                imem_wait <= 1'b0;
            end else if (fp_wait) begin
                fp_wait <= 1'b0;
            end else if (inj_valid) begin
                current_instr <= inj_instr;
                if (is_inj_read) begin
                    mem_wait2  <= 1'b1;
                    inj_active <= 1'b1;
                end else begin
                    instr_valid <= 1'b1;
                    fp_wait     <= is_fp_slow(inj_instr);
                end
            end else if ((running && pc < end_pc) || single_step) begin
                current_instr <= fetched_instr;
                if (is_mem_read) begin
                    mem_wait2 <= 1'b1;
                end else begin
                    instr_valid <= 1'b1;
                    pc          <= pc + 9'd1;
                    imem_wait   <= 1'b1;
                    fp_wait     <= is_fp_slow(fetched_instr);
                end
            end
        end
    end

endmodule