`timescale 1ns/1ns
// DM16 on the Tang Console 138K (GW5AST-138):
// 64 PEs (16 clusters x 4, 4-D hypercube, 8x8 NEWS torus).
// Keep the geometry in sync with dm16_host.py.
module dm16_top (
    input  wire       clk_in,        // 50 MHz (V22)
    input  wire       key_reset_n,   // active low (AA13)
    input  wire       uart_rx,       // from BL616 USB-UART
    output wire       uart_tx,       // to BL616 USB-UART
    output wire [1:0] leds_n         // active low
);

    // System clock: 1200 MHz VCO / ODIV  (CLK_HZ must match for the UART)
    localparam integer ODIV   = 24;   // 50 MHz
    localparam integer CLK_HZ = 1_200_000_000 / ODIV;

    wire clk, pll_lock;
    dm16_pll #(.ODIV(ODIV)) u_pll (
        .clkin  (clk_in),
        .clkout (clk),
        .lock   (pll_lock)
    );

    wire       running, all_active;
    wire [3:0] quarter_active;

    dm16_core #(
        .DIMS   (4),
        .LANES  (4),
        .COLS   (8),
        .CLK_HZ (CLK_HZ),
        .W       (16),        // PE width; keep in sync with WORD_BITS in dm16_host.py
        .NREGS   (8),         // registers r0..r7; keep in sync with NUM_REGS
        .RF_BRAM (1'b0),      // registers in flip-flops
        .MEM_AW  (10),        // 1024 words per PE; keep in sync with MEM_WORDS
        .HAS_FPU (1'b1)       // bf16 FPU in every PE
    ) u_core (
        .clk            (clk),
        .rst_n          (key_reset_n & pll_lock),
        .uart_rx        (uart_rx),
        .uart_tx        (uart_tx),
        .running        (running),
        .quarter_active (quarter_active),
        .all_active     (all_active)
    );

    assign leds_n[0] = ~running;      // sequencer running
    assign leds_n[1] = ~all_active;   // all PEs active

endmodule
