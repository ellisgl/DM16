`timescale 1ns/1ns
// DM16 on the Sipeed Tang Nano 20K (Gowin GW2AR-18): 8 PEs of 16 bits with
// bf16 FPUs (4 clusters x 2, 2-D hypercube, 4 x 2 NEWS torus), at the
// board's 27 MHz crystal. Keep the geometry in sync with the "nano20k"
// profile in dm16_host.py.
module dm16_top_nano20k (
    input  wire       clk_27m,       // 27 MHz crystal (pin 4)
    input  wire       rst_btn,       // S1, active high (pin 88)
    input  wire       uart_rx,       // from the BL616 USB-UART (pin 70)
    output wire       uart_tx,       // to the BL616 USB-UART (pin 69)
    output wire [5:0] led_n          // active low (pins 15-20)
);

    // The pad clock goes onto the global clock tree (without the BUFG the
    // place-and-route used fabric routing for it, with unknown skew)
    wire clk;
    BUFG u_clk_bufg (.I(clk_27m), .O(clk));

    wire       running;
    wire       all_active;
    wire [3:0] quarter_active;

    dm16_core #(
        .DIMS    (2),
        .LANES   (2),
        .COLS    (4),
        .CLK_HZ  (27_000_000),
        .W       (16),
        .NREGS   (8),
        .RF_BRAM (1'b0),
        .MEM_AW  (10),
        .HAS_FPU (1'b1)
    ) u_core (
        .clk            (clk),
        .rst_n          (~rst_btn),
        .uart_rx        (uart_rx),
        .uart_tx        (uart_tx),
        .running        (running),
        .quarter_active (quarter_active),
        .all_active     (all_active)
    );

    assign led_n = ~{quarter_active, all_active, running};

endmodule
