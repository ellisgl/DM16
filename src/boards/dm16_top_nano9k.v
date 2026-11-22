`timescale 1ns/1ns
// DM16 on the Sipeed Tang Nano 9K (Gowin GW1NR-9): 8 PEs of 16 bits, integer
// only (no bf16 FPU: FPU instructions are no-ops), 4 clusters x 2, 2-D
// hypercube, 4 x 2 NEWS torus, at the board's 27 MHz crystal. Keep the
// geometry in sync with the "nano9k" profile in dm16_host.py.
module dm16_top_nano9k (
    input  wire       clk_27m,       // 27 MHz crystal (pin 52)
    input  wire       uart_rx,       // from the BL702 USB-UART (pin 18)
    output wire       uart_tx,       // to the BL702 USB-UART (pin 17)
    output wire [5:0] led_n          // active low (pins 10, 11, 13-16)
);

    wire clk;
    BUFG u_clk_bufg (.I(clk_27m), .O(clk));

    // Power-on reset: held for 15 clocks after configuration (flip-flops
    // start at their initial values on Gowin parts). There is no reset
    // button: S1 is on pin 4, which is the dual-purpose JTAGSEL_N pin and
    // doesn't work as an ordinary input here. To reset, reload the
    // bitstream or power-cycle the board.
    reg [3:0] por = 4'd0;
    always @(posedge clk)
        if (por != 4'hF) begin
            por <= por + 4'd1;
        end

    wire       rst_n = (por == 4'hF);
    wire       running;
    wire       all_active;
    wire [3:0] quarter_active;

    dm16_core #(
        .DIMS    (2),
        .LANES   (2),
        .COLS    (4),
        .CLK_HZ  (27_000_000),
        .BAUD    (1_000_000),     // boots at 1 Mbaud: the 9K's BL702 bridge doesn't
                                  // handle the host changing rate at run time
        .W       (16),
        .NREGS   (8),
        .RF_BRAM (1'b0),
        .MEM_AW  (10),
        .HAS_FPU (1'b0)
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .uart_rx        (uart_rx),
        .uart_tx        (uart_tx),
        .running        (running),
        .quarter_active (quarter_active),
        .all_active     (all_active)
    );

    assign led_n = ~{quarter_active, all_active, running};

endmodule
