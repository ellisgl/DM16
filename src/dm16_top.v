`timescale 1ns/1ns
// DM16 on the Tang Console 138K (GW5AST-138):
// 64 PEs (16 clusters x 4, 4-D hypercube, 8x8 NEWS torus).
// Keep the geometry in sync with dm16_host.py.
// HAS_HDMI adds a 320x240 RGB332 frame buffer shown on the HDMI port at
// 640x480 60 Hz (dm16_hdmi.v), written by the FBX/FBY/FBW ops.
module dm16_top #(
    parameter HAS_HDMI = 1'b1
)(
    input  wire       clk_in,        // 50 MHz (V22)
    input  wire       key_reset_n,   // active low (AA13)
    input  wire       uart_rx,       // from BL616 USB-UART
    output wire       uart_tx,       // to BL616 USB-UART
    output wire [1:0] leds_n,        // active low

    // HDMI (DVI) output
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_d_p,
    output wire [2:0] tmds_d_n
);

    // System clock: 1200 MHz VCO / ODIV  (CLK_HZ must match for the UART)
    localparam integer ODIV   = 24;   // 50 MHz
    localparam integer CLK_HZ = 1_200_000_000 / ODIV;

    wire clk, pll_lock;
    dm16_pll #(.ODIV(ODIV)) u_pll (
        .clkin  (clk_in),
        .clkout (clk),
        .clkout1(),
        .lock   (pll_lock)
    );

    wire        fb_we;
    wire [16:0] fb_waddr;
    wire [7:0]  fb_wdata;

    wire       running, all_active;
    wire [3:0] quarter_active;

    dm16_core #(
        .DIMS    (4),
        .LANES   (4),
        .COLS    (8),
        .CLK_HZ  (CLK_HZ),
        .W       (16),        // PE width; keep in sync with WORD_BITS in dm16_host.py
        .NREGS   (8),         // registers r0..r7; keep in sync with NUM_REGS
        .RF_BRAM (1'b0),      // registers in flip-flops
        .MEM_AW  (10),        // 1024 words per PE; keep in sync with MEM_WORDS
        .HAS_FPU (1'b1),      // bf16 FPU in every PE
        .HAS_HDMI(HAS_HDMI)   // FBX/FBY/FBW frame-buffer ops
    ) u_core (
        .clk            (clk),
        .rst_n          (key_reset_n & pll_lock),
        .uart_rx        (uart_rx),
        .uart_tx        (uart_tx),
        .running        (running),
        .quarter_active (quarter_active),
        .all_active     (all_active),
        .fb_we          (fb_we),
        .fb_waddr       (fb_waddr),
        .fb_wdata       (fb_wdata)
    );

    generate
        if (HAS_HDMI) begin : gen_hdmi
            // Video clocks: 1000 MHz VCO -> 125 MHz (serializer) and 25 MHz (pixel)
            wire clk_x5, clk_pixel, video_lock;
            dm16_pll #(.MDIV(20), .ODIV(8), .ODIV1(40), .EN1("TRUE")) u_video_pll (
                .clkin  (clk_in),
                .clkout (clk_x5),
                .clkout1(clk_pixel),
                .lock   (video_lock)
            );

            dm16_hdmi u_hdmi (
                .clk        (clk),
                .fb_we      (fb_we),
                .fb_waddr   (fb_waddr),
                .fb_wdata   (fb_wdata),
                .clk_pixel  (clk_pixel),
                .clk_x5     (clk_x5),
                .video_rst  (!video_lock),
                .tmds_clk_p (tmds_clk_p),
                .tmds_clk_n (tmds_clk_n),
                .tmds_d_p   (tmds_d_p),
                .tmds_d_n   (tmds_d_n)
            );
        end else begin : gen_no_hdmi
            ELVDS_OBUF u_off_clk (.I(1'b0), .O(tmds_clk_p),  .OB(tmds_clk_n));
            ELVDS_OBUF u_off_d0  (.I(1'b0), .O(tmds_d_p[0]), .OB(tmds_d_n[0]));
            ELVDS_OBUF u_off_d1  (.I(1'b0), .O(tmds_d_p[1]), .OB(tmds_d_n[1]));
            ELVDS_OBUF u_off_d2  (.I(1'b0), .O(tmds_d_p[2]), .OB(tmds_d_n[2]));
        end
    endgenerate

    assign leds_n[0] = ~running;      // sequencer running
    assign leds_n[1] = ~all_active;   // all PEs active

endmodule
