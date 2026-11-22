`timescale 1ns/1ns
// DM16 on the Sipeed Tang Nano 9K (Gowin GW1NR-9): 8 PEs of 16 bits, integer
// only (no bf16 FPU: FPU instructions are no-ops), 4 clusters x 2, 2-D
// hypercube, 4 x 2 NEWS torus, at the board's 27 MHz crystal. Keep the
// geometry in sync with the "nano9k" profile in dm16_host.py.
// HAS_HDMI adds a 160x120 16-colour frame buffer (CGA palette) on the HDMI
// port, shown at 640x480 60 Hz with each pixel as a 4x4 block, written by
// FBX/FBY/FBW (src/dm16_hdmi.v): 5 of the 12 block RAMs left free.
module dm16_top_nano9k #(
    parameter HAS_HDMI = 1'b1
)(
    input  wire       clk_27m,       // 27 MHz crystal (pin 52)
    input  wire       uart_rx,       // from the BL702 USB-UART (pin 18)
    output wire       uart_tx,       // to the BL702 USB-UART (pin 17)
    output wire [5:0] led_n,         // active low (pins 10, 11, 13-16)

    // HDMI (DVI) output
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_d_p,
    output wire [2:0] tmds_d_n
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
    wire        fb_we;
    wire [16:0] fb_waddr;
    wire [7:0]  fb_wdata;

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
        .HAS_FPU (1'b0),
        .HAS_HDMI(HAS_HDMI),
        .FB_W    (160),
        .FB_H    (120),
        .FB_BITS (4)
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
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
            // 27 MHz * 14 / 3 = 126 MHz serializer clock (VCO 504 MHz, as in
            // Sipeed's 9K HDMI example), / 5 = 25.2 MHz pixel clock
            wire clk_x5, clk_pixel, video_lock;
            rPLL #(
                .FCLKIN        ("27"),
                .DEVICE        ("GW1NR-9C"),
                .DYN_IDIV_SEL  ("false"),
                .IDIV_SEL      (2),
                .DYN_FBDIV_SEL ("false"),
                .FBDIV_SEL     (13),
                .DYN_ODIV_SEL  ("false"),
                .ODIV_SEL      (4),
                .PSDA_SEL      ("0000"),
                .DYN_DA_EN     ("true"),
                .DUTYDA_SEL    ("1000"),
                .CLKOUT_FT_DIR (1'b1),
                .CLKOUTP_FT_DIR(1'b1),
                .CLKOUT_DLY_STEP (0),
                .CLKOUTP_DLY_STEP(0),
                .CLKFB_SEL     ("internal"),
                .CLKOUT_BYPASS ("false"),
                .CLKOUTP_BYPASS("false"),
                .CLKOUTD_BYPASS("false"),
                .DYN_SDIV_SEL  (2),
                .CLKOUTD_SRC   ("CLKOUT"),
                .CLKOUTD3_SRC  ("CLKOUT")
            ) u_video_pll (
                .CLKOUT  (clk_x5),
                .LOCK    (video_lock),
                .CLKOUTP (),
                .CLKOUTD (),
                .CLKOUTD3(),
                .RESET   (1'b0),
                .RESET_P (1'b0),
                .CLKIN   (clk_27m),
                .CLKFB   (1'b0),
                .FBDSEL  (6'd0),
                .IDSEL   (6'd0),
                .ODSEL   (6'd0),
                .PSDA    (4'd0),
                .DUTYDA  (4'd0),
                .FDLY    (4'd0)
            );

            CLKDIV #(.DIV_MODE("5"), .GSREN("false")) u_pixdiv (
                .RESETN (video_lock),
                .HCLKIN (clk_x5),
                .CLKOUT (clk_pixel),
                .CALIB  (1'b0)
            );

            dm16_hdmi #(.FB_W(160), .FB_H(120), .FB_AW(17), .PIX_BITS(4), .SCALE(4)) u_hdmi (
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

    assign led_n = ~{quarter_active, all_active, running};

endmodule
