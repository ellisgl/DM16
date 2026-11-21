// Tang Console 138K (GW5AST-138) system clock:
//   50 MHz in -> VCO 1200 MHz (x24) -> CLKOUT0 = 1200 / ODIV MHz
// PLL_INIT (pll_init.v, from Gowin's generated IP) calibrates the charge
// pump / loop filter at start-up and resets the PLL until it locks.
module dm16_pll #(
    parameter integer ODIV = 16          // 1200 / 16 = 75 MHz
)(
    input  wire clkin,                   // 50 MHz
    output wire clkout,
    output wire lock
);

    wire       pll_lock, pll_rst;
    wire [5:0] icpsel;
    wire [2:0] lpfres;
    wire       gnd = 1'b0;
    wire       vcc = 1'b1;

    PLL #(
        .FCLKIN        ("50"),
        .IDIV_SEL      (1),
        .FBDIV_SEL     (1),
        .MDIV_SEL      (24),
        .MDIV_FRAC_SEL (0),
        .ODIV0_SEL     (ODIV),
        .ODIV0_FRAC_SEL(0),
        .ODIV1_SEL(8), .ODIV2_SEL(8), .ODIV3_SEL(8), .ODIV4_SEL(8), .ODIV5_SEL(8), .ODIV6_SEL(8),
        .CLKOUT0_EN("TRUE"),  .CLKOUT1_EN("FALSE"), .CLKOUT2_EN("FALSE"), .CLKOUT3_EN("FALSE"),
        .CLKOUT4_EN("FALSE"), .CLKOUT5_EN("FALSE"), .CLKOUT6_EN("FALSE"),
        .CLKFB_SEL     ("INTERNAL"),
        .CLKOUT0_DT_DIR(1'b1), .CLKOUT1_DT_DIR(1'b1), .CLKOUT2_DT_DIR(1'b1), .CLKOUT3_DT_DIR(1'b1),
        .CLKOUT0_DT_STEP(0), .CLKOUT1_DT_STEP(0), .CLKOUT2_DT_STEP(0), .CLKOUT3_DT_STEP(0),
        .CLK0_IN_SEL(1'b0), .CLK0_OUT_SEL(1'b0), .CLK1_IN_SEL(1'b0), .CLK1_OUT_SEL(1'b0),
        .CLK2_IN_SEL(1'b0), .CLK2_OUT_SEL(1'b0), .CLK3_IN_SEL(1'b0), .CLK3_OUT_SEL(1'b0),
        .CLK4_IN_SEL(2'b00), .CLK4_OUT_SEL(1'b0), .CLK5_IN_SEL(1'b0), .CLK5_OUT_SEL(1'b0),
        .CLK6_IN_SEL(1'b0), .CLK6_OUT_SEL(1'b0),
        .DYN_DPA_EN    ("FALSE"),
        .CLKOUT0_PE_COARSE(0), .CLKOUT0_PE_FINE(0), .CLKOUT1_PE_COARSE(0), .CLKOUT1_PE_FINE(0),
        .CLKOUT2_PE_COARSE(0), .CLKOUT2_PE_FINE(0), .CLKOUT3_PE_COARSE(0), .CLKOUT3_PE_FINE(0),
        .CLKOUT4_PE_COARSE(0), .CLKOUT4_PE_FINE(0), .CLKOUT5_PE_COARSE(0), .CLKOUT5_PE_FINE(0),
        .CLKOUT6_PE_COARSE(0), .CLKOUT6_PE_FINE(0),
        .DYN_PE0_SEL("FALSE"), .DYN_PE1_SEL("FALSE"), .DYN_PE2_SEL("FALSE"), .DYN_PE3_SEL("FALSE"),
        .DYN_PE4_SEL("FALSE"), .DYN_PE5_SEL("FALSE"), .DYN_PE6_SEL("FALSE"),
        .DE0_EN("FALSE"), .DE1_EN("FALSE"), .DE2_EN("FALSE"), .DE3_EN("FALSE"),
        .DE4_EN("FALSE"), .DE5_EN("FALSE"), .DE6_EN("FALSE"),
        .RESET_I_EN    ("FALSE"),
        .RESET_O_EN    ("FALSE"),
        .ICP_SEL       (6'bXXXXXX),
        .LPF_RES       (3'bXXX),
        .LPF_CAP       (2'b00),
        .SSC_EN        ("FALSE"),
        .DYN_IDIV_SEL("FALSE"), .DYN_FBDIV_SEL("FALSE"), .DYN_MDIV_SEL("FALSE"),
        .DYN_ODIV0_SEL("FALSE"), .DYN_ODIV1_SEL("FALSE"), .DYN_ODIV2_SEL("FALSE"), .DYN_ODIV3_SEL("FALSE"),
        .DYN_ODIV4_SEL("FALSE"), .DYN_ODIV5_SEL("FALSE"), .DYN_ODIV6_SEL("FALSE"),
        .DYN_DT0_SEL("FALSE"), .DYN_DT1_SEL("FALSE"), .DYN_DT2_SEL("FALSE"), .DYN_DT3_SEL("FALSE"),
        .DYN_ICP_SEL   ("TRUE"),
        .DYN_LPF_SEL   ("TRUE")
    ) u_pll (
        .LOCK     (pll_lock),
        .CLKOUT0  (clkout),
        .CLKOUT1  (), .CLKOUT2  (), .CLKOUT3  (), .CLKOUT4  (), .CLKOUT5  (), .CLKOUT6  (),
        .CLKFBOUT (),
        .CLKIN    (clkin),
        .CLKFB    (gnd),
        .RESET    (pll_rst),
        .PLLPWD   (gnd),
        .RESET_I  (gnd),
        .RESET_O  (gnd),
        .FBDSEL   (6'd0), .IDSEL (6'd0), .MDSEL (7'd0), .MDSEL_FRAC (3'd0),
        .ODSEL0   (7'd0), .ODSEL0_FRAC (3'd0),
        .ODSEL1   (7'd0), .ODSEL2 (7'd0), .ODSEL3 (7'd0), .ODSEL4 (7'd0), .ODSEL5 (7'd0), .ODSEL6 (7'd0),
        .DT0 (4'd0), .DT1 (4'd0), .DT2 (4'd0), .DT3 (4'd0),
        .ICPSEL   (icpsel),
        .LPFRES   (lpfres),
        .LPFCAP   (2'b00),
        .PSSEL    (3'd0),
        .PSDIR    (gnd),
        .PSPULSE  (gnd),
        .ENCLK0 (vcc), .ENCLK1 (vcc), .ENCLK2 (vcc), .ENCLK3 (vcc),
        .ENCLK4 (vcc), .ENCLK5 (vcc), .ENCLK6 (vcc),
        .SSCPOL   (gnd),
        .SSCON    (gnd),
        .SSCMDSEL (7'd0),
        .SSCMDSEL_FRAC (3'd0)
    );

    PLL_INIT #(
        .CLK_PERIOD (20),      // 50 MHz input
        .MULTI_FAC  (24)
    ) u_pll_init (
        .CLKIN   (clkin),
        .I_RST   (1'b0),
        .O_RST   (pll_rst),
        .PLLLOCK (pll_lock),
        .O_LOCK  (lock),
        .ICPSEL  (icpsel),
        .LPFRES  (lpfres)
    );

endmodule
