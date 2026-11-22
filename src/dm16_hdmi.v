`timescale 1ns/1ns
// DM16 HDMI (DVI) output: a frame buffer shown at 640x480, 60 Hz, each
// frame-buffer pixel as a SCALE x SCALE block: 320x240 (SCALE 2) or
// 160x120 (SCALE 4, the Nano 9K).
//   PIX_BITS = 8: one byte per pixel, RGB332 (rrrgggbb)        - 38 BSRAMs
//   PIX_BITS = 4: 16 colours, the CGA palette (index = low 4 bits) - 19 BSRAMs
//
//   clk / fb_we / fb_waddr / fb_wdata   write port, system clock (dm16_fb_writer)
//   clk_pixel (25 MHz), clk_x5 (125 MHz) from the video PLL
//
// Plain DVI: no audio or InfoFrames, which every HDMI sink accepts.
// 640x480 timing: 800 x 525 total, 25 MHz -> 59.5 Hz (the standard asks for
// 25.175 MHz; capture cards and monitors lock to 25.0 fine).
module dm16_hdmi #(
    parameter integer FB_W  = 320,
    parameter integer FB_H  = 240,
    parameter integer FB_AW = 17,
    parameter integer PIX_BITS = 8,
    parameter integer SCALE = 2         // 640 / FB_W: 2 or 4
)(
    // Frame-buffer write port (system clock)
    input  wire             clk,
    input  wire             fb_we,
    input  wire [FB_AW-1:0] fb_waddr,
    input  wire [7:0]       fb_wdata,

    // Video clocks
    input  wire             clk_pixel,
    input  wire             clk_x5,
    input  wire             video_rst,    // high while the video PLL is unlocked

    output wire             tmds_clk_p,
    output wire             tmds_clk_n,
    output wire [2:0]       tmds_d_p,
    output wire [2:0]       tmds_d_n
);

    // -------------------------------------------------------------------------
    // Frame buffer: FB_W * FB_H bytes of block RAM, written from the system
    // clock and read from the pixel clock. The write controls are registered
    // here, so the BSRAM write enable always comes from a flop.
    // -------------------------------------------------------------------------
    (* syn_ramstyle = "block_ram" *)
    reg [PIX_BITS-1:0] fb [0:FB_W*FB_H-1];

    reg                we_q;
    reg [FB_AW-1:0]    wa_q;
    reg [PIX_BITS-1:0] wd_q;
    always @(posedge clk) begin
        we_q <= fb_we;
        wa_q <= fb_waddr;
        wd_q <= fb_wdata[PIX_BITS-1:0];
        if (we_q) begin
            fb[wa_q] <= wd_q;
        end
    end

    // -------------------------------------------------------------------------
    // 640x480 timing
    // -------------------------------------------------------------------------
    localparam integer H_ACTIVE = 640, H_FP = 16, H_SYNC = 96, H_TOTAL = 800;
    localparam integer V_ACTIVE = 480, V_FP = 10, V_SYNC = 2,  V_TOTAL = 525;

    reg [9:0] hc, vc;
    always @(posedge clk_pixel) begin
        if (video_rst) begin
            hc <= 0;
            vc <= 0;
        end else if (hc == H_TOTAL - 1) begin
            hc <= 0;
            vc <= (vc == V_TOTAL - 1) ? 10'd0 : vc + 10'd1;
        end else begin
            hc <= hc + 10'd1;
        end
    end

    // Stage 1: frame-buffer address and the video signals for this clock
    wire [8:0] fx = (SCALE == 4) ? {1'b0, hc[9:2]} : hc[9:1];
    wire [8:0] fy = (SCALE == 4) ? {1'b0, vc[9:2]} : vc[9:1];
    reg  [FB_AW-1:0] ra;
    reg  [2:0] ctl1, ctl2;                    // {de, vsync, hsync}
    always @(posedge clk_pixel) begin
        ra   <= fy * FB_W + fx;               // constant multiply: shifts and adds
        ctl1 <= {(hc < H_ACTIVE) && (vc < V_ACTIVE),
                 !((vc >= V_ACTIVE + V_FP) && (vc < V_ACTIVE + V_FP + V_SYNC)),   // negative
                 !((hc >= H_ACTIVE + H_FP) && (hc < H_ACTIVE + H_FP + H_SYNC))};  // negative
    end

    // Stage 2: the block RAM read
    reg [PIX_BITS-1:0] pix;
    always @(posedge clk_pixel) begin
        pix  <= fb[ra];
        ctl2 <= ctl1;
    end

    // Pixel -> 8 bits per channel
    reg [23:0] cga;
    always @(*) begin
        case (pix[3:0])
            4'd0:  cga = 24'h000000;  4'd1:  cga = 24'h0000AA;   // black, blue
            4'd2:  cga = 24'h00AA00;  4'd3:  cga = 24'h00AAAA;   // green, cyan
            4'd4:  cga = 24'hAA0000;  4'd5:  cga = 24'hAA00AA;   // red, magenta
            4'd6:  cga = 24'hAA5500;  4'd7:  cga = 24'hAAAAAA;   // brown, light grey
            4'd8:  cga = 24'h555555;  4'd9:  cga = 24'h5555FF;   // dark grey, light blue
            4'd10: cga = 24'h55FF55;  4'd11: cga = 24'h55FFFF;   // light green, light cyan
            4'd12: cga = 24'hFF5555;  4'd13: cga = 24'hFF55FF;   // light red, light magenta
            4'd14: cga = 24'hFFFF55;  default: cga = 24'hFFFFFF; // yellow, white
        endcase
    end
    wire [7:0] pix8 = pix;    // zero-extended when PIX_BITS = 4 (unused then)
    wire [7:0] r = (PIX_BITS == 4) ? cga[23:16] : {pix8[7:5], pix8[7:5], pix8[7:6]};
    wire [7:0] g = (PIX_BITS == 4) ? cga[15:8]  : {pix8[4:2], pix8[4:2], pix8[4:3]};
    wire [7:0] b = (PIX_BITS == 4) ? cga[7:0]   : {pix8[1:0], pix8[1:0], pix8[1:0], pix8[1:0]};

    // Stage 3: TMDS encoding (channel 0 carries the syncs)
    wire [9:0] sym0, sym1, sym2;
    dm16_tmds_enc u_enc0 (.clk(clk_pixel), .rst(video_rst), .de(ctl2[2]), .d(b),
                          .c({ctl2[1], ctl2[0]}), .q(sym0));
    dm16_tmds_enc u_enc1 (.clk(clk_pixel), .rst(video_rst), .de(ctl2[2]), .d(g),
                          .c(2'b00), .q(sym1));
    dm16_tmds_enc u_enc2 (.clk(clk_pixel), .rst(video_rst), .de(ctl2[2]), .d(r),
                          .c(2'b00), .q(sym2));

    // -------------------------------------------------------------------------
    // 10:1 serializers (bit 0 first) and differential outputs
    // -------------------------------------------------------------------------
    wire [9:0] sym [0:3];
    assign sym[0] = sym0;
    assign sym[1] = sym1;
    assign sym[2] = sym2;
    assign sym[3] = 10'b0000011111;           // the TMDS clock: one cycle per symbol

    wire [3:0] ser;
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_ser
            OSER10 u_ser (
                .Q    (ser[i]),
                .D0   (sym[i][0]), .D1(sym[i][1]), .D2(sym[i][2]), .D3(sym[i][3]),
                .D4   (sym[i][4]), .D5(sym[i][5]), .D6(sym[i][6]), .D7(sym[i][7]),
                .D8   (sym[i][8]), .D9(sym[i][9]),
                .PCLK (clk_pixel),
                .FCLK (clk_x5),
                .RESET(video_rst)
            );
        end
    endgenerate

    ELVDS_OBUF u_obuf_d0  (.I(ser[0]), .O(tmds_d_p[0]), .OB(tmds_d_n[0]));
    ELVDS_OBUF u_obuf_d1  (.I(ser[1]), .O(tmds_d_p[1]), .OB(tmds_d_n[1]));
    ELVDS_OBUF u_obuf_d2  (.I(ser[2]), .O(tmds_d_p[2]), .OB(tmds_d_n[2]));
    ELVDS_OBUF u_obuf_clk (.I(ser[3]), .O(tmds_clk_p),  .OB(tmds_clk_n));

endmodule

// DVI 1.0 TMDS 8b/10b encoder, one registered symbol per clock.
module dm16_tmds_enc (
    input  wire       clk,
    input  wire       rst,
    input  wire       de,
    input  wire [7:0] d,
    input  wire [1:0] c,       // {c1, c0}, sent during blanking
    output reg  [9:0] q
);
    function [3:0] ones8;
        input [7:0] v;
        integer k;
        begin
            ones8 = 0;
            for (k = 0; k < 8; k = k + 1) ones8 = ones8 + v[k];
        end
    endfunction

    // Transition minimisation
    wire [3:0] n1d = ones8(d);
    wire       use_xnor = (n1d > 4'd4) || ((n1d == 4'd4) && !d[0]);
    wire [8:0] qm;
    assign qm[0] = d[0];
    genvar i;
    generate
        for (i = 1; i < 8; i = i + 1) begin : gen_qm
            assign qm[i] = use_xnor ? ~(qm[i-1] ^ d[i]) : (qm[i-1] ^ d[i]);
        end
    endgenerate
    assign qm[8] = !use_xnor;

    // DC balance; disparity counts (ones - zeros) of the symbols sent
    wire [3:0]        n1q  = ones8(qm[7:0]);
    wire signed [4:0] diff = $signed({1'b0, n1q, 1'b0}) - 5'sd8;   // n1q - n0q (even)
    reg  signed [4:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            cnt <= 0;
            q   <= 10'b1101010100;
        end else if (!de) begin
            cnt <= 0;
            case (c)
                2'b00: q <= 10'b1101010100;
                2'b01: q <= 10'b0010101011;
                2'b10: q <= 10'b0101010100;
                2'b11: q <= 10'b1010101011;
            endcase
        end else if ((cnt == 0) || (diff == 0)) begin
            q   <= {~qm[8], qm[8], qm[8] ? qm[7:0] : ~qm[7:0]};
            cnt <= qm[8] ? (cnt + diff) : (cnt - diff);
        end else if (((cnt > 0) && (diff > 0)) || ((cnt < 0) && (diff < 0))) begin
            q   <= {1'b1, qm[8], ~qm[7:0]};
            cnt <= cnt + (qm[8] ? 5'sd2 : 5'sd0) - diff;
        end else begin
            q   <= {1'b0, qm[8], qm[7:0]};
            cnt <= cnt - (qm[8] ? 5'sd0 : 5'sd2) + diff;
        end
    end
endmodule
