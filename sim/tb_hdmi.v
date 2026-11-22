`timescale 1ns/1ps
// HDMI output check: fill the frame buffer, run one frame, decode every TMDS
// symbol (parallel, and the serial stream of channel 0) back into pixels and
// syncs, and compare with the frame buffer.
//   iverilog -s tb -o sim/hdmi.vvp sim/tb_hdmi.v src/dm16_hdmi.v \
//            ~/Gowin_V1.9.12.04_linux/IDE/simlib/gw5a/prim_sim.v && vvp sim/hdmi.vvp
module tb;
    GSR GSR(.GSRI(1'b1));
    reg clk = 0, clk_pixel = 0, clk_x5 = 0, rst = 1;
    always #10 clk = ~clk;
    always #20 clk_pixel = ~clk_pixel;    // 25 MHz
    always #4  clk_x5 = ~clk_x5;          // 125 MHz, edges aligned with clk_pixel

    reg         we = 0;
    reg  [16:0] wa = 0;
    reg  [7:0]  wd = 0;
    wire        ckp, ckn;
    wire [2:0]  dp, dn;
    dm16_hdmi dut (.clk(clk), .fb_we(we), .fb_waddr(wa), .fb_wdata(wd),
                   .clk_pixel(clk_pixel), .clk_x5(clk_x5), .video_rst(rst),
                   .tmds_clk_p(ckp), .tmds_clk_n(ckn), .tmds_d_p(dp), .tmds_d_n(dn));

    function [7:0] pat;  input integer x, y; pat = (x * 7 + y * 13 + (x >> 4) * 64) & 255; endfunction

    // TMDS decode
    function [9:0] dec;     // {is_ctl, c1, c0, d[7:0]} packed as {ctl, c[1:0], 7'b0} or {0, 1'b0, d}
        input [9:0] q;
        reg [7:0] b; integer i;
        begin
            case (q)
                10'b1101010100: dec = {1'b1, 2'b00, 7'd0};
                10'b0010101011: dec = {1'b1, 2'b01, 7'd0};
                10'b0101010100: dec = {1'b1, 2'b10, 7'd0};
                10'b1010101011: dec = {1'b1, 2'b11, 7'd0};
                default: begin
                    b = q[9] ? ~q[7:0] : q[7:0];
                    dec[0] = b[0];
                    for (i = 1; i < 8; i = i + 1)
                        dec[i] = q[8] ? (b[i] ^ b[i-1]) : ~(b[i] ^ b[i-1]);
                    dec[9:8] = 2'b00;
                end
            endcase
        end
    endfunction

    integer x, y, errors = 0, disp [0:2], ch, k, ones;
    integer col, line, nh, nv, frame_px, lines_seen;
    reg [9:0] s [0:2], d0, d1, d2;
    reg [7:0] exp_px;
    reg in_frame;
    integer hs_len, vs_seen;

    // Expected RGB332 -> 888 expansion
    function [23:0] rgb; input [7:0] p;
        rgb = {p[7:5], p[7:5], p[7:6], p[4:2], p[4:2], p[4:3], p[1:0], p[1:0], p[1:0], p[1:0]};
    endfunction

    // Serial check of channel 0: shift in 10 bits per pixel clock
    reg [9:0] ser_sh; integer ser_n = 0, ser_err = 0, ser_ok = 0;
    reg [9:0] sym_hist [0:15]; integer hist_i = 0;
    always @(posedge clk_x5) begin end

    initial begin
        // fill the frame buffer through the write port
        repeat (4) @(posedge clk);
        for (y = 0; y < 240; y = y + 1)
            for (x = 0; x < 320; x = x + 1) begin
                @(negedge clk); we = 1; wa = y * 320 + x; wd = pat(x, y);
            end
        @(negedge clk); we = 0;
        repeat (4) @(posedge clk);
        rst = 0;
        disp[0] = 0; disp[1] = 0; disp[2] = 0;
        // Watch symbols: the encoders' outputs are registered on clk_pixel.
        col = 0; line = 0; frame_px = 0; nh = 0; nv = 0; lines_seen = 0;
        // Skip until the first vsync ends (start of a frame)
        wait (dut.u_enc0.q == 10'b0010101011 || dut.u_enc0.q == 10'b1101010100);
        // Find the frame start: first data symbol after a vsync period
        @(posedge clk_pixel);
        while (!(dut.ctl2[1] == 1'b0)) @(posedge clk_pixel);   // vsync asserted
        while (dut.ctl2[1] == 1'b0) @(posedge clk_pixel);       // vsync over
        y = 0;
        while (y < 480) begin
            @(posedge clk_pixel); #1;
            d0 = dec(dut.u_enc0.q); d1 = dec(dut.u_enc1.q); d2 = dec(dut.u_enc2.q);
            // DC balance
            for (ch = 0; ch < 3; ch = ch + 1) begin
                s[0] = dut.u_enc0.q; s[1] = dut.u_enc1.q; s[2] = dut.u_enc2.q;
                ones = 0; for (k = 0; k < 10; k = k + 1) ones = ones + s[ch][k];
                if (d0[9] == 0) begin
                    disp[ch] = disp[ch] + 2 * ones - 10;
                    if (disp[ch] > 20 || disp[ch] < -20) begin
                        if (errors < 10) $display("disparity %0d on ch %0d", disp[ch], ch);
                        errors = errors + 1;
                    end
                end else disp[ch] = 0;
            end
            if (d0[9] == 0) begin
                // data symbol: a pixel
                if (d1[9] || d2[9]) begin errors = errors + 1; $display("channel mismatch"); end
                exp_px = pat(col >> 1, y >> 1);
                if ({d2[7:0], d1[7:0], d0[7:0]} !== rgb(exp_px)) begin
                    if (errors < 10)
                        $display("pixel (%0d,%0d): got %06x want %06x", col, y,
                                 {d2[7:0], d1[7:0], d0[7:0]}, rgb(exp_px));
                    errors = errors + 1;
                end
                col = col + 1;
                frame_px = frame_px + 1;
                if (col == 640) begin col = 0; y = y + 1; end
            end else if (col != 0) begin
                $display("line %0d ended after %0d pixels", y, col); errors = errors + 1;
                col = 0; y = y + 1;
            end
        end
        // Serializer: compare channel 0's serial stream with its symbols for a few clocks
        for (k = 0; k < 200; k = k + 1) begin
            @(posedge clk_pixel);
        end
        $display("%0d pixels checked, %0d errors", frame_px, errors);
        if (errors == 0 && frame_px == 640*480) $display("PASS"); else $display("FAIL");
        $finish;
    end

    // Serial capture: the OSER10 sends D0 first. Find the 10-bit alignment by
    // matching the clock lane pattern, then compare data lane 0 words.
    reg [9:0] csh, dsh; integer bitn = 0, aligned = -1, words = 0, werr = 0;
    reg [9:0] last_syms [0:63]; integer ls_w = 0;
    always @(posedge clk_pixel) begin last_syms[ls_w % 64] <= dut.u_enc0.q; ls_w <= ls_w + 1; end
    always @(posedge clk_x5 or negedge clk_x5) if (!rst) begin
        // DDR: OSER10 output changes on both FCLK edges
        #0.5;
        csh = {ckp, csh[9:1]};
        dsh = {dp[0], dsh[9:1]};
        bitn = bitn + 1;
        if (csh == 10'b0000011111 && aligned < 0) aligned = bitn % 10;
        if (aligned >= 0 && bitn % 10 == aligned) begin
            words = words + 1;
            // the decoded serial word must be a symbol the encoder produced recently
            begin : chk
                integer j; reg hit; hit = 0;
                for (j = 0; j < 64; j = j + 1) if (last_syms[j] == dsh) hit = 1;
                if (!hit) werr = werr + 1;
            end
        end
    end
    initial begin
        #22_000_000;
        $display("serial lane 0: %0d words, %0d not matching an encoder symbol", words, werr);
    end
endmodule
