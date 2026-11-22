`timescale 1ns/1ns
// dm16_fb_writer: tile mode (40x30 FBWs cover 320x240) and row mode
// (stride 5, 5 FBWs per row), with a reference model of the frame buffer.
//   iverilog -s tb -o sim/fbw.vvp sim/tb_fbw.v src/dm16_fb_writer.v && vvp sim/fbw.vvp
module tb;
    localparam NUM_PE = 64, W = 16, COLS = 8;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg op = 0; reg [1:0] sel = 0; reg [15:0] field = 0;
    reg [NUM_PE*W-1:0] acc;
    wire busy, we; wire [16:0] wa; wire [7:0] wd;
    dm16_fb_writer #(.NUM_PE(NUM_PE), .W(W), .COLS(COLS)) dut (
        .clk(clk), .rst_n(rst_n), .op(op), .sel(sel), .field(field), .all_acc(acc),
        .busy(busy), .fb_we(we), .fb_waddr(wa), .fb_wdata(wd));

    reg [7:0] fb [0:76799];
    reg [16:0] writes_ok;
    integer writes = 0, oob = 0;
    always @(posedge clk) if (we) begin
        if (wa >= 76800) oob = oob + 1; else fb[wa] <= wd;
        writes = writes + 1;
    end

    task do_op(input [1:0] s, input [15:0] f);
        begin
            while (busy && s == 2) @(negedge clk);
            @(negedge clk); op = 1; sel = s; field = f;
            @(negedge clk); op = 0;
        end
    endtask

    function [7:0] tile_px(input integer x, input integer y);
        tile_px = (x * 3 + y * 5) ^ (y >> 3);
    endfunction
    function [7:0] row_px(input integer x, input integer y);
        row_px = x + y * 11 + 1;
    endfunction

    integer tx, ty, p, x, y, errors = 0, s;
    initial begin
        #55 rst_n = 1;
        // Tile mode: each FBW covers an 8x8 tile; PE p = (row p/8, col p%8)
        do_op(0, 0); do_op(1, 0);
        for (ty = 0; ty < 30; ty = ty + 1)
            for (tx = 0; tx < 40; tx = tx + 1) begin
                for (p = 0; p < NUM_PE; p = p + 1)
                    acc[p*W +: W] = {8'hA5, tile_px(tx*8 + p%8, ty*8 + p/8)};
                do_op(2, 0);
            end
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        for (y = 0; y < 240; y = y + 1) for (x = 0; x < 320; x = x + 1)
            if (fb[y*320+x] !== tile_px(x, y)) begin
                if (errors < 5) $display("tile (%0d,%0d) %h want %h", x, y, fb[y*320+x], tile_px(x, y));
                errors = errors + 1;
            end
        $display("tile mode: %0d writes, %0d errors", writes, errors);

        // Row mode, stride 5: FBW k of a row puts PE p at x = k + 5p
        do_op(0, 0); do_op(1, 0);
        for (y = 0; y < 240; y = y + 1)
            for (s = 0; s < 5; s = s + 1) begin
                for (p = 0; p < NUM_PE; p = p + 1)
                    acc[p*W +: W] = {8'h00, row_px(s + 5*p, y)};
                do_op(2, 5);
            end
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        for (y = 0; y < 240; y = y + 1) for (x = 0; x < 320; x = x + 1)
            if (fb[y*320+x] !== row_px(x, y)) begin
                if (errors < 5) $display("row (%0d,%0d) %h want %h", x, y, fb[y*320+x], row_px(x, y));
                errors = errors + 1;
            end
        // Zoomed tiles: FBW with zoom 2 (field 0x10): 20 x 15 tiles of 8 x 8
        // cells, each cell a 2 x 2 block, cover the screen
        do_op(0, 0); do_op(1, 0);
        for (ty = 0; ty < 15; ty = ty + 1)
            for (tx = 0; tx < 20; tx = tx + 1) begin
                for (p = 0; p < NUM_PE; p = p + 1)
                    acc[p*W +: W] = {8'h00, tile_px(tx*8 + p%8, ty*8 + p/8) ^ 8'h5A};
                do_op(2, 16'h0010);
            end
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        for (y = 0; y < 240; y = y + 1) for (x = 0; x < 320; x = x + 1)
            if (fb[y*320+x] !== (tile_px(x/2, y/2) ^ 8'h5A)) begin
                if (errors < 5) $display("zoom2 (%0d,%0d) %h want %h", x, y, fb[y*320+x], tile_px(x/2, y/2) ^ 8'h5A);
                errors = errors + 1;
            end
        $display("zoom 2: errors %0d", errors);
        // Zoom 8 (field 0x30): one tile is 64 x 64; at (288, 200) it clips to 32 x 40
        writes = 0;
        do_op(0, 288); do_op(1, 200); do_op(2, 16'h0030);
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        if (writes != 32*40) begin $display("zoom 8 clipped tile wrote %0d", writes); errors = errors + 1; end
        // Zoom 4 (field 0x20): a tile is 32 x 32
        do_op(0, 0); do_op(1, 0);
        for (p = 0; p < NUM_PE; p = p + 1) acc[p*W +: W] = p * 3;
        do_op(2, 16'h0020);
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        for (y = 0; y < 32; y = y + 1) for (x = 0; x < 32; x = x + 1)
            if (fb[y*320+x] !== ((((y/4)*8 + x/4) * 3) & 255)) begin
                if (errors < 5) $display("zoom4 (%0d,%0d) %h", x, y, fb[y*320+x]);
                errors = errors + 1;
            end

        // Clipping: a tile at x=316 writes only 4 columns; y=236 only 4 rows
        writes = 0;
        do_op(0, 316); do_op(1, 236); do_op(2, 0);
        while (busy) @(negedge clk); repeat (3) @(negedge clk);
        if (writes != 16) begin $display("clipped tile wrote %0d", writes); errors = errors + 1; end
        $display("oob %0d, errors %0d", oob, errors);
        if (errors == 0 && oob == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end
endmodule
