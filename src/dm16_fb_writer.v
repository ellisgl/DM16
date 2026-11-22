`timescale 1ns/1ns
// Frame-buffer writer for the HDMI option: FBX, FBY and FBW (sequencer
// control ops F6/F7/F8) copy the low byte of every PE's ACC into the frame
// buffer in dm16_hdmi.
//
//   FBX x        x = field[8:0]
//   FBY y        y = field[7:0]
//   FBW          write one pixel per PE (all PEs, active or not);
//                field[3:0] = s (stride), field[5:4] = zs (zoom 2^zs):
//     s = 0      tile:  PE p -> a z x z block (z = 2^zs: 1, 2, 4 or 8) at
//                (x + (p % COLS) * z, y + (p / COLS) * z);
//                then x += COLS * z, and when x reaches FB_W: x = 0,
//                y += ROWS * z
//     s = 1..15  row:   PE p -> (x + p * s, y) (zs is ignored);
//                then x += 1, and when x reaches s: x = 0, y += 1
//   Pixels outside the frame buffer are skipped.
//
// FBW snapshots the ACC bytes in one clock and writes them over
// NUM_PE * z * z clocks; the sequencer only stalls if the next FBW arrives
// before that.
module dm16_fb_writer #(
    parameter integer NUM_PE = 64,
    parameter integer W      = 16,
    parameter integer COLS   = 8,
    parameter integer FB_W   = 320,
    parameter integer FB_H   = 240,
    parameter integer FB_AW  = 17
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                op,        // one-clock pulse from the sequencer
    input  wire [1:0]          sel,       // 0 FBX, 1 FBY, 2 FBW
    input  wire [15:0]         field,
    input  wire [NUM_PE*W-1:0] all_acc,
    output wire                busy,

    output reg                 fb_we,
    output reg  [FB_AW-1:0]    fb_waddr,
    output reg  [7:0]          fb_wdata
);
    localparam integer ROWS = NUM_PE / COLS;
    localparam integer PB   = (NUM_PE <= 2) ? 1 : $clog2(NUM_PE);

    reg [8:0] x, y;                  // the next FBW's position

    // The copy in progress
    reg [NUM_PE*8-1:0] snap;
    reg [PB:0]         left;         // PEs still to write
    reg                tile;
    reg [3:0]          stride;
    reg [1:0]          zs;           // zoom shift (tile mode)
    reg [2:0]          sx, sy;       // position inside the z x z block
    reg [10:0]         px, py;       // this PE's pixel (top left of its block)
    reg [10:0]         row_x;        // tile mode: x of the tile's left column
    reg [PB-1:0]       col;

    assign busy = (left != 0);

    wire [3:0]  zm1      = (4'd1 << zs) - 4'd1;     // z - 1
    wire        blk_done = ({1'b0, sx} == zm1) && ({1'b0, sy} == zm1);
    wire [10:0] wx       = px + sx;
    wire [10:0] wy       = py + sy;
    wire [8:0]  tile_w   = COLS << field[5:4];     // tile size at the requested zoom
    wire [8:0]  tile_h   = ROWS << field[5:4];

    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x     <= 0;
            y     <= 0;
            left  <= 0;
            fb_we <= 1'b0;
        end else begin
            fb_we <= 1'b0;

            if (op && sel == 2'd0) x <= field[8:0];
            if (op && sel == 2'd1) y <= {1'b0, field[7:0]};

            if (op && sel == 2'd2 && !busy) begin
                for (p = 0; p < NUM_PE; p = p + 1) begin
                    snap[p*8 +: 8] <= all_acc[p*W +: 8];
                end
                left   <= NUM_PE;
                tile   <= (field[3:0] == 4'd0);
                stride <= field[3:0];
                zs     <= (field[3:0] == 4'd0) ? field[5:4] : 2'd0;
                sx     <= 0;
                sy     <= 0;
                px     <= {2'b0, x};
                py     <= {2'b0, y};
                row_x  <= {2'b0, x};
                col    <= 0;
                if (field[3:0] == 4'd0) begin
                    if (x + tile_w >= FB_W) begin
                        x <= 0;
                        y <= y + tile_h;
                    end else begin
                        x <= x + tile_w;
                    end
                end else begin
                    if (x + 9'd1 >= {5'd0, field[3:0]}) begin
                        x <= 0;
                        y <= y + 9'd1;
                    end else begin
                        x <= x + 9'd1;
                    end
                end
            end else if (busy) begin
                fb_we    <= (wx < FB_W) && (wy < FB_H);
                fb_waddr <= wy * FB_W + wx;
                fb_wdata <= snap[7:0];
                if (!blk_done) begin
                    // next pixel of this PE's block
                    if ({1'b0, sx} == zm1) begin
                        sx <= 0;
                        sy <= sy + 3'd1;
                    end else begin
                        sx <= sx + 3'd1;
                    end
                end else begin
                    // next PE
                    sx   <= 0;
                    sy   <= 0;
                    snap <= snap >> 8;
                    left <= left - 1'b1;
                    if (tile) begin
                        if (col == COLS - 1) begin
                            col <= 0;
                            px  <= row_x;
                            py  <= py + (11'd1 << zs);
                        end else begin
                            col <= col + 1'b1;
                            px  <= px + (11'd1 << zs);
                        end
                    end else begin
                        px <= px + stride;
                    end
                end
            end
        end
    end
endmodule
