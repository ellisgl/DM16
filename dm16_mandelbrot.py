"""
Mandelbrot set on DM16, using BANY for the data-dependent loop.

Each PE computes one pixel of a block (8 x 8 on the Tang Console's 64 PEs,
4 x 2 on the Nano 20K, 2 x 2 on the Nano 9K). All PEs iterate
z = z^2 + c together; a PE switches itself off (context flag) when its z
escapes or it reaches the iteration limit, and the sequencer's BANY loops
back while ANY PE is still active. Two hardware LOOPs step the block across
and down the image, so the whole picture is one program run.

Arithmetic is Q4.12 fixed point (range +-8, step 1/4096):
  - MULH gives the high half of x*x, which is x^2 in Q8.8: enough range for
    the escape test |z|^2 > 4 even after z has grown to ~6.
  - MUL + MULH + a funnel SHRN by 12 give x^2 in Q4.12 (by 11: 2xy).

Registers: r1 = x, r2 = y, r3 = cr, r4 = ci, r5 = iteration count (bits
9..15 set once the PE is done), r6/r7 = temporaries, b = high halves for the
funnel shifts.

  python3 dm16_mandelbrot.py                 # whole set, 80 x 40
  python3 dm16_mandelbrot.py --check         # also compare with a Python model
  python3 dm16_mandelbrot.py --center=-0.75,0.1 --width 0.5 --max-iter 400
  python3 dm16_mandelbrot.py --zoom --color      # zoom into a miniature copy of the set
  python3 dm16_mandelbrot.py --zoom --loop       # in and out until Ctrl-C
  python3 dm16_mandelbrot.py --hdmi              # 320 x 240 on the Console's HDMI port
  python3 dm16_mandelbrot.py --hdmi --capture    # ... checked through a capture card
  python3 dm16_mandelbrot.py --hdmi --zoom --loop

With --hdmi each finished 8 x 8 block goes straight to the frame buffer
(FBW): a palette lookup in PE memory turns the escape count into a colour,
and the set itself is black. Nothing is read back over the UART.

A zoom stops at about 75x on an 80-column view: by then neighbouring pixels
are 2 Q4.12 steps (2/4096) apart, and deeper would only get blockier.
"""
import argparse
import math
import time

import numpy as np

from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE, COLS,
    ALU_ADD, ALU_SUB, ALU_AND, ALU_OR, ALU_XOR, ALU_MOV, ALU_MUL, ALU_MULH, ALU_SHRN, ALU_ASRN,
    SRC_R, SRC_IMM, SRC_ID, SRC_MEM, CTX_ALWAYS, CTX_IF_N, CTX_INVERT,
    OP_LDI, OP_MAR, OP_STX, OP_LDX, MEM_INC,
    LOOP, ENDL, BANY, LABEL, FBX, FBY, FBW, FB_W, FB_H, rgb332,
    alu, ctx, ldm, stm, put, get, encode_instr, ldi16,
    read_mem_block, upload_and_run,
)

FRAC = 12                       # Q4.12
ONE = 1 << FRAC
BLOCK = COLS                    # 8 x 8 pixels per pass
ROWS_PER_BLOCK = NUM_PE // COLS

# Per-PE memory
CR0, CRB, CIB, DX8, DY8 = 0x3F0, 0x3F1, 0x3F2, 0x3F3, 0x3F4
RES = 0x000                     # one result word per block, MAR-addressed (up to 0x3EF)

PAL = 0x100                     # --hdmi: 64-entry palette (RGB332), indexed by count & 63
DONE_MARK = 0x3FF & -512        # OR-ing -512 sets bits 9..15: "this PE is done"
ESCAPE = 257                    # escape when (x^2 + y^2) / 4 >= 257 in Q8.8, i.e. |z|^2 > ~4


def q(v):
    return round(v * ONE) & 0xFFFF


def build_program(bw, bh, max_iter, hdmi=False):
    """Whole-image program: bh block rows x bw block columns."""
    a = lambda op, n: alu(op, SRC_R, n)                 # acc = acc OP r[n]
    i = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)       # acc = acc OP imm
    if hdmi:
        start = [FBX(0), FBY(0)]
        # colour = PAL[count & 63], or black inside the set (count == max_iter)
        out = [ctx(CTX_ALWAYS), get(5), i(ALU_AND, 511), i(ALU_SUB, max_iter),
               ctx(CTX_IF_N),
               get(5), i(ALU_AND, 63), i(ALU_ADD, PAL),
               encode_instr(OP_MAR), encode_instr(OP_LDX),
               ctx(action=CTX_INVERT), encode_instr(OP_LDI),
               ctx(CTX_ALWAYS), FBW(0)]
    else:
        start = [encode_instr(OP_MAR, src_sel=1, imm10=RES)]
        out = [ctx(CTX_ALWAYS), get(5), i(ALU_AND, 511),
               encode_instr(OP_STX, flag_ctl=MEM_INC)]
    return [
        *start,
        LOOP(bh),
            ldm(CR0), stm(CRB),                         # back to the left edge
            LOOP(bw),
                # --- one 8 x 8 block: c = (CRB, CIB), z = 0, count = 0 ---
                ctx(CTX_ALWAYS),
                encode_instr(OP_LDI), put(1), put(2), put(5),
                ldm(CRB), put(3), ldm(CIB), put(4),

                LABEL("iter"),
                # active = not done (done PEs get N = 0, which keeps them
                # off through the CTX IF_N tests below)
                ctx(CTX_ALWAYS), get(5), i(ALU_XOR, -1), ctx(CTX_IF_N),

                # escape test in Q8.8: x^2 + y^2 from the high halves
                get(1), a(ALU_MULH, 1), put(6),         # r6 = x^2 (Q8.8)
                get(2), a(ALU_MULH, 2), put(7),         # r7 = y^2 (Q8.8)
                a(ALU_ADD, 6), i(ALU_ASRN, 2), i(ALU_SUB, ESCAPE),
                ctx(CTX_IF_N),                          # still inside: keep going

                # x^2 and y^2 in Q4.12: ({hi, lo} >> 12)
                get(6), put(0), get(1), a(ALU_MUL, 1), alu(ALU_SHRN, SRC_IMM, FRAC, x=True), put(6),
                get(7), put(0), get(2), a(ALU_MUL, 2), alu(ALU_SHRN, SRC_IMM, FRAC, x=True), put(7),
                get(6), a(ALU_SUB, 7), a(ALU_ADD, 3), put(6),          # r6 = x' = x^2 - y^2 + cr
                # y' = 2xy + ci: ({hi, lo} >> 11) is 2xy in Q4.12
                get(1), a(ALU_MULH, 2), put(0),
                get(1), a(ALU_MUL, 2), alu(ALU_SHRN, SRC_IMM, FRAC - 1, x=True),
                a(ALU_ADD, 4), put(2),
                get(6), put(1),

                # count; stop at max_iter
                get(5), i(ALU_ADD, 1), put(5), i(ALU_SUB, max_iter),
                ctx(CTX_IF_N),                          # PEs that carry on
                ctx(action=CTX_INVERT),                 # the rest: mark them done
                get(5), i(ALU_OR, DONE_MARK), put(5),
                ctx(action=CTX_INVERT),
                BANY("iter"),

                # store the count (or draw the block), next block to the right
                *out,
                ldm(CRB), alu(ALU_ADD, SRC_MEM, DX8), stm(CRB),
            ENDL,
            ldm(CIB), alu(ALU_ADD, SRC_MEM, DY8), stm(CIB),    # next block row
        ENDL,
    ]


# ------------------------------------------------------------------------------
# Exact Python model of the same fixed-point arithmetic, for --check
# ------------------------------------------------------------------------------
def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _mulh(a, b):
    return (_s16(a) * _s16(b) >> 16) & 0xFFFF


def _funnel(hi, lo, n):
    return (((hi << 16) | lo) >> n) & 0xFFFF


def model_pixel(cr, ci, max_iter):
    x = y = cnt = 0
    while True:
        x2h, y2h = _mulh(x, x), _mulh(y, y)
        t = ((_s16((x2h + y2h) & 0xFFFF) >> 2) - ESCAPE) & 0xFFFF
        if not t & 0x8000:
            return cnt
        xs = _funnel(x2h, (x * x) & 0xFFFF, FRAC)
        ys = _funnel(y2h, (y * y) & 0xFFFF, FRAC)
        nx = (xs - ys + cr) & 0xFFFF
        ny = (_funnel(_mulh(x, y), (x * y) & 0xFFFF, FRAC - 1) + ci) & 0xFFFF
        x, y, cnt = nx, ny, cnt + 1
        if not (cnt - max_iter) & 0x8000:
            return cnt


# ------------------------------------------------------------------------------
PALETTE = " .:-=+*%"       # escape speed, fast to slow; the set itself is '#'
BLUES = [16, 17, 18, 19, 20, 21, 27, 33, 39, 45, 51, 87, 123, 159, 195, 231]


def render(counts, max_iter, color):
    """Shade by escape time on a log scale (most points escape in a few steps)."""
    top = math.log(max_iter)
    lines = []
    for row in counts:
        s = []
        for n in row:
            if n >= max_iter:
                s.append("\x1b[48;5;232m \x1b[0m" if color else "#")
                continue
            f = math.log(n + 1) / top
            if color:
                s.append(f"\x1b[48;5;{BLUES[min(len(BLUES) - 1, int(f * len(BLUES)))]}m \x1b[0m")
            else:
                s.append(PALETTE[min(len(PALETTE) - 1, int(f * len(PALETTE)))])
        lines.append("".join(s))
    return "\n".join(lines)


def setup_program(view):
    """Per-PE start values, computed on the board: PE (row, col) does pixel
    (col, row) of each 8 x 8 block, so cr = x0 + col * dx, ci = y0 - row * dy."""
    qx0, qy0, qdx, qdy = view
    return [
        ctx(CTX_ALWAYS),
        *ldi16(qx0), put(1), *ldi16(qdx), put(2),
        alu(ALU_MOV, SRC_ID), alu(ALU_AND, SRC_IMM, COLS - 1),            # col
        alu(ALU_MUL, SRC_R, 2), alu(ALU_ADD, SRC_R, 1), stm(CR0),
        *ldi16(qy0), put(1), *ldi16(-qdy), put(2),
        alu(ALU_MOV, SRC_ID), alu(ALU_SHRN, SRC_IMM, COLS.bit_length() - 1),  # row
        alu(ALU_MUL, SRC_R, 2), alu(ALU_ADD, SRC_R, 1), stm(CIB),
        *ldi16(qdx * BLOCK), stm(DX8), *ldi16(-qdy * ROWS_PER_BLOCK), stm(DY8),
    ]


def make_view(cx, cy, width, w, h, aspect):
    """Q4.12 top-left corner and pixel steps, or None if out of range."""
    qdx = round(width / w * ONE)
    qdy = round(width / w * aspect * ONE)
    if qdx < 1 or abs(cx) + width / 2 >= 7 or abs(cy) + qdy * h / ONE / 2 >= 7:
        return None
    qx0 = round(cx * ONE) - qdx * (w - 1) // 2
    qy0 = round(cy * ONE) + qdy * (h - 1) // 2
    return qx0 & 0xFFFF, qy0 & 0xFFFF, qdx, qdy


def compute(ser, view, w, h, max_iter):
    """Run one image on the board; returns (counts[h][w], seconds on the board)."""
    bw, bh = w // BLOCK, h // ROWS_PER_BLOCK
    prog = setup_program(view) + build_program(bw, bh, max_iter)
    t0 = time.monotonic()
    upload_and_run(ser, prog, timeout=120)
    elapsed = time.monotonic() - t0
    counts = [[0] * w for _ in range(h)]
    for k, block in enumerate(read_mem_block(ser, RES, bw * bh)):
        by, bx = divmod(k, bw)
        for pe, n in enumerate(block):
            counts[by * ROWS_PER_BLOCK + pe // COLS][bx * BLOCK + pe % COLS] = n
    return counts, elapsed, len(prog)


def model_image(view, w, h, max_iter):
    """model_pixel for a whole image at once (NumPy): counts[h][w]."""
    qx0, qy0, qdx, qdy = view
    xs = np.arange(w)
    ys = np.arange(h)
    cr = ((qx0 + xs[None, :] * qdx) & 0xFFFF) * np.ones((h, 1), np.int64)
    ci = ((qy0 - ys[:, None] * qdy) & 0xFFFF) * np.ones((1, w), np.int64)
    s16 = lambda v: ((v & 0xFFFF) ^ 0x8000) - 0x8000
    mulh = lambda a, b: (s16(a) * s16(b) >> 16) & 0xFFFF
    funnel = lambda hi, lo, n: (((hi << 16) | lo) >> n) & 0xFFFF
    x = np.zeros_like(cr)
    y = np.zeros_like(cr)
    cnt = np.zeros_like(cr)
    live = np.ones(cr.shape, bool)
    for _ in range(max_iter + 1):
        x2h, y2h = mulh(x, x), mulh(y, y)
        t = ((s16((x2h + y2h) & 0xFFFF) >> 2) - ESCAPE) & 0xFFFF
        live &= (t & 0x8000) != 0
        if not live.any():
            break
        xs_ = funnel(x2h, (x * x) & 0xFFFF, FRAC)
        ys_ = funnel(y2h, (y * y) & 0xFFFF, FRAC)
        nx = (xs_ - ys_ + cr) & 0xFFFF
        ny = (funnel(mulh(x, y), (x * y) & 0xFFFF, FRAC - 1) + ci) & 0xFFFF
        x = np.where(live, nx, x)
        y = np.where(live, ny, y)
        cnt = np.where(live, cnt + 1, cnt)
        live &= cnt < max_iter
    return cnt


# 64 colours for --hdmi, cycled by escape count: dark blue -> cyan ->
# white -> orange -> dark red -> dark blue
_STOPS = [(0, 0, 96), (0, 160, 255), (255, 255, 255), (255, 160, 0), (128, 0, 0), (0, 0, 96)]


def palette():
    out = []
    for k in range(64):
        f = k / 64 * (len(_STOPS) - 1)
        j, t = int(f), f - int(f)
        c = [round(a + (b - a) * t) for a, b in zip(_STOPS[j], _STOPS[j + 1])]
        out.append(rgb332(*c) or 1)        # 0 is reserved for the set itself
    return out


def palette_program():
    prog = [ctx(CTX_ALWAYS)]
    for k, c in enumerate(palette()):
        prog += [*ldi16(c), stm(PAL + k)]
    return prog


def expected_screen(view, max_iter):
    """The RGB332 frame buffer an --hdmi run should leave."""
    counts = model_image(view, FB_W, FB_H, max_iter)
    pal = np.array(palette())
    return np.where(counts >= max_iter, 0, pal[counts & 63])


def compute_hdmi(ser, view, max_iter, with_palette=True):
    bw, bh = FB_W // BLOCK, FB_H // ROWS_PER_BLOCK
    prog = (palette_program() if with_palette else []) + setup_program(view) + \
        build_program(bw, bh, max_iter, hdmi=True)
    t0 = time.monotonic()
    upload_and_run(ser, prog, timeout=120)
    return time.monotonic() - t0, len(prog)


def check(counts, view, max_iter):
    qx0, qy0, qdx, qdy = view
    bad = 0
    for yy, row in enumerate(counts):
        for xx, got in enumerate(row):
            exp = model_pixel((qx0 + xx * qdx) & 0xFFFF, (qy0 - yy * qdy) & 0xFFFF, max_iter) & 511
            if got != exp:
                bad += 1
                if bad <= 5:
                    print(f"  pixel ({xx},{yy}): board {got}, model {exp}")
    print("check: ALL PASS" if not bad else f"check: {bad} pixels differ")
    return bad


# Zoom targets. At 16-bit precision the zoom reaches ~70x, where the
# period-3 "minibrot" on the antenna fills the view: a complete miniature
# copy of the set. Seahorse Valley needs far more iterations and precision
# than 16 bits give at that depth, so it mostly turns black.
MINIBROT = "-1.7625,0"
SEAHORSE = "-0.7436,0.1318"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--size", default="80x40", help="WxH in characters (multiples of 8)")
    ap.add_argument("--center", help=f"default -0.65,0 (still) or {MINIBROT}, the "
                                     f"period-3 minibrot (zoom); try {SEAHORSE} too")
    ap.add_argument("--width", type=float, default=3.0, help="width of the view in the complex plane")
    ap.add_argument("--aspect", type=float, default=2.0, help="character height / width")
    ap.add_argument("--max-iter", type=int, default=200, help="1..511 (a zoom raises it as it goes deeper)")
    ap.add_argument("--color", action="store_true", help="ANSI 256-colour blocks")
    ap.add_argument("--check", action="store_true", help="compare every pixel with the Python model")
    ap.add_argument("--zoom", action="store_true", help="animate a zoom into --center")
    ap.add_argument("--zoom-factor", type=float, default=0.93, help="width multiplier per frame")
    ap.add_argument("--loop", action="store_true", help="zoom in and back out until Ctrl-C")
    ap.add_argument("--hdmi", action="store_true", help="draw 320 x 240 on the HDMI output")
    ap.add_argument("--capture", action="store_true",
                    help="with --hdmi: check the screen through an HDMI capture card")
    ap.add_argument("--png", help="with --capture: save the capture here")
    args = ap.parse_args()

    if args.hdmi:
        return main_hdmi(args)

    w, h = (int(v) for v in args.size.lower().split("x"))
    if w % BLOCK or h % ROWS_PER_BLOCK:
        ap.error(f"size must be a multiple of {BLOCK} x {ROWS_PER_BLOCK}")
    if not 1 <= args.max_iter <= 511:
        ap.error("--max-iter must be 1..511")
    if (w // BLOCK) * (h // ROWS_PER_BLOCK) > CR0 - RES:
        ap.error("image too large for the result area")
    if not 0 < args.zoom_factor < 1:
        ap.error("--zoom-factor must be between 0 and 1")
    cx, cy = (float(v) for v in (args.center or (MINIBROT if args.zoom else "-0.65,0")).split(","))
    if make_view(cx, cy, args.width, w, h, args.aspect) is None:
        ap.error("view out of Q4.12 range or too small to resolve")

    with open_dm16(args.port) as ser:
        if not args.zoom:
            view = make_view(cx, cy, args.width, w, h, args.aspect)
            counts, elapsed, n = compute(ser, view, w, h, args.max_iter)
            print(render(counts, args.max_iter, args.color))
            print(f"{w}x{h}, {args.max_iter} iterations max, {n} instructions: "
                  f"{elapsed * 1000:.0f} ms on the board including upload "
                  f"({sum(map(sum, counts))} pixel-iterations)")
            return 1 if args.check and check(counts, view, args.max_iter) else 0

        # Zoom: shrink the width each frame until neighbouring pixels are
        # 2 Q4.12 steps apart (finer than that the image just gets blocky)
        widths, width = [], args.width
        while make_view(cx, cy, width, w, h, args.aspect) and width / w * ONE >= 2:
            widths.append(width)
            width *= args.zoom_factor
        sequence = widths + (widths[-2:0:-1] if args.loop else [])
        print("\x1b[2J\x1b[?25l", end="")
        bad = frames = 0
        t_start = time.monotonic()
        try:
            while True:
                for width in sequence:
                    depth = math.log2(args.width / width)
                    max_iter = min(511, round(args.max_iter * (1 + depth / 3)))
                    view = make_view(cx, cy, width, w, h, args.aspect)
                    t0 = time.monotonic()
                    counts, elapsed, _ = compute(ser, view, w, h, max_iter)
                    frames += 1
                    print("\x1b[H" + render(counts, max_iter, args.color))
                    print(f"\x1b[Kzoom {args.width / width:6.1f}x  width {width:.5f}  "
                          f"{max_iter} iterations max  board {elapsed * 1000:4.0f} ms  "
                          f"frame {(time.monotonic() - t0) * 1000:4.0f} ms  "
                          f"({frames / (time.monotonic() - t_start):.1f} fps)")
                    if args.check:
                        bad += check(counts, view, max_iter)
                if not args.loop:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            print("\x1b[?25h", end="")
    return 1 if bad else 0


def main_hdmi(args):
    import dm16_hdmi
    cx, cy = (float(v) for v in (args.center or (MINIBROT if args.zoom else "-0.65,0")).split(","))
    if not 1 <= args.max_iter <= 511:
        raise SystemExit("--max-iter must be 1..511")
    device = None
    if args.capture:
        device = dm16_hdmi.find_capture_device()
        if not device:
            raise SystemExit("no capture device found")
    widths, width = [], args.width
    while make_view(cx, cy, width, FB_W, FB_H, 1.0) and width / FB_W * ONE >= 2:
        widths.append(width)
        if not args.zoom:
            break
        width *= args.zoom_factor
    if not widths:
        raise SystemExit("view out of Q4.12 range or too small to resolve")
    sequence = widths + (widths[-2:0:-1] if args.loop else [])
    bad = frames = 0
    t_start = time.monotonic()
    with open_dm16(args.port) as ser:
        cfg = dm16_hdmi.read_config(ser)
        if not cfg or not cfg.get("has_hdmi"):
            raise SystemExit("this bitstream has no HDMI output")
        first = True
        try:
            while True:
                for width in sequence:
                    depth = math.log2(args.width / width)
                    max_iter = min(511, round(args.max_iter * (1 + depth / 3)))
                    view = make_view(cx, cy, width, FB_W, FB_H, 1.0)
                    elapsed, n = compute_hdmi(ser, view, max_iter, with_palette=first)
                    first = False
                    frames += 1
                    print(f"zoom {args.width / width:5.1f}x  {max_iter} iterations max  "
                          f"{n} instructions  {elapsed * 1000:5.0f} ms  "
                          f"({frames / (time.monotonic() - t_start):.1f} fps)", flush=True)
                    if device and not dm16_hdmi.check_screen(
                            device, expected_screen(view, max_iter), args.png):
                        bad += 1
                if not args.loop:
                    break
        except KeyboardInterrupt:
            pass
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
