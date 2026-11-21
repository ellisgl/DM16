"""
Mandelbrot set on DM16, using BANY for the data-dependent loop.

Each of the 64 PEs computes one pixel of an 8 x 8 block. All PEs iterate
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
  python3 dm16_mandelbrot.py --center -0.75,0.1 --width 0.5 --max-iter 400
"""
import argparse
import math
import time

from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE, COLS,
    ALU_ADD, ALU_SUB, ALU_AND, ALU_OR, ALU_XOR, ALU_MUL, ALU_MULH, ALU_SHRN, ALU_ASRN,
    SRC_R, SRC_IMM, SRC_MEM, CTX_ALWAYS, CTX_IF_N, CTX_INVERT,
    OP_LDI, OP_MAR, OP_STX, MEM_INC,
    LOOP, ENDL, BANY, LABEL,
    alu, ctx, ldm, stm, put, get, encode_instr, ldi16,
    read_mem_grid, upload_and_run, write_words,
)

FRAC = 12                       # Q4.12
ONE = 1 << FRAC
BLOCK = COLS                    # 8 x 8 pixels per pass
ROWS_PER_BLOCK = NUM_PE // COLS

# Per-PE memory
CR0, CRB, CIB, DX8, DY8 = 0x100, 0x101, 0x102, 0x103, 0x104
RES = 0x200                     # one result word per block, MAR-addressed

DONE_MARK = 0x3FF & -512        # OR-ing -512 sets bits 9..15: "this PE is done"
ESCAPE = 257                    # escape when (x^2 + y^2) / 4 >= 257 in Q8.8, i.e. |z|^2 > ~4


def q(v):
    return round(v * ONE) & 0xFFFF


def build_program(bw, bh, max_iter):
    """Whole-image program: bh block rows x bw block columns."""
    a = lambda op, n: alu(op, SRC_R, n)                 # acc = acc OP r[n]
    i = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)       # acc = acc OP imm
    return [
        encode_instr(OP_MAR, src_sel=1, imm10=RES),
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

                # store the count, next block to the right
                ctx(CTX_ALWAYS), get(5), i(ALU_AND, 511),
                encode_instr(OP_STX, flag_ctl=MEM_INC),
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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--size", default="80x40", help="WxH in characters (multiples of 8)")
    ap.add_argument("--center", default="-0.65,0.0")
    ap.add_argument("--width", type=float, default=3.0, help="width of the view in the complex plane")
    ap.add_argument("--aspect", type=float, default=2.0, help="character height / width")
    ap.add_argument("--max-iter", type=int, default=200, help="1..511")
    ap.add_argument("--color", action="store_true", help="ANSI 256-colour blocks")
    ap.add_argument("--check", action="store_true", help="compare every pixel with the Python model")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    if w % BLOCK or h % ROWS_PER_BLOCK:
        ap.error(f"size must be a multiple of {BLOCK} x {ROWS_PER_BLOCK}")
    if not 1 <= args.max_iter <= 511:
        ap.error("--max-iter must be 1..511")
    bw, bh = w // BLOCK, h // ROWS_PER_BLOCK
    if bw * bh > 0x400 - RES:
        ap.error("image too large for the result area")
    cx, cy = (float(v) for v in args.center.split(","))
    dx = args.width / w
    dy = dx * args.aspect
    if not (q(dx) and abs(cx) + args.width / 2 < 7 and abs(cy) + dy * h / 2 < 7):
        ap.error("view out of Q4.12 range or too small to resolve")
    x0, y0 = cx - dx * (w - 1) / 2, cy + dy * (h - 1) / 2       # top-left pixel

    # Per-PE start values: PE (row, col) does pixel (col, row) of each block
    cr0 = [q(x0 + (pe % COLS) * dx) for pe in range(NUM_PE)]
    ci0 = [q(y0 - (pe // COLS) * dy) for pe in range(NUM_PE)]
    dx8 = (q(dx) * BLOCK) & 0xFFFF
    dy8 = (-q(dy) * ROWS_PER_BLOCK) & 0xFFFF

    with open_dm16(args.port) as ser:
        write_words(ser, CR0, cr0)
        write_words(ser, CIB, ci0)
        upload_and_run(ser, [ctx(CTX_ALWAYS), *ldi16(dx8), stm(DX8), *ldi16(dy8), stm(DY8)])

        prog = build_program(bw, bh, args.max_iter)
        t0 = time.monotonic()
        upload_and_run(ser, prog, timeout=120)
        elapsed = time.monotonic() - t0

        blocks = [read_mem_grid(ser, RES + k) for k in range(bw * bh)]

    counts = [[0] * w for _ in range(h)]
    for k, vals in enumerate(blocks):
        by, bx = divmod(k, bw)
        for pe, n in enumerate(vals):
            counts[by * ROWS_PER_BLOCK + pe // COLS][bx * BLOCK + pe % COLS] = n

    print(render(counts, args.max_iter, args.color))
    iters = sum(map(sum, counts))
    print(f"{w}x{h}, {args.max_iter} iterations max, {len(prog)} instructions: "
          f"{elapsed * 1000:.0f} ms on the board including upload ({iters} pixel-iterations)")

    if args.check:
        bad = 0
        for yy in range(h):
            for xx in range(w):
                bx, px = divmod(xx, BLOCK)
                by, py = divmod(yy, ROWS_PER_BLOCK)
                cr = (cr0[px] + bx * dx8) & 0xFFFF
                ci = (ci0[py * COLS] + by * dy8) & 0xFFFF
                exp = model_pixel(cr, ci, args.max_iter) & 511
                if counts[yy][xx] != exp:
                    bad += 1
                    if bad <= 5:
                        print(f"  pixel ({xx},{yy}): board {counts[yy][xx]}, model {exp}")
        print("check: ALL PASS" if not bad else f"check: {bad} pixels differ")
        return 1 if bad else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
