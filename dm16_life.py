"""
Conway's Game of Life on DM16.

  python3 dm16_life.py                # one cell per PE on the NEWS torus (8 x 8 on
                                      # the Console, 4 x 2 on a Nano), every
                                      # generation checked against a Python Life
  python3 dm16_life.py --hdmi         # a big universe on the HDMI output, until Ctrl-C
  python3 dm16_life.py --hdmi --check --runs 5      # ... compared with a NumPy Life
  python3 dm16_life.py --hdmi --capture --runs 5    # ... and the screen through a capture card

With --hdmi each PE runs many cells, like the Connection Machine's virtual
processors: the universe is cut into tiles the shape of the PE grid (8 x 8
cells on the Console, 4 x 2 on a Nano), and PE p holds cell p of every
tile, one word each. A cell's neighbours come over the NEWS grid; at a
tile's edge the neighbouring PE supplies the cell from the next tile over,
by loading a different word (each PE keeps the offset to use for each of
the eight directions). Tiles are numbered row by row and neighbours wrap
around that numbering, so the universe is a torus with a one-row twist:
going off the right edge brings you back on the left, one row down.

  Console:  160 x 120 cells (20 x 15 tiles), each shown as 2 x 2 pixels
  Nano 20K:  80 x  60 cells (20 x 30 tiles), each 4 x 4 pixels
  Nano 9K:   80 x  60 cells (20 x 30 tiles), each 2 x 2 pixels (160 x 120 screen)

A generation computes every tile (state in bit 0, the next state written to
bit 1), then a second pass shifts the next state down and draws each tile
with one zoomed FBW. The starting universe is random, made on the board.
"""
import argparse
import struct
import time

import numpy as np


from dm16_host import (
    DEFAULT_PORT, open_dm16,
    ROWS, COLS, NUM_PE,
    OP_ALU, OP_LDI, OP_LDM, OP_NEWS, OP_SET_CTX, OP_STM, OP_SWP,
    OP_MAR, OP_LDX, OP_STX, MEM_INC,
    ALU_ADD, ALU_SUB, ALU_MOV, ALU_AND, ALU_OR, ALU_XOR, ALU_MUL,
    ALU_SHL, ALU_SHR, ALU_SHLN, ALU_SHRN,
    SRC_R, SRC_IMM, SRC_ID, SRC_MEM, CTX_ALWAYS, CTX_IF_Z,
    NEWS_N, NEWS_E, NEWS_W, NEWS_S,
    LOOP, ENDL, FBX, FBY, FBW,
    alu, ctx, put, get, ldi16, encode_instr, inject_instr, read_mem_grid, read_mem_block,
    per_pe_words_program, read_config, upload_and_run, wait_idle, assemble,
)

# Memory Map inside each PE's 1 KB BRAM:
# 0x000 = Current cell state (0 or 1)
# 0x001 = Neighbor count (0..8)
# 0x002 = Next cell state (0 or 1)
MEM_STATE = 0x000
MEM_COUNT = 0x001
MEM_NEXT  = 0x002

# ==============================================================================
# Build 1-Generation Parallel Game of Life Program (All PEs in Lockstep)
# ==============================================================================
def build_life_generation_program():
    p = []
    # Enable all PEs
    p.append(encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))

    # --- 1. Sum Cardinal Neighbors (N + S + E + W) into MEM_COUNT ---
    p.append(encode_instr(OP_LDM, imm10=MEM_STATE))
    p.append(encode_instr(OP_NEWS, sub_op=NEWS_N))          # ACC = North
    p.append(encode_instr(OP_STM, imm10=MEM_COUNT))

    for direction in (NEWS_S, NEWS_E, NEWS_W):
        p.append(encode_instr(OP_LDM, imm10=MEM_STATE))
        p.append(encode_instr(OP_NEWS, sub_op=direction))
        p.append(encode_instr(OP_SWP))                       # B_REG = neighbor
        p.append(encode_instr(OP_LDM, imm10=MEM_COUNT))
        p.append(encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x0))
        p.append(encode_instr(OP_STM, imm10=MEM_COUNT))

    # --- 2. Sum Northern Diagonals (NW + NE) ---
    p.append(encode_instr(OP_LDM, imm10=MEM_STATE))
    p.append(encode_instr(OP_NEWS, sub_op=NEWS_N))          # ACC = North row
    p.append(encode_instr(OP_STM, imm10=MEM_NEXT))           # Temp save North row

    for direction in (NEWS_E, NEWS_W):                       # NE and NW
        p.append(encode_instr(OP_LDM, imm10=MEM_NEXT))
        p.append(encode_instr(OP_NEWS, sub_op=direction))
        p.append(encode_instr(OP_SWP))
        p.append(encode_instr(OP_LDM, imm10=MEM_COUNT))
        p.append(encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x0))
        p.append(encode_instr(OP_STM, imm10=MEM_COUNT))

    # --- 3. Sum Southern Diagonals (SW + SE) ---
    p.append(encode_instr(OP_LDM, imm10=MEM_STATE))
    p.append(encode_instr(OP_NEWS, sub_op=NEWS_S))          # ACC = South row
    p.append(encode_instr(OP_STM, imm10=MEM_NEXT))           # Temp save South row

    for direction in (NEWS_E, NEWS_W):                       # SE and SW
        p.append(encode_instr(OP_LDM, imm10=MEM_NEXT))
        p.append(encode_instr(OP_NEWS, sub_op=direction))
        p.append(encode_instr(OP_SWP))
        p.append(encode_instr(OP_LDM, imm10=MEM_COUNT))
        p.append(encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x0))
        p.append(encode_instr(OP_STM, imm10=MEM_COUNT))

    # --- 4. Apply Conway's Rules in Parallel via CTX Masking ---
    # Default next state = 0 (dies unless count == 2 or count == 3)
    p.append(encode_instr(OP_LDI, imm10=0))
    p.append(encode_instr(OP_STM, imm10=MEM_NEXT))

    # Rule A: If (MEM_COUNT - 2 == 0), cell survives -> MEM_NEXT = MEM_STATE
    p.append(encode_instr(OP_LDM, imm10=MEM_COUNT))
    p.append(encode_instr(OP_ALU, sub_op=ALU_SUB, src_sel=0x1, imm10=2))
    p.append(encode_instr(OP_SET_CTX, sub_op=0x2, flag_ctl=0x0))  # CTX = IF ZERO
    p.append(encode_instr(OP_LDM, imm10=MEM_STATE))
    p.append(encode_instr(OP_STM, imm10=MEM_NEXT))

    # Rule B: If (MEM_COUNT - 3 == 0), cell is born/survives -> MEM_NEXT = 1
    p.append(encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))  # CTX = ALL ON
    p.append(encode_instr(OP_LDM, imm10=MEM_COUNT))
    p.append(encode_instr(OP_ALU, sub_op=ALU_SUB, src_sel=0x1, imm10=3))
    p.append(encode_instr(OP_SET_CTX, sub_op=0x2, flag_ctl=0x0))  # CTX = IF ZERO
    p.append(encode_instr(OP_LDI, imm10=1))
    p.append(encode_instr(OP_STM, imm10=MEM_NEXT))

    # Commit MEM_NEXT -> MEM_STATE across all PEs
    p.append(encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    p.append(encode_instr(OP_LDM, imm10=MEM_NEXT))
    p.append(encode_instr(OP_STM, imm10=MEM_STATE))

    # Leave live cells as CTX=1 so onboard LEDs blink with live quadrants!
    p.append(encode_instr(OP_ALU, sub_op=ALU_SUB, src_sel=0x1, imm10=1))
    p.append(encode_instr(OP_SET_CTX, sub_op=0x2, flag_ctl=0x0))
    return p

# ==============================================================================
# Host Loader & Terminal Animator
# ==============================================================================
def seed_pattern(ser, alive_coords):
    # Clear all cells to 0
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    inject_instr(ser, encode_instr(OP_LDI, imm10=0))
    inject_instr(ser, encode_instr(OP_STM, imm10=MEM_STATE))

    # Set initial alive cells to 1
    for (r, c) in alive_coords:
        pe_id = (r % ROWS) * COLS + (c % COLS)
        inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x6, imm10=pe_id))
        inject_instr(ser, encode_instr(OP_LDI, imm10=1))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_STATE))

    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))

def render_grid(gen, grid):
    print(f"\033[H\033[J=== DM16 Connection Machine: Game of Life (Gen {gen}) ===")
    for r in range(ROWS):
        line = " ".join("██" if grid[r * COLS + c] else "··" for c in range(COLS))
        print(f"  {line}")

def life_step(grid):
    """One generation on a ROWS x COLS torus. Wrapping neighbours are counted
    as often as they occur, as the NEWS moves do (on a 2-row torus the cell
    above is also the cell below)."""
    nxt = []
    for r in range(ROWS):
        for c in range(COLS):
            n = sum(grid[((r + dr) % ROWS) * COLS + (c + dc) % COLS]
                    for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc)
            alive = grid[r * COLS + c]
            nxt.append(1 if n == 3 or (alive and n == 2) else 0)
    return nxt


def main(port=None, generations=40, delay=0.15):
    prog = build_life_generation_program()

    # Classic glider (period 4, travels diagonally around the 8x8 torus; on
    # the smaller tori it wraps onto itself, which is still Life)
    pattern = [(0, 1), (1, 2), (2, 0), (2, 1), (2, 2)]
    expect, bad = None, 0

    port = port or DEFAULT_PORT
    with open_dm16(port) as ser:
        # Upload the 1-generation kernel once into IMEM
        ser.write(b"\x04")
        for addr, instr in enumerate(prog):
            ser.write(struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big"))

        seed_pattern(ser, pattern)

        for gen in range(generations + 1):
            grid = read_mem_grid(ser, MEM_STATE)
            render_grid(gen, grid)
            if expect is not None and grid != expect:
                bad += 1
                print(f"  generation {gen} differs from the Python Life")
            expect = life_step(grid)
            time.sleep(delay)

            # Reset PC to 0 and run 1 generation across all PEs
            ser.write(b"\x04")
            ser.write(struct.pack(">BH", 0x02, len(prog)))
            time.sleep(0.002)
            wait_idle(ser)

    print(f"\n{generations} generations: {'ALL PASS' if not bad else f'{bad} differ (FAIL)'}")
    return 1 if bad else 0


# ==============================================================================
# --hdmi: a big universe, many cells per PE
# ==============================================================================
# The eight neighbours as (dx, dy), and the NEWS moves that fetch them:
# NEWS_N gives the value of the PE above (dy = -1), NEWS_W the one to the
# left (dx = -1). Diagonals take a vertical move, then a horizontal one.
DIRS = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
_VMOVE = {-1: NEWS_N, 1: NEWS_S}
_HMOVE = {-1: NEWS_W, 1: NEWS_E}

OFF = 0x3E0          # OFF + d: word offset of direction d's source cell (d = 8: the cell itself)


class Universe:
    """Sizes for this board's frame buffer: tiles of ROWS x COLS cells."""

    def __init__(self, fb_w, fb_h):
        # the largest zoom (2 or 4) that keeps a cell per PE word in memory
        for zoom in (2, 4, 8):
            tx, ty = fb_w // (COLS * zoom), fb_h // (ROWS * zoom)
            if tx * ty + 2 * (tx + 1) <= OFF:
                break
        self.zoom, self.tx, self.ty = zoom, tx, ty
        self.nt = tx * ty                       # tiles
        self.halo = tx + 1                      # copies of the wrapped-around tiles
        self.st0 = self.halo                    # word of tile 0
        self.w, self.h = tx * COLS, ty * ROWS   # in cells

    def source_tile_delta(self, dx, dy, r, c):
        """Which tile, relative to the one being computed, the PE at (r, c)
        must load so that after the NEWS moves its value lands on the PE
        whose (dx, dy) neighbour it is."""
        d = 0
        if dx == 1 and c == 0:
            d += 1
        if dx == -1 and c == COLS - 1:
            d -= 1
        if dy == 1 and r == 0:
            d += self.tx
        if dy == -1 and r == ROWS - 1:
            d -= self.tx
        return d


def life_setup_program(u, seed):
    """Random universe (about 1 cell in 4 alive) made on the board with a
    16-bit xorshift per PE; the per-PE direction offsets are written by the
    host separately (offset_programs)."""
    i = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)
    return [
        ctx(CTX_ALWAYS),
        alu(ALU_MOV, SRC_ID), i(ALU_MUL, 389), i(ALU_ADD, (seed % 500) + 1), put(5),
        encode_instr(OP_MAR, src_sel=1, imm10=u.st0),
        LOOP(u.nt),
            get(5), i(ALU_SHLN, 7), alu(ALU_XOR, SRC_R, 5), put(5),
            get(5), i(ALU_SHRN, 9), alu(ALU_XOR, SRC_R, 5), put(5),
            get(5), i(ALU_SHLN, 8), alu(ALU_XOR, SRC_R, 5), put(5),
            encode_instr(OP_LDI), put(4),
            get(5), i(ALU_AND, 3), ctx(CTX_IF_Z), i(ALU_MOV, 1), put(4),
            ctx(CTX_ALWAYS), get(4), encode_instr(OP_STX, flag_ctl=MEM_INC),
        ENDL,
    ]


def offset_programs(u):
    progs = []
    for d, (dx, dy) in enumerate(DIRS + [(0, 0)]):
        vals = [u.st0 + u.source_tile_delta(dx, dy, p // COLS, p % COLS) for p in range(NUM_PE)]
        progs.append(per_pe_words_program(OFF + d, vals))
    return progs


def life_hdmi_program(u, generations, colour):
    """`generations` generations, each drawn when it is done."""
    i = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)

    def copy(src, dst, n):                         # mem[dst + j] = mem[src + j], j < n
        return [
            *ldi16(src), put(5), *ldi16(dst), put(6),
            LOOP(n),
                get(5), encode_instr(OP_MAR), encode_instr(OP_LDX), put(4),
                get(6), encode_instr(OP_MAR), get(4), encode_instr(OP_STX),
                get(5), i(ALU_ADD, 1), put(5), get(6), i(ALU_ADD, 1), put(6),
            ENDL,
        ]

    count = []
    for d, (dx, dy) in enumerate(DIRS):
        count += [get(1), alu(ALU_ADD, SRC_MEM, OFF + d), encode_instr(OP_MAR),
                  encode_instr(OP_LDX), i(ALU_AND, 1)]
        if dy:
            count.append(encode_instr(OP_NEWS, sub_op=_VMOVE[dy]))
        if dx:
            count.append(encode_instr(OP_NEWS, sub_op=_HMOVE[dx]))
        count += [put(2)] if d == 0 else [alu(ALU_ADD, SRC_R, 2), put(2)]

    return [
        ctx(CTX_ALWAYS),
        LOOP(generations),
            # wrap-around copies: before tile 0 the last tiles, after the last tile the first
            *copy(u.st0 + u.nt - u.halo, 0, u.halo),
            *copy(u.st0, u.st0 + u.nt, u.halo),
            # every tile: r2 = live neighbours, r3 = this cell; bit 1 = next state
            encode_instr(OP_LDI), put(1),               # r1 = tile
            LOOP(u.nt),
                *count,
                get(1), alu(ALU_ADD, SRC_MEM, OFF + 8), encode_instr(OP_MAR),
                encode_instr(OP_LDX), put(3),
                encode_instr(OP_LDI), put(4),                          # dies...
                get(2), i(ALU_SUB, 3), ctx(CTX_IF_Z), i(ALU_MOV, 2), put(4),   # ...unless 3
                ctx(CTX_ALWAYS), get(2), i(ALU_SUB, 2), ctx(CTX_IF_Z),
                get(3), alu(ALU_SHL), put(4),                          # ...or 2 and alive
                ctx(CTX_ALWAYS), get(4), alu(ALU_OR, SRC_R, 3), encode_instr(OP_STX),
                get(1), i(ALU_ADD, 1), put(1),
            ENDL,
            # next state -> state, and draw
            encode_instr(OP_MAR, src_sel=1, imm10=u.st0), FBX(0), FBY(0),
            LOOP(u.nt),
                encode_instr(OP_LDX), alu(ALU_SHR), encode_instr(OP_STX, flag_ctl=MEM_INC),
                i(ALU_MUL, colour), FBW(0, u.zoom),
            ENDL,
        ENDL,
    ]


def read_universe(ser, u):
    """The board's universe as (NT, ROWS, COLS) 0/1."""
    blocks = read_mem_block(ser, u.st0, u.nt)
    return np.array(blocks, np.int64).reshape(u.nt, ROWS, COLS) & 1


def model_step(u, s):
    """One generation of the same twisted torus, in tile-and-PE space."""
    n = np.zeros_like(s)
    rr, cc = np.meshgrid(np.arange(ROWS), np.arange(COLS), indexing="ij")
    for dx, dy in DIRS:
        sr, sc = (rr + dy) % ROWS, (cc + dx) % COLS            # source PE of each PE
        delta = np.vectorize(lambda r, c: u.source_tile_delta(dx, dy, r, c))(sr, sc)
        k = (np.arange(u.nt)[:, None, None] + delta[None]) % u.nt
        n += s[k, sr[None], sc[None]]
    return ((n == 3) | ((n == 2) & (s == 1))).astype(np.int64)


def to_cells(u, s):
    """(NT, ROWS, COLS) -> the universe as an (h, w) picture."""
    return s.reshape(u.ty, u.tx, ROWS, COLS).transpose(0, 2, 1, 3).reshape(u.h, u.w)


def main_hdmi(args):
    import dm16_hdmi
    with open_dm16(args.port) as ser:
        ncol = dm16_hdmi.configure(read_config(ser))
        if not ncol:
            raise SystemExit("this bitstream has no HDMI output")
        u = Universe(dm16_hdmi.FB_W, dm16_hdmi.FB_H)
        colour = args.colour if args.colour is not None else (0x1C if ncol == 256 else 10)
        print(f"universe {u.w} x {u.h} cells ({u.tx} x {u.ty} tiles of {COLS} x {ROWS}, "
              f"{u.nt} cells per PE), shown {u.zoom} x {u.zoom}")
        for prog in offset_programs(u):
            upload_and_run(ser, prog)
        upload_and_run(ser, life_setup_program(u, args.seed))
        state = read_universe(ser, u) if (args.check or args.capture) else None
        device = dm16_hdmi.find_capture_device() if args.capture else None
        if args.capture and not device:
            raise SystemExit("no capture device found")

        prog = life_hdmi_program(u, args.per_run, colour)
        n_instr = len(assemble(prog))
        bad = runs = 0
        t0 = time.monotonic()
        try:
            while args.runs == 0 or runs < args.runs:
                t = time.monotonic()
                if runs == 0:
                    upload_and_run(ser, prog, timeout=60)     # load and run it
                else:
                    ser.write(b"\x04" + struct.pack(">BH", 0x02, n_instr))   # run it again
                    time.sleep(0.002)
                    wait_idle(ser, 60)
                runs += 1
                dt = time.monotonic() - t
                if args.fps and not (args.check or args.capture):
                    time.sleep(max(0.0, args.per_run / args.fps - dt))
                line = (f"generation {runs * args.per_run:6d}  {dt * 1000 / args.per_run:6.2f} ms "
                        f"per generation  "
                        f"({runs * args.per_run / (time.monotonic() - t0):.0f} generations/s)")
                if state is not None:
                    for _ in range(args.per_run):
                        state = model_step(u, state)
                    board = read_universe(ser, u)
                    ok = np.array_equal(board, state)
                    line += f"  universe {'matches' if ok else 'DIFFERS from'} the model"
                    if not ok:
                        bad += 1
                        state = board
                    if device:
                        cells = to_cells(u, state)
                        fb = np.kron(cells, np.ones((u.zoom, u.zoom), np.int64)) * colour
                        full = np.zeros((dm16_hdmi.FB_H, dm16_hdmi.FB_W), np.int64)
                        full[:fb.shape[0], :fb.shape[1]] = fb
                        print(line, flush=True)
                        line = ""
                        if not dm16_hdmi.check_screen(device, full, args.png, ncol=ncol):
                            bad += 1
                if line:
                    print(line, flush=True)
        except KeyboardInterrupt:
            pass
    if args.check or args.capture:
        print("ALL PASS" if not bad else f"{bad} FAILED")
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Conway's Game of Life on DM16")
    ap.add_argument("--port", default=None)
    ap.add_argument("--hdmi", action="store_true", help="big universe on the HDMI output")
    ap.add_argument("--fps", type=float, default=15,
                    help="--hdmi: generations per second to show (0: as fast as it goes)")
    ap.add_argument("--per-run", type=int, default=1,
                    help="--hdmi: generations per program run (each one is drawn)")
    ap.add_argument("--runs", type=int, default=0, help="--hdmi: program runs (0: until Ctrl-C)")
    ap.add_argument("--seed", type=int, default=1, help="--hdmi: random start")
    ap.add_argument("--colour", type=int, help="--hdmi: live cell colour (pixel value)")
    ap.add_argument("--check", action="store_true", help="--hdmi: compare with a NumPy Life after each run")
    ap.add_argument("--capture", action="store_true", help="--hdmi: also check the screen with a capture card")
    ap.add_argument("--png", help="--capture: save the capture here")
    a = ap.parse_args()
    if a.hdmi:
        raise SystemExit(main_hdmi(a))
    raise SystemExit(main(a.port))