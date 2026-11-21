"""
Wolfenstein-style ray caster on DM16: 64 screen columns, one ray per PE.

Every PE holds the same 16 x 16 map in its memory (one word per cell, 0 =
empty, 1-4 = wall type). Each frame the host sends the player's position,
view direction and camera plane; each PE builds its own ray,

    ray = dir + plane * camX,   camX = -1 (left column) .. +1 (right column)

and marches along it 1/16 of a cell at a time, looking the cell up with
MAR + LDX, until it reaches a wall. A PE switches itself off when its ray
hits, and BANY loops until every ray has hit something. Because the ray is
built on the camera plane, the number of steps is already the perpendicular
distance to the wall, so there is no fisheye correction and no division.

Positions are unsigned Q4.12 (the top 4 bits are the map cell). Each PE
returns one word: wall type << 10 | side << 9 | steps, read with a single
ACC dump.

  python3 dm16_raycast.py               # walk around: W/S or arrows, A/D turn, Q quits
  python3 dm16_raycast.py --demo        # autopilot tour (also used when not on a terminal)
  python3 dm16_raycast.py --demo --check   # compare every column with a Python model
"""
import argparse
import math
import os
import select
import sys
import time

from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE,
    ALU_ADD, ALU_SUB, ALU_AND, ALU_OR, ALU_XOR, ALU_MOV, ALU_MUL, ALU_MULH,
    ALU_SHLN, ALU_SHRN, ALU_ASRN,
    SRC_R, SRC_IMM, SRC_ID, CTX_ALWAYS, CTX_IF_N, CTX_IF_Z, CTX_INVERT,
    OP_LDI, OP_MAR, OP_LDX, BANY, LABEL,
    alu, ctx, ldm, stm, put, get, encode_instr, ldi16,
    read_acc_grid, upload_and_run,
)

FRAC = 12                   # Q4.12
ONE = 1 << FRAC
STEP_SHIFT = 4              # a march step is 1/16 of the ray direction
MAX_STEPS = 400             # beyond any wall in a 16 x 16 map
FOV_PLANE = 0.66            # camera plane length: ~66 degree field of view
CELL = 0x300                # per-PE word: the cell the ray is in

MAP = [
    "1111111111111111",
    "1..............1",
    "1..2222....33..1",
    "1..2..2....33..1",
    "1..2..2........1",
    "1......44444...1",
    "1..............1",
    "1...3......4...1",
    "1...3......4...1",
    "1...3...2..4...1",
    "1.......2......1",
    "1..1111.2..33331",
    "1..............1",
    "1...4......2...1",
    "1..............1",
    "1111111111111111",
]
GRID = [[0 if ch == "." else int(ch) for ch in row] for row in MAP]


def q(v):
    return round(v * ONE) & 0xFFFF


def map_program():
    """Store the map at words 0..255 of every PE (cell index = y * 16 + x)."""
    p = [ctx(CTX_ALWAYS)]
    for y in range(16):
        for x in range(16):
            p += [encode_instr(OP_LDI, imm10=GRID[y][x]), stm(y * 16 + x)]
    return p


def frame_program(pos, direction, plane, max_steps=MAX_STEPS):
    a = lambda op, n: alu(op, SRC_R, n)                 # acc = acc OP r[n]
    i = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)       # acc = acc OP imm
    # camX for PE c, at the centre of its column: (2c + 1) / 64 - 1 = c/32 - 63/64
    cam0 = -ONE + ONE // NUM_PE
    p = [ctx(CTX_ALWAYS),
         *ldi16(cam0), put(7), alu(ALU_MOV, SRC_ID), i(ALU_SHLN, 7), a(ALU_ADD, 7), put(7)]
    for d, pl, reg in ((direction[0], plane[0], 3), (direction[1], plane[1], 4)):
        p += [*ldi16(q(pl)), put(6),
              get(7), a(ALU_MULH, 6), put(0),           # b = high half of camX * plane
              get(7), a(ALU_MUL, 6), alu(ALU_SHRN, SRC_IMM, FRAC, x=True),   # camX * plane (Q4.12)
              put(6), *ldi16(q(d)), a(ALU_ADD, 6),      # ray = dir + plane * camX
              i(ALU_ASRN, STEP_SHIFT), put(reg)]        # step = ray / 16
    p += [*ldi16(q(pos[0])), put(1), *ldi16(q(pos[1])), put(2),
          encode_instr(OP_LDI), put(5),

          LABEL("ray"),
          # active = not done (r5 gets bits 9..15 set when the ray is finished)
          ctx(CTX_ALWAYS), get(5), i(ALU_XOR, -1), ctx(CTX_IF_N),
          get(1), a(ALU_ADD, 3), put(1),                # x += step x
          get(2), a(ALU_ADD, 4), put(2),                # y += step y
          i(ALU_SHRN, 8), i(ALU_AND, 0xF0), put(6),     # (y >> 12) << 4
          get(1), i(ALU_SHRN, FRAC), a(ALU_OR, 6),      # | x >> 12: the cell index
          encode_instr(OP_MAR), encode_instr(OP_LDX), stm(CELL),
          i(ALU_OR, 0), ctx(CTX_IF_Z),                  # still in an empty cell?
          get(5), i(ALU_ADD, 1), put(5), i(ALU_SUB, max_steps),
          ctx(CTX_IF_N),                                # rays that carry on
          ctx(action=CTX_INVERT),                       # the rest: mark them done
          get(5), i(ALU_OR, -512), put(5),
          ctx(action=CTX_INVERT),
          BANY("ray"),

          # side = 1 if the last step crossed a vertical grid line (an east or
          # west face), from comparing the cell column before and after it
          ctx(CTX_ALWAYS),
          get(1), a(ALU_SUB, 3), i(ALU_SHRN, FRAC), put(6),
          get(1), i(ALU_SHRN, FRAC), a(ALU_XOR, 6),
          i(ALU_ADD, 15), i(ALU_SHRN, 4),               # 0 if equal, else 1
          i(ALU_SHLN, 9), put(6),
          get(5), i(ALU_AND, 511), a(ALU_OR, 6), put(6),
          ldm(CELL), i(ALU_SHLN, 10), a(ALU_OR, 6)]     # acc = type << 10 | side << 9 | steps
    return p


# ------------------------------------------------------------------------------
# Exact Python model of the same fixed-point arithmetic, for --check
# ------------------------------------------------------------------------------
def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def model_column(pe, pos, direction, plane, max_steps=MAX_STEPS):
    M = 0xFFFF
    cam = ((-ONE + ONE // NUM_PE) + (pe << 7)) & M     # as on the PE: cam0 + PE_ID << 7
    steps = []
    for d, pl in ((direction[0], plane[0]), (direction[1], plane[1])):
        qp = q(pl)
        hi = (_s16(cam) * _s16(qp) >> 16) & M
        lo = (cam * qp) & M
        prod = (((hi << 16) | lo) >> FRAC) & M
        ray = (q(d) + prod) & M
        steps.append((_s16(ray) >> STEP_SHIFT) & M)
    sx, sy = steps
    x, y, n = q(pos[0]), q(pos[1]), 0
    while True:
        x, y = (x + sx) & M, (y + sy) & M
        cell = GRID[y >> FRAC][x >> FRAC]
        if cell:
            break
        n += 1
        if n >= max_steps:
            break
    side = 1 if (x >> FRAC) != (((x - sx) & M) >> FRAC) else 0
    return (cell << 10) | (side << 9) | n


# ------------------------------------------------------------------------------
# Drawing
# ------------------------------------------------------------------------------
WALL_RGB = {1: (200, 60, 50), 2: (60, 170, 80), 3: (70, 110, 210), 4: (220, 190, 60)}
SHADES = "█▓▒░"


def rgb256(r, g, b):
    c = lambda v: max(0, min(5, round(v / 255 * 5)))
    return 16 + 36 * c(r) + 6 * c(g) + c(b)


def render(words, height, color, pos, angle, wide=2):
    cols = []
    for w in words:
        steps, side, kind = w & 511, (w >> 9) & 1, w >> 10
        if kind == 0:
            cols.append((0, 0, 0, 0.0))
            continue
        dist = (steps + 1) / (1 << STEP_SHIFT)
        h = min(height, int(height / max(dist, 1e-3)))
        cols.append((h, kind, side, dist))
    lines = []
    mid = height / 2
    for row in range(height):
        out = []
        for h, kind, side, dist in cols:
            top = (height - h) // 2
            if h and top <= row < top + h:
                fade = max(0.25, 1 - dist / 12) * (0.65 if side else 1.0)
                if color:
                    r, g, b = (v * fade for v in WALL_RGB[kind])
                    out.append(f"\x1b[48;5;{rgb256(r, g, b)}m" + " " * wide)
                else:
                    ch = SHADES[min(3, int(dist / 3) + side)]
                    out.append(ch * wide)
            else:
                floor = row >= mid
                if color:
                    t = abs(row - mid) / mid
                    g = int(40 + 70 * t) if floor else int(20 + 30 * t)
                    out.append(f"\x1b[48;5;{rgb256(g, g * 0.9, g * 0.8) if floor else rgb256(g * 0.7, g * 0.8, g)}m" + " " * wide)
                else:
                    out.append(("." if floor and abs(row - mid) > mid / 3 else " ") * wide)
        lines.append("".join(out) + ("\x1b[0m" if color else ""))
    # minimap in the top-right corner
    arrow = "→↘↓↙←↖↑↗"[round((angle % (2 * math.pi)) / (math.pi / 4)) % 8]
    mini = []
    for y in range(16):
        row = ""
        for x in range(16):
            if int(pos[0]) == x and int(pos[1]) == y:
                row += arrow
            else:
                row += "#" if GRID[y][x] else "·"
        mini.append(row)
    for k, m in enumerate(mini):
        if k < len(lines):
            lines[k] += ("\x1b[0m" if color else "") + "  " + m
    return "\n".join(lines)


# ------------------------------------------------------------------------------
# Movement
# ------------------------------------------------------------------------------
def free(x, y):
    return GRID[int(y)][int(x)] == 0


def move(pos, angle, dist):
    nx, ny = pos[0] + math.cos(angle) * dist, pos[1] + math.sin(angle) * dist
    pad = 0.2 * (1 if dist > 0 else -1)
    x = nx if free(nx + math.cos(angle) * pad, pos[1]) else pos[0]
    y = ny if free(x, ny + math.sin(angle) * pad) else pos[1]
    return x, y


# A loop along open corridors, at least half a cell from every wall
TOUR = [(1.5, 1.5), (13.5, 1.5), (13.5, 10.5), (9.5, 10.5), (9.5, 12.5), (1.5, 12.5),
        (1.5, 6.5), (9.5, 6.5), (9.5, 8.0), (6.5, 8.0), (6.5, 6.5), (1.5, 6.5)]


def autopilot(pos, angle, target_idx):
    tx, ty = TOUR[target_idx]
    want = math.atan2(ty - pos[1], tx - pos[0])
    diff = (want - angle + math.pi) % (2 * math.pi) - math.pi
    angle += max(-0.12, min(0.12, diff))
    if abs(diff) < 0.4:
        pos = move(pos, angle, 0.12)
    if math.hypot(tx - pos[0], ty - pos[1]) < 0.3:
        target_idx = (target_idx + 1) % len(TOUR)
    return pos, angle, target_idx


def read_key():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = os.read(sys.stdin.fileno(), 3).decode(errors="ignore")
    return {"\x1b[A": "w", "\x1b[B": "s", "\x1b[C": "d", "\x1b[D": "a"}.get(ch, ch[:1].lower())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--height", type=int, default=32, help="screen rows")
    ap.add_argument("--ascii", action="store_true", help="shade characters instead of colour")
    ap.add_argument("--demo", action="store_true", help="autopilot tour instead of the keyboard")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames (0: run until Q / Ctrl-C)")
    ap.add_argument("--check", action="store_true", help="compare every column with the Python model")
    args = ap.parse_args()

    interactive = sys.stdin.isatty() and not args.demo
    pos, angle, target = TOUR[0], 0.0, 1
    frames = bad = 0
    old_tty = None
    if interactive:
        import termios
        import tty
        old_tty = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    with open_dm16(args.port) as ser:
        upload_and_run(ser, map_program())
        print("\x1b[2J\x1b[?25l", end="")
        t_start = time.monotonic()
        try:
            while not args.frames or frames < args.frames:
                if interactive:
                    key = read_key()
                    if key == "q":
                        break
                    if key == "w":
                        pos = move(pos, angle, 0.25)
                    elif key == "s":
                        pos = move(pos, angle, -0.25)
                    elif key == "a":
                        angle -= 0.15
                    elif key == "d":
                        angle += 0.15
                else:
                    pos, angle, target = autopilot(pos, angle, target)

                direction = (math.cos(angle), math.sin(angle))
                plane = (-math.sin(angle) * FOV_PLANE, math.cos(angle) * FOV_PLANE)
                t0 = time.monotonic()
                upload_and_run(ser, frame_program(pos, direction, plane))
                words = read_acc_grid(ser)
                board_ms = (time.monotonic() - t0) * 1000
                frames += 1

                if args.check:
                    exp = [model_column(pe, pos, direction, plane) for pe in range(NUM_PE)]
                    bad += sum(g != e for g, e in zip(words, exp))

                status = (f"DM16 ray caster: 64 rays, one per PE   board {board_ms:4.0f} ms/frame   "
                          f"{frames / (time.monotonic() - t_start):4.1f} fps")
                if args.check:
                    status += f"   check: {'ALL PASS' if not bad else f'{bad} columns differ'}"
                if interactive:
                    status += "   W/S move, A/D turn, Q quit"
                print("\x1b[H" + render(words, args.height, not args.ascii, pos, angle)
                      + "\n\x1b[K" + status, flush=True)
                if interactive:
                    time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            print("\x1b[0m\x1b[?25h", end="")
            if old_tty is not None:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
    if args.check:
        print(f"\n{frames} frames, {frames * NUM_PE} columns checked: "
              f"{'ALL PASS' if not bad else f'{bad} differ'}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
