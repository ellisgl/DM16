"""
Conway's Game of Life on the DM8 8x8 NEWS torus (one cell per PE).
"""
import serial
import struct
import time


from dm8_host import (
    DEFAULT_PORT, open_dm8,
    ROWS, COLS, NUM_PE,
    OP_ALU, OP_LDI, OP_LDM, OP_NEWS, OP_SET_CTX, OP_STM, OP_SWP,
    ALU_ADD, ALU_SUB, ALU_MOV, NEWS_N, NEWS_E, NEWS_W, NEWS_S,
    encode_instr, inject_instr, read_mem_grid,
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
    print(f"\033[H\033[J=== DM8 Connection Machine: Game of Life (Gen {gen}) ===")
    for r in range(ROWS):
        line = " ".join("██" if grid[r * COLS + c] else "··" for c in range(COLS))
        print(f"  {line}")

def main(port=None, generations=40, delay=0.15):
    prog = build_life_generation_program()

    # Classic glider (period 4, travels diagonally around the 8x8 torus)
    pattern = [(0, 1), (1, 2), (2, 0), (2, 1), (2, 2)]

    port = port or DEFAULT_PORT
    with open_dm8(port) as ser:
        # Upload the 1-generation kernel once into IMEM
        ser.write(b"\x04")
        for addr, instr in enumerate(prog):
            ser.write(struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big"))

        seed_pattern(ser, pattern)

        for gen in range(generations + 1):
            grid = read_mem_grid(ser, MEM_STATE)
            render_grid(gen, grid)
            time.sleep(delay)

            # Reset PC to 0 and run 1 generation across all PEs
            ser.write(b"\x04")
            ser.write(struct.pack(">BH", 0x02, len(prog)))
            time.sleep(0.005)

if __name__ == "__main__":
    main()