import serial
import struct
import time

# ==============================================================================
# DM8 Instruction Encoder
# ==============================================================================
OP_NOP     = 0x0
OP_LDI     = 0x1
OP_LDM     = 0x2
OP_STM     = 0x3
OP_ALU     = 0x7
OP_SWP     = 0x8
OP_SET_CTX = 0x9
OP_NEWS    = 0xC

ALU_ADD = 0x0
ALU_SUB = 0x2
ALU_MOV = 0x8

NEWS_N = 0x0
NEWS_E = 0x1
NEWS_W = 0x2
NEWS_S = 0x3

# Memory Map inside each PE's 1 KB BRAM:
# 0x000 = Current cell state (0 or 1)
# 0x001 = Neighbor count (0..8)
# 0x002 = Next cell state (0 or 1)
MEM_STATE = 0x000
MEM_COUNT = 0x001
MEM_NEXT  = 0x002

def encode_instr(opcode, sub_op=0, src_sel=0, flag_ctl=0, imm10=0):
    return (
        ((opcode & 0xF) << 20)
        | ((sub_op & 0xF) << 16)
        | ((src_sel & 0x3) << 14)
        | ((flag_ctl & 0xF) << 10)
        | (imm10 & 0x3FF)
    )

# ==============================================================================
# Build 1-Generation Parallel Game of Life Program (All 64 PEs in Lockstep)
# ==============================================================================
def build_life_generation_program():
    p = []
    # Enable all 64 PEs
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

    # Commit MEM_NEXT -> MEM_STATE across all 64 PEs
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
def inject_instr(ser, instr):
    ser.write(b"\x06" + instr.to_bytes(3, "big"))

def read_mem_grid(ser, addr):
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    inject_instr(ser, encode_instr(OP_LDM, imm10=addr))
    ser.write(b"\x07")
    return list(ser.read(64))

def seed_pattern(ser, alive_coords):
    # Clear all 64 cells to 0
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    inject_instr(ser, encode_instr(OP_LDI, imm10=0))
    inject_instr(ser, encode_instr(OP_STM, imm10=MEM_STATE))

    # Set initial alive cells to 1
    for (r, c) in alive_coords:
        pe_id = (r % 8) * 8 + (c % 8)
        inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x6, imm10=pe_id))
        inject_instr(ser, encode_instr(OP_LDI, imm10=1))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_STATE))

    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))

def render_grid(gen, grid):
    print(f"\033[H\033[J=== DM8 Connection Machine: Game of Life (Gen {gen}) ===")
    for r in range(8):
        line = " ".join("██" if grid[r * 8 + c] else "··" for c in range(8))
        print(f"  {line}")

def main(port="/dev/ttyUSB1", generations=40, delay=0.15):
    prog = build_life_generation_program()

    # Classic Glider (period 4, moves diagonally down-right across the 8x8 torus)
    glider = [(0, 1), (1, 2), (2, 0), (2, 1), (2, 2)]

    with serial.Serial(port, 115200, timeout=1) as ser:
        # Upload the 1-generation kernel once into IMEM
        ser.write(b"\x04")
        for addr, instr in enumerate(prog):
            ser.write(struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big"))

        seed_pattern(ser, glider)

        for gen in range(generations + 1):
            grid = read_mem_grid(ser, MEM_STATE)
            render_grid(gen, grid)
            time.sleep(delay)

            # Reset PC to 0 and run 1 generation across all 64 PEs
            ser.write(b"\x04")
            ser.write(struct.pack(">BH", 0x02, len(prog)))
            time.sleep(0.005)

if __name__ == "__main__":
    main("/dev/ttyUSB1")