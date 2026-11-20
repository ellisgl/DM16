import serial
import struct
import time

# ==============================================================================
# DM8 Instruction Encoder (24-bit SIMD Microinstructions)
# ==============================================================================
OP_NOP     = 0x0
OP_LDI     = 0x1
OP_LDM     = 0x2
OP_STM     = 0x3
OP_LDX     = 0x4
OP_STX     = 0x5
OP_MOV_MAR = 0x6
OP_ALU     = 0x7
OP_SWP     = 0x8
OP_SET_CTX = 0x9
OP_SEND    = 0xA
OP_RECV    = 0xB
OP_NEWS    = 0xC

# ALU Sub-Ops
ALU_ADD = 0x0
ALU_ADC = 0x1
ALU_SUB = 0x2
ALU_AND = 0x3
ALU_OR  = 0x4
ALU_XOR = 0x5
ALU_SHL = 0x6
ALU_SHR = 0x7
ALU_MOV = 0x8

# NEWS Directions
NEWS_N = 0x0
NEWS_E = 0x1
NEWS_W = 0x2
NEWS_S = 0x3

def encode_instr(opcode, sub_op=0, src_sel=0, flag_ctl=0, imm10=0):
    return (
        ((opcode & 0xF) << 20)
        | ((sub_op & 0xF) << 16)
        | ((src_sel & 0x3) << 14)
        | ((flag_ctl & 0xF) << 10)
        | (imm10 & 0x3FF)
    )

# ==============================================================================
# Host UART Control & Memory Read/Write Helpers
# ==============================================================================
def reset_pc(ser):
    ser.write(b"\x04")
    time.sleep(0.01)

def inject_instr(ser, instr):
    ser.write(b"\x06" + instr.to_bytes(3, "big"))

def read_acc_grid(ser):
    ser.write(b"\x07")
    data = ser.read(64)
    if len(data) != 64:
        raise RuntimeError(f"Expected 64 ACC bytes, got {len(data)}")
    return list(data)

def read_mem_grid(ser, addr):
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    inject_instr(ser, encode_instr(OP_LDM, imm10=addr))
    return read_acc_grid(ser)

def write_pe_mem(ser, pe_id, addr, value):
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x6, imm10=pe_id))
    inject_instr(ser, encode_instr(OP_LDI, imm10=value))
    inject_instr(ser, encode_instr(OP_STM, imm10=addr))
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))

def upload_and_run(ser, program):
    reset_pc(ser)
    for addr, instr in enumerate(program):
        pkt = struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big")
        ser.write(pkt)
    ser.write(struct.pack(">BH", 0x02, len(program)))
    time.sleep(0.03)

def print_grid(title, values, fmt="{:3d}"):
    print(f"\n=== {title} ===")
    for r in range(8):
        row_str = " ".join(fmt.format(values[r * 8 + c]) for c in range(8))
        print(f"Row {r}: {row_str}")

# ==============================================================================
# Test Suite Execution
# ==============================================================================
def main(port="/dev/ttyUSB1"):
    with serial.Serial(port, 115200, timeout=1) as ser:
        # ----------------------------------------------------------------------
        # TEST 1: PE Local BRAM Read/Write + Parallel Math
        # ----------------------------------------------------------------------
        # Clear MEM[0x010] to 0 across all 64 PEs, then write custom seeds to PE 0 & PE 63
        inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
        inject_instr(ser, encode_instr(OP_LDI, imm10=10))
        inject_instr(ser, encode_instr(OP_STM, imm10=0x010))
        write_pe_mem(ser, pe_id=0,  addr=0x010, value=90)
        write_pe_mem(ser, pe_id=63, addr=0x010, value=190)

        # Program: Load MEM[0x010], add PE_ID, store to MEM[0x011]
        prog_bram = [
            encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
            encode_instr(OP_LDM, imm10=0x010),
            encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x2),  # ACC += PE_ID
            encode_instr(OP_STM, imm10=0x011),
        ]
        upload_and_run(ser, prog_bram)
        mem_011 = read_mem_grid(ser, 0x011)
        print_grid("Test 1: MEM[0x011] after (Seed + PE_ID) [PE0=90, PE63=253]", mem_011)

        # ----------------------------------------------------------------------
        # TEST 2: 2D NEWS Torus Grid Shift (East + South)
        # ----------------------------------------------------------------------
        prog_news = [
            encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
            encode_instr(OP_ALU, sub_op=ALU_MOV, src_sel=0x2),  # ACC <= PE_ID
            encode_instr(OP_NEWS, sub_op=NEWS_E),               # Pull from East neighbor
            encode_instr(OP_NEWS, sub_op=NEWS_S),               # Pull from South neighbor
            encode_instr(OP_STM, imm10=0x020),
        ]
        upload_and_run(ser, prog_news)
        news_grid = read_mem_grid(ser, 0x020)
        print_grid("Test 2: 2D NEWS Torus (Each PE holds neighbor at Row+1, Col+1)", news_grid)

        # ----------------------------------------------------------------------
        # TEST 3: 4D Hypercube Router (Multi-Hop Cross-Chip Permutation)
        # ----------------------------------------------------------------------
        # Send one packet from PE 0 -> PE 63 (traverses all 4 hypercube dimensions!)
        # and PE 63 -> PE 0 simultaneously
        prog_hc = [
            encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
            encode_instr(OP_LDI, imm10=0),
            encode_instr(OP_STM, imm10=0x030),                  # Clear RX slot

            # PE 0 sends 0xAA to PE 63 (Node 15, PE 3 -> 4 hops across 4D hypercube)
            encode_instr(OP_SET_CTX, sub_op=0x6, imm10=0),
            encode_instr(OP_LDI, imm10=0xAA),
            encode_instr(OP_SWP),                               # Move 0xAA into B_REG
            encode_instr(OP_SEND, imm10=63),                    # Inject into 4D Hypercube

            # PE 63 sends 0x55 to PE 0 (Node 0, PE 0 -> 4 hops back)
            encode_instr(OP_SET_CTX, sub_op=0x6, imm10=63),
            encode_instr(OP_LDI, imm10=0x55),
            encode_instr(OP_SWP),
            encode_instr(OP_SEND, imm10=0),

            # Wait ~40 NOP cycles for 4-hop 2-bit serial traversal (8 clocks/hop * 4 hops = 32)
            *[encode_instr(OP_NOP) for _ in range(45)],

            # All PEs check RX FIFO and store received byte to MEM[0x030]
            encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
            encode_instr(OP_RECV),                              # Pop RX FIFO into B_REG
            encode_instr(OP_SWP),                               # Move B_REG -> ACC
            encode_instr(OP_STM, imm10=0x030),
        ]
        upload_and_run(ser, prog_hc)
        hc_grid = read_mem_grid(ser, 0x030)
        print_grid("Test 3: 4D Hypercube 4-Hop Exchange (PE0=0x55/85, PE63=0xAA/170)", hc_grid)

if __name__ == "__main__":
    main("/dev/ttyUSB1")
