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
OP_SEND    = 0xA
OP_RECV    = 0xB

ALU_ADD = 0x0
ALU_AND = 0x3

# Local PE Memory Map:
# 0x010 = Running inclusive prefix sum
# 0x011 = Incoming subcube total from hypercube partner
# 0x012 = Total of the subcube this PE currently belongs to
MEM_SUM = 0x010
MEM_RX  = 0x011
MEM_TOT = 0x012

def encode_instr(opcode, sub_op=0, src_sel=0, flag_ctl=0, imm10=0):
    return (
        ((opcode & 0xF) << 20)
        | ((sub_op & 0xF) << 16)
        | ((src_sel & 0x3) << 14)
        | ((flag_ctl & 0xF) << 10)
        | (imm10 & 0x3FF)
    )

# ==============================================================================
# Host Control & Memory Helpers
# ==============================================================================
def inject_instr(ser, instr):
    ser.write(b"\x06" + instr.to_bytes(3, "big"))

def read_mem_grid(ser, addr):
    inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
    inject_instr(ser, encode_instr(OP_LDM, imm10=addr))
    ser.write(b"\x07")
    data = ser.read(64)
    if len(data) != 64:
        raise RuntimeError(f"Expected 64 bytes, got {len(data)}")
    return list(data)

def upload_and_run(ser, program):
    ser.write(b"\x04")
    time.sleep(0.005)
    payload = bytearray()
    for addr, instr in enumerate(program):
        payload += struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big")
    payload += struct.pack(">BH", 0x02, len(program))
    ser.write(payload)
    ser.flush()
    # Wait for full UART transmission (6 bytes/instr @ 115200 baud = ~0.52 ms/instr) + execution
    time.sleep(len(program) * 0.0006 + 0.02)

def print_grid(title, values):
    print(f"\n=== {title} ===")
    for r in range(8):
        row_str = " ".join(f"{values[r * 8 + c]:3d}" for c in range(8))
        print(f"Row {r}: {row_str}")

# ==============================================================================
# Run One Complete Hypercube Exchange Stage (Dimension d = 0..5)
# ==============================================================================
def run_hypercube_stage(ser, dim):
    step_mask = 1 << dim

    # Step 1: Clear MEM_RX across all 64 PEs
    upload_and_run(ser, [
        encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
        encode_instr(OP_LDI, imm10=0),
        encode_instr(OP_STM, imm10=MEM_RX),
    ])

    # Step 2: Every PE sends its subcube TOTAL to its partner across dimension
    # 'dim' (both directions: the upper half needs it for its prefix, and both
    # halves need it to form the merged subcube total). 8 transfers per batch.
    pairs = [(s, s ^ step_mask) for s in range(64)]
    for batch_start in range(0, len(pairs), 8):
        p = []
        for src_id, dst_id in pairs[batch_start:batch_start + 8]:
            # Sender (src_id) loads MEM_TOT -> B_REG and sends to dst_id
            p.append(encode_instr(OP_SET_CTX, sub_op=0x6, imm10=src_id))
            p.append(encode_instr(OP_LDM, imm10=MEM_TOT))
            p.append(encode_instr(OP_SWP))
            p.append(encode_instr(OP_SEND, imm10=dst_id))

            # Wait 28 clocks for 2-bit serial hypercube link + router arbitration
            p.extend([encode_instr(OP_NOP) for _ in range(28)])

            # Receiver (dst_id) clears B_REG, pops packet from router, saves to MEM_RX
            p.append(encode_instr(OP_SET_CTX, sub_op=0x6, imm10=dst_id))
            p.append(encode_instr(OP_LDI, imm10=0))
            p.append(encode_instr(OP_SWP))
            p.append(encode_instr(OP_RECV))
            p.append(encode_instr(OP_SWP))
            p.append(encode_instr(OP_STM, imm10=MEM_RX))

        upload_and_run(ser, p)

    # Step 3: All PEs: TOT += RX.  Upper-half PEs (bit 'dim' set): SUM += RX.
    upload_and_run(ser, [
        encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
        encode_instr(OP_LDM, imm10=MEM_RX),
        encode_instr(OP_SWP),
        encode_instr(OP_LDM, imm10=MEM_TOT),
        encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x0),
        encode_instr(OP_STM, imm10=MEM_TOT),
        encode_instr(OP_ALU, sub_op=0x8, src_sel=0x2),
        encode_instr(OP_ALU, sub_op=ALU_AND, src_sel=0x1, imm10=step_mask),
        encode_instr(OP_SET_CTX, sub_op=0x3, flag_ctl=0x0),
        encode_instr(OP_LDM, imm10=MEM_RX),
        encode_instr(OP_SWP),
        encode_instr(OP_LDM, imm10=MEM_SUM),
        encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x0),
        encode_instr(OP_STM, imm10=MEM_SUM),
        encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0),
    ])
    
# ==============================================================================
# Main Execution
# ==============================================================================
def main(port="/dev/ttyUSB1"):
    with serial.Serial(port, 115200, timeout=1) as ser:
        # Initialize every PE's MEM_SUM and MEM_TOT to 1 (inclusive prefix sum = 1..64)
        inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0, flag_ctl=0x0))
        inject_instr(ser, encode_instr(OP_LDI, imm10=1))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_SUM))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_TOT))

        print_grid("Initial Input Array (All 64 PEs = 1)", read_mem_grid(ser, MEM_SUM))

        # Execute all 6 Hypercube Dimensions (Dims 0..1 intra-node, Dims 2..5 4D hypercube)
        for dim in range(6):
            run_hypercube_stage(ser, dim)
            print_grid(
                f"After Hypercube Dimension {dim} (XOR mask 0x{1 << dim:02X})",
                read_mem_grid(ser, MEM_SUM),
            )

if __name__ == "__main__":
    main("/dev/ttyUSB1")
