"""
DM8 host library: machine geometry, instruction encoding and UART helpers.

Every DM8 script imports from here, so the machine size only lives in one
place. Keep the geometry in sync with src/dm8_top.v.

Tang Console 138K: 64 PEs (16 clusters x 4 on a 4-D hypercube, 8x8 NEWS
torus), 16-bit datapath, bfloat16 FPU in every PE, 1024 x 16-bit words of
memory per PE.

The serial port defaults to /dev/tang_console_uart (udev symlink, see
udev/85-tang-console-138k.rules); override it with DM8_PORT. The link comes
up at 115200 baud after every FPGA reset; open_dm8() switches both ends to
DM8_BAUD (default 2 Mbaud) and closing the port switches back to 115200.
"""
import os
import struct
import time

# ==============================================================================
# Machine geometry (must match src/dm8_top.v)
# ==============================================================================
DIMS  = 4                         # hypercube dimensions between clusters
LANES = 4                         # PEs per cluster
COLS  = 8                         # NEWS torus width
DEFAULT_PORT = os.environ.get("DM8_PORT", "/dev/tang_console_uart")
SYS_CLK_HZ   = 50_000_000         # must match src/dm8_top.v (1200 MHz / ODIV)
BOOT_BAUD    = 115_200            # FPGA UART rate after reset
DEFAULT_BAUD = int(os.environ.get("DM8_BAUD", "2000000"))
MAX_BAUD     = 2_000_000          # the console's USB-UART bridge drops bytes above this
NUM_PE  = (1 << DIMS) * LANES     # PEs
ROWS    = NUM_PE // COLS          # NEWS torus height
PE_DIMS = DIMS + (LANES - 1).bit_length()   # bits in a PE_ID
MEM_WORDS = 1024

# ==============================================================================
# Instruction set (24-bit: opcode[23:20] sub_op[19:16] src_sel[15:14]
#                  flag_ctl[13:10] imm10[9:0])
# ==============================================================================
OP_NOP  = 0x0
OP_LDI  = 0x1   # sub 0: acc = sext(imm10)  1: acc[15:8] = imm8  2: acc[7:0] = imm8
OP_LDM  = 0x2   # acc = mem[addr]
OP_STM  = 0x3   # mem[addr] = acc
OP_LDX  = 0x4   # acc = mem[mar]
OP_STX  = 0x5   # mem[mar] = acc
OP_MAR  = 0x6   # mar = src_sel==0 ? acc : imm10
OP_ALU  = 0x7
OP_SWP  = 0x8   # acc <-> b
OP_CTX  = 0x9
OP_SEND = 0xA   # send b
OP_RECV = 0xB   # b = mailbox, C = got one
OP_NEWS = 0xC   # acc = neighbour's acc
OP_FPU  = 0xD
OP_SEQ  = 0xF   # sequencer control (never sent to the PEs)
OP_SET_CTX = OP_CTX
IMEM_SIZE = 4096

LDI_FULL, LDI_HI, LDI_LO = 0, 1, 2

# ALU sub-ops: acc = acc OP operand
ALU_ADD, ALU_ADC, ALU_SUB, ALU_AND, ALU_OR, ALU_XOR = 0, 1, 2, 3, 4, 5
ALU_SHL, ALU_SHR, ALU_MOV, ALU_ASR, ALU_MUL = 6, 7, 8, 9, 10

# Operand B (src_sel) for ALU / FPU
SRC_B, SRC_IMM, SRC_ID, SRC_MEM = 0, 1, 2, 3

# SEND destination modes (src_sel)
DEST_IMM, DEST_ACC, DEST_XOR = 0, 1, 2     # DEST_XOR: PE_ID ^ imm (cube swap)

# Memory addressing bits (flag_ctl) for any memory-touching instruction
MEM_MAR = 0x1   # use mar instead of imm10
MEM_INC = 0x2   # post-increment mar

# SET_CTX conditions (sub_op) and stack actions (flag_ctl)
CTX_ALWAYS, CTX_NEVER, CTX_IF_Z, CTX_IF_NZ, CTX_IF_C, CTX_IF_N, CTX_IF_ID = 0, 1, 2, 3, 4, 5, 6
CTX_SET, CTX_PUSH, CTX_POP, CTX_INVERT = 0, 1, 2, 3

# NEWS directions
NEWS_N, NEWS_E, NEWS_W, NEWS_S = 0, 1, 2, 3

# bfloat16 FPU sub-ops: acc = acc OP operand
F_ADD = 0x0   # acc = acc + opB
F_SUB = 0x1   # acc = acc - opB
F_MUL = 0x2   # acc = acc * opB
F_CMP = 0x3   # Z = acc==opB, N = acc<opB, C = unordered
F_I2F = 0x4   # acc = bf16(int16 acc)
F_F2I = 0x5   # acc = int16(acc), truncate toward zero, saturate
F_NEG = 0x6   # acc = -acc
F_ABS = 0x7   # acc = |acc|


def encode_instr(opcode, sub_op=0, src_sel=0, flag_ctl=0, imm10=0):
    return (
        ((opcode & 0xF) << 20)
        | ((sub_op & 0xF) << 16)
        | ((src_sel & 0x3) << 14)
        | ((flag_ctl & 0xF) << 10)
        | (imm10 & 0x3FF)
    )


def alu(sub_op, src=SRC_B, imm=0, mem=0):
    """ALU op. For src=SRC_MEM pass the address as imm (or mem flags)."""
    return encode_instr(OP_ALU, sub_op=sub_op, src_sel=src, flag_ctl=mem, imm10=imm)


def fpu(sub_op, src=SRC_B, imm=0, mem=0):
    return encode_instr(OP_FPU, sub_op=sub_op, src_sel=src, flag_ctl=mem, imm10=imm)


def ctx(cond=CTX_ALWAYS, action=CTX_SET, imm=0):
    return encode_instr(OP_CTX, sub_op=cond, flag_ctl=action, imm10=imm)


def ldm(addr=0, mem=0):
    return encode_instr(OP_LDM, flag_ctl=mem, imm10=addr)


def stm(addr=0, mem=0):
    return encode_instr(OP_STM, flag_ctl=mem, imm10=addr)


def ldi16(value):
    """Instructions that load any 16-bit constant into acc."""
    v = value & 0xFFFF
    sv = v - 0x10000 if v & 0x8000 else v
    if -512 <= sv <= 511:
        return [encode_instr(OP_LDI, sub_op=LDI_FULL, imm10=sv & 0x3FF)]
    return [encode_instr(OP_LDI, sub_op=LDI_HI, imm10=v >> 8),
            encode_instr(OP_LDI, sub_op=LDI_LO, imm10=v & 0xFF)]


def send(dest, mode=DEST_IMM):
    return encode_instr(OP_SEND, src_sel=mode, imm10=dest)


# ==============================================================================
# Sequencer control + a tiny assembler with labels
#
#   prog = [LOOP(10), alu(ALU_ADD, SRC_IMM, 1), ENDL,
#           LABEL("top"), ..., BANY("top"), HALT]
#   upload_and_run(ser, prog)        # labels are resolved automatically
# ==============================================================================
SEQ_LOOP, SEQ_ENDL, SEQ_JMP, SEQ_BANY, SEQ_BNONE, SEQ_HALT = 0, 1, 2, 3, 4, 5


def _seq(op, field=0):
    return (OP_SEQ << 20) | (op << 16) | (field & 0xFFFF)


class LABEL:
    def __init__(self, name):
        self.name = name


class _Branch:
    def __init__(self, op, target):
        self.op, self.target = op, target


def LOOP(n):
    """Run the following instructions up to the matching ENDL n times (1..65535)."""
    if not 1 <= n <= 0xFFFF:
        raise ValueError("loop count must be 1..65535")
    return _seq(SEQ_LOOP, n)


ENDL = _seq(SEQ_ENDL)
HALT = _seq(SEQ_HALT)


def JMP(target):
    return _Branch(SEQ_JMP, target)


def BANY(target):
    """Branch if any PE is active (global OR of the context flags)."""
    return _Branch(SEQ_BANY, target)


def BNONE(target):
    """Branch if no PE is active."""
    return _Branch(SEQ_BNONE, target)


def assemble(items):
    """Resolve LABELs and branch targets; returns a list of 24-bit words."""
    labels, addr = {}, 0
    for it in items:
        if isinstance(it, LABEL):
            if it.name in labels:
                raise ValueError(f"duplicate label {it.name!r}")
            labels[it.name] = addr
        else:
            addr += 1
    out = []
    for it in items:
        if isinstance(it, LABEL):
            continue
        if isinstance(it, _Branch):
            tgt = labels[it.target] if isinstance(it.target, str) else it.target
            out.append(_seq(it.op, tgt))
        else:
            out.append(it)
    return out


NOP  = encode_instr(OP_NOP)
SWP  = encode_instr(OP_SWP)
RECV = encode_instr(OP_RECV)


# ==============================================================================
# bfloat16 <-> Python float
# ==============================================================================
def float_to_bf16(f):
    """Round a Python float to bf16 bits (round to nearest even)."""
    u = struct.unpack("<I", struct.pack("<f", f))[0]
    if (u & 0x7F800000) == 0x7F800000 and (u & 0x7FFFFF):
        return 0x7FC0
    u += 0x7FFF + ((u >> 16) & 1)
    return (u >> 16) & 0xFFFF


def bf16_to_float(b):
    if (b & 0x7F80) == 0 and (b & 0x7F):   # subnormal: the FPU flushes these
        b &= 0x8000
    return struct.unpack("<f", struct.pack("<I", (b & 0xFFFF) << 16))[0]


def to_signed(v):
    return v - 0x10000 if v & 0x8000 else v


# ==============================================================================
# UART helpers
# ==============================================================================
def _set_fpga_divider(ser, divider):
    ser.write(bytes([0x08, divider >> 8, divider & 0xFF]))
    ser.flush()
    # flush() only drains the OS buffer; give the USB-UART bridge time to
    # send the bytes before the host side changes rate
    time.sleep(0.03)


def open_dm8(port=None, baud=None):
    """Open the DM8 UART and switch both ends to `baud` (default DM8_BAUD).

    The FPGA always boots at 115200, and closing the returned port (or
    leaving a `with` block) switches it back to 115200, so every session
    starts from a known rate. If a script died before closing, the last
    rate used is remembered and tried too. If the link is ever lost, press
    the board's reset button (or reload the bitstream) to return to 115200.
    """
    import serial

    class _DM8Serial(serial.Serial):
        def close(self):
            if self.is_open and self.baudrate != BOOT_BAUD:
                try:
                    _set_fpga_divider(self, round(SYS_CLK_HZ / BOOT_BAUD))
                    _remember_baud(BOOT_BAUD)
                except Exception:
                    pass
            super().close()

    port = port or DEFAULT_PORT
    baud = baud or DEFAULT_BAUD
    if baud > MAX_BAUD:
        raise ValueError(f"{baud} baud is above the bridge's reliable limit ({MAX_BAUD})")
    divider = round(SYS_CLK_HZ / baud)
    if abs(SYS_CLK_HZ / divider - baud) / baud > 0.02:
        raise ValueError(f"{baud} baud is not reachable from {SYS_CLK_HZ} Hz "
                         f"(nearest {SYS_CLK_HZ / divider:.0f})")
    ser = _DM8Serial(port, baud, timeout=0.2)
    try:
        ser.set_low_latency_mode(True)     # FTDI latency timer -> 1 ms
    except Exception:
        pass

    def answers():
        ser.baudrate = baud
        ser.reset_input_buffer()
        return len(read_ctx_mask_bytes(ser)) == (NUM_PE + 7) // 8

    if not answers():
        for rate in dict.fromkeys([BOOT_BAUD, _last_baud()]):
            if rate == baud:
                continue
            time.sleep(0.005)       # let the FPGA's command timeout clear junk
            ser.baudrate = rate
            _set_fpga_divider(ser, divider)
            if answers():
                break
        else:
            serial.Serial.close(ser)
            raise RuntimeError(f"DM8 did not answer at {baud} baud on {port} "
                               "(press the board's reset button to return it to 115200)")
    _remember_baud(baud)
    ser.timeout = 1
    return ser


_BAUD_FILE = os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "dm8_last_baud")


def _last_baud():
    try:
        with open(_BAUD_FILE) as f:
            return int(f.read())
    except (OSError, ValueError):
        return BOOT_BAUD


def _remember_baud(baud):
    try:
        with open(_BAUD_FILE, "w") as f:
            f.write(str(baud))
    except OSError:
        pass


def read_ctx_mask_bytes(ser):
    ser.write(b"\x05")
    return ser.read((NUM_PE + 7) // 8)
def inject_instr(ser, instr):
    """Execute one instruction immediately on all PEs."""
    ser.write(b"\x06" + instr.to_bytes(3, "big"))


def read_acc_grid(ser):
    """Return every PE's 16-bit ACC (list of NUM_PE ints)."""
    ser.write(b"\x07")
    data = ser.read(2 * NUM_PE)
    if len(data) != 2 * NUM_PE:
        raise RuntimeError(f"Expected {2 * NUM_PE} ACC bytes, got {len(data)}")
    return list(struct.unpack(f"<{NUM_PE}H", data))


def read_ctx_mask(ser):
    """Return the active-context mask as an int (bit i = PE i)."""
    ser.write(b"\x05")
    return int.from_bytes(ser.read((NUM_PE + 7) // 8), "little")


def read_mem_grid(ser, addr):
    """Read word `addr` from every PE (clobbers ACC, enables all PEs)."""
    inject_instr(ser, ctx(CTX_ALWAYS))
    inject_instr(ser, ldm(addr))
    return read_acc_grid(ser)


def read_status(ser):
    """Return (running, any_active) from the sequencer."""
    ser.write(b"\x09")
    b = ser.read(1)
    if len(b) != 1:
        raise RuntimeError("no status byte from DM8")
    return bool(b[0] & 1), bool(b[0] & 2)


def wait_idle(ser, timeout=10.0):
    """Wait until the sequencer has finished running its program."""
    t_end = time.monotonic() + timeout
    while read_status(ser)[0]:
        if time.monotonic() > t_end:
            raise TimeoutError("DM8 program still running")
        time.sleep(0.001)


def upload_and_run(ser, program, timeout=10.0):
    """Load a program (labels allowed) into instruction memory and run it
    to completion (end of program or HALT)."""
    program = assemble(program)
    if len(program) > IMEM_SIZE:
        raise ValueError(f"program has {len(program)} instructions (max {IMEM_SIZE})")
    ser.write(b"\x04")                       # halt / reset PC
    time.sleep(0.005)
    payload = bytearray()
    for addr, instr in enumerate(program):
        payload += struct.pack(">BH", 0x01, addr) + instr.to_bytes(3, "big")
    payload += struct.pack(">BH", 0x02, len(program))
    ser.write(payload)
    ser.flush()
    # Wait for the bytes to drain (10 bits each, plus 15% margin), then
    # until the program has finished
    time.sleep(len(payload) * 10 * 1.15 / ser.baudrate + 0.002)
    wait_idle(ser, timeout)


def per_pe_words_program(addr, values):
    """Program that writes values[pe] (16-bit) to word `addr` of each PE."""
    p = []
    for pe, v in enumerate(values):
        p += [ctx(CTX_IF_ID, imm=pe)] + ldi16(v) + [stm(addr)]
    p.append(ctx(CTX_ALWAYS))
    return p


def write_words(ser, addr, values):
    upload_and_run(ser, per_pe_words_program(addr, values))


def write_bf16_grid(ser, addr, floats):
    write_words(ser, addr, [float_to_bf16(f) for f in floats])


def print_grid(title, values, fmt="{:6d}"):
    print(f"\n=== {title} ===")
    for r in range(ROWS):
        row = " ".join(fmt.format(values[r * COLS + c]) for c in range(COLS))
        print(f"Row {r}: {row}")
