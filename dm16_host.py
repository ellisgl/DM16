"""
DM16 host library: machine geometry, instruction encoding and UART helpers.

Every DM16 script imports from here, so the machine's shape only lives in
one place. Pick the board with the DM16_BOARD environment variable:

  console138k  Tang Console 138K: 64 PEs (16 clusters x 4, 4-D hypercube,
               8 x 8 NEWS torus), bf16 FPUs, 50 MHz, 2 Mbaud   (the default)
  nano20k      Tang Nano 20K:  8 PEs (4 clusters x 2, 2-D hypercube, 4 x 2
               torus), bf16 FPUs, 27 MHz, 1 Mbaud
  nano9k       Tang Nano 9K:   8 PEs (4 clusters x 2, 2-D hypercube, 4 x 2
               torus), integer only (no FPU), 27 MHz, 1 Mbaud from reset (its
               BL702 USB bridge doesn't cope with changing rate at run time)

All have 16-bit PEs, 8 registers and 1024 words of memory per PE. open_dm16()
reads the configuration the bitstream reports (command 0x0B) and refuses a
board that doesn't match the profile.

The serial port is found automatically when one Sipeed debugger is plugged
in (the Tang Console's udev symlink /dev/tang_console_uart is used if it
exists); override it with DM16_PORT. The link comes up at 115200 baud after
every FPGA reset; open_dm16() switches both ends to DM16_BAUD (default: the
board's rate) and closing the port switches back to 115200.
"""
import os
import struct
import sys
import time

# ==============================================================================
# Machine geometry: one profile per board (must match the board's top-level
# file in src/; open_dm16() checks)
# ==============================================================================
BOARDS = {
    "console138k": dict(dims=4, lanes=4, cols=8, clk_hz=50_000_000, baud=2_000_000,
                        has_fpu=True, port="/dev/tang_console_uart"),
    "nano20k":     dict(dims=2, lanes=2, cols=4, clk_hz=27_000_000, baud=1_000_000,
                        has_fpu=True, port=None),
    "nano9k":      dict(dims=2, lanes=2, cols=4, clk_hz=27_000_000, baud=1_000_000,
                        has_fpu=False, port=None, boot_baud=1_000_000),
}
BOARD = os.environ.get("DM16_BOARD", "console138k")
if BOARD not in BOARDS:
    raise SystemExit(f"DM16_BOARD={BOARD!r} is not a known board: {', '.join(BOARDS)}")
_P = BOARDS[BOARD]

DIMS  = _P["dims"]                # hypercube dimensions between clusters
LANES = _P["lanes"]               # PEs per cluster
COLS  = _P["cols"]                # NEWS torus width
HAS_FPU = _P["has_fpu"]           # bf16 FPU (without it, FPU instructions are no-ops)
DEFAULT_PORT = os.environ.get("DM16_PORT") or None   # None: find it (see find_port)
SYS_CLK_HZ   = _P["clk_hz"]
BOOT_BAUD    = _P.get("boot_baud", 115_200)   # FPGA UART rate after reset
DEFAULT_BAUD = int(os.environ.get("DM16_BAUD", _P["baud"]))
MAX_BAUD     = 2_000_000          # the Sipeed USB-UART bridges drop bytes above this
# Rates the bridge handles: an odd rate such as 1.5 Mbaud wedged its serial side
SUPPORTED_BAUDS = (115_200, 230_400, 460_800, 500_000, 921_600, 1_000_000, 2_000_000)
NUM_PE  = (1 << DIMS) * LANES     # PEs
ROWS    = NUM_PE // COLS          # NEWS torus height
PE_DIMS = DIMS + (LANES - 1).bit_length()   # bits in a PE_ID
MEM_WORDS = 1024                  # words per PE (2^MEM_AW); checked by open_dm16
DIRECT_WORDS = 1024               # an immediate address reaches only these; above, use MAR
WORD_BITS = 16                    # PE width W (the scripts assume 16); checked by open_dm16
WORD_BYTES = WORD_BITS // 8
WORD_MASK = (1 << WORD_BITS) - 1

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
ALU_MULH = 11   # high 16 bits of acc * opB, signed (x=True: unsigned)
ALU_SHLN = 12   # acc <<= opB[3:0]          (x=True: funnel, acc = ({acc,b} << n)[31:16])
ALU_SHRN = 13   # acc >>= opB[3:0], logical (x=True: funnel, acc = ({b,acc} >> n)[15:0])
ALU_ASRN = 14   # acc >>= opB[3:0], arithmetic
ALU_ROL  = 15   # rotate left by opB[3:0]   (x=True: rotate right)
ALU_X    = 0x4  # flag_ctl bit that selects the variant above

# Operand B (src_sel) for ALU / FPU
SRC_B, SRC_IMM, SRC_ID, SRC_MEM = 0, 1, 2, 3
NUM_REGS = 8    # r0 (= b) .. r[NUM_REGS-1]; checked by open_dm16
SRC_R = SRC_B   # register r[imm]; r0 is b, so imm = 0 is the b register

# SWP sub-ops (register number in imm)
SWP_XCHG, SWP_PUT, SWP_GET = 0, 1, 2

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


def _check_reg(r):
    # The hardware uses the register number modulo NREGS, so a typo or a
    # program written for more registers would silently use another one
    if not 0 <= r < NUM_REGS:
        raise ValueError(f"register r{r} does not exist (this build has r0..r{NUM_REGS - 1})")


def _check_direct(addr, mem):
    # An immediate address is 10 bits; anything else would silently wrap
    if not mem & MEM_MAR and not 0 <= addr < DIRECT_WORDS:
        raise ValueError(f"address {addr} is beyond what an immediate can reach "
                         f"(0..{DIRECT_WORDS - 1}); set MAR and use LDX/STX or MEM_MAR")


def _check_operand(src, imm, mem):
    if src == SRC_R:
        _check_reg(imm)
    elif src == SRC_MEM:
        _check_direct(imm, mem)


def alu(sub_op, src=SRC_B, imm=0, mem=0, x=False):
    """ALU op. For src=SRC_MEM pass the address as imm (or mem flags); for
    src=SRC_R the register number.
    x=True selects the variant: MULH unsigned, SHLN/SHRN funnel, ROL -> ROR."""
    _check_operand(src, imm, mem)
    return encode_instr(OP_ALU, sub_op=sub_op, src_sel=src,
                        flag_ctl=mem | (ALU_X if x else 0), imm10=imm)


def alu_ref(sub_op, acc, opb, b=0, x=False):
    """Reference model of the barrel-shift and MULH ops: returns (acc, C)."""
    acc, opb, b = acc & 0xFFFF, opb & 0xFFFF, b & 0xFFFF
    n = opb & 0xF
    if sub_op == ALU_MULH:
        if x:
            return (acc * opb) >> 16, 0
        return ((to_signed(acc) * to_signed(opb)) >> 16) & 0xFFFF, 0
    if sub_op == ALU_SHLN:
        w = (acc << 16) | (b if x else 0)
        return ((w << n) >> 16) & 0xFFFF, (acc >> (16 - n)) & 1 if n else 0
    if sub_op in (ALU_SHRN, ALU_ASRN):
        hi = (0xFFFF if acc & 0x8000 else 0) if sub_op == ALU_ASRN else (b if x else 0)
        w = (hi << 16) | acc
        return (w >> n) & 0xFFFF, (acc >> (n - 1)) & 1 if n else 0
    if sub_op == ALU_ROL:
        r = ((acc >> n) | (acc << (16 - n))) & 0xFFFF if x else \
            ((acc << n) | (acc >> (16 - n))) & 0xFFFF
        return r, ((r >> 15) if x else r) & 1 if n else 0
    raise ValueError(f"no reference for ALU sub-op {sub_op}")


def fpu(sub_op, src=SRC_B, imm=0, mem=0):
    _check_operand(src, imm, mem)
    return encode_instr(OP_FPU, sub_op=sub_op, src_sel=src, flag_ctl=mem, imm10=imm)


def ctx(cond=CTX_ALWAYS, action=CTX_SET, imm=0):
    return encode_instr(OP_CTX, sub_op=cond, flag_ctl=action, imm10=imm)


def ldm(addr=0, mem=0):
    _check_direct(addr, mem)
    return encode_instr(OP_LDM, flag_ctl=mem, imm10=addr)


def stm(addr=0, mem=0):
    _check_direct(addr, mem)
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
SEQ_FBX, SEQ_FBY, SEQ_FBW = 6, 7, 8


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


# Frame buffer (HDMI builds, 320x240 RGB332: rrrgggbb; no-ops without HDMI)
FB_W, FB_H = 320, 240          # 160 x 120 on the Nano 9K (read_config's fb_size)


def FBX(x):
    """Frame-buffer x for the next FBW (0..319)."""
    if not 0 <= x < 512:
        raise ValueError("FBX: x must be 0..511")
    return _seq(SEQ_FBX, x)


def FBY(y):
    """Frame-buffer y for the next FBW (0..239)."""
    if not 0 <= y < 256:
        raise ValueError("FBY: y must be 0..255")
    return _seq(SEQ_FBY, y)


def FBW(stride=0, zoom=1):
    """Write every PE's ACC[7:0] to the frame buffer, all PEs whether active
    or not. stride 0 (tile): PE p goes to (x + p % COLS, y + p // COLS), as a
    zoom x zoom block (zoom 1, 2, 4 or 8) at (x + (p % COLS) * zoom,
    y + (p // COLS) * zoom); then x += COLS * zoom, wrapping to x = 0,
    y += ROWS * zoom at the right edge. stride s (row, 1..15): PE p goes to
    (x + p * s, y), then x += 1, wrapping to x = 0, y += 1 after s writes.
    Pixels outside the frame buffer are dropped."""
    if not 0 <= stride <= 15:
        raise ValueError("FBW: stride must be 0..15")
    if zoom not in (1, 2, 4, 8) or (zoom != 1 and stride):
        raise ValueError("FBW: zoom must be 1, 2, 4 or 8, and only in tile mode (stride 0)")
    return _seq(SEQ_FBW, stride | ((zoom.bit_length() - 1) << 4))


def rgb332(r, g, b):
    """8-bit colour from 0..255 components."""
    return (r & 0xE0) | ((g >> 3) & 0x1C) | (b >> 6)


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


def swp(r=0):
    """acc <-> r[r]"""
    _check_reg(r)
    return encode_instr(OP_SWP, sub_op=SWP_XCHG, imm10=r)


def put(r):
    """r[r] = acc"""
    _check_reg(r)
    return encode_instr(OP_SWP, sub_op=SWP_PUT, imm10=r)


def get(r):
    """acc = r[r] (flags unchanged)"""
    _check_reg(r)
    return encode_instr(OP_SWP, sub_op=SWP_GET, imm10=r)


NOP  = encode_instr(OP_NOP)
SWP  = swp(0)
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
    # send the bytes before the host side changes rate (or closes the port)
    time.sleep(0.05)


def find_port():
    """The DM16 serial port: DM16_PORT, the board's udev symlink if there is
    one, or else the UART interface (the second one) of the only Sipeed
    debugger plugged in. Sipeed's debuggers present as an FTDI FT2232
    (0403:6010): interface 0 is JTAG, interface 1 the UART to the FPGA."""
    if DEFAULT_PORT:
        return DEFAULT_PORT
    if _P["port"] and os.path.exists(_P["port"]):
        return _P["port"]
    from serial.tools import list_ports
    found = [p.device for p in list_ports.comports()
             if p.vid == 0x0403 and p.pid == 0x6010 and (p.location or "").endswith(".1")]
    if len(found) == 1:
        return found[0]
    if not found:
        raise RuntimeError("no Sipeed USB debugger found: plug the board in, or set DM16_PORT")
    raise RuntimeError(f"several Sipeed debuggers found ({', '.join(found)}): "
                       "set DM16_PORT to the one to use")


CONFIG_LEN = 12


def read_config(ser):
    """The configuration the bitstream reports (command 0x0B), as a dict, or
    None if it doesn't answer (bitstreams from before the command existed)."""
    ser.reset_input_buffer()
    ser.write(b"\x0b")
    b = ser.read(CONFIG_LEN)
    if len(b) != CONFIG_LEN or b[0] != ord("D"):
        return None
    return dict(w=b[2], dims=b[3], lanes=b[4], cols=b[5], nregs=1 << b[6],
                mem_words=1 << b[7], has_fpu=bool(b[8] & 1), rf_bram=bool(b[8] & 2),
                has_hdmi=bool(b[8] & 4), fb_colors=16 if b[8] & 8 else 256,
                fb_size=(160, 120) if b[8] & 16 else (320, 240),
                clk_hz=int.from_bytes(b[9:12], "little") * 1000)


def check_config(cfg):
    """Raise if the board's reported configuration doesn't match the profile."""
    want = dict(w=WORD_BITS, dims=DIMS, lanes=LANES, cols=COLS, nregs=NUM_REGS,
                mem_words=MEM_WORDS, has_fpu=HAS_FPU, clk_hz=SYS_CLK_HZ)
    bad = [f"{k}: board {cfg[k]}, profile {v}" for k, v in want.items() if cfg[k] != v]
    if bad:
        match = [name for name, b in BOARDS.items()
                 if (b["dims"], b["lanes"], b["cols"], b["has_fpu"], b["clk_hz"]) ==
                    (cfg["dims"], cfg["lanes"], cfg["cols"], cfg["has_fpu"], cfg["clk_hz"])]
        hint = f"; try DM16_BOARD={match[0]}" if match else ""
        raise RuntimeError(f"the board doesn't match DM16_BOARD={BOARD} ({'; '.join(bad)}){hint}")


def open_dm16(port=None, baud=None):
    """Open the DM16 UART and switch both ends to `baud` (default DM16_BAUD).

    The FPGA always boots at 115200, and closing the returned port (or
    leaving a `with` block) switches it back to 115200, so every session
    starts from a known rate. If a script died before closing, the last
    rate used is remembered and tried too. If the link is ever lost, press
    the board's reset button (or reload the bitstream) to return to 115200.
    """
    import serial

    class _DM16Serial(serial.Serial):
        def close(self):
            if self.is_open and self.baudrate != BOOT_BAUD:
                try:
                    _set_fpga_divider(self, round(SYS_CLK_HZ / BOOT_BAUD))
                    _remember_baud(BOOT_BAUD)
                except Exception:
                    pass
            super().close()

    port = port or find_port()
    baud = baud or DEFAULT_BAUD
    if baud > MAX_BAUD:
        raise ValueError(f"{baud} baud is above the bridge's reliable limit ({MAX_BAUD})")
    if baud not in SUPPORTED_BAUDS:
        raise ValueError(f"{baud} baud is not one the USB bridge is known to handle; "
                         f"use one of {SUPPORTED_BAUDS}")
    divider = round(SYS_CLK_HZ / baud)
    if abs(SYS_CLK_HZ / divider - baud) / baud > 0.02:
        raise ValueError(f"{baud} baud is not reachable from {SYS_CLK_HZ} Hz "
                         f"(nearest {SYS_CLK_HZ / divider:.0f})")
    # The Tang Nano 9K's BL702 bridge drops bytes sent right after the port
    # is opened or its rate changes, so let it settle each time; and after
    # the FPGA has been loaded over JTAG its UART side only works from the
    # second time the port is opened, so a failed handshake reopens it once.
    SETTLE = 0.05

    def handshake(ser):
        def answers():
            if ser.baudrate != baud:
                ser.baudrate = baud
                time.sleep(SETTLE)
            ser.reset_input_buffer()
            return len(read_ctx_mask_bytes(ser)) == (NUM_PE + 7) // 8

        if answers() or answers():          # (a second try clears stale bytes)
            return True
        for rate in dict.fromkeys([BOOT_BAUD, _last_baud()]):
            if rate == baud:
                continue
            time.sleep(0.01)        # let the FPGA's command timeout (2^17 clocks) clear junk
            ser.baudrate = rate
            time.sleep(SETTLE)
            _set_fpga_divider(ser, divider)
            if answers():
                return True
        return False

    for attempt in range(2):
        ser = _DM16Serial(port, baud, timeout=0.2)
        try:
            ser.set_low_latency_mode(True)     # FTDI latency timer -> 1 ms
        except Exception:
            pass
        time.sleep(SETTLE)
        if handshake(ser):
            break
        serial.Serial.close(ser)
    else:
        raise RuntimeError(f"DM16 did not answer at {baud} baud on {port} (reload the "
                           f"bitstream or press the board's reset button to return it to "
                           f"{BOOT_BAUD} baud)")
    _remember_baud(baud)
    ser.timeout = 1
    cfg = read_config(ser)
    if cfg is None:
        print("dm16_host: the board does not report its configuration (an older "
              "bitstream?), so it can't be checked against the profile", file=sys.stderr)
    else:
        try:
            check_config(cfg)
        except RuntimeError:
            ser.close()
            raise
    return ser


_BAUD_FILE = os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "dm16_last_baud")


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


def _words(data):
    """Little-endian WORD_BYTES-byte words -> list of ints."""
    return [int.from_bytes(data[i:i + WORD_BYTES], "little")
            for i in range(0, len(data), WORD_BYTES)]


def read_acc_grid(ser):
    """Return every PE's ACC (list of NUM_PE ints)."""
    ser.write(b"\x07")
    data = ser.read(WORD_BYTES * NUM_PE)
    if len(data) != WORD_BYTES * NUM_PE:
        raise RuntimeError(f"Expected {WORD_BYTES * NUM_PE} ACC bytes, got {len(data)}")
    return _words(data)


def read_ctx_mask(ser):
    """Return the active-context mask as an int (bit i = PE i)."""
    ser.write(b"\x05")
    return int.from_bytes(ser.read((NUM_PE + 7) // 8), "little")


def read_mem_grid(ser, addr):
    """Read word `addr` from every PE (clobbers ACC, enables all PEs)."""
    inject_instr(ser, ctx(CTX_ALWAYS))
    inject_instr(ser, ldm(addr))
    return read_acc_grid(ser)


BULK_MAX_WORDS = 8192 // (WORD_BYTES * NUM_PE)   # 8 KB per request: the USB bridge buffers that safely


def read_mem_block(ser, addr, count):
    """Read `count` consecutive words from `addr` on every PE (command 0x0A).
    Returns a list of `count` lists of NUM_PE ints. Like read_mem_grid it
    clobbers ACC and enables all PEs; the program must be idle.

    One USB round trip per BULK_MAX_WORDS words instead of one per word. The
    FPGA sends at the full baud rate but the board's USB bridge forwards only
    ~80 KB/s, so it has to buffer the difference; a single 128 KB read
    overflowed it and left its serial side dead until a USB power cycle."""
    if not (0 <= addr < MEM_WORDS and 0 < count and addr + count <= MEM_WORDS):
        raise ValueError(f"words {addr}..{addr + count - 1} are outside PE memory")
    out = []
    for start in range(addr, addr + count, BULK_MAX_WORDS):
        out += _read_mem_block(ser, start, min(BULK_MAX_WORDS, addr + count - start))
    return out


def _read_mem_block(ser, addr, count):
    ser.write(struct.pack(">BHH", 0x0A, addr, count))
    nbytes = count * WORD_BYTES * NUM_PE
    old_timeout = ser.timeout
    ser.timeout = nbytes * 10 * 1.5 / ser.baudrate + 1
    try:
        data = ser.read(nbytes)
    finally:
        ser.timeout = old_timeout
    if len(data) != nbytes:
        raise RuntimeError(f"bulk read: expected {nbytes} bytes, got {len(data)}")
    words = _words(data)
    return [words[k * NUM_PE:(k + 1) * NUM_PE] for k in range(count)]


def read_status(ser):
    """Return (running, any_active) from the sequencer."""
    ser.write(b"\x09")
    b = ser.read(1)
    if len(b) != 1:
        raise RuntimeError("no status byte from DM16")
    return bool(b[0] & 1), bool(b[0] & 2)


def wait_idle(ser, timeout=10.0):
    """Wait until the sequencer has finished running its program."""
    t_end = time.monotonic() + timeout
    while read_status(ser)[0]:
        if time.monotonic() > t_end:
            raise TimeoutError("DM16 program still running")
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
    """Program that writes values[pe] (16-bit) to word `addr` of each PE.
    Above the first 1024 words it goes through MAR (and leaves MAR = addr)."""
    if not 0 <= addr < MEM_WORDS:
        raise ValueError(f"address {addr} is outside PE memory (0..{MEM_WORDS - 1})")
    if addr < DIRECT_WORDS:
        store = [stm(addr)]
        p = []
    else:
        store = [encode_instr(OP_STX)]
        p = [ctx(CTX_ALWAYS), encode_instr(OP_LDI), *ldi16(addr), encode_instr(OP_MAR)]
    for pe, v in enumerate(values):
        p += [ctx(CTX_IF_ID, imm=pe)] + ldi16(v) + store
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
