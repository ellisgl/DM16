"""
DM16 register file test (r0 = b, r1..r7).

The test runs the same programs on all PEs and checks every PE's results
against a small Python interpreter of the instructions used. --rf-pes says
which PEs have the register file (HAS_RF in dm16_pe.v; default all); on any
other PE every r[n] must behave as b.

  - PUT / GET of all 8 registers
  - ALU ops with a register operand
  - SWP acc <-> r[n]
  - context masking of register writes
  - a bf16 dot product with the accumulator held in a register
"""
import argparse

import dm16_bf16_ref as ref
from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE, NUM_REGS, HAS_FPU,
    OP_LDI, LDI_HI, LDI_LO, OP_ALU, OP_SWP, OP_CTX, OP_STM, OP_LDM, OP_FPU,
    ALU_ADD, ALU_AND, ALU_MOV, ALU_SHLN, SRC_R, SRC_IMM, SRC_ID, SRC_MEM,
    CTX_ALWAYS, CTX_IF_Z, CTX_IF_NZ, SWP_XCHG, SWP_PUT, SWP_GET, F_ADD, F_MUL,
    alu, fpu, ctx, ldm, stm, ldi16, swp, put, get,
    read_mem_grid, upload_and_run, write_words, float_to_bf16,
)

OUT = 0x300          # results land in OUT, OUT+1, ...
X, Y = 0x200, 0x240  # bf16 vectors for the dot product


class PEModel:
    """Just enough of a PE to run this test's programs."""

    def __init__(self, pe_id, has_rf, mem):
        self.id, self.has_rf, self.mem = pe_id, has_rf, mem
        self.acc, self.z, self.ctx = 0, False, True
        self.r = [None] * NUM_REGS      # unwritten registers are unknown
        self.r[0] = 0

    def reg(self, n):
        return self.r[n if self.has_rf else 0]

    def setreg(self, n, v):
        self.r[n if self.has_rf else 0] = v

    def run(self, prog):
        for w in prog:
            op, sub, src = w >> 20, (w >> 16) & 15, (w >> 14) & 3
            imm = w & 0x3FF
            simm = (imm | 0xFC00) if imm & 0x200 else imm
            if op == OP_CTX:
                self.ctx = {CTX_ALWAYS: True, CTX_IF_Z: self.z, CTX_IF_NZ: not self.z}[sub]
                continue
            if not self.ctx:
                continue
            opb = [self.reg(imm & (NUM_REGS - 1)), simm, self.id, self.mem.get(imm, 0)][src]
            if op == OP_LDI:
                self.acc = {LDI_HI: ((imm & 0xFF) << 8) | (self.acc & 0xFF),
                            LDI_LO: (self.acc & 0xFF00) | (imm & 0xFF)}.get(sub, simm)
            elif op == OP_LDM:
                self.acc = self.mem.get(imm, 0)
            elif op == OP_STM:
                self.mem[imm] = self.acc
            elif op == OP_SWP:
                old = self.reg(imm & (NUM_REGS - 1))
                if sub != SWP_GET:
                    self.setreg(imm & (NUM_REGS - 1), self.acc)
                if sub != SWP_PUT:
                    self.acc = old
            elif op == OP_ALU:
                self.acc = {ALU_ADD: self.acc + opb, ALU_AND: self.acc & opb, ALU_MOV: opb,
                            ALU_SHLN: self.acc << (opb & 15)}[sub] & 0xFFFF
                self.z = self.acc == 0
            elif op == OP_FPU:
                self.acc = {F_ADD: ref.add, F_MUL: ref.mul}[sub](self.acc, opb)
            else:
                raise ValueError(f"model can't run {w:06X}")
            if self.acc is None:
                raise ValueError("program read a register it never wrote")


def check(name, got, exp):
    bad = [(pe, g, e) for pe, (g, e) in enumerate(zip(got, exp)) if g != e]
    print(f"  {name:<40s} {'PASS' if not bad else f'FAIL ({len(bad)} PEs)'}")
    for pe, g, e in bad[:6]:
        print(f"      PE{pe}: got 0x{g:04X}  expected 0x{e:04X}")
    return not bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--rf-pes", default="all",
                    help="PEs built with the register file: comma list, or 'all'")
    args = ap.parse_args()
    rf_pes = set(range(NUM_PE)) if args.rf_pes == "all" else {int(v) for v in args.rf_pes.split(",")}
    ok = True

    n_dot = 32
    xs = [[float_to_bf16(((pe * 7 + i * 3) % 17 - 8) / 4) for i in range(n_dot)] for pe in range(NUM_PE)]
    ys = [[float_to_bf16(((pe * 5 + i * 11) % 13 - 6) / 2) for i in range(n_dot)] for pe in range(NUM_PE)]

    tests = []

    # 1. PUT r[n] = PE_ID * 16 + n for every register, then GET each back
    p = [ctx(CTX_ALWAYS)]
    for n in range(NUM_REGS):
        p += [alu(ALU_MOV, SRC_ID), alu(ALU_SHLN, SRC_IMM, 4), alu(ALU_ADD, SRC_IMM, n), put(n)]
    for n in range(NUM_REGS):
        p += [get(n), stm(OUT + n)]
    tests.append((f"PUT / GET r0..r{NUM_REGS - 1}", p, list(range(NUM_REGS))))

    # 2. Register operands: sum r1..r7, then AND with r3
    p = [*ldi16(0)] + [alu(ALU_ADD, SRC_R, n) for n in range(1, NUM_REGS)] + [stm(OUT)]
    p += [alu(ALU_AND, SRC_R, 3), stm(OUT + 1)]
    tests.append(("ALU with register operands", p, [0, 1]))

    # 3. SWP: acc <-> r5, then read both back
    p = [*ldi16(1000), swp(5), stm(OUT), get(5), stm(OUT + 1), get(0), stm(OUT + 2)]
    tests.append(("SWP acc <-> r5", p, [0, 1, 2]))

    # 4. Masking: only odd PEs write r7 (and b); everyone reads them back
    p = [alu(ALU_MOV, SRC_ID), alu(ALU_AND, SRC_IMM, 1), ctx(CTX_IF_NZ),
         *ldi16(0x5A5A), put(7), ctx(CTX_ALWAYS), get(7), stm(OUT), get(0), stm(OUT + 1)]
    tests.append(("PUT masked by context", p, [0, 1]))

    # 5. bf16 dot product of x and y with the running sum in r2
    p = [*ldi16(0), put(2)]
    for i in range(n_dot):
        p += [ldm(X + i), fpu(F_MUL, SRC_MEM, Y + i),
              fpu(F_ADD, SRC_R, 2), put(2)]
    p += [get(2), stm(OUT)]
    if HAS_FPU:
        tests.append((f"bf16 dot product ({n_dot} terms, sum in r2)", p, [0]))
    else:
        print("(no FPU on this board: the bf16 dot product is skipped)")

    models = [PEModel(pe, pe in rf_pes, {}) for pe in range(NUM_PE)]
    for pe, m in enumerate(models):
        for i in range(n_dot):
            m.mem[X + i], m.mem[Y + i] = xs[pe][i], ys[pe][i]

    print(f"Register file on PE(s): {sorted(rf_pes) if len(rf_pes) < NUM_PE else 'all'}")
    with open_dm16(args.port) as ser:
        for i in range(n_dot):
            write_words(ser, X + i, [xs[pe][i] for pe in range(NUM_PE)])
            write_words(ser, Y + i, [ys[pe][i] for pe in range(NUM_PE)])
        for name, prog, outs in tests:
            for m in models:
                m.run(prog)
            upload_and_run(ser, prog)
            for k in outs:
                got = read_mem_grid(ser, OUT + k)
                exp = [m.mem[OUT + k] for m in models]
                ok &= check(f"{name} [{k}]" if len(outs) > 1 else name, got, exp)
            # read_mem_grid clobbers acc and context, so the model must too
            for m in models:
                m.acc, m.ctx = m.mem[OUT + outs[-1]], True

    print("\nALL PASS" if ok else "\nFAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
