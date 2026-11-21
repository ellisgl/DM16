"""
DM16 barrel shifter / MULH board test.

Every PE gets its own acc and b values and runs the same programs; results
and the carry flag are checked against dm16_host.alu_ref.

  - SHLN / SHRN / ASRN / ROL / ROR by an immediate and by b
  - funnel SHLN / SHRN across acc:b
  - MULH signed and unsigned
  - a fixed-point use: Q8.8 multiply with MUL + MULH + funnel shift
"""
import argparse
import random

from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE,
    ALU_MUL, ALU_MULH, ALU_SHLN, ALU_SHRN, ALU_ASRN, ALU_ROL, ALU_MOV,
    SRC_B, SRC_IMM, SRC_MEM, CTX_ALWAYS, CTX_IF_C,
    SWP, alu, alu_ref, ctx, ldm, stm, ldi16,
    read_mem_grid, upload_and_run, write_words, to_signed,
)

A, B, RES, CARRY, TMP = 0x100, 0x101, 0x102, 0x103, 0x104

SPECIAL = [0x0000, 0x0001, 0x8000, 0x7FFF, 0xFFFF, 0x8001, 0x00FF, 0xFF00]


def check(name, got, exp):
    bad = [(pe, g, e) for pe, (g, e) in enumerate(zip(got, exp)) if g != e]
    print(f"  {name:<40s} {'PASS' if not bad else f'FAIL ({len(bad)} PEs)'}")
    for pe, g, e in bad[:6]:
        print(f"      PE{pe}: got 0x{g:04X}  expected 0x{e:04X}")
    return not bad


def run_op(ser, op, src, imm, x):
    """acc = A, b = B, run the op, store acc to RES and C (0/1) to CARRY."""
    upload_and_run(ser, [
        ctx(CTX_ALWAYS),
        ldm(B), SWP, ldm(A),
        alu(op, src, imm, x=x),
        stm(RES),
        *ldi16(0), stm(CARRY),       # LDI leaves C alone
        ctx(CTX_IF_C), *ldi16(1), stm(CARRY),
        ctx(CTX_ALWAYS),
    ])
    return read_mem_grid(ser, RES), read_mem_grid(ser, CARRY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--rounds", type=int, default=3, help="random data sets")
    args = ap.parse_args()
    rng = random.Random(args.seed)
    ok = True

    def word():
        return rng.choice(SPECIAL) if rng.random() < 0.2 else rng.getrandbits(16)

    names = {ALU_SHLN: "SHLN", ALU_SHRN: "SHRN", ALU_ASRN: "ASRN", ALU_ROL: "ROL", ALU_MULH: "MULH"}
    xnames = {ALU_SHLN: "SHLN funnel", ALU_SHRN: "SHRN funnel", ALU_ROL: "ROR", ALU_MULH: "MULHU"}

    with open_dm16(args.port) as ser:
        for rnd in range(args.rounds):
            a = [word() for _ in range(NUM_PE)]
            b = [word() for _ in range(NUM_PE)]
            # b's low nibble is a per-PE shift amount, so cover 0..15 evenly
            b = [(v & ~0xF) | (pe & 0xF) for pe, v in enumerate(b)]
            write_words(ser, A, a)
            write_words(ser, B, b)
            print(f"Round {rnd + 1}")

            cases = []
            for op in (ALU_SHLN, ALU_SHRN, ALU_ASRN, ALU_ROL):
                n = rng.randint(1, 15)
                for x in ((False, True) if op != ALU_ASRN else (False,)):
                    cases.append((op, x, SRC_IMM, n))
                    cases.append((op, x, SRC_B, 0))
            for x in (False, True):
                cases.append((ALU_MULH, x, SRC_B, 0))
                cases.append((ALU_MULH, x, SRC_IMM, rng.getrandbits(10)))

            for op, x, src, imm in cases:
                opb = [((imm | 0xFC00) if imm & 0x200 else imm) if src == SRC_IMM else bv for bv in b]
                exp = [alu_ref(op, av, ob, bv, x) for av, ob, bv in zip(a, opb, b)]
                got_r, got_c = run_op(ser, op, src, imm, x)
                label = (xnames if x else names)[op] + (f" #{imm}" if src == SRC_IMM else " b")
                ok &= check(label, got_r, [e[0] for e in exp])
                ok &= check(label + " carry", got_c, [e[1] for e in exp])

        # Fixed point: Q8.8 product of a and b = ({hi, lo} >> 8)[15:0],
        # i.e. a funnel shift of the high half (MULH) over the low half (MUL)
        a = [rng.randint(-0x800, 0x7FF) & 0xFFFF for _ in range(NUM_PE)]   # -8.0 .. 8.0
        b = [rng.randint(-0x800, 0x7FF) & 0xFFFF for _ in range(NUM_PE)]
        write_words(ser, A, a)
        write_words(ser, B, b)
        upload_and_run(ser, [
            ctx(CTX_ALWAYS),
            ldm(A), alu(ALU_MUL, SRC_MEM, B), stm(TMP),      # low half
            ldm(A), alu(ALU_MULH, SRC_MEM, B),               # high half
            SWP, ldm(TMP),                                   # b = hi, acc = lo
            alu(ALU_SHRN, SRC_IMM, 8, x=True),               # acc = ({hi,lo} >> 8)
            stm(RES),
        ])
        exp = [((to_signed(av) * to_signed(bv)) >> 8) & 0xFFFF for av, bv in zip(a, b)]
        print("Fixed point")
        ok &= check("Q8.8 multiply (MUL, MULH, funnel SHRN)", read_mem_grid(ser, RES), exp)

    print("\nALL PASS" if ok else "\nFAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
