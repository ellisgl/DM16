"""
DM16 16-bit PE / bfloat16 FPU board test.

Every PE gets its own operands, runs the same SIMD program, and the results
are checked bit-for-bit against dm16_bf16_ref (exact model).

  - FADD / FSUB / FMUL with a memory operand
  - a dependent chain: y = (a*b + a) * b
  - FCMP + SET_CTX (data-dependent masking on a float compare)
  - FI2F (int16 -> bf16, rounds above 256) / FF2I / FNEG / FABS
  - 16-bit integer MUL, ASR, and MAR-indirect post-increment addressing
"""
import argparse
import random

import serial

import dm16_bf16_ref as ref
from dm16_host import (
    DEFAULT_PORT, open_dm16, HAS_FPU, BOARD,
    NUM_PE,
    ALU_ASR, ALU_MOV, ALU_MUL, ALU_SUB, SRC_ID, SRC_IMM, SRC_MEM, MEM_MAR, MEM_INC,
    CTX_ALWAYS, CTX_IF_N,
    F_ABS, F_ADD, F_CMP, F_F2I, F_I2F, F_MUL, F_NEG, F_SUB,
    OP_MAR, alu, fpu, ctx, ldm, stm, ldi16, encode_instr,
    read_mem_grid, upload_and_run, write_words, print_grid, bf16_to_float, to_signed,
)

A, B = 0x100, 0x101
SUM, DIFF, PROD, CHAIN, LT, CONV, F2I, NEG, ABS, IMUL, ASR = range(0x110, 0x11B)
ARR = 0x200   # 4-word array per PE for the MAR test


def random_bf16(rng):
    if rng.random() < 0.1:
        return rng.choice([0x0000, 0x8000, 0x3F80, 0xBF80, 0x7F7F, 0x0080, 0x4300, 0xC300])
    return (rng.getrandbits(1) << 15) | (rng.randint(100, 150) << 7) | rng.getrandbits(7)


def check(name, got, exp, fmt=lambda v: f"0x{v:04X}"):
    bad = [(pe, g, e) for pe, (g, e) in enumerate(zip(got, exp)) if g != e]
    print(f"  {name:<36s} {'PASS' if not bad else f'FAIL ({len(bad)} PEs)'}")
    for pe, g, e in bad[:6]:
        print(f"      PE{pe}: got {fmt(g)}  expected {fmt(e)}")
    return not bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--show", action="store_true", help="print value grids")
    args = ap.parse_args()
    if not HAS_FPU:
        print(f"SKIPPED: the {BOARD} build has no FPU (FPU instructions are no-ops)")
        return 0
    rng = random.Random(args.seed)

    a = [random_bf16(rng) for _ in range(NUM_PE)]
    b = [random_bf16(rng) for _ in range(NUM_PE)]
    arr = [[rng.getrandbits(16) for _ in range(NUM_PE)] for _ in range(4)]
    ok = True

    with open_dm16(args.port) as ser:
        write_words(ser, A, a)
        write_words(ser, B, b)
        for k in range(4):
            write_words(ser, ARR + k, arr[k])

        mem = lambda addr: dict(src=SRC_MEM, imm=addr)
        prog = [
            ctx(CTX_ALWAYS),
            ldm(A), fpu(F_ADD, **mem(B)), stm(SUM),
            ldm(A), fpu(F_SUB, **mem(B)), stm(DIFF),
            ldm(A), fpu(F_MUL, **mem(B)), stm(PROD),
            ldm(A), fpu(F_MUL, **mem(B)), fpu(F_ADD, **mem(A)), fpu(F_MUL, **mem(B)), stm(CHAIN),
            *ldi16(0), stm(LT),
            ldm(A), fpu(F_CMP, **mem(B)), ctx(CTX_IF_N), *ldi16(1), stm(LT), ctx(CTX_ALWAYS),
            alu(ALU_MOV, SRC_ID), alu(ALU_SUB, SRC_IMM, 4), alu(ALU_MUL, SRC_IMM, 300),
            fpu(F_I2F), stm(CONV),
            ldm(A), fpu(F_F2I), stm(F2I),
            ldm(A), fpu(F_NEG), stm(NEG),
            ldm(A), fpu(F_ABS), stm(ABS),
            alu(ALU_MOV, SRC_ID), alu(ALU_SUB, SRC_IMM, 4), alu(ALU_MUL, SRC_IMM, -77 & 0x3FF), stm(IMUL),
            alu(ALU_ASR), stm(ASR),
            # MAR test: reverse the 4-word array in place via post-increment reads
            encode_instr(OP_MAR, src_sel=1, imm10=ARR),
            ldm(mem=MEM_MAR | MEM_INC), stm(ARR + 0x13),
            ldm(mem=MEM_MAR | MEM_INC), stm(ARR + 0x12),
            ldm(mem=MEM_MAR | MEM_INC), stm(ARR + 0x11),
            ldm(mem=MEM_MAR | MEM_INC), stm(ARR + 0x10),
        ]
        upload_and_run(ser, prog)

        conv_in = [((pe - 4) * 300) & 0xFFFF for pe in range(NUM_PE)]
        imul = [((pe - 4) * -77) & 0xFFFF for pe in range(NUM_PE)]
        canon = lambda x: x if x & 0x7F80 else x & 0x8000
        exp = {
            "FADD a + b": [ref.add(x, y) for x, y in zip(a, b)],
            "FSUB a - b": [ref.add(x, y, sub=True) for x, y in zip(a, b)],
            "FMUL a * b": [ref.mul(x, y) for x, y in zip(a, b)],
            "chain (a*b + a) * b": [ref.mul(ref.add(ref.mul(x, y), x), y) for x, y in zip(a, b)],
            "FCMP a < b -> SET_CTX IF_N": [int(ref.bf2f(canon(x)) < ref.bf2f(canon(y))) for x, y in zip(a, b)],
            "FI2F (PE_ID - 4) * 300": [ref.i2f(v) for v in conv_in],
            "FF2I int16(a)": [ref.f2i(x) for x in a],
            "FNEG -a": [x ^ 0x8000 for x in a],
            "FABS |a|": [x & 0x7FFF for x in a],
            "MUL (PE_ID - 4) * -77": imul,
            "ASR of that": [(to_signed(v) >> 1) & 0xFFFF for v in imul],
        }
        addrs = [SUM, DIFF, PROD, CHAIN, LT, CONV, F2I, NEG, ABS, IMUL, ASR]
        print("16-bit PE / bfloat16 FPU")
        got = {}
        for (name, e), addr in zip(exp.items(), addrs):
            got[name] = read_mem_grid(ser, addr)
            ok &= check(name, got[name], e)
        for k in range(4):
            ok &= check(f"MAR post-increment, word {k}", read_mem_grid(ser, ARR + 0x10 + k), arr[3 - k])

        if args.show:
            fl = lambda bits: [bf16_to_float(x) for x in bits]
            print_grid("a", fl(a), fmt="{:10.4g}")
            print_grid("b", fl(b), fmt="{:10.4g}")
            print_grid("a + b (DM16)", fl(got["FADD a + b"]), fmt="{:10.4g}")
            print_grid("a * b (DM16)", fl(got["FMUL a * b"]), fmt="{:10.4g}")

    print("\nALL PASS" if ok else "\nFAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
