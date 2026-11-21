"""
DM16 sequencer control-flow test: hardware loops, jumps, HALT and
data-dependent branches on the global OR of the PE context flags.
"""
import argparse

from dm16_host import (
    DEFAULT_PORT, open_dm16, NUM_PE,
    ALU_ADD, ALU_MOV, ALU_OR, ALU_SUB, SRC_ID, SRC_IMM, CTX_ALWAYS, CTX_IF_NZ, CTX_NEVER,
    LOOP, ENDL, JMP, BANY, BNONE, HALT, LABEL,
    alu, ctx, ldm, stm, ldi16, read_mem_grid, read_status, upload_and_run,
)

R_LOOP, R_NEST, R_LONG, R_JMP, R_HALT, R_CNT, R_STEPS, R_NONE = range(0x300, 0x308)


def check(name, got, exp):
    ok = got == exp
    print(f"  {name:<44s} {'PASS' if ok else 'FAIL'}")
    if not ok:
        print(f"      got      {got}\n      expected {exp}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    args = ap.parse_args()
    ok = True

    with open_dm16(args.port) as ser:
        print("Sequencer control flow")

        # 1. Simple loop: acc = PE_ID + 100
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), alu(ALU_MOV, SRC_ID),
            LOOP(100), alu(ALU_ADD, SRC_IMM, 1), ENDL,
            stm(R_LOOP),
        ])
        ok &= check("LOOP 100", read_mem_grid(ser, R_LOOP), [i + 100 for i in range(NUM_PE)])

        # 2. Nested loops: 10 x 7 = 70, plus an inner-loop-only counter
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(0),
            LOOP(10),
                LOOP(7), alu(ALU_ADD, SRC_IMM, 1), ENDL,
            ENDL,
            stm(R_NEST),
        ])
        ok &= check("nested LOOP 10 x LOOP 7", read_mem_grid(ser, R_NEST), [70] * NUM_PE)

        # 3. Long loop (exercises completion polling): 3 levels, 20 x 50 x 30 = 30000
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(0),
            LOOP(20), LOOP(50), LOOP(30), alu(ALU_ADD, SRC_IMM, 1), ENDL, ENDL, ENDL,
            stm(R_LONG),
        ])
        ok &= check("LOOP 20 x 50 x 30 (30000 iterations)", read_mem_grid(ser, R_LONG), [30000] * NUM_PE)

        # 4. JMP skips an instruction
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(5), stm(R_JMP),
            JMP("skip"),
            *ldi16(99), stm(R_JMP),          # must not run
            LABEL("skip"),
            ldm(R_JMP), alu(ALU_ADD, SRC_IMM, 1), stm(R_JMP),
        ])
        ok &= check("JMP over code", read_mem_grid(ser, R_JMP), [6] * NUM_PE)

        # 5. HALT stops the program early
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(7), stm(R_HALT),
            HALT,
            *ldi16(99), stm(R_HALT),         # must not run
        ])
        running, _ = read_status(ser)
        ok &= check("HALT", read_mem_grid(ser, R_HALT), [7] * NUM_PE)
        ok &= check("not running after HALT", running, False)

        # 6. Data-dependent loop on the global OR:
        #    each PE counts CNT = PE_ID + 1 down to zero; STEPS counts its
        #    own iterations; the loop runs while ANY PE still has CNT != 0.
        upload_and_run(ser, [
            ctx(CTX_ALWAYS),
            alu(ALU_MOV, SRC_ID), alu(ALU_ADD, SRC_IMM, 1), stm(R_CNT),
            *ldi16(0), stm(R_STEPS),
            LABEL("top"),
                ctx(CTX_ALWAYS), ldm(R_CNT), alu(ALU_OR, SRC_IMM, 0),
                ctx(CTX_IF_NZ),                                     # PEs still counting
                ldm(R_CNT), alu(ALU_SUB, SRC_IMM, 1), stm(R_CNT),
                ldm(R_STEPS), alu(ALU_ADD, SRC_IMM, 1), stm(R_STEPS),
                ldm(R_CNT), alu(ALU_OR, SRC_IMM, 0),
                ctx(CTX_IF_NZ),                                     # still active?
            BANY("top"),
            ctx(CTX_ALWAYS),
        ])
        ok &= check("BANY loop: per-PE iteration counts", read_mem_grid(ser, R_STEPS),
                    [i + 1 for i in range(NUM_PE)])
        ok &= check("BANY loop: all counters reached 0", read_mem_grid(ser, R_CNT), [0] * NUM_PE)

        # 7. BNONE: taken when every PE is inactive, not taken otherwise
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(0), stm(R_NONE),
            ctx(CTX_NEVER), BNONE("none1"),
            ctx(CTX_ALWAYS), *ldi16(99), stm(R_NONE),        # skipped
            LABEL("none1"),
            ctx(CTX_ALWAYS), BNONE("none2"),                 # not taken
            ldm(R_NONE), alu(ALU_ADD, SRC_IMM, 1), stm(R_NONE),
            LABEL("none2"),
        ])
        ok &= check("BNONE taken / not taken", read_mem_grid(ser, R_NONE), [1] * NUM_PE)

    print("\nALL PASS" if ok else "\nFAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
