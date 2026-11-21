"""
DM16 basic hardware tests: local memory + ALU, NEWS torus, hypercube network.
"""
import serial

from dm16_host import (
    DEFAULT_PORT, open_dm16,
    NUM_PE, ROWS, COLS, PE_DIMS,
    ALU_ADD, ALU_MOV, ALU_MUL, SRC_B, SRC_ID, SRC_IMM, CTX_ALWAYS, CTX_IF_ID,
    NEWS_E, NEWS_S, DEST_IMM, DEST_XOR, OP_NEWS,
    NOP, RECV, SWP, alu, ctx, ldm, stm, ldi16, send, encode_instr,
    read_mem_grid, upload_and_run, write_words, print_grid,
)

LAST = NUM_PE - 1   # PE furthest from PE 0 in the hypercube


def report(name, got, exp):
    ok = got == exp
    print(f"  {name}: {'PASS' if ok else 'FAIL'}")
    if not ok:
        print(f"    got      {got}\n    expected {exp}")
    return ok


def main(port=None):
    ok = True
    port = port or DEFAULT_PORT
    with open_dm16(port) as ser:
        # ------------------------------------------------------------------
        # 1. Local memory + ALU: mem[0x011] = seed + PE_ID
        # ------------------------------------------------------------------
        seeds = [1000] * NUM_PE
        seeds[0], seeds[LAST] = 9000, 30000
        write_words(ser, 0x010, seeds)
        upload_and_run(ser, [ctx(CTX_ALWAYS), ldm(0x010), alu(ALU_ADD, SRC_ID), stm(0x011)])
        got = read_mem_grid(ser, 0x011)
        print_grid("Test 1: MEM[0x011] = seed + PE_ID", got)
        ok &= report("memory + ALU", got, [s + i for i, s in enumerate(seeds)])

        # ------------------------------------------------------------------
        # 2. NEWS torus: acc = PE_ID, shift East then South
        # ------------------------------------------------------------------
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), alu(ALU_MOV, SRC_ID),
            encode_instr(OP_NEWS, sub_op=NEWS_E),
            encode_instr(OP_NEWS, sub_op=NEWS_S),
            stm(0x020),
        ])
        got = read_mem_grid(ser, 0x020)
        print_grid("Test 2: NEWS torus (each PE holds the PE at row+1, col+1)", got)
        exp = [((r + 1) % ROWS) * COLS + (c + 1) % COLS for r in range(ROWS) for c in range(COLS)]
        ok &= report("NEWS", got, exp)

        # ------------------------------------------------------------------
        # 3. Point-to-point: PE 0 <-> last PE across every hypercube dimension
        # ------------------------------------------------------------------
        upload_and_run(ser, [
            ctx(CTX_ALWAYS), *ldi16(0), stm(0x030),
            ctx(CTX_IF_ID, imm=0),    *ldi16(0xAAAA), SWP, send(LAST, DEST_IMM),
            ctx(CTX_IF_ID, imm=LAST), *ldi16(0x5555), SWP, send(0, DEST_IMM),
            *[NOP] * 30,
            ctx(CTX_ALWAYS), *ldi16(0), SWP, RECV, SWP, stm(0x030),
        ])
        got = read_mem_grid(ser, 0x030)
        print_grid(f"Test 3: PE0 <-> PE{LAST} exchange", got, fmt="0x{:04X}")
        exp = [0] * NUM_PE
        exp[0], exp[LAST] = 0x5555, 0xAAAA
        ok &= report("point-to-point", got, exp)

        # ------------------------------------------------------------------
        # 4. Cube swap: every PE sends PE_ID * 100 to PE_ID ^ (1 << d) at once
        # ------------------------------------------------------------------
        for d in range(PE_DIMS):
            upload_and_run(ser, [
                ctx(CTX_ALWAYS), alu(ALU_MOV, SRC_ID), alu(ALU_MUL, SRC_IMM, 100), SWP,
                send(1 << d, DEST_XOR), *[NOP] * 24,
                *ldi16(0), SWP, RECV, SWP, stm(0x040 + d),
            ])
            got = read_mem_grid(ser, 0x040 + d)
            ok &= report(f"cube swap, dimension {d}", got, [(i ^ (1 << d)) * 100 for i in range(NUM_PE)])

    print("\nALL PASS" if ok else "\nFAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
