"""
Hypercube inclusive prefix sum (scan) across all DM8 PEs.

Each PE starts with 1, so the result in PE i is i + 1.

Per dimension d, every PE at once (CM-2 style "cube swap"):
  send TOT to partner PE_ID ^ (1 << d)     -- one SIMD SEND, all PEs
  receive the partner's TOT
  TOT += received
  if PE_ID bit d is set: SUM += received
"""
import serial

from dm8_host import (
    DEFAULT_PORT,
    NUM_PE, PE_DIMS,
    ALU_ADD, ALU_AND, ALU_MOV, SRC_B, SRC_ID, SRC_IMM, CTX_ALWAYS, CTX_IF_NZ, DEST_XOR,
    NOP, RECV, SWP, alu, ctx, ldm, stm, send, ldi16,
    inject_instr, read_mem_grid, upload_and_run, print_grid,
)

MEM_SUM = 0x010   # running inclusive prefix sum
MEM_TOT = 0x011   # total of the subcube this PE currently belongs to

LINK_WAIT_NOPS = 24   # covers both PEs of a cluster crossing the same link


def stage_program(dim):
    mask = 1 << dim
    return [
        ctx(CTX_ALWAYS),
        ldm(MEM_TOT), SWP,                 # b = TOT
        send(mask, DEST_XOR),              # every PE -> PE_ID ^ mask
        *[NOP] * LINK_WAIT_NOPS,
        RECV,                              # b = partner's TOT
        ldm(MEM_TOT), alu(ALU_ADD, SRC_B), stm(MEM_TOT),
        alu(ALU_MOV, SRC_ID), alu(ALU_AND, SRC_IMM, mask),
        ctx(CTX_IF_NZ),                    # upper half of this dimension
        ldm(MEM_SUM), alu(ALU_ADD, SRC_B), stm(MEM_SUM),
        ctx(CTX_ALWAYS),
    ]


def main(port=None):
    port = port or DEFAULT_PORT
    with serial.Serial(port, 115200, timeout=1) as ser:
        inject_instr(ser, ctx(CTX_ALWAYS))
        for ins in ldi16(1):
            inject_instr(ser, ins)
        inject_instr(ser, stm(MEM_SUM))
        inject_instr(ser, stm(MEM_TOT))

        print_grid(f"Initial Input Array (All {NUM_PE} PEs = 1)", read_mem_grid(ser, MEM_SUM))

        for dim in range(PE_DIMS):
            upload_and_run(ser, stage_program(dim))
            print_grid(
                f"After Hypercube Dimension {dim} (XOR mask 0x{1 << dim:02X})",
                read_mem_grid(ser, MEM_SUM),
            )

        got = read_mem_grid(ser, MEM_SUM)
        ok = got == list(range(1, NUM_PE + 1))
        print("\nPASS" if ok else f"\nFAIL: expected {list(range(1, NUM_PE + 1))}")
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
