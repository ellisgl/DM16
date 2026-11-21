"""
DM8 glitch localiser.

Runs the same small SIMD kernel many times and checks every PE afterwards.
The kernel mirrors the prefix-sum combine step: unconditional BRAM
read-modify-write, then a context-masked read-modify-write on odd PEs only.

Mode "once":   upload the kernel ONE time, then re-run it N times with only
               the 3-byte RUN command (almost no UART traffic).
Mode "reload": re-upload the kernel before every run (heavy UART traffic,
               like dm8_prefix_sum.py does).

If errors show up only in "reload" mode, uploads are being corrupted
(UART / BL616 bridge). If they show up in "once" mode as well, the fault
is on-chip (PE / BRAM / context logic).

Expected after N runs:  MEM_A = N on every PE
                        MEM_B = N on odd PEs, 0 on even PEs
"""
import argparse
import time

import serial

from dm8_host import (
    DEFAULT_PORT, open_dm8,
    NUM_PE, OP_ALU, OP_LDI, OP_LDM, OP_SET_CTX, OP_STM, ALU_ADD, ALU_AND,
    encode_instr, inject_instr, read_mem_grid, upload_and_run, print_grid,
)

MEM_A = 0x020
MEM_B = 0x022

KERNEL = [
    encode_instr(OP_SET_CTX, sub_op=0x0),                      # all PEs on
    encode_instr(OP_LDM, imm10=MEM_A),
    encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x1, imm10=1),
    encode_instr(OP_STM, imm10=MEM_A),
    encode_instr(OP_ALU, sub_op=0x8, src_sel=0x2),              # acc = PE_ID
    encode_instr(OP_ALU, sub_op=ALU_AND, src_sel=0x1, imm10=1),
    encode_instr(OP_SET_CTX, sub_op=0x3),                      # odd PEs only
    encode_instr(OP_LDM, imm10=MEM_B),
    encode_instr(OP_ALU, sub_op=ALU_ADD, src_sel=0x1, imm10=1),
    encode_instr(OP_STM, imm10=MEM_B),
    encode_instr(OP_SET_CTX, sub_op=0x0),
]


def rerun(ser, length):
    ser.write(b"\x04")                              # reset PC
    ser.write(bytes([0x02, length >> 8, length & 0xFF]))
    ser.flush()
    time.sleep(0.002)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--mode", choices=["once", "reload"], default="once")
    ap.add_argument("-n", type=int, default=200, help="runs (max 255)")
    args = ap.parse_args()
    n = min(args.n, 255)

    with open_dm8(args.port) as ser:
        inject_instr(ser, encode_instr(OP_SET_CTX, sub_op=0x0))
        inject_instr(ser, encode_instr(OP_LDI, imm10=0))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_A))
        inject_instr(ser, encode_instr(OP_STM, imm10=MEM_B))

        if args.mode == "once":
            upload_and_run(ser, KERNEL)
            for _ in range(n - 1):
                rerun(ser, len(KERNEL))
        else:
            for _ in range(n):
                upload_and_run(ser, KERNEL)
        time.sleep(0.05)

        a = read_mem_grid(ser, MEM_A)
        b = read_mem_grid(ser, MEM_B)

    print_grid(f"MEM_A (expect {n} everywhere)", a)
    print_grid(f"MEM_B (expect {n} on odd PEs, 0 on even)", b)

    bad = []
    for pe in range(NUM_PE):
        exp_b = n if pe & 1 else 0
        if a[pe] != n or b[pe] != exp_b:
            bad.append((pe, a[pe], b[pe], exp_b))
    print(f"\nmode={args.mode} runs={n}: {len(bad)} bad PE(s)")
    for pe, va, vb, eb in bad:
        print(f"  PE{pe:2d} (cluster {pe >> 2}, lane {pe & 3}, DPB port {'B' if pe & 1 else 'A'}):"
              f" A={va} (exp {n})  B={vb} (exp {eb})")


if __name__ == "__main__":
    main()
