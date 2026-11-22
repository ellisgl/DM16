# Scaling DM16

DM16 on the Tang Console 138K is 64 PEs of 16 bits each, and it fills most of
that chip. The design is parameterized so it can grow on a bigger FPGA, or
across several boards. This page covers each way it can scale: what is
already a parameter, what it costs, and what would have to change.

## The knobs

| Parameter | Where | Today | What it sets |
|---|---|---|---|
| `W` | `dm16_core` (passed down) | 16 | PE data width: registers, ALU, memory words, NEWS and network data |
| `DIMS` | `dm16_core` | 4 | hypercube dimensions between clusters: 2^`DIMS` clusters |
| `LANES` | `dm16_core` | 4 | PEs per cluster (2 or 4) |
| `COLS` | `dm16_core` | 8 | width of the NEWS torus; rows = PEs / `COLS` |
| `NREGS` | `dm16_core` | 8 | registers `r0`–`r[NREGS-1]` per PE (`r0` is `b`): a power of two, 2–1,024 |
| `RF_BRAM` | `dm16_core` | 0 | registers in flip-flops (0) or in a block RAM (1) |
| `MEM_AW` | `dm16_core` | 10 | PE memory = 2^`MEM_AW` words (1,024 on the Tang) |
| `GOWIN_SP` | `dm16_core` | 1 | PE memories from Gowin `SP` block RAMs (1) or inferred RAM (0, for any other FPGA) |
| `HAS_RF` | `dm16_pe` | 1 | registers `r1` and up; without them every `r[n]` is `b` |
| `HAS_FPU` | `dm16_core` | 1 (0 on the Nano 9K) | bf16 FPU in every PE; without it FPU instructions are no-ops |
| `HAS_HDMI` | `dm16_top` (Console), `dm16_core` | 1 | 320×240 RGB332 frame buffer on HDMI and the `FBX`/`FBY`/`FBW` ops; without it they are no-ops (Console only: the Nanos lack the block RAM) |

PEs = 2^`DIMS` × `LANES`. The Tang build sets all of these in `dm16_top.v`.
The host library has matching constants (`DIMS`, `LANES`, `COLS`,
`WORD_BITS`, `NUM_REGS`, `MEM_WORDS` in `dm16_host.py`), which **must agree
with the bitstream**: nothing checks this at run time, and a mismatch shows
up as wrong results, not an error.

## Measured costs

Per-PE logic, from Yosys synthesizing `dm16_pe` alone for AMD UltraScale+
(`synth_xilinx -family xcup`). Vivado's numbers will differ somewhat, but the
trend is reliable. Each PE includes its `W`/16 bfloat16 FPU lanes.

| `W` | LUTs | Flip-flops | DSP48E2 | Registers `r1`–`r7` |
|---|---|---|---|---|
| 16 | 980 | 300 | 2 | 2 LUTRAM cells |
| 32 | 1,920 | 528 | 6 | 3 LUTRAM cells |
| 64 | 4,222 | 984 | 20 | 5 LUTRAM cells |
| 128 | 9,751 | 1,896 | 64 | 10 LUTRAM cells |

Logic grows a little faster than `W` (the barrel shifter grows as *W* log *W*),
and the single-clock multiplier grows as *W*² in DSPs.

Shared logic is small by comparison:

| Block | LUTs | Flip-flops | Notes |
|---|---|---|---|
| Hypercube router, one per cluster, 4 dimensions | ~390 | ~340–370 | links are 4 bits wide, so it hardly depends on `W` |
| Hypercube router, 6 dimensions | ~540 | ~500–515 | |
| Sequencer and host interface | small | small | its 4096-instruction program memory is block RAM on Gowin (6 blocks); Yosys builds it from flip-flops because it is written inside a reset block, which would need restructuring for other vendors |

On the Tang Console itself (Gowin GW5AST-138, measured by the Gowin tools),
the whole 64 × 16-bit machine uses 72% of the logic, 87% of the logic slices,
23% of the flip-flops, 21% of the block RAM and 33% of the DSPs, with Fmax
58.3 MHz; the slowest path is inside the hypercube router, not the PEs. (A
one-packet buffer between each cluster's PEs and its router removed the
previous slowest path, the injection handshake.) The Tang Nano 20K (8 PEs
with FPUs) uses 58% of its GW2AR-18 at 54.2 MHz, and the Nano 9K (8
integer-only PEs) 82% of its GW1NR-9 at 48.9 MHz; on those the slowest paths
are in the PEs.

## Data width (`W`)

`W` can be any power of two from 16 upwards. Every instruction keeps its
meaning, scaled to the width:

* **Integer ops** (`ADD` … `ROL`) work on all `W` bits. Shift amounts are the
  operand modulo `W`. `MUL` gives the low `W` bits of the product and `MULH`
  the high `W` bits.
* **Immediates** are still 10 bits, sign-extended. `LDI` hi/lo still set bits
  15:8 and 7:0. Wider constants are built with `SHLN 8` then `OR imm8`, byte
  by byte.
* **Floating point** works per 16-bit lane: a 64-bit PE has four bf16 lanes,
  and `FADD`, `FMUL` and the rest act on all four at once (SIMD within the
  SIMD). `FCMP` compares lane 0 and sets the flags from it.
* **Memory, NEWS and the network** carry whole `W`-bit words. Network packets
  are `W` + 8 bits; the 4-bit links take (`W` + 8) / 4 beats per packet.
* **Readback**: the host's `acc` dump and bulk read send `W`/8 bytes per PE.

The RTL is checked at 16, 32, 64 and 128 bits against `dm16_model.py`, a
reference model of one PE at any width, on random programs of about 15,000
instructions, and the whole core (sequencer, network, UART) is simulated at 16
and 64 bits.

**What gets expensive**

* **The multiplier.** A single-clock `W` × `W` multiply needs about (`W`/16)²
  DSP multipliers: 20 DSP48E2s per PE at 64 bits, 64 at 128 bits, and about
  225 at 256 bits, more than most FPGAs have for a whole array. Above 64 bits,
  make `MUL`/`MULH` multi-cycle: a 64 × 64 multiplier used for 4 passes at
  128 bits or 16 at 256 bits, with the sequencer waiting as it does for
  `FADD`.
* **The FPU lanes** grow linearly with `W`. Where floating point is not
  needed, fewer lanes (or none) would save a lot of logic; making the lane
  count a separate parameter is a small change.
* **NEWS wiring**: four `W`-bit links per PE. At 256 bits that is 1,024 wires
  per PE, which can congest routing; a NEWS move could take several clocks
  over a narrower link instead.

**Which width?** Wide PEs suit big-number arithmetic (256-bit elliptic curves,
wide bit masks). For most SIMD algorithms (stencils, cellular automata,
sorting, Monte Carlo) more, narrower PEs use the same chip better: 1,024 PEs
× 32 bits beat 128 PEs × 256 bits for most work.

## Number of PEs (`DIMS`, `LANES`, `COLS`)

PEs = 2^`DIMS` × `LANES`, laid out on a torus `COLS` wide. Up to **64 PEs**
this is purely a matter of parameters. Beyond 64, a few hard-coded fields need
widening:

| Limit | Where | Change for more than 64 PEs |
|---|---|---|
| `PE_ID` is 6 bits | `dm16_pe`, `dm16_cluster` | widen to `log2(PEs)` bits |
| packet destination is 6 bits | `dm16_cluster`, `dm16_core` | widen with `PE_ID`; packets grow by the same amount |
| `SEND` takes its destination from `imm[5:0]` or `acc[5:0]` | `dm16_pe` | use more bits; `imm10` allows up to 1,024 PEs |
| `CTX IF_ID` compares 8 bits | `dm16_pe` | fine up to 256 PEs; beyond that compare more `imm10` bits |
| `LANES` is 2 or 4 | `LANE_BITS` in `dm16_cluster`, `dm16_core` | generalize `LANE_BITS` to `log2(LANES)` |

Other things that grow with the PE count:

* **Instruction broadcast.** Each instruction goes through two register stages
  (one per cluster, one per PE). Large arrays need a deeper tree, one stage
  per fan-out of about 8–16. The PE's early decode reads the stage before its
  own, so adding stages above the cluster changes nothing in the PE.
* **Global OR** (for `BANY`/`BNONE`). It is a two-stage OR tree today, and the
  sequencer waits `GOR_WAIT` = 8 clocks before testing it. A bigger tree needs
  more stages and a matching `GOR_WAIT`.
* **Hypercube.** Each extra dimension doubles the clusters and adds one link
  per router (~75 LUTs each). The worst-case route is `DIMS` hops.
* **NEWS torus.** Any `COLS` that divides the PE count works. A square torus
  (`COLS` = √PEs) keeps diagonal distances short.
* **Readback.** The host's `acc` dump is `W`/8 × PEs bytes. Over the Tang's
  USB bridge (~80 KB/s), a 1,024 × 64-bit machine would need 0.1 s per dump.
  Big arrays want a faster host link (see below).

### What each hypercube dimension buys

`DIMS` is not limited to the 4 used on the Tang, or the 6 in the 256-PE
example below; FPGA size is the limit. The Connection Machine CM-1/CM-2 used
12: 65,536 one-bit processors, 16 per router chip, so 4,096 routers in a
12-dimensional cube. A 12-dimensional DM16 with 4 PEs per cluster would have
16,384 PEs.

Each extra dimension:

| | Effect of +1 dimension |
|---|---|
| PEs | ×2, so peak throughput ×2 on data-parallel work |
| Worst-case route | +1 hop (diameter = `DIMS`; average distance `DIMS`/2) |
| Links per router | +1: ~75 LUTs per router on an FPGA, or one more cable per board |
| Total links | N × `DIMS` / 2, growing slightly faster than the node count |
| Bisection bandwidth | ×2: half the nodes always have a direct link across any cut through the middle, so bandwidth per PE stays constant as the machine grows |
| Collective operations | +1 step for a sum, broadcast, prefix sum or FFT stage |

Compute doubles with each dimension, while communication-heavy operations
take one more step: they grow as log₂ N. On a 2-D mesh of the same size, the
distance across, and so the time to sum or broadcast, grows as √N instead: at
65,536 nodes, 16 hypercube steps against hundreds of mesh hops. That is why
Thinking Machines chose the hypercube.

**In DM16 terms**, using `dm16_prefix_sum.py`: each dimension is one "cube
swap" step (`SEND` to `PE_ID ^ 2^d`, wait for delivery, `RECV`, add), about
30 instructions or roughly 1.5 µs at 50 MHz, estimated from the instruction
count. The 64 PEs on the Tang need 6 steps (4 hypercube dimensions plus 2 bits
for the 4 PEs in a cluster), about 9 µs. At 12 dimensions × 4 PEs per cluster
(16,384 PEs) a prefix sum would take 14 steps, about 21 µs: 256× the PEs for
about 2.3× the time.

Cube swaps are the network's best case: every PE sends across the same
dimension at once, each link carries exactly one packet each way, and nothing
collides. Arbitrary permutations contend for links and cost more per
dimension.

**Dimensions or lanes?** Doubling `LANES` (PEs per cluster) instead also
doubles the machine, more cheaply: no new links and no extra hops. But a
cluster's PEs share one router port, so more lanes means more PEs queuing to
send, and network bandwidth per PE falls. The CM-2's 16 per router was its
balance point for one-bit PEs; for 16–64-bit PEs, 4–8 per router is a
reasonable range.

**Inside one FPGA versus across boards.** On a single chip, higher dimensions
connect clusters that sit far apart on the die, so wiring delay and routing
congestion grow with each one; single-chip designs often prefer a 2-D mesh
for that reason. Across boards it is the other way round: each dimension is
just a cable, and the hypercube's short distances pay off fully (see
Chip-to-chip below).

**What fits where**, using the per-PE numbers above plus ~15% for routers,
broadcast and sequencer (AMD figures from memory; check the datasheets):

| Machine | LUTs needed | A comfortable fit (≤ ~75%) |
|---|---|---|
| 64 × 16 bits (today) | ~75 k | Gowin GW5AST-138 (Tang Console), AMD Kintex UltraScale+ KU3P class |
| 256 × 32 bits | ~570 k | AMD Virtex UltraScale+ VU9P (1.18 M LUTs) |
| 256 × 64 bits | ~1.25 M | AMD Virtex UltraScale+ VU13P (1.73 M), e.g. an Alveo U250 card |
| 1,024 × 32 bits | ~2.3 M | AMD VU19P class, or several boards |
| 256 × 256 bits | ~2–2.6 M ALMs, multi-cycle multiply | Intel Stratix 10 GX 10M class |

## Registers (`NREGS`, `RF_BRAM`)

Every PE has registers `r0`–`r[NREGS-1]`, where `r0` is the `b` register.
`NREGS` is a power of two from 2 to 1,024; the Tang build has 8. A register
is named by the instruction's 10-bit immediate field, so the instruction
format already allows 1,024, and programs written for fewer registers run
unchanged on a build with more.

There are two ways to build them:

* **Flip-flops** (`RF_BRAM = 0`, the default): an `NREGS`-way select in front
  of operand B. On AMD and Intel parts, synthesis puts these in distributed
  (LUT) RAM, so 32 registers cost the same as 8. On the Tang's Gowin chip,
  which has no distributed RAM, each register is `W` flip-flops plus select
  logic, so this suits small register files.
* **Block RAM** (`RF_BRAM = 1`): `r1` and up live in one block RAM per PE
  (two or more for very wide `W`). This is cheap in logic for any size up to
  1,024. It works without extra clocks because the PE reads operand B one
  clock early, from the copy of the next instruction on `instr_next`, which
  is exactly when a block RAM needs its address.

Measured per PE with Yosys for AMD UltraScale+ (`synth_xilinx -family xcup`):

| `W` | `NREGS` | Style | LUTs | Flip-flops | RAM |
|---|---|---|---|---|---|
| 16 | 8 | flip-flops | 984 | 300 | 2 LUTRAM cells |
| 16 | 32 | flip-flops | 978 | 300 | 2 LUTRAM cells |
| 64 | 8 | flip-flops | 4,248 | 984 | 5 LUTRAM cells |
| 64 | 32 | flip-flops | 4,232 | 984 | 5 LUTRAM cells |
| 64 | 256 | block RAM | 4,222 | 1,123 | 1 block RAM |

On the Tang Console (Gowin GW5AST-138, 64 PEs × 16 bits), measured by the
Gowin tools with 32 registers in block RAM and 4,096 words of memory per PE,
against the shipped 8 registers and 1,024 words:

| | 8 registers, 1 K words (shipped) | 32 registers (block RAM), 4 K words |
|---|---|---|
| Logic | 71% | 73% |
| Logic slices | 86% | 85% |
| Flip-flops | 31,554 (23%) | 27,296 (20%) |
| Block RAM | 70 / 340 | 326 / 340 (96%) |
| Fmax (50 MHz target) | 54.7 MHz | 54.1 MHz |

The larger configuration fits and meets timing; its slowest path is in the
hypercube router, not the register file. It does use almost all the block
RAM: 256 blocks of PE memory, 64 register files and 6 for the program.

**Caveats**

* **Register numbers wrap.** The hardware uses the register number modulo
  `NREGS`, so a program written for 32 registers that runs on an 8-register
  build silently uses `r[n mod 8]`. The host library's helpers (`put`, `get`,
  `swp`, `alu(..., SRC_R, n)`) raise an error for registers that don't exist
  in the build `NUM_REGS` describes, but hand-encoded instructions are not
  checked.
* **The block RAM's write is registered.** Gowin's block RAMs make spurious
  writes when their write enable glitches, so the register file's write
  enable comes from a flip-flop, and the write lands in the idle clock after
  `SWP`/`PUT`. That is the same edge at which the next instruction reads its
  operand, so a write and read of the same register on that edge are
  bypassed. Like the rest of the PE, this relies on the sequencer leaving an
  idle clock after every instruction.
* **The block RAM's output goes straight into the ALU.** With flip-flops,
  operand B comes from a register; with block RAM it comes from the RAM's
  output, which is slower, so expect a lower Fmax (measured above).
* **Gowin needs telling to use block RAM.** Without a
  `syn_ramstyle = "block_ram"` attribute (which `dm16_pe.v` sets), Gowin's
  synthesis builds a 32-entry register file from flip-flops: a first trial
  build used 61 k flip-flops and 98% of the logic slices instead of 27 k and
  85%. If you move the register file, keep the attribute, and check the
  block RAM count in the report.
* **Registers have no reset.** Their contents are undefined until written.

## Memory per PE (`MEM_AW`)

Each PE has 2^`MEM_AW` words; the Tang build has 1,024 (`MEM_AW` = 10).
`mar` is `MEM_AW` bits wide, so `MAR`, `LDX`/`STX` and post-increment reach
the whole memory.

**Caveats**

* **Immediate addresses reach only the first 1,024 words.** `LDM`/`STM` and
  memory operands take a 10-bit immediate address. Above 1,024, set `mar`
  (`MAR` from `acc`) and use `LDX`/`STX` or the `MEM_MAR` addressing flag.
  The host's `ldm`, `stm`, `alu` and `fpu` helpers raise an error for an
  immediate address above 1,023 instead of letting it wrap.
  A base-plus-offset addressing mode would remove this limit; it is not
  implemented.
* **`MAR` loads from `acc`, so `MEM_AW` ≤ `W`.** At 16 bits that allows up
  to 64 K words.
* **`mar` wraps** at 2^`MEM_AW`; with `MEM_AW` < 10, an immediate address
  uses its low `MEM_AW` bits.
* **Host reads and writes above 1 K change `mar`.** The bulk-read command
  (`0x0A`, `read_mem_block`) reads the first 1,024 words with `LDM`, leaving
  `mar` alone; a read that reaches past them sets `mar` and uses `LDX` with
  post-increment, so afterwards `mar` points just past the last word read.
  `write_words` above 1 K likewise leaves `mar` at the written address.
  Programs should set `mar` themselves rather than rely on its value from
  before a host access. (Both commands always clobber `acc` and enable every
  PE.)
* **Gowin banks.** With `GOWIN_SP` = 1, each 1,024 words is one Gowin `SP`
  block per 16 bits of width, and deeper memories are banked using the `SP`'s
  own block-select input (so the write enable stays a clean flip-flop
  signal). The block select is 3 bits, so `MEM_AW` ≤ 13 (8 K words) with
  Gowin RAMs; use `GOWIN_SP` = 0 for more.
* **Block RAM budget.** The Tang has 340 block RAMs; 6 hold the program and
  each 1 K × 16 of PE memory takes one. 64 PEs × 4 K words is 256 blocks,
  which fits; 8 K words per PE would not.

On AMD UltraScale+, one UltraRAM block (4K × 72) per PE gives 4,096 × 64-bit
words (32 KB) cheaply; on Intel, M20K blocks in parallel.

## Chip-to-chip

The design is a hypercube already, so several boards extend it naturally:
each board becomes a corner of a hypercube of boards.

* **16 boards** form a 4-dimensional hypercube of boards: each board needs 4
  links, 32 cables in all, for 16 × 64 = 1,024 PEs with 10-bit `PE_ID`s.
* **The same cables give a NEWS torus.** A 4-dimensional hypercube is a 4 × 4
  torus (it is a square of squares, and a square of 4 is a ring of 4). Each
  board is an 8 × 8 tile of a 32 × 32 torus, passing its edges to its N, E,
  W and S neighbour boards.
* **Links.** The Tang Console has two 2 × 20-pin headers and two PMOD
  connectors, roughly 60–70 usable I/O pins: enough for 4 links of 8–12 wires.
  The router already serializes packets onto 4-bit links, so a board-to-board
  link is the same protocol over a cable, with a synchronizer or small FIFO
  at each end. LVDS or the FPGA's transceivers would go faster.
* **Keeping boards in step.** Each board has its own oscillator. Either one
  master board sends every instruction (and its clock) to all the others,
  as the CM-2 did; or each board runs its own copy of the program and waits
  for the others only at network operations and `BANY`, as the CM-5 did.
  The second needs only a few shared wires and tolerates cable delays, so it
  is the place to start.
* **Global OR across boards.** One shared open-drain wire that any board pulls
  low while it has an active PE gives a machine-wide OR in tens of
  nanoseconds; `GOR_WAIT` grows a little.
* **NEWS across boards** moves 8 PEs × `W` bits per edge, so over a narrow
  cable it takes several clocks, and the sequencer waits for it.
* **Host I/O.** Connect every board's USB to the PC through a hub, so data
  loads and reads back in parallel instead of through one 80 KB/s link.
* **Getting there in steps**: one board with a loopback cable between its own
  headers tests the link logic; two boards test synchronization and the
  shared OR wire; four boards (a 2-D cube, also a 2 × 2 torus) exercise
  everything except scale.

## Other directions

* **A faster host link.** This is the biggest practical limit today: the
  Tang's USB bridge adds ~6 ms per request and forwards ~80 KB/s. A PCIe card
  such as an AMD Alveo gives gigabytes per second; Ethernet would be a middle
  ground.
* **Clock speed.** The PE's critical paths are short now (operand select and
  decode happen a clock early; the multiplier result is registered). The
  slowest path on the Tang is in the hypercube router. A modern AMD or Intel
  part should run this design at 200–300 MHz.
* **Memory bandwidth.** HBM parts (AMD VU37P / Alveo U280, Intel Agilex M)
  have gigabytes on-package; a DMA engine streaming into PE memories would
  let the array work on data sets far bigger than its on-chip memory.
* **Bit-serial PEs.** The original CM-1 used 65,536 one-bit PEs. A bit-serial
  DM16 PE (one bit of a 16-bit operation per clock) is a few dozen LUTs, so a
  large FPGA could hold tens of thousands of them: much slower per PE, but
  enormous parallelism for simple operations.
* **Per-PE addressing.** Today `MAR` gives each PE its own address register,
  but every PE reads the same immediate address otherwise. Indexed addressing
  (`mem[r[n] + imm]`) would make table lookups and gathers cheaper.
* **More registers, wider instructions.** Eight registers fit the 24-bit
  instruction format. A 32-bit format would allow 16–32 registers and
  three-operand instructions (`r[d] = r[a] OP r[b]`), at the cost of a
  larger program memory.
* **Floating-point formats.** At `W` ≥ 32, fp32 lanes (or one fp64 unit at
  `W` = 64) may serve better than bf16 lanes, using the FPGA's floating-point
  DSP modes where available (Intel's DSP blocks support fp32 natively).
