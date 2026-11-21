"""
A small chat model designed for DM16 (train it with dm16_chat_train.py).

Credit: this builds on Z80-μLM by HarryR (https://github.com/HarryR/z80ai),
a chat model for the Z80. Its input encoding (query trigrams and context
n-grams hashed into buckets), the character-at-a-time design and the
tinychat training data come from that project. The network, its 4-bit
integer arithmetic, the trainer and the DM16 program are new.

It uses the same inputs as Z80-μLM, so the two can be compared on the same
data: 128 buckets of query trigram counts and 128 buckets of 1-, 2-
and 3-grams of the last 8 output characters. The network is
256 -> 256 -> 128 -> charset with 4-bit weights, and its arithmetic is what
DM16 computes, so the board and the model agree exactly:

  x       = 16 * bucket count                         (layer-1 inputs)
  acc     = sum(w * x) + bias, wrapping at 16 bits    (w in -8..7: MUL + ADD)
  y       = acc >> shift (arithmetic)                 (shift chosen per layer)
  hidden  = clamp(y, 0, 127)
  output  = y; the character is the first maximum

This module holds the model format, the encoders, the integer model and
greedy generation, and the DM16 program that runs a whole reply on the board.

  python3 dm16_chat.py                                # chat (Tang Console)
  python3 dm16_chat.py --query "hello" --check        # one reply, checked step by step
  python3 dm16_chat.py --test-file data.txt.gz        # board replies vs the integer model
"""
import gzip
import json

import numpy as np

M16 = 0xFFFF
INPUT_SCALE = 16          # layer-1 input = 16 * count
ACT_MAX = 127             # hidden activations are clamped to 0..ACT_MAX
W_MIN, W_MAX = -8, 7      # 4-bit weights
CTX_LEN = 8
MAX_REPLY = 30


# ==============================================================================
# Data and encoding (the same hashing as z80ai's Z80 program)
# ==============================================================================
def read_pairs(path):
    """input|response lines (gzip ok); '#' lines are comments."""
    opener = gzip.open if path.endswith(".gz") else open
    pairs = []
    with opener(path, "rt", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "|" not in line:
                continue
            q, r = line.split("|", 1)
            q, r = q.strip()[:62], r.strip().upper()[:MAX_REPLY]
            if q and r:
                pairs.append((q, r))
    return pairs


def build_charset(pairs):
    """Space, letters, digits, punctuation, then EOS (as z80ai)."""
    chars = set("".join(r for _, r in pairs))
    letters = sorted(c for c in chars if c.isalpha())
    digits = sorted(c for c in chars if c.isdigit())
    punct = sorted(c for c in chars if not c.isalnum() and c != " ")
    return (" " if " " in chars else "") + "".join(letters + digits + punct) + "\x00"


def lower(c):
    """Only A-Z are lowercased (as the Z80 code, and DM16's context update)."""
    return chr(ord(c) + 32) if "A" <= c <= "Z" else c


def _hash(chars, seed=0):
    h = seed & M16
    for c in chars:
        h = (h * 31 + ord(c)) & M16
    return h & 127


def query_counts(text):
    """Trigram bucket counts of ' ' + text + ' ' (leading spaces skipped)."""
    counts = [0] * 128
    text = text.lstrip(" ")
    if text:
        chars = " " + "".join(lower(c) for c in text) + " "
        for i in range(len(chars) - 2):
            counts[_hash(chars[i:i + 3])] += 1
    return counts


def context_counts(ctxchars):
    """1-, 2- and 3-gram bucket counts of the last 8 output characters."""
    counts = [0] * 128
    for n in (1, 2, 3):
        for pos in range(CTX_LEN - n + 1):
            counts[_hash(ctxchars[pos:pos + n], pos * 7)] += 1
    return counts


def training_examples(pairs, charset):
    """(256 bucket counts, next-character index) for every character of every
    response, plus EOS."""
    idx = {c: i for i, c in enumerate(charset)}
    eos = len(charset) - 1
    xs, ys = [], []
    for q, r in pairs:
        qc = query_counts(q)
        ctx = " " * CTX_LEN
        for ch in r + "\x00":
            xs.append(qc + context_counts(ctx))
            ys.append(eos if ch == "\x00" else idx[ch])
            if ch != "\x00":
                ctx = ctx[1:] + lower(ch)
    return np.array(xs, dtype=np.int64), np.array(ys, dtype=np.int64)


# ==============================================================================
# Model file and the exact integer model
# ==============================================================================
def save_model(path, layers, charset, meta=None):
    """layers: [(W int (out, in) in -8..7, B int16 (out,), shift)]."""
    d = {"_charset": np.frombuffer(charset.encode("utf-8"), dtype=np.uint8),
         "_meta": np.frombuffer(json.dumps(meta or {}).encode("utf-8"), dtype=np.uint8),
         "_shifts": np.array([sh for _, _, sh in layers], dtype=np.int64)}
    for i, (w, b, _) in enumerate(layers):
        d[f"w{i}"] = np.asarray(w, dtype=np.int8)
        d[f"b{i}"] = np.asarray(b, dtype=np.int16)
    np.savez_compressed(path, **d)


def load_model(path):
    d = np.load(path, allow_pickle=False)
    charset = bytes(d["_charset"]).decode("utf-8")
    shifts = [int(s) for s in d["_shifts"]]
    layers = [(d[f"w{i}"].astype(np.int64), d[f"b{i}"].astype(np.int64), sh)
              for i, sh in enumerate(shifts)]
    meta = json.loads(bytes(d["_meta"]).decode("utf-8") or "{}")
    return layers, charset, meta


def int_forward(layers, counts, stats=None):
    """Batch of bucket counts (n, 256) -> output values (n, chars), exactly as
    DM16: 16-bit wraparound sums, arithmetic shift, hidden clamp to 0..127."""
    x = np.asarray(counts, dtype=np.int64) * INPUT_SCALE
    for li, (w, b, sh) in enumerate(layers):
        acc = x @ w.T + b
        if stats is not None:
            stats["overflow"] = stats.get("overflow", 0) + int(np.sum(np.abs(acc) > 32767))
            stats["sums"] = stats.get("sums", 0) + acc.size
        acc = ((acc + 0x8000) & M16) - 0x8000          # 16-bit wraparound, signed
        y = acc >> sh                                  # arithmetic shift (floor)
        x = np.clip(y, 0, ACT_MAX) if li < len(layers) - 1 else y
    return x


def first_argmax(v):
    """Index of the first maximum (rows of a batch)."""
    return np.argmax(v, axis=-1)                        # numpy picks the first


def generate_batch(layers, charset, queries, max_len=MAX_REPLY, logits_fn=None):
    """Greedy replies for many queries at once."""
    logits_fn = logits_fn or (lambda c: int_forward(layers, c))
    eos = len(charset) - 1
    qcs = [query_counts(q) for q in queries]
    ctx = [" " * CTX_LEN] * len(queries)
    out = [""] * len(queries)
    live = list(range(len(queries)))
    for _ in range(max_len):
        if not live:
            break
        counts = np.array([qcs[i] + context_counts(ctx[i]) for i in live])
        pick = first_argmax(logits_fn(counts))
        nxt = []
        for i, k in zip(live, pick):
            if k == eos:
                continue
            out[i] += charset[k]
            ctx[i] = ctx[i][1:] + lower(charset[k])
            nxt.append(i)
        live = nxt
    return out


def evaluate(layers, charset, pairs):
    """Exact-reply rate over distinct queries (any of a query's responses
    counts), and character accuracy with the true previous characters."""
    answers = {}
    for q, r in pairs:
        answers.setdefault(q, set()).add(r)
    queries = sorted(answers)
    got = generate_batch(layers, charset, queries)
    exact = sum(g in answers[q] for q, g in zip(queries, got)) / len(queries)
    x, y = training_examples(pairs, charset)
    char_acc = float(np.mean(first_argmax(int_forward(layers, x)) == y))
    return exact, char_acc


# ==============================================================================
# DM16: the whole reply on the board
# ==============================================================================
# Per-PE memory (1024 words)
XA, XB = 0x000, 0x100      # layer-1 input (query from the host, context built on board); gathered inputs
W_BASE = 0x200             # 4-bit weights: 4 nibbles per word
CHARTAB = 0x3A8            # lowercased character codes, for the context
CTXC = 0x3D0               # the last 8 output characters
# GOT, POS, RCH, LOGIT are adjacent so one 4-word read returns them all
(GOT, POS, RCH, LOGIT, XPTR, WPTR, SPTR, DPTR, MAXL, BV, PV, BK, PK, WIN) = range(0x3D8, 0x3E6)
PAD = -16384               # output value of the PEs beyond the charset
LINK_WAIT_NOPS = 24        # cube swap: four PEs of a cluster share a link
WAIT_UNROLL = 3
PROLOGUE_MAX = 640         # imem words reserved for the per-request prologue


def plan(layers, num_pe=64):
    """Per layer: (inputs, slots per PE, inputs per word, weight base, bias base)."""
    out, wbase = [], W_BASE
    biases = [(-(-w.shape[0] // num_pe)) for w, _, _ in layers]
    bbase = CHARTAB - sum(biases)
    for (w, _, sh), slots in zip(layers, biases):
        n_out, n_in = w.shape
        if slots > 4 or 4 % slots:
            raise ValueError("each layer needs 64, 128 or 256 outputs (1, 2 or 4 per PE)")
        ipw = 4 // slots
        out.append((n_in, slots, ipw, wbase, bbase, sh))
        wbase += n_in // ipw
        bbase += slots
    if wbase > CHARTAB - sum(biases):
        raise ValueError("model too big for PE memory")
    return out


def pe_words(layers, charset, pl, num_pe=64):
    """{address: value} for every PE: weights, biases, character table."""
    data = [dict() for _ in range(num_pe)]
    table = {CHARTAB + i: ord(lower(c)) if c != "\x00" else 0 for i, c in enumerate(charset)}
    for (w, b, _), (n_in, slots, ipw, wbase, bbase, _) in zip(layers, pl):
        n_out = w.shape[0]
        for p in range(num_pe):
            for k in range(n_in // ipw):
                word = 0
                for u in range(ipw):
                    for s in range(slots):
                        j = p + num_pe * s
                        nib = int(w[j, k * ipw + u]) & 0xF if j < n_out else 0
                        word |= nib << (4 * (u * slots + s))
                data[p][wbase + k] = word
            for s in range(slots):
                j = p + num_pe * s
                data[p][bbase + s] = int(b[j]) & M16 if j < n_out else 0
            data[p].update(table)
    return data


def _program(layers, charset, pl):
    """The generation loop: one reply per run."""
    from dm16_host import (ALU_ADD, ALU_ADC, ALU_SUB, ALU_AND, ALU_XOR, ALU_MOV, ALU_MUL, ALU_SHLN,
                           ALU_ASRN, SRC_R, SRC_IMM, SRC_ID, SRC_MEM, CTX_ALWAYS, CTX_IF_N,
                           CTX_IF_NZ, CTX_IF_Z, CTX_INVERT, OP_LDI, OP_MAR, OP_LDX, OP_STX,
                           MEM_INC, DEST_XOR, LOOP, ENDL, BANY, LABEL, NOP, RECV,
                           alu, ctx, ldm, stm, put, get, send, encode_instr, ldi16)
    num_pe = 64
    a = lambda op, n: alu(op, SRC_R, n)
    i_ = lambda op, v: alu(op, SRC_IMM, v & 0x3FF)
    mem = lambda op, addr: alu(op, SRC_MEM, addr)
    MAR = encode_instr(OP_MAR)
    LDX = encode_instr(OP_LDX)
    LDX_INC = encode_instr(OP_LDX, flag_ctl=MEM_INC)
    STX = encode_instr(OP_STX)
    STX_INC = encode_instr(OP_STX, flag_ctl=MEM_INC)
    LDI0 = encode_instr(OP_LDI)
    eos = len(charset) - 1
    dims = (num_pe - 1).bit_length()

    def exchange(word, into, d):
        """Send memory word `word` to PE_ID ^ 2^d; store the partner's in `into`."""
        return [ldm(word), put(0), send(1 << d, DEST_XOR),
                LOOP(LINK_WAIT_NOPS // WAIT_UNROLL), *[NOP] * WAIT_UNROLL, ENDL,
                RECV, ldm(GOT), i_(ALU_ADC, 0), stm(GOT),        # count received packets (C = got one)
                get(0), stm(into)]

    p = [LABEL("step"), ctx(CTX_ALWAYS)]

    # --- context buckets XA[128..255] from the last 8 characters ---
    p += [encode_instr(OP_MAR, src_sel=1, imm10=XA + 128), LDI0,
          LOOP(128), STX_INC, ENDL]
    for n in (1, 2, 3):
        for pos in range(CTX_LEN - n + 1):
            p += ldi16(pos * 7)
            for t in range(n):
                p += [i_(ALU_MUL, 31), mem(ALU_ADD, CTXC + pos + t)]
            p += [i_(ALU_AND, 127), i_(ALU_ADD, XA + 128), MAR, LDX,
                  i_(ALU_ADD, INPUT_SCALE), STX]

    # --- layers ---
    for li, (n_in, slots, ipw, wbase, bbase, sh) in enumerate(pl):
        last = li == len(pl) - 1
        p += [ctx(CTX_ALWAYS), LDI0] + [put(2 + s) for s in range(slots)]
        p += [*ldi16(XA if li == 0 else XB), stm(XPTR), *ldi16(wbase), stm(WPTR),
              LOOP(n_in // ipw),
              ldm(WPTR), MAR, i_(ALU_ADD, 1), stm(WPTR), LDX, put(6),
              ldm(XPTR), MAR, i_(ALU_ADD, ipw), stm(XPTR)]
        for u in range(ipw):
            p += [LDX_INC, put(1)]
            for s in range(slots):
                nib = u * slots + s
                p += [get(6), i_(ALU_SHLN, 12 - 4 * nib), i_(ALU_ASRN, 12),   # signed nibble
                      a(ALU_MUL, 1), a(ALU_ADD, 2 + s), put(2 + s)]
        p += [ENDL]
        for s in range(slots):
            r = 2 + s
            p += [ctx(CTX_ALWAYS), get(r), mem(ALU_ADD, bbase + s), i_(ALU_ASRN, sh), put(r)]
            if last:
                # PEs beyond the charset report PAD
                p += [alu(ALU_MOV, SRC_ID), i_(ALU_SUB, len(charset)), ctx(CTX_IF_N),
                      get(r), stm(LOGIT), ctx(action=CTX_INVERT), *ldi16(PAD), stm(LOGIT)]
            else:
                p += [ctx(CTX_IF_N), LDI0, put(r), ctx(CTX_ALWAYS),            # clamp to 0..127
                      get(r), i_(ALU_SUB, ACT_MAX), ctx(CTX_IF_N), ctx(action=CTX_INVERT),
                      i_(ALU_MOV, ACT_MAX), put(r), ctx(CTX_ALWAYS),
                      alu(ALU_MOV, SRC_ID), i_(ALU_ADD, XB + num_pe * s), MAR, get(r), STX]
        if not last:                                               # all-gather into XB
            for d in range(dims):
                n = 1 << d
                for s in range(slots):
                    p += [ctx(CTX_ALWAYS),
                          alu(ALU_MOV, SRC_ID), i_(ALU_AND, (num_pe - 1) & ~(n - 1)),
                          i_(ALU_ADD, XB + num_pe * s), stm(SPTR), i_(ALU_XOR, n), stm(DPTR),
                          LOOP(n),
                          ldm(SPTR), MAR, i_(ALU_ADD, 1), stm(SPTR), LDX, put(0),
                          send(n, DEST_XOR),
                          LOOP(LINK_WAIT_NOPS // WAIT_UNROLL), *[NOP] * WAIT_UNROLL, ENDL,
                          RECV, ldm(GOT), i_(ALU_ADC, 0), stm(GOT),
                          ldm(DPTR), MAR, i_(ALU_ADD, 1), stm(DPTR), get(0), STX,
                          ENDL]

    # --- the chosen character: maximum value, then the lowest index with it ---
    p += [ctx(CTX_ALWAYS), ldm(LOGIT), stm(BV)]
    for d in range(dims):
        p += [ctx(CTX_ALWAYS)] + exchange(BV, PV, d) + [
              ldm(PV), mem(ALU_SUB, BV), ctx(CTX_IF_N), ctx(action=CTX_INVERT),   # partner >= mine
              ldm(PV), stm(BV)]
    p += [ctx(CTX_ALWAYS), *ldi16(num_pe), stm(BK),
          ldm(LOGIT), mem(ALU_SUB, BV), ctx(CTX_IF_Z), alu(ALU_MOV, SRC_ID), stm(BK)]
    for d in range(dims):
        p += [ctx(CTX_ALWAYS)] + exchange(BK, PK, d) + [
              ldm(BK), mem(ALU_SUB, PK), ctx(CTX_IF_N), ctx(action=CTX_INVERT),   # partner <= mine
              ldm(PK), stm(BK)]

    # --- record it: PE number POS keeps it in RCH; update the context ---
    p += [ctx(CTX_ALWAYS), ldm(BK), stm(WIN),
          alu(ALU_MOV, SRC_ID), mem(ALU_SUB, POS), ctx(CTX_IF_Z), ldm(WIN), stm(RCH),
          ctx(CTX_ALWAYS)]
    for k in range(CTX_LEN - 1):
        p += [ldm(CTXC + k + 1), stm(CTXC + k)]
    p += [ldm(WIN), i_(ALU_ADD, CHARTAB), MAR, LDX, stm(CTXC + CTX_LEN - 1),
          ldm(POS), i_(ALU_ADD, 1), stm(POS),
          # carry on while the character isn't EOS and POS < MAXL
          ldm(WIN), i_(ALU_XOR, eos), ctx(CTX_IF_NZ),
          ldm(POS), mem(ALU_SUB, MAXL), ctx(CTX_IF_N),
          BANY("step"),
          ctx(CTX_ALWAYS)]
    return p


def gathered_per_step(pl, num_pe=64):
    """Packets each PE receives per character: all-gathers plus the argmax."""
    return sum((num_pe - 1) * slots for _, slots, *_ in pl[:-1]) + 2 * (num_pe - 1).bit_length()


class Board:
    """The model loaded into DM16."""

    def __init__(self, ser, layers, charset, verbose=True):
        import time
        from dm16_host import NOP, assemble
        if len(charset) > CTXC - CHARTAB:
            raise ValueError(f"charset too big ({len(charset)} > {CTXC - CHARTAB})")
        self.ser, self.layers, self.charset = ser, layers, charset
        self.pl = plan(layers)
        self.per_step = gathered_per_step(self.pl)
        t0 = time.monotonic()
        self._load(pe_words(layers, charset, self.pl))
        body = assemble([NOP] * PROLOGUE_MAX + _program(layers, charset, self.pl))[PROLOGUE_MAX:]
        if PROLOGUE_MAX + len(body) > 4096:
            raise RuntimeError(f"program too long ({len(body)} instructions)")
        self._write_imem(PROLOGUE_MAX, body)
        self.end_pc = PROLOGUE_MAX + len(body)
        if verbose:
            print(f"model loaded into DM16 in {time.monotonic() - t0:.1f} s "
                  f"({len(body)}-instruction generation program)")

    def _load(self, data):
        from dm16_host import CTX_ALWAYS, CTX_IF_ID, ctx, ldi16, stm, upload_and_run
        prog = []
        for p, words in enumerate(data):
            block = [ctx(CTX_IF_ID, imm=p)]
            for addr, v in sorted(words.items()):
                block += ldi16(v) + [stm(addr)]
            if len(prog) + len(block) > 4000:
                upload_and_run(self.ser, prog + [ctx(CTX_ALWAYS)])
                prog = []
            prog += block
        upload_and_run(self.ser, prog + [ctx(CTX_ALWAYS)])

    def _write_imem(self, addr, words):
        import struct
        self.ser.write(b"".join(struct.pack(">BH", 0x01, addr + k) + w.to_bytes(3, "big")
                                for k, w in enumerate(words)))

    def _run(self, query, ctxchars, maxlen):
        import struct
        import time
        from dm16_host import (CTX_ALWAYS, OP_LDI, OP_MAR, OP_STX, MEM_INC, LOOP, ENDL, JMP,
                               ctx, stm, ldi16, encode_instr, assemble, wait_idle, read_mem_block)
        pro = [ctx(CTX_ALWAYS), encode_instr(OP_MAR, src_sel=1, imm10=XA), encode_instr(OP_LDI),
               LOOP(128), encode_instr(OP_STX, flag_ctl=MEM_INC), ENDL]
        for i, c in enumerate(query_counts(query)):
            if c:
                pro += ldi16(INPUT_SCALE * c) + [stm(XA + i)]
        for k, ch in enumerate(ctxchars):
            pro += ldi16(ord(ch)) + [stm(CTXC + k)]
        pro += [encode_instr(OP_LDI), stm(POS), stm(GOT), *ldi16(maxlen), stm(MAXL),
                JMP(PROLOGUE_MAX)]
        pro = assemble(pro)
        if len(pro) > PROLOGUE_MAX:
            raise RuntimeError("prologue too long")
        self.ser.write(b"\x04")
        time.sleep(0.002)
        self._write_imem(0, pro)
        self.ser.write(struct.pack(">BH", 0x02, self.end_pc))
        self.ser.flush()
        time.sleep(0.002)
        wait_idle(self.ser, timeout=30)
        got, pos, rch, logit = read_mem_block(self.ser, GOT, 4)
        if pos[0] < 1 or any(g != self.per_step * pos[0] for g in got):
            raise RuntimeError(f"lost packets: received {sorted(set(got))[:4]}, "
                               f"expected {self.per_step * pos[0]} on every PE")
        return pos[0], rch, logit

    def reply(self, query, maxlen=MAX_REPLY):
        """The whole reply, generated on the board."""
        n, rch, _ = self._run(query, " " * CTX_LEN, maxlen)
        eos = len(self.charset) - 1
        out = ""
        for k in range(n):
            if rch[k] == eos:
                break
            out += self.charset[rch[k]]
        return out

    def step(self, query, ctxchars):
        """One character: the output values and the chosen index."""
        _, rch, logit = self._run(query, ctxchars, 1)
        return logit[:len(self.charset)], rch[0]


def main():
    import argparse
    import sys
    import time
    from dm16_host import DEFAULT_PORT, NUM_PE, WORD_BITS, open_dm16, to_signed

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--model", default="models/tinychat_int4.npz")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--query", help="answer one query and exit (otherwise: chat)")
    ap.add_argument("--check", action="store_true",
                    help="also run each reply one character at a time and compare every "
                         "output value with the integer model")
    ap.add_argument("--test-file", help="input|response file: compare the board's replies "
                                        "with the integer model's for its queries")
    ap.add_argument("--count", type=int, default=200)
    args = ap.parse_args()

    layers, charset, meta = load_model(args.model)
    if NUM_PE != 64 or WORD_BITS != 16:
        raise SystemExit("this needs the Tang Console build (64 PEs of 16 bits)")
    sizes = [layers[0][0].shape[1]] + [w.shape[0] for w, _, _ in layers]
    print(f"model {args.model}: {' -> '.join(map(str, sizes))}, 4-bit weights"
          + (f", {100 * meta['exact']:.1f}% of training replies exact" if "exact" in meta else ""))

    def check_steps(board, q, reply):
        """Every character: the board's output values against the integer model."""
        ctxc, bad = " " * CTX_LEN, 0
        for ch in reply + "\x00":
            got, k = board.step(q, ctxc)
            want = int_forward(layers, [query_counts(q) + context_counts(ctxc)])[0]
            if [to_signed(v) for v in got] != [int(v) for v in want] or k != first_argmax(want):
                bad += 1
            if ch != "\x00":
                ctxc = ctxc[1:] + lower(ch)
        return bad

    with open_dm16(args.port) as ser:
        board = Board(ser, layers, charset)
        if args.test_file:
            queries = sorted({q for q, _ in read_pairs(args.test_file)})[:args.count]
            ref = generate_batch(layers, charset, queries)
            t0, same, chars, bad = time.monotonic(), 0, 0, 0
            for q, r in zip(queries, ref):
                a = board.reply(q)
                same += a == r
                chars += len(a) + 1
                if a != r:
                    print(f"  {q!r}: board {a!r}, integer model {r!r}")
            dt = time.monotonic() - t0
            print(f"{same}/{len(queries)} replies identical to the integer model "
                  f"({len(queries) / dt:.1f} replies/s, {chars / dt:.0f} characters/s)")
            if args.check:
                for q, r in zip(queries[:20], ref[:20]):
                    bad += check_steps(board, q, r)
                print(f"step-by-step check on 20 replies: "
                      f"{'every output value matches' if not bad else f'{bad} steps differ'}")
            return 0 if same == len(queries) and not bad else 1

        def answer(q):
            t0 = time.monotonic()
            a = board.reply(q[:62])
            dt = time.monotonic() - t0
            print(f"{a}   ({dt * 1000:.0f} ms)")
            if args.check:
                bad = check_steps(board, q[:62], a)
                print(f"  check: {'every output value matches the integer model' if not bad else f'{bad} steps differ'}")
                return bad
            return 0

        if args.query:
            return 1 if answer(args.query) else 0
        print("Chat with the model on DM16 (empty line to quit)")
        while True:
            try:
                q = input("> ")
            except EOFError:
                break
            if not q.strip():
                break
            answer(q)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
