"""
Reference model of one DM16 PE at any data width W (16, 32, 64, ...), any
number of registers NREGS and any memory size 2^MEM_AW.

It executes 24-bit instruction words with the same results as src/dm16_pe.v:
acc, b, r1..r[NREGS-1], mar, flags, the context stack and local memory. The network
and NEWS inputs are supplied by the caller (constant values are enough for
testing). Floating point works per 16-bit lane through dm16_bf16_ref; FCMP
compares lane 0.

Used by the RTL simulations to check the PE at widths that have no board.
"""
import dm16_bf16_ref as ref

OP_NOP, OP_LDI, OP_LDM, OP_STM, OP_LDX, OP_STX, OP_MAR, OP_ALU = range(8)
OP_SWP, OP_CTX, OP_SEND, OP_RECV, OP_NEWS, OP_FPU = range(8, 14)


class PE:
    def __init__(self, W=16, pe_id=0, has_rf=True, mem=None, news=(0, 0, 0, 0), rx=None,
                 nregs=8, mem_aw=10, has_fpu=True):
        assert W % 16 == 0 and W >= 16 and mem_aw <= W
        self.W, self.M = W, (1 << W) - 1
        self.nregs, self.amask = nregs, (1 << mem_aw) - 1
        self.has_fpu = has_fpu      # without it every FPU instruction is a no-op
        self.id, self.has_rf = pe_id, has_rf
        self.mem = list(mem) if mem is not None else [0] * (1 << mem_aw)
        self.news = news            # values seen from N, E, W, S
        self.rx = rx                # mailbox word, or None if empty
        self.acc = self.b = self.mar = 0
        self.c = self.z = self.n = 0
        self.ctx_stack = 0b1111
        self.rf = [0] * nregs       # rf[0] unused: r0 is b

    # --- helpers -------------------------------------------------------
    def sext10(self, v):
        return (v - 1024 if v & 0x200 else v) & self.M

    def reg(self, n):
        return self.b if (n == 0 or not self.has_rf) else self.rf[n]

    def set_reg(self, n, v):
        if n == 0 or not self.has_rf:
            self.b = v
        else:
            self.rf[n] = v

    def lanes(self, v):
        return [(v >> (16 * i)) & 0xFFFF for i in range(self.W // 16)]

    def join(self, ls):
        return sum(x << (16 * i) for i, x in enumerate(ls))

    def signed(self, v):
        return v - (1 << self.W) if v >> (self.W - 1) else v

    @property
    def ctx(self):
        return self.ctx_stack & 1

    def state(self):
        return (self.acc, self.b, self.mar, self.c, self.z, self.n, self.ctx_stack,
                tuple(self.rf[1:]) if self.has_rf else ())

    # --- one instruction ----------------------------------------------
    def step(self, w):
        W, M = self.W, self.M
        op, sub = w >> 20, (w >> 16) & 15
        src, fc, imm = (w >> 14) & 3, (w >> 10) & 15, w & 0x3FF
        x = (fc >> 2) & 1

        if op == OP_CTX:
            cond = [1, 0, self.z, 1 - self.z, self.c, self.n,
                    int(self.id == (imm & 0xFF))][sub] if sub <= 6 else self.ctx
            a = fc & 3
            if a == 0:
                self.ctx_stack = (self.ctx_stack & 0b1110) | cond
            elif a == 1:
                self.ctx_stack = ((self.ctx_stack << 1) & 0b1110) | cond
            elif a == 2:
                self.ctx_stack = 0b1000 | (self.ctx_stack >> 1)
            else:
                self.ctx_stack = ~self.ctx_stack & 0b1111
            return
        if not self.ctx:
            return

        uses_mem = op in (OP_LDM, OP_STM, OP_LDX, OP_STX) or (op in (OP_ALU, OP_FPU) and src == 3)
        use_mar = op in (OP_LDX, OP_STX) or (uses_mem and fc & 1)
        rn = imm % self.nregs
        addr = self.mar if use_mar else imm & self.amask
        opb = [self.reg(rn), self.sext10(imm), self.id, self.mem[addr]][src]
        acc = self.acc

        if op == OP_LDI:
            if sub & 3 == 1:
                self.acc = (acc & ~0xFF00 & M) | ((imm & 0xFF) << 8)
            elif sub & 3 == 2:
                self.acc = (acc & ~0xFF & M) | (imm & 0xFF)
            else:
                self.acc = self.sext10(imm)
        elif op in (OP_LDM, OP_LDX):
            self.acc = self.mem[addr]
        elif op in (OP_STM, OP_STX):
            self.mem[addr] = acc
        elif op == OP_MAR:
            self.mar = (acc if src == 0 else imm) & self.amask
        elif op == OP_ALU:
            self.alu(sub, x, opb)
        elif op == OP_SWP:
            old = self.reg(rn)
            if sub != 2:
                self.set_reg(rn, acc)
            if sub != 1:
                self.acc = old
        elif op == OP_RECV:
            if self.rx is not None:
                self.b, self.c = self.rx, 1
            else:
                self.c = 0
        elif op == OP_NEWS:
            self.acc = self.news[sub & 3]
        elif op == OP_FPU and self.has_fpu:
            self.fpu(sub, opb)

        if uses_mem and fc & 2:
            self.mar = (self.mar + 1) & self.amask

    def alu(self, sub, x, opb):
        W, M, acc = self.W, self.M, self.acc
        c = 0
        n = opb % W
        if sub in (0, 1, 2):
            if sub == 2:
                t = acc + ((~opb) & M) + 1
                r, c = t & M, 1 - (t >> W)
            else:
                t = acc + opb + (self.c if sub == 1 else 0)
                r, c = t & M, t >> W
        elif sub == 3: r = acc & opb
        elif sub == 4: r = acc | opb
        elif sub == 5: r = acc ^ opb
        elif sub == 6: r, c = (acc << 1) & M, acc >> (W - 1)
        elif sub == 7: r, c = acc >> 1, acc & 1
        elif sub == 8: r = opb
        elif sub == 9: r, c = (acc >> 1) | (acc & (1 << (W - 1))), acc & 1
        elif sub == 10: r = (acc * opb) & M
        elif sub == 11:
            p = acc * opb if x else self.signed(acc) * self.signed(opb)
            r = (p >> W) & M
        elif sub == 12:   # SHLN (x: funnel from b)
            wv = (acc << W) | (self.b if x else 0)
            r = ((wv << n) >> W) & M
            c = (acc >> (W - n)) & 1 if n else 0
        elif sub in (13, 14):   # SHRN (x: funnel from b), ASRN
            hi = (M if acc >> (W - 1) else 0) if sub == 14 else (self.b if x else 0)
            r = (((hi << W) | acc) >> n) & M
            c = (acc >> (n - 1)) & 1 if n else 0
        else:             # ROL (x: ROR)
            if x:
                r = ((acc >> n) | (acc << (W - n))) & M
                c = (r >> (W - 1)) & 1 if n else 0
            else:
                r = ((acc << n) | (acc >> (W - n))) & M
                c = r & 1 if n else 0
        self.acc, self.c = r, c
        self.z, self.n = int(r == 0), r >> (W - 1)

    def fpu(self, sub, opb):
        A, B = self.lanes(self.acc), self.lanes(opb)
        if sub == 0:   self.acc = self.join([ref.add(a, b) for a, b in zip(A, B)])
        elif sub == 1: self.acc = self.join([ref.add(a, b, True) for a, b in zip(A, B)])
        elif sub == 2: self.acc = self.join([ref.mul(a, b) for a, b in zip(A, B)])
        elif sub == 4: self.acc = self.join([ref.i2f(a) for a in A])
        elif sub == 5: self.acc = self.join([ref.f2i(a) for a in A])
        elif sub == 6: self.acc = self.join([a ^ 0x8000 for a in A])
        elif sub == 7: self.acc = self.join([a & 0x7FFF for a in A])
        elif sub == 3:
            a, b = A[0], B[0]
            nan = lambda v: ((v >> 7) & 0xFF) == 0xFF and (v & 0x7F) != 0
            key = lambda v: (lambda y: (~y & 0xFFFF) if y & 0x8000 else (y | 0x8000))(
                0 if ((v >> 7) & 0xFF) == 0 else v)
            un = nan(a) or nan(b)
            self.c = int(un)
            self.z = int(not un and key(a) == key(b))
            self.n = int(not un and key(a) < key(b))
