"""Exact reference model of the DM16 bfloat16 unit (RNE, flush-to-zero, canonical NaN 0x7FC0).

Used by dm16_fp_test.py to check board results bit-for-bit.
"""
from fractions import Fraction
QNAN = 0x7FC0
def fields(x): return (x >> 15) & 1, (x >> 7) & 0xFF, x & 0x7F
def is_nan(x): s,e,f = fields(x); return e == 0xFF and f != 0
def is_inf(x): s,e,f = fields(x); return e == 0xFF and f == 0
def is_zero(x): return fields(x)[1] == 0
def value(x):
    s,e,f = fields(x)
    v = Fraction(128 + f, 128) * Fraction(2) ** (e - 127)
    return -v if s else v
def round_bf16(v):
    """Exact nonzero rational -> bf16 with RNE, overflow->inf, underflow->0 (FTZ)."""
    s = 1 if v < 0 else 0
    m = -v if s else v
    E = 0
    while m >= 2 ** (E + 1): E += 1
    while m < 2 ** E: E -= 1
    q = m / Fraction(2) ** (E - 7)          # in [128, 256)
    n = q.numerator // q.denominator
    r = q - n
    if r > Fraction(1, 2) or (r == Fraction(1, 2) and n % 2 == 1): n += 1
    if n == 256: n = 128; E += 1
    be = E + 127
    if be >= 255: return (s << 15) | 0x7F80
    if be <= 0:   return s << 15
    return (s << 15) | (be << 7) | (n - 128)
def add(a, b, sub=False):
    if sub: b ^= 0x8000
    sa, sb = a >> 15, b >> 15
    if is_nan(a) or is_nan(b): return QNAN
    if is_inf(a) and is_inf(b): return QNAN if sa != sb else (sa << 15) | 0x7F80
    if is_inf(a): return (sa << 15) | 0x7F80
    if is_inf(b): return (sb << 15) | 0x7F80
    if is_zero(a) and is_zero(b): return (sa & sb) << 15
    if is_zero(a): return b
    if is_zero(b): return a
    v = value(a) + value(b)
    return 0 if v == 0 else round_bf16(v)
def mul(a, b):
    s = (a ^ b) >> 15
    if is_nan(a) or is_nan(b): return QNAN
    if (is_inf(a) and is_zero(b)) or (is_inf(b) and is_zero(a)): return QNAN
    if is_inf(a) or is_inf(b): return (s << 15) | 0x7F80
    if is_zero(a) or is_zero(b): return s << 15
    return round_bf16(value(a) * value(b))
def i2f(i16):
    """int16 (given as 16-bit pattern) -> bf16, round to nearest even."""
    v = i16 - 65536 if i16 >= 32768 else i16
    return 0 if v == 0 else round_bf16(Fraction(v))
def f2i(x):
    """bf16 -> int16 (16-bit pattern), truncate toward zero, saturate."""
    s,e,f = fields(x)
    if e < 127: return 0
    if e >= 142: return 0x8000 if s else 0x7FFF
    mag = ((128 + f) << 8) >> (142 - e)
    return (-mag) & 0xFFFF if s else mag
def f2bf(f):
    """Python float -> bf16 bits (RNE) for convenience."""
    import struct
    u = struct.unpack('<I', struct.pack('<f', f))[0]
    if (u & 0x7F800000) == 0x7F800000 and (u & 0x7FFFFF): return QNAN
    u += 0x7FFF + ((u >> 16) & 1)
    return (u >> 16) & 0xFFFF
def bf2f(x):
    import struct
    return struct.unpack('<f', struct.pack('<I', x << 16))[0]
