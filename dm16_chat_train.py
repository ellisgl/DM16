"""
Train dm16_chat's 4-bit chat model with NumPy (no PyTorch).

The training data format (input|response lines) and the tinychat data set
come from Z80-μLM by HarryR (https://github.com/HarryR/z80ai).

Quantization-aware training: the forward pass uses the weights rounded to
4 bits at a power-of-two scale (chosen per layer as training goes), and the
board's activation rounding (floor of 16 * z, clamped to 0..127), with
straight-through gradients. The model kept is the one whose exact integer
version (dm16_chat.int_forward, which is what DM16 computes) gives the most
exactly right replies.

  python3 dm16_chat_train.py --data ~/z80ai/examples/tinychat/training-data.txt.gz \\
                             --out models/tinychat_int4.npz
"""
import argparse
import time

import numpy as np

import dm16_chat as C


class Net:
    def __init__(self, sizes, rng):
        self.W, self.b = [], []
        for n_in, n_out in zip(sizes[:-1], sizes[1:]):
            self.W.append(rng.normal(0, np.sqrt(2.0 / n_in), (n_out, n_in)).astype(np.float32))
            self.b.append(np.zeros(n_out, dtype=np.float32))
        self.shifts = [self.pick_shift(w) for w in self.W]

    @staticmethod
    def pick_shift(w):
        """Power-of-two weight scale: the 99th percentile of |w| maps to ~6."""
        p = float(np.percentile(np.abs(w), 99)) or 1e-3
        return int(np.clip(np.round(np.log2(6.0 / p)), 2, 8))

    def quantized(self, i):
        k = float(1 << self.shifts[i])
        wq = np.clip(np.round(self.W[i] * k), C.W_MIN, C.W_MAX)
        bq = np.round(self.b[i] * k * C.INPUT_SCALE)
        inside = (self.W[i] * k > C.W_MIN - 0.5) & (self.W[i] * k < C.W_MAX + 0.5)
        return wq / k, bq / (k * C.INPUT_SCALE), inside

    def forward(self, x):
        """x: bucket counts (float). Returns logits and what backward needs."""
        cache, a = [], x
        n = len(self.W)
        for i in range(n):
            wq, bq, inside = self.quantized(i)
            z = a @ wq.T + bq
            if i < n - 1:
                s = np.floor(z * C.INPUT_SCALE)
                gate = (s > 0) & (s < C.ACT_MAX)               # straight-through
                out = np.clip(s, 0, C.ACT_MAX) / C.INPUT_SCALE
            else:
                gate, out = None, z
            cache.append((a, wq, inside, gate))
            a = out
        return a, cache

    def backward(self, dlogits, cache):
        grads = [None] * len(self.W)
        d = dlogits
        for i in reversed(range(len(self.W))):
            a, wq, inside, gate = cache[i]
            if gate is not None:
                d = d * gate
            dW = (d.T @ a) * inside
            db = d.sum(axis=0)
            grads[i] = (dW, db)
            if i:
                d = d @ wq
        return grads

    def export(self):
        layers = []
        for i in range(len(self.W)):
            sh = self.shifts[i]
            k = 1 << sh
            w = np.clip(np.round(self.W[i] * k), C.W_MIN, C.W_MAX).astype(np.int64)
            b = np.clip(np.round(self.b[i] * k * C.INPUT_SCALE), -32768, 32767).astype(np.int64)
            layers.append((w, b, sh))
        return layers


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--data", required=True, help="input|response training file (.gz ok)")
    ap.add_argument("--out", required=True, help="model file to write (.npz)")
    ap.add_argument("--hidden", default="256,128", help="hidden layer sizes")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--eval-every", type=int, default=10)
    ap.add_argument("--holdout", type=float, default=0.0,
                    help="fraction of distinct queries kept out of training, to measure "
                         "replies to unseen queries")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    pairs = C.read_pairs(args.data)
    charset = C.build_charset(pairs)
    held = []
    if args.holdout:
        qs = sorted({q for q, _ in pairs})
        out = set(rng.choice(qs, int(len(qs) * args.holdout), replace=False))
        held = [pq for pq in pairs if pq[0] in out]
        pairs = [pq for pq in pairs if pq[0] not in out]
        print(f"holding out {len(out)} queries ({len(held)} pairs)")
    x, y = C.training_examples(pairs, charset)
    x = x.astype(np.float32)
    sizes = [256] + [int(v) for v in args.hidden.split(",")] + [len(charset)]
    print(f"{len(pairs)} pairs, {len(x)} examples, charset {charset[:-1]!r} + EOS, "
          f"network {' -> '.join(map(str, sizes))} "
          f"({sum(a * b for a, b in zip(sizes[:-1], sizes[1:]))} weights)")

    net = Net(sizes, rng)
    params = net.W + net.b
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    step, best, t0 = 0, (-1, None), time.monotonic()
    for epoch in range(1, args.epochs + 1):
        lr = args.lr * (0.5 * (1 + np.cos(np.pi * epoch / args.epochs)))   # cosine decay
        order = rng.permutation(len(x))
        loss_sum = 0.0
        for s in range(0, len(x), args.batch):
            idx = order[s:s + args.batch]
            logits, cache = net.forward(x[idx])
            logits = logits - logits.max(axis=1, keepdims=True)
            p = np.exp(logits)
            p /= p.sum(axis=1, keepdims=True)
            loss_sum += float(-np.log(p[np.arange(len(idx)), y[idx]] + 1e-12).sum())
            p[np.arange(len(idx)), y[idx]] -= 1
            grads = net.backward(p / len(idx), cache)
            step += 1
            flat = [g[0] for g in grads] + [g[1] for g in grads]
            for i, (prm, g) in enumerate(zip(params, flat)):
                m[i] = 0.9 * m[i] + 0.1 * g
                v[i] = 0.999 * v[i] + 0.001 * g * g
                mh = m[i] / (1 - 0.9 ** step)
                vh = v[i] / (1 - 0.999 ** step)
                prm -= lr * mh / (np.sqrt(vh) + 1e-8)
        if epoch % 5 == 0:
            net.shifts = [net.pick_shift(w) for w in net.W]
        if epoch % args.eval_every == 0 or epoch == args.epochs:
            layers = net.export()
            stats = {}
            C.int_forward(layers, x[:2000].astype(np.int64), stats)
            exact, char_acc = C.evaluate(layers, charset, pairs)
            tag = ""
            if exact > best[0]:
                best = (exact, layers)
                C.save_model(args.out, layers, charset,
                             {"epoch": epoch, "exact": exact, "char_acc": char_acc,
                              "sizes": sizes, "data": args.data.rsplit("/", 1)[-1]})
                tag = "  (saved)"
            if held:
                tag += f"  unseen queries: {100 * C.evaluate(layers, charset, held)[0]:.1f}%"
            print(f"epoch {epoch:4d}  loss {loss_sum / len(x):.3f}  integer model: "
                  f"replies {100 * exact:5.1f}%  characters {100 * char_acc:5.1f}%  "
                  f"shifts {net.shifts}  overflow {stats['overflow']}/{stats['sums']}  "
                  f"[{time.monotonic() - t0:.0f} s]{tag}", flush=True)
    print(f"best: {100 * best[0]:.1f}% of replies exactly right; model in {args.out}")


if __name__ == "__main__":
    raise SystemExit(main())
