#!/usr/bin/env python3
"""Summarise and compare xr_frame_logger.gd CSVs.

In VR the frame rate is pinned to the compositor, so averages and FPS say almost
nothing. What matters is the tail of the frame-interval distribution and how often
we blow the budget, so that is what this reports.

    python3 analyze_bench.py safe.csv separate.csv
    python3 analyze_bench.py out/safe_*.csv -- out/separate_*.csv

With `--`, every CSV on each side is pooled into one arm and the per-run spread is
reported alongside, which is the honest measure of run-to-run variability.
"""
import math
import sys
from pathlib import Path

Z = 1.96  # 95%


def load(path):
    meta, rows = {}, []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            for tok in line.lstrip("#").split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    meta[k] = v
            continue
        if line.startswith("t_sec"):
            continue
        p = line.split(",")
        if len(p) >= 4:
            rows.append(tuple(float(x) for x in p[:4]))
    if not rows:
        sys.exit(f"{path}: no samples")
    return meta, rows


def pct(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def wilson(hits, n):
    """Score interval for a proportion; behaves sanely when hits is 0."""
    if n == 0:
        return (0.0, 0.0)
    p = hits / n
    d = 1 + Z * Z / n
    c = (p + Z * Z / (2 * n)) / d
    h = Z / d * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n))
    return (max(0.0, c - h), min(1.0, c + h))


def summarise(paths):
    """Pool every CSV in `paths` into one arm, keeping per-run figures too."""
    pooled, runs, hzs, labels = [], [], [], []
    for path in paths:
        meta, rows = load(path)
        hz = float(meta.get("refresh", 72.0))
        budget = 1000.0 / hz
        hzs.append(hz)
        labels.append(meta.get("label", "?"))
        pooled.extend(rows)

        half = len(rows) // 2
        missed = sum(1 for r in rows if r[1] > budget * 1.5)
        runs.append({
            "path": Path(path).name,
            "n": len(rows),
            "miss_rate": missed / len(rows),
            "drift": (pct(sorted(r[1] for r in rows[half:]), 0.95)
                      - pct(sorted(r[1] for r in rows[:half]), 0.95)),
        })

    if max(hzs) - min(hzs) > 0.5:
        sys.exit(f"refresh rates differ across {paths}; not poolable")

    hz = hzs[0]
    budget = 1000.0 / hz
    n = len(pooled)
    missed = sum(1 for r in pooled if r[1] > budget * 1.5)
    missed2 = sum(1 for r in pooled if r[1] > budget * 2.5)
    lo, hi = wilson(missed, n)

    return {
        "name": ", ".join(r["path"] for r in runs),
        "label": labels[0],
        "hz": hz,
        "budget": budget,
        "n": n,
        "delta": sorted(r[1] for r in pooled),
        "proc": sorted(r[2] for r in pooled),
        "missed": missed,
        "missed2": missed2,
        "miss_rate": missed / n,
        "miss_lo": lo,
        "miss_hi": hi,
        "runs": runs,
    }


def report(s):
    d, p = s["delta"], s["proc"]
    print(f"\n=== {s['label']}  ({s['hz']:.0f}Hz, {len(s['runs'])} run(s), {s['n']} frames) ===")
    print(f"  files                  {s['name']}")
    print(f"  frame budget           {s['budget']:.2f} ms")
    print(f"  frame interval  p50    {pct(d, 0.50):.2f} ms")
    print(f"                  p95    {pct(d, 0.95):.2f} ms")
    print(f"                  p99    {pct(d, 0.99):.2f} ms")
    print(f"                  max    {d[-1]:.2f} ms")
    print(f"  main-thread     p50    {pct(p, 0.50):.2f} ms")
    print(f"  process()       p95    {pct(p, 0.95):.2f} ms")
    print(f"                  p99    {pct(p, 0.99):.2f} ms")
    print(f"  missed frames          {s['missed']} ({s['miss_rate'] * 100:.3f}%)"
          f"  95% CI [{s['miss_lo'] * 100:.3f}%, {s['miss_hi'] * 100:.3f}%]")
    print(f"  missed >=2 frames      {s['missed2']}")
    rates = [r["miss_rate"] * 100 for r in s["runs"]]
    print(f"  per-run miss rate      {min(rates):.3f}% .. {max(rates):.3f}%")
    for r in s["runs"]:
        note = "  <-- throttling" if r["drift"] > 1.0 else ""
        print(f"    {r['path']:<24} {r['miss_rate'] * 100:7.3f}%   p95 drift {r['drift']:+.2f} ms{note}")


def compare(a, b):
    print(f"\n=== {a['label']} -> {b['label']} ===")
    if abs(a["hz"] - b["hz"]) > 0.5:
        print("  !! different refresh rates; results are not comparable")
    for name, q in (("p50", 0.50), ("p95", 0.95), ("p99", 0.99)):
        va, vb = pct(a["delta"], q), pct(b["delta"], q)
        print(f"  frame {name:<4} {va:7.2f} -> {vb:7.2f} ms   {vb - va:+6.2f}")
    for name, q in (("p50", 0.50), ("p95", 0.95)):
        va, vb = pct(a["proc"], q), pct(b["proc"], q)
        print(f"  main  {name:<4} {va:7.2f} -> {vb:7.2f} ms   {vb - va:+6.2f}")

    print(f"  missed     {a['miss_rate'] * 100:7.3f}% -> {b['miss_rate'] * 100:7.3f}%")

    # Frame misses cluster, so treating frames as independent trials makes the Wilson
    # interval optimistically narrow. Require the per-run ranges to separate as well
    # before calling it: that is the part that survives run-to-run variance.
    ra = [r["miss_rate"] for r in a["runs"]]
    rb = [r["miss_rate"] for r in b["runs"]]
    runs_separate = max(rb) < min(ra) or min(rb) > max(ra)
    single_run = len(ra) < 2 or len(rb) < 2

    if b["miss_hi"] < a["miss_lo"] and runs_separate:
        verdict = "IMPROVED"
    elif b["miss_lo"] > a["miss_hi"] and runs_separate:
        verdict = "REGRESSED"
    elif b["miss_hi"] < a["miss_lo"] or b["miss_lo"] > a["miss_hi"]:
        verdict = ("pooled CIs separate but the per-run ranges overlap"
                   " -- run-to-run noise, collect more reps")
    else:
        verdict = "inconclusive (CIs overlap -- repeat the runs)"
    if single_run and verdict in ("IMPROVED", "REGRESSED"):
        verdict += " (single run per arm; confirm with >=3)"
    print(f"  verdict    {verdict}")


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)

    if "--" in args:
        i = args.index("--")
        groups = [args[:i], args[i + 1:]]
        if not groups[0] or not groups[1]:
            sys.exit("both sides of `--` need at least one CSV")
    else:
        groups = [[a] for a in args]

    arms = [summarise(g) for g in groups]
    for arm in arms:
        report(arm)
    if len(arms) == 2:
        compare(arms[0], arms[1])
    print()


if __name__ == "__main__":
    main()
