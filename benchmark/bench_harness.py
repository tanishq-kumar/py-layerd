"""Phase 8 harness — run via: uv run python benchmark/bench_harness.py [--quick]

Profiles py_layerd._core (release build) on synthetic ladders:
- chain (linear)
- layered DAG (wide fanout)
- dense DAG (8-10x edges per node)
- cycle (with back edges)
- bipartite / fanout

Writes results to benchmark/results/<timestamp>/summary.json.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from py_layerd._core import EdgeSpec, NodeSpec, layout_flat_py


def bench(name: str, nodes, edges):
    t0 = time.perf_counter()
    r = layout_flat_py(nodes, edges)
    ms = (time.perf_counter() - t0) * 1000
    return {
        "name": name,
        "nodes": len(nodes),
        "edges": len(edges),
        "ms": round(ms, 1),
        "width": round(float(r.width), 1),
        "height": round(float(r.height), 1),
        "bends": len(r.bend_x),
    }


def run_all(quick: bool = False):
    results = []
    random.seed(0)

    for n in [64, 1000, 10_000, 30_000, 60_000, 100_000]:
        if quick and n > 10_000:
            continue
        nodes = [NodeSpec(i, 80, 30) for i in range(1, n + 1)]
        edges = [EdgeSpec(i, i, i + 1) for i in range(1, n)]
        results.append(bench(f"chain {n}", nodes, edges))

    # chain 100k + double for ~200k edges
    if not quick:
        n = 100_000
        nodes = [NodeSpec(i, 80, 30) for i in range(1, n + 1)]
        edges = [EdgeSpec(i, i, i + 1) for i in range(1, n)] + [
            EdgeSpec(n - 1 + i, i, i + 2) for i in range(1, n - 1)
        ]
        edges = edges[:200_000]
        edges = [EdgeSpec(i + 1, e.source, e.target) for i, e in enumerate(edges)]
        results.append(bench("chain 100k + double 200k", nodes, edges))

    for n, m in [(50, 400), (100, 1000), (200, 2000), (500, 5000)]:
        nodes = [NodeSpec(i, 60, 30) for i in range(1, n + 1)]
        edges_set: set[tuple[int,int]] = set()
        while len(edges_set) < m:
            s = random.randint(1, n - 1)
            t = random.randint(s + 1, n)
            edges_set.add((s, t))
        edges = [EdgeSpec(i + 1, s, t) for i, (s, t) in enumerate(edges_set)]
        results.append(bench(f"dense DAG {n} {m} ({m/n:.0f}x)", nodes, edges))

    for n, m in [(50, 300), (100, 800)]:
        nodes = [NodeSpec(i, 60, 30) for i in range(1, n + 1)]
        edges_set = set()
        while len(edges_set) < m:
            s = random.randint(1, n)
            t = random.randint(1, n)
            if s != t:
                edges_set.add((s, t))
        edges = [EdgeSpec(i + 1, s, t) for i, (s, t) in enumerate(edges_set)]
        results.append(bench(f"cycle {n} {m} ({m/n:.0f}x)", nodes, edges))

    # bipartite / fanout
    results.append(bench("bipartite 32x32", [NodeSpec(i, 40, 20) for i in range(1, 65)], [EdgeSpec(i + 1, (i // 32) + 1, 33 + (i % 32)) for i in range(1024)]))
    n = 102
    nodes = [NodeSpec(i, 80, 30) for i in range(1, n + 1)]
    edges = []
    for i in range(2, 102):
        edges.append(EdgeSpec(len(edges) + 1, 1, i))
        edges.append(EdgeSpec(len(edges) + 1, i, n))
    results.append(bench("fanout 1-100-1", nodes, edges))

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path("benchmark/results") / ts
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(results, indent=2))
    # markdown table
    lines = ["| name | n | e | ms | w×h | bends |", "|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| {r['name']} | {r['nodes']} | {r['edges']} | {r['ms']} | {r['width']}×{r['height']} | {r['bends']} |")
    (out_dir / "SUMMARY.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nWrote {out_dir / 'summary.json'}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    run_all(quick=args.quick)
