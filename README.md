# py-layerd

Python bindings for [tanishq-kumar/layerd](https://github.com/tanishq-kumar/layerd) (fork of [Nightwalk2001/layerd](https://github.com/Nightwalk2001/layerd)) — fast, low-RAM layered (Sugiyama) graph layout in Rust.

- **Core:** Rust arena graph + typed `NodeId`/`EdgeId`/`PortId` + in-place passes (upstream `layerd/core`).
- **Wire:** LRD1 binary format via `ffi-types` (`layout_flat` / `layout_bytes` with panic-safe `catch_unwind`).
- **Python:** PyO3 + maturin, `uv` project, `ruff`, `requires-python >=3.10`.

> Fork Purpose (Rust): see [tanishq-kumar/layerd](https://github.com/tanishq-kumar/layerd) — offset-aware positioning, React Flow + JSON Canvas adapters, large-scale profiling (100k/200k+). This repo is the Python package for that fork.

## Quickstart

```bash
uv sync --dev
uv run maturin develop
uv run python -c "from py_layerd.layout import layout; print(layout([{'id':'a','width':100,'height':40},{'id':'b','width':100,'height':40}], [{'id':'e1','source':'a','target':'b'}]))"
```

## API

- `py_layerd._core.NodeSpec(id, width, height)` / `EdgeSpec(id, source, target)` / `layout_flat_py(nodes, edges) -> LayoutResult`
- `py_layerd._core.PyLayoutOptions(direction, layering, node_placement, edge_routing, cycle_breaking, node_node, padding, ...)` / `layout_with_options_py(nodes, edges, options)`
- `py_layerd.layout.layout(nodes: list[dict], edges: list[dict], offset=(0,0), direction="RIGHT", layering="network_simplex", node_placement="brandes_koepf", edge_routing="orthogonal", cycle_breaking="greedy", spacing_node_node=20, spacing_node_between_layers=20, padding=12, thoroughness=7, random_seed=1, options=None) -> {nodes, edges, width, height}` — high-level dict API with `offset` translation.

## Toolchain

- `uv` (project + env), `ruff check` + `ruff format`, `maturin develop`, `pytest`.
- Rust: `cargo test -p layerd` on the fork; Python: `uv run pytest`.

## Roadmap

- Phase 1: scaffold + smoke — done (this repo).
- Phase 2: `LayoutOptions` surface — done (direction / layering / node placement / edge routing / spacing / padding / seed).
- Phase 3/4: React Flow (`measured`/`sourceHandle`→ports) + JSON Canvas (`group` hierarchy, side→ports) adapters.
- Phase 5: incremental / warm-start.
- Phase 8: 100k nodes / 200k edges ladder profiling.
