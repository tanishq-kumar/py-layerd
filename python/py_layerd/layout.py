from __future__ import annotations

from dataclasses import dataclass
from typing import TypedDict

from py_layerd._core import (
    EdgeSpec,
    LayoutResult,
    NodeSpec,
    PyLayoutOptions,
    layout_flat_py,
    layout_with_options_py,
)


class NodeInput(TypedDict, total=False):
    id: str | int
    width: float
    height: float


class EdgeInput(TypedDict, total=False):
    id: str | int
    source: str | int
    target: str | int


@dataclass(frozen=True, slots=True)
class PositionedNode:
    id: str
    x: float
    y: float
    width: float
    height: float


@dataclass(frozen=True, slots=True)
class PositionedEdge:
    id: str
    source: str
    target: str
    bends: tuple[tuple[float, float], ...]


def _to_u32_id(raw: str | int, mapping: dict[str | int, int], kind: str) -> int:
    if raw in mapping:
        return mapping[raw]
    raise ValueError(f"unknown {kind} id: {raw!r}")


def layout(
    nodes: list[dict],
    edges: list[dict],
    *,
    offset: tuple[float, float] = (0.0, 0.0),
    algorithm: str | None = None,
    direction: str = "RIGHT",
    layering: str = "network_simplex",
    node_placement: str = "brandes_koepf",
    edge_routing: str = "orthogonal",
    cycle_breaking: str = "greedy",
    spacing_node_node: float = 20.0,
    spacing_node_between_layers: float = 20.0,
    padding: float = 12.0,
    thoroughness: int = 7,
    random_seed: int = 1,
    options: PyLayoutOptions | None = None,
) -> dict:
    """High-level layout: dict nodes/edges -> positioned nodes/edges with offset.

    algorithm: "elk" (layered/Sugiyama, brandes_koepf + orthogonal) or "dagre"
               (dagre-like: simple placement + polyline). Use options=PyLayoutOptions
               to override any dagre preset (e.g. dagre + orthogonal).
    """
    if algorithm is not None:
        algo = algorithm.lower()
        if algo not in ("elk", "dagre", "layered"):
            raise ValueError(f"unknown algorithm: {algorithm!r} (use elk/dagre)")
        if algo == "dagre":
            # dagre emulation: simple placement + polyline edges (no orthogonal bends)
            if layering == "network_simplex":
                layering = "network_simplex"
            if node_placement == "brandes_koepf":
                node_placement = "simple"
            if edge_routing == "orthogonal":
                edge_routing = "polyline"

    if not nodes:
        return {"nodes": [], "edges": [], "width": 0.0, "height": 0.0}

    id_to_u32: dict[str | int, int] = {}
    u32_to_str: dict[int, str] = {}
    for idx, n in enumerate(nodes):
        raw = n["id"]
        u = idx + 1
        id_to_u32[raw] = u
        u32_to_str[u] = str(raw)

    edge_u32_to_str: dict[int, str] = {}
    for idx, e in enumerate(edges):
        raw = e["id"]
        u = idx + 1
        edge_u32_to_str[u] = str(raw)

    specs: list[NodeSpec] = []
    for n in nodes:
        u = id_to_u32[n["id"]]
        w = float(n.get("width", 100))
        h = float(n.get("height", 40))
        specs.append(NodeSpec(u, w, h))

    edge_specs: list[EdgeSpec] = []
    for idx, e in enumerate(edges):
        u = idx + 1
        s = id_to_u32[e["source"]]
        t = id_to_u32[e["target"]]
        edge_specs.append(EdgeSpec(u, s, t))

    if options is not None:
        result: LayoutResult = layout_with_options_py(specs, edge_specs, options)
    elif (
        direction != "RIGHT"
        or layering != "network_simplex"
        or node_placement != "brandes_koepf"
        or edge_routing != "orthogonal"
        or cycle_breaking != "greedy"
        or spacing_node_node != 20.0
        or spacing_node_between_layers != 20.0
        or padding != 12.0
        or thoroughness != 7
        or random_seed != 1
    ):
        opts = PyLayoutOptions(
            direction=direction,
            layering=layering,
            node_placement=node_placement,
            edge_routing=edge_routing,
            cycle_breaking=cycle_breaking,
            node_node=spacing_node_node,
            node_node_between_layers=spacing_node_between_layers,
            padding=padding,
            thoroughness=thoroughness,
            random_seed=random_seed,
        )
        result = layout_with_options_py(specs, edge_specs, opts)
    else:
        result = layout_flat_py(specs, edge_specs)

    ox, oy = offset
    positioned_nodes: list[dict] = []
    for nid, x, y, w, h in zip(
        result.node_ids,
        result.node_x,
        result.node_y,
        result.node_width,
        result.node_height,
        strict=True,
    ):
        positioned_nodes.append(
            {"id": u32_to_str[nid], "x": x + ox, "y": y + oy, "width": w, "height": h}
        )

    positioned_edges: list[dict] = []
    for eid, src, tgt, start, length in zip(
        result.edge_ids,
        result.edge_source,
        result.edge_target,
        result.edge_bend_start,
        result.edge_bend_length,
        strict=True,
    ):
        bends: list[tuple[float, float]] = []
        for j in range(start, start + length):
            bends.append((result.bend_x[j] + ox, result.bend_y[j] + oy))
        positioned_edges.append(
            {
                "id": edge_u32_to_str[eid],
                "source": u32_to_str[src],
                "target": u32_to_str[tgt],
                "bends": bends,
            }
        )

    return {
        "nodes": positioned_nodes,
        "edges": positioned_edges,
        "width": result.width,
        "height": result.height,
    }
