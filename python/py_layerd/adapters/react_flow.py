from __future__ import annotations

from typing import Any, TypedDict


class ReactFlowNode(TypedDict, total=False):
    id: str
    position: dict[str, float]
    measured: dict[str, float]
    width: float
    height: float
    data: Any
    type: str
    parentId: str


class ReactFlowEdge(TypedDict, total=False):
    id: str
    source: str
    target: str
    sourceHandle: str
    targetHandle: str
    label: str
    data: Any


def _node_size(node: dict, default_w: float = 100, default_h: float = 40) -> tuple[float, float]:
    if "measured" in node and isinstance(node["measured"], dict):
        m = node["measured"]
        w = m.get("width")
        h = m.get("height")
        if w is not None and h is not None:
            return float(w), float(h)
    w = node.get("width")
    h = node.get("height")
    if w is not None and h is not None:
        return float(w), float(h)
    style = node.get("style")
    if isinstance(style, dict):
        sw = style.get("width")
        sh = style.get("height")
        if sw is not None and sh is not None:
            return float(sw), float(sh)
    return default_w, default_h


def react_flow(
    nodes: list[dict],
    edges: list[dict],
    *,
    offset: tuple[float, float] = (0.0, 0.0),
    algorithm: str | None = None,
    direction: str = "RIGHT",
    layering: str = "network_simplex",
    node_placement: str = "brandes_koepf",
    edge_routing: str = "orthogonal",
    spacing_node_node: float = 20.0,
    spacing_node_between_layers: float = 20.0,
    padding: float = 12.0,
    default_width: float = 100,
    default_height: float = 40,
) -> tuple[list[dict], list[dict]]:
    """Layout React Flow nodes/edges. Returns (nodes_with_position, edges_with_bends).

    - Reads size from measured -> width/height -> default.
    - Preserves node data/type/parentId. Writes node['position'] = {x, y}.
    - Offset shifts all positions and bend points.
    - Accepts same spacing/direction options as py_layerd.layout.layout.
    """
    from py_layerd.layout import layout

    size_overrides: dict[str, tuple[float, float]] = {}
    for n in nodes:
        w, h = _node_size(n, default_width, default_height)
        size_overrides[str(n["id"])] = (w, h)

    # Build generic graph with explicit sizes
    generic_nodes = [
        {
            "id": str(n["id"]),
            "width": size_overrides[str(n["id"])][0],
            "height": size_overrides[str(n["id"])][1],
        }
        for n in nodes
    ]
    generic_edges = [
        {"id": str(e["id"]), "source": str(e["source"]), "target": str(e["target"])} for e in edges
    ]

    # sourceHandle/targetHandle map to ports in Rust core; for now we route via
    # generic layout (port side inferred from direction). Bend points computed by core.
    result = layout(
        generic_nodes,
        generic_edges,
        offset=offset,
        algorithm=algorithm,
        direction=direction,
        layering=layering,
        node_placement=node_placement,
        edge_routing=edge_routing,
        spacing_node_node=spacing_node_node,
        spacing_node_between_layers=spacing_node_between_layers,
        padding=padding,
    )

    pos_by_id = {n["id"]: (n["x"], n["y"], n["width"], n["height"]) for n in result["nodes"]}
    bends_by_id = {e["id"]: e["bends"] for e in result["edges"]}

    out_nodes: list[dict] = []
    for n in nodes:
        nid = str(n["id"])
        x, y, w, h = pos_by_id[nid]
        # preserve all original fields, overwrite position and measured
        out = dict(n)
        out["position"] = {"x": x, "y": y}
        out["measured"] = {"width": w, "height": h}
        out_nodes.append(out)

    out_edges: list[dict] = []
    for e in edges:
        eid = str(e["id"])
        out = dict(e)
        bends = bends_by_id.get(eid, [])
        if bends:
            out["bends"] = bends
            # also provide bendPoints for consumers expecting that key
            out["bendPoints"] = [{"x": bx, "y": by} for bx, by in bends]
        out_edges.append(out)

    return out_nodes, out_edges


def from_xyflow(*args, **kwargs):
    """Alias for react_flow — xyflow is the React Flow org/package name."""
    return react_flow(*args, **kwargs)
