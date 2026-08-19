from __future__ import annotations

from typing import Any


def json_canvas(
    canvas: dict[str, Any],
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
    initial_positions: dict | None = None,
    fixed_ids: frozenset | set | None = None,
    preserve_positions: bool = False,
) -> dict[str, Any]:
    """Layout a JSON Canvas 1.0 dict {nodes: [...], edges: [...]}.

    - Nodes: {id, type, x, y, width, height, color?, ...}. type=color group etc preserved.
    - Edges: {id, fromNode, toNode, fromSide?, toSide?, fromEnd?, toEnd?, label?, color?}
      fromSide/toSide are port hints; group type maps to hierarchy (future).
    - Group nodes (type == "group") are excluded from layout and keep their x/y;
      only non-group nodes are positioned. This preserves containers while layout
      runs on the flat graph.
    - If preserve_positions is True, incoming x/y are kept and only missing nodes are placed;
      otherwise x/y are recomputed. Offset shifts all x/y and bend points.
    - Returns new dict with updated x/y. Unknown fields (color, background) are preserved.
    """
    from py_layerd.layout import layout

    nodes_in = canvas.get("nodes") or []
    edges_in = canvas.get("edges") or []

    if not nodes_in:
        return {"nodes": [], "edges": list(edges_in)}

    # Group nodes stay at their current position; only non-group nodes are laid out.
    group_ids = {str(n["id"]) for n in nodes_in if n.get("type") == "group"}
    layout_nodes = [n for n in nodes_in if str(n["id"]) not in group_ids]
    # Edges touching a group are not laid out (group = container, not a box)
    layout_edges = [
        e
        for e in edges_in
        if str(e["fromNode"]) not in group_ids and str(e["toNode"]) not in group_ids
    ]

    if not layout_nodes:
        return {"nodes": list(nodes_in), "edges": list(edges_in)}

    generic_nodes = [
        {"id": str(n["id"]), "width": float(n["width"]), "height": float(n["height"])}
        for n in layout_nodes
    ]
    generic_edges = [
        {"id": str(e["id"]), "source": str(e["fromNode"]), "target": str(e["toNode"])}
        for e in layout_edges
    ]

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
        initial_positions=initial_positions,
        fixed_ids=fixed_ids,
    )

    pos_by_id = {n["id"]: (n["x"], n["y"]) for n in result["nodes"]}
    bends_by_id = {e["id"]: e["bends"] for e in result["edges"]}

    out_nodes: list[dict] = []
    for n in nodes_in:
        nid = str(n["id"])
        out = dict(n)
        if nid in group_ids:
            # group: keep x/y, apply offset only
            if not preserve_positions or out.get("x") is None:
                ox, oy = offset
                out["x"] = int(out.get("x", 0) + ox)
                out["y"] = int(out.get("y", 0) + oy)
            out_nodes.append(out)
            continue
        x, y = pos_by_id[nid]
        if not preserve_positions or out.get("x") is None:
            out["x"] = round(x)
            out["y"] = round(y)
        out_nodes.append(out)

    out_edges: list[dict] = []
    for e in edges_in:
        eid = str(e["id"])
        out = dict(e)
        bends = bends_by_id.get(eid, [])
        if bends:
            out["bends"] = [{"x": round(bx), "y": round(by)} for bx, by in bends]
        out_edges.append(out)

    out: dict[str, Any] = {}
    for k, v in canvas.items():
        if k not in ("nodes", "edges"):
            out[k] = v
    out["nodes"] = out_nodes
    out["edges"] = out_edges
    return out
