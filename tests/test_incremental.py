from __future__ import annotations

from py_layerd.adapters import json_canvas, react_flow
from py_layerd.layout import layout


def _base():
    nodes = [
        {"id": "a", "width": 100, "height": 40},
        {"id": "b", "width": 100, "height": 40},
    ]
    edges = [{"id": "e1", "source": "a", "target": "b"}]
    return nodes, edges


def test_fixed_keeps_pinned():
    nodes, edges = _base()
    r1 = layout(nodes, edges)
    pos = {n["id"]: (n["x"], n["y"]) for n in r1["nodes"]}
    nodes2 = [*nodes, {"id": "c", "width": 100, "height": 40}]
    edges2 = [*edges, {"id": "e2", "source": "a", "target": "c"}]
    r2 = layout(nodes2, edges2, initial_positions=pos, fixed_ids=frozenset({"a", "b"}))
    by_id = {n["id"]: n for n in r2["nodes"]}
    assert by_id["a"]["x"] == pos["a"][0] and by_id["a"]["y"] == pos["a"][1]
    assert by_id["b"]["x"] == pos["b"][0] and by_id["b"]["y"] == pos["b"][1]
    assert "c" in by_id


def test_without_fixed_recomputes():
    nodes, edges = _base()
    r1 = layout(nodes, edges)
    pos = {n["id"]: (n["x"], n["y"]) for n in r1["nodes"]}
    nodes2 = [*nodes, {"id": "c", "width": 100, "height": 40}]
    edges2 = [*edges, {"id": "e2", "source": "a", "target": "c"}]
    r2 = layout(nodes2, edges2, initial_positions=pos, fixed_ids=frozenset())
    # without fixed, layout may shift (or keep) — just ensure it runs
    assert len(r2["nodes"]) == 3


def test_offset_with_fixed_pinned_not_shifted():
    nodes, edges = _base()
    r1 = layout(nodes, edges)
    pos = {n["id"]: (n["x"], n["y"]) for n in r1["nodes"]}
    nodes2 = [*nodes, {"id": "c", "width": 100, "height": 40}]
    edges2 = [*edges, {"id": "e2", "source": "a", "target": "c"}]
    r2 = layout(nodes2, edges2, initial_positions=pos, fixed_ids={"a"}, offset=(10, 20))
    by_id = {n["id"]: n for n in r2["nodes"]}
    assert by_id["a"]["x"] == pos["a"][0]


def test_adapters_warm_start():
    nodes, edges = _base()
    r1 = layout(nodes, edges)
    pos = {n["id"]: (n["x"], n["y"]) for n in r1["nodes"]}
    nodes2 = [
        {"id": "a", "width": 100, "height": 40},
        {"id": "b", "width": 100, "height": 40},
        {"id": "c", "width": 100, "height": 40},
    ]
    edges2 = [
        {"id": "e1", "source": "a", "target": "b"},
        {"id": "e2", "source": "a", "target": "c"},
    ]
    rn, _ = react_flow(nodes2, edges2, initial_positions=pos, fixed_ids={"a", "b"})
    assert rn[0]["position"]["x"] == pos["a"][0]
    canvas = {
        "nodes": [
            {"id": "a", "type": "text", "x": 0, "y": 0, "width": 100, "height": 80},
            {"id": "b", "type": "text", "x": 0, "y": 0, "width": 100, "height": 80},
        ],
        "edges": [{"id": "e1", "fromNode": "a", "toNode": "b"}],
    }
    c1 = json_canvas(canvas)
    pos_c = {n["id"]: (n["x"], n["y"]) for n in c1["nodes"]}
    canvas2 = {
        "nodes": [
            {"id": "a", "type": "text", "x": 0, "y": 0, "width": 100, "height": 80},
            {"id": "b", "type": "text", "x": 0, "y": 0, "width": 100, "height": 80},
            {"id": "c", "type": "text", "x": 0, "y": 0, "width": 100, "height": 80},
        ],
        "edges": [
            {"id": "e1", "fromNode": "a", "toNode": "b"},
            {"id": "e2", "fromNode": "a", "toNode": "c"},
        ],
    }
    c2 = json_canvas(canvas2, initial_positions=pos_c, fixed_ids={"a", "b"})
    by = {n["id"]: n for n in c2["nodes"]}
    assert by["a"]["x"] == pos_c["a"][0]
