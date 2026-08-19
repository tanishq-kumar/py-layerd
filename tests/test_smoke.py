from __future__ import annotations

from py_layerd._core import EdgeSpec, NodeSpec, layout_flat_py
from py_layerd.layout import layout


def test_chain_3_positions_exist():
    nodes = [NodeSpec(1, 100, 40), NodeSpec(2, 100, 40), NodeSpec(3, 100, 40)]
    edges = [EdgeSpec(1, 1, 2), EdgeSpec(2, 1, 3)]
    r = layout_flat_py(nodes, edges)
    assert len(r.node_ids) == 3
    assert len(r.edge_ids) == 2
    assert r.width > 0 and r.height > 0
    for x, y in zip(r.node_x, r.node_y, strict=True):
        assert x >= 0 and y >= 0


def test_offset_shifts_positions():
    base = layout(
        nodes=[{"id": "a", "width": 100, "height": 40}, {"id": "b", "width": 100, "height": 40}],
        edges=[{"id": "e1", "source": "a", "target": "b"}],
        offset=(0, 0),
    )
    shifted = layout(
        nodes=[{"id": "a", "width": 100, "height": 40}, {"id": "b", "width": 100, "height": 40}],
        edges=[{"id": "e1", "source": "a", "target": "b"}],
        offset=(100, 200),
    )
    for b, s in zip(base["nodes"], shifted["nodes"], strict=True):
        assert s["x"] == b["x"] + 100
        assert s["y"] == b["y"] + 200


def test_empty_graph():
    assert layout(nodes=[], edges=[]) == {"nodes": [], "edges": [], "width": 0.0, "height": 0.0}
    r = layout_flat_py([], [])
    assert r.node_ids == []
    assert r.edge_ids == []
