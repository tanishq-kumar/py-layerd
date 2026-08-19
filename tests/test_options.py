from __future__ import annotations

import pytest
from py_layerd._core import PyLayoutOptions
from py_layerd.layout import layout


def _tiny():
    nodes = [
        {"id": "a", "width": 100, "height": 40},
        {"id": "b", "width": 100, "height": 40},
        {"id": "c", "width": 100, "height": 40},
    ]
    edges = [
        {"id": "e1", "source": "a", "target": "b"},
        {"id": "e2", "source": "a", "target": "c"},
    ]
    return nodes, edges


def test_direction_changes_layout():
    nodes, edges = _tiny()
    r_right = layout(nodes, edges, direction="RIGHT")
    r_down = layout(nodes, edges, direction="DOWN")
    assert r_right["nodes"] != r_down["nodes"]


def test_layering_options():
    nodes, edges = _tiny()
    for layering in ["network_simplex", "longest_path", "coffman_graham"]:
        r = layout(nodes, edges, layering=layering)
        assert len(r["nodes"]) == 3


def test_edge_routing_options():
    nodes, edges = _tiny()
    for routing in ["orthogonal", "polyline", "splines"]:
        r = layout(nodes, edges, edge_routing=routing)
        assert len(r["edges"]) == 2


def test_node_placement_options():
    nodes, edges = _tiny()
    for placement in ["brandes_koepf", "simple", "network_simplex", "linear_segments"]:
        r = layout(nodes, edges, node_placement=placement)
        assert len(r["nodes"]) == 3


def test_cycle_breaking_options():
    nodes, edges = _tiny()
    for cb in ["greedy", "depth_first"]:
        r = layout(nodes, edges, cycle_breaking=cb)
        assert len(r["nodes"]) == 3


def test_spacing_affects_dimensions():
    nodes = [{"id": str(i), "width": 60, "height": 30} for i in ["a", "b", "c", "d", "e", "f"]]
    edges = [
        {"id": "e1", "source": "a", "target": "b"},
        {"id": "e2", "source": "a", "target": "c"},
        {"id": "e3", "source": "b", "target": "d"},
        {"id": "e4", "source": "c", "target": "e"},
        {"id": "e5", "source": "d", "target": "f"},
        {"id": "e6", "source": "e", "target": "f"},
    ]
    r_tight = layout(nodes, edges, spacing_node_node=5, spacing_node_between_layers=5)
    r_wide = layout(nodes, edges, spacing_node_node=50, spacing_node_between_layers=50)
    assert r_wide["height"] > r_tight["height"]


def test_padding_and_thoroughness():
    nodes, edges = _tiny()
    r1 = layout(nodes, edges, padding=5)
    r2 = layout(nodes, edges, padding=30)
    assert r2["width"] >= r1["width"]
    # thoroughness does not crash
    layout(nodes, edges, thoroughness=1)
    layout(nodes, edges, thoroughness=10)


def test_random_seed_changes_order_but_not_crash():
    nodes, edges = _tiny()
    r1 = layout(nodes, edges, random_seed=1)
    r2 = layout(nodes, edges, random_seed=42)
    # seed affects at least one node position (crossing tie-breaks)
    assert r1["nodes"] != r2["nodes"]


def test_options_object_passthrough():
    nodes, edges = _tiny()
    opts = PyLayoutOptions(direction="DOWN", layering="longest_path")
    r = layout(nodes, edges, options=opts)
    assert len(r["nodes"]) == 3


def test_invalid_options_raise():
    with pytest.raises(ValueError, match="unknown direction"):
        PyLayoutOptions(direction="DIAGONAL")
    nodes, edges = _tiny()
    with pytest.raises(ValueError, match="unknown direction"):
        layout(nodes, edges, direction="DIAGONAL")
    with pytest.raises(ValueError, match="unknown layering"):
        layout(nodes, edges, layering="bogus")


def test_offset_still_works_with_options():
    nodes, edges = _tiny()
    r0 = layout(nodes, edges, direction="DOWN", offset=(0, 0))
    r1 = layout(nodes, edges, direction="DOWN", offset=(100, 200))
    for a, b in zip(r0["nodes"], r1["nodes"], strict=True):
        assert b["x"] == a["x"] + 100
        assert b["y"] == a["y"] + 200
