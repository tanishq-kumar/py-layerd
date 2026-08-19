from __future__ import annotations

from py_layerd._core import EdgeSpec, NodeSpec

"""Shared IR helpers for adapters: generic graph construction.

Future place for frozenset/lru helpers when adapters grow.
"""


def build_specs(
    nodes: list[dict],
    edges: list[dict],
    id_to_u32: dict[str, int] | None = None,
) -> tuple[list[NodeSpec], list[EdgeSpec], dict[str, int], dict[int, str], dict[int, str]]:
    if id_to_u32 is None:
        id_to_u32 = {str(n["id"]): i + 1 for i, n in enumerate(nodes)}
    u32_to_str = {v: k for k, v in id_to_u32.items()}
    edge_u32_to_str = {i + 1: str(e["id"]) for i, e in enumerate(edges)}
    specs = [
        NodeSpec(id_to_u32[str(n["id"])], float(n.get("width", 100)), float(n.get("height", 40)))
        for n in nodes
    ]
    edge_specs = [
        EdgeSpec(i + 1, id_to_u32[str(e["source"])], id_to_u32[str(e["target"])])
        for i, e in enumerate(edges)
    ]
    return specs, edge_specs, id_to_u32, u32_to_str, edge_u32_to_str
