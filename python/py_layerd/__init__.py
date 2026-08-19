from __future__ import annotations

from py_layerd._core import EdgeSpec, LayoutResult, NodeSpec, layout_bytes_py, layout_flat_py

__all__ = ["EdgeSpec", "LayoutResult", "NodeSpec", "layout", "layout_bytes", "layout_flat"]

from py_layerd import adapters  # noqa: F401
from py_layerd.layout import layout

# Re-export for convenience
__all__.extend(["layout"])


def layout_flat(nodes: list[NodeSpec], edges: list[EdgeSpec]) -> LayoutResult:
    return layout_flat_py(nodes, edges)


def layout_bytes(nodes: list[NodeSpec], edges: list[EdgeSpec]) -> bytes:
    return bytes(layout_bytes_py(nodes, edges))
