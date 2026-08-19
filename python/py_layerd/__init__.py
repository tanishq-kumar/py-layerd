from __future__ import annotations

from py_layerd._core import (
    EdgeSpec,
    LayoutResult,
    NodeSpec,
    PyLayoutOptions,
    layout_bytes_py,
    layout_flat_py,
    layout_with_options_py,
)

__all__ = [
    "EdgeSpec",
    "LayoutResult",
    "NodeSpec",
    "PyLayoutOptions",
    "layout",
    "layout_bytes",
    "layout_flat",
]

from py_layerd import adapters  # noqa: F401
from py_layerd.layout import layout


def layout_flat(nodes: list[NodeSpec], edges: list[EdgeSpec]) -> LayoutResult:
    return layout_flat_py(nodes, edges)


def layout_bytes(nodes: list[NodeSpec], edges: list[EdgeSpec]) -> bytes:
    return bytes(layout_bytes_py(nodes, edges))


def layout_with_options(
    nodes: list[NodeSpec], edges: list[EdgeSpec], options: PyLayoutOptions
) -> LayoutResult:
    return layout_with_options_py(nodes, edges, options)
