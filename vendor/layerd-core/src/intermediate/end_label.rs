//! End-label pre/post processors.
//!
//! This pair of processors uses a direct per-port computation: preprocess
//! computes label positions relative to their owning node's top-left corner,
//! and the post-processor adds the node's final position. Node margins are
//! widened so downstream passes reserve room for the labels. Overlap removal
//! between same-side cells is deferred — typical fixtures have at most one
//! or two end labels per port side and do not collide.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::{Margin, Vec2},
    options::enums::{EdgeLabelPlacement, LabelSide, LayoutDirection},
    properties::internal::{
        EDGE_LABEL_PLACEMENT, EDGE_THICKNESS, END_LABELS, LABEL_SIDE, ORIGIN_PORT,
    },
};

/// Collect HEAD labels on `port`'s incoming edges and TAIL labels on its
/// outgoing edges.
fn gather_labels_from_port(
    graph: &LGraph,
    port_id: PortId,
    labels: &mut Vec<LabelId>,
) -> Option<f64> {
    let mut max_edge_thickness: Option<f64> = None;
    let incoming: SmallVec<EdgeId, 4> =
        graph.port(port_id).incoming_edges.iter().copied().collect();
    for eid in &incoming {
        let thickness = graph.edge(*eid).properties.get(&EDGE_THICKNESS);
        max_edge_thickness = Some(max_edge_thickness.map_or(thickness, |max| max.max(thickness)));
        let label_ids: SmallVec<LabelId, 3> = graph.edge(*eid).labels.iter().copied().collect();
        for lid in label_ids {
            if graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Head {
                labels.push(lid);
            }
        }
    }
    let outgoing: SmallVec<EdgeId, 4> =
        graph.port(port_id).outgoing_edges.iter().copied().collect();
    for eid in &outgoing {
        let thickness = graph.edge(*eid).properties.get(&EDGE_THICKNESS);
        max_edge_thickness = Some(max_edge_thickness.map_or(thickness, |max| max.max(thickness)));
        let label_ids: SmallVec<LabelId, 3> = graph.edge(*eid).labels.iter().copied().collect();
        for lid in label_ids {
            if graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Tail {
                labels.push(lid);
            }
        }
    }
    max_edge_thickness
}

/// Compute per-label positions relative to each node's top-left corner and
/// reserve room for them in the node's margin. Runs before P4.
pub fn preprocess(graph: &mut LGraph) {
    let edge_label_spacing = graph.options.spacing.edge_label;
    let label_label_spacing = graph.options.spacing.label_label;
    let vertical_layout =
        matches!(graph.options.direction, LayoutDirection::Down | LayoutDirection::Up);

    for layer_idx in 0..graph.layers.len() {
        let layer_nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for &node_id in &layer_nodes {
            // Walk every layered node, not just `NodeType::Normal`.
            // External-port nodes and long-edge dummies are eligible
            // end-label hosts when their incident edges (or, for north/south
            // ports, their PORT_DUMMY detour) carry HEAD/TAIL labels.
            let base_margin = *graph.node(node_id).margin;
            let node_size = graph.node(node_id).size;
            let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
            let mut cells: SmallVec<EndLabelCell, 4> = SmallVec::new();
            for port_id in ports {
                if let Some(cell) = place_port_end_labels(
                    graph,
                    node_id,
                    port_id,
                    edge_label_spacing,
                    label_label_spacing,
                    base_margin,
                    vertical_layout,
                ) {
                    cells.push(cell);
                }
            }
            if !cells.is_empty() {
                remove_label_overlaps(
                    &mut cells,
                    PortSide::North,
                    label_label_spacing,
                    edge_label_spacing,
                    base_margin,
                    node_size,
                );
                remove_label_overlaps(
                    &mut cells,
                    PortSide::South,
                    label_label_spacing,
                    edge_label_spacing,
                    base_margin,
                    node_size,
                );
                remove_label_overlaps(
                    &mut cells,
                    PortSide::East,
                    label_label_spacing,
                    edge_label_spacing,
                    base_margin,
                    node_size,
                );
                remove_label_overlaps(
                    &mut cells,
                    PortSide::West,
                    label_label_spacing,
                    edge_label_spacing,
                    base_margin,
                    node_size,
                );
                apply_cell_label_positions(graph, &cells);
                update_node_margins(graph, node_id, node_size, base_margin, &cells);
                graph.node_mut(node_id).properties.set(&END_LABELS, true);
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum HorizontalAlignment {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy)]
enum VerticalAlignment {
    Top,
    Bottom,
}

#[derive(Debug, Clone)]
struct EndLabelCell {
    side: PortSide,
    labels: Vec<LabelId>,
    horizontal_layout: bool,
    horizontal_alignment: HorizontalAlignment,
    vertical_alignment: VerticalAlignment,
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

/// Compute cell size + layout for a single port's end labels and update the
/// node's margin to enclose the cell. Returns `true` if at least one label
/// was placed. Positions are written relative to the node's top-left corner;
/// the postprocessor later adds the node's final position.
fn place_port_end_labels(
    graph: &mut LGraph,
    node_id: NodeId,
    port_id: PortId,
    edge_label_spacing: f64,
    label_label_spacing: f64,
    node_margin: Margin,
    vertical_layout: bool,
) -> Option<EndLabelCell> {
    let port_side = graph.port(port_id).side;
    let port_pos = graph.port(port_id).position;
    let port_anchor = graph.port(port_id).anchor;
    let anchor_x = port_pos.x + port_anchor.x;
    let anchor_y = port_pos.y + port_anchor.y;
    let node_size = graph.node(node_id).size;

    // Gather HEAD labels on incoming + TAIL labels on outgoing edges. Also
    // visit a dummy node's ports (for north/south ports rerouted through a
    // `PORT_DUMMY`) and pick up the labels carried on those dummy edges.
    // Without this branch, end labels for ports whose edges have been
    // rerouted disappear from the layout.
    let mut labels: Vec<LabelId> = Vec::new();
    let mut max_edge_thickness = gather_labels_from_port(graph, port_id, &mut labels);
    if let Some(dummy_node) = graph.port(port_id).port_dummy
        && let Some(dummy_data) = graph.try_node(dummy_node)
    {
        // PORT_DUMMY can point at a node in a different arena under
        // cross-hierarchy; only traverse it when it lives in this graph.
        let dummy_ports: SmallVec<PortId, 4> = dummy_data.ports.iter().copied().collect();
        for dpid in dummy_ports {
            let Some(dummy_port_data) = graph.try_port(dpid) else {
                continue;
            };
            let origin: Option<PortId> = dummy_port_data.properties.get(&ORIGIN_PORT);
            if origin == Some(port_id)
                && let Some(thickness) = gather_labels_from_port(graph, dpid, &mut labels)
            {
                max_edge_thickness =
                    Some(max_edge_thickness.map_or(thickness, |max| max.max(thickness)));
            }
        }
    }

    if labels.is_empty() {
        return None;
    }
    let max_edge_thickness = max_edge_thickness.unwrap_or(0.0);
    let label_side = graph.label(labels[0]).properties.get(&LABEL_SIDE);

    // End-label cells use the graph layout direction, not the port side, to
    // decide whether labels stack vertically (horizontal graph) or horizontally
    // (vertical graph).
    let horizontal_layout = !vertical_layout;
    let mut cell_w = 0.0_f64;
    let mut cell_h = 0.0_f64;
    for &lid in &labels {
        let sz = graph.label(lid).size;
        if horizontal_layout {
            cell_w = cell_w.max(sz.x);
            cell_h += sz.y + label_label_spacing;
        } else {
            cell_h = cell_h.max(sz.y);
            cell_w += sz.x + label_label_spacing;
        }
    }
    if horizontal_layout {
        cell_h -= label_label_spacing;
    } else {
        cell_w -= label_label_spacing;
    }
    cell_h = cell_h.max(0.0);
    cell_w = cell_w.max(0.0);

    // Cell origin (top-left) relative to node's top-left corner.
    let side_above = label_side == LabelSide::Above;
    let (cell_x, cell_y, horizontal_alignment, vertical_alignment) = match port_side {
        PortSide::North => {
            let y = -node_margin.top - edge_label_spacing - cell_h;
            let x = if side_above {
                anchor_x - max_edge_thickness - edge_label_spacing - cell_w
            } else {
                anchor_x + max_edge_thickness + edge_label_spacing
            };
            let h = if side_above { HorizontalAlignment::Right } else { HorizontalAlignment::Left };
            (x, y, h, VerticalAlignment::Bottom)
        }
        PortSide::South => {
            let y = node_size.y + node_margin.bottom + edge_label_spacing;
            let x = if side_above {
                anchor_x - max_edge_thickness - edge_label_spacing - cell_w
            } else {
                anchor_x + max_edge_thickness + edge_label_spacing
            };
            let h = if side_above { HorizontalAlignment::Right } else { HorizontalAlignment::Left };
            (x, y, h, VerticalAlignment::Top)
        }
        PortSide::East => {
            let x = node_size.x + node_margin.right + edge_label_spacing;
            let y = if side_above {
                anchor_y - max_edge_thickness - edge_label_spacing - cell_h
            } else {
                anchor_y + max_edge_thickness + edge_label_spacing
            };
            let v = if side_above { VerticalAlignment::Bottom } else { VerticalAlignment::Top };
            (x, y, HorizontalAlignment::Left, v)
        }
        PortSide::West => {
            let x = -node_margin.left - edge_label_spacing - cell_w;
            let y = if side_above {
                anchor_y - max_edge_thickness - edge_label_spacing - cell_h
            } else {
                anchor_y + max_edge_thickness + edge_label_spacing
            };
            let v = if side_above { VerticalAlignment::Bottom } else { VerticalAlignment::Top };
            (x, y, HorizontalAlignment::Right, v)
        }
        _ => return None,
    };

    Some(EndLabelCell {
        side: port_side,
        labels,
        horizontal_layout,
        horizontal_alignment,
        vertical_alignment,
        x: cell_x,
        y: cell_y,
        w: cell_w,
        h: cell_h,
    })
}

fn remove_label_overlaps(
    cells: &mut [EndLabelCell],
    side: PortSide,
    label_label_spacing: f64,
    edge_label_spacing: f64,
    node_margin: Margin,
    node_size: Vec2,
) {
    let mut side_indices: Vec<usize> = cells
        .iter()
        .enumerate()
        .filter_map(|(idx, cell)| (cell.side == side).then_some(idx))
        .collect();
    if side_indices.len() < 2 {
        return;
    }

    match side {
        PortSide::North | PortSide::South => {
            side_indices.sort_by(|&a, &b| {
                cells[a].x.partial_cmp(&cells[b].x).unwrap_or(std::cmp::Ordering::Equal)
            });
            let gap = 2.0 * label_label_spacing;
            let mut placed: Vec<(usize, f64)> = Vec::with_capacity(side_indices.len());
            for idx in side_indices {
                let mut offset = 0.0;
                for &(placed_idx, placed_offset) in &placed {
                    if horizontal_overlap(&cells[idx], &cells[placed_idx]) {
                        let required = placed_offset + cells[placed_idx].h + gap;
                        if offset < required && offset + cells[idx].h + gap > placed_offset {
                            offset = required;
                        }
                    }
                }
                placed.push((idx, offset));
                let start = if side == PortSide::North {
                    -node_margin.top - edge_label_spacing
                } else {
                    node_size.y + node_margin.bottom + edge_label_spacing
                };
                if side == PortSide::North {
                    cells[idx].y = start - cells[idx].h - offset;
                } else {
                    cells[idx].y = start + offset;
                }
            }
        }
        PortSide::East | PortSide::West => {
            side_indices.sort_by(|&a, &b| {
                cells[a].y.partial_cmp(&cells[b].y).unwrap_or(std::cmp::Ordering::Equal)
            });
            let gap = 2.0 * label_label_spacing;
            let mut placed: Vec<(usize, f64)> = Vec::with_capacity(side_indices.len());
            for idx in side_indices {
                let mut offset = 0.0;
                for &(placed_idx, placed_offset) in &placed {
                    if vertical_overlap(&cells[idx], &cells[placed_idx]) {
                        let required = placed_offset + cells[placed_idx].w + gap;
                        if offset < required && offset + cells[idx].w + gap > placed_offset {
                            offset = required;
                        }
                    }
                }
                placed.push((idx, offset));
                let start = if side == PortSide::West {
                    -node_margin.left - edge_label_spacing
                } else {
                    node_size.x + node_margin.right + edge_label_spacing
                };
                if side == PortSide::West {
                    cells[idx].x = start - cells[idx].w - offset;
                } else {
                    cells[idx].x = start + offset;
                }
            }
        }
        PortSide::Undefined => {}
    }
}

fn horizontal_overlap(a: &EndLabelCell, b: &EndLabelCell) -> bool {
    !(b.x + b.w < a.x || a.x + a.w < b.x)
}

fn vertical_overlap(a: &EndLabelCell, b: &EndLabelCell) -> bool {
    !(b.y + b.h < a.y || a.y + a.h < b.y)
}

fn apply_cell_label_positions(graph: &mut LGraph, cells: &[EndLabelCell]) {
    let label_label_spacing = graph.options.spacing.label_label;
    for cell in cells {
        if cell.horizontal_layout {
            let mut y = cell.y;
            for &lid in &cell.labels {
                let size = graph.label(lid).size;
                let x = match cell.horizontal_alignment {
                    HorizontalAlignment::Left => cell.x,
                    HorizontalAlignment::Right => cell.x + cell.w - size.x,
                };
                graph.label_mut(lid).position = Vec2::new(x, y);
                y += size.y + label_label_spacing;
            }
        } else {
            let mut x = cell.x;
            for &lid in &cell.labels {
                let size = graph.label(lid).size;
                let y = match cell.vertical_alignment {
                    VerticalAlignment::Top => cell.y,
                    VerticalAlignment::Bottom => cell.y + cell.h - size.y,
                };
                graph.label_mut(lid).position = Vec2::new(x, y);
                x += size.x + label_label_spacing;
            }
        }
    }
}

/// Widen node margin to cover the union of the original margin rectangle and
/// all end-label cells. Each cell is placed using the node's margin at
/// processor entry, then the margin is updated once per node.
fn update_node_margins(
    graph: &mut LGraph,
    node_id: NodeId,
    node_size: Vec2,
    base_margin: Margin,
    cells: &[EndLabelCell],
) {
    let margin = &mut graph.node_mut(node_id).margin;
    let mut union_left = -base_margin.left;
    let mut union_top = -base_margin.top;
    let mut union_right = node_size.x + base_margin.right;
    let mut union_bottom = node_size.y + base_margin.bottom;
    for cell in cells {
        union_left = union_left.min(cell.x);
        union_top = union_top.min(cell.y);
        union_right = union_right.max(cell.x + cell.w);
        union_bottom = union_bottom.max(cell.y + cell.h);
    }
    margin.left = -union_left;
    margin.top = -union_top;
    margin.right = union_right - node_size.x;
    margin.bottom = union_bottom - node_size.y;
}

/// After P5 — final node positions are known, offset every pre-placed label
/// by the node's position so the label lands at its final graph coordinate.
pub fn postprocess(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let layer_nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for &node_id in &layer_nodes {
            if !graph.node(node_id).properties.get(&END_LABELS) {
                continue;
            }
            let node_pos = graph.node(node_id).position;
            let ports: SmallVec<PortId, 6> = graph.node(node_id).ports.iter().copied().collect();
            for port_id in ports {
                offset_port_end_labels(graph, port_id, node_pos);
            }
            graph.node_mut(node_id).properties.set(&END_LABELS, false);
        }
    }
}

fn offset_port_end_labels(graph: &mut LGraph, port_id: PortId, node_pos: Vec2) {
    let incoming: SmallVec<EdgeId, 4> =
        graph.port(port_id).incoming_edges.iter().copied().collect();
    for eid in incoming {
        let label_ids: SmallVec<LabelId, 3> = graph.edge(eid).labels.iter().copied().collect();
        for lid in label_ids {
            if graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Head {
                let old = graph.label(lid).position;
                graph.label_mut(lid).position = Vec2::new(old.x + node_pos.x, old.y + node_pos.y);
            }
        }
    }
    let outgoing: SmallVec<EdgeId, 4> =
        graph.port(port_id).outgoing_edges.iter().copied().collect();
    for eid in outgoing {
        let label_ids: SmallVec<LabelId, 3> = graph.edge(eid).labels.iter().copied().collect();
        for lid in label_ids {
            if graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Tail {
                let old = graph.label(lid).position;
                graph.label_mut(lid).position = Vec2::new(old.x + node_pos.x, old.y + node_pos.y);
            }
        }
    }
}

/// Sorts each port's HEAD labels by the horizontal position of the connected
/// source node, so labels appear in a reading-friendly order. Best-effort
/// stable sort.
pub fn sort_labels(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let layer_nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for &node_id in &layer_nodes {
            if graph.node(node_id).node_type != NodeType::Normal {
                continue;
            }
            let ports: SmallVec<PortId, 6> = graph.node(node_id).ports.iter().copied().collect();
            for port_id in ports {
                sort_head_labels_for_port(graph, port_id);
            }
        }
    }
}

fn sort_head_labels_for_port(graph: &mut LGraph, port_id: PortId) {
    let incoming: SmallVec<EdgeId, 4> =
        graph.port(port_id).incoming_edges.iter().copied().collect();
    // Sort the incoming edge list by the x position of the source node so
    // later iterations consume labels in that order. Stable sort preserves
    // existing order on ties.
    let mut keyed: SmallVec<(EdgeId, f64), 4> = incoming
        .iter()
        .map(|&eid| {
            let src_port = graph.edge(eid).source;
            let src_node = graph.port(src_port).owner;
            (eid, graph.node(src_node).position.x)
        })
        .collect();
    keyed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    let new_order = keyed.iter().map(|&(e, _)| e).collect();
    graph.port_mut(port_id).incoming_edges = new_order;
}
