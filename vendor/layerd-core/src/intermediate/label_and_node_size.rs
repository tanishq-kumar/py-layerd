//! Label placement and node size adjustment entry point.
//!
//! Thin adapter that delegates to [`node_dimension_calculation::calculate`],
//! which implements the cell-system based label and node-size pass. Also
//! places labels on external-port dummy nodes so the downstream
//! HierarchicalNodeResizer sees correct margins.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{LabelId, NodeId},
        node::NodeType,
        port::PortSide,
    },
    intermediate::node_dimension_calculation,
    math::Vec2,
    options::enums::PortLabelPlacement,
    properties::{
        graph_properties::GraphProperties,
        internal::{EXT_PORT_SIDE, GRAPH_PROPERTIES},
    },
};

/// Runs the processor: sizes each non-dummy node, positions inside labels,
/// places ports, and then handles external-port dummy labels.
pub fn process(graph: &mut LGraph) {
    node_dimension_calculation::calculate(graph);

    let graph_props = graph.properties.get(&GRAPH_PROPERTIES);
    if !graph_props.contains(GraphProperties::EXTERNAL_PORTS) {
        return;
    }

    let port_label_placement = graph.options.port_labels_placement;
    let label_port_h = graph.options.spacing.label_port_horizontal;
    let label_port_v = graph.options.spacing.label_port_vertical;
    let label_label = graph.options.spacing.label_label;

    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for node_id in nodes {
            if graph.node(node_id).node_type != NodeType::ExternalPort {
                continue;
            }
            place_external_port_dummy_labels(
                graph,
                node_id,
                port_label_placement,
                label_port_h,
                label_port_v,
                label_label,
            );
        }
    }
}

/// Position the labels on an external-port dummy's (single) port.
///
/// `placement` is a bitset; the function branches on `INSIDE` vs the rest
/// and reads `NEXT_TO_PORT_IF_POSSIBLE` / `ALWAYS_SAME_SIDE` flags.
fn place_external_port_dummy_labels(
    graph: &mut LGraph,
    dummy_node: NodeId,
    placement: PortLabelPlacement,
    label_port_h: f64,
    label_port_v: f64,
    label_label: f64,
) {
    // External port dummies carry exactly one port.
    let Some(&dummy_port) = graph.node(dummy_node).ports.first() else {
        return;
    };
    let port_pos = graph.port(dummy_port).position;
    let dummy_size = graph.node(dummy_node).size;
    let side = graph.node(dummy_node).properties.get(&EXT_PORT_SIDE);

    let label_ids: SmallVec<LabelId, 3> = graph.port(dummy_port).labels.iter().copied().collect();
    if label_ids.is_empty() {
        return;
    }

    // Compute label cell size: stack vertically, width = max of label widths.
    let mut box_w = 0.0_f64;
    let mut box_h = 0.0_f64;
    for &lid in &label_ids {
        let sz = graph.label(lid).size;
        box_w = box_w.max(sz.x);
        box_h += sz.y;
    }
    box_h += (label_ids.len().saturating_sub(1) as f64) * label_label;

    // Place the cell (x, y) in the dummy's local coordinate system.
    let inside = placement.contains(PortLabelPlacement::INSIDE);
    let place_next_to_port = placement.contains(PortLabelPlacement::NEXT_TO_PORT_IF_POSSIBLE);
    let next_to_port = if inside {
        place_next_to_port
            && graph.port(dummy_port).incoming_edges.is_empty()
            && graph.port(dummy_port).outgoing_edges.is_empty()
    } else {
        place_next_to_port && !graph.port(dummy_port).connected_to_external_nodes
    };

    let (box_x, box_y) = if inside {
        match side {
            PortSide::North => ((dummy_size.x - box_w) / 2.0 - port_pos.x, label_port_v),
            PortSide::South => ((dummy_size.x - box_w) / 2.0 - port_pos.x, -label_port_v - box_h),
            PortSide::East => {
                let y = if next_to_port {
                    let first_label_h = graph.label(label_ids[0]).size.y;
                    let label_h = if graph.options.port_labels_treat_as_group {
                        box_h
                    } else {
                        first_label_h
                    };
                    (dummy_size.y - label_h) / 2.0 - port_pos.y
                } else {
                    dummy_size.y + label_port_v - port_pos.y
                };
                (-label_port_h - box_w, y)
            }
            PortSide::West => {
                let y = if next_to_port {
                    let first_label_h = graph.label(label_ids[0]).size.y;
                    let label_h = if graph.options.port_labels_treat_as_group {
                        box_h
                    } else {
                        first_label_h
                    };
                    (dummy_size.y - label_h) / 2.0 - port_pos.y
                } else {
                    dummy_size.y + label_port_v - port_pos.y
                };
                (label_port_h, y)
            }
            _ => return,
        }
    } else if placement.contains(PortLabelPlacement::OUTSIDE) {
        match side {
            PortSide::North | PortSide::South => {
                let x = port_pos.x + label_port_h;
                (x, 0.0)
            }
            PortSide::East | PortSide::West => {
                let y = if next_to_port {
                    let first_label_h = graph.label(label_ids[0]).size.y;
                    let label_h = if graph.options.port_labels_treat_as_group {
                        box_h
                    } else {
                        first_label_h
                    };
                    (dummy_size.y - label_h) / 2.0 - port_pos.y
                } else {
                    port_pos.y + label_port_v
                };
                (0.0, y)
            }
            _ => return,
        }
    } else {
        // Only INSIDE and OUTSIDE are special-cased; fixed/empty placement
        // leaves the freshly-created box at (0, 0) and still applies label
        // layout below.
        (0.0, 0.0)
    };

    // Stack labels vertically inside the cell.
    let mut cursor_y = box_y;
    for lid in label_ids {
        graph.label_mut(lid).position = Vec2::new(box_x, cursor_y);
        cursor_y += graph.label(lid).size.y + label_label;
    }

    // Expand the dummy node's margin to include the cell so the hierarchical
    // resizer reserves space for it.
    let margin = &mut graph.node_mut(dummy_node).margin;
    if box_x < 0.0 && -box_x > margin.left {
        margin.left = -box_x;
    }
    let box_right = box_x + box_w;
    if box_right > dummy_size.x {
        let needed = box_right - dummy_size.x;
        if needed > margin.right {
            margin.right = needed;
        }
    }
    if box_y < 0.0 && -box_y > margin.top {
        margin.top = -box_y;
    }
    let box_bottom = box_y + box_h;
    if box_bottom > dummy_size.y {
        let needed = box_bottom - dummy_size.y;
        if needed > margin.bottom {
            margin.bottom = needed;
        }
    }
}
