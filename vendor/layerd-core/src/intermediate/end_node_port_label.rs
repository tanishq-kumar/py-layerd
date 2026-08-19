//! End-node port label management processor (`centerLabels = false`).
//!
//! The non-center-labels branch applies a registered label manager to node
//! labels, port labels, and edge head/tail labels. Layerd does not wire a
//! label manager, so this processor walks the graph structure without
//! transforming label sizes. Kept as a faithful structural port so that a
//! future label manager hook can be attached without changing the pipeline.

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    options::enums::LayoutDirection,
};

/// Runs label-management iteration for everything except edge center labels.
///
/// Node labels, port labels, and edge head/tail labels are visited once per
/// layered node. With no label manager present, the visit is side-effect free.
pub fn process(graph: &mut LGraph) {
    let _vertical_layout = is_vertical(graph.options.direction);

    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            if graph.node(node_id).node_type == NodeType::Normal {
                walk_node_labels(graph, node_id);
                walk_port_labels(graph, node_id);
            }
            walk_head_tail_labels(graph, node_id);
        }
    }
}

fn is_vertical(direction: LayoutDirection) -> bool {
    matches!(direction, LayoutDirection::Up | LayoutDirection::Down)
}

/// Visit each of the node's labels. Without a label manager this is a
/// read-only walk; the hook is reserved for a future
/// `manage_label_size(origin, MIN_WIDTH_NODE_LABELS)` equivalent.
fn walk_node_labels(graph: &LGraph, node: NodeId) {
    for _label_id in graph.node(node).labels.iter() {
        // Hook point for a future `ILabelManager` equivalent.
    }
}

fn walk_port_labels(graph: &LGraph, node: NodeId) {
    for &port_id in &graph.node(node).ports {
        for _label_id in graph.port(port_id).labels.iter() {
            // Hook point for a future `ILabelManager` equivalent.
        }
    }
}

/// Walk head and tail labels on outgoing edges. Center labels have already
/// been separated onto label dummy nodes by `LabelDummyInserter` and are
/// handled by `center_label_management`.
fn walk_head_tail_labels(graph: &LGraph, node: NodeId) {
    for &port_id in &graph.node(node).ports {
        for &edge_id in &graph.port(port_id).outgoing_edges {
            for _label_id in graph.edge(edge_id).labels.iter() {
                // Hook point for a future `ILabelManager` equivalent.
            }
        }
    }
}
