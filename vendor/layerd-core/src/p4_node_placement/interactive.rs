//! Interactive node placement.
//!
//! Preserves pre-existing y coordinates for regular nodes. For dummy
//! nodes, uses the `ORIGINAL_DUMMY_NODE_POSITION` hint recorded by an
//! interactive crossing minimizer, or falls back to a minValidY-based
//! stacking rule. Shifts nodes downward only when they would overlap
//! the previous node in the layer.

use crate::graph::{LGraph, index::NodeId, node::NodeType};

/// Entry point for the interactive placer.
pub fn place_nodes(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }
    let mut max_height = 0.0f64;
    for layer_idx in 0..graph.layers.len() {
        place_layer(graph, layer_idx);
        let layer_bottom = compute_layer_bottom(graph, layer_idx);
        graph.layers[layer_idx].size.y = layer_bottom;
        if layer_bottom > max_height {
            max_height = layer_bottom;
        }
    }
    graph.size.y = max_height;
}

fn place_layer(graph: &mut LGraph, layer_idx: usize) {
    let node_ids: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
    let mut min_valid_y = f64::NEG_INFINITY;
    let mut prev_node_type = NodeType::Normal;

    for nid in node_ids {
        let node_type = graph.node(nid).node_type;
        let original_y = graph
            .node(nid)
            .properties
            .get(&crate::properties::internal::ORIGINAL_DUMMY_NODE_POSITION);

        // For non-normal nodes, prefer the original dummy position if
        // available; otherwise stack above `minValidY` with a type-aware
        // vertical gap.
        if node_type != NodeType::Normal {
            if let Some(original) = original_y {
                graph.node_mut(nid).position.y = original;
            } else {
                let adjusted =
                    if min_valid_y.is_infinite() { 0.0f64.max(min_valid_y) } else { min_valid_y };
                let spacing = type_vertical_spacing(graph, node_type, prev_node_type);
                graph.node_mut(nid).position.y = adjusted + spacing;
            }
        }

        // If the node overlaps the previous one, shift it down enough to
        // clear: when `position.y < min_valid_y + spacing + margin.top`,
        // pin it to the lower bound.
        let spacing = type_vertical_spacing(graph, node_type, prev_node_type);
        let node = graph.node(nid);
        let lower_bound = min_valid_y + spacing + node.margin.top;
        if node.position.y < lower_bound {
            graph.node_mut(nid).position.y = lower_bound;
        }

        let node = graph.node(nid);
        min_valid_y = node.position.y + node.size.y + node.margin.bottom;
        prev_node_type = node_type;
    }
}

fn compute_layer_bottom(graph: &LGraph, layer_idx: usize) -> f64 {
    let mut bottom = 0.0f64;
    for &nid in &graph.layers[layer_idx].nodes {
        let node = graph.node(nid);
        let candidate = node.position.y + node.size.y + node.margin.bottom;
        if candidate > bottom {
            bottom = candidate;
        }
    }
    bottom
}

/// Vertical spacing between two node types.
///
/// Returns the vertical spacing between two adjacent nodes given their
/// `NodeType`s, taking the subset of `getVerticalSpacing(NodeType, NodeType)`
/// reachable after the pre-P4 intermediate stack has run.
fn type_vertical_spacing(graph: &LGraph, t1: NodeType, t2: NodeType) -> f64 {
    use NodeType::*;
    let sp = &graph.options.spacing;
    match (t1, t2) {
        (Normal, Normal) => sp.node_node,
        (Normal, LongEdge) | (LongEdge, Normal) => sp.edge_node,
        (Normal, NorthSouthPort) | (NorthSouthPort, Normal) => sp.edge_node,
        (Normal, ExternalPort) | (ExternalPort, Normal) => sp.edge_node,
        (Normal, Label) | (Label, Normal) => sp.node_node,
        (LongEdge, LongEdge) => sp.edge_edge,
        (LongEdge, NorthSouthPort) | (NorthSouthPort, LongEdge) => sp.edge_edge,
        (LongEdge, Label) | (Label, LongEdge) => sp.edge_node,
        (NorthSouthPort, NorthSouthPort) => sp.edge_edge,
        (NorthSouthPort, Label) | (Label, NorthSouthPort) => sp.label_node,
        (ExternalPort, ExternalPort) => sp.port_port,
        (Label, Label) => sp.edge_edge,
        (BreakingPoint, BreakingPoint) => sp.edge_edge,
        (BreakingPoint, Normal) | (Normal, BreakingPoint) => sp.edge_node,
        (BreakingPoint, LongEdge) | (LongEdge, BreakingPoint) => sp.edge_node,
        _ => sp.edge_edge,
    }
}
