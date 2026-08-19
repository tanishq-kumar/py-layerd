use smallvec::SmallVec;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    properties::internal::{
        SPACING_EDGE_EDGE_OVERRIDE, SPACING_EDGE_NODE_OVERRIDE, SPACING_NODE_NODE_OVERRIDE,
    },
};

/// Place nodes by stacking them vertically within each layer,
/// centering layers within the tallest layer's height.
pub fn place_nodes(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    // Calculate height of each layer
    let mut max_height: f64 = 0.0;
    let mut layer_heights: Vec<f64> = Vec::new();

    for layer in &graph.layers {
        let mut height = 0.0;
        let mut prev: Option<NodeId> = None;
        for &node_id in layer.nodes.iter() {
            let node = graph.node(node_id);
            if let Some(p) = prev {
                height += pair_vertical_spacing(graph, p, node_id);
            }
            height += node.margin.top + node.size.y + node.margin.bottom;
            prev = Some(node_id);
        }
        layer_heights.push(height);
        max_height = max_height.max(height);
    }

    // Place nodes centered in tallest layer
    for (layer_idx, &layer_height) in layer_heights.iter().enumerate().take(graph.layers.len()) {
        let mut y = (max_height - layer_height) / 2.0;

        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        let mut prev: Option<NodeId> = None;
        for &node_id in node_ids.iter() {
            if let Some(p) = prev {
                y += pair_vertical_spacing(graph, p, node_id);
            }
            let node = graph.node_mut(node_id);
            y += node.margin.top;
            node.position.y = y;
            y += node.size.y + node.margin.bottom;
            prev = Some(node_id);
        }
    }

    graph.size.y = max_height;
}

/// Compute the vertical spacing between two nodes for `SimpleNodePlacer`,
/// honouring per-node `SPACING_*_OVERRIDE` entries.
///
/// The per-pair `(NodeType, NodeType)` lookup picks a base spacing key, then
/// the result is `max(individual_override(n1, key), individual_override(n2, key))`.
/// Per-property `SPACING_*_OVERRIDE` keys live in `core/src/properties/internal.rs`.
fn pair_vertical_spacing(graph: &LGraph, n1: NodeId, n2: NodeId) -> f64 {
    use NodeType::*;
    let t1 = graph.node(n1).node_type;
    let t2 = graph.node(n2).node_type;
    let sp = &graph.options.spacing;

    let (base, kind) = match (t1, t2) {
        (Normal, Normal) => (sp.node_node, SpacingKind::NodeNode),
        (Normal, Label) | (Label, Normal) => (sp.node_node, SpacingKind::NodeNode),
        (Normal, LongEdge) | (LongEdge, Normal) => (sp.edge_node, SpacingKind::EdgeNode),
        (Normal, NorthSouthPort) | (NorthSouthPort, Normal) =>
            (sp.edge_node, SpacingKind::EdgeNode),
        (Normal, ExternalPort) | (ExternalPort, Normal) => (sp.edge_node, SpacingKind::EdgeNode),
        (LongEdge, LongEdge) => (sp.edge_edge, SpacingKind::EdgeEdge),
        (LongEdge, NorthSouthPort) | (NorthSouthPort, LongEdge) =>
            (sp.edge_edge, SpacingKind::EdgeEdge),
        (LongEdge, ExternalPort) | (ExternalPort, LongEdge) =>
            (sp.edge_edge, SpacingKind::EdgeEdge),
        (LongEdge, Label) | (Label, LongEdge) => (sp.edge_node, SpacingKind::EdgeNode),
        (NorthSouthPort, NorthSouthPort) => (sp.edge_edge, SpacingKind::EdgeEdge),
        (NorthSouthPort, ExternalPort) | (ExternalPort, NorthSouthPort) =>
            (sp.edge_edge, SpacingKind::EdgeEdge),
        (NorthSouthPort, Label) | (Label, NorthSouthPort) => (sp.label_node, SpacingKind::None),
        // ExternalPort-ExternalPort pairs use `edge_edge_between_layers`, so
        // two external ports in the same layer get edge-edge spacing rather
        // than port-port.
        (ExternalPort, ExternalPort) => (sp.edge_edge_between_layers, SpacingKind::None),
        (ExternalPort, Label) | (Label, ExternalPort) =>
            (sp.label_port_vertical, SpacingKind::None),
        (Label, Label) => (sp.edge_edge, SpacingKind::EdgeEdge),
        (BreakingPoint, BreakingPoint) => (sp.edge_edge, SpacingKind::EdgeEdge),
        (BreakingPoint, Normal) | (Normal, BreakingPoint) => (sp.edge_node, SpacingKind::EdgeNode),
        (BreakingPoint, LongEdge) | (LongEdge, BreakingPoint) =>
            (sp.edge_node, SpacingKind::EdgeNode),
        (BreakingPoint, Label) | (Label, BreakingPoint) => (sp.edge_node, SpacingKind::EdgeNode),
        _ => (sp.edge_edge, SpacingKind::EdgeEdge),
    };

    match kind {
        SpacingKind::None => base,
        SpacingKind::NodeNode => max_individual(graph, n1, n2, &SPACING_NODE_NODE_OVERRIDE, base),
        SpacingKind::EdgeNode => max_individual(graph, n1, n2, &SPACING_EDGE_NODE_OVERRIDE, base),
        SpacingKind::EdgeEdge => max_individual(graph, n1, n2, &SPACING_EDGE_EDGE_OVERRIDE, base),
    }
}

#[derive(Clone, Copy)]
enum SpacingKind {
    None,
    NodeNode,
    EdgeNode,
    EdgeEdge,
}

/// Returns `max(individual_override(n1, key), individual_override(n2, key))`,
/// falling back to `base` when neither node defines an override.
fn max_individual(
    graph: &LGraph,
    n1: NodeId,
    n2: NodeId,
    key: &crate::properties::PropertyKey<Option<f64>>,
    base: f64,
) -> f64 {
    let s1 = graph.node(n1).properties.get(key).unwrap_or(base);
    let s2 = graph.node(n2).properties.get(key).unwrap_or(base);
    s1.max(s2)
}
