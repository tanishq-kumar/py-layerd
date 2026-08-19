use smallvec::SmallVec;

use crate::{
    graph::{LGraph, index::NodeId},
    options::enums::InLayerConstraint,
    properties::internal::IN_LAYER_CONSTRAINT,
};

/// Reorders nodes within each layer to satisfy in-layer constraints.
///
/// Nodes with `InLayerConstraint::Top` are moved to the top of their layer,
/// and nodes with `InLayerConstraint::Bottom` are moved to the bottom,
/// preserving their relative ordering within each group.
///
/// In-layer-constraint processor.
pub fn process(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        let mut top_nodes = Vec::new();
        let mut middle_nodes = Vec::new();
        let mut bottom_nodes = Vec::new();

        for &node_id in &nodes {
            let constraint = graph.node(node_id).properties.get(&IN_LAYER_CONSTRAINT);
            match constraint {
                InLayerConstraint::Top => top_nodes.push(node_id),
                InLayerConstraint::Bottom => bottom_nodes.push(node_id),
                InLayerConstraint::None => middle_nodes.push(node_id),
            }
        }

        let mut new_nodes = Vec::with_capacity(nodes.len());
        new_nodes.extend_from_slice(&top_nodes);
        new_nodes.extend_from_slice(&middle_nodes);
        new_nodes.extend_from_slice(&bottom_nodes);
        graph.layers[layer_idx].nodes = new_nodes;
    }
}
