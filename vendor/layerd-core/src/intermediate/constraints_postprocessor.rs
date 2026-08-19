//! Constraint metadata postprocessing.
//!
//! Writes per-node `LAYERING_LAYER_ID` and `CROSSING_MINIMIZATION_POSITION_ID`
//! properties so that callers can read the final layer / intra-layer position
//! the algorithm assigned. Only counts `Normal` nodes; layers consisting
//! solely of dummies do not advance the layer id counter.

use crate::{
    graph::{LGraph, node::NodeType},
    properties::internal::{CROSSING_MINIMIZATION_POSITION_ID, LAYERING_LAYER_ID},
};

/// Assigns `LAYERING_LAYER_ID` and `CROSSING_MINIMIZATION_POSITION_ID` to every
/// normal node.
pub fn process(graph: &mut LGraph) {
    let mut layer_index: i32 = 0;
    for layer_idx in 0..graph.layers.len() {
        let mut pos_index: i32 = 0;
        let mut any_normal_node = false;
        let node_ids: Vec<_> = graph.layers[layer_idx].nodes.iter().copied().collect();
        for node_id in node_ids {
            if graph.node(node_id).node_type == NodeType::Normal {
                any_normal_node = true;
                graph.node_mut(node_id).properties.set(&LAYERING_LAYER_ID, layer_index);
                graph
                    .node_mut(node_id)
                    .properties
                    .set(&CROSSING_MINIMIZATION_POSITION_ID, pos_index);
                pos_index += 1;
            }
        }
        if any_normal_node {
            layer_index += 1;
        }
    }
}
