//! Semi-interactive crossing minimization processor.
//!
//! Inserts in-layer successor constraints between regular (`NORMAL`) nodes whose
//! position has been pinned by the user. Sorts those nodes by ascending y and
//! constrains them pair-wise so the downstream crossing minimizer respects the
//! user-specified order.

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    properties::internal::{
        IN_LAYER_SUCCESSOR_CONSTRAINTS, IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES,
        POSITION,
    },
};

pub fn process(graph: &mut LGraph) {
    let mut added_constraints = false;

    for layer_idx in 0..graph.layers.len() {
        // Filter by `node.properties.has(POSITION)`, then sort by
        // `orig_pos.y` from that explicit POSITION vector — not the live
        // `node.position` field. Falling back to the live position would
        // (a) constrain every Normal node whose layout already moved it
        // off the origin and (b) miss explicitly pinned nodes at (0, 0).
        let mut pinned: Vec<(NodeId, f64)> = graph.layers[layer_idx]
            .nodes
            .iter()
            .copied()
            .filter(|&node_id| graph.node(node_id).node_type == NodeType::Normal)
            .filter_map(|node_id| {
                graph.node(node_id).properties.get(&POSITION).map(|pos| (node_id, pos.y))
            })
            .collect();

        pinned.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        for window in pinned.windows(2) {
            let prev = window[0].0;
            let curr = window[1].0;
            let mut existing =
                graph.node(prev).properties.get_slice(&IN_LAYER_SUCCESSOR_CONSTRAINTS).to_vec();
            existing.push(curr);
            graph.node_mut(prev).properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS, existing);
            added_constraints = true;
        }
    }

    if added_constraints {
        graph.properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES, true);
    }
}
