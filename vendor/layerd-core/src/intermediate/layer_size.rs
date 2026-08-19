//! Layer size and graph height computation.
//!
//! Sets each layer's `size.x` to the widest node-plus-margin in that layer and
//! its `size.y` to the vertical extent from the first node's top margin to the
//! last node's bottom margin. Then writes the graph's overall height and
//! shifts `graph.offset.y` so that the topmost element sits at y=0.
//!
//! External-port surrounding spacing is read from the graph-level
//! `SPACING_PORTS_SURROUNDING` property (default empty margin).

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    math::Vec2,
    properties::internal::SPACING_PORTS_SURROUNDING,
};

/// Compute layer sizes and graph height after node placement.
pub fn calculate(graph: &mut LGraph) {
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut found_nodes = false;

    // SPACING_PORTS_SURROUNDING is read once per graph at the top of the
    // method (not per layer), so hoist it out of the inner loop.
    let port_surrounding = graph.properties.get(&SPACING_PORTS_SURROUNDING);

    let num_layers = graph.layers.len();
    for layer_idx in 0..num_layers {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        graph.layers[layer_idx].size = Vec2::ZERO;
        if nodes.is_empty() {
            continue;
        }

        found_nodes = true;

        // Layer width: widest node + horizontal margins.
        let mut layer_x = 0.0_f64;
        for &nid in &nodes {
            let n = graph.node(nid);
            layer_x = layer_x.max(n.size.x + n.margin.left + n.margin.right);
        }

        // Layer height: from first node top to last node bottom (assumes
        // post-P4 layers are sorted top-to-bottom by y position).
        let first = graph.node(nodes[0]);
        let mut top = first.position.y - first.margin.top;
        if first.node_type == NodeType::ExternalPort {
            top -= port_surrounding.top;
        }

        let last = graph.node(*nodes.last().unwrap());
        let mut bottom = last.position.y + last.size.y + last.margin.bottom;
        if last.node_type == NodeType::ExternalPort {
            bottom += port_surrounding.bottom;
        }

        // Defensively expand to enclose any node that sits outside the
        // first/last extremes (can occur with hierarchical port dummies).
        for &nid in &nodes {
            let n = graph.node(nid);
            top = top.min(n.position.y - n.margin.top);
            bottom = bottom.max(n.position.y + n.size.y + n.margin.bottom);
        }

        graph.layers[layer_idx].size = Vec2::new(layer_x, bottom - top);
        min_y = min_y.min(top);
        max_y = max_y.max(bottom);
    }

    if !found_nodes {
        min_y = 0.0;
        max_y = 0.0;
    }

    graph.size.y = max_y - min_y;
    graph.offset.y -= min_y;
}
