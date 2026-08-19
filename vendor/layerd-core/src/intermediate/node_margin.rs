//! Innermost node margin calculator (excluding edge head/tail labels).
//!
//! The resulting margin covers ports, port labels, and the node's own labels
//! via a holistic bounding-box union. Edge head/tail labels are handled
//! separately by `intermediate/end_label.rs`; comment boxes by
//! `comment::calculate_node_margin`; self-loop curves by `SelfLoopRouter`.

use crate::{
    graph::{LGraph, index::NodeId},
    math::Margin,
};

/// Expands each node's margin to cover ports, port labels, and node-level
/// labels via a bounding-box union.
///
/// Runs before P4.
pub fn calculate_innermost(graph: &mut LGraph) {
    let node_ids: Vec<NodeId> = graph.nodes_iter().map(|(id, _)| id).collect();
    for node_id in node_ids {
        let (x_min, y_min, x_max, y_max) = node_bounding_box(graph, node_id);
        let node_size = graph.node(node_id).size;
        let margin = *graph.node(node_id).margin;
        let next_margin = Margin {
            left: margin.left.max((-x_min).max(0.0)),
            top: margin.top.max((-y_min).max(0.0)),
            right: margin.right.max((x_max - node_size.x).max(0.0)),
            bottom: margin.bottom.max((y_max - node_size.y).max(0.0)),
        };
        if next_margin != margin {
            graph.node_mut(node_id).margin = next_margin.into();
        }
    }
}

/// Bounding box in the node's local coordinate system, expressed as
/// `(x_min, y_min, x_max, y_max)`. The box always contains the node rectangle
/// itself; ports, port labels, and node labels grow it via union.
fn node_bounding_box(graph: &LGraph, node_id: NodeId) -> (f64, f64, f64, f64) {
    let node = graph.node(node_id);
    let mut x_min = 0.0_f64;
    let mut y_min = 0.0_f64;
    let mut x_max = node.size.x;
    let mut y_max = node.size.y;

    // Node-level labels.
    for &lid in &node.labels {
        let lab = graph.label(lid);
        x_min = x_min.min(lab.position.x);
        y_min = y_min.min(lab.position.y);
        x_max = x_max.max(lab.position.x + lab.size.x);
        y_max = y_max.max(lab.position.y + lab.size.y);
    }

    // Ports + port labels.
    for &pid in &node.ports {
        let port = graph.port(pid);
        let px = port.position.x;
        let py = port.position.y;
        x_min = x_min.min(px);
        y_min = y_min.min(py);
        x_max = x_max.max(px + port.size.x);
        y_max = y_max.max(py + port.size.y);

        for &lid in &port.labels {
            let lab = graph.label(lid);
            let lx = px + lab.position.x;
            let ly = py + lab.position.y;
            x_min = x_min.min(lx);
            y_min = y_min.min(ly);
            x_max = x_max.max(lx + lab.size.x);
            y_max = y_max.max(ly + lab.size.y);
        }
    }

    (x_min, y_min, x_max, y_max)
}
