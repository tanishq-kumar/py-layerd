pub mod orthogonal;
pub mod polyline;
pub mod splines;

use crate::{
    graph::{LGraph, index::NodeId},
    options::enums::Alignment,
    properties::internal::ALIGNMENT,
};

/// Determines a horizontal placement for every node within a layer.
///
/// Each node is shifted within `layer.size.x` based on its alignment hint
/// (or, for `AUTOMATIC`, the ratio of outgoing to incoming ports), then
/// clamped to the node's individual margins. The
/// layer's `size.x` is expected to already reflect the widest margin-padded
/// node — set either by `LayerSizeAndGraphHeightCalculator` or by the caller
/// when it iterates layer-by-layer.
pub(crate) fn place_nodes_horizontally(graph: &mut LGraph, layer_idx: usize, xoffset: f64) {
    if layer_idx >= graph.layers.len() {
        return;
    }
    let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
    if nodes.is_empty() {
        return;
    }
    let layer_size_x = graph.layers[layer_idx].size.x;

    let mut max_left_margin = 0.0_f64;
    let mut max_right_margin = 0.0_f64;
    for &nid in &nodes {
        let m = *graph.node(nid).margin;
        max_left_margin = max_left_margin.max(m.left);
        max_right_margin = max_right_margin.max(m.right);
    }

    for &nid in &nodes {
        let alignment = graph.node(nid).properties.get(&ALIGNMENT);
        let ratio = match alignment {
            Alignment::Left => 0.0,
            Alignment::Right => 1.0,
            Alignment::Center => 0.5,
            // AUTOMATIC falls through to a port-ratio shift; TOP / BOTTOM
            // are vertical-only and never reach this code path because the
            // direction transformer rotates them into LEFT / RIGHT before
            // P5 runs.
            _ => {
                let mut inports = 0_i32;
                let mut outports = 0_i32;
                let port_ids: Vec<_> = graph.node(nid).ports.to_vec();
                for pid in port_ids {
                    let p = graph.port(pid);
                    if !p.incoming_edges.is_empty() {
                        inports += 1;
                    }
                    if !p.outgoing_edges.is_empty() {
                        outports += 1;
                    }
                }
                if inports + outports == 0 {
                    0.5
                } else {
                    f64::from(outports) / f64::from(inports + outports)
                }
            }
        };

        let node = graph.node(nid);
        let node_size = node.size.x;
        let mut xpos = (layer_size_x - node_size) * ratio;
        if ratio > 0.5 {
            xpos -= max_right_margin * 2.0 * (ratio - 0.5);
        } else if ratio < 0.5 {
            xpos += max_left_margin * 2.0 * (0.5 - ratio);
        }

        let left_margin = node.margin.left;
        if xpos < left_margin {
            xpos = left_margin;
        }
        let right_margin = node.margin.right;
        if xpos > layer_size_x - right_margin - node_size {
            xpos = layer_size_x - right_margin - node_size;
        }

        graph.node_mut(nid).position.x = xoffset + xpos;
    }
}
