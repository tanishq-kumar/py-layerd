//! Longest-path-source layering.
//!
//! Dual of `longest_path`: walks incoming edges to assign each node the
//! length of the longest path from any source. Sources land in layer 0.

use crate::graph::{LGraph, LayerData, index::NodeId};

/// Assign layers by longest path from sources.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    let max_id = nodes.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;
    let mut id_to_idx = vec![usize::MAX; max_id + 1];
    for (i, &nid) in nodes.iter().enumerate() {
        id_to_idx[graph.node(nid).id as usize] = i;
    }

    // Reverse adjacency: for each node index, list of source node indices (incoming).
    let n = nodes.len();
    let mut rev_adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, &nid) in nodes.iter().enumerate() {
        for eid in graph.incoming_edges(nid) {
            let source_port = graph.edge(eid).source;
            let source_node = graph.port(source_port).owner;
            let source_id_val = graph.node(source_node).id as usize;
            if source_id_val < id_to_idx.len() {
                let j = id_to_idx[source_id_val];
                if j != usize::MAX && j != i {
                    rev_adj[i].push(j);
                }
            }
        }
    }

    // Heights: 1-based longest path from any source.
    let mut heights: Vec<i32> = vec![-1; n];
    for i in 0..n {
        if heights[i] < 0 {
            visit_iterative(i, &rev_adj, &mut heights);
        }
    }

    let max_height = heights.iter().copied().max().unwrap_or(1) as usize;

    graph.layers.clear();
    for _ in 0..max_height {
        graph.layers.push(LayerData::new());
    }

    for (i, &nid) in nodes.iter().enumerate() {
        let layer_index = (heights[i] - 1) as usize;
        graph.layers[layer_index].nodes.push(nid);
        graph.node_mut(nid).layer = Some(layer_index).into();
    }

    graph.layerless_nodes.clear();
}

fn visit_iterative(node: usize, rev_adj: &[Vec<usize>], heights: &mut [i32]) -> i32 {
    if heights[node] >= 0 {
        return heights[node];
    }

    let mut stack = vec![(node, false)];
    while let Some((current, finish)) = stack.pop() {
        if heights[current] >= 0 {
            continue;
        }
        if finish {
            let max_height =
                rev_adj[current].iter().map(|&source| heights[source] + 1).max().unwrap_or(1);
            heights[current] = max_height;
        } else {
            stack.push((current, true));
            for &source in rev_adj[current].iter().rev() {
                if heights[source] < 0 {
                    stack.push((source, false));
                }
            }
        }
    }

    heights[node]
}
