use crate::graph::{LGraph, LayerData, index::NodeId};

/// Assign layers using the longest-path algorithm.
///
/// Assigns layers based on the longest path from each node to any sink.
/// Nodes with no outgoing edges (sinks) are placed at the bottom layer.
/// All other nodes are placed such that their layer index equals
/// `total_layers - height`, where height is the longest path to any sink + 1.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    // Build node id -> index mapping using the node's `id` field.
    let max_id = nodes.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;

    let mut id_to_idx = vec![usize::MAX; max_id + 1];
    for (i, &nid) in nodes.iter().enumerate() {
        let node_id_val = graph.node(nid).id as usize;
        id_to_idx[node_id_val] = i;
    }

    // Build adjacency list: for each node index, list of target node indices.
    let n = nodes.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    for (i, &nid) in nodes.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target_node = graph.port(target_port).owner;
            let target_id_val = graph.node(target_node).id as usize;
            if target_id_val < id_to_idx.len() {
                let j = id_to_idx[target_id_val];
                if j != usize::MAX && j != i {
                    adj[i].push(j);
                }
            }
        }
    }

    // Compute heights using DFS on the adjacency list.
    let mut heights: Vec<i32> = vec![-1; n];

    for i in 0..n {
        if heights[i] < 0 {
            visit_iterative(i, &adj, &mut heights);
        }
    }

    // Determine total number of layers needed.
    let max_height = heights.iter().copied().max().unwrap_or(1) as usize;

    // Create layers.
    graph.layers.clear();
    for _ in 0..max_height {
        graph.layers.push(LayerData::new());
    }

    // Assign nodes to layers.
    for (i, &nid) in nodes.iter().enumerate() {
        let h = heights[i] as usize;
        let layer_index = max_height - h;
        graph.layers[layer_index].nodes.push(nid);
        graph.node_mut(nid).layer = Some(layer_index).into();
    }

    // Clear layerless nodes.
    graph.layerless_nodes.clear();
}

/// Height is the longest path from this node to any sink + 1.
/// A sink node (no outgoing edges) has height 1.
fn visit_iterative(node: usize, adj: &[Vec<usize>], heights: &mut [i32]) -> i32 {
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
                adj[current].iter().map(|&target| heights[target] + 1).max().unwrap_or(1);
            heights[current] = max_height;
        } else {
            stack.push((current, true));
            for &target in adj[current].iter().rev() {
                if heights[target] < 0 {
                    stack.push((target, false));
                }
            }
        }
    }

    heights[node]
}
