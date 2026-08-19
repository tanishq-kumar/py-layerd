use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    properties::internal::CYCLIC,
};

/// Break cycles using depth-first search back-edge reversal.
///
/// Performs DFS and reverses back edges to break all cycles. Starts from
/// source nodes (no incoming edges) first, then processes any remaining
/// unvisited nodes.
pub fn break_cycles(graph: &mut LGraph) {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    let n = node_ids.len();
    if n == 0 {
        return;
    }

    let max_id = node_ids.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;

    let mut id_to_idx = vec![usize::MAX; max_id + 1];
    for (i, &nid) in node_ids.iter().enumerate() {
        id_to_idx[graph.node(nid).id as usize] = i;
    }

    // Build adjacency: for each node index, list of (neighbor_index, edge_id).
    let mut adj: Vec<Vec<(usize, EdgeId)>> = vec![Vec::new(); n];
    let mut has_incoming = vec![false; n];

    for (i, &nid) in node_ids.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target_node = graph.port(target_port).owner;
            let target_id_val = graph.node(target_node).id as usize;
            if target_id_val < id_to_idx.len() {
                let j = id_to_idx[target_id_val];
                if j != usize::MAX && j != i {
                    adj[i].push((j, eid));
                    has_incoming[j] = true;
                }
            }
        }
    }

    let mut visited = vec![false; n];
    let mut active = vec![false; n]; // on current DFS stack
    let mut back_edges: Vec<EdgeId> = Vec::new();

    fn dfs_iterative(
        u: usize,
        adj: &[Vec<(usize, EdgeId)>],
        visited: &mut [bool],
        active: &mut [bool],
        back_edges: &mut Vec<EdgeId>,
    ) {
        if visited[u] {
            return;
        }
        let mut stack = vec![(u, 0usize)];
        visited[u] = true;
        active[u] = true;
        while let Some((node, next_edge)) = stack.last_mut() {
            if *next_edge >= adj[*node].len() {
                active[*node] = false;
                stack.pop();
                continue;
            }
            let (v, eid) = adj[*node][*next_edge];
            *next_edge += 1;
            if active[v] {
                back_edges.push(eid);
            } else if !visited[v] {
                visited[v] = true;
                active[v] = true;
                stack.push((v, 0));
            }
        }
    }

    // Start DFS from source nodes (no incoming edges) first.
    for i in 0..n {
        if !has_incoming[i] && !visited[i] {
            dfs_iterative(i, &adj, &mut visited, &mut active, &mut back_edges);
        }
    }

    // Then process any remaining unvisited nodes.
    for i in 0..n {
        if !visited[i] {
            dfs_iterative(i, &adj, &mut visited, &mut active, &mut back_edges);
        }
    }

    // Reverse all back edges.
    if !back_edges.is_empty() {
        graph.properties.set(&CYCLIC, true);
    }
    for eid in back_edges {
        graph.reverse_edge_adapt_ports(eid);
    }
}
