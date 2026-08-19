//! Network-simplex layering.
//!
//! Thin adapter around the shared Gansner network simplex solver in
//! `crate::algorithms::network_simplex`. Builds an auxiliary graph
//! for each connected component, runs the solver with `withBalancing(true)`,
//! and threads per-layer node counts between components so balancing sees
//! already-layered work.

use hashbrown::HashMap;

use crate::{
    algorithms::network_simplex::{NGraph, Solver},
    graph::{LGraph, LayerData, index::NodeId},
    properties::internal::PRIORITY_SHORTNESS,
};

/// Pivot iteration limit factor. Multiplied with the graph's `THOROUGHNESS`
/// option to derive the per-component pivot cap.
const ITER_LIMIT_FACTOR: usize = 4;

/// Assign layers using the network simplex algorithm.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    let components = find_connected_components(graph, &nodes);
    let multi_component = components.len() > 1;

    // Per-layer node counts, rebuilt after each component from the current
    // `graph.layers` so the next component's normalize/balance steps see
    // existing per-layer counts.
    let mut previous_counts: Option<Vec<usize>> = None;

    for component in components {
        let ng = build_ngraph(graph, &component);

        let iter_limit = (graph.options.thoroughness as usize)
            * ITER_LIMIT_FACTOR
            * ((component.len() as f64).sqrt() as usize).max(1);

        let mut solver = Solver::new(ng)
            .with_iter_limit(iter_limit)
            .with_balancing(true)
            .with_subtree_optimization(true);
        if let Some(prev) = previous_counts.as_ref() {
            solver = solver.with_previous_counts(prev.clone());
        }
        let result = solver.solve();

        for n in &result.graph.nodes {
            let origin = component[n.stable_id as usize];
            let layer_idx = n.layer as usize;
            while graph.layers.len() <= layer_idx {
                graph.layers.push(LayerData::new());
            }
            graph.layers[layer_idx].nodes.push(origin);
            graph.node_mut(origin).layer = Some(layer_idx).into();
        }

        if multi_component {
            previous_counts = Some(graph.layers.iter().map(|l| l.nodes.len()).collect::<Vec<_>>());
        }
    }

    graph.layerless_nodes.clear();
}

// Connected components

fn find_connected_components(graph: &LGraph, nodes: &[NodeId]) -> Vec<Vec<NodeId>> {
    if nodes.is_empty() {
        return vec![];
    }

    let mut node_to_idx: HashMap<NodeId, usize> = HashMap::with_capacity(nodes.len());
    for (i, &nid) in nodes.iter().enumerate() {
        node_to_idx.insert(nid, i);
    }

    let n = nodes.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];

    // For each node, walk its ports in order, and within each port walk the
    // connected edges (incoming first, then outgoing). Each opposite is
    // appended once per occurrence to `adj[i]`. The reverse direction is
    // filled when iteration reaches the opposite node, so we do not
    // double-fill here.
    for (i, &nid) in nodes.iter().enumerate() {
        let ports: Vec<crate::graph::index::PortId> =
            graph.node(nid).ports.iter().copied().collect();
        for port_id in ports {
            let port = graph.port(port_id);
            for eid in port.incoming_edges.iter().chain(port.outgoing_edges.iter()).copied() {
                let edge = graph.edge(eid);
                let opposite_port =
                    if graph.port(edge.source).owner == nid { edge.target } else { edge.source };
                let opposite_node = graph.port(opposite_port).owner;
                if let Some(&j) = node_to_idx.get(&opposite_node)
                    && j != i
                {
                    adj[i].push(j);
                }
            }
        }
    }

    let mut visited = vec![false; n];
    // Use a deque and apply the "first-component-seen-or-larger-than-head
    // goes push_front, else push_back" rule. This keeps the largest
    // component at the head (so simplex attribute buffers can be reused
    // across runs) while preserving DFS discovery order for everything
    // else. Sorting the whole list by size descending would reorder
    // equally-sized components and yield divergent balance results.
    let mut components: std::collections::VecDeque<Vec<NodeId>> = std::collections::VecDeque::new();

    for start in 0..n {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![start];
        while let Some(cur) = stack.pop() {
            if visited[cur] {
                continue;
            }
            visited[cur] = true;
            component.push(nodes[cur]);
            // Reverse iteration so `stack.pop()` consumes neighbours in
            // the same order a recursive DFS would descend into them.
            // Without the reverse the DFS visits the last neighbour first,
            // which would put layer-1 nodes (and every later layer) in the
            // opposite order from the reference output.
            for &next in adj[cur].iter().rev() {
                if !visited[next] {
                    stack.push(next);
                }
            }
        }
        if components.front().is_none_or(|head| head.len() < component.len()) {
            components.push_front(component);
        } else {
            components.push_back(component);
        }
    }

    components.into_iter().collect()
}

// Build shared NGraph

/// Build an [`NGraph`] for a single connected component. Node at `component[i]`
/// becomes the NGraph node at index `i` with `stable_id = i as u32`. Edge
/// weights are `max(1, PRIORITY_SHORTNESS)`: default priority 0 yields
/// weight 1, positive priorities pull tighter.
fn build_ngraph(graph: &LGraph, component: &[NodeId]) -> NGraph {
    let mut node_to_ns: HashMap<NodeId, usize> = HashMap::with_capacity(component.len());
    let mut ng = NGraph::with_capacity(component.len(), component.len() * 2);

    for (i, &nid) in component.iter().enumerate() {
        let ns_idx = ng.add_node(i as u32);
        node_to_ns.insert(nid, ns_idx);
    }

    for (i, &nid) in component.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target_node = graph.port(target_port).owner;
            if let Some(&j) = node_to_ns.get(&target_node)
                && j != i
            {
                let priority = graph.edge(eid).properties.get(&PRIORITY_SHORTNESS);
                let weight = priority.max(1) as f64;
                ng.add_edge(i, j, weight, 1);
            }
        }
    }

    ng
}
