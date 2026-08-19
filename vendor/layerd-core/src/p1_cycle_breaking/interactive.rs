use hashbrown::HashMap;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    options::enums::InteractiveReferencePoint,
};

/// Interactive cycle breaker that respects user-supplied node positions.
///
/// In the first pass every edge whose target sits to the left of its source
/// (smaller `x` coordinate) is reversed. The second pass is a DFS safety net
/// that catches any remaining cycles, for example when multiple nodes share
/// the same `x` coordinate.
/// DFS state markers used by `find_cycles`.
const UNVISITED: i8 = 1;
const ON_STACK: i8 = -1;
const DONE: i8 = 0;

/// Compute the interactive reference x of a node based on the configured
/// reference point (`Center` uses x + width/2, `TopLeft` uses x).
fn interactive_reference_x(graph: &LGraph, node: NodeId, mode: InteractiveReferencePoint) -> f64 {
    let n = graph.node(node);
    match mode {
        InteractiveReferencePoint::Center => n.position.x + n.size.x / 2.0,
        InteractiveReferencePoint::TopLeft => n.position.x,
    }
}

/// Break cycles by reversing edges whose target is left of source,
/// with a DFS cleanup pass for remaining cycles.
pub fn break_cycles(graph: &mut LGraph) {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    if node_ids.is_empty() {
        return;
    }

    // The reference point is read once at the start of the phase from the
    // graph-level `INTERACTIVE_REFERENCE_POINT` option.
    let mode = graph.options.interactive_reference_point;

    // Pass 1: reverse edges whose target reference x is strictly left of
    // the source reference x.
    let mut rev_edges: Vec<EdgeId> = Vec::new();
    for &source in &node_ids {
        let source_x = interactive_reference_x(graph, source, mode);
        for eid in graph.outgoing_edges(source) {
            let target_port = graph.edge(eid).target;
            let target = graph.port(target_port).owner;
            if target != source {
                let target_x = interactive_reference_x(graph, target, mode);
                if target_x < source_x {
                    rev_edges.push(eid);
                }
            }
        }
    }
    for &eid in &rev_edges {
        graph.reverse_edge_adapt_ports(eid);
    }

    // Pass 2: DFS cleanup. A node that was not reached through pass 1
    // might still sit on a cycle.
    //
    // NOTE: this strategy never sets `CYCLIC` on the graph, even when it
    // reverses edges. Only Greedy, DepthFirst, and ModelOrder set it.
    let mut state: HashMap<NodeId, i8> = node_ids.iter().map(|&n| (n, UNVISITED)).collect();
    let mut dfs_rev: Vec<EdgeId> = Vec::new();
    for &node in &node_ids {
        if state.get(&node).copied() == Some(UNVISITED) {
            find_cycles_iterative(graph, node, &mut state, &mut dfs_rev);
        }
    }
    for eid in dfs_rev {
        graph.reverse_edge_adapt_ports(eid);
    }
}

/// DFS that marks back edges for reversal.
fn find_cycles_iterative(
    graph: &LGraph,
    node1: NodeId,
    state: &mut HashMap<NodeId, i8>,
    rev_edges: &mut Vec<EdgeId>,
) {
    let mut stack = vec![(node1, graph.outgoing_edges(node1).collect::<Vec<_>>(), 0usize)];
    state.insert(node1, ON_STACK);
    while let Some((node, edges, next_edge)) = stack.last_mut() {
        if *next_edge >= edges.len() {
            state.insert(*node, DONE);
            stack.pop();
            continue;
        }
        let eid = edges[*next_edge];
        *next_edge += 1;
        let target_port = graph.edge(eid).target;
        let node2 = graph.port(target_port).owner;
        if *node != node2 {
            match state.get(&node2).copied() {
                Some(ON_STACK) => rev_edges.push(eid),
                Some(UNVISITED) => {
                    state.insert(node2, ON_STACK);
                    stack.push((node2, graph.outgoing_edges(node2).collect(), 0));
                }
                _ => {}
            }
        }
    }
}
