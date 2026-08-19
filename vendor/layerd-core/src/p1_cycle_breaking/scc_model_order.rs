//! Shared strongly connected component cycle-breaking skeleton.
//!
//! # Algorithm
//!
//! Repeatedly run Tarjan's strongly-connected-components algorithm. Every
//! SCC with more than one node is a cycle; the caller-supplied "node finder"
//! decides which edges to reverse. Reverse those edges, then rerun Tarjan,
//! until no multi-node SCCs remain.

use hashbrown::HashMap;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    options::enums::GroupOrderStrategy,
    properties::internal::{
        CB_GROUP_ORDER_STRATEGY, CB_NUM_MODEL_ORDER_GROUPS, CYCLIC, LAYERING_LAYER_ID,
        MAX_MODEL_ORDER_NODES,
    },
};

/// Shared state that the per-strategy `NodeFinder` callback reads and writes.
///
/// The `graph` reference is borrowed immutably by the finder; edges are
/// collected into `rev_edges` and reversed in batch at the end of each
/// outer iteration.
pub(crate) struct SccContext<'a> {
    pub graph: &'a LGraph,
    pub sccs: &'a [Vec<NodeId>],
    pub rev_edges: &'a mut Vec<EdgeId>,
    pub offset: i32,
    pub big_offset: i32,
    pub enforce_group_model_order: bool,
}

/// Strategy-specific callback that decides which edges to reverse for each
/// multi-node SCC.
pub(crate) type NodeFinder = fn(ctx: &mut SccContext<'_>);

/// Run the SCC-based cycle-breaking outer loop with the given node finder.
pub(crate) fn run(graph: &mut LGraph, find_nodes: NodeFinder) {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    if node_ids.is_empty() {
        return;
    }

    let offset = (node_ids.len() as i32).max(graph.properties.get(&MAX_MODEL_ORDER_NODES));
    let num_groups = graph.properties.get(&CB_NUM_MODEL_ORDER_GROUPS).max(1);
    // Use 32-bit wrapping multiplication — silent overflow is the
    // intended behavior here; `saturating_mul` would diverge.
    let big_offset = offset.wrapping_mul(num_groups);
    let enforce_group_model_order =
        graph.properties.get(&CB_GROUP_ORDER_STRATEGY) == GroupOrderStrategy::Enforced;

    let mut rev_edges: Vec<EdgeId> = Vec::new();

    loop {
        let sccs = tarjan(graph, &node_ids);
        if sccs.is_empty() {
            break;
        }

        rev_edges.clear();
        {
            let mut ctx = SccContext {
                graph,
                sccs: &sccs,
                rev_edges: &mut rev_edges,
                offset,
                big_offset,
                enforce_group_model_order,
            };
            find_nodes(&mut ctx);
        }

        for &eid in &rev_edges {
            // Reverse first (with `adaptPorts=false`), then bump
            // LAYERING_LAYER_ID on the new source — which is the OLD target
            // of the original edge.
            graph.reverse_edge(eid);
            let new_source_port = graph.edge(eid).source;
            let new_source_node = graph.port(new_source_port).owner;
            let current_layer = graph.node(new_source_node).properties.get(&LAYERING_LAYER_ID);
            graph
                .node_mut(new_source_node)
                .properties
                .set(&LAYERING_LAYER_ID, current_layer + 1);
            graph.properties.set(&CYCLIC, true);
        }
    }
}

/// Compute strongly-connected components using Tarjan's algorithm.
///
/// Directed variant that ignores `edgesToBeReversed` early-outs (those are
/// always empty in practice by the time Tarjan runs). Only SCCs with more
/// than one node are returned.
fn tarjan(graph: &LGraph, node_ids: &[NodeId]) -> Vec<Vec<NodeId>> {
    let n = node_ids.len();

    let mut id_to_idx: HashMap<NodeId, usize> = HashMap::with_capacity(n);
    for (i, &nid) in node_ids.iter().enumerate() {
        id_to_idx.insert(nid, i);
    }

    let mut out: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, &nid) in node_ids.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target = graph.port(target_port).owner;
            if let Some(&j) = id_to_idx.get(&target)
                && j != i
            {
                out[i].push(j);
            }
        }
    }

    let mut tarjan_id = vec![-1i32; n];
    let mut lowlink = vec![-1i32; n];
    let mut on_stack = vec![false; n];
    let mut stack: Vec<usize> = Vec::new();
    let mut next_index: i32 = 0;

    let mut sccs: Vec<Vec<NodeId>> = Vec::new();

    // Iterative Tarjan. The recursive form would overflow the stack on
    // large graphs; this uses an explicit work stack of
    // `(node_idx, next_child_index)` frames.
    for start in 0..n {
        if tarjan_id[start] != -1 {
            continue;
        }
        let mut work: Vec<(usize, usize)> = Vec::new();
        tarjan_id[start] = next_index;
        lowlink[start] = next_index;
        next_index += 1;
        stack.push(start);
        on_stack[start] = true;
        work.push((start, 0));

        while let Some(&(v, child_idx)) = work.last() {
            if child_idx < out[v].len() {
                let w = out[v][child_idx];
                work.last_mut().unwrap().1 += 1;
                if tarjan_id[w] == -1 {
                    tarjan_id[w] = next_index;
                    lowlink[w] = next_index;
                    next_index += 1;
                    stack.push(w);
                    on_stack[w] = true;
                    work.push((w, 0));
                } else if on_stack[w] {
                    lowlink[v] = lowlink[v].min(tarjan_id[w]);
                }
            } else {
                // Finished visiting all children of v.
                if lowlink[v] == tarjan_id[v] {
                    let mut scc: Vec<NodeId> = Vec::new();
                    loop {
                        let w = stack.pop().expect("tarjan stack underflow");
                        on_stack[w] = false;
                        scc.push(node_ids[w]);
                        if w == v {
                            break;
                        }
                    }
                    if scc.len() > 1 {
                        sccs.push(scc);
                    }
                }
                work.pop();
                if let Some(&(parent, _)) = work.last() {
                    lowlink[parent] = lowlink[parent].min(lowlink[v]);
                }
            }
        }
    }

    sccs
}
