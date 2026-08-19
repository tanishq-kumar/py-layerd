//! Coffman-Graham layering.
//!
//! Three phases:
//! 1. Remove transitive edges (DFS from each node).
//! 2. Compute a Coffman-Graham topological order using a priority queue
//!    whose priority compares predecessor topo numbers.
//! 3. Assign layers by processing nodes sink-first, respecting the
//!    `LAYERING_COFFMAN_GRAHAM_LAYER_BOUND` width constraint and avoiding
//!    in-layer edges.

use std::collections::BinaryHeap;

use hashbrown::HashMap;

use crate::graph::{
    LGraph, LayerData,
    index::{EdgeId, NodeId},
};

/// Assign layers using Coffman-Graham.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    let n = nodes.len();
    let mut node_idx: HashMap<NodeId, usize> = HashMap::with_capacity(n);
    for (i, &nid) in nodes.iter().enumerate() {
        node_idx.insert(nid, i);
    }

    // Collect all edges that connect two layerless nodes. Build adjacency
    // and an `edge_id_to_idx` map so `edge_mark` can be indexed.
    let mut edge_idx: HashMap<EdgeId, usize> = HashMap::new();
    let mut outgoing: Vec<Vec<(usize, EdgeId)>> = vec![Vec::new(); n];
    let mut incoming: Vec<Vec<(usize, EdgeId)>> = vec![Vec::new(); n];
    for (i, &nid) in nodes.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let tgt_node = graph.port(graph.edge(eid).target).owner;
            let Some(&j) = node_idx.get(&tgt_node) else {
                continue;
            };
            if i == j {
                continue;
            }
            let e_idx = edge_idx.len();
            edge_idx.entry(eid).or_insert(e_idx);
            outgoing[i].push((j, eid));
            incoming[j].push((i, eid));
        }
    }
    let num_edges = edge_idx.len();
    let mut edge_mark = vec![false; num_edges];

    // #1 Transitive reduction.
    {
        let mut node_mark = vec![false; n];
        for start in 0..n {
            node_mark.fill(false);
            let outs_of_start: Vec<(usize, EdgeId)> = outgoing[start].clone();
            for &(t, _eid) in &outs_of_start {
                transitive_dfs_iterative(
                    start,
                    t,
                    &outgoing,
                    &incoming,
                    &edge_idx,
                    &mut node_mark,
                    &mut edge_mark,
                );
            }
        }
    }

    // #2 Topological order (Coffman-Graham labeling).
    let mut in_deg = vec![0i32; n];
    for i in 0..n {
        for &(_src, eid) in &incoming[i] {
            if !edge_mark[edge_idx[&eid]] {
                in_deg[i] += 1;
            }
        }
    }

    let mut in_topo: Vec<Vec<i32>> = vec![Vec::new(); n];
    let mut topo_ord = vec![-1i32; n];

    let mut pq: BinaryHeap<TopoItem> = BinaryHeap::new();
    for (i, &degree) in in_deg.iter().enumerate().take(n) {
        if degree == 0 {
            pq.push(TopoItem { idx: i });
        }
    }
    let mut next_rank = 0i32;
    while let Some(item) = pop_topo(&mut pq, &in_topo) {
        let v = item;
        topo_ord[v] = next_rank;
        let my_rank = next_rank;
        next_rank += 1;
        let outs: Vec<(usize, EdgeId)> = outgoing[v].clone();
        for (t, eid) in outs {
            if edge_mark[edge_idx[&eid]] {
                continue;
            }
            in_deg[t] -= 1;
            in_topo[t].push(my_rank);
            if in_deg[t] == 0 {
                pq.push(TopoItem { idx: t });
            }
        }
    }

    // #3 Backward layer assignment.
    let mut out_deg = vec![0i32; n];
    for i in 0..n {
        for &(_tgt, eid) in &outgoing[i] {
            if !edge_mark[edge_idx[&eid]] {
                out_deg[i] += 1;
            }
        }
    }

    let w = graph.options.layering_coffman_graham_layer_bound.max(1);

    let mut sinks: BinaryHeap<SinkItem> = BinaryHeap::new();
    for i in 0..n {
        if out_deg[i] == 0 {
            sinks.push(SinkItem { idx: i, topo: topo_ord[i] });
        }
    }

    // Layers are built sink → source, then reversed.
    let mut reverse_layers: Vec<Vec<usize>> = Vec::new();
    reverse_layers.push(Vec::new());

    while let Some(SinkItem { idx: u, .. }) = sinks.pop() {
        let cur = reverse_layers.len() - 1;
        let full = reverse_layers[cur].len() as i32 >= w;
        let mut needs_new = full;
        if !needs_new {
            // The `canAdd` test iterates ALL outgoing edges, including
            // those marked by transitive reduction. A transitive A→C edge
            // with C already in the current layer must still force a new
            // layer for A.
            for &(t, _eid) in &outgoing[u] {
                if reverse_layers[cur].contains(&t) {
                    needs_new = true;
                    break;
                }
            }
        }
        if needs_new {
            reverse_layers.push(Vec::new());
        }
        let cur = reverse_layers.len() - 1;
        reverse_layers[cur].push(u);

        let ins: Vec<(usize, EdgeId)> = incoming[u].clone();
        for (s, eid) in ins {
            if edge_mark[edge_idx[&eid]] {
                continue;
            }
            out_deg[s] -= 1;
            if out_deg[s] == 0 {
                sinks.push(SinkItem { idx: s, topo: topo_ord[s] });
            }
        }
    }

    // Flip into final order.
    graph.layers.clear();
    for rev in reverse_layers.iter().rev() {
        let mut layer = LayerData::new();
        for &idx in rev {
            let nid = nodes[idx];
            layer.nodes.push(nid);
            graph.node_mut(nid).layer = Some(graph.layers.len()).into();
        }
        graph.layers.push(layer);
    }
    graph.layerless_nodes.clear();
}

fn transitive_dfs_iterative(
    start: usize,
    v: usize,
    outgoing: &[Vec<(usize, EdgeId)>],
    incoming: &[Vec<(usize, EdgeId)>],
    edge_idx: &HashMap<EdgeId, usize>,
    node_mark: &mut [bool],
    edge_mark: &mut [bool],
) {
    if node_mark[v] {
        return;
    }
    let mut stack = vec![(v, 0usize)];
    while let Some((node, next_edge)) = stack.last_mut() {
        if node_mark[*node] {
            stack.pop();
            continue;
        }
        if *next_edge >= outgoing[*node].len() {
            node_mark[*node] = true;
            stack.pop();
            continue;
        }
        let (w, _) = outgoing[*node][*next_edge];
        *next_edge += 1;
        for &(src, eid) in &incoming[w] {
            if src == start {
                edge_mark[edge_idx[&eid]] = true;
            }
        }
        if !node_mark[w] {
            stack.push((w, 0));
        }
    }
}

/// Priority-queue item for the Coffman-Graham topological ordering. The
/// underlying BinaryHeap is max-ordered, so our `Ord` impl yields
/// "greater" for the element that should come OUT FIRST — i.e. the
/// smallest under `compare_nodes_in_topo`.
#[derive(Clone, Copy, PartialEq, Eq)]
struct TopoItem {
    idx: usize,
}

// Topo order compares predecessor lists lexicographically from the
// largest element down. BinaryHeap is max-heap, so the correct element
// to pop is whichever scores smallest under the comparator.
//
// We implement the comparison by consulting `in_topo` during pop instead
// of embedding the predecessor list in the item — the list grows as the
// topo ordering progresses, so a stable per-item comparator would risk
// staleness. Hence we pop candidates one by one and re-rank before
// committing.
impl Ord for TopoItem {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Default fallback order — only hit when `pop_topo` is not used.
        other.idx.cmp(&self.idx)
    }
}

impl PartialOrd for TopoItem {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

fn pop_topo(pq: &mut BinaryHeap<TopoItem>, in_topo: &[Vec<i32>]) -> Option<usize> {
    if pq.is_empty() {
        return None;
    }
    // Drain PQ, then scan for the minimum per compareNodesInTopo.
    let mut candidates: Vec<usize> = pq.drain().map(|t| t.idx).collect();
    let mut best = 0usize;
    for i in 1..candidates.len() {
        if compare_nodes_in_topo(candidates[i], candidates[best], in_topo)
            == std::cmp::Ordering::Less
        {
            best = i;
        }
    }
    let winner = candidates.swap_remove(best);
    for idx in candidates {
        pq.push(TopoItem { idx });
    }
    Some(winner)
}

fn compare_nodes_in_topo(u: usize, v: usize, in_topo: &[Vec<i32>]) -> std::cmp::Ordering {
    let in_u = &in_topo[u];
    let in_v = &in_topo[v];
    let mut iu = in_u.len();
    let mut iv = in_v.len();
    // The ordering intentionally preserves the cached-integer comparison
    // behavior used by the priority queue. Equal values in [-128, 127]
    // continue comparing, while equal values outside that range return 0
    // from the inner integer comparison immediately.
    while iu > 0 && iv > 0 {
        iu -= 1;
        iv -= 1;
        let a = in_u[iu];
        let b = in_v[iv];
        if a != b {
            return a.cmp(&b);
        }
        if !(-128..=127).contains(&a) {
            return std::cmp::Ordering::Equal;
        }
    }
    // Post-loop branch: when one list is empty and the other still has its
    // iterator at the end position, fall through to the id-based tie-breaker
    // instead of treating the non-empty list as longer.
    if (in_u.is_empty() && in_v.is_empty()) || (in_u.is_empty() != in_v.is_empty()) {
        return u.cmp(&v);
    }
    std::cmp::Ordering::Greater
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct SinkItem {
    idx: usize,
    topo: i32,
}

impl Ord for SinkItem {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Higher topo → pop first. BinaryHeap is max-heap, so natural
        // comparison on `topo` does the right thing. Equal topo values
        // compare equal so the BinaryHeap's internal ordering takes over
        // (insertion-order fallback) instead of an idx tiebreak.
        self.topo.cmp(&other.topo)
    }
}

impl PartialOrd for SinkItem {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
