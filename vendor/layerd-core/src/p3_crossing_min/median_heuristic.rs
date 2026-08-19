//! Median heuristic for crossing minimization.
//!
//! Given a free layer and a reference layer (the one the free layer should
//! be ordered against), every node takes the WEIGHT of the median of its
//! adjacent nodes in the reference layer. Nodes with no adjacency fall back
//! to the layer average. The initial layer gets random weights that are then
//! rewritten as a 1..n integer sequence.
//!
//! `MedianHeuristic` is deterministic but not guaranteed to always reduce
//! crossings, which drives the `minimize_with_counter_single_pass` dispatch
//! path.

use crate::{
    graph::{LGraph, index::NodeId},
    properties::internal::WEIGHT,
    rng::Rng,
};

/// Assign random weights to the start layer, sort by weight, then rewrite
/// weights as the 1..n order index. Always returns `false`.
pub(crate) fn set_first_layer_order(graph: &mut LGraph, rng: &mut impl Rng, forward: bool) -> bool {
    let start_idx = if forward { 0 } else { graph.layers.len().saturating_sub(1) };
    if start_idx >= graph.layers.len() {
        return false;
    }
    let nodes = graph.layers[start_idx].nodes.clone();
    let mut weighted: Vec<(NodeId, f64)> =
        nodes.iter().copied().map(|nid| (nid, rng.next_f64())).collect();
    weighted.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    for (idx, (nid, _)) in weighted.iter().enumerate() {
        graph.node_mut(*nid).properties.set(&WEIGHT, (idx + 1) as f64);
    }
    let ordered: Vec<NodeId> = weighted.into_iter().map(|(nid, _)| nid).collect();
    if graph.layers[start_idx].nodes != ordered {
        graph.layers[start_idx].nodes = ordered;
        graph.bump_layer_order_version(start_idx);
    }
    false
}

/// Compute medians against the adjacent reference layer, then stable-sort
/// the free layer by the resulting WEIGHTs. Always returns `false` (the
/// outer crossing counter decides whether to keep the result).
pub(crate) fn minimize_layer(graph: &mut LGraph, free_layer_idx: usize, forward: bool) -> bool {
    if free_layer_idx >= graph.layers.len() {
        return false;
    }
    let reference_idx = if forward {
        if free_layer_idx == 0 {
            return false;
        }
        free_layer_idx - 1
    } else {
        free_layer_idx + 1
    };
    if reference_idx >= graph.layers.len() {
        return false;
    }

    let free_nodes: Vec<NodeId> = graph.layers[free_layer_idx].nodes.clone();
    calculate_medians(graph, &free_nodes, reference_idx);

    let mut sorted = free_nodes;
    sorted.sort_by(|a, b| compare_by_weight(graph, *a, *b));
    if graph.layers[free_layer_idx].nodes != sorted {
        graph.layers[free_layer_idx].nodes = sorted;
        graph.bump_layer_order_version(free_layer_idx);
    }
    false
}

/// For each node in `free_nodes` collect the neighbours in `reference_idx`,
/// sort them by WEIGHT, and pick the median's WEIGHT. Nodes with no
/// reference-layer neighbours get the layer-wide midpoint afterwards.
fn calculate_medians(graph: &mut LGraph, free_nodes: &[NodeId], reference_idx: usize) {
    let mut min_weight = f64::MIN;
    let mut max_weight = f64::MAX;
    let mut to_revisit: Vec<NodeId> = Vec::new();

    let reference_layer: std::collections::HashSet<NodeId> =
        graph.layers[reference_idx].nodes.iter().copied().collect();

    for &nid in free_nodes {
        let mut connected: Vec<NodeId> = Vec::new();
        for eid in graph.outgoing_edges(nid) {
            let tgt = graph.port(graph.edge(eid).target).owner;
            if reference_layer.contains(&tgt) {
                connected.push(tgt);
            }
            let src = graph.port(graph.edge(eid).source).owner;
            if reference_layer.contains(&src) {
                connected.push(src);
            }
        }
        for eid in graph.incoming_edges(nid) {
            let tgt = graph.port(graph.edge(eid).target).owner;
            if reference_layer.contains(&tgt) {
                connected.push(tgt);
            }
            let src = graph.port(graph.edge(eid).source).owner;
            if reference_layer.contains(&src) {
                connected.push(src);
            }
        }

        if connected.is_empty() {
            to_revisit.push(nid);
            continue;
        }
        connected.sort_by(|a, b| compare_by_weight(graph, *a, *b));
        let median_idx = connected.len() / 2;
        let new_weight = graph.node(connected[median_idx]).properties.get(&WEIGHT);
        graph.node_mut(nid).properties.set(&WEIGHT, new_weight);
        if new_weight < min_weight || min_weight == f64::MIN {
            min_weight = new_weight;
        }
        if new_weight > max_weight || max_weight == f64::MAX {
            max_weight = new_weight;
        }
    }

    let avg_weight = if min_weight == f64::MIN || max_weight == f64::MAX {
        0.0
    } else {
        (max_weight + min_weight) / 2.0
    };
    for nid in to_revisit {
        graph.node_mut(nid).properties.set(&WEIGHT, avg_weight);
    }
}

/// Compare two nodes by WEIGHT: nodes without WEIGHT set (NaN default)
/// sort before nodes with WEIGHT.
fn compare_by_weight(graph: &LGraph, a: NodeId, b: NodeId) -> std::cmp::Ordering {
    let wa = graph.node(a).properties.get(&WEIGHT);
    let wb = graph.node(b).properties.get(&WEIGHT);
    match (wa.is_nan(), wb.is_nan()) {
        (true, true) => std::cmp::Ordering::Equal,
        (true, false) => std::cmp::Ordering::Greater,
        (false, true) => std::cmp::Ordering::Less,
        (false, false) => wa.partial_cmp(&wb).unwrap_or(std::cmp::Ordering::Equal),
    }
}
