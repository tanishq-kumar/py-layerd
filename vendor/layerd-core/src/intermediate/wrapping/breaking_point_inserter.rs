//! Inserts breaking-point dummy nodes into the layering after phase 2.
//! Expects the layered graph to contain no long-edge dummies yet.

use hashbrown::HashSet;
use smallvec::SmallVec;

use super::{
    breaking_point_info::{BPInfo, BPInfoId, BREAKING_POINT_INFO, BREAKING_POINT_INFO_STORE},
    cut_index_calc,
    graph_stats::GraphStats,
    single_edge_graph_wrapper::{validify_indexes_greedily, validify_indexes_looking_back},
};
use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::PortConstraints,
    properties::internal::MODEL_ORDER,
};

/// Breaking-point inserter entry point.
pub fn insert(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    // #1 raw cuts
    let (cuts, guaranteed_valid) = {
        let mut stats = GraphStats::new(graph);
        let result =
            cut_index_calc::calculate(graph.options.wrapping_cutting_strategy, graph, &mut stats);
        (result.indexes, result.guaranteed_valid)
    };

    // #2 improve
    let cuts = if graph.options.wrapping_multi_edge_improve_cuts {
        improve_cuts(graph, &cuts)
    } else {
        cuts
    };

    // #3 validify
    let cuts = if !guaranteed_valid {
        let mut stats = GraphStats::new(graph);
        match graph.options.wrapping_validify_strategy {
            Some(crate::options::enums::WrappingValidifyStrategy::LookBack) =>
                validify_indexes_looking_back(&mut stats, &cuts),
            Some(crate::options::enums::WrappingValidifyStrategy::Greedy) =>
                validify_indexes_greedily(&mut stats, &cuts),
            _ => cuts,
        }
    } else {
        cuts
    };

    if cuts.is_empty() {
        return;
    }

    // #4 apply
    apply_cuts(graph, &cuts);
}

fn apply_cuts(graph: &mut LGraph, cuts: &[i32]) {
    let mut cut_iter = cuts.iter().copied();
    let mut cut = match cut_iter.next() {
        Some(c) => c,
        None => return,
    };

    let mut idx: usize = 0;
    let mut open_edges: Vec<EdgeId> = Vec::new();
    let mut already_split: HashSet<EdgeId> = HashSet::new();

    // We mutate the layer list via `layers.insert(...)`. `idx` tracks the
    // original layer index; replicate the iterator behaviour by indexing via
    // `current_layer_pos`, which advances past inserted layers.
    let mut current_layer_pos: usize = 0;

    loop {
        let layer_count = graph.layers.len();
        if current_layer_pos >= layer_count {
            break;
        }

        // Track open edges for this layer.
        let layer_nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[current_layer_pos].nodes);
        for &n in &layer_nodes {
            let outgoing: Vec<EdgeId> = graph.outgoing_edges(n).collect();
            for e in outgoing {
                open_edges.push(e);
            }
            let incoming: Vec<EdgeId> = graph.incoming_edges(n).collect();
            for e in incoming {
                open_edges.retain(|x| *x != e);
            }
        }

        if idx as i32 + 1 == cut {
            // Insert two new layers after the current one.
            graph.layers.insert(current_layer_pos + 1, LayerData::new());
            graph.layers.insert(current_layer_pos + 2, LayerData::new());
            let bp_layer1 = current_layer_pos + 1;
            let bp_layer2 = current_layer_pos + 2;

            // Re-index subsequent nodes' `layer` so that `insert_node_in_layer`
            // and downstream lookups stay consistent.
            reindex_layers(graph);

            for original_edge in open_edges.clone() {
                if already_split.insert(original_edge) {
                    // first time we see this edge
                }

                // Start marker
                let bp_start = graph.add_node(Vec2::ZERO);
                graph.node_mut(bp_start).node_type = NodeType::BreakingPoint;
                graph.layerless_nodes.retain(|&n| n != bp_start);
                graph.node_mut(bp_start).node_port_constraints = Some(PortConstraints::FixedSide);
                let bp_start_pos = graph.layers[bp_layer1].nodes.len();
                graph.insert_node_in_layer(bp_start, bp_layer1, bp_start_pos);
                let in_port_bp1 = graph.add_port(bp_start, PortSide::West);
                let out_port_bp1 = graph.add_port(bp_start, PortSide::East);

                // End marker
                let bp_end = graph.add_node(Vec2::ZERO);
                graph.node_mut(bp_end).node_type = NodeType::BreakingPoint;
                graph.layerless_nodes.retain(|&n| n != bp_end);
                graph.node_mut(bp_end).node_port_constraints = Some(PortConstraints::FixedSide);
                let bp_end_pos = graph.layers[bp_layer2].nodes.len();
                graph.insert_node_in_layer(bp_end, bp_layer2, bp_end_pos);
                let in_port_bp2 = graph.add_port(bp_end, PortSide::West);
                let out_port_bp2 = graph.add_port(bp_end, PortSide::East);

                // nodeStartEdge : source(original) -> in_port_bp1
                let original_src = graph.edge(original_edge).source;
                let original_model_order: i32 =
                    graph.edge(original_edge).properties.get(&MODEL_ORDER);
                let node_start_edge = graph.add_edge(original_src, in_port_bp1);
                graph
                    .edge_mut(node_start_edge)
                    .properties
                    .set(&MODEL_ORDER, original_model_order);

                // startEndEdge : out_port_bp1 -> in_port_bp2
                let start_end_edge = graph.add_edge(out_port_bp1, in_port_bp2);
                graph
                    .edge_mut(start_end_edge)
                    .properties
                    .set(&MODEL_ORDER, original_model_order);

                // Reroute the original edge: new source = out_port_bp2.
                graph.reroute_edge_source(original_edge, out_port_bp2);

                // Attach BPInfo and link into chain if the previous node in
                // the chain is also a breaking-point dummy.
                let info =
                    BPInfo::new(bp_start, bp_end, node_start_edge, start_end_edge, original_edge);
                let mut store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
                let new_id = BPInfoId(store.len() as u32);
                store.push(info);

                // Link with the previous chain if applicable. We need to
                // resolve the predecessor from the nodeStartEdge's source
                // node (i.e. the original source).
                let prev_node = graph.port(original_src).owner;
                if graph.node(prev_node).node_type == NodeType::BreakingPoint
                    && let Some(prev_id) =
                        graph.node(prev_node).properties.get(&BREAKING_POINT_INFO)
                {
                    store[prev_id.index()].next = Some(new_id);
                    store[new_id.index()].prev = Some(prev_id);
                }
                graph.properties.set(&BREAKING_POINT_INFO_STORE, store);

                graph.node_mut(bp_start).properties.set(&BREAKING_POINT_INFO, Some(new_id));
                graph.node_mut(bp_end).properties.set(&BREAKING_POINT_INFO, Some(new_id));

                // Suppress unused warnings for ports whose identities are
                // already captured via the node.
                let _ = (in_port_bp1, out_port_bp1, in_port_bp2, out_port_bp2);
            }

            match cut_iter.next() {
                Some(next) => cut = next,
                None => break,
            }

            // Skip the two freshly inserted layers so the `idx` counter tracks
            // only original layers.
            current_layer_pos += 2;
        }

        idx += 1;
        current_layer_pos += 1;
    }
}

fn reindex_layers(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for n in nodes {
            graph.node_mut(n).layer = Some(layer_idx).into();
        }
    }
}

/// Iteratively pick the best cut index for every raw cut.
fn improve_cuts(graph: &LGraph, cuts: &[i32]) -> Vec<i32> {
    if cuts.is_empty() {
        return Vec::new();
    }

    let mut nodes: Vec<Cut> = cuts.iter().copied().map(Cut::new).collect();
    // Link prev/suc.
    for i in 0..nodes.len() {
        if i > 0 {
            nodes[i].prev = Some(i - 1);
        }
        if i + 1 < nodes.len() {
            nodes[i].suc = Some(i + 1);
        }
    }

    let spans = compute_edge_spans(graph);
    let distance_penalty = graph.options.wrapping_multi_edge_distance_penalty;

    let mut improved_cuts: Vec<i32> = Vec::new();

    for _ in 0..nodes.len() {
        let mut l_cut: Option<usize> = None;
        let mut r_cut = self_or_next(&nodes, Some(0));

        let mut best_cut: Option<usize> = None;
        let mut best_score = f64::INFINITY;

        let max_idx = graph.layers.len() as i32;
        for idx in 1..max_idx {
            let r_dist = match r_cut {
                Some(r) => (nodes[r].index - idx).abs(),
                None => {
                    let l = l_cut.expect("l_cut must be set when r_cut is None");
                    (idx - nodes[l].index).abs() + 1
                }
            };
            let l_dist = match l_cut {
                Some(l) => (idx - nodes[l].index).abs(),
                None => r_dist + 1,
            };
            let (hit, dist) = if l_dist < r_dist { (l_cut, l_dist) } else { (r_cut, r_dist) };

            if let Some(hit_idx) = hit {
                let score = compute_score(spans[idx as usize], dist, distance_penalty);
                if score < best_score {
                    best_score = score;
                    best_cut = Some(hit_idx);
                    nodes[hit_idx].new_index = idx;
                }
            }

            if let Some(r) = r_cut
                && idx == nodes[r].index
            {
                l_cut = r_cut;
                r_cut = next_assigned_aware(&nodes, r);
            }
        }

        if let Some(best) = best_cut {
            improved_cuts.push(nodes[best].new_index);
            nodes[best].assigned = true;
            apply_offset(&mut nodes, best);
        }
    }

    improved_cuts.sort_unstable();
    improved_cuts
}

fn compute_score(spans: i32, dist: i32, distance_penalty: f64) -> f64 {
    spans as f64 + (dist as f64).powf(distance_penalty)
}

/// Number of edges spanning each layer boundary. `spans[i]` counts edges
/// between layer `L_{i-1}` and `L_i`.
fn compute_edge_spans(graph: &LGraph) -> Vec<i32> {
    let mut spans: Vec<i32> = vec![0; graph.layers.len() + 1];
    let mut open: HashSet<EdgeId> = HashSet::new();
    for (i, layer) in graph.layers.iter().enumerate() {
        spans[i] = open.len() as i32;
        for &n in &layer.nodes {
            let outgoing: Vec<EdgeId> = graph.outgoing_edges(n).collect();
            for e in outgoing {
                open.insert(e);
            }
        }
        for &n in &layer.nodes {
            let incoming: Vec<EdgeId> = graph.incoming_edges(n).collect();
            for e in incoming {
                open.remove(&e);
            }
        }
    }
    spans
}

#[derive(Debug, Clone, Copy)]
struct Cut {
    index: i32,
    new_index: i32,
    prev: Option<usize>,
    suc: Option<usize>,
    assigned: bool,
}

impl Cut {
    fn new(index: i32) -> Self {
        Cut { index, new_index: index, prev: None, suc: None, assigned: false }
    }
}

fn self_or_next(nodes: &[Cut], start: Option<usize>) -> Option<usize> {
    let mut cur = start?;
    loop {
        if !nodes[cur].assigned {
            return Some(cur);
        }
        {
            let s = nodes[cur].suc?;
            cur = s
        }
    }
}

fn next_assigned_aware(nodes: &[Cut], from: usize) -> Option<usize> {
    let suc = nodes[from].suc?;
    self_or_next(nodes, Some(suc))
}

fn apply_offset(nodes: &mut [Cut], best: usize) {
    let offset = nodes[best].new_index - nodes[best].index;
    nodes[best].index += offset;
    // Propagate toward the head if the siblings are still unassigned.
    let mut prev = nodes[best].prev;
    while let Some(p) = prev {
        if nodes[p].assigned {
            break;
        }
        nodes[p].index += offset;
        prev = nodes[p].prev;
    }
    // Same toward the tail.
    let mut suc = nodes[best].suc;
    while let Some(s) = suc {
        if nodes[s].assigned {
            break;
        }
        nodes[s].index += offset;
        suc = nodes[s].suc;
    }
}
