//! Minimum-width layering.
//!
//! Implements the Nikolov / Tarassov / Branke "MinWidth" heuristic for the
//! NP-hard minimum-width layering problem with consideration of dummy nodes.
//!
//! The bottom-up loop selects nodes whose outgoing edges all point to
//! already-placed lower layers. A new layer opens when adding the current
//! node would push the running width above `upper_bound_on_width *
//! avg_normalised_size`, or when the estimated next-layer width crosses
//! `compensator * upper_bound_on_width * avg_normalised_size`. When the
//! configuration values are `-1` the layerer enumerates the recommended
//! ranges (`1..=4` x `1..=2`) and keeps the narrowest layering.
use hashbrown::HashSet;

use crate::graph::{LGraph, LayerData, index::NodeId, node::NodeType};

const UB_RANGE_LOWER: i32 = 1;
const UB_RANGE_UPPER: i32 = 4;
const COMP_RANGE_LOWER: i32 = 1;
const COMP_RANGE_UPPER: i32 = 2;

/// Assign layers using the MinWidth heuristic.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    let prep = MinWidthPrep::build(graph, &nodes);

    let ub_opt = graph.options.layering_min_width_upper_bound_on_width;
    let comp_opt = graph.options.layering_min_width_upper_layer_estimation_scaling_factor;
    let (ub_start, ub_end) =
        if ub_opt < 0 { (UB_RANGE_LOWER, UB_RANGE_UPPER) } else { (ub_opt, ub_opt) };
    let (comp_start, comp_end) = if comp_opt < 0 {
        (COMP_RANGE_LOWER, COMP_RANGE_UPPER)
    } else {
        (comp_opt, comp_opt)
    };

    let mut best: Option<MinWidthResult> = None;
    for ubw in ub_start..=ub_end {
        for c in comp_start..=comp_end {
            let candidate = compute_min_width_layering(&prep, ubw, c);
            best = Some(match best.take() {
                None => candidate,
                Some(b) => pick_narrower(b, candidate),
            });
        }
    }
    let chosen = best.expect("at least one layering attempted");
    apply_layering(graph, &nodes, &prep, chosen);
}

struct MinWidthPrep {
    /// Map dense index → original `NodeId`.
    idx_to_node: Vec<NodeId>,
    /// In-/out-degree per dense index, ignoring self-loops.
    in_degree: Vec<i32>,
    out_degree: Vec<i32>,
    /// Successor set per dense index (as a `Vec` for stable iteration).
    successors: Vec<Vec<usize>>,
    /// Normalised size per dense index (`node.size.y / minimum_real_size`).
    norm_size: Vec<f64>,
    /// Average normalised size across all (real + dummy) layerless nodes.
    avg_size: f64,
    /// Normalised dummy node size (`SPACING_EDGE_EDGE / minimum_real_size`).
    dummy_size: f64,
    /// Layerless node indices sorted by descending out-degree (ConditionSelect).
    sorted_layerless: Vec<usize>,
}

impl MinWidthPrep {
    fn build(graph: &LGraph, nodes: &[NodeId]) -> Self {
        let n = nodes.len();
        let max_id = nodes.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;
        let mut id_to_idx = vec![usize::MAX; max_id + 1];
        let mut idx_to_node = Vec::with_capacity(n);
        for (i, &nid) in nodes.iter().enumerate() {
            id_to_idx[graph.node(nid).id as usize] = i;
            idx_to_node.push(nid);
        }

        let mut in_degree = vec![0i32; n];
        let mut out_degree = vec![0i32; n];
        let mut successors: Vec<Vec<usize>> = (0..n).map(|_| Vec::new()).collect();

        for (i, &nid) in nodes.iter().enumerate() {
            let mut seen = HashSet::new();
            for eid in graph.outgoing_edges(nid) {
                let tgt_node = graph.port(graph.edge(eid).target).owner;
                if tgt_node == nid {
                    continue; // skip self-loop
                }
                out_degree[i] += 1;
                let tgt_id = graph.node(tgt_node).id as usize;
                if tgt_id < id_to_idx.len() {
                    let j = id_to_idx[tgt_id];
                    if j != usize::MAX && seen.insert(j) {
                        successors[i].push(j);
                    }
                }
            }
            for eid in graph.incoming_edges(nid) {
                let src_node = graph.port(graph.edge(eid).source).owner;
                if src_node == nid {
                    continue;
                }
                in_degree[i] += 1;
            }
        }

        // Compute minimum real-node size for normalisation.
        let mut min_real = f64::INFINITY;
        for &nid in nodes {
            if graph.node(nid).node_type != NodeType::Normal {
                continue;
            }
            let h = graph.node(nid).size.y;
            if h < min_real {
                min_real = h;
            }
        }
        if !min_real.is_finite() {
            min_real = 1.0;
        }
        let min_real = min_real.max(1.0);

        let dummy_size = graph.options.spacing.edge_edge / min_real;
        let mut norm_size = vec![0.0f64; n];
        let mut total: f64 = 0.0;
        for (i, &nid) in nodes.iter().enumerate() {
            let s = graph.node(nid).size.y / min_real;
            norm_size[i] = s;
            total += s;
        }
        let avg_size = if n > 0 { total / n as f64 } else { 1.0 };

        let mut sorted_layerless: Vec<usize> = (0..n).collect();
        // ConditionSelect: descending out-degree, stable sort. Ties keep
        // their original index order — required for the tiebreak when
        // multiple nodes share the same out-degree.
        sorted_layerless.sort_by_key(|&i| std::cmp::Reverse(out_degree[i]));

        MinWidthPrep {
            idx_to_node,
            in_degree,
            out_degree,
            successors,
            norm_size,
            avg_size,
            dummy_size,
            sorted_layerless,
        }
    }
}

struct MinWidthResult {
    /// Layers stored bottom-up in computation order (the apply step reverses).
    layers_reversed: Vec<Vec<usize>>,
    /// Maximum normalised layer width across all produced layers.
    max_width: f64,
}

fn compute_min_width_layering(
    prep: &MinWidthPrep,
    upper_bound_on_width: i32,
    compensator: i32,
) -> MinWidthResult {
    let ubw_consider_size = upper_bound_on_width as f64 * prep.avg_size;
    let mut layers: Vec<Vec<usize>> = Vec::new();

    let mut unplaced: HashSet<usize> = prep.sorted_layerless.iter().copied().collect();
    let mut already_placed_other: HashSet<usize> = HashSet::new();
    let mut already_placed_current: HashSet<usize> = HashSet::new();
    let mut current_layer: Vec<usize> = Vec::new();

    let mut width_current: f64 = 0.0;
    let mut width_up: f64 = 0.0;
    let mut max_width: f64 = 0.0;
    let mut real_width: f64 = 0.0;
    let mut current_spanning_edges: f64 = 0.0;
    let mut going_out_from_this_layer: f64 = 0.0;

    while !unplaced.is_empty() {
        // Pick the highest-priority (max out-degree first) node whose successors
        // are all already placed in earlier layers.
        let current_node = select_node(prep, &unplaced, &already_placed_other);

        if let Some(idx) = current_node {
            unplaced.remove(&idx);
            current_layer.push(idx);
            already_placed_current.insert(idx);

            let out_deg = prep.out_degree[idx];
            width_current += prep.norm_size[idx] - out_deg as f64 * prep.dummy_size;
            let in_deg = prep.in_degree[idx];
            width_up += in_deg as f64 * prep.dummy_size;
            going_out_from_this_layer += out_deg as f64 * prep.dummy_size;
            real_width += prep.norm_size[idx];
        }

        // conditionGoUp from the paper: 4 disjoint triggers.
        let go_up = current_node.is_none()
            || unplaced.is_empty()
            || (current_node.is_some() && {
                let idx = current_node.unwrap();
                width_current >= ubw_consider_size
                    && prep.norm_size[idx] > prep.out_degree[idx] as f64 * prep.dummy_size
            })
            || width_up >= compensator as f64 * ubw_consider_size;

        if go_up {
            layers.push(std::mem::take(&mut current_layer));
            already_placed_other.extend(already_placed_current.drain());

            current_spanning_edges -= going_out_from_this_layer;
            let layer_width = current_spanning_edges * prep.dummy_size + real_width;
            if layer_width > max_width {
                max_width = layer_width;
            }
            current_spanning_edges += width_up;

            width_current = width_up;
            width_up = 0.0;
            going_out_from_this_layer = 0.0;
            real_width = 0.0;
        }
    }

    MinWidthResult { layers_reversed: layers, max_width }
}

/// Pick the first unplaced node (in the precomputed descending-out-degree
/// order) whose successor set is fully contained in `targets`.
///
/// Returns `None` if no candidate exists.
fn select_node(
    prep: &MinWidthPrep,
    unplaced: &HashSet<usize>,
    targets: &HashSet<usize>,
) -> Option<usize> {
    for &idx in &prep.sorted_layerless {
        if !unplaced.contains(&idx) {
            continue;
        }
        if prep.successors[idx].iter().all(|s| targets.contains(s)) {
            return Some(idx);
        }
    }
    None
}

fn pick_narrower(a: MinWidthResult, b: MinWidthResult) -> MinWidthResult {
    let a_layers = a.layers_reversed.len();
    let b_layers = b.layers_reversed.len();
    if b.max_width < a.max_width || (b.max_width == a.max_width && b_layers < a_layers) {
        b
    } else {
        a
    }
}

fn apply_layering(
    graph: &mut LGraph,
    nodes: &[NodeId],
    prep: &MinWidthPrep,
    mut result: MinWidthResult,
) {
    // Layers were built bottom-up; flip to top-down before applying.
    result.layers_reversed.reverse();
    graph.layers.clear();
    for layer in result.layers_reversed.into_iter() {
        let mut ld = LayerData::new();
        for idx in layer {
            ld.nodes.push(prep.idx_to_node[idx]);
        }
        graph.layers.push(ld);
    }
    let assignments: Vec<(usize, NodeId)> = graph
        .layers
        .iter()
        .enumerate()
        .flat_map(|(li, l)| l.nodes.iter().map(move |&nid| (li, nid)))
        .collect();
    for (li, nid) in assignments {
        graph.node_mut(nid).layer = Some(li).into();
    }
    let _ = nodes;
    graph.layerless_nodes.clear();
}
