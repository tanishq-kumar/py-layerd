//! Aspect-ratio-driven wrapper for path-like graphs. Runs before phase 4 when
//! `WrappingStrategy == SINGLE_EDGE` and only rearranges when the current
//! layering is wider than the desired aspect ratio. Also exposes the
//! validify helpers used by `BreakingPointInserter`.

use smallvec::SmallVec;

use super::{cut_index_calc, cutting_utils, graph_stats::GraphStats};
use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId},
    },
    options::enums::WrappingValidifyStrategy,
    properties::internal::CYCLIC,
};

/// Top-level wrapping entry point. Returns early when wrapping is not
/// needed to match the current aspect ratio.
pub fn wrap(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    let cuts = {
        let mut stats = GraphStats::new(graph);
        let sum_width = stats.max_width() * stats.longest_path as f64;
        let mw = stats.max_width();
        if mw <= 0.0 {
            return;
        }
        let current_ar = sum_width / mw;
        if stats.dar > current_ar {
            return;
        }

        let (cuts, guaranteed) = compute_cuts(graph, &mut stats);
        if !guaranteed {
            apply_validify(graph.options.wrapping_validify_strategy, &mut stats, &cuts)
        } else {
            cuts
        }
    };

    perform_cuts(graph, &cuts);
}

fn compute_cuts(graph: &LGraph, stats: &mut GraphStats<'_>) -> (Vec<i32>, bool) {
    let result = cut_index_calc::calculate(graph.options.wrapping_cutting_strategy, graph, stats);
    (result.indexes, result.guaranteed_valid)
}

fn apply_validify(
    strategy: Option<WrappingValidifyStrategy>,
    stats: &mut GraphStats<'_>,
    cuts: &[i32],
) -> Vec<i32> {
    match strategy {
        Some(WrappingValidifyStrategy::LookBack) => validify_indexes_looking_back(stats, cuts),
        Some(WrappingValidifyStrategy::Greedy) => validify_indexes_greedily(stats, cuts),
        _ => cuts.to_vec(),
    }
}

/// Bump every forbidden cut (and its successors) to the next allowed layer
/// index.
pub fn validify_indexes_greedily(stats: &mut GraphStats<'_>, cuts: &[i32]) -> Vec<i32> {
    let mut valid_cuts: Vec<i32> = Vec::new();
    let mut offset: i32 = 0;
    let longest_path = stats.longest_path as i32;

    for raw in cuts {
        let mut cut = *raw + offset;
        while cut < longest_path && !stats.is_cut_allowed(cut as usize) {
            cut += 1;
            offset += 1;
        }
        if cut >= longest_path {
            break;
        }
        valid_cuts.push(cut);
    }

    valid_cuts
}

/// Snap every desired cut to the closest allowed index.
pub fn validify_indexes_looking_back(stats: &mut GraphStats<'_>, desired_cuts: &[i32]) -> Vec<i32> {
    if desired_cuts.is_empty() {
        return Vec::new();
    }
    let mut valid_cuts: Vec<i32> = Vec::new();
    valid_cuts.push(i32::MIN);
    for i in 1..stats.longest_path {
        if stats.is_cut_allowed(i) {
            valid_cuts.push(i as i32);
        }
    }
    if valid_cuts.len() == 1 {
        return Vec::new();
    }
    valid_cuts.push(i32::MAX);

    validify_inner(desired_cuts, &valid_cuts)
}

fn validify_inner(desired_cuts: &[i32], valid_cuts: &[i32]) -> Vec<i32> {
    debug_assert_eq!(valid_cuts[0], i32::MIN);
    debug_assert_eq!(valid_cuts[valid_cuts.len() - 1], i32::MAX);

    let mut final_cuts: Vec<i32> = Vec::new();
    let mut i_idx: usize = 0;
    let mut c_idx: usize = 0;
    let mut offset: i32 = 0;

    while i_idx < valid_cuts.len() - 1 && c_idx < desired_cuts.len() {
        let current = desired_cuts[c_idx].saturating_add(offset);
        while valid_cuts[i_idx + 1] < current {
            i_idx += 1;
        }

        let dist_lower = current.saturating_sub(valid_cuts[i_idx]);
        let dist_higher = valid_cuts[i_idx + 1].saturating_sub(current);
        let select: usize = if dist_lower > dist_higher { 1 } else { 0 };

        let selected_cut = valid_cuts[i_idx + select];
        final_cuts.push(selected_cut);
        offset = offset.saturating_add(selected_cut.saturating_sub(current));
        c_idx += 1;
        while c_idx < desired_cuts.len()
            && desired_cuts[c_idx].saturating_add(offset) <= selected_cut
        {
            c_idx += 1;
        }
        i_idx += 1 + select;
    }

    final_cuts
}

fn perform_cuts(graph: &mut LGraph, cuts: &[i32]) {
    if cuts.is_empty() {
        return;
    }

    let layer_count = graph.layers.len();
    let mut index: usize = 0;
    let mut new_index: usize = 0;
    let mut cut_iter = cuts.iter().copied();
    let mut next_cut = cut_iter.next().expect("cuts not empty");

    while index < layer_count {
        if index == next_cut as usize {
            new_index = 0;
            next_cut = match cut_iter.next() {
                Some(c) => c,
                None => (layer_count + 1) as i32,
            };
        }

        if index != new_index {
            let old_layer_nodes: SmallVec<NodeId, 32> =
                SmallVec::from_slice_copy(&graph.layers[index].nodes);
            for n in old_layer_nodes {
                let insert_pos = graph.layers[new_index].nodes.len();
                graph.insert_node_in_layer(n, new_index, insert_pos);

                if new_index == 0 {
                    let incoming: Vec<EdgeId> = graph.incoming_edges(n).collect();
                    for e in incoming {
                        graph.reverse_edge(e);
                        graph.properties.set(&CYCLIC, true);
                        cutting_utils::insert_dummies(graph, e, 1);
                    }
                }
            }
        }

        new_index += 1;
        index += 1;
    }

    // Drop empty layers.
    let kept_layers: Vec<LayerData> = std::mem::take(&mut graph.layers)
        .into_iter()
        .filter(|l| !l.nodes.is_empty())
        .collect();
    graph.layers = kept_layers;
    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for n in nodes {
            graph.node_mut(n).layer = Some(layer_idx).into();
        }
    }
}
