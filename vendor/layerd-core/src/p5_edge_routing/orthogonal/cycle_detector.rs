//! Eades-Lin-Smyth feedback-arc detector for the hyperedge dependency graph.
//!
//! Given the segment/dependency arena, this module assigns each segment a
//! linear ordering mark and returns the dependencies whose reversal or removal
//! breaks every cycle. Phase A only produces the set — the actual reversal
//! and segment splitting live in Phase B.

use super::{
    hyper_edge_dependency::{DependencyId, DependencyType, HyperEdgeSegmentDependency},
    hyper_edge_segment::{HyperEdgeId, HyperEdgeSegment},
};
use crate::rng::Rng;

/// Detects the feedback arcs that break every cycle.
///
/// Returns the ids of outgoing dependencies that should be reversed or removed
/// to make the dependency graph acyclic. Marks are also written back to
/// `segments[*].mark`, which callers can use for ordering.
pub fn detect_cycles(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    critical_only: bool,
    rng: &mut impl Rng,
) -> Vec<DependencyId> {
    let mut sources: Vec<HyperEdgeId> = Vec::new();
    let mut sinks: Vec<HyperEdgeId> = Vec::new();

    initialize(segments, deps, critical_only, &mut sources, &mut sinks);
    compute_linear_ordering_marks(segments, deps, critical_only, &mut sources, &mut sinks, rng);

    let mut feedback: Vec<DependencyId> = Vec::new();
    for seg_id in 0..segments.len() {
        let out_ids = segments[seg_id].outgoing_dependencies.clone();
        for dep_id in out_ids {
            let dep = &deps[dep_id.index()];
            if critical_only && dep.dep_type != DependencyType::Critical {
                continue;
            }
            let Some(tgt) = dep.target else { continue };
            if segments[seg_id].mark > segments[tgt.index()].mark {
                feedback.push(dep_id);
            }
        }
    }
    feedback
}

fn initialize(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    critical_only: bool,
    sources: &mut Vec<HyperEdgeId>,
    sinks: &mut Vec<HyperEdgeId>,
) {
    let mut next_mark = -1i32;
    for seg_idx in 0..segments.len() {
        segments[seg_idx].mark = next_mark;
        next_mark -= 1;

        let (critical_in, critical_out, any_in, any_out) =
            sum_incident_weights(segments, deps, seg_idx);

        let (in_weight, out_weight) =
            if critical_only { (critical_in, critical_out) } else { (any_in, any_out) };

        segments[seg_idx].in_weight = in_weight;
        segments[seg_idx].critical_in_weight = critical_in;
        segments[seg_idx].out_weight = out_weight;
        segments[seg_idx].critical_out_weight = critical_out;

        if out_weight == 0 {
            sinks.push(HyperEdgeId(seg_idx as u32));
        } else if in_weight == 0 {
            sources.push(HyperEdgeId(seg_idx as u32));
        }
    }
}

fn sum_incident_weights(
    segments: &[HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    seg_idx: usize,
) -> (i32, i32, i32, i32) {
    let mut critical_in = 0;
    let mut critical_out = 0;
    let mut any_in = 0;
    let mut any_out = 0;

    for &dep_id in &segments[seg_idx].incoming_dependencies {
        let dep = &deps[dep_id.index()];
        any_in += dep.weight;
        if dep.dep_type == DependencyType::Critical {
            critical_in += dep.weight;
        }
    }
    for &dep_id in &segments[seg_idx].outgoing_dependencies {
        let dep = &deps[dep_id.index()];
        any_out += dep.weight;
        if dep.dep_type == DependencyType::Critical {
            critical_out += dep.weight;
        }
    }
    (critical_in, critical_out, any_in, any_out)
}

fn compute_linear_ordering_marks(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    critical_only: bool,
    sources: &mut Vec<HyperEdgeId>,
    sinks: &mut Vec<HyperEdgeId>,
    rng: &mut impl Rng,
) {
    let mut unprocessed: Vec<HyperEdgeId> =
        (0..segments.len()).map(|i| HyperEdgeId(i as u32)).collect();
    unprocessed.sort_by_key(|id| segments[id.index()].mark);
    let mark_base = segments.len() as i32;
    let mut next_sink_mark = mark_base - 1;
    let mut next_source_mark = mark_base + 1;

    let mut max_segments: Vec<HyperEdgeId> = Vec::new();

    while !unprocessed.is_empty() {
        while let Some(sink) = pop_front(sinks) {
            if !remove_unprocessed(&mut unprocessed, sink) {
                continue;
            }
            segments[sink.index()].mark = next_sink_mark;
            next_sink_mark -= 1;
            update_neighbors(segments, deps, sink, critical_only, sources, sinks);
        }

        while let Some(source) = pop_front(sources) {
            if !remove_unprocessed(&mut unprocessed, source) {
                continue;
            }
            segments[source.index()].mark = next_source_mark;
            next_source_mark += 1;
            update_neighbors(segments, deps, source, critical_only, sources, sinks);
        }

        // Pick the unprocessed segment with the highest out-flow. When not
        // restricted to critical dependencies we first try to pick a segment
        // with strictly positive critical out-flow, so critical dependencies
        // keep pointing right and are never reversed downstream.
        let mut max_outflow = i32::MIN;
        max_segments.clear();
        let mut forced: Option<HyperEdgeId> = None;

        for &seg_id in &unprocessed {
            let seg = &segments[seg_id.index()];
            if !critical_only && seg.critical_out_weight > 0 && seg.critical_in_weight <= 0 {
                forced = Some(seg_id);
                break;
            }
            let outflow = seg.out_weight - seg.in_weight;
            match outflow.cmp(&max_outflow) {
                std::cmp::Ordering::Greater => {
                    max_segments.clear();
                    max_segments.push(seg_id);
                    max_outflow = outflow;
                }
                std::cmp::Ordering::Equal => {
                    max_segments.push(seg_id);
                }
                std::cmp::Ordering::Less => {}
            }
        }

        let max_node = match forced {
            Some(id) => id,
            None => {
                if max_segments.is_empty() {
                    break;
                }
                let pick = rng.next_int(max_segments.len() as i32) as usize;
                max_segments[pick]
            }
        };
        remove_unprocessed(&mut unprocessed, max_node);
        segments[max_node.index()].mark = next_source_mark;
        next_source_mark += 1;
        update_neighbors(segments, deps, max_node, critical_only, sources, sinks);
    }

    let shift_base = segments.len() as i32 + 1;
    for seg in segments.iter_mut() {
        if seg.mark < mark_base {
            seg.mark += shift_base;
        }
    }
}

fn update_neighbors(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    seg: HyperEdgeId,
    critical_only: bool,
    sources: &mut Vec<HyperEdgeId>,
    sinks: &mut Vec<HyperEdgeId>,
) {
    let out_ids = segments[seg.index()].outgoing_dependencies.clone();
    for dep_id in out_ids {
        let dep = &deps[dep_id.index()];
        if critical_only && dep.dep_type != DependencyType::Critical {
            continue;
        }
        let Some(target) = dep.target else { continue };
        if segments[target.index()].mark >= 0 {
            continue;
        }
        if dep.weight <= 0 {
            continue;
        }
        let new_in = segments[target.index()].in_weight - dep.weight;
        segments[target.index()].in_weight = new_in;
        if dep.dep_type == DependencyType::Critical {
            segments[target.index()].critical_in_weight -= dep.weight;
        }
        if new_in <= 0 && segments[target.index()].out_weight > 0 {
            sources.push(target);
        }
    }

    let in_ids = segments[seg.index()].incoming_dependencies.clone();
    for dep_id in in_ids {
        let dep = &deps[dep_id.index()];
        if critical_only && dep.dep_type != DependencyType::Critical {
            continue;
        }
        let Some(source) = dep.source else { continue };
        if segments[source.index()].mark >= 0 {
            continue;
        }
        if dep.weight <= 0 {
            continue;
        }
        let new_out = segments[source.index()].out_weight - dep.weight;
        segments[source.index()].out_weight = new_out;
        if dep.dep_type == DependencyType::Critical {
            segments[source.index()].critical_out_weight -= dep.weight;
        }
        if new_out <= 0 && segments[source.index()].in_weight > 0 {
            sinks.push(source);
        }
    }
}

#[inline]
fn pop_front(list: &mut Vec<HyperEdgeId>) -> Option<HyperEdgeId> {
    if list.is_empty() { None } else { Some(list.remove(0)) }
}

fn remove_unprocessed(unprocessed: &mut Vec<HyperEdgeId>, id: HyperEdgeId) -> bool {
    if let Some(pos) = unprocessed.iter().position(|candidate| *candidate == id) {
        unprocessed.remove(pos);
        true
    } else {
        false
    }
}

/// Resolves the dependency graph by detecting feedback arcs and reversing
/// non-critical ones.
///
/// Regular (non-critical) feedback dependencies are reversed in place since
/// reversal is cheaper than segment splitting and does not risk edge overlap.
/// Critical feedback dependencies are returned to the caller, who is expected
/// to resolve them via `segment_splitter` — reversing them would cause visible
/// edge overlap.
pub fn detect_cycles_and_break(
    segments: &mut [HyperEdgeSegment],
    deps: &mut [HyperEdgeSegmentDependency],
    rng: &mut impl Rng,
) -> Vec<DependencyId> {
    // First pass: resolve critical cycles on their own and collect the
    // resulting feedback set for the splitter.
    let critical_feedback = detect_cycles(segments, deps, true, rng);

    break_non_critical_cycles(segments, deps, rng);

    critical_feedback
}

/// Detects cycles including regular dependencies and reverses only the
/// regular feedback arcs. Run after critical cycles have already been
/// handled by segment splitting (the `breakNonCriticalCycles` step in
pub fn break_non_critical_cycles(
    segments: &mut [HyperEdgeSegment],
    deps: &mut [HyperEdgeSegmentDependency],
    rng: &mut impl Rng,
) -> Vec<DependencyId> {
    let mut regular_feedback = detect_cycles(segments, deps, false, rng);
    regular_feedback.retain(|&id| deps[id.index()].dep_type == DependencyType::Regular);
    for &dep_id in &regular_feedback {
        HyperEdgeSegmentDependency::reverse(dep_id, deps, segments);
    }
    regular_feedback
}
