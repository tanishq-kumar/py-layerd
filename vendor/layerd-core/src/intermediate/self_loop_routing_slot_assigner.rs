//! Routing-slot assignment for self-hyper-loops.
//!
//! Determines which routing slot each hyper-loop occupies on each of the
//! sides it spans. Builds a `HyperEdgeSegment` graph from pairwise crossing
//! counts, runs the orthogonal router's cycle breaker, drains segment slots
//! from sinks toward sources, and compacts per side with label-conflict
//! avoidance.

use hashbrown::HashMap;

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        port::PortSide,
    },
    intermediate::{
        self_hyper_loop_labels::SelfHyperLoopLabels,
        self_loop_holder::{SelfLoopHolder, portside_index, portside_set_of},
    },
    p5_edge_routing::orthogonal::{
        cycle_detector,
        hyper_edge_dependency::HyperEdgeSegmentDependency,
        hyper_edge_segment::{HyperEdgeId, HyperEdgeSegment},
    },
};

/// Assign routing slots to every hyper-loop on every side it occupies.
pub fn assign_routing_slots(graph: &mut LGraph, holder: &mut SelfLoopHolder, node: NodeId) {
    let port_index = build_port_index(graph, node);
    let activity = compute_loop_activity(holder, &port_index);
    let label_matrix = compute_label_crossing_matrix(holder);

    // Materialize per-port-side now so the immutable borrow of `graph` ends
    // before we take `&mut graph.rng` for the cycle breaker.
    let side_per_port_idx: Vec<PortSide> =
        graph.node(node).ports.iter().map(|&pid| graph.port(pid).side).collect();

    let (mut segments, mut deps) =
        build_segment_graph(holder, &activity, &label_matrix, &port_index);

    // Use the shared LGraph rng so cycle-break choices stay deterministic
    // across components.
    let _critical =
        cycle_detector::detect_cycles_and_break(&mut segments, &mut deps, &mut graph.rng);

    assign_raw_slots_via_segments(&mut segments, &deps);
    apply_segment_slots_to_loops(holder, &segments);
    shift_towards_node(holder, &activity, &label_matrix, &side_per_port_idx);
    refresh_routing_slot_count(holder);
}

fn build_port_index(graph: &LGraph, node: NodeId) -> HashMap<PortId, u32> {
    let mut map = HashMap::new();
    for (i, &pid) in graph.node(node).ports.iter().enumerate() {
        map.insert(pid, i as u32);
    }
    map
}

/// For each hyper-loop, return a bit array indexed by port index indicating
/// whether the trunk runs along that port.
fn compute_loop_activity(
    holder: &SelfLoopHolder,
    port_index: &HashMap<PortId, u32>,
) -> Vec<Vec<bool>> {
    let port_count = port_index.len();
    let mut activity = Vec::with_capacity(holder.sl_hyper_loops.len());
    for hyper in &holder.sl_hyper_loops {
        let mut row = vec![false; port_count];
        // When `port_count == 0` the node's ports were hidden by the
        // self-loop preprocessor (pure self-loop ports + non-fixed order
        // + no nested external ports), so there is no trunk to mark.
        // Short-circuit the activity walk to avoid `port_count - 1`
        // underflow when a hyper-loop's leftmost/rightmost port is no
        // longer in `port_index`.
        if port_count == 0 {
            activity.push(row);
            continue;
        }
        if let (Some(left), Some(right)) = (hyper.leftmost_port, hyper.rightmost_port) {
            let Some(&left_idx_u32) = port_index.get(&left) else {
                activity.push(row);
                continue;
            };
            let Some(&right_idx_u32) = port_index.get(&right) else {
                activity.push(row);
                continue;
            };
            let left_idx = left_idx_u32 as usize;
            let right_idx = right_idx_u32 as usize;
            // Walk clockwise from (left+1) through right, wrapping around.
            let mut curr = if left_idx == 0 { port_count - 1 } else { left_idx - 1 };
            while curr != right_idx {
                curr = (curr + 1) % port_count;
                if curr < port_count {
                    row[curr] = true;
                }
            }
        }
        activity.push(row);
    }
    activity
}

fn count_crossings(
    activity: &[Vec<bool>],
    upper: usize,
    lower: usize,
    port_index: &HashMap<PortId, u32>,
    holder: &SelfLoopHolder,
) -> usize {
    let mut crossings = 0;
    for &pid in &holder.sl_hyper_loops[upper].sl_ports {
        let idx = port_index.get(&pid).copied().unwrap_or(0) as usize;
        if idx < activity[lower].len() && activity[lower][idx] {
            crossings += 1;
        }
    }
    crossings
}

/// One segment per hyper-loop. Segments host dependencies only; no edge
/// geometry.
fn build_segment_graph(
    holder: &SelfLoopHolder,
    activity: &[Vec<bool>],
    label_matrix: &[Vec<bool>],
    port_index: &HashMap<PortId, u32>,
) -> (Vec<HyperEdgeSegment>, Vec<HyperEdgeSegmentDependency>) {
    let n = holder.sl_hyper_loops.len();
    let mut segments: Vec<HyperEdgeSegment> =
        (0..n).map(|i| HyperEdgeSegment::new(HyperEdgeId(i as u32))).collect();
    let mut deps: Vec<HyperEdgeSegmentDependency> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            create_pairwise_dependency(
                holder,
                activity,
                label_matrix,
                port_index,
                i,
                j,
                &mut segments,
                &mut deps,
            );
        }
    }
    (segments, deps)
}

/// When directional crossings tie AND (crossings non-zero OR labels
/// overlap), emit bidirectional 0-weight dependencies — the cycle breaker
/// removes one and forces the loops onto separate slots.
#[allow(clippy::too_many_arguments)]
fn create_pairwise_dependency(
    holder: &SelfLoopHolder,
    activity: &[Vec<bool>],
    label_matrix: &[Vec<bool>],
    port_index: &HashMap<PortId, u32>,
    i: usize,
    j: usize,
    segments: &mut [HyperEdgeSegment],
    deps: &mut Vec<HyperEdgeSegmentDependency>,
) {
    let cross_i_above = count_crossings(activity, i, j, port_index, holder) as i32;
    let cross_j_above = count_crossings(activity, j, i, port_index, holder) as i32;
    let id_i = HyperEdgeId(i as u32);
    let id_j = HyperEdgeId(j as u32);
    if cross_i_above < cross_j_above {
        HyperEdgeSegmentDependency::create_and_add_regular(
            segments,
            deps,
            id_i,
            id_j,
            cross_j_above - cross_i_above,
        );
    } else if cross_j_above < cross_i_above {
        HyperEdgeSegmentDependency::create_and_add_regular(
            segments,
            deps,
            id_j,
            id_i,
            cross_i_above - cross_j_above,
        );
    } else if cross_i_above != 0 || labels_conflict(holder, label_matrix, i, j) {
        HyperEdgeSegmentDependency::create_and_add_regular(segments, deps, id_i, id_j, 0);
        HyperEdgeSegmentDependency::create_and_add_regular(segments, deps, id_j, id_i, 0);
    }
}

fn labels_conflict(
    holder: &SelfLoopHolder,
    label_matrix: &[Vec<bool>],
    i: usize,
    j: usize,
) -> bool {
    let Some(la) = holder.sl_hyper_loops[i].sl_labels.as_ref() else {
        return false;
    };
    let Some(lb) = holder.sl_hyper_loops[j].sl_labels.as_ref() else {
        return false;
    };
    label_matrix[la.id as usize][lb.id as usize]
}

/// Walks the now-acyclic dependency graph from sinks toward sources,
/// assigning each segment a slot one larger than its lowest-slot successor.
fn assign_raw_slots_via_segments(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
) {
    let n = segments.len();
    let mut out_remaining: Vec<usize> =
        (0..n).map(|i| segments[i].outgoing_dependencies.len()).collect();
    let mut sinks: Vec<usize> = (0..n).filter(|&i| out_remaining[i] == 0).collect();
    for seg in segments.iter_mut() {
        seg.routing_slot = 0;
    }
    while let Some(curr) = sinks.pop() {
        let next_slot = segments[curr].routing_slot + 1;
        let in_ids = segments[curr].incoming_dependencies.clone();
        for dep_id in in_ids {
            let Some(src_id) = deps[dep_id.index()].source else { continue };
            let src = src_id.index();
            if segments[src].routing_slot < next_slot {
                segments[src].routing_slot = next_slot;
            }
            if out_remaining[src] > 0 {
                out_remaining[src] -= 1;
                if out_remaining[src] == 0 {
                    sinks.push(src);
                }
            }
        }
    }
}

/// Project per-segment routing slots back onto the hyper-loop set.
fn apply_segment_slots_to_loops(holder: &mut SelfLoopHolder, segments: &[HyperEdgeSegment]) {
    for (idx, hyper) in holder.sl_hyper_loops.iter_mut().enumerate() {
        let slot = segments[idx].routing_slot.max(0) as u32;
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            if hyper.occupied_port_sides.contains(portside_set_of(side)) {
                hyper.set_routing_slot(side, slot);
            }
        }
    }
}

fn refresh_routing_slot_count(holder: &mut SelfLoopHolder) {
    let mut counts = [0_u32; 4];
    for hyper in &holder.sl_hyper_loops {
        for (side_idx, &slot) in hyper.routing_slot.iter().enumerate() {
            if hyper.occupied_port_sides.bits() & (1u8 << side_idx) != 0 {
                counts[side_idx] = counts[side_idx].max(slot + 1);
            }
        }
    }
    holder.routing_slot_count = counts;
}

/// `matrix[a][b]` is `true` when loops with labels `a` and `b` carry
/// horizontally-overlapping labels on the same N/S side.
fn compute_label_crossing_matrix(holder: &mut SelfLoopHolder) -> Vec<Vec<bool>> {
    let mut next_id = 0_u32;
    for hyper in &mut holder.sl_hyper_loops {
        if let Some(slabels) = hyper.sl_labels.as_mut() {
            slabels.id = next_id;
            next_id += 1;
        }
    }
    let n = next_id as usize;
    let mut matrix = vec![vec![false; n]; n];
    let loops = &holder.sl_hyper_loops;
    for i in 0..loops.len() {
        let Some(la) = loops[i].sl_labels.as_ref() else { continue };
        for other_loop in loops.iter().skip(i + 1) {
            let Some(lb) = other_loop.sl_labels.as_ref() else { continue };
            if labels_overlap(la, lb) {
                matrix[la.id as usize][lb.id as usize] = true;
                matrix[lb.id as usize][la.id as usize] = true;
            }
        }
    }
    matrix
}

fn labels_overlap(a: &SelfHyperLoopLabels, b: &SelfHyperLoopLabels) -> bool {
    let Some(side_a) = a.side else { return false };
    let Some(side_b) = b.side else { return false };
    if side_a != side_b || (side_a != PortSide::North && side_a != PortSide::South) {
        return false;
    }
    let (start_a, end_a) = (a.position.x, a.position.x + a.size.x);
    let (start_b, end_b) = (b.position.x, b.position.x + b.size.x);
    start_a <= end_b && end_a >= start_b
}

/// Push every loop on every side as close to the node as port reservations
/// and label conflicts allow.
fn shift_towards_node(
    holder: &mut SelfLoopHolder,
    activity: &[Vec<bool>],
    label_matrix: &[Vec<bool>],
    side_per_port_idx: &[PortSide],
) {
    let mut next_free = vec![0_u32; side_per_port_idx.len()];
    for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
        shift_side(holder, activity, label_matrix, side, side_per_port_idx, &mut next_free);
    }
}

/// Single-side shift driver invoked by `shift_towards_node` for each side.
fn shift_side(
    holder: &mut SelfLoopHolder,
    activity: &[Vec<bool>],
    label_matrix: &[Vec<bool>],
    side: PortSide,
    side_per_port_idx: &[PortSide],
    next_free: &mut [u32],
) {
    let side_idx = portside_index(side);
    if side_idx >= 4 {
        return;
    }
    let mask = 1u8 << side_idx;

    // Loops on this side, sorted ascending by current slot.
    let mut candidates: Vec<usize> = (0..holder.sl_hyper_loops.len())
        .filter(|&i| holder.sl_hyper_loops[i].occupied_port_sides.bits() & mask != 0)
        .collect();
    candidates.sort_by_key(|&i| holder.sl_hyper_loops[i].routing_slot[side_idx]);

    let mut min_idx = u32::MAX;
    let mut max_idx = 0_u32;
    let mut have_ports = false;
    for (i, &port_side) in side_per_port_idx.iter().enumerate() {
        if port_side == side {
            have_ports = true;
            min_idx = min_idx.min(i as u32);
            max_idx = max_idx.max(i as u32);
        }
    }

    if !have_ports {
        // No ports on this side — assign consecutive 0..N.
        for (i, &loop_idx) in candidates.iter().enumerate() {
            holder.sl_hyper_loops[loop_idx].set_routing_slot(side, i as u32);
        }
        return;
    }

    let n_labels = label_matrix.len();
    let mut slot_assigned_to_label = vec![-1_i32; n_labels];

    for &loop_idx in &candidates {
        let active = &activity[loop_idx];
        let mut lowest = 0_u32;
        for port_idx in min_idx..=max_idx {
            if active[port_idx as usize] {
                lowest = lowest.max(next_free[port_idx as usize]);
            }
        }
        // Skip past slots used by labels we conflict with.
        if let Some(our_labels) = holder.sl_hyper_loops[loop_idx].sl_labels.as_ref() {
            let our_id = our_labels.id as usize;
            let mut conflicts: Vec<i32> = Vec::new();
            for other in 0..n_labels {
                if label_matrix[our_id][other] && slot_assigned_to_label[other] >= 0 {
                    conflicts.push(slot_assigned_to_label[other]);
                }
            }
            while conflicts.contains(&(lowest as i32)) {
                lowest += 1;
            }
        }
        holder.sl_hyper_loops[loop_idx].set_routing_slot(side, lowest);
        for port_idx in min_idx..=max_idx {
            if active[port_idx as usize] {
                next_free[port_idx as usize] = lowest + 1;
            }
        }
        if let Some(our_labels) = holder.sl_hyper_loops[loop_idx].sl_labels.as_ref() {
            slot_assigned_to_label[our_labels.id as usize] = lowest as i32;
        }
    }
}
