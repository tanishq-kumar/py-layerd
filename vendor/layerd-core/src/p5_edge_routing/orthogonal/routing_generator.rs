//! Orthogonal edge routing generator.
//!
//! Drives the full hyperedge-based routing pipeline: construct segments, add
//! dependencies, break critical and non-critical cycles, topologically
//! number the segments, and emit bend points via the direction strategy.
//! The caller is the top-level `route_edges` entry point exposed from
//! `p5_edge_routing::orthogonal`.

use hashbrown::HashMap;

use super::{
    cycle_detector::{break_non_critical_cycles, detect_cycles},
    direction::RoutingDirection,
    hyper_edge_dependency::{DependencyType, HyperEdgeSegmentDependency},
    hyper_edge_segment::{HyperEdgeId, HyperEdgeSegment},
    segment_splitter,
};
use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    properties::internal::EXT_PORT_SIDE,
    rng::Rng,
};

/// Factor applied to the edge spacing to derive the non-critical conflict
/// threshold.
const CONFLICT_THRESHOLD_FACTOR: f64 = 0.5;

/// Factor applied to the minimum distance between adjacent connections to
/// derive the critical conflict threshold.
const CRITICAL_CONFLICT_THRESHOLD_FACTOR: f64 = 0.2;

/// Per-conflict weight penalty used when rating a dependency.
const CONFLICT_PENALTY: i32 = 1;

/// Per-crossing weight penalty used when rating a dependency.
const CROSSING_PENALTY: i32 = 16;

/// Tolerance below which a segment is treated as a pure straight edge and
/// does not participate in slot assignment.
const TOLERANCE: f64 = 1.0e-3;

/// Sentinel returned by `count_conflicts` to signal a critical conflict.
const CRITICAL_CONFLICTS_DETECTED: i32 = -1;

/// Routes edges across every consecutive layer pair using the hyperedge
/// segment graph.
///
/// Returns the total horizontal extent consumed by the graph including the
/// routing channels inserted between layers. This replaces the legacy
/// linear-slot assignment in `p5_edge_routing::orthogonal::route_edges`.
pub fn route_edges(graph: &mut LGraph) -> f64 {
    if graph.layers.is_empty() {
        return 0.0;
    }

    let node_spacing = graph.options.spacing.node_node_between_layers;
    let edge_spacing = graph.options.spacing.edge_edge_between_layers;
    let edge_node_spacing = graph.options.spacing.edge_node_between_layers;

    let mut rng = graph.take_rng();
    let layer_count = graph.layers.len();

    let mut xpos = 0.0_f64;
    let mut left_idx: Option<usize> = None;
    let mut left_nodes: Vec<NodeId> = Vec::new();

    for next_idx in 0..=layer_count {
        let right_idx = (next_idx < layer_count).then_some(next_idx);
        let right_nodes: Vec<NodeId> =
            right_idx.map(|idx| graph.layers[idx].nodes.clone()).unwrap_or_default();

        if let Some(idx) = left_idx {
            crate::p5_edge_routing::place_nodes_horizontally(graph, idx, xpos);
            xpos += graph.layers[idx].size.x;
        }

        let start_pos = if left_idx.is_some() { xpos + edge_node_spacing } else { xpos };
        let slot_count = route_edges_between_nodes(
            graph,
            &left_nodes,
            &right_nodes,
            RoutingDirection::WestToEast,
            start_pos,
            edge_spacing,
            &mut rng,
        );

        let is_left_external = left_idx.is_none_or(|idx| is_layer_all_external_we(graph, idx));
        let is_right_external = right_idx.is_none_or(|idx| is_layer_all_external_we(graph, idx));

        if slot_count > 0 {
            let mut routing_width = (slot_count as f64 - 1.0).max(0.0) * edge_spacing;
            if left_idx.is_some() {
                routing_width += edge_node_spacing;
            }
            if right_idx.is_some() {
                routing_width += edge_node_spacing;
            }
            if routing_width < node_spacing && !is_left_external && !is_right_external {
                routing_width = node_spacing;
            }
            xpos += routing_width;
        } else if !is_left_external && !is_right_external {
            xpos += node_spacing;
        }

        left_idx = right_idx;
        left_nodes = right_nodes;
    }

    graph.put_rng(rng);

    graph.size.x = xpos;
    graph.size.x
}

fn is_layer_all_external_we(graph: &LGraph, layer_idx: usize) -> bool {
    let layer = &graph.layers[layer_idx];
    if layer.nodes.is_empty() {
        return false;
    }
    layer.nodes.iter().all(|&nid| {
        let node = graph.node(nid);
        if node.node_type != NodeType::ExternalPort {
            return false;
        }
        let side = node.properties.get(&EXT_PORT_SIDE);
        matches!(side, PortSide::West | PortSide::East)
    })
}

/// Creates hyperedge segments for arbitrary node sets.
fn create_hyper_edge_segments_for_nodes<I>(
    graph: &LGraph,
    nodes: I,
    port_side: PortSide,
    direction: RoutingDirection,
    segments: &mut Vec<HyperEdgeSegment>,
    port_to_segment: &mut HashMap<PortId, HyperEdgeId>,
) where
    I: IntoIterator<Item = NodeId>,
{
    for node_id in nodes {
        let node = graph.node(node_id);
        for &port_id in &node.ports {
            if graph.port(port_id).side != port_side {
                continue;
            }
            if port_to_segment.contains_key(&port_id) {
                continue;
            }
            if graph.port(port_id).outgoing_edges.is_empty() {
                continue;
            }
            let seg_id = HyperEdgeId(segments.len() as u32);
            let mut segment = HyperEdgeSegment::new(seg_id);
            add_port_to_segment(graph, direction, &mut segment, port_id, port_to_segment);
            segments.push(segment);
        }
    }
}

fn add_port_to_segment(
    graph: &LGraph,
    direction: RoutingDirection,
    segment: &mut HyperEdgeSegment,
    port_id: PortId,
    port_to_segment: &mut HashMap<PortId, HyperEdgeId>,
) {
    let mut stack = vec![port_id];
    while let Some(port_id) = stack.pop() {
        if port_to_segment.contains_key(&port_id) {
            continue;
        }
        port_to_segment.insert(port_id, segment.id);
        segment.ports.push(port_id);
        let coord = direction.port_position_on_hyper_node(graph, port_id);
        if graph.port(port_id).side == direction.source_port_side() {
            segment.add_incoming_connection(coord);
        } else {
            segment.add_outgoing_connection(coord);
        }

        let mut neighbours: Vec<PortId> = graph
            .port(port_id)
            .incoming_edges
            .iter()
            .map(|&eid| graph.edge(eid).source)
            .collect();
        neighbours
            .extend(graph.port(port_id).outgoing_edges.iter().map(|&eid| graph.edge(eid).target));
        for other in neighbours.into_iter().rev() {
            if !port_to_segment.contains_key(&other) {
                stack.push(other);
            }
        }
    }
}

/// Returns the minimum distance between any two distinct connection
/// coordinates.
fn minimum_horizontal_segment_distance(segments: &[HyperEdgeSegment]) -> f64 {
    let incoming = segments.iter().flat_map(|s| s.incoming_connection_coordinates.iter().copied());
    let outgoing = segments.iter().flat_map(|s| s.outgoing_connection_coordinates.iter().copied());
    minimum_difference(incoming).min(minimum_difference(outgoing))
}

fn minimum_difference(values: impl Iterator<Item = f64>) -> f64 {
    let mut sorted: Vec<f64> = values.collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    sorted.dedup_by(|a, b| (*a - *b).abs() < TOLERANCE);
    let mut min_diff = f64::MAX;
    for w in sorted.windows(2) {
        min_diff = min_diff.min(w[1] - w[0]);
    }
    min_diff
}

/// Adds the appropriate dependency between two segments when one is needed.
///
/// Applies the conflict/crossing weighting and short-circuits segments that
/// have zero length.
pub(super) fn create_dependency_if_necessary(
    segments: &mut [HyperEdgeSegment],
    deps: &mut Vec<HyperEdgeSegmentDependency>,
    a: HyperEdgeId,
    b: HyperEdgeId,
    conflict_threshold: f64,
    critical_conflict_threshold: f64,
) {
    let seg_a = &segments[a.index()];
    let seg_b = &segments[b.index()];
    if (seg_a.start_position - seg_a.end_position).abs() < TOLERANCE
        || (seg_b.start_position - seg_b.end_position).abs() < TOLERANCE
    {
        return;
    }

    let conflicts_ab = count_conflicts(
        &seg_a.outgoing_connection_coordinates,
        &seg_b.incoming_connection_coordinates,
        conflict_threshold,
        critical_conflict_threshold,
    );
    let conflicts_ba = count_conflicts(
        &seg_b.outgoing_connection_coordinates,
        &seg_a.incoming_connection_coordinates,
        conflict_threshold,
        critical_conflict_threshold,
    );

    let critical =
        conflicts_ab == CRITICAL_CONFLICTS_DETECTED || conflicts_ba == CRITICAL_CONFLICTS_DETECTED;

    if critical {
        if conflicts_ab == CRITICAL_CONFLICTS_DETECTED {
            HyperEdgeSegmentDependency::create_and_add_critical(segments, deps, b, a);
        }
        if conflicts_ba == CRITICAL_CONFLICTS_DETECTED {
            HyperEdgeSegmentDependency::create_and_add_critical(segments, deps, a, b);
        }
        return;
    }

    let seg_a = &segments[a.index()];
    let seg_b = &segments[b.index()];
    let crossings_ab = count_crossings(
        &seg_a.outgoing_connection_coordinates,
        seg_b.start_position,
        seg_b.end_position,
    ) + count_crossings(
        &seg_b.incoming_connection_coordinates,
        seg_a.start_position,
        seg_a.end_position,
    );
    let crossings_ba = count_crossings(
        &seg_b.outgoing_connection_coordinates,
        seg_a.start_position,
        seg_a.end_position,
    ) + count_crossings(
        &seg_a.incoming_connection_coordinates,
        seg_b.start_position,
        seg_b.end_position,
    );

    let dep_ab = CONFLICT_PENALTY * conflicts_ab + CROSSING_PENALTY * crossings_ab;
    let dep_ba = CONFLICT_PENALTY * conflicts_ba + CROSSING_PENALTY * crossings_ba;

    match dep_ab.cmp(&dep_ba) {
        std::cmp::Ordering::Less => {
            HyperEdgeSegmentDependency::create_and_add_regular(
                segments,
                deps,
                a,
                b,
                dep_ba - dep_ab,
            );
        }
        std::cmp::Ordering::Greater => {
            HyperEdgeSegmentDependency::create_and_add_regular(
                segments,
                deps,
                b,
                a,
                dep_ab - dep_ba,
            );
        }
        std::cmp::Ordering::Equal =>
            if dep_ab > 0 && dep_ba > 0 {
                HyperEdgeSegmentDependency::create_and_add_regular(segments, deps, a, b, 0);
                HyperEdgeSegmentDependency::create_and_add_regular(segments, deps, b, a, 0);
            },
    }
}

/// Counts conflicts between two sorted coordinate lists. Returns
/// `CRITICAL_CONFLICTS_DETECTED` the moment any two coordinates are closer
/// than the critical threshold.
fn count_conflicts(
    a: &[f64],
    b: &[f64],
    conflict_threshold: f64,
    critical_conflict_threshold: f64,
) -> i32 {
    if a.is_empty() || b.is_empty() {
        return 0;
    }
    let mut i = 0;
    let mut j = 0;
    let mut conflicts = 0i32;
    let mut pos1 = a[i];
    let mut pos2 = b[j];
    loop {
        let diff = pos1 - pos2;
        if diff.abs() < critical_conflict_threshold {
            return CRITICAL_CONFLICTS_DETECTED;
        }
        if diff.abs() < conflict_threshold {
            conflicts += 1;
        }
        if pos1 <= pos2 && i + 1 < a.len() {
            i += 1;
            pos1 = a[i];
        } else if pos2 <= pos1 && j + 1 < b.len() {
            j += 1;
            pos2 = b[j];
        } else {
            break;
        }
    }
    conflicts
}

/// Counts entries in a sorted coordinate list that fall inside `[start, end]`.
pub(super) fn count_crossings(positions: &[f64], start: f64, end: f64) -> i32 {
    let mut crossings = 0i32;
    for &pos in positions {
        if pos > end {
            break;
        } else if pos >= start {
            crossings += 1;
        }
    }
    crossings
}

/// Longest-path numbering on the dependency graph.
///
/// Assigns each segment's `routing_slot` so that every dependency points to
/// a strictly larger slot. Returns the maximum slot used, or `-1` when no
/// slots were assigned.
fn topological_numbering(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
) -> i32 {
    // Reset routing slots and compute remaining-degree counters using the
    // dependency arena directly — the `in_weight`/`out_weight` fields on the
    // segments were clobbered by the cycle breaker's weight accounting and
    // cannot be reused here.
    let n = segments.len();
    let mut in_degree: Vec<i32> = vec![0; n];
    let mut out_degree: Vec<i32> = vec![0; n];
    for dep in deps {
        let (Some(src), Some(tgt)) = (dep.source, dep.target) else { continue };
        out_degree[src.index()] += 1;
        in_degree[tgt.index()] += 1;
    }
    for seg in segments.iter_mut() {
        seg.routing_slot = 0;
    }

    let mut sources: Vec<HyperEdgeId> = Vec::new();
    let mut rightward_targets: Vec<HyperEdgeId> = Vec::new();
    for idx in 0..n {
        if in_degree[idx] == 0 {
            sources.push(HyperEdgeId(idx as u32));
        }
        if out_degree[idx] == 0 && segments[idx].incoming_connection_coordinates.is_empty() {
            rightward_targets.push(HyperEdgeId(idx as u32));
        }
    }

    let mut max_slot = -1i32;
    while let Some(source) = if sources.is_empty() { None } else { Some(sources.remove(0)) } {
        let source_slot = segments[source.index()].routing_slot;
        let out_count = segments[source.index()].outgoing_dependencies.len();
        for pos in 0..out_count {
            let dep_id = segments[source.index()].outgoing_dependencies[pos];
            let Some(target) = deps[dep_id.index()].target else { continue };
            let new_slot = (source_slot + 1).max(segments[target.index()].routing_slot);
            segments[target.index()].routing_slot = new_slot;
            max_slot = max_slot.max(new_slot);
            in_degree[target.index()] -= 1;
            if in_degree[target.index()] == 0 {
                sources.push(target);
            }
        }
    }

    // Pull right-facing targets to the far right so back-edges stay close to
    // their endpoints.
    if max_slot > -1 {
        for &id in &rightward_targets {
            segments[id.index()].routing_slot = max_slot;
        }
        while let Some(node) = if rightward_targets.is_empty() {
            None
        } else {
            Some(rightward_targets.remove(0))
        } {
            let node_slot = segments[node.index()].routing_slot;
            let in_count = segments[node.index()].incoming_dependencies.len();
            for pos in 0..in_count {
                let dep_id = segments[node.index()].incoming_dependencies[pos];
                let Some(source) = deps[dep_id.index()].source else { continue };
                if !segments[source.index()].incoming_connection_coordinates.is_empty() {
                    continue;
                }
                let new_slot = (node_slot - 1).min(segments[source.index()].routing_slot);
                segments[source.index()].routing_slot = new_slot;
                out_degree[source.index()] -= 1;
                if out_degree[source.index()] == 0 {
                    rightward_targets.push(source);
                }
            }
        }
    }

    max_slot
}

/// Run the full hyperedge pipeline between two arbitrary node sets.
///
/// Used by `HierarchicalPortOrthogonalEdgeRouter` when routing edges from
/// the graph body into external N/S port dummies. Returns the number of
/// routing slots consumed.
pub fn route_edges_between_nodes(
    graph: &mut LGraph,
    source_nodes: &[NodeId],
    target_nodes: &[NodeId],
    direction: RoutingDirection,
    start_pos: f64,
    edge_spacing: f64,
    rng: &mut impl Rng,
) -> usize {
    let mut segments: Vec<HyperEdgeSegment> = Vec::new();
    let mut port_to_segment: HashMap<PortId, HyperEdgeId> = HashMap::new();

    create_hyper_edge_segments_for_nodes(
        graph,
        source_nodes.iter().copied(),
        direction.source_port_side(),
        direction,
        &mut segments,
        &mut port_to_segment,
    );
    create_hyper_edge_segments_for_nodes(
        graph,
        target_nodes.iter().copied(),
        direction.target_port_side(),
        direction,
        &mut segments,
        &mut port_to_segment,
    );
    if segments.is_empty() {
        return 0;
    }

    let conflict_threshold = CONFLICT_THRESHOLD_FACTOR * edge_spacing;
    let critical_conflict_threshold =
        CRITICAL_CONFLICT_THRESHOLD_FACTOR * minimum_horizontal_segment_distance(&segments);

    let mut deps: Vec<HyperEdgeSegmentDependency> = Vec::new();
    for first_idx in 0..segments.len().saturating_sub(1) {
        for second_idx in (first_idx + 1)..segments.len() {
            create_dependency_if_necessary(
                &mut segments,
                &mut deps,
                HyperEdgeId(first_idx as u32),
                HyperEdgeId(second_idx as u32),
                conflict_threshold,
                critical_conflict_threshold,
            );
        }
    }

    let critical_count = deps.iter().filter(|d| d.dep_type == DependencyType::Critical).count();
    if critical_count >= 2 {
        let critical_feedback = detect_cycles(&mut segments, &deps, true, rng);
        segment_splitter::split_segments(
            &mut segments,
            &mut deps,
            &critical_feedback,
            conflict_threshold,
            critical_conflict_threshold,
        );
    }
    let _remaining = break_non_critical_cycles(&mut segments, &mut deps, rng);

    let _ = topological_numbering(&mut segments, &deps);

    // Rank count is the maximum `routing_slot` over segments that have
    // non-zero horizontal span. `topological_numbering` resets every
    // segment's slot to `0` before propagating along dependencies, so a
    // single segment with no incoming/outgoing dependency edges still has
    // slot `0` — counting it as one routing slot. Reading the propagation-
    // local `max_slot` return value here would stay at `-1` whenever the
    // dependency graph is empty, dropping the routing-area allocation for
    // outer in-layer edges and shifting the graph horizontally.
    let mut rank_count: i32 = -1;
    for segment in &segments {
        if (segment.start_position - segment.end_position).abs() < TOLERANCE {
            continue;
        }
        rank_count = rank_count.max(segment.routing_slot);
        direction.calculate_bend_points(graph, segment, &segments, start_pos, edge_spacing);
    }

    if rank_count < 0 { 0 } else { (rank_count + 1) as usize }
}
