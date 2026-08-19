//! Spline edge router.
//!
//! Drives spline routing: walks every layer pair, constructs `SplineSegment`s
//! (single-edge + 1:n hyperedges), builds the inter-segment dependency graph,
//! breaks cycles using a randomised tie-breaker, and topologically numbers
//! the survivors so each segment ends up with a routing `rank`. Horizontal
//! coordinates are then laid out like the orthogonal routing generator.
//!
//! Preliminary control points are **not** computed here — they are produced
//! in `FinalSplineBendpointsCalculator` after the rest of the pipeline has
//! had a chance to join long edges and remove label dummies.

use hashbrown::HashMap;
use smallvec::SmallVec;

use super::segment::{
    Dependency, SPLINE_EDGE_CHAIN, SPLINE_ROUTE_START, SPLINE_SEGMENT_STORE, SegmentId,
    SideToProcess, SplineSegment, is_qualified_as_starting_node,
};
use crate::{
    graph::{
        LGraph,
        index::{EdgeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    rng::SeededRng,
};

/// Ordered set of ports that preserves insertion order without hashing cost.
/// Implemented with a `SmallVec` + linear membership test; segment-local
/// membership sizes stay small (≤ ~16 ports per layer).
fn push_unique(vec: &mut SmallVec<PortId, 16>, port: PortId) {
    if !vec.contains(&port) {
        vec.push(port);
    }
}

fn contains_port(vec: &SmallVec<PortId, 16>, port: PortId) -> bool {
    vec.contains(&port)
}

/// Entry point used by `splines::route_edges`.
pub fn route(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        graph.size.x = 0.0;
        return;
    }

    let node_node_spacing = graph.options.spacing.node_node_between_layers;
    let edge_node_spacing = graph.options.spacing.edge_node_between_layers;
    let edge_edge_spacing = graph.options.spacing.edge_edge_between_layers;

    let sloppy_routing = matches!(
        graph.options.edge_routing_splines_mode,
        crate::options::enums::SplineRoutingMode::Sloppy
    );
    let sloppy_layer_spacing_factor =
        graph.options.edge_routing_splines_sloppy_layer_spacing_factor;

    // Router-level state that persists across every layer pair.
    let mut segments: Vec<SplineSegment> = Vec::new();
    let mut edge_to_segment: HashMap<EdgeId, SegmentId> = HashMap::new();
    let mut start_edges: Vec<EdgeId> = Vec::new();
    let mut successing_edge: HashMap<EdgeId, EdgeId> = HashMap::new();

    // Per-iteration state (reset in `clear_then_fill_mappings`).
    let mut edges_remaining_layer: Vec<EdgeId> = Vec::new();
    let mut segments_of_layer: Vec<SegmentId> = Vec::new();
    let mut left_ports_layer: SmallVec<PortId, 16> = SmallVec::new();
    let mut right_ports_layer: SmallVec<PortId, 16> = SmallVec::new();

    // Detect external-port-only first / last layers; the router skips a
    // node-node gap on those boundaries.
    let is_left_layer_external = is_external_west_east_layer(graph, 0);
    let is_right_layer_external = is_external_west_east_layer(graph, graph.layers.len() - 1);

    // Need the RNG for cycle-breaker tie-breaks; pull it out early so the
    // borrow on graph is unambiguous inside the main loop.
    let mut rng = graph.take_rng();

    let layer_count = graph.layers.len();
    let mut xpos = 0.0;
    // Iterate with `leftLayer = null` on first pass, then walk through
    // each right layer (encoded as `Option<usize>`).
    let mut left_layer: Option<usize> = None;
    let mut is_special_left_layer;
    let mut right_idx: i32 = 0;

    loop {
        let right_layer: Option<usize> =
            if right_idx < layer_count as i32 { Some(right_idx as usize) } else { None };

        // Reset per-layer bookkeeping.
        edges_remaining_layer.clear();
        segments_of_layer.clear();
        left_ports_layer.clear();
        right_ports_layer.clear();

        fill_mappings(
            graph,
            left_layer,
            right_layer,
            &mut edges_remaining_layer,
            &mut left_ports_layer,
            &mut right_ports_layer,
            &mut start_edges,
            &mut successing_edge,
        );

        create_segments_and_compute_ranking(
            graph,
            &mut segments,
            &mut edge_to_segment,
            &mut segments_of_layer,
            &mut edges_remaining_layer,
            &left_ports_layer,
            &right_ports_layer,
            &mut rng,
        );

        let slot_count = segments_of_layer
            .iter()
            .filter_map(|&id| {
                let seg = &segments[id.0 as usize];
                if seg.is_straight { None } else { Some(seg.rank + 1) }
            })
            .max()
            .unwrap_or(0);

        let mut x_segment_delta = 0.0;
        let mut right_layer_position = xpos;

        is_special_left_layer =
            left_layer.is_none() || (is_left_layer_external && left_layer == Some(0));
        let is_special_right_layer = right_layer.is_none()
            || (is_right_layer_external && right_layer == Some(layer_count - 1));

        if slot_count > 0 {
            let mut increment = 0.0;
            if left_layer.is_some() {
                increment += edge_node_spacing;
            }
            increment += (slot_count - 1) as f64 * edge_edge_spacing;
            if right_layer.is_some() {
                increment += edge_node_spacing;
            }

            if sloppy_routing && let Some(r) = right_layer {
                let sloppy_spacing = compute_sloppy_spacing(
                    graph,
                    r,
                    edge_edge_spacing,
                    node_node_spacing,
                    sloppy_layer_spacing_factor,
                );
                increment = increment.max(sloppy_spacing);
            }

            if increment < node_node_spacing && !is_special_left_layer && !is_special_right_layer {
                x_segment_delta = (node_node_spacing - increment) / 2.0;
                increment = node_node_spacing;
            }
            right_layer_position += increment;
        } else if !is_special_left_layer && !is_special_right_layer {
            right_layer_position += node_node_spacing;
        }

        if let Some(r) = right_layer {
            place_nodes_horizontally(graph, r, right_layer_position);
        }

        // Update per-segment bounding box info for this layer pair.
        let bbox_x = xpos;
        let bbox_width = right_layer_position - xpos;
        for &seg_id in &segments_of_layer {
            let seg = &mut segments[seg_id.0 as usize];
            seg.bbox_x = bbox_x;
            seg.bbox_width = bbox_width;
            seg.x_delta = x_segment_delta;
            seg.is_west_of_initial_layer = left_layer.is_none();
        }

        xpos = right_layer_position;
        if let Some(r) = right_layer {
            xpos += graph.layers[r].size.x;
        }

        left_layer = right_layer;
        if right_layer.is_none() {
            break;
        }
        right_idx += 1;
    }

    graph.put_rng(rng);

    // Store the edge chain and segment route on every starting edge.
    for &start in &start_edges {
        let chain = build_edge_chain(&successing_edge, start);
        graph.edge_mut(start).properties.set(&SPLINE_EDGE_CHAIN, chain.clone());

        let spline_ids = build_spline_path(graph, &mut segments, &edge_to_segment, &chain);
        graph.edge_mut(start).properties.set(&SPLINE_ROUTE_START, spline_ids);
    }

    graph.size.x = xpos;
    graph.properties.set(&SPLINE_SEGMENT_STORE, segments);
}

fn is_external_west_east_layer(graph: &LGraph, layer_idx: usize) -> bool {
    if layer_idx >= graph.layers.len() {
        return false;
    }
    for &node_id in &graph.layers[layer_idx].nodes {
        let node = graph.node(node_id);
        if node.node_type != NodeType::ExternalPort {
            return false;
        }
        let side = node.properties.get(&crate::properties::internal::EXT_PORT_SIDE);
        if !matches!(side, PortSide::West | PortSide::East) {
            return false;
        }
    }
    !graph.layers[layer_idx].nodes.is_empty()
}

fn place_nodes_horizontally(graph: &mut LGraph, layer_idx: usize, xpos: f64) {
    crate::p5_edge_routing::place_nodes_horizontally(graph, layer_idx, xpos);
}

fn absolute_anchor_y(graph: &LGraph, port_id: PortId) -> f64 {
    let port = graph.port(port_id);
    let node = graph.node(port.owner);
    node.position.y + port.position.y + port.anchor.y
}

fn compute_sloppy_spacing(
    graph: &LGraph,
    right_layer_idx: usize,
    edge_edge_spacing: f64,
    node_node_spacing: f64,
    sloppy_layer_spacing_factor: f64,
) -> f64 {
    let mut max_vert_diff = 0.0_f64;
    for &node_id in &graph.layers[right_layer_idx].nodes {
        let mut max_curr_input_y_diff = 0.0_f64;
        for &port_id in graph.node(node_id).ports.iter() {
            for &edge_id in &graph.port(port_id).incoming_edges {
                let edge = graph.edge(edge_id);
                let source_pos = absolute_anchor_y(graph, edge.source);
                let target_pos = absolute_anchor_y(graph, edge.target);
                max_curr_input_y_diff = max_curr_input_y_diff.max((target_pos - source_pos).abs());
            }
        }
        max_vert_diff = max_vert_diff.max(max_curr_input_y_diff);
    }

    sloppy_layer_spacing_factor
        * (1.0_f64).min(edge_edge_spacing / node_node_spacing)
        * max_vert_diff
}

fn fill_mappings(
    graph: &LGraph,
    left_layer: Option<usize>,
    right_layer: Option<usize>,
    edges_remaining: &mut Vec<EdgeId>,
    left_ports: &mut SmallVec<PortId, 16>,
    right_ports: &mut SmallVec<PortId, 16>,
    start_edges: &mut Vec<EdgeId>,
    successing_edge: &mut HashMap<EdgeId, EdgeId>,
) {
    if let Some(l) = left_layer {
        for &node_id in &graph.layers[l].nodes {
            let ports: SmallVec<PortId, 8> = SmallVec::from_iter(
                graph
                    .node(node_id)
                    .ports
                    .iter()
                    .copied()
                    .filter(|&p| graph.port(p).side == PortSide::East),
            );
            for source_port in ports {
                push_unique(left_ports, source_port);
                let outgoing: SmallVec<EdgeId, 4> =
                    graph.port(source_port).outgoing_edges.iter().copied().collect();
                for edge_id in outgoing {
                    let edge = graph.edge(edge_id);
                    if edge.source == edge.target
                        || graph.port(edge.source).owner == graph.port(edge.target).owner
                    {
                        continue; // skip self-loops
                    }
                    edges_remaining.push(edge_id);
                    if let Some(next) = successor_edge(graph, edge_id) {
                        successing_edge.insert(edge_id, next);
                    }

                    let source_node = graph.port(edge.source).owner;
                    if is_qualified_as_starting_node(graph, source_node) {
                        start_edges.push(edge_id);
                    }

                    let target_port = edge.target;
                    let target_layer = graph.node(graph.port(target_port).owner).layer;
                    if Some(target_layer.unwrap_or(usize::MAX)) == right_layer {
                        push_unique(right_ports, target_port);
                    } else if Some(target_layer.unwrap_or(usize::MAX)) == left_layer {
                        push_unique(left_ports, target_port);
                    } else {
                        edges_remaining.retain(|&e| e != edge_id);
                    }
                }
            }
        }
    }

    if let Some(r) = right_layer {
        for &node_id in &graph.layers[r].nodes {
            // Self-loops regardless of port side are tracked but we currently
            // defer self-loop spline routing to the dedicated self-loop
            // subsystem — just skip them here.
            for &_port_id in graph.node(node_id).ports.iter() {
                // No-op: self-loop tracking happens elsewhere.
            }

            let ports: SmallVec<PortId, 8> = SmallVec::from_iter(
                graph
                    .node(node_id)
                    .ports
                    .iter()
                    .copied()
                    .filter(|&p| graph.port(p).side == PortSide::West),
            );
            for source_port in ports {
                push_unique(right_ports, source_port);
                let outgoing: SmallVec<EdgeId, 4> =
                    graph.port(source_port).outgoing_edges.iter().copied().collect();
                for edge_id in outgoing {
                    let edge = graph.edge(edge_id);
                    if graph.port(edge.source).owner == graph.port(edge.target).owner {
                        continue;
                    }
                    edges_remaining.push(edge_id);
                    if let Some(next) = successor_edge(graph, edge_id) {
                        successing_edge.insert(edge_id, next);
                    }

                    let source_node = graph.port(edge.source).owner;
                    if is_qualified_as_starting_node(graph, source_node) {
                        start_edges.push(edge_id);
                    }

                    let target_port = edge.target;
                    let target_layer = graph.node(graph.port(target_port).owner).layer;
                    if Some(target_layer.unwrap_or(usize::MAX)) == right_layer {
                        push_unique(right_ports, target_port);
                    } else if Some(target_layer.unwrap_or(usize::MAX)) == left_layer {
                        push_unique(left_ports, target_port);
                    } else {
                        edges_remaining.retain(|&e| e != edge_id);
                    }
                }
            }
        }
    }
}

/// Find the successor of `edge` in a long-edge chain: the first outgoing edge
/// of the target node, but only if the target is itself a dummy node.
fn successor_edge(graph: &LGraph, edge_id: EdgeId) -> Option<EdgeId> {
    let target_node = graph.port(graph.edge(edge_id).target).owner;
    if matches!(graph.node(target_node).node_type, NodeType::Normal | NodeType::BreakingPoint) {
        return None;
    }
    for &port in graph.node(target_node).ports.iter() {
        if let Some(&next) = graph.port(port).outgoing_edges.first() {
            return Some(next);
        }
    }
    None
}

fn create_segments_and_compute_ranking(
    graph: &LGraph,
    segments: &mut Vec<SplineSegment>,
    edge_to_segment: &mut HashMap<EdgeId, SegmentId>,
    segments_of_layer: &mut Vec<SegmentId>,
    edges_remaining: &mut Vec<EdgeId>,
    left_ports: &SmallVec<PortId, 16>,
    right_ports: &SmallVec<PortId, 16>,
    rng: &mut SeededRng,
) {
    create_hyperedges(
        graph,
        segments,
        edge_to_segment,
        segments_of_layer,
        edges_remaining,
        left_ports,
        right_ports,
        SideToProcess::Left,
        true,
    );
    create_hyperedges(
        graph,
        segments,
        edge_to_segment,
        segments_of_layer,
        edges_remaining,
        left_ports,
        right_ports,
        SideToProcess::Left,
        false,
    );
    create_hyperedges(
        graph,
        segments,
        edge_to_segment,
        segments_of_layer,
        edges_remaining,
        left_ports,
        right_ports,
        SideToProcess::Right,
        true,
    );
    create_hyperedges(
        graph,
        segments,
        edge_to_segment,
        segments_of_layer,
        edges_remaining,
        left_ports,
        right_ports,
        SideToProcess::Right,
        false,
    );

    create_single_edge_segments(
        graph,
        segments,
        edge_to_segment,
        segments_of_layer,
        edges_remaining,
        left_ports,
        right_ports,
    );

    let len = segments_of_layer.len();
    for i in 0..len {
        for j in (i + 1)..len {
            let id_i = segments_of_layer[i];
            let id_j = segments_of_layer[j];
            create_dependency_with_graph(graph, segments, id_i, id_j);
        }
    }

    break_cycles(segments, segments_of_layer, rng);
    topological_numbering(segments, segments_of_layer);
}

fn create_single_edge_segments(
    graph: &LGraph,
    segments: &mut Vec<SplineSegment>,
    edge_to_segment: &mut HashMap<EdgeId, SegmentId>,
    segments_of_layer: &mut Vec<SegmentId>,
    edges_remaining: &mut Vec<EdgeId>,
    left_ports: &SmallVec<PortId, 16>,
    right_ports: &SmallVec<PortId, 16>,
) {
    let edges: Vec<EdgeId> = edges_remaining.clone();
    for edge_id in edges {
        let edge = graph.edge(edge_id);
        let source_side = if contains_port(left_ports, edge.source) {
            SideToProcess::Left
        } else if contains_port(right_ports, edge.source) {
            SideToProcess::Right
        } else {
            panic!("Source port must be in one of the port sets");
        };
        let target_side = if contains_port(left_ports, edge.target) {
            SideToProcess::Left
        } else if contains_port(right_ports, edge.target) {
            SideToProcess::Right
        } else {
            panic!("Target port must be in one of the port sets");
        };

        let seg = SplineSegment::single_edge(graph, edge_id, source_side, target_side);
        let new_id = SegmentId(segments.len() as u32);
        segments.push(seg);
        edge_to_segment.insert(edge_id, new_id);
        segments_of_layer.push(new_id);
    }
    edges_remaining.clear();
}

fn create_hyperedges(
    graph: &LGraph,
    segments: &mut Vec<SplineSegment>,
    edge_to_segment: &mut HashMap<EdgeId, SegmentId>,
    segments_of_layer: &mut Vec<SegmentId>,
    edges_remaining: &mut Vec<EdgeId>,
    left_ports: &SmallVec<PortId, 16>,
    right_ports: &SmallVec<PortId, 16>,
    side_to_process: SideToProcess,
    reversed: bool,
) {
    let ports_to_process: &SmallVec<PortId, 16> =
        if side_to_process == SideToProcess::Left { left_ports } else { right_ports };

    for &single_port in ports_to_process.iter() {
        let single_port_y = absolute_anchor_y(graph, single_port);
        let mut up_edges: Vec<(SideToProcess, EdgeId)> = Vec::new();
        let mut down_edges: Vec<(SideToProcess, EdgeId)> = Vec::new();

        // All edges connected to this port (incoming ∪ outgoing).
        let mut connected: SmallVec<EdgeId, 4> = SmallVec::new();
        for &e in graph.port(single_port).incoming_edges.iter() {
            connected.push(e);
        }
        for &e in graph.port(single_port).outgoing_edges.iter() {
            connected.push(e);
        }

        for edge_id in connected {
            let edge = graph.edge(edge_id);
            let is_reversed = edge.flags.contains(crate::graph::edge::EdgeFlags::REVERSED);
            if is_reversed != reversed {
                continue;
            }
            if !edges_remaining.contains(&edge_id) {
                continue;
            }
            let target_port = if edge.target == single_port { edge.source } else { edge.target };
            let target_y = absolute_anchor_y(graph, target_port);
            if super::segment::is_straight(target_y, single_port_y) {
                continue;
            }
            let tgt_side = if contains_port(left_ports, target_port) {
                SideToProcess::Left
            } else {
                SideToProcess::Right
            };
            if target_y < single_port_y {
                up_edges.push((tgt_side, edge_id));
            } else {
                down_edges.push((tgt_side, edge_id));
            }
        }

        if up_edges.len() > 1 {
            let seg = SplineSegment::hyperedge(graph, single_port, &up_edges, side_to_process);
            let new_id = SegmentId(segments.len() as u32);
            segments.push(seg);
            for &(_, eid) in &up_edges {
                edge_to_segment.insert(eid, new_id);
                edges_remaining.retain(|&e| e != eid);
            }
            segments_of_layer.push(new_id);
        }
        if down_edges.len() > 1 {
            let seg = SplineSegment::hyperedge(graph, single_port, &down_edges, side_to_process);
            let new_id = SegmentId(segments.len() as u32);
            segments.push(seg);
            for &(_, eid) in &down_edges {
                edge_to_segment.insert(eid, new_id);
                edges_remaining.retain(|&e| e != eid);
            }
            segments_of_layer.push(new_id);
        }
    }
}

fn create_dependency_with_graph(
    graph: &LGraph,
    segments: &mut [SplineSegment],
    a: SegmentId,
    b: SegmentId,
) {
    use super::math::is_between_f64;

    let (top_a, bot_a) = {
        let s = &segments[a.0 as usize];
        (s.hyper_edge_top_y_pos, s.hyper_edge_bottom_y_pos)
    };
    let (top_b, bot_b) = {
        let s = &segments[b.0 as usize];
        (s.hyper_edge_top_y_pos, s.hyper_edge_bottom_y_pos)
    };
    if top_a > bot_b || top_b > bot_a {
        return;
    }

    let a_right = segments[a.0 as usize].right_ports.clone();
    let a_left = segments[a.0 as usize].left_ports.clone();
    let b_right = segments[b.0 as usize].right_ports.clone();
    let b_left = segments[b.0 as usize].left_ports.clone();

    let mut a_counter = 0i32;
    for p in a_right {
        if is_between_f64(absolute_anchor_y(graph, p), top_b, bot_b) {
            a_counter += 1;
        }
    }
    for p in a_left {
        if is_between_f64(absolute_anchor_y(graph, p), top_b, bot_b) {
            a_counter -= 1;
        }
    }
    let mut b_counter = 0i32;
    for p in b_right {
        if is_between_f64(absolute_anchor_y(graph, p), top_a, bot_a) {
            b_counter += 1;
        }
    }
    for p in b_left {
        if is_between_f64(absolute_anchor_y(graph, p), top_a, bot_a) {
            b_counter -= 1;
        }
    }

    if a_counter < b_counter {
        push_dependency(segments, a, b, b_counter - a_counter);
    } else if b_counter < a_counter {
        push_dependency(segments, b, a, a_counter - b_counter);
    } else {
        push_dependency(segments, b, a, 0);
        push_dependency(segments, a, b, 0);
    }
}

fn push_dependency(
    segments: &mut [SplineSegment],
    source: SegmentId,
    target: SegmentId,
    weight: i32,
) {
    let dep = Dependency { source, target, weight };
    segments[source.0 as usize].outgoing.push(dep);
    segments[target.0 as usize].incoming.push(dep);
}

fn break_cycles(
    segments: &mut [SplineSegment],
    segments_of_layer: &[SegmentId],
    rng: &mut SeededRng,
) {
    let mut sources: Vec<SegmentId> = Vec::new();
    let mut sinks: Vec<SegmentId> = Vec::new();

    let mut next_mark = -1i32;
    for &id in segments_of_layer {
        let seg = &mut segments[id.0 as usize];
        seg.mark = next_mark;
        next_mark -= 1;
        let mut inweight = 0;
        let mut outweight = 0;
        for d in &seg.outgoing {
            outweight += d.weight;
        }
        for d in &seg.incoming {
            inweight += d.weight;
        }
        seg.inweight = inweight;
        seg.outweight = outweight;
        if outweight == 0 {
            sinks.push(id);
        } else if inweight == 0 {
            sources.push(id);
        }
    }

    let mut unprocessed: Vec<SegmentId> = segments_of_layer.to_vec();
    let mark_base = segments_of_layer.len() as i32;
    let mut next_left = mark_base + 1;
    let mut next_right = mark_base - 1;
    let mut max_edges: Vec<SegmentId> = Vec::new();

    while !unprocessed.is_empty() {
        while let Some(sink) = sinks.pop() {
            unprocessed.retain(|&id| id != sink);
            segments[sink.0 as usize].mark = next_right;
            next_right -= 1;
            update_neighbors(segments, sink, &mut sources, &mut sinks);
        }
        while let Some(source) = sources.pop() {
            unprocessed.retain(|&id| id != source);
            segments[source.0 as usize].mark = next_left;
            next_left += 1;
            update_neighbors(segments, source, &mut sources, &mut sinks);
        }

        let mut max_outflow = i32::MIN;
        for &id in &unprocessed {
            let seg = &segments[id.0 as usize];
            let outflow = seg.outweight - seg.inweight;
            if outflow >= max_outflow {
                if outflow > max_outflow {
                    max_edges.clear();
                    max_outflow = outflow;
                }
                max_edges.push(id);
            }
        }

        if !max_edges.is_empty() {
            let chosen_idx = rng.next_int(max_edges.len() as i32) as usize;
            let chosen = max_edges[chosen_idx];
            unprocessed.retain(|&id| id != chosen);
            segments[chosen.0 as usize].mark = next_left;
            next_left += 1;
            update_neighbors(segments, chosen, &mut sources, &mut sinks);
            max_edges.clear();
        }
    }

    // Shift ranks that are left of the mark base.
    let shift_base = segments_of_layer.len() as i32 + 1;
    for &id in segments_of_layer {
        let seg = &mut segments[id.0 as usize];
        if seg.mark < mark_base {
            seg.mark += shift_base;
        }
    }

    // Process edges that point left: remove zero-weight, reverse others.
    let ids_snapshot: Vec<SegmentId> = segments_of_layer.to_vec();
    for source_id in ids_snapshot {
        let outgoing = segments[source_id.0 as usize].outgoing.clone();
        let source_mark = segments[source_id.0 as usize].mark;
        for (i, dep) in outgoing.iter().enumerate() {
            let target_id = dep.target;
            let target_mark = segments[target_id.0 as usize].mark;
            if source_mark > target_mark {
                // Remove from source.outgoing (by identity of i-th dep).
                segments[source_id.0 as usize].outgoing.remove_matching_index(i, dep);
                segments[target_id.0 as usize].incoming.remove_matching_dep(*dep);
                if dep.weight > 0 {
                    let reversed =
                        Dependency { source: target_id, target: source_id, weight: dep.weight };
                    segments[target_id.0 as usize].outgoing.push(reversed);
                    segments[source_id.0 as usize].incoming.push(reversed);
                }
            }
        }
    }
}

/// Helpers used above because plain `Vec::remove` cannot find by value.
trait DepListExt {
    fn remove_matching_index(&mut self, i: usize, probe: &Dependency);
    fn remove_matching_dep(&mut self, probe: Dependency);
}

impl DepListExt for Vec<Dependency> {
    fn remove_matching_index(&mut self, i: usize, probe: &Dependency) {
        if self.get(i).map(|d| dep_eq(d, probe)).unwrap_or(false) {
            self.remove(i);
            return;
        }
        // Fallback: locate by value.
        if let Some(pos) = self.iter().position(|d| dep_eq(d, probe)) {
            self.remove(pos);
        }
    }

    fn remove_matching_dep(&mut self, probe: Dependency) {
        if let Some(pos) = self.iter().position(|d| dep_eq(d, &probe)) {
            self.remove(pos);
        }
    }
}

fn dep_eq(a: &Dependency, b: &Dependency) -> bool {
    a.source == b.source && a.target == b.target && a.weight == b.weight
}

fn update_neighbors(
    segments: &mut [SplineSegment],
    edge: SegmentId,
    sources: &mut Vec<SegmentId>,
    sinks: &mut Vec<SegmentId>,
) {
    let outgoing = segments[edge.0 as usize].outgoing.clone();
    for dep in outgoing {
        let target_id = dep.target;
        if segments[target_id.0 as usize].mark < 0 && dep.weight > 0 {
            segments[target_id.0 as usize].inweight -= dep.weight;
            if segments[target_id.0 as usize].inweight <= 0
                && segments[target_id.0 as usize].outweight > 0
            {
                sources.push(target_id);
            }
        }
    }
    let incoming = segments[edge.0 as usize].incoming.clone();
    for dep in incoming {
        let source_id = dep.source;
        if segments[source_id.0 as usize].mark < 0 && dep.weight > 0 {
            segments[source_id.0 as usize].outweight -= dep.weight;
            if segments[source_id.0 as usize].outweight <= 0
                && segments[source_id.0 as usize].inweight > 0
            {
                sinks.push(source_id);
            }
        }
    }
}

fn topological_numbering(segments: &mut [SplineSegment], segments_of_layer: &[SegmentId]) {
    let mut sources: Vec<SegmentId> = Vec::new();
    let mut rightward_targets: Vec<SegmentId> = Vec::new();

    for &id in segments_of_layer {
        let seg = &mut segments[id.0 as usize];
        seg.rank = 0;
        seg.inweight = seg.incoming.len() as i32;
        seg.outweight = seg.outgoing.len() as i32;
        if seg.inweight == 0 {
            sources.push(id);
        }
        if seg.outweight == 0 && seg.left_ports.is_empty() {
            rightward_targets.push(id);
        }
    }

    let mut max_rank = -1i32;
    while !sources.is_empty() {
        let current = sources.remove(0);
        let outgoing = segments[current.0 as usize].outgoing.clone();
        for dep in outgoing {
            let target_id = dep.target;
            let new_rank = segments[current.0 as usize].rank + 1;
            if new_rank > segments[target_id.0 as usize].rank {
                segments[target_id.0 as usize].rank = new_rank;
            }
            max_rank = max_rank.max(segments[target_id.0 as usize].rank);
            segments[target_id.0 as usize].inweight -= 1;
            if segments[target_id.0 as usize].inweight == 0 {
                sources.push(target_id);
            }
        }
    }

    if max_rank > -1 {
        for &id in &rightward_targets {
            segments[id.0 as usize].rank = max_rank;
        }
        while !rightward_targets.is_empty() {
            let current = rightward_targets.remove(0);
            let incoming = segments[current.0 as usize].incoming.clone();
            for dep in incoming {
                let source_id = dep.source;
                if !segments[source_id.0 as usize].left_ports.is_empty() {
                    continue;
                }
                let new_rank = segments[current.0 as usize].rank - 1;
                if new_rank < segments[source_id.0 as usize].rank {
                    segments[source_id.0 as usize].rank = new_rank;
                }
                segments[source_id.0 as usize].outweight -= 1;
                if segments[source_id.0 as usize].outweight == 0 {
                    rightward_targets.push(source_id);
                }
            }
        }
    }
}

fn build_edge_chain(successing_edge: &HashMap<EdgeId, EdgeId>, start: EdgeId) -> Vec<EdgeId> {
    let mut chain = Vec::new();
    let mut current = start;
    loop {
        chain.push(current);
        match successing_edge.get(&current) {
            Some(&next) => current = next,
            None => break,
        }
    }
    chain
}

fn build_spline_path(
    graph: &LGraph,
    segments: &mut [SplineSegment],
    edge_to_segment: &HashMap<EdgeId, SegmentId>,
    edge_chain: &[EdgeId],
) -> Vec<SegmentId> {
    let mut out: Vec<SegmentId> = Vec::new();
    for &edge_id in edge_chain {
        let Some(&seg_id) = edge_to_segment.get(&edge_id) else { continue };
        let edge = graph.edge(edge_id);
        let seg = &mut segments[seg_id.0 as usize];
        seg.source_port = Some(edge.source);
        seg.target_port = Some(edge.target);
        out.push(seg_id);
    }
    if let Some(&first_id) = out.first() {
        let seg = &mut segments[first_id.0 as usize];
        seg.initial_segment = true;
        if let Some(&first_edge) = edge_chain.first() {
            let edge = graph.edge(first_edge);
            seg.source_node = Some(graph.port(edge.source).owner);
        }
    }
    if let Some(&last_id) = out.last() {
        let seg = &mut segments[last_id.0 as usize];
        seg.last_segment = true;
        if let Some(&last_edge) = edge_chain.last() {
            let edge = graph.edge(last_edge);
            seg.target_node = Some(graph.port(edge.target).owner);
        }
    }
    out
}
