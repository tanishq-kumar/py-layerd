//! Final spline endpoint calculation.
//!
//! Post-P5 intermediate. Turns the preliminary `SplineSegment` routes produced
//! by `p5_edge_routing::splines::router` into concrete NUB control points on
//! every edge, then converts each long-edge chain's combined control points
//! into bezier bend points via `NubSpline::to_bezier`.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    math::Vec2,
    options::enums::SplineRoutingMode,
    p5_edge_routing::splines::{
        math as splines_math,
        nub_spline::{DIM as SPLINE_DIMENSION, NubSpline},
        segment::{
            EdgeInformation, SPLINE_EDGE_CHAIN, SPLINE_ROUTE_START, SPLINE_SEGMENT_STORE,
            SegmentId, SplineSegment, is_qualified_as_starting_node,
        },
    },
};

const ONE_HALF: f64 = 0.5;

/// Gap between the source/target anchor and the straightening control point
/// inserted for `SplineRoutingMode::Conservative`.
const NODE_TO_STRAIGHTENING_CP_GAP: f64 = 5.0;

/// Multiplier for sloppy curve center offset.
const SLOPPY_CENTER_CP_MULTIPLIER: f64 = 0.4;

/// Runs the final spline bend-point calculation.
pub fn calculate(graph: &mut LGraph) {
    if !graph.properties.has(&SPLINE_SEGMENT_STORE) {
        return;
    }

    let edge_edge_spacing = graph.options.spacing.edge_edge_between_layers;
    let edge_node_spacing = graph.options.spacing.edge_node_between_layers;
    let spline_mode = graph.options.edge_routing_splines_mode;

    index_nodes_per_layer(graph);

    // Take ownership of the segment store so we can mutate it alongside the
    // graph without alias conflicts; we put it back at the end.
    let mut segments: Vec<SplineSegment> =
        graph.properties.get_ref(&SPLINE_SEGMENT_STORE).cloned().unwrap_or_default();

    let start_edges: Vec<EdgeId> = collect_start_edges(graph);

    // Pass 1: populate NUB control points per segment.
    for &edge_id in &start_edges {
        let seg_ids: Vec<SegmentId> = graph
            .edge(edge_id)
            .properties
            .get_ref(&SPLINE_ROUTE_START)
            .cloned()
            .unwrap_or_default();
        for &sid in &seg_ids {
            calculate_control_points(
                graph,
                &mut segments,
                sid,
                edge_edge_spacing,
                edge_node_spacing,
                spline_mode,
            );
        }
        graph.edge_mut(edge_id).properties.set(&SPLINE_ROUTE_START, Vec::new());
    }

    // Pass 2: turn the per-edge NUB control points into bezier bend points.
    for &edge_id in &start_edges {
        let surviving: Option<EdgeId> = graph
            .edge(edge_id)
            .properties
            .get(&crate::properties::internal::SPLINE_SURVIVING_EDGE);
        let edge_chain: Vec<EdgeId> = graph
            .edge(edge_id)
            .properties
            .get_ref(&SPLINE_EDGE_CHAIN)
            .cloned()
            .unwrap_or_default();
        calculate_bezier_bend_points(graph, &edge_chain, surviving, spline_mode);
        graph.edge_mut(edge_id).properties.set(&SPLINE_EDGE_CHAIN, Vec::new());
    }

    graph.properties.set(&SPLINE_SEGMENT_STORE, segments);
}

fn index_nodes_per_layer(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for (i, node_id) in node_ids.iter().enumerate() {
            graph.node_mut(*node_id).id = i as u32;
        }
    }
}

fn collect_start_edges(graph: &LGraph) -> Vec<EdgeId> {
    let mut out = Vec::new();
    for layer_idx in 0..graph.layers.len() {
        for &node_id in &graph.layers[layer_idx].nodes {
            for &port_id in graph.node(node_id).ports.iter() {
                for &edge_id in &graph.port(port_id).outgoing_edges {
                    let edge = graph.edge(edge_id);
                    if graph.port(edge.source).owner == graph.port(edge.target).owner {
                        continue;
                    }
                    if graph.edge(edge_id).properties.has(&SPLINE_ROUTE_START) {
                        out.push(edge_id);
                    }
                }
            }
        }
    }
    out
}

fn calculate_control_points(
    graph: &mut LGraph,
    segments: &mut [SplineSegment],
    seg_id: SegmentId,
    edge_edge_spacing: f64,
    edge_node_spacing: f64,
    spline_mode: SplineRoutingMode,
) {
    let idx = seg_id.0 as usize;
    if segments[idx].handled {
        return;
    }
    segments[idx].handled = true;

    let edge_ids: Vec<EdgeId> = segments[idx].edges.clone();
    for edge in edge_ids {
        let ei = *segments[idx].edge_information.get(&edge).expect("edge info missing");
        let is_straight = segments[idx].is_straight;
        let is_hyper = segments[idx].is_hyper_edge();

        if is_straight && !is_hyper {
            append_cp_straight(graph, segments, idx, edge);
            continue;
        }

        if ei.inverted_left || ei.inverted_right {
            append_cp_inverted(
                graph,
                segments,
                idx,
                edge,
                ei,
                edge_edge_spacing,
                edge_node_spacing,
            );
            continue;
        }

        let sloppy = spline_mode == SplineRoutingMode::Sloppy
            && (ei.normal_source_node || ei.normal_target_node)
            && segment_allows_sloppy_routing(graph, &segments[idx])
            && !is_hyper;

        if sloppy {
            append_cp_sloppy(graph, segments, idx, edge, ei);
        } else {
            append_cp_conservative(
                graph,
                segments,
                idx,
                edge,
                ei,
                edge_edge_spacing,
                edge_node_spacing,
            );
        }
    }

    if segments[idx].inverse_order {
        let edges = segments[idx].edges.clone();
        for e in edges {
            let mut bps = std::mem::take(&mut graph.edge_mut(e).bend_points);
            bps.reverse();
            graph.edge_mut(e).bend_points = bps;
        }
    }
}

fn append_cp_straight(graph: &mut LGraph, segments: &[SplineSegment], idx: usize, edge_id: EdgeId) {
    let seg = &segments[idx];
    let x_start = seg.bbox_x;
    let x_end = seg.bbox_x + seg.bbox_width;
    let cp = Vec2::new(x_start + (x_end - x_start) / 2.0, seg.center_control_point_y);
    graph.edge_mut(edge_id).bend_points.push(cp);
}

fn append_cp_inverted(
    graph: &mut LGraph,
    segments: &[SplineSegment],
    idx: usize,
    edge_id: EdgeId,
    ei: EdgeInformation,
    edge_edge_spacing: f64,
    edge_node_spacing: f64,
) {
    let seg = &segments[idx];
    let start_x = seg.bbox_x;
    let end_x = seg.bbox_x + seg.bbox_width;
    let y_source = ei.start_y;
    let y_target = ei.end_y;

    let source_straight = if ei.inverted_left {
        Vec2::new(end_x, y_source)
    } else {
        Vec2::new(start_x, y_source)
    };
    let target_straight = if ei.inverted_right {
        Vec2::new(start_x, y_target)
    } else {
        Vec2::new(end_x, y_target)
    };

    let mut center_x = start_x;
    if !seg.is_west_of_initial_layer {
        center_x += edge_node_spacing;
    }
    center_x += seg.x_delta + seg.rank as f64 * edge_edge_spacing;

    let source_vertical = Vec2::new(center_x, y_source);
    let target_vertical = Vec2::new(center_x, y_target);

    let is_hyperedge = seg.edges.len() > 1;
    graph.edge_mut(edge_id).bend_points.push(source_straight);
    graph.edge_mut(edge_id).bend_points.push(source_vertical);
    if is_hyperedge {
        let center = Vec2::new(center_x, seg.center_control_point_y);
        graph.edge_mut(edge_id).bend_points.push(center);
    }
    graph.edge_mut(edge_id).bend_points.push(target_vertical);
    graph.edge_mut(edge_id).bend_points.push(target_straight);
}

fn append_cp_conservative(
    graph: &mut LGraph,
    segments: &[SplineSegment],
    idx: usize,
    edge_id: EdgeId,
    ei: EdgeInformation,
    edge_edge_spacing: f64,
    edge_node_spacing: f64,
) {
    let seg = &segments[idx];
    let start_x = seg.bbox_x;
    let end_x = seg.bbox_x + seg.bbox_width;
    let y_source = ei.start_y;
    let y_target = ei.end_y;

    let source_straight = Vec2::new(start_x, y_source);
    let target_straight = Vec2::new(end_x, y_target);

    let mut center_x = start_x;
    if !seg.is_west_of_initial_layer {
        center_x += edge_node_spacing;
    }
    center_x += seg.x_delta + seg.rank as f64 * edge_edge_spacing;
    let source_vertical = Vec2::new(center_x, y_source);
    let target_vertical = Vec2::new(center_x, y_target);

    let is_hyperedge = seg.edges.len() > 1;
    graph.edge_mut(edge_id).bend_points.push(source_straight);
    graph.edge_mut(edge_id).bend_points.push(source_vertical);
    if is_hyperedge {
        let center = Vec2::new(center_x, seg.center_control_point_y);
        graph.edge_mut(edge_id).bend_points.push(center);
    }
    graph.edge_mut(edge_id).bend_points.push(target_vertical);
    graph.edge_mut(edge_id).bend_points.push(target_straight);
}

fn append_cp_sloppy(
    graph: &mut LGraph,
    segments: &[SplineSegment],
    idx: usize,
    edge_id: EdgeId,
    ei: EdgeInformation,
) {
    let seg = &segments[idx];
    let start_x = seg.bbox_x;
    let end_x = seg.bbox_x + seg.bbox_width;
    let y_source = ei.start_y;
    let y_target = ei.end_y;
    let edge_points_downwards = y_source < y_target;

    let source_straight = Vec2::new(start_x, y_source);
    let target_straight = Vec2::new(end_x, y_target);
    let center_x = (start_x + end_x) / 2.0;
    let source_vertical = Vec2::new(center_x, y_source);
    let target_vertical = Vec2::new(center_x, y_target);

    let center_y = compute_sloppy_center_y(graph, edge_id, y_source, y_target);

    let v1 = match seg.source_port {
        Some(p) => abs_anchor(graph, p),
        None => Vec2::new(start_x, y_source),
    };
    let v2 = Vec2::new(center_x, center_y);
    let v3 = match seg.target_port {
        Some(p) => abs_anchor(graph, p),
        None => Vec2::new(end_x, y_target),
    };

    let approx = approximate_bezier_segment_2(v1, v2, v3);

    let mut short_cut_source = false;
    if let Some(src_port) = seg.source_port {
        let src_node = graph.port(src_port).owner;
        if let Some(layer_idx) = graph.node(src_node).layer.get()
            && ei.normal_source_node
        {
            let nodes_in_layer = graph.layers[layer_idx].nodes.len();
            let src_node_id = graph.node(src_node).id;
            let need_check = (edge_points_downwards
                && (src_node_id as usize) < nodes_in_layer.saturating_sub(1))
                || (!edge_points_downwards && src_node_id > 0);
            if !need_check {
                short_cut_source = true;
            } else {
                let neighbor_idx = if edge_points_downwards {
                    (src_node_id + 1) as usize
                } else {
                    (src_node_id - 1) as usize
                };
                if neighbor_idx < graph.layers[layer_idx].nodes.len() {
                    let neighbor_node = graph.layers[layer_idx].nodes[neighbor_idx];
                    let bbox = node_to_bounding_box(graph, neighbor_node);
                    short_cut_source = !(rect_intersects(&bbox, v1, approx[0])
                        || rect_contains(&bbox, v1, approx[0]));
                }
            }
        }
    }

    let mut short_cut_target = false;
    if let Some(tgt_port) = seg.target_port {
        let tgt_node = graph.port(tgt_port).owner;
        if let Some(layer_idx) = graph.node(tgt_node).layer.get()
            && ei.normal_target_node
        {
            let nodes_in_layer = graph.layers[layer_idx].nodes.len();
            let tgt_node_id = graph.node(tgt_node).id;
            let need_check = (edge_points_downwards && tgt_node_id > 0)
                || (!edge_points_downwards
                    && (tgt_node_id as usize) < nodes_in_layer.saturating_sub(1));
            if !need_check {
                short_cut_target = true;
            } else {
                let neighbor_idx = if edge_points_downwards {
                    (tgt_node_id - 1) as usize
                } else {
                    (tgt_node_id + 1) as usize
                };
                if neighbor_idx < graph.layers[layer_idx].nodes.len() {
                    let neighbor_node = graph.layers[layer_idx].nodes[neighbor_idx];
                    let bbox = node_to_bounding_box(graph, neighbor_node);
                    short_cut_target = !(rect_intersects(&bbox, approx[0], v3)
                        || rect_contains(&bbox, approx[0], v3));
                }
            }
        }
    }

    if short_cut_source && short_cut_target {
        graph.edge_mut(edge_id).bend_points.push(v2);
    }
    if !short_cut_source {
        graph.edge_mut(edge_id).bend_points.push(source_straight);
        graph.edge_mut(edge_id).bend_points.push(source_vertical);
    }
    if !short_cut_target {
        graph.edge_mut(edge_id).bend_points.push(target_vertical);
        graph.edge_mut(edge_id).bend_points.push(target_straight);
    }
}

/// Rough bezier sample at `t=0.5` used by sloppy routing to decide whether a
/// direct curve would overlap a neighbour node. Index `0` is the curve
/// center.
fn approximate_bezier_segment_2(v1: Vec2, v2: Vec2, v3: Vec2) -> [Vec2; 2] {
    let m1 = Vec2::new((v1.x + v2.x) / 2.0, (v1.y + v2.y) / 2.0);
    let m2 = Vec2::new((v2.x + v3.x) / 2.0, (v2.y + v3.y) / 2.0);
    let center = Vec2::new((m1.x + m2.x) / 2.0, (m1.y + m2.y) / 2.0);
    [center, v2]
}

#[derive(Debug, Clone, Copy)]
struct NodeBox {
    x: f64,
    y: f64,
    width: f64,
    height: f64,
}

fn node_to_bounding_box(graph: &LGraph, node_id: NodeId) -> NodeBox {
    let n = graph.node(node_id);
    NodeBox {
        x: n.position.x - n.margin.left,
        y: n.position.y - n.margin.top,
        width: n.size.x + n.margin.horizontal(),
        height: n.size.y + n.margin.vertical(),
    }
}

fn rect_contains(rect: &NodeBox, p1: Vec2, p2: Vec2) -> bool {
    let inside = |p: Vec2| {
        p.x >= rect.x && p.x <= rect.x + rect.width && p.y >= rect.y && p.y <= rect.y + rect.height
    };
    inside(p1) && inside(p2)
}

fn rect_intersects(rect: &NodeBox, p1: Vec2, p2: Vec2) -> bool {
    let x1 = rect.x;
    let y1 = rect.y;
    let x2 = rect.x + rect.width;
    let y2 = rect.y + rect.height;
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let mut t_min = 0.0_f64;
    let mut t_max = 1.0_f64;
    for (p, q) in [(-dx, p1.x - x1), (dx, x2 - p1.x), (-dy, p1.y - y1), (dy, y2 - p1.y)] {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                if t > t_max {
                    return false;
                }
                if t > t_min {
                    t_min = t;
                }
            } else if t < t_min {
                return false;
            } else if t < t_max {
                t_max = t;
            }
        }
    }
    t_min <= t_max
}

fn abs_anchor(graph: &LGraph, port_id: PortId) -> Vec2 {
    let port = graph.port(port_id);
    let node = graph.node(port.owner);
    Vec2::new(
        node.position.x + port.position.x + port.anchor.x,
        node.position.y + port.position.y + port.anchor.y,
    )
}

fn compute_sloppy_center_y(graph: &LGraph, edge_id: EdgeId, y_source: f64, y_target: f64) -> f64 {
    let e = graph.edge(edge_id);
    let target_node = graph.port(e.target).owner;
    let mut indegree: i32 = 0;
    for &port in graph.node(target_node).ports.iter() {
        indegree += graph.port(port).incoming_edges.len() as i32;
    }
    let source_node = graph.port(e.source).owner;
    let mut outdegree: i32 = 0;
    for &port in graph.node(source_node).ports.iter() {
        outdegree += graph.port(port).outgoing_edges.len() as i32;
    }
    let degree_diff = (outdegree - indegree).signum() as f64;
    ((y_target + y_source) / 2.0)
        + (y_target - y_source) * (SLOPPY_CENTER_CP_MULTIPLIER * degree_diff)
}

fn segment_allows_sloppy_routing(graph: &LGraph, seg: &SplineSegment) -> bool {
    let start_x = seg.bbox_x;
    let end_x = seg.bbox_x + seg.bbox_width;

    if seg.initial_segment
        && let Some(n) = seg.source_node
    {
        let t = segment_node_distance_threshold(graph, n);
        let node = graph.node(n);
        let distance = start_x - (node.position.x + node.size.x);
        if distance > t {
            return false;
        }
    }
    if seg.last_segment
        && let Some(n) = seg.target_node
    {
        let t = segment_node_distance_threshold(graph, n);
        let node = graph.node(n);
        let distance = node.position.x - end_x;
        if distance > t {
            return false;
        }
    }
    true
}

fn segment_node_distance_threshold(graph: &LGraph, node_id: NodeId) -> f64 {
    let node = graph.node(node_id);
    let layer = node.layer.unwrap_or(0);
    let layer_size_x = graph.layers.get(layer).map(|l| l.size.x).unwrap_or(0.0);
    layer_size_x - node.size.x / 2.0
}

fn calculate_bezier_bend_points(
    graph: &mut LGraph,
    edge_chain: &[EdgeId],
    surviving_edge: Option<EdgeId>,
    spline_mode: SplineRoutingMode,
) {
    if edge_chain.is_empty() {
        return;
    }
    let target_edge = surviving_edge.unwrap_or(edge_chain[0]);
    let source_port = graph.edge(target_edge).source;
    if !is_qualified_as_starting_node(graph, graph.port(source_port).owner) {
        panic!("The source node of the edge must be a normal node or a northSouthPort.");
    }

    let mut all_cp: Vec<Vec2> = Vec::new();
    all_cp.push(abs_anchor(graph, source_port));

    if matches!(graph.port(source_port).side, PortSide::North | PortSide::South) {
        let y = graph
            .port(source_port)
            .properties
            .get(&crate::properties::internal::SPLINE_NS_PORT_Y_COORD);
        let ns_cp = Vec2::new(abs_anchor(graph, source_port).x, y);
        all_cp.push(ns_cp);
    }

    let mut last_cp: Option<Vec2> = None;
    let mut add_mid_point = false;
    for &current_edge in edge_chain {
        let current_bps: Vec<Vec2> = graph.edge(current_edge).bend_points.clone();
        if !current_bps.is_empty() {
            if add_mid_point {
                if let (Some(last), Some(&first)) = (last_cp, current_bps.first()) {
                    let halfway =
                        Vec2::new((last.x + first.x) * ONE_HALF, (last.y + first.y) * ONE_HALF);
                    all_cp.push(halfway);
                }
                add_mid_point = false;
            } else {
                add_mid_point = true;
            }
            last_cp = current_bps.last().copied();
            all_cp.extend(current_bps);
            graph.edge_mut(current_edge).bend_points.clear();
        }
    }

    let target_port = graph.edge(target_edge).target;
    if matches!(graph.port(target_port).side, PortSide::North | PortSide::South) {
        let y = graph
            .port(target_port)
            .properties
            .get(&crate::properties::internal::SPLINE_NS_PORT_Y_COORD);
        let ns_cp = Vec2::new(abs_anchor(graph, target_port).x, y);
        all_cp.push(ns_cp);
    }
    all_cp.push(abs_anchor(graph, target_port));

    if spline_mode == SplineRoutingMode::Conservative {
        insert_straightening_control_points(graph, &mut all_cp, source_port, target_port);
    }

    let mut nub = NubSpline::new(true, SPLINE_DIMENSION, all_cp);
    let bezier_cp = nub.get_bezier_cp_default();
    graph.edge_mut(target_edge).bend_points.extend(bezier_cp);
}

fn insert_straightening_control_points(
    graph: &LGraph,
    all_cps: &mut Vec<Vec2>,
    src_port: PortId,
    tgt_port: PortId,
) {
    if all_cps.len() < 2 {
        return;
    }
    let first = all_cps[0];
    let second = all_cps[1];
    let dir = splines_math::port_side_to_direction(graph.port(src_port).side);
    let v = Vec2::new(
        dir.cos() * NODE_TO_STRAIGHTENING_CP_GAP,
        dir.sin() * NODE_TO_STRAIGHTENING_CP_GAP,
    );
    let v2 = Vec2::new(second.x - first.x, second.y - first.y);
    let straighten_begin = Vec2::new(first.x + abs_min(v.x, v2.x), first.y + abs_min(v.y, v2.y));
    all_cps.insert(1, straighten_begin);

    let last_idx = all_cps.len() - 1;
    let last = all_cps[last_idx];
    let second_last = all_cps[last_idx - 1];
    let dir_t = splines_math::port_side_to_direction(graph.port(tgt_port).side);
    let v = Vec2::new(
        dir_t.cos() * NODE_TO_STRAIGHTENING_CP_GAP,
        dir_t.sin() * NODE_TO_STRAIGHTENING_CP_GAP,
    );
    let v2 = Vec2::new(second_last.x - last.x, second_last.y - last.y);
    let straighten_end = Vec2::new(last.x + abs_min(v.x, v2.x), last.y + abs_min(v.y, v2.y));
    all_cps.insert(all_cps.len() - 1, straighten_end);
}

fn abs_min(a: f64, b: f64) -> f64 {
    if a.abs() < b.abs() { a } else { b }
}
