//! Self-loop routing.
//!
//! Entry point for the pipeline stage that materialises self-loop bend points
//! in node-local coordinates. Runs the routing director, slot assigner, and
//! the edge-routing-style dispatch (orthogonal, polyline, splines).

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    intermediate::{
        polyline_self_loop_router::cut_corners,
        self_hyper_loop_labels::SelfHyperLoopLabels,
        self_loop_holder::{
            SELF_LOOP_HOLDER, SelfHyperLoop, SelfLoopEdge, SelfLoopHolder, portside_index,
            portside_left, portside_right, portside_set_of,
        },
        self_loop_routing_director::determine_loop_routes,
        self_loop_routing_slot_assigner::assign_routing_slots,
    },
    math::{Margin, Vec2},
    options::enums::EdgeRoutingStrategy,
    p5_edge_routing::splines::nub_spline::{DIM as SPLINE_DIM, NubSpline},
};

/// Polyline corner cutting distance.
const POLYLINE_CORNER_DISTANCE: f64 = 10.0;

/// Half of a unit, used by the spline offset math.
const SPLINE_HALF: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EdgeRoutingDirection {
    Clockwise,
    CounterClockwise,
}

/// Pipeline entry point dispatched by `IntermediateProcessorId::SelfLoopRouter`.
pub fn route(graph: &mut LGraph) {
    let mut targets: Vec<NodeId> = Vec::new();
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            if graph.node(nid).node_type == NodeType::Normal
                && graph.node(nid).properties.has(&SELF_LOOP_HOLDER)
            {
                targets.push(nid);
            }
        }
    }
    if targets.is_empty() {
        return;
    }

    let edge_routing = graph.options.edge_routing;
    let edge_edge = graph.options.spacing.edge_edge;
    let edge_label = graph.options.spacing.edge_label;
    let node_self_loop = graph.options.spacing.node_self_loop;

    for node in targets {
        let Some(mut holder) = graph.node(node).properties.get(&SELF_LOOP_HOLDER) else {
            continue;
        };

        determine_loop_routes(graph, &mut holder, node);
        crate::intermediate::self_loop_label_placer::place_labels(graph, &mut holder, node);
        assign_routing_slots(graph, &mut holder, node);

        route_node(graph, node, &mut holder, edge_routing, edge_edge, edge_label, node_self_loop);

        graph.node_mut(node).properties.set(&SELF_LOOP_HOLDER, Some(holder));
    }
}

#[allow(clippy::too_many_arguments)]
fn route_node(
    graph: &mut LGraph,
    node: NodeId,
    holder: &mut SelfLoopHolder,
    edge_routing: EdgeRoutingStrategy,
    edge_edge: f64,
    edge_label: f64,
    node_self_loop: f64,
) {
    let node_size = graph.node(node).size;
    let node_margin = *graph.node(node).margin;

    let positions = compute_routing_slot_positions(
        holder,
        node_size,
        node_margin,
        edge_edge,
        edge_label,
        node_self_loop,
    );

    let mut new_margin = node_margin;

    // Collect work items so we can borrow `holder` immutably during the
    // routing math and write back once the loop is done.
    let mut bend_points_by_edge: Vec<(EdgeId, Vec<Vec2>)> = Vec::new();
    let mut label_anchors: Vec<(usize, Vec2)> = Vec::new();

    for hyper_idx in 0..holder.sl_hyper_loops.len() {
        let hyper = &holder.sl_hyper_loops[hyper_idx];
        for &edge_idx in &hyper.sl_edges {
            let sl_edge = &holder.sl_edges[edge_idx.0 as usize];
            let direction = compute_edge_routing_direction(graph, sl_edge, hyper);
            let bend_points =
                route_edge(graph, sl_edge, hyper, direction, &positions, edge_routing, edge_label);
            for bp in &bend_points {
                update_new_node_margin(node_size, &mut new_margin, *bp);
            }
            bend_points_by_edge.push((sl_edge.edge_id, bend_points));
        }

        if let Some(slabels) = &hyper.sl_labels
            && slabels.side.is_some()
        {
            let anchor = compute_label_anchor(graph, hyper, slabels, &positions, edge_label);
            let far_corner = Vec2::new(anchor.x + slabels.size.x, anchor.y + slabels.size.y);
            update_new_node_margin(node_size, &mut new_margin, anchor);
            update_new_node_margin(node_size, &mut new_margin, far_corner);
            label_anchors.push((hyper_idx, anchor));
        }
    }

    for (edge_id, bps) in bend_points_by_edge {
        graph.edge_mut(edge_id).bend_points = bps;
    }

    // Record the bounding-box anchor on the holder. Per-label text positions
    // are applied later by `self_loop::postprocess` via
    // `SelfHyperLoopLabels::apply_placement`.
    for (hyper_idx, anchor) in label_anchors {
        if let Some(slabels) = holder.sl_hyper_loops[hyper_idx].sl_labels.as_mut() {
            slabels.position = anchor;
        }
    }

    graph.node_mut(node).margin = new_margin.into();
}

/// Decide whether a given self-loop edge wraps clockwise or counter-clockwise
/// around the node.
fn compute_edge_routing_direction(
    graph: &LGraph,
    sl_edge: &SelfLoopEdge,
    hyper: &SelfHyperLoop,
) -> EdgeRoutingDirection {
    let source_side = graph.port(sl_edge.source).side;
    let target_side = graph.port(sl_edge.target).side;

    if source_side == target_side {
        // Same-side edges fall back to port id ordering.
        if graph.port(sl_edge.source).id < graph.port(sl_edge.target).id {
            EdgeRoutingDirection::Clockwise
        } else {
            EdgeRoutingDirection::CounterClockwise
        }
    } else if portside_right(source_side) == target_side {
        EdgeRoutingDirection::Clockwise
    } else if portside_left(source_side) == target_side {
        EdgeRoutingDirection::CounterClockwise
    } else {
        // Opposing sides: pick the direction the hyper-loop already occupies.
        // Prefer clockwise when both sides are covered.
        let right = portside_right(source_side);
        if hyper.occupied_port_sides.contains(portside_set_of(right)) {
            EdgeRoutingDirection::Clockwise
        } else {
            EdgeRoutingDirection::CounterClockwise
        }
    }
}

fn route_edge(
    graph: &LGraph,
    sl_edge: &SelfLoopEdge,
    hyper: &SelfHyperLoop,
    direction: EdgeRoutingDirection,
    positions: &SlotPositions,
    edge_routing: EdgeRoutingStrategy,
    edge_label: f64,
) -> Vec<Vec2> {
    let orth = compute_orthogonal_bend_points(graph, sl_edge, hyper, direction, positions);
    match edge_routing {
        EdgeRoutingStrategy::Polyline => polyline_modify(graph, sl_edge, &orth),
        EdgeRoutingStrategy::Splines => spline_modify(graph, sl_edge, direction, &orth, edge_label),
        _ => orth,
    }
}

fn polyline_modify(graph: &LGraph, sl_edge: &SelfLoopEdge, orth: &[Vec2]) -> Vec<Vec2> {
    let mut chain = Vec::with_capacity(orth.len() + 2);
    chain.push(port_anchor(graph, sl_edge.source));
    chain.extend_from_slice(orth);
    chain.push(port_anchor(graph, sl_edge.target));
    if chain.len() <= 2 {
        return orth.to_vec();
    }
    cut_corners(&chain, POLYLINE_CORNER_DISTANCE)
}

/// Convert the orthogonal self-loop bend chain into Bezier control points.
/// Includes source/target anchors while constructing the NUB spline, then
/// returns the Bezier control polygon without those terminal anchor vectors.
fn spline_modify(
    graph: &LGraph,
    sl_edge: &SelfLoopEdge,
    direction: EdgeRoutingDirection,
    orth: &[Vec2],
    edge_label: f64,
) -> Vec<Vec2> {
    if orth.len() < 2 {
        return orth.to_vec();
    }
    let source_anchor = port_anchor(graph, sl_edge.source);
    let target_anchor = port_anchor(graph, sl_edge.target);

    let mut spline_points = Vec::with_capacity(orth.len() * 2 + 2);
    spline_points.push(source_anchor);

    let mut curr_side = graph.port(sl_edge.source).side;
    let mut first_bp = orth[0];
    for second_bp in orth.iter().skip(1).copied() {
        spline_points.push(first_bp);

        let mid = Vec2::new(
            (first_bp.x + second_bp.x) * SPLINE_HALF,
            (first_bp.y + second_bp.y) * SPLINE_HALF,
        );
        let direction_offset = portside_outward(curr_side);
        spline_points.push(Vec2::new(
            mid.x + direction_offset.x * edge_label,
            mid.y + direction_offset.y * edge_label,
        ));

        first_bp = second_bp;
        curr_side = match direction {
            EdgeRoutingDirection::Clockwise => portside_right(curr_side),
            EdgeRoutingDirection::CounterClockwise => portside_left(curr_side),
        };
    }

    spline_points.push(*orth.last().expect("orth len checked"));
    spline_points.push(target_anchor);

    NubSpline::new(true, SPLINE_DIM, spline_points).get_bezier_cp_default()
}

fn portside_outward(side: PortSide) -> Vec2 {
    match side {
        PortSide::North => Vec2::new(0.0, -1.0),
        PortSide::East => Vec2::new(1.0, 0.0),
        PortSide::South => Vec2::new(0.0, 1.0),
        PortSide::West => Vec2::new(-1.0, 0.0),
        PortSide::Undefined => Vec2::ZERO,
    }
}

fn port_anchor(graph: &LGraph, port: PortId) -> Vec2 {
    let p = graph.port(port);
    Vec2::new(p.position.x + p.anchor.x, p.position.y + p.anchor.y)
}

/// Full orthogonal bend-point chain: outer source, corners, outer target.
fn compute_orthogonal_bend_points(
    graph: &LGraph,
    sl_edge: &SelfLoopEdge,
    hyper: &SelfHyperLoop,
    direction: EdgeRoutingDirection,
    positions: &SlotPositions,
) -> Vec<Vec2> {
    let mut bps: Vec<Vec2> = Vec::with_capacity(4);
    bps.push(outer_bend_point(graph, sl_edge.source, hyper, positions));
    add_corner_bend_points(graph, sl_edge, hyper, direction, positions, &mut bps);
    bps.push(outer_bend_point(graph, sl_edge.target, hyper, positions));
    bps
}

fn outer_bend_point(
    graph: &LGraph,
    port: PortId,
    hyper: &SelfHyperLoop,
    positions: &SlotPositions,
) -> Vec2 {
    let side = graph.port(port).side;
    let slot = hyper.routing_slot_for(side) as usize;
    let level = positions.position(side, slot);
    let anchor = port_anchor(graph, port);
    match side {
        PortSide::North | PortSide::South => Vec2::new(anchor.x, level),
        PortSide::East | PortSide::West => Vec2::new(level, anchor.y),
        PortSide::Undefined => Vec2::new(anchor.x, anchor.y),
    }
}

fn add_corner_bend_points(
    graph: &LGraph,
    sl_edge: &SelfLoopEdge,
    hyper: &SelfHyperLoop,
    direction: EdgeRoutingDirection,
    positions: &SlotPositions,
    out: &mut Vec<Vec2>,
) {
    let source_side = graph.port(sl_edge.source).side;
    let target_side = graph.port(sl_edge.target).side;
    if source_side == target_side
        || source_side == PortSide::Undefined
        || target_side == PortSide::Undefined
    {
        return;
    }

    let inline_side = if sl_edge.is_inline(graph) {
        hyper.sl_labels.as_ref().and_then(|l| l.side)
    } else {
        None
    };
    let inline_size = inline_side.and_then(|_| hyper.sl_labels.as_ref().map(|l| l.size));

    let mut curr = source_side;
    for _ in 0..4 {
        if curr == target_side {
            break;
        }
        let next = match direction {
            EdgeRoutingDirection::Clockwise => portside_right(curr),
            EdgeRoutingDirection::CounterClockwise => portside_left(curr),
        };
        if next == PortSide::Undefined {
            break;
        }

        let mut curr_component =
            side_to_vector(curr, positions.position(curr, hyper.routing_slot_for(curr) as usize));
        let mut next_component =
            side_to_vector(next, positions.position(next, hyper.routing_slot_for(next) as usize));

        if let (Some(label_side), Some(label_size)) = (inline_side, inline_size) {
            if curr == label_side {
                adjust_for_label(&mut curr_component, label_side, label_size);
            } else if next == label_side {
                adjust_for_label(&mut next_component, label_side, label_size);
            }
        }

        out.push(Vec2::new(
            curr_component.x + next_component.x,
            curr_component.y + next_component.y,
        ));

        curr = next;
    }
}

fn side_to_vector(side: PortSide, level: f64) -> Vec2 {
    match side {
        PortSide::North | PortSide::South => Vec2::new(0.0, level),
        PortSide::East | PortSide::West => Vec2::new(level, 0.0),
        PortSide::Undefined => Vec2::ZERO,
    }
}

fn adjust_for_label(component: &mut Vec2, side: PortSide, size: Vec2) {
    match side {
        PortSide::North => component.y -= size.y / 2.0,
        PortSide::South => component.y += size.y / 2.0,
        PortSide::West => component.x -= size.x / 2.0,
        PortSide::East => component.x += size.x / 2.0,
        PortSide::Undefined => {}
    }
}

fn update_new_node_margin(node_size: Vec2, margin: &mut Margin, bp: Vec2) {
    margin.left = margin.left.max(-bp.x);
    margin.right = margin.right.max(bp.x - node_size.x);
    margin.top = margin.top.max(-bp.y);
    margin.bottom = margin.bottom.max(bp.y - node_size.y);
}

/// Per-side table of slot-to-level distances. Index the side with
/// `portside_index` and the slot with the value returned by
/// `SelfHyperLoop::routing_slot_for`.
struct SlotPositions {
    data: [Vec<f64>; 4],
}

impl SlotPositions {
    fn new(sizes: [usize; 4]) -> Self {
        SlotPositions {
            data: [
                vec![0.0; sizes[0].max(1)],
                vec![0.0; sizes[1].max(1)],
                vec![0.0; sizes[2].max(1)],
                vec![0.0; sizes[3].max(1)],
            ],
        }
    }

    fn position(&self, side: PortSide, slot: usize) -> f64 {
        let idx = portside_index(side);
        if idx >= 4 {
            return 0.0;
        }
        let bucket = &self.data[idx];
        if bucket.is_empty() {
            return 0.0;
        }
        bucket[slot.min(bucket.len() - 1)]
    }

    fn set(&mut self, side: PortSide, slot: usize, value: f64) {
        let idx = portside_index(side);
        if idx >= 4 {
            return;
        }
        if slot < self.data[idx].len() {
            self.data[idx][slot] = value;
        }
    }

    fn init_label(&mut self, side: PortSide, slot: usize, label_size: f64) {
        let idx = portside_index(side);
        if idx >= 4 {
            return;
        }
        if slot < self.data[idx].len() {
            self.data[idx][slot] = self.data[idx][slot].max(label_size);
        }
    }
}

/// Compute per-side slot levels. North/south slots reserve space for the
/// tallest label assigned to them; east/west sides stack a constant
/// edge-to-edge distance.
#[allow(clippy::too_many_arguments)]
fn compute_routing_slot_positions(
    holder: &SelfLoopHolder,
    node_size: Vec2,
    margin: Margin,
    edge_edge: f64,
    edge_label: f64,
    node_self_loop: f64,
) -> SlotPositions {
    let mut sizes = [0_usize; 4];
    for (i, &c) in holder.routing_slot_count.iter().enumerate() {
        sizes[i] = (c as usize).max(1);
    }
    let mut positions = SlotPositions::new(sizes);

    // Pre-fill north/south label heights per slot.
    for side in [PortSide::North, PortSide::South] {
        for hyper in &holder.sl_hyper_loops {
            if let Some(labels) = &hyper.sl_labels
                && labels.side == Some(side)
            {
                let slot = hyper.routing_slot_for(side) as usize;
                positions.init_label(side, slot, labels.size.y);
            }
        }
    }

    compute_side_positions(
        &mut positions,
        PortSide::North,
        node_size,
        margin,
        edge_edge,
        edge_label,
        node_self_loop,
    );
    compute_side_positions(
        &mut positions,
        PortSide::East,
        node_size,
        margin,
        edge_edge,
        edge_label,
        node_self_loop,
    );
    compute_side_positions(
        &mut positions,
        PortSide::South,
        node_size,
        margin,
        edge_edge,
        edge_label,
        node_self_loop,
    );
    compute_side_positions(
        &mut positions,
        PortSide::West,
        node_size,
        margin,
        edge_edge,
        edge_label,
        node_self_loop,
    );

    positions
}

#[allow(clippy::too_many_arguments)]
fn compute_side_positions(
    positions: &mut SlotPositions,
    side: PortSide,
    node_size: Vec2,
    margin: Margin,
    edge_edge: f64,
    edge_label: f64,
    node_self_loop: f64,
) {
    let mut curr = match side {
        PortSide::North => -margin.top - node_self_loop,
        PortSide::East => node_size.x + margin.right + node_self_loop,
        PortSide::South => node_size.y + margin.bottom + node_self_loop,
        PortSide::West => -margin.left - node_self_loop,
        PortSide::Undefined => 0.0,
    };
    let factor = match side {
        PortSide::North | PortSide::West => -1.0,
        _ => 1.0,
    };

    let idx = portside_index(side);
    if idx >= 4 {
        return;
    }
    let count = positions.data[idx].len();
    for slot in 0..count {
        let label_size = positions.data[idx][slot];
        let effective_label = if label_size > 0.0 { label_size + edge_label } else { 0.0 };
        positions.set(side, slot, curr);
        curr += factor * (effective_label + edge_edge);
    }
}

/// Compute the bounding-box anchor for a hyper-loop's labels based on the
/// assigned side, the loop's per-side routing slot, and the edge-label
/// distance.
fn compute_label_anchor(
    graph: &LGraph,
    hyper: &SelfHyperLoop,
    slabels: &SelfHyperLoopLabels,
    positions: &SlotPositions,
    edge_label: f64,
) -> Vec2 {
    let label_side = slabels.side.unwrap_or(PortSide::North);
    let slot = hyper.routing_slot_for(label_side) as usize;
    let baseline = positions.position(label_side, slot);

    let inline = slabels.labels.iter().any(|&lid| {
        graph
            .label(lid)
            .properties
            .get(&crate::properties::internal::EDGE_LABELS_INLINE)
    });
    let edge_label_distance = if inline { 0.0 } else { edge_label };

    match label_side {
        PortSide::North =>
            Vec2::new(slabels.position.x, baseline - edge_label_distance - slabels.size.y),
        PortSide::South => Vec2::new(slabels.position.x, baseline + edge_label_distance),
        PortSide::West =>
            Vec2::new(baseline - edge_label_distance - slabels.size.x, slabels.position.y),
        PortSide::East => Vec2::new(baseline + edge_label_distance, slabels.position.y),
        PortSide::Undefined => slabels.position,
    }
}
