//! Polyline edge router.
//!
//! Places every layer at a horizontal offset that reserves enough room for
//! the vertical span of the edges running between adjacent layers. Then
//! introduces bend points where a port's absolute anchor differs from the
//! layer boundary by more than the configured sloped-edge tolerance, and
//! adds junction points when multiple edges share the same port.

use hashbrown::HashSet;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    properties::internal::{EXT_PORT_SIDE, JUNCTION_POINTS, ORIGIN_PORT},
};

/// Minimum vertical difference between an edge's endpoint and a candidate
/// bend point before the bend point is actually inserted.
const MIN_VERT_DIFF: f64 = 1.0;
/// Factor applied to the maximum vertical edge span when computing how much
/// horizontal spacing to allocate between two adjacent layers.
const LAYER_SPACE_FAC: f64 = 0.4;

/// Route all edges as polyline segments and assign layer x-coordinates.
pub fn route_edges(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    let node_spacing = graph.options.spacing.node_node_between_layers;
    let edge_spacing = graph.options.spacing.edge_edge_between_layers;
    let sloped_edge_zone_width = graph.options.edge_routing_polyline_sloped_edge_zone_width;
    let edge_space_fac = (edge_spacing / node_spacing).min(1.0);

    // The polyline router does not redistribute port positions; earlier
    // processors (`LabelAndNodeSizeProcessor` for Normal nodes,
    // `LongEdgeSplitter` for long-edge dummies, etc.) are authoritative.
    // An earlier fallback overwrote LongEdge dummy ports with
    // `node_size.y / 2.0` (= 0.5 for thickness=1) instead of the
    // `floor(thickness / 2)` assigned by the splitter, producing 0.5 px
    // bend-point drift.

    // Reserve enough room to route the first layer's west-side in-layer
    // edges.
    let mut xpos = {
        let y_diff = calculate_west_in_layer_edge_y_diff(graph, 0);
        LAYER_SPACE_FAC * edge_space_fac * y_diff
    };

    let mut created_junction_points: HashSet<(i64, i64)> = HashSet::new();

    for layer_idx in 0..graph.layers.len() {
        let external_layer = is_external_west_or_east_port_layer(graph, layer_idx);

        // Strip the extra spacing for a pure external port layer sitting at
        // the rightmost slot.
        if external_layer && xpos > 0.0 {
            xpos -= node_spacing;
        }

        // Place nodes horizontally at `xpos`, shifting each node within the
        // layer by its port-ratio.
        crate::p5_edge_routing::place_nodes_horizontally(graph, layer_idx, xpos);
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        // For each node, process in-layer edges (which adds mid-edge bend
        // points) and regular outgoing edges (which contribute to
        // `maxVertDiff` for the inter-layer spacing).
        let mut max_vert_diff = 0.0_f64;
        for &node_id in &node_ids {
            let mut max_curr_output_y_diff = 0.0_f64;
            let outgoing_edges: Vec<EdgeId> = graph.outgoing_edges(node_id).collect::<Vec<_>>();
            for eid in outgoing_edges {
                let src_port = graph.edge(eid).source;
                let tgt_port = graph.edge(eid).target;
                let src_owner = graph.port(src_port).owner;
                let tgt_owner = graph.port(tgt_port).owner;
                if src_owner == tgt_owner {
                    // Self-loop: skip polyline routing (dedicated self-loop
                    // subsystem handles these).
                    continue;
                }
                let mut source_pos = absolute_port_y(graph, src_port);
                let mut target_pos = absolute_port_y(graph, tgt_port);

                let src_layer = graph.node(src_owner).layer;
                let tgt_layer = graph.node(tgt_owner).layer;
                if src_layer == tgt_layer {
                    // In-layer edge gets an extra mid bend point.
                    let abs_diff = (source_pos - target_pos).abs();
                    let spacing = LAYER_SPACE_FAC * edge_space_fac * abs_diff;
                    process_in_layer_edge(graph, eid, xpos, spacing);
                    if graph.port(src_port).side == PortSide::West {
                        // West-side in-layer edges already had their spacing
                        // reserved earlier (see `xpos` init / the pre-fetch
                        // block below). Zeroing the span here prevents
                        // double-counting into `maxVertDiff`.
                        source_pos = 0.0;
                        target_pos = 0.0;
                    }
                }

                max_curr_output_y_diff =
                    max_curr_output_y_diff.max((target_pos - source_pos).abs());
            }

            // Only certain node types participate in the per-node bend-point
            // insertion pass.
            let nt = graph.node(node_id).node_type;
            if matches!(
                nt,
                NodeType::Normal
                    | NodeType::Label
                    | NodeType::LongEdge
                    | NodeType::NorthSouthPort
                    | NodeType::BreakingPoint
            ) {
                process_node(
                    graph,
                    node_id,
                    xpos,
                    sloped_edge_zone_width,
                    &mut created_junction_points,
                );
            }

            max_vert_diff = max_vert_diff.max(max_curr_output_y_diff);
        }

        // Factor in the next layer's west in-layer edge span before
        // computing the inter-layer gap.
        if layer_idx + 1 < graph.layers.len() {
            let next_y_diff = calculate_west_in_layer_edge_y_diff(graph, layer_idx + 1);
            max_vert_diff = max_vert_diff.max(next_y_diff);
        }

        let mut layer_spacing = LAYER_SPACE_FAC * edge_space_fac * max_vert_diff;
        if !external_layer && layer_idx + 1 < graph.layers.len() {
            layer_spacing += node_spacing;
        }

        xpos += graph.layers[layer_idx].size.x + layer_spacing;
    }

    graph.size.x = xpos;
}

/// For an in-layer edge, add a single mid bend point just outside the
/// layer's boundary in the direction of the port side.
fn process_in_layer_edge(graph: &mut LGraph, eid: EdgeId, layer_x_pos: f64, edge_spacing: f64) {
    let src_port = graph.edge(eid).source;
    let tgt_port = graph.edge(eid).target;
    let source_y = absolute_port_y(graph, src_port);
    let target_y = absolute_port_y(graph, tgt_port);
    let mid_y = (source_y + target_y) / 2.0;

    let src_owner = graph.port(src_port).owner;
    let src_layer_idx = graph.node(src_owner).layer.unwrap_or(0);
    let layer_width = graph.layers[src_layer_idx].size.x;
    let bend = if graph.port(src_port).side == PortSide::East {
        Vec2::new(layer_x_pos + layer_width + edge_spacing, mid_y)
    } else {
        Vec2::new(layer_x_pos - edge_spacing, mid_y)
    };

    // Insert at head of bend_points so the bend appears before any existing
    // mid-route points emitted by earlier processors.
    graph.edge_mut(eid).bend_points.insert(0, bend);
}

/// Each eligible port picks a candidate bend point at the layer boundary;
/// if the port's anchor is too far from that boundary we insert the bend
/// point and, optionally, a junction point if the port has more than one
/// connected edge.
fn process_node(
    graph: &mut LGraph,
    node_id: NodeId,
    layer_left_x_pos: f64,
    sloped_edge_zone_width: f64,
    created_junction_points: &mut HashSet<(i64, i64)>,
) {
    let node_type = graph.node(node_id).node_type;
    let layer_idx = match graph.node(node_id).layer.get() {
        Some(l) => l,
        None => return,
    };
    let layer_width = graph.layers[layer_idx].size.x;
    let layer_right_x_pos = layer_left_x_pos + layer_width;
    let port_ids: Vec<PortId> = graph.node(node_id).ports.iter().copied().collect();

    for pid in port_ids {
        let port_side = graph.port(pid).side;
        let (bend_x, port_y) = {
            let mut anchor = absolute_port_anchor(graph, pid);

            if node_type == NodeType::NorthSouthPort {
                // The N/S dummy's x must be `actualNode.position.x +
                // originPort.position.x` — **without** the port anchor.x
                // term. `absolute_port_anchor` returns nodePos + portPos +
                // anchor, so strip the anchor contribution here.
                if let Some(origin_port_id) = graph.port(pid).properties.get(&ORIGIN_PORT) {
                    let origin_port = graph.port(origin_port_id);
                    let origin_node_pos = graph.node(origin_port.owner).position;
                    anchor.x = origin_node_pos.x + origin_port.position.x;
                    graph.node_mut(node_id).position.x = anchor.x;
                }
            }

            let bend_x = match port_side {
                PortSide::East => layer_right_x_pos,
                PortSide::West => layer_left_x_pos,
                // Only east/west ports participate in this pass.
                _ => continue,
            };
            (bend_x, anchor.y)
        };

        let x_distance = (absolute_port_anchor(graph, pid).x - bend_x).abs();
        let in_layer_dummy = is_in_layer_dummy(graph, node_id);
        // Skip bend insertion if the port already sits close enough to the
        // layer boundary, unless the node is an in-layer dummy.
        if x_distance <= sloped_edge_zone_width && !in_layer_dummy {
            continue;
        }

        let connected_count =
            graph.port(pid).outgoing_edges.len() + graph.port(pid).incoming_edges.len();
        let add_junction_point = connected_count > 1;

        let outgoing: Vec<EdgeId> = graph.port(pid).outgoing_edges.iter().copied().collect();
        let incoming: Vec<EdgeId> = graph.port(pid).incoming_edges.iter().copied().collect();
        for eid in outgoing.into_iter().chain(incoming) {
            let other_port = {
                let e = graph.edge(eid);
                if e.source == pid { e.target } else { e.source }
            };
            let other_y = absolute_port_y(graph, other_port);
            if (other_y - port_y).abs() > MIN_VERT_DIFF {
                let bend = Vec2::new(bend_x, port_y);
                add_bend_point(graph, eid, bend, add_junction_point, pid, created_junction_points);
            }
        }
    }
}

/// Prepend or append a bend point depending on whether the port is the
/// source or the target. Optionally record a junction point if this is the
/// first edge at this coordinate.
fn add_bend_point(
    graph: &mut LGraph,
    eid: EdgeId,
    bend: Vec2,
    add_junction_point: bool,
    curr_port: PortId,
    created_junction_points: &mut HashSet<(i64, i64)>,
) {
    let (e_src, e_tgt) = {
        let e = graph.edge(eid);
        (e.source, e.target)
    };
    let src_owner = graph.port(e_src).owner;
    let tgt_owner = graph.port(e_tgt).owner;
    let is_self_loop = src_owner == tgt_owner;
    let is_in_layer = graph.node(src_owner).layer == graph.node(tgt_owner).layer;

    if is_self_loop {
        return;
    }

    let curr_anchor = absolute_port_anchor(graph, curr_port);
    let equals_anchor = (curr_anchor.x - bend.x).abs() < f64::EPSILON
        && (curr_anchor.y - bend.y).abs() < f64::EPSILON;

    if is_in_layer || !equals_anchor {
        if e_src == curr_port {
            graph.edge_mut(eid).bend_points.insert(0, bend);
        } else {
            graph.edge_mut(eid).bend_points.push(bend);
        }

        if add_junction_point {
            let key = quantize(bend);
            if !created_junction_points.contains(&key) {
                let mut jps = graph.edge(eid).properties.get(&JUNCTION_POINTS).clone();
                jps.push(bend);
                graph.edge_mut(eid).properties.set(&JUNCTION_POINTS, jps);
                created_junction_points.insert(key);
            }
        }
    }
}

/// Quantize a `Vec2` to a fixed integer grid so the junction-point de-dup
/// set tolerates the usual floating-point wiggle around exact layer
/// coordinates.
fn quantize(v: Vec2) -> (i64, i64) {
    let scale = 1_000.0;
    ((v.x * scale).round() as i64, (v.y * scale).round() as i64)
}

/// Maximum y-span of the west-side in-layer edges in the given layer. Used
/// to size the horizontal slot reserved before the layer.
fn calculate_west_in_layer_edge_y_diff(graph: &LGraph, layer_idx: usize) -> f64 {
    let mut max_y_diff = 0.0_f64;
    for &nid in &graph.layers[layer_idx].nodes {
        for eid in graph.outgoing_edges(nid) {
            let src_port = graph.edge(eid).source;
            let tgt_port = graph.edge(eid).target;
            let tgt_owner = graph.port(tgt_port).owner;
            if graph.node(tgt_owner).layer != Some(layer_idx) {
                continue;
            }
            if graph.port(src_port).side != PortSide::West {
                continue;
            }
            let source_y = absolute_port_y(graph, src_port);
            let target_y = absolute_port_y(graph, tgt_port);
            max_y_diff = max_y_diff.max((target_y - source_y).abs());
        }
    }
    max_y_diff
}

/// Returns true only if every node in the layer is an external-port dummy
/// whose `EXT_PORT_SIDE` is West or East.
fn is_external_west_or_east_port_layer(graph: &LGraph, layer_idx: usize) -> bool {
    let nodes = &graph.layers[layer_idx].nodes;
    if nodes.is_empty() {
        return false;
    }
    for &nid in nodes {
        if graph.node(nid).node_type != NodeType::ExternalPort {
            return false;
        }
        let side = graph.node(nid).properties.get(&EXT_PORT_SIDE);
        if !matches!(side, PortSide::West | PortSide::East) {
            return false;
        }
    }
    true
}

/// Returns true for a `LongEdge` dummy with any in-layer incident edge.
fn is_in_layer_dummy(graph: &LGraph, node_id: NodeId) -> bool {
    if graph.node(node_id).node_type != NodeType::LongEdge {
        return false;
    }
    let node_layer = graph.node(node_id).layer;
    for eid in graph.outgoing_edges(node_id).chain(graph.incoming_edges(node_id)) {
        let src = graph.port(graph.edge(eid).source).owner;
        let tgt = graph.port(graph.edge(eid).target).owner;
        let other = if src == node_id { tgt } else { src };
        if graph.node(other).layer == node_layer {
            return true;
        }
    }
    false
}

fn absolute_port_y(graph: &LGraph, port_id: PortId) -> f64 {
    absolute_port_anchor(graph, port_id).y
}

fn absolute_port_anchor(graph: &LGraph, port_id: PortId) -> Vec2 {
    let port = graph.port(port_id);
    let node = graph.node(port.owner);
    Vec2::new(
        node.position.x + port.position.x + port.anchor.x,
        node.position.y + port.position.y + port.anchor.y,
    )
}
