//! West-to-east routing direction.

use super::{
    super::hyper_edge_segment::HyperEdgeSegment, TOLERANCE, is_self_loop, port_absolute_anchor,
    push_bend,
};
use crate::{
    graph::{LGraph, index::PortId},
    math::Vec2,
};

pub(super) fn port_position_on_hyper_node(graph: &LGraph, port_id: PortId) -> f64 {
    port_absolute_anchor(graph, port_id).y
}

pub(super) fn calculate_bend_points(
    graph: &mut LGraph,
    segment: &HyperEdgeSegment,
    segments: &[HyperEdgeSegment],
    start_pos: f64,
    edge_spacing: f64,
) {
    if segment.is_dummy() {
        return;
    }

    let segment_x = start_pos + segment.routing_slot as f64 * edge_spacing;

    // Collect per-edge data first so the bend-point writes are not
    // aliasing against `graph.edge` reads inside the loop.
    struct EdgeRoute {
        edge_id: crate::graph::index::EdgeId,
        source_y: f64,
        target_y: f64,
        split_partner: Option<(f64, i32)>,
    }

    let split_partner = segment.split_partner.map(|partner_id| {
        let partner = &segments[partner_id.index()];
        let split_y = partner.incoming_connection_coordinates[0];
        (split_y, partner.routing_slot)
    });

    let mut routes: Vec<EdgeRoute> = Vec::with_capacity(segment.ports.len());
    for &port_id in &segment.ports {
        let port = graph.port(port_id);
        let source_y = port_absolute_anchor(graph, port_id).y;
        // Iterate **every** port in the segment and process its outgoing
        // edges. Filtering by `port.side == East` is wrong — a segment
        // legitimately includes:
        //   * source-east ports of the source layer (the common case),
        //   * target-west ports whose outgoing edges represent in-layer
        //     edges that loop within the target layer's west side
        //     (the case this routing iteration was built for in
        //     phantom iter 0; same shape recurs anywhere a west port
        //     happens to have an outgoing edge to another west port).
        // Skipping non-East ports drops bend points for that second
        // class of edges and leaves the restored long edges with
        // 2 bend points instead of 4.
        for pos in 0..port.outgoing_edges.len() {
            let edge_id = port.outgoing_edges[pos];
            if is_self_loop(graph, edge_id) {
                continue;
            }
            let target = graph.edge(edge_id).target;
            let target_y = port_absolute_anchor(graph, target).y;
            if (source_y - target_y).abs() <= TOLERANCE {
                continue;
            }
            routes.push(EdgeRoute { edge_id, source_y, target_y, split_partner });
        }
    }

    for route in routes {
        let mut current_x = segment_x;
        push_bend(graph, route.edge_id, Vec2::new(current_x, route.source_y));
        if let Some((split_y, partner_slot)) = route.split_partner {
            push_bend(graph, route.edge_id, Vec2::new(current_x, split_y));
            current_x = start_pos + partner_slot as f64 * edge_spacing;
            push_bend(graph, route.edge_id, Vec2::new(current_x, split_y));
        }
        push_bend(graph, route.edge_id, Vec2::new(current_x, route.target_y));
    }
}
