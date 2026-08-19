//! South-to-north routing direction.
//!
//! Edges travel upward from southern external ports into the graph. The
//! routing axis remains horizontal; only the source/target port sides swap.

use super::{
    super::hyper_edge_segment::HyperEdgeSegment, TOLERANCE, is_self_loop, port_absolute_anchor,
    push_bend,
};
use crate::{graph::LGraph, math::Vec2};

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

    let segment_y = start_pos - segment.routing_slot as f64 * edge_spacing;

    struct EdgeRoute {
        edge_id: crate::graph::index::EdgeId,
        source_x: f64,
        target_x: f64,
        split_partner: Option<(f64, i32)>,
    }

    let split_partner = segment.split_partner.map(|partner_id| {
        let partner = &segments[partner_id.index()];
        let split_x = partner.incoming_connection_coordinates[0];
        (split_x, partner.routing_slot)
    });

    let mut routes: Vec<EdgeRoute> = Vec::with_capacity(segment.ports.len());
    for &port_id in &segment.ports {
        let port = graph.port(port_id);
        // Process every port's outgoing edges, not just north-side
        // sources. See west_to_east.rs for the symmetric reasoning.
        let source_x = port_absolute_anchor(graph, port_id).x;
        for pos in 0..port.outgoing_edges.len() {
            let edge_id = port.outgoing_edges[pos];
            if is_self_loop(graph, edge_id) {
                continue;
            }
            let target = graph.edge(edge_id).target;
            let target_x = port_absolute_anchor(graph, target).x;
            if (source_x - target_x).abs() <= TOLERANCE {
                continue;
            }
            routes.push(EdgeRoute { edge_id, source_x, target_x, split_partner });
        }
    }

    for route in routes {
        let mut current_y = segment_y;
        push_bend(graph, route.edge_id, Vec2::new(route.source_x, current_y));
        if let Some((split_x, partner_slot)) = route.split_partner {
            push_bend(graph, route.edge_id, Vec2::new(split_x, current_y));
            current_y = start_pos - partner_slot as f64 * edge_spacing;
            push_bend(graph, route.edge_id, Vec2::new(split_x, current_y));
        }
        push_bend(graph, route.edge_id, Vec2::new(route.target_x, current_y));
    }
}
