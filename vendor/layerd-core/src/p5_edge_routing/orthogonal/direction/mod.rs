//! Routing directions for the orthogonal router.
//!
//! The router supports west-to-east, north-to-south, and south-to-north
//! routing. The direction decides which port sides feed a layer pair, how
//! to project port coordinates onto the hyper-edge axis, and how to emit
//! bend points once routing slots are assigned.

use super::hyper_edge_segment::HyperEdgeSegment;
use crate::{
    graph::{
        LGraph,
        index::{EdgeId, PortId},
        port::PortSide,
    },
    math::Vec2,
};

pub mod north_to_south;
pub mod south_to_north;
pub mod west_to_east;

/// Direction of the routing pass.
///
/// The router does not include an east-to-west direction because the
/// generator is always invoked left to right on the canonical layer order;
/// east-to-west routing is emulated by reversing the layer iteration, not
/// by an independent strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoutingDirection {
    WestToEast,
    NorthToSouth,
    SouthToNorth,
}

impl RoutingDirection {
    /// Returns the coordinate of a port along the hyperedge's routing axis.
    pub(super) fn port_position_on_hyper_node(self, graph: &LGraph, port_id: PortId) -> f64 {
        match self {
            RoutingDirection::WestToEast =>
                west_to_east::port_position_on_hyper_node(graph, port_id),
            RoutingDirection::NorthToSouth | RoutingDirection::SouthToNorth =>
                north_to_south::port_position_on_hyper_node(graph, port_id),
        }
    }

    /// Returns the side of ports to consider on the source layer of a pair.
    pub(super) fn source_port_side(self) -> PortSide {
        match self {
            RoutingDirection::WestToEast => PortSide::East,
            RoutingDirection::NorthToSouth => PortSide::South,
            RoutingDirection::SouthToNorth => PortSide::North,
        }
    }

    /// Returns the side of ports to consider on the target layer of a pair.
    pub(super) fn target_port_side(self) -> PortSide {
        match self {
            RoutingDirection::WestToEast => PortSide::West,
            RoutingDirection::NorthToSouth => PortSide::North,
            RoutingDirection::SouthToNorth => PortSide::South,
        }
    }

    /// Writes bend points onto every edge incident to the given hyperedge
    /// segment, using `start_pos` as the origin of the layer-gap's routing
    /// channel and `edge_spacing` as the distance between adjacent slots.
    pub(super) fn calculate_bend_points(
        self,
        graph: &mut LGraph,
        segment: &HyperEdgeSegment,
        segments: &[HyperEdgeSegment],
        start_pos: f64,
        edge_spacing: f64,
    ) {
        match self {
            RoutingDirection::WestToEast => west_to_east::calculate_bend_points(
                graph,
                segment,
                segments,
                start_pos,
                edge_spacing,
            ),
            RoutingDirection::NorthToSouth => north_to_south::calculate_bend_points(
                graph,
                segment,
                segments,
                start_pos,
                edge_spacing,
            ),
            RoutingDirection::SouthToNorth => south_to_north::calculate_bend_points(
                graph,
                segment,
                segments,
                start_pos,
                edge_spacing,
            ),
        }
    }
}

/// Computes the absolute anchor position of a port on the graph.
///
/// Returns node origin plus port offset plus the port anchor offset. All
/// four coordinates are used by the bend-point calculation regardless of
/// direction — each strategy projects onto the relevant axis.
pub(crate) fn port_absolute_anchor(graph: &LGraph, port_id: PortId) -> Vec2 {
    let port = graph.port(port_id);
    let node = graph.node(port.owner);
    Vec2::new(
        node.position.x + port.position.x + port.anchor.x,
        node.position.y + port.position.y + port.anchor.y,
    )
}

/// Pushes a bend point onto an edge's bend-point list.
pub(crate) fn push_bend(graph: &mut LGraph, edge_id: EdgeId, bend: Vec2) {
    graph.edge_mut(edge_id).bend_points.push(bend);
}

/// Below-tolerance equality threshold for floating-point port positions.
pub(crate) const TOLERANCE: f64 = 1.0e-4;

/// Returns whether the edge is a self-loop.
pub(crate) fn is_self_loop(graph: &LGraph, edge_id: EdgeId) -> bool {
    let edge = graph.edge(edge_id);
    graph.port(edge.source).owner == graph.port(edge.target).owner
}
