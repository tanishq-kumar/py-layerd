//! Orthogonal edge routing entry point.
//!
//! Submodules implement the hyperedge-based routing pipeline. This module is
//! a thin shim that exposes the public `route_edges` function consumed by the
//! pipeline, delegating to `routing_generator`.

use crate::graph::LGraph;

pub mod cycle_detector;
pub mod direction;
pub mod hyper_edge_dependency;
pub mod hyper_edge_segment;
pub mod routing_generator;
pub mod segment_splitter;

/// Routes every edge with 90-degree bend points and assigns layer positions.
///
/// The hyperedge dependency graph decides which routing slot each trunk ends
/// up in; the direction strategy turns the slot into concrete bend points.
/// Consumes the graph's RNG so that cycle-breaking randomness remains
/// deterministic across the rest of the layout pipeline.
pub fn route_edges(graph: &mut LGraph) {
    let _ = routing_generator::route_edges(graph);
}
