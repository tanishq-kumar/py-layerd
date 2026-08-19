//! Spline edge routing entry point.
//!
//! Preliminary segment routes are produced here; the final bezier bend points
//! are emitted by `core/src/intermediate/final_spline_bendpoints.rs` after long
//! edges have been joined back.

use crate::graph::LGraph;

pub mod math;
pub mod nub_spline;
pub mod router;
pub mod segment;

/// Route every edge as a spline: assigns per-layer horizontal coordinates,
/// builds `SplineSegment`s, and stores them on the graph so the final
/// bend-point calculator can turn them into bezier control points.
pub fn route_edges(graph: &mut LGraph) {
    router::route(graph);
}
