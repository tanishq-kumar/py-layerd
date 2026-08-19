//! Thin re-export front for the single-edge graph wrapper.
//!
//! The real implementation lives in `intermediate::wrapping::single_edge_graph_wrapper`.

use crate::graph::LGraph;

/// Runs the single-edge graph wrapper. Wraps path-like graphs for better
/// aspect ratios.
pub fn wrap(graph: &mut LGraph) {
    super::wrapping::wrap_single_edge_graph(graph);
}
