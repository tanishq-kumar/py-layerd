//! Read-only view of graph metadata used during layer sweep.
//!
//! Per-LGraph crossing-minimization state holder.

use crate::{graph::LGraph, p3_crossing_min::layer_sweep_type_decider, properties::PropertyKey};

/// Metadata about a graph's crossing minimization state.
#[derive(Clone, Copy)]
pub struct GraphInfoHolder {
    dont_sweep_into: bool,
}

struct P3GraphInfoMarker;

/// Stored GraphInfoHolder from P3 crossing minimization, for test introspection.
pub static P3_GRAPH_INFO: std::sync::LazyLock<PropertyKey<GraphInfoHolder>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<P3GraphInfoMarker>(GraphInfoHolder::empty));

impl GraphInfoHolder {
    /// Empty default for property initialization.
    pub fn empty() -> Self {
        Self { dont_sweep_into: true }
    }

    /// Build from the current graph state.
    pub fn from_graph(graph: &LGraph) -> Self {
        Self { dont_sweep_into: layer_sweep_type_decider::decide_bottom_up(graph) }
    }

    /// Whether this graph should be skipped during hierarchical sweep.
    pub fn dont_sweep_into(&self) -> bool {
        self.dont_sweep_into
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<GraphInfoHolder>();
    }
}
