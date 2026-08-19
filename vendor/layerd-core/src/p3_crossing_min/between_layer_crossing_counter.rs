//! Between-layer two-node crossings counter for the greedy-switch heuristic.
//!
//! Counts between-layer edge crossings for two nodes in a free layer. The
//! counter is stateful: callers invoke one of `count_*_crossings` to recompute
//! for a given pair, then read results via `upper_lower_crossings` /
//! `lower_upper_crossings`. The heavy lifting delegates to the stateless
//! `counting::count_crossings_between_pair_nodes` helper, which is the free-fn
//! equivalent of the eastern/western-crossings counters used by greedy switching.

use crate::{
    graph::{LGraph, index::NodeId, port::PortSide},
    p3_crossing_min::counting::count_crossings_between_pair_nodes,
};

pub struct BetweenLayerEdgeTwoNodeCrossingsCounter<'a> {
    graph: &'a LGraph,
    free_layer_idx: usize,
    upper_lower: usize,
    lower_upper: usize,
}

impl<'a> BetweenLayerEdgeTwoNodeCrossingsCounter<'a> {
    pub fn new(graph: &'a LGraph, free_layer_index: usize) -> Self {
        Self { graph, free_layer_idx: free_layer_index, upper_lower: 0, lower_upper: 0 }
    }

    pub fn count_both_side_crossings(&mut self, upper: NodeId, lower: NodeId) {
        self.reset();
        if upper == lower {
            return;
        }
        self.add_side_crossings(upper, lower, PortSide::West);
        self.add_side_crossings(upper, lower, PortSide::East);
    }

    pub fn upper_lower_crossings(&self) -> usize {
        self.upper_lower
    }

    pub fn lower_upper_crossings(&self) -> usize {
        self.lower_upper
    }

    fn reset(&mut self) {
        self.upper_lower = 0;
        self.lower_upper = 0;
    }

    fn add_side_crossings(&mut self, upper: NodeId, lower: NodeId, side: PortSide) {
        let (ul, lu) =
            count_crossings_between_pair_nodes(self.graph, self.free_layer_idx, upper, lower, side);
        self.upper_lower += ul;
        self.lower_upper += lu;
    }
}
