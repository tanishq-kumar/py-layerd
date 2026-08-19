//! Greedy fix-point port distributor for crossing minimization.
//!
//! Port reordering is driven by pair-wise crossing counts.
//! For every node in the current layer this walks adjacent port pairs on
//! the sweep side and swaps them whenever a swap strictly reduces the
//! number of crossings. The loop reruns until no further improvement is
//! possible.

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        port::PortSide,
    },
    p3_crossing_min::{
        between_layer_crossing_counter::BetweenLayerEdgeTwoNodeCrossingsCounter, counting,
        layer_sweep::node_port_constraints,
    },
};

pub(crate) struct GreedyPortDistributorScratch {
    current_nodes: Vec<NodeId>,
    side_port_indices: Vec<usize>,
}

impl GreedyPortDistributorScratch {
    pub(crate) fn new() -> Self {
        Self { current_nodes: Vec::new(), side_port_indices: Vec::new() }
    }
}

pub(crate) fn distribute_ports_while_sweeping_with_scratch(
    graph: &mut LGraph,
    current_index: usize,
    forward: bool,
    scratch: &mut GreedyPortDistributorScratch,
) -> bool {
    if current_index >= graph.layers.len() {
        return false;
    }
    let side = if forward { PortSide::West } else { PortSide::East };
    let mut improved = false;
    let GreedyPortDistributorScratch { current_nodes, side_port_indices } = scratch;
    current_nodes.clear();
    current_nodes.extend_from_slice(&graph.layers[current_index].nodes);
    for &node_id in current_nodes.iter() {
        let constraints = node_port_constraints(graph, node_id);
        if constraints.is_order_fixed() {
            continue;
        }
        // The hierarchical counter requires both a non-empty side-view and a
        // nested graph on the node; skipping either check silently falls back
        // to a flat counter even when compound geometry is active.
        let has_side_ports = graph
            .node(node_id)
            .ports
            .iter()
            .any(|&port_id| graph.port(port_id).side == side);
        let use_hierarchical_cross_counter =
            has_side_ports && graph.node(node_id).nested_graph.is_some();
        improved |= distribute_ports_greedily_on_node(
            graph,
            node_id,
            side,
            use_hierarchical_cross_counter,
            side_port_indices,
        );
    }
    improved
}

fn distribute_ports_greedily_on_node(
    graph: &mut LGraph,
    node_id: NodeId,
    side: PortSide,
    use_hierarchical_cross_counter: bool,
    side_port_indices: &mut Vec<usize>,
) -> bool {
    let mut any_improved = false;
    loop {
        let mut round_improved = false;
        // For SOUTH and WEST iterate from the last port backwards, matching
        // the counter-clockwise port order assigned by `PortListSorter`.
        collect_side_port_indices(graph, node_id, side, side_port_indices);
        if matches!(side, PortSide::South | PortSide::West) {
            side_port_indices.reverse();
        }
        if side_port_indices.len() < 2 {
            break;
        }
        for pair_start in 0..side_port_indices.len() - 1 {
            let upper_idx = side_port_indices[pair_start];
            let lower_idx = side_port_indices[pair_start + 1];
            let upper_port = graph.node(node_id).ports[upper_idx];
            let lower_port = graph.node(node_id).ports[lower_idx];
            let (upper_lower, lower_upper) = count_port_pair_crossings(
                graph,
                node_id,
                upper_port,
                lower_port,
                side,
                use_hierarchical_cross_counter,
            );
            if upper_lower > lower_upper {
                graph.node_mut(node_id).ports.swap(upper_idx, lower_idx);
                graph.bump_node_order_version(node_id);
                round_improved = true;
                any_improved = true;
            }
        }
        if !round_improved {
            break;
        }
    }
    any_improved
}

fn count_port_pair_crossings(
    graph: &LGraph,
    node_id: NodeId,
    upper_port: PortId,
    lower_port: PortId,
    side: PortSide,
    use_hierarchical_cross_counter: bool,
) -> (usize, usize) {
    let layer = graph.node(node_id).layer.unwrap_or(0);
    let (mut upper_lower, mut lower_upper) = counting::count_crossings_between_ports_in_both_orders(
        graph, layer, upper_port, lower_port, side,
    );

    if use_hierarchical_cross_counter
        && let Some(upper_dummy) = graph.port(upper_port).port_dummy
        && let Some(lower_dummy) = graph.port(lower_port).port_dummy
        && let Some(child) = graph.nested(node_id)
        && !child.layers.is_empty()
    {
        // Build the counter once per node for the child graph's boundary
        // layer (first for a forward sweep, last for backward). The counter
        // replays the adjacency-merge algorithm over child-side port
        // positions, which is what the pair-wise crossing count actually
        // measures; `child_layer.len() - 1` was a cardinality upper bound,
        // not a crossing count.
        let child_free_layer_idx = if side == PortSide::West { 0 } else { child.layers.len() - 1 };
        let mut counter = BetweenLayerEdgeTwoNodeCrossingsCounter::new(child, child_free_layer_idx);
        counter.count_both_side_crossings(upper_dummy, lower_dummy);
        upper_lower += counter.upper_lower_crossings();
        lower_upper += counter.lower_upper_crossings();
    }

    (upper_lower, lower_upper)
}

fn collect_side_port_indices(
    graph: &LGraph,
    node_id: NodeId,
    side: PortSide,
    indices: &mut Vec<usize>,
) {
    indices.clear();
    indices.extend(graph.node(node_id).ports.iter().enumerate().filter_map(|(idx, &port_id)| {
        if graph.port(port_id).side == side { Some(idx) } else { None }
    }));
}
