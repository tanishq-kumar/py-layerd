use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    properties::internal::LONG_EDGE_BEFORE_LABEL_DUMMY,
};

/// Merges adjacent long-edge dummy nodes that belong to the same hyperedge.
///
/// Two adjacent dummies are merged when they share a long-edge source or
/// target port (and no label dummies block the merge), or when they are
/// connected through an already-identified hyperedge identified by DFS over
/// connected ports.
///
/// Runs after P3 (crossing minimization) and before P4 (node placement).
pub fn merge(graph: &mut LGraph) {
    let hyperedge_ids = identify_hyperedges(graph);

    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let mut nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        if nodes.is_empty() {
            continue;
        }
        let mut changed = false;

        let mut current_node: Option<NodeId>;
        let mut current_type: Option<NodeType>;
        let mut last_node: Option<NodeId> = None;
        let mut last_type: Option<NodeType> = None;

        let mut i = 0;
        while i < nodes.len() {
            let curr = nodes[i];
            let curr_type = graph.node(curr).node_type;
            current_node = Some(curr);
            current_type = Some(curr_type);

            if curr_type == NodeType::LongEdge && last_type == Some(NodeType::LongEdge) {
                let last = last_node.expect("last_type set implies last_node set");
                let state = check_merge_allowed(graph, curr, last, &hyperedge_ids);
                if state.allow_merge {
                    merge_nodes(graph, curr, last, state.same_source, state.same_target);
                    nodes.remove(i);
                    changed = true;
                    current_node = Some(last);
                    current_type = Some(last_type.unwrap());
                    i = i.saturating_sub(1);
                }
            }

            last_node = current_node;
            last_type = current_type;
            i += 1;
        }

        if changed {
            graph.layers[layer_idx].nodes = nodes;
        }
    }
}

struct MergeState {
    allow_merge: bool,
    same_source: bool,
    same_target: bool,
}

fn check_merge_allowed(
    graph: &LGraph,
    curr: NodeId,
    last: NodeId,
    hyperedge_ids: &HyperedgeIds,
) -> MergeState {
    let curr_has_labels = graph.node(curr).long_edge_has_label_dummies;
    let last_has_labels = graph.node(last).long_edge_has_label_dummies;

    let curr_source = graph.node(curr).long_edge_source;
    let last_source = graph.node(last).long_edge_source;
    let curr_target = graph.node(curr).long_edge_target;
    let last_target = graph.node(last).long_edge_target;

    let same_source = curr_source.is_some() && curr_source == last_source;
    let same_target = curr_target.is_some() && curr_target == last_target;

    if !curr_has_labels && !last_has_labels {
        let curr_first_port = graph.node(curr).ports.first().copied();
        let last_first_port = graph.node(last).ports.first().copied();
        let allow_merge = match (curr_first_port, last_first_port) {
            (Some(a), Some(b)) => match (hyperedge_ids.get(a), hyperedge_ids.get(b)) {
                (Some(x), Some(y)) => x == y,
                _ => false,
            },
            _ => false,
        };
        return MergeState { allow_merge, same_source, same_target };
    }

    let curr_before = graph.node(curr).properties.get(&LONG_EDGE_BEFORE_LABEL_DUMMY);
    let last_before = graph.node(last).properties.get(&LONG_EDGE_BEFORE_LABEL_DUMMY);

    let eligible_source = (!curr_has_labels || curr_before) && (!last_has_labels || last_before);
    let eligible_target = (!curr_has_labels || !curr_before) && (!last_has_labels || !last_before);

    let allow_merge = (same_source && eligible_source) || (same_target && eligible_target);

    MergeState { allow_merge, same_source, same_target }
}

/// Move all incoming/outgoing edges of `merge_source` to the single west/east
/// port of `merge_target`. Clears `LONG_EDGE_SOURCE` / `LONG_EDGE_TARGET` on
/// the target if the merged dummies disagreed on them.
fn merge_nodes(
    graph: &mut LGraph,
    merge_source: NodeId,
    merge_target: NodeId,
    keep_source: bool,
    keep_target: bool,
) {
    let target_input_port = find_port_on_side(graph, merge_target, PortSide::West)
        .expect("long-edge dummy missing west port");
    let target_output_port = find_port_on_side(graph, merge_target, PortSide::East)
        .expect("long-edge dummy missing east port");

    let source_ports: SmallVec<PortId, 4> =
        SmallVec::from_slice_copy(&graph.node(merge_source).ports);
    for port_id in source_ports {
        graph.move_incoming_edges(port_id, target_input_port);
        graph.move_outgoing_edges(port_id, target_output_port);
    }

    if !keep_source {
        graph.node_mut(merge_target).long_edge_source = None;
    }
    if !keep_target {
        graph.node_mut(merge_target).long_edge_target = None;
    }
}

fn find_port_on_side(graph: &LGraph, node: NodeId, side: PortSide) -> Option<PortId> {
    graph.node(node).ports.iter().copied().find(|&p| graph.port(p).side == side)
}

/// DFS every port of every layer node to assign hyperedge indices.
///
/// Two ports share an index iff they are reachable via edges-between-ports
/// plus, for `LongEdge` dummies, via same-node port traversal.
struct HyperedgeIds {
    values: Vec<i32>,
}

impl HyperedgeIds {
    const MISSING: i32 = -2;
    const UNVISITED: i32 = -1;

    fn new() -> Self {
        Self { values: Vec::new() }
    }

    #[inline]
    fn ensure(&mut self, port: PortId) {
        let idx = Self::index(port);
        if idx >= self.values.len() {
            self.values.resize(idx + 1, Self::MISSING);
        }
    }

    #[inline]
    fn mark_unvisited(&mut self, port: PortId) {
        self.ensure(port);
        self.values[Self::index(port)] = Self::UNVISITED;
    }

    #[inline]
    fn get(&self, port: PortId) -> Option<i32> {
        self.values
            .get(Self::index(port))
            .copied()
            .filter(|&value| value != Self::MISSING && value != Self::UNVISITED)
    }

    #[inline]
    fn is_unvisited(&self, port: PortId) -> bool {
        self.values.get(Self::index(port)).copied() == Some(Self::UNVISITED)
    }

    #[inline]
    fn assign(&mut self, port: PortId, index: i32) -> bool {
        let idx = Self::index(port);
        if self.values.get(idx).copied() == Some(index) {
            return false;
        }
        self.values[idx] = index;
        true
    }

    #[inline]
    fn index(port: PortId) -> usize {
        port.0.index() as usize
    }
}

fn identify_hyperedges(graph: &LGraph) -> HyperedgeIds {
    let mut ids = HyperedgeIds::new();
    let mut order: Vec<PortId> = Vec::new();

    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            for &port_id in &graph.node(node_id).ports {
                ids.mark_unvisited(port_id);
                order.push(port_id);
            }
        }
    }

    let mut index: i32 = 0;
    for port in order {
        if ids.is_unvisited(port) {
            dfs(graph, port, index, &mut ids);
            index += 1;
        }
    }

    ids
}

fn dfs(graph: &LGraph, port: PortId, index: i32, ids: &mut HyperedgeIds) {
    let mut stack: Vec<PortId> = vec![port];
    while let Some(current) = stack.pop() {
        if !ids.assign(current, index) {
            continue;
        }

        for &edge_id in &graph.port(current).outgoing_edges {
            let connected = graph.edge(edge_id).target;
            if ids.is_unvisited(connected) {
                stack.push(connected);
            }
        }
        for &edge_id in &graph.port(current).incoming_edges {
            let connected = graph.edge(edge_id).source;
            if ids.is_unvisited(connected) {
                stack.push(connected);
            }
        }

        let owner = graph.port(current).owner;
        if graph.node(owner).node_type == NodeType::LongEdge {
            for &p2 in &graph.node(owner).ports {
                if p2 != current && ids.is_unvisited(p2) {
                    stack.push(p2);
                }
            }
        }
    }
}
