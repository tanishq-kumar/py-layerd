//! Partition pre-, mid- and post-processors.

use std::collections::{HashSet, VecDeque};

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    properties::internal::{PARTITION_DUMMY, PARTITIONING_PARTITION, PRIORITY_DIRECTION},
};

/// Base priority assigned to partition constraint edges added by the preprocessor.
const PARTITION_CONSTRAINT_EDGE_PRIORITY: i32 = 1_000;

/// Reverses edges that connect higher-index to lower-index partitions.
///
/// Runs before phase 1 on layerless nodes.
pub fn preprocess(graph: &mut LGraph) {
    // Collect nodes that have a partition assigned.
    let partitioned_nodes: SmallVec<NodeId, 16> = graph
        .layerless_nodes
        .iter()
        .copied()
        .filter(|nid| graph.node(*nid).properties.has(&PARTITIONING_PARTITION))
        .collect();

    if partitioned_nodes.is_empty() {
        return;
    }

    // Collect edges that must be reversed in a separate pass to avoid mutating
    // adjacency lists during traversal.
    let mut edges_to_reverse: Vec<EdgeId> = Vec::new();
    for &nid in &partitioned_nodes {
        let outgoing: SmallVec<EdgeId, 8> = graph.outgoing_edges(nid).collect();
        for edge_id in outgoing {
            if must_be_reversed(graph, edge_id, partitioned_nodes.as_slice()) {
                edges_to_reverse.push(edge_id);
            }
        }
    }

    for edge_id in edges_to_reverse {
        reverse_with_priority(graph, edge_id);
    }
}

/// Returns `true` when the edge contradicts partition ordering.
///
/// Source node is assumed partitioned. If the target is partitioned, compares
/// the integer values directly. Otherwise BFS from the source through the
/// graph for any reachable partitioned node with a strictly lower partition.
fn must_be_reversed(graph: &LGraph, edge_id: EdgeId, partitioned_nodes: &[NodeId]) -> bool {
    let edge = graph.edge(edge_id);
    let source_node = graph.port(edge.source).owner;
    let target_node = graph.port(edge.target).owner;

    let source_partition = graph.node(source_node).properties.get(&PARTITIONING_PARTITION);

    if graph.node(target_node).properties.has(&PARTITIONING_PARTITION) {
        let target_partition = graph.node(target_node).properties.get(&PARTITIONING_PARTITION);
        return source_partition > target_partition;
    }

    // Target unpartitioned: walk the outgoing reachability set looking for a
    // partitioned node with strictly lower partition.
    let lower_partition_nodes: SmallVec<NodeId, 16> = partitioned_nodes
        .iter()
        .copied()
        .filter(|nid| graph.node(*nid).properties.get(&PARTITIONING_PARTITION) < source_partition)
        .collect();

    let mut queue: VecDeque<NodeId> = VecDeque::new();
    let mut visited: HashSet<NodeId> = HashSet::new();
    queue.push_back(source_node);
    visited.insert(source_node);

    while let Some(current) = queue.pop_front() {
        if lower_partition_nodes.contains(&current) {
            return true;
        }
        for out in graph.outgoing_edges(current) {
            let next = graph.port(graph.edge(out).target).owner;
            if visited.insert(next) {
                queue.push_back(next);
            }
        }
    }

    false
}

/// Reverses an edge and records a high direction priority so cycle breaking
/// will not undo the flip.
fn reverse_with_priority(graph: &mut LGraph, edge_id: EdgeId) {
    graph.reverse_edge_adapt_ports(edge_id);

    let mut priority = PARTITION_CONSTRAINT_EDGE_PRIORITY;
    if graph.edge(edge_id).properties.has(&PRIORITY_DIRECTION) {
        priority += graph.edge(edge_id).properties.get(&PRIORITY_DIRECTION);
    }
    graph.edge_mut(edge_id).properties.set(&PRIORITY_DIRECTION, priority);
}

/// Adds dummy edges between consecutive partition groups so the layering
/// phase respects partition ordering.
///
/// Runs before phase 2 on layerless nodes.
pub fn midprocess(graph: &mut LGraph) {
    // Collect (partition_id, node_id) pairs for all partitioned nodes.
    let mut entries: SmallVec<(i32, NodeId), 32> = graph
        .layerless_nodes
        .iter()
        .copied()
        .filter_map(|nid| {
            if graph.node(nid).properties.has(&PARTITIONING_PARTITION) {
                Some((graph.node(nid).properties.get(&PARTITIONING_PARTITION), nid))
            } else {
                None
            }
        })
        .collect();

    if entries.is_empty() {
        return;
    }

    // Group nodes by partition id, sorted ascending.
    entries.sort_by_key(|&(id, _)| id);

    // Build groups as consecutive runs of equal partition id.
    let mut groups: Vec<(i32, SmallVec<NodeId, 8>)> = Vec::new();
    for (id, nid) in entries {
        match groups.last_mut() {
            Some((last_id, nodes)) if *last_id == id => nodes.push(nid),
            _ => groups.push((id, SmallVec::from_iter([nid]))),
        }
    }

    // Connect every node in group `k` to every node in group `k + 1`.
    for pair in groups.windows(2) {
        let first: SmallVec<NodeId, 8> = pair[0].1.clone();
        let second: SmallVec<NodeId, 8> = pair[1].1.clone();
        connect_nodes(graph, first.as_slice(), second.as_slice());
    }
}

/// Creates partition dummy ports and edges so that every node in `firsts`
/// has a constraint edge to every node in `seconds`.
fn connect_nodes(graph: &mut LGraph, firsts: &[NodeId], seconds: &[NodeId]) {
    for &source_node in firsts {
        let source_port = graph.add_port(source_node, PortSide::East);
        graph.port_mut(source_port).properties.set(&PARTITION_DUMMY, true);

        for &target_node in seconds {
            let target_port = graph.add_port(target_node, PortSide::West);
            graph.port_mut(target_port).properties.set(&PARTITION_DUMMY, true);

            let edge_id = graph.add_edge(source_port, target_port);
            graph.edge_mut(edge_id).properties.set(&PARTITION_DUMMY, true);
        }
    }
}

/// Removes the partition dummy ports (and their dangling edges) installed by
/// [`midprocess`] so the rest of the pipeline does not see them.
///
/// Runs before phase 3 once the graph has been layered.
pub fn postprocess(graph: &mut LGraph) {
    // First collect every partition dummy port across the layered graph.
    let mut dummy_ports: Vec<PortId> = Vec::new();
    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            for &port_id in &graph.node(node_id).ports {
                if graph.port(port_id).properties.get(&PARTITION_DUMMY) {
                    dummy_ports.push(port_id);
                }
            }
        }
    }

    // Remove every edge incident to a dummy port, then drop the port from
    // its owning node. The arena needs explicit cleanup since there is no
    // GC fallback.
    for port_id in dummy_ports {
        let incident: SmallVec<EdgeId, 4> = graph
            .port(port_id)
            .outgoing_edges
            .iter()
            .copied()
            .chain(graph.port(port_id).incoming_edges.iter().copied())
            .collect();
        for edge_id in incident {
            graph.remove_edge(edge_id);
        }

        let owner = graph.port(port_id).owner;
        graph.node_mut(owner).ports.retain(|p| *p != port_id);
    }
}
