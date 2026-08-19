//! Decide whether to use bottom-up or cross-hierarchical sweep.

use hashbrown::HashMap;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType, port::PortSide},
    properties::internal::EXT_PORT_SIDE,
};

#[derive(Clone, Copy, Default)]
struct NodeInfluence {
    connected_edges: usize,
    hierarchical_influence: usize,
    random_influence: usize,
}

/// Decide whether to use bottom-up sweep for this graph.
pub(crate) fn decide_bottom_up(graph: &LGraph) -> bool {
    let boundary = graph.options.crossing_minimization_hierarchical_sweepiness;

    if boundary < -1.0 {
        return true;
    }

    if graph.parent_node.is_none() {
        return true;
    }

    if graph.options.port_constraints.is_order_fixed() {
        return true;
    }

    if fewer_than_two_in_out_edges(graph) {
        return true;
    }

    let (paths_to_random, paths_to_hierarchical) = path_influence_balance(graph);
    let all_paths = paths_to_random + paths_to_hierarchical;
    let normalized = if all_paths == 0 {
        f64::INFINITY
    } else {
        (paths_to_random as f64 - paths_to_hierarchical as f64) / all_paths as f64
    };

    normalized >= boundary
}

fn path_influence_balance(graph: &LGraph) -> (usize, usize) {
    let mut paths_to_random = 0usize;
    let mut paths_to_hierarchical = 0usize;
    let mut node_info: HashMap<NodeId, NodeInfluence> = HashMap::new();

    for layer in &graph.layers {
        let mut north_south_dummies = Vec::new();

        for &node_id in &layer.nodes {
            if graph.node(node_id).node_type == NodeType::NorthSouthPort {
                north_south_dummies.push(node_id);
                continue;
            }

            let mut current = node_info.get(&node_id).copied().unwrap_or_default();

            if is_external_port_dummy(graph, node_id) {
                current.hierarchical_influence = 1;
                if is_eastern_dummy(graph, node_id) {
                    paths_to_hierarchical += current.connected_edges;
                }
            } else if has_no_connected_ports_on_side(graph, node_id, PortSide::West) {
                current.random_influence = 1;
            } else if has_no_connected_ports_on_side(graph, node_id, PortSide::East) {
                paths_to_random += current.connected_edges;
            }

            node_info.insert(node_id, current);

            for edge_id in graph.outgoing_edges(node_id) {
                let target = graph.port(graph.edge(edge_id).target).owner;
                paths_to_random += current.random_influence;
                paths_to_hierarchical += current.hierarchical_influence;
                transfer_influence(&mut node_info, target, current);
            }

            for dummy_id in connected_north_south_dummies(graph, node_id) {
                paths_to_random += current.random_influence;
                paths_to_hierarchical += current.hierarchical_influence;
                transfer_influence(&mut node_info, dummy_id, current);
            }
        }

        for node_id in north_south_dummies {
            let current = node_info.get(&node_id).copied().unwrap_or_default();
            for edge_id in graph.outgoing_edges(node_id) {
                let target = graph.port(graph.edge(edge_id).target).owner;
                paths_to_random += current.random_influence;
                paths_to_hierarchical += current.hierarchical_influence;
                transfer_influence(&mut node_info, target, current);
            }
        }
    }

    (paths_to_random, paths_to_hierarchical)
}

fn transfer_influence(
    node_info: &mut HashMap<NodeId, NodeInfluence>,
    target: NodeId,
    current: NodeInfluence,
) {
    let target_info = node_info.entry(target).or_default();
    target_info.hierarchical_influence += current.hierarchical_influence;
    target_info.random_influence += current.random_influence;
    target_info.connected_edges += current.connected_edges + 1;
}

fn fewer_than_two_in_out_edges(graph: &LGraph) -> bool {
    let mut east = 0usize;
    let mut west = 0usize;

    for (_, node) in graph.nodes_iter() {
        if node.node_type != NodeType::ExternalPort {
            continue;
        }

        let side = node.properties.get(&EXT_PORT_SIDE);
        let connected = node.ports.iter().any(|&port_id| {
            !graph.port(port_id).incoming_edges.is_empty()
                || !graph.port(port_id).outgoing_edges.is_empty()
        });

        if !connected {
            continue;
        }

        match side {
            PortSide::East => east += 1,
            PortSide::West => west += 1,
            _ => {}
        }
    }

    east < 2 && west < 2
}

fn is_external_port_dummy(graph: &LGraph, node_id: NodeId) -> bool {
    graph.node(node_id).node_type == NodeType::ExternalPort
}

fn is_eastern_dummy(graph: &LGraph, node_id: NodeId) -> bool {
    graph.node(node_id).properties.get(&EXT_PORT_SIDE) == PortSide::East
}

fn has_no_connected_ports_on_side(graph: &LGraph, node_id: NodeId, side: PortSide) -> bool {
    for &port_id in &graph.node(node_id).ports {
        let port = graph.port(port_id);
        if port.side != side {
            continue;
        }

        if !port.incoming_edges.is_empty() || !port.outgoing_edges.is_empty() {
            return false;
        }
    }

    true
}

fn connected_north_south_dummies(graph: &LGraph, node_id: NodeId) -> Vec<NodeId> {
    let mut dummies = Vec::new();
    for &port_id in &graph.node(node_id).ports {
        let port = graph.port(port_id);
        if !matches!(port.side, PortSide::North | PortSide::South) {
            continue;
        }

        if let Some(dummy) = port.port_dummy
            && graph.node(dummy).node_type == NodeType::NorthSouthPort
        {
            dummies.push(dummy);
        }
    }
    dummies
}
