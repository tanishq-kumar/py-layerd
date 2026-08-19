//! Rust-only helper: if `CONSIDER_MODEL_ORDER_PORT_MODEL_ORDER` is true, sorts
//! each node's ports by the model order of the connected node across their
//! incident edges. Ports without explicit model order fall back to their
//! creation order, which matches the seed assigned during graph import.
//!
//! Kept as a standalone preprocessor (rather than folded into
//! `SortByInputModelProcessor`) so later processors can consume a stable
//! port order.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
    },
    properties::internal::MODEL_ORDER,
};

/// Orders ports by the model order of their connected nodes, for every node
/// in every layer (plus layerless).
pub fn order(graph: &mut LGraph) {
    if !graph.options.consider_model_order_port_model_order {
        return;
    }

    let mut targets: Vec<NodeId> = Vec::new();
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            targets.push(nid);
        }
    }
    targets.extend_from_slice(&graph.layerless_nodes);

    for nid in targets {
        sort_ports_by_connected_model_order(graph, nid);
    }
}

/// Sort one node's port list by the minimum model order observed across the
/// nodes connected via each port's incident edges. Ports with no connected
/// node that carries `MODEL_ORDER` fall back to `i32::MAX` so they sort to the
/// end, preserving their relative insertion order via a stable sort.
fn sort_ports_by_connected_model_order(graph: &mut LGraph, node_id: NodeId) {
    let port_ids: SmallVec<PortId, 6> = graph.node(node_id).ports.iter().copied().collect();
    if port_ids.len() < 2 {
        return;
    }
    let mut keyed: SmallVec<(PortId, i32), 6> = port_ids
        .iter()
        .map(|&pid| (pid, min_connected_model_order(graph, pid)))
        .collect();
    keyed.sort_by_key(|&(_, key)| key);
    let new_order: SmallVec<PortId, 2> = keyed.iter().map(|&(pid, _)| pid).collect();
    graph.node_mut(node_id).ports = new_order;
}

fn min_connected_model_order(graph: &LGraph, port_id: PortId) -> i32 {
    let mut best = i32::MAX;
    let port = graph.port(port_id);
    for &eid in port.outgoing_edges.iter().chain(port.incoming_edges.iter()) {
        let edge = graph.edge(eid);
        let other_port = if edge.source == port_id { edge.target } else { edge.source };
        let other_node = graph.port(other_port).owner;
        if graph.node(other_node).properties.has(&MODEL_ORDER) {
            let mo = graph.node(other_node).properties.get(&MODEL_ORDER);
            if mo < best {
                best = mo;
            }
        }
    }
    best
}
