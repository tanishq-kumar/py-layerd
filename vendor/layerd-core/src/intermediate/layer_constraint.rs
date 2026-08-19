use hashbrown::HashMap;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
    },
    options::enums::LayerConstraint,
    properties::{
        PropertyKey,
        internal::{LAYER_CONSTRAINT, ORIGINAL_OPPOSITE_PORT},
    },
};

/// Bookkeeping for `update_opposite_node_layer_constraints`: tracks which
/// kinds of hidden nodes a given "opposite" node was connected to. When the
/// opposite node becomes isolated and its only ties were to one kind, we
/// promote it into the matching layer so it doesn't wander off.
#[derive(Clone, Copy, PartialEq, Eq)]
enum HiddenNodeConnections {
    None,
    FirstSeparate,
    LastSeparate,
    Both,
}

impl HiddenNodeConnections {
    fn combine(self, lc: LayerConstraint) -> Self {
        match (self, lc) {
            (Self::None, LayerConstraint::FirstSeparate) => Self::FirstSeparate,
            (Self::None, LayerConstraint::LastSeparate) => Self::LastSeparate,
            (Self::FirstSeparate, LayerConstraint::FirstSeparate) => Self::FirstSeparate,
            (Self::FirstSeparate, LayerConstraint::LastSeparate) => Self::Both,
            (Self::LastSeparate, LayerConstraint::FirstSeparate) => Self::Both,
            (Self::LastSeparate, LayerConstraint::LastSeparate) => Self::LastSeparate,
            _ => Self::Both,
        }
    }
}

/// Hides nodes with `FIRST_SEPARATE` or `LAST_SEPARATE` layer constraints
/// before layering (P2). Their edges are disconnected and stored so they can be
/// restored later.
pub fn preprocess(graph: &mut LGraph) {
    let mut hidden_nodes: Vec<HiddenNodeInfo> = Vec::new();
    let mut opposite_connections: HashMap<NodeId, HiddenNodeConnections> = HashMap::new();

    // Iterate over layerless nodes and find those with FIRST_SEPARATE or LAST_SEPARATE
    let layerless: Vec<NodeId> = graph.layerless_nodes.clone();
    let mut to_remove: Vec<NodeId> = Vec::new();

    for &nid in &layerless {
        let constraint = graph.node(nid).properties.get(&LAYER_CONSTRAINT);
        match constraint {
            LayerConstraint::FirstSeparate | LayerConstraint::LastSeparate => {
                // Validate edges via `ensure_no_inacceptable_edges`.
                ensure_no_inacceptable_edges(graph, nid, constraint);
                // Collect opposite nodes for the constraint-propagation pass
                // before disconnecting.
                let opposites = collect_opposite_nodes(graph, nid);
                let disconnected = disconnect_node(graph, nid);
                for other in opposites {
                    let prev = opposite_connections
                        .get(&other)
                        .copied()
                        .unwrap_or(HiddenNodeConnections::None);
                    let next = prev.combine(constraint);
                    opposite_connections.insert(other, next);
                }
                hidden_nodes.push(HiddenNodeInfo { node_id: nid, constraint, edges: disconnected });
                to_remove.push(nid);
            }
            _ => {}
        }
    }

    // Remove hidden nodes from layerless list
    for &nid in &to_remove {
        graph.layerless_nodes.retain(|&n| n != nid);
    }

    // Promote orphaned opposite nodes into FIRST / LAST if they had only
    // connections to one kind of hidden node.
    for (other, connections) in opposite_connections {
        // Skip if the opposite node already has a constraint.
        let existing = graph.node(other).properties.get(&LAYER_CONSTRAINT);
        if existing != LayerConstraint::None {
            continue;
        }
        // Skip if the opposite node still has any connected edge left —
        // reuses the full-graph iterator because individual ports are
        // already stripped of the hidden edges.
        let still_connected = graph.incoming_edges(other).next().is_some()
            || graph.outgoing_edges(other).next().is_some();
        if still_connected {
            continue;
        }
        match connections {
            HiddenNodeConnections::FirstSeparate => {
                graph.node_mut(other).properties.set(&LAYER_CONSTRAINT, LayerConstraint::First);
            }
            HiddenNodeConnections::LastSeparate => {
                graph.node_mut(other).properties.set(&LAYER_CONSTRAINT, LayerConstraint::Last);
            }
            _ => {}
        }
    }

    // Store hidden nodes
    if !hidden_nodes.is_empty() {
        graph.properties.set(&HIDDEN_NODES, hidden_nodes);
    }
}

/// Collect every node connected to `nid` via an outgoing or incoming edge.
/// Ignores self-loops.
fn collect_opposite_nodes(graph: &LGraph, nid: NodeId) -> Vec<NodeId> {
    let mut opposites: Vec<NodeId> = Vec::new();
    for eid in graph.outgoing_edges(nid) {
        let other = graph.port(graph.edge(eid).target).owner;
        if other != nid {
            opposites.push(other);
        }
    }
    for eid in graph.incoming_edges(nid) {
        let other = graph.port(graph.edge(eid).source).owner;
        if other != nid {
            opposites.push(other);
        }
    }
    opposites
}

/// Validate that a FIRST_SEPARATE node has no non-external incoming edges
/// and a LAST_SEPARATE node has no non-external outgoing edges.
///
/// Uses `debug_assert!`: the invariant is a producer contract, so violations
/// are programmer errors rather than runtime data errors.
fn ensure_no_inacceptable_edges(graph: &LGraph, nid: NodeId, lc: LayerConstraint) {
    let is_acceptable = |src: NodeId, tgt: NodeId| -> bool {
        graph.node(src).node_type == NodeType::ExternalPort
            && graph.node(tgt).node_type == NodeType::ExternalPort
    };
    match lc {
        LayerConstraint::FirstSeparate =>
            for eid in graph.incoming_edges(nid) {
                let src = graph.port(graph.edge(eid).source).owner;
                debug_assert!(
                    is_acceptable(src, nid),
                    "FIRST_SEPARATE node has a non-external incoming edge"
                );
            },
        LayerConstraint::LastSeparate =>
            for eid in graph.outgoing_edges(nid) {
                let tgt = graph.port(graph.edge(eid).target).owner;
                debug_assert!(
                    is_acceptable(nid, tgt),
                    "LAST_SEPARATE node has a non-external outgoing edge"
                );
            },
        _ => {}
    }
}

/// Restores hidden nodes and moves constrained nodes to the appropriate layers.
/// Runs before crossing minimization (P3).
///
/// - `FIRST_SEPARATE`: placed in a new layer at position 0
/// - `LAST_SEPARATE`: placed in a new layer at the end
/// - `FIRST`: moved to the existing first layer
/// - `LAST`: moved to the existing last layer
pub fn postprocess(graph: &mut LGraph) {
    // FIRST/LAST movement is gated on `!layers.isEmpty()` (no movement
    // possible without an existing first/last layer), but FIRST_SEPARATE /
    // LAST_SEPARATE *restoration* runs unconditionally — fresh layers are
    // created and inserted even on graphs whose only nodes were hidden
    // FIRST_SEPARATE EP dummies (e.g. trivial components emitted when an
    // external port has no inner edges, which would otherwise be layerless
    // and report `graph.size = (0, 0)`).
    if !graph.layers.is_empty() {
        // Handle FIRST and LAST constraints for nodes already in layers
        move_first_and_last_nodes(graph);
    }

    // Restore FIRST_SEPARATE and LAST_SEPARATE hidden nodes (always)
    let hidden_nodes: Vec<HiddenNodeInfo> = graph.properties.get(&HIDDEN_NODES);
    if hidden_nodes.is_empty() {
        return;
    }

    let mut first_separate_nodes: Vec<NodeId> = Vec::new();
    let mut last_separate_nodes: Vec<NodeId> = Vec::new();

    for info in &hidden_nodes {
        // Reconnect edges
        for disconnected_edge in &info.edges {
            reconnect_edge(graph, disconnected_edge);
        }

        match info.constraint {
            LayerConstraint::FirstSeparate => {
                first_separate_nodes.push(info.node_id);
            }
            LayerConstraint::LastSeparate => {
                last_separate_nodes.push(info.node_id);
            }
            _ => {}
        }
    }

    // Create layers for FIRST_SEPARATE nodes
    if !first_separate_nodes.is_empty() {
        let mut layer = LayerData::new();
        for &nid in &first_separate_nodes {
            graph.node_mut(nid).layer = Some(0).into();
            layer.nodes.push(nid);
        }
        graph.layers.insert(0, layer);

        // Update layer indices for all subsequent layers
        for layer_idx in 1..graph.layers.len() {
            let layer_nodes: SmallVec<NodeId, 32> =
                SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
            for &nid in &layer_nodes {
                graph.node_mut(nid).layer = Some(layer_idx).into();
            }
        }
    }

    // Create layers for LAST_SEPARATE nodes
    if !last_separate_nodes.is_empty() {
        let new_idx = graph.layers.len();
        let mut layer = LayerData::new();
        for &nid in &last_separate_nodes {
            graph.node_mut(nid).layer = Some(new_idx).into();
            layer.nodes.push(nid);
        }
        graph.layers.push(layer);
    }
}

/// Move nodes with FIRST and LAST constraints to the appropriate existing
/// layers. Label dummy nodes incident to those FIRST / LAST nodes are
/// collected into separate label layers, which are inserted right outside
/// the FIRST / LAST layers so long edges don't overlap.
fn move_first_and_last_nodes(graph: &mut LGraph) {
    let first_layer_idx = 0;
    let last_layer_idx = graph.layers.len() - 1;

    let mut to_first: Vec<(NodeId, usize)> = Vec::new();
    let mut to_last: Vec<(NodeId, usize)> = Vec::new();
    let mut first_label_dummies: Vec<(NodeId, usize)> = Vec::new();
    let mut last_label_dummies: Vec<(NodeId, usize)> = Vec::new();

    for layer_idx in 0..graph.layers.len() {
        let nodes = graph.layers[layer_idx].nodes.clone();
        for &nid in &nodes {
            let constraint = graph.node(nid).properties.get(&LAYER_CONSTRAINT);
            match constraint {
                LayerConstraint::First => {
                    to_first.push((nid, layer_idx));
                    // Label dummies on incoming edges move into the label
                    // layer *before* the FIRST layer.
                    for eid in graph.incoming_edges(nid) {
                        let src = graph.port(graph.edge(eid).source).owner;
                        if graph.node(src).node_type == NodeType::Label {
                            let cur = graph.node(src).layer.unwrap_or(layer_idx);
                            first_label_dummies.push((src, cur));
                        }
                    }
                }
                LayerConstraint::Last => {
                    to_last.push((nid, layer_idx));
                    for eid in graph.outgoing_edges(nid) {
                        let tgt = graph.port(graph.edge(eid).target).owner;
                        if graph.node(tgt).node_type == NodeType::Label {
                            let cur = graph.node(tgt).layer.unwrap_or(layer_idx);
                            last_label_dummies.push((tgt, cur));
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // Move FIRST nodes.
    for (nid, old_layer) in &to_first {
        graph.layers[*old_layer].nodes.retain(|&n| n != *nid);
        graph.layers[first_layer_idx].nodes.push(*nid);
        graph.node_mut(*nid).layer = Some(first_layer_idx).into();
    }
    // Move LAST nodes.
    for (nid, old_layer) in &to_last {
        graph.layers[*old_layer].nodes.retain(|&n| n != *nid);
        graph.layers[last_layer_idx].nodes.push(*nid);
        graph.node_mut(*nid).layer = Some(last_layer_idx).into();
    }

    // Build the two label layers from the collected dummies. Remove each
    // dummy from its current layer first.
    let mut first_label_layer = LayerData::new();
    let mut first_seen: hashbrown::HashSet<NodeId> = hashbrown::HashSet::new();
    for (nid, old_layer) in first_label_dummies {
        if !first_seen.insert(nid) {
            continue;
        }
        if old_layer < graph.layers.len() {
            graph.layers[old_layer].nodes.retain(|&n| n != nid);
        }
        first_label_layer.nodes.push(nid);
    }
    let mut last_label_layer = LayerData::new();
    let mut last_seen: hashbrown::HashSet<NodeId> = hashbrown::HashSet::new();
    for (nid, old_layer) in last_label_dummies {
        if !last_seen.insert(nid) {
            continue;
        }
        if old_layer < graph.layers.len() {
            graph.layers[old_layer].nodes.retain(|&n| n != nid);
        }
        last_label_layer.nodes.push(nid);
    }

    // Remove empty layers.
    let mut i = 0;
    while i < graph.layers.len() {
        if graph.layers[i].nodes.is_empty() {
            graph.layers.remove(i);
        } else {
            i += 1;
        }
    }

    // Insert the label layers right outside the current first / last
    // layers, if they carry any dummy.
    if !first_label_layer.nodes.is_empty() {
        graph.layers.insert(0, first_label_layer);
    }
    if !last_label_layer.nodes.is_empty() {
        graph.layers.push(last_label_layer);
    }

    // Re-index every layer now that positions may have shifted.
    for j in 0..graph.layers.len() {
        let layer_nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[j].nodes);
        for &nid in &layer_nodes {
            graph.node_mut(nid).layer = Some(j).into();
        }
    }
}

/// Information about a hidden node, stored for restoration.
#[derive(Debug, Clone)]
pub struct HiddenNodeInfo {
    pub node_id: NodeId,
    pub constraint: LayerConstraint,
    pub edges: Vec<DisconnectedEdge>,
}

struct HiddenNodesMarker;

/// Hidden nodes stored during layer constraint preprocessing.
pub static HIDDEN_NODES: std::sync::LazyLock<PropertyKey<Vec<HiddenNodeInfo>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<HiddenNodesMarker>(Vec::new));

/// Information about a disconnected edge, stored for restoration.
#[derive(Debug, Clone, Copy)]
pub struct DisconnectedEdge {
    pub edge_id: EdgeId,
    pub is_outgoing: bool,
    pub opposite_port: PortId,
}

/// Disconnect all edges from a node, storing info for later reconnection.
fn disconnect_node(graph: &mut LGraph, nid: NodeId) -> Vec<DisconnectedEdge> {
    let mut disconnected = Vec::new();

    let ports: Vec<PortId> = graph.node(nid).ports.to_vec();

    for &pid in &ports {
        // Handle outgoing edges
        let outgoing: Vec<EdgeId> = graph.port(pid).outgoing_edges.to_vec();
        for &eid in &outgoing {
            let target_port = graph.edge(eid).target;
            let target_owner = graph.port(target_port).owner;

            // Only disconnect if it goes to a different node
            if target_owner != nid {
                // Remove from target port's incoming list
                graph.port_mut(target_port).incoming_edges.retain(|e| *e != eid);
                // Store original opposite port
                graph.edge_mut(eid).properties.set(&ORIGINAL_OPPOSITE_PORT, Some(target_port));
                disconnected.push(DisconnectedEdge {
                    edge_id: eid,
                    is_outgoing: true,
                    opposite_port: target_port,
                });
            }
        }

        // Handle incoming edges
        let incoming: Vec<EdgeId> = graph.port(pid).incoming_edges.to_vec();
        for &eid in &incoming {
            let source_port = graph.edge(eid).source;
            let source_owner = graph.port(source_port).owner;

            if source_owner != nid {
                // Remove from source port's outgoing list
                graph.port_mut(source_port).outgoing_edges.retain(|e| *e != eid);
                graph.edge_mut(eid).properties.set(&ORIGINAL_OPPOSITE_PORT, Some(source_port));
                disconnected.push(DisconnectedEdge {
                    edge_id: eid,
                    is_outgoing: false,
                    opposite_port: source_port,
                });
            }
        }
    }

    disconnected
}

/// Reconnect a previously disconnected edge.
fn reconnect_edge(graph: &mut LGraph, edge_info: &DisconnectedEdge) {
    if edge_info.is_outgoing {
        let target_port = edge_info.opposite_port;
        graph.port_mut(target_port).incoming_edges.push(edge_info.edge_id);
    } else {
        let source_port = edge_info.opposite_port;
        graph.port_mut(source_port).outgoing_edges.push(edge_info.edge_id);
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<DisconnectedEdge>();
    }
}
