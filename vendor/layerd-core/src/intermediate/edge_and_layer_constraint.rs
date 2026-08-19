//! Edge reversal for layer and edge constraints.
//!
//! Reverses edges that violate per-node layer/edge constraints. Layer
//! constraints are read **per node** (not from the graph-level option).
//!
//! Two passes:
//!
//! 1. Outer nodes: nodes with FIRST / LAST / FIRST_SEPARATE / LAST_SEPARATE
//!    get their edges reversed so the node has only outgoing (FIRST*) or
//!    only incoming (LAST*) edges.
//! 2. Inner nodes: nodes without a layer constraint whose port sides are
//!    fixed *and* whose ports are all "reversed" (east with net_flow > 0 or
//!    west with net_flow < 0) get all of their edges reversed too; this
//!    turns feedback nodes into regular nodes the layered pipeline can
//!    handle. Skipped if any connected node has a conflicting layer
//!    constraint.

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{EdgeConstraint, LayerConstraint},
    properties::internal::{LAYER_CONSTRAINT, NODE_EDGE_CONSTRAINT},
};

/// Port flow direction selector passed to `reverse_edges_for_node`.
/// `Undefined` means "reverse both directions".
#[derive(Clone, Copy, PartialEq, Eq)]
enum PortFlow {
    Input,
    Output,
    Undefined,
}

/// Reverses edges based on layer and edge constraints.
pub fn reverse_edges(graph: &mut LGraph) {
    let remaining = handle_outer_nodes(graph);
    handle_inner_nodes(graph, &remaining);
}

/// First pass: reverse edges on nodes with an explicit layer constraint.
/// Returns the nodes that were *not* outer nodes (i.e. no layer constraint).
fn handle_outer_nodes(graph: &mut LGraph) -> Vec<NodeId> {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    let mut remaining = Vec::with_capacity(node_ids.len());

    for node_id in node_ids {
        let lc = graph.node(node_id).properties.get(&LAYER_CONSTRAINT);
        let edge_constraint = match lc {
            LayerConstraint::First | LayerConstraint::FirstSeparate =>
                Some(EdgeConstraint::OutgoingOnly),
            LayerConstraint::Last | LayerConstraint::LastSeparate =>
                Some(EdgeConstraint::IncomingOnly),
            LayerConstraint::None => None,
        };

        match edge_constraint {
            Some(EdgeConstraint::OutgoingOnly) => {
                // Always set EDGE_CONSTRAINT to OutgoingOnly regardless of
                // the layer constraint. The property is set before the
                // reversal so later processors see the post-reversal role.
                graph
                    .node_mut(node_id)
                    .properties
                    .set(&NODE_EDGE_CONSTRAINT, EdgeConstraint::OutgoingOnly);
                reverse_edges_for_node(graph, node_id, lc, PortFlow::Output);
            }
            Some(EdgeConstraint::IncomingOnly) => {
                graph
                    .node_mut(node_id)
                    .properties
                    .set(&NODE_EDGE_CONSTRAINT, EdgeConstraint::OutgoingOnly);
                reverse_edges_for_node(graph, node_id, lc, PortFlow::Input);
            }
            _ => remaining.push(node_id),
        }
    }
    remaining
}

/// Second pass: turn "feedback nodes" into regular nodes.
fn handle_inner_nodes(graph: &mut LGraph, remaining: &[NodeId]) {
    for &node_id in remaining {
        let lc = graph.node(node_id).properties.get(&LAYER_CONSTRAINT);
        // This pass only fires for nodes *without* a layer constraint. The
        // match below duplicates `handle_outer_nodes` for safety: should a
        // later processor ever re-enter here with a constrained node, the
        // correct edge reversal still runs.
        let edge_constraint = match lc {
            LayerConstraint::First | LayerConstraint::FirstSeparate =>
                Some(EdgeConstraint::OutgoingOnly),
            LayerConstraint::Last | LayerConstraint::LastSeparate =>
                Some(EdgeConstraint::IncomingOnly),
            LayerConstraint::None => None,
        };
        if let Some(ec) = edge_constraint {
            graph
                .node_mut(node_id)
                .properties
                .set(&NODE_EDGE_CONSTRAINT, EdgeConstraint::OutgoingOnly);
            let flow = if ec == EdgeConstraint::IncomingOnly {
                PortFlow::Input
            } else {
                PortFlow::Output
            };
            reverse_edges_for_node(graph, node_id, lc, flow);
            continue;
        }

        // Feedback-node detection: port sides fixed + all ports reversed.
        let port_constraints = graph.node(node_id).port_constraints();
        if !port_constraints.is_side_fixed() {
            continue;
        }
        let port_ids: Vec<PortId> = graph.node(node_id).ports.iter().copied().collect();
        if port_ids.is_empty() {
            continue;
        }

        let mut all_reversed = true;
        'outer: for pid in &port_ids {
            let side = graph.port(*pid).side;
            let net_flow = port_net_flow(graph, *pid);
            // A non-reversed east port has net_flow <= 0; a non-reversed
            // west port has net_flow >= 0. Either inverted signals "this
            // port is a feedback port".
            let flow_reversed = (side == PortSide::East && net_flow > 0)
                || (side == PortSide::West && net_flow < 0);
            if !flow_reversed {
                all_reversed = false;
                break;
            }

            // Any outgoing edge landing in a LAST / LAST_SEPARATE node
            // disqualifies the whole reversal.
            for eid in graph.port(*pid).outgoing_edges.iter().copied() {
                let tgt_node = graph.port(graph.edge(eid).target).owner;
                let tgt_lc = graph.node(tgt_node).properties.get(&LAYER_CONSTRAINT);
                if matches!(tgt_lc, LayerConstraint::Last | LayerConstraint::LastSeparate) {
                    all_reversed = false;
                    break 'outer;
                }
            }
            // Any incoming edge originating in a FIRST / FIRST_SEPARATE
            // node disqualifies.
            for eid in graph.port(*pid).incoming_edges.iter().copied() {
                let src_node = graph.port(graph.edge(eid).source).owner;
                let src_lc = graph.node(src_node).properties.get(&LAYER_CONSTRAINT);
                if matches!(src_lc, LayerConstraint::First | LayerConstraint::FirstSeparate) {
                    all_reversed = false;
                    break 'outer;
                }
            }
        }
        if all_reversed {
            reverse_edges_for_node(graph, node_id, lc, PortFlow::Undefined);
        }
    }
}

/// Reverse outgoing / incoming / all edges of a node, subject to the
/// reversal legality checks in `can_reverse_outgoing_edge` and
/// `can_reverse_incoming_edge`.
fn reverse_edges_for_node(
    graph: &mut LGraph,
    node_id: NodeId,
    layer_constraint: LayerConstraint,
    target_flow: PortFlow,
) {
    let port_ids: Vec<PortId> = graph.node(node_id).ports.iter().copied().collect();

    for pid in port_ids {
        // For Input / Undefined: reverse *outgoing* edges so this port
        // ultimately behaves as an input.
        if matches!(target_flow, PortFlow::Input | PortFlow::Undefined) {
            let outgoing: Vec<EdgeId> = graph.port(pid).outgoing_edges.iter().copied().collect();
            for eid in outgoing {
                if can_reverse_outgoing_edge(graph, layer_constraint, eid) {
                    graph.reverse_edge_adapt_ports(eid);
                }
            }
        }
        // For Output / Undefined: reverse *incoming* edges so this port
        // ultimately behaves as an output.
        if matches!(target_flow, PortFlow::Output | PortFlow::Undefined) {
            let incoming: Vec<EdgeId> = graph.port(pid).incoming_edges.iter().copied().collect();
            for eid in incoming {
                if can_reverse_incoming_edge(graph, layer_constraint, eid) {
                    graph.reverse_edge_adapt_ports(eid);
                }
            }
        }
    }
}

/// Whether the edge `edge` outgoing from a node with constraint
/// `source_layer_constraint` may be reversed.
fn can_reverse_outgoing_edge(
    graph: &LGraph,
    source_layer_constraint: LayerConstraint,
    edge: EdgeId,
) -> bool {
    use crate::graph::edge::EdgeFlags;
    if graph.edge(edge).flags.contains(EdgeFlags::REVERSED) {
        return false;
    }
    let tgt_node = graph.port(graph.edge(edge).target).owner;
    // LAST layer → LABEL target would force the label dummy into the
    // LAST_SEPARATE slot; skip.
    if source_layer_constraint == LayerConstraint::Last
        && graph.node(tgt_node).node_type == NodeType::Label
    {
        return false;
    }
    // Target in LAST_SEPARATE: do not reverse.
    let tgt_lc = graph.node(tgt_node).properties.get(&LAYER_CONSTRAINT);
    if tgt_lc == LayerConstraint::LastSeparate {
        return false;
    }
    true
}

/// Whether the edge `edge` incoming to a node with constraint
/// `target_layer_constraint` may be reversed.
fn can_reverse_incoming_edge(
    graph: &LGraph,
    target_layer_constraint: LayerConstraint,
    edge: EdgeId,
) -> bool {
    use crate::graph::edge::EdgeFlags;
    if graph.edge(edge).flags.contains(EdgeFlags::REVERSED) {
        return false;
    }
    let src_node = graph.port(graph.edge(edge).source).owner;
    if target_layer_constraint == LayerConstraint::First
        && graph.node(src_node).node_type == NodeType::Label
    {
        return false;
    }
    let src_lc = graph.node(src_node).properties.get(&LAYER_CONSTRAINT);
    if src_lc == LayerConstraint::FirstSeparate {
        return false;
    }
    true
}

/// Net flow of a port — how many more incoming edges than outgoing.
/// Positive means inbound-heavy.
fn port_net_flow(graph: &LGraph, port_id: PortId) -> i32 {
    let p = graph.port(port_id);
    p.incoming_edges.len() as i32 - p.outgoing_edges.len() as i32
}
