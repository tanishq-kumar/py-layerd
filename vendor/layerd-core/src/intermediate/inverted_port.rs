use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::{EdgeLabelPlacement, PortConstraints},
    properties::internal::{EDGE_LABEL_PLACEMENT, END_LABEL_EDGE, JUNCTION_POINTS},
};

/// Handles "inverted" ports: West-side ports with outgoing edges or East-side
/// ports with incoming edges.
///
/// Creates `LongEdge` dummy nodes in the same layer and reroutes edges through
/// them, so all edges flow left-to-right between layers.
///
/// Inverted-port processor.
pub fn process(graph: &mut LGraph) {
    // Process layers from left to right. Dummies created in layer i are
    // assigned to layer i (same layer as the node with the inverted port).
    for layer_idx in 0..graph.layers.len() {
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        let mut unassigned: Vec<NodeId> = Vec::new();

        for &node_id in &node_ids {
            // Skip dummy nodes
            if graph.node(node_id).node_type != NodeType::Normal {
                continue;
            }

            // Look for input ports on the East side (inverted: incoming from east)
            let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
            for &port_id in &ports {
                if graph.port(port_id).side != PortSide::East {
                    continue;
                }
                let incoming: Vec<EdgeId> = graph.port(port_id).incoming_edges.to_vec();
                for edge_id in incoming {
                    // Skip self-loops
                    let src_port = graph.edge(edge_id).source;
                    let src_node = graph.port(src_port).owner;
                    if src_node == node_id {
                        continue;
                    }
                    create_east_port_dummy(graph, port_id, edge_id, &mut unassigned);
                }
            }

            // Look for output ports on the West side (inverted: outgoing from west)
            for &port_id in &ports {
                if graph.port(port_id).side != PortSide::West {
                    continue;
                }
                let outgoing: Vec<EdgeId> = graph.port(port_id).outgoing_edges.to_vec();
                for edge_id in outgoing {
                    // Skip self-loops
                    let tgt_port = graph.edge(edge_id).target;
                    let tgt_node = graph.port(tgt_port).owner;
                    if tgt_node == node_id {
                        continue;
                    }
                    create_west_port_dummy(graph, port_id, edge_id, &mut unassigned);
                }
            }
        }

        // Assign unassigned dummy nodes to the current layer
        for dummy in unassigned {
            graph.node_mut(dummy).layer = Some(layer_idx).into();
            graph.layers[layer_idx].nodes.push(dummy);
        }
    }
}

/// Creates a dummy for an east-side input port.
/// The original edge is rerouted to the dummy's input, and a new edge goes
/// from the dummy's output to the original east port.
fn create_east_port_dummy(
    graph: &mut LGraph,
    east_port: PortId,
    edge_id: EdgeId,
    unassigned: &mut Vec<NodeId>,
) {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::LongEdge;
    // dummy.set(ORIGIN, edge) and PORT_CONSTRAINTS = FIXED_POS.
    graph.node_mut(dummy).origin_edge = Some(edge_id);
    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.layerless_nodes.retain(|&n| n != dummy);

    let dummy_input = graph.add_port(dummy, PortSide::West);
    let dummy_output = graph.add_port(dummy, PortSide::East);

    // Reroute the original edge to target the dummy's input
    graph.reroute_edge_target(edge_id, dummy_input);

    // Create a new edge from the dummy's output to the original east port
    let new_edge = graph.add_edge(dummy_output, east_port);

    // Copy properties from the original edge to the new dummy edge, then
    // reset JUNCTION_POINTS.
    copy_edge_properties(graph, edge_id, new_edge);
    graph
        .edge_mut(new_edge)
        .properties
        .set(&JUNCTION_POINTS, smallvec::SmallVec::new());

    // Move HEAD labels from the old edge to the new dummy edge, remembering
    // the original edge via END_LABEL_EDGE so postprocessing can reattach
    // them.
    move_head_labels(graph, edge_id, new_edge);

    // Set LONG_EDGE_SOURCE and LONG_EDGE_TARGET
    set_long_edge_properties(graph, dummy, dummy_input, dummy_output);

    unassigned.push(dummy);
}

/// Creates a dummy for a west-side output port.
/// The original edge is rerouted to target the dummy's input, and a new edge
/// goes from the dummy's output to the original target.
fn create_west_port_dummy(
    graph: &mut LGraph,
    _west_port: PortId,
    edge_id: EdgeId,
    unassigned: &mut Vec<NodeId>,
) {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::LongEdge;
    graph.node_mut(dummy).origin_edge = Some(edge_id);
    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.layerless_nodes.retain(|&n| n != dummy);

    let dummy_input = graph.add_port(dummy, PortSide::West);
    let dummy_output = graph.add_port(dummy, PortSide::East);

    // Save the original target
    let original_target = graph.edge(edge_id).target;

    // Reroute the original edge to target the dummy's input
    graph.reroute_edge_target(edge_id, dummy_input);

    // Create a new edge from the dummy's output to the original target
    let new_edge = graph.add_edge(dummy_output, original_target);

    copy_edge_properties(graph, edge_id, new_edge);
    graph
        .edge_mut(new_edge)
        .properties
        .set(&JUNCTION_POINTS, smallvec::SmallVec::new());

    // Move HEAD labels onto the dummy edge.
    move_head_labels(graph, edge_id, new_edge);

    // Set LONG_EDGE_SOURCE and LONG_EDGE_TARGET
    set_long_edge_properties(graph, dummy, dummy_input, dummy_output);

    unassigned.push(dummy);
}

/// Clone edge metadata from `src_edge` into `dst_edge`.
fn copy_edge_properties(graph: &mut LGraph, src_edge: EdgeId, dst_edge: EdgeId) {
    let src_props = graph.edge(src_edge).properties.clone();
    let src_flags = graph.edge(src_edge).flags;
    let dst = graph.edge_mut(dst_edge);
    dst.properties = src_props;
    dst.flags = src_flags;
}

/// Move every HEAD-placed label from `from_edge` onto `to_edge`, tagging the
/// label with `END_LABEL_EDGE = from_edge` so the postprocessor knows where
/// it came from.
fn move_head_labels(graph: &mut LGraph, from_edge: EdgeId, to_edge: EdgeId) {
    let from_labels: Vec<LabelId> = graph.edge(from_edge).labels.iter().copied().collect();
    let mut remaining: SmallVec<LabelId, 2> = SmallVec::new();
    let mut moved: Vec<LabelId> = Vec::new();
    for lid in from_labels {
        let placement = graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT);
        if placement == EdgeLabelPlacement::Head {
            // Remember the original edge on the label if not already set.
            let has_end_label = graph.label(lid).properties.has(&END_LABEL_EDGE);
            if !has_end_label {
                graph.label_mut(lid).properties.set(&END_LABEL_EDGE, Some(from_edge));
            }
            moved.push(lid);
        } else {
            remaining.push(lid);
        }
    }
    if moved.is_empty() {
        return;
    }
    graph.edge_mut(from_edge).labels = remaining;
    for lid in moved {
        graph.edge_mut(to_edge).labels.push(lid);
    }
}

/// Sets the LONG_EDGE_SOURCE and LONG_EDGE_TARGET properties on a dummy node.
fn set_long_edge_properties(
    graph: &mut LGraph,
    dummy: NodeId,
    dummy_input: PortId,
    dummy_output: PortId,
) {
    // Source: follow the incoming edge's source
    let source_port = if let Some(&edge_id) = graph.port(dummy_input).incoming_edges.first() {
        let src = graph.edge(edge_id).source;
        let src_node = graph.port(src).owner;
        if graph.node(src_node).node_type == NodeType::LongEdge {
            graph.node(src_node).long_edge_source
        } else {
            Some(src)
        }
    } else {
        None
    };

    // Target: follow the outgoing edge's target
    let target_port = if let Some(&edge_id) = graph.port(dummy_output).outgoing_edges.first() {
        let tgt = graph.edge(edge_id).target;
        let tgt_node = graph.port(tgt).owner;
        if graph.node(tgt_node).node_type == NodeType::LongEdge {
            graph.node(tgt_node).long_edge_target
        } else {
            Some(tgt)
        }
    } else {
        None
    };

    graph.node_mut(dummy).long_edge_source = source_port;
    graph.node_mut(dummy).long_edge_target = target_port;
}
