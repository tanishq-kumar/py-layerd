use hashbrown::HashMap;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::{Alignment, PortConstraints},
    properties::internal::{
        ALIGNMENT, EXT_PORT_REPLACED_DUMMIES, EXT_PORT_REPLACED_DUMMY, EXT_PORT_SIDE,
        IN_LAYER_SUCCESSOR_CONSTRAINTS, PORT_RATIO_OR_POSITION,
    },
};

/// Processes constraints imposed on hierarchical node dummies.
///
/// For eastern/western external port dummies with fixed order, inserts
/// in-layer successor constraints to maintain the correct ordering.
///
/// For northern/southern external port dummies, replaces them with new
/// dummies in adjacent layers.
///
/// Hierarchical-port constraint processor.
pub fn process(graph: &mut LGraph) {
    process_east_west_dummies(graph);
    // N/S processing is a simplified version - we only set up
    // successor constraints if there are external port dummies
    process_north_south_dummies(graph);
}

fn process_east_west_dummies(graph: &mut LGraph) {
    if !graph.options.port_constraints.is_order_fixed() {
        return;
    }

    if graph.layers.is_empty() {
        return;
    }

    let first_layer_idx = 0;
    let last_layer_idx = graph.layers.len() - 1;

    process_east_west_layer(graph, first_layer_idx);
    if first_layer_idx != last_layer_idx {
        process_east_west_layer(graph, last_layer_idx);
    }
}

fn process_east_west_layer(graph: &mut LGraph, layer_idx: usize) {
    let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

    // Sort external port dummies by position
    let mut ext_dummies: Vec<(NodeId, f64)> = Vec::new();
    for &node_id in &nodes {
        if graph.node(node_id).node_type == NodeType::ExternalPort {
            let side = graph.node(node_id).properties.get(&EXT_PORT_SIDE);
            if side == PortSide::West || side == PortSide::East {
                let pos = graph.node(node_id).properties.get(&PORT_RATIO_OR_POSITION);
                ext_dummies.push((node_id, pos));
            }
        }
    }

    ext_dummies.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    // Set successor constraints between consecutive external port dummies
    for i in 0..ext_dummies.len().saturating_sub(1) {
        let current = ext_dummies[i].0;
        let next = ext_dummies[i + 1].0;

        let mut succ = graph.node(current).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
        succ.push(next);
        graph.node_mut(current).properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS, succ);
    }
}

/// Replaces N/S external-port dummies with per-layer stand-ins so that
/// crossing minimization and node placement see them in the correct slot.
fn process_north_south_dummies(graph: &mut LGraph) {
    if !graph.options.port_constraints.is_side_fixed() {
        return;
    }
    if graph.layers.is_empty() {
        return;
    }

    let layer_count = graph.layers.len();
    // Maps indexed by `layer_count + 2` slots (a new layer may be prepended
    // and appended). Index `i` refers to the "layer before original layer
    // `i - 1`", so original layer `k` maps to slot `k + 1`.
    let mut ext_port_to_dummy: Vec<HashMap<NodeId, NodeId>> =
        (0..(layer_count + 2)).map(|_| HashMap::new()).collect();
    let mut new_dummy_nodes: Vec<Vec<NodeId>> =
        (0..(layer_count + 2)).map(|_| Vec::new()).collect();

    let mut original_dummies: Vec<NodeId> = Vec::new();

    for curr_layer_idx in 0..layer_count {
        let nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[curr_layer_idx].nodes);
        for node_id in nodes {
            if is_northern_or_southern_dummy(graph, node_id) {
                // Original N/S dummy: schedule for removal.
                original_dummies.push(node_id);
                continue;
            }

            // Incoming edges from N/S dummies → wire through a replacement in the PREVIOUS layer.
            let incoming: Vec<EdgeId> = graph.incoming_edges(node_id).collect();
            for eid in incoming {
                let src_port = graph.edge(eid).source;
                let src_node = graph.port(src_port).owner;
                if !is_northern_or_southern_dummy(graph, src_node) {
                    continue;
                }
                // Slot `curr_layer_idx` holds the map for the layer BEFORE this one.
                let prev_dummy_id = match ext_port_to_dummy[curr_layer_idx].get(&src_node).copied()
                {
                    Some(id) => id,
                    None => {
                        let new_dummy = create_replacement(graph, src_node);
                        ext_port_to_dummy[curr_layer_idx].insert(src_node, new_dummy);
                        new_dummy_nodes[curr_layer_idx].push(new_dummy);
                        new_dummy
                    }
                };
                // Reroute edge.source to the replacement's output port.
                let prev_dummy_output = dummy_output_port(graph, prev_dummy_id);
                graph.reroute_edge_source(eid, prev_dummy_output);
            }

            // Outgoing edges to N/S dummies → wire through a replacement in the NEXT layer.
            let outgoing: Vec<EdgeId> = graph.outgoing_edges(node_id).collect();
            for eid in outgoing {
                let tgt_port = graph.edge(eid).target;
                let tgt_node = graph.port(tgt_port).owner;
                if !is_northern_or_southern_dummy(graph, tgt_node) {
                    continue;
                }
                // Slot `curr_layer_idx + 2` holds the map for the layer AFTER this one.
                let next_slot = curr_layer_idx + 2;
                let next_dummy_id = match ext_port_to_dummy[next_slot].get(&tgt_node).copied() {
                    Some(id) => id,
                    None => {
                        let new_dummy = create_replacement(graph, tgt_node);
                        ext_port_to_dummy[next_slot].insert(tgt_node, new_dummy);
                        new_dummy_nodes[next_slot].push(new_dummy);
                        new_dummy
                    }
                };
                let next_dummy_input = dummy_input_port(graph, next_dummy_id);
                graph.reroute_edge_target(eid, next_dummy_input);
            }
        }
    }

    // Place newly-created dummies into their destination layer. Prepend /
    // append a fresh `Layer` when slot 0 / last is used.
    for slot in 0..new_dummy_nodes.len() {
        if new_dummy_nodes[slot].is_empty() {
            continue;
        }
        let target_layer_idx: usize = if slot == 0 {
            graph.layers.insert(0, LayerData::new());
            // Every following slot's `- 1` math still works because we only
            // ever prepend once at the start — the loop visits slots in
            // order and `slot == 0` happens first.
            0
        } else if slot == new_dummy_nodes.len() - 1 {
            graph.layers.push(LayerData::new());
            graph.layers.len() - 1
        } else {
            // Intentional correctness fix: a naive `layers.get(i - 1)` read
            // would target the newly inserted empty layer after slot 0
            // prepended one, dropping the middle dummies into the wrong
            // slot. Account for the prepend so middle-slot dummies land in
            // the correct original layer. Only triggers when both slot 0
            // and a middle slot produce dummies (uncommon).
            let prepended = !new_dummy_nodes[0].is_empty() && !graph.layers.is_empty();
            if prepended { slot } else { slot - 1 }
        };
        for &dummy in &new_dummy_nodes[slot] {
            graph.node_mut(dummy).layer = Some(target_layer_idx).into();
            graph.layers[target_layer_idx].nodes.push(dummy);
        }
    }

    // Drop original N/S dummies from their layer assignment; restoration
    // later will reattach them.
    for &original in &original_dummies {
        if let Some(old_layer) = graph.node(original).layer.get()
            && old_layer < graph.layers.len()
        {
            graph.layers[old_layer].nodes.retain(|&n| n != original);
        }
        graph.node_mut(original).layer = None.into();
    }

    // Re-index layers in case a new first layer was inserted.
    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for &nid in &nodes {
            graph.node_mut(nid).layer = Some(layer_idx).into();
        }
    }

    // Record original N/S dummies for the postprocessor to restore.
    if !original_dummies.is_empty() {
        let mut existing = graph.properties.get(&EXT_PORT_REPLACED_DUMMIES);
        existing.extend(original_dummies);
        graph.properties.set(&EXT_PORT_REPLACED_DUMMIES, existing);
    }

    // Per-layer successor constraints between replacement dummies preserve
    // the ordering crossing minimization needs. Sort each new-dummy group
    // by `PORT_RATIO_OR_POSITION` of their original dummy and wire
    // successor constraints head → tail.
    for group in &new_dummy_nodes {
        let mut pairs: Vec<(NodeId, f64)> = group
            .iter()
            .map(|&new_dummy| {
                let orig = graph.node(new_dummy).properties.get(&EXT_PORT_REPLACED_DUMMY);
                let pos = orig
                    .map(|od| graph.node(od).properties.get(&PORT_RATIO_OR_POSITION))
                    .unwrap_or(0.0);
                (new_dummy, pos)
            })
            .collect();
        pairs.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for i in 0..pairs.len().saturating_sub(1) {
            let current = pairs[i].0;
            let next = pairs[i + 1].0;
            let mut succ = graph.node(current).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
            succ.push(next);
            graph.node_mut(current).properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS, succ);
        }
    }
}

fn is_northern_or_southern_dummy(graph: &LGraph, node_id: NodeId) -> bool {
    if graph.node(node_id).node_type != NodeType::ExternalPort {
        return false;
    }
    let side = graph.node(node_id).properties.get(&EXT_PORT_SIDE);
    matches!(side, PortSide::North | PortSide::South)
}

/// Create a replacement external-port dummy with a West input and an East
/// output port, remembering the original via `EXT_PORT_REPLACED_DUMMY`.
fn create_replacement(graph: &mut LGraph, original: NodeId) -> NodeId {
    let new_dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(new_dummy).node_type = NodeType::ExternalPort;
    // Freshly-created nodes land in `layerless_nodes`; we will assign them
    // to a layer shortly, so pull them out first.
    graph.layerless_nodes.retain(|&n| n != new_dummy);

    // Carry over every cold property from the original so downstream
    // processors see the same EXT_PORT_SIDE / PORT_RATIO_OR_POSITION /
    // etc. as before.
    let original_props = graph.node(original).properties.clone();
    graph.node_mut(new_dummy).properties = original_props;

    graph
        .node_mut(new_dummy)
        .properties
        .set(&EXT_PORT_REPLACED_DUMMY, Some(original));
    graph.node_mut(new_dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    // ALIGNMENT=Center so BK treats the replacement as a
    // horizontally-centred external-port slot, same as the original.
    graph.node_mut(new_dummy).properties.set(&ALIGNMENT, Alignment::Center);

    let _input = graph.add_port(new_dummy, PortSide::West);
    let _output = graph.add_port(new_dummy, PortSide::East);
    new_dummy
}

fn dummy_input_port(graph: &LGraph, dummy: NodeId) -> PortId {
    // First port added is the West input.
    graph.node(dummy).ports[0]
}

fn dummy_output_port(graph: &LGraph, dummy: NodeId) -> PortId {
    // Second port added is the East output.
    graph.node(dummy).ports[1]
}
