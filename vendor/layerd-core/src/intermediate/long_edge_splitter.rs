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
    properties::internal::{EDGE_LABEL_PLACEMENT, EDGE_THICKNESS, END_LABEL_EDGE, JUNCTION_POINTS},
};

/// Splits long edges that span more than one layer by inserting dummy nodes.
///
/// After this function runs, every edge in the graph spans exactly one layer
/// (i.e., source layer + 1 == target layer). Dummy nodes have
/// `NodeType::LongEdge` and carry `LONG_EDGE_SOURCE` / `LONG_EDGE_TARGET`
/// properties pointing to the original edge endpoints.
pub fn split(graph: &mut LGraph) {
    if graph.layers.len() <= 2 {
        // With 0, 1, or 2 layers there can be no long edges.
        // (An edge from layer 0 to layer 1 spans exactly 1 layer.)
        return;
    }

    reserve_split_storage(graph);

    // Walk layers in pairs `(layer, next_layer)` from L0..L(len-2). For
    // every node currently in `layer`, split each long outgoing edge by
    // inserting a single dummy into `next_layer`. Dummies appended to a
    // layer become part of that layer's node list when the outer loop
    // reaches it, so chains extend layer by layer rather than all at once.
    for src_layer_idx in 0..(graph.layers.len() - 1) {
        let next_layer_idx = src_layer_idx + 1;
        // Re-read per outer iteration: the previous iteration may have
        // appended dummies to this layer.
        let layer_nodes: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[src_layer_idx].nodes);
        for &node_id in &layer_nodes {
            let ports: SmallVec<PortId, 8> = SmallVec::from_slice_copy(&graph.node(node_id).ports);
            for &port_id in &ports {
                let outgoing: SmallVec<EdgeId, 4> =
                    SmallVec::from_slice_copy(&graph.port(port_id).outgoing_edges);
                for &edge_id in &outgoing {
                    let target_port = graph.edge(edge_id).target;
                    let target_node = graph.port(target_port).owner;
                    let Some(target_layer) = graph.node(target_node).layer.get() else {
                        continue;
                    };
                    if target_layer != src_layer_idx && target_layer != next_layer_idx {
                        let dummy = create_long_edge_dummy(graph, next_layer_idx, edge_id);
                        split_edge_with_dummy(graph, edge_id, dummy);
                    }
                }
            }
        }
    }
}

fn reserve_split_storage(graph: &mut LGraph) {
    let (dummy_count, layer_additions) = count_long_edge_dummies(graph);
    if dummy_count == 0 {
        return;
    }

    let (nodes, ports, edges, _labels, layers) = graph.split_all();
    nodes.reserve(dummy_count);
    ports.reserve(dummy_count.saturating_mul(2));
    edges.reserve(dummy_count);
    for (layer_idx, additional_nodes) in layer_additions.into_iter().enumerate() {
        if additional_nodes != 0 {
            layers[layer_idx].nodes.reserve(additional_nodes);
        }
    }
}

fn count_long_edge_dummies(graph: &LGraph) -> (usize, Vec<usize>) {
    let mut dummy_count = 0usize;
    let mut layer_delta = vec![0isize; graph.layers.len() + 1];

    for src_layer_idx in 0..(graph.layers.len() - 1) {
        let next_layer_idx = src_layer_idx + 1;
        for &node_id in &graph.layers[src_layer_idx].nodes {
            for &port_id in &graph.node(node_id).ports {
                for &edge_id in &graph.port(port_id).outgoing_edges {
                    let target_port = graph.edge(edge_id).target;
                    let target_node = graph.port(target_port).owner;
                    let Some(target_layer) = graph.node(target_node).layer.get() else {
                        continue;
                    };
                    if target_layer <= next_layer_idx {
                        continue;
                    }

                    let dummies_for_edge = target_layer - next_layer_idx;
                    dummy_count += dummies_for_edge;
                    layer_delta[next_layer_idx] += 1;
                    layer_delta[target_layer] -= 1;
                }
            }
        }
    }

    let mut running = 0isize;
    let mut layer_additions = vec![0usize; graph.layers.len()];
    for (layer_idx, additional) in layer_additions.iter_mut().enumerate() {
        running += layer_delta[layer_idx];
        *additional = running as usize;
    }

    (dummy_count, layer_additions)
}

/// Creates a fresh `LongEdge` dummy in `target_layer`, attaches it to the
/// graph, and pins `ORIGIN_EDGE` plus `PORT_CONSTRAINTS=FIXED_POS`.
fn create_long_edge_dummy(
    graph: &mut LGraph,
    target_layer: usize,
    edge_to_split: EdgeId,
) -> NodeId {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::LongEdge;
    graph.node_mut(dummy).origin_edge = Some(edge_to_split);
    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.layerless_nodes.retain(|&n| n != dummy);
    graph.node_mut(dummy).layer = Some(target_layer).into();
    graph.layers[target_layer].nodes.push(dummy);
    dummy
}

/// Rewires `edge` so it terminates at `dummy_node`'s newly-created West
/// input, then creates a dummy edge from `dummy_node`'s East output to the
/// original target. Sets `LONG_EDGE_SOURCE` / `LONG_EDGE_TARGET` on the
/// dummy and migrates HEAD labels.
fn split_edge_with_dummy(graph: &mut LGraph, edge_id: EdgeId, dummy_node: NodeId) {
    // EDGE_THICKNESS: negative is clamped to 0 on the original edge; the
    // dummy node's height matches the thickness and each port sits at
    // floor(thickness / 2).
    let mut thickness = graph.edge(edge_id).properties.get(&EDGE_THICKNESS);
    if thickness < 0.0 {
        thickness = 0.0;
        graph.edge_mut(edge_id).properties.set(&EDGE_THICKNESS, 0.0);
    }
    graph.node_mut(dummy_node).size.y = thickness;
    let port_pos_y = (thickness / 2.0).floor();

    let dummy_in = graph.add_port(dummy_node, PortSide::West);
    let dummy_out = graph.add_port(dummy_node, PortSide::East);
    graph.port_mut(dummy_in).position.y = port_pos_y;
    graph.port_mut(dummy_out).position.y = port_pos_y;

    let prev_target = graph.edge(edge_id).target;
    let cloned_properties = graph.edge(edge_id).properties.clone();
    let cloned_flags = graph.edge(edge_id).flags;

    graph.port_mut(prev_target).incoming_edges.retain(|e| *e != edge_id);
    let dummy_owner = graph.port_owner(dummy_in);
    let edge = graph.edge_mut(edge_id);
    edge.target = dummy_in;
    edge.target_owner = dummy_owner;
    graph.port_mut(dummy_in).incoming_edges.push(edge_id);

    // copyProperties + clear JUNCTION_POINTS so the dummy edge starts
    // fresh.
    let new_edge = graph.add_edge(dummy_out, prev_target);
    graph.edge_mut(new_edge).properties = cloned_properties;
    graph.edge_mut(new_edge).flags = cloned_flags;
    graph.edge_mut(new_edge).properties.remove(&JUNCTION_POINTS);

    set_dummy_node_long_edge_properties(graph, dummy_node, edge_id, new_edge);
    move_head_labels(graph, edge_id, new_edge);
}

/// Sets `LONG_EDGE_SOURCE` / `LONG_EDGE_TARGET` on `dummy_node`. Handles
/// the four branches that depend on whether the in-edge's source or
/// out-edge's target is a `LongEdge` / `Label` / normal node.
fn set_dummy_node_long_edge_properties(
    graph: &mut LGraph,
    dummy_node: NodeId,
    in_edge: EdgeId,
    out_edge: EdgeId,
) {
    let in_source_port = graph.edge(in_edge).source;
    let in_source_node = graph.port(in_source_port).owner;
    let out_target_port = graph.edge(out_edge).target;
    let out_target_node = graph.port(out_target_port).owner;

    let in_source_type = graph.node(in_source_node).node_type;
    let out_target_type = graph.node(out_target_node).node_type;

    if in_source_type == NodeType::LongEdge {
        let src = graph.node(in_source_node).long_edge_source;
        let tgt = graph.node(in_source_node).long_edge_target;
        let has_label = graph.node(in_source_node).long_edge_has_label_dummies;
        graph.node_mut(dummy_node).long_edge_source = src;
        graph.node_mut(dummy_node).long_edge_target = tgt;
        graph.node_mut(dummy_node).long_edge_has_label_dummies = has_label;
    } else if in_source_type == NodeType::Label {
        let src = graph.node(in_source_node).long_edge_source;
        let tgt = graph.node(in_source_node).long_edge_target;
        graph.node_mut(dummy_node).long_edge_source = src;
        graph.node_mut(dummy_node).long_edge_target = tgt;
        graph.node_mut(dummy_node).long_edge_has_label_dummies = true;
    } else if out_target_type == NodeType::Label {
        let src = graph.node(out_target_node).long_edge_source;
        let tgt = graph.node(out_target_node).long_edge_target;
        graph.node_mut(dummy_node).long_edge_source = src;
        graph.node_mut(dummy_node).long_edge_target = tgt;
        graph.node_mut(dummy_node).long_edge_has_label_dummies = true;
    } else {
        graph.node_mut(dummy_node).long_edge_source = Some(in_source_port);
        graph.node_mut(dummy_node).long_edge_target = Some(out_target_port);
    }
}

/// Moves HEAD-placement labels from `old_edge` to `new_edge`, setting
/// `END_LABEL_EDGE` on each label so the label-restore pass knows which
/// edge the label originally belonged to.
fn move_head_labels(graph: &mut LGraph, old_edge: EdgeId, new_edge: EdgeId) {
    let label_ids: Vec<LabelId> = graph.edge(old_edge).labels.to_vec();
    let mut kept: Vec<LabelId> = Vec::with_capacity(label_ids.len());
    let mut moved: Vec<LabelId> = Vec::new();
    for label_id in label_ids {
        let placement = graph.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT);
        if placement == EdgeLabelPlacement::Head {
            if graph.label(label_id).properties.get(&END_LABEL_EDGE).is_none() {
                graph.label_mut(label_id).properties.set(&END_LABEL_EDGE, Some(old_edge));
            }
            moved.push(label_id);
        } else {
            kept.push(label_id);
        }
    }
    graph.edge_mut(old_edge).labels = SmallVec::from_vec(kept);
    for label_id in moved {
        graph.edge_mut(new_edge).labels.push(label_id);
    }
}
