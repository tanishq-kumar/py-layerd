//! Depth-first model-order layerer.
//!
//! Walks real nodes in `MODEL_ORDER`, placing each as if continuing the
//! current depth-first strip. When a node opens a new strip (no incoming
//! edge to the current layer), drains the deferred `nodes_to_place` queue,
//! then either starts the new strip at layer 0 (no incoming edges) or at
//! the layer after the deepest already-placed predecessor.
//!
//! The "desired layer" of a queued node is kept in a separate
//! `desired_layer` map so that `NodeData.id` retains its dense-index
//! semantics for other consumers.
use hashbrown::HashMap;

use crate::{
    graph::{LGraph, LayerData, index::NodeId, node::NodeType},
    properties::internal::MODEL_ORDER,
};

/// Assign layers using the depth-first model-order heuristic.
pub fn assign_layers(graph: &mut LGraph) {
    let layerless = graph.layerless_nodes.clone();
    if layerless.is_empty() {
        return;
    }

    let mut real_nodes: Vec<NodeId> = layerless
        .iter()
        .copied()
        .filter(|nid| graph.node(*nid).node_type == NodeType::Normal)
        .collect();
    real_nodes.sort_by_key(|nid| {
        let mo = graph.node(*nid).properties.get(&MODEL_ORDER);
        if mo < 0 {
            panic!("DepthFirstModelOrderLayerer requires MODEL_ORDER on every real node");
        }
        mo
    });

    graph.layers.clear();
    graph.layers.push(LayerData::new());
    let mut current_layer_id: usize = 0;
    let mut current_dummy_layer: Option<usize> = None;
    // Deferred-placement queue: node + the layer the node is destined for.
    let mut nodes_to_place: Vec<(NodeId, usize)> = Vec::new();
    // Side-channel for queued nodes' desired layer.
    let mut desired_layer: HashMap<NodeId, usize> = HashMap::new();
    let mut max_to_place: usize = 0;

    for (idx, nid) in real_nodes.iter().copied().enumerate() {
        if idx == 0 {
            graph.layers[current_layer_id].nodes.push(nid);
            graph.node_mut(nid).layer = Some(current_layer_id).into();
            continue;
        }
        let connected = is_connected_to_current_layer(
            graph,
            nid,
            current_layer_id,
            &nodes_to_place,
            &desired_layer,
        );
        if connected {
            let max_layer = get_max_connected_layer(graph, nid, current_layer_id);
            let desired = max_layer + 2;
            let layer_diff = max_layer.saturating_sub(current_layer_id);
            if !nodes_to_place.is_empty() {
                if layer_diff > 0 {
                    let shift = max_layer.saturating_sub(max_to_place);
                    let snapshot: Vec<(NodeId, usize)> = std::mem::take(&mut nodes_to_place);
                    for (queued_nid, queued_layer) in snapshot {
                        let final_layer = queued_layer + shift;
                        desired_layer.insert(queued_nid, final_layer);
                        place_queued(graph, queued_nid, final_layer, &mut current_dummy_layer);
                    }
                    desired_layer.clear();
                    add_node_to_layer(
                        graph,
                        desired,
                        nid,
                        &mut current_layer_id,
                        &mut current_dummy_layer,
                    );
                } else {
                    nodes_to_place.push((nid, desired));
                    desired_layer.insert(nid, desired);
                    if desired > max_to_place {
                        max_to_place = desired;
                    }
                    for eid in graph.incoming_edges(nid).collect::<Vec<_>>() {
                        let src_node = graph.port(graph.edge(eid).source).owner;
                        if graph.node(src_node).layer.is_none()
                            && graph.node(src_node).node_type == NodeType::Label
                            && !desired_layer.contains_key(&src_node)
                        {
                            nodes_to_place.push((src_node, desired - 1));
                            desired_layer.insert(src_node, desired - 1);
                        }
                    }
                    current_layer_id = desired;
                }
            } else {
                add_node_to_layer(
                    graph,
                    desired,
                    nid,
                    &mut current_layer_id,
                    &mut current_dummy_layer,
                );
            }
        } else {
            // Drain any pending queue then start a new strip.
            let snapshot: Vec<(NodeId, usize)> = std::mem::take(&mut nodes_to_place);
            for (queued_nid, queued_layer) in snapshot {
                place_queued(graph, queued_nid, queued_layer, &mut current_dummy_layer);
            }
            desired_layer.clear();
            max_to_place = 0;

            if graph.incoming_edges(nid).next().is_none() {
                nodes_to_place.push((nid, 0));
                desired_layer.insert(nid, 0);
                current_layer_id = 0;
            } else {
                let max_layer = get_max_connected_layer(graph, nid, 0);
                let desired = max_layer + 2;
                add_node_to_layer(
                    graph,
                    desired,
                    nid,
                    &mut current_layer_id,
                    &mut current_dummy_layer,
                );
            }
        }
    }

    // Drain any leftover queue.
    let snapshot: Vec<(NodeId, usize)> = std::mem::take(&mut nodes_to_place);
    for (queued_nid, queued_layer) in snapshot {
        place_queued(graph, queued_nid, queued_layer, &mut current_dummy_layer);
    }

    drop_empty_layers_and_renumber(graph);
    graph.layerless_nodes.clear();
}

fn is_connected_to_current_layer(
    graph: &LGraph,
    nid: NodeId,
    current_layer_id: usize,
    nodes_to_place: &[(NodeId, usize)],
    desired_layer: &HashMap<NodeId, usize>,
) -> bool {
    let queue_empty = nodes_to_place.is_empty();
    for eid in graph.incoming_edges(nid) {
        let src_node = graph.port(graph.edge(eid).source).owner;
        let directly = if queue_empty {
            graph.node(src_node).node_type == NodeType::Normal
                && graph.node(src_node).layer == Some(current_layer_id)
        } else {
            graph.node(src_node).node_type == NodeType::Normal
                && desired_layer.get(&src_node) == Some(&current_layer_id)
        };
        let via_label = graph.node(src_node).node_type == NodeType::Label
            && graph
                .incoming_edges(src_node)
                .next()
                .map(|inner_eid| {
                    let inner_src = graph.port(graph.edge(inner_eid).source).owner;
                    if queue_empty {
                        graph.node(inner_src).layer == Some(current_layer_id)
                    } else {
                        desired_layer.get(&inner_src) == Some(&current_layer_id)
                    }
                })
                .unwrap_or(false);
        if directly || via_label {
            return true;
        }
    }
    false
}

fn get_max_connected_layer(graph: &LGraph, nid: NodeId, base: usize) -> usize {
    // Only consider sources whose layer is already assigned. Do not peek at
    // pending `desired_layer` state — reading it would change the layer
    // chosen for nodes whose predecessor is still queued.
    let mut max_layer = base;
    for eid in graph.incoming_edges(nid) {
        let src_node = graph.port(graph.edge(eid).source).owner;
        if let Some(l) = graph.node(src_node).layer.get()
            && l > max_layer
        {
            max_layer = l;
        }
    }
    max_layer
}

fn add_node_to_layer(
    graph: &mut LGraph,
    layer_id: usize,
    nid: NodeId,
    current_layer_id: &mut usize,
    current_dummy_layer: &mut Option<usize>,
) {
    while graph.layers.len() <= layer_id {
        graph.layers.push(LayerData::new());
    }
    *current_dummy_layer = Some(layer_id.saturating_sub(1));
    *current_layer_id = layer_id;
    graph.layers[layer_id].nodes.push(nid);
    graph.node_mut(nid).layer = Some(layer_id).into();

    let dummy_target = *current_dummy_layer;
    if let Some(d) = dummy_target {
        for eid in graph.incoming_edges(nid).collect::<Vec<_>>() {
            let src_node = graph.port(graph.edge(eid).source).owner;
            if graph.node(src_node).layer.is_none()
                && graph.node(src_node).node_type == NodeType::Label
            {
                graph.layers[d].nodes.push(src_node);
                graph.node_mut(src_node).layer = Some(d).into();
            }
        }
    }
}

fn place_queued(
    graph: &mut LGraph,
    nid: NodeId,
    layer_id: usize,
    _current_dummy_layer: &mut Option<usize>,
) {
    while graph.layers.len() <= layer_id {
        graph.layers.push(LayerData::new());
    }
    if graph.node(nid).layer.is_some() {
        return;
    }
    graph.layers[layer_id].nodes.push(nid);
    graph.node_mut(nid).layer = Some(layer_id).into();
}

fn drop_empty_layers_and_renumber(graph: &mut LGraph) {
    graph.layers.retain(|l| !l.nodes.is_empty());
    let assignments: Vec<(usize, NodeId)> = graph
        .layers
        .iter()
        .enumerate()
        .flat_map(|(li, l)| l.nodes.iter().map(move |&nid| (li, nid)))
        .collect();
    for (li, nid) in assignments {
        graph.node_mut(nid).layer = Some(li).into();
    }
}
