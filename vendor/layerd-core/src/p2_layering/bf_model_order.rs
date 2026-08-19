//! Breadth-first model-order layering.
//!
//! Sort real nodes by `MODEL_ORDER`, walk them in order; when a candidate
//! connects (directly or via a label dummy) to the current layer, open a new
//! dummy + real layer pair and place the candidate in the newly-opened real
//! layer. Otherwise place in the current layer. Pre-condition: every real
//! node has a `MODEL_ORDER` property set.
use crate::{
    graph::{LGraph, LayerData, index::NodeId, node::NodeType},
    properties::internal::MODEL_ORDER,
};

/// Assign layers using the breadth-first model-order heuristic.
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
            panic!("BreadthFirstModelOrderLayerer requires MODEL_ORDER on every real node");
        }
        mo
    });

    graph.layers.clear();
    graph.layers.push(LayerData::new());
    let mut current_layer = 0usize;
    let mut current_dummy_layer: Option<usize> = None;

    for (idx, nid) in real_nodes.iter().copied().enumerate() {
        if idx == 0 {
            graph.layers[current_layer].nodes.push(nid);
            graph.node_mut(nid).layer = Some(current_layer).into();
            continue;
        }
        let connects = incoming_connects_to_layer(graph, nid, current_layer);
        if connects {
            current_dummy_layer = Some(graph.layers.len());
            graph.layers.push(LayerData::new());
            current_layer = graph.layers.len();
            graph.layers.push(LayerData::new());
        }
        if let Some(d) = current_dummy_layer {
            for eid in graph.incoming_edges(nid).collect::<Vec<_>>() {
                let src_node = graph.port(graph.edge(eid).source).owner;
                if graph.node(src_node).node_type == NodeType::Label
                    && graph.node(src_node).layer.is_none()
                {
                    graph.layers[d].nodes.push(src_node);
                    graph.node_mut(src_node).layer = Some(d).into();
                }
            }
        }
        graph.layers[current_layer].nodes.push(nid);
        graph.node_mut(nid).layer = Some(current_layer).into();
    }

    drop_empty_layers_and_renumber(graph);
    graph.layerless_nodes.clear();
}

fn incoming_connects_to_layer(graph: &LGraph, nid: NodeId, current_layer: usize) -> bool {
    for eid in graph.incoming_edges(nid) {
        let src_node = graph.port(graph.edge(eid).source).owner;
        let direct = graph.node(src_node).node_type == NodeType::Normal
            && graph.node(src_node).layer == Some(current_layer);
        let via_label = graph.node(src_node).node_type == NodeType::Label
            && graph
                .incoming_edges(src_node)
                .next()
                .map(|inner_eid| {
                    let inner_src = graph.port(graph.edge(inner_eid).source).owner;
                    graph.node(inner_src).layer == Some(current_layer)
                })
                .unwrap_or(false);
        if direct || via_label {
            return true;
        }
    }
    false
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
