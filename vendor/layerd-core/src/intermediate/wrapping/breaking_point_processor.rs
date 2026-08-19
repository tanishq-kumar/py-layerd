//! Performs the actual wrapping once the breaking-point dummies have been
//! inserted. Runs after phase 3 so that crossing minimization has already
//! committed a node order in each layer.

use smallvec::SmallVec;

use super::{
    breaking_point_info::{
        BPInfo, BPInfoId, BREAKING_POINT_INFO, BREAKING_POINT_INFO_STORE, is_end, is_start,
    },
    cutting_utils,
};
use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId},
        node::NodeType,
    },
    properties::internal::CYCLIC,
};

/// Breaking-point processor entry point.
pub fn process(graph: &mut LGraph) {
    if !has_any_breaking_point(graph) {
        return;
    }

    perform_wrapping(graph);

    if graph.options.wrapping_multi_edge_improve_wrapped_edges {
        assign_layer_indexes(graph);
        improve_multi_cut_index_edges(graph);
        improve_unnecessarily_long_edges(graph, true);
        improve_unnecessarily_long_edges(graph, false);
    }
}

fn has_any_breaking_point(graph: &LGraph) -> bool {
    for layer in &graph.layers {
        for &n in &layer.nodes {
            if graph.node(n).node_type == NodeType::BreakingPoint {
                return true;
            }
        }
    }
    false
}

/// Primary wrapping pass. Prepends an empty layer, then walks left-to-right
/// moving every layer's contents one slot forward unless a `BreakingPoint`
/// start is encountered, at which point the next layer is moved to layer 0
/// (the freshly-prepended empty one) to wrap the drawing.
fn perform_wrapping(graph: &mut LGraph) {
    graph.layers.insert(0, LayerData::new());
    reindex_layers(graph);

    let mut reverse = false;
    let mut idx: usize = 1;
    let mut cursor: usize = 1;

    while cursor < graph.layers.len() {
        // Source layer (where nodes currently live) and target layer (where
        // they should be moved to; lags by one because of the prepended empty).
        let source_layer = cursor;
        let target_layer = idx;

        let nodes_to_move: Vec<NodeId> = graph.layers[source_layer].nodes.to_vec();

        // Move each node to the target layer.
        for &n in &nodes_to_move {
            let at = graph.layers[target_layer].nodes.len();
            graph.insert_node_in_layer(n, target_layer, at);
        }

        if reverse {
            // The first layer after a breaking-point wrap-back: reverse the
            // incoming edges of the nodes we just moved, and for each edge
            // insert the in-layer / long-edge dummy chain described in
            // `CuttingUtils.insertDummies`.
            let offset = nodes_to_move.len();
            let mut reversed_nodes = nodes_to_move.clone();
            reversed_nodes.reverse();
            for n in reversed_nodes {
                let incoming: Vec<EdgeId> = graph.incoming_edges(n).collect();
                for e in incoming {
                    graph.reverse_edge(e);
                    graph.properties.set(&CYCLIC, true);

                    let dummy_edges = cutting_utils::insert_dummies(graph, e, offset);

                    // Amend BPInfo with the in-layer dummies.
                    let bpi: Option<BPInfoId> = graph.node(n).properties.get(&BREAKING_POINT_INFO);
                    if let Some(id) = bpi
                        && let Some(&last_edge) = dummy_edges.last()
                    {
                        let start_in_layer_dummy = graph.port(graph.edge(last_edge).source).owner;
                        let end_in_layer_dummy = graph.port(graph.edge(e).target).owner;
                        let mut store: Vec<BPInfo> =
                            graph.properties.get(&BREAKING_POINT_INFO_STORE);
                        let info = &mut store[id.index()];
                        info.start_in_layer_dummy = Some(start_in_layer_dummy);
                        info.start_in_layer_edge = Some(last_edge);
                        info.end_in_layer_dummy = Some(end_in_layer_dummy);
                        info.end_in_layer_edge = Some(e);
                        graph.properties.set(&BREAKING_POINT_INFO_STORE, store);
                    }
                }
            }
            reverse = false;
        } else if !nodes_to_move.is_empty() {
            let first = nodes_to_move[0];
            if graph.node(first).node_type == NodeType::BreakingPoint {
                reverse = true;
                // The next pass needs to drop back to layer 0 (our prepended one)
                // so set `idx = -1 + 1 = 0` before the `idx += 1` at the bottom.
                idx = 0;
                cursor += 1;
                // Don't increment `idx` when restarting; we want target = 0.
                continue;
            }
        }

        cursor += 1;
        idx += 1;
    }

    drop_empty_layers(graph);
    reindex_layers(graph);
}

fn improve_multi_cut_index_edges(graph: &mut LGraph) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for n in nodes {
            let store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
            if !is_start(n, &graph.node(n).properties, &store) {
                continue;
            }
            let info_id = match graph.node(n).properties.get(&BREAKING_POINT_INFO) {
                Some(id) => id,
                None => continue,
            };
            let mut info = store[info_id.index()];
            if info.prev.is_some() || info.next.is_none() {
                continue;
            }

            let current_id = info_id;
            while let Some(next_id) = info.next {
                let next = {
                    let s: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
                    s[next_id.index()]
                };

                drop_dummies(graph, next.start, next.start_in_layer_dummy, false, true);

                update_indexes_after(graph, info.end);
                update_indexes_after(graph, next.start);
                if let Some(d) = next.start_in_layer_dummy {
                    update_indexes_after(graph, d);
                }
                if let Some(d) = next.end_in_layer_dummy {
                    update_indexes_after(graph, d);
                }

                // Reconnect: next.endInLayerEdge.target = current.endInLayerEdge.target; current.endInLayerEdge.target = null
                let (cur_end_edge, next_end_edge) =
                    (info.end_in_layer_edge, next.end_in_layer_edge);
                if let (Some(cur_e), Some(next_e)) = (cur_end_edge, next_end_edge) {
                    let cur_target = graph.edge(cur_e).target;
                    graph.reroute_edge_target(next_e, cur_target);
                    // Detach the current in-layer edge.
                    let src = graph.edge(cur_e).source;
                    let tgt = graph.edge(cur_e).target;
                    graph.port_mut(src).outgoing_edges.retain(|x| *x != cur_e);
                    graph.port_mut(tgt).incoming_edges.retain(|x| *x != cur_e);
                }

                // Throw out intermediate nodes.
                remove_node_from_layer(graph, info.end);
                remove_node_from_layer(graph, next.start);
                if let Some(d) = next.start_in_layer_dummy {
                    remove_node_from_layer(graph, d);
                }
                if let Some(d) = next.end_in_layer_dummy {
                    remove_node_from_layer(graph, d);
                }

                // Build merged BPInfo.
                let mut new_info = BPInfo::new(
                    info.start,
                    next.end,
                    info.node_start_edge,
                    next.start_end_edge,
                    next.original_edge,
                );
                new_info.start_in_layer_dummy = info.start_in_layer_dummy;
                new_info.start_in_layer_edge = info.start_in_layer_edge;
                new_info.end_in_layer_dummy = info.end_in_layer_dummy;
                new_info.end_in_layer_edge = next.end_in_layer_edge;
                new_info.prev = info.prev;
                new_info.next = next.next;

                // Store merged info under `current_id`, update pointers on nodes.
                let mut store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
                store[current_id.index()] = new_info;
                graph.properties.set(&BREAKING_POINT_INFO_STORE, store);

                graph
                    .node_mut(info.start)
                    .properties
                    .set(&BREAKING_POINT_INFO, Some(current_id));
                graph.node_mut(next.end).properties.set(&BREAKING_POINT_INFO, Some(current_id));

                info = new_info;
            }
        }
    }
}

fn improve_unnecessarily_long_edges(graph: &mut LGraph, forwards: bool) {
    loop {
        let layer_count = graph.layers.len();
        let layer_order: Vec<usize> = if forwards {
            (0..layer_count).rev().collect()
        } else {
            (0..layer_count).collect()
        };

        let mut did_some = false;
        for layer_idx in layer_order {
            let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.to_vec();
            let nodes = if !forwards {
                let mut rev = nodes;
                rev.reverse();
                rev
            } else {
                nodes
            };

            for n in nodes {
                let store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
                let matches = if forwards {
                    is_end(n, &graph.node(n).properties, &store)
                } else {
                    is_start(n, &graph.node(n).properties, &store)
                };
                if !matches {
                    continue;
                }
                let info_id = match graph.node(n).properties.get(&BREAKING_POINT_INFO) {
                    Some(id) => id,
                    None => continue,
                };
                let info = store[info_id.index()];
                let dummy =
                    if forwards { info.end_in_layer_dummy } else { info.start_in_layer_dummy };
                did_some |= drop_dummies(graph, n, dummy, forwards, false);
            }
        }
        if !did_some {
            break;
        }
    }
}

/// Drop adjacent dummy nodes belonging to the breaking-point pair. Returns
/// `true` when at least one pair was dropped.
fn drop_dummies(
    graph: &mut LGraph,
    bp_node: NodeId,
    in_layer_dummy: Option<NodeId>,
    forwards: bool,
    force: bool,
) -> bool {
    let Some(in_layer_dummy) = in_layer_dummy else {
        return false;
    };
    let mut pred_one = next_long_edge_dummy(graph, bp_node, forwards);
    let mut pred_two = next_long_edge_dummy(graph, in_layer_dummy, forwards);

    let mut did_some = false;
    while let (Some(p1), Some(p2)) = (pred_one, pred_two) {
        if !(force || is_adjacent_or_separated_by_bp(graph, p1, p2, forwards)) {
            break;
        }

        let next_one = next_long_edge_dummy(graph, p1, forwards);
        let next_two = next_long_edge_dummy(graph, p2, forwards);

        update_indexes_after(graph, in_layer_dummy);
        update_indexes_after(graph, bp_node);

        let new_layer = graph.node(p1).layer.expect("pred_one must be layered");

        // Remove p1 / p2 via long-edge-joiner semantics (detach ports, merge
        // bend points onto surviving edge).
        join_at_simple(graph, p1);
        join_at_simple(graph, p2);

        let p1_id = graph.node(p1).id as usize;
        let p2_id = graph.node(p2).id as usize;
        if forwards {
            insert_at(graph, in_layer_dummy, new_layer, p2_id);
            graph.node_mut(in_layer_dummy).id = p2_id as u32;
            insert_at(graph, bp_node, new_layer, p1_id + 1);
            graph.node_mut(bp_node).id = p1_id as u32;
        } else {
            insert_at(graph, bp_node, new_layer, p1_id);
            graph.node_mut(bp_node).id = p1_id as u32;
            insert_at(graph, in_layer_dummy, new_layer, p2_id + 1);
            graph.node_mut(in_layer_dummy).id = p2_id as u32;
        }

        // Detach p1 / p2 from any layer completely.
        remove_node_from_layer(graph, p1);
        remove_node_from_layer(graph, p2);

        pred_one = next_one;
        pred_two = next_two;
        did_some = true;
    }
    did_some
}

fn is_adjacent_or_separated_by_bp(graph: &LGraph, d1: NodeId, d2: NodeId, forwards: bool) -> bool {
    let layer = match graph.node(d1).layer.get() {
        Some(l) => l,
        None => return false,
    };
    let (start, end) = if forwards { (d2, d1) } else { (d1, d2) };
    let start_id = graph.node(start).id as usize;
    let end_id = graph.node(end).id as usize;
    let layer_nodes = &graph.layers[layer].nodes;
    for i in (start_id + 1)..end_id {
        if i >= layer_nodes.len() {
            return false;
        }
        let node = layer_nodes[i];
        let t = graph.node(node).node_type;
        if !(t == NodeType::BreakingPoint || is_in_layer_dummy(graph, node)) {
            return false;
        }
    }
    true
}

fn next_long_edge_dummy(graph: &LGraph, start: NodeId, forwards: bool) -> Option<NodeId> {
    let edges: Vec<EdgeId> = if forwards {
        graph.outgoing_edges(start).collect()
    } else {
        graph.incoming_edges(start).collect()
    };
    for e in edges {
        let other_port = if forwards { graph.edge(e).target } else { graph.edge(e).source };
        let other = graph.port(other_port).owner;
        if graph.node(other).node_type == NodeType::LongEdge
            && graph.node(other).layer != graph.node(start).layer
        {
            return Some(other);
        }
    }
    None
}

fn is_in_layer_dummy(graph: &LGraph, node: NodeId) -> bool {
    if graph.node(node).node_type != NodeType::LongEdge {
        return false;
    }
    let own_layer = graph.node(node).layer;
    let ports: Vec<_> = graph.node(node).ports.to_vec();
    for p in ports {
        let incoming: Vec<EdgeId> = graph.port(p).incoming_edges.to_vec();
        for e in incoming {
            let other_port = graph.edge(e).source;
            let other = graph.port(other_port).owner;
            if other != node && graph.node(other).layer == own_layer {
                return true;
            }
        }
        let outgoing: Vec<EdgeId> = graph.port(p).outgoing_edges.to_vec();
        for e in outgoing {
            let other_port = graph.edge(e).target;
            let other = graph.port(other_port).owner;
            if other != node && graph.node(other).layer == own_layer {
                return true;
            }
        }
    }
    false
}

/// Minimal join-at: drop the node's incoming edge and stitch its outgoing
/// edge's target back onto the incoming edge's source. This is the
/// `joinAt(dummy, false)` variant used by `dropDummies`.
fn join_at_simple(graph: &mut LGraph, dummy: NodeId) {
    let ports: Vec<_> = graph.node(dummy).ports.to_vec();
    let mut west: Option<crate::graph::index::PortId> = None;
    let mut east: Option<crate::graph::index::PortId> = None;
    for p in ports {
        match graph.port(p).side {
            crate::graph::port::PortSide::West => west = Some(p),
            crate::graph::port::PortSide::East => east = Some(p),
            _ => {}
        }
    }
    let Some(west) = west else { return };
    let Some(east) = east else { return };

    let input: Vec<EdgeId> = graph.port(west).incoming_edges.to_vec();
    let output: Vec<EdgeId> = graph.port(east).outgoing_edges.to_vec();
    let n = input.len().min(output.len());
    for i in 0..n {
        let survivor = input[i];
        let dropped = output[i];
        let dropped_target = graph.edge(dropped).target;
        graph.reroute_edge_target(survivor, dropped_target);
        // Detach dropped edge.
        let d_src = graph.edge(dropped).source;
        let d_tgt = graph.edge(dropped).target;
        graph.port_mut(d_src).outgoing_edges.retain(|e| *e != dropped);
        graph.port_mut(d_tgt).incoming_edges.retain(|e| *e != dropped);
    }
}

fn insert_at(graph: &mut LGraph, node: NodeId, layer: usize, pos: usize) {
    graph.insert_node_in_layer(node, layer, pos);
}

fn remove_node_from_layer(graph: &mut LGraph, node: NodeId) {
    if let Some(l) = graph.node(node).layer.get()
        && l < graph.layers.len()
    {
        graph.layers[l].nodes.retain(|&n| n != node);
    }
    graph.node_mut(node).layer = None.into();
}

fn update_indexes_after(graph: &mut LGraph, node: NodeId) {
    let layer_idx = match graph.node(node).layer.get() {
        Some(l) => l,
        None => return,
    };
    let pos = graph.node(node).id as usize;
    let layer_len = graph.layers[layer_idx].nodes.len();
    for i in (pos + 1)..layer_len {
        let other = graph.layers[layer_idx].nodes[i];
        if graph.node(other).id > 0 {
            graph.node_mut(other).id -= 1;
        }
    }
}

fn assign_layer_indexes(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.to_vec();
        for (idx, n) in nodes.into_iter().enumerate() {
            graph.node_mut(n).id = idx as u32;
        }
    }
}

fn drop_empty_layers(graph: &mut LGraph) {
    let kept: Vec<LayerData> = std::mem::take(&mut graph.layers)
        .into_iter()
        .filter(|l| !l.nodes.is_empty())
        .collect();
    graph.layers = kept;
}

fn reindex_layers(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        for n in nodes {
            graph.node_mut(n).layer = Some(layer_idx).into();
        }
    }
}
