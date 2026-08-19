use std::collections::VecDeque;

use hashbrown::{HashMap, HashSet};

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    options::enums::GroupOrderStrategy,
    properties::internal::{
        CB_CYCLE_BREAKING_ID, CB_GROUP_ORDER_STRATEGY, CYCLIC, MAX_MODEL_ORDER_NODES, MODEL_ORDER,
    },
};

/// Break cycles using breadth-first search with model-order iteration.
///
/// Traverses the graph breadth-first from every source node, visiting
/// outgoing edges in ascending model-order of the target. Any edge whose
/// target has already been visited becomes a "back edge" and is reversed —
/// with the guard that a back edge is *not* reversed if the source is a
/// source node or the target is a sink node.
pub fn break_cycles(graph: &mut LGraph) {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    let n = node_ids.len();
    if n == 0 {
        return;
    }

    let mut id_to_idx: HashMap<NodeId, usize> = HashMap::with_capacity(n);
    for (i, &nid) in node_ids.iter().enumerate() {
        id_to_idx.insert(nid, i);
    }

    let mut outgoing: Vec<Vec<(NodeId, EdgeId)>> = vec![Vec::new(); n];
    let mut sources: HashSet<usize> = HashSet::new();
    let mut sinks: HashSet<usize> = HashSet::new();
    for (i, &nid) in node_ids.iter().enumerate() {
        let has_incoming = graph.incoming_edges(nid).next().is_some();
        let has_outgoing = graph.outgoing_edges(nid).next().is_some();
        if !has_incoming {
            sources.insert(i);
        }
        if !has_outgoing {
            sinks.insert(i);
        }
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target = graph.port(target_port).owner;
            outgoing[i].push((target, eid));
        }
    }

    let group_model_order =
        graph.properties.get(&CB_GROUP_ORDER_STRATEGY) == GroupOrderStrategy::Enforced;
    let max_model_order = graph.properties.get(&MAX_MODEL_ORDER_NODES);

    let mut visited = vec![false; n];
    let mut bfs_queue: VecDeque<usize> = VecDeque::new();
    let mut edges_to_reverse: Vec<EdgeId> = Vec::new();

    // Start BFS from every source node so sources are processed first.
    let source_indices: Vec<usize> = sources.iter().copied().collect();
    for src in source_indices {
        bfs_queue.push_back(src);
        drain_bfs_queue(
            graph,
            &node_ids,
            &id_to_idx,
            &outgoing,
            &sources,
            &sinks,
            &mut visited,
            &mut bfs_queue,
            &mut edges_to_reverse,
            group_model_order,
            max_model_order,
        );
    }

    // Any unvisited node is part of a cycle.
    loop {
        let mut started = false;
        for (i, seen) in visited.iter().copied().enumerate().take(n) {
            if !seen {
                bfs_queue.push_back(i);
                started = true;
                break;
            }
        }
        if !started {
            break;
        }
        drain_bfs_queue(
            graph,
            &node_ids,
            &id_to_idx,
            &outgoing,
            &sources,
            &sinks,
            &mut visited,
            &mut bfs_queue,
            &mut edges_to_reverse,
            group_model_order,
            max_model_order,
        );
    }

    if !edges_to_reverse.is_empty() {
        graph.properties.set(&CYCLIC, true);
    }
    for eid in edges_to_reverse {
        graph.reverse_edge_adapt_ports(eid);
    }
}

#[allow(clippy::too_many_arguments)]
fn drain_bfs_queue(
    graph: &LGraph,
    node_ids: &[NodeId],
    id_to_idx: &HashMap<NodeId, usize>,
    outgoing: &[Vec<(NodeId, EdgeId)>],
    sources: &HashSet<usize>,
    sinks: &HashSet<usize>,
    visited: &mut [bool],
    bfs_queue: &mut VecDeque<usize>,
    edges_to_reverse: &mut Vec<EdgeId>,
    group_model_order: bool,
    max_model_order: i32,
) {
    while let Some(i) = bfs_queue.pop_front() {
        if visited[i] {
            continue;
        }
        visited[i] = true;
        let nid = node_ids[i];

        // Bucket outgoing edges by target model order. Edges whose target
        // lacks a MODEL_ORDER property land in the tail slots
        // (`i32::MAX - model_order_map.len()` *at put-time*), guaranteeing
        // they sort last. The key uses the size of the map *as it grows*,
        // including any already-bucketed with-MO entries — so the offset
        // is not equivalent to a separate "missing" counter.
        let mut model_order_map: HashMap<i32, Vec<EdgeId>> = HashMap::new();
        for &(target, eid) in &outgoing[i] {
            if !graph.node(target).properties.has(&MODEL_ORDER) {
                let key = i32::MAX - model_order_map.len() as i32;
                model_order_map.entry(key).or_default().push(eid);
            } else {
                let target_mo = if group_model_order {
                    max_model_order * graph.node(target).properties.get(&CB_CYCLE_BREAKING_ID)
                        + graph.node(target).properties.get(&MODEL_ORDER)
                } else {
                    graph.node(target).properties.get(&MODEL_ORDER)
                };
                model_order_map.entry(target_mo).or_default().push(eid);
            }
        }

        let mut ordered_keys: Vec<i32> = model_order_map.keys().copied().collect();
        ordered_keys.sort_unstable();

        for key in ordered_keys {
            let bucket = &model_order_map[&key];
            // Pick the first edge from the bucket to examine its target. All
            // edges in the bucket sit in the same logical
            // bucket, but two nodes can collide on `missing_counter`, so
            // inspect the first edge to recover the target node.
            let repr = bucket[0];
            let target_port = graph.edge(repr).target;
            let target = graph.port(target_port).owner;
            if target == nid {
                continue;
            }
            let Some(&j) = id_to_idx.get(&target) else {
                continue;
            };
            if visited[j] && !sources.contains(&i) && !sinks.contains(&j) {
                edges_to_reverse.extend(bucket.iter().copied());
            } else {
                bfs_queue.push_back(j);
            }
        }
    }
}
