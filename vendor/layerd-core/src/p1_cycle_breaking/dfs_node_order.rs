use hashbrown::HashMap;

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

/// Break cycles using depth-first search with model-order iteration order.
///
/// Traverses outgoing edges in ascending target model order. Any edge that
/// leads back to a currently active node (on the DFS stack) is a back edge
/// and is reversed.
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
    let mut sources: Vec<usize> = Vec::new();
    for (i, &nid) in node_ids.iter().enumerate() {
        if graph.incoming_edges(nid).next().is_none() {
            sources.push(i);
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
    let mut active = vec![false; n];
    let mut edges_to_reverse: Vec<EdgeId> = Vec::new();

    for &src in &sources {
        dfs_iterative(
            src,
            graph,
            &node_ids,
            &id_to_idx,
            &outgoing,
            &mut visited,
            &mut active,
            &mut edges_to_reverse,
            group_model_order,
            max_model_order,
        );
    }

    for i in 0..n {
        if !visited[i] {
            dfs_iterative(
                i,
                graph,
                &node_ids,
                &id_to_idx,
                &outgoing,
                &mut visited,
                &mut active,
                &mut edges_to_reverse,
                group_model_order,
                max_model_order,
            );
        }
    }

    if !edges_to_reverse.is_empty() {
        graph.properties.set(&CYCLIC, true);
    }
    for eid in edges_to_reverse {
        graph.reverse_edge_adapt_ports(eid);
    }
}

#[allow(clippy::too_many_arguments)]
fn dfs_iterative(
    i: usize,
    graph: &LGraph,
    node_ids: &[NodeId],
    id_to_idx: &HashMap<NodeId, usize>,
    outgoing: &[Vec<(NodeId, EdgeId)>],
    visited: &mut [bool],
    active: &mut [bool],
    edges_to_reverse: &mut Vec<EdgeId>,
    group_model_order: bool,
    max_model_order: i32,
) {
    if visited[i] {
        return;
    }

    let mut stack = vec![DfsFrame {
        idx: i,
        buckets: ordered_edge_buckets(i, graph, outgoing, group_model_order, max_model_order),
        next: 0,
    }];
    visited[i] = true;
    active[i] = true;

    while let Some(frame) = stack.last_mut() {
        if frame.next >= frame.buckets.len() {
            active[frame.idx] = false;
            stack.pop();
            continue;
        }

        let bucket = &frame.buckets[frame.next];
        frame.next += 1;
        let repr = bucket[0];
        let target_port = graph.edge(repr).target;
        let target = graph.port(target_port).owner;
        let nid = node_ids[frame.idx];
        if target == nid {
            continue;
        }
        let Some(&j) = id_to_idx.get(&target) else {
            continue;
        };
        if active[j] {
            edges_to_reverse.extend(bucket.iter().copied());
        } else if !visited[j] {
            visited[j] = true;
            active[j] = true;
            stack.push(DfsFrame {
                idx: j,
                buckets: ordered_edge_buckets(
                    j,
                    graph,
                    outgoing,
                    group_model_order,
                    max_model_order,
                ),
                next: 0,
            });
        }
    }
}

struct DfsFrame {
    idx: usize,
    buckets: Vec<Vec<EdgeId>>,
    next: usize,
}

fn ordered_edge_buckets(
    i: usize,
    graph: &LGraph,
    outgoing: &[Vec<(NodeId, EdgeId)>],
    group_model_order: bool,
    max_model_order: i32,
) -> Vec<Vec<EdgeId>> {
    // Missing-MO buckets are keyed on `i32::MAX - model_order_map.len()`
    // *at put-time* — the running map size, not a separate missing-only
    // counter. This guarantees they sort last while preserving insertion
    // order between missing-MO siblings.
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
    ordered_keys
        .into_iter()
        .map(|key| model_order_map.remove(&key).unwrap())
        .collect()
}
