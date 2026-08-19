use std::collections::VecDeque;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    properties::internal::{CYCLIC, PRIORITY_DIRECTION},
};

/// Simulate removing a node from the graph by walking its ports' connected
/// edges (`incoming` first, then `outgoing`). For each edge, the endpoint on
/// the other side has its `indeg` or `outdeg` decreased; if it crosses zero
/// with the right precondition, it joins the sources / sinks queue. The walk
/// order matters because both queues are FIFO — a different visitation order
/// seeds them with a different node sequence, which changes `mark[]` and
/// ultimately which edges get reversed.
fn update_neighbors(
    graph: &LGraph,
    node_id: NodeId,
    id_to_idx: &[usize],
    indeg: &mut [i32],
    outdeg: &mut [i32],
    assigned: &[bool],
    sources: &mut VecDeque<usize>,
    sinks: &mut VecDeque<usize>,
) {
    for &port_id in &graph.node(node_id).ports {
        // Walk incoming edges first, then outgoing — endpoint discovery
        // order is load-bearing for the FIFO queues below.
        let port = graph.port(port_id);
        let incoming: Vec<EdgeId> = port.incoming_edges.iter().copied().collect();
        let outgoing: Vec<EdgeId> = port.outgoing_edges.iter().copied().collect();
        for &edge_id in incoming.iter().chain(outgoing.iter()) {
            let edge = graph.edge(edge_id);
            // The endpoint on the other side of this edge.
            let endpoint_port = if edge.source == port_id { edge.target } else { edge.source };
            let endpoint = graph.port(endpoint_port).owner;
            if endpoint == node_id {
                continue;
            }
            let endpoint_node_id = graph.node(endpoint).id as usize;
            if endpoint_node_id >= id_to_idx.len() {
                continue;
            }
            let j = id_to_idx[endpoint_node_id];
            if j == usize::MAX || assigned[j] {
                continue;
            }
            let mut priority = edge.properties.get(&PRIORITY_DIRECTION);
            if priority < 0 {
                priority = 0;
            }
            let w = priority + 1;
            // Branch on which side the endpoint port sits on: when it's the
            // edge target, the endpoint is the edge's destination, so
            // removing `node_id` (the source side) strips one incoming from
            // `endpoint`. When it's the edge source, the endpoint is the
            // source and removing `node_id` strips one outgoing from it.
            if edge.target == endpoint_port {
                indeg[j] -= w;
                if indeg[j] <= 0 && outdeg[j] > 0 {
                    sources.push_back(j);
                }
            } else {
                outdeg[j] -= w;
                if outdeg[j] <= 0 && indeg[j] > 0 {
                    sinks.push_back(j);
                }
            }
        }
    }
}

/// Break cycles using the Eades-Lin-Smyth greedy feedback arc set heuristic.
///
/// Produces a node ordering such that reversing edges that go against the
/// ordering breaks all cycles. The resulting set of reversed edges is
/// typically small.
pub fn break_cycles(graph: &mut LGraph) {
    let mut rng = graph.take_rng();
    run_greedy_with_tiebreak(graph, |candidates, _node_ids| {
        let idx = rng.next_int(candidates.len() as i32) as usize;
        candidates[idx]
    });
    graph.put_rng(rng);
}

/// Run the Eades-Lin-Smyth feedback arc set algorithm with a caller-supplied
/// tiebreak policy.
///
/// Sinks and sources are kept in FIFO queues; each queue entry is placed
/// exactly once, based on whichever of (`indeg <= 0 && outdeg > 0`) or
/// (`outdeg <= 0 && indeg > 0`) becomes true first. A node with both degrees
/// zero (due to cascading removals) is *not* added to a queue a second time.
///
/// The `tiebreak` closure is invoked whenever more than one unassigned node
/// shares the maximum `outdeg - indeg`. It receives the candidate indices
/// (positions in `node_ids`) together with the `node_ids` slice so it can
/// map indices back to `NodeId`s when needed.
pub(super) fn run_greedy_with_tiebreak<F>(graph: &mut LGraph, mut tiebreak: F)
where
    F: FnMut(&[usize], &[NodeId]) -> usize,
{
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    let n = node_ids.len();
    if n == 0 {
        return;
    }

    // Build a mapping from NodeId to index.
    let max_id = node_ids.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;
    let mut id_to_idx = vec![usize::MAX; max_id + 1];
    for (i, &nid) in node_ids.iter().enumerate() {
        let node_id_val = graph.node(nid).id as usize;
        id_to_idx[node_id_val] = i;
    }

    // Build adjacency: for each node index, list of (neighbor_index, edge_id, weight).
    // Weight is `priority > 0 ? priority + 1 : 1`. Precomputing here avoids
    // re-reading `PRIORITY_DIRECTION` during neighbor updates.
    let mut out_neighbors: Vec<Vec<(usize, EdgeId, i32)>> = vec![Vec::new(); n];
    let mut in_neighbors: Vec<Vec<(usize, EdgeId, i32)>> = vec![Vec::new(); n];
    let mut indeg = vec![0i32; n];
    let mut outdeg = vec![0i32; n];

    for (i, &nid) in node_ids.iter().enumerate() {
        for eid in graph.outgoing_edges(nid) {
            let target_port = graph.edge(eid).target;
            let target_node_id = graph.port(target_port).owner;
            let target_node_id_val = graph.node(target_node_id).id as usize;
            if target_node_id_val < id_to_idx.len() {
                let j = id_to_idx[target_node_id_val];
                if j != usize::MAX && j != i {
                    // Clamp negative priority values to zero before applying
                    // the `priority > 0` weighting rule. Without the clamp,
                    // a negative priority would fall into the `else` branch
                    // and still pick weight 1 — same result today, but the
                    // explicit clamp avoids surprises if the weight formula
                    // changes.
                    let mut priority = graph.edge(eid).properties.get(&PRIORITY_DIRECTION);
                    if priority < 0 {
                        priority = 0;
                    }
                    let weight = if priority > 0 { priority + 1 } else { 1 };
                    out_neighbors[i].push((j, eid, weight));
                    in_neighbors[j].push((i, eid, weight));
                    outdeg[i] += weight;
                    indeg[j] += weight;
                }
            }
        }
    }

    // Initial sink/source classification:
    //   if outdeg == 0 -> sinks
    //   else if indeg == 0 -> sources
    let mut sinks: VecDeque<usize> = VecDeque::new();
    let mut sources: VecDeque<usize> = VecDeque::new();
    for i in 0..n {
        if outdeg[i] == 0 {
            sinks.push_back(i);
        } else if indeg[i] == 0 {
            sources.push_back(i);
        }
    }

    let mut assigned = vec![false; n];
    let mut mark = vec![0i32; n];
    let mut next_left: i32 = 1;
    let mut next_right: i32 = -1;
    let mut remaining = n;

    while remaining > 0 {
        // Drain the sinks queue. Each sink is processed in FIFO discovery
        // order.
        while let Some(i) = sinks.pop_front() {
            if assigned[i] {
                continue;
            }
            assigned[i] = true;
            remaining -= 1;
            mark[i] = next_right;
            next_right -= 1;
            update_neighbors(
                graph,
                node_ids[i],
                &id_to_idx,
                &mut indeg,
                &mut outdeg,
                &assigned,
                &mut sources,
                &mut sinks,
            );
        }

        // Drain the sources queue.
        while let Some(i) = sources.pop_front() {
            if assigned[i] {
                continue;
            }
            assigned[i] = true;
            remaining -= 1;
            mark[i] = next_left;
            next_left += 1;
            update_neighbors(
                graph,
                node_ids[i],
                &id_to_idx,
                &mut indeg,
                &mut outdeg,
                &assigned,
                &mut sources,
                &mut sinks,
            );
        }

        // If nodes remain, pick one with max `outdeg - indeg` via the
        // caller-supplied tiebreak. Neighbor updates use the same degree
        // maintenance as the source and sink cases above.
        if remaining > 0 {
            let mut best_diff = i32::MIN;
            let mut candidates: Vec<usize> = Vec::new();

            for i in 0..n {
                if !assigned[i] {
                    let diff = outdeg[i] - indeg[i];
                    if diff > best_diff {
                        best_diff = diff;
                        candidates.clear();
                        candidates.push(i);
                    } else if diff == best_diff {
                        candidates.push(i);
                    }
                }
            }

            // Always invoke the tiebreak even when there is a single
            // candidate so the RNG advances by one draw. Skipping the call
            // here when `candidates.len() == 1` would desync the RNG for
            // every subsequent multi-candidate
            // tiebreak (and, because the RNG is shared with P3 layer sweep,
            // every random draw thereafter).
            let chosen = tiebreak(&candidates, &node_ids);

            assigned[chosen] = true;
            remaining -= 1;
            mark[chosen] = next_left;
            next_left += 1;

            update_neighbors(
                graph,
                node_ids[chosen],
                &id_to_idx,
                &mut indeg,
                &mut outdeg,
                &assigned,
                &mut sources,
                &mut sinks,
            );
        }
    }

    // Shift negative marks to positive:
    //   shift_base = n + 1;
    //   if mark[index] < 0 { mark[index] += shift_base; }
    //
    // Only negative marks (sinks) are shifted; positive marks (sources) stay
    // put. The resulting ordering is sources-first, sinks-last.
    let shift_base = (n as i32) + 1;
    for m in mark.iter_mut() {
        if *m < 0 {
            *m += shift_base;
        }
    }

    // Reverse edges where mark[source] > mark[target].
    let mut edges_to_reverse = Vec::new();
    for (i, _nid) in node_ids.iter().enumerate() {
        for &(j, eid, _) in &out_neighbors[i] {
            if mark[i] > mark[j] {
                edges_to_reverse.push(eid);
            }
        }
    }

    if !edges_to_reverse.is_empty() {
        graph.properties.set(&CYCLIC, true);
    }
    for eid in edges_to_reverse {
        graph.reverse_edge_adapt_ports(eid);
    }
}
