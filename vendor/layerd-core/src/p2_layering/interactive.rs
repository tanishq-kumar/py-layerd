//! Interactive layering.
//!
//! Uses the pre-existing x coordinates of each node to group them into
//! overlapping horizontal spans — each span becomes a layer. Once the
//! initial layering is built, a topology correction pass shifts target
//! nodes into later layers whenever an edge would otherwise violate the
//! layer order.

use hashbrown::HashSet;

use crate::graph::{LGraph, LayerData, index::NodeId};

/// Assign layers by clustering nodes along their existing x coordinates.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }

    // Build layer spans. `spans` is kept sorted by `start` ascending and
    // scanned left-to-right.
    let mut spans: Vec<Span> = Vec::new();
    for &nid in &nodes {
        let node = graph.node(nid);
        let min_x = node.position.x;
        // Force every node to have nonzero width so nodes lining up
        // exactly land in the same span.
        let max_x = (min_x + node.size.x).max(min_x + 1.0);

        let mut found: Option<usize> = None;
        let mut i = 0;
        while i < spans.len() {
            if spans[i].start >= max_x {
                break;
            }
            if spans[i].end > min_x {
                if let Some(f) = found {
                    // Merge span `i` into `f`, then remove `i`.
                    let (left, right) = spans.split_at_mut(i);
                    left[f].nodes.append(&mut right[0].nodes);
                    left[f].end = left[f].end.max(right[0].end);
                    spans.remove(i);
                    // Don't bump `i` — the list shrank by one.
                } else {
                    spans[i].nodes.push(nid);
                    spans[i].start = spans[i].start.min(min_x);
                    spans[i].end = spans[i].end.max(max_x);
                    found = Some(i);
                    i += 1;
                }
            } else {
                i += 1;
            }
        }
        if found.is_none() {
            let insert_at = spans.iter().position(|s| s.start >= max_x).unwrap_or(spans.len());
            spans.insert(insert_at, Span { start: min_x, end: max_x, nodes: vec![nid] });
        }
    }

    // Materialise layers.
    graph.layers.clear();
    for span in &spans {
        let mut layer = LayerData::new();
        for &nid in &span.nodes {
            layer.nodes.push(nid);
            graph.node_mut(nid).layer = Some(graph.layers.len()).into();
        }
        graph.layers.push(layer);
    }

    // Topology correction: for every original node, check that each
    // outgoing edge goes to a later layer; shift targets if not.
    let mut checked: HashSet<NodeId> = HashSet::new();
    for &nid in &nodes {
        if checked.contains(&nid) {
            continue;
        }
        let mut frontier: Vec<NodeId> = vec![nid];
        while let Some(n) = frontier.pop() {
            checked.insert(n);
            let shifted = check_node(graph, n);
            for s in shifted {
                frontier.push(s);
            }
        }
    }

    // Drop any empty layers that the correction pass left behind.
    graph.layers.retain(|l| !l.nodes.is_empty());
    // Re-stamp layer indices on nodes after the retain.
    let stamps: Vec<(NodeId, usize)> = graph
        .layers
        .iter()
        .enumerate()
        .flat_map(|(idx, layer)| layer.nodes.iter().map(move |&nid| (nid, idx)))
        .collect();
    for (nid, idx) in stamps {
        graph.node_mut(nid).layer = Some(idx).into();
    }

    graph.layerless_nodes.clear();
}

fn check_node(graph: &mut LGraph, nid: NodeId) -> Vec<NodeId> {
    let mut shifted: Vec<NodeId> = Vec::new();
    let Some(layer_idx) = graph.node(nid).layer.get() else {
        return shifted;
    };

    let edges: Vec<_> = graph.outgoing_edges(nid).collect();
    let outgoing: Vec<NodeId> = edges
        .into_iter()
        .map(|eid| graph.port(graph.edge(eid).target).owner)
        .filter(|&tgt| tgt != nid)
        .collect();

    for tgt in outgoing {
        let tgt_layer = graph.node(tgt).layer.unwrap_or(usize::MAX);
        if tgt_layer <= layer_idx {
            let new_idx = layer_idx + 1;
            if new_idx == graph.layers.len() {
                graph.layers.push(LayerData::new());
            }
            // Remove `tgt` from its current layer.
            let old_layer = graph.node(tgt).layer.unwrap();
            graph.layers[old_layer].nodes.retain(|&n| n != tgt);
            graph.layers[new_idx].nodes.push(tgt);
            graph.node_mut(tgt).layer = Some(new_idx).into();
            shifted.push(tgt);
        }
    }
    shifted
}

struct Span {
    start: f64,
    end: f64,
    nodes: Vec<NodeId>,
}
