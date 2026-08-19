use crate::graph::{LGraph, edge::EdgeFlags, index::EdgeId};

/// Restores reversed edges to their original direction after layout is complete.
///
/// During cycle breaking, some edges are reversed to break cycles. This
/// intermediate processor runs after layout to flip those edges back.
///
/// Traverses layers -> nodes -> ports -> outgoing edges, reversing only those
/// tagged REVERSED. Deliberately scopes the scan to the layered body so
/// orphan / layerless edges (e.g. cross-hierarchy proxy edges hidden inside
/// a parent arena) stay untouched.
pub fn restore(graph: &mut LGraph) {
    let mut edges_to_reverse: Vec<EdgeId> = Vec::new();
    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            let ports = graph.node(node_id).ports.clone();
            for port_id in ports {
                let outgoing = graph.port(port_id).outgoing_edges.clone();
                for eid in outgoing {
                    if graph.edge(eid).flags.contains(EdgeFlags::REVERSED) {
                        edges_to_reverse.push(eid);
                    }
                }
            }
        }
    }

    for edge_id in edges_to_reverse {
        graph.reverse_edge(edge_id);
    }
}
