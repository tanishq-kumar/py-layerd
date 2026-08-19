//! Dummy-node insertion for self-loops that cross layer boundaries.
//!
//! Reduces the set of self-loop configurations subsequent phases have to
//! handle. Edges that are "backwards" in the current layout direction are
//! reversed, and edges spanning opposing east/west ports get a long-edge
//! dummy injected so they look like regular layer-crossing edges.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::PortConstraints,
    properties::internal::JUNCTION_POINTS,
};

/// Entry point dispatched by `IntermediateProcessorId::DummySelfLoopProcessor`.
pub fn process(graph: &mut LGraph) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);
        let mut created_dummies: SmallVec<NodeId, 4> = SmallVec::new();

        for node_id in nodes {
            let ports: SmallVec<PortId, 8> = SmallVec::from_slice_copy(&graph.node(node_id).ports);
            for port_id in ports {
                let outgoing: SmallVec<EdgeId, 4> =
                    SmallVec::from_slice_copy(&graph.port(port_id).outgoing_edges);
                for edge_id in outgoing {
                    handle_edge(graph, edge_id, node_id, &mut created_dummies);
                }
            }
        }

        // Attach newly created dummies to the current layer.
        for dummy in created_dummies {
            graph.node_mut(dummy).layer = Some(layer_idx).into();
            graph.layers[layer_idx].nodes.push(dummy);
        }
    }
}

fn handle_edge(
    graph: &mut LGraph,
    edge_id: EdgeId,
    node: NodeId,
    created_dummies: &mut SmallVec<NodeId, 4>,
) {
    let source_port = graph.edge(edge_id).source;
    let target_port = graph.edge(edge_id).target;
    // Skip non-self-loops. Walking outgoing edges guarantees
    // `port(source_port).owner == node`; only the target owner needs to be
    // checked.
    if graph.port(target_port).owner != node {
        return;
    }

    let source_side = graph.port(source_port).side;
    let target_side = graph.port(target_port).side;

    // Six self-loop patterns:
    //   1. N/S -> W        : reverse
    //   2. E -> N/S/W      : reverse (E -> anything-but-E)
    //   3. S -> N          : reverse
    //   4. W -> E          : insert dummy
    //   5. E -> W          : reverse, then insert dummy
    //   6. N -> S          : no-op
    let should_reverse = matches!(
        (source_side, target_side),
        (PortSide::North, PortSide::West)
            | (PortSide::South, PortSide::West)
            | (PortSide::South, PortSide::North)
    ) || (source_side == PortSide::East && target_side != PortSide::East);

    if should_reverse {
        graph.reverse_edge(edge_id);
    }

    // Original sides decide the dummy direction; after reversal the edge's
    // stored source / target are already flipped, so we pass the original
    // ports in the correct order.
    if source_side == PortSide::East && target_side == PortSide::West {
        created_dummies.push(create_dummy(graph, edge_id, target_port, source_port));
    } else if source_side == PortSide::West && target_side == PortSide::East {
        created_dummies.push(create_dummy(graph, edge_id, source_port, target_port));
    }
}

fn create_dummy(
    graph: &mut LGraph,
    edge_id: EdgeId,
    long_edge_source: PortId,
    long_edge_target: PortId,
) -> NodeId {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::LongEdge;
    graph.layerless_nodes.retain(|&n| n != dummy);

    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.node_mut(dummy).long_edge_source = Some(long_edge_source);
    graph.node_mut(dummy).long_edge_target = Some(long_edge_target);

    let dummy_in = graph.add_port(dummy, PortSide::West);
    let dummy_out = graph.add_port(dummy, PortSide::East);

    // Reroute the original edge so it feeds the dummy input.
    let old_target = graph.edge(edge_id).target;
    graph.port_mut(old_target).incoming_edges.retain(|e| *e != edge_id);
    let dummy_owner = graph.port_owner(dummy_in);
    let edge = graph.edge_mut(edge_id);
    edge.target = dummy_in;
    edge.target_owner = dummy_owner;
    graph.port_mut(dummy_in).incoming_edges.push(edge_id);

    // Clone cold state so the trailing dummy edge inherits the right
    // properties. JUNCTION_POINTS is reset because a freshly routed segment
    // has no junctions yet.
    let cloned_properties = graph.edge(edge_id).properties.clone();
    let cloned_flags = graph.edge(edge_id).flags;

    let new_edge = graph.add_edge(dummy_out, old_target);
    graph.edge_mut(new_edge).properties = cloned_properties;
    graph.edge_mut(new_edge).flags = cloned_flags;
    graph.edge_mut(new_edge).properties.set(&JUNCTION_POINTS, SmallVec::new());

    dummy
}
