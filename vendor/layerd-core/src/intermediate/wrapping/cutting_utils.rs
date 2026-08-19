//! Helpers shared by the breaking-point and single-edge wrappers for
//! inserting dummy-node chains that implement a back-wrapped edge.

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
    properties::internal::{EDGE_THICKNESS, JUNCTION_POINTS, SPACING_EDGE_NODE_OVERRIDE},
};

/// Insert two in-layer dummies and `m` long-edge dummies in between so that
/// `original_edge = (u, v)` becomes a chain
///
/// ```text
/// u → il_1 → d_1 → … → d_m → il_2 → v
/// ```
///
/// Returns the list of freshly-created dummy edges, in chain order. The
/// original edge now connects the last dummy's output to `v`.
pub fn insert_dummies(
    graph: &mut LGraph,
    original_edge: EdgeId,
    offset_first_in_layer_dummy: usize,
) -> Vec<EdgeId> {
    let mut current_edge = original_edge;
    let target_port = graph.edge(current_edge).target;
    let source_port = graph.edge(current_edge).source;

    let src_owner = graph.port(source_port).owner;
    let tgt_owner = graph.port(target_port).owner;
    let src_index = graph.node(src_owner).layer.expect("source node must be layered");
    let tgt_index = graph.node(tgt_owner).layer.expect("target node must be layered");

    let mut created_edges: Vec<EdgeId> =
        Vec::with_capacity(tgt_index.saturating_sub(src_index) + 1);

    for i in src_index..=tgt_index {
        let mut thickness: f64 = graph.edge(current_edge).properties.get(&EDGE_THICKNESS);
        if thickness < 0.0 {
            thickness = 0.0;
            graph.edge_mut(current_edge).properties.set(&EDGE_THICKNESS, thickness);
        }
        let port_pos_y = (thickness / 2.0).floor();

        let dummy = graph.add_node(Vec2 { x: 0.0, y: thickness });
        let wrapping_edge_node_spacing =
            graph.options.spacing.edge_node + graph.options.wrapping_additional_edge_spacing;
        graph.node_mut(dummy).node_type = NodeType::LongEdge;
        graph.layerless_nodes.retain(|&n| n != dummy);
        graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
        graph
            .node_mut(dummy)
            .properties
            .set(&SPACING_EDGE_NODE_OVERRIDE, Some(wrapping_edge_node_spacing));

        if i == src_index {
            let layer_size = graph.layers[i].nodes.len();
            let pos = layer_size.saturating_sub(offset_first_in_layer_dummy);
            graph.insert_node_in_layer(dummy, i, pos);
        } else {
            graph.node_mut(dummy).layer = Some(i).into();
            graph.layers[i].nodes.push(dummy);
        }

        let dummy_in = graph.add_port(dummy, PortSide::West);
        graph.port_mut(dummy_in).position = Vec2 { x: 0.0, y: port_pos_y };
        let dummy_out = graph.add_port(dummy, PortSide::East);
        graph.port_mut(dummy_out).position = Vec2 { x: 0.0, y: port_pos_y };

        graph.reroute_edge_target(current_edge, dummy_in);

        let cloned_properties = graph.edge(current_edge).properties.clone();
        let cloned_flags = graph.edge(current_edge).flags;
        let new_edge = graph.add_edge(dummy_out, target_port);
        graph.edge_mut(new_edge).properties = cloned_properties;
        graph.edge_mut(new_edge).flags = cloned_flags;
        graph.edge_mut(new_edge).properties.set(&JUNCTION_POINTS, SmallVec::new());

        set_dummy_properties(graph, dummy, current_edge, new_edge);
        created_edges.push(new_edge);
        current_edge = new_edge;
    }

    created_edges
}

/// Mark the long-edge dummy with the original `LONG_EDGE_SOURCE` /
/// `LONG_EDGE_TARGET` so downstream processors can reconstruct the
/// original endpoints.
fn set_dummy_properties(graph: &mut LGraph, dummy: NodeId, in_edge: EdgeId, out_edge: EdgeId) {
    let in_src_port = graph.edge(in_edge).source;
    let in_src_owner = graph.port(in_src_port).owner;

    let (src, tgt): (Option<PortId>, Option<PortId>) =
        if graph.node(in_src_owner).node_type == NodeType::LongEdge {
            let s: Option<PortId> = graph.node(in_src_owner).long_edge_source;
            let t: Option<PortId> = graph.node(in_src_owner).long_edge_target;
            (s, t)
        } else {
            (Some(in_src_port), Some(graph.edge(out_edge).target))
        };

    graph.node_mut(dummy).long_edge_source = src;
    graph.node_mut(dummy).long_edge_target = tgt;
}
