use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{LabelId, NodeId, PortId},
        node::NodeType,
    },
    math::Vec2,
    properties::internal::{JUNCTION_POINTS, UNNECESSARY_BENDPOINTS},
};

/// Removes dummy `LongEdge` nodes and reconnects the original edges.
///
/// For each dummy node along a long-edge chain:
/// - the dummy's incoming edge survives, its target is redirected to the
///   dropped edge's target;
/// - the dropped edge is removed;
/// - bend points, labels, and junction points from the dropped edge are
///   merged onto the surviving edge;
/// - if `UNNECESSARY_BENDPOINTS` is set, the dummy's absolute-anchor is added
///   as an explicit bend point.
pub fn join(graph: &mut LGraph) {
    let add_unnecessary = graph.properties.get(&UNNECESSARY_BENDPOINTS);

    for layer_idx in 0..graph.layers.len() {
        let mut nodes = std::mem::take(&mut graph.layers[layer_idx].nodes);
        for &node_id in &nodes {
            if graph.node(node_id).node_type == NodeType::LongEdge {
                join_at(graph, node_id, add_unnecessary);
            }
        }

        nodes.retain(|&n| graph.node(n).node_type != NodeType::LongEdge);
        graph.layers[layer_idx].nodes = nodes;
    }
}

pub(super) fn join_at(graph: &mut LGraph, dummy: NodeId, add_unnecessary: bool) {
    let (west_port, east_port) = input_output_ports(graph, dummy);
    let Some(west) = west_port else { return };
    let Some(east) = east_port else { return };

    let unnecessary_point = absolute_anchor_of_any_port(graph, dummy);

    let count = graph.port(west).incoming_edges.len();
    for _ in 0..count {
        let Some(&surviving) = graph.port(west).incoming_edges.first() else {
            break;
        };
        let Some(&dropped) = graph.port(east).outgoing_edges.first() else {
            break;
        };

        let dropped_target = graph.edge(dropped).target;
        let drop_idx = graph.port(dropped_target).incoming_edges.iter().position(|&e| e == dropped);

        // Detach the surviving edge's current target (the dummy west port),
        // then attach it to the dropped edge's final target at the same
        // list index where `dropped` lived (KIPRA-1670 ordering invariant).
        graph.port_mut(west).incoming_edges.retain(|e| *e != surviving);
        graph.port_mut(dropped_target).incoming_edges.retain(|e| *e != dropped);
        let dropped_owner = graph.port_owner(dropped_target);
        let edge = graph.edge_mut(surviving);
        edge.target = dropped_target;
        edge.target_owner = dropped_owner;

        if let Some(idx) = drop_idx {
            let at = idx.min(graph.port(dropped_target).incoming_edges.len());
            graph.port_mut(dropped_target).incoming_edges.insert(at, surviving);
        } else {
            graph.port_mut(dropped_target).incoming_edges.push(surviving);
        }

        // Merge bend points.
        if add_unnecessary {
            graph.edge_mut(surviving).bend_points.push(unnecessary_point);
        }
        let mut dropped_bends = std::mem::take(&mut graph.edge_mut(dropped).bend_points);
        graph.edge_mut(surviving).bend_points.append(&mut dropped_bends);

        // Merge labels.
        let dropped_labels: SmallVec<LabelId, 2> =
            std::mem::take(&mut graph.edge_mut(dropped).labels);
        for label in dropped_labels {
            graph.edge_mut(surviving).labels.push(label);
        }

        // Merge junction points.
        let mut sjps: SmallVec<Vec2, 4> = graph.edge(surviving).properties.get(&JUNCTION_POINTS);
        let djps: SmallVec<Vec2, 4> = graph.edge(dropped).properties.get(&JUNCTION_POINTS);
        if !djps.is_empty() {
            for jp in djps {
                sjps.push(jp);
            }
            graph.edge_mut(surviving).properties.set(&JUNCTION_POINTS, sjps);
        }

        // Detach the dropped edge so it no longer references any port.
        graph.port_mut(east).outgoing_edges.retain(|e| *e != dropped);
    }
}

fn input_output_ports(graph: &LGraph, node: NodeId) -> (Option<PortId>, Option<PortId>) {
    use crate::graph::port::PortSide;
    let mut west = None;
    let mut east = None;
    for &port_id in &graph.node(node).ports {
        match graph.port(port_id).side {
            PortSide::West => west = Some(port_id),
            PortSide::East => east = Some(port_id),
            _ => {}
        }
    }
    (west, east)
}

fn absolute_anchor_of_any_port(graph: &LGraph, node: NodeId) -> Vec2 {
    if let Some(&port_id) = graph.node(node).ports.first() {
        graph.absolute_anchor(port_id)
    } else {
        graph.node(node).position
    }
}
