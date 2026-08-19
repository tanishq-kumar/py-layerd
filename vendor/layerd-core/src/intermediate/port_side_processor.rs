use crate::{
    graph::{LGraph, index::NodeId, port::PortSide},
    options::enums::PortConstraints,
    properties::internal::EXT_PORT_SIDE,
};

/// Assigns sides to every port on every node.
///
/// Side-fixed nodes only have their `Undefined` ports filled in; non-fixed
/// nodes get every port sided and their `PORT_CONSTRAINTS` bumped up to
/// `FixedSide`.
pub fn assign_sides(graph: &mut LGraph) {
    let mut node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    for layer in &graph.layers {
        node_ids.extend(&layer.nodes);
    }

    for node_id in node_ids {
        process_node(graph, node_id);
    }
}

fn process_node(graph: &mut LGraph, node_id: NodeId) {
    let constraints = node_port_constraints(graph, node_id);
    let ports: Vec<_> = graph.node(node_id).ports.to_vec();

    if constraints.is_side_fixed() {
        for port_id in ports {
            if graph.port(port_id).side == PortSide::Undefined {
                set_port_side(graph, port_id);
            }
        }
    } else {
        for port_id in ports {
            set_port_side(graph, port_id);
        }
        graph.node_mut(node_id).node_port_constraints = Some(PortConstraints::FixedSide);
    }
}

fn node_port_constraints(graph: &LGraph, node_id: NodeId) -> PortConstraints {
    let pc = graph.node(node_id).port_constraints();
    if matches!(pc, PortConstraints::Undefined) {
        graph.options.port_constraints
    } else {
        pc
    }
}

fn set_port_side(graph: &mut LGraph, port_id: crate::graph::index::PortId) {
    if let Some(dummy) = graph.port(port_id).port_dummy {
        let side = graph.node(dummy).properties.get(&EXT_PORT_SIDE);
        graph.port_mut(port_id).side = side;
        return;
    }
    let net_flow = graph.port(port_id).incoming_edges.len() as i32
        - graph.port(port_id).outgoing_edges.len() as i32;
    graph.port_mut(port_id).side = if net_flow < 0 { PortSide::East } else { PortSide::West };
}
