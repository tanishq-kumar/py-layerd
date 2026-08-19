use smallvec::SmallVec;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType, port::PortSide},
    options::enums::Alignment,
    properties::internal::{ALIGNMENT, EXT_PORT_SIDE},
};

/// Sets widths for hierarchical port dummy nodes.
///
/// Northern/southern external port dummies get increasing widths so that
/// their edges are properly spaced. Uses center alignment so that vertical
/// edge segments are spaced by twice the edge-edge spacing between layers.
///
/// Sets the size of hierarchical-port dummies based on edge spacing.
pub fn process(graph: &mut LGraph) {
    let edge_spacing = graph.options.spacing.edge_edge_between_layers;
    let delta = edge_spacing * 2.0;

    for layer_idx in 0..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        let mut northern_dummies: Vec<NodeId> = Vec::new();
        let mut southern_dummies: Vec<NodeId> = Vec::new();

        for &node_id in &nodes {
            if graph.node(node_id).node_type == NodeType::ExternalPort {
                let side = graph.node(node_id).properties.get(&EXT_PORT_SIDE);
                match side {
                    PortSide::North => northern_dummies.push(node_id),
                    PortSide::South => southern_dummies.push(node_id),
                    _ => {}
                }
            }
        }

        // Northern dummies: widths increase top to bottom
        set_widths(graph, &northern_dummies, true, delta);

        // Southern dummies: widths decrease top to bottom
        set_widths(graph, &southern_dummies, false, delta);
    }
}

fn set_widths(graph: &mut LGraph, nodes: &[NodeId], top_down: bool, delta: f64) {
    if nodes.is_empty() {
        return;
    }

    let mut current_width;
    let step;

    if top_down {
        current_width = 0.0;
        step = delta;
    } else {
        current_width = delta * (nodes.len() as f64 - 1.0);
        step = -delta;
    }

    for &node_id in nodes {
        // Force center alignment so BK sees every external-port dummy as
        // horizontally centred within its variable-width slot.
        graph.node_mut(node_id).properties.set(&ALIGNMENT, Alignment::Center);
        graph.node_mut(node_id).size.x = current_width;

        // Move eastern ports to the right border
        let ports = graph.node(node_id).ports.to_vec();
        for &port_id in &ports {
            if graph.port(port_id).side == PortSide::East {
                graph.port_mut(port_id).position.x = current_width;
            }
        }

        current_width += step;
    }
}
