use smallvec::SmallVec;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType, port::PortSide},
    options::enums::PortConstraints,
    properties::internal::{EXT_PORT_SIDE, PORT_ANCHOR, PORT_RATIO_OR_POSITION},
};

/// Positions hierarchical port dummy nodes at the graph boundary.
///
/// Sets the y coordinate of external node dummies representing eastern or
/// western hierarchical ports. For fixed-ratio constraints the position is
/// computed by multiplying the ratio with the graph height.
///
/// Hierarchical-port position processor.
pub fn position(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    // Fix coordinates in the first and last layer
    fix_coordinates(graph, 0);
    if graph.layers.len() > 1 {
        fix_coordinates(graph, graph.layers.len() - 1);
    }
}

fn fix_coordinates(graph: &mut LGraph, layer_idx: usize) {
    let port_constraints = graph.options.port_constraints;
    if !(port_constraints.is_ratio_fixed() || port_constraints.is_pos_fixed()) {
        return;
    }

    // `actual_size.y` = `size.y + padding.top + padding.bottom`.
    let graph_height = graph.size.y + graph.padding.top + graph.padding.bottom;
    let padding_top = graph.padding.top;
    let offset_y = graph.offset.y;

    let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

    for &node_id in &nodes {
        if graph.node(node_id).node_type != NodeType::ExternalPort {
            continue;
        }

        let ext_port_side = graph.node(node_id).properties.get(&EXT_PORT_SIDE);
        if ext_port_side != PortSide::East && ext_port_side != PortSide::West {
            continue;
        }

        let mut final_y = graph.node(node_id).properties.get(&PORT_RATIO_OR_POSITION);

        if port_constraints == PortConstraints::FixedRatio {
            final_y *= graph_height;
        }

        // Subtract the node's PORT_ANCHOR.y then translate from border to
        // content-area coordinates (shift by `- padding.top - offset.y`).
        let port_anchor_y =
            graph.node(node_id).properties.get(&PORT_ANCHOR).map(|v| v.y).unwrap_or(0.0);
        graph.node_mut(node_id).position.y = final_y - port_anchor_y - padding_top - offset_y;
    }
}
