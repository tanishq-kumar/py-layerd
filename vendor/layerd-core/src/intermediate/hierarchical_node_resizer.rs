use crate::{
    graph::{LGraph, index::NodeId, node::NodeType, port::PortSide},
    math::Vec2,
    options::enums::{ContentAlignment, SizeConstraint, SizeOptions},
    properties::{
        graph_properties::GraphProperties,
        internal::{
            CONTENT_ALIGNMENT, EXT_PORT_SIDE, GRAPH_PROPERTIES, NODE_SIZE_CONSTRAINTS,
            NODE_SIZE_FIXED_GRAPH_SIZE, NODE_SIZE_MINIMUM, NODE_SIZE_OPTIONS,
        },
    },
};

/// Default minimum width when `DEFAULT_MINIMUM_SIZE` size option kicks in.
const DEFAULT_MIN_WIDTH: f64 = 20.0;
/// Default minimum height when `DEFAULT_MINIMUM_SIZE` size option kicks in.
const DEFAULT_MIN_HEIGHT: f64 = 20.0;

/// Resizes the graph to fit its parent node after layout completes.
///
/// Runs after P5. Collapses all layers into `layerless_nodes`, applies
/// minimum-size constraints, realigns content based on `CONTENT_ALIGNMENT`,
/// and corrects east/south external port positions when the graph grew.
pub fn resize(graph: &mut LGraph) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes_in_layer: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for &node_id in &nodes_in_layer {
            graph.node_mut(node_id).layer = None.into();
            graph.layerless_nodes.push(node_id);
        }
    }
    graph.layers.clear();

    resize_graph(graph);
}

/// Computes the actual size of a graph: `size + padding`. Padding has already
/// been folded into `offset` and `size` by the time resizing runs.
fn actual_size(graph: &LGraph) -> Vec2 {
    Vec2 {
        x: graph.size.x + graph.padding.left + graph.padding.right,
        y: graph.size.y + graph.padding.top + graph.padding.bottom,
    }
}

pub(crate) fn resize_graph(graph: &mut LGraph) {
    if graph.properties.get(&NODE_SIZE_FIXED_GRAPH_SIZE) {
        return;
    }

    let size_constraints = graph.properties.get(&NODE_SIZE_CONSTRAINTS);
    let calculated = actual_size(graph);
    let mut adjusted = calculated;

    if size_constraints.contains(SizeConstraint::MINIMUM_SIZE) {
        let mut min_size = graph.properties.get(&NODE_SIZE_MINIMUM);
        // The 20×20 fallback only kicks in when `NODE_SIZE_OPTIONS`
        // contains `DEFAULT_MINIMUM_SIZE`. Without that flag the explicit
        // minimum (even if 0) is honoured verbatim.
        let size_options = graph.properties.get(&NODE_SIZE_OPTIONS);
        if size_options.contains(SizeOptions::DEFAULT_MINIMUM_SIZE) {
            if min_size.x <= 0.0 {
                min_size.x = DEFAULT_MIN_WIDTH;
            }
            if min_size.y <= 0.0 {
                min_size.y = DEFAULT_MIN_HEIGHT;
            }
        }
        adjusted.x = calculated.x.max(min_size.x);
        adjusted.y = calculated.y.max(min_size.y);
    }

    apply_new_size(graph, calculated, adjusted);
}

fn apply_new_size(graph: &mut LGraph, old_size: Vec2, new_size: Vec2) {
    let alignment = graph.properties.get(&CONTENT_ALIGNMENT);

    if new_size.x > old_size.x {
        if alignment.contains(ContentAlignment::H_CENTER) {
            graph.offset.x += (new_size.x - old_size.x) / 2.0;
        } else if alignment.contains(ContentAlignment::H_RIGHT) {
            graph.offset.x += new_size.x - old_size.x;
        }
    }

    if new_size.y > old_size.y {
        if alignment.contains(ContentAlignment::V_CENTER) {
            graph.offset.y += (new_size.y - old_size.y) / 2.0;
        } else if alignment.contains(ContentAlignment::V_BOTTOM) {
            graph.offset.y += new_size.y - old_size.y;
        }
    }

    let graph_props = graph.properties.get(&GRAPH_PROPERTIES);
    let has_ext_ports = graph_props.contains(GraphProperties::EXTERNAL_PORTS);
    let grew = new_size.x > old_size.x || new_size.y > old_size.y;
    if has_ext_ports && grew {
        let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
        for node_id in node_ids {
            if graph.node(node_id).node_type != NodeType::ExternalPort {
                continue;
            }
            match graph.node(node_id).properties.get(&EXT_PORT_SIDE) {
                PortSide::East => {
                    graph.node_mut(node_id).position.x += new_size.x - old_size.x;
                }
                PortSide::South => {
                    graph.node_mut(node_id).position.y += new_size.y - old_size.y;
                }
                _ => {}
            }
        }
    }

    let padding = graph.padding;
    graph.size.x = new_size.x - padding.left - padding.right;
    graph.size.y = new_size.y - padding.top - padding.bottom;
}
