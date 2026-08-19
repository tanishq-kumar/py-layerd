use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
    },
    options::enums::{InLayerConstraint, LayerConstraint},
    properties::{
        graph_properties::GraphProperties,
        internal::{GRAPH_PROPERTIES, IN_LAYER_CONSTRAINT, LAYER_CONSTRAINT},
    },
};

/// Arbitrary spacing value used to separate external port dummies from other nodes.
const ARBITRARY_SPACING: f64 = 10.0;

/// Assigns reasonable positions to external port dummy nodes so that interactive
/// cycle breaking / layering phases can make correct decisions based on them.
///
/// Runs before phase 1. Westward external ports go to the left of all regular
/// nodes, eastward ports to the right, northward above, southward below.
pub fn position_external_ports(graph: &mut LGraph) {
    let graph_props = graph.properties.get(&GRAPH_PROPERTIES);
    if !graph_props.contains(GraphProperties::EXTERNAL_PORTS) {
        return;
    }

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for &node_id in &graph.layerless_nodes {
        let node = graph.node(node_id);
        if node.node_type != NodeType::Normal {
            continue;
        }
        let pos = node.position;
        let size = node.size;
        let margin = *node.margin;
        min_x = min_x.min(pos.x - margin.left);
        max_x = max_x.max(pos.x + size.x + margin.right);
        min_y = min_y.min(pos.y - margin.top);
        max_y = max_y.max(pos.y + size.y + margin.bottom);
    }

    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    for node_id in node_ids {
        let node_type = graph.node(node_id).node_type;
        if node_type == NodeType::Normal {
            continue;
        }
        if node_type != NodeType::ExternalPort {
            panic!(
                "unsupported node type for interactive external port positioning: {node_type:?}"
            );
        }

        let lc = graph.node(node_id).properties.get(&LAYER_CONSTRAINT);
        match lc {
            LayerConstraint::FirstSeparate => {
                graph.node_mut(node_id).position.x = min_x - ARBITRARY_SPACING;
                if let Some(y) = find_y_coordinate(graph, node_id, true) {
                    graph.node_mut(node_id).position.y = y;
                }
                continue;
            }
            LayerConstraint::LastSeparate => {
                graph.node_mut(node_id).position.x = max_x + ARBITRARY_SPACING;
                if let Some(y) = find_y_coordinate(graph, node_id, false) {
                    graph.node_mut(node_id).position.y = y;
                }
                continue;
            }
            _ => {}
        }

        let ilc = graph.node(node_id).properties.get(&IN_LAYER_CONSTRAINT);
        match ilc {
            InLayerConstraint::Top => {
                if let Some(x) = find_north_south_port_x(graph, node_id) {
                    graph.node_mut(node_id).position.x = x + ARBITRARY_SPACING;
                }
                graph.node_mut(node_id).position.y = min_y - ARBITRARY_SPACING;
            }
            InLayerConstraint::Bottom => {
                if let Some(x) = find_north_south_port_x(graph, node_id) {
                    graph.node_mut(node_id).position.x = x + ARBITRARY_SPACING;
                }
                graph.node_mut(node_id).position.y = max_y + ARBITRARY_SPACING;
            }
            InLayerConstraint::None => {}
        }
    }
}

/// Find the y coordinate of the first connected node.
///
/// `use_target = true` walks outgoing edges (for FIRST_SEPARATE / west ports),
/// `false` walks incoming edges (for LAST_SEPARATE / east ports).
fn find_y_coordinate(graph: &LGraph, dummy: NodeId, use_target: bool) -> Option<f64> {
    for &port_id in &graph.node(dummy).ports {
        let port = graph.port(port_id);
        let edges = if use_target { &port.outgoing_edges } else { &port.incoming_edges };
        if let Some(&edge_id) = edges.into_iter().next() {
            let edge = graph.edge(edge_id);
            let other_port = if use_target { edge.target } else { edge.source };
            let other_node = graph.port(other_port).owner;
            let other = graph.node(other_node);
            return Some(other.position.y + other.size.y / 2.0);
        }
    }
    None
}

/// Find the x coordinate used to align a north or south external port dummy.
///
/// A TOP/BOTTOM dummy has exactly one port. If the port has outgoing edges it
/// picks the minimum `position.x - margin.left` across all targets; if it has
/// incoming edges it picks the maximum `position.x + size.x + margin.right`
/// across all sources. Mixed incoming/outgoing is unsupported.
fn find_north_south_port_x(graph: &LGraph, dummy: NodeId) -> Option<f64> {
    let ports = &graph.node(dummy).ports;
    debug_assert_eq!(ports.len(), 1, "external port dummy must have exactly one port");
    let port_id: PortId = *ports.first()?;
    let port = graph.port(port_id);

    if !port.outgoing_edges.is_empty() && !port.incoming_edges.is_empty() {
        panic!(
            "Interactive layout does not support NORTH/SOUTH ports with incoming and outgoing edges."
        );
    }

    if !port.outgoing_edges.is_empty() {
        let mut min = f64::INFINITY;
        for &edge_id in &port.outgoing_edges {
            let other = graph.port(graph.edge(edge_id).target).owner;
            let n = graph.node(other);
            min = min.min(n.position.x - n.margin.left);
        }
        return Some(min);
    }

    if !port.incoming_edges.is_empty() {
        let mut max = f64::NEG_INFINITY;
        for &edge_id in &port.incoming_edges {
            let other = graph.port(graph.edge(edge_id).source).owner;
            let n = graph.node(other);
            max = max.max(n.position.x + n.size.x + n.margin.right);
        }
        return Some(max);
    }

    None
}
