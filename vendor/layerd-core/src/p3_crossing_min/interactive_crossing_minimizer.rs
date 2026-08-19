//! Interactive crossing minimizer.
//!
//! Reorders nodes within each layer by preserving an externally-supplied
//! vertical ordering. For a normal node, the reference is the interactive
//! anchor point (`position.y` or `position.y + size.y / 2`). For a
//! `NorthSouthPort` dummy, the reference is the N or S edge of the origin
//! node so the dummy stays with its port. For a `LongEdge` dummy the
//! original bend-point chain would be interpolated; the interpolation hook
//! is kept as a fallback but `ORIGINAL_BENDPOINTS` is not yet wired, so
//! long-edge dummies fall back to `position.y` — the interactive-anchor
//! equivalent.
//!
//! In-layer successor constraints break ties.

use std::cmp::Ordering;

use crate::{
    graph::{
        LGraph,
        edge::EdgeFlags,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::InteractiveReferencePoint,
    p3_crossing_min::layer_sweep::{self, PortDistributionScratch, PortRankMode},
    properties::internal::{
        IN_LAYER_SUCCESSOR_CONSTRAINTS, ORIGIN_NODE, ORIGIN_PORT, ORIGINAL_DUMMY_NODE_POSITION,
        PORT_ANCHOR,
    },
};

/// Reorder nodes in each layer by previous vertical positions.
pub fn minimize_crossings(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    let mode = graph.options.interactive_reference_point;
    let node_order: Vec<Vec<NodeId>> =
        graph.layers.iter().map(|layer| layer.nodes.to_vec()).collect();
    let mut port_distribution = PortDistributionScratch::new();

    for layer_idx in 0..graph.layers.len() {
        let horiz_pos = compute_horizontal_position(graph, layer_idx);
        let layer_nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        let positions: Vec<(NodeId, f64)> = layer_nodes
            .iter()
            .map(|&nid| (nid, interactive_y(graph, nid, horiz_pos, mode)))
            .collect();
        for &(nid, y) in &positions {
            if graph.node(nid).node_type == NodeType::LongEdge {
                graph.node_mut(nid).properties.set(&ORIGINAL_DUMMY_NODE_POSITION, Some(y));
            }
        }
        if layer_nodes.len() >= 2 {
            // Sort the layer using successor constraints as tiebreak for equal Y.
            let mut sorted = positions;
            sorted.sort_by(|&(na, ya), &(nb, yb)| {
                let primary = ya.partial_cmp(&yb).unwrap_or(Ordering::Equal);
                if primary == Ordering::Equal {
                    return successor_constraint_cmp(graph, na, nb);
                }
                primary
            });
            let ordered: Vec<NodeId> = sorted.into_iter().map(|(nid, _)| nid).collect();
            if graph.layers[layer_idx].nodes != ordered {
                graph.layers[layer_idx].nodes = ordered;
                graph.bump_layer_order_version(layer_idx);
            }
        }

        layer_sweep::barycenter_distribute_ports_in_layer_with_node_order(
            graph,
            &node_order,
            layer_idx,
            true,
            PortRankMode::NodeRelative,
            &mut port_distribution,
        );
    }
}

/// Average the horizontal position (`x + size.x/2`) of nodes whose
/// `position.x > 0`. Used when interpolating along a long-edge bend chain.
/// Returns 0 if no node qualifies.
fn compute_horizontal_position(graph: &LGraph, layer_idx: usize) -> f64 {
    let mut horiz = 0.0;
    let mut count = 0;
    for &nid in &graph.layers[layer_idx].nodes {
        let node = graph.node(nid);
        if node.position.x > 0.0 {
            horiz += node.position.x + node.size.x / 2.0;
            count += 1;
        }
    }
    if count > 0 { horiz / count as f64 } else { 0.0 }
}

/// Compute the interactive Y reference for `node`. For `LongEdge` dummies
/// the full implementation would reconstruct a bend-point chain and
/// interpolate vertically at `horiz_pos`; this falls back to `position.y`
/// when bend-point property chains are not available.
fn interactive_y(
    graph: &LGraph,
    node: NodeId,
    horiz_pos: f64,
    mode: InteractiveReferencePoint,
) -> f64 {
    let n = graph.node(node);
    match n.node_type {
        NodeType::NorthSouthPort => {
            // Use the first port's ORIGIN_PORT to find the origin node and
            // the original port side.
            let first_port = n.ports.first().copied();
            if let Some(pid) = first_port {
                let origin_port_opt: Option<PortId> = graph.port(pid).properties.get(&ORIGIN_PORT);
                if let Some(origin_port_id) = origin_port_opt {
                    let side = graph.port(origin_port_id).side;
                    let owner = graph.port(origin_port_id).owner;
                    let origin_node_y = graph.node(owner).position.y;
                    let origin_node_size_y = graph.node(owner).size.y;
                    match side {
                        PortSide::North => return origin_node_y,
                        PortSide::South => return origin_node_y + origin_node_size_y,
                        _ => {}
                    }
                }
            }
            interactive_reference_y(graph, node, mode)
        }
        NodeType::LongEdge => {
            let mut points: Vec<Vec2> = n
                .origin_edge
                .map(|edge_id| {
                    let edge = graph.edge(edge_id);
                    let mut bends = edge.bend_points.clone();
                    if edge.flags.contains(EdgeFlags::REVERSED) {
                        bends.reverse();
                    }
                    bends
                })
                .unwrap_or_default();
            if let Some(source) = n.long_edge_source {
                let source_point = absolute_port_anchor(graph, source);
                if horiz_pos <= source_point.x {
                    return source_point.y;
                }
                points.insert(0, source_point);
            }
            if let Some(target) = n.long_edge_target {
                let target_point = absolute_port_anchor(graph, target);
                if target_point.x <= horiz_pos {
                    return target_point.y;
                }
                points.push(target_point);
            }
            if points.len() >= 2 {
                let mut point1 = points[0];
                let mut point2 = points[1];
                for &next in points.iter().skip(2) {
                    if point2.x < horiz_pos {
                        point1 = point2;
                        point2 = next;
                    } else {
                        break;
                    }
                }
                let dx = point2.x - point1.x;
                if dx.abs() <= f64::EPSILON {
                    return point1.y;
                }
                return point1.y + (horiz_pos - point1.x) / dx * (point2.y - point1.y);
            }
            interactive_reference_y(graph, node, mode)
        }
        NodeType::Normal => {
            // Some dummy types store their associated node via `ORIGIN_NODE`;
            // if present, inherit its y.
            let origin = n.properties.get(&ORIGIN_NODE);
            if let Some(origin_node) = origin {
                return interactive_reference_y(graph, origin_node, mode);
            }
            interactive_reference_y(graph, node, mode)
        }
        _ => interactive_reference_y(graph, node, mode),
    }
}

/// Y component of the interactive reference point for `node`.
fn interactive_reference_y(graph: &LGraph, node: NodeId, mode: InteractiveReferencePoint) -> f64 {
    let n = graph.node(node);
    match mode {
        InteractiveReferencePoint::Center => n.position.y + n.size.y / 2.0,
        InteractiveReferencePoint::TopLeft => n.position.y,
    }
}

fn absolute_port_anchor(graph: &LGraph, port: PortId) -> Vec2 {
    let port_data = graph.port(port);
    let node = graph.node(port_data.owner);
    let anchor = import_time_port_anchor(graph, port);
    Vec2::new(
        node.position.x + port_data.position.x + anchor.x,
        node.position.y + port_data.position.y + anchor.y,
    )
}

fn import_time_port_anchor(graph: &LGraph, port: PortId) -> Vec2 {
    let port_data = graph.port(port);
    if port_data.explicitly_supplied_anchor {
        return port_data.anchor;
    }
    if let Some(anchor) = port_data.properties.get(&PORT_ANCHOR) {
        return anchor;
    }
    if graph.node(port_data.owner).port_constraints().is_side_fixed()
        && port_data.side != PortSide::Undefined
    {
        match port_data.side {
            PortSide::North => Vec2::new(port_data.size.x / 2.0, 0.0),
            PortSide::East => Vec2::new(port_data.size.x, port_data.size.y / 2.0),
            PortSide::South => Vec2::new(port_data.size.x / 2.0, port_data.size.y),
            PortSide::West => Vec2::new(0.0, port_data.size.y / 2.0),
            PortSide::Undefined => Vec2::new(port_data.size.x / 2.0, port_data.size.y / 2.0),
        }
    } else {
        Vec2::new(port_data.size.x / 2.0, port_data.size.y / 2.0)
    }
}

/// Tiebreak for equal Y: honour `IN_LAYER_SUCCESSOR_CONSTRAINTS` so that a
/// predecessor always sorts before its declared successor.
fn successor_constraint_cmp(graph: &LGraph, a: NodeId, b: NodeId) -> Ordering {
    let a_succ = graph.node(a).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
    let b_succ = graph.node(b).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
    if a_succ.contains(&b) {
        Ordering::Less
    } else if b_succ.contains(&a) {
        Ordering::Greater
    } else {
        Ordering::Equal
    }
}
