use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    math::Vec2,
    properties::internal::{HYPERNODE, JUNCTION_POINTS},
};

/// Improves the placement of hypernodes by moving them so that they replace the
/// join points of the edges they connect.
///
/// Runs after P5. Only hypernodes with at most two ports and no north/south
/// edges are candidates for relocation.
pub fn process_hypernodes(graph: &mut LGraph) {
    let candidates: Vec<(NodeId, bool)> = collect_candidates(graph);
    for (node, move_right) in candidates {
        move_hypernode(graph, node, move_right);
    }
}

/// Scan all layers for hypernodes that can be moved, returning their ids and
/// whether they should move right (toward the next layer) or left.
fn collect_candidates(graph: &LGraph) -> Vec<(NodeId, bool)> {
    let mut out = Vec::new();
    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            let node = graph.node(node_id);
            if !node.properties.get(&HYPERNODE) || node.ports.len() > 2 {
                continue;
            }
            let mut top = 0;
            let mut right = 0;
            let mut bottom = 0;
            let mut left = 0;
            for &port_id in &node.ports {
                match graph.port(port_id).side {
                    PortSide::North => top += 1,
                    PortSide::East => right += 1,
                    PortSide::South => bottom += 1,
                    PortSide::West => left += 1,
                    PortSide::Undefined => {}
                }
            }
            if top == 0 && bottom == 0 {
                out.push((node_id, left <= right));
            }
        }
    }
    out
}

/// Move `hypernode` toward the previous layer (`move_right = false`) or the
/// next layer (`move_right = true`), replacing the first/last bend point of
/// each hyperedge segment with a port on the hypernode.
fn move_hypernode(graph: &mut LGraph, hypernode: NodeId, move_right: bool) {
    let size = graph.node(hypernode).size;
    let graph_width = graph.size.x;

    let mut bend_edges: Vec<EdgeId> = Vec::new();
    let mut bendx: f64;
    let mut diffx = f64::INFINITY;
    let mut diffy = f64::INFINITY;

    if move_right {
        bendx = graph_width;
        for &port_id in &graph.node(hypernode).ports {
            for &edge_id in &graph.port(port_id).outgoing_edges {
                let bps = &graph.edge(edge_id).bend_points;
                if bps.is_empty() {
                    continue;
                }
                let first = bps[0];
                if first.x < bendx {
                    diffx = bendx - first.x;
                    diffy = f64::INFINITY;
                    bend_edges.clear();
                    bendx = first.x;
                }
                if first.x <= bendx {
                    bend_edges.push(edge_id);
                    if bps.len() > 1 {
                        diffy = diffy.min((bps[1].y - first.y).abs());
                    }
                }
            }
        }
    } else {
        bendx = f64::NEG_INFINITY;
        for &port_id in &graph.node(hypernode).ports {
            for &edge_id in &graph.port(port_id).incoming_edges {
                let bps = &graph.edge(edge_id).bend_points;
                if bps.is_empty() {
                    continue;
                }
                let last = bps[bps.len() - 1];
                if last.x > bendx {
                    diffx = last.x - bendx;
                    diffy = f64::INFINITY;
                    bend_edges.clear();
                    bendx = last.x;
                }
                if last.x >= bendx {
                    bend_edges.push(edge_id);
                    if bps.len() > 1 {
                        diffy = diffy.min((bps[bps.len() - 2].y - last.y).abs());
                    }
                }
            }
        }
    }

    if bend_edges.is_empty() || diffx <= size.x / 2.0 || diffy <= size.y / 2.0 {
        return;
    }

    let north_port = graph.add_port(hypernode, PortSide::North);
    graph.port_mut(north_port).position = Vec2 { x: size.x / 2.0, y: 0.0 };
    let south_port = graph.add_port(hypernode, PortSide::South);
    graph.port_mut(south_port).position = Vec2 { x: size.x / 2.0, y: size.y };

    for edge_id in bend_edges.iter().copied() {
        let removed = if move_right {
            remove_first_bend_point(graph, edge_id)
        } else {
            remove_last_bend_point(graph, edge_id)
        };
        let Some(first) = removed else { continue };
        let second = neighbor_point(graph, edge_id, move_right);
        let target_port = if second.y >= first.y { south_port } else { north_port };
        if move_right {
            reroute_source(graph, edge_id, target_port);
        } else {
            reroute_target(graph, edge_id, target_port);
        }
        remove_junction_point(graph, edge_id, first);
    }

    graph.node_mut(hypernode).position.x = bendx - size.x / 2.0;
}

fn remove_first_bend_point(graph: &mut LGraph, edge: EdgeId) -> Option<Vec2> {
    let bps = &mut graph.edge_mut(edge).bend_points;
    if bps.is_empty() { None } else { Some(bps.remove(0)) }
}

fn remove_last_bend_point(graph: &mut LGraph, edge: EdgeId) -> Option<Vec2> {
    let bps = &mut graph.edge_mut(edge).bend_points;
    bps.pop()
}

/// After a bend point is removed, the neighboring reference point is either
/// the new first/last bend point, or (if no more remain) the absolute anchor
/// of the opposite endpoint.
fn neighbor_point(graph: &LGraph, edge: EdgeId, move_right: bool) -> Vec2 {
    let bps = &graph.edge(edge).bend_points;
    if move_right {
        if let Some(first) = bps.first() {
            return *first;
        }
        graph.absolute_anchor(graph.edge(edge).target)
    } else {
        if let Some(last) = bps.last() {
            return *last;
        }
        graph.absolute_anchor(graph.edge(edge).source)
    }
}

fn reroute_source(graph: &mut LGraph, edge: EdgeId, new_source: PortId) {
    graph.reroute_edge_source(edge, new_source);
}

fn reroute_target(graph: &mut LGraph, edge: EdgeId, new_target: PortId) {
    graph.reroute_edge_target(edge, new_target);
}

fn remove_junction_point(graph: &mut LGraph, edge: EdgeId, point: Vec2) {
    let mut jps: smallvec::SmallVec<Vec2, 4> = graph.edge(edge).properties.get(&JUNCTION_POINTS);
    if jps.is_empty() {
        return;
    }
    let before = jps.len();
    jps.retain(|jp| jp.x != point.x || jp.y != point.y);
    if jps.len() != before {
        graph.edge_mut(edge).properties.set(&JUNCTION_POINTS, jps);
    }
}
