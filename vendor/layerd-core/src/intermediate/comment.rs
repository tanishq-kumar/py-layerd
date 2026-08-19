//! Handles nodes marked `COMMENT_BOX`. During preprocessing, comment boxes with
//! exactly one connection to a non-comment node are detached from the graph and
//! recorded on the connected node's `TOP_COMMENTS` / `BOTTOM_COMMENTS`
//! property. Layout proceeds without the comments. During postprocessing, the
//! comments are placed above or below the node and their edges reattached.

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    options::enums::PortConstraints,
    properties::internal::{BOTTOM_COMMENTS, COMMENT_BOX, COMMENT_CONN_PORT, TOP_COMMENTS},
};

/// Detaches single-connection comment boxes from the graph.
pub fn preprocess(graph: &mut LGraph) {
    let candidates: Vec<NodeId> = graph.layerless_nodes.clone();
    let mut to_remove: Vec<NodeId> = Vec::new();

    for box_node in candidates {
        if !graph.node(box_node).properties.get(&COMMENT_BOX) {
            continue;
        }

        let (edge_count, connection) = find_single_connection(graph, box_node);
        if edge_count == 1
            && let Some(edge) = connection
            && port_degree(graph, edge.real_port) == 1
            && !graph.node(edge.real_node).properties.get(&COMMENT_BOX)
        {
            process_box(graph, box_node, edge);
            to_remove.push(box_node);
        } else {
            reverse_oddly_connected_edges(graph, box_node);
        }
    }

    graph.layerless_nodes.retain(|n| !to_remove.contains(n));
}

struct BoxConnection {
    edge: EdgeId,
    box_port: PortId,
    real_port: PortId,
    real_node: NodeId,
    reversed: bool,
}

fn find_single_connection(graph: &LGraph, box_node: NodeId) -> (usize, Option<BoxConnection>) {
    let ports: Vec<PortId> = graph.node(box_node).ports.to_vec();
    let mut edge_count = 0;
    let mut conn: Option<BoxConnection> = None;
    for port_id in ports {
        let port = graph.port(port_id);
        edge_count += port.incoming_edges.len() + port.outgoing_edges.len();
        if port.incoming_edges.len() == 1 {
            let edge = port.incoming_edges[0];
            let real_port = graph.edge(edge).source;
            let real_node = graph.port(real_port).owner;
            conn = Some(BoxConnection {
                edge,
                box_port: port_id,
                real_port,
                real_node,
                reversed: true,
            });
        }
        if port.outgoing_edges.len() == 1 {
            let edge = port.outgoing_edges[0];
            let real_port = graph.edge(edge).target;
            let real_node = graph.port(real_port).owner;
            conn = Some(BoxConnection {
                edge,
                box_port: port_id,
                real_port,
                real_node,
                reversed: false,
            });
        }
    }
    (edge_count, conn)
}

fn port_degree(graph: &LGraph, port: PortId) -> usize {
    let p = graph.port(port);
    p.incoming_edges.len() + p.outgoing_edges.len()
}

fn reverse_oddly_connected_edges(graph: &mut LGraph, box_node: NodeId) {
    let ports: Vec<PortId> = graph.node(box_node).ports.to_vec();
    let mut to_reverse: Vec<EdgeId> = Vec::new();

    for port_id in ports {
        let outgoing: Vec<EdgeId> = graph.port(port_id).outgoing_edges.to_vec();
        for edge_id in outgoing {
            let target_port = graph.edge(edge_id).target;
            if !graph.port(target_port).outgoing_edges.is_empty() {
                to_reverse.push(edge_id);
            }
        }
        let incoming: Vec<EdgeId> = graph.port(port_id).incoming_edges.to_vec();
        for edge_id in incoming {
            let source_port = graph.edge(edge_id).source;
            if !graph.port(source_port).incoming_edges.is_empty() {
                to_reverse.push(edge_id);
            }
        }
    }

    for edge_id in to_reverse {
        graph.reverse_edge(edge_id);
    }
}

fn process_box(graph: &mut LGraph, box_node: NodeId, conn: BoxConnection) {
    let (only_top, only_bottom) = classify_sides(graph, conn.real_node);
    let top_first = decide_top_first(graph, conn.real_node, only_top, only_bottom);

    let (target_key, other_key) = if top_first {
        (&TOP_COMMENTS, &BOTTOM_COMMENTS)
    } else {
        (&BOTTOM_COMMENTS, &TOP_COMMENTS)
    };

    let mut target_boxes: Vec<NodeId> = graph.node(conn.real_node).properties.get(target_key);
    let other_boxes: Vec<NodeId> = graph.node(conn.real_node).properties.get(other_key);

    let use_other = !target_boxes.is_empty()
        && !(if top_first { only_top } else { only_bottom })
        && other_boxes.len() < target_boxes.len();

    if use_other {
        let mut other_target = other_boxes;
        other_target.push(box_node);
        graph.node_mut(conn.real_node).properties.set(other_key, other_target);
    } else {
        target_boxes.push(box_node);
        graph.node_mut(conn.real_node).properties.set(target_key, target_boxes);
    }

    graph
        .node_mut(box_node)
        .properties
        .set(&COMMENT_CONN_PORT, Some(conn.real_port));

    graph.edge_mut(conn.edge).bend_points.clear();

    detach_edge_from_real_port(graph, conn.edge, conn.real_port, conn.reversed);
    remove_hierarchical_port_dummy(graph, conn.real_port);
    let _ = conn.box_port;
}

fn classify_sides(graph: &LGraph, real_node: NodeId) -> (bool, bool) {
    let pc = graph.node(real_node).port_constraints();
    let pc = if matches!(pc, PortConstraints::Undefined) {
        graph.options.port_constraints
    } else {
        pc
    };
    if !pc.is_side_fixed() {
        return (false, false);
    }

    let mut has_north = false;
    let mut has_south = false;

    'outer: for &port_id in &graph.node(real_node).ports {
        for connected in connected_ports(graph, port_id) {
            let other_owner = graph.port(connected).owner;
            if !graph.node(other_owner).properties.get(&COMMENT_BOX) {
                match graph.port(port_id).side {
                    PortSide::North => {
                        has_north = true;
                        break 'outer;
                    }
                    PortSide::South => {
                        has_south = true;
                        break 'outer;
                    }
                    _ => {}
                }
            }
        }
    }

    let only_top = has_south && !has_north;
    let only_bottom = has_north && !has_south;
    (only_top, only_bottom)
}

fn connected_ports(graph: &LGraph, port: PortId) -> Vec<PortId> {
    let mut out = Vec::new();
    for &edge in &graph.port(port).outgoing_edges {
        out.push(graph.edge(edge).target);
    }
    for &edge in &graph.port(port).incoming_edges {
        out.push(graph.edge(edge).source);
    }
    out
}

fn decide_top_first(graph: &LGraph, real_node: NodeId, only_top: bool, only_bottom: bool) -> bool {
    if !only_top && !only_bottom && !graph.node(real_node).labels.is_empty() {
        let labels = graph.node(real_node).labels.clone();
        let size_y = graph.node(real_node).size.y;
        let mut label_pos_sum = 0.0;
        for &label_id in labels.iter() {
            let label = graph.label(label_id);
            label_pos_sum += label.position.y + label.size.y / 2.0;
        }
        let avg = label_pos_sum / labels.len() as f64;
        avg >= size_y / 2.0
    } else {
        !only_bottom
    }
}

/// Remove `edge` from `real_port`'s adjacency list without removing the edge
/// from the arena, leaving the box side connected.
fn detach_edge_from_real_port(graph: &mut LGraph, edge: EdgeId, real_port: PortId, reversed: bool) {
    if reversed {
        graph.port_mut(real_port).outgoing_edges.retain(|e| *e != edge);
    } else {
        graph.port_mut(real_port).incoming_edges.retain(|e| *e != edge);
    }
}

fn remove_hierarchical_port_dummy(graph: &mut LGraph, port: PortId) {
    let dummy = graph.port(port).port_dummy;
    let Some(dummy_id) = dummy else { return };
    if let Some(layer_idx) = graph.node(dummy_id).layer.get() {
        graph.layers[layer_idx].nodes.retain(|n| *n != dummy_id);
        if graph.layers[layer_idx].nodes.is_empty() {
            graph.layers.remove(layer_idx);
            let total = graph.layers.len();
            for i in 0..total {
                let node_ids: Vec<NodeId> = graph.layers[i].nodes.clone();
                for node_id in node_ids {
                    graph.node_mut(node_id).layer = Some(i).into();
                }
            }
        }
    }
}

/// Extends each node's margin to reserve space for attached comment boxes.
pub fn calculate_node_margin(graph: &mut LGraph) {
    let comment_comment = graph.options.spacing.comment_comment;
    let comment_node = graph.options.spacing.comment_node;

    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            let top_boxes = graph.node(node_id).properties.get(&TOP_COMMENTS);
            let bottom_boxes = graph.node(node_id).properties.get(&BOTTOM_COMMENTS);
            if top_boxes.is_empty() && bottom_boxes.is_empty() {
                continue;
            }

            let node_size_x = graph.node(node_id).size.x;

            let mut top_width = 0.0;
            let mut extra_top = 0.0;
            if !top_boxes.is_empty() {
                let mut max_height = 0.0_f64;
                for &box_id in &top_boxes {
                    let size = graph.node(box_id).size;
                    max_height = max_height.max(size.y);
                    top_width += size.x;
                }
                top_width += comment_comment * (top_boxes.len() as f64 - 1.0);
                extra_top = max_height + comment_node;
            }

            let mut bottom_width = 0.0;
            let mut extra_bottom = 0.0;
            if !bottom_boxes.is_empty() {
                let mut max_height = 0.0_f64;
                for &box_id in &bottom_boxes {
                    let size = graph.node(box_id).size;
                    max_height = max_height.max(size.y);
                    bottom_width += size.x;
                }
                bottom_width += comment_comment * (bottom_boxes.len() as f64 - 1.0);
                extra_bottom = max_height + comment_node;
            }

            let margin = &mut graph.node_mut(node_id).margin;
            margin.top += extra_top;
            margin.bottom += extra_bottom;

            let max_comment_width = top_width.max(bottom_width);
            if max_comment_width > node_size_x {
                let protrusion = (max_comment_width - node_size_x) / 2.0;
                let margin = &mut graph.node_mut(node_id).margin;
                margin.left = margin.left.max(protrusion);
                margin.right = margin.right.max(protrusion);
            }
        }
    }
}

/// Reattaches comment boxes above and below their connected nodes.
pub fn postprocess(graph: &mut LGraph) {
    let comment_comment = graph.options.spacing.comment_comment;

    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let mut boxes_to_add: Vec<NodeId> = Vec::new();
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            let top_boxes: Vec<NodeId> = graph.node(node_id).properties.get(&TOP_COMMENTS);
            let bottom_boxes: Vec<NodeId> = graph.node(node_id).properties.get(&BOTTOM_COMMENTS);

            if top_boxes.is_empty() && bottom_boxes.is_empty() {
                continue;
            }

            process_node_with_comments(graph, node_id, &top_boxes, &bottom_boxes, comment_comment);

            boxes_to_add.extend(top_boxes);
            boxes_to_add.extend(bottom_boxes);
        }

        for box_id in boxes_to_add {
            graph.node_mut(box_id).layer = Some(layer_idx).into();
            graph.layers[layer_idx].nodes.push(box_id);
        }
    }
}

fn process_node_with_comments(
    graph: &mut LGraph,
    node: NodeId,
    top_boxes: &[NodeId],
    bottom_boxes: &[NodeId],
    comment_comment: f64,
) {
    let node_pos = graph.node(node).position;
    let node_size = graph.node(node).size;
    let margin = *graph.node(node).margin;

    if !top_boxes.is_empty() {
        let mut boxes_width = comment_comment * (top_boxes.len() as f64 - 1.0);
        let mut max_height = 0.0_f64;
        for &box_id in top_boxes {
            let size = graph.node(box_id).size;
            boxes_width += size.x;
            max_height = max_height.max(size.y);
        }

        let mut x = node_pos.x - (boxes_width - node_size.x) / 2.0;
        let base_line = node_pos.y - margin.top + max_height;
        let anchor_inc = node_size.x / (top_boxes.len() as f64 + 1.0);
        let mut anchor_x = anchor_inc;

        for &box_id in top_boxes {
            let box_size = graph.node(box_id).size;
            graph.node_mut(box_id).position.x = x;
            graph.node_mut(box_id).position.y = base_line - box_size.y;
            x += box_size.x + comment_comment;

            if let Some(box_port) = reconnect_box_edge(graph, box_id) {
                let box_anchor = graph.port(box_port).anchor;
                graph.port_mut(box_port).position.x = box_size.x / 2.0 - box_anchor.x;
                graph.port_mut(box_port).position.y = box_size.y;
            }
            let node_port = graph.node(box_id).properties.get(&COMMENT_CONN_PORT);
            if let Some(np) = node_port {
                let degree = port_degree(graph, np);
                if degree == 1 {
                    let anchor = graph.port(np).anchor;
                    graph.port_mut(np).position.x = anchor_x - anchor.x;
                    graph.port_mut(np).position.y = 0.0;
                }
            }
            anchor_x += anchor_inc;
        }
    }

    if !bottom_boxes.is_empty() {
        let mut boxes_width = comment_comment * (bottom_boxes.len() as f64 - 1.0);
        let mut max_height = 0.0_f64;
        for &box_id in bottom_boxes {
            let size = graph.node(box_id).size;
            boxes_width += size.x;
            max_height = max_height.max(size.y);
        }

        let mut x = node_pos.x - (boxes_width - node_size.x) / 2.0;
        let base_line = node_pos.y + node_size.y + margin.bottom - max_height;
        let anchor_inc = node_size.x / (bottom_boxes.len() as f64 + 1.0);
        let mut anchor_x = anchor_inc;

        for &box_id in bottom_boxes {
            let box_size = graph.node(box_id).size;
            graph.node_mut(box_id).position.x = x;
            graph.node_mut(box_id).position.y = base_line;
            x += box_size.x + comment_comment;

            if let Some(box_port) = reconnect_box_edge(graph, box_id) {
                let box_anchor = graph.port(box_port).anchor;
                graph.port_mut(box_port).position.x = box_size.x / 2.0 - box_anchor.x;
                graph.port_mut(box_port).position.y = 0.0;
            }
            let node_port = graph.node(box_id).properties.get(&COMMENT_CONN_PORT);
            if let Some(np) = node_port {
                let degree = port_degree(graph, np);
                if degree == 1 {
                    let anchor = graph.port(np).anchor;
                    graph.port_mut(np).position.x = anchor_x - anchor.x;
                    graph.port_mut(np).position.y = node_size.y;
                }
            }
            anchor_x += anchor_inc;
        }
    }
}

/// Reattach the edge that was detached from the real node's port during
/// preprocessing. Returns the box-side port that owns the edge.
fn reconnect_box_edge(graph: &mut LGraph, box_node: NodeId) -> Option<PortId> {
    let node_port = graph.node(box_node).properties.get(&COMMENT_CONN_PORT)?;
    let ports: Vec<PortId> = graph.node(box_node).ports.to_vec();
    for box_port in ports {
        let outgoing: Vec<EdgeId> = graph.port(box_port).outgoing_edges.to_vec();
        if let Some(edge_id) = outgoing.into_iter().next() {
            graph.port_mut(node_port).incoming_edges.push(edge_id);
            let node_owner = graph.port_owner(node_port);
            let edge = graph.edge_mut(edge_id);
            edge.target = node_port;
            edge.target_owner = node_owner;
            return Some(box_port);
        }
        let incoming: Vec<EdgeId> = graph.port(box_port).incoming_edges.to_vec();
        if let Some(edge_id) = incoming.into_iter().next() {
            graph.port_mut(node_port).outgoing_edges.push(edge_id);
            let node_owner = graph.port_owner(node_port);
            let edge = graph.edge_mut(edge_id);
            edge.source = node_port;
            edge.source_owner = node_owner;
            return Some(box_port);
        }
    }
    None
}
