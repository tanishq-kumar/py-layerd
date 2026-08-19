//! Six-step processor that routes edges incident on hierarchical
//! (external) ports once the body of the graph has already been routed.
//!
//! Pipeline dispatch gates this on the presence of external-port replacement
//! dummies set up by `HierarchicalPortConstraintProcessor`.

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
    p5_edge_routing::orthogonal::{direction::RoutingDirection, routing_generator},
    properties::internal::{
        EXT_PORT_REPLACED_DUMMIES, EXT_PORT_REPLACED_DUMMY, EXT_PORT_SIDE, EXT_PORT_SIZE,
        PORT_ANCHOR, PORT_RATIO_OR_POSITION,
    },
};

/// Routes edges incident on hierarchical ports.
pub fn route(graph: &mut LGraph) {
    let mut state = RouterState { northern_extent: 0.0 };

    let ns_dummies = restore_north_south_dummies(graph);
    set_north_south_dummy_coordinates(graph, &ns_dummies);
    route_edges(graph, &ns_dummies, &mut state);
    remove_temporary_north_south_dummies(graph);
    fix_coordinates(graph);
    correct_slanted_edge_segments(graph);
}

struct RouterState {
    /// Vertical space consumed by northern external-port edge routing. Used
    /// to both shift the graph down and grow its height.
    northern_extent: f64,
}

fn restore_north_south_dummies(graph: &mut LGraph) -> Vec<NodeId> {
    let replaced: Vec<NodeId> = graph.properties.get(&EXT_PORT_REPLACED_DUMMIES);
    if replaced.is_empty() {
        return Vec::new();
    }

    // 1a — "restore" each original dummy: it currently has no layer, so it
    //      still sits in the graph's node arena from the constraint
    //      preprocessor. We just need to flip the port side so that routing
    //      can reach it from the graph body.
    for &dummy in &replaced {
        let port_side = graph.node(dummy).properties.get(&EXT_PORT_SIDE);
        // Dummy has exactly one port (the single-owner representation kept by
        // the constraint preprocessor).
        let Some(&dummy_port) = graph.node(dummy).ports.first() else { continue };
        match port_side {
            PortSide::North => graph.port_mut(dummy_port).side = PortSide::South,
            PortSide::South => graph.port_mut(dummy_port).side = PortSide::North,
            _ => {}
        }
    }

    // 1b — find any temporary external-port dummies that currently stand in
    //      for the restored ones and link them back via a fresh edge.
    let layer_count = graph.layers.len();
    let mut node_origin_pairs: Vec<(NodeId, NodeId)> = Vec::new();
    for layer_idx in 0..layer_count {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.to_vec();
        for n in nodes {
            if graph.node(n).node_type != NodeType::ExternalPort {
                continue;
            }
            let Some(replaced_dummy) = graph.node(n).properties.get(&EXT_PORT_REPLACED_DUMMY)
            else {
                continue;
            };
            node_origin_pairs.push((n, replaced_dummy));
        }
    }
    for (node, original) in node_origin_pairs {
        connect_node_to_dummy(graph, node, original);
    }

    // 1c — place the restored dummies in the last layer. Any layer works for
    //      this temporary attachment.
    let target_layer = graph.layers.len().saturating_sub(1);
    for &dummy in &replaced {
        let pos = graph.layers[target_layer].nodes.len();
        graph.insert_node_in_layer(dummy, target_layer, pos);
    }

    replaced
}

/// Adds a new port to `node` on its hierarchical side and connects it to the
/// restored dummy's single port.
fn connect_node_to_dummy(graph: &mut LGraph, node: NodeId, dummy: NodeId) {
    let ext_side = graph.node(node).properties.get(&EXT_PORT_SIDE);
    let out_port = graph.add_port(node, ext_side);
    let Some(&in_port) = graph.node(dummy).ports.first() else {
        return;
    };
    let _edge = graph.add_edge(out_port, in_port);
}

fn set_north_south_dummy_coordinates(graph: &mut LGraph, ns_dummies: &[NodeId]) {
    let constraints = graph.options.port_constraints;
    let graph_padding = graph.padding;
    let offset = graph.offset;
    let graph_width = graph.size.x + graph_padding.left + graph_padding.right;
    let north_y = 0.0 - graph_padding.top - offset.y;
    let south_y = graph.size.y + graph_padding.top + graph_padding.bottom - offset.y;

    let mut northern: Vec<NodeId> = Vec::new();
    let mut southern: Vec<NodeId> = Vec::new();

    for &dummy in ns_dummies {
        // x
        match constraints {
            PortConstraints::Free | PortConstraints::FixedSide | PortConstraints::FixedOrder => {
                calculate_north_south_dummy_positions(graph, dummy);
            }
            PortConstraints::FixedRatio => {
                apply_north_south_dummy_ratio(graph, dummy, graph_width);
                border_to_content_area(graph, dummy, true, false);
            }
            PortConstraints::FixedPos => {
                apply_north_south_dummy_position(graph, dummy);
                border_to_content_area(graph, dummy, true, false);
                // Ensure graph is wide enough.
                let pos_x = graph.node(dummy).position.x + graph.node(dummy).size.x / 2.0;
                graph.size.x = graph.size.x.max(pos_x);
            }
            PortConstraints::Undefined => {}
        }

        // y + bookkeeping
        let side = graph.node(dummy).properties.get(&EXT_PORT_SIDE);
        match side {
            PortSide::North => {
                graph.node_mut(dummy).position.y = north_y;
                northern.push(dummy);
            }
            PortSide::South => {
                graph.node_mut(dummy).position.y = south_y;
                southern.push(dummy);
            }
            _ => {}
        }
    }

    match constraints {
        PortConstraints::Free | PortConstraints::FixedSide => {
            ensure_unique_positions(graph, &mut northern);
            ensure_unique_positions(graph, &mut southern);
        }
        PortConstraints::FixedOrder => {
            restore_proper_order(graph, &mut northern);
            restore_proper_order(graph, &mut southern);
        }
        _ => {}
    }
}

fn calculate_north_south_dummy_positions(graph: &mut LGraph, dummy: NodeId) {
    let Some(&dummy_port) = graph.node(dummy).ports.first() else {
        return;
    };
    let connected: Vec<PortId> = graph
        .port(dummy_port)
        .incoming_edges
        .iter()
        .map(|&e| graph.edge(e).source)
        .chain(graph.port(dummy_port).outgoing_edges.iter().map(|&e| graph.edge(e).target))
        .collect();

    if connected.is_empty() {
        graph.node_mut(dummy).position.x = 0.0;
        return;
    }

    let mut pos_sum = 0.0_f64;
    for cp in &connected {
        let port = graph.port(*cp);
        let owner_pos_x = graph.node(port.owner).position.x;
        pos_sum += owner_pos_x + port.position.x + port.anchor.x;
    }
    let anchor_offset = graph.node(dummy).properties.get(&PORT_ANCHOR).map(|a| a.x).unwrap_or(0.0);
    graph.node_mut(dummy).position.x = pos_sum / connected.len() as f64 - anchor_offset;
}

fn apply_north_south_dummy_ratio(graph: &mut LGraph, dummy: NodeId, width: f64) {
    let anchor_offset = graph.node(dummy).properties.get(&PORT_ANCHOR).map(|a| a.x).unwrap_or(0.0);
    let ratio: f64 = graph.node(dummy).properties.get(&PORT_RATIO_OR_POSITION);
    graph.node_mut(dummy).position.x = width * ratio - anchor_offset;
}

fn apply_north_south_dummy_position(graph: &mut LGraph, dummy: NodeId) {
    let anchor_offset = graph.node(dummy).properties.get(&PORT_ANCHOR).map(|a| a.x).unwrap_or(0.0);
    let pos: f64 = graph.node(dummy).properties.get(&PORT_RATIO_OR_POSITION);
    graph.node_mut(dummy).position.x = pos - anchor_offset;
}

fn ensure_unique_positions(graph: &mut LGraph, dummies: &mut [NodeId]) {
    if dummies.is_empty() {
        return;
    }
    dummies.sort_by(|a, b| {
        graph
            .node(*a)
            .position
            .x
            .partial_cmp(&graph.node(*b).position.x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    assign_ascending_coordinates(graph, dummies);
}

fn restore_proper_order(graph: &mut LGraph, dummies: &mut [NodeId]) {
    if dummies.is_empty() {
        return;
    }
    dummies.sort_by(|a, b| {
        let pa: f64 = graph.node(*a).properties.get(&PORT_RATIO_OR_POSITION);
        let pb: f64 = graph.node(*b).properties.get(&PORT_RATIO_OR_POSITION);
        pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
    });
    assign_ascending_coordinates(graph, dummies);
}

fn assign_ascending_coordinates(graph: &mut LGraph, dummies: &[NodeId]) {
    let spacing = graph.options.spacing.port_port;
    let first = dummies[0];
    let mut next_valid = graph.node(first).position.x
        + graph.node(first).size.x
        + graph.node(first).margin.right
        + spacing;

    for &d in &dummies[1..] {
        let current_x = graph.node(d).position.x;
        let current_size_x = graph.node(d).size.x;
        let margin_left = graph.node(d).margin.left;
        let margin_right = graph.node(d).margin.right;
        let delta = current_x - margin_left - next_valid;
        if delta < 0.0 {
            graph.node_mut(d).position.x -= delta;
        }
        let new_x = graph.node(d).position.x;
        graph.size.x = graph.size.x.max(new_x + current_size_x);
        next_valid = new_x + current_size_x + margin_right + spacing;
    }
}

fn border_to_content_area(graph: &mut LGraph, node: NodeId, horizontal: bool, vertical: bool) {
    let padding = graph.padding;
    let offset = graph.offset;
    let pos = &mut graph.node_mut(node).position;
    if horizontal {
        pos.x -= padding.left + offset.x;
    }
    if vertical {
        pos.y -= padding.top + offset.y;
    }
}

fn route_edges(graph: &mut LGraph, ns_dummies: &[NodeId], state: &mut RouterState) {
    let mut northern_sources: Vec<NodeId> = Vec::new();
    let mut northern_targets: Vec<NodeId> = Vec::new();
    let mut southern_sources: Vec<NodeId> = Vec::new();
    let mut southern_targets: Vec<NodeId> = Vec::new();

    for &dummy in ns_dummies {
        let side: PortSide = graph.node(dummy).properties.get(&EXT_PORT_SIDE);
        match side {
            PortSide::North => {
                northern_targets.push(dummy);
                for e in graph.incoming_edges(dummy) {
                    let owner = graph.port(graph.edge(e).source).owner;
                    if !northern_sources.contains(&owner) {
                        northern_sources.push(owner);
                    }
                }
            }
            PortSide::South => {
                southern_targets.push(dummy);
                for e in graph.incoming_edges(dummy) {
                    let owner = graph.port(graph.edge(e).source).owner;
                    if !southern_sources.contains(&owner) {
                        southern_sources.push(owner);
                    }
                }
            }
            _ => {}
        }
    }

    let node_spacing = graph.options.spacing.node_node_between_layers;
    let edge_spacing = graph.options.spacing.edge_edge_between_layers;

    // Northern routing.
    if !northern_sources.is_empty() {
        let start_pos = -node_spacing - graph.offset.y;
        let mut rng = graph.take_rng();
        let slots = routing_generator::route_edges_between_nodes(
            graph,
            &northern_sources,
            &northern_targets,
            RoutingDirection::SouthToNorth,
            start_pos,
            edge_spacing,
            &mut rng,
        );
        graph.put_rng(rng);
        if slots > 0 {
            let routing_h = node_spacing + (slots as f64 - 1.0) * edge_spacing;
            state.northern_extent = routing_h;
            graph.offset.y += routing_h;
            graph.size.y += routing_h;
        }
    }

    // Southern routing.
    if !southern_sources.is_empty() {
        let start_pos = graph.size.y + node_spacing - graph.offset.y;
        let mut rng = graph.take_rng();
        let slots = routing_generator::route_edges_between_nodes(
            graph,
            &southern_sources,
            &southern_targets,
            RoutingDirection::NorthToSouth,
            start_pos,
            edge_spacing,
            &mut rng,
        );
        graph.put_rng(rng);
        if slots > 0 {
            let routing_h = node_spacing + (slots as f64 - 1.0) * edge_spacing;
            graph.size.y += routing_h;
        }
    }
}

fn remove_temporary_north_south_dummies(graph: &mut LGraph) {
    let mut nodes_to_remove: Vec<NodeId> = Vec::new();

    for layer_idx in 0..graph.layers.len() {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.to_vec();
        for node in nodes {
            if graph.node(node).node_type != NodeType::ExternalPort {
                continue;
            }
            if graph.node(node).properties.get(&EXT_PORT_REPLACED_DUMMY).is_none() {
                continue;
            }

            // Collect ports by role. Up to three ports are expected.
            let mut in_port: Option<PortId> = None;
            let mut out_port: Option<PortId> = None;
            let mut origin_port: Option<PortId> = None;
            for &p in &graph.node(node).ports {
                match graph.port(p).side {
                    PortSide::West => in_port = Some(p),
                    PortSide::East => out_port = Some(p),
                    _ => origin_port = Some(p),
                }
            }
            let (in_p, out_p, origin_p) = match (in_port, out_port, origin_port) {
                (Some(i), Some(o), Some(op)) => (i, o, op),
                _ => continue,
            };

            // The edge to the restored dummy was created in step 1.
            let node_to_origin_edge = match graph.port(origin_p).outgoing_edges.first().copied() {
                Some(e) => e,
                None => continue,
            };

            // Bend-point additions using the origin port position in node local.
            let origin_port_pos = graph.port(origin_p).position;
            let node_pos = graph.node(node).position;
            let anchor =
                Vec2 { x: origin_port_pos.x + node_pos.x, y: origin_port_pos.y + node_pos.y };

            let origin_bends: Vec<Vec2> = graph.edge(node_to_origin_edge).bend_points.to_vec();
            let mut incoming_bends: Vec<Vec2> = Vec::with_capacity(origin_bends.len() + 1);
            incoming_bends.push(anchor);
            incoming_bends.extend(origin_bends.iter().copied());

            let mut outgoing_bends: Vec<Vec2> = origin_bends.iter().rev().copied().collect();
            outgoing_bends.push(anchor);

            // Retrieve the restored original dummy.
            let replaced_dummy = match graph.node(node).properties.get(&EXT_PORT_REPLACED_DUMMY) {
                Some(d) => d,
                None => continue,
            };
            let Some(&replaced_port) = graph.node(replaced_dummy).ports.first() else {
                continue;
            };

            // Reroute input edges.
            let in_edges = graph.move_incoming_edges(in_p, replaced_port);
            for e in in_edges {
                graph.edge_mut(e).bend_points.extend(incoming_bends.iter().copied());
            }

            // Reroute output edges.
            let out_edges = graph.move_outgoing_edges(out_p, replaced_port);
            for e in out_edges {
                let mut new_bends: Vec<Vec2> = outgoing_bends.to_vec();
                new_bends.extend(graph.edge(e).bend_points.iter().copied());
                graph.edge_mut(e).bend_points = new_bends;
            }

            // Disconnect the node-to-origin edge.
            detach_edge(graph, node_to_origin_edge);

            nodes_to_remove.push(node);
        }
    }

    for node in nodes_to_remove {
        detach_node_from_layer(graph, node);
    }
}

fn detach_edge(graph: &mut LGraph, edge: EdgeId) {
    let src = graph.edge(edge).source;
    let tgt = graph.edge(edge).target;
    graph.port_mut(src).outgoing_edges.retain(|e| *e != edge);
    graph.port_mut(tgt).incoming_edges.retain(|e| *e != edge);
}

fn detach_node_from_layer(graph: &mut LGraph, node: NodeId) {
    if let Some(l) = graph.node(node).layer.get()
        && l < graph.layers.len()
    {
        graph.layers[l].nodes.retain(|&n| n != node);
    }
    graph.node_mut(node).layer = None.into();
}

fn fix_coordinates(graph: &mut LGraph) {
    let constraints = graph.options.port_constraints;
    let layer_count = graph.layers.len();
    if layer_count == 0 {
        return;
    }

    fix_coordinates_for_layer(graph, 0, constraints);
    if layer_count > 1 {
        fix_coordinates_for_layer(graph, layer_count - 1, constraints);
    }
}

fn fix_coordinates_for_layer(graph: &mut LGraph, layer_idx: usize, constraints: PortConstraints) {
    let padding = graph.padding;
    let offset = graph.offset;
    let actual_size = Vec2 {
        x: graph.size.x + padding.left + padding.right,
        y: graph.size.y + padding.top + padding.bottom,
    };

    let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

    // Pass 1 — east/west.
    let mut new_actual_height = actual_size.y;
    for &node in &nodes {
        if graph.node(node).node_type != NodeType::ExternalPort {
            continue;
        }
        let side: PortSide = graph.node(node).properties.get(&EXT_PORT_SIDE);
        let ext_size = ext_port_size_of(graph, node);

        match side {
            PortSide::East => {
                graph.node_mut(node).position.x = graph.size.x + padding.right - offset.x;
            }
            PortSide::West => {
                graph.node_mut(node).position.x = -offset.x - padding.left;
            }
            _ => {}
        }

        let required = match (side, constraints) {
            (PortSide::East | PortSide::West, PortConstraints::FixedRatio) => {
                let ratio: f64 = graph.node(node).properties.get(&PORT_RATIO_OR_POSITION);
                let anchor_y =
                    graph.node(node).properties.get(&PORT_ANCHOR).map(|a| a.y).unwrap_or(0.0);
                let new_y = actual_size.y * ratio - anchor_y;
                graph.node_mut(node).position.y = new_y;
                let req = new_y + ext_size.y;
                border_to_content_area(graph, node, false, true);
                req
            }
            (PortSide::East | PortSide::West, PortConstraints::FixedPos) => {
                let pos: f64 = graph.node(node).properties.get(&PORT_RATIO_OR_POSITION);
                let anchor_y =
                    graph.node(node).properties.get(&PORT_ANCHOR).map(|a| a.y).unwrap_or(0.0);
                let new_y = pos - anchor_y;
                graph.node_mut(node).position.y = new_y;
                let req = new_y + ext_size.y;
                border_to_content_area(graph, node, false, true);
                req
            }
            _ => 0.0,
        };

        new_actual_height = new_actual_height.max(required);
    }

    graph.size.y += new_actual_height - actual_size.y;

    // Pass 2 — north/south.
    for &node in &nodes {
        if graph.node(node).node_type != NodeType::ExternalPort {
            continue;
        }
        let side: PortSide = graph.node(node).properties.get(&EXT_PORT_SIDE);
        match side {
            PortSide::North => graph.node_mut(node).position.y = -offset.y - padding.top,
            PortSide::South => {
                graph.node_mut(node).position.y = graph.size.y + padding.bottom - offset.y;
            }
            _ => {}
        }
    }
}

fn ext_port_size_of(graph: &LGraph, node: NodeId) -> Vec2 {
    // Read `EXT_PORT_SIZE` set during the dummy size processor. Fall back
    // to the node's own size so pre-phase-4 setups still behave.
    let stored: Vec2 = graph.node(node).properties.get(&EXT_PORT_SIZE);
    if stored.x == 0.0 && stored.y == 0.0 { graph.node(node).size } else { stored }
}

fn correct_slanted_edge_segments(graph: &mut LGraph) {
    let layer_count = graph.layers.len();
    if layer_count == 0 {
        return;
    }
    correct_slanted_layer(graph, 0);
    if layer_count > 1 {
        correct_slanted_layer(graph, layer_count - 1);
    }
}

fn correct_slanted_layer(graph: &mut LGraph, layer_idx: usize) {
    let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.to_vec();
    for node in nodes {
        if graph.node(node).node_type != NodeType::ExternalPort {
            continue;
        }
        let side: PortSide = graph.node(node).properties.get(&EXT_PORT_SIDE);
        if side != PortSide::East && side != PortSide::West {
            continue;
        }

        let mut edges: Vec<EdgeId> = Vec::new();
        for p in graph.node(node).ports.iter().copied() {
            for &e in &graph.port(p).incoming_edges {
                edges.push(e);
            }
            for &e in &graph.port(p).outgoing_edges {
                edges.push(e);
            }
        }

        for e in edges {
            if graph.edge(e).bend_points.is_empty() {
                continue;
            }
            let src_port = graph.edge(e).source;
            let tgt_port = graph.edge(e).target;
            if graph.port(src_port).owner == node {
                let anchor_y = graph.absolute_anchor(src_port).y;
                if let Some(first) = graph.edge_mut(e).bend_points.first_mut() {
                    first.y = anchor_y;
                }
            }
            if graph.port(tgt_port).owner == node {
                let anchor_y = graph.absolute_anchor(tgt_port).y;
                if let Some(last) = graph.edge_mut(e).bend_points.last_mut() {
                    last.y = anchor_y;
                }
            }
        }
    }
}
