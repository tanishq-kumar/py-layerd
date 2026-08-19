//! Self-loop pre/post processing.
//!
//! `preprocess` installs a `SelfLoopHolder` on every normal node that has
//! self-loops, hides the loop edges from port adjacency lists, and optionally
//! hides ports that are only incident to self-loops. `postprocess` restores the
//! edges, re-attaches hidden ports, and translates the router's node-local
//! bend points back into graph coordinates.

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
    },
    intermediate::self_loop_holder::{SELF_LOOP_HOLDER, SelfLoopHolder},
    math::Vec2,
    options::enums::PortConstraints,
    properties::{
        graph_properties::GraphProperties,
        internal::{GRAPH_PROPERTIES, ORIGINAL_PORT_CONSTRAINTS, SELF_LOOP_EDGES},
    },
};

/// Hides self-loop edges before cycle breaking and layering.
pub fn preprocess(graph: &mut LGraph) {
    let candidate_nodes: Vec<NodeId> = graph
        .layerless_nodes
        .iter()
        .copied()
        .filter(|&nid| SelfLoopHolder::needs_self_loop_processing(graph, nid))
        .collect();

    let mut all_loop_edges: Vec<EdgeId> = Vec::new();

    for node in candidate_nodes {
        let mut holder = SelfLoopHolder::install(graph, node);
        let port_constraints = effective_port_constraints(graph, node);

        // Snapshot current port constraints so the restorer can route the
        // hidden ports back to their original side semantics.
        graph
            .node_mut(node)
            .properties
            .set(&ORIGINAL_PORT_CONSTRAINTS, port_constraints);

        // Hide self-loop edges from port adjacency lists. The reference
        // additionally clears `edge.source` / `edge.target`; that would
        // break our restorer because `EdgeData.source/target` are
        // non-optional `PortId`s. Instead we keep both ends intact on the
        // edge struct and rely on
        // every downstream processor reaching edges via port adjacency
        // (`outgoing_edges` / `incoming_edges` / `graph.edge_at_port`),
        // never iterating the edge arena directly for pipeline logic —
        // see `reversed_edge_restorer.rs` for the only `edges_iter` caller,
        // which runs post-P5 after self-loop restoration.
        let edge_ids: Vec<EdgeId> = holder.sl_edges.iter().map(|e| e.edge_id).collect();
        for &eid in &edge_ids {
            let source_port = graph.edge(eid).source;
            let target_port = graph.edge(eid).target;
            graph.port_mut(source_port).outgoing_edges.retain(|e| *e != eid);
            graph.port_mut(target_port).incoming_edges.retain(|e| *e != eid);
            all_loop_edges.push(eid);
        }

        // Hide pure self-loop ports unless port order is fixed or the node has
        // a nested graph with external ports.
        let order_fixed = port_constraints.is_order_fixed();
        let hierarchy_mode = nested_has_external_ports(graph, node);

        if !order_fixed && !hierarchy_mode {
            let to_hide: Vec<PortId> = holder
                .sl_port_map
                .values()
                .filter(|p| p.had_only_self_loops)
                .map(|p| p.port_id)
                .collect();
            if !to_hide.is_empty() {
                holder.ports_hidden = true;
                for pid in to_hide {
                    if let Some(slot) = holder.sl_port_map.get_mut(&pid) {
                        slot.is_hidden = true;
                    }
                    graph.node_mut(node).ports.retain(|p| *p != pid);
                }
            }
        }

        graph.node_mut(node).properties.set(&SELF_LOOP_HOLDER, Some(holder));
    }

    if !all_loop_edges.is_empty() {
        graph.properties.set(&SELF_LOOP_EDGES, all_loop_edges);
    }
}

/// Restores self-loop edges and offsets their bend points by the node position.
pub fn postprocess(graph: &mut LGraph) {
    let mut targets: Vec<NodeId> = Vec::new();
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            if graph.node(nid).node_type == NodeType::Normal
                && graph.node(nid).properties.has(&SELF_LOOP_HOLDER)
            {
                targets.push(nid);
            }
        }
    }

    for node in targets {
        let holder_opt = graph.node(node).properties.get(&SELF_LOOP_HOLDER);
        let Some(holder) = holder_opt else { continue };

        let node_pos = graph.node(node).position;

        for sl_edge in &holder.sl_edges {
            let eid = sl_edge.edge_id;
            let source = sl_edge.source;
            let target = sl_edge.target;

            // Re-attach to port adjacency lists if not already present.
            if !graph.port(source).outgoing_edges.contains(&eid) {
                graph.port_mut(source).outgoing_edges.push(eid);
            }
            if !graph.port(target).incoming_edges.contains(&eid) {
                graph.port_mut(target).incoming_edges.push(eid);
            }

            // Translate node-local bend points to graph-global coordinates.
            for bp in &mut graph.edge_mut(eid).bend_points {
                bp.x += node_pos.x;
                bp.y += node_pos.y;
            }
        }

        // Apply per-label text positions using the bounding-box anchor the
        // router stored.
        for hyper in &holder.sl_hyper_loops {
            if let Some(slabels) = &hyper.sl_labels
                && slabels.side.is_some()
            {
                slabels.apply_placement(graph, node_pos);
            }
        }
    }

    // Fallback for any leftover graph-level edges that were collected before a
    // holder was installed (older preprocess paths).
    let leftover = graph.properties.get(&SELF_LOOP_EDGES);
    for eid in leftover {
        let source_port = graph.edge(eid).source;
        let target_port = graph.edge(eid).target;
        if !graph.port(source_port).outgoing_edges.contains(&eid) {
            graph.port_mut(source_port).outgoing_edges.push(eid);
        }
        if !graph.port(target_port).incoming_edges.contains(&eid) {
            graph.port_mut(target_port).incoming_edges.push(eid);
        }

        if graph.edge(eid).bend_points.is_empty() {
            // Generate a simple rectangular self-loop trace if no router has
            // already placed bend points on this edge.
            let owner = graph.port(source_port).owner;
            let node_pos = graph.node(owner).position;
            let source_pos = graph.port(source_port).position;
            let loop_height = 20.0;
            let loop_width = 20.0;
            let port_x = node_pos.x + source_pos.x;
            let port_y = node_pos.y;
            graph.edge_mut(eid).bend_points = vec![
                Vec2::new(port_x, port_y - loop_height),
                Vec2::new(port_x + loop_width, port_y - loop_height),
                Vec2::new(port_x + loop_width, port_y + source_pos.y),
            ];
        }
    }
}

/// Look up effective port constraints for a node, falling back to the graph
/// option if no per-node override is set.
fn effective_port_constraints(graph: &LGraph, node: NodeId) -> PortConstraints {
    let per_node = graph.node(node).port_constraints();
    if per_node == PortConstraints::Undefined {
        graph.options.port_constraints
    } else {
        per_node
    }
}

/// True when the node owns a nested graph that itself contains external ports.
fn nested_has_external_ports(graph: &LGraph, node: NodeId) -> bool {
    let Some(nested) = graph.nested(node) else {
        return false;
    };
    let nested_props = nested.properties.get(&GRAPH_PROPERTIES);
    nested_props.contains(GraphProperties::EXTERNAL_PORTS)
}
