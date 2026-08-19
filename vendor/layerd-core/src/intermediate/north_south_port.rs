use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::{EdgeRoutingStrategy, OrderingStrategy, PortConstraints},
    properties::internal::{
        ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES, BARYCENTER_ASSOCIATES, CROSSING_HINT,
        IN_LAYER_LAYOUT_UNIT, IN_LAYER_SUCCESSOR_CONSTRAINTS, JUNCTION_POINTS, ORIGIN_NODE,
        ORIGIN_PORT, SPLINE_NS_PORT_Y_COORD,
    },
};

/// Mirror `NorthSouthPortPreprocessor.modelOrderNorthSouthInputReversing` L372-386.
///
/// Partition `ports` by whether each has an incoming edge; reverse the
/// incoming sub-list, then append the outgoing sub-list. Applied only when
/// `CONSIDER_MODEL_ORDER_STRATEGY != NONE`.
fn model_order_north_south_input_reversing(graph: &LGraph, ports: Vec<PortId>) -> Vec<PortId> {
    let mut incoming: Vec<PortId> = Vec::with_capacity(ports.len());
    let mut outgoing: Vec<PortId> = Vec::new();
    for pid in ports {
        if !graph.port(pid).incoming_edges.is_empty() {
            incoming.push(pid);
        } else {
            outgoing.push(pid);
        }
    }
    incoming.reverse();
    incoming.extend(outgoing);
    incoming
}

fn sort_node_ports_for_north_south_processing(graph: &mut LGraph, node_id: NodeId) {
    let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
    let port_count = ports.len() as i32;
    let mut next_input = 0i32;
    let mut next_bidirectional = port_count;
    let mut next_output = 2 * port_count;
    let mut ids: Vec<(PortId, i32)> = Vec::with_capacity(ports.len());

    for &port_id in &ports {
        let side = graph.port(port_id).side;
        let id = if matches!(side, PortSide::North | PortSide::South) {
            let has_incoming = !graph.port(port_id).incoming_edges.is_empty();
            let has_outgoing = !graph.port(port_id).outgoing_edges.is_empty();
            if has_incoming && has_outgoing {
                let id = next_bidirectional;
                next_bidirectional += 1;
                id
            } else if has_incoming {
                let id = next_input;
                next_input += 1;
                id
            } else if has_outgoing {
                let id = next_output;
                next_output += 1;
                id
            } else {
                let id = next_input;
                next_input += 1;
                id
            }
        } else {
            -1
        };
        ids.push((port_id, id));
    }

    let port_sort_id = |port_id: PortId| -> i32 {
        ids.iter()
            .find(|(candidate, _)| *candidate == port_id)
            .map(|(_, id)| *id)
            .unwrap_or(-1)
    };
    let mut sorted = ports;
    sorted.sort_by(|a, b| {
        let side_a = graph.port(*a).side;
        let side_b = graph.port(*b).side;
        if side_a != side_b {
            return (side_a as u8).cmp(&(side_b as u8));
        }

        let id_a = port_sort_id(*a);
        let id_b = port_sort_id(*b);
        if id_a == id_b {
            std::cmp::Ordering::Equal
        } else if side_a == PortSide::North {
            id_a.cmp(&id_b)
        } else {
            id_b.cmp(&id_a)
        }
    });

    if graph.node(node_id).ports.as_slice() != sorted.as_slice() {
        graph.node_mut(node_id).ports = sorted.into();
        graph.cache_port_sides(node_id);
        graph.bump_node_order_version(node_id);
    }
}

/// Creates dummy nodes for ports on North/South sides of nodes.
///
/// The layered algorithm works with East/West ports only. This preprocessor
/// creates `NorthSouthPort` dummy nodes in the same layer for each N/S port
/// and reroutes edges through those dummies.
///
/// Uses one dummy per port, with successor constraints linking N dummies
/// -> regular node -> S dummies.
///
/// North-south port preprocessor.
///
/// # Deferred work
///
/// `NorthSouthPortPreprocessor` supports the following extras that are
/// not implemented yet:
/// * N → S self-loop handling — creates extra dummies for self-loops that
///   enter a N port and leave from a S port. Requires the self-loop
///   subsystem.
/// * Same-side self-loop handling (N-N / S-S).
/// * Model-order strategy integration — tied to the global model-order
///   strategy flag; requires coordination with P1 / P3.
/// * Old `USE_NEW_APPROACH = false` path — the in/out port pairing heuristic
///   used before the per-port dummy model. Not needed when the new approach
///   is correct.
pub fn preprocess(graph: &mut LGraph) {
    for layer_idx in 0..graph.layers.len() {
        // Snapshot the current layer nodes to avoid borrow issues.
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        // Track insertion offset as we add dummies before/after nodes.
        let mut insert_offset: usize = 0;

        for (orig_pos, &node_id) in node_ids.iter().enumerate() {
            // Only process normal nodes with fixed port sides.
            if graph.node(node_id).node_type != NodeType::Normal {
                continue;
            }
            let port_constraints = graph.node(node_id).port_constraints();
            // Undefined port constraints defer to the graph-level setting
            // (consistent with the rest of the pipeline); other cases must
            // have side-fixed constraints set explicitly.
            let effective_constraints = if port_constraints == PortConstraints::Undefined {
                graph.options.port_constraints
            } else {
                port_constraints
            };
            if !effective_constraints.is_side_fixed() {
                continue;
            }

            // Set the node as its own layout unit
            graph.node_mut(node_id).properties.set(&IN_LAYER_LAYOUT_UNIT, Some(node_id));

            // Self-loop pre-pass
            // Before the main in/out/inOut bucketing, classify self-loops
            // rooted on N/S ports and route them through specialised dummy
            // nodes. Three cases:
            //   - Same-side (N-N or S-S): one dummy with W input + E output.
            //   - North -> South: two dummies, each with one E port,
            //     connected by the (rerouted) self-loop.
            //   - South -> North: not generated by the self-loop
            //     preprocessor, so it is ignored here.
            //
            // Self-loop edges are detached from the original ports' adjacency
            // lists so the downstream regular dummy-creation loop skips them.
            let (same_side_loops, north_south_loops) = collect_self_loops(graph, node_id);
            let mut self_loop_north_dummies: Vec<NodeId> = Vec::new();
            let mut self_loop_south_dummies: Vec<NodeId> = Vec::new();
            for eid in &same_side_loops {
                let dummy = create_same_side_self_loop_dummy(graph, node_id, *eid);
                // Same-side N-N goes on the "north" visual band;
                // S-S on the "south" visual band. We pick by the source
                // port's side before detachment — safe because we recorded
                // the original endpoints inside `collect_self_loops`.
                let src_side = graph.port(graph.edge(*eid).source).side;
                match src_side {
                    PortSide::North => self_loop_north_dummies.push(dummy),
                    PortSide::South => self_loop_south_dummies.push(dummy),
                    _ => {}
                }
            }
            for eid in &north_south_loops {
                let (north_d, south_d) = create_north_south_self_loop_dummies(graph, node_id, *eid);
                // North dummies stack at the head of the list (reversed
                // creation order); south dummies are appended.
                self_loop_north_dummies.insert(0, north_d);
                self_loop_south_dummies.push(south_d);
            }

            if !effective_constraints.is_order_fixed()
                && graph.options.ordering_strategy == OrderingStrategy::None
            {
                sort_node_ports_for_north_south_processing(graph, node_id);
            }

            let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
            let mut north_ports: Vec<PortId> = ports
                .iter()
                .copied()
                .filter(|&pid| graph.port(pid).side == PortSide::North)
                .collect();
            let mut south_ports: Vec<PortId> = ports
                .iter()
                .copied()
                .filter(|&pid| graph.port(pid).side == PortSide::South)
                .collect();
            south_ports.reverse();

            // When a model-order strategy is active, reverse the incoming-edge
            // prefix of each side's port list so incoming ports appear in
            // reverse model order while outgoing ports keep their forward
            // order.
            if graph.options.ordering_strategy != OrderingStrategy::None {
                north_ports = model_order_north_south_input_reversing(graph, north_ports);
                south_ports = model_order_north_south_input_reversing(graph, south_ports);
            }

            if north_ports.is_empty() && south_ports.is_empty() {
                continue;
            }

            // Create dummies for northern ports.
            // Each gets its own dummy. The insert point is before the current node.
            let current_pos = orig_pos + insert_offset;

            let north_dummies = create_regular_dummies_for_ports(graph, node_id, &north_ports);
            for &dummy in &north_dummies {
                graph.node_mut(dummy).properties.set(&IN_LAYER_LAYOUT_UNIT, Some(node_id));

                // Successor constraint: dummy -> node, unless the origin port
                // is marked `ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES`.
                let allow_switch = dummy_origin_port(graph, dummy)
                    .map(|p| graph.port(p).properties.get(&ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES))
                    .unwrap_or(false);
                if !allow_switch {
                    let mut succ =
                        graph.node(dummy).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
                    succ.push(node_id);
                    graph.node_mut(dummy).properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS, succ);
                }
            }

            // Merge self-loop-driven north dummies at the front of the
            // north list. The opposing-side list is prepended, which becomes
            // the north dummies for a N→S self-loop.
            let mut all_north_dummies: Vec<NodeId> = self_loop_north_dummies.clone();
            all_north_dummies.extend(north_dummies.iter().copied());

            // Insert north dummies before the current node. Always inserting
            // at the same fixed index shifts previously inserted dummies one
            // slot to the right, so dummies enter the layer in reverse
            // creation order.
            for &dummy in &all_north_dummies {
                graph.node_mut(dummy).layer = Some(layer_idx).into();
                graph.layers[layer_idx].nodes.insert(current_pos, dummy);
                insert_offset += 1;
            }
            // Track each north dummy as a barycenter associate so the P3
            // heuristic links the dummies to the regular node when
            // computing layer-sweep positions.
            if !all_north_dummies.is_empty() {
                let mut associates = graph.node(node_id).properties.get(&BARYCENTER_ASSOCIATES);
                associates.extend(all_north_dummies.iter().copied());
                graph.node_mut(node_id).properties.set(&BARYCENTER_ASSOCIATES, associates);
            }

            let south_dummies = create_regular_dummies_for_ports(graph, node_id, &south_ports);
            for &dummy in &south_dummies {
                graph.node_mut(dummy).properties.set(&IN_LAYER_LAYOUT_UNIT, Some(node_id));
            }

            // Successor constraint: node -> each south dummy, unless the
            // origin port is marked `ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES`.
            // The flag is read from the dummy's first port (W input or E
            // output, see `create_dummy_for_port`) via `ORIGIN_PORT`, not
            // from the dummy node.
            for &dummy in &south_dummies {
                let allow_switch = dummy_origin_port(graph, dummy)
                    .map(|p| graph.port(p).properties.get(&ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES))
                    .unwrap_or(false);
                if !allow_switch {
                    let mut succ =
                        graph.node(node_id).properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
                    succ.push(dummy);
                    graph.node_mut(node_id).properties.set(&IN_LAYER_SUCCESSOR_CONSTRAINTS, succ);
                }
            }

            // Append self-loop-driven south dummies to the south list (used
            // for all same-side S-S loops plus the south side of a N→S loop).
            let mut all_south_dummies: Vec<NodeId> = south_dummies.clone();
            all_south_dummies.extend(self_loop_south_dummies.iter().copied());

            // Insert south dummies after the current node
            let after_node_pos = current_pos + all_north_dummies.len() + 1;
            for (i, &dummy) in all_south_dummies.iter().enumerate() {
                let insert_pos = after_node_pos + i;
                graph.node_mut(dummy).layer = Some(layer_idx).into();
                graph.layers[layer_idx].nodes.insert(insert_pos, dummy);
                insert_offset += 1;
            }
            if !all_south_dummies.is_empty() {
                let mut associates = graph.node(node_id).properties.get(&BARYCENTER_ASSOCIATES);
                associates.extend(all_south_dummies.iter().copied());
                graph.node_mut(node_id).properties.set(&BARYCENTER_ASSOCIATES, associates);
            }
        }
    }
}

/// Creates a NorthSouthPort dummy node for a given port, rerouting its edges.
fn create_dummy_for_port(
    graph: &mut LGraph,
    origin_node: NodeId,
    port_id: PortId,
    has_incoming: bool,
    has_outgoing: bool,
) -> NodeId {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::NorthSouthPort;
    graph.layerless_nodes.retain(|&n| n != dummy);

    graph.node_mut(dummy).properties.set(&ORIGIN_NODE, Some(origin_node));
    // Every dummy node locks its port positions so downstream port-sort
    // passes do not reshuffle the inputs we have just rerouted.
    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);

    let mut crossing_hint = 0;

    // Handle incoming edges: create a West-side port on the dummy.
    if has_incoming {
        let dummy_input = graph.add_port(dummy, PortSide::West);
        graph.port_mut(dummy_input).properties.set(&ORIGIN_PORT, Some(port_id));

        // Reroute all incoming edges from the original port to this dummy port.
        graph.move_incoming_edges(port_id, dummy_input);

        // Let the original port know about its dummy
        graph.port_mut(port_id).port_dummy = Some(dummy);

        crossing_hint += 1;
    }

    // Handle outgoing edges: create an East-side port on the dummy.
    if has_outgoing {
        let dummy_output = graph.add_port(dummy, PortSide::East);
        graph.port_mut(dummy_output).properties.set(&ORIGIN_PORT, Some(port_id));

        // Reroute all outgoing edges from the original port to this dummy port.
        graph.move_outgoing_edges(port_id, dummy_output);

        // Let the original port know about its dummy
        graph.port_mut(port_id).port_dummy = Some(dummy);

        crossing_hint += 1;
    }

    graph.node_mut(dummy).properties.set(&CROSSING_HINT, crossing_hint);

    dummy
}

fn create_regular_dummies_for_ports(
    graph: &mut LGraph,
    origin_node: NodeId,
    ports: &[PortId],
) -> Vec<NodeId> {
    let mut incoming_ports = Vec::new();
    let mut outgoing_ports = Vec::new();
    let mut bidirectional_ports = Vec::new();

    for &port_id in ports {
        let has_incoming = !graph.port(port_id).incoming_edges.is_empty();
        let has_outgoing = !graph.port(port_id).outgoing_edges.is_empty();
        if has_incoming && has_outgoing {
            bidirectional_ports.push(port_id);
        } else if has_incoming {
            incoming_ports.push(port_id);
        } else if has_outgoing {
            outgoing_ports.push(port_id);
        }
    }

    let mut dummies =
        Vec::with_capacity(incoming_ports.len() + outgoing_ports.len() + bidirectional_ports.len());
    for port_id in incoming_ports {
        dummies.push(create_dummy_for_port(graph, origin_node, port_id, true, false));
    }
    for port_id in outgoing_ports {
        dummies.push(create_dummy_for_port(graph, origin_node, port_id, false, true));
    }
    for port_id in bidirectional_ports {
        dummies.push(create_dummy_for_port(graph, origin_node, port_id, true, true));
    }
    dummies
}

fn dummy_origin_port(graph: &LGraph, dummy: NodeId) -> Option<PortId> {
    graph
        .node(dummy)
        .ports
        .first()
        .copied()
        .and_then(|port_id| graph.port(port_id).properties.get(&ORIGIN_PORT))
}

/// Classify self-loop edges rooted at N/S ports of `node_id`. Returns two
/// lists: same-side loops (N-N / S-S) and north->south loops. Each returned
/// edge is removed from its source/target port adjacency lists so the main
/// dummy-creation loop ignores it.
fn collect_self_loops(graph: &mut LGraph, node_id: NodeId) -> (Vec<EdgeId>, Vec<EdgeId>) {
    let mut same_side: Vec<EdgeId> = Vec::new();
    let mut north_south: Vec<EdgeId> = Vec::new();

    let port_ids: Vec<PortId> = graph.node(node_id).ports.iter().copied().collect();
    for pid in &port_ids {
        let side = graph.port(*pid).side;
        if !matches!(side, PortSide::North | PortSide::South) {
            continue;
        }
        for &eid in &graph.port(*pid).outgoing_edges {
            let tgt_port = graph.edge(eid).target;
            if graph.port(tgt_port).owner != node_id {
                continue;
            }
            let tgt_side = graph.port(tgt_port).side;
            if !matches!(tgt_side, PortSide::North | PortSide::South) {
                continue;
            }
            if side == tgt_side {
                same_side.push(eid);
            } else if side == PortSide::North && tgt_side == PortSide::South {
                // Only N->S is expected; the cycle orientation step prevents
                // S->N from reaching this pass.
                north_south.push(eid);
            }
        }
    }

    // Detach from port adjacency lists.
    for eid in same_side.iter().chain(north_south.iter()) {
        let src = graph.edge(*eid).source;
        let tgt = graph.edge(*eid).target;
        graph.port_mut(src).outgoing_edges.retain(|&e| e != *eid);
        graph.port_mut(tgt).incoming_edges.retain(|&e| e != *eid);
    }

    (same_side, north_south)
}

/// Build the single dummy for a same-side (N-N or S-S) self-loop, with a
/// West input port (origin = target) and an East output port (origin =
/// source). `CROSSING_HINT = 2` because the dummy contributes two hyperedge
/// crossings for the counting pass.
fn create_same_side_self_loop_dummy(
    graph: &mut LGraph,
    origin_node: NodeId,
    self_loop_edge: EdgeId,
) -> NodeId {
    let src_port = graph.edge(self_loop_edge).source;
    let tgt_port = graph.edge(self_loop_edge).target;

    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = NodeType::NorthSouthPort;
    graph.layerless_nodes.retain(|&n| n != dummy);
    graph.node_mut(dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.node_mut(dummy).origin_edge = Some(self_loop_edge);
    graph.node_mut(dummy).properties.set(&ORIGIN_NODE, Some(origin_node));
    graph.node_mut(dummy).properties.set(&CROSSING_HINT, 2);

    let west_in = graph.add_port(dummy, PortSide::West);
    graph.port_mut(west_in).properties.set(&ORIGIN_PORT, Some(tgt_port));

    let east_out = graph.add_port(dummy, PortSide::East);
    graph.port_mut(east_out).properties.set(&ORIGIN_PORT, Some(src_port));

    // Remember the dummy on both original ports so the restorer can find it.
    graph.port_mut(src_port).port_dummy = Some(dummy);
    graph.port_mut(tgt_port).port_dummy = Some(dummy);

    dummy
}

/// Build the paired (north, south) dummies for a N->S self-loop. Each
/// dummy has a single East port; the self-loop edge is rerouted from the
/// north dummy's output to the south dummy's input. `CROSSING_HINT = 1`
/// on both dummies.
fn create_north_south_self_loop_dummies(
    graph: &mut LGraph,
    origin_node: NodeId,
    self_loop_edge: EdgeId,
) -> (NodeId, NodeId) {
    let src_port = graph.edge(self_loop_edge).source;
    let tgt_port = graph.edge(self_loop_edge).target;

    // North dummy: East-side output port, origin = source.
    let north_dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(north_dummy).node_type = NodeType::NorthSouthPort;
    graph.layerless_nodes.retain(|&n| n != north_dummy);
    graph.node_mut(north_dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.node_mut(north_dummy).properties.set(&ORIGIN_NODE, Some(origin_node));
    graph.node_mut(north_dummy).properties.set(&CROSSING_HINT, 1);
    let north_out = graph.add_port(north_dummy, PortSide::East);
    graph.port_mut(north_out).properties.set(&ORIGIN_PORT, Some(src_port));

    // South dummy: East-side input port, origin = target.
    let south_dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(south_dummy).node_type = NodeType::NorthSouthPort;
    graph.layerless_nodes.retain(|&n| n != south_dummy);
    graph.node_mut(south_dummy).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.node_mut(south_dummy).properties.set(&ORIGIN_NODE, Some(origin_node));
    graph.node_mut(south_dummy).properties.set(&CROSSING_HINT, 1);
    let south_in = graph.add_port(south_dummy, PortSide::East);
    graph.port_mut(south_in).properties.set(&ORIGIN_PORT, Some(tgt_port));

    // Reroute: now self_loop_edge goes north_out → south_in.
    let north_owner = graph.port_owner(north_out);
    let south_owner = graph.port_owner(south_in);
    let edge = graph.edge_mut(self_loop_edge);
    edge.source = north_out;
    edge.target = south_in;
    edge.source_owner = north_owner;
    edge.target_owner = south_owner;
    graph.port_mut(north_out).outgoing_edges.push(self_loop_edge);
    graph.port_mut(south_in).incoming_edges.push(self_loop_edge);

    graph.port_mut(src_port).port_dummy = Some(north_dummy);
    graph.port_mut(tgt_port).port_dummy = Some(south_dummy);

    (north_dummy, south_dummy)
}

/// Removes NorthSouthPort dummy nodes and reroutes edges back to original ports.
///
/// Also adds bend points at the dummy positions for proper edge routing.
///
/// North-south port postprocessor.
pub fn postprocess(graph: &mut LGraph) {
    let mut dummies_to_remove = Vec::new();
    let edge_routing = graph.options.edge_routing;

    for layer_idx in 0..graph.layers.len() {
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        for &node_id in &node_ids {
            if graph.node(node_id).node_type != NodeType::NorthSouthPort {
                continue;
            }
            dummies_to_remove.push(node_id);

            // SPLINES routing skips the bend-point insertion entirely;
            // instead, mark the dummy's y coord on each origin port via
            // `SPLINE_NS_PORT_Y_COORD` so the downstream spline pipeline
            // (`FinalSplineBendpointsCalculator`) can pick it up, and reroute
            // every edge back to the origin port.
            if matches!(edge_routing, EdgeRoutingStrategy::Splines) {
                let dummy_y = graph.node(node_id).position.y;
                let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
                for &port_id in &ports {
                    let Some(origin_port) = graph.port(port_id).properties.get(&ORIGIN_PORT) else {
                        continue;
                    };
                    graph.port_mut(origin_port).properties.set(&SPLINE_NS_PORT_Y_COORD, dummy_y);
                    graph.move_incoming_edges(port_id, origin_port);
                    graph.move_outgoing_edges(port_id, origin_port);
                }
                continue;
            }

            // Same-side (N-N / S-S) self-loop dummies carry
            // `ORIGIN_EDGE = LEdge`. The generic loop below cannot reroute
            // them because the preprocessor detaches the self-loop edge from
            // every port's adjacency list while keeping its source/target
            // fields on the original ports. Handle these here: re-insert the
            // edge into both origin ports' adjacency lists and append two
            // bend points (one on the source anchor, one on the target
            // anchor, both at the dummy's y).
            if let Some(edge_id) = graph.node(node_id).origin_edge {
                let dummy_y = graph.node(node_id).position.y;
                let src_port = graph.edge(edge_id).source;
                let tgt_port = graph.edge(edge_id).target;
                let src_owner = graph.port(src_port).owner;
                let tgt_owner = graph.port(tgt_port).owner;
                let src_node_pos = graph.node(src_owner).position;
                let tgt_node_pos = graph.node(tgt_owner).position;
                let src_port_pos = graph.port(src_port).position;
                let tgt_port_pos = graph.port(tgt_port).position;
                let src_anchor = graph.port(src_port).anchor;
                let tgt_anchor = graph.port(tgt_port).anchor;
                let src_x = src_node_pos.x + src_port_pos.x + src_anchor.x;
                let tgt_x = tgt_node_pos.x + tgt_port_pos.x + tgt_anchor.x;
                graph.port_mut(src_port).outgoing_edges.push(edge_id);
                graph.port_mut(tgt_port).incoming_edges.push(edge_id);
                graph.edge_mut(edge_id).bend_points.push(Vec2::new(src_x, dummy_y));
                graph.edge_mut(edge_id).bend_points.push(Vec2::new(tgt_x, dummy_y));
                continue;
            }

            let ports: Vec<PortId> = graph.node(node_id).ports.to_vec();

            // Determine whether all dummy ports share the same origin port.
            // Only then are junction points added to each edge (otherwise
            // they'd land on mismatched endpoints).
            let same_origin_port = if ports.len() >= 2 {
                let first_origin = graph.port(ports[0]).properties.get(&ORIGIN_PORT);
                first_origin.is_some()
                    && ports[1..]
                        .iter()
                        .all(|&p| graph.port(p).properties.get(&ORIGIN_PORT) == first_origin)
            } else {
                false
            };

            for &port_id in &ports {
                let origin_port = graph.port(port_id).properties.get(&ORIGIN_PORT);
                if origin_port.is_none() {
                    continue;
                }
                let origin_port = origin_port.unwrap();

                // Get the y coordinate of the dummy node position
                let dummy_y = graph.node(node_id).position.y;

                // Get origin port's absolute anchor x
                let origin_owner = graph.port(origin_port).owner;
                let origin_node_pos = graph.node(origin_owner).position;
                let port_pos = graph.port(origin_port).position;
                let anchor = graph.port(origin_port).anchor;
                let x = origin_node_pos.x + port_pos.x + anchor.x;
                let bend = Vec2::new(x, dummy_y);

                // Process incoming edges: reroute to original port, add bend point
                let incoming = graph.move_incoming_edges(port_id, origin_port);
                for edge_id in incoming {
                    graph.edge_mut(edge_id).bend_points.push(bend);
                    if same_origin_port {
                        let mut jps = graph.edge(edge_id).properties.get(&JUNCTION_POINTS);
                        jps.push(bend);
                        graph.edge_mut(edge_id).properties.set(&JUNCTION_POINTS, jps);
                    }
                }

                // Process outgoing edges: reroute to original port, add bend point at front
                let outgoing = graph.move_outgoing_edges(port_id, origin_port);
                for edge_id in outgoing {
                    graph.edge_mut(edge_id).bend_points.insert(0, bend);
                    if same_origin_port {
                        let mut jps = graph.edge(edge_id).properties.get(&JUNCTION_POINTS);
                        jps.push(bend);
                        graph.edge_mut(edge_id).properties.set(&JUNCTION_POINTS, jps);
                    }
                }
            }
        }
    }

    // Remove dummy nodes
    for dummy in dummies_to_remove {
        graph.remove_node(dummy);
    }
}
