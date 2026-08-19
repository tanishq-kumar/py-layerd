//! Sort nodes and ports by input model order.

use std::collections::{HashMap, HashSet};

use crate::{
    graph::{
        LGraph,
        edge::EdgeFlags,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{GroupOrderStrategy, OrderingStrategy},
    properties::internal::{
        CM_ENFORCED_GROUP_ORDERS, CM_GROUP_ORDER_STRATEGY, CONSIDER_MODEL_ORDER_PORT_MODEL_ORDER,
        CROSSING_MINIMIZATION_ID, MAX_MODEL_ORDER_NODES, MODEL_ORDER,
    },
};

pub fn sort(graph: &mut LGraph) {
    let num_layers = graph.layers.len();
    for layer_idx in 0..num_layers {
        let prev_layer_idx = if layer_idx == 0 { 0 } else { layer_idx - 1 };
        let prev_layer_nodes: Vec<NodeId> = graph.layers[prev_layer_idx].nodes.clone();
        let ordering = graph.options.ordering_strategy;
        let port_model_order = graph.properties.get(&CONSIDER_MODEL_ORDER_PORT_MODEL_ORDER);

        let mut layer_nodes = graph.layers[layer_idx].nodes.clone();

        insertion_sort_nodes(graph, &mut layer_nodes, &prev_layer_nodes, ordering, true);

        // The processor mutates the layer's node list in place via the
        // insertion sort above, so the per-node `sort_ports` calls below — and
        // any `check_reference_layer_nodes` they make against
        // `graph.layers[*].nodes` — see the freshly-sorted node order.
        graph.layers[layer_idx].nodes = layer_nodes.clone();

        for &node_id in &layer_nodes {
            let pc = graph.node(node_id).port_constraints();
            use crate::options::PortConstraints;
            if pc != PortConstraints::FixedOrder && pc != PortConstraints::FixedPos {
                let target_node_model_order = long_edge_target_node_preprocessing(graph, node_id);
                let mut ports: Vec<PortId> = graph.node(node_id).ports.to_vec();
                sort_ports(
                    graph,
                    &mut ports,
                    &prev_layer_nodes,
                    ordering,
                    &target_node_model_order,
                    port_model_order,
                );
                graph.node_mut(node_id).ports = ports.into();
            }
        }

        insertion_sort_nodes(graph, &mut layer_nodes, &prev_layer_nodes, ordering, false);
        graph.layers[layer_idx].nodes = layer_nodes;
    }
}

fn insertion_sort_nodes(
    graph: &LGraph,
    nodes: &mut [NodeId],
    prev_layer: &[NodeId],
    ordering: OrderingStrategy,
    before_ports: bool,
) {
    let mut bigger_than: HashMap<NodeId, HashSet<NodeId>> = HashMap::new();
    let mut smaller_than: HashMap<NodeId, HashSet<NodeId>> = HashMap::new();
    for &n in nodes.iter() {
        bigger_than.entry(n).or_default();
        smaller_than.entry(n).or_default();
    }

    for i in 1..nodes.len() {
        let temp = nodes[i];
        let mut j = i;
        while j > 0 {
            let cmp = compare_nodes(
                graph,
                nodes[j - 1],
                temp,
                prev_layer,
                ordering,
                before_ports,
                &mut bigger_than,
                &mut smaller_than,
            );
            if cmp > 0 {
                nodes[j] = nodes[j - 1];
                j -= 1;
            } else {
                break;
            }
        }
        nodes[j] = temp;
    }
    bigger_than.clear();
    smaller_than.clear();
}

enum CompareStep {
    Done(i32),
    Defer(NodeId, NodeId),
}

#[allow(clippy::too_many_arguments)]
fn compare_nodes(
    graph: &LGraph,
    mut n1: NodeId,
    mut n2: NodeId,
    prev_layer: &[NodeId],
    ordering: OrderingStrategy,
    before_ports: bool,
    bigger_than: &mut HashMap<NodeId, HashSet<NodeId>>,
    smaller_than: &mut HashMap<NodeId, HashSet<NodeId>>,
) -> i32 {
    let mut deferred_pairs: Vec<(NodeId, NodeId)> = Vec::new();
    let mut seen_pairs: Vec<(NodeId, NodeId)> = Vec::new();

    loop {
        if seen_pairs.contains(&(n1, n2)) {
            return 0;
        }
        seen_pairs.push((n1, n2));

        match compare_nodes_step(
            graph,
            n1,
            n2,
            prev_layer,
            ordering,
            before_ports,
            bigger_than,
            smaller_than,
        ) {
            CompareStep::Done(cmp) => {
                if cmp != 0 {
                    for (left, right) in deferred_pairs.into_iter().chain(std::iter::once((n1, n2)))
                    {
                        if cmp > 0 {
                            update_node_associations(bigger_than, smaller_than, left, right);
                        } else {
                            update_node_associations(bigger_than, smaller_than, right, left);
                        }
                    }
                }
                return cmp;
            }
            CompareStep::Defer(next_n1, next_n2) => {
                deferred_pairs.push((n1, n2));
                n1 = next_n1;
                n2 = next_n2;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn compare_nodes_step(
    graph: &LGraph,
    n1: NodeId,
    n2: NodeId,
    prev_layer: &[NodeId],
    ordering: OrderingStrategy,
    _before_ports: bool,
    bigger_than: &HashMap<NodeId, HashSet<NodeId>>,
    smaller_than: &HashMap<NodeId, HashSet<NodeId>>,
) -> CompareStep {
    if let Some(bt) = bigger_than.get(&n1)
        && bt.contains(&n2)
    {
        return CompareStep::Done(1);
    }
    if let Some(bt) = bigger_than.get(&n2)
        && bt.contains(&n1)
    {
        return CompareStep::Done(-1);
    }
    if let Some(st) = smaller_than.get(&n1)
        && st.contains(&n2)
    {
        return CompareStep::Done(-1);
    }
    if let Some(st) = smaller_than.get(&n2)
        && st.contains(&n1)
    {
        return CompareStep::Done(1);
    }

    let n1_has_mo = graph.node(n1).properties.has(&MODEL_ORDER);
    let n2_has_mo = graph.node(n2).properties.has(&MODEL_ORDER);

    if ordering == OrderingStrategy::PreferEdges || !n1_has_mo || !n2_has_mo {
        let n1_layer = graph.node(n1).layer;
        let n2_layer = graph.node(n2).layer;

        let p1_source = find_source_port_in_prev_layer(graph, n1, n1_layer.get());
        let p2_source = find_source_port_in_prev_layer(graph, n2, n2_layer.get());

        match (p1_source, p2_source) {
            (Some(p1s), Some(p2s)) => {
                let p1_node = graph.port(p1s).owner;
                let p2_node = graph.port(p2s).owner;

                if p1_node == p2_node {
                    for &port_id in &graph.node(p1_node).ports {
                        if port_id == p1s {
                            return CompareStep::Done(-1);
                        } else if port_id == p2s {
                            return CompareStep::Done(1);
                        }
                    }
                }

                for &prev_node in prev_layer {
                    if prev_node == p1_node {
                        return CompareStep::Done(-1);
                    } else if prev_node == p2_node {
                        return CompareStep::Done(1);
                    }
                }
            }
            (Some(_), None) | (None, Some(_)) => {
                // Run the long-edge feedback-dummy helper before falling
                // back to edge-MO comparison.
                match handle_helper_dummy_nodes(graph, n1, n2) {
                    CompareStep::Done(0) => {}
                    step => return step,
                }

                if !n1_has_mo || !n2_has_mo {
                    let n1_edge_mo = model_order_from_edges(graph, n1);
                    let n2_edge_mo = model_order_from_edges(graph, n2);
                    if n1_edge_mo > n2_edge_mo {
                        return CompareStep::Done(1);
                    } else {
                        return CompareStep::Done(-1);
                    }
                }
            }
            (None, None) => {
                // Same helper for the case when neither node has a
                // previous-layer connection.
                match handle_helper_dummy_nodes(graph, n1, n2) {
                    CompareStep::Done(0) => {}
                    step => return step,
                }
            }
        }
    }

    if n1_has_mo && n2_has_mo {
        // Read effective MO via group-model-order resolution so the Enforced
        // branch can scale by `CROSSING_MINIMIZATION_ID *
        // MAX_MODEL_ORDER_NODES`.
        let n1_mo =
            effective_model_order(graph, &graph.node(n1).properties, &graph.node(n2).properties);
        let n2_mo =
            effective_model_order(graph, &graph.node(n2).properties, &graph.node(n1).properties);
        if n1_mo > n2_mo {
            return CompareStep::Done(1);
        } else {
            return CompareStep::Done(-1);
        }
    }

    CompareStep::Done(-1)
}

/// Group-aware model-order resolver.
///
/// Returns `MODEL_ORDER` for the element, optionally multiplied by
/// `CROSSING_MINIMIZATION_ID * MAX_MODEL_ORDER_NODES` when the graph's
/// `CM_GROUP_ORDER_STRATEGY` is `Enforced` and both element and other have
/// CM IDs in `CM_ENFORCED_GROUP_ORDERS`. Returns `-1` when the element has
/// no `MODEL_ORDER`.
fn effective_model_order(
    graph: &LGraph,
    elem: &crate::properties::PropertyMap,
    other: &crate::properties::PropertyMap,
) -> i32 {
    if !elem.has(&MODEL_ORDER) {
        return -1;
    }
    let mo = elem.get(&MODEL_ORDER);
    if graph.properties.get(&CM_GROUP_ORDER_STRATEGY) != GroupOrderStrategy::Enforced {
        return mo;
    }
    let enforced = graph.properties.get(&CM_ENFORCED_GROUP_ORDERS);
    let elem_cm_id = elem.get(&CROSSING_MINIMIZATION_ID);
    let other_cm_id = other.get(&CROSSING_MINIMIZATION_ID);
    if enforced.contains(&elem_cm_id) && enforced.contains(&other_cm_id) {
        let offset = graph.properties.get(&MAX_MODEL_ORDER_NODES);
        return offset * elem_cm_id + mo;
    }
    mo
}

fn find_source_port_in_prev_layer(
    graph: &LGraph,
    node: NodeId,
    node_layer: Option<usize>,
) -> Option<PortId> {
    let node_layer_idx = node_layer?;
    let prev_idx = if node_layer_idx == 0 { 0 } else { node_layer_idx - 1 };
    for &port_id in &graph.node(node).ports {
        let port = graph.port(port_id);
        if let Some(&edge_id) = port.incoming_edges.first() {
            let source_port = graph.edge(edge_id).source;
            let source_node = graph.port(source_port).owner;
            if graph.node(source_node).layer == Some(prev_idx) {
                return Some(source_port);
            }
        }
    }
    None
}

/// Helper-dummy-aware comparison. Returns +/-1 when one of the nodes is a
/// feedback LONG_EDGE dummy and a routing decision can be reached from the
/// dummy's source or target endpoints (or, for LongEdge x LongEdge, from
/// the reference nodes pointed at by the dummies' source ports). Returns 0
/// to fall through to the caller's regular edge-MO / model-order
/// comparison.
fn handle_helper_dummy_nodes(graph: &LGraph, n1: NodeId, n2: NodeId) -> CompareStep {
    let n1_type = graph.node(n1).node_type;
    let n2_type = graph.node(n2).node_type;

    if n1_type == NodeType::LongEdge && n2_type == NodeType::Normal {
        if let Some((src_node, tgt_node, dummy_layer)) = long_edge_dummy_endpoints(graph, n1) {
            // Only feedback dummies — at least one endpoint shares the
            // dummy's own layer.
            let src_layer = graph.node(src_node).layer;
            let tgt_layer = graph.node(tgt_node).layer;
            if src_layer != Some(dummy_layer) && tgt_layer != Some(dummy_layer) {
                return CompareStep::Done(0);
            }
            if src_node == n2 || tgt_node == n2 {
                return CompareStep::Done(1);
            }
            return CompareStep::Defer(src_node, n2);
        }
    } else if n1_type == NodeType::Normal && n2_type == NodeType::LongEdge {
        if let Some((src_node, tgt_node, dummy_layer)) = long_edge_dummy_endpoints(graph, n2) {
            let src_layer = graph.node(src_node).layer;
            let tgt_layer = graph.node(tgt_node).layer;
            if src_layer != Some(dummy_layer) && tgt_layer != Some(dummy_layer) {
                return CompareStep::Done(0);
            }
            if src_node == n1 || tgt_node == n1 {
                return CompareStep::Done(-1);
            }
            return CompareStep::Defer(n1, src_node);
        }
    } else if n1_type == NodeType::LongEdge && n2_type == NodeType::LongEdge {
        return handle_long_edge_pair(graph, n1, n2);
    }
    CompareStep::Done(0)
}

/// LongEdge × LongEdge helper-dummy comparison. Picks each dummy's reference
/// node by walking its first incoming source / first outgoing target and
/// finding the one that lies in the same layer (= feedback endpoint). When
/// both dummies resolve to the same reference node, sort by which dummy's
/// source port comes first in that reference's port list. When references
/// differ, defer to the outer comparator on (`n1_ref`, `n2_ref`).
fn handle_long_edge_pair(graph: &LGraph, n1: NodeId, n2: NodeId) -> CompareStep {
    let Some(n1_layer) = graph.node(n1).layer.get() else {
        return CompareStep::Done(0);
    };
    let Some(n2_layer) = graph.node(n2).layer.get() else {
        return CompareStep::Done(0);
    };

    let Some(n1_src_port) = first_incoming_source_port(graph, n1) else {
        return CompareStep::Done(0);
    };
    let Some(n1_tgt_port) = first_outgoing_target_port(graph, n1) else {
        return CompareStep::Done(0);
    };
    let n1_src_node = graph.port(n1_src_port).owner;
    let n1_tgt_node = graph.port(n1_tgt_port).owner;

    let Some(n2_src_port) = first_incoming_source_port(graph, n2) else {
        return CompareStep::Done(0);
    };
    let Some(n2_tgt_port) = first_outgoing_target_port(graph, n2) else {
        return CompareStep::Done(0);
    };
    let n2_src_node = graph.port(n2_src_port).owner;
    let n2_tgt_node = graph.port(n2_tgt_port).owner;

    let mut n1_ref = n1;
    if graph.node(n1_src_node).layer == Some(n1_layer) {
        n1_ref = n1_src_node;
    } else if graph.node(n1_tgt_node).layer == Some(n1_layer) {
        n1_ref = n1_tgt_node;
    }
    let mut n2_ref = n2;
    if graph.node(n2_src_node).layer == Some(n2_layer) {
        n2_ref = n2_src_node;
    } else if graph.node(n2_tgt_node).layer == Some(n2_layer) {
        n2_ref = n2_tgt_node;
    }

    if n1_ref == n2_ref {
        // `!beforePorts` branch: iterate ports of the reference node; the
        // dummy whose source port appears first sorts first. For the
        // `beforePorts` branch — when the ports have not yet been sorted —
        // we keep the same port-order shortcut here as a pragmatic
        // approximation; the full port-comparator branch is not implemented.
        for &port_id in &graph.node(n1_ref).ports {
            if port_id == n1_src_port {
                return CompareStep::Done(-1);
            }
            if port_id == n2_src_port {
                return CompareStep::Done(1);
            }
        }
    }

    CompareStep::Defer(n1_ref, n2_ref)
}

fn first_incoming_source_port(graph: &LGraph, node: NodeId) -> Option<PortId> {
    for &pid in &graph.node(node).ports {
        if let Some(&eid) = graph.port(pid).incoming_edges.first() {
            return Some(graph.edge(eid).source);
        }
    }
    None
}

fn first_outgoing_target_port(graph: &LGraph, node: NodeId) -> Option<PortId> {
    for &pid in &graph.node(node).ports {
        if let Some(&eid) = graph.port(pid).outgoing_edges.first() {
            return Some(graph.edge(eid).target);
        }
    }
    None
}

/// Return the (source node, target node, dummy's own layer) tuple for a
/// LONG_EDGE dummy that has both an incoming edge (source side) and an
/// outgoing edge (target side). Returns `None` when the dummy is
/// mid-chain in a way that can't be resolved — callers fall through.
fn long_edge_dummy_endpoints(graph: &LGraph, dummy: NodeId) -> Option<(NodeId, NodeId, usize)> {
    let dummy_layer = graph.node(dummy).layer.get()?;
    let mut src_node: Option<NodeId> = None;
    let mut tgt_node: Option<NodeId> = None;
    for &pid in &graph.node(dummy).ports {
        let port = graph.port(pid);
        if src_node.is_none()
            && let Some(&eid) = port.incoming_edges.first()
        {
            let src_port = graph.edge(eid).source;
            src_node = Some(graph.port(src_port).owner);
        }
        if tgt_node.is_none()
            && let Some(&eid) = port.outgoing_edges.first()
        {
            let tgt_port = graph.edge(eid).target;
            tgt_node = Some(graph.port(tgt_port).owner);
        }
        if src_node.is_some() && tgt_node.is_some() {
            break;
        }
    }
    Some((src_node?, tgt_node?, dummy_layer))
}

fn model_order_from_edges(graph: &LGraph, node: NodeId) -> i32 {
    for &port_id in &graph.node(node).ports {
        let port = graph.port(port_id);
        if let Some(&edge_id) = port.incoming_edges.first() {
            return graph.edge(edge_id).properties.get(&MODEL_ORDER);
        }
    }
    // When the node has no incoming edge carrying a model order, return the
    // `LongEdgeOrderingStrategy.returnValue()` sentinel so dummy nodes sort
    // over / under / equal to unconnected normal nodes as the user requested.
    // A previous hardcoded 0 only matched the `Equal` case.
    graph.options.consider_model_order_long_edge_strategy.return_value()
}

fn update_node_associations(
    bigger_than: &mut HashMap<NodeId, HashSet<NodeId>>,
    smaller_than: &mut HashMap<NodeId, HashSet<NodeId>>,
    bigger: NodeId,
    smaller: NodeId,
) {
    let smaller_bt: HashSet<NodeId> = bigger_than.get(&smaller).cloned().unwrap_or_default();
    let bigger_st: HashSet<NodeId> = smaller_than.get(&bigger).cloned().unwrap_or_default();

    bigger_than.entry(bigger).or_default().insert(smaller);
    smaller_than.entry(smaller).or_default().insert(bigger);

    for &very_small in &smaller_bt {
        bigger_than.entry(bigger).or_default().insert(very_small);
        let s = smaller_than.entry(very_small).or_default();
        s.insert(bigger);
        for &very_big in &bigger_st {
            s.insert(very_big);
        }
    }

    for &very_big in &bigger_st {
        smaller_than.entry(smaller).or_default().insert(very_big);
        let b = bigger_than.entry(very_big).or_default();
        b.insert(smaller);
        for &very_small in &smaller_bt {
            b.insert(very_small);
        }
    }
}

fn long_edge_target_node_preprocessing(graph: &LGraph, node: NodeId) -> HashMap<NodeId, i32> {
    let mut target_node_model_order: HashMap<NodeId, i32> = HashMap::new();
    for &port_id in &graph.node(node).ports {
        let port = graph.port(port_id);
        if port.outgoing_edges.is_empty() {
            continue;
        }
        if let Some(target_node) = get_target_node(graph, port_id) {
            let prev = *target_node_model_order.get(&target_node).unwrap_or(&i32::MAX);
            let edge_id = port.outgoing_edges[0];
            if !graph.edge(edge_id).flags.contains(EdgeFlags::REVERSED) {
                let edge_mo = graph.edge(edge_id).properties.get(&MODEL_ORDER);
                target_node_model_order.insert(target_node, prev.min(edge_mo));
            }
        }
    }
    target_node_model_order
}

fn get_target_node(graph: &LGraph, port: PortId) -> Option<NodeId> {
    let port_data = graph.port(port);
    let edge_id = *port_data.outgoing_edges.first()?;
    let mut target_port = graph.edge(edge_id).target;
    let mut node = graph.port(target_port).owner;

    loop {
        if let Some(long_edge_target_port) = graph.node(node).long_edge_target {
            return Some(graph.port(long_edge_target_port).owner);
        }
        if graph.node(node).node_type == NodeType::Normal {
            return Some(node);
        }
        let next_edge = graph.outgoing_edges(node).next();
        {
            let eid = next_edge?;
            target_port = graph.edge(eid).target;
            node = graph.port(target_port).owner;
        }
    }
}

fn sort_ports(
    graph: &LGraph,
    ports: &mut [PortId],
    prev_layer: &[NodeId],
    ordering: OrderingStrategy,
    target_node_model_order: &HashMap<NodeId, i32>,
    port_model_order: bool,
) {
    let mut bigger_than: HashMap<PortId, HashSet<PortId>> = HashMap::new();
    let mut smaller_than: HashMap<PortId, HashSet<PortId>> = HashMap::new();
    for &p in ports.iter() {
        bigger_than.entry(p).or_default();
        smaller_than.entry(p).or_default();
    }

    for i in 1..ports.len() {
        let temp = ports[i];
        let mut j = i;
        while j > 0 {
            let cmp = compare_ports(
                graph,
                ports[j - 1],
                temp,
                prev_layer,
                ordering,
                target_node_model_order,
                port_model_order,
                &bigger_than,
                &smaller_than,
            );
            if cmp > 0 {
                update_port_associations(&mut bigger_than, &mut smaller_than, ports[j - 1], temp);
                ports[j] = ports[j - 1];
                j -= 1;
            } else {
                if cmp < 0 {
                    update_port_associations(
                        &mut bigger_than,
                        &mut smaller_than,
                        temp,
                        ports[j - 1],
                    );
                }
                break;
            }
        }
        ports[j] = temp;
    }
}

#[allow(clippy::too_many_arguments)]
fn compare_ports(
    graph: &LGraph,
    p1: PortId,
    p2: PortId,
    prev_layer: &[NodeId],
    ordering: OrderingStrategy,
    target_node_model_order: &HashMap<NodeId, i32>,
    port_model_order_enabled: bool,
    bigger_than: &HashMap<PortId, HashSet<PortId>>,
    smaller_than: &HashMap<PortId, HashSet<PortId>>,
) -> i32 {
    if let Some(bt) = bigger_than.get(&p1)
        && bt.contains(&p2)
    {
        return 1;
    }
    if let Some(bt) = bigger_than.get(&p2)
        && bt.contains(&p1)
    {
        return -1;
    }
    if let Some(st) = smaller_than.get(&p1)
        && st.contains(&p2)
    {
        return -1;
    }
    if let Some(st) = smaller_than.get(&p2)
        && st.contains(&p1)
    {
        return 1;
    }

    let s1 = graph.port(p1).side;
    let s2 = graph.port(p2).side;
    if s1 != s2 {
        return side_ordinal(s1).cmp(&side_ordinal(s2)) as i32;
    }

    let mut reverse_order: i32 = 1;

    let p1_incoming = !graph.port(p1).incoming_edges.is_empty();
    let p2_incoming = !graph.port(p2).incoming_edges.is_empty();
    let p1_outgoing = !graph.port(p1).outgoing_edges.is_empty();
    let p2_outgoing = !graph.port(p2).outgoing_edges.is_empty();

    if p1_incoming && p2_incoming {
        if matches!(s1, PortSide::West | PortSide::North | PortSide::South) {
            reverse_order = -reverse_order;
        }

        let p1_src = graph.edge(graph.port(p1).incoming_edges[0]).source;
        let p2_src = graph.edge(graph.port(p2).incoming_edges[0]).source;
        let p1_node = graph.port(p1_src).owner;
        let p2_node = graph.port(p2_src).owner;

        if p1_node == p2_node {
            for &port_id in &graph.node(p1_node).ports {
                if port_id == p1_src {
                    return -reverse_order;
                } else if port_id == p2_src {
                    return reverse_order;
                }
            }
        }

        let p1_node_layer = graph.node(p1_node).layer;
        let p2_node_layer = graph.node(p2_node).layer;
        let owner_layer = graph.node(graph.port(p1).owner).layer;
        if graph.node(p1_node).node_type == NodeType::LongEdge
            && graph.node(p2_node).node_type == NodeType::LongEdge
            && p1_node_layer == p2_node_layer
            && p1_node_layer == owner_layer
        {
            let in_layer = check_reference_layer_nodes(
                &graph.layers[p1_node_layer.unwrap()].nodes,
                p1_node,
                p2_node,
            );
            if in_layer != 0 {
                let mut rv = reverse_order;
                if s1 == PortSide::East {
                    rv = -rv;
                }
                return if in_layer > 0 { rv } else { -rv };
            }
        }

        let in_prev = check_reference_layer_nodes(prev_layer, p1_node, p2_node);
        if in_prev != 0 {
            return if in_prev > 0 { reverse_order } else { -reverse_order };
        }

        if port_model_order_enabled {
            let result = check_port_model_order(graph, p1, p2);
            if result != 0 {
                return if result > 0 { reverse_order } else { -reverse_order };
            }
        }
    }

    if p1_outgoing && p2_outgoing {
        if matches!(s1, PortSide::West | PortSide::South) {
            reverse_order = -reverse_order;
        }

        let p1_target = get_target_node(graph, p1);
        let p2_target = get_target_node(graph, p2);

        if ordering == OrderingStrategy::PreferNodes
            && let (Some(t1), Some(t2)) = (p1_target, p2_target)
        {
            let t1_has = graph.node(t1).properties.has(&MODEL_ORDER);
            let t2_has = graph.node(t2).properties.has(&MODEL_ORDER);
            if t1_has && t2_has {
                let p1_mo = effective_model_order(
                    graph,
                    &graph.node(t1).properties,
                    &graph.node(t2).properties,
                );
                let p2_mo = effective_model_order(
                    graph,
                    &graph.node(t2).properties,
                    &graph.node(t1).properties,
                );
                return if p1_mo > p2_mo { reverse_order } else { -reverse_order };
            }
        }

        if port_model_order_enabled {
            let result = check_port_model_order(graph, p1, p2);
            if result != 0 {
                return if result > 0 { reverse_order } else { -reverse_order };
            }
        }

        let mut p1_order: i32 = 0;
        let mut p2_order: i32 = 0;
        let e1 = graph.port(p1).outgoing_edges[0];
        let e2 = graph.port(p2).outgoing_edges[0];
        if graph.edge(e1).properties.has(&MODEL_ORDER) {
            p1_order = effective_model_order(
                graph,
                &graph.edge(e1).properties,
                &graph.edge(e2).properties,
            );
        }
        if graph.edge(e2).properties.has(&MODEL_ORDER) {
            p2_order = effective_model_order(
                graph,
                &graph.edge(e2).properties,
                &graph.edge(e1).properties,
            );
        }

        if let (Some(t1), Some(t2)) = (p1_target, p2_target)
            && t1 == t2
        {
            return if p1_order > p2_order { reverse_order } else { -reverse_order };
        }

        if let Some(t1) = p1_target
            && let Some(&mo) = target_node_model_order.get(&t1)
        {
            p1_order = mo;
        }
        if let Some(t2) = p2_target
            && let Some(&mo) = target_node_model_order.get(&t2)
        {
            p2_order = mo;
        }

        return if p1_order > p2_order { reverse_order } else { -reverse_order };
    }

    if p1_incoming && p2_outgoing {
        return 1;
    } else if p1_outgoing && p2_incoming {
        return -1;
    }

    if graph.port(p1).properties.has(&MODEL_ORDER) && graph.port(p2).properties.has(&MODEL_ORDER) {
        let mut rv = 1;
        if matches!(s1, PortSide::West | PortSide::South) {
            rv = -rv;
        }
        let p1_mo =
            effective_model_order(graph, &graph.port(p1).properties, &graph.port(p2).properties);
        let p2_mo =
            effective_model_order(graph, &graph.port(p2).properties, &graph.port(p1).properties);
        return if p1_mo > p2_mo { rv } else { -rv };
    }

    -1
}

fn check_reference_layer_nodes(layer: &[NodeId], n1: NodeId, n2: NodeId) -> i32 {
    for &node in layer {
        if node == n1 {
            return -1;
        } else if node == n2 {
            return 1;
        }
    }
    0
}

fn check_port_model_order(graph: &LGraph, p1: PortId, p2: PortId) -> i32 {
    if graph.port(p1).properties.has(&MODEL_ORDER) && graph.port(p2).properties.has(&MODEL_ORDER) {
        let p1_mo =
            effective_model_order(graph, &graph.port(p1).properties, &graph.port(p2).properties);
        let p2_mo =
            effective_model_order(graph, &graph.port(p2).properties, &graph.port(p1).properties);
        return p1_mo.cmp(&p2_mo) as i32;
    }
    0
}

fn side_ordinal(side: PortSide) -> u8 {
    match side {
        PortSide::Undefined => 0,
        PortSide::North => 1,
        PortSide::East => 2,
        PortSide::South => 3,
        PortSide::West => 4,
    }
}

fn update_port_associations(
    bigger_than: &mut HashMap<PortId, HashSet<PortId>>,
    smaller_than: &mut HashMap<PortId, HashSet<PortId>>,
    bigger: PortId,
    smaller: PortId,
) {
    let smaller_bt: HashSet<PortId> = bigger_than.get(&smaller).cloned().unwrap_or_default();
    let bigger_st: HashSet<PortId> = smaller_than.get(&bigger).cloned().unwrap_or_default();

    bigger_than.entry(bigger).or_default().insert(smaller);
    smaller_than.entry(smaller).or_default().insert(bigger);

    for &very_small in &smaller_bt {
        bigger_than.entry(bigger).or_default().insert(very_small);
        let s = smaller_than.entry(very_small).or_default();
        s.insert(bigger);
        for &very_big in &bigger_st {
            s.insert(very_big);
        }
    }

    for &very_big in &bigger_st {
        smaller_than.entry(smaller).or_default().insert(very_big);
        let b = bigger_than.entry(very_big).or_default();
        b.insert(smaller);
        for &very_small in &smaller_bt {
            b.insert(very_small);
        }
    }
}
