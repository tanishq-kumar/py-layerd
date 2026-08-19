//! Reference: Gansner, Koutsofios, North, Vo, "A technique for drawing
//! directed graphs", Software Engineering 19(3), 1993.
//!
//! # Scope
//!
//! Auxiliary-graph construction:
//! * Non-flexible nodes map to a single `NNode`.
//! * Flexible nodes (`isFlexibleNode`) get two `NNode`s (top/bottom corners)
//!   plus one `NNode` per east/west port, connected by weight-0 edges that
//!   enforce port spacing and node minimum height. The node-size edge
//!   carries a high weight (`NODE_SIZE_WEIGHT_STATIC`) unless the
//!   flexibility level permits resizing.
//! * Edges connect two new `NEdge`s (`left` / `right`) from a dummy node to
//!   each endpoint's representative NNode (either the node head or, for
//!   flexible nodes, the port NNode).
//! * `NODE_PLACEMENT_FAVOR_STRAIGHT_EDGES` drives the `preferStraightEdges`
//!   pre-pass (identify paths, reweight) and the `postProcessTwoPaths`
//!   post-pass (move center nodes of two-hop paths to straighten one of
//!   their edges).
//! * `NodeFlexibility::NodeSizeWhereSpacePermits` triggers a second
//!   network-simplex solve after inserting global source / sink anchor
//!   edges and relaxing the node-size edge weights.

use hashbrown::HashMap;

use crate::{
    algorithms::network_simplex::{NGraph, Solver},
    graph::{
        LGraph,
        index::{EdgeId, LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{NodeFlexibility, NodeLabelPlacement, PortConstraints},
    properties::internal::{
        NODE_FLEXIBILITY, NODE_LABEL_PLACEMENT, PRIORITY_STRAIGHTNESS, SPACING_PORT_PORT_OVERRIDE,
        SPACING_PORTS_SURROUNDING,
    },
};

/// Base weight coefficient for regular edges.
const EDGE_WEIGHT_BASE: f64 = 4.0;
/// Smaller weight used for north/south port edges.
const SMALL_EDGE_WEIGHT: f64 = 0.1;
/// Weight applied to the top-to-bottom edge of a node when the node may not
/// be resized.
const NODE_SIZE_WEIGHT_STATIC: f64 = 10_000.0;
/// Weight applied when the node is allowed to resize to whatever the simplex
/// solver finds convenient.
const NODE_SIZE_WEIGHT_FLEXIBLE: f64 = 1.0;
/// Multiplier applied to path edge weights in `prefer_straight_edges` so long
/// straight paths outrank short ones. Values smaller than one would invert
/// that precedence.
const LONG_EDGE_VS_PATH_FACTOR: f64 = 2.0;

// NodeRep / EdgeRep auxiliary types.

struct NodeRep {
    /// NNode that represents the top of the node (lower y).
    head: usize,
    /// NNode that represents the bottom of the node (larger y). Equals
    /// `head` for non-flexible nodes.
    tail: usize,
    /// Whether the node is flexible (has 2+ NNodes for corners / ports).
    is_flexible: bool,
    /// NEdge index of the top-to-bottom "size" edge. `None` for non-flexible
    /// nodes (they have no such edge).
    size_edge: Option<usize>,
}

#[derive(Clone, Copy)]
struct EdgeRep {
    /// NEdge from the dummy node to the source port's NNode.
    left: usize,
    /// NEdge from the dummy node to the target port's NNode.
    right: usize,
}

impl EdgeRep {
    fn is_straight(&self, ng: &NGraph) -> bool {
        self.not_straight_by(ng) == 0
    }

    /// Returns `(left.target.layer - left.delta) - (right.target.layer - right.delta)`.
    fn not_straight_by(&self, ng: &NGraph) -> i32 {
        let le = &ng.edges[self.left];
        let re = &ng.edges[self.right];
        (ng.nodes[le.target].layer - le.delta) - (ng.nodes[re.target].layer - re.delta)
    }
}

// Main entry point

/// Place nodes by encoding the placement problem as a network simplex
/// instance and solving it.
pub fn place_nodes(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }
    let favor_straight = graph.options.node_placement_favor_straight_edges;

    let mut placer = Placer::new();
    placer.prepare(graph);
    placer.build_initial_auxiliary_graph(graph);
    placer.insert_north_south_auxiliary_edges(graph);
    placer.insert_in_layer_edge_auxiliary_edges(graph);

    if favor_straight {
        placer.prefer_straight_edges(graph);
    }

    placer.make_connected();

    let iter_limit = (graph.options.thoroughness as usize).max(1) * placer.ng.nodes.len();
    placer.run_network_simplex(iter_limit);

    if !placer.flexible_where_space_permits_edges.is_empty() {
        placer.insert_flexible_where_space_auxiliary_edges(graph);
        for &eidx in &placer.flexible_where_space_permits_edges {
            placer.ng.edges[eidx].weight = NODE_SIZE_WEIGHT_FLEXIBLE;
        }
        placer.run_network_simplex(iter_limit);
    }

    if favor_straight {
        placer.post_process_two_paths(graph);
    }

    placer.apply_positions(graph);
}

struct Placer {
    ng: NGraph,
    /// Per-node: head/tail NNode indices + flexibility.
    node_reps: HashMap<NodeId, NodeRep>,
    /// Per-edge: left/right NEdge indices.
    edge_reps: HashMap<EdgeId, EdgeRep>,
    /// Per-port NNode index for flexible-node ports. Non-flexible nodes
    /// omit entries here.
    port_map: HashMap<PortId, usize>,
    /// NNode indices whose weight should be relaxed once
    /// `insert_flexible_where_space_auxiliary_edges` has run.
    flexible_where_space_permits_edges: Vec<usize>,
    /// `preferStraightEdges` book-keeping.
    node_state: HashMap<NodeId, NodeState>,
    edge_crossing: HashMap<EdgeId, bool>,
    two_paths: Vec<(EdgeId, EdgeId)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeState {
    Unvisited,
    Visited,
    Junction,
}

impl Placer {
    fn new() -> Self {
        Self {
            ng: NGraph::new(),
            node_reps: HashMap::new(),
            edge_reps: HashMap::new(),
            port_map: HashMap::new(),
            flexible_where_space_permits_edges: Vec::new(),
            node_state: HashMap::new(),
            edge_crossing: HashMap::new(),
            two_paths: Vec::new(),
        }
    }

    fn prepare(&mut self, graph: &mut LGraph) {
        // "integerify" port positions: for flexible nodes the port anchor must
        // be rounded too because we hook the auxiliary edge directly to the
        // port NNode rather than to the node corner.
        for layer_idx in 0..graph.layers.len() {
            let nids: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
            for nid in nids {
                let anchor_must_be_integer = is_flexible_node(graph, nid);
                let port_ids: Vec<_> = graph.node(nid).ports.iter().copied().collect();
                for pid in port_ids {
                    if anchor_must_be_integer {
                        let port = graph.port_mut(pid);
                        let y = port.anchor.y;
                        if y != y.floor() {
                            let rounded = y.round();
                            port.anchor.y -= y - rounded;
                        }
                    }
                    let port = graph.port_mut(pid);
                    let y = port.position.y + port.anchor.y;
                    if y != y.floor() {
                        let rounded = y.round();
                        port.position.y -= y - rounded;
                    }
                }
            }
        }
    }

    fn build_initial_auxiliary_graph(&mut self, graph: &LGraph) {
        for layer_idx in 0..graph.layers.len() {
            self.transform_layer(graph, layer_idx);
        }
        self.transform_edges(graph);
    }

    fn transform_layer(&mut self, graph: &LGraph, layer_idx: usize) {
        let nodes = graph.layers[layer_idx].nodes.clone();
        let mut last: Option<NodeId> = None;
        for nid in nodes {
            let rep = if is_flexible_node(graph, nid) {
                self.transform_fixed_order_node(graph, nid)
            } else {
                self.transform_fixed_pos_node(nid)
            };

            if let Some(prev) = last {
                let prev_node = graph.node(prev);
                let cur_node = graph.node(nid);
                let mut spacing = prev_node.margin.bottom
                    + vertical_spacing(graph, prev, nid)
                    + cur_node.margin.top;
                if !self.node_reps[&prev].is_flexible {
                    // for non-flexible nodes the minimum separation must account for their height
                    spacing += prev_node.size.y;
                }
                let prev_tail = self.node_reps[&prev].tail;
                self.ng.add_edge(prev_tail, rep.head, 0.0, spacing.ceil() as i32);
            }

            self.node_reps.insert(nid, rep);
            last = Some(nid);
        }
    }

    fn transform_fixed_pos_node(&mut self, _nid: NodeId) -> NodeRep {
        let single = self.ng.add_node(0);
        NodeRep { head: single, tail: single, is_flexible: false, size_edge: None }
    }

    fn transform_fixed_order_node(&mut self, graph: &LGraph, nid: NodeId) -> NodeRep {
        let top = self.ng.add_node(0);
        let bottom = self.ng.add_node(0);

        let min_height = graph.node(nid).size.y;
        let nf = get_node_flexibility(graph, nid);
        let size_weight = if nf.is_flexible_size() {
            NODE_SIZE_WEIGHT_FLEXIBLE
        } else {
            NODE_SIZE_WEIGHT_STATIC
        };
        let size_edge = self.ng.add_edge(top, bottom, size_weight, min_height.ceil() as i32);
        if matches!(nf, NodeFlexibility::NodeSizeWhereSpacePermits) {
            self.flexible_where_space_permits_edges.push(size_edge);
        }

        let mut corners =
            NodeRep { head: top, tail: bottom, is_flexible: true, size_edge: Some(size_edge) };

        // West ports: reversed so the iteration walks top-to-bottom rather
        // than the source list's bottom-to-top order.
        let west_ports: Vec<PortId> = {
            let mut v: Vec<PortId> = graph
                .node(nid)
                .ports
                .iter()
                .copied()
                .filter(|&p| graph.port(p).side == PortSide::West)
                .collect();
            v.reverse();
            v
        };
        self.transform_ports(graph, nid, &west_ports, &mut corners);

        let east_ports: Vec<PortId> = graph
            .node(nid)
            .ports
            .iter()
            .copied()
            .filter(|&p| graph.port(p).side == PortSide::East)
            .collect();
        self.transform_ports(graph, nid, &east_ports, &mut corners);

        corners
    }

    fn transform_ports(
        &mut self,
        graph: &LGraph,
        nid: NodeId,
        ports: &[PortId],
        corners: &mut NodeRep,
    ) {
        if ports.is_empty() {
            return;
        }
        // Fall back to graph-level `spacing.port_port` when the node does not
        // carry its own override.
        let port_spacing = graph
            .node(nid)
            .properties
            .get(&SPACING_PORT_PORT_OVERRIDE)
            .unwrap_or(graph.options.spacing.port_port);
        let ports_surrounding = ports_surrounding_for_node(graph, nid);
        let surrounding_top = ports_surrounding.top;
        let surrounding_bottom = ports_surrounding.bottom;

        let mut last_nnode = corners.head;
        let mut last_port: Option<PortId> = None;
        for &port in ports {
            let spacing = if let Some(prev) = last_port {
                port_spacing + graph.port(prev).size.y
            } else {
                surrounding_top
            };

            let port_nn = self.ng.add_node(0);
            self.port_map.insert(port, port_nn);

            self.ng.add_edge(last_nnode, port_nn, 0.0, spacing.ceil() as i32);

            last_port = Some(port);
            last_nnode = port_nn;
        }
        if let Some(last) = last_port {
            let tail_delta = surrounding_bottom + graph.port(last).size.y;
            self.ng.add_edge(last_nnode, corners.tail, 0.0, tail_delta.ceil() as i32);
        }
        // Silence unused warning on nid until we wire node margins into the
        // port-surrounding formula.
        let _ = nid;
    }

    fn transform_edges(&mut self, graph: &LGraph) {
        for layer_idx in 0..graph.layers.len() {
            let nodes = graph.layers[layer_idx].nodes.clone();
            for nid in nodes {
                for eid in graph.outgoing_edges(nid) {
                    if !is_handled_edge(graph, eid) {
                        continue;
                    }
                    self.transform_edge(graph, eid);
                }
            }
        }
    }

    fn transform_edge(&mut self, graph: &LGraph, eid: EdgeId) {
        let edge = graph.edge(eid);
        let src_port_id = edge.source;
        let tgt_port_id = edge.target;
        let src_port = graph.port(src_port_id);
        let tgt_port = graph.port(tgt_port_id);
        let src_nid = src_port.owner;
        let tgt_nid = tgt_port.owner;

        let src_rep = &self.node_reps[&src_nid];
        let tgt_rep = &self.node_reps[&tgt_nid];

        // For flexible nodes the auxiliary edge attaches to the port NNode
        // itself; only the port anchor counts as offset. For non-flexible
        // nodes the edge attaches to the node head; position + anchor both
        // count.
        let src_offset = if src_rep.is_flexible {
            src_port.anchor.y
        } else {
            src_port.position.y + src_port.anchor.y
        };
        let tgt_offset = if tgt_rep.is_flexible {
            tgt_port.anchor.y
        } else {
            tgt_port.position.y + tgt_port.anchor.y
        };

        let tgt_delta = (src_offset - tgt_offset).max(0.0) as i32;
        let src_delta = (tgt_offset - src_offset).max(0.0) as i32;

        let weight = edge_weight(graph, eid);

        let dummy = self.ng.add_node(0);

        let src_nnode =
            if src_rep.is_flexible { self.port_map[&src_port_id] } else { src_rep.head };
        let tgt_nnode =
            if tgt_rep.is_flexible { self.port_map[&tgt_port_id] } else { tgt_rep.head };

        let left = self.ng.add_edge(dummy, src_nnode, weight, src_delta);
        let right = self.ng.add_edge(dummy, tgt_nnode, weight, tgt_delta);
        self.edge_reps.insert(eid, EdgeRep { left, right });
    }

    fn insert_in_layer_edge_auxiliary_edges(&mut self, graph: &LGraph) {
        for layer_idx in 0..graph.layers.len() {
            let nodes = graph.layers[layer_idx].nodes.clone();
            for nid in nodes {
                if graph.node(nid).node_type != NodeType::Normal {
                    continue;
                }
                for eid in graph.incoming_edges(nid).chain(graph.outgoing_edges(nid)) {
                    if !is_in_layer_edge(graph, eid) {
                        continue;
                    }
                    let e = graph.edge(eid);
                    let src_nid = graph.port(e.source).owner;
                    let tgt_nid = graph.port(e.target).owner;
                    let src_is_dummy = graph.node(src_nid).node_type != NodeType::Normal;
                    let the_port = if src_is_dummy { e.target } else { e.source };
                    let the_port_owner = graph.port(the_port).owner;
                    let dummy_node = if the_port_owner == src_nid { tgt_nid } else { src_nid };

                    let (port_rep, dummy_rep) = match (
                        self.node_reps.get(&the_port_owner),
                        self.node_reps.get(&dummy_node),
                    ) {
                        (Some(a), Some(b)) => (a.head, b.head),
                        _ => continue,
                    };

                    let the_port_idx = layer_position(graph, layer_idx, the_port_owner);
                    let dummy_idx = layer_position(graph, layer_idx, dummy_node);
                    let (src, tgt) = if the_port_idx < dummy_idx {
                        (port_rep, dummy_rep)
                    } else {
                        (dummy_rep, port_rep)
                    };
                    self.ng.add_edge(src, tgt, EDGE_WEIGHT_BASE, 0);
                }
            }
        }
    }

    fn insert_north_south_auxiliary_edges(&mut self, graph: &LGraph) {
        for layer in &graph.layers {
            for &nid in &layer.nodes {
                let ports: Vec<_> = graph.node(nid).ports.iter().copied().collect();
                for port_id in ports {
                    let side = graph.port(port_id).side;
                    let Some(dummy_nid) = graph.port(port_id).port_dummy else {
                        continue;
                    };
                    let (Some(own_rep), Some(dummy_rep)) =
                        (self.node_reps.get(&nid), self.node_reps.get(&dummy_nid))
                    else {
                        continue;
                    };
                    match side {
                        PortSide::South => {
                            self.ng.add_edge(own_rep.tail, dummy_rep.head, SMALL_EDGE_WEIGHT, 0);
                        }
                        PortSide::North => {
                            self.ng.add_edge(dummy_rep.tail, own_rep.head, SMALL_EDGE_WEIGHT, 0);
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    fn make_connected(&mut self) {
        let n = self.ng.nodes.len();
        if n == 0 {
            return;
        }
        let components = find_components(&self.ng);
        if components.len() <= 1 {
            return;
        }
        let mut last_representative: Option<usize> = None;
        for comp in &components {
            if comp.is_empty() {
                continue;
            }
            let rep = comp[0];
            if let Some(prev) = last_representative {
                self.ng.add_edge(prev, rep, 0.0, 0);
            }
            last_representative = Some(rep);
        }
    }

    // Flexible-where-space-permits second-round.

    fn insert_flexible_where_space_auxiliary_edges(&mut self, graph: &LGraph) {
        // Snapshot current layering extents.
        let (min_layer, max_layer) = self
            .ng
            .nodes
            .iter()
            .map(|n| n.layer)
            .fold((i32::MAX, i32::MIN), |(lo, hi), l| (lo.min(l), hi.max(l)));
        let used_layers = (max_layer - min_layer).max(0);

        let global_source = self.ng.add_node(0);
        let global_sink = self.ng.add_node(0);

        self.ng
            .add_edge(global_source, global_sink, NODE_SIZE_WEIGHT_STATIC * 2.0, used_layers);

        // Pin only NORMAL multi-port nodes to their current layering. Leaves
        // (<=1 port) and dummy nodes (LongEdge / NorthSouthPort / ...) stay
        // unanchored so the second simplex pass can grow flexible nodes into
        // whatever slack the initial solve produced.
        let reps: Vec<(usize, usize)> = self
            .node_reps
            .iter()
            .filter(|(nid, _)| graph.node(**nid).node_type == NodeType::Normal)
            .filter(|(nid, _)| graph.node(**nid).ports.len() > 1)
            .map(|(_, rep)| (rep.head, rep.tail))
            .collect();
        for (head, tail) in reps {
            let tail_layer = self.ng.nodes[tail].layer;
            let head_layer = self.ng.nodes[head].layer;
            self.ng.add_edge(global_source, tail, 0.0, tail_layer - min_layer);
            self.ng.add_edge(head, global_sink, 0.0, used_layers - head_layer);
        }
    }

    // Prefer-straight-edges pre-pass.

    fn prefer_straight_edges(&mut self, graph: &LGraph) {
        self.node_state.clear();
        self.edge_crossing.clear();
        self.two_paths.clear();

        for layer in &graph.layers {
            for &nid in &layer.nodes {
                let state = compute_node_state(graph, nid);
                self.node_state.insert(nid, state);
            }
        }

        self.mark_edge_crossings(graph);

        let paths = self.identify_paths(graph);
        for path in paths {
            if path.len() <= 1 {
                continue;
            }
            if path.len() == 2 {
                let ordered = order_two_path(graph, &path);
                if !is_two_path_center_node_flexible(graph, &ordered) {
                    self.two_paths.push((ordered[0], ordered[1]));
                }
                continue;
            }
            if path_contains_long_edge_dummy(graph, &path)
                || path_contains_flexible_size_permits(graph, &path)
            {
                continue;
            }
            // Reweight path edges like long edges.
            let last_idx = path.len() - 1;
            for (idx, eid) in path.iter().copied().enumerate() {
                let Some(rep) = self.edge_reps.get(&eid).copied() else {
                    continue;
                };
                let weight = if idx == 0 || idx == last_idx {
                    weight_for_types(NodeType::Normal, NodeType::LongEdge)
                } else {
                    weight_for_types(NodeType::LongEdge, NodeType::LongEdge)
                } * LONG_EDGE_VS_PATH_FACTOR;
                let old_left = self.ng.edges[rep.left].weight;
                let old_right = self.ng.edges[rep.right].weight;
                self.ng.edges[rep.left].weight = old_left.max(old_left + (weight - old_left));
                self.ng.edges[rep.right].weight = old_right.max(old_right + (weight - old_right));
            }
        }
    }

    fn mark_edge_crossings(&mut self, graph: &LGraph) {
        let layer_count = graph.layers.len();
        for i in 0..layer_count.saturating_sub(1) {
            self.mark_crossing_edges(graph, i, i + 1);
        }
    }

    fn mark_crossing_edges(&mut self, graph: &LGraph, left_idx: usize, right_idx: usize) {
        // Collect all edges from `left` layer to `right` layer in the order
        // the left-side east ports produce them.
        let mut open_edges: Vec<EdgeId> = Vec::new();
        for &nid in &graph.layers[left_idx].nodes {
            for &port_id in &graph.node(nid).ports {
                if graph.port(port_id).side != PortSide::East {
                    continue;
                }
                for &eid in &graph.port(port_id).outgoing_edges {
                    if !is_handled_edge(graph, eid) {
                        continue;
                    }
                    let tgt = graph.port(graph.edge(eid).target).owner;
                    if graph.node(tgt).layer != Some(right_idx) {
                        continue;
                    }
                    open_edges.push(eid);
                }
            }
        }

        // Close edges in right-layer reverse order, walking west ports top to bottom.
        let right_nodes_rev: Vec<NodeId> =
            graph.layers[right_idx].nodes.iter().rev().copied().collect();
        for nid in right_nodes_rev {
            for &port_id in &graph.node(nid).ports {
                if graph.port(port_id).side != PortSide::West {
                    continue;
                }
                for &eid in &graph.port(port_id).incoming_edges {
                    if !is_handled_edge(graph, eid) {
                        continue;
                    }
                    let src = graph.port(graph.edge(eid).source).owner;
                    if graph.node(src).layer != Some(left_idx) {
                        continue;
                    }
                    if open_edges.is_empty() {
                        continue;
                    }
                    // Walk the stack from the top; everything we pass marks a crossing.
                    let mut cursor = open_edges.len();
                    while cursor > 0 {
                        cursor -= 1;
                        let last = open_edges[cursor];
                        if last == eid {
                            open_edges.remove(cursor);
                            break;
                        }
                        self.edge_crossing.insert(last, true);
                        self.edge_crossing.insert(eid, true);
                    }
                }
            }
        }
    }

    fn identify_paths(&mut self, graph: &LGraph) -> Vec<Vec<EdgeId>> {
        let mut paths: Vec<Vec<EdgeId>> = Vec::new();
        let layer_count = graph.layers.len();
        // Iterate junctions in node-id order so two sibling junctions claim
        // intermediate Visited nodes in the same order every run. The natural
        // layer-walk order is deterministic but differs across ports — sort
        // by the arena `id` field so the resulting path → weight assignment
        // is stable.
        let mut junctions: Vec<NodeId> = (0..layer_count)
            .flat_map(|i| graph.layers[i].nodes.clone())
            .filter(|nid| self.node_state.get(nid).copied() == Some(NodeState::Junction))
            .collect();
        junctions.sort_by_key(|nid| graph.node(*nid).id);
        for junction in junctions {
            let incidents: Vec<EdgeId> = connected_handled_edges(graph, junction);
            for eid in incidents {
                let mut path: Vec<EdgeId> = Vec::new();
                self.follow(graph, eid, junction, &mut path);
                if path.len() > 1 {
                    paths.push(path);
                }
            }
        }
        paths
    }

    fn follow(&mut self, graph: &LGraph, edge: EdgeId, current: NodeId, path: &mut Vec<EdgeId>) {
        let mut edge = edge;
        let mut current = current;
        loop {
            let other = other_end(graph, edge, current);
            path.push(edge);
            let other_state = self.node_state.get(&other).copied().unwrap_or(NodeState::Unvisited);
            if other_state == NodeState::Visited
                || other_state == NodeState::Junction
                || self.edge_crossing.get(&edge).copied().unwrap_or(false)
            {
                return;
            }
            self.node_state.insert(other, NodeState::Visited);
            let incident: Vec<EdgeId> = connected_handled_edges(graph, other);
            let Some(next_edge) = incident.into_iter().find(|&inc| inc != edge) else {
                return;
            };
            current = other;
            edge = next_edge;
        }
    }

    // Post-process two-paths.

    fn post_process_two_paths(&mut self, graph: &LGraph) {
        let mut queue: std::collections::VecDeque<(EdgeId, EdgeId)> =
            self.two_paths.drain(..).collect();
        let mut again: Vec<(EdgeId, EdgeId)> = Vec::new();
        while let Some(pair) = queue.pop_front() {
            if self.improve_two_path(graph, pair, true) {
                again.push(pair);
            }
        }
        while let Some(pair) = again.pop() {
            self.improve_two_path(graph, pair, false);
        }
    }

    fn improve_two_path(
        &mut self,
        graph: &LGraph,
        (left_edge_id, right_edge_id): (EdgeId, EdgeId),
        probe: bool,
    ) -> bool {
        let left_rep = match self.edge_reps.get(&left_edge_id) {
            Some(r) => EdgeRep { left: r.left, right: r.right },
            None => return false,
        };
        let right_rep = match self.edge_reps.get(&right_edge_id) {
            Some(r) => EdgeRep { left: r.left, right: r.right },
            None => return false,
        };

        if left_rep.is_straight(&self.ng) && right_rep.is_straight(&self.ng) {
            return false;
        }

        // Center node: the LNode that owns the port at `leftEdge.right.target`
        // — which is the target of the left edge (so src node of the right).
        let center_nid = graph.port(graph.edge(left_edge_id).target).owner;
        let Some(center_rep) = self.node_reps.get(&center_nid).map(|r| NodeRep {
            head: r.head,
            tail: r.tail,
            is_flexible: r.is_flexible,
            size_edge: r.size_edge,
        }) else {
            return false;
        };

        // Identify space above and below the center node.
        let center_layer_idx = match graph.node(center_nid).layer.get() {
            Some(i) => i,
            None => return false,
        };
        let node_idx = layer_position(graph, center_layer_idx, center_nid);
        let layer_nodes = &graph.layers[center_layer_idx].nodes;

        let mut above_dist = f64::INFINITY;
        if node_idx > 0 {
            let above = layer_nodes[node_idx - 1];
            let above_rep = &self.node_reps[&above];
            let spacing = vertical_spacing(graph, above, center_nid).ceil();
            above_dist = (self.ng.nodes[center_rep.head].layer as f64
                - graph.node(center_nid).margin.top)
                - (self.ng.nodes[above_rep.head].layer as f64
                    + graph.node(above).size.y
                    + graph.node(above).margin.bottom)
                - spacing;
        }
        let mut below_dist = f64::INFINITY;
        if node_idx + 1 < layer_nodes.len() {
            let below = layer_nodes[node_idx + 1];
            let below_rep = &self.node_reps[&below];
            let spacing = vertical_spacing(graph, below, center_nid).ceil();
            below_dist = (self.ng.nodes[below_rep.head].layer as f64
                - graph.node(below).margin.top)
                - (self.ng.nodes[center_rep.head].layer as f64
                    + graph.node(center_nid).size.y
                    + graph.node(center_nid).margin.bottom)
                - spacing;
        }

        let epsilon = 1e-5;
        if probe && (above_dist - below_dist).abs() < epsilon {
            return true;
        }

        let a = length(&self.ng, left_rep.left);
        let b = -length(&self.ng, left_rep.right);
        let c = -length(&self.ng, right_rep.left);
        let d = length(&self.ng, right_rep.right);

        let left_not_straight = left_rep.not_straight_by(&self.ng);
        let right_not_straight = right_rep.not_straight_by(&self.ng);
        let case_d = left_not_straight > 0 && right_not_straight < 0;
        let case_c = left_not_straight < 0 && right_not_straight > 0;

        let left_sum = self.ng.nodes[self.ng.edges[left_rep.left].target].layer
            + self.ng.edges[left_rep.right].delta;
        let right_sum = self.ng.nodes[self.ng.edges[right_rep.right].target].layer
            + self.ng.edges[right_rep.left].delta;
        let case_b = left_sum < right_sum;
        let case_a = left_sum > right_sum;

        let mut move_by = 0;
        if !case_d && !case_c {
            if case_a {
                if (above_dist + c as f64) > 0.0 {
                    move_by = c;
                } else if (below_dist - a as f64) > 0.0 {
                    move_by = a;
                }
            } else if case_b {
                if (above_dist + b as f64) > 0.0 {
                    move_by = b;
                } else if (below_dist - d as f64) > 0.0 {
                    move_by = d;
                }
            }
        }

        if move_by != 0 {
            self.ng.nodes[center_rep.head].layer += move_by;
            if center_rep.is_flexible {
                self.ng.nodes[center_rep.tail].layer += move_by;
            }
        }

        false
    }

    // Delegate to the shared Gansner solver. Subtree optimization is
    // disabled because `node_reps` / `edge_reps` / `port_map` cache stable
    // `NGraph` indices that must survive across the solve, and the subtree
    // pre-pass rewrites both `nodes` and `edges` vectors.

    fn run_network_simplex(&mut self, iter_limit: usize) {
        if self.ng.nodes.is_empty() {
            return;
        }
        let ng = std::mem::take(&mut self.ng);
        let result = Solver::new(ng).with_iter_limit(iter_limit).solve();
        self.ng = result.graph;
    }

    // Apply positions back to the LGraph.

    fn apply_positions(&self, graph: &mut LGraph) {
        let mut max_height = 0.0f64;
        for layer_idx in 0..graph.layers.len() {
            let nids = graph.layers[layer_idx].nodes.clone();
            let mut layer_bottom = 0.0f64;
            for nid in nids {
                let Some(rep) = self.node_reps.get(&nid) else {
                    continue;
                };
                let min_y = self.ng.nodes[rep.head].layer as f64;
                let max_y = self.ng.nodes[rep.tail].layer as f64;

                graph.node_mut(nid).position.y = min_y;

                let size_delta = (max_y - min_y) - graph.node(nid).size.y;
                let flexible_node = is_flexible_node(graph, nid);
                let nf = get_node_flexibility(graph, nid);

                if flexible_node && nf.is_flexible_size_where_space_permits() {
                    graph.node_mut(nid).size.y += size_delta;
                }

                if flexible_node && nf.is_flexible_ports() {
                    let ports: Vec<PortId> = graph.node(nid).ports.to_vec();
                    for port_id in ports {
                        let side = graph.port(port_id).side;
                        if matches!(side, PortSide::East | PortSide::West) {
                            let nn = self.port_map[&port_id];
                            let port_layer = self.ng.nodes[nn].layer as f64;
                            graph.port_mut(port_id).position.y = port_layer - min_y;
                        }
                    }
                    if size_delta.abs() > 0.0 {
                        let label_ids: Vec<LabelId> = graph.node(nid).labels.to_vec();
                        let placement = graph.node(nid).properties.get(&NODE_LABEL_PLACEMENT);
                        for label_id in label_ids {
                            adjust_label_position(graph, label_id, placement, size_delta);
                        }
                        if nf.is_flexible_size_where_space_permits() {
                            let south_ports: Vec<PortId> = graph
                                .node(nid)
                                .ports
                                .iter()
                                .copied()
                                .filter(|&p| graph.port(p).side == PortSide::South)
                                .collect();
                            for p in south_ports {
                                graph.port_mut(p).position.y += size_delta;
                            }
                        }
                    }
                }

                let node = graph.node(nid);
                let bottom = node.position.y + node.size.y + node.margin.bottom;
                if bottom > layer_bottom {
                    layer_bottom = bottom;
                }
            }
            graph.layers[layer_idx].size.y = layer_bottom;
            if layer_bottom > max_height {
                max_height = layer_bottom;
            }
        }
        graph.size.y = max_height;
    }
}

// Flexibility helpers

fn get_node_flexibility(graph: &LGraph, nid: NodeId) -> NodeFlexibility {
    let per_node = graph.node(nid).properties.get(&NODE_FLEXIBILITY);
    if !matches!(per_node, NodeFlexibility::None) {
        return per_node;
    }
    graph.options.node_placement_network_simplex_node_flexibility
}

fn is_flexible_node(graph: &LGraph, nid: NodeId) -> bool {
    if graph.node(nid).node_type != NodeType::Normal {
        return false;
    }
    if graph.node(nid).ports.len() <= 1 {
        return false;
    }
    let pc = graph.node(nid).port_constraints();
    let pc = if matches!(pc, PortConstraints::Undefined) {
        graph.options.port_constraints
    } else {
        pc
    };
    if pc.is_pos_fixed() {
        return false;
    }
    let nf = get_node_flexibility(graph, nid);
    if matches!(nf, NodeFlexibility::None) {
        return false;
    }
    if !nf.is_flexible_size_where_space_permits() {
        // Same per-node spacing override fallback as in `transform_ports`.
        let port_spacing = graph
            .node(nid)
            .properties
            .get(&SPACING_PORT_PORT_OVERRIDE)
            .unwrap_or(graph.options.spacing.port_port);
        let west_count = graph
            .node(nid)
            .ports
            .iter()
            .filter(|&&p| graph.port(p).side == PortSide::West)
            .count();
        let east_count = graph
            .node(nid)
            .ports
            .iter()
            .filter(|&&p| graph.port(p).side == PortSide::East)
            .count();
        let required_west =
            if west_count >= 1 { (west_count as f64 - 1.0) * port_spacing } else { 0.0 };
        let required_east =
            if east_count >= 1 { (east_count as f64 - 1.0) * port_spacing } else { 0.0 };
        if required_west > graph.node(nid).size.y {
            return false;
        }
        if required_east > graph.node(nid).size.y {
            return false;
        }
    }
    true
}

fn ports_surrounding_for_node(graph: &LGraph, nid: NodeId) -> crate::math::Margin {
    if graph.node(nid).properties.has(&SPACING_PORTS_SURROUNDING) {
        graph.node(nid).properties.get(&SPACING_PORTS_SURROUNDING)
    } else {
        graph.properties.get(&SPACING_PORTS_SURROUNDING)
    }
}

// Straight-edge helpers

fn compute_node_state(graph: &LGraph, nid: NodeId) -> NodeState {
    let mut inco = 0usize;
    let mut ouco = 0usize;
    for &port_id in &graph.node(nid).ports {
        for &eid in &graph.port(port_id).incoming_edges {
            if is_handled_edge(graph, eid) {
                inco += 1;
            }
        }
        for &eid in &graph.port(port_id).outgoing_edges {
            if is_handled_edge(graph, eid) {
                ouco += 1;
            }
        }
        if inco > 1 || ouco > 1 {
            return NodeState::Junction;
        }
    }
    if inco + ouco == 1 {
        return NodeState::Junction;
    }
    NodeState::Unvisited
}

fn connected_handled_edges(graph: &LGraph, nid: NodeId) -> Vec<EdgeId> {
    let mut out: Vec<EdgeId> = Vec::new();
    for &port_id in &graph.node(nid).ports {
        for &eid in &graph.port(port_id).outgoing_edges {
            if is_handled_edge(graph, eid) {
                out.push(eid);
            }
        }
        for &eid in &graph.port(port_id).incoming_edges {
            if is_handled_edge(graph, eid) {
                out.push(eid);
            }
        }
    }
    out
}

fn other_end(graph: &LGraph, edge: EdgeId, current: NodeId) -> NodeId {
    let e = graph.edge(edge);
    let src_owner = graph.port(e.source).owner;
    if src_owner == current { graph.port(e.target).owner } else { src_owner }
}

fn order_two_path(graph: &LGraph, path: &[EdgeId]) -> Vec<EdgeId> {
    assert_eq!(path.len(), 2);
    let first = path[0];
    let second = path[1];
    let first_tgt = graph.port(graph.edge(first).target).owner;
    let second_src = graph.port(graph.edge(second).source).owner;
    if first_tgt == second_src { vec![first, second] } else { vec![second, first] }
}

fn is_two_path_center_node_flexible(graph: &LGraph, path: &[EdgeId]) -> bool {
    let first = path[0];
    let center = graph.port(graph.edge(first).target).owner;
    is_flexible_node(graph, center)
}

fn path_contains_long_edge_dummy(graph: &LGraph, path: &[EdgeId]) -> bool {
    if path.is_empty() {
        return false;
    }
    let first_src = graph.port(graph.edge(path[0]).source).owner;
    if graph.node(first_src).node_type == NodeType::LongEdge {
        return true;
    }
    path.iter().any(|&eid| {
        graph.node(graph.port(graph.edge(eid).target).owner).node_type == NodeType::LongEdge
    })
}

fn path_contains_flexible_size_permits(graph: &LGraph, path: &[EdgeId]) -> bool {
    if path.is_empty() {
        return false;
    }
    let first_src = graph.port(graph.edge(path[0]).source).owner;
    if get_node_flexibility(graph, first_src).is_flexible_size_where_space_permits() {
        return true;
    }
    path.iter().any(|&eid| {
        let tgt = graph.port(graph.edge(eid).target).owner;
        get_node_flexibility(graph, tgt).is_flexible_size_where_space_permits()
    })
}

fn adjust_label_position(
    graph: &mut LGraph,
    label_id: LabelId,
    placement: NodeLabelPlacement,
    size_delta: f64,
) {
    if placement.contains(NodeLabelPlacement::V_BOTTOM) {
        graph.label_mut(label_id).position.y += size_delta;
    } else if placement.contains(NodeLabelPlacement::V_CENTER) {
        graph.label_mut(label_id).position.y += size_delta / 2.0;
    }
}

fn length(ng: &NGraph, eidx: usize) -> i32 {
    let e = &ng.edges[eidx];
    (ng.nodes[e.source].layer - ng.nodes[e.target].layer).abs() - e.delta
}

fn find_components(g: &NGraph) -> Vec<Vec<usize>> {
    let n = g.nodes.len();
    let mut visited = vec![false; n];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for start in 0..n {
        if visited[start] {
            continue;
        }
        let mut comp = Vec::new();
        let mut stack = vec![start];
        while let Some(cur) = stack.pop() {
            if visited[cur] {
                continue;
            }
            visited[cur] = true;
            comp.push(cur);
            let neighbors: Vec<usize> = g.nodes[cur]
                .outgoing
                .iter()
                .chain(g.nodes[cur].incoming.iter())
                .map(|&eid| {
                    let e = &g.edges[eid];
                    if e.source == cur { e.target } else { e.source }
                })
                .collect();
            for nxt in neighbors {
                if !visited[nxt] {
                    stack.push(nxt);
                }
            }
        }
        components.push(comp);
    }
    components
}

// Helpers

fn is_handled_edge(graph: &LGraph, eid: EdgeId) -> bool {
    let edge = graph.edge(eid);
    let src = graph.port(edge.source).owner;
    let tgt = graph.port(edge.target).owner;
    if src == tgt {
        return false;
    }
    if graph.node(src).layer == graph.node(tgt).layer {
        return false;
    }
    true
}

fn is_in_layer_edge(graph: &LGraph, eid: EdgeId) -> bool {
    let edge = graph.edge(eid);
    let src = graph.port(edge.source).owner;
    let tgt = graph.port(edge.target).owner;
    if src == tgt {
        return false;
    }
    graph.node(src).layer == graph.node(tgt).layer
}

fn layer_position(graph: &LGraph, layer_idx: usize, nid: NodeId) -> usize {
    graph.layers[layer_idx]
        .nodes
        .iter()
        .position(|&n| n == nid)
        .unwrap_or(usize::MAX)
}

fn edge_weight(graph: &LGraph, eid: EdgeId) -> f64 {
    let edge = graph.edge(eid);
    let priority = edge.properties.get(&PRIORITY_STRAIGHTNESS).max(1);
    let src_type = graph.node(graph.port(edge.source).owner).node_type;
    let tgt_type = graph.node(graph.port(edge.target).owner).node_type;
    priority as f64 * weight_for_types(src_type, tgt_type)
}

fn weight_for_types(t1: NodeType, t2: NodeType) -> f64 {
    if t1 == NodeType::Normal && t2 == NodeType::Normal {
        EDGE_WEIGHT_BASE
    } else if t1 == NodeType::Normal || t2 == NodeType::Normal {
        2.0 * EDGE_WEIGHT_BASE
    } else {
        8.0 * EDGE_WEIGHT_BASE
    }
}

fn vertical_spacing(graph: &LGraph, n1: NodeId, n2: NodeId) -> f64 {
    let t1 = graph.node(n1).node_type;
    let t2 = graph.node(n2).node_type;
    let sp = &graph.options.spacing;
    use NodeType::*;
    match (t1, t2) {
        (Normal, Normal) => sp.node_node,
        (Normal, LongEdge) | (LongEdge, Normal) => sp.edge_node,
        (Normal, NorthSouthPort) | (NorthSouthPort, Normal) => sp.edge_node,
        (LongEdge, LongEdge) => sp.edge_edge,
        (LongEdge, NorthSouthPort) | (NorthSouthPort, LongEdge) => sp.edge_edge,
        (NorthSouthPort, NorthSouthPort) => sp.edge_edge,
        _ => sp.edge_edge,
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<EdgeRep>();
    }
}
