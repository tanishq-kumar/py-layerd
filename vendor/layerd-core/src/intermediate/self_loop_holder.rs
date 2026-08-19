//! Self-loop data model and per-node `SelfLoopHolder` storage.
//!
//! Public surface: `needs_self_loop_processing`, `install`, `sl_port_map`,
//! `sl_hyper_loops`, `routing_slot_count`. The holder is stored as a
//! property via `SELF_LOOP_HOLDER`.

use std::{collections::VecDeque, sync::LazyLock};

use hashbrown::HashMap;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::{PortSide, PortSideSet},
    },
    intermediate::self_hyper_loop_labels::SelfHyperLoopLabels,
    properties::{PropertyKey, internal::EDGE_LABELS_INLINE},
};

/// Discrete classification of a self-hyper-loop based on the set of port sides
/// its ports occupy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelfLoopType {
    OneSide,
    TwoSidesCorner,
    TwoSidesOpposing,
    ThreeSides,
    FourSides,
}

impl SelfLoopType {
    /// Classify a hyper-loop by the set of port sides its ports occupy.
    pub fn from_port_sides(sides: PortSideSet) -> Option<Self> {
        let count = sides.bits().count_ones();
        match count {
            1 => Some(SelfLoopType::OneSide),
            2 => {
                let opposing = (sides.contains(PortSideSet::EAST)
                    && sides.contains(PortSideSet::WEST))
                    || (sides.contains(PortSideSet::NORTH) && sides.contains(PortSideSet::SOUTH));
                Some(if opposing {
                    SelfLoopType::TwoSidesOpposing
                } else {
                    SelfLoopType::TwoSidesCorner
                })
            }
            3 => Some(SelfLoopType::ThreeSides),
            4 => Some(SelfLoopType::FourSides),
            _ => None,
        }
    }
}

/// Convert a `PortSide` into a one-element `PortSideSet`.
#[inline]
pub fn portside_set_of(side: PortSide) -> PortSideSet {
    match side {
        PortSide::North => PortSideSet::NORTH,
        PortSide::East => PortSideSet::EAST,
        PortSide::South => PortSideSet::SOUTH,
        PortSide::West => PortSideSet::WEST,
        PortSide::Undefined => PortSideSet::empty(),
    }
}

/// Side ordinal `0..4` for `North/East/South/West`. Returns `4` for
/// `Undefined`.
#[inline]
pub fn portside_index(side: PortSide) -> usize {
    match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => 4,
    }
}

/// Clockwise neighbor of a port side (N→E→S→W→N).
#[inline]
pub fn portside_right(side: PortSide) -> PortSide {
    match side {
        PortSide::North => PortSide::East,
        PortSide::East => PortSide::South,
        PortSide::South => PortSide::West,
        PortSide::West => PortSide::North,
        PortSide::Undefined => PortSide::Undefined,
    }
}

/// Counter-clockwise neighbor of a port side (N→W→S→E→N).
#[inline]
pub fn portside_left(side: PortSide) -> PortSide {
    match side {
        PortSide::North => PortSide::West,
        PortSide::West => PortSide::South,
        PortSide::South => PortSide::East,
        PortSide::East => PortSide::North,
        PortSide::Undefined => PortSide::Undefined,
    }
}

/// Index into a `SelfLoopHolder`'s `sl_edges` vector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SelfLoopEdgeIdx(pub u32);

/// Wrapper around an `LEdge` that represents a single self-loop edge.
#[derive(Debug, Clone, Copy)]
pub struct SelfLoopEdge {
    pub edge_id: EdgeId,
    pub source: PortId,
    pub target: PortId,
    pub hyper_loop: Option<u32>,
}

impl SelfLoopEdge {
    /// True when any label on this edge carries the `EDGE_LABELS_INLINE` hint.
    pub fn is_inline(&self, graph: &LGraph) -> bool {
        graph
            .edge(self.edge_id)
            .labels
            .iter()
            .any(|&lid| graph.label(lid).properties.get(&EDGE_LABELS_INLINE))
    }
}

/// Wrapper around an `LPort` that participates in at least one self-loop.
#[derive(Debug, Clone)]
pub struct SelfLoopPort {
    pub port_id: PortId,
    pub side: PortSide,
    pub had_only_self_loops: bool,
    pub is_hidden: bool,
    pub incoming_sl_edges: SmallVec<SelfLoopEdgeIdx, 1>,
    pub outgoing_sl_edges: SmallVec<SelfLoopEdgeIdx, 1>,
}

impl SelfLoopPort {
    /// Number of incoming minus outgoing self-loop edges incident to this port.
    pub fn sl_net_flow(&self) -> i32 {
        self.incoming_sl_edges.len() as i32 - self.outgoing_sl_edges.len() as i32
    }
}

/// A connected component of self-loop edges sharing ports on a single node.
#[derive(Debug, Clone)]
pub struct SelfHyperLoop {
    pub sl_ports: Vec<PortId>,
    pub sl_edges: SmallVec<SelfLoopEdgeIdx, 1>,
    pub leftmost_port: Option<PortId>,
    pub rightmost_port: Option<PortId>,
    /// Set of port sides the loop is routed along. Populated lazily during
    /// routing by `RoutingDirector`, not by `install`.
    pub occupied_port_sides: PortSideSet,
    pub routing_slot: [u32; 4],
    pub self_loop_type: Option<SelfLoopType>,
    /// Ports belonging to this loop, bucketed by current port side. Populated
    /// by `compute_ports_per_side` once port-side assignment is final.
    pub sl_ports_by_side: [Vec<PortId>; 4],
    /// Labels attached to any edge in this loop. `None` when the loop carries
    /// no labels at all.
    pub sl_labels: Option<SelfHyperLoopLabels>,
}

impl Default for SelfHyperLoop {
    fn default() -> Self {
        SelfHyperLoop {
            sl_ports: Vec::new(),
            sl_edges: SmallVec::new(),
            leftmost_port: None,
            rightmost_port: None,
            occupied_port_sides: PortSideSet::empty(),
            routing_slot: [0; 4],
            self_loop_type: None,
            sl_ports_by_side: [const { Vec::new() }; 4],
            sl_labels: None,
        }
    }
}

impl SelfHyperLoop {
    /// Returns the routing slot this loop occupies on a given side.
    pub fn routing_slot_for(&self, side: PortSide) -> u32 {
        let idx = portside_index(side);
        if idx < 4 { self.routing_slot[idx] } else { 0 }
    }

    /// Set the routing slot on a given side and return the resulting cap so
    /// that the holder can update its per-side count.
    pub fn set_routing_slot(&mut self, side: PortSide, slot: u32) -> u32 {
        let idx = portside_index(side);
        if idx < 4 {
            self.routing_slot[idx] = slot;
            slot + 1
        } else {
            0
        }
    }

    /// Populate `sl_ports_by_side` from the current port-side assignment and
    /// recompute `self_loop_type`.
    pub fn compute_ports_per_side(&mut self, graph: &LGraph) {
        for bucket in &mut self.sl_ports_by_side {
            bucket.clear();
        }
        let mut sides = PortSideSet::empty();
        for &pid in &self.sl_ports {
            let side = graph.port(pid).side;
            let idx = portside_index(side);
            if idx < 4 {
                self.sl_ports_by_side[idx].push(pid);
                sides |= portside_set_of(side);
            }
        }
        self.self_loop_type = SelfLoopType::from_port_sides(sides);
    }

    /// Ports on the given side once `compute_ports_per_side` has run.
    pub fn sl_ports_on_side(&self, side: PortSide) -> &[PortId] {
        let idx = portside_index(side);
        if idx < 4 { &self.sl_ports_by_side[idx] } else { &[] }
    }
}

/// Aggregates self-loop bookkeeping for a single node.
#[derive(Debug, Clone)]
pub struct SelfLoopHolder {
    pub sl_port_map: HashMap<PortId, SelfLoopPort>,
    pub sl_edges: Vec<SelfLoopEdge>,
    pub sl_hyper_loops: Vec<SelfHyperLoop>,
    pub ports_hidden: bool,
    pub routing_slot_count: [u32; 4],
}

impl SelfLoopHolder {
    /// Returns true if `node` is a normal node with at least one self-loop edge.
    pub fn needs_self_loop_processing(graph: &LGraph, node: NodeId) -> bool {
        if graph.node(node).node_type != NodeType::Normal {
            return false;
        }
        for &pid in &graph.node(node).ports {
            for &eid in &graph.port(pid).outgoing_edges {
                if graph.edge(eid).target_owner == node {
                    return true;
                }
            }
        }
        false
    }

    /// Build a `SelfLoopHolder` describing every self-loop on `node`.
    ///
    /// Panics if the node does not require self-loop processing.
    pub fn install(graph: &LGraph, node: NodeId) -> Self {
        assert!(
            Self::needs_self_loop_processing(graph, node),
            "install() called on node that does not need self-loop processing"
        );

        let mut sl_port_map: HashMap<PortId, SelfLoopPort> = HashMap::new();
        let mut sl_edges: Vec<SelfLoopEdge> = Vec::new();

        // Collect per-port classification (side, had-only-self-loops) and the
        // set of self-loop edges leaving the node.
        for &pid in &graph.node(node).ports {
            let port = graph.port(pid);
            let mut had_self = false;
            let mut had_non_self = false;
            for &eid in port.outgoing_edges.iter().chain(port.incoming_edges.iter()) {
                let edge = graph.edge(eid);
                if edge.source_owner == node && edge.target_owner == node {
                    had_self = true;
                } else {
                    had_non_self = true;
                }
            }
            sl_port_map.insert(
                pid,
                SelfLoopPort {
                    port_id: pid,
                    side: port.side,
                    had_only_self_loops: had_self && !had_non_self,
                    is_hidden: false,
                    incoming_sl_edges: SmallVec::new(),
                    outgoing_sl_edges: SmallVec::new(),
                },
            );
        }

        // Enumerate self-loop edges via outgoing-edges scan to keep insertion
        // order deterministic across runs.
        let mut bfs_port_order: Vec<PortId> = Vec::new();
        for &pid in &graph.node(node).ports {
            for &eid in &graph.port(pid).outgoing_edges {
                let edge = graph.edge(eid);
                if edge.target_owner != node {
                    continue;
                }
                push_unique_port(&mut bfs_port_order, edge.source);
                push_unique_port(&mut bfs_port_order, edge.target);
                let idx = SelfLoopEdgeIdx(sl_edges.len() as u32);
                sl_edges.push(SelfLoopEdge {
                    edge_id: eid,
                    source: edge.source,
                    target: edge.target,
                    hyper_loop: None,
                });
                sl_port_map.get_mut(&edge.source).unwrap().outgoing_sl_edges.push(idx);
                sl_port_map.get_mut(&edge.target).unwrap().incoming_sl_edges.push(idx);
            }
        }

        // BFS-merge self-loop edges into hyper-loops by shared ports.
        let port_order: Vec<PortId> = bfs_port_order;
        let mut visited: HashMap<PortId, bool> = HashMap::with_capacity(port_order.len());
        for &pid in &port_order {
            visited.insert(pid, false);
        }

        let layout_direction = graph.options.direction;
        let label_label_spacing = graph.options.spacing.label_label;

        let mut sl_hyper_loops: Vec<SelfHyperLoop> = Vec::new();
        for &start in &port_order {
            if !sl_port_map.contains_key(&start) {
                continue;
            }
            let port_entry = &sl_port_map[&start];
            let starts_loop = !port_entry.incoming_sl_edges.is_empty()
                || !port_entry.outgoing_sl_edges.is_empty();
            if !starts_loop || *visited.get(&start).unwrap_or(&true) {
                continue;
            }
            let loop_idx = sl_hyper_loops.len() as u32;
            let mut loop_state = SelfHyperLoop::default();

            let mut queue: VecDeque<PortId> = VecDeque::new();
            queue.push_back(start);
            while let Some(curr) = queue.pop_front() {
                if std::mem::replace(visited.get_mut(&curr).unwrap(), true) {
                    continue;
                }
                if !loop_state.sl_ports.contains(&curr) {
                    loop_state.sl_ports.push(curr);
                }
                let side = sl_port_map[&curr].side;
                loop_state.occupied_port_sides |= portside_set_of(side);

                let outgoing = sl_port_map[&curr].outgoing_sl_edges.clone();
                for edge_idx in outgoing {
                    add_edge_with_labels(
                        &mut loop_state,
                        edge_idx,
                        &sl_edges,
                        graph,
                        layout_direction,
                        label_label_spacing,
                    );
                    let target = sl_edges[edge_idx.0 as usize].target;
                    if !visited.get(&target).copied().unwrap_or(true) {
                        queue.push_back(target);
                    }
                }
                let incoming = sl_port_map[&curr].incoming_sl_edges.clone();
                for edge_idx in incoming {
                    add_edge_with_labels(
                        &mut loop_state,
                        edge_idx,
                        &sl_edges,
                        graph,
                        layout_direction,
                        label_label_spacing,
                    );
                    let source = sl_edges[edge_idx.0 as usize].source;
                    if !visited.get(&source).copied().unwrap_or(true) {
                        queue.push_back(source);
                    }
                }
            }

            for &edge_idx in &loop_state.sl_edges {
                sl_edges[edge_idx.0 as usize].hyper_loop = Some(loop_idx);
            }
            sl_hyper_loops.push(loop_state);
        }

        SelfLoopHolder {
            sl_port_map,
            sl_edges,
            sl_hyper_loops,
            ports_hidden: false,
            routing_slot_count: [0; 4],
        }
    }

    /// Run `SelfHyperLoop::compute_ports_per_side` on every hyper-loop.
    pub fn compute_ports_per_side_all(&mut self, graph: &LGraph) {
        for hyper in &mut self.sl_hyper_loops {
            hyper.compute_ports_per_side(graph);
        }
    }
}

fn push_unique_port(ports: &mut Vec<PortId>, port: PortId) {
    if !ports.contains(&port) {
        ports.push(port);
    }
}

/// Add an edge to `loop_state.sl_edges` if not already present, and collect
/// the edge's labels into the loop's `SelfHyperLoopLabels`.
fn add_edge_with_labels(
    loop_state: &mut SelfHyperLoop,
    edge_idx: SelfLoopEdgeIdx,
    sl_edges: &[SelfLoopEdge],
    graph: &LGraph,
    layout_direction: crate::options::enums::LayoutDirection,
    label_label_spacing: f64,
) {
    if loop_state.sl_edges.contains(&edge_idx) {
        return;
    }
    loop_state.sl_edges.push(edge_idx);

    let edge_id = sl_edges[edge_idx.0 as usize].edge_id;
    let label_ids: SmallVec<crate::graph::index::LabelId, 3> =
        graph.edge(edge_id).labels.iter().copied().collect();
    if label_ids.is_empty() {
        return;
    }
    if loop_state.sl_labels.is_none() {
        loop_state.sl_labels =
            Some(SelfHyperLoopLabels::new(layout_direction, label_label_spacing));
    }
    if let Some(labels) = loop_state.sl_labels.as_mut() {
        for lid in label_ids {
            let size = graph.label(lid).size;
            labels.add_label(lid, size);
        }
    }
}

struct SelfLoopHolderMarker;

/// Property storing the `SelfLoopHolder` for a node, set by self-loop pre-processing.
pub static SELF_LOOP_HOLDER: LazyLock<PropertyKey<Option<SelfLoopHolder>>> =
    LazyLock::new(|| PropertyKey::of::<SelfLoopHolderMarker>(|| None));

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<SelfLoopEdge>();
    }
}
