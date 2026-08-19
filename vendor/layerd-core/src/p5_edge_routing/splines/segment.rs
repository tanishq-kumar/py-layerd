//! Spline routing segment data structure.
//!
//! A segment represents one or more `LEdge`s between adjacent layers. When it
//! represents more than one edge the segment models a 1:n hyperedge. Segments
//! carry the bookkeeping consumed by the spline router's cycle-breaking /
//! topological numbering, plus the per-edge `EdgeInformation` the
//! `FinalSplineBendpointsCalculator` needs after long edges are joined back.
//!
//! ## Storage model
//!
//! Segments live in a flat `Vec<SplineSegment>` that the router hangs off the
//! graph via the `SPLINE_SEGMENT_STORE` property. Cross-segment references
//! (dependency source/target) use local `SegmentId(u32)` indices rather than
//! heap-allocated pointers; this keeps the cycle-breaking inner loop
//! cache-friendly and avoids `Rc`/`RefCell` gymnastics.

use std::sync::LazyLock;

use hashbrown::HashMap;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    properties::PropertyKey,
};

/// Local handle for a segment inside the router's `Vec<SplineSegment>` store.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SegmentId(pub u32);

/// Dependency edge in the segment-ordering graph: source must be placed left
/// of target to minimise crossings. Weight of `0` means "equal slots would
/// overlap a shared vertical run" — still a dependency, just no preference.
#[derive(Debug, Clone, Copy)]
pub struct Dependency {
    pub source: SegmentId,
    pub target: SegmentId,
    pub weight: i32,
}

/// Per-edge snapshot captured during routing so the calculator can run after
/// long-edge joining has invalidated some of the underlying `LEdge` refs.
#[derive(Debug, Clone, Copy)]
pub struct EdgeInformation {
    pub start_y: f64,
    pub end_y: f64,
    pub normal_source_node: bool,
    pub normal_target_node: bool,
    pub inverted_left: bool,
    pub inverted_right: bool,
}

/// Which side of a between-layer gap a port sits on during segment creation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SideToProcess {
    Left,
    Right,
}

/// A spline segment between two adjacent layers.
#[derive(Debug, Clone)]
pub struct SplineSegment {
    // Multi-visit guard for hyperedges.
    pub handled: bool,

    // Cycle breaking / topological sort state.
    pub left_ports: Vec<PortId>,
    pub right_ports: Vec<PortId>,
    pub outgoing: Vec<Dependency>,
    pub incoming: Vec<Dependency>,
    pub mark: i32,
    pub inweight: i32,
    pub outweight: i32,
    pub rank: i32,

    // Segment characteristics.
    pub edges: Vec<EdgeId>,
    pub is_straight: bool,
    pub bbox_x: f64,
    pub bbox_y: f64,
    pub bbox_width: f64,
    pub bbox_height: f64,
    pub is_west_of_initial_layer: bool,
    pub x_delta: f64,

    // Edge endpoints (optional — long-edge dummies may omit one side).
    pub source_port: Option<PortId>,
    pub target_port: Option<PortId>,

    // Role flags.
    pub initial_segment: bool,
    pub last_segment: bool,
    pub source_node: Option<NodeId>,
    pub target_node: Option<NodeId>,
    pub inverse_order: bool,

    // Hyperedge Y extents.
    pub hyper_edge_top_y_pos: f64,
    pub hyper_edge_bottom_y_pos: f64,
    pub center_control_point_y: f64,

    // Per-edge information captured at routing time.
    pub edge_information: HashMap<EdgeId, EdgeInformation>,
}

impl SplineSegment {
    /// Empty segment with default-initialised bookkeeping.
    fn empty() -> Self {
        SplineSegment {
            handled: false,
            left_ports: Vec::new(),
            right_ports: Vec::new(),
            outgoing: Vec::new(),
            incoming: Vec::new(),
            mark: 0,
            inweight: 0,
            outweight: 0,
            rank: 0,
            edges: Vec::new(),
            is_straight: false,
            bbox_x: 0.0,
            bbox_y: 0.0,
            bbox_width: 0.0,
            bbox_height: 0.0,
            is_west_of_initial_layer: false,
            x_delta: 0.0,
            source_port: None,
            target_port: None,
            initial_segment: false,
            last_segment: false,
            source_node: None,
            target_node: None,
            inverse_order: false,
            hyper_edge_top_y_pos: 0.0,
            hyper_edge_bottom_y_pos: 0.0,
            center_control_point_y: 0.0,
            edge_information: HashMap::new(),
        }
    }

    /// Build a 1:n hyper-edge segment. `single_port` is the single-side port;
    /// `hyper_edges` enumerates every opposing edge with its target-side tag.
    pub fn hyperedge(
        graph: &LGraph,
        single_port: PortId,
        hyper_edges: &[(SideToProcess, EdgeId)],
        source_side: SideToProcess,
    ) -> Self {
        let mut seg = SplineSegment::empty();
        if source_side == SideToProcess::Left {
            seg.left_ports.push(single_port);
        } else {
            seg.right_ports.push(single_port);
        }

        let mut y_min_target = f64::INFINITY;
        let mut y_max_target = f64::NEG_INFINITY;

        for &(side, edge_id) in hyper_edges {
            let edge = graph.edge(edge_id);
            let tgt_port = if edge.source == single_port { edge.target } else { edge.source };
            if side == SideToProcess::Left {
                seg.left_ports.push(tgt_port);
            } else {
                seg.right_ports.push(tgt_port);
            }
            let y_pos = anchor_y(graph, tgt_port);
            y_min_target = y_min_target.min(y_pos);
            y_max_target = y_max_target.max(y_pos);
        }

        let y_single = anchor_y(graph, single_port);
        seg.set_relevant_positions(y_single, y_min_target, y_max_target);

        for &(_, edge_id) in hyper_edges {
            seg.add_edge(graph, edge_id);
        }
        seg.is_straight = false;
        seg
    }

    /// Build a single-edge segment. May represent a straight edge.
    pub fn single_edge(
        graph: &LGraph,
        edge: EdgeId,
        source_side: SideToProcess,
        target_side: SideToProcess,
    ) -> Self {
        let mut seg = SplineSegment::empty();
        let e = graph.edge(edge);
        if source_side == SideToProcess::Left {
            seg.left_ports.push(e.source);
        } else {
            seg.right_ports.push(e.source);
        }
        if target_side == SideToProcess::Left {
            seg.left_ports.push(e.target);
        } else {
            seg.right_ports.push(e.target);
        }

        seg.add_edge(graph, edge);

        let source_y = anchor_y(graph, e.source);
        let target_y = anchor_y(graph, e.target);
        seg.set_relevant_positions(source_y, target_y, target_y);

        seg.is_straight = is_straight(source_y, target_y);
        seg
    }

    fn add_edge(&mut self, graph: &LGraph, edge_id: EdgeId) {
        self.edges.push(edge_id);
        let edge = graph.edge(edge_id);
        let source_y = anchor_y(graph, edge.source);
        let target_y = anchor_y(graph, edge.target);
        let ei = EdgeInformation {
            start_y: source_y,
            end_y: target_y,
            normal_source_node: is_normal_node(graph, graph.port(edge.source).owner),
            normal_target_node: is_normal_node(graph, graph.port(edge.target).owner),
            inverted_left: graph.port(edge.source).side == PortSide::West,
            inverted_right: graph.port(edge.target).side == PortSide::East,
        };
        self.edge_information.insert(edge_id, ei);
    }

    /// Position top/bottom/center based on source + min/max target Y.
    fn set_relevant_positions(&mut self, source_y: f64, target_y_min: f64, target_y_max: f64) {
        const HYPEREDGE_POS_OUTER_RATE: f64 = 0.9;
        const HYPEREDGE_POS_MID_RATE: f64 = 1.0 - HYPEREDGE_POS_OUTER_RATE;
        const ONE_HALF: f64 = 0.5;

        self.bbox_y = source_y.min(target_y_min);
        self.bbox_height = source_y.max(target_y_max) - self.bbox_y;

        if source_y < target_y_min {
            self.center_control_point_y = ONE_HALF * (source_y + target_y_min);
            self.hyper_edge_top_y_pos = HYPEREDGE_POS_MID_RATE * self.center_control_point_y
                + HYPEREDGE_POS_OUTER_RATE * source_y;
            self.hyper_edge_bottom_y_pos = HYPEREDGE_POS_MID_RATE * self.center_control_point_y
                + HYPEREDGE_POS_OUTER_RATE * target_y_min;
        } else {
            self.center_control_point_y = ONE_HALF * (source_y + target_y_max);
            self.hyper_edge_top_y_pos = HYPEREDGE_POS_MID_RATE * self.center_control_point_y
                + HYPEREDGE_POS_OUTER_RATE * target_y_max;
            self.hyper_edge_bottom_y_pos = HYPEREDGE_POS_MID_RATE * self.center_control_point_y
                + HYPEREDGE_POS_OUTER_RATE * source_y;
        }
    }

    pub fn is_hyper_edge(&self) -> bool {
        self.edges.len() > 1
    }
}

/// An edge is treated as a straight horizontal segment when the Y delta is
/// below this threshold.
pub const MAX_VERTICAL_DIFF_FOR_STRAIGHT: f64 = 0.2;

/// Returns true when an edge endpoint pair is below the straight threshold.
pub fn is_straight(first_y: f64, second_y: f64) -> bool {
    (first_y - second_y).abs() < MAX_VERTICAL_DIFF_FOR_STRAIGHT
}

/// Absolute Y anchor for a port. North/south ports read their routing-stage Y
/// from `SPLINE_NS_PORT_Y_COORD`; everything else uses the port's own
/// position + anchor offset.
pub fn anchor_y(graph: &LGraph, port_id: PortId) -> f64 {
    let port = graph.port(port_id);
    if matches!(port.side, PortSide::North | PortSide::South) {
        return port.properties.get(&crate::properties::internal::SPLINE_NS_PORT_Y_COORD);
    }
    let node = graph.node(port.owner);
    node.position.y + port.position.y + port.anchor.y
}

/// Normal-like node predicate.
///
/// `BIG_NODE` is handled as normal but the port does not currently model
/// it separately from `Normal`, so the check is simpler.
pub fn is_normal_node(graph: &LGraph, node: NodeId) -> bool {
    use crate::graph::node::NodeType;
    let nt = graph.node(node).node_type;
    matches!(nt, NodeType::Normal | NodeType::BreakingPoint)
}

/// Starting-node predicate.
pub fn is_qualified_as_starting_node(graph: &LGraph, node: NodeId) -> bool {
    use crate::graph::node::NodeType;
    let nt = graph.node(node).node_type;
    matches!(
        nt,
        NodeType::Normal
            | NodeType::NorthSouthPort
            | NodeType::ExternalPort
            | NodeType::BreakingPoint
    )
}

struct SplineSegmentStoreMarker;
struct SplineRouteStartMarker;
struct SplineEdgeChainMarker;

/// Per-graph `Vec<SplineSegment>` that owns every segment produced by the
/// router. Calculator reads `SPLINE_ROUTE_START` indices into this store.
pub static SPLINE_SEGMENT_STORE: LazyLock<PropertyKey<Vec<SplineSegment>>> =
    LazyLock::new(|| PropertyKey::of::<SplineSegmentStoreMarker>(Vec::new));

/// Per-edge list of segment indices describing the spline route for the edge
/// chain starting at this edge. Indices reference `SPLINE_SEGMENT_STORE`.
pub static SPLINE_ROUTE_START: LazyLock<PropertyKey<Vec<SegmentId>>> =
    LazyLock::new(|| PropertyKey::of::<SplineRouteStartMarker>(Vec::new));

/// Per-edge chain of `LEdge`s that form one long edge.
pub static SPLINE_EDGE_CHAIN: LazyLock<PropertyKey<Vec<EdgeId>>> =
    LazyLock::new(|| PropertyKey::of::<SplineEdgeChainMarker>(Vec::new));
