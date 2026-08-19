/// The direction in which the layout is computed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayoutDirection {
    #[default]
    Undefined,
    Right,
    Left,
    Down,
    Up,
}

/// Strategy for breaking cycles in the graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CycleBreakingStrategy {
    #[default]
    Greedy,
    DepthFirst,
    Interactive,
    ModelOrder,
    GreedyModelOrder,
    SccConnectivity,
    SccNodeType,
    DfsNodeOrder,
    BfsNodeOrder,
}

/// Strategy for assigning nodes to layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayeringStrategy {
    #[default]
    NetworkSimplex,
    LongestPath,
    LongestPathSource,
    CoffmanGraham,
    Interactive,
    StretchWidth,
    MinWidth,
    /// Breadth-first model order layering.
    BfModelOrder,
    /// Depth-first model order layering.
    DfModelOrder,
}

/// Strategy for crossing minimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CrossingMinimizationStrategy {
    #[default]
    BarycenterLayerSweep,
    MedianLayerSweep,
    Interactive,
    None,
}

/// Strategy for node placement within layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodePlacementStrategy {
    Simple,
    Interactive,
    LinearSegments,
    #[default]
    BrandesKoepf,
    NetworkSimplex,
}

/// Strategy for edge routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeRoutingStrategy {
    #[default]
    Orthogonal,
    Polyline,
    Splines,
}

/// Post-process graph compaction strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GraphCompactionStrategy {
    #[default]
    None,
    Left,
    Right,
    LeftRightConstraintLocking,
    LeftRightConnectionLocking,
    EdgeLength,
}

/// Constraint calculation strategy used by post-process compaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConstraintCalculationStrategy {
    Quadratic,
    #[default]
    Scanline,
}

/// Routing flavor used when `EdgeRoutingStrategy::Splines` is selected.
/// Default `Sloppy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SplineRoutingMode {
    /// Inserts extra control points that keep the edge straight near the
    /// endpoints before curving. More predictable but visually stiffer.
    Conservative,
    /// Takes shortcuts whenever the direct curve does not intersect a
    /// neighboring node.
    #[default]
    Sloppy,
}

/// Direction of flow at a port. Used by the compound graph
/// preprocessor/postprocessor to distinguish between inbound and outbound
/// segments of a cross-hierarchy edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum PortType {
    #[default]
    Undefined,
    Input,
    Output,
}

/// Constraints on port placement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortConstraints {
    #[default]
    Undefined,
    Free,
    FixedSide,
    FixedOrder,
    FixedRatio,
    FixedPos,
}

impl PortConstraints {
    /// Return the ordinal of this variant.
    pub fn ordinal(self) -> u8 {
        match self {
            PortConstraints::Undefined => 0,
            PortConstraints::Free => 1,
            PortConstraints::FixedSide => 2,
            PortConstraints::FixedOrder => 3,
            PortConstraints::FixedRatio => 4,
            PortConstraints::FixedPos => 5,
        }
    }

    /// Return true if this constraint is strictly weaker than `other`.
    pub fn is_weaker_than(self, other: PortConstraints) -> bool {
        self.ordinal() < other.ordinal()
    }

    /// Whether at least the side of the port is fixed.
    pub fn is_side_fixed(self) -> bool {
        matches!(
            self,
            PortConstraints::FixedSide
                | PortConstraints::FixedOrder
                | PortConstraints::FixedRatio
                | PortConstraints::FixedPos
        )
    }

    /// Whether the port order is fixed.
    pub fn is_order_fixed(self) -> bool {
        matches!(
            self,
            PortConstraints::FixedOrder | PortConstraints::FixedRatio | PortConstraints::FixedPos
        )
    }

    /// Whether the port ratio is fixed.
    pub fn is_ratio_fixed(self) -> bool {
        matches!(self, PortConstraints::FixedRatio)
    }

    /// Whether the port position is fixed.
    pub fn is_pos_fixed(self) -> bool {
        matches!(self, PortConstraints::FixedPos)
    }
}

/// Constraint on which layer a node should be placed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LayerConstraint {
    #[default]
    None,
    First,
    FirstSeparate,
    Last,
    LastSeparate,
}

/// Constraint on edge direction for a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeConstraint {
    #[default]
    None,
    IncomingOnly,
    OutgoingOnly,
}

/// Alignment strategy for the Brandes-Koepf node placement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FixedAlignment {
    #[default]
    None,
    Leftmost,
    Rightmost,
    Balanced,
    LeftUp,
    LeftDown,
    RightUp,
    RightDown,
}

/// Constraint for ordering nodes within a single layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InLayerConstraint {
    #[default]
    None,
    Top,
    Bottom,
}

/// Strategy for node promotion to reduce dummy nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodePromotionStrategy {
    #[default]
    None,
    NoBoundary,
    Nikolov,
    NikolovImproved,
    NikolovPixel,
    NikolovImprovedPixel,
    /// Stop promoting after moving at most `max_iterations * node_count / 100`
    /// nodes.
    NodecountPercentage,
    /// Stop promoting after reducing at most
    /// `max_iterations * dummy_count / 100` dummy nodes.
    DummynodePercentage,
    ModelOrderLeftToRight,
    ModelOrderRightToLeft,
}

/// Alignment for node placement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Alignment {
    #[default]
    Automatic,
    Left,
    Right,
    Center,
    Top,
    Bottom,
}

/// Strategy for ordering ports within a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OrderingStrategy {
    #[default]
    None,
    /// Consider both node order and edge connections.
    NodesAndEdges,
    /// The node ordering is only used as a secondary criterion; edge order is preserved.
    PreferEdges,
    /// Prefer node order over edge order.
    PreferNodes,
}

/// Strategy for layer unzipping to reduce dummy nodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayerUnzippingStrategy {
    #[default]
    None,
    Simple,
    NetworkFlow,
    Alternating,
}

/// Control how hierarchy is handled during layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HierarchyHandling {
    /// Inherit hierarchy handling from the parent graph.
    #[default]
    Inherit,
    /// Include all hierarchy levels in a single layout run.
    Include,
    /// Lay out each hierarchy level separately.
    Separate,
}

/// Placement strategy for edge labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeLabelPlacement {
    #[default]
    Undefined,
    /// Place the label at the center of the edge.
    Center,
    /// Place the label near the head of the edge.
    Head,
    /// Place the label near the tail of the edge.
    Tail,
}

/// Distribution strategy for self-loop edges around a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SelfLoopDistribution {
    /// Route all self-loops on the north side.
    #[default]
    North,
    /// Route all self-loops on the south side.
    South,
    /// Route self-loops on both north and south sides.
    NorthSouth,
    /// Distribute self-loops equally around the node.
    EquallyDistributed,
}

/// Ordering strategy for multiple self-loop edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SelfLoopOrdering {
    /// Stack self-loops on top of each other.
    #[default]
    Stacked,
    /// Place self-loops side by side.
    Sequenced,
    /// Reverse the order in which self-loops on the same side are stacked.
    ReverseStacked,
}

/// Strategy to sort long-edge dummy nodes against normal nodes with no
/// previous-layer connection. Default `DummyNodeOver`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LongEdgeOrderingStrategy {
    /// Dummy nodes sort over normal nodes (maps to `i32::MAX`).
    #[default]
    DummyNodeOver,
    /// Dummy nodes sort under normal nodes (maps to `i32::MIN`).
    DummyNodeUnder,
    /// Dummy nodes sort equal to normal nodes (maps to `0`).
    Equal,
}

impl LongEdgeOrderingStrategy {
    /// Sorting key value: `i32::MAX` (over), `i32::MIN` (under), `0` (equal).
    pub fn return_value(self) -> i32 {
        match self {
            Self::DummyNodeOver => i32::MAX,
            Self::DummyNodeUnder => i32::MIN,
            Self::Equal => 0,
        }
    }
}

/// Type of greedy switch heuristic for crossing minimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GreedySwitchType {
    /// One-sided greedy switch (check from one direction only).
    OneSided,
    /// Two-sided greedy switch (check from both directions).
    #[default]
    TwoSided,
    /// Disable greedy switch entirely.
    Off,
}

/// Strategy for straightening edges during node placement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeStraighteningStrategy {
    /// Do not straighten edges.
    None,
    /// Improve straightness of long edges.
    #[default]
    ImproveStraightness,
}

/// Control how layout direction maps to edge direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DirectionCongruency {
    /// Use reading direction for edge orientation.
    #[default]
    ReadingDirection,
    /// Rotate edges to match the layout direction.
    Rotation,
}

/// How ports are distributed along a given node side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortAlignment {
    /// Leave the decision to the layout algorithm.
    #[default]
    Undefined,
    /// Distribute the ports evenly across the side.
    Distributed,
    /// Justify port positions so the outermost ports touch the node border.
    Justified,
    /// Pack ports at the start of the side.
    Begin,
    /// Center the block of ports on the side.
    Center,
    /// Pack ports at the end of the side.
    End,
}

/// Strategy for placing center edge labels within layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CenterEdgeLabelPlacementStrategy {
    /// Place labels in the layer closest to the physical edge center (width-weighted).
    CenterLayer,
    /// Place labels in the median layer of the edge's dummy chain.
    #[default]
    MedianLayer,
    /// Place labels in the widest layer of the edge's dummy chain.
    WidestLayer,
    /// Place labels in the layer closest to the head (target).
    HeadLayer,
    /// Place labels in the layer closest to the tail (source).
    TailLayer,
    /// Space-efficient heuristic: assigns labels to layers so overall width is minimized.
    SpaceEfficientLayer,
}

impl CenterEdgeLabelPlacementStrategy {
    /// Returns `true` if the strategy depends on per-layer or per-label width
    /// information (in which case layer widths must be computed up-front).
    pub fn uses_label_size_information(self) -> bool {
        matches!(
            self,
            CenterEdgeLabelPlacementStrategy::CenterLayer
                | CenterEdgeLabelPlacementStrategy::WidestLayer
                | CenterEdgeLabelPlacementStrategy::SpaceEfficientLayer
        )
    }
}

/// Strategy for sorting ports within a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortSortingStrategy {
    /// Do not sort ports.
    #[default]
    None,
    /// Sort ports by their input order.
    InputOrder,
    /// Sort ports by their assigned side.
    PortSide,
    /// Sort ports by their degree (number of connected edges).
    PortDegree,
}

/// Flexibility of a node during `NetworkSimplexPlacer` node placement.
///
/// Four levels: `None`, `PortPosition`, `NodeSizeWhereSpacePermits`, `NodeSize`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodeFlexibility {
    /// Node size must not be altered, ports remain where they were placed
    /// prior to node placement.
    #[default]
    None,
    /// Ports are allowed to move on the node's border but the node's size is
    /// fixed.
    PortPosition,
    /// Ports may move and the node may enlarge, but only where unused
    /// auxiliary-graph slack permits it.
    NodeSizeWhereSpacePermits,
    /// The node's size may change freely, implying that ports may also be
    /// repositioned.
    NodeSize,
}

impl NodeFlexibility {
    /// Returns `true` if the flexibility level implies the node's size can
    /// grow freely.
    pub fn is_flexible_size(self) -> bool {
        matches!(self, NodeFlexibility::NodeSize)
    }

    /// Returns `true` if the flexibility level is at least
    /// `NodeSizeWhereSpacePermits`.
    pub fn is_flexible_size_where_space_permits(self) -> bool {
        matches!(self, NodeFlexibility::NodeSizeWhereSpacePermits | NodeFlexibility::NodeSize)
    }

    /// Returns `true` if the flexibility level lets ports move. True for all
    /// non-`None` variants.
    pub fn is_flexible_ports(self) -> bool {
        !matches!(self, NodeFlexibility::None)
    }
}

/// Which side of an edge a label is placed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LabelSide {
    /// Side not determined yet.
    #[default]
    Unknown,
    /// Label placed above the edge.
    Above,
    /// Label placed below the edge.
    Below,
    /// Label placed on top of the edge (inline).
    Inline,
}

impl LabelSide {
    /// Returns the opposite side. `Inline` and `Unknown` are their own opposites.
    pub fn opposite(self) -> Self {
        match self {
            LabelSide::Above => LabelSide::Below,
            LabelSide::Below => LabelSide::Above,
            other => other,
        }
    }
}

/// Strategy for selecting which side of an edge its labels are placed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeLabelSideSelection {
    /// Place all labels above their respective edge.
    AlwaysUp,
    /// Place all labels below their respective edge.
    AlwaysDown,
    /// Place labels based on edge direction; rightward edges get the primary side.
    DirectionUp,
    /// Place labels based on edge direction; rightward edges get the primary side (below).
    DirectionDown,
    /// Smart placement that falls back to `Above`.
    SmartUp,
    /// Smart placement that falls back to `Below`.
    #[default]
    SmartDown,
}

impl EdgeLabelSideSelection {
    /// Swap every up/down pair. Used by `GraphTransformer` when rotating
    /// the graph: vertical layout directions need labels on the opposite side.
    pub fn transpose(self) -> Self {
        match self {
            Self::AlwaysUp => Self::AlwaysDown,
            Self::AlwaysDown => Self::AlwaysUp,
            Self::DirectionUp => Self::DirectionDown,
            Self::DirectionDown => Self::DirectionUp,
            Self::SmartUp => Self::SmartDown,
            Self::SmartDown => Self::SmartUp,
        }
    }
}

use bitflags::bitflags;

bitflags! {
    /// Placement of node labels relative to the node.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct NodeLabelPlacement: u16 {
        const H_LEFT     = 0x01;
        const H_CENTER   = 0x02;
        const H_RIGHT    = 0x04;
        const V_TOP      = 0x08;
        const V_CENTER   = 0x10;
        const V_BOTTOM   = 0x20;
        const INSIDE     = 0x40;
        const OUTSIDE    = 0x80;
        /// When set on an outside placement, prefers WEST/EAST sides over
        /// NORTH/SOUTH; on inside placements it does not change the slot.
        const H_PRIORITY = 0x100;
    }
}

bitflags! {
    /// Constraints that determine how a node's size is computed.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct SizeConstraint: u8 {
        const NODE_LABELS  = 0x01;
        const PORTS         = 0x02;
        const MINIMUM_SIZE = 0x04;
        const PORT_LABELS  = 0x08;
    }
}

bitflags! {
    /// Modifiers applied to size-constraint interpretation.
    ///
    /// Each bit flips one aspect of how the size computation treats minimums,
    /// padding, labels, and port spacing.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct SizeOptions: u16 {
        const DEFAULT_MINIMUM_SIZE              = 0x001;
        const MINIMUM_SIZE_ACCOUNTS_FOR_PADDING = 0x002;
        const COMPUTE_PADDING                   = 0x004;
        const OUTSIDE_NODE_LABELS_OVERHANG      = 0x008;
        const PORTS_OVERHANG                    = 0x010;
        const UNIFORM_PORT_SPACING              = 0x020;
        const SPACE_EFFICIENT_PORT_LABELS       = 0x040;
        const FORCE_TABULAR_NODE_LABELS         = 0x080;
        const ASYMMETRICAL                      = 0x100;
    }
}

bitflags! {
    /// Alignment of content within a node.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct ContentAlignment: u8 {
        const H_LEFT   = 0x01;
        const H_CENTER = 0x02;
        const H_RIGHT  = 0x04;
        const V_TOP    = 0x08;
        const V_CENTER = 0x10;
        const V_BOTTOM = 0x20;
    }
}

bitflags! {
    /// Placement strategy for port labels.
    ///
    /// `INSIDE` and `OUTSIDE` are mutually exclusive in valid sets, but the
    /// shape allows both to be absent (= "fixed", labels are not placed but
    /// left untouched). Other bits modify alignment.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct PortLabelPlacement: u8 {
        /// Port labels are placed outside the node.
        const OUTSIDE                  = 0x01;
        /// Port labels are placed inside the node.
        const INSIDE                   = 0x02;
        /// Place the label next to (center-aligned with) its port when possible.
        const NEXT_TO_PORT_IF_POSSIBLE = 0x04;
        /// Place all port labels on the same side: below (W/E) or right (N/S).
        const ALWAYS_SAME_SIDE         = 0x08;
        /// Place all port labels on the same side: above (W/E) or left (N/S).
        const ALWAYS_OTHER_SAME_SIDE   = 0x10;
        /// Allow alternating sides for outside labels to keep node sizes small.
        const SPACE_EFFICIENT          = 0x20;
    }
}

impl PortLabelPlacement {
    /// Empty set means fixed: label positions are not computed but left untouched.
    pub fn fixed() -> Self {
        Self::empty()
    }

    /// Default inside placement: a single-bit set containing `INSIDE`.
    pub fn inside_default() -> Self {
        Self::INSIDE
    }

    /// Default outside placement: a single-bit set containing `OUTSIDE`.
    pub fn outside_default() -> Self {
        Self::OUTSIDE
    }

    /// Whether neither `INSIDE` nor `OUTSIDE` is set (label positions are
    /// left untouched).
    pub fn is_fixed(self) -> bool {
        !self.contains(Self::INSIDE) && !self.contains(Self::OUTSIDE)
    }
}

/// How group model order relates to the plain model order during P1 cycle
/// breaking and P3 crossing minimization. Default `OnlyWithinGroup`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GroupOrderStrategy {
    /// Different groups are not comparable. Ordering only applies within a group.
    #[default]
    OnlyWithinGroup,
    /// Plain model order is primary, group id is secondary.
    ModelOrder,
    /// Group id is primary, model order is secondary.
    Enforced,
}

/// Strategy for preserving model order when connected components are split
/// and later packed back into one graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ComponentOrderingStrategy {
    /// Components are ordered by priority, then by component area.
    #[default]
    None,
    /// Components stay ordered inside their external-port side groups.
    InsidePortSideGroups,
    /// Component groups are ordered by their minimal node model order.
    GroupModelOrder,
    /// Components are placed in rows according to their minimal node model order.
    ModelOrder,
}

/// Reference point used by `InteractiveCycleBreaker` (and other interactive
/// phases) when comparing node positions to decide edge directions.
/// Default `Center`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InteractiveReferencePoint {
    /// Compare nodes by the center of their bounding box: `(position + size / 2)`.
    #[default]
    Center,
    /// Compare nodes by their top-left corner: `position` only.
    TopLeft,
}

/// Strategy for picking cut indexes when wrapping wide-and-narrow layerings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WrappingCuttingStrategy {
    /// Pick cuts by minimizing the maximum scale.
    #[default]
    Msd,
    /// Pick cuts from the desired aspect ratio.
    Ard,
    /// Read explicit cut indexes from the graph property
    /// `WRAPPING_CUTTING_CUTS`.
    Manual,
}

/// Strategy used to turn a raw cut-index list into a list of valid cut
/// indexes. Variants other than `No` only apply when the cut calculator does
/// not guarantee validity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WrappingValidifyStrategy {
    /// Do not modify the raw cut indexes.
    #[default]
    No,
    /// For each forbidden cut, walk back to the nearest valid index.
    LookBack,
    /// For each forbidden cut, walk forward to the next valid index.
    Greedy,
}

/// Top-level switch that enables wrapping.
///
/// - `Off` (default): no wrapping processors are added.
/// - `SingleEdge`: run `SingleEdgeGraphWrapper` before P4. Path-like graphs
///   only.
/// - `MultiEdge`: run the breaking-point triumvirate
///   (`BreakingPointInserter` pre-P3, `BreakingPointProcessor` pre-P4,
///   `BreakingPointRemover` post-P5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WrappingStrategy {
    /// No wrapping.
    #[default]
    Off,
    /// Path-like single-edge wrapping via `SingleEdgeGraphWrapper`.
    SingleEdge,
    /// General multi-edge wrapping via the breaking-point processors.
    MultiEdge,
}
