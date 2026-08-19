use super::PropertyKey;
use crate::{
    graph::index::{EdgeId, NodeId, PortId},
    options::enums::{SelfLoopDistribution, SelfLoopOrdering},
};

struct OriginEdgeMarker;

/// Origin edge that a dummy node / edge was created for.
pub static ORIGIN_EDGE: std::sync::LazyLock<PropertyKey<Option<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginEdgeMarker>(|| None));

struct CrossingMinPositionIdMarker;

/// Per-node position id within its layer, written by `ConstraintsPostprocessor`
/// after P5.
pub static CROSSING_MINIMIZATION_POSITION_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CrossingMinPositionIdMarker>(|| 0));

/// Marker types for internal property keys.
struct LongEdgeSourceMarker;
struct LongEdgeTargetMarker;
struct CyclicMarker;
struct RandomSeedMarker;
struct SelfLoopEdgesMarker;
struct LayerConstraintMarker;
struct OriginalOppositePortMarker;
struct LabelDummyEdgeMarker;
struct InLayerConstraintMarker;
struct InLayerLayoutUnitMarker;
struct P3InitialLayerOrderMarker;
struct InLayerSuccessorConstraintsMarker;
struct InLayerSuccessorConstraintsBetweenNonDummiesMarker;
struct OriginNodeMarker;
struct OriginPortMarker;
struct PartitionDummyMarker;
struct GreedySwitchActivateMarker;
struct BarycenterAssociatesMarker;
struct CrossingHintMarker;
struct PriorityMarker;
struct PriorityStraightnessMarker;
struct PriorityShortnessMarker;
struct ExtPortSideMarker;
struct ExtPortReplacedDummyMarker;
struct ExtPortReplacedDummiesMarker;
struct PortRatioOrPositionMarker;
struct EndLabelsMarker;
struct NodePortConstraintsMarker;
struct ModelOrderMarker;
struct PortIndexMarker;
struct PreserveIdsForEdgeWritebackMarker;

/// The original source port for a long edge dummy node.
pub static LONG_EDGE_SOURCE: std::sync::LazyLock<PropertyKey<Option<PortId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LongEdgeSourceMarker>(|| None));

/// The original target port for a long edge dummy node.
pub static LONG_EDGE_TARGET: std::sync::LazyLock<PropertyKey<Option<PortId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LongEdgeTargetMarker>(|| None));

/// Whether the graph contains cycles (set by cycle breaking phase).
pub static CYCLIC: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CyclicMarker>(|| false));

/// The random seed used for layout decisions.
pub static RANDOM_SEED: std::sync::LazyLock<PropertyKey<u64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<RandomSeedMarker>(|| 1));

/// Self-loop edge IDs hidden during preprocessing, to be restored in postprocessing.
pub static SELF_LOOP_EDGES: std::sync::LazyLock<PropertyKey<Vec<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SelfLoopEdgesMarker>(Vec::new));

/// Per-node layer constraint (FIRST, LAST, FIRST_SEPARATE, LAST_SEPARATE).
pub static LAYER_CONSTRAINT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::LayerConstraint>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<LayerConstraintMarker>(|| crate::options::enums::LayerConstraint::None)
});

/// The original opposite port of a disconnected edge (used during layer constraint processing).
pub static ORIGINAL_OPPOSITE_PORT: std::sync::LazyLock<PropertyKey<Option<PortId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginalOppositePortMarker>(|| None));

/// The original edge that a label dummy node was created for.
pub static LABEL_DUMMY_EDGE: std::sync::LazyLock<PropertyKey<Option<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LabelDummyEdgeMarker>(|| None));

/// In-layer constraint for a node (TOP, BOTTOM, or NONE).
pub static IN_LAYER_CONSTRAINT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::InLayerConstraint>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<InLayerConstraintMarker>(|| crate::options::enums::InLayerConstraint::None)
});

/// The layout unit a node belongs to within its layer (used for N/S port dummies).
pub static IN_LAYER_LAYOUT_UNIT: std::sync::LazyLock<PropertyKey<Option<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InLayerLayoutUnitMarker>(|| None));

/// Stable layer position captured before P3 starts reordering nodes.
pub static P3_INITIAL_LAYER_ORDER: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<P3InitialLayerOrderMarker>(|| i32::MAX));

/// In-layer successor constraints: nodes that must appear after this node in the same layer.
pub static IN_LAYER_SUCCESSOR_CONSTRAINTS: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InLayerSuccessorConstraintsMarker>(Vec::new));

/// True when in-layer successor constraints have been added between non-dummy
/// (regular) nodes. Set by `SemiInteractiveCrossMinProcessor` and read by
/// `ForsterConstraintResolver` to enable a two-stage constraint resolution.
pub static IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES: std::sync::LazyLock<
    PropertyKey<bool>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<InLayerSuccessorConstraintsBetweenNonDummiesMarker>(|| false)
});

/// The origin node that a dummy was created for.
pub static ORIGIN_NODE: std::sync::LazyLock<PropertyKey<Option<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginNodeMarker>(|| None));

/// The origin port that a dummy port was created for.
pub static ORIGIN_PORT: std::sync::LazyLock<PropertyKey<Option<PortId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginPortMarker>(|| None));

/// Marks a port or edge as a partition dummy created by `PartitionMidprocessor`
/// to enforce partition ordering during layering.
pub static PARTITION_DUMMY: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PartitionDummyMarker>(|| false));

/// Cached greedy-switch activation decision computed once on the
/// pre-split parent graph, then inherited by every component via
/// `extract_component_graphs`'s property clone.
///
/// The decision is made on the parent before `ComponentsProcessor.split`.
/// Without caching, the decision would be re-made per component using the
/// component's own `layerless_nodes.len()`, which for a small component of a
/// large parent (size > 40 default threshold) would flip the activation from
/// off to on.
pub static GREEDY_SWITCH_ACTIVATE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<GreedySwitchActivateMarker>(|| false));

/// Barycenter associates: dummy nodes associated with a normal node for N/S port processing.
pub static BARYCENTER_ASSOCIATES: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<BarycenterAssociatesMarker>(Vec::new));

/// Crossing hint used during cross counting (1 or 2 for N/S port dummies).
pub static CROSSING_HINT: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CrossingHintMarker>(|| 0));

/// Per-node layout priority. Summed across a component to rank it during
/// `SimpleRowGraphPlacer` placement (higher priority = placed first). Default 0.
pub static PRIORITY: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PriorityMarker>(|| 0));

/// Per-edge straightness priority used by Brandes-Köpf neighborhood selection.
/// Higher values make the edge preferred as the aligning neighbor. Default 0.
pub static PRIORITY_STRAIGHTNESS: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PriorityStraightnessMarker>(|| 0));

/// Per-edge shortness priority used as the edge weight in network simplex
/// layering. Effective weight is `max(1, priority)`. Default 0.
pub static PRIORITY_SHORTNESS: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PriorityShortnessMarker>(|| 0));

/// The external port side for hierarchical port dummies.
pub static EXT_PORT_SIDE: std::sync::LazyLock<PropertyKey<crate::graph::port::PortSide>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<ExtPortSideMarker>(|| crate::graph::port::PortSide::Undefined)
    });

/// The original external port dummy that was replaced.
pub static EXT_PORT_REPLACED_DUMMY: std::sync::LazyLock<PropertyKey<Option<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<ExtPortReplacedDummyMarker>(|| None));

/// All original external port dummies that were replaced (stored on graph).
pub static EXT_PORT_REPLACED_DUMMIES: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<ExtPortReplacedDummiesMarker>(Vec::new));

/// Position or ratio value for external port dummies.
pub static PORT_RATIO_OR_POSITION: std::sync::LazyLock<PropertyKey<f64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PortRatioOrPositionMarker>(|| 0.0));

/// Whether a node has end labels associated with it.
pub static END_LABELS: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<EndLabelsMarker>(|| false));

/// Per-node port constraints override (separate from the graph-level option).
pub static NODE_PORT_CONSTRAINTS: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortConstraints>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<NodePortConstraintsMarker>(|| {
        crate::options::enums::PortConstraints::Undefined
    })
});

struct GraphPropertiesMarker;
struct CommentBoxMarker;
struct NodeLabelPlacementMarker;
struct NodeSizeConstraintsMarker;
struct NodeSizeMinimumMarker;
struct NodeSizeFixedGraphSizeMarker;
struct PortLabelPlacementMarker;
struct EdgeLabelPlacementMarker;
struct CenterLabelPlacementStrategyMarker;
struct ContentAlignmentMarker;
struct NodeFlexibilityMarker;
struct HierarchyHandlingMarker;
struct InsideSelfLoopsActivateMarker;
struct InsideSelfLoopsYoMarker;
struct SeparateCcMarker;
struct MergeEdgesMarker;
struct MergeHierarchyEdgesMarker;
struct PositionChoiceConstraintMarker;
struct PositionMarker;
struct LayerChoiceConstraintMarker;
struct NodeLabelsPaddingMarker;

/// Graph structural properties bitflags.
pub static GRAPH_PROPERTIES: std::sync::LazyLock<
    PropertyKey<crate::properties::graph_properties::GraphProperties>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<GraphPropertiesMarker>(|| {
        crate::properties::graph_properties::GraphProperties::empty()
    })
});

/// Whether a node is a comment box.
pub static COMMENT_BOX: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CommentBoxMarker>(|| false));

/// Node label placement configuration (bitflags).
pub static NODE_LABEL_PLACEMENT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::NodeLabelPlacement>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<NodeLabelPlacementMarker>(|| {
        crate::options::enums::NodeLabelPlacement::empty()
    })
});

/// Constraints on how a node's size is computed (bitflags).
pub static NODE_SIZE_CONSTRAINTS: std::sync::LazyLock<
    PropertyKey<crate::options::enums::SizeConstraint>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<NodeSizeConstraintsMarker>(crate::options::enums::SizeConstraint::empty)
});

/// Minimum size for a node.
pub static NODE_SIZE_MINIMUM: std::sync::LazyLock<PropertyKey<crate::math::Vec2>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<NodeSizeMinimumMarker>(|| crate::math::Vec2::ZERO)
    });

/// Whether a parent node keeps its declared graph size instead of being
/// resized to the actual nested graph size.
pub static NODE_SIZE_FIXED_GRAPH_SIZE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<NodeSizeFixedGraphSizeMarker>(|| false));

/// Placement of port labels relative to the port. Default `EnumSet.of(OUTSIDE)`.
pub static PORT_LABEL_PLACEMENT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortLabelPlacement>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortLabelPlacementMarker>(|| {
        crate::options::enums::PortLabelPlacement::OUTSIDE
    })
});

/// Placement of edge labels relative to the edge. Default `Center` so that
/// `LabelDummyInserter`, `cache_graph_properties`, and the label-side selector
/// classify labels without an explicit placement as center labels.
pub static EDGE_LABEL_PLACEMENT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::EdgeLabelPlacement>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<EdgeLabelPlacementMarker>(|| {
        crate::options::enums::EdgeLabelPlacement::Center
    })
});

/// Per-label override for center edge label layer selection.
pub static CENTER_LABEL_PLACEMENT_STRATEGY: std::sync::LazyLock<
    PropertyKey<Option<crate::options::enums::CenterEdgeLabelPlacementStrategy>>,
> = std::sync::LazyLock::new(|| PropertyKey::of::<CenterLabelPlacementStrategyMarker>(|| None));

/// Content alignment within a node (bitflags).
pub static CONTENT_ALIGNMENT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::ContentAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<ContentAlignmentMarker>(crate::options::enums::ContentAlignment::empty)
});

/// Flexibility of a node during node placement.
pub static NODE_FLEXIBILITY: std::sync::LazyLock<
    PropertyKey<crate::options::enums::NodeFlexibility>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<NodeFlexibilityMarker>(|| crate::options::enums::NodeFlexibility::None)
});

/// How hierarchy is handled during layout.
pub static HIERARCHY_HANDLING: std::sync::LazyLock<
    PropertyKey<crate::options::enums::HierarchyHandling>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<HierarchyHandlingMarker>(|| crate::options::enums::HierarchyHandling::Inherit)
});

/// Whether inside self-loops are activated for a node.
pub static INSIDE_SELF_LOOPS_ACTIVATE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InsideSelfLoopsActivateMarker>(|| false));

/// Whether an edge should be routed as an inside self-loop.
pub static INSIDE_SELF_LOOPS_YO: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InsideSelfLoopsYoMarker>(|| false));

/// Runtime guard for graph-local ids that are captured before layout and
/// written back afterwards.
pub static PRESERVE_IDS_FOR_EDGE_WRITEBACK: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PreserveIdsForEdgeWritebackMarker>(|| false));

/// Whether to process connected components separately.
pub static SEPARATE_CC: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SeparateCcMarker>(|| true));

/// Whether to merge multi-edges between the same pair of nodes.
pub static MERGE_EDGES: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<MergeEdgesMarker>(|| false));

/// Whether to merge edges that cross hierarchy boundaries.
pub static MERGE_HIERARCHY_EDGES: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<MergeHierarchyEdgesMarker>(|| true));

/// Constraint on which position a node should be placed at within its layer.
pub static POSITION_CHOICE_CONSTRAINT: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PositionChoiceConstraintMarker>(|| 0));

/// User-supplied interactive position for a node (vector).
///
/// Stored separately from the mutable `position` field on `NodeData` so
/// processors that need the original interactive coordinate
/// (`SemiInteractiveCrossMinProcessor`, `GraphTransformer.transposeProperties`,
/// etc.) can read it without confusing it with the layout result.
pub static POSITION: std::sync::LazyLock<PropertyKey<Option<crate::math::Vec2>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PositionMarker>(|| None));

/// Constraint on which layer a node should be placed in (integer choice).
pub static LAYER_CHOICE_CONSTRAINT: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LayerChoiceConstraintMarker>(|| 0));

/// Padding around node labels (top, right, bottom, left). Default `5, 5, 5, 5`.
pub static NODE_LABELS_PADDING: std::sync::LazyLock<PropertyKey<crate::math::Padding>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<NodeLabelsPaddingMarker>(|| crate::math::Padding::uniform(5.0))
    });

/// Model order index assigned to real nodes for model-order-aware crossing minimization.
pub static MODEL_ORDER: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<ModelOrderMarker>(|| -1));

struct ConsiderModelOrderStrategyMarker;
struct ConsiderModelOrderPortModelOrderMarker;
struct LayerUnzippingLayerSplitMarker;
struct LayerUnzippingResetOnLongEdgesMarker;
struct LayerUnzippingMinimizeEdgeLengthMarker;
struct AllowNonFlowPortsToSwitchSidesMarker;
struct PortBorderOffsetMarker;
struct LabelSideMarker;
struct PartitioningPartitionMarker;
struct PartitioningActivateMarker;
struct EndLabelCellsMarker;
struct RepresentedLabelsMarker;
struct P3IgnoreNestedGraphsMarker;

/// Whether to consider model order during crossing minimization.
pub static CONSIDER_MODEL_ORDER_STRATEGY: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<ConsiderModelOrderStrategyMarker>(|| false));

/// Whether to preserve port model order in sorting.
pub static CONSIDER_MODEL_ORDER_PORT_MODEL_ORDER: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<ConsiderModelOrderPortModelOrderMarker>(|| false)
    });

/// Number of sub-layers for alternating layer unzipping.
pub static LAYER_UNZIPPING_LAYER_SPLIT: std::sync::LazyLock<PropertyKey<usize>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LayerUnzippingLayerSplitMarker>(|| 1));

/// Whether to reset unzipping at long edge boundaries.
pub static LAYER_UNZIPPING_RESET_ON_LONG_EDGES: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LayerUnzippingResetOnLongEdgesMarker>(|| false));

/// Skip splitting a layer when its width/height ratio crosses the heuristic
/// threshold.
pub static LAYER_UNZIPPING_MINIMIZE_EDGE_LENGTH: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<LayerUnzippingMinimizeEdgeLengthMarker>(|| false)
    });

/// Whether a non-flow port is allowed to switch sides.
pub static ALLOW_NON_FLOW_PORTS_TO_SWITCH_SIDES: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<AllowNonFlowPortsToSwitchSidesMarker>(|| false));

/// Index of a port within its owning node, used by PortListSorter when
/// port constraints are FIXED_ORDER.
pub static PORT_INDEX: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PortIndexMarker>(|| 0));

/// Offset of a port from the node border.
pub static PORT_BORDER_OFFSET: std::sync::LazyLock<PropertyKey<f64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PortBorderOffsetMarker>(|| 0.0));

/// Which side of the edge a label is placed on.
pub static LABEL_SIDE: std::sync::LazyLock<PropertyKey<crate::options::enums::LabelSide>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<LabelSideMarker>(|| crate::options::enums::LabelSide::Unknown)
    });

/// Partition index for a node.
pub static PARTITIONING_PARTITION: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PartitioningPartitionMarker>(|| 0));

/// Whether partitioning is active for the graph.
pub static PARTITIONING_ACTIVATE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PartitioningActivateMarker>(|| false));

/// Grouped label cells at a node's end (for EndLabelSorter).
pub static END_LABEL_CELLS: std::sync::LazyLock<PropertyKey<Vec<crate::graph::label::LabelCell>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<EndLabelCellsMarker>(Vec::new));

/// Labels represented by a label dummy node.
pub static REPRESENTED_LABELS: std::sync::LazyLock<PropertyKey<Vec<crate::graph::index::LabelId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<RepresentedLabelsMarker>(Vec::new));

struct ExtPortConnectionsMarker;

/// The set of external port sides a connected component connects to.
pub static EXT_PORT_CONNECTIONS: std::sync::LazyLock<PropertyKey<crate::graph::port::PortSideSet>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<ExtPortConnectionsMarker>(crate::graph::port::PortSideSet::empty)
    });

/// True when P3 should ignore retained Rust nested-graph pointers. The
/// `SEPARATE_CHILDREN` path imports the current graph flat, so P3 only sees a
/// node's nested graph on the `INCLUDE_CHILDREN` importer path.
pub static P3_IGNORE_NESTED_GRAPHS: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<P3IgnoreNestedGraphsMarker>(|| false));

struct FirstTryWithInitialOrderMarker;
struct HiddenNodesMarker;
struct InputCollectMarker;
struct OutputCollectMarker;
struct OriginalPortConstraintsMarker;

/// First sweep attempt of the layer sweep crossing minimizer uses the initial
/// node order rather than a random permutation.
pub static FIRST_TRY_WITH_INITIAL_ORDER: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<FirstTryWithInitialOrderMarker>(|| false));

/// Second sweep attempt of the layer sweep crossing minimizer uses the initial
/// node order rather than a random permutation.
///
/// Both `FIRST_TRY_WITH_INITIAL_ORDER` and `SECOND_TRY_WITH_INITIAL_ORDER` use
/// the same backing property name (`"firstTryWithInitialOrder"`), so the two
/// flags alias the same backing value. This aliasing is intentional because
/// the P3 outer loop observes it and consumes a different random sequence
/// otherwise.
pub static SECOND_TRY_WITH_INITIAL_ORDER: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<FirstTryWithInitialOrderMarker>(|| false));

/// Nodes hidden by `LayerConstraintPreprocessor` for later restoration by
/// `LayerConstraintPostprocessor`. Stored on the graph.
pub static HIDDEN_NODES: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<HiddenNodesMarker>(Vec::new));

/// Port flag: marks an input-collector port. Used by `LEdge.reverse(adaptPorts)`
/// to reroute through collector ports.
pub static INPUT_COLLECT: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InputCollectMarker>(|| false));

/// Port flag: marks an output-collector port. Used by `LEdge.reverse(adaptPorts)`
/// to reroute through collector ports.
pub static OUTPUT_COLLECT: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OutputCollectMarker>(|| false));

/// Original port constraints of a node, saved by pipeline processors before
/// being overridden. Restored by `self_loop_port_restorer::restore` and
/// `port_side_processor::assign_sides`.
pub static ORIGINAL_PORT_CONSTRAINTS: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortConstraints>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<OriginalPortConstraintsMarker>(|| {
        crate::options::enums::PortConstraints::Undefined
    })
});

struct PriorityDirectionMarker;

/// Edge priority hint used by P1 cycle breaking to weight edge importance.
/// Higher-priority edges contribute more to indeg/outdeg counts, making them
/// less likely to be reversed.
pub static PRIORITY_DIRECTION: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PriorityDirectionMarker>(|| 0));

struct MaxModelOrderNodesMarker;
struct CbNumModelOrderGroupsMarker;
struct LayeringLayerIdMarker;
struct CbGroupOrderStrategyMarker;
struct CbCycleBreakingIdMarker;
struct CbPreferredSourceIdMarker;
struct CbPreferredTargetIdMarker;

/// Graph-level upper bound on `MODEL_ORDER` values. Used by group-model-order
/// cycle breakers to scale the group id into the effective order:
/// `MAX_MODEL_ORDER_NODES * groupId + modelOrder`.
pub static MAX_MODEL_ORDER_NODES: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<MaxModelOrderNodesMarker>(|| 0));

/// Graph-level count of distinct cycle-breaking model-order groups. Used by SCC
/// cycle breakers to scale the per-group offset into a unique total order.
pub static CB_NUM_MODEL_ORDER_GROUPS: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CbNumModelOrderGroupsMarker>(|| 1));

/// Per-node layer id assigned by cycle breakers that shift nodes whose edge
/// was reversed into the next layer.
pub static LAYERING_LAYER_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LayeringLayerIdMarker>(|| 0));

/// Graph-level cycle-breaking group-order strategy. When set to `Enforced`,
/// group id becomes the primary sort key and model order the secondary
/// criterion.
pub static CB_GROUP_ORDER_STRATEGY: std::sync::LazyLock<
    PropertyKey<crate::options::enums::GroupOrderStrategy>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<CbGroupOrderStrategyMarker>(|| {
        crate::options::enums::GroupOrderStrategy::OnlyWithinGroup
    })
});

/// Per-node cycle-breaking group id.
pub static CB_CYCLE_BREAKING_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CbCycleBreakingIdMarker>(|| 0));

/// Graph-level preferred source group id for `SCC_NODE_TYPE`.
pub static CB_PREFERRED_SOURCE_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CbPreferredSourceIdMarker>(|| -1));

/// Graph-level preferred target group id for `SCC_NODE_TYPE`.
pub static CB_PREFERRED_TARGET_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CbPreferredTargetIdMarker>(|| -1));

struct CmGroupOrderStrategyMarker;
struct CmEnforcedGroupOrdersMarker;
struct CmCrossingMinimizationIdMarker;

/// Graph-level crossing-minimization group-order strategy. When set to
/// `Enforced`, the per-element `CROSSING_MINIMIZATION_ID` becomes the primary
/// sort key (scaled by `MAX_MODEL_ORDER_NODES`) and model order the secondary
/// criterion.
pub static CM_GROUP_ORDER_STRATEGY: std::sync::LazyLock<
    PropertyKey<crate::options::enums::GroupOrderStrategy>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<CmGroupOrderStrategyMarker>(|| {
        crate::options::enums::GroupOrderStrategy::OnlyWithinGroup
    })
});

/// Graph-level whitelist of group ids whose ordering must be enforced. Only
/// groups whose `CROSSING_MINIMIZATION_ID` appears in this list have their
/// order multiplied by the model-order offset.
pub static CM_ENFORCED_GROUP_ORDERS: std::sync::LazyLock<PropertyKey<Vec<i32>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CmEnforcedGroupOrdersMarker>(Vec::new));

/// Per-element crossing-minimization group id.
pub static CROSSING_MINIMIZATION_ID: std::sync::LazyLock<PropertyKey<i32>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CmCrossingMinimizationIdMarker>(|| 0));

struct AlignmentMarker;

/// Per-node alignment.
pub static ALIGNMENT: std::sync::LazyLock<PropertyKey<crate::options::enums::Alignment>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<AlignmentMarker>(|| crate::options::enums::Alignment::Automatic)
    });

struct EndLabelEdgeMarker;

/// Back-reference placed on a label that was moved from an original edge
/// onto a dummy edge. Used when restoring edges to know which edge the
/// label belonged to before a long-edge or inverted-port split.
pub static END_LABEL_EDGE: std::sync::LazyLock<PropertyKey<Option<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<EndLabelEdgeMarker>(|| None));

struct NodeEdgeConstraintMarker;

/// Per-node edge constraint set by `EdgeAndLayerConstraintEdgeReverser` when
/// a FIRST/LAST layer constraint forces the node into an outgoing-only or
/// incoming-only role.
pub static NODE_EDGE_CONSTRAINT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::EdgeConstraint>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<NodeEdgeConstraintMarker>(|| crate::options::enums::EdgeConstraint::None)
});

struct WeightMarker;

/// Per-node weight used by `MedianHeuristic` to sort nodes in a layer by the
/// median of connected-layer weights. Also carries the integer 1..n position
/// after `setFirstLayerOrder`.
pub static WEIGHT: std::sync::LazyLock<PropertyKey<f64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<WeightMarker>(|| f64::NAN));

// The `CROSS_HIERARCHY_MAP` key lives alongside its value type in
// `core/src/intermediate/compound/` because the value struct is algorithm-specific
// and should stay out of the stable public API.

struct InsideConnectionsMarker;
struct OriginalLabelEdgeMarker;
struct JunctionPointsMarker;
struct UnnecessaryBendpointsMarker;

/// Marker on external ports indicating that their associated dummy contains
/// inbound connections.
pub static INSIDE_CONNECTIONS: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<InsideConnectionsMarker>(|| false));

/// Back-reference on a label moved from an original edge to a dummy segment.
/// Stores the original edge so the postprocessor can restore the label when
/// reassembling the hierarchical edge.
pub static ORIGINAL_LABEL_EDGE: std::sync::LazyLock<PropertyKey<Option<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginalLabelEdgeMarker>(|| None));

/// Junction points computed by the orthogonal router. Stored per edge and
/// surfaced to the postprocessor when stitching cross-hierarchy bend chains.
pub static JUNCTION_POINTS: std::sync::LazyLock<
    PropertyKey<smallvec::SmallVec<crate::math::Vec2, 4>>,
> = std::sync::LazyLock::new(|| PropertyKey::of::<JunctionPointsMarker>(smallvec::SmallVec::new));

/// Whether the router may insert bend points even when a straight segment
/// would also be legal.
pub static UNNECESSARY_BENDPOINTS: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<UnnecessaryBendpointsMarker>(|| false));

struct SplineNsPortYCoordMarker;
struct SplineSurvivingEdgeMarker;
struct OriginalDummyNodePositionMarker;

/// Y coordinate a dummy node inherited from the edge segment it stands in
/// for, recorded during interactive crossing minimization so that
/// `InteractiveNodePlacer` can preserve the original routing.
///
/// `None` means no original position was captured; the placer falls back to
/// its `minValidY` stacking rule.
pub static ORIGINAL_DUMMY_NODE_POSITION: std::sync::LazyLock<PropertyKey<Option<f64>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<OriginalDummyNodePositionMarker>(|| None));

/// Y coordinate assigned to a north/south port by the spline router, used by
/// `FinalSplineBendpointsCalculator` to place an extra control point.
pub static SPLINE_NS_PORT_Y_COORD: std::sync::LazyLock<PropertyKey<f64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SplineNsPortYCoordMarker>(|| 0.0));

/// Edge that inherits the final spline bend points after a long edge chain
/// collapses. Kept as a defensive hook; typically unset (the calculator falls
/// back to the first edge of the chain).
pub static SPLINE_SURVIVING_EDGE: std::sync::LazyLock<PropertyKey<Option<EdgeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SplineSurvivingEdgeMarker>(|| None));

struct HypernodeMarker;

/// Marks a node as a hypernode. Hypernodes serve as join points for multiple
/// edges that share a common routing path.
pub static HYPERNODE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<HypernodeMarker>(|| false));

struct PortAnchorMarker;
struct ExtPortSizeMarker;

/// Explicit anchor point of a port relative to its own coordinate origin.
/// `None` is treated as the zero vector by consumers.
pub static PORT_ANCHOR: std::sync::LazyLock<PropertyKey<Option<crate::math::Vec2>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<PortAnchorMarker>(|| None));

/// Original (port-side) size of an external-port dummy node. Set by
/// `HierarchicalPortDummySizeProcessor` and consumed by
/// `HierarchicalPortOrthogonalEdgeRouter.fixCoordinates`.
pub static EXT_PORT_SIZE: std::sync::LazyLock<PropertyKey<crate::math::Vec2>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<ExtPortSizeMarker>(|| crate::math::Vec2::ZERO));

struct NodeSizeOptionsMarker;

/// Flags describing how a node's size constraint is interpreted.
pub static NODE_SIZE_OPTIONS: std::sync::LazyLock<PropertyKey<crate::options::enums::SizeOptions>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<NodeSizeOptionsMarker>(|| {
            crate::options::enums::SizeOptions::DEFAULT_MINIMUM_SIZE
        })
    });

struct PortAlignmentDefaultMarker;
struct PortAlignmentNorthMarker;
struct PortAlignmentSouthMarker;
struct PortAlignmentEastMarker;
struct PortAlignmentWestMarker;

/// Default port alignment strategy.
pub static PORT_ALIGNMENT_DEFAULT: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortAlignmentDefaultMarker>(|| {
        crate::options::enums::PortAlignment::Distributed
    })
});

/// Port alignment on the node's north side.
pub static PORT_ALIGNMENT_NORTH: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortAlignmentNorthMarker>(|| crate::options::enums::PortAlignment::Undefined)
});

/// Port alignment on the node's south side.
pub static PORT_ALIGNMENT_SOUTH: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortAlignmentSouthMarker>(|| crate::options::enums::PortAlignment::Undefined)
});

/// Port alignment on the node's east side.
pub static PORT_ALIGNMENT_EAST: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortAlignmentEastMarker>(|| crate::options::enums::PortAlignment::Undefined)
});

/// Port alignment on the node's west side.
pub static PORT_ALIGNMENT_WEST: std::sync::LazyLock<
    PropertyKey<crate::options::enums::PortAlignment>,
> = std::sync::LazyLock::new(|| {
    PropertyKey::of::<PortAlignmentWestMarker>(|| crate::options::enums::PortAlignment::Undefined)
});

struct TopCommentsMarker;
struct BottomCommentsMarker;
struct CommentConnPortMarker;

/// Comment boxes attached above a real node. Populated by the comment
/// preprocessor and consumed by the comment postprocessor.
pub static TOP_COMMENTS: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<TopCommentsMarker>(Vec::new));

/// Comment boxes attached below a real node.
pub static BOTTOM_COMMENTS: std::sync::LazyLock<PropertyKey<Vec<NodeId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<BottomCommentsMarker>(Vec::new));

/// Port on the real node that connects to a removed comment box. Stored on
/// the comment box node so the postprocessor can reattach the edge.
pub static COMMENT_CONN_PORT: std::sync::LazyLock<PropertyKey<Option<PortId>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<CommentConnPortMarker>(|| None));

struct EdgeLabelsInlineMarker;
struct EdgeThicknessMarker;

/// Marks an edge label as an inline label, drawn on top of the edge rather
/// than above or below.
pub static EDGE_LABELS_INLINE: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<EdgeLabelsInlineMarker>(|| false));

/// Per-edge thickness in pixels, used when computing label dummy vertical
/// extent. Default `1.0`.
pub static EDGE_THICKNESS: std::sync::LazyLock<PropertyKey<f64>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<EdgeThicknessMarker>(|| 1.0));

struct LongEdgeHasLabelDummiesMarker;
struct LongEdgeBeforeLabelDummyMarker;
struct SpacingPortsSurroundingMarker;
struct SelfLoopDistributionOverrideMarker;
struct SelfLoopOrderingOverrideMarker;

/// Per-node extra spacing around the ports of a node. Read per-node by
/// `NetworkSimplexPlacer.transformPorts` and `LayerSizeAndGraphHeightCalculator`.
/// Default is an empty margin.
pub static SPACING_PORTS_SURROUNDING: std::sync::LazyLock<PropertyKey<crate::math::Margin>> =
    std::sync::LazyLock::new(|| {
        PropertyKey::of::<SpacingPortsSurroundingMarker>(crate::math::Margin::default)
    });

struct SpacingPortPortOverrideMarker;
struct SpacingNodeNodeOverrideMarker;
struct SpacingEdgeNodeOverrideMarker;
struct SpacingEdgeEdgeOverrideMarker;

/// Per-node override for port-port spacing. When the node has it set, the
/// individual lookup (`getIndividualOrDefault(node, SPACING_PORT_PORT)`)
/// returns this value; otherwise it falls back to the graph-level
/// `spacing.port_port`. `None` = "use graph default".
pub static SPACING_PORT_PORT_OVERRIDE: std::sync::LazyLock<PropertyKey<Option<f64>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SpacingPortPortOverrideMarker>(|| None));

/// Per-node override for node-node spacing. Read by node placers (e.g.
/// `SimpleNodePlacer.type_vertical_spacing`) via the `getLocalSpacing(n1, n2,
/// SPACING_NODE_NODE)` rule `max(getIndividualOrDefault(n1),
/// getIndividualOrDefault(n2))`. `None` = "use graph default".
pub static SPACING_NODE_NODE_OVERRIDE: std::sync::LazyLock<PropertyKey<Option<f64>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SpacingNodeNodeOverrideMarker>(|| None));

/// Per-node override for edge-node spacing. `None` = "use graph default".
pub static SPACING_EDGE_NODE_OVERRIDE: std::sync::LazyLock<PropertyKey<Option<f64>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SpacingEdgeNodeOverrideMarker>(|| None));

/// Per-node override for edge-edge spacing. `None` = "use graph default".
pub static SPACING_EDGE_EDGE_OVERRIDE: std::sync::LazyLock<PropertyKey<Option<f64>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SpacingEdgeEdgeOverrideMarker>(|| None));

/// Per-node override for the self-loop distribution option.
/// `None` means use the graph-level option.
pub static SELF_LOOP_DISTRIBUTION_OVERRIDE: std::sync::LazyLock<
    PropertyKey<Option<SelfLoopDistribution>>,
> = std::sync::LazyLock::new(|| PropertyKey::of::<SelfLoopDistributionOverrideMarker>(|| None));

/// Per-node override for the self-loop ordering option.
/// `None` means use the graph-level option.
pub static SELF_LOOP_ORDERING_OVERRIDE: std::sync::LazyLock<PropertyKey<Option<SelfLoopOrdering>>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<SelfLoopOrderingOverrideMarker>(|| None));

/// Set on a long-edge dummy when its long edge carries one or more label
/// dummies.
pub static LONG_EDGE_HAS_LABEL_DUMMIES: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LongEdgeHasLabelDummiesMarker>(|| false));

/// Set on a long-edge dummy that precedes the first label dummy on its long
/// edge. Used by `HyperedgeDummyMerger` to keep label-bearing chains from
/// merging across the label boundary.
pub static LONG_EDGE_BEFORE_LABEL_DUMMY: std::sync::LazyLock<PropertyKey<bool>> =
    std::sync::LazyLock::new(|| PropertyKey::of::<LongEdgeBeforeLabelDummyMarker>(|| false));
