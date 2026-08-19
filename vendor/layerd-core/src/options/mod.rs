pub mod enums;
pub mod spacing;

pub use enums::*;
pub use spacing::{
    BASE_SPACING_DEFAULT, LayeredSpacingsBuilder, SPACING_EDGE_EDGE,
    SPACING_EDGE_EDGE_BETWEEN_LAYERS, SPACING_EDGE_LABEL, SPACING_EDGE_NODE,
    SPACING_EDGE_NODE_BETWEEN_LAYERS, SPACING_LABEL_LABEL, SPACING_LABEL_NODE,
    SPACING_LABEL_PORT_HORIZONTAL, SPACING_LABEL_PORT_VERTICAL, SPACING_NODE_NODE_BETWEEN_LAYERS,
    SPACING_NODE_SELF_LOOP, SPACING_PORT_PORT, SpacingOptions, SpacingProperty,
};

use crate::math::Padding;

/// Layout options controlling the behavior of the layered layout algorithm.
///
/// Provides configuration for each phase of the algorithm, spacing, and
/// various constraints that influence graph layout.
#[derive(Clone)]
pub struct LayoutOptions {
    /// The direction in which the layout is computed.
    pub direction: LayoutDirection,
    /// Strategy for breaking cycles.
    pub cycle_breaking: CycleBreakingStrategy,
    /// Strategy for assigning nodes to layers.
    pub layering: LayeringStrategy,
    /// Strategy for crossing minimization.
    pub crossing_minimization: CrossingMinimizationStrategy,
    /// Strategy for node placement within layers.
    pub node_placement: NodePlacementStrategy,
    /// Strategy for edge routing.
    pub edge_routing: EdgeRoutingStrategy,
    /// Optional post-process horizontal compaction strategy. Default `None`.
    pub post_compaction_strategy: GraphCompactionStrategy,
    /// Constraint calculation used by post-process horizontal compaction.
    /// Default `Scanline`.
    pub post_compaction_constraints: ConstraintCalculationStrategy,
    /// Spacing between graph elements.
    pub spacing: SpacingOptions,
    /// Strategy for ordering ports within nodes.
    pub ordering_strategy: OrderingStrategy,
    /// Strategy for layer unzipping.
    pub layer_unzipping: LayerUnzippingStrategy,
    /// Whether to consider model order in cycle breaking.
    pub model_order: bool,
    /// How much effort to spend on optimizations (higher = better but slower).
    pub thoroughness: u32,
    /// Seed for random number generation (deterministic layout).
    pub random_seed: u64,
    /// Constraints on port placement.
    pub port_constraints: PortConstraints,
    /// Padding around the graph.
    pub padding: Padding,
    /// Constraint on which layer a node should be placed in.
    pub layer_constraint: LayerConstraint,
    /// Constraint on edge direction for a node.
    pub edge_constraint: EdgeConstraint,
    /// Alignment strategy for the Brandes-Koepf node placement.
    pub fixed_alignment: FixedAlignment,
    /// Strategy for node promotion after layering.
    pub node_promotion: NodePromotionStrategy,
    /// Maximum iterations for node promotion (0 = unlimited).
    pub node_promotion_max_iterations: u32,
    /// Threshold for high-degree node detection.
    pub high_degree_threshold: usize,
    /// Maximum tree height for high-degree node layering.
    pub high_degree_tree_height: usize,
    /// Spacing between edge labels.
    pub edge_label_spacing: f64,
    /// Distribution strategy for self-loop edges around a node.
    pub self_loop_distribution: SelfLoopDistribution,
    /// Ordering strategy for multiple self-loop edges.
    pub self_loop_ordering: SelfLoopOrdering,
    /// Strategy for sorting long-edge dummy nodes against normal nodes
    /// that have no connection to the previous layer.
    pub consider_model_order_long_edge_strategy: LongEdgeOrderingStrategy,
    /// Strategy for straightening edges during node placement.
    pub edge_straightening: EdgeStraighteningStrategy,
    /// How layout direction maps to edge direction.
    pub direction_congruency: DirectionCongruency,
    /// Strategy for placing center edge labels within layers.
    pub center_label_placement: CenterEdgeLabelPlacementStrategy,
    /// Strategy for sorting ports within a node.
    pub port_sorting_strategy: PortSortingStrategy,
    /// How hierarchy is handled during layout.
    pub hierarchy_handling: HierarchyHandling,
    /// Control the eagerness of hierarchical crossing minimization sweeps.
    ///
    /// Higher values make the algorithm more likely to sweep into nested graphs.
    /// `-1.0` means always sweep recursively, `0.0` means never, values in
    /// between use a connectivity heuristic.
    pub crossing_minimization_hierarchical_sweepiness: f64,
    /// Type of greedy switch heuristic for post-P3 optimization.
    pub greedy_switch_type: GreedySwitchType,
    /// Type of greedy switch heuristic for hierarchical crossing minimization.
    pub greedy_switch_hierarchical_type: GreedySwitchType,
    /// Force node model order during crossing minimization.
    pub crossing_minimization_force_node_model_order: bool,
    /// Whether feedback edges (back-edges in model order) are allowed.
    pub feedback_edges: bool,
    /// Semi-interactive crossing minimization: preserve existing relative node
    /// order within layers when `position` properties are present. Default `false`.
    pub crossing_minimization_semi_interactive: bool,
    /// Enable the high-degree-node tree layering preprocessor that moves
    /// trees of high-degree nodes to separate layers.
    pub high_degree_nodes_treatment: bool,
    /// Graph-size threshold below which greedy switch post-processing runs.
    /// `0` disables the threshold (greedy switch always runs); otherwise
    /// greedy switch only runs when `threshold > layerless_nodes.len()`.
    pub greedy_switch_activation_threshold: i32,
    /// Whether a label manager is attached to the graph. When `true`, label
    /// management processors are inserted before P4.
    pub label_manager: bool,
    /// Node influence weight for model-order crossing counter.
    pub consider_model_order_crossing_counter_node_influence: f64,
    /// Port influence weight for model-order crossing counter.
    pub consider_model_order_crossing_counter_port_influence: f64,
    /// Whether port model order is considered during crossing minimization.
    pub consider_model_order_port_model_order: bool,
    /// Placement strategy for port labels. Default `OUTSIDE` (single bit).
    /// The type is a bitflags struct, so multiple bits can be active.
    pub port_labels_placement: PortLabelPlacement,
    /// Whether consecutive port labels should be treated as a single group
    /// when deciding whether they can sit next to the port.
    pub port_labels_treat_as_group: bool,
    /// How disconnected components are ordered before component packing.
    pub consider_model_order_components: ComponentOrderingStrategy,
    /// Reference point used by interactive layout phases to compare node
    /// positions. Default `Center`.
    pub interactive_reference_point: InteractiveReferencePoint,
    /// MinWidth: loose upper bound on layer width.
    ///
    /// `-1` means "use the recommended values 1..=4 and pick the narrowest
    /// resulting layering".
    pub layering_min_width_upper_bound_on_width: i32,
    /// MinWidth: scaling factor for the upper layer estimation.
    ///
    /// `-1` means "use 1..=2 and pick the narrowest".
    pub layering_min_width_upper_layer_estimation_scaling_factor: i32,
    /// Routing flavor used when edge routing is `Splines`. Default `Sloppy`.
    pub edge_routing_splines_mode: SplineRoutingMode,
    /// Layer spacing multiplier used by the sloppy spline routing to guarantee
    /// room for curves that span large vertical distances. Default `0.2`.
    pub edge_routing_splines_sloppy_layer_spacing_factor: f64,
    /// Dampens node movement during the `LinearSegments` balancing pass.
    /// Default `0.3`.
    pub node_placement_linear_segments_deflection_dampening: f64,
    /// Maximum number of nodes allowed per layer in Coffman-Graham layering.
    /// Default `i32::MAX`.
    pub layering_coffman_graham_layer_bound: i32,
    /// Polyline router's acceptable horizontal distance between a port's
    /// anchor and the layer boundary before a bend point must be inserted.
    /// Default `2.0`.
    pub edge_routing_polyline_sloped_edge_zone_width: f64,
    /// Strategy for choosing which side of an edge a label is placed on.
    /// Default `SmartDown`.
    pub edge_labels_side_selection: EdgeLabelSideSelection,
    /// If set, the network-simplex node placer tries to straighten edges at
    /// the cost of taller layouts. Default `false`.
    pub node_placement_favor_straight_edges: bool,
    /// Default node-flexibility level applied by the network-simplex node
    /// placer when a node does not carry its own per-node override.
    /// Default `None`.
    pub node_placement_network_simplex_node_flexibility: NodeFlexibility,
    /// Desired aspect ratio (width / height) used by the wrapping cut-index
    /// heuristics. Default `1.6`.
    pub aspect_ratio: f64,
    /// Corrective factor applied to `aspect_ratio` when estimating the
    /// desired layering aspect during breaking-point selection. Default `1.0`.
    pub wrapping_correction_factor: f64,
    /// Additional edge-node spacing applied to dummy chains produced by the
    /// wrapping processor so that backward wrapping edges stay visually
    /// separated. Default `10.0`.
    pub wrapping_additional_edge_spacing: f64,
    /// When true, `BreakingPointInserter` re-scores raw cuts by a spans + dist
    /// weighting to pick cut indexes that minimize dummy-chain length.
    /// Default `true`.
    pub wrapping_multi_edge_improve_cuts: bool,
    /// When true, `BreakingPointProcessor` runs the dummy-shortening pass that
    /// drops adjacent long-edge dummy pairs around each breaking point.
    /// Default `true`.
    pub wrapping_multi_edge_improve_wrapped_edges: bool,
    /// Exponent applied to the distance term in the improved-cuts score.
    /// Values greater than 1 penalize cuts close to existing cuts more heavily.
    /// Default `2.0`.
    pub wrapping_multi_edge_distance_penalty: f64,
    /// Extra freedom granted to the MSD cut heuristic. The heuristic tries
    /// `cut_cnt ± freedom` variants and picks the one with the best scale.
    /// Default `0`.
    pub wrapping_cutting_msd_freedom: i32,
    /// Heuristic used to pick raw cut indexes. Default `MSD`.
    pub wrapping_cutting_strategy: WrappingCuttingStrategy,
    /// Top-level wrapping switch. Default `Off`.
    pub wrapping_strategy: WrappingStrategy,
    /// Optional override that turns raw cut indexes into a guaranteed-valid
    /// list. `None` means the raw cuts are returned unchanged when the
    /// heuristic does not guarantee validity itself.
    pub wrapping_validify_strategy: Option<WrappingValidifyStrategy>,
    /// Explicit cut-index list consumed by the manual cutting strategy.
    /// `None` means the property is not set.
    pub wrapping_cutting_cuts: Option<Vec<i32>>,
    /// Explicit forbidden cut-index list consumed by
    /// `GraphStats::is_cut_allowed`. `None` means the property is not set.
    pub wrapping_validify_forbidden_indices: Option<Vec<i32>>,
    /// Whether a flat graph is split into its weakly connected components
    /// before the main pipeline runs. Each component is laid out in
    /// isolation and then combined via `SimpleRowGraphPlacer`. Default `true`.
    pub separate_connected_components: bool,
    /// Spacing applied between connected components when combining their
    /// per-component layouts. Default `20.0`.
    pub spacing_component_component: f64,
    /// Whether `SimpleRowGraphPlacer` hands off to `ComponentsCompactor` to
    /// pack the final layout more tightly. Default `false`.
    pub compaction_connected_components: bool,
}

impl Default for LayoutOptions {
    fn default() -> Self {
        LayoutOptions {
            direction: LayoutDirection::default(),
            cycle_breaking: CycleBreakingStrategy::default(),
            layering: LayeringStrategy::default(),
            crossing_minimization: CrossingMinimizationStrategy::default(),
            node_placement: NodePlacementStrategy::default(),
            edge_routing: EdgeRoutingStrategy::default(),
            post_compaction_strategy: GraphCompactionStrategy::default(),
            post_compaction_constraints: ConstraintCalculationStrategy::default(),
            spacing: SpacingOptions::default(),
            ordering_strategy: OrderingStrategy::default(),
            layer_unzipping: LayerUnzippingStrategy::default(),
            model_order: false,
            thoroughness: 7,
            random_seed: 1,
            port_constraints: PortConstraints::default(),
            padding: Padding::uniform(12.0),
            layer_constraint: LayerConstraint::default(),
            edge_constraint: EdgeConstraint::default(),
            fixed_alignment: FixedAlignment::default(),
            node_promotion: NodePromotionStrategy::default(),
            node_promotion_max_iterations: 0,
            high_degree_threshold: 16,
            high_degree_tree_height: 5,
            edge_label_spacing: 5.0,
            self_loop_distribution: SelfLoopDistribution::default(),
            self_loop_ordering: SelfLoopOrdering::default(),
            consider_model_order_long_edge_strategy: LongEdgeOrderingStrategy::default(),
            edge_straightening: EdgeStraighteningStrategy::default(),
            direction_congruency: DirectionCongruency::default(),
            center_label_placement: CenterEdgeLabelPlacementStrategy::default(),
            port_sorting_strategy: PortSortingStrategy::default(),
            hierarchy_handling: HierarchyHandling::default(),
            crossing_minimization_hierarchical_sweepiness: 0.1,
            greedy_switch_type: GreedySwitchType::default(),
            greedy_switch_hierarchical_type: GreedySwitchType::Off,
            crossing_minimization_force_node_model_order: false,
            feedback_edges: false,
            crossing_minimization_semi_interactive: false,
            high_degree_nodes_treatment: false,
            greedy_switch_activation_threshold: 40,
            label_manager: false,
            consider_model_order_crossing_counter_node_influence: 0.0,
            consider_model_order_crossing_counter_port_influence: 0.0,
            consider_model_order_port_model_order: false,
            port_labels_placement: PortLabelPlacement::OUTSIDE,
            port_labels_treat_as_group: true,
            consider_model_order_components: ComponentOrderingStrategy::default(),
            interactive_reference_point: InteractiveReferencePoint::default(),
            layering_min_width_upper_bound_on_width: -1,
            layering_min_width_upper_layer_estimation_scaling_factor: -1,
            edge_routing_splines_mode: SplineRoutingMode::default(),
            edge_routing_splines_sloppy_layer_spacing_factor: 0.2,
            node_placement_linear_segments_deflection_dampening: 0.3,
            layering_coffman_graham_layer_bound: i32::MAX,
            edge_routing_polyline_sloped_edge_zone_width: 2.0,
            edge_labels_side_selection: EdgeLabelSideSelection::SmartDown,
            node_placement_favor_straight_edges: false,
            node_placement_network_simplex_node_flexibility: NodeFlexibility::None,
            aspect_ratio: 1.6,
            wrapping_correction_factor: 1.0,
            wrapping_additional_edge_spacing: 10.0,
            wrapping_multi_edge_improve_cuts: true,
            wrapping_multi_edge_improve_wrapped_edges: true,
            wrapping_multi_edge_distance_penalty: 2.0,
            wrapping_cutting_msd_freedom: 0,
            wrapping_cutting_strategy: WrappingCuttingStrategy::default(),
            wrapping_strategy: WrappingStrategy::default(),
            wrapping_validify_strategy: None,
            wrapping_cutting_cuts: None,
            wrapping_validify_forbidden_indices: None,
            separate_connected_components: true,
            spacing_component_component: 20.0,
            compaction_connected_components: false,
        }
    }
}
