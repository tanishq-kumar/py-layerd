use crate::{
    graph::LGraph,
    intermediate::IntermediateProcessorId,
    options::enums::{
        CrossingMinimizationStrategy, CycleBreakingStrategy, EdgeRoutingStrategy,
        GraphCompactionStrategy, GreedySwitchType, HierarchyHandling, LayerUnzippingStrategy,
        LayeringStrategy, LayoutDirection, NodePlacementStrategy, NodePromotionStrategy,
        WrappingStrategy,
    },
    pipeline::{PhaseImpl, PhaseSlot, PipelineStage, stage_metadata::PipelineBuilder},
    properties::{graph_properties::GraphProperties, internal},
};

/// True when hierarchy handling is set to `INCLUDE_CHILDREN`.
fn is_hierarchical_layout(graph: &LGraph) -> bool {
    matches!(graph.options.hierarchy_handling, HierarchyHandling::Include)
}

/// Decide greedy-switch activation purely from the graph's current options
/// and `layerless_nodes.len()`. Called on the parent graph in
/// `prepare_graph_for_layout` so the result can be cached and inherited by
/// every component (parent decides once, components share by clone).
pub(crate) fn compute_greedy_switch_activation(graph: &LGraph) -> bool {
    if is_hierarchical_layout(graph) {
        return graph.parent_node.is_none()
            && graph.options.greedy_switch_hierarchical_type != GreedySwitchType::Off;
    }
    let interactive_cross_min = graph.options.crossing_minimization_semi_interactive
        || matches!(graph.options.crossing_minimization, CrossingMinimizationStrategy::Interactive);
    let threshold = graph.options.greedy_switch_activation_threshold;
    let size = graph.layerless_nodes.len() as i32;
    !interactive_cross_min
        && graph.options.greedy_switch_type != GreedySwitchType::Off
        && (threshold == 0 || threshold > size)
}

/// Read the parent-decided greedy-switch activation if `prepare_graph_for_layout`
/// stamped one, otherwise compute it from the graph at hand. The cache path
/// matters for multi-component flat layouts: activation is decided once on
/// the parent and shared with components via property clone in
/// `LGraph::extract_component_graphs`.
fn activate_greedy_switch_for(graph: &LGraph) -> bool {
    if let Some(&cached) = graph.properties.get_ref(&internal::GREEDY_SWITCH_ACTIVATE) {
        return cached;
    }
    compute_greedy_switch_activation(graph)
}

fn pick_p1(graph: &LGraph) -> PhaseImpl {
    match graph.options.cycle_breaking {
        CycleBreakingStrategy::Greedy => PhaseImpl::P1Greedy,
        CycleBreakingStrategy::DepthFirst => PhaseImpl::P1DepthFirst,
        CycleBreakingStrategy::GreedyModelOrder => PhaseImpl::P1GreedyModelOrder,
        CycleBreakingStrategy::ModelOrder => PhaseImpl::P1ModelOrder,
        CycleBreakingStrategy::Interactive => PhaseImpl::P1Interactive,
        CycleBreakingStrategy::BfsNodeOrder => PhaseImpl::P1BfsNodeOrder,
        CycleBreakingStrategy::DfsNodeOrder => PhaseImpl::P1DfsNodeOrder,
        CycleBreakingStrategy::SccConnectivity => PhaseImpl::P1SccConnectivity,
        CycleBreakingStrategy::SccNodeType => PhaseImpl::P1SccNodeType,
    }
}

fn pick_p2(graph: &LGraph) -> PhaseImpl {
    match graph.options.layering {
        LayeringStrategy::LongestPath => PhaseImpl::P2LongestPath,
        LayeringStrategy::LongestPathSource => PhaseImpl::P2LongestPathSource,
        LayeringStrategy::NetworkSimplex => PhaseImpl::P2NetworkSimplex,
        LayeringStrategy::CoffmanGraham => PhaseImpl::P2CoffmanGraham,
        LayeringStrategy::MinWidth => PhaseImpl::P2MinWidth,
        LayeringStrategy::StretchWidth => PhaseImpl::P2StretchWidth,
        LayeringStrategy::Interactive => PhaseImpl::P2Interactive,
        LayeringStrategy::BfModelOrder => PhaseImpl::P2BfModelOrder,
        LayeringStrategy::DfModelOrder => PhaseImpl::P2DfModelOrder,
    }
}

fn pick_p3(graph: &LGraph) -> PhaseImpl {
    match graph.options.crossing_minimization {
        CrossingMinimizationStrategy::None => PhaseImpl::P3NoCrossing,
        CrossingMinimizationStrategy::BarycenterLayerSweep
        | CrossingMinimizationStrategy::MedianLayerSweep => PhaseImpl::P3LayerSweep,
        CrossingMinimizationStrategy::Interactive => PhaseImpl::P3Interactive,
    }
}

fn pick_p4(graph: &LGraph) -> PhaseImpl {
    match graph.options.node_placement {
        NodePlacementStrategy::Simple => PhaseImpl::P4Simple,
        NodePlacementStrategy::BrandesKoepf => PhaseImpl::P4BrandesKoepf,
        NodePlacementStrategy::LinearSegments => PhaseImpl::P4LinearSegments,
        NodePlacementStrategy::NetworkSimplex => PhaseImpl::P4NetworkSimplex,
        NodePlacementStrategy::Interactive => PhaseImpl::P4Interactive,
    }
}

fn pick_p5(graph: &LGraph) -> PhaseImpl {
    match graph.options.edge_routing {
        EdgeRoutingStrategy::Polyline => PhaseImpl::P5Polyline,
        EdgeRoutingStrategy::Orthogonal => PhaseImpl::P5Orthogonal,
        EdgeRoutingStrategy::Splines => PhaseImpl::P5Splines,
    }
}

/// Build the complete layout pipeline for the given graph.
///
/// The pipeline is assembled declaratively via [`PipelineBuilder`]: each
/// configuration phrase is `add_before(P_X, processor)` or
/// `add_after(P_X, processor)`. The builder sorts each slot's processors by
/// declaration ordinal at flush time, so SelfLoopRouter cannot accidentally
/// land before `LabelAndNodeSizeProcessor` even when the gate is added next
/// to the wrong neighbour.
///
/// Compound-graph preprocessing/postprocessing is intentionally NOT added
/// here; `do_compound_layout` invokes those once on the root graph before /
/// after `hierarchical_layout` returns.
pub fn build_pipeline(graph: &LGraph) -> Vec<PipelineStage> {
    use IntermediateProcessorId as I;
    use PhaseSlot::*;

    let mut b = PipelineBuilder::new(
        pick_p1(graph),
        pick_p2(graph),
        pick_p3(graph),
        pick_p4(graph),
        pick_p5(graph),
    );

    let graph_props = graph.properties.get(&crate::properties::internal::GRAPH_PROPERTIES);

    let needs_direction_transform =
        !matches!(graph.options.direction, LayoutDirection::Right | LayoutDirection::Undefined);
    let has_comments = graph_props.contains(GraphProperties::COMMENTS);
    let has_partitions = graph_props.contains(GraphProperties::PARTITIONS)
        || graph.properties.get(&crate::properties::internal::PARTITIONING_ACTIVATE);
    let has_hypernodes = graph_props.contains(GraphProperties::HYPERNODES);
    let has_hyperedges = graph_props.contains(GraphProperties::HYPEREDGES);
    let has_center_labels = graph_props.contains(GraphProperties::CENTER_LABELS);
    let has_self_loops = graph_props.contains(GraphProperties::SELF_LOOPS);
    let has_external_ports = graph_props.contains(GraphProperties::EXTERNAL_PORTS);
    let has_end_labels = graph_props.contains(GraphProperties::END_LABELS);
    let is_interactive_crossing_min =
        matches!(graph.options.crossing_minimization, CrossingMinimizationStrategy::Interactive);
    let is_semi_interactive_crossing_min = graph.options.crossing_minimization_semi_interactive;
    let is_hierarchical = is_hierarchical_layout(graph);
    let is_multi_edge_wrapping =
        matches!(graph.options.wrapping_strategy, WrappingStrategy::MultiEdge);
    let is_single_edge_wrapping =
        matches!(graph.options.wrapping_strategy, WrappingStrategy::SingleEdge);
    let consider_model_order = graph
        .properties
        .get(&crate::properties::internal::CONSIDER_MODEL_ORDER_STRATEGY);

    // BASELINE_PROCESSING_CONFIGURATION
    b.add_before(P4NodePlacement, I::InnermostNodeMarginCalculator);
    b.add_before(P4NodePlacement, I::LabelAndNodeSizeProcessor);
    b.add_before(P5EdgeRouting, I::LayerSizeAndGraphHeightCalculator);
    b.add_after(P5EdgeRouting, I::EndLabelSorter);

    // Always-on additions that fire on every fixture
    // EdgeAndLayerConstraintEdgeReverser is implicitly part of every
    // configuration; in practice every fixture exercises this preprocessor.
    b.add_before(P1CycleBreaking, I::EdgeAndLayerConstraintEdgeReverser);
    // LayerConstraintPreprocessor is added unconditionally; every P2 layerer
    // expects it.
    b.add_before(P2Layering, I::LayerConstraintPreprocessor);
    // P3 additions that all P3 strategies share.
    b.add_before(P3NodeOrdering, I::LayerConstraintPostprocessor);
    b.add_before(P3NodeOrdering, I::LongEdgeSplitter);
    b.add_before(P3NodeOrdering, I::InvertedPortProcessor);
    b.add_before(P3NodeOrdering, I::PortListSorter);
    b.add_before(P3NodeOrdering, I::NorthSouthPortPreprocessor);
    b.add_before(P4NodePlacement, I::InLayerConstraintProcessor);
    // After P5: long-edge / NS-port / reversed-edge cleanup is in every P5
    // router's baseline configuration.
    b.add_after(P5EdgeRouting, I::LongEdgeJoiner);
    b.add_after(P5EdgeRouting, I::ReversedEdgeRestorer);

    // Direction preprocessing (transform to LTR)
    if needs_direction_transform {
        b.add_before(P1CycleBreaking, I::DirectionPreprocessor);
        b.add_after(P5EdgeRouting, I::DirectionPostprocessor);
    }

    // Hierarchical addition
    if is_hierarchical {
        b.add_after(P5EdgeRouting, I::HierarchicalNodeResizer);
    }

    // Comments
    if has_comments {
        b.add_before(P1CycleBreaking, I::CommentPreprocessor);
        b.add_before(P4NodePlacement, I::CommentNodeMarginCalculator);
        b.add_after(P5EdgeRouting, I::CommentPostprocessor);
    }

    // Partitions
    if has_partitions {
        b.add_before(P1CycleBreaking, I::PartitionPreprocessor);
        b.add_before(P2Layering, I::PartitionMidprocessor);
        b.add_before(P3NodeOrdering, I::PartitionPostprocessor);
    }

    // Interactive crossing minimization → external port positioner
    if is_interactive_crossing_min {
        b.add_before(P1CycleBreaking, I::InteractiveExternalPortPositioner);
    }

    // Port-side processor: pre-P1 if FEEDBACK_EDGES, else pre-P3
    if graph.options.feedback_edges {
        b.add_before(P1CycleBreaking, I::PortSideProcessor);
    } else {
        b.add_before(P3NodeOrdering, I::PortSideProcessor);
    }

    // Self-loop additions
    // All three P5 routers add the same four processors when
    // `GraphProperties.SELF_LOOPS` is set; the slot is fixed by the call site
    // (one pre-P1, one post-P5, two pre-P4 entries).
    if has_self_loops {
        b.add_before(P1CycleBreaking, I::SelfLoopPreProcessor);
        b.add_before(P4NodePlacement, I::SelfLoopPortRestorer);
        b.add_before(P4NodePlacement, I::SelfLoopRouter);
        b.add_after(P5EdgeRouting, I::SelfLoopPostProcessor);
    }

    // Hyperedge / hypernode / hierarchical-port additions
    if has_hyperedges && matches!(graph.options.edge_routing, EdgeRoutingStrategy::Orthogonal) {
        b.add_before(P4NodePlacement, I::HyperedgeDummyMerger);
    }
    if has_hypernodes {
        // HypernodesProcessor runs before P2 (legacy slot preserved to avoid
        // a surprise structural change here).
        b.add_before(P2Layering, I::HypernodesProcessor);
    }
    if has_external_ports {
        if matches!(graph.options.edge_routing, EdgeRoutingStrategy::Orthogonal) {
            b.add_before(P3NodeOrdering, I::HierarchicalPortConstraintProcessor);
            b.add_before(P4NodePlacement, I::HierarchicalPortDummySizeProcessor);
        }
        b.add_before(P5EdgeRouting, I::HierarchicalPortPositionProcessor);
    }
    if has_external_ports && matches!(graph.options.edge_routing, EdgeRoutingStrategy::Orthogonal) {
        b.add_after(P5EdgeRouting, I::HierarchicalPortOrthogonalEdgeRouter);
    }

    // Center-edge label processing
    if has_center_labels {
        b.add_before(P2Layering, I::LabelDummyInserter);
        b.add_before(P4NodePlacement, I::LabelDummySwitcher);
        b.add_after(P5EdgeRouting, I::LabelDummyRemover);
    }
    if has_center_labels || has_end_labels {
        b.add_before(P4NodePlacement, I::LabelSideSelector);
    }

    // End-edge label additions
    if has_end_labels {
        b.add_before(P4NodePlacement, I::EndLabelPreprocessor);
        b.add_after(P5EdgeRouting, I::EndLabelPostprocessor);
    }

    // Label manager
    if graph.options.label_manager {
        b.add_before(P4NodePlacement, I::CenterLabelManagementProcessor);
        b.add_before(P4NodePlacement, I::EndNodePortLabelManagementProcessor);
    }

    // Node promotion
    if graph.options.node_promotion != NodePromotionStrategy::None {
        b.add_before(P3NodeOrdering, I::NodePromotion);
    }

    // High-degree node treatment
    if graph.options.high_degree_nodes_treatment {
        b.add_before(P3NodeOrdering, I::HighDegreeNodeLayeringProcessor);
    }

    // Semi-interactive crossing minimization
    if is_semi_interactive_crossing_min {
        b.add_before(P3NodeOrdering, I::SemiInteractiveCrossMinProcessor);
    }

    // Wrapping
    if is_multi_edge_wrapping {
        b.add_before(P3NodeOrdering, I::BreakingPointInserter);
        b.add_before(P4NodePlacement, I::BreakingPointProcessor);
        b.add_after(P5EdgeRouting, I::BreakingPointRemover);
    }
    if is_single_edge_wrapping {
        b.add_before(P4NodePlacement, I::SingleEdgeGraphWrapper);
    }

    // Layer unzipping
    if matches!(graph.options.layer_unzipping, LayerUnzippingStrategy::Alternating) {
        b.add_before(P4NodePlacement, I::AlternatingLayerUnzipper);
    }

    // Additional horizontal post-compaction
    if graph.options.post_compaction_strategy != GraphCompactionStrategy::None
        && graph.options.edge_routing != EdgeRoutingStrategy::Polyline
    {
        b.add_after(P5EdgeRouting, I::HorizontalCompactor);
    }

    // Greedy switch
    if activate_greedy_switch_for(graph) {
        let greedy_switch_type = if is_hierarchical {
            graph.options.greedy_switch_hierarchical_type
        } else {
            graph.options.greedy_switch_type
        };
        let gs_id = match greedy_switch_type {
            GreedySwitchType::OneSided => I::OneSidedGreedySwitch,
            // Any non-OneSided value collapses to TwoSided; `Off` is gated
            // out by `activate_greedy_switch_for` so the unreachable branch
            // never fires.
            GreedySwitchType::TwoSided => I::TwoSidedGreedySwitch,
            GreedySwitchType::Off => unreachable!(),
        };
        b.add_before(P4NodePlacement, gs_id);
    }

    // Model-order processing
    if consider_model_order {
        b.add_before(P3NodeOrdering, I::SortByInputModelProcessor);
    }

    // North-south port postprocessor
    // Spline routing consumes restored north/south ports while constructing
    // its route descriptors; polyline and orthogonal routing restore them
    // after P5, matching the router-specific processor configuration.
    if matches!(graph.options.edge_routing, EdgeRoutingStrategy::Splines) {
        b.add_before(P5EdgeRouting, I::NorthSouthPortPostprocessor);
    } else {
        b.add_after(P5EdgeRouting, I::NorthSouthPortPostprocessor);
    }

    // Splines: final-bendpoint calculation
    if matches!(graph.options.edge_routing, EdgeRoutingStrategy::Splines) {
        b.add_after(P5EdgeRouting, I::FinalSplineBendpointsCalculator);
    }

    b.build()
}
