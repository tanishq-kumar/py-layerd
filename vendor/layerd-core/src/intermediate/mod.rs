pub mod alternating_layer_unzipper;
pub mod breaking_point;
pub mod center_label_management;
pub mod comment;
pub mod compound_graph;
#[cfg(feature = "devtools")]
pub mod constraints_postprocessor;
pub mod direction;
#[cfg(feature = "devtools")]
pub mod dummy_self_loop;
pub mod edge_and_layer_constraint;
pub mod end_label;
pub mod end_node_port_label;
pub mod final_spline_bendpoints;
pub mod hierarchical_node_resizer;
pub mod hierarchical_port_constraint;
pub mod hierarchical_port_dummy_size;
pub mod hierarchical_port_edge_router;
pub mod hierarchical_port_position;
pub mod high_degree_node;
pub mod horizontal_compactor;
pub mod hyperedge_dummy_merger;
pub mod hypernode;
pub mod in_layer_constraint;
pub mod interactive_external_port;
pub mod inverted_port;
pub mod label_and_node_size;
pub mod label_dummy;
pub mod label_dummy_switcher;
pub mod label_side_selector;
pub mod layer_constraint;
pub mod layer_size;
pub mod long_edge_joiner;
pub mod long_edge_splitter;
#[cfg(feature = "devtools")]
pub mod model_order_port_orderer;
pub mod node_dimension_calculation;
pub mod node_margin;
pub mod node_promotion;
pub mod north_south_port;
pub mod partition;
pub mod polyline_self_loop_router;
pub mod port_list_sorter;
pub mod port_side_processor;
pub mod reversed_edge_restorer;
pub mod self_hyper_loop_labels;
pub mod self_loop;
pub mod self_loop_holder;
pub mod self_loop_label_placer;
pub mod self_loop_port_restorer;
pub mod self_loop_router;
pub mod self_loop_routing_director;
pub mod self_loop_routing_slot_assigner;
pub mod semi_interactive_crossmin;
pub mod single_edge_graph_wrapper;
pub mod sort_by_input_model;
pub mod wrapping;

use crate::graph::LGraph;

/// Identifiers for intermediate processors that run between phases.
///
/// Sorted by typical execution order within each phase slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum IntermediateProcessorId {
    // Before P1
    #[cfg(feature = "devtools")]
    CompoundGraphPreprocessor,
    DirectionPreprocessor,
    InteractiveExternalPortPositioner,
    CommentPreprocessor,
    SelfLoopPreProcessor,
    EdgeAndLayerConstraintEdgeReverser,
    PartitionPreprocessor,

    // Before P2
    LabelDummyInserter,
    LayerConstraintPreprocessor,
    PartitionMidprocessor,
    HypernodesProcessor,

    // Before P3
    LayerConstraintPostprocessor,
    HierarchicalPortConstraintProcessor,
    NodePromotion,
    PartitionPostprocessor,
    HighDegreeNodeLayeringProcessor,
    LongEdgeSplitter,
    PortSideProcessor,
    InvertedPortProcessor,
    PortListSorter,
    NorthSouthPortPreprocessor,
    SortByInputModelProcessor,
    SemiInteractiveCrossMinProcessor,
    BreakingPointInserter,
    #[cfg(feature = "devtools")]
    DummySelfLoopProcessor,

    // After P3
    #[cfg(feature = "devtools")]
    GreedySwitch,
    OneSidedGreedySwitch,
    TwoSidedGreedySwitch,
    HyperedgeDummyMerger,
    #[cfg(feature = "devtools")]
    ModelOrderPortOrderer,

    // Before P4
    InLayerConstraintProcessor,
    LabelAndNodeSizeProcessor,
    InnermostNodeMarginCalculator,
    EndLabelPreprocessor,
    HierarchicalPortDummySizeProcessor,
    LabelDummySwitcher,
    LabelSideSelector,
    CenterLabelManagementProcessor,
    CommentNodeMarginCalculator,
    AlternatingLayerUnzipper,
    BreakingPointProcessor,
    EndNodePortLabelManagementProcessor,
    SingleEdgeGraphWrapper,

    // After P4
    HorizontalCompactor,

    // Before P5
    LayerSizeAndGraphHeightCalculator,
    HierarchicalPortPositionProcessor,

    // After P5
    LongEdgeJoiner,
    NorthSouthPortPostprocessor,
    SelfLoopPostProcessor,
    LabelDummyRemover,
    EndLabelPostprocessor,
    EndLabelSorter,
    ReversedEdgeRestorer,
    HierarchicalPortOrthogonalEdgeRouter,
    HierarchicalNodeResizer,
    FinalSplineBendpointsCalculator,
    SelfLoopRouter,
    SelfLoopPortRestorer,
    #[cfg(feature = "devtools")]
    ConstraintsPostprocessor,
    CommentPostprocessor,
    BreakingPointRemover,

    // Post
    DirectionPostprocessor,
    #[cfg(feature = "devtools")]
    CompoundGraphPostprocessor,
}

impl IntermediateProcessorId {
    /// Stable diagnostic stage id emitted for this processor.
    ///
    /// Most variant names match the emitted id 1:1; the few mismatches below
    /// preserve existing parity artifact names that include a "Processor"
    /// suffix.
    #[cfg(feature = "devtools")]
    pub fn stage_id(self) -> &'static str {
        match self {
            Self::CompoundGraphPreprocessor => "CompoundGraphPreprocessor",
            Self::DirectionPreprocessor => "DirectionPreprocessor",
            Self::InteractiveExternalPortPositioner => "InteractiveExternalPortPositioner",
            Self::CommentPreprocessor => "CommentPreprocessor",
            Self::SelfLoopPreProcessor => "SelfLoopPreProcessor",
            Self::EdgeAndLayerConstraintEdgeReverser => "EdgeAndLayerConstraintEdgeReverser",
            Self::PartitionPreprocessor => "PartitionPreprocessor",
            Self::LabelDummyInserter => "LabelDummyInserter",
            Self::LayerConstraintPreprocessor => "LayerConstraintPreprocessor",
            Self::PartitionMidprocessor => "PartitionMidprocessor",
            Self::HypernodesProcessor => "HypernodesProcessor",
            Self::LayerConstraintPostprocessor => "LayerConstraintPostprocessor",
            Self::HierarchicalPortConstraintProcessor => "HierarchicalPortConstraintProcessor",
            Self::NodePromotion => "NodePromotion",
            Self::PartitionPostprocessor => "PartitionPostprocessor",
            Self::HighDegreeNodeLayeringProcessor => "HighDegreeNodeLayeringProcessor",
            Self::LongEdgeSplitter => "LongEdgeSplitter",
            Self::PortSideProcessor => "PortSideProcessor",
            Self::InvertedPortProcessor => "InvertedPortProcessor",
            Self::PortListSorter => "PortListSorter",
            Self::NorthSouthPortPreprocessor => "NorthSouthPortPreprocessor",
            Self::SortByInputModelProcessor => "SortByInputModelProcessor",
            Self::SemiInteractiveCrossMinProcessor => "SemiInteractiveCrossMinProcessor",
            Self::BreakingPointInserter => "BreakingPointInserter",
            Self::DummySelfLoopProcessor => "DummySelfLoopProcessor",
            // ONE_SIDED_GREEDY_SWITCH / TWO_SIDED_GREEDY_SWITCH both
            // resolve to the same `LayerSweepCrossingMinimizer` name. The
            // diagnostic pipeline also records the stage index, so this
            // collision is harmless.
            Self::GreedySwitch | Self::OneSidedGreedySwitch | Self::TwoSidedGreedySwitch =>
                "LayerSweepCrossingMinimizer",
            Self::HyperedgeDummyMerger => "HyperedgeDummyMerger",
            Self::ModelOrderPortOrderer => "ModelOrderPortOrderer",
            Self::InLayerConstraintProcessor => "InLayerConstraintProcessor",
            Self::LabelAndNodeSizeProcessor => "LabelAndNodeSizeProcessor",
            Self::InnermostNodeMarginCalculator => "InnermostNodeMarginCalculator",
            Self::EndLabelPreprocessor => "EndLabelPreprocessor",
            Self::HierarchicalPortDummySizeProcessor => "HierarchicalPortDummySizeProcessor",
            Self::LabelDummySwitcher => "LabelDummySwitcher",
            Self::LabelSideSelector => "LabelSideSelector",
            Self::CenterLabelManagementProcessor => "CenterLabelManagementProcessor",
            Self::CommentNodeMarginCalculator => "CommentNodeMarginCalculator",
            Self::AlternatingLayerUnzipper => "AlternatingLayerUnzipper",
            Self::BreakingPointProcessor => "BreakingPointProcessor",
            Self::EndNodePortLabelManagementProcessor => "EndNodePortLabelManagementProcessor",
            Self::SingleEdgeGraphWrapper => "SingleEdgeGraphWrapper",
            Self::HorizontalCompactor => "HorizontalGraphCompactor",
            Self::LayerSizeAndGraphHeightCalculator => "LayerSizeAndGraphHeightCalculator",
            Self::HierarchicalPortPositionProcessor => "HierarchicalPortPositionProcessor",
            Self::LongEdgeJoiner => "LongEdgeJoiner",
            Self::NorthSouthPortPostprocessor => "NorthSouthPortPostprocessor",
            Self::SelfLoopPostProcessor => "SelfLoopPostProcessor",
            Self::LabelDummyRemover => "LabelDummyRemover",
            Self::EndLabelPostprocessor => "EndLabelPostprocessor",
            Self::EndLabelSorter => "EndLabelSorter",
            Self::ReversedEdgeRestorer => "ReversedEdgeRestorer",
            Self::HierarchicalPortOrthogonalEdgeRouter => "HierarchicalPortOrthogonalEdgeRouter",
            Self::HierarchicalNodeResizer => "HierarchicalNodeResizingProcessor",
            Self::FinalSplineBendpointsCalculator => "FinalSplineBendpointsCalculator",
            Self::SelfLoopRouter => "SelfLoopRouter",
            Self::SelfLoopPortRestorer => "SelfLoopPortRestorer",
            Self::ConstraintsPostprocessor => "ConstraintsPostprocessor",
            Self::CommentPostprocessor => "CommentPostprocessor",
            Self::BreakingPointRemover => "BreakingPointRemover",
            Self::DirectionPostprocessor => "DirectionPostprocessor",
            Self::CompoundGraphPostprocessor => "CompoundGraphPostprocessor",
        }
    }

    /// Execute this intermediate processor on the graph.
    pub fn run(self, graph: &mut LGraph) {
        match self {
            #[cfg(feature = "devtools")]
            Self::CompoundGraphPreprocessor => compound_graph::preprocess(graph),
            Self::DirectionPreprocessor => direction::preprocess(graph),
            Self::InteractiveExternalPortPositioner =>
                interactive_external_port::position_external_ports(graph),
            Self::CommentPreprocessor => comment::preprocess(graph),
            Self::SelfLoopPreProcessor => self_loop::preprocess(graph),
            Self::EdgeAndLayerConstraintEdgeReverser =>
                edge_and_layer_constraint::reverse_edges(graph),
            Self::LabelDummyInserter => label_dummy::insert(graph),
            Self::LayerConstraintPreprocessor => layer_constraint::preprocess(graph),
            Self::PartitionPreprocessor => partition::preprocess(graph),
            Self::HypernodesProcessor => hypernode::process_hypernodes(graph),
            Self::LayerConstraintPostprocessor => layer_constraint::postprocess(graph),
            Self::HierarchicalPortConstraintProcessor =>
                hierarchical_port_constraint::process(graph),
            Self::NodePromotion => node_promotion::promote_nodes(graph),
            Self::HighDegreeNodeLayeringProcessor => high_degree_node::process(graph),
            Self::LongEdgeSplitter => long_edge_splitter::split(graph),
            Self::PortSideProcessor => port_side_processor::assign_sides(graph),
            Self::InvertedPortProcessor => inverted_port::process(graph),
            Self::PortListSorter => port_list_sorter::sort(graph),
            Self::NorthSouthPortPreprocessor => north_south_port::preprocess(graph),
            Self::SortByInputModelProcessor => sort_by_input_model::sort(graph),
            Self::PartitionMidprocessor => partition::midprocess(graph),
            Self::PartitionPostprocessor => partition::postprocess(graph),
            Self::SemiInteractiveCrossMinProcessor => semi_interactive_crossmin::process(graph),
            Self::BreakingPointInserter => breaking_point::insert(graph),
            #[cfg(feature = "devtools")]
            Self::DummySelfLoopProcessor => dummy_self_loop::process(graph),
            // Greedy switch is a mode of LayerSweepCrossingMinimizer, not a
            // separate processor. Dispatch to the layer_sweep free function
            // with the appropriate CrossMinType.
            #[cfg(feature = "devtools")]
            Self::GreedySwitch => {
                run_layer_sweep_cross_min(
                    graph,
                    crate::p3_crossing_min::layer_sweep::CrossMinType::OneSidedGreedySwitch,
                );
            }
            Self::OneSidedGreedySwitch => {
                run_layer_sweep_cross_min(
                    graph,
                    crate::p3_crossing_min::layer_sweep::CrossMinType::OneSidedGreedySwitch,
                );
            }
            Self::TwoSidedGreedySwitch => {
                run_layer_sweep_cross_min(
                    graph,
                    crate::p3_crossing_min::layer_sweep::CrossMinType::TwoSidedGreedySwitch,
                );
            }
            Self::HyperedgeDummyMerger => hyperedge_dummy_merger::merge(graph),
            #[cfg(feature = "devtools")]
            Self::ModelOrderPortOrderer => model_order_port_orderer::order(graph),
            Self::InLayerConstraintProcessor => in_layer_constraint::process(graph),
            Self::LabelAndNodeSizeProcessor => label_and_node_size::process(graph),
            Self::InnermostNodeMarginCalculator => node_margin::calculate_innermost(graph),
            Self::EndLabelPreprocessor => end_label::preprocess(graph),
            Self::HierarchicalPortDummySizeProcessor =>
                hierarchical_port_dummy_size::process(graph),
            Self::LabelDummySwitcher => label_dummy_switcher::switch(graph),
            Self::LabelSideSelector => label_side_selector::select(graph),
            Self::CenterLabelManagementProcessor => center_label_management::process(graph),
            Self::CommentNodeMarginCalculator => comment::calculate_node_margin(graph),
            Self::AlternatingLayerUnzipper => alternating_layer_unzipper::unzip(graph),
            Self::BreakingPointProcessor => breaking_point::process(graph),
            Self::EndNodePortLabelManagementProcessor => end_node_port_label::process(graph),
            Self::SingleEdgeGraphWrapper => single_edge_graph_wrapper::wrap(graph),
            Self::HorizontalCompactor => horizontal_compactor::compact(graph),
            Self::LayerSizeAndGraphHeightCalculator => layer_size::calculate(graph),
            Self::HierarchicalPortPositionProcessor => hierarchical_port_position::position(graph),
            Self::LongEdgeJoiner => long_edge_joiner::join(graph),
            Self::NorthSouthPortPostprocessor => north_south_port::postprocess(graph),
            Self::SelfLoopPostProcessor => self_loop::postprocess(graph),
            Self::LabelDummyRemover => label_dummy::remove(graph),
            Self::EndLabelPostprocessor => end_label::postprocess(graph),
            Self::EndLabelSorter => end_label::sort_labels(graph),
            Self::ReversedEdgeRestorer => reversed_edge_restorer::restore(graph),
            Self::HierarchicalPortOrthogonalEdgeRouter =>
                hierarchical_port_edge_router::route(graph),
            Self::HierarchicalNodeResizer => hierarchical_node_resizer::resize(graph),
            Self::FinalSplineBendpointsCalculator => final_spline_bendpoints::calculate(graph),
            Self::SelfLoopRouter => self_loop_router::route(graph),
            Self::SelfLoopPortRestorer => self_loop_port_restorer::restore(graph),
            #[cfg(feature = "devtools")]
            Self::ConstraintsPostprocessor => constraints_postprocessor::process(graph),
            Self::CommentPostprocessor => comment::postprocess(graph),
            Self::BreakingPointRemover => breaking_point::remove(graph),
            Self::DirectionPostprocessor => direction::postprocess(graph),
            #[cfg(feature = "devtools")]
            Self::CompoundGraphPostprocessor => compound_graph::postprocess(graph),
        }
    }
}

fn run_layer_sweep_cross_min(
    graph: &mut LGraph,
    cross_min_type: crate::p3_crossing_min::layer_sweep::CrossMinType,
) {
    let mut rng = graph.take_rng();
    crate::p3_crossing_min::layer_sweep::minimize_crossings_with_graph_rngs(
        graph,
        cross_min_type,
        &mut rng,
    );
    graph.put_rng(rng);
}
