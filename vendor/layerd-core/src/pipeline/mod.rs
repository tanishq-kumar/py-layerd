pub mod configurator;
pub mod stage_metadata;

use crate::{graph::LGraph, intermediate::IntermediateProcessorId};

/// Identifies which phase slot a pipeline stage belongs to.
///
/// Used by test helpers (`run_pipeline_up_to_phase_exclusive`,
/// `run_up_to_phase`) for phase-level stop conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PhaseSlot {
    P1CycleBreaking,
    P2Layering,
    P3NodeOrdering,
    P4NodePlacement,
    P5EdgeRouting,
}

/// A single step in the assembled layout pipeline.
///
/// Each variant dispatches to a free function via `run()`, with zero
/// heap allocation and no vtable indirection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineStage {
    Phase(PhaseImpl),
    Intermediate(IntermediateProcessorId),
}

impl PipelineStage {
    /// Execute this pipeline stage on the graph.
    pub fn run(self, graph: &mut LGraph) {
        match self {
            Self::Phase(phase) => phase.run(graph),
            Self::Intermediate(id) => id.run(graph),
        }
    }

    /// Return the phase slot if this is a phase stage, `None` for intermediates.
    ///
    /// Used by test helpers like `run_pipeline_up_to_phase_exclusive` to
    /// stop iteration before a given phase slot, regardless of which
    /// concrete `PhaseImpl` was configured.
    #[cfg(feature = "devtools")]
    pub fn phase_slot(self) -> Option<PhaseSlot> {
        match self {
            Self::Phase(phase) => Some(phase.slot()),
            Self::Intermediate(_) => None,
        }
    }

    /// Stable stage identifier used by diagnostic dumps.
    #[cfg(feature = "devtools")]
    pub fn stage_id(self) -> &'static str {
        match self {
            Self::Phase(phase) => phase.stage_id(),
            Self::Intermediate(id) => id.stage_id(),
        }
    }

    /// `"Phase"` or `"Intermediate"` — used in dumps to disambiguate cases
    /// where a phase and an intermediate share a class simple name.
    #[cfg(feature = "devtools")]
    pub fn stage_kind(self) -> &'static str {
        match self {
            Self::Phase(_) => "Phase",
            Self::Intermediate(_) => "Intermediate",
        }
    }
}

/// Concrete phase algorithm choice, reified from the strategy enums.
///
/// Only strategies that have existing `impl Processor` in `core/src/p1..p5/`
/// become variants. Strategies that `todo!()` at the configurator dispatch
/// point stay as configurator-level `todo!()`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseImpl {
    // P1 cycle breaking
    P1Greedy,
    P1DepthFirst,
    P1ModelOrder,
    P1GreedyModelOrder,
    P1Interactive,
    P1BfsNodeOrder,
    P1DfsNodeOrder,
    P1SccConnectivity,
    P1SccNodeType,
    // P2 layering
    P2NetworkSimplex,
    P2LongestPath,
    P2LongestPathSource,
    P2CoffmanGraham,
    P2Interactive,
    P2MinWidth,
    P2StretchWidth,
    P2BfModelOrder,
    P2DfModelOrder,
    // P3 crossing minimization
    P3LayerSweep,
    P3NoCrossing,
    P3Interactive,
    // P4 node placement
    P4BrandesKoepf,
    P4Simple,
    P4LinearSegments,
    P4NetworkSimplex,
    P4Interactive,
    // P5 edge routing
    P5Orthogonal,
    P5Polyline,
    P5Splines,
}

impl PhaseImpl {
    /// Return the phase slot this implementation runs in.
    #[cfg(feature = "devtools")]
    pub fn slot(self) -> PhaseSlot {
        match self {
            Self::P1Greedy
            | Self::P1DepthFirst
            | Self::P1ModelOrder
            | Self::P1GreedyModelOrder
            | Self::P1Interactive
            | Self::P1BfsNodeOrder
            | Self::P1DfsNodeOrder
            | Self::P1SccConnectivity
            | Self::P1SccNodeType => PhaseSlot::P1CycleBreaking,
            Self::P2NetworkSimplex
            | Self::P2LongestPath
            | Self::P2LongestPathSource
            | Self::P2CoffmanGraham
            | Self::P2Interactive
            | Self::P2MinWidth
            | Self::P2StretchWidth
            | Self::P2BfModelOrder
            | Self::P2DfModelOrder => PhaseSlot::P2Layering,
            Self::P3LayerSweep | Self::P3NoCrossing | Self::P3Interactive =>
                PhaseSlot::P3NodeOrdering,
            Self::P4BrandesKoepf
            | Self::P4Simple
            | Self::P4LinearSegments
            | Self::P4NetworkSimplex
            | Self::P4Interactive => PhaseSlot::P4NodePlacement,
            Self::P5Orthogonal | Self::P5Polyline | Self::P5Splines => PhaseSlot::P5EdgeRouting,
        }
    }

    /// Stable diagnostic stage id emitted for this phase implementation.
    #[cfg(feature = "devtools")]
    pub fn stage_id(self) -> &'static str {
        match self {
            Self::P1Greedy => "GreedyCycleBreaker",
            Self::P1DepthFirst => "DepthFirstCycleBreaker",
            Self::P1ModelOrder => "ModelOrderCycleBreaker",
            Self::P1GreedyModelOrder => "GreedyModelOrderCycleBreaker",
            Self::P1Interactive => "InteractiveCycleBreaker",
            Self::P1BfsNodeOrder => "BFSNodeOrderCycleBreaker",
            Self::P1DfsNodeOrder => "DFSNodeOrderCycleBreaker",
            Self::P1SccConnectivity => "SCConnectivity",
            Self::P1SccNodeType => "SCCNodeTypeCycleBreaker",
            Self::P2NetworkSimplex => "NetworkSimplexLayerer",
            Self::P2LongestPath => "LongestPathLayerer",
            Self::P2LongestPathSource => "LongestPathSourceLayerer",
            Self::P2CoffmanGraham => "CoffmanGrahamLayerer",
            Self::P2Interactive => "InteractiveLayerer",
            Self::P2MinWidth => "MinWidthLayerer",
            Self::P2StretchWidth => "StretchWidthLayerer",
            Self::P2BfModelOrder => "BreadthFirstModelOrderLayerer",
            Self::P2DfModelOrder => "DepthFirstModelOrderLayerer",
            Self::P3LayerSweep => "LayerSweepCrossingMinimizer",
            Self::P3NoCrossing => "NoCrossingMinimizer",
            Self::P3Interactive => "InteractiveCrossingMinimizer",
            Self::P4BrandesKoepf => "BKNodePlacer",
            Self::P4Simple => "SimpleNodePlacer",
            Self::P4LinearSegments => "LinearSegmentsNodePlacer",
            Self::P4NetworkSimplex => "NetworkSimplexPlacer",
            Self::P4Interactive => "InteractiveNodePlacer",
            Self::P5Orthogonal => "OrthogonalEdgeRouter",
            Self::P5Polyline => "PolylineEdgeRouter",
            Self::P5Splines => "SplineEdgeRouter",
        }
    }

    /// Execute the phase algorithm by dispatching to the corresponding free function.
    pub fn run(self, graph: &mut LGraph) {
        use crate::{
            p1_cycle_breaking as p1, p2_layering as p2, p3_crossing_min as p3,
            p4_node_placement as p4, p5_edge_routing as p5,
        };
        match self {
            Self::P1Greedy => p1::greedy::break_cycles(graph),
            Self::P1DepthFirst => p1::depth_first::break_cycles(graph),
            Self::P1ModelOrder => p1::model_order::break_cycles(graph),
            Self::P1GreedyModelOrder => p1::greedy_model_order::break_cycles(graph),
            Self::P1Interactive => p1::interactive::break_cycles(graph),
            Self::P1BfsNodeOrder => p1::bfs_node_order::break_cycles(graph),
            Self::P1DfsNodeOrder => p1::dfs_node_order::break_cycles(graph),
            Self::P1SccConnectivity => p1::scc_connectivity::break_cycles(graph),
            Self::P1SccNodeType => p1::scc_node_type::break_cycles(graph),
            Self::P2NetworkSimplex => p2::network_simplex::assign_layers(graph),
            Self::P2LongestPath => p2::longest_path::assign_layers(graph),
            Self::P2LongestPathSource => p2::longest_path_source::assign_layers(graph),
            Self::P2CoffmanGraham => p2::coffman_graham::assign_layers(graph),
            Self::P2Interactive => p2::interactive::assign_layers(graph),
            Self::P2MinWidth => p2::min_width::assign_layers(graph),
            Self::P2StretchWidth => p2::stretch_width::assign_layers(graph),
            Self::P2BfModelOrder => p2::bf_model_order::assign_layers(graph),
            Self::P2DfModelOrder => p2::df_model_order::assign_layers(graph),
            Self::P3LayerSweep => p3::layer_sweep::minimize_crossings(graph),
            Self::P3NoCrossing => p3::no_crossing_minimizer::minimize_crossings(graph),
            Self::P3Interactive => p3::interactive_crossing_minimizer::minimize_crossings(graph),
            Self::P4BrandesKoepf => p4::bk::place_nodes(graph),
            Self::P4Simple => p4::simple::place_nodes(graph),
            Self::P4LinearSegments => p4::linear_segments::place_nodes(graph),
            Self::P4NetworkSimplex => p4::network_simplex_placer::place_nodes(graph),
            Self::P4Interactive => p4::interactive::place_nodes(graph),
            Self::P5Orthogonal => p5::orthogonal::route_edges(graph),
            Self::P5Polyline => p5::polyline::route_edges(graph),
            Self::P5Splines => p5::splines::route_edges(graph),
        }
    }
}
