//! Pipeline-stage metadata: slot mapping + intra-slot declaration ordinal.
//!
//! Each intermediate processor has a stable declaration ordinal that
//! determines the run order within a single slot; intra-slot ordering is
//! the tie-breaker when several `add_before(P_X, ...)` / `add_after(P_X, ...)`
//! calls land in the same slot.
//!
//! The slot itself is determined dynamically at the call site by
//! `add_before(P_X, ...)` / `add_after(P_X, ...)`.
//!
//! [`PipelineBuilder`] lets callers describe what should run *relative to a
//! phase*, and produces the flat phase sequence with intra-slot processors
//! sorted by ordinal.

use super::{PhaseImpl, PipelineStage};
use crate::{intermediate::IntermediateProcessorId, pipeline::PhaseSlot};

/// Slot in the layered pipeline. Each slot sits adjacent to one or two phases
/// (e.g. `Slot::BeforeP3` is the slot between P2 and P3 = `add_after(P2)` =
/// `add_before(P3)` from a caller's perspective).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Slot {
    BeforeP1 = 0,
    BeforeP2 = 1,
    BeforeP3 = 2,
    BeforeP4 = 3,
    BeforeP5 = 4,
    AfterP5 = 5,
}

impl Slot {
    pub const ALL: [Slot; 6] = [
        Slot::BeforeP1,
        Slot::BeforeP2,
        Slot::BeforeP3,
        Slot::BeforeP4,
        Slot::BeforeP5,
        Slot::AfterP5,
    ];
}

/// Declaration ordinal used as the intra-slot tie-breaker.
///
/// Internal processors that are not emitted by the default configurator
/// receive synthetic ordinals past the standard range so they stay
/// deterministic if a builder ever adds them. Compound graph handling runs
/// outside the per-graph pipeline, and greedy switching is split into
/// one-sided and two-sided passes.
pub fn ordinal_of(id: IntermediateProcessorId) -> u32 {
    use IntermediateProcessorId as I;
    match id {
        // Before Phase 1
        I::DirectionPreprocessor => 0,
        I::CommentPreprocessor => 1,
        I::EdgeAndLayerConstraintEdgeReverser => 2,
        I::InteractiveExternalPortPositioner => 3,
        I::PartitionPreprocessor => 4,
        // Before Phase 2
        I::LabelDummyInserter => 5,
        I::SelfLoopPreProcessor => 6,
        I::LayerConstraintPreprocessor => 7,
        I::PartitionMidprocessor => 8,
        // Before Phase 3
        I::HighDegreeNodeLayeringProcessor => 9,
        I::NodePromotion => 10,
        I::LayerConstraintPostprocessor => 11,
        I::PartitionPostprocessor => 12,
        I::HierarchicalPortConstraintProcessor => 13,
        I::SemiInteractiveCrossMinProcessor => 14,
        I::BreakingPointInserter => 15,
        I::LongEdgeSplitter => 16,
        I::PortSideProcessor => 17,
        I::InvertedPortProcessor => 18,
        I::PortListSorter => 19,
        I::SortByInputModelProcessor => 20,
        I::NorthSouthPortPreprocessor => 21,
        // Before Phase 4
        I::BreakingPointProcessor => 22,
        I::OneSidedGreedySwitch => 23,
        I::TwoSidedGreedySwitch => 24,
        I::SelfLoopPortRestorer => 25,
        I::AlternatingLayerUnzipper => 26,
        I::SingleEdgeGraphWrapper => 27,
        I::InLayerConstraintProcessor => 28,
        I::EndNodePortLabelManagementProcessor => 29,
        I::LabelAndNodeSizeProcessor => 30,
        I::InnermostNodeMarginCalculator => 31,
        I::SelfLoopRouter => 32,
        I::CommentNodeMarginCalculator => 33,
        I::EndLabelPreprocessor => 34,
        I::LabelDummySwitcher => 35,
        I::CenterLabelManagementProcessor => 36,
        I::LabelSideSelector => 37,
        I::HyperedgeDummyMerger => 38,
        I::HierarchicalPortDummySizeProcessor => 39,
        // Before Phase 5
        I::LayerSizeAndGraphHeightCalculator => 40,
        I::HierarchicalPortPositionProcessor => 41,
        // After Phase 5
        #[cfg(feature = "devtools")]
        I::ConstraintsPostprocessor => 42,
        I::CommentPostprocessor => 43,
        I::HypernodesProcessor => 44,
        I::HierarchicalPortOrthogonalEdgeRouter => 45,
        I::LongEdgeJoiner => 46,
        I::SelfLoopPostProcessor => 47,
        I::BreakingPointRemover => 48,
        I::NorthSouthPortPostprocessor => 49,
        I::HorizontalCompactor => 50,
        I::LabelDummyRemover => 51,
        I::FinalSplineBendpointsCalculator => 52,
        I::EndLabelSorter => 53,
        I::ReversedEdgeRestorer => 54,
        I::EndLabelPostprocessor => 55,
        I::HierarchicalNodeResizer => 56,
        I::DirectionPostprocessor => 57,
        // Rust-only / not added by the configurator path. Synthetic ordinals
        // past the standard range so any accidental insertion stays
        // deterministic.
        #[cfg(feature = "devtools")]
        I::CompoundGraphPreprocessor => 1000,
        #[cfg(feature = "devtools")]
        I::CompoundGraphPostprocessor => 1001,
        #[cfg(feature = "devtools")]
        I::DummySelfLoopProcessor => 1002,
        #[cfg(feature = "devtools")]
        I::GreedySwitch => 1003,
        #[cfg(feature = "devtools")]
        I::ModelOrderPortOrderer => 1004,
    }
}

/// Builds the layered pipeline declaratively.
///
/// Callers pick phase implementations up-front (`new`), describe their
/// processor additions relative to phases (driven by graph properties +
/// per-strategy additions), and finally call [`PipelineBuilder::build`] to
/// flatten the configuration into the final `Vec<PipelineStage>`. Ordering
/// within each slot is determined by declaration ordinal.
pub struct PipelineBuilder {
    by_slot: [Vec<(u32, IntermediateProcessorId)>; 6],
    p1: PhaseImpl,
    p2: PhaseImpl,
    p3: PhaseImpl,
    p4: PhaseImpl,
    p5: PhaseImpl,
}

impl PipelineBuilder {
    pub fn new(p1: PhaseImpl, p2: PhaseImpl, p3: PhaseImpl, p4: PhaseImpl, p5: PhaseImpl) -> Self {
        Self { by_slot: Default::default(), p1, p2, p3, p4, p5 }
    }

    /// Add `id` to the slot just before `phase`. `phase` is the phase the
    /// processor should run before (e.g. `PhaseSlot::P1CycleBreaking` →
    /// `Slot::BeforeP1`). Duplicate adds are allowed; deduplication is
    /// performed by the builder using the processor id.
    pub fn add_before(&mut self, phase: PhaseSlot, id: IntermediateProcessorId) {
        self.add_to(slot_before(phase), id);
    }

    /// Add `id` to the slot just after `phase`. `phase` is the phase the
    /// processor should run after (e.g. `PhaseSlot::P5EdgeRouting` →
    /// `Slot::AfterP5`).
    pub fn add_after(&mut self, phase: PhaseSlot, id: IntermediateProcessorId) {
        self.add_to(slot_after(phase), id);
    }

    /// Add a processor directly to a named slot. Useful when the call site
    /// does not phrase its dependency in terms of `add_before` / `add_after`.
    pub fn add_to(&mut self, slot: Slot, id: IntermediateProcessorId) {
        self.by_slot[slot as usize].push((ordinal_of(id), id));
    }

    /// Flattens the configuration into a sequence of pipeline stages.
    ///
    /// Each slot's processors are sorted by ordinal and de-duplicated by id
    /// (idempotent `add_before` calls do not produce duplicate stages).
    pub fn build(mut self) -> Vec<PipelineStage> {
        let mut pipeline: Vec<PipelineStage> = Vec::new();
        for slot in Slot::ALL {
            let mut entries = std::mem::take(&mut self.by_slot[slot as usize]);
            entries.sort_by_key(|&(ord, _)| ord);
            // Dedup by id while preserving the sorted ordinal order.
            let mut seen = std::collections::HashSet::new();
            entries.retain(|&(_, id)| seen.insert(id));
            entries.sort_by_key(|&(ord, _)| ord);
            for (_, id) in entries {
                pipeline.push(PipelineStage::Intermediate(id));
            }
            if let Some(phase) = phase_after_slot(slot, &self) {
                pipeline.push(PipelineStage::Phase(phase));
            }
        }
        pipeline
    }
}

fn slot_before(phase: PhaseSlot) -> Slot {
    match phase {
        PhaseSlot::P1CycleBreaking => Slot::BeforeP1,
        PhaseSlot::P2Layering => Slot::BeforeP2,
        PhaseSlot::P3NodeOrdering => Slot::BeforeP3,
        PhaseSlot::P4NodePlacement => Slot::BeforeP4,
        PhaseSlot::P5EdgeRouting => Slot::BeforeP5,
    }
}

fn slot_after(phase: PhaseSlot) -> Slot {
    match phase {
        PhaseSlot::P1CycleBreaking => Slot::BeforeP2,
        PhaseSlot::P2Layering => Slot::BeforeP3,
        PhaseSlot::P3NodeOrdering => Slot::BeforeP4,
        PhaseSlot::P4NodePlacement => Slot::BeforeP5,
        PhaseSlot::P5EdgeRouting => Slot::AfterP5,
    }
}

/// `Some(phase)` if a phase runs immediately after `slot`. `Slot::AfterP5`
/// has no successor phase and returns `None`.
fn phase_after_slot(slot: Slot, builder: &PipelineBuilder) -> Option<PhaseImpl> {
    match slot {
        Slot::BeforeP1 => Some(builder.p1),
        Slot::BeforeP2 => Some(builder.p2),
        Slot::BeforeP3 => Some(builder.p3),
        Slot::BeforeP4 => Some(builder.p4),
        Slot::BeforeP5 => Some(builder.p5),
        Slot::AfterP5 => None,
    }
}
