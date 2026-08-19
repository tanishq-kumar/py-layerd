//! Thin re-export front for the wrapping-based breaking-point processors.
//!
//! The real implementation lives in `intermediate::wrapping::*`. This module
//! exists to keep the flat `IntermediateProcessorId` dispatch stable.

use crate::graph::LGraph;

/// Inserts breaking-point dummies after phase 2.
pub fn insert(graph: &mut LGraph) {
    super::wrapping::insert_breaking_points(graph);
}

/// Reroutes breaking-point dummy chains before phase 4.
pub fn process(graph: &mut LGraph) {
    super::wrapping::process_breaking_points(graph);
}

/// Removes breaking-point dummies after phase 5.
pub fn remove(graph: &mut LGraph) {
    super::wrapping::remove_breaking_points(graph);
}
