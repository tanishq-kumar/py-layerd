//! Null-object crossing minimizer.
//!
//! A null-object phase that preserves the existing node and port order.
//! Intermediate processor dependencies (long edge splitter / joiner,
//! in-layer constraint processor, port list sorter) are scheduled
//! unconditionally by `pipeline::configurator`, so the phase body itself
//! is a no-op.

use crate::graph::LGraph;

/// Do nothing. Node and port order remain as they were before P3.
pub fn minimize_crossings(_graph: &mut LGraph) {}
