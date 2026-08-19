//! Shared Gansner network simplex solver used by P2 layer
//! assignment, P4 node placement, and (eventually) intermediate compaction.
//! Each caller builds an [`NGraph`] over its own domain data and drives a
//! [`Solver`] through its builder API.

mod graph;
mod solver;

pub use graph::NGraph;
pub use solver::Solver;
