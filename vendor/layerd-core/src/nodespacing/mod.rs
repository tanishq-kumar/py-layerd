//! Cell-system based node spacing infrastructure.
//!
//! This module provides the building blocks that `NodeDimensionCalculation`
//! and port placement use to describe a node's internal layout (padding,
//! label cells, port placement slots) without committing to a particular
//! graph type.
//!
//! Only the pieces required by the layered pipeline are implemented;
//! adapter plumbing and hierarchical helpers used by other layout algorithms
//! are intentionally out of scope.

pub mod cell_system;
