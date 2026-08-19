//! Side selector for greedy-switch crossing counts.
//!
//! Extracted into its own module so `SwitchDecider` and crossing-count helpers
//! can share the enum without forming a cross-module dependency cycle.

/// Which side of the free layer to count crossings on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossingCountSide {
    /// Count crossings on the west (left) side of the free layer.
    West,
    /// Count crossings on the east (right) side of the free layer.
    East,
}
