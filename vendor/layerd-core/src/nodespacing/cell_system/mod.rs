//! Cell-system primitives for node-size and label-placement calculation.
//!
//! A cell represents a rectangular region inside a node with a minimum size
//! and a padding. Container cells arrange other cells as strips or 3x3 grids
//! and propagate minimum sizes upward. The implementation uses a static enum
//! (`CellKind`) instead of a trait object so that iteration stays zero-cost.

mod atomic_cell;
mod cell;
mod container_cell;
mod label_cell;

pub use atomic_cell::AtomicCell;
pub use cell::{
    CellKind, ContainerArea, HorizontalLabelAlignment, Rect, Strip, VerticalLabelAlignment,
};
pub use container_cell::{GridContainerCell, StripContainerCell};
pub use label_cell::LabelCell;
