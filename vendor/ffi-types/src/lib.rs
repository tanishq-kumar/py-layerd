//! Shared FFI layer for layerd bindings.
//!
//! Defines the LRD1 binary wire format and provides a single `layout_bytes`
//! entry point consumed by platform-specific binding crates.

mod decode;
mod encode;
mod layout;
mod wire;

pub use layout::{
    FfiError, FlatLayoutOutput, LayoutProfile, ProfiledLayout, layout_bytes, layout_bytes_profiled,
    layout_flat,
};
pub use wire::{
    BEND_RECORD_SIZE, EDGE_RECORD_SIZE, FLAG_IS_OUTPUT, HEADER_SIZE, MAGIC, MAX_BEND_COUNT,
    MAX_EDGE_COUNT, MAX_NODE_COUNT, NODE_RECORD_SIZE, VERSION,
};
