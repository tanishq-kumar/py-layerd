//! LRD1 wire format constants.
//!
//! Every buffer starts with a 40-byte header, followed by node records,
//! edge records, and (on output) bend point records. All integers and floats
//! are little-endian. Coordinates and dimensions use `f64` values; record
//! offsets are arranged so every `f64` falls on an 8-byte alignment boundary,
//! allowing zero-copy reads on platforms that mmap the buffer directly.
//!
//! # Current layout
//!
//! ## Header (40 bytes)
//! | offset | size | field         |
//! |--------|------|---------------|
//! |   0    |  4   | MAGIC ("LRD1")|
//! |   4    |  4   | VERSION (u32) |
//! |   8    |  4   | FLAGS (u32)   |
//! |  12    |  4   | NODE_COUNT    |
//! |  16    |  4   | EDGE_COUNT    |
//! |  20    |  4   | BEND_COUNT    |
//! |  24    |  8   | GRAPH_WIDTH (f64) |
//! |  32    |  8   | GRAPH_HEIGHT (f64)|
//!
//! ## Node record (40 bytes)
//! | offset | size | field         |
//! |--------|------|---------------|
//! |   0    |  4   | ID (u32)      |
//! |   4    |  4   | reserved      |
//! |   8    |  8   | WIDTH (f64)   |
//! |  16    |  8   | HEIGHT (f64)  |
//! |  24    |  8   | X (f64)       |
//! |  32    |  8   | Y (f64)       |
//!
//! ## Edge record (24 bytes)
//! | offset | size | field         |
//! |--------|------|---------------|
//! |   0    |  4   | ID            |
//! |   4    |  4   | SOURCE_NODE   |
//! |   8    |  4   | TARGET_NODE   |
//! |  12    |  4   | BEND_START    |
//! |  16    |  4   | BEND_LENGTH   |
//! |  20    |  4   | reserved      |
//!
//! ## Bend point record (16 bytes)
//! | offset | size | field         |
//! |--------|------|---------------|
//! |   0    |  8   | X (f64)       |
//! |   8    |  8   | Y (f64)       |
//!
//! # Cross-codec contract
//!
//! The same constants, field offsets, and layout rules are reimplemented
//! in two other places:
//!
//!   - JS/TS: `wasm/sdk/src/index.ts`
//!   - Swift: `ios/swift/Sources/Layerd/LRD1Codec.swift`
//!
//! Any change to `MAGIC`, `VERSION`, `HEADER_SIZE`, record sizes, `MAX_*`
//! limits, or the field offsets below MUST be synchronized across all three
//! codec implementations in the same change. The Rust wire tests validate
//! against `testdata/v1_canonical.bin`.

/// Magic bytes at the start of every LRD1 buffer.
pub const MAGIC: &[u8; 4] = b"LRD1";

/// Current wire format version.
pub const VERSION: u32 = 1;

/// Flag bit set on output buffers to indicate laid-out data is present.
pub const FLAG_IS_OUTPUT: u32 = 1 << 0;

/// Maximum allowed node count per graph.
pub const MAX_NODE_COUNT: u32 = 1_000_000;

/// Maximum allowed edge count per graph.
pub const MAX_EDGE_COUNT: u32 = 2_000_000;

/// Maximum allowed total bend point count per graph.
pub const MAX_BEND_COUNT: u32 = 10_000_000;

/// Size of the fixed header in bytes.
pub const HEADER_SIZE: usize = 40;

/// Size of a single node record in bytes.
pub const NODE_RECORD_SIZE: usize = 40;

/// Size of a single edge record in bytes.
pub const EDGE_RECORD_SIZE: usize = 24;

/// Size of a single bend point record in bytes.
pub const BEND_RECORD_SIZE: usize = 16;

/// Field offsets within the 40-byte header.
pub(crate) mod header_offset {
    pub const MAGIC: usize = 0;
    pub const VERSION: usize = 4;
    pub const FLAGS: usize = 8;
    pub const NODE_COUNT: usize = 12;
    pub const EDGE_COUNT: usize = 16;
    pub const BEND_COUNT: usize = 20;
    pub const GRAPH_WIDTH: usize = 24;
    pub const GRAPH_HEIGHT: usize = 32;
}

/// Field offsets within a 40-byte node record.
pub(crate) mod node_offset {
    pub const ID: usize = 0;
    // 4..8 reserved padding so subsequent f64 fields are 8-byte aligned.
    pub const WIDTH: usize = 8;
    pub const HEIGHT: usize = 16;
    pub const X: usize = 24;
    pub const Y: usize = 32;
}

/// Field offsets within a 24-byte edge record.
pub(crate) mod edge_offset {
    pub const ID: usize = 0;
    pub const SOURCE_NODE: usize = 4;
    pub const TARGET_NODE: usize = 8;
    pub const BEND_START: usize = 12;
    pub const BEND_LENGTH: usize = 16;
    // 20..24 reserved for future use
}

/// Field offsets within a 16-byte bend point record.
pub(crate) mod bend_offset {
    pub const X: usize = 0;
    pub const Y: usize = 8;
}
