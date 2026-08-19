//! Panic-safe `layout_bytes` entry point.

use std::panic::{AssertUnwindSafe, catch_unwind};

use web_time::{Duration, Instant};

use crate::{decode::decode, encode::encode};

/// Errors reported across the FFI boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FfiError {
    /// Buffer magic bytes do not match "LRD1".
    InvalidMagic,
    /// Buffer declares a wire version the current build cannot read.
    UnsupportedVersion(u32),
    /// Node, edge, or bend count exceeds the allowed maximum.
    TooLarge { kind: &'static str, count: u32 },
    /// Buffer is shorter than the declared counts require.
    BufferTooShort { expected: usize, actual: usize },
    /// An edge references a node id that is not present in the NODE section.
    InvalidNodeReference { edge_id: u32, node_id: u32 },
    /// Two nodes or edges share the same caller-assigned id.
    DuplicateId { kind: &'static str, id: u32 },
    /// A node's width, height, x, or y was NaN or infinite. The wire format
    /// requires finite f64 geometry; encoders are expected to reject this
    /// at encode time, but the decoder defends against malformed inputs.
    InvalidGeometry { node_id: u32 },
    /// The underlying `layerd::layout` pipeline panicked. The FFI caught the
    /// panic to avoid cross-FFI undefined behavior.
    LayoutPanic,
}

impl std::fmt::Display for FfiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FfiError::InvalidMagic => write!(f, "invalid magic bytes (expected LRD1)"),
            FfiError::UnsupportedVersion(v) => write!(f, "unsupported wire version: {}", v),
            FfiError::TooLarge { kind, count } => {
                write!(f, "{} count {} exceeds maximum", kind, count)
            }
            FfiError::BufferTooShort { expected, actual } => {
                write!(f, "buffer too short: expected {} bytes, got {}", expected, actual)
            }
            FfiError::InvalidNodeReference { edge_id, node_id } => {
                write!(f, "edge {} references unknown node {}", edge_id, node_id)
            }
            FfiError::DuplicateId { kind, id } => write!(f, "duplicate {} id: {}", kind, id),
            FfiError::InvalidGeometry { node_id } => {
                write!(f, "node {} has non-finite width, height, x, or y", node_id)
            }
            FfiError::LayoutPanic => write!(f, "layerd pipeline panicked"),
        }
    }
}

impl std::error::Error for FfiError {}

/// Internal timing breakdown for one `layout_bytes_profiled` call.
///
/// This is intended for benchmark tooling. The production `layout_bytes`
/// entry point intentionally keeps its current behavior.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LayoutProfile {
    pub decode: Duration,
    pub layout: Duration,
    pub encode: Duration,
}

#[derive(Debug)]
pub struct ProfiledLayout {
    pub bytes: Vec<u8>,
    pub profile: LayoutProfile,
}

#[derive(Debug)]
pub struct FlatLayoutOutput {
    pub width: f64,
    pub height: f64,
    pub node_wire_ids: Vec<u32>,
    pub node_width: Vec<f64>,
    pub node_height: Vec<f64>,
    pub node_x: Vec<f64>,
    pub node_y: Vec<f64>,
    pub edge_wire_ids: Vec<u32>,
    pub edge_source_wire_ids: Vec<u32>,
    pub edge_target_wire_ids: Vec<u32>,
    pub edge_bend_start: Vec<u32>,
    pub edge_bend_length: Vec<u32>,
    pub bend_x: Vec<f64>,
    pub bend_y: Vec<f64>,
}

/// Decodes LRD1 input bytes, runs the layerd pipeline, and encodes the result.
///
/// The layout and encode phases are wrapped in `catch_unwind` so that panics
/// from unimplemented or buggy pipeline stages do not cross the FFI boundary.
/// `encode` itself returns a `Result` so it can enforce `MAX_BEND_COUNT`
/// against pipeline failures that produce runaway bend counts.
pub fn layout_bytes(input: &[u8]) -> Result<Vec<u8>, FfiError> {
    let mut ctx = decode(input)?;

    // Short-circuit empty graphs to avoid exercising potentially unstable
    // processor code paths on zero nodes.
    if ctx.node_arena_ids.is_empty() {
        return encode(&ctx);
    }

    catch_unwind(AssertUnwindSafe(move || {
        layerd::layout(&mut ctx.graph);
        encode(&ctx)
    }))
    .unwrap_or_else(|_| Err(FfiError::LayoutPanic))
}

/// Decodes LRD1 input bytes, runs the layerd pipeline, and returns flat arrays.
///
/// This entry point is intended for bindings that can expose primitive arrays
/// directly and want to avoid allocating and reparsing an output LRD1 byte
/// buffer on the host side.
pub fn layout_flat(input: &[u8]) -> Result<FlatLayoutOutput, FfiError> {
    let mut ctx = decode(input)?;

    if ctx.node_arena_ids.is_empty() {
        return flatten_layout(&ctx);
    }

    catch_unwind(AssertUnwindSafe(move || {
        layerd::layout(&mut ctx.graph);
        flatten_layout(&ctx)
    }))
    .unwrap_or_else(|_| Err(FfiError::LayoutPanic))
}

/// Profiled variant of [`layout_bytes`] for native benchmark tooling.
pub fn layout_bytes_profiled(input: &[u8]) -> Result<ProfiledLayout, FfiError> {
    let decode_start = Instant::now();
    let mut ctx = decode(input)?;
    let decode_time = decode_start.elapsed();

    if ctx.node_arena_ids.is_empty() {
        let encode_start = Instant::now();
        let bytes = encode(&ctx)?;
        return Ok(ProfiledLayout {
            bytes,
            profile: LayoutProfile {
                decode: decode_time,
                layout: Duration::ZERO,
                encode: encode_start.elapsed(),
            },
        });
    }

    catch_unwind(AssertUnwindSafe(move || {
        let layout_start = Instant::now();
        layerd::layout(&mut ctx.graph);
        let layout_time = layout_start.elapsed();

        let encode_start = Instant::now();
        let bytes = encode(&ctx)?;
        let encode_time = encode_start.elapsed();

        Ok(ProfiledLayout {
            bytes,
            profile: LayoutProfile {
                decode: decode_time,
                layout: layout_time,
                encode: encode_time,
            },
        })
    }))
    .unwrap_or_else(|_| Err(FfiError::LayoutPanic))
}

fn flatten_layout(ctx: &crate::decode::LayoutContext) -> Result<FlatLayoutOutput, FfiError> {
    let n_nodes = ctx.node_arena_ids.len();
    let n_edges = ctx.edge_arena_ids.len();

    let mut out = FlatLayoutOutput {
        width: ctx.graph.size.x,
        height: ctx.graph.size.y,
        node_wire_ids: Vec::with_capacity(n_nodes),
        node_width: Vec::with_capacity(n_nodes),
        node_height: Vec::with_capacity(n_nodes),
        node_x: Vec::with_capacity(n_nodes),
        node_y: Vec::with_capacity(n_nodes),
        edge_wire_ids: Vec::with_capacity(n_edges),
        edge_source_wire_ids: Vec::with_capacity(n_edges),
        edge_target_wire_ids: Vec::with_capacity(n_edges),
        edge_bend_start: Vec::with_capacity(n_edges),
        edge_bend_length: Vec::with_capacity(n_edges),
        bend_x: Vec::new(),
        bend_y: Vec::new(),
    };

    for i in 0..n_nodes {
        out.node_wire_ids.push(ctx.node_caller_ids[i]);
        if let Some(node) = ctx.graph.try_node(ctx.node_arena_ids[i]) {
            out.node_width.push(node.size.x);
            out.node_height.push(node.size.y);
            out.node_x.push(node.position.x);
            out.node_y.push(node.position.y);
        } else {
            out.node_width.push(ctx.node_widths[i]);
            out.node_height.push(ctx.node_heights[i]);
            out.node_x.push(0.0);
            out.node_y.push(0.0);
        }
    }

    let bend_limit = crate::MAX_BEND_COUNT as usize;
    for i in 0..n_edges {
        let bend_start = out.bend_x.len();
        if let Some(edge) = ctx.graph.try_edge(ctx.edge_arena_ids[i]) {
            for p in &edge.bend_points {
                if out.bend_x.len() >= bend_limit {
                    return Err(FfiError::TooLarge { kind: "bend", count: crate::MAX_BEND_COUNT });
                }
                out.bend_x.push(p.x);
                out.bend_y.push(p.y);
            }
        }
        let bend_length = out.bend_x.len() - bend_start;

        out.edge_wire_ids.push(ctx.edge_caller_ids[i]);
        out.edge_source_wire_ids.push(ctx.edge_source_callers[i]);
        out.edge_target_wire_ids.push(ctx.edge_target_callers[i]);
        out.edge_bend_start.push(
            bend_start
                .try_into()
                .map_err(|_| FfiError::TooLarge { kind: "bend", count: crate::MAX_BEND_COUNT })?,
        );
        out.edge_bend_length.push(
            bend_length
                .try_into()
                .map_err(|_| FfiError::TooLarge { kind: "bend", count: crate::MAX_BEND_COUNT })?,
        );
    }

    Ok(out)
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<FfiError>();
    }
}
