//! LRD1 output encoder: laid-out `LGraph` into a byte buffer.

use crate::{
    decode::LayoutContext,
    layout::FfiError,
    wire::{
        BEND_RECORD_SIZE, EDGE_RECORD_SIZE, FLAG_IS_OUTPUT, HEADER_SIZE, MAGIC, MAX_BEND_COUNT,
        NODE_RECORD_SIZE, VERSION, bend_offset, edge_offset, header_offset, node_offset,
    },
};

/// Serializes a laid-out graph into an LRD1 output buffer.
///
/// Enforces `MAX_BEND_COUNT` incrementally during bend collection so that
/// a runaway pipeline (or a buggy edge router) producing unbounded bend
/// points fails fast with `FfiError::TooLarge` instead of exhausting memory.
pub(crate) fn encode(ctx: &LayoutContext) -> Result<Vec<u8>, FfiError> {
    let n_nodes = ctx.node_arena_ids.len();
    let n_edges = ctx.edge_arena_ids.len();

    // First pass: collect bend points per edge into a flat f64 list,
    // hard-capped at MAX_BEND_COUNT to guard against pipeline OOM.
    let bend_limit = MAX_BEND_COUNT as usize;
    let mut bend_meta: Vec<(u32, u32)> = Vec::with_capacity(n_edges);
    let mut all_bends: Vec<(f64, f64)> = Vec::new();
    for &edge_id in &ctx.edge_arena_ids {
        let start = all_bends.len() as u32;
        if let Some(edge) = ctx.graph.try_edge(edge_id) {
            for p in &edge.bend_points {
                if all_bends.len() >= bend_limit {
                    return Err(FfiError::TooLarge { kind: "bend", count: MAX_BEND_COUNT });
                }
                all_bends.push((p.x, p.y));
            }
        }
        let length = (all_bends.len() as u32) - start;
        bend_meta.push((start, length));
    }

    let bend_count = all_bends.len();
    let total_size = HEADER_SIZE
        + n_nodes * NODE_RECORD_SIZE
        + n_edges * EDGE_RECORD_SIZE
        + bend_count * BEND_RECORD_SIZE;

    let mut out = vec![0u8; total_size];

    // Header
    out[header_offset::MAGIC..header_offset::MAGIC + 4].copy_from_slice(MAGIC);
    write_u32(&mut out, header_offset::VERSION, VERSION);
    write_u32(&mut out, header_offset::FLAGS, FLAG_IS_OUTPUT);
    write_u32(&mut out, header_offset::NODE_COUNT, n_nodes as u32);
    write_u32(&mut out, header_offset::EDGE_COUNT, n_edges as u32);
    write_u32(&mut out, header_offset::BEND_COUNT, bend_count as u32);
    write_f64(&mut out, header_offset::GRAPH_WIDTH, ctx.graph.size.x);
    write_f64(&mut out, header_offset::GRAPH_HEIGHT, ctx.graph.size.y);

    // Nodes
    let node_section_start = HEADER_SIZE;
    for i in 0..n_nodes {
        let base = node_section_start + i * NODE_RECORD_SIZE;
        write_u32(&mut out, base + node_offset::ID, ctx.node_caller_ids[i]);
        // 4..8 padding intentionally left zero.
        if let Some(node) = ctx.graph.try_node(ctx.node_arena_ids[i]) {
            write_f64(&mut out, base + node_offset::WIDTH, node.size.x);
            write_f64(&mut out, base + node_offset::HEIGHT, node.size.y);
            write_f64(&mut out, base + node_offset::X, node.position.x);
            write_f64(&mut out, base + node_offset::Y, node.position.y);
        } else {
            write_f64(&mut out, base + node_offset::WIDTH, ctx.node_widths[i]);
            write_f64(&mut out, base + node_offset::HEIGHT, ctx.node_heights[i]);
            write_f64(&mut out, base + node_offset::X, 0.0);
            write_f64(&mut out, base + node_offset::Y, 0.0);
        }
    }

    // Edges
    let edge_section_start = node_section_start + n_nodes * NODE_RECORD_SIZE;
    for i in 0..n_edges {
        let base = edge_section_start + i * EDGE_RECORD_SIZE;
        let (bend_start, bend_length) = bend_meta[i];
        write_u32(&mut out, base + edge_offset::ID, ctx.edge_caller_ids[i]);
        write_u32(&mut out, base + edge_offset::SOURCE_NODE, ctx.edge_source_callers[i]);
        write_u32(&mut out, base + edge_offset::TARGET_NODE, ctx.edge_target_callers[i]);
        write_u32(&mut out, base + edge_offset::BEND_START, bend_start);
        write_u32(&mut out, base + edge_offset::BEND_LENGTH, bend_length);
        // 20..24 reserved.
    }

    // Bends
    let bend_section_start = edge_section_start + n_edges * EDGE_RECORD_SIZE;
    for (i, (x, y)) in all_bends.iter().enumerate() {
        let base = bend_section_start + i * BEND_RECORD_SIZE;
        write_f64(&mut out, base + bend_offset::X, *x);
        write_f64(&mut out, base + bend_offset::Y, *y);
    }

    Ok(out)
}

#[inline]
fn write_u32(buf: &mut [u8], offset: usize, value: u32) {
    buf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

#[inline]
fn write_f64(buf: &mut [u8], offset: usize, value: f64) {
    buf[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}
