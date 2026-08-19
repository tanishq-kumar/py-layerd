//! LRD1 input decoder: byte buffer into `LGraph` plus caller id tracking.

use hashbrown::HashMap;
use layerd::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        port::PortSide,
    },
    math::Vec2,
};

use crate::{
    layout::FfiError,
    wire::{
        EDGE_RECORD_SIZE, HEADER_SIZE, MAGIC, MAX_BEND_COUNT, MAX_EDGE_COUNT, MAX_NODE_COUNT,
        NODE_RECORD_SIZE, VERSION, edge_offset, header_offset, node_offset,
    },
};

/// State tracked across decode, layout, and encode phases.
///
/// Caller-assigned ids are stored alongside arena ids so the encoder can
/// emit stable ids back to the consumer regardless of how the layerd
/// pipeline rearranges internal arenas.
pub(crate) struct LayoutContext {
    pub graph: LGraph,
    pub node_caller_ids: Vec<u32>,
    pub node_arena_ids: Vec<NodeId>,
    pub node_widths: Vec<f64>,
    pub node_heights: Vec<f64>,
    pub edge_caller_ids: Vec<u32>,
    pub edge_source_callers: Vec<u32>,
    pub edge_target_callers: Vec<u32>,
    pub edge_arena_ids: Vec<EdgeId>,
}

/// Parses an LRD1 input buffer into a graph with caller id mappings.
pub(crate) fn decode(input: &[u8]) -> Result<LayoutContext, FfiError> {
    if input.len() < HEADER_SIZE {
        return Err(FfiError::BufferTooShort { expected: HEADER_SIZE, actual: input.len() });
    }

    if &input[header_offset::MAGIC..header_offset::MAGIC + 4] != MAGIC {
        return Err(FfiError::InvalidMagic);
    }

    let version = read_u32(input, header_offset::VERSION);
    if version != VERSION {
        return Err(FfiError::UnsupportedVersion(version));
    }

    let node_count = read_u32(input, header_offset::NODE_COUNT);
    let edge_count = read_u32(input, header_offset::EDGE_COUNT);
    let bend_count = read_u32(input, header_offset::BEND_COUNT);

    if node_count > MAX_NODE_COUNT {
        return Err(FfiError::TooLarge { kind: "node", count: node_count });
    }
    if edge_count > MAX_EDGE_COUNT {
        return Err(FfiError::TooLarge { kind: "edge", count: edge_count });
    }
    // Defensive: input buffers should never carry bend data (bends are
    // produced by the pipeline on output). A malformed or malicious input
    // declaring a huge bend_count would otherwise be accepted silently.
    if bend_count > MAX_BEND_COUNT {
        return Err(FfiError::TooLarge { kind: "bend", count: bend_count });
    }

    let node_section_start = HEADER_SIZE;
    let node_section_size = (node_count as usize) * NODE_RECORD_SIZE;
    let edge_section_start = node_section_start + node_section_size;
    let edge_section_size = (edge_count as usize) * EDGE_RECORD_SIZE;
    let expected_len = edge_section_start + edge_section_size;

    if input.len() < expected_len {
        return Err(FfiError::BufferTooShort { expected: expected_len, actual: input.len() });
    }

    let n_nodes = node_count as usize;
    let n_edges = edge_count as usize;

    let mut graph = LGraph::new();
    let mut node_caller_ids: Vec<u32> = Vec::with_capacity(n_nodes);
    let mut node_arena_ids: Vec<NodeId> = Vec::with_capacity(n_nodes);
    let mut node_widths: Vec<f64> = Vec::with_capacity(n_nodes);
    let mut node_heights: Vec<f64> = Vec::with_capacity(n_nodes);
    let mut node_ports: Vec<(PortId, PortId)> = Vec::with_capacity(n_nodes);
    let mut caller_to_index: HashMap<u32, usize> = HashMap::with_capacity(n_nodes);

    for i in 0..n_nodes {
        let base = node_section_start + i * NODE_RECORD_SIZE;
        let caller_id = read_u32(input, base + node_offset::ID);
        let width = read_f64(input, base + node_offset::WIDTH);
        let height = read_f64(input, base + node_offset::HEIGHT);

        if !width.is_finite() || !height.is_finite() {
            return Err(FfiError::InvalidGeometry { node_id: caller_id });
        }

        if caller_to_index.insert(caller_id, i).is_some() {
            return Err(FfiError::DuplicateId { kind: "node", id: caller_id });
        }

        let node_id = graph.add_node(Vec2::new(width, height));
        let input = graph.add_port(node_id, PortSide::West);
        let output = graph.add_port(node_id, PortSide::East);

        node_caller_ids.push(caller_id);
        node_arena_ids.push(node_id);
        node_widths.push(width);
        node_heights.push(height);
        node_ports.push((input, output));
    }

    let mut edge_caller_ids: Vec<u32> = Vec::with_capacity(n_edges);
    let mut edge_source_callers: Vec<u32> = Vec::with_capacity(n_edges);
    let mut edge_target_callers: Vec<u32> = Vec::with_capacity(n_edges);
    let mut edge_arena_ids: Vec<EdgeId> = Vec::with_capacity(n_edges);
    let mut seen_edge_ids: HashMap<u32, ()> = HashMap::with_capacity(n_edges);

    for i in 0..n_edges {
        let base = edge_section_start + i * EDGE_RECORD_SIZE;
        let caller_id = read_u32(input, base + edge_offset::ID);
        let src_caller = read_u32(input, base + edge_offset::SOURCE_NODE);
        let tgt_caller = read_u32(input, base + edge_offset::TARGET_NODE);

        if seen_edge_ids.insert(caller_id, ()).is_some() {
            return Err(FfiError::DuplicateId { kind: "edge", id: caller_id });
        }

        let src_idx = *caller_to_index
            .get(&src_caller)
            .ok_or(FfiError::InvalidNodeReference { edge_id: caller_id, node_id: src_caller })?;
        let tgt_idx = *caller_to_index
            .get(&tgt_caller)
            .ok_or(FfiError::InvalidNodeReference { edge_id: caller_id, node_id: tgt_caller })?;

        let edge_id = graph.add_edge(node_ports[src_idx].1, node_ports[tgt_idx].0);

        edge_caller_ids.push(caller_id);
        edge_source_callers.push(src_caller);
        edge_target_callers.push(tgt_caller);
        edge_arena_ids.push(edge_id);
    }

    Ok(LayoutContext {
        graph,
        node_caller_ids,
        node_arena_ids,
        node_widths,
        node_heights,
        edge_caller_ids,
        edge_source_callers,
        edge_target_callers,
        edge_arena_ids,
    })
}

#[inline]
fn read_u32(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]])
}

#[inline]
fn read_f64(buf: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
        buf[offset + 4],
        buf[offset + 5],
        buf[offset + 6],
        buf[offset + 7],
    ])
}
