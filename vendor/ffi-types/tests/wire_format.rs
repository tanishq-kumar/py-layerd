//! LRD1 wire format integration tests.

use ffi_types::{
    EDGE_RECORD_SIZE, FfiError, HEADER_SIZE, MAX_NODE_COUNT, NODE_RECORD_SIZE, VERSION,
    layout_bytes,
};

/// Builds a header-only LRD1 buffer with the given counts.
fn make_header_only(node_count: u32, edge_count: u32) -> Vec<u8> {
    let mut buf = vec![0u8; HEADER_SIZE];
    buf[0..4].copy_from_slice(b"LRD1");
    buf[4..8].copy_from_slice(&VERSION.to_le_bytes());
    buf[12..16].copy_from_slice(&node_count.to_le_bytes());
    buf[16..20].copy_from_slice(&edge_count.to_le_bytes());
    buf
}

/// Writes a node record at byte offset `at` with the given id and dimensions.
/// Mirrors the LRD1 layout: id@0, reserved@4, width@8 (f64), height@16 (f64),
/// x@24 (f64, zero on input), y@32 (f64, zero on input).
fn write_node(buf: &mut [u8], at: usize, id: u32, width: f64, height: f64) {
    buf[at..at + 4].copy_from_slice(&id.to_le_bytes());
    buf[at + 8..at + 16].copy_from_slice(&width.to_le_bytes());
    buf[at + 16..at + 24].copy_from_slice(&height.to_le_bytes());
}

/// Writes an edge record at byte offset `at`.
fn write_edge(buf: &mut [u8], at: usize, id: u32, source: u32, target: u32) {
    buf[at..at + 4].copy_from_slice(&id.to_le_bytes());
    buf[at + 4..at + 8].copy_from_slice(&source.to_le_bytes());
    buf[at + 8..at + 12].copy_from_slice(&target.to_le_bytes());
}

#[test]
fn empty_graph_roundtrip() {
    let input = make_header_only(0, 0);
    let output = layout_bytes(&input).expect("empty graph must round-trip");
    assert!(output.len() >= HEADER_SIZE);
    assert_eq!(&output[0..4], b"LRD1");
    assert_eq!(u32::from_le_bytes([output[12], output[13], output[14], output[15]]), 0);
    assert_eq!(u32::from_le_bytes([output[16], output[17], output[18], output[19]]), 0);
    assert_eq!(u32::from_le_bytes([output[20], output[21], output[22], output[23]]), 0);
}

#[test]
fn buffer_too_short_for_header() {
    let input = vec![0u8; 16];
    match layout_bytes(&input) {
        Err(FfiError::BufferTooShort { expected, actual: 16 }) if expected == HEADER_SIZE => {}
        other => panic!(
            "expected BufferTooShort {{ expected: {HEADER_SIZE}, actual: 16 }}, got {other:?}"
        ),
    }
}

#[test]
fn invalid_magic_rejected() {
    let mut input = make_header_only(0, 0);
    input[0..4].copy_from_slice(b"XXXX");
    match layout_bytes(&input) {
        Err(FfiError::InvalidMagic) => {}
        other => panic!("expected InvalidMagic, got {other:?}"),
    }
}

#[test]
fn unsupported_version_rejected() {
    let mut input = make_header_only(0, 0);
    input[4..8].copy_from_slice(&99u32.to_le_bytes());
    match layout_bytes(&input) {
        Err(FfiError::UnsupportedVersion(99)) => {}
        other => panic!("expected UnsupportedVersion(99), got {other:?}"),
    }
}

#[test]
fn too_many_nodes_rejected() {
    let input = make_header_only(MAX_NODE_COUNT + 1, 0);
    match layout_bytes(&input) {
        Err(FfiError::TooLarge { kind: "node", count }) if count == MAX_NODE_COUNT + 1 => {}
        other => panic!("expected TooLarge node, got {other:?}"),
    }
}

#[test]
fn buffer_too_short_for_declared_nodes() {
    let input = make_header_only(2, 0);
    match layout_bytes(&input) {
        Err(FfiError::BufferTooShort { expected, actual }) => {
            assert_eq!(expected, HEADER_SIZE + 2 * NODE_RECORD_SIZE);
            assert_eq!(actual, HEADER_SIZE);
        }
        other => panic!("expected BufferTooShort, got {other:?}"),
    }
}

#[test]
fn duplicate_node_id_rejected() {
    let mut input = vec![0u8; HEADER_SIZE + 2 * NODE_RECORD_SIZE];
    input[0..4].copy_from_slice(b"LRD1");
    input[4..8].copy_from_slice(&VERSION.to_le_bytes());
    input[12..16].copy_from_slice(&2u32.to_le_bytes());

    write_node(&mut input, HEADER_SIZE, 7, 10.0, 10.0);
    write_node(&mut input, HEADER_SIZE + NODE_RECORD_SIZE, 7, 10.0, 10.0);

    match layout_bytes(&input) {
        Err(FfiError::DuplicateId { kind: "node", id: 7 }) => {}
        other => panic!("expected DuplicateId node 7, got {other:?}"),
    }
}

#[test]
fn edge_references_unknown_node_rejected() {
    let total = HEADER_SIZE + NODE_RECORD_SIZE + EDGE_RECORD_SIZE;
    let mut input = vec![0u8; total];
    input[0..4].copy_from_slice(b"LRD1");
    input[4..8].copy_from_slice(&VERSION.to_le_bytes());
    input[12..16].copy_from_slice(&1u32.to_le_bytes());
    input[16..20].copy_from_slice(&1u32.to_le_bytes());

    write_node(&mut input, HEADER_SIZE, 10, 20.0, 20.0);
    write_edge(&mut input, HEADER_SIZE + NODE_RECORD_SIZE, 100, 10, 999);

    match layout_bytes(&input) {
        Err(FfiError::InvalidNodeReference { edge_id: 100, node_id: 999 }) => {}
        other => panic!("expected InvalidNodeReference, got {other:?}"),
    }
}
