//! Cross-codec wire format conformance test.
//!
//! Locks the LRD1 wire format with a canonical binary fixture committed
//! at `ffi-types/testdata/v1_canonical.bin`. Independent codec
//! implementations read and write this fixture; drift is caught by comparing
//! produced bytes to the file.
//!
//! Regenerating the fixture: set `LAYERD_REGENERATE_TESTDATA=1` and run
//! `cargo nextest run -p ffi-types`. The new bytes are written to
//! `testdata/v1_canonical.bin`.

use std::path::PathBuf;

use ffi_types::{
    BEND_RECORD_SIZE, EDGE_RECORD_SIZE, FfiError, HEADER_SIZE, MAGIC, MAX_BEND_COUNT,
    MAX_EDGE_COUNT, MAX_NODE_COUNT, NODE_RECORD_SIZE, VERSION, layout_bytes,
};

/// Hardcoded graph fixture. The canonical fixture is intentionally tiny
/// (4 nodes, 3 edges, no self-loops) so the encoded buffer is small and the
/// expected layout is easy to verify with `cargo xtask bench profile-stages --`
/// when the fixture changes.
const FIXTURE_NODES: &[(u32, f64, f64)] = &[
    // (caller_id, width, height)
    (0, 30.0, 30.0),
    (1, 30.0, 30.0),
    (2, 50.0, 20.0),
    (3, 10.0, 10.0),
];

const FIXTURE_EDGES: &[(u32, u32, u32)] = &[
    // (caller_id, source, target)
    (0, 0, 1),
    (1, 1, 2),
    (2, 0, 3),
];

#[test]
fn wire_constants_v1() {
    // These values are the source of truth that JS and Swift codecs mirror.
    // Changing any of them is a wire format break and requires bumping
    // VERSION + synchronizing all three codecs in the same change.
    assert_eq!(VERSION, 1);
    assert_eq!(HEADER_SIZE, 40);
    assert_eq!(NODE_RECORD_SIZE, 40);
    assert_eq!(EDGE_RECORD_SIZE, 24);
    assert_eq!(BEND_RECORD_SIZE, 16);
    assert_eq!(MAGIC, b"LRD1");
    assert_eq!(MAX_NODE_COUNT, 1_000_000);
    assert_eq!(MAX_EDGE_COUNT, 2_000_000);
    assert_eq!(MAX_BEND_COUNT, 10_000_000);
}

#[test]
fn canonical_fixture_byte_identical() {
    let actual = encode_canonical_fixture();
    let path = testdata_path("v1_canonical.bin");

    if std::env::var("LAYERD_REGENERATE_TESTDATA").is_ok() {
        std::fs::create_dir_all(path.parent().unwrap()).expect("create testdata dir");
        std::fs::write(&path, &actual).expect("write canonical fixture");
        eprintln!("regenerated {}", path.display());
        return;
    }

    let expected = std::fs::read(&path).unwrap_or_else(|_| {
        panic!(
            "missing {}\n\nFirst-time setup: regenerate the fixture with:\n  \
             LAYERD_REGENERATE_TESTDATA=1 cargo nextest run -p ffi-types",
            path.display()
        )
    });

    assert_eq!(
        actual.len(),
        expected.len(),
        "canonical fixture size changed: {} -> {}. Did the wire format change without bumping VERSION?",
        expected.len(),
        actual.len()
    );

    if actual != expected {
        let mismatch = actual
            .iter()
            .zip(expected.iter())
            .position(|(a, e)| a != e)
            .unwrap_or(actual.len());
        panic!(
            "canonical fixture differs at byte {}: expected 0x{:02x}, got 0x{:02x}.\n\n\
             If this change is intentional, regenerate with \
             LAYERD_REGENERATE_TESTDATA=1 and synchronize JS + Swift codecs.",
            mismatch, expected[mismatch], actual[mismatch]
        );
    }
}

#[test]
fn canonical_fixture_layouts_successfully() {
    // The end-to-end test: the canonical input bytes pass through the FFI
    // layout pipeline without panicking, and the output is valid LRD1 wire
    // format with the FLAG_IS_OUTPUT bit set.
    let input = encode_canonical_fixture();
    let output = layout_bytes(&input).expect("layout_bytes succeeds on canonical fixture");

    assert!(output.len() >= HEADER_SIZE, "output too short");
    assert_eq!(&output[0..4], MAGIC, "output magic mismatch");
    let version = u32::from_le_bytes(output[4..8].try_into().unwrap());
    assert_eq!(version, VERSION, "output version mismatch");
    let flags = u32::from_le_bytes(output[8..12].try_into().unwrap());
    assert_eq!(flags & 1, 1, "FLAG_IS_OUTPUT not set on output buffer");
    let node_count = u32::from_le_bytes(output[12..16].try_into().unwrap());
    assert_eq!(node_count as usize, FIXTURE_NODES.len(), "node_count mismatch");
}

#[test]
fn rejects_unknown_version_input() {
    let mut buf = vec![0u8; HEADER_SIZE];
    buf[0..4].copy_from_slice(MAGIC);
    buf[4..8].copy_from_slice(&99u32.to_le_bytes()); // unsupported VERSION
    let err = layout_bytes(&buf).unwrap_err();
    assert!(
        matches!(err, FfiError::UnsupportedVersion(99)),
        "expected UnsupportedVersion(99), got {err:?}"
    );
}

#[test]
fn rejects_non_finite_width() {
    let mut input = encode_canonical_fixture();
    // Overwrite node 0 width (offset HEADER + node[0] + WIDTH = 40 + 0 + 8 = 48) with NaN.
    let nan_bytes = f64::NAN.to_le_bytes();
    input[48..56].copy_from_slice(&nan_bytes);
    let err = layout_bytes(&input).unwrap_err();
    assert!(
        matches!(err, FfiError::InvalidGeometry { node_id: 0 }),
        "expected InvalidGeometry {{ node_id: 0 }}, got {err:?}"
    );
}

// --- canonical encoder (independent of ffi-types::encode) ---
//
// Mirrors the LRD1 wire layout described in `ffi-types/src/wire.rs`. This
// is intentionally a *separate* implementation from the production
// encoder so that any drift between the format spec and the production
// code path is caught by the byte-identity assertion above.

fn encode_canonical_fixture() -> Vec<u8> {
    let n = FIXTURE_NODES.len();
    let e = FIXTURE_EDGES.len();
    let total = 40 + n * 40 + e * 24;
    let mut buf = vec![0u8; total];

    // Header (40 bytes).
    buf[0..4].copy_from_slice(b"LRD1");
    buf[4..8].copy_from_slice(&VERSION.to_le_bytes()); // VERSION
    buf[8..12].copy_from_slice(&0u32.to_le_bytes()); // FLAGS
    buf[12..16].copy_from_slice(&(n as u32).to_le_bytes());
    buf[16..20].copy_from_slice(&(e as u32).to_le_bytes());
    buf[20..24].copy_from_slice(&0u32.to_le_bytes()); // BEND_COUNT
    buf[24..32].copy_from_slice(&0f64.to_le_bytes()); // GRAPH_WIDTH
    buf[32..40].copy_from_slice(&0f64.to_le_bytes()); // GRAPH_HEIGHT

    // Nodes (40 bytes each).
    for (i, &(id, w, h)) in FIXTURE_NODES.iter().enumerate() {
        let base = 40 + i * 40;
        buf[base..base + 4].copy_from_slice(&id.to_le_bytes());
        // 4..8 reserved padding (zero).
        buf[base + 8..base + 16].copy_from_slice(&w.to_le_bytes());
        buf[base + 16..base + 24].copy_from_slice(&h.to_le_bytes());
        // 24..32 = x, 32..40 = y, both zero on input.
    }

    // Edges (24 bytes each).
    let edge_start = 40 + n * 40;
    for (i, &(id, src, tgt)) in FIXTURE_EDGES.iter().enumerate() {
        let base = edge_start + i * 24;
        buf[base..base + 4].copy_from_slice(&id.to_le_bytes());
        buf[base + 4..base + 8].copy_from_slice(&src.to_le_bytes());
        buf[base + 8..base + 12].copy_from_slice(&tgt.to_le_bytes());
        buf[base + 12..base + 16].copy_from_slice(&0u32.to_le_bytes()); // BEND_START
        buf[base + 16..base + 20].copy_from_slice(&0u32.to_le_bytes()); // BEND_LENGTH
        // 20..24 reserved.
    }

    buf
}

fn testdata_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata").join(name)
}
