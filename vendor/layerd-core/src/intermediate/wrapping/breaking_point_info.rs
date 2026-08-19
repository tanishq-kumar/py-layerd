//! Per-node bookkeeping for the breaking-point subsystem.
//!
//! The full `BPInfo` list is owned on the graph as a `Vec<BPInfo>` and the
//! per-dummy node references it by index via `BREAKING_POINT_INFO`,
//! matching the `SELF_LOOP_HOLDER` pattern used elsewhere in this crate.

use std::sync::LazyLock;

use crate::{
    graph::index::{EdgeId, NodeId},
    properties::{PropertyKey, PropertyMap},
};

/// Information about a single breaking point, shared between its start and
/// end dummies.
#[derive(Debug, Clone, Copy)]
pub struct BPInfo {
    pub start: NodeId,
    pub end: NodeId,
    pub node_start_edge: EdgeId,
    pub start_end_edge: EdgeId,
    pub original_edge: EdgeId,
    pub start_in_layer_dummy: Option<NodeId>,
    pub start_in_layer_edge: Option<EdgeId>,
    pub end_in_layer_dummy: Option<NodeId>,
    pub end_in_layer_edge: Option<EdgeId>,
    pub prev: Option<BPInfoId>,
    pub next: Option<BPInfoId>,
}

impl BPInfo {
    pub fn new(
        start: NodeId,
        end: NodeId,
        node_start_edge: EdgeId,
        start_end_edge: EdgeId,
        original_edge: EdgeId,
    ) -> Self {
        Self {
            start,
            end,
            node_start_edge,
            start_end_edge,
            original_edge,
            start_in_layer_dummy: None,
            start_in_layer_edge: None,
            end_in_layer_dummy: None,
            end_in_layer_edge: None,
            prev: None,
            next: None,
        }
    }
}

/// Newtype index into the per-graph `BPInfo` store.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BPInfoId(pub u32);

impl BPInfoId {
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

struct BreakingPointInfoMarker;
struct BreakingPointInfoStoreMarker;

/// Per-node property: index of the BPInfo attached to this breaking-point
/// dummy. Start and end dummies of the same chain share the same id.
pub static BREAKING_POINT_INFO: LazyLock<PropertyKey<Option<BPInfoId>>> =
    LazyLock::new(|| PropertyKey::of::<BreakingPointInfoMarker>(|| None));

/// Graph-level store of all BPInfos. The inserter appends and looks up via
/// `BPInfoId`. Cleared by the remover once every info has been consumed.
pub static BREAKING_POINT_INFO_STORE: LazyLock<PropertyKey<Vec<BPInfo>>> =
    LazyLock::new(|| PropertyKey::of::<BreakingPointInfoStoreMarker>(Vec::new));

/// Returns true if the BPInfo pointed to by `node_props` marks `node` as its
/// start dummy.
pub fn is_start(node: NodeId, node_props: &PropertyMap, store: &[BPInfo]) -> bool {
    node_props
        .get(&BREAKING_POINT_INFO)
        .and_then(|id| store.get(id.index()))
        .is_some_and(|info| info.start == node)
}

/// Returns true if the BPInfo pointed to by `node_props` marks `node` as its
/// end dummy.
pub fn is_end(node: NodeId, node_props: &PropertyMap, store: &[BPInfo]) -> bool {
    node_props
        .get(&BREAKING_POINT_INFO)
        .and_then(|id| store.get(id.index()))
        .is_some_and(|info| info.end == node)
}
