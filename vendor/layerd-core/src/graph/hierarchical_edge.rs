//! Cross-hierarchy edge support.
//!
//! `LGraph` uses per-graph arenas, so a `PortId` from a nested graph cannot be
//! placed directly in another graph's `EdgeData`. To express edges whose
//! endpoints live in different levels, the root graph carries a parallel
//! `Vec<HierarchicalEdgeData>` that the compound preprocessor drains at Pre-P1
//! time, replacing each entry with local dummy edges plus external-port
//! dummies via `transform_hierarchy_edges`.
//!
//! `LEdge` endpoints can point at any `LPort` anywhere in the unified
//! hierarchy. The preprocessor's transformation rule lifts cross-hierarchy
//! edges onto graph-local segments; the storage shape is Rust-specific.
use super::index::{NodeId, PortId};
use crate::{math::Vec2, properties::PropertyMap};

/// Qualified reference to a port that may live in a nested graph.
///
/// `graph_parent == None` means the port lives in the root graph itself.
/// `graph_parent == Some(n)` means the port lives in
/// `root.find_graph_containing(n).unwrap().nested(n).unwrap()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HierarchicalPortRef {
    pub graph_parent: Option<NodeId>,
    pub port: PortId,
}

impl HierarchicalPortRef {
    /// Build a reference to a port that lives directly on the root graph.
    pub fn root(port: PortId) -> Self {
        HierarchicalPortRef { graph_parent: None, port }
    }

    /// Build a reference to a port that lives inside `graph_parent.nested_graph`.
    pub fn nested(graph_parent: NodeId, port: PortId) -> Self {
        HierarchicalPortRef { graph_parent: Some(graph_parent), port }
    }
}

/// Side-channel record for an edge whose endpoints span hierarchy levels.
///
/// Stored only on the root `LGraph`. The preprocessor consumes the list,
/// produces local dummy edges, and clears the list before downstream phases
/// run. After preprocess, downstream phases observe only local edges.
pub struct HierarchicalEdgeData {
    pub source: HierarchicalPortRef,
    pub target: HierarchicalPortRef,
    pub order: i32,
    pub properties: PropertyMap,
    pub labels: Vec<HierarchicalEdgeLabel>,
}

pub struct HierarchicalEdgeLabel {
    pub text: String,
    pub position: Vec2,
    pub size: Vec2,
    pub properties: PropertyMap,
}

impl HierarchicalEdgeData {
    pub fn new(source: HierarchicalPortRef, target: HierarchicalPortRef) -> Self {
        HierarchicalEdgeData {
            source,
            target,
            order: i32::MAX,
            properties: PropertyMap::new(),
            labels: Vec::new(),
        }
    }
}
