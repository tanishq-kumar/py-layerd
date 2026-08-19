use std::{
    num::NonZeroUsize,
    ops::{Deref, DerefMut},
    ptr::NonNull,
    slice,
};

use smallvec::SmallVec;

use super::{
    LGraph,
    index::{EdgeId, LabelId, NodeId, PortId},
    port::PortSide,
};
use crate::{
    math::{Margin, Padding, Vec2},
    options::enums::PortConstraints,
    properties::{PropertyMap, internal::NODE_PORT_CONSTRAINTS},
};

/// The type of a node in the layered graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodeType {
    #[default]
    Normal,
    LongEdge,
    ExternalPort,
    NorthSouthPort,
    Label,
    BreakingPoint,
    Placeholder,
    NonShiftingPlaceholder,
}

/// Sentinel value in `NodeData.port_side_ends[0]` meaning the per-side range
/// cache has not been filled. `u16::MAX` is unreachable as a valid end index
/// since a single node's port count never approaches 65535 in practice.
const PORT_SIDE_NOT_CACHED: u16 = u16::MAX;
const DEFAULT_MARGIN: Margin = Margin { top: 0.0, right: 0.0, bottom: 0.0, left: 0.0 };
const DEFAULT_PADDING: Padding = Padding { top: 0.0, right: 0.0, bottom: 0.0, left: 0.0 };

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NodeLayer(Option<NonZeroUsize>);

impl NodeLayer {
    #[inline]
    pub fn some(layer: usize) -> Self {
        let stored = layer.checked_add(1).expect("node layer index overflow");
        Self(Some(NonZeroUsize::new(stored).expect("stored node layer must be non-zero")))
    }

    #[inline]
    pub fn get(self) -> Option<usize> {
        self.0.map(|layer| layer.get() - 1)
    }

    #[inline]
    pub fn is_none(self) -> bool {
        self.0.is_none()
    }

    #[inline]
    pub fn is_some(self) -> bool {
        self.0.is_some()
    }

    #[inline]
    pub fn is_some_and(self, f: impl FnOnce(usize) -> bool) -> bool {
        self.get().is_some_and(f)
    }

    #[inline]
    pub fn unwrap(self) -> usize {
        self.get().unwrap()
    }

    #[inline]
    pub fn unwrap_or(self, default: usize) -> usize {
        self.get().unwrap_or(default)
    }

    #[inline]
    pub fn expect(self, msg: &str) -> usize {
        self.get().expect(msg)
    }

    #[inline]
    pub fn map<T>(self, f: impl FnOnce(usize) -> T) -> Option<T> {
        self.get().map(f)
    }
}

impl From<Option<usize>> for NodeLayer {
    #[inline]
    fn from(layer: Option<usize>) -> Self {
        match layer {
            Some(layer) => Self::some(layer),
            None => Self::default(),
        }
    }
}

impl PartialEq<Option<usize>> for NodeLayer {
    #[inline]
    fn eq(&self, other: &Option<usize>) -> bool {
        self.get() == *other
    }
}

/// Map a `PortSide` to its index in the cumulative `port_side_ends` table.
/// Order: N→E→S→W→Undefined.
#[inline]
pub const fn port_side_table_index(side: PortSide) -> usize {
    match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => 4,
    }
}

#[derive(Clone, Debug, Default)]
pub struct NodePadding {
    padding: Option<Box<Padding>>,
}

impl From<Padding> for NodePadding {
    #[inline]
    fn from(padding: Padding) -> Self {
        if padding == Padding::default() {
            Self { padding: None }
        } else {
            Self { padding: Some(Box::new(padding)) }
        }
    }
}

impl Deref for NodePadding {
    type Target = Padding;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.padding.as_deref().unwrap_or(&DEFAULT_PADDING)
    }
}

impl DerefMut for NodePadding {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.padding.get_or_insert_with(|| Box::new(Padding::default()))
    }
}

#[derive(Clone, Debug, Default)]
pub struct NodeMargin {
    margin: Option<Box<Margin>>,
}

impl From<Margin> for NodeMargin {
    #[inline]
    fn from(margin: Margin) -> Self {
        if margin == Margin::default() {
            Self { margin: None }
        } else {
            Self { margin: Some(Box::new(margin)) }
        }
    }
}

impl Deref for NodeMargin {
    type Target = Margin;

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.margin.as_deref().unwrap_or(&DEFAULT_MARGIN)
    }
}

impl DerefMut for NodeMargin {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.margin.get_or_insert_with(|| Box::new(Margin::default()))
    }
}

// Keep sparse label lists out of line so the common unlabeled node stays small.
#[allow(clippy::box_collection)]
#[derive(Clone, Debug, Default)]
pub struct NodeLabels {
    labels: Option<Box<Vec<LabelId>>>,
}

impl NodeLabels {
    #[inline]
    pub fn new() -> Self {
        Self { labels: None }
    }

    #[inline]
    pub fn push(&mut self, label: LabelId) {
        self.labels.get_or_insert_with(|| Box::new(Vec::new())).push(label);
    }

    #[inline]
    pub fn as_slice(&self) -> &[LabelId] {
        self.labels.as_deref().map(Vec::as_slice).unwrap_or(&[])
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [LabelId] {
        self.labels.as_deref_mut().map(Vec::as_mut_slice).unwrap_or(&mut [])
    }
}

impl Deref for NodeLabels {
    type Target = [LabelId];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for NodeLabels {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<'a> IntoIterator for &'a NodeLabels {
    type IntoIter = slice::Iter<'a, LabelId>;
    type Item = &'a LabelId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

// Keep nested-child storage out of line for the common flat-node case.
#[allow(clippy::box_collection)]
#[derive(Default)]
pub struct NodeChildren {
    children: Option<Box<Vec<NodeId>>>,
}

impl NodeChildren {
    #[inline]
    pub fn new() -> Self {
        Self { children: None }
    }

    #[inline]
    pub fn push(&mut self, child: NodeId) {
        self.children.get_or_insert_with(|| Box::new(Vec::new())).push(child);
    }

    #[inline]
    pub fn retain<F>(&mut self, keep: F)
    where
        F: FnMut(&NodeId) -> bool,
    {
        if let Some(children) = &mut self.children {
            children.retain(keep);
            if children.is_empty() {
                self.children = None;
            }
        }
    }

    #[inline]
    pub fn as_slice(&self) -> &[NodeId] {
        self.children.as_deref().map(Vec::as_slice).unwrap_or(&[])
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [NodeId] {
        self.children.as_deref_mut().map(Vec::as_mut_slice).unwrap_or(&mut [])
    }
}

impl Deref for NodeChildren {
    type Target = [NodeId];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for NodeChildren {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<'a> IntoIterator for &'a NodeChildren {
    type IntoIter = slice::Iter<'a, NodeId>;
    type Item = &'a NodeId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

pub enum NodeChildrenIntoIter {
    Empty,
    Children(std::vec::IntoIter<NodeId>),
}

impl Iterator for NodeChildrenIntoIter {
    type Item = NodeId;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Empty => None,
            Self::Children(children) => children.next(),
        }
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Empty => (0, Some(0)),
            Self::Children(children) => children.size_hint(),
        }
    }
}

impl ExactSizeIterator for NodeChildrenIntoIter {}

impl IntoIterator for NodeChildren {
    type IntoIter = NodeChildrenIntoIter;
    type Item = NodeId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        match self.children {
            None => NodeChildrenIntoIter::Empty,
            Some(children) => NodeChildrenIntoIter::Children((*children).into_iter()),
        }
    }
}

/// Data stored for each node in the graph.
///
/// `nested_graph` stores a raw owning pointer to a child `LGraph` (allocated via
/// `Box::into_raw`) rather than `Option<Box<LGraph>>`. This breaks the borrow chain
/// between parent and child graphs so processors can hold `&mut` on both simultaneously.
/// Access is mediated by `LGraph::nested` / `LGraph::nested_mut` / `LGraph::set_nested` /
/// `LGraph::take_nested`. Ownership is released in `Drop`.
pub struct NodeData {
    pub position: Vec2,
    pub size: Vec2,
    pub node_type: NodeType,
    pub layer: NodeLayer,
    pub ports: SmallVec<PortId, 2>,
    pub labels: NodeLabels,
    pub margin: NodeMargin,
    pub padding: NodePadding,
    pub nested_graph: Option<NonNull<LGraph>>,
    pub parent: Option<NodeId>,
    pub children: NodeChildren,
    pub properties: PropertyMap,
    pub origin_edge: Option<EdgeId>,
    pub long_edge_source: Option<PortId>,
    pub long_edge_target: Option<PortId>,
    pub long_edge_has_label_dummies: bool,
    pub node_port_constraints: Option<PortConstraints>,
    pub id: u32,
    /// Cumulative end indices into `ports` for each `PortSide`, in N→E→S→W→Undefined
    /// order. Range for side `s` is `[ends[i-1] .. ends[i])` with `i = port_side_table_index(s)`
    /// and the empty prefix for `i == 0`. Sentinel `port_side_ends[0] == u16::MAX` means
    /// the cache has not been filled. Filled by `LGraph::cache_port_sides` after `PortListSorter`
    /// stable-groups ports by side; consumed by P3 cross-min hot paths to return
    /// `&[PortId]` slices instead of allocating filtered `SmallVec`s.
    pub port_side_ends: [u16; 5],
}

impl NodeData {
    pub fn new(size: Vec2, id: u32) -> Self {
        NodeData {
            position: Vec2::ZERO,
            size,
            node_type: NodeType::default(),
            layer: NodeLayer::default(),
            ports: SmallVec::new(),
            labels: NodeLabels::new(),
            margin: NodeMargin::default(),
            padding: NodePadding::default(),
            nested_graph: None,
            parent: None,
            children: NodeChildren::new(),
            properties: PropertyMap::new(),
            origin_edge: None,
            long_edge_source: None,
            long_edge_target: None,
            long_edge_has_label_dummies: false,
            node_port_constraints: None,
            id,
            port_side_ends: [PORT_SIDE_NOT_CACHED; 5],
        }
    }

    #[inline]
    pub fn port_constraints(&self) -> PortConstraints {
        self.node_port_constraints
            .unwrap_or_else(|| self.properties.get(&NODE_PORT_CONSTRAINTS))
    }

    /// Returns true if `port_side_ends` has been populated by `LGraph::cache_port_sides`.
    #[inline]
    pub fn is_port_side_cached(&self) -> bool {
        self.port_side_ends[0] != PORT_SIDE_NOT_CACHED
    }

    /// Marks the per-side range cache as stale. Callers that mutate `ports`
    /// (push, remove, reorder) must invoke this so the next consumer recomputes.
    #[inline]
    pub fn invalidate_port_sides(&mut self) {
        self.port_side_ends[0] = PORT_SIDE_NOT_CACHED;
    }

    /// Returns the index range into `self.ports` that holds ports on `side`.
    /// Requires the cache to be filled (`is_port_side_cached()`).
    #[inline]
    pub fn port_side_range(&self, side: PortSide) -> std::ops::Range<usize> {
        debug_assert!(self.is_port_side_cached(), "port side cache not initialized");
        let i = port_side_table_index(side);
        let start = if i == 0 { 0 } else { self.port_side_ends[i - 1] as usize };
        let end = self.port_side_ends[i] as usize;
        start..end
    }

    /// Returns the slice of ports on the requested side using the cached range.
    /// Requires the cache to be filled.
    #[inline]
    pub fn ports_on_side(&self, side: PortSide) -> &[PortId] {
        let range = self.port_side_range(side);
        &self.ports[range]
    }

    /// Returns a shared reference to the nested graph pointed to by `nested_graph`.
    ///
    /// Equivalent to `LGraph::nested` but operates on a `&NodeData` captured during
    /// iteration (e.g. inside `LGraph::nodes_iter`) where re-borrowing the parent
    /// graph is not possible.
    pub fn nested_graph_ref(&self) -> Option<&LGraph> {
        // SAFETY: the pointer was created by `Box::into_raw` in `LGraph::set_nested`
        // and is owned by this `NodeData` (released in `Drop`). The returned reference
        // borrows from `&self`, so the pointer is guaranteed live.
        self.nested_graph.map(|p| unsafe { p.as_ref() })
    }

    /// Returns a mutable reference to the nested graph pointed to by `nested_graph`.
    pub fn nested_graph_mut(&mut self) -> Option<&mut LGraph> {
        // SAFETY: same invariant as `nested_graph_ref`; `&mut self` proves exclusive
        // access to this `NodeData`, hence exclusive access to the heap allocation.
        self.nested_graph.map(|mut p| unsafe { p.as_mut() })
    }
}

impl Drop for NodeData {
    fn drop(&mut self) {
        if let Some(ptr) = self.nested_graph.take() {
            // SAFETY: the pointer was created by `Box::into_raw` in `LGraph::set_nested`.
            // Exclusive ownership is guaranteed because `nested_graph` is an `Option` and
            // we just `take()`d it; no other `NodeData` can observe the same pointer.
            unsafe {
                drop(Box::from_raw(ptr.as_ptr()));
            }
        }
    }
}

// SAFETY: `NonNull<LGraph>` behaves as an owning `Box<LGraph>`. The inner `LGraph` is
// `Send` because its fields are `Send`. We can soundly move `NodeData` across threads.
unsafe impl Send for NodeData {}
// SAFETY: Shared access to `NodeData` only exposes `&LGraph` via `LGraph::nested`, which
// aliases the heap allocation but is immutable — safe to share.
unsafe impl Sync for NodeData {}
