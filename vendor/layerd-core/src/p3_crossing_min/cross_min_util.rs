//! Cross-minimization shared helpers.
//!
//! Utility for iterating over ports in a stable "north to south,
//! east to west" order used throughout crossing minimization.
//!
//! Ordering source note: callers reach for the per-side O(1) `subList`-style
//! view backed by the pre-computed `cache_port_sides` table rather than
//! `NodeData.port_side_ends` plus `LGraph::cache_port_sides`, populated by
//! `PortListSorter`. When the cache is filled (the common P3 case),
//! `iter_ports_in_nsew` returns an iterator over a contiguous `&[PortId]`
//! slice with zero allocation. When it is not (test fixtures that bypass
//! `PortListSorter`, or new nodes added after caching), it falls back to
//! filter + collect into a `PortBuf`.

use smallvec::SmallVec;

use crate::graph::{
    LGraph,
    index::{NodeId, PortId},
    port::PortSide,
};

/// Inline capacity keeps common multi-port side views heap-free.
pub type PortBuf = SmallVec<PortId, 6>;

/// Iterator over a node's ports on a given side in stable
/// north-to-south, east-to-west order.
///
/// Three backing variants, all delegating to std-library iterators so LLVM
/// can inline and (for the slice paths) vectorise:
///
/// - `Fwd`: forward iteration over the cached per-side `&[PortId]` (N / E).
/// - `Rev`: reverse iteration over the cached per-side `&[PortId]` (S / W).
/// - `Owned`: filtered + collected into an owned `PortBuf`, used when the
///   per-side cache has not been populated yet.
pub enum SideIter<'a> {
    Fwd(std::slice::Iter<'a, PortId>),
    Rev(std::iter::Rev<std::slice::Iter<'a, PortId>>),
    Owned(smallvec::IntoIter<PortId, 6>),
}

impl<'a> Iterator for SideIter<'a> {
    type Item = PortId;

    #[inline]
    fn next(&mut self) -> Option<PortId> {
        match self {
            Self::Fwd(it) => it.next().copied(),
            Self::Rev(it) => it.next().copied(),
            Self::Owned(it) => it.next(),
        }
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Fwd(it) => it.size_hint(),
            Self::Rev(it) => it.size_hint(),
            Self::Owned(it) => it.size_hint(),
        }
    }
}

impl<'a> ExactSizeIterator for SideIter<'a> {}

/// Return an iterator over `node_id`'s ports on `side` in north-to-south,
/// east-to-west order.
///
/// Hits the per-side range cache when populated (the P3 common case after
/// `PortListSorter` runs); falls back to filter + collect otherwise.
#[inline]
pub fn iter_ports_in_nsew<'a>(graph: &'a LGraph, node_id: NodeId, side: PortSide) -> SideIter<'a> {
    let node = graph.node(node_id);

    if matches!(side, PortSide::Undefined) {
        return SideIter::Fwd([].iter());
    }

    let rev = matches!(side, PortSide::South | PortSide::West);

    if node.is_port_side_cached() {
        let slice = node.ports_on_side(side);
        if rev {
            SideIter::Rev(slice.iter().rev())
        } else {
            SideIter::Fwd(slice.iter())
        }
    } else {
        let mut ports: PortBuf = node
            .ports
            .iter()
            .copied()
            .filter(|&port_id| graph.port(port_id).side == side)
            .collect();
        if rev {
            ports.reverse();
        }
        SideIter::Owned(ports.into_iter())
    }
}
