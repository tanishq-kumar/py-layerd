//! Barycenter scratch state shared by crossing-minimization helpers.
//!
//! Extracted into its own module so both the barycenter heuristic and the
//! constraint resolver can reference the state type without forming a
//! cross-module dependency cycle.

use std::cmp::Ordering;

use crate::graph::index::NodeId;

/// Per-node scratch state used during barycenter computation and sorting.
#[derive(Clone, Copy, Default)]
pub struct BarycenterState {
    /// Accumulated port-rank weight from incident edges.
    pub summed_weight: f64,
    /// Number of contributing edges used for the barycenter average.
    pub degree: usize,
    /// Effective barycenter: `summed_weight / degree`, or `None` if the
    /// node has no connected neighbours on the ranked side.
    pub barycenter: Option<f64>,
    /// Recursion guard for the constraint resolver and cycle detection.
    pub visited: bool,
}

/// Dense scratch map for barycenter state keyed by `NodeId`.
///
/// The locator is indexed by `NodeId`'s arena slot, but every lookup validates
/// the full `NodeId` stored in `nodes` before returning a state. This keeps
/// the cross-`LGraph`/generation invariant intact for side channels such as
/// `BARYCENTER_ASSOCIATES`, where an id from another graph may share the same
/// slot index.
pub struct BarycenterStateMap {
    index_to_pos: Vec<u32>,
    touched_indices: Vec<usize>,
    nodes: Vec<NodeId>,
    states: Vec<BarycenterState>,
    sort_entries: Vec<BarycenterSortEntry>,
}

#[derive(Clone, Copy)]
struct BarycenterSortEntry {
    node_id: NodeId,
    barycenter: Option<f64>,
}

pub(crate) fn compare_barycenter_values(a: Option<f64>, b: Option<f64>) -> Ordering {
    match (a, b) {
        (Some(x), Some(y)) => x.partial_cmp(&y).unwrap_or(Ordering::Equal),
        (Some(_), None) => Ordering::Less,
        (None, Some(_)) => Ordering::Greater,
        (None, None) => Ordering::Equal,
    }
}

impl BarycenterStateMap {
    const MISSING: u32 = u32::MAX;

    pub fn new() -> Self {
        Self {
            index_to_pos: Vec::new(),
            touched_indices: Vec::new(),
            nodes: Vec::new(),
            states: Vec::new(),
            sort_entries: Vec::new(),
        }
    }

    pub fn reset_with_nodes(&mut self, nodes: &[NodeId]) {
        self.clear();
        self.nodes.reserve(nodes.len());
        self.states.reserve(nodes.len());
        self.touched_indices.reserve(nodes.len());

        for &node_id in nodes {
            let index = Self::index(node_id);
            if index >= self.index_to_pos.len() {
                self.index_to_pos.resize(index + 1, Self::MISSING);
            }
            let pos = self.nodes.len() as u32;
            self.index_to_pos[index] = pos;
            self.touched_indices.push(index);
            self.nodes.push(node_id);
            self.states.push(BarycenterState::default());
        }
    }

    fn clear(&mut self) {
        for &index in &self.touched_indices {
            self.index_to_pos[index] = Self::MISSING;
        }
        self.touched_indices.clear();
        self.nodes.clear();
        self.states.clear();
    }

    #[inline]
    pub fn get(&self, node_id: NodeId) -> Option<&BarycenterState> {
        self.position(node_id).map(|pos| &self.states[pos])
    }

    #[inline]
    pub fn get_mut(&mut self, node_id: NodeId) -> Option<&mut BarycenterState> {
        self.position(node_id).map(|pos| &mut self.states[pos])
    }

    #[inline]
    pub(crate) fn position_of(&self, node_id: NodeId) -> Option<usize> {
        self.position(node_id)
    }

    #[inline]
    pub(crate) fn get_at(&self, pos: usize) -> &BarycenterState {
        &self.states[pos]
    }

    #[inline]
    pub(crate) fn get_at_mut(&mut self, pos: usize) -> &mut BarycenterState {
        &mut self.states[pos]
    }

    pub fn sort_nodes_by_barycenter(&mut self, nodes: &mut [NodeId]) {
        self.sort_entries.clear();
        self.sort_entries.reserve(nodes.len());

        for &node_id in nodes.iter() {
            let barycenter = self.get(node_id).and_then(|state| state.barycenter);
            self.sort_entries.push(BarycenterSortEntry { node_id, barycenter });
        }

        self.sort_entries
            .sort_by(|a, b| compare_barycenter_values(a.barycenter, b.barycenter));

        for (node_id, entry) in nodes.iter_mut().zip(self.sort_entries.iter()) {
            *node_id = entry.node_id;
        }
    }

    #[inline]
    fn position(&self, node_id: NodeId) -> Option<usize> {
        let index = Self::index(node_id);
        let &pos = self.index_to_pos.get(index)?;
        if pos == Self::MISSING {
            return None;
        }
        let pos = pos as usize;
        if self.nodes[pos] == node_id { Some(pos) } else { None }
    }

    #[inline]
    fn index(node_id: NodeId) -> usize {
        node_id.0.index() as usize
    }
}
