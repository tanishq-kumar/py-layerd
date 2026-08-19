//! Stateless free-function crossing counter implementations.
//!
//! Stateful caches (`portPositions[]` / `nodeCardinalities[]` between calls,
//! `switchPorts` / `switchNodes` notifications for incremental updates) are
//! intentionally absent: each entry point recomputes positions from the
//! current graph. This is correct because callers (e.g.
//! `LayerSweepCrossingMinimizer`) always re-call the counter after mutating
//! node order; the cache was a micro-optimization, not a semantic guarantee.

use std::mem;

use hashbrown::HashSet;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    p3_crossing_min::{
        binary_indexed_tree::BinaryIndexedTree,
        cross_min_util::iter_ports_in_nsew,
        scratch_stats::{self, CountingScratchFootprint},
    },
    properties::internal::{IN_LAYER_LAYOUT_UNIT, ORIGIN_NODE, ORIGIN_PORT},
};

const POS_NONE: u32 = u32::MAX;

/// Sparse port→position table backed by a flat `Vec<u32>` keyed by the
/// port's arena index. Avoids hashing a `u32` newtype (PortId) through
/// `RandomState::SipHash`, which dominated the hot pair-counting loops on
/// large fixtures. `max_position_hint` tracks the largest position ever
/// inserted so the BIT-sizing path doesn't need to scan the full table.
///
/// `clear()` resets only touched slots. This keeps the arena-indexed O(1)
/// lookup but drops the previous parallel epoch vector, which improves cache
/// locality and cuts retained scratch bytes for sparse post-dummy arenas.
#[derive(Default)]
struct PortPositions {
    inner: Vec<u32>,
    touched: Vec<PortId>,
    /// Upper bound on values currently in `inner`. Monotonic across `insert`
    /// and `add_assign`; left stale by `sub_assign` (a stale upper bound is
    /// safe for `bit_capacity`, which only needs a not-too-loose hint).
    max_position_hint: i64,
}

impl PortPositions {
    fn new() -> Self {
        Self { inner: Vec::new(), touched: Vec::new(), max_position_hint: -1 }
    }

    /// Reset the logical contents while keeping the underlying allocation.
    fn clear(&mut self) {
        let inner = &mut self.inner;
        for port_id in self.touched.drain(..) {
            if let Some(slot) = inner.get_mut(port_id.0.index() as usize) {
                *slot = POS_NONE;
            }
        }
        self.max_position_hint = -1;
    }

    #[inline]
    fn insert(&mut self, port_id: PortId, position: usize) {
        debug_assert!(position < POS_NONE as usize);
        let i = port_id.0.index() as usize;
        if i >= self.inner.len() {
            self.inner.resize(i + 1, POS_NONE);
        }
        if self.inner[i] == POS_NONE {
            self.touched.push(port_id);
        }
        // SAFETY: the resize above guarantees `i < self.inner.len()`.
        unsafe { *self.inner.get_unchecked_mut(i) = position as u32 };
        if (position as i64) > self.max_position_hint {
            self.max_position_hint = position as i64;
        }
    }

    #[inline]
    fn get(&self, port_id: &PortId) -> Option<usize> {
        let i = port_id.0.index() as usize;
        let value = *self.inner.get(i)?;
        (value != POS_NONE).then_some(value as usize)
    }

    #[inline]
    fn contains_key(&self, port_id: &PortId) -> bool {
        let i = port_id.0.index() as usize;
        self.inner.get(i).is_some_and(|&value| value != POS_NONE)
    }

    /// Add `delta` to the existing position of `port_id` if it is set.
    #[inline]
    fn add_assign(&mut self, port_id: PortId, delta: usize) {
        let i = port_id.0.index() as usize;
        let Some(slot) = self.inner.get_mut(i) else { return };
        if *slot == POS_NONE {
            return;
        }
        *slot = slot.saturating_add(delta as u32);
        if (*slot as i64) > self.max_position_hint {
            self.max_position_hint = *slot as i64;
        }
    }

    /// Saturating-subtract `delta` from the existing position of `port_id`
    /// if it is set.
    #[inline]
    fn sub_assign(&mut self, port_id: PortId, delta: usize) {
        let i = port_id.0.index() as usize;
        let Some(slot) = self.inner.get_mut(i) else { return };
        if *slot == POS_NONE {
            return;
        }
        *slot = slot.saturating_sub(delta as u32);
        // `max_position_hint` may now be a strict overestimate; that is
        // fine because `bit_capacity` only requires an upper bound.
    }

    /// Largest position currently assigned, or `None` if the table is empty.
    /// May overestimate after `sub_assign`; never underestimates.
    fn max_position(&self) -> Option<usize> {
        if self.max_position_hint >= 0 {
            Some(self.max_position_hint as usize)
        } else {
            None
        }
    }

    fn retained_bytes(&self) -> usize {
        self.inner.capacity() * mem::size_of::<u32>()
            + self.touched.capacity() * mem::size_of::<PortId>()
    }
}

/// Sparse node→count table backed by `Vec<u32>` indexed by node arena index.
/// Default value is `0`, so an absent cardinality contributes nothing.
/// `clear()` resets only nodes touched by the previous fill.
#[derive(Default)]
struct NodeCardinalities {
    inner: Vec<u32>,
    touched: Vec<NodeId>,
}

impl NodeCardinalities {
    fn new() -> Self {
        Self { inner: Vec::new(), touched: Vec::new() }
    }

    fn clear(&mut self) {
        let inner = &mut self.inner;
        for node_id in self.touched.drain(..) {
            if let Some(slot) = inner.get_mut(node_id.0.index() as usize) {
                *slot = 0;
            }
        }
    }

    #[inline]
    fn insert(&mut self, node_id: NodeId, value: usize) {
        if value == 0 {
            return;
        }
        let i = node_id.0.index() as usize;
        if i >= self.inner.len() {
            self.inner.resize(i + 1, 0);
        }
        if self.inner[i] == 0 {
            self.touched.push(node_id);
        }
        // SAFETY: the resize above guarantees `i < self.inner.len()`.
        unsafe { *self.inner.get_unchecked_mut(i) = value as u32 };
    }

    #[inline]
    fn get(&self, node_id: NodeId) -> usize {
        let i = node_id.0.index() as usize;
        self.inner.get(i).copied().unwrap_or(0) as usize
    }

    fn retained_bytes(&self) -> usize {
        self.inner.capacity() * mem::size_of::<u32>()
            + self.touched.capacity() * mem::size_of::<NodeId>()
    }
}

/// Bitset-style port-id presence table that resets only touched slots.
#[derive(Default)]
struct SeenPorts {
    flags: Vec<u8>,
    touched: Vec<PortId>,
}

impl SeenPorts {
    fn new() -> Self {
        Self { flags: Vec::new(), touched: Vec::new() }
    }

    fn clear(&mut self) {
        let flags = &mut self.flags;
        for port_id in self.touched.drain(..) {
            if let Some(slot) = flags.get_mut(port_id.0.index() as usize) {
                *slot = 0;
            }
        }
    }

    /// Mark `port_id` as seen. Returns `true` iff this call is the first
    /// time the port is seen before the next clear.
    #[inline]
    fn mark(&mut self, port_id: PortId) -> bool {
        let i = port_id.0.index() as usize;
        if i >= self.flags.len() {
            self.flags.resize(i + 1, 0);
        }
        // SAFETY: the resize above guarantees `i < self.flags.len()`.
        let slot = unsafe { self.flags.get_unchecked_mut(i) };
        let already = *slot != 0;
        if !already {
            *slot = 1;
            self.touched.push(port_id);
        }
        !already
    }

    fn retained_bytes(&self) -> usize {
        self.flags.capacity() * mem::size_of::<u8>()
            + self.touched.capacity() * mem::size_of::<PortId>()
    }
}

#[derive(Default)]
struct DensePortAdjacency {
    edges: Vec<SmallVec<PortId, 2>>,
    active: Vec<u8>,
    keys: Vec<PortId>,
}

impl DensePortAdjacency {
    fn new() -> Self {
        Self { edges: Vec::new(), active: Vec::new(), keys: Vec::new() }
    }

    fn clear(&mut self) {
        let active = &mut self.active;
        for port_id in self.keys.drain(..) {
            if let Some(slot) = active.get_mut(port_id.0.index() as usize) {
                *slot = 0;
            }
        }
    }

    #[inline]
    fn push(&mut self, port_id: PortId, adjacent: PortId) {
        let i = port_id.0.index() as usize;
        if i >= self.edges.len() {
            self.edges.resize_with(i + 1, SmallVec::new);
            self.active.resize(i + 1, 0);
        }
        if self.active[i] == 0 {
            self.active[i] = 1;
            self.edges[i].clear();
            self.keys.push(port_id);
        }
        self.edges[i].push(adjacent);
    }

    #[inline]
    fn get(&self, port_id: PortId) -> Option<&SmallVec<PortId, 2>> {
        let i = port_id.0.index() as usize;
        if self.active.get(i).copied() == Some(1) {
            // SAFETY: `active` and `edges` are resized together in `push`.
            Some(unsafe { self.edges.get_unchecked(i) })
        } else {
            None
        }
    }

    #[inline]
    fn keys(&self) -> impl Iterator<Item = PortId> + '_ {
        self.keys.iter().copied()
    }

    fn retained_bytes(&self) -> usize {
        let edge_slots = self.edges.capacity() * mem::size_of::<SmallVec<PortId, 2>>();
        let inline_spill = self
            .edges
            .iter()
            .filter(|edges| edges.spilled())
            .map(SmallVec::capacity)
            .sum::<usize>()
            * mem::size_of::<PortId>();
        edge_slots
            + inline_spill
            + self.active.capacity() * mem::size_of::<u8>()
            + self.keys.capacity() * mem::size_of::<PortId>()
    }

    fn slots(&self) -> usize {
        self.edges.len()
    }
}

/// Reusable allocations for the hot pair-counting loops.
///
/// Retained by `SwitchDecider` across its inner sweep: each
/// `does_switch_reduce_crossings` cache miss would otherwise `Vec::new()` four
/// different working buffers and drop them immediately. The sweep does tens
/// of thousands of such queries on large fixtures, and samply showed
/// `alloc::alloc::{alloc,realloc,dealloc}` accumulating to ~55 % of total
/// self time after the per-call HashMap → Vec migration. Passing a single
/// `CountingScratch` down into the counting helpers keeps every buffer
/// allocated once and just clears them between calls.
pub struct CountingScratch {
    port_positions: PortPositions,
    ports: Vec<PortId>,
    node_cardinalities: NodeCardinalities,
    relevant_ports: Vec<PortId>,
    seen_ports: SeenPorts,
    /// Pooled Fenwick tree reused by every inner crossing counter (replaces
    /// `BinaryIndexedTree::new` per call). Reset via
    /// `BinaryIndexedTree::reset` so the underlying `Vec<i32>` capacity is
    /// kept across calls when it already covers the requested `max_num`.
    bit: BinaryIndexedTree,
    /// Deferred-add scratch buffer for the BIT loop. Cleared (not reallocated)
    /// between counter calls.
    deferred_ends: Vec<usize>,
    /// Reusable position buffers + AdjacencyCursors for between-layer pair
    /// counting. Single shared pair refilled on each call; avoids the
    /// per-pair `Vec<usize>` and `Vec<AdjacencyRun>` allocations the prior
    /// code paid in `count_crossings_between_pair_nodes_with_table`.
    adjacency_upper: Vec<usize>,
    adjacency_lower: Vec<usize>,
    upper_cursor: AdjacencyCursor,
    lower_cursor: AdjacencyCursor,
    hyperedge_adjacency: DensePortAdjacency,
    hyperedge_visited: SeenPorts,
    hyperedges: Vec<HyperedgeBounds>,
    hyperedge_stack: Vec<PortId>,
    hyperedge_ports: Vec<PortId>,
    hyperedge_corners: Vec<HyperedgeCorner>,
    ns_stack: Vec<NodeId>,
    inversion_values: Vec<i32>,
    inversion_unique: Vec<i32>,
}

impl CountingScratch {
    pub fn new() -> Self {
        Self {
            port_positions: PortPositions::new(),
            ports: Vec::new(),
            node_cardinalities: NodeCardinalities::new(),
            relevant_ports: Vec::new(),
            seen_ports: SeenPorts::new(),
            bit: BinaryIndexedTree::empty(),
            deferred_ends: Vec::new(),
            adjacency_upper: Vec::new(),
            adjacency_lower: Vec::new(),
            upper_cursor: AdjacencyCursor::new(),
            lower_cursor: AdjacencyCursor::new(),
            hyperedge_adjacency: DensePortAdjacency::new(),
            hyperedge_visited: SeenPorts::new(),
            hyperedges: Vec::new(),
            hyperedge_stack: Vec::new(),
            hyperedge_ports: Vec::new(),
            hyperedge_corners: Vec::new(),
            ns_stack: Vec::new(),
            inversion_values: Vec::new(),
            inversion_unique: Vec::new(),
        }
    }

    /// Clear the buffers that `init_positions` and its companions write into.
    /// `relevant_ports` / `seen_ports` are cleared at their use sites so we
    /// don't pay twice.
    fn reset_primary(&mut self) {
        self.port_positions.clear();
        self.node_cardinalities.clear();
        self.ports.clear();
    }

    fn footprint(&self) -> CountingScratchFootprint {
        let port_position_bytes = self.port_positions.retained_bytes();
        let node_cardinality_bytes = self.node_cardinalities.retained_bytes();
        let seen_bytes = self.seen_ports.retained_bytes() + self.hyperedge_visited.retained_bytes();
        let dense_adjacency_bytes = self.hyperedge_adjacency.retained_bytes();
        let bit_bytes = self.bit.retained_bytes();
        let port_vec_bytes = (self.ports.capacity()
            + self.relevant_ports.capacity()
            + self.hyperedge_stack.capacity()
            + self.hyperedge_ports.capacity())
            * mem::size_of::<PortId>();
        let adjacency_bytes = (self.adjacency_upper.capacity()
            + self.adjacency_lower.capacity()
            + self.deferred_ends.capacity())
            * mem::size_of::<usize>()
            + (self.upper_cursor.runs.capacity() + self.lower_cursor.runs.capacity())
                * mem::size_of::<AdjacencyRun>();
        let hyperedge_bytes = self.hyperedges.capacity() * mem::size_of::<HyperedgeBounds>()
            + self.hyperedge_corners.capacity() * mem::size_of::<HyperedgeCorner>();
        let ns_bytes = self.ns_stack.capacity() * mem::size_of::<NodeId>();
        let inversion_bytes = (self.inversion_values.capacity() + self.inversion_unique.capacity())
            * mem::size_of::<i32>();

        CountingScratchFootprint {
            retained_bytes: port_position_bytes
                + node_cardinality_bytes
                + seen_bytes
                + dense_adjacency_bytes
                + bit_bytes
                + port_vec_bytes
                + adjacency_bytes
                + hyperedge_bytes
                + ns_bytes
                + inversion_bytes,
            port_position_slots: self.port_positions.inner.len(),
            port_position_capacity: self.port_positions.inner.capacity(),
            node_cardinality_slots: self.node_cardinalities.inner.len(),
            node_cardinality_capacity: self.node_cardinalities.inner.capacity(),
            seen_port_slots: self.seen_ports.flags.len().max(self.hyperedge_visited.flags.len()),
            dense_adjacency_slots: self.hyperedge_adjacency.slots(),
            bit_capacity: self.bit.capacity(),
            ports_capacity: self.ports.capacity(),
            relevant_ports_capacity: self.relevant_ports.capacity(),
            adjacency_capacity: self
                .adjacency_upper
                .capacity()
                .max(self.adjacency_lower.capacity())
                .max(self.upper_cursor.runs.capacity())
                .max(self.lower_cursor.runs.capacity()),
            hyperedge_capacity: self
                .hyperedges
                .capacity()
                .max(self.hyperedge_ports.capacity())
                .max(self.hyperedge_corners.capacity()),
            ns_stack_capacity: self.ns_stack.capacity(),
        }
    }

    pub(crate) fn count_i32_inversions<I>(&mut self, values: I) -> usize
    where
        I: IntoIterator<Item = i32>,
    {
        self.inversion_values.clear();
        self.inversion_values.extend(values);
        match self.inversion_values.len() {
            0 | 1 => 0,
            2..=128 => count_i32_inversions_quadratic(&self.inversion_values),
            _ => {
                self.inversion_unique.clear();
                self.inversion_unique.extend_from_slice(&self.inversion_values);
                self.inversion_unique.sort_unstable();
                self.inversion_unique.dedup();

                self.bit.reset(self.inversion_unique.len());
                let mut inversions = 0usize;
                for (seen, &value) in self.inversion_values.iter().enumerate() {
                    let rank = self
                        .inversion_unique
                        .binary_search(&value)
                        .expect("rank source and unique values are the same");
                    let less_or_equal = self.bit.rank(rank + 1);
                    inversions += seen - less_or_equal;
                    self.bit.add(rank);
                }
                inversions
            }
        }
    }
}

fn count_i32_inversions_quadratic(values: &[i32]) -> usize {
    let mut inversions = 0usize;
    for i in 0..values.len() {
        for j in (i + 1)..values.len() {
            if values[i] > values[j] {
                inversions += 1;
            }
        }
    }
    inversions
}

impl Default for CountingScratch {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for CountingScratch {
    fn drop(&mut self) {
        if scratch_stats::enabled() {
            scratch_stats::record_counting(self.footprint());
        }
    }
}

#[derive(Clone, Copy)]
struct AdjacencyRun {
    position: usize,
    total_cardinality: usize,
    current_cardinality: usize,
}

pub struct AdjacencyCursor {
    runs: Vec<AdjacencyRun>,
    current_index: usize,
    current_size: usize,
}

impl Default for AdjacencyCursor {
    fn default() -> Self {
        Self::new()
    }
}

impl AdjacencyCursor {
    pub fn new() -> Self {
        Self { runs: Vec::new(), current_index: 0, current_size: 0 }
    }

    /// Reset and rebuild from a sorted run-length encoding of `positions`.
    /// Reuses `self.runs` capacity so the n² greedy-switch loop does not
    /// allocate a fresh `Vec<AdjacencyRun>` per pair query. `positions` is
    /// sorted in place and may be reused by the caller after this returns.
    fn clear_and_load_from(&mut self, positions: &mut [usize]) {
        positions.sort_unstable();
        self.runs.clear();
        for &position in positions.iter() {
            if let Some(last) = self.runs.last_mut()
                && last.position == position
            {
                last.total_cardinality += 1;
                last.current_cardinality += 1;
                continue;
            }
            self.runs
                .push(AdjacencyRun { position, total_cardinality: 1, current_cardinality: 1 });
        }
        self.current_index = 0;
        self.current_size = self.runs.iter().map(|run| run.total_cardinality).sum();
    }

    fn is_empty(&self) -> bool {
        self.current_size == 0
    }

    fn first(&self) -> usize {
        self.runs[self.current_index].position
    }

    fn size(&self) -> usize {
        self.current_size
    }

    fn count_below_first_position(&self) -> usize {
        self.current_size - self.runs[self.current_index].current_cardinality
    }

    fn remove_first(&mut self) {
        if self.is_empty() {
            return;
        }

        let current = &mut self.runs[self.current_index];
        current.current_cardinality -= 1;
        self.current_size -= 1;

        if current.current_cardinality == 0 {
            self.current_index += 1;
            if self.current_index < self.runs.len() {
                self.runs[self.current_index].current_cardinality =
                    self.runs[self.current_index].total_cardinality;
            }
        }
    }
}

/// Scratch-buffer variant of `count_all_crossings`.
///
/// Reuses one `CountingScratch` across every per-layer / per-pair counter
/// invocation. `effective_score` in the layer-sweep crossing minimizer
/// fires `count_all_crossings` on every initial / per-sweep / inner-loop
/// score check, so passing in a single scratch amortises the per-layer
/// `PortPositions` / `NodeCardinalities` / `Vec<PortId>` allocations that
/// the allocating wrappers would otherwise cycle through.
pub fn count_all_crossings_with_scratch(graph: &LGraph, scratch: &mut CountingScratch) -> usize {
    if graph.layers.is_empty() {
        return 0;
    }

    let mut crossings =
        count_in_layer_crossings_on_side_with_scratch(graph, 0, PortSide::West, scratch);
    let last_layer_idx = graph.layers.len() - 1;
    crossings += count_in_layer_crossings_on_side_with_scratch(
        graph,
        last_layer_idx,
        PortSide::East,
        scratch,
    );

    for layer_idx in 0..graph.layers.len() {
        crossings += count_crossings_at_with_scratch(graph, layer_idx, scratch);
    }

    crossings
}

#[derive(Clone, Copy, Debug)]
struct HyperedgeBounds {
    upper_left: usize,
    lower_left: usize,
    upper_right: usize,
    lower_right: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HyperedgeCornerType {
    Upper,
    Lower,
}

#[derive(Clone, Copy, Debug)]
struct HyperedgeCorner {
    hyperedge_idx: usize,
    position: usize,
    opposite_position: usize,
    corner_type: HyperedgeCornerType,
}

pub(crate) fn count_crossings_at_with_scratch(
    graph: &LGraph,
    layer_idx: usize,
    scratch: &mut CountingScratch,
) -> usize {
    let has_hyperedges = layer_idx + 1 < graph.layers.len()
        && has_hyperedges_between_layers(graph, layer_idx, layer_idx + 1);
    count_crossings_at_with_hyperedge_hint(graph, layer_idx, scratch, has_hyperedges)
}

pub(crate) fn count_crossings_at_with_hyperedge_hint(
    graph: &LGraph,
    layer_idx: usize,
    scratch: &mut CountingScratch,
    has_hyperedges: bool,
) -> usize {
    let mut total = 0;

    if layer_idx + 1 < graph.layers.len() {
        if has_hyperedges {
            total += count_hyperedge_crossings_between_layers_with_scratch(
                graph,
                layer_idx,
                layer_idx + 1,
                scratch,
            );
            total += count_in_layer_crossings_on_side_with_scratch(
                graph,
                layer_idx,
                PortSide::East,
                scratch,
            );
            total += count_in_layer_crossings_on_side_with_scratch(
                graph,
                layer_idx + 1,
                PortSide::West,
                scratch,
            );
        } else {
            total += count_crossings_between_layers_with_scratch(
                graph,
                layer_idx,
                layer_idx + 1,
                scratch,
            );
        }
    }

    total + count_north_south_port_crossings_in_layer_with_scratch(graph, layer_idx, scratch)
}

pub(crate) fn has_hyperedges_between_layers(
    graph: &LGraph,
    left_idx: usize,
    right_idx: usize,
) -> bool {
    if left_idx >= graph.layers.len()
        || right_idx >= graph.layers.len()
        || left_idx + 1 != right_idx
    {
        return false;
    }

    layer_has_hyperedge_port_on_side(graph, left_idx, PortSide::East)
        || layer_has_hyperedge_port_on_side(graph, right_idx, PortSide::West)
}

fn layer_has_hyperedge_port_on_side(graph: &LGraph, layer_idx: usize, side: PortSide) -> bool {
    graph.layers[layer_idx].nodes.iter().copied().any(|node_id| {
        let node = graph.node(node_id);
        if node.is_port_side_cached() {
            node.ports_on_side(side).iter().copied().any(|port_id| {
                let port = graph.port(port_id);
                port.incoming_edges.len() + port.outgoing_edges.len() > 1
            })
        } else {
            node.ports.iter().copied().any(|port_id| {
                let port = graph.port(port_id);
                port.side == side && port.incoming_edges.len() + port.outgoing_edges.len() > 1
            })
        }
    })
}

fn count_hyperedge_crossings_between_layers_with_scratch(
    graph: &LGraph,
    left_idx: usize,
    right_idx: usize,
    scratch: &mut CountingScratch,
) -> usize {
    if left_idx >= graph.layers.len()
        || right_idx >= graph.layers.len()
        || left_idx + 1 != right_idx
    {
        return 0;
    }

    let left_layer = &graph.layers[left_idx].nodes;
    let right_layer = &graph.layers[right_idx].nodes;
    if left_layer.is_empty() || right_layer.is_empty() {
        return 0;
    }

    scratch.port_positions.clear();
    let source_count =
        assign_hyperedge_left_positions(graph, left_layer, left_idx, &mut scratch.port_positions);
    let target_count = assign_hyperedge_right_positions(
        graph,
        right_layer,
        right_idx,
        &mut scratch.port_positions,
    );

    collect_hyperedge_bounds(
        graph,
        left_layer,
        left_idx,
        right_idx,
        &scratch.port_positions,
        source_count,
        target_count,
        &mut scratch.hyperedge_adjacency,
        &mut scratch.hyperedge_visited,
        &mut scratch.hyperedge_stack,
        &mut scratch.hyperedge_ports,
        &mut scratch.hyperedges,
    );
    if scratch.hyperedges.is_empty() {
        return 0;
    }

    scratch.hyperedges.sort_by(|left, right| {
        left.upper_left
            .cmp(&right.upper_left)
            .then(left.upper_right.cmp(&right.upper_right))
    });

    let mut crossings = 0;
    for i in 0..scratch.hyperedges.len() {
        for j in (i + 1)..scratch.hyperedges.len() {
            if scratch.hyperedges[i].upper_right > scratch.hyperedges[j].upper_right {
                crossings += 1;
            }
        }
    }

    crossings
        + count_hyperedge_overlap_crossings(
            &scratch.hyperedges,
            true,
            &mut scratch.hyperedge_corners,
        )
        + count_hyperedge_overlap_crossings(
            &scratch.hyperedges,
            false,
            &mut scratch.hyperedge_corners,
        )
}

fn assign_hyperedge_left_positions(
    graph: &LGraph,
    left_layer: &[NodeId],
    left_idx: usize,
    port_positions: &mut PortPositions,
) -> usize {
    let mut source_count = 0;

    for &node_id in left_layer {
        for &port_id in &graph.node(node_id).ports {
            let has_between_layer_edge =
                graph.port(port_id).outgoing_edges.iter().any(|&edge_id| {
                    graph
                        .node(graph.edge(edge_id).target_owner)
                        .layer
                        .is_some_and(|layer| layer != left_idx)
                });
            if has_between_layer_edge {
                port_positions.insert(port_id, source_count);
                source_count += 1;
            }
        }
    }

    source_count
}

fn assign_hyperedge_right_positions(
    graph: &LGraph,
    right_layer: &[NodeId],
    right_idx: usize,
    port_positions: &mut PortPositions,
) -> usize {
    let mut target_count = 0;

    for &node_id in right_layer {
        let node_ports = &graph.node(node_id).ports;
        let mut north_input_ports = 0usize;
        for &port_id in node_ports {
            let port = graph.port(port_id);
            if port.side != PortSide::North {
                break;
            }
            let has_between_layer_edge = port.incoming_edges.iter().any(|&edge_id| {
                graph
                    .node(graph.edge(edge_id).source_owner)
                    .layer
                    .is_some_and(|layer| layer != right_idx)
            });
            if has_between_layer_edge {
                north_input_ports += 1;
            }
        }

        let mut other_input_ports = 0usize;
        for &port_id in node_ports.iter().rev() {
            let has_between_layer_edge =
                graph.port(port_id).incoming_edges.iter().any(|&edge_id| {
                    graph
                        .node(graph.edge(edge_id).source_owner)
                        .layer
                        .is_some_and(|layer| layer != right_idx)
                });
            if !has_between_layer_edge {
                continue;
            }

            if graph.port(port_id).side == PortSide::North {
                port_positions.insert(port_id, target_count);
                target_count += 1;
            } else {
                port_positions
                    .insert(port_id, target_count + north_input_ports + other_input_ports);
                other_input_ports += 1;
            }
        }

        target_count += other_input_ports;
    }

    target_count
}

fn collect_hyperedge_bounds(
    graph: &LGraph,
    left_layer: &[NodeId],
    left_idx: usize,
    right_idx: usize,
    port_positions: &PortPositions,
    source_count: usize,
    target_count: usize,
    adjacency: &mut DensePortAdjacency,
    visited: &mut SeenPorts,
    stack: &mut Vec<PortId>,
    ports: &mut Vec<PortId>,
    hyperedges: &mut Vec<HyperedgeBounds>,
) {
    adjacency.clear();
    visited.clear();
    stack.clear();
    ports.clear();
    hyperedges.clear();

    for &node_id in left_layer {
        for &source_port in &graph.node(node_id).ports {
            for &edge_id in &graph.port(source_port).outgoing_edges {
                let edge = graph.edge(edge_id);
                let target_port = edge.target;
                let target_node = edge.target_owner;
                if graph.node(target_node).layer != Some(right_idx) {
                    continue;
                }

                adjacency.push(source_port, target_port);
                adjacency.push(target_port, source_port);
            }
        }
    }

    for start_port in adjacency.keys() {
        if !visited.mark(start_port) {
            continue;
        }

        stack.clear();
        ports.clear();
        stack.push(start_port);
        while let Some(port_id) = stack.pop() {
            ports.push(port_id);
            if let Some(neighbors) = adjacency.get(port_id) {
                for &next_port in neighbors {
                    if visited.mark(next_port) {
                        stack.push(next_port);
                    }
                }
            }
        }

        let mut bounds = HyperedgeBounds {
            upper_left: source_count,
            lower_left: 0,
            upper_right: target_count,
            lower_right: 0,
        };

        for &port_id in ports.iter() {
            let Some(position) = port_positions.get(&port_id) else {
                continue;
            };
            match graph.node(graph.port(port_id).owner).layer.get() {
                Some(layer) if layer == left_idx => {
                    bounds.upper_left = bounds.upper_left.min(position);
                    bounds.lower_left = bounds.lower_left.max(position);
                }
                Some(layer) if layer == right_idx => {
                    bounds.upper_right = bounds.upper_right.min(position);
                    bounds.lower_right = bounds.lower_right.max(position);
                }
                _ => {}
            }
        }

        hyperedges.push(bounds);
    }
}

fn count_hyperedge_overlap_crossings(
    hyperedges: &[HyperedgeBounds],
    left_side: bool,
    corners: &mut Vec<HyperedgeCorner>,
) -> usize {
    corners.clear();
    corners.reserve(hyperedges.len() * 2);

    for (hyperedge_idx, hyperedge) in hyperedges.iter().enumerate() {
        let (upper, lower) = if left_side {
            (hyperedge.upper_left, hyperedge.lower_left)
        } else {
            (hyperedge.upper_right, hyperedge.lower_right)
        };
        corners.push(HyperedgeCorner {
            hyperedge_idx,
            position: upper,
            opposite_position: lower,
            corner_type: HyperedgeCornerType::Upper,
        });
        corners.push(HyperedgeCorner {
            hyperedge_idx,
            position: lower,
            opposite_position: upper,
            corner_type: HyperedgeCornerType::Lower,
        });
    }

    corners.sort_by(|left, right| {
        left.position
            .cmp(&right.position)
            .then(left.opposite_position.cmp(&right.opposite_position))
            .then(left.hyperedge_idx.cmp(&right.hyperedge_idx))
            .then(match (left.corner_type, right.corner_type) {
                (HyperedgeCornerType::Upper, HyperedgeCornerType::Lower) =>
                    std::cmp::Ordering::Less,
                (HyperedgeCornerType::Lower, HyperedgeCornerType::Upper) =>
                    std::cmp::Ordering::Greater,
                _ => std::cmp::Ordering::Equal,
            })
    });

    let mut open_hyperedges = 0usize;
    let mut crossings = 0usize;
    for corner in corners.iter().copied() {
        match corner.corner_type {
            HyperedgeCornerType::Upper => open_hyperedges += 1,
            HyperedgeCornerType::Lower => {
                open_hyperedges = open_hyperedges.saturating_sub(1);
                crossings += open_hyperedges;
            }
        }
    }

    crossings
}

/// Scratch-buffer variant of `count_crossings_between_layers`.
pub fn count_crossings_between_layers_with_scratch(
    graph: &LGraph,
    left_idx: usize,
    right_idx: usize,
    scratch: &mut CountingScratch,
) -> usize {
    if left_idx >= graph.layers.len() || right_idx >= graph.layers.len() || left_idx >= right_idx {
        return 0;
    }

    scratch.reset_primary();
    init_positions(
        graph,
        &graph.layers[left_idx].nodes,
        &mut scratch.ports,
        &mut scratch.port_positions,
        PortSide::East,
        true,
        None,
    );
    init_positions(
        graph,
        &graph.layers[right_idx].nodes,
        &mut scratch.ports,
        &mut scratch.port_positions,
        PortSide::West,
        false,
        None,
    );

    count_crossings_on_ports(
        graph,
        &scratch.port_positions,
        &scratch.ports,
        &mut scratch.bit,
        &mut scratch.deferred_ends,
    )
}

/// Scratch-buffer variant of `count_in_layer_crossings_on_side`.
pub fn count_in_layer_crossings_on_side_with_scratch(
    graph: &LGraph,
    layer_idx: usize,
    side: PortSide,
    scratch: &mut CountingScratch,
) -> usize {
    if layer_idx >= graph.layers.len() {
        return 0;
    }

    scratch.reset_primary();
    init_positions(
        graph,
        &graph.layers[layer_idx].nodes,
        &mut scratch.ports,
        &mut scratch.port_positions,
        side,
        true,
        Some(&mut scratch.node_cardinalities),
    );

    count_in_layer_crossings_on_ports(
        graph,
        &scratch.port_positions,
        &scratch.ports,
        &mut scratch.bit,
        &mut scratch.deferred_ends,
    )
}

/// Scratch-buffer variant of `count_north_south_port_crossings_in_layer`.
pub fn count_north_south_port_crossings_in_layer_with_scratch(
    graph: &LGraph,
    layer_idx: usize,
    scratch: &mut CountingScratch,
) -> usize {
    if layer_idx >= graph.layers.len() {
        return 0;
    }
    let layer = &graph.layers[layer_idx].nodes;
    // Skip the whole counting path if the layer has no N/S dummies.
    if !layer.iter().any(|&nid| graph.node(nid).node_type == NodeType::NorthSouthPort) {
        return 0;
    }

    scratch.reset_primary();
    init_positions_for_north_south_counting(
        graph,
        layer,
        &mut scratch.ports,
        &mut scratch.port_positions,
        &mut scratch.ns_stack,
    );

    count_north_south_crossings_on_ports(
        graph,
        &scratch.port_positions,
        &scratch.ports,
        &mut scratch.bit,
        &mut scratch.deferred_ends,
    )
}

/// Count crossings contributed by two concrete nodes on one side of a free layer.
///
/// Pair-based between-layer counter used by greedy switching.
pub fn count_crossings_between_pair_nodes(
    graph: &LGraph,
    free_layer_idx: usize,
    upper_node: NodeId,
    lower_node: NodeId,
    side: PortSide,
) -> (usize, usize) {
    let mut scratch = CountingScratch::new();
    count_crossings_between_pair_nodes_with_scratch(
        graph,
        free_layer_idx,
        upper_node,
        lower_node,
        side,
        &mut scratch,
    )
}

/// Scratch-buffer variant of `count_crossings_between_pair_nodes`.
pub fn count_crossings_between_pair_nodes_with_scratch(
    graph: &LGraph,
    free_layer_idx: usize,
    upper_node: NodeId,
    lower_node: NodeId,
    side: PortSide,
    scratch: &mut CountingScratch,
) -> (usize, usize) {
    count_crossings_for_side(graph, free_layer_idx, upper_node, lower_node, side, scratch)
}

/// Sequential port-position table for one side of one layer. Used as a
/// precomputed cache for the neighbour layer during a greedy-switch sweep:
/// the neighbour's ports never change while the free layer is being
/// reordered, so building the table once and passing it into the pair
/// counter is substantially cheaper than rerunning
/// `set_port_positions_for_layer` per query.
pub struct NeighborPortTable {
    positions: PortPositions,
}

impl NeighborPortTable {
    /// Build the table for `graph.layers[layer_idx]` on the given side.
    pub fn build(graph: &LGraph, layer_idx: usize, side: PortSide) -> Self {
        let mut positions = PortPositions::new();
        set_port_positions_for_layer(graph, &graph.layers[layer_idx].nodes, side, &mut positions);
        Self { positions }
    }
}

/// Cache-aware variant of `count_crossings_between_pair_nodes`: reuses a
/// pre-built `NeighborPortTable` so the hot sweep no longer re-runs
/// `set_port_positions_for_layer` per pair query.
///
/// `free_side` is the side on the *free* layer (WEST means the neighbour
/// sits on the left, EAST means it sits on the right). The caller is
/// responsible for passing a table built against the neighbouring layer
/// on the opposite side (East table for WEST, West table for EAST).
pub fn count_crossings_between_pair_nodes_with_table(
    graph: &LGraph,
    upper_node: NodeId,
    lower_node: NodeId,
    free_side: PortSide,
    neighbor_table: &NeighborPortTable,
    scratch: &mut CountingScratch,
) -> (usize, usize) {
    adjacency_positions_for_node(
        graph,
        upper_node,
        free_side,
        &neighbor_table.positions,
        &mut scratch.adjacency_upper,
    );
    adjacency_positions_for_node(
        graph,
        lower_node,
        free_side,
        &neighbor_table.positions,
        &mut scratch.adjacency_lower,
    );
    if scratch.adjacency_upper.is_empty() || scratch.adjacency_lower.is_empty() {
        return (0, 0);
    }
    scratch.upper_cursor.clear_and_load_from(&mut scratch.adjacency_upper);
    scratch.lower_cursor.clear_and_load_from(&mut scratch.adjacency_lower);
    count_crossings_by_merging_adjacency_lists(&mut scratch.upper_cursor, &mut scratch.lower_cursor)
}

/// Pre-built per-side port positions and node cardinalities for an entire
/// layer. Callers (currently `SwitchDecider`) build one of these per side
/// at the start of a greedy-switch sweep and reuse it across every
/// `(upper_idx, lower_idx)` pair query for that side.
///
/// Validity: the snapshot reflects the layer order at construction time.
/// `SwitchDecider` mutates only its own `free_layer` copy during the sweep
/// (the underlying `graph.layers[layer_idx].nodes` and `graph.node(...).ports`
/// stay constant until `apply_to_graph` writes the final order back), so the
/// snapshot is valid for the decider's full lifetime.
pub struct LayerSidePositions {
    port_positions: PortPositions,
    node_cardinalities: NodeCardinalities,
}

impl LayerSidePositions {
    /// Build the snapshot for `graph.layers[layer_idx]` on the given side.
    /// Returns an empty snapshot if `layer_idx` is out of range.
    pub fn build(graph: &LGraph, layer_idx: usize, side: PortSide) -> Self {
        let mut port_positions = PortPositions::new();
        let mut node_cardinalities = NodeCardinalities::new();
        let mut ports = Vec::new();
        if layer_idx < graph.layers.len() {
            init_positions(
                graph,
                &graph.layers[layer_idx].nodes,
                &mut ports,
                &mut port_positions,
                side,
                true,
                Some(&mut node_cardinalities),
            );
        }
        Self { port_positions, node_cardinalities }
    }

    /// Permanently apply a node-pair swap to the cached positions.
    ///
    /// Called from `SwitchDecider::notify_of_switch` after the greedy switch
    /// loop commits a swap of two adjacent nodes in the free layer. Every
    /// other node's port position changes relative to the swapped pair, and
    /// the cached snapshot must reflect that or subsequent pair queries see
    /// stale in-layer crossing counts and the outer `minimize_no_counter`
    /// loop can oscillate forever (forward sweep prefers `[u, l]`, backward
    /// sweep prefers `[l, u]`).
    ///
    /// `upper` is the node that sat above `lower` before the swap, in the
    /// order `notify_of_switch` is invoked (`upper_idx, lower_idx` where
    /// `lower_idx == upper_idx + 1`). The cardinalities indicate how many
    /// ports each node contributes on `side`.
    pub fn switch_pair(&mut self, graph: &LGraph, upper: NodeId, lower: NodeId, side: PortSide) {
        let lower_card = self.node_cardinalities.get(lower);
        let upper_card = self.node_cardinalities.get(upper);
        for port_id in iter_ports_in_nsew(graph, upper, side) {
            self.port_positions.add_assign(port_id, lower_card);
        }
        for port_id in iter_ports_in_nsew(graph, lower, side) {
            self.port_positions.sub_assign(port_id, upper_card);
        }
    }
}

/// Cache-aware pair counter that skips the per-call `init_positions` sweep
/// and instead reads pre-computed positions from a `LayerSidePositions`
/// snapshot.
///
/// Takes the snapshot by `&mut` because the inner pair query mutates
/// `port_positions` in place (then restores it) for an incremental update.
/// Callers see the snapshot in its original state when the function returns.
pub fn count_in_layer_crossings_between_pair_on_side_with_cached_layer(
    graph: &LGraph,
    layer_idx: usize,
    upper_idx: usize,
    lower_idx: usize,
    side: PortSide,
    layer_positions: &mut LayerSidePositions,
    scratch: &mut CountingScratch,
) -> (usize, usize) {
    if layer_idx >= graph.layers.len() {
        return (0, 0);
    }
    let nodes = &graph.layers[layer_idx].nodes;
    if upper_idx >= nodes.len() || lower_idx >= nodes.len() || upper_idx == lower_idx {
        return (0, 0);
    }
    let upper_node = nodes[upper_idx];
    let lower_node = nodes[lower_idx];

    pair_counts_from_positions(
        graph,
        side,
        upper_node,
        lower_node,
        &mut layer_positions.port_positions,
        &layer_positions.node_cardinalities,
        scratch,
    )
}

/// Shared body for the in-layer pair counter once positions are available.
///
/// Both the per-call (`init_positions`) and cached (`LayerSidePositions`)
/// entry points end up here. Mutates `scratch.relevant_ports` and
/// `scratch.seen_ports`. Mutates `port_positions` in place during the swap
/// query and restores it before returning, so callers see a stable view.
///
/// Updates the shared port-position table by per-port deltas instead of
/// cloning the whole table on every pair query (the prior implementation
/// paid an `O(layer_ports)` `clone_from` per call inside the n²
/// greedy-switch loop).
#[inline]
fn pair_counts_from_positions(
    graph: &LGraph,
    side: PortSide,
    upper_node: NodeId,
    lower_node: NodeId,
    port_positions: &mut PortPositions,
    node_cardinalities: &NodeCardinalities,
    scratch: &mut CountingScratch,
) -> (usize, usize) {
    scratch.relevant_ports.clear();
    connected_in_layer_ports_sorted_by_position_into_raw(
        graph,
        upper_node,
        lower_node,
        side,
        port_positions,
        &mut scratch.relevant_ports,
        &mut scratch.seen_ports,
    );
    let upper_lower = count_in_layer_crossings_on_ports(
        graph,
        port_positions,
        &scratch.relevant_ports,
        &mut scratch.bit,
        &mut scratch.deferred_ends,
    );

    let lower_cardinality = node_cardinalities.get(lower_node);
    let upper_cardinality = node_cardinalities.get(upper_node);

    for port_id in iter_ports_in_nsew(graph, upper_node, side) {
        port_positions.add_assign(port_id, lower_cardinality);
    }
    for port_id in iter_ports_in_nsew(graph, lower_node, side) {
        port_positions.sub_assign(port_id, upper_cardinality);
    }

    scratch
        .relevant_ports
        .sort_by_key(|port_id| port_positions.get(port_id).unwrap_or(usize::MAX));
    let lower_upper = count_in_layer_crossings_on_ports(
        graph,
        port_positions,
        &scratch.relevant_ports,
        &mut scratch.bit,
        &mut scratch.deferred_ends,
    );

    for port_id in iter_ports_in_nsew(graph, upper_node, side) {
        port_positions.sub_assign(port_id, lower_cardinality);
    }
    for port_id in iter_ports_in_nsew(graph, lower_node, side) {
        port_positions.add_assign(port_id, upper_cardinality);
    }

    (upper_lower, lower_upper)
}

/// Count crossings caused by two ports for both relative orders.
pub fn count_crossings_between_ports_in_both_orders(
    graph: &LGraph,
    layer_idx: usize,
    upper_port: PortId,
    lower_port: PortId,
    side: PortSide,
) -> (usize, usize) {
    let mut port_positions = PortPositions::new();
    let mut scratch_ports = Vec::new();
    match side {
        PortSide::West if layer_idx > 0 => {
            init_positions(
                graph,
                &graph.layers[layer_idx - 1].nodes,
                &mut scratch_ports,
                &mut port_positions,
                PortSide::East,
                true,
                None,
            );
            init_positions(
                graph,
                &graph.layers[layer_idx].nodes,
                &mut scratch_ports,
                &mut port_positions,
                PortSide::West,
                false,
                None,
            );
        }
        PortSide::East if layer_idx + 1 < graph.layers.len() => {
            init_positions(
                graph,
                &graph.layers[layer_idx].nodes,
                &mut scratch_ports,
                &mut port_positions,
                PortSide::East,
                true,
                None,
            );
            init_positions(
                graph,
                &graph.layers[layer_idx + 1].nodes,
                &mut scratch_ports,
                &mut port_positions,
                PortSide::West,
                false,
                None,
            );
        }
        PortSide::West | PortSide::East => {
            init_positions(
                graph,
                &graph.layers[layer_idx].nodes,
                &mut scratch_ports,
                &mut port_positions,
                side,
                side == PortSide::East,
                None,
            );
        }
        _ => return (0, 0),
    }

    let mut ports =
        connected_ports_sorted_by_position(graph, upper_port, lower_port, &port_positions);
    let upper_lower = count_crossings_on_selected_ports(graph, &port_positions, &ports);

    if let (Some(upper_position), Some(lower_position)) =
        (port_positions.get(&upper_port), port_positions.get(&lower_port))
    {
        port_positions.insert(upper_port, lower_position);
        port_positions.insert(lower_port, upper_position);
    }

    ports.sort_by_key(|port_id| port_positions.get(port_id).unwrap_or(usize::MAX));
    let lower_upper = count_crossings_on_selected_ports(graph, &port_positions, &ports);

    (upper_lower, lower_upper)
}

fn count_crossings_on_ports(
    graph: &LGraph,
    port_positions: &PortPositions,
    ports: &[PortId],
    bit: &mut BinaryIndexedTree,
    deferred_ends: &mut Vec<usize>,
) -> usize {
    bit.reset(bit_capacity(port_positions));
    deferred_ends.clear();
    let mut crossings = 0;

    for &port_id in ports {
        let Some(position) = port_positions.get(&port_id) else {
            continue;
        };

        bit.remove_all(position);

        for edge_id in connected_edges(graph, port_id) {
            if is_node_self_loop(graph, edge_id) {
                continue;
            }

            let other_end = other_end_of(graph, edge_id, port_id);
            let Some(end_position) = port_positions.get(&other_end) else {
                continue;
            };

            if end_position > position {
                crossings += bit.rank(end_position);
                deferred_ends.push(end_position);
            }
        }

        for end_position in deferred_ends.drain(..) {
            bit.add(end_position);
        }
    }

    crossings
}

fn count_in_layer_crossings_on_ports(
    graph: &LGraph,
    port_positions: &PortPositions,
    ports: &[PortId],
    bit: &mut BinaryIndexedTree,
    deferred_ends: &mut Vec<usize>,
) -> usize {
    bit.reset(bit_capacity(port_positions));
    deferred_ends.clear();
    let mut crossings = 0;

    for &port_id in ports {
        let Some(position) = port_positions.get(&port_id) else {
            continue;
        };

        bit.remove_all(position);

        let mut between_layer_edges = 0;
        for edge_id in connected_edges(graph, port_id) {
            if is_node_self_loop(graph, edge_id) {
                continue;
            }

            if is_in_layer(graph, edge_id) {
                let other_end = other_end_of(graph, edge_id, port_id);
                let Some(end_position) = port_positions.get(&other_end) else {
                    continue;
                };

                if end_position > position {
                    crossings += bit.rank(end_position);
                    deferred_ends.push(end_position);
                }
            } else {
                between_layer_edges += 1;
            }
        }

        crossings += bit.size() * between_layer_edges;

        for end_position in deferred_ends.drain(..) {
            bit.add(end_position);
        }
    }

    crossings
}

fn count_north_south_crossings_on_ports(
    graph: &LGraph,
    port_positions: &PortPositions,
    ports: &[PortId],
    bit: &mut BinaryIndexedTree,
    deferred_ends: &mut Vec<usize>,
) -> usize {
    bit.reset(bit_capacity(port_positions));
    deferred_ends.clear();
    let mut crossings = 0;

    for &port_id in ports {
        let Some(position) = port_positions.get(&port_id) else {
            continue;
        };

        bit.remove_all(position);

        let owner = graph.port(port_id).owner;
        let owner_type = graph.node(owner).node_type;
        let mut targets_and_degrees: smallvec::SmallVec<(PortId, usize), 6> =
            smallvec::SmallVec::new();

        match owner_type {
            NodeType::Normal =>
                if let Some(dummy) = graph.port(port_id).port_dummy {
                    // `port_dummy` can point into a nested LGraph when the
                    // compound preprocessor installed a cross-arena
                    // dummy; those dummies are counted by the nested
                    // graph's own P3 sweep, not ours.
                    if let Some(dummy_node) = graph.try_node(dummy) {
                        for &dummy_port in &dummy_node.ports {
                            targets_and_degrees.push((dummy_port, port_degree(graph, dummy_port)));
                        }
                    }
                },
            NodeType::LongEdge =>
                for &other_port in &graph.node(owner).ports {
                    if other_port != port_id {
                        targets_and_degrees.push((other_port, port_degree(graph, other_port)));
                    }
                },
            NodeType::NorthSouthPort => {
                if let Some(origin_port) = graph.port(port_id).properties.get(&ORIGIN_PORT) {
                    // `ORIGIN_PORT` may also reference a port in a nested
                    // LGraph from a dummy-port cross-arena case.
                    if graph.try_port(origin_port).is_some() {
                        targets_and_degrees.push((origin_port, port_degree(graph, port_id)));
                    }
                }
            }
            _ => {}
        }

        for (target, degree) in targets_and_degrees {
            let Some(end_position) = port_positions.get(&target) else {
                continue;
            };

            if end_position > position {
                crossings += bit.rank(end_position) * degree;
                deferred_ends.push(end_position);
            }
        }

        for end_position in deferred_ends.drain(..) {
            bit.add(end_position);
        }
    }

    crossings
}

fn count_crossings_for_side(
    graph: &LGraph,
    free_layer_idx: usize,
    upper_node: NodeId,
    lower_node: NodeId,
    side: PortSide,
    scratch: &mut CountingScratch,
) -> (usize, usize) {
    let neighbor_layer_idx = match side {
        PortSide::West => free_layer_idx.checked_sub(1),
        PortSide::East => Some(free_layer_idx + 1).filter(|&idx| idx < graph.layers.len()),
        _ => None,
    };
    let Some(neighbor_layer_idx) = neighbor_layer_idx else {
        return (0, 0);
    };

    scratch.port_positions.clear();
    let neighbor_side = if side == PortSide::West { PortSide::East } else { PortSide::West };
    set_port_positions_for_layer(
        graph,
        &graph.layers[neighbor_layer_idx].nodes,
        neighbor_side,
        &mut scratch.port_positions,
    );

    adjacency_positions_for_node(
        graph,
        upper_node,
        side,
        &scratch.port_positions,
        &mut scratch.adjacency_upper,
    );
    adjacency_positions_for_node(
        graph,
        lower_node,
        side,
        &scratch.port_positions,
        &mut scratch.adjacency_lower,
    );

    if scratch.adjacency_upper.is_empty() || scratch.adjacency_lower.is_empty() {
        return (0, 0);
    }

    scratch.upper_cursor.clear_and_load_from(&mut scratch.adjacency_upper);
    scratch.lower_cursor.clear_and_load_from(&mut scratch.adjacency_lower);
    count_crossings_by_merging_adjacency_lists(&mut scratch.upper_cursor, &mut scratch.lower_cursor)
}

fn connected_ports_sorted_by_position(
    graph: &LGraph,
    upper_port: PortId,
    lower_port: PortId,
    port_positions: &PortPositions,
) -> Vec<PortId> {
    let mut ports: Vec<PortId> = Vec::new();
    let mut seen = HashSet::new();

    for port_id in [upper_port, lower_port] {
        if port_positions.contains_key(&port_id) && seen.insert(port_id) {
            ports.push(port_id);
        }
        for edge_id in connected_edges(graph, port_id) {
            if is_port_self_loop(graph, edge_id) {
                continue;
            }
            let other_end = other_end_of(graph, edge_id, port_id);
            if port_positions.contains_key(&other_end) && seen.insert(other_end) {
                ports.push(other_end);
            }
        }
    }

    ports.sort_by_key(|port_id| port_positions.get(port_id).unwrap_or(usize::MAX));
    ports
}

/// Append the "relevant" connected-in-layer ports for a pair of nodes to
/// `out`, deduping via the `seen_flags` scratch bitmap keyed by port arena
/// index. The caller owns both buffers and is responsible for clearing
/// `out` before the call; `seen_flags` is reset internally.
fn connected_in_layer_ports_sorted_by_position_into_raw(
    graph: &LGraph,
    upper_node: NodeId,
    lower_node: NodeId,
    side: PortSide,
    port_positions: &PortPositions,
    out: &mut Vec<PortId>,
    seen_flags: &mut SeenPorts,
) {
    seen_flags.clear();

    for node_id in [upper_node, lower_node] {
        for port_id in iter_ports_in_nsew(graph, node_id, side) {
            for edge_id in connected_edges(graph, port_id) {
                if is_node_self_loop(graph, edge_id) {
                    continue;
                }
                if seen_flags.mark(port_id) {
                    out.push(port_id);
                }
                if is_in_layer(graph, edge_id) {
                    let other_end = other_end_of(graph, edge_id, port_id);
                    if port_positions.contains_key(&other_end) && seen_flags.mark(other_end) {
                        out.push(other_end);
                    }
                }
            }
        }
    }

    out.sort_by_key(|port_id| port_positions.get(port_id).unwrap_or(usize::MAX));
}

fn count_crossings_by_merging_adjacency_lists(
    upper_cursor: &mut AdjacencyCursor,
    lower_cursor: &mut AdjacencyCursor,
) -> (usize, usize) {
    let mut upper_lower = 0;
    let mut lower_upper = 0;

    while !upper_cursor.is_empty() && !lower_cursor.is_empty() {
        if upper_cursor.first() > lower_cursor.first() {
            upper_lower += upper_cursor.size();
            lower_cursor.remove_first();
        } else if lower_cursor.first() > upper_cursor.first() {
            lower_upper += lower_cursor.size();
            upper_cursor.remove_first();
        } else {
            upper_lower += upper_cursor.count_below_first_position();
            lower_upper += lower_cursor.count_below_first_position();
            upper_cursor.remove_first();
            lower_cursor.remove_first();
        }
    }

    (upper_lower, lower_upper)
}

/// Count NS-neighbour crossings contributed by two specific nodes in the same layer.
///
/// Returns `(upper_lower, lower_upper)`. Accepts arbitrary `NodeId` pairs; adjacency is
/// not assumed.
pub fn count_ns_crossings_between_nodes(
    graph: &LGraph,
    upper_node: NodeId,
    lower_node: NodeId,
) -> (usize, usize) {
    let upper_type = graph.node(upper_node).node_type;
    let lower_type = graph.node(lower_node).node_type;

    if upper_type == NodeType::NorthSouthPort
        && lower_type == NodeType::NorthSouthPort
        && !have_different_origins(graph, upper_node, lower_node)
    {
        if is_north_of_normal_node(graph, upper_node) {
            return count_two_north_south_dummies(graph, upper_node, lower_node);
        }
        return count_two_north_south_dummies(graph, lower_node, upper_node);
    }

    if upper_type == NodeType::NorthSouthPort && lower_type == NodeType::LongEdge {
        return if is_north_of_normal_node(graph, upper_node) { (1, 0) } else { (0, 1) };
    }

    if lower_type == NodeType::NorthSouthPort && upper_type == NodeType::LongEdge {
        return if is_north_of_normal_node(graph, lower_node) { (0, 1) } else { (1, 0) };
    }

    if upper_type == NodeType::Normal && lower_type == NodeType::LongEdge {
        return (
            number_of_north_south_edges(graph, upper_node, PortSide::South),
            number_of_north_south_edges(graph, upper_node, PortSide::North),
        );
    }

    if lower_type == NodeType::Normal && upper_type == NodeType::LongEdge {
        return (
            number_of_north_south_edges(graph, lower_node, PortSide::North),
            number_of_north_south_edges(graph, lower_node, PortSide::South),
        );
    }

    (0, 0)
}

fn count_two_north_south_dummies(
    graph: &LGraph,
    further_from_normal: NodeId,
    closer_to_normal: NodeId,
) -> (usize, usize) {
    if origin_port_position_of(graph, further_from_normal)
        > origin_port_position_of(graph, closer_to_normal)
    {
        (
            first_port_degree_on_side(graph, closer_to_normal, PortSide::East),
            first_port_degree_on_side(graph, further_from_normal, PortSide::West),
        )
    } else {
        (
            first_port_degree_on_side(graph, closer_to_normal, PortSide::West),
            first_port_degree_on_side(graph, further_from_normal, PortSide::East),
        )
    }
}

fn init_positions(
    graph: &LGraph,
    nodes: &[NodeId],
    ports: &mut Vec<PortId>,
    port_positions: &mut PortPositions,
    side: PortSide,
    top_down: bool,
    mut node_cardinalities: Option<&mut NodeCardinalities>,
) {
    let mut next_position = ports.len();

    if top_down {
        for &node_id in nodes {
            let count = append_ports_on_side_in_order(
                graph,
                node_id,
                side,
                top_down,
                ports,
                port_positions,
                &mut next_position,
            );
            if let Some(cardinalities) = node_cardinalities.as_deref_mut() {
                cardinalities.insert(node_id, count);
            }
        }
    } else {
        for &node_id in nodes.iter().rev() {
            let count = append_ports_on_side_in_order(
                graph,
                node_id,
                side,
                top_down,
                ports,
                port_positions,
                &mut next_position,
            );
            if let Some(cardinalities) = node_cardinalities.as_deref_mut() {
                cardinalities.insert(node_id, count);
            }
        }
    }
}

fn append_ports_on_side_in_order(
    graph: &LGraph,
    node_id: NodeId,
    side: PortSide,
    top_down: bool,
    ports: &mut Vec<PortId>,
    port_positions: &mut PortPositions,
    next_position: &mut usize,
) -> usize {
    if matches!(side, PortSide::Undefined) {
        return 0;
    }

    let mut count = 0;
    let mut push_port = |port_id: PortId| {
        port_positions.insert(port_id, *next_position);
        *next_position += 1;
        ports.push(port_id);
        count += 1;
    };

    let node = graph.node(node_id);
    let reverse_after = match side {
        PortSide::East => !top_down,
        _ => top_down,
    };

    if node.is_port_side_cached() {
        let side_ports = node.ports_on_side(side);
        if reverse_after {
            for &port_id in side_ports.iter().rev() {
                push_port(port_id);
            }
        } else {
            for &port_id in side_ports {
                push_port(port_id);
            }
        }
    } else if reverse_after {
        for &port_id in node.ports.iter().rev() {
            if graph.port(port_id).side == side {
                push_port(port_id);
            }
        }
    } else {
        for &port_id in &node.ports {
            if graph.port(port_id).side == side {
                push_port(port_id);
            }
        }
    }

    count
}

fn init_positions_for_north_south_counting(
    graph: &LGraph,
    nodes: &[NodeId],
    ports: &mut Vec<PortId>,
    port_positions: &mut PortPositions,
    stack: &mut Vec<NodeId>,
) {
    const INDEXING_SIDE: PortSide = PortSide::West;
    const STACK_SIDE: PortSide = PortSide::East;

    stack.clear();
    let mut last_layout_unit = None;
    let mut next_position = 0;

    for &node_id in nodes {
        if is_layout_unit_changed(graph, last_layout_unit, node_id) {
            next_position =
                empty_stack(graph, stack, STACK_SIDE, ports, port_positions, next_position);
        }

        // A `set(key, None)` removes the entry, so a missing
        // `IN_LAYER_LAYOUT_UNIT` for `LongEdge` dummies (cleared by
        // `set_as_long_edge_dummy`) is treated the same as a stored `None`.
        // Otherwise `last_layout_unit` would be reset by every long-edge
        // dummy and confuse subsequent stack drains.
        if let Some(unit) = graph.node(node_id).properties.get(&IN_LAYER_LAYOUT_UNIT) {
            last_layout_unit = Some(unit);
        }

        match graph.node(node_id).node_type {
            NodeType::Normal => {
                next_position = append_north_south_ports_with_incident_edges(
                    graph,
                    node_id,
                    PortSide::North,
                    ports,
                    port_positions,
                    next_position,
                );

                next_position =
                    empty_stack(graph, stack, STACK_SIDE, ports, port_positions, next_position);

                next_position = append_north_south_ports_with_incident_edges(
                    graph,
                    node_id,
                    PortSide::South,
                    ports,
                    port_positions,
                    next_position,
                );
            }
            NodeType::NorthSouthPort => {
                if let Some(&port_id) = graph
                    .node(node_id)
                    .ports
                    .iter()
                    .find(|&&pid| graph.port(pid).side == INDEXING_SIDE)
                {
                    port_positions.insert(port_id, next_position);
                    ports.push(port_id);
                    next_position += 1;
                }

                for &port_id in &graph.node(node_id).ports {
                    if graph.port(port_id).side == STACK_SIDE {
                        stack.push(node_id);
                    }
                }
            }
            NodeType::LongEdge => {
                for &port_id in &graph.node(node_id).ports {
                    if graph.port(port_id).side == PortSide::West {
                        port_positions.insert(port_id, next_position);
                        ports.push(port_id);
                        next_position += 1;
                    }
                }

                for &port_id in &graph.node(node_id).ports {
                    if graph.port(port_id).side == PortSide::East {
                        let _ = port_id;
                        stack.push(node_id);
                    }
                }
            }
            _ => {}
        }
    }

    let _ = empty_stack(graph, stack, STACK_SIDE, ports, port_positions, next_position);
}

fn append_north_south_ports_with_incident_edges(
    graph: &LGraph,
    node_id: NodeId,
    side: PortSide,
    ports: &mut Vec<PortId>,
    port_positions: &mut PortPositions,
    mut next_position: usize,
) -> usize {
    for &port_id in &graph.node(node_id).ports {
        let port = graph.port(port_id);
        if port.side == side && port.port_dummy.is_some() {
            port_positions.insert(port_id, next_position);
            ports.push(port_id);
            next_position += 1;
        }
    }
    next_position
}

fn empty_stack(
    graph: &LGraph,
    stack: &mut Vec<NodeId>,
    side: PortSide,
    ports: &mut Vec<PortId>,
    port_positions: &mut PortPositions,
    mut next_position: usize,
) -> usize {
    while let Some(node_id) = stack.pop() {
        if let Some(&port_id) =
            graph.node(node_id).ports.iter().find(|&&pid| graph.port(pid).side == side)
        {
            port_positions.insert(port_id, next_position);
            ports.push(port_id);
            next_position += 1;
        }
    }
    next_position
}

fn set_port_positions_for_layer(
    graph: &LGraph,
    layer_nodes: &[NodeId],
    side: PortSide,
    port_positions: &mut PortPositions,
) {
    let mut port_index = 0;
    for &node_id in layer_nodes {
        for port_id in iter_ports_in_nsew(graph, node_id, side) {
            port_positions.insert(port_id, port_index);
            port_index += 1;
        }
    }
}

fn adjacency_positions_for_node(
    graph: &LGraph,
    node_id: NodeId,
    side: PortSide,
    port_positions: &PortPositions,
    out: &mut Vec<usize>,
) {
    out.clear();
    for port_id in iter_ports_in_nsew(graph, node_id, side) {
        let port = graph.port(port_id);
        let edges =
            if side == PortSide::West { &port.incoming_edges } else { &port.outgoing_edges };

        for &edge_id in edges {
            if is_node_self_loop(graph, edge_id) || is_in_layer(graph, edge_id) {
                continue;
            }

            let adjacent_port = if side == PortSide::West {
                graph.edge(edge_id).source
            } else {
                graph.edge(edge_id).target
            };

            if let Some(position) = port_positions.get(&adjacent_port) {
                out.push(position);
            }
        }
    }
}

fn connected_edges(graph: &LGraph, port_id: PortId) -> impl Iterator<Item = EdgeId> + '_ {
    let port = graph.port(port_id);
    port.outgoing_edges.iter().chain(port.incoming_edges.iter()).copied()
}

fn other_end_of(graph: &LGraph, edge_id: EdgeId, from_port: PortId) -> PortId {
    let edge = graph.edge(edge_id);
    if edge.source == from_port { edge.target } else { edge.source }
}

fn port_degree(graph: &LGraph, port_id: PortId) -> usize {
    let port = graph.port(port_id);
    port.incoming_edges.len() + port.outgoing_edges.len()
}

fn is_in_layer(graph: &LGraph, edge_id: EdgeId) -> bool {
    let edge = graph.edge(edge_id);
    let source_layer = graph.node(edge.source_owner).layer;
    let target_layer = graph.node(edge.target_owner).layer;
    source_layer == target_layer
}

fn is_node_self_loop(graph: &LGraph, edge_id: EdgeId) -> bool {
    let edge = graph.edge(edge_id);
    edge.source_owner == edge.target_owner
}

fn is_port_self_loop(graph: &LGraph, edge_id: EdgeId) -> bool {
    let edge = graph.edge(edge_id);
    edge.source == edge.target
}

fn count_crossings_on_selected_ports(
    graph: &LGraph,
    port_positions: &PortPositions,
    ports: &[PortId],
) -> usize {
    let mut bit = BinaryIndexedTree::new(bit_capacity(port_positions));
    let mut deferred_ends = Vec::new();
    let mut crossings = 0;

    for &port_id in ports {
        let Some(position) = port_positions.get(&port_id) else {
            continue;
        };

        bit.remove_all(position);

        for edge_id in connected_edges(graph, port_id) {
            if is_port_self_loop(graph, edge_id) {
                continue;
            }

            let other_end = other_end_of(graph, edge_id, port_id);
            let Some(end_position) = port_positions.get(&other_end) else {
                continue;
            };

            if end_position > position {
                crossings += bit.rank(end_position);
                deferred_ends.push(end_position);
            }
        }

        for end_position in deferred_ends.drain(..) {
            bit.add(end_position);
        }
    }

    crossings
}

fn bit_capacity(port_positions: &PortPositions) -> usize {
    port_positions.max_position().map(|max| max + 1).unwrap_or(0)
}

fn have_different_origins(graph: &LGraph, upper: NodeId, lower: NodeId) -> bool {
    graph.node(upper).properties.get(&ORIGIN_NODE) != graph.node(lower).properties.get(&ORIGIN_NODE)
}

fn origin_port_of(graph: &LGraph, node_id: NodeId) -> Option<PortId> {
    graph
        .node(node_id)
        .ports
        .iter()
        .find_map(|&port_id| graph.port(port_id).properties.get(&ORIGIN_PORT))
}

fn is_north_of_normal_node(graph: &LGraph, node_id: NodeId) -> bool {
    origin_port_of(graph, node_id)
        .map(|origin_port| graph.port(origin_port).side == PortSide::North)
        .unwrap_or(false)
}

fn origin_port_position_of(graph: &LGraph, node_id: NodeId) -> usize {
    let Some(origin_port) = origin_port_of(graph, node_id) else {
        return 0;
    };

    let origin_side = graph.port(origin_port).side;
    let origin_node = graph.port(origin_port).owner;
    iter_ports_in_nsew(graph, origin_node, origin_side)
        .position(|port_id| port_id == origin_port)
        .unwrap_or(0)
}

fn first_port_degree_on_side(graph: &LGraph, node_id: NodeId, side: PortSide) -> usize {
    iter_ports_in_nsew(graph, node_id, side)
        .next()
        .map(|port_id| port_degree(graph, port_id))
        .unwrap_or(0)
}

fn number_of_north_south_edges(graph: &LGraph, node_id: NodeId, side: PortSide) -> usize {
    graph
        .node(node_id)
        .ports
        .iter()
        .copied()
        .filter(|&port_id| graph.port(port_id).side == side)
        .filter(|&port_id| graph.port(port_id).port_dummy.is_some())
        .count()
}

fn is_layout_unit_changed(
    graph: &LGraph,
    last_layout_unit: Option<NodeId>,
    node_id: NodeId,
) -> bool {
    // `LongEdge` dummies whose layout unit was cleared end up with a stored
    // `None` rather than a missing key, so treat an empty `Option<NodeId>`
    // as absent and skip the change check.
    let current_layout_unit = graph.node(node_id).properties.get(&IN_LAYER_LAYOUT_UNIT);
    let Some(unit) = current_layout_unit else {
        return false;
    };

    if last_layout_unit.is_none() || last_layout_unit == Some(node_id) {
        return false;
    }

    Some(unit) != last_layout_unit
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<HyperedgeBounds>();
    }

    #[test]
    fn count_i32_inversions_matches_quadratic_path() {
        let mut scratch = CountingScratch::new();
        let values = [
            4, 1, 4, -2, 10, 3, 3, 7, 6, 5, 2, 0, 9, 8, 11, -1, 12, 13, 14, 15, 16, 17, 18, 19, 20,
            21, 22, 23, 24, 25, 26, 27, 0,
        ];

        assert_eq!(scratch.count_i32_inversions(values), count_i32_inversions_quadratic(&values));
    }
}
