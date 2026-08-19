//! Greedy switch decision logic.
//!
//! The pair-wise crossings and constraint tables are populated lazily: greedy
//! sweep touches roughly `O(n)` adjacent pairs (per inner pass) over a few
//! passes, so precomputing the full `n × n` matrix wastes ~6-10× of the work
//! on a large layer. We allocate both caches once as flat `Vec<_>`s of size
//! `n²` and fill entries on demand.

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    p3_crossing_min::{
        counting::{self, CountingScratch, LayerSidePositions, NeighborPortTable},
        crossing_count_side::CrossingCountSide,
    },
    properties::internal::{IN_LAYER_LAYOUT_UNIT, IN_LAYER_SUCCESSOR_CONSTRAINTS, ORIGIN_PORT},
};

/// Parent-graph access window used by the two-sided greedy switch to count crossings
/// caused by swapping external port dummies in the free layer.
pub struct ParentContext<'a> {
    /// Mutable borrow of the parent graph so port order can be swapped during
    /// `notify_of_switch` and re-read during `does_switch_reduce_crossings`.
    pub graph: &'a mut LGraph,
    /// Parent node that hosts the nested graph whose free layer we are switching.
    pub parent_node_id: NodeId,
    /// Layer index of `parent_node_id` in its own graph.
    pub parent_layer_idx: usize,
}

impl<'a> ParentContext<'a> {
    /// Create a shorter-lived mutable reborrow of this context so it can be passed
    /// down through multiple call sites without moving the outer binding.
    pub fn reborrow(&mut self) -> ParentContext<'_> {
        ParentContext {
            graph: &mut *self.graph,
            parent_node_id: self.parent_node_id,
            parent_layer_idx: self.parent_layer_idx,
        }
    }
}

/// Sentinel stored in the crossings cache for "not yet computed". Using
/// `i32::MIN` avoids an `Option<i32>` at the cost of rejecting a single
/// otherwise-legal cached value (crossings counts are always non-negative).
const UNCOMPUTED: i32 = i32::MIN;
const CS_UNKNOWN: u8 = 0;
const CS_FALSE: u8 = 1;
const CS_TRUE: u8 = 2;

/// Decide whether swapping two adjacent nodes in a layer reduces crossings.
///
/// Pair-wise crossing counts and constraint checks are computed lazily on first
/// query and cached for the remaining lifetime of this decider. See module docs.
///
/// When a `ParentContext` is supplied and the two-sided external-port-layer gate
/// passes, the decider also queries the parent graph on every call so it can account
/// for crossings caused by swapping external port dummies.
pub struct SwitchDecider<'a> {
    /// Current node ordering in the free layer (swapped by `notify_of_switch`).
    free_layer: Vec<NodeId>,
    /// Node identity → stable index, indexed by `NodeId.0.index() as usize`.
    /// Sentinel `u32::MAX` marks slots for nodes outside this layer (never read
    /// in practice; queries always use a node taken from `free_layer`). Replaces
    /// a `HashMap<NodeId, usize>` to dodge SipHash on the inner hot path.
    node_to_idx: Vec<u32>,
    /// Stable-index → NodeId snapshot captured at construction. Used by lazy
    /// compute paths to map a cache slot back to the original layer nodes.
    stable_nodes: Vec<NodeId>,
    /// Lazy flat `n*n` crossings cache. `crossings[si * n + sj]` holds the
    /// total child-layer crossings when stable-`si` is above stable-`sj`, or
    /// `UNCOMPUTED` if not yet filled.
    crossings: Vec<i32>,
    /// Lazy flat `n*n` constraint cache.
    /// `0 = unknown`, `1 = constraints prevent switch`, `2 = switch allowed`.
    can_switch: Vec<u8>,
    /// Layer width. Cached so cache indexing never reads Vec::len.
    n: usize,
    /// Raw pointer to the child graph held for the lifetime of this decider.
    /// The borrow checker does not track this pointer, so callers must keep
    /// the `&LGraph` passed to `new` alive (and not mutate/invalidate it)
    /// until the decider is dropped. `apply_to_graph` is the only mutation
    /// point; it takes a fresh `&mut LGraph` from the caller.
    graph: *const LGraph,
    free_layer_index: usize,
    side: CrossingCountSide,
    one_sided: bool,
    /// Parent port aligned with each free-layer stable index. `None` for nodes whose
    /// ORIGIN_PORT property is unset (non-external-port dummies) or when the parent
    /// counter gate is disabled.
    origin_ports: Vec<Option<PortId>>,
    /// Parent-graph handle; `Some` only when the two-sided external-port gate passes.
    parent_ctx: Option<ParentContext<'a>>,
    /// Side of the parent graph adjacent to `parent_node_id` that supplies the
    /// between-layer edges whose crossings are affected by swapping external ports.
    parent_side: PortSide,
    /// Reusable counting buffers amortised across the sweep's pair queries.
    scratch: CountingScratch,
    /// Cached port→position map for the west neighbour (layer - 1, East
    /// ports). `None` when the free layer is the left-most. Built once in
    /// `new()` and reused across every pair query; the neighbour's own
    /// ordering does not change during this sweep.
    neighbor_west: Option<NeighborPortTable>,
    /// Cached port→position map for the east neighbour (layer + 1, West
    /// ports). `None` when the free layer is the right-most.
    neighbor_east: Option<NeighborPortTable>,
    /// Pre-built per-side port positions and node cardinalities for the
    /// free layer itself. Built once in `new()` so the in-layer pair counter
    /// no longer reruns `init_positions` over the whole free layer on every
    /// `crossings_pair` cache miss. The decider mutates only its own
    /// `free_layer` copy; `graph.layers[free_layer_index].nodes` and the
    /// underlying ports stay constant for the decider's lifetime, so these
    /// snapshots remain valid until `apply_to_graph`.
    free_layer_west: LayerSidePositions,
    free_layer_east: LayerSidePositions,
}

impl<'a> SwitchDecider<'a> {
    /// Create a SwitchDecider for the given free layer and side.
    pub fn new(
        graph: &LGraph,
        free_layer_index: usize,
        side: CrossingCountSide,
        one_sided: bool,
        parent_context: Option<ParentContext<'a>>,
    ) -> Self {
        let layer_nodes = &graph.layers[free_layer_index].nodes;
        let n = layer_nodes.len();
        let free_layer: Vec<NodeId> = layer_nodes.to_vec();
        let stable_nodes = free_layer.clone();

        let max_arena_index =
            stable_nodes.iter().map(|nid| nid.0.index() as usize).max().unwrap_or(0);
        let mut node_to_idx = vec![u32::MAX; max_arena_index + 1];
        for (i, &node_id) in stable_nodes.iter().enumerate() {
            node_to_idx[node_id.0.index() as usize] = i as u32;
        }

        // The parent-cross-counter gate is `!one_sided && hasParent() &&
        // !dont_sweep_into() && freeLayer[0].type == EXTERNAL_PORT`. We arrive
        // here only via the P3 hierarchical sweep frame, which already verified
        // `child_info.dont_sweep_into()`, so when the caller hands us a
        // `parent_context` we are already in the "child being swept
        // hierarchically" branch. Checking `dont_sweep_into()` again here
        // would re-derive the decision from the child's own state and miss
        // any caller-driven hierarchical entry. The surrounding gate
        // (`parent_context.is_some()` + the external-port first-layer check)
        // is sufficient.
        let is_external_port_layer = stable_nodes
            .first()
            .is_some_and(|&first| graph.node(first).node_type == NodeType::ExternalPort);
        let parent_enabled = !one_sided && parent_context.is_some() && is_external_port_layer;
        let parent_ctx = if parent_enabled { parent_context } else { None };

        let parent_side = if free_layer_index + 1 == graph.layers.len() {
            PortSide::East
        } else {
            PortSide::West
        };

        // The origin parent port is read via `ORIGIN_PORT` on the dummy
        // NODE — production sets it through `create_external_port_dummy_in_nested`
        // and the testing harness equivalent; only NS dummies set it on a port.
        let origin_ports: Vec<Option<PortId>> = if parent_ctx.is_some() {
            stable_nodes
                .iter()
                .map(|&node_id| graph.node(node_id).properties.get(&ORIGIN_PORT))
                .collect()
        } else {
            Vec::new()
        };

        let neighbor_west = if free_layer_index > 0 {
            Some(NeighborPortTable::build(graph, free_layer_index - 1, PortSide::East))
        } else {
            None
        };
        let neighbor_east = if free_layer_index + 1 < graph.layers.len() {
            Some(NeighborPortTable::build(graph, free_layer_index + 1, PortSide::West))
        } else {
            None
        };

        let free_layer_west = LayerSidePositions::build(graph, free_layer_index, PortSide::West);
        let free_layer_east = LayerSidePositions::build(graph, free_layer_index, PortSide::East);

        Self {
            free_layer,
            node_to_idx,
            stable_nodes,
            crossings: vec![UNCOMPUTED; n * n],
            can_switch: vec![CS_UNKNOWN; n * n],
            n,
            graph: graph as *const LGraph,
            free_layer_index,
            side,
            one_sided,
            origin_ports,
            parent_ctx,
            parent_side,
            scratch: CountingScratch::new(),
            neighbor_west,
            neighbor_east,
            free_layer_west,
            free_layer_east,
        }
    }

    /// Return whether switching the two nodes reduces crossings.
    pub fn does_switch_reduce_crossings(&mut self, upper_idx: usize, lower_idx: usize) -> bool {
        let upper_node = self.free_layer[upper_idx];
        let lower_node = self.free_layer[lower_idx];

        let si = self.node_to_idx[upper_node.0.index() as usize] as usize;
        let sj = self.node_to_idx[lower_node.0.index() as usize] as usize;

        if !self.can_switch_pair(si, sj) {
            return false;
        }

        // Compute `(upper_above_lower, lower_above_upper)` totals via four
        // sources whose pair-counters return both orders in a single call.
        // Calling the in-layer pair counter twice with arguments flipped is
        // unsafe: it assumes the caller's `upper` is the cache-actual upper,
        // so the flipped call would not correspond to "lower above upper" —
        // making `notify_of_switch`'s permanent layer-order update bite the
        // next query and letting the inner `greedy_switch_layer` loop
        // oscillate forever.
        // SAFETY: see `crossings_pair` for the graph-pointer contract.
        let graph = unsafe { &*self.graph };
        let upper = self.stable_nodes[si];
        let lower = self.stable_nodes[sj];

        let (static_uv, static_vu) = self.static_pair(si, sj);
        let (in_w_uv, in_w_vu) =
            counting::count_in_layer_crossings_between_pair_on_side_with_cached_layer(
                graph,
                self.free_layer_index,
                upper_idx,
                lower_idx,
                PortSide::West,
                &mut self.free_layer_west,
                &mut self.scratch,
            );
        let (in_e_uv, in_e_vu) =
            counting::count_in_layer_crossings_between_pair_on_side_with_cached_layer(
                graph,
                self.free_layer_index,
                upper_idx,
                lower_idx,
                PortSide::East,
                &mut self.free_layer_east,
                &mut self.scratch,
            );

        let mut upper_lower = static_uv + in_w_uv as i32 + in_e_uv as i32;
        let mut lower_upper = static_vu + in_w_vu as i32 + in_e_vu as i32;

        if let Some(ctx) = self.parent_ctx.as_ref() {
            let upper_port = self.origin_ports.get(si).copied().flatten();
            let lower_port = self.origin_ports.get(sj).copied().flatten();
            if let (Some(upper_port), Some(lower_port)) = (upper_port, lower_port) {
                let (pul, plu) = counting::count_crossings_between_ports_in_both_orders(
                    ctx.graph,
                    ctx.parent_layer_idx,
                    upper_port,
                    lower_port,
                    self.parent_side,
                );
                upper_lower += pul as i32;
                lower_upper += plu as i32;
            }
        }

        let _ = (upper, lower);
        upper_lower > lower_upper
    }

    /// Returns the `(static_si_above_sj, static_sj_above_si)` portions
    /// (between-layer + NS) which are invariant under `notify_of_switch`.
    /// Both directions are cached the first time either is requested.
    fn static_pair(&mut self, si: usize, sj: usize) -> (i32, i32) {
        let idx_uv = si * self.n + sj;
        let idx_vu = sj * self.n + si;
        // SAFETY: indices come from `node_to_idx`, all in `0..n`.
        let cached_uv = unsafe { *self.crossings.get_unchecked(idx_uv) };
        let cached_vu = unsafe { *self.crossings.get_unchecked(idx_vu) };
        if cached_uv != UNCOMPUTED && cached_vu != UNCOMPUTED {
            return (cached_uv, cached_vu);
        }

        // SAFETY: `self.graph` valid for the decider's lifetime.
        let graph = unsafe { &*self.graph };
        let upper = self.stable_nodes[si];
        let lower = self.stable_nodes[sj];

        let mut uv: i32 = 0;
        let mut vu: i32 = 0;
        // Between-layer counts both directions: with two-sided we need both
        // (West + East), with one-sided only one. Use the existing helper
        // for upper-above-lower then flip the upper/lower call to get
        // lower-above-upper. The neighbor tables are read-only so flipping
        // is safe and side-effect free.
        uv += between_layer_crossings(
            graph,
            upper,
            lower,
            self.side,
            self.one_sided,
            self.neighbor_west.as_ref(),
            self.neighbor_east.as_ref(),
            &mut self.scratch,
        );
        vu += between_layer_crossings(
            graph,
            lower,
            upper,
            self.side,
            self.one_sided,
            self.neighbor_west.as_ref(),
            self.neighbor_east.as_ref(),
            &mut self.scratch,
        );
        let (ns_uv, ns_vu) = counting::count_ns_crossings_between_nodes(graph, upper, lower);
        uv += ns_uv as i32;
        vu += ns_vu as i32;

        unsafe {
            *self.crossings.get_unchecked_mut(idx_uv) = uv;
            *self.crossings.get_unchecked_mut(idx_vu) = vu;
        }
        (uv, vu)
    }

    /// Return whether constraints allow swapping the two stable indices,
    /// filling the cache on first access. `si == sj` is always rejected.
    fn can_switch_pair(&mut self, si: usize, sj: usize) -> bool {
        if si == sj {
            return false;
        }
        let idx = si * self.n + sj;
        // SAFETY: `si`, `sj` in `0..n`; `idx` in `0..n*n` == `can_switch.len()`.
        let state = unsafe { *self.can_switch.get_unchecked(idx) };
        if state != CS_UNKNOWN {
            return state == CS_TRUE;
        }
        // SAFETY: as in `crossings_pair`.
        let graph = unsafe { &*self.graph };
        let upper = self.stable_nodes[si];
        let lower = self.stable_nodes[sj];

        let prevented = constraints_prevent_switch(graph, upper, lower);
        let resolved = if prevented { CS_FALSE } else { CS_TRUE };
        unsafe {
            *self.can_switch.get_unchecked_mut(idx) = resolved;
        }
        resolved == CS_TRUE
    }

    /// Notify that nodes at the given indices were switched.
    pub fn notify_of_switch(&mut self, upper_idx: usize, lower_idx: usize) {
        let upper_node = self.free_layer[upper_idx];
        let lower_node = self.free_layer[lower_idx];
        let si = self.node_to_idx[upper_node.0.index() as usize] as usize;
        let sj = self.node_to_idx[lower_node.0.index() as usize] as usize;

        self.free_layer.swap(upper_idx, lower_idx);

        // Permanently shift the cached port positions on both in-layer
        // counters to reflect the swap. Without this, subsequent
        // `does_switch_reduce_crossings` queries read stale in-layer crossing
        // counts and `minimize_no_counter` oscillates between forward and
        // backward sweeps forever (each direction sees the swap as
        // crossing-reducing).
        // SAFETY: `self.graph` was cast from a `&LGraph` at construction and
        // the caller contract keeps that graph alive for `self`'s lifetime.
        // We additionally need a `&mut LGraph` to swap the layer's node
        // order so the in-layer counters (which look up
        // `graph.layers[layer_idx].nodes[idx]`) see the same ordering as
        // `free_layer`. Casting away const here is safe because the only
        // other writer is `apply_to_graph`, called after the inner loop
        // exits.
        let graph_const = unsafe { &*self.graph };
        self.free_layer_west
            .switch_pair(graph_const, upper_node, lower_node, PortSide::West);
        self.free_layer_east
            .switch_pair(graph_const, upper_node, lower_node, PortSide::East);
        // SAFETY: same exclusivity contract as the const cast above. The
        // caller contract for SwitchDecider says "do not mutate this graph
        // through other paths" while the decider lives; that is preserved
        // because the layer-sweep driver feeds the same `&mut LGraph` only
        // to `decider.apply_to_graph`, and we are the only writer here.
        let graph_mut = unsafe { &mut *(self.graph as *mut LGraph) };
        graph_mut.layers[self.free_layer_index].nodes.swap(upper_idx, lower_idx);
        graph_mut.bump_layer_order_version(self.free_layer_index);

        if let Some(ctx) = self.parent_ctx.as_mut() {
            let upper_port = self.origin_ports.get(si).copied().flatten();
            let lower_port = self.origin_ports.get(sj).copied().flatten();
            if let (Some(upper_port), Some(lower_port)) = (upper_port, lower_port) {
                let parent_ports = &mut ctx.graph.node_mut(ctx.parent_node_id).ports;
                let upper_pos = parent_ports.iter().position(|&p| p == upper_port);
                let lower_pos = parent_ports.iter().position(|&p| p == lower_port);
                if let (Some(up), Some(lp)) = (upper_pos, lower_pos)
                    && up != lp
                {
                    parent_ports.swap(up, lp);
                    ctx.graph.bump_layer_order_version(ctx.parent_layer_idx);
                }
            }
        }
    }

    /// Return the number of nodes in the free layer.
    pub fn free_layer_len(&self) -> usize {
        self.free_layer.len()
    }

    /// Write the current node order back to the graph layer.
    pub fn apply_to_graph(&self, graph: &mut LGraph, layer_index: usize) {
        if graph.layers[layer_index].nodes.as_slice() != self.free_layer.as_slice() {
            graph.layers[layer_index].nodes.clone_from_slice(&self.free_layer);
            graph.bump_layer_order_version(layer_index);
        }
    }
}

// Per-pair compute helpers (shared by the lazy fillers)

fn between_layer_crossings(
    graph: &LGraph,
    upper_node: NodeId,
    lower_node: NodeId,
    side: CrossingCountSide,
    one_sided: bool,
    neighbor_west: Option<&NeighborPortTable>,
    neighbor_east: Option<&NeighborPortTable>,
    scratch: &mut CountingScratch,
) -> i32 {
    let mut total = 0i32;

    if one_sided {
        match side {
            CrossingCountSide::West =>
                if let Some(table) = neighbor_west {
                    let (ul, _) = counting::count_crossings_between_pair_nodes_with_table(
                        graph,
                        upper_node,
                        lower_node,
                        PortSide::West,
                        table,
                        scratch,
                    );
                    total += ul as i32;
                },
            CrossingCountSide::East =>
                if let Some(table) = neighbor_east {
                    let (ul, _) = counting::count_crossings_between_pair_nodes_with_table(
                        graph,
                        upper_node,
                        lower_node,
                        PortSide::East,
                        table,
                        scratch,
                    );
                    total += ul as i32;
                },
        }
    } else {
        if let Some(table) = neighbor_west {
            let (ul, _) = counting::count_crossings_between_pair_nodes_with_table(
                graph,
                upper_node,
                lower_node,
                PortSide::West,
                table,
                scratch,
            );
            total += ul as i32;
        }
        if let Some(table) = neighbor_east {
            let (ul, _) = counting::count_crossings_between_pair_nodes_with_table(
                graph,
                upper_node,
                lower_node,
                PortSide::East,
                table,
                scratch,
            );
            total += ul as i32;
        }
    }

    total
}

fn constraints_prevent_switch(graph: &LGraph, upper_node: NodeId, lower_node: NodeId) -> bool {
    have_successor_constraints(graph, upper_node, lower_node)
        || have_layout_unit_constraints(graph, upper_node, lower_node)
        || are_normal_and_north_south_port_dummy(graph, upper_node, lower_node)
}

fn have_successor_constraints(graph: &LGraph, upper_node: NodeId, lower_node: NodeId) -> bool {
    let successors = graph.node(upper_node).properties.get_slice(&IN_LAYER_SUCCESSOR_CONSTRAINTS);
    !successors.is_empty() && successors.contains(&lower_node)
}

fn have_layout_unit_constraints(graph: &LGraph, upper_node: NodeId, lower_node: NodeId) -> bool {
    let neither_long_edge = graph.node(upper_node).node_type != NodeType::LongEdge
        && graph.node(lower_node).node_type != NodeType::LongEdge;

    if !neither_long_edge {
        return false;
    }

    let upper_layout = graph.node(upper_node).properties.get(&IN_LAYER_LAYOUT_UNIT);
    let lower_layout = graph.node(lower_node).properties.get(&IN_LAYER_LAYOUT_UNIT);
    let upper_multi = upper_layout.is_some() && upper_layout != Some(upper_node);
    let lower_multi = lower_layout.is_some() && lower_layout != Some(lower_node);
    let different_layout_units = upper_layout != lower_layout;

    let upper_has_north = has_edges_or_port_dummy_on_side(graph, upper_node, PortSide::North);
    let lower_has_south = has_edges_or_port_dummy_on_side(graph, lower_node, PortSide::South);

    // hotfix for #162
    let has_layout_units = upper_multi
        || lower_multi
        || has_edges_or_port_dummy_on_side(graph, upper_node, PortSide::South)
        || has_edges_or_port_dummy_on_side(graph, lower_node, PortSide::North);

    (has_layout_units && different_layout_units) || upper_has_north || lower_has_south
}

fn has_edges_or_port_dummy_on_side(graph: &LGraph, node_id: NodeId, side: PortSide) -> bool {
    graph.node(node_id).ports.iter().copied().any(|port_id| {
        let port = graph.port(port_id);
        port.side == side
            && (port.port_dummy.is_some()
                || !port.incoming_edges.is_empty()
                || !port.outgoing_edges.is_empty())
    })
}

fn are_normal_and_north_south_port_dummy(
    graph: &LGraph,
    upper_node: NodeId,
    lower_node: NodeId,
) -> bool {
    let upper_type = graph.node(upper_node).node_type;
    let lower_type = graph.node(lower_node).node_type;
    (upper_type == NodeType::NorthSouthPort && lower_type == NodeType::Normal)
        || (lower_type == NodeType::NorthSouthPort && upper_type == NodeType::Normal)
}
