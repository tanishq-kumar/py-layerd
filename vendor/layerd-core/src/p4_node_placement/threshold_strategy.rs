//! Threshold computation for Brandes-Köpf compaction.
//!
//! Provides the null and simple threshold variants used by the compaction
//! phase:
//!
//! - `ThresholdStrategy::Null` — classic bk compaction. `calculate_threshold`
//!   returns a sentinel that disables any threshold-driven shifting.
//! - `ThresholdStrategy::Simple(_)` — favours straight edges at block
//!   boundaries. For each block's first and last node, it picks one candidate
//!   edge and returns a threshold that would land the block so the edge is
//!   drawn straight. Blocks with unplaced neighbors are deferred onto a queue
//!   and re-tried in `post_process`.

use std::collections::VecDeque;

use hashbrown::HashSet;

use super::bk::{
    AlignedLayout, HDir, Neighborhood, VDir, calculate_delta, check_space_above, check_space_below,
    shift_block,
};
use crate::graph::{LGraph, index::EdgeId};

/// Bundle of per-run mutable state used by `SimpleThresholdStrategy`.
pub(super) struct SimpleState {
    /// Block roots (by node index) whose compaction has finished.
    block_finished: HashSet<usize>,
    /// Postponed blocks whose candidate edges pointed at unplaced neighbors.
    postponed_queue: VecDeque<Postprocessable>,
    /// Scratch stack used for a second, reversed post-processing pass.
    postponed_stack: Vec<Postprocessable>,
}

impl SimpleState {
    fn new() -> Self {
        SimpleState {
            block_finished: HashSet::new(),
            postponed_queue: VecDeque::new(),
            postponed_stack: Vec::new(),
        }
    }

    fn clear(&mut self) {
        self.block_finished.clear();
        self.postponed_queue.clear();
        self.postponed_stack.clear();
    }
}

/// A deferred candidate edge that might be straightened after the main
/// compaction pass finishes.
#[derive(Clone, Copy)]
struct Postprocessable {
    /// Index of the block node (root or last) whose edge may be straightened.
    free: usize,
    /// True if `free` is the block's root node; false if it is the last.
    is_root: bool,
    /// Latched by `pick_edge` to remember whether candidate edges exist.
    has_edges: bool,
    /// Edge chosen by `pick_edge`, if any.
    edge: Option<EdgeId>,
}

pub(super) enum ThresholdStrategy {
    Null,
    Simple(SimpleState),
}

impl ThresholdStrategy {
    pub(super) fn null() -> Self {
        ThresholdStrategy::Null
    }

    pub(super) fn simple() -> Self {
        ThresholdStrategy::Simple(SimpleState::new())
    }

    /// Reset internal state between the four layout passes.
    pub(super) fn init(&mut self) {
        if let ThresholdStrategy::Simple(state) = self {
            state.clear();
        }
    }

    /// Called by `place_block` once a block's main compaction finishes.
    pub(super) fn finish_block(&mut self, root_idx: usize) {
        if let ThresholdStrategy::Simple(state) = self {
            state.block_finished.insert(root_idx);
        }
    }

    /// Compute the threshold bound for the block currently being placed.
    pub(super) fn calculate_threshold(
        &mut self,
        old_thresh: f64,
        block_root: usize,
        current: usize,
        bal: &mut AlignedLayout,
        graph: &LGraph,
        ni: &Neighborhood,
    ) -> f64 {
        match self {
            ThresholdStrategy::Null =>
                if bal.vdir == VDir::Up {
                    f64::INFINITY
                } else {
                    f64::NEG_INFINITY
                },
            ThresholdStrategy::Simple(_) =>
                simple_calculate_threshold(self, old_thresh, block_root, current, bal, graph, ni),
        }
    }

    /// Flush blocks whose straightening was deferred during main compaction.
    pub(super) fn post_process(
        &mut self,
        bal: &mut AlignedLayout,
        graph: &LGraph,
        ni: &Neighborhood,
    ) {
        if matches!(self, ThresholdStrategy::Null) {
            return;
        }
        simple_post_process(self, bal, graph, ni);
    }
}

/// Entry point used by `ThresholdStrategy::calculate_threshold` for the
/// `Simple` variant.
fn simple_calculate_threshold(
    strat: &mut ThresholdStrategy,
    old_thresh: f64,
    block_root: usize,
    current: usize,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
) -> f64 {
    // Only the first and last node of a block participate. For single-node
    // blocks both flags end up true.
    let is_root = block_root == current;
    let is_last = bal.align[current] == block_root;
    if !(is_root || is_last) {
        return old_thresh;
    }

    let mut t = old_thresh;
    if is_root {
        t = simple_get_bound(strat, block_root, true, bal, graph, ni);
    }
    if !t.is_finite() && is_last {
        t = simple_get_bound(strat, current, false, bal, graph, ni);
    }
    t
}

/// Chooses one candidate edge and returns the threshold y-coordinate that
/// would land the block so the edge is straight. Side-effect: marks the two
/// incident blocks with `su` so that future calls cannot pick the same edge.
fn simple_get_bound(
    strat: &mut ThresholdStrategy,
    block_node: usize,
    is_root: bool,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
) -> f64 {
    let invalid = if bal.vdir == VDir::Up { f64::INFINITY } else { f64::NEG_INFINITY };

    let pp = Postprocessable { free: block_node, is_root, has_edges: false, edge: None };
    let picked = pick_edge(strat, pp, bal, graph, ni);

    if picked.edge.is_none() {
        if picked.has_edges
            && let ThresholdStrategy::Simple(state) = strat
        {
            state.postponed_queue.push_back(picked);
        }
        return invalid;
    }

    let eid = picked.edge.expect("pick_edge returned has_edges but no edge");
    let edge = graph.edge(eid);
    let src_pid = edge.source;
    let tgt_pid = edge.target;
    let src_port = graph.port(src_pid);
    let tgt_port = graph.port(tgt_pid);
    let src_idx = ni.idx(src_port.owner);
    let tgt_idx = ni.idx(tgt_port.owner);

    // For a block's root node, the block extends "forward" along the horizontal
    // traversal direction; for a block's last node, the block extends
    // "backwards". Combine the two into: root->forward + last->backward, so
    // both reuse the same arithmetic but with flipped roles for `rootPort` and
    // `otherPort`.
    let (
        root_port_idx,
        root_port_pos_y,
        root_port_anchor_y,
        other_idx,
        other_pos_y,
        other_anchor_y,
    ) = if (is_root && bal.hdir == HDir::Right) || (!is_root && bal.hdir == HDir::Left) {
        (
            tgt_idx,
            tgt_port.position.y,
            tgt_port.anchor.y,
            src_idx,
            src_port.position.y,
            src_port.anchor.y,
        )
    } else {
        (
            src_idx,
            src_port.position.y,
            src_port.anchor.y,
            tgt_idx,
            tgt_port.position.y,
            tgt_port.anchor.y,
        )
    };

    let other_root = bal.root[other_idx];
    let threshold = bal.y[other_root] + bal.inner_shift[other_idx] + other_pos_y + other_anchor_y
        - bal.inner_shift[root_port_idx]
        - root_port_pos_y
        - root_port_anchor_y;

    // Mark both incident blocks as already straightened.
    let src_block_root = bal.root[src_idx];
    let tgt_block_root = bal.root[tgt_idx];
    bal.su[src_block_root] = true;
    bal.su[tgt_block_root] = true;

    threshold
}

/// Pick the first candidate edge that could straighten the block. Returns the
/// input `pp` decorated with `edge` and `has_edges`.
fn pick_edge(
    strat: &ThresholdStrategy,
    mut pp: Postprocessable,
    bal: &AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
) -> Postprocessable {
    let free_nid = ni.idx_to_node[pp.free];
    let use_incoming = if pp.is_root { bal.hdir == HDir::Right } else { bal.hdir == HDir::Left };
    let edges: Vec<EdgeId> = if use_incoming {
        graph.incoming_edges(free_nid).collect()
    } else {
        graph.outgoing_edges(free_nid).collect()
    };

    let block_finished = match strat {
        ThresholdStrategy::Simple(state) => &state.block_finished,
        ThresholdStrategy::Null => {
            pp.has_edges = false;
            pp.edge = None;
            return pp;
        }
    };

    let free_block_root = bal.root[pp.free];
    let only_dummies = bal.od[free_block_root];
    let node_layer = graph.node(free_nid).layer;

    let mut has_edges = false;
    for eid in edges {
        let edge = graph.edge(eid);
        let src_nid = graph.port(edge.source).owner;
        let tgt_nid = graph.port(edge.target).owner;

        // Skip self-loops.
        if src_nid == tgt_nid {
            continue;
        }
        // Skip in-layer edges unless the block is made of dummies only.
        let is_in_layer = graph.node(src_nid).layer == graph.node(tgt_nid).layer
            && graph.node(src_nid).layer == node_layer;
        if !only_dummies && is_in_layer {
            continue;
        }
        // The block was already straightened by a previous bound; do not shift
        // it again.
        if bal.su[free_block_root] {
            continue;
        }

        has_edges = true;

        let other_nid = if src_nid == free_nid { tgt_nid } else { src_nid };
        let Some(other_idx) = ni.lookup_idx(other_nid) else {
            continue;
        };
        if block_finished.contains(&bal.root[other_idx]) {
            pp.has_edges = true;
            pp.edge = Some(eid);
            return pp;
        }
    }

    pp.has_edges = has_edges;
    pp.edge = None;
    pp
}

fn simple_post_process(
    strat: &mut ThresholdStrategy,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
) {
    // Drain the deferred queue; re-pick an edge now that more blocks have been
    // placed, and shift the block toward the straightening y if room permits.
    loop {
        let pp = match strat {
            ThresholdStrategy::Simple(state) => state.postponed_queue.pop_front(),
            _ => return,
        };
        let Some(pp) = pp else {
            break;
        };
        let picked = pick_edge(strat, pp, bal, graph, ni);
        if picked.edge.is_none() {
            continue;
        }
        let eid = picked.edge.expect("pick_edge returned edge but was none");
        let edge = graph.edge(eid);
        let src_nid = graph.port(edge.source).owner;
        let tgt_nid = graph.port(edge.target).owner;
        let only_dummies = bal.od[bal.root[picked.free]];
        let src_layer = graph.node(src_nid).layer;
        let tgt_layer = graph.node(tgt_nid).layer;
        if !only_dummies && src_layer == tgt_layer {
            continue;
        }
        let moved = process_postprocessable(picked, bal, graph, ni);
        if !moved && let ThresholdStrategy::Simple(state) = strat {
            state.postponed_stack.push(picked);
        }
    }

    // Second pass: try the remaining items in reverse order.
    loop {
        let pp = match strat {
            ThresholdStrategy::Simple(state) => state.postponed_stack.pop(),
            _ => return,
        };
        let Some(pp) = pp else {
            break;
        };
        let _ = process_postprocessable(pp, bal, graph, ni);
    }
}

/// Attempt to straighten the edge stashed in `pp` by shifting its block. Returns
/// true if any shift happened.
fn process_postprocessable(
    pp: Postprocessable,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
) -> bool {
    let Some(eid) = pp.edge else {
        return false;
    };
    let edge = graph.edge(eid);
    let src_pid = edge.source;
    let tgt_pid = edge.target;
    let free_nid = ni.idx_to_node[pp.free];
    let src_is_free = graph.port(src_pid).owner == free_nid;

    // `block` is the port attached to the block we may shift; `fix` is on the
    // other block we are trying to align with.
    let (fix_pid, block_pid) = if src_is_free { (tgt_pid, src_pid) } else { (src_pid, tgt_pid) };

    let delta = calculate_delta(bal, graph, ni, fix_pid, block_pid);

    let block_node_idx = ni.idx(graph.port(block_pid).owner);
    let block_root = bal.root[block_node_idx];

    if delta > 0.0 && delta.is_finite() {
        let available = check_space_above(bal, graph, ni, block_root, delta);
        if available > 0.0 {
            shift_block(bal, block_root, -available);
            return true;
        }
        false
    } else if delta < 0.0 && delta.is_finite() {
        let available = check_space_below(bal, graph, ni, block_root, -delta);
        if available > 0.0 {
            shift_block(bal, block_root, available);
            return true;
        }
        false
    } else {
        false
    }
}
