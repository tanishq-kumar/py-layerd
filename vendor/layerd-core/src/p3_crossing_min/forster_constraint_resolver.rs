//! Forster-style in-layer constraint resolver.
//!
//! Detects and resolves violated in-layer successor and layout-unit ordering
//! constraints, based on Michael Forster's two-level crossing-reduction
//! heuristic (Forster 2005).
//!
//! ## Design
//!
//! The resolver keeps `groups` as an append-only arena of `ConstraintGroup`s
//! and `order` as the sorted list of currently-alive group ids. Each merge:
//!
//! 1. Creates a new merged group appended to `groups`
//! 2. Removes the two merged groups from `order`
//! 3. Inserts the new merged group id at its sorted position in `order`
//! 4. Rewrites outgoing references in all other live groups to point to
//!    the merged group instead of `g1` / `g2` (with dedup)
//!
//! Each merge strictly decreases `order.len()` by 1, so the algorithm
//! terminates in at most `n - 1` merges.

use hashbrown::HashMap;
use smallvec::SmallVec;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    p3_crossing_min::barycenter_state::BarycenterStateMap,
    properties::internal::{
        IN_LAYER_LAYOUT_UNIT, IN_LAYER_SUCCESSOR_CONSTRAINTS,
        IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES, P3_INITIAL_LAYER_ORDER,
    },
};

/// Type-safe handle for a constraint group.
///
/// Stored as `u32` so `SmallVec<GroupId, 2>` fits comfortably inline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct GroupId(u32);

impl GroupId {
    #[inline]
    fn idx(self) -> usize {
        self.0 as usize
    }
}

/// A constraint group in the Forster resolver.
///
/// Persistent across merges: a merge appends a new group to
/// [`ConstraintResolver::groups`], removes the old group ids from `order`,
/// and rewrites referring edges in remaining groups. Old merged groups are
/// not deleted from `groups` (append-only arena), they simply stop appearing
/// in `order` and therefore in any subsequent traversal.
#[derive(Clone)]
struct ConstraintGroup {
    /// Nodes contained in this group, in final emit order. `SmallVec<_, 1>`
    /// because most groups are single-node (merges are rare).
    nodes: SmallVec<NodeId, 1>,
    /// Aggregate summed weight across all nodes in this group.
    summed_weight: f64,
    /// Aggregate degree.
    degree: u32,
    /// Effective barycenter: `summed_weight / degree`, or `None` if the
    /// group has no contributing edges.
    barycenter: Option<f64>,
    /// Outgoing constraint targets (groups this one must precede).
    outgoing: SmallVec<GroupId, 2>,
    /// Stable count of predecessors; updated only on merge fix-up.
    incoming_count: u32,
}

/// Forster constraint resolver working state.
///
/// Owns the append-only group arena and reusable scratch buffers.
struct ConstraintResolver {
    /// All groups ever created. Index = `GroupId.0 as usize`.
    groups: Vec<ConstraintGroup>,
    /// Currently-alive groups, in barycenter-sorted order.
    order: Vec<GroupId>,
    /// Scratch: per-group predecessor list, refilled at the start of each
    /// [`ConstraintResolver::find_violated_constraint`] call. Indexed by
    /// `GroupId.0`.
    incoming_seen: Vec<SmallVec<GroupId, 2>>,
    /// Scratch: FIFO queue for the topological walk in
    /// [`ConstraintResolver::find_violated_constraint`].
    active_queue: Vec<GroupId>,
}

impl ConstraintResolver {
    fn new() -> Self {
        Self {
            groups: Vec::new(),
            order: Vec::new(),
            incoming_seen: Vec::new(),
            active_queue: Vec::new(),
        }
    }

    /// Initialise one single-node group per entry of `ordered`, reading
    /// aggregate barycenter state from `states`.
    fn build_initial(&mut self, ordered: &[NodeId], states: &BarycenterStateMap) {
        self.groups.clear();
        self.order.clear();
        self.groups.reserve(ordered.len());
        self.order.reserve(ordered.len());

        for (i, &nid) in ordered.iter().enumerate() {
            let state = states.get(nid).copied().unwrap_or_default();
            let mut nodes: SmallVec<NodeId, 1> = SmallVec::new();
            nodes.push(nid);
            // The per-node aggregate `summed_weight` / `degree` fields
            // default to 0 and are NOT seeded from the node's
            // `BarycenterState`. They only ever accumulate through group-merge
            // constructor calls — and since both inputs start at 0, the merged
            // `degree` stays 0 forever. As a result `ConstraintGroup` merges
            // always fall through to the `else` branch and compute the merged
            // barycenter as the unweighted mean `(b1 + b2) / 2`, not a
            // weighted `sum / degree`.
            self.groups.push(ConstraintGroup {
                nodes,
                summed_weight: 0.0,
                degree: 0,
                barycenter: state.barycenter,
                outgoing: SmallVec::new(),
                incoming_count: 0,
            });
            self.order.push(GroupId(i as u32));
        }

        self.incoming_seen.clear();
        self.incoming_seen.resize(ordered.len(), SmallVec::new());
        self.active_queue.clear();
    }

    /// Build the constraint DAG by reading `IN_LAYER_SUCCESSOR_CONSTRAINTS`
    /// and `IN_LAYER_LAYOUT_UNIT` from each node's properties.
    ///
    /// When `only_between_normal_nodes` is true, run "stage 1": successor
    /// constraints from dummy nodes, successor constraints to dummy nodes,
    /// and layout-unit constraints are skipped entirely. This lets the
    /// resolver fix the relative order of normal nodes before the stage 2
    /// run, which brings the dummies in.
    fn build_constraints_graph(
        &mut self,
        graph: &LGraph,
        ordered: &[NodeId],
        only_between_normal_nodes: bool,
    ) {
        // NodeId → GroupId mapping (only valid for the initial single-node
        // groups; after merges, groups no longer correspond 1:1 to nodes).
        let index_of: HashMap<NodeId, GroupId> =
            ordered.iter().enumerate().map(|(i, &nid)| (nid, GroupId(i as u32))).collect();

        for (i, &nid) in ordered.iter().enumerate() {
            if only_between_normal_nodes && graph.node(nid).node_type != NodeType::Normal {
                continue;
            }
            let succs: SmallVec<NodeId, 4> = graph
                .node(nid)
                .properties
                .get_slice(&IN_LAYER_SUCCESSOR_CONSTRAINTS)
                .iter()
                .copied()
                .collect();
            for succ in succs {
                if only_between_normal_nodes && graph.node(succ).node_type != NodeType::Normal {
                    continue;
                }
                if let Some(&succ_gid) = index_of.get(&succ) {
                    self.add_outgoing_edge(GroupId(i as u32), succ_gid);
                }
            }
        }

        if only_between_normal_nodes {
            return;
        }

        // Build the layout-unit membership map: owner NodeId → initial
        // GroupIds of all nodes whose `IN_LAYER_LAYOUT_UNIT` points to that
        // owner. A normal node typically owns a unit containing itself and
        // any north/south port dummies created for its hierarchical ports.
        let mut layout_units: HashMap<NodeId, SmallVec<GroupId, 2>> = HashMap::new();
        for (i, &nid) in ordered.iter().enumerate() {
            if let Some(owner) = graph.node(nid).properties.get(&IN_LAYER_LAYOUT_UNIT) {
                layout_units.entry(owner).or_default().push(GroupId(i as u32));
            }
        }
        for members in layout_units.values_mut() {
            members.sort_by_key(|gid| {
                let node_id = self.groups[gid.idx()].nodes[0];
                graph.node(node_id).properties.get(&P3_INITIAL_LAYER_ORDER)
            });
        }

        // For each pair of consecutive normal nodes, add cross-product edges
        // between the two layout units' members. This keeps north/south port
        // dummies pinned to their owning normal node: dummies of an earlier
        // normal must all precede dummies of a later normal.
        let mut last_non_dummy: Option<NodeId> = None;
        for &nid in ordered.iter() {
            if graph.node(nid).node_type != NodeType::Normal {
                continue;
            }
            if let Some(prev_nid) = last_non_dummy {
                let prev_members = layout_units.get(&prev_nid).cloned().unwrap_or_default();
                let curr_members = layout_units.get(&nid).cloned().unwrap_or_default();
                for &src in &prev_members {
                    for &dst in &curr_members {
                        self.add_outgoing_edge(src, dst);
                    }
                }
            }
            last_non_dummy = Some(nid);
        }
    }

    /// Add an outgoing edge from `src` to `dst`, deduplicating against
    /// existing edges. Also increments `dst.incoming_count`.
    #[inline]
    fn add_outgoing_edge(&mut self, src: GroupId, dst: GroupId) {
        if !self.groups[src.idx()].outgoing.contains(&dst) {
            self.groups[src.idx()].outgoing.push(dst);
            self.groups[dst.idx()].incoming_count += 1;
        }
    }

    /// Scan the current `order` for a violated constraint.
    ///
    /// Returns `Some((first_order_idx, second_order_idx))` where both are
    /// positions in `self.order`. `first` is a predecessor whose barycenter
    /// is strictly greater than `second`'s, or equal barycenter but a
    /// higher position in the sorted order. Returns `None` if no violation
    /// exists (the order already respects every constraint).
    fn find_violated_constraint(&mut self) -> Option<(usize, usize)> {
        // Grow scratch to match groups.len() in case a prior merge appended.
        if self.incoming_seen.len() < self.groups.len() {
            self.incoming_seen.resize(self.groups.len(), SmallVec::new());
        }
        // Reset per-group incoming lists for live groups.
        for &gid in &self.order {
            self.incoming_seen[gid.idx()].clear();
        }

        // Seed active queue with sources of the DAG — groups that have
        // outgoing edges but zero predecessors.
        self.active_queue.clear();
        for &gid in &self.order {
            let g = &self.groups[gid.idx()];
            if !g.outgoing.is_empty() && g.incoming_count == 0 {
                self.active_queue.push(gid);
            }
        }

        while !self.active_queue.is_empty() {
            // FIFO pop: O(n) `remove(0)`.
            let group_gid = self.active_queue.remove(0);
            let group_bary = self.groups[group_gid.idx()].barycenter.unwrap_or(f64::NEG_INFINITY);

            // Check all predecessors seen so far for a violation. Clone the
            // predecessor list to release the `incoming_seen` borrow before
            // touching `self.groups` or scanning `order`.
            let preds = self.incoming_seen[group_gid.idx()].clone();
            for pred_gid in &preds {
                let pred_bary = self.groups[pred_gid.idx()].barycenter.unwrap_or(f64::NEG_INFINITY);
                // Quantize both sides to f32 then compare exactly.
                let equal = (pred_bary as f32) == (group_bary as f32);
                let violation_possible = equal || pred_bary > group_bary;
                if !violation_possible {
                    continue;
                }
                // Locate both groups in `order` to compare positions.
                let pred_pos = self
                    .order
                    .iter()
                    .position(|&g| g == *pred_gid)
                    .expect("predecessor must be in order");
                let group_pos = self
                    .order
                    .iter()
                    .position(|&g| g == group_gid)
                    .expect("current group must be in order");
                if equal {
                    // Equal barycenter: violation only if predecessor has
                    // a strictly later position.
                    if pred_pos > group_pos {
                        return Some((pred_pos, group_pos));
                    }
                } else {
                    // Strict `pred_bary > group_bary`: always a violation.
                    return Some((pred_pos, group_pos));
                }
            }

            // Propagate to successors. Clone outgoing list to release the
            // `self.groups` borrow before mutating `incoming_seen` and
            // `active_queue`.
            let outgoing = self.groups[group_gid.idx()].outgoing.clone();
            for succ_gid in outgoing {
                // Insert at position 0 (LIFO of seen predecessors).
                self.incoming_seen[succ_gid.idx()].insert(0, group_gid);
                let seen_len = self.incoming_seen[succ_gid.idx()].len() as u32;
                if seen_len == self.groups[succ_gid.idx()].incoming_count {
                    self.active_queue.push(succ_gid);
                }
            }
        }

        None
    }

    /// Handle a violated constraint by merging the two offending groups.
    ///
    /// `first_order_idx` and `second_order_idx` are positions in
    /// `self.order`. After this call, `self.order.len()` has decreased by 1:
    /// the two old groups are removed and a new merged group is inserted
    /// at the correct sorted position.
    fn handle_violated_constraint(&mut self, first_order_idx: usize, second_order_idx: usize) {
        let g1 = self.order[first_order_idx];
        let g2 = self.order[second_order_idx];

        // Combine node list: g1's nodes first, then g2's.
        let mut merged_nodes: SmallVec<NodeId, 1> = SmallVec::new();
        for &nid in &self.groups[g1.idx()].nodes {
            merged_nodes.push(nid);
        }
        for &nid in &self.groups[g2.idx()].nodes {
            merged_nodes.push(nid);
        }

        let degree = self.groups[g1.idx()].degree + self.groups[g2.idx()].degree;
        let summed_weight =
            self.groups[g1.idx()].summed_weight + self.groups[g2.idx()].summed_weight;

        let barycenter = if degree > 0 {
            Some(summed_weight / degree as f64)
        } else {
            match (self.groups[g1.idx()].barycenter, self.groups[g2.idx()].barycenter) {
                (Some(a), Some(b)) => Some((a + b) / 2.0),
                (Some(a), None) => Some(a),
                (None, Some(b)) => Some(b),
                (None, None) => None,
            }
        };

        // Compute merged outgoing list with dedup of shared successors.
        let mut merged_outgoing: SmallVec<GroupId, 2> = SmallVec::new();
        let g1_outgoing = self.groups[g1.idx()].outgoing.clone();
        for out in g1_outgoing {
            if out == g2 || merged_outgoing.contains(&out) {
                continue;
            }
            merged_outgoing.push(out);
        }
        let g2_outgoing = self.groups[g2.idx()].outgoing.clone();
        for out in g2_outgoing {
            if out == g1 {
                continue;
            }
            if merged_outgoing.contains(&out) {
                // Shared successor: `out.incoming_count` was previously
                // incremented once per edge from g1 and once per edge from
                // g2. The merged group replaces both with a single edge,
                // so decrement `out.incoming_count` by 1 to account for
                // the collapsed duplicate.
                self.groups[out.idx()].incoming_count -= 1;
            } else {
                merged_outgoing.push(out);
            }
        }

        // Append merged group to the arena.
        let merged_gid = GroupId(self.groups.len() as u32);
        self.groups.push(ConstraintGroup {
            nodes: merged_nodes,
            summed_weight,
            degree,
            barycenter,
            outgoing: merged_outgoing,
            incoming_count: 0, // accumulated during fix-up pass below
        });
        self.incoming_seen.push(SmallVec::new());

        // Remove g1 and g2 from `order`, then insert merged_gid at the
        // correct sorted position by barycenter.
        self.order.retain(|&gid| gid != g1 && gid != g2);
        let new_bary = barycenter.unwrap_or(f64::INFINITY);
        let insert_at = self
            .order
            .iter()
            .position(|&gid| self.groups[gid.idx()].barycenter.unwrap_or(f64::INFINITY) > new_bary)
            .unwrap_or(self.order.len());
        self.order.insert(insert_at, merged_gid);

        // Fix up all remaining groups' outgoing edges: anything referring
        // to g1 or g2 now refers to merged_gid. Each remaining group that
        // had an edge to g1 and/or g2 contributes exactly +1 to
        // `merged.incoming_count`, regardless of whether it had one or two
        // original edges.
        for i in 0..self.order.len() {
            let gid = self.order[i];
            if gid == merged_gid {
                continue;
            }

            let had_any = {
                let outgoing = &mut self.groups[gid.idx()].outgoing;
                let before_len = outgoing.len();
                outgoing.retain(|&out| out != g1 && out != g2);
                before_len != outgoing.len()
            };

            if had_any {
                let already = self.groups[gid.idx()].outgoing.contains(&merged_gid);
                if !already {
                    self.groups[gid.idx()].outgoing.push(merged_gid);
                    self.groups[merged_gid.idx()].incoming_count += 1;
                }
            }
        }

        // g1 and g2 are no longer in `order`, so no subsequent traversal
        // will touch them. Their entries in `self.groups` remain as
        // tombstones (simpler than tracking alive flags or rebuilding the
        // arena). Scratch buffers for them in `incoming_seen` are reset at
        // the start of each `find_violated_constraint` call.
    }
}

/// Resolve constraint violations in `ordered` using Forster's two-level
/// crossing reduction resolver.
///
/// After this function returns:
/// * `ordered` is the sorted final node order, with merged groups emitted
///   in contiguous blocks.
/// * For every node in a merged group, `states[node].barycenter` is updated
///   to the group's merged barycenter.
///
/// When the graph property `IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES`
/// is set, a two-stage pipeline runs: stage 1 resolves constraints between
/// normal nodes only, then stage 2 resolves all constraints including dummy
/// successor constraints and layout-unit constraints.
pub fn apply_constraint_resolution(
    graph: &LGraph,
    _layer_idx: usize,
    ordered: &mut Vec<NodeId>,
    states: &mut BarycenterStateMap,
) {
    if ordered.len() < 2 || !has_forster_constraints(graph, ordered) {
        return;
    }

    let two_stage = graph.properties.get(&IN_LAYER_SUCCESSOR_CONSTRAINTS_BETWEEN_NON_DUMMIES);

    if two_stage {
        run_stage(graph, ordered, states, true);
    }
    run_stage(graph, ordered, states, false);
}

fn has_forster_constraints(graph: &LGraph, ordered: &[NodeId]) -> bool {
    ordered.iter().copied().any(|node_id| {
        !graph
            .node(node_id)
            .properties
            .get_slice(&IN_LAYER_SUCCESSOR_CONSTRAINTS)
            .is_empty()
            || graph.node(node_id).properties.get(&IN_LAYER_LAYOUT_UNIT).is_some()
    })
}

/// Single-stage resolver run. Writes merged order + barycenters back to
/// `ordered` and `states`. Caller invokes twice when staging is enabled.
///
/// The theoretical iteration bound is `n - 1` merges. A defensive cap of
/// `2n` (min 16) guards against any accidental non-termination.
fn run_stage(
    graph: &LGraph,
    ordered: &mut Vec<NodeId>,
    states: &mut BarycenterStateMap,
    only_between_normal_nodes: bool,
) {
    if ordered.len() < 2 {
        return;
    }

    let mut resolver = ConstraintResolver::new();
    resolver.build_initial(ordered, states);
    resolver.build_constraints_graph(graph, ordered, only_between_normal_nodes);

    // Each merge removes one entry from `order`, so the loop terminates in
    // at most `n - 1` iterations. No explicit bound needed.
    while let Some((first, second)) = resolver.find_violated_constraint() {
        resolver.handle_violated_constraint(first, second);
    }

    ordered.clear();
    for &gid in &resolver.order {
        let g = &resolver.groups[gid.idx()];
        for &nid in &g.nodes {
            ordered.push(nid);
            if let Some(state) = states.get_mut(nid) {
                state.barycenter = g.barycenter;
            }
        }
    }
}
