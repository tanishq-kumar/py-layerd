use crate::{
    graph::{LGraph, index::NodeId},
    options::enums::{GroupOrderStrategy, LayerConstraint},
    p1_cycle_breaking::greedy::run_greedy_with_tiebreak,
    properties::internal::{
        CB_CYCLE_BREAKING_ID, CB_GROUP_ORDER_STRATEGY, CB_NUM_MODEL_ORDER_GROUPS, LAYER_CONSTRAINT,
        MAX_MODEL_ORDER_NODES, MODEL_ORDER,
    },
};

/// Snapshot of the per-node properties consumed by the model-order tiebreak.
/// Populated once before the cycle-breaking loop because
/// `run_greedy_with_tiebreak` borrows the graph mutably and the tiebreak
/// closure cannot re-read the graph.
struct NodeSnapshot {
    constraint: LayerConstraint,
    model_order: Option<i32>,
    cb_id: i32,
}

/// Tiebreak-local calculator.
///
/// A new instance is constructed for each tiebreak invocation so the
/// FIRST_SEPARATE / LAST_SEPARATE counters reset on every call. This struct
/// is constructed inside the tiebreak closure to keep that scope exact.
struct TiebreakCalculator {
    first_separate_nodes: i32,
    last_separate_nodes: i32,
}

impl TiebreakCalculator {
    fn new() -> Self {
        Self { first_separate_nodes: 0, last_separate_nodes: 0 }
    }

    /// Compute the effective constraint-aware model order for one node.
    fn compute_constraint(&mut self, snap: &NodeSnapshot, offset: i32) -> i32 {
        let mut model_order = self.layer_constraint_offset(snap.constraint, offset);
        if let Some(mo) = snap.model_order {
            model_order += mo;
        }
        model_order
    }

    /// Compute the effective constraint-aware group model order for one node.
    fn compute_constraint_group(
        &mut self,
        snap: &NodeSnapshot,
        big_offset: i32,
        small_offset: i32,
    ) -> i32 {
        let mut model_order = self.layer_constraint_offset(snap.constraint, big_offset);
        if let Some(mo) = snap.model_order {
            model_order += snap.cb_id * small_offset + mo;
        }
        model_order
    }

    fn layer_constraint_offset(&mut self, constraint: LayerConstraint, offset: i32) -> i32 {
        match constraint {
            LayerConstraint::FirstSeparate => {
                let mo = 2 * -offset + self.first_separate_nodes;
                self.first_separate_nodes += 1;
                mo
            }
            LayerConstraint::First => -offset,
            LayerConstraint::Last => offset,
            LayerConstraint::LastSeparate => {
                let mo = 2 * offset + self.last_separate_nodes;
                self.last_separate_nodes += 1;
                mo
            }
            LayerConstraint::None => 0,
        }
    }
}

/// Greedy cycle breaker that breaks ties by minimum model order instead of
/// randomly.
///
/// A fresh calculator is instantiated inside every tiebreak invocation, so
/// the `FIRST_SEPARATE` / `LAST_SEPARATE` counters reset on every call. For
/// graphs without those layer constraints the counters never increment and
/// a single shared calculator would behave identically.
pub fn break_cycles(graph: &mut LGraph) {
    let node_ids: Vec<_> = graph.layerless_nodes.clone();
    if node_ids.is_empty() {
        return;
    }
    // offset = max(layerless.size, MAX_MODEL_ORDER_NODES).
    let max_mo = graph.properties.get(&MAX_MODEL_ORDER_NODES);
    let offset = (node_ids.len() as i32).max(max_mo);
    let cb_num_groups = graph.properties.get(&CB_NUM_MODEL_ORDER_GROUPS);
    let big_offset = offset * cb_num_groups;
    let enforce_group_order =
        graph.properties.get(&CB_GROUP_ORDER_STRATEGY) == GroupOrderStrategy::Enforced;

    // Snapshot the node properties read by `GroupModelOrderCalculator`. The
    // greedy main loop only reverses edges; it never mutates layer-constraint
    // / model-order / cb-id properties, so a one-shot snapshot is correct.
    let snapshots: hashbrown::HashMap<NodeId, NodeSnapshot> = node_ids
        .iter()
        .map(|&nid| {
            let n = graph.node(nid);
            (
                nid,
                NodeSnapshot {
                    constraint: n.properties.get(&LAYER_CONSTRAINT),
                    model_order: n
                        .properties
                        .has(&MODEL_ORDER)
                        .then(|| n.properties.get(&MODEL_ORDER)),
                    cb_id: n.properties.get(&CB_CYCLE_BREAKING_ID),
                },
            )
        })
        .collect();

    let mut rng = graph.take_rng();

    run_greedy_with_tiebreak(graph, |candidates, node_ids_slice| {
        // A fresh calculator per tiebreak resets the FIRST_SEPARATE /
        // LAST_SEPARATE counters so every invocation sees a clean slate.
        let mut calculator = TiebreakCalculator::new();
        let mut best_idx: Option<usize> = None;
        let mut best_mo = i32::MAX;
        for &ci in candidates {
            let nid = node_ids_slice[ci];
            let Some(snap) = snapshots.get(&nid) else { continue };
            // Nodes without an explicit MODEL_ORDER property are skipped
            // here and only used by the random fallback below.
            if snap.model_order.is_none() {
                continue;
            }
            let mo = if enforce_group_order {
                calculator.compute_constraint_group(snap, big_offset, offset)
            } else {
                calculator.compute_constraint(snap, offset)
            };
            if mo < best_mo {
                best_mo = mo;
                best_idx = Some(ci);
            }
        }
        best_idx.unwrap_or_else(|| {
            let idx = rng.next_int(candidates.len() as i32) as usize;
            candidates[idx]
        })
    });
    graph.put_rng(rng);
}
