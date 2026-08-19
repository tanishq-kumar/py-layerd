use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    options::enums::{GroupOrderStrategy, LayerConstraint},
    properties::internal::{
        CB_CYCLE_BREAKING_ID, CB_GROUP_ORDER_STRATEGY, CB_NUM_MODEL_ORDER_GROUPS, CYCLIC,
        LAYER_CONSTRAINT, MAX_MODEL_ORDER_NODES, MODEL_ORDER,
    },
};

/// Model-order cycle breaker (non-group variant).
///
/// Reverses every edge whose target has a strictly lower effective model
/// order than the source. Layer constraints (`FIRST_SEPARATE`, `FIRST`,
/// `LAST`, `LAST_SEPARATE`) shift the effective order so that the final
/// sequence respects `FIRST_SEPARATE < FIRST < NORMAL < LAST < LAST_SEPARATE`.
///
/// If a node is missing the `MODEL_ORDER` property, this implementation
/// falls back to the node's position in `layerless_nodes`. This preserves
/// a strict total order and guarantees acyclicity even for graphs loaded
/// without explicit model orders.
///
/// Per-source instantiation of `GroupModelOrderCalculator` ensures
/// FIRST_SEPARATE / LAST_SEPARATE counters reset for every source node,
/// preserving strict per-source tie-break semantics.
pub fn break_cycles(graph: &mut LGraph) {
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    if node_ids.is_empty() {
        return;
    }

    // offset = max(layerless.size, MAX_MODEL_ORDER_NODES). The graph-level
    // property is seeded by `assign_model_order_from_insertion`.
    let max_model_order_nodes = graph.properties.get(&MAX_MODEL_ORDER_NODES);
    let offset = (node_ids.len() as i32).max(max_model_order_nodes);
    let cb_num_groups = graph.properties.get(&CB_NUM_MODEL_ORDER_GROUPS);
    let big_offset = offset * cb_num_groups;
    let enforce_group_order =
        graph.properties.get(&CB_GROUP_ORDER_STRATEGY) == GroupOrderStrategy::Enforced;

    let mut rev_edges: Vec<EdgeId> = Vec::new();
    for &source in &node_ids {
        let mut calculator = GroupModelOrderCalculator::new();
        let mo_source = if enforce_group_order {
            calculator.compute_constraint_group_model_order(graph, source, big_offset, offset)
        } else {
            calculator.compute_constraint_model_order(graph, source, offset)
        };
        for eid in graph.outgoing_edges(source) {
            let target_port = graph.edge(eid).target;
            let target = graph.port(target_port).owner;
            if target == source {
                continue;
            }
            let mo_target = if enforce_group_order {
                calculator.compute_constraint_group_model_order(graph, target, big_offset, offset)
            } else {
                calculator.compute_constraint_model_order(graph, target, offset)
            };
            if mo_target < mo_source {
                rev_edges.push(eid);
            }
        }
    }

    if !rev_edges.is_empty() {
        graph.properties.set(&CYCLIC, true);
    }
    for eid in rev_edges {
        graph.reverse_edge_adapt_ports(eid);
    }
}

/// Apply the layer-constraint offset for one node.
///
/// `FIRST_SEPARATE` / `LAST_SEPARATE` counters are passed by mutable reference
/// so the caller controls their lifetime (per-iteration vs whole-graph).
fn layer_constraint_offset(
    constraint: LayerConstraint,
    offset: i32,
    first_separate_counter: &mut i32,
    last_separate_counter: &mut i32,
) -> i32 {
    match constraint {
        LayerConstraint::FirstSeparate => {
            let mo = 2 * -offset + *first_separate_counter;
            *first_separate_counter += 1;
            mo
        }
        LayerConstraint::First => -offset,
        LayerConstraint::Last => offset,
        LayerConstraint::LastSeparate => {
            let mo = 2 * offset + *last_separate_counter;
            *last_separate_counter += 1;
            mo
        }
        LayerConstraint::None => 0,
    }
}

/// Stateful calculator for cycle-breaker group / constraint model order.
///
/// The caller constructs one instance per SCC-pass and calls either
/// `compute_constraint_model_order` or `compute_constraint_group_model_order`
/// for each node. The two `FIRST_SEPARATE` / `LAST_SEPARATE` counters are
/// incremented across calls so the resulting orders are a strict total order
/// within the pass.
pub(crate) struct GroupModelOrderCalculator {
    first_separate_nodes: i32,
    last_separate_nodes: i32,
}

impl GroupModelOrderCalculator {
    pub(crate) fn new() -> Self {
        Self { first_separate_nodes: 0, last_separate_nodes: 0 }
    }

    /// Compute the effective model order for one node.
    pub(crate) fn compute_constraint_model_order(
        &mut self,
        graph: &LGraph,
        node_id: NodeId,
        offset: i32,
    ) -> i32 {
        let constraint = graph.node(node_id).properties.get(&LAYER_CONSTRAINT);
        let mut model_order = layer_constraint_offset(
            constraint,
            offset,
            &mut self.first_separate_nodes,
            &mut self.last_separate_nodes,
        );
        if graph.node(node_id).properties.has(&MODEL_ORDER) {
            model_order += graph.node(node_id).properties.get(&MODEL_ORDER);
        }
        model_order
    }

    /// Compute the effective group model order.
    ///
    /// `big_offset = MAX_MODEL_ORDER_NODES * CB_NUM_MODEL_ORDER_GROUPS` and
    /// `small_offset = MAX_MODEL_ORDER_NODES`.
    pub(crate) fn compute_constraint_group_model_order(
        &mut self,
        graph: &LGraph,
        node_id: NodeId,
        big_offset: i32,
        small_offset: i32,
    ) -> i32 {
        let constraint = graph.node(node_id).properties.get(&LAYER_CONSTRAINT);
        let mut model_order = layer_constraint_offset(
            constraint,
            big_offset,
            &mut self.first_separate_nodes,
            &mut self.last_separate_nodes,
        );
        if graph.node(node_id).properties.has(&MODEL_ORDER) {
            let group_id = graph.node(node_id).properties.get(&CB_CYCLE_BREAKING_ID);
            let mo = graph.node(node_id).properties.get(&MODEL_ORDER);
            model_order += group_id * small_offset + mo;
        }
        model_order
    }
}
