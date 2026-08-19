use hashbrown::HashSet;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    p1_cycle_breaking::{
        model_order::GroupModelOrderCalculator,
        scc_model_order::{self, SccContext},
    },
    properties::internal::{CB_CYCLE_BREAKING_ID, CB_PREFERRED_SOURCE_ID, CB_PREFERRED_TARGET_ID},
};

/// Break cycles using the SCC-based strategy with preferred source / target
/// group ids.
///
/// Per SCC:
///   1. If the min-model-order node's group id equals
///      `CB_PREFERRED_SOURCE_ID`, reverse all its incoming SCC-internal edges.
///   2. Else if the max-model-order node's group id equals
///      `CB_PREFERRED_TARGET_ID`, reverse all its outgoing edges.
///   3. Else fall back to the connectivity rule (compare in-degree of min to
///      out-degree of max).
pub fn break_cycles(graph: &mut LGraph) {
    scc_model_order::run(graph, find_nodes);
}

fn find_nodes(ctx: &mut SccContext<'_>) {
    let preferred_source = ctx.graph.properties.get(&CB_PREFERRED_SOURCE_ID);
    let preferred_target = ctx.graph.properties.get(&CB_PREFERRED_TARGET_ID);

    for scc in ctx.sccs {
        if scc.len() <= 1 {
            continue;
        }
        let scc_set: HashSet<NodeId> = scc.iter().copied().collect();

        let mut calc = GroupModelOrderCalculator::new();
        let mut min: Option<NodeId> = None;
        let mut max: Option<NodeId> = None;
        let mut model_order_min = i32::MAX;
        let mut model_order_max = i32::MIN;
        for &n in scc {
            let current = if ctx.enforce_group_model_order {
                calc.compute_constraint_group_model_order(ctx.graph, n, ctx.big_offset, ctx.offset)
            } else {
                calc.compute_constraint_model_order(ctx.graph, n, ctx.offset)
            };
            if min.is_none() {
                min = Some(n);
                max = Some(n);
                model_order_min = current;
                model_order_max = current;
            } else {
                if current < model_order_min {
                    min = Some(n);
                    model_order_min = current;
                }
                if current > model_order_max {
                    max = Some(n);
                    model_order_max = current;
                }
            }
        }
        let (Some(min_n), Some(max_n)) = (min, max) else {
            continue;
        };

        let min_group = ctx.graph.node(min_n).properties.get(&CB_CYCLE_BREAKING_ID);
        let max_group = ctx.graph.node(max_n).properties.get(&CB_CYCLE_BREAKING_ID);

        if min_group == preferred_source {
            for eid in ctx.graph.incoming_edges(min_n) {
                let source_port = ctx.graph.edge(eid).source;
                let source = ctx.graph.port(source_port).owner;
                if scc_set.contains(&source) {
                    ctx.rev_edges.push(eid);
                }
            }
        } else if max_group == preferred_target {
            // The original spec filters outgoing edges by
            // `edge.source.node`. For any outgoing edge of `max`, the source
            // is `max` itself, so this effectively reverses *every* outgoing
            // edge of `max`.
            for eid in ctx.graph.outgoing_edges(max_n) {
                let source_port = ctx.graph.edge(eid).source;
                let source = ctx.graph.port(source_port).owner;
                if scc_set.contains(&source) {
                    ctx.rev_edges.push(eid);
                }
            }
        } else {
            // Connectivity fallback (same as SCConnectivity).
            let min_incoming: Vec<EdgeId> = ctx.graph.incoming_edges(min_n).collect();
            let max_outgoing: Vec<EdgeId> = ctx.graph.outgoing_edges(max_n).collect();

            if min_incoming.len() > max_outgoing.len() {
                for eid in min_incoming {
                    let source_port = ctx.graph.edge(eid).source;
                    let source = ctx.graph.port(source_port).owner;
                    if scc_set.contains(&source) {
                        ctx.rev_edges.push(eid);
                    }
                }
            } else {
                for eid in max_outgoing {
                    let target_port = ctx.graph.edge(eid).target;
                    let target = ctx.graph.port(target_port).owner;
                    if scc_set.contains(&target) {
                        ctx.rev_edges.push(eid);
                    }
                }
            }
        }
    }
}
