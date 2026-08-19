use hashbrown::HashSet;

use crate::{
    graph::index::{EdgeId, NodeId},
    p1_cycle_breaking::{
        model_order::GroupModelOrderCalculator,
        scc_model_order::{self, SccContext},
    },
};

/// Break cycles by repeatedly running Tarjan and, per SCC, reversing either
/// all incoming edges of the min-model-order node or all outgoing edges of
/// the max-model-order node — whichever has the larger degree.
pub fn break_cycles(graph: &mut crate::graph::LGraph) {
    scc_model_order::run(graph, find_nodes);
}

fn find_nodes(ctx: &mut SccContext<'_>) {
    for scc in ctx.sccs {
        if scc.len() <= 1 {
            continue;
        }
        let scc_set: HashSet<NodeId> = scc.iter().copied().collect();

        // Find min / max by effective model order.
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
