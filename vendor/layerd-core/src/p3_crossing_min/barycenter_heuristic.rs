//! Barycenter heuristic for crossing minimization.

use std::cmp::Ordering;

use crate::{
    graph::{LGraph, index::NodeId},
    p3_crossing_min::{
        barycenter_state::{BarycenterState, BarycenterStateMap, compare_barycenter_values},
        forster_constraint_resolver::apply_constraint_resolution,
        layer_sweep::PortRanks,
        model_order_barycenter_heuristic::ModelOrderSorter,
    },
    properties::internal::BARYCENTER_ASSOCIATES,
    rng::Rng,
};

const RANDOM_AMOUNT: f64 = 0.07;

pub(crate) fn order_free_layer_by_heuristic_with_state_scratch(
    graph: &mut LGraph,
    free_layer_idx: usize,
    forward: bool,
    pre_ordered: bool,
    rng: &mut impl Rng,
    port_ranks: &PortRanks,
    use_median: bool,
    states: &mut BarycenterStateMap,
    ordered: &mut Vec<NodeId>,
) -> bool {
    ordered.clear();
    ordered.extend_from_slice(&graph.layers[free_layer_idx].nodes);
    if ordered.is_empty() {
        return false;
    }

    states.reset_with_nodes(ordered);

    for &node_id in ordered.iter() {
        calculate_barycenter(graph, node_id, forward, rng, port_ranks, states);
    }

    fill_in_unknown_barycenters(ordered, states, pre_ordered, rng);

    // When `crossing_minimization_force_node_model_order` is set, sort with
    // the model-order aware insertion sort and skip constraint resolution,
    // since the insertion sort already honours the transitive ordering
    // induced by model order. Otherwise follow the plain barycenter + Forster
    // path.
    if graph.options.crossing_minimization_force_node_model_order {
        let mut sorter = ModelOrderSorter::new();
        sorter.insertion_sort(ordered, graph, states);
    } else {
        states.sort_nodes_by_barycenter(ordered);
        apply_constraint_resolution(graph, free_layer_idx, ordered, states);
    }

    if use_median {
        ordered.sort_by(|a, b| {
            compare_median_tiebreak(graph, *a, *b, states)
                .then(compare_barycenters(states.get(*a), states.get(*b)))
        });
    }

    if graph.layers[free_layer_idx].nodes.as_slice() != ordered.as_slice() {
        graph.layers[free_layer_idx].nodes.clear();
        graph.layers[free_layer_idx].nodes.extend_from_slice(ordered);
        graph.bump_layer_order_version(free_layer_idx);
    }
    false
}

fn compare_median_tiebreak(
    graph: &LGraph,
    a: NodeId,
    b: NodeId,
    states: &BarycenterStateMap,
) -> Ordering {
    let a_value = states.get(a).and_then(|state| state.barycenter).unwrap_or(f64::INFINITY);
    let b_value = states.get(b).and_then(|state| state.barycenter).unwrap_or(f64::INFINITY);
    a_value
        .partial_cmp(&b_value)
        .unwrap_or(Ordering::Equal)
        .then_with(|| graph.node(a).id.cmp(&graph.node(b).id))
}

fn calculate_barycenter(
    graph: &LGraph,
    node_id: NodeId,
    forward: bool,
    rng: &mut impl Rng,
    port_ranks: &PortRanks,
    states: &mut BarycenterStateMap,
) {
    let mut stack = vec![(node_id, false)];
    while let Some((node_id, finish)) = stack.pop() {
        let Some(state_pos) = states.position_of(node_id) else {
            continue;
        };
        if finish {
            finish_barycenter(graph, node_id, forward, rng, port_ranks, states, state_pos);
            continue;
        }
        if states.get_at(state_pos).visited {
            continue;
        }

        {
            let state = states.get_at_mut(state_pos);
            state.visited = true;
            state.degree = 0;
            state.summed_weight = 0.0;
            state.barycenter = None;
        }

        stack.push((node_id, true));
        let dependencies = same_layer_barycenter_dependencies(graph, node_id, forward, states);
        for dependency in dependencies.into_iter().rev() {
            if let Some(pos) = states.position_of(dependency)
                && !states.get_at(pos).visited
            {
                stack.push((dependency, false));
            }
        }
    }
}

fn finish_barycenter(
    graph: &LGraph,
    node_id: NodeId,
    forward: bool,
    rng: &mut impl Rng,
    port_ranks: &PortRanks,
    states: &mut BarycenterStateMap,
    state_pos: usize,
) {
    let node_layer = graph.node(node_id).layer;
    for &port_id in &graph.node(node_id).ports {
        let edge_ids = if forward {
            &graph.port(port_id).incoming_edges
        } else {
            &graph.port(port_id).outgoing_edges
        };
        for &edge_id in edge_ids {
            let edge = graph.edge(edge_id);
            let fixed_node = if forward { edge.source_owner } else { edge.target_owner };
            if fixed_node == node_id {
                continue;
            }
            if graph.node(fixed_node).layer == node_layer {
                if fixed_node != node_id
                    && let Some(fixed_pos) = states.position_of(fixed_node)
                {
                    let fixed_state = *states.get_at(fixed_pos);
                    let state = states.get_at_mut(state_pos);
                    state.degree += fixed_state.degree;
                    state.summed_weight += fixed_state.summed_weight;
                }
            } else {
                let fixed_port = if forward { edge.source } else { edge.target };
                let rank = port_ranks.get(fixed_port).unwrap_or(0.0);
                let state = states.get_at_mut(state_pos);
                state.summed_weight += rank;
                state.degree += 1;
            }
        }
    }

    for &associate in graph.node(node_id).properties.get_slice(&BARYCENTER_ASSOCIATES) {
        let Some(associate_node) = graph.try_node(associate) else {
            continue;
        };
        if associate_node.layer == node_layer
            && let Some(associate_pos) = states.position_of(associate)
        {
            let associate_state = *states.get_at(associate_pos);
            let state = states.get_at_mut(state_pos);
            state.degree += associate_state.degree;
            state.summed_weight += associate_state.summed_weight;
        }
    }

    let state = states.get_at_mut(state_pos);
    if state.degree > 0 {
        state.summed_weight += rng.next_f32() as f64 * RANDOM_AMOUNT - RANDOM_AMOUNT / 2.0;
        state.barycenter = Some(state.summed_weight / state.degree as f64);
    }
}

fn same_layer_barycenter_dependencies(
    graph: &LGraph,
    node_id: NodeId,
    forward: bool,
    states: &BarycenterStateMap,
) -> Vec<NodeId> {
    let mut dependencies = Vec::new();
    let node_layer = graph.node(node_id).layer;
    for &port_id in &graph.node(node_id).ports {
        let edge_ids = if forward {
            &graph.port(port_id).incoming_edges
        } else {
            &graph.port(port_id).outgoing_edges
        };
        for &edge_id in edge_ids {
            let edge = graph.edge(edge_id);
            let fixed_node = if forward { edge.source_owner } else { edge.target_owner };
            if fixed_node != node_id
                && graph.node(fixed_node).layer == node_layer
                && states.position_of(fixed_node).is_some()
            {
                dependencies.push(fixed_node);
            }
        }
    }

    for &associate in graph.node(node_id).properties.get_slice(&BARYCENTER_ASSOCIATES) {
        let Some(associate_node) = graph.try_node(associate) else {
            continue;
        };
        if associate_node.layer == node_layer && states.position_of(associate).is_some() {
            dependencies.push(associate);
        }
    }
    dependencies
}

fn fill_in_unknown_barycenters(
    nodes: &mut [NodeId],
    states: &mut BarycenterStateMap,
    pre_ordered: bool,
    rng: &mut impl Rng,
) {
    if pre_ordered {
        let mut last_value = -1.0;
        for idx in 0..nodes.len() {
            let node_id = nodes[idx];
            if states.get(node_id).and_then(|state| state.barycenter).is_none() {
                let mut next_value = last_value + 1.0;
                for &next_node in &nodes[idx + 1..] {
                    if let Some(value) = states.get(next_node).and_then(|state| state.barycenter) {
                        next_value = value;
                        break;
                    }
                }
                let value = (last_value + next_value) / 2.0;
                if let Some(state) = states.get_mut(node_id) {
                    state.barycenter = Some(value);
                    state.summed_weight = value;
                    state.degree = 1;
                }
            }
            if let Some(value) = states.get(node_id).and_then(|state| state.barycenter) {
                last_value = value;
            }
        }
    } else {
        let mut max_bary: f64 = 0.0;
        for node_id in nodes.iter().copied() {
            if let Some(value) = states.get(node_id).and_then(|state| state.barycenter) {
                max_bary = max_bary.max(value);
            }
        }
        max_bary += 2.0;
        for node_id in nodes.iter().copied() {
            if states.get(node_id).and_then(|state| state.barycenter).is_none() {
                let value = rng.next_f32() as f64 * max_bary - 1.0;
                if let Some(state) = states.get_mut(node_id) {
                    state.barycenter = Some(value);
                    state.summed_weight = value;
                    state.degree = 1;
                }
            }
        }
    }
}

pub(crate) fn compare_barycenters(
    a: Option<&BarycenterState>,
    b: Option<&BarycenterState>,
) -> Ordering {
    compare_barycenter_values(
        a.and_then(|state| state.barycenter),
        b.and_then(|state| state.barycenter),
    )
}
