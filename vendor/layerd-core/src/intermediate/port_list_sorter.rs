use std::{cmp::Ordering, collections::HashMap};

use crate::{
    graph::{
        LGraph,
        edge::EdgeFlags,
        index::{NodeId, PortId},
        port::PortSide,
    },
    options::enums::{PortConstraints, PortSortingStrategy},
};

/// Sort node port lists into clockwise order.
///
/// Nodes whose port side is not fixed are left untouched. For side-fixed ports we
/// normalize the list to north, east, south, west and reverse the south / west
/// segments so the stored order follows the canonical clockwise convention.
/// For `FIXED_POS`, ports are additionally sorted geometrically within each side
/// (N/E ascending, S/W descending), matching the port-index/geometry semantics.
pub fn sort(graph: &mut LGraph) {
    sort_graph_ports(graph);
}

fn sort_graph_ports(graph: &mut LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers are unique nested graph boxes and are only
        // borrowed one at a time.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let node_ids: Vec<NodeId> = graph
            .layers
            .iter()
            .flat_map(|layer| layer.nodes.iter().copied())
            .chain(graph.layerless_nodes.iter().copied())
            .collect();

        for node_id in &node_ids {
            let constraints = node_port_constraints(graph, *node_id);
            if constraints.is_side_fixed() {
                let mut ports = graph.node(*node_id).ports.clone();
                if constraints.is_order_fixed() || constraints.is_pos_fixed() {
                    // The FIXED_ORDER + FIXED_POS branches share one comparator
                    // — side first, then PORT_INDEX (FIXED_ORDER only), then
                    // position.
                    let original_index = build_original_index(&ports);
                    ports.sort_by(|a, b| {
                        compare_for_fixed_constraints(graph, *a, *b, constraints, &original_index)
                    });
                } else {
                    // FIXED_SIDE without fixed order / position — keep side
                    // grouping and reverse the S/W segments so the resulting
                    // iteration is N → E → S ↓ → W ↑.
                    ports.sort_by_key(|&port_id| side_order(graph.port(port_id).side));
                    reverse_side_segment(graph, &mut ports, PortSide::South);
                    reverse_side_segment(graph, &mut ports, PortSide::West);

                    // If PortSortingStrategy == PORT_DEGREE, do a secondary
                    // stable sort within the East and West segments — East
                    // descending by real (non-reversed) out-degree, West
                    // ascending by real in-degree. The side sort + west/south
                    // reverse above already produced the required pre-sort.
                    if graph.options.port_sorting_strategy == PortSortingStrategy::PortDegree {
                        sort_east_west_by_real_degree(graph, &mut ports);
                    }
                }
                graph.node_mut(*node_id).ports = ports;
            }
            // Always populate the per-side range cache. For side-fixed nodes the
            // stable side regrouping inside `cache_port_sides` is a no-op; for
            // non-side-fixed nodes it brings ports into N->E->S->W groups, the
            // contract that downstream port-side cache reads rely on.
            graph.cache_port_sides(*node_id);
        }

        for node_id in node_ids.into_iter().rev() {
            if let Some(child) = graph.nested_mut(node_id) {
                stack.push(std::ptr::NonNull::from(child));
            }
        }
    }
}

fn build_original_index(
    ports: &[crate::graph::index::PortId],
) -> HashMap<crate::graph::index::PortId, usize> {
    ports.iter().enumerate().map(|(idx, &port_id)| (port_id, idx)).collect()
}

/// Port comparator for FIXED_ORDER + FIXED_POS constraints. Sorts by side
/// first, then `PORT_INDEX` (FIXED_ORDER only), then side-dependent position,
/// then the original list index as a stable tie-breaker.
fn compare_for_fixed_constraints(
    graph: &LGraph,
    left: crate::graph::index::PortId,
    right: crate::graph::index::PortId,
    constraints: PortConstraints,
    original_index: &HashMap<crate::graph::index::PortId, usize>,
) -> Ordering {
    let left_port = graph.port(left);
    let right_port = graph.port(right);

    let side_cmp = side_order(left_port.side).cmp(&side_order(right_port.side));
    if side_cmp != Ordering::Equal {
        return side_cmp;
    }

    // PORT_INDEX only applies to FIXED_ORDER (not FIXED_POS). If either
    // port is missing the explicit property, fall through to the position
    // comparison.
    if constraints == PortConstraints::FixedOrder {
        let has_l = left_port.properties.has(&crate::properties::internal::PORT_INDEX);
        let has_r = right_port.properties.has(&crate::properties::internal::PORT_INDEX);
        if has_l && has_r {
            let idx_l = left_port.properties.get(&crate::properties::internal::PORT_INDEX);
            let idx_r = right_port.properties.get(&crate::properties::internal::PORT_INDEX);
            let idx_cmp = idx_l.cmp(&idx_r);
            if idx_cmp != Ordering::Equal {
                return idx_cmp;
            }
        }
    }

    let fixed_pos_cmp =
        compare_same_side_fixed_position(left_port.side, left_port.position, right_port.position);
    if fixed_pos_cmp != Ordering::Equal {
        return fixed_pos_cmp;
    }

    let left_idx = original_index.get(&left).copied().unwrap_or(usize::MAX);
    let right_idx = original_index.get(&right).copied().unwrap_or(usize::MAX);
    left_idx.cmp(&right_idx)
}

fn compare_same_side_fixed_position(
    side: PortSide,
    left: crate::math::Vec2,
    right: crate::math::Vec2,
) -> Ordering {
    let cmp_f64 = |a: f64, b: f64| a.partial_cmp(&b).unwrap_or(Ordering::Equal);

    match side {
        PortSide::North => cmp_f64(left.x, right.x),
        PortSide::East => cmp_f64(left.y, right.y),
        PortSide::South => cmp_f64(right.x, left.x),
        PortSide::West => cmp_f64(right.y, left.y),
        PortSide::Undefined => Ordering::Equal,
    }
}

fn node_port_constraints(graph: &LGraph, node_id: NodeId) -> PortConstraints {
    let node_constraints = graph.node(node_id).port_constraints();
    if node_constraints == PortConstraints::Undefined {
        graph.options.port_constraints
    } else {
        node_constraints
    }
}

fn side_order(side: PortSide) -> u8 {
    match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => 4,
    }
}

fn reverse_side_segment(graph: &LGraph, ports: &mut [crate::graph::index::PortId], side: PortSide) {
    let (start, end) = find_java_reverse_range(graph, ports, side);

    // The early-return guard `if (highIdx <= lowIdx + 2) return;` skips
    // reversal whenever the side has 1 or 2 ports. Layouts that depend on
    // this expect the unsorted-side input order to flow into the input-
    // model sorter unchanged for 2-port sides.
    if end > start + 2 {
        ports[start..end].reverse();
    }
}

fn find_java_reverse_range(
    graph: &LGraph,
    ports: &[crate::graph::index::PortId],
    side: PortSide,
) -> (usize, usize) {
    if ports.is_empty() {
        return (0, 0);
    }

    // In the second loop, refresh `currentSide` from `lowIdx`, not
    // `highIdx`, so the returned exclusive upper bound is the final index
    // before the side group ends when the group reaches the list tail.
    let mut current_side = graph.port(ports[0]).side;
    let mut low_idx = 0usize;
    let lower_bound = side_order(side);
    let upper_bound = lower_bound + 1;

    while low_idx < ports.len() - 1 && side_order(current_side) < lower_bound {
        low_idx += 1;
        current_side = graph.port(ports[low_idx]).side;
    }

    let mut high_idx = low_idx;
    while high_idx < ports.len() - 1 && side_order(current_side) < upper_bound {
        high_idx += 1;
        current_side = graph.port(ports[low_idx]).side;
    }

    (low_idx, high_idx)
}

/// Stable secondary sort inside the east / west segments by real port
/// degree. East sorts by out-degree **descending**, West by in-degree
/// **ascending**. "Real" degree counts only non-REVERSED edges.
fn sort_east_west_by_real_degree(graph: &LGraph, ports: &mut [PortId]) {
    stable_sort_segment(graph, ports, PortSide::East, |g, pid| -real_out_degree(g, pid));
    stable_sort_segment(graph, ports, PortSide::West, real_in_degree);
}

fn stable_sort_segment<F: Fn(&LGraph, PortId) -> i32>(
    graph: &LGraph,
    ports: &mut [PortId],
    side: PortSide,
    key: F,
) {
    let (Some(start), Some(end)) = find_side_range(graph, ports, side) else {
        return;
    };
    if end <= start + 1 {
        return;
    }
    ports[start..end].sort_by_key(|&pid| key(graph, pid));
}

fn find_side_range(
    graph: &LGraph,
    ports: &[PortId],
    side: PortSide,
) -> (Option<usize>, Option<usize>) {
    let mut start = None;
    let mut end = None;
    for (idx, &pid) in ports.iter().enumerate() {
        if graph.port(pid).side == side {
            start.get_or_insert(idx);
            end = Some(idx + 1);
        } else if start.is_some() {
            break;
        }
    }
    (start, end)
}

fn real_out_degree(graph: &LGraph, port: PortId) -> i32 {
    graph
        .port(port)
        .outgoing_edges
        .iter()
        .filter(|&&eid| !graph.edge(eid).flags.contains(EdgeFlags::REVERSED))
        .count() as i32
}

fn real_in_degree(graph: &LGraph, port: PortId) -> i32 {
    graph
        .port(port)
        .incoming_edges
        .iter()
        .filter(|&&eid| !graph.edge(eid).flags.contains(EdgeFlags::REVERSED))
        .count() as i32
}
