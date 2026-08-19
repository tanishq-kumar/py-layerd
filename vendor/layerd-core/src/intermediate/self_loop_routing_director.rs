//! Route direction decisions for self-loops.
//!
//! For each hyper-loop, computes the leftmost and rightmost ports along the
//! trunk (starting at the leftmost, running clockwise around the node to the
//! rightmost) and records every port side the trunk passes through.

use hashbrown::{HashMap, HashSet};

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        port::{PortSide, PortSideSet},
    },
    intermediate::self_loop_holder::{
        SelfHyperLoop, SelfLoopHolder, SelfLoopType, portside_right, portside_set_of,
    },
};

/// Penalty incurred by routing past a port that has no external connections.
const UNCONNECTED_PORT_PENALTY: i32 = 1;
/// Penalty incurred by routing past a port that has external connections.
const CONNECTED_PORT_PENALTY: i32 = 3;

/// Populate `leftmost_port`, `rightmost_port`, and `occupied_port_sides` on
/// every hyper-loop inside `holder`.
pub fn determine_loop_routes(graph: &LGraph, holder: &mut SelfLoopHolder, node: NodeId) {
    // Port indices: the position of each port within the owning node's port
    // list, used for leftmost/rightmost comparisons and for the penalty
    // table.
    let mut port_index: HashMap<PortId, u32> = HashMap::new();
    for (i, &pid) in graph.node(node).ports.iter().enumerate() {
        port_index.insert(pid, i as u32);
    }

    let side_order_before_sort: Vec<(PortSide, PortSide)> = holder
        .sl_hyper_loops
        .iter()
        .map(|hyper| two_distinct_sides_in_hyper_port_order(hyper, graph))
        .collect();

    // Ensure each hyper-loop's port list is sorted by port index and the
    // per-side bucket / self-loop-type fields are current, so the
    // determination routines can query them directly.
    for hyper in &mut holder.sl_hyper_loops {
        hyper
            .sl_ports
            .sort_by_key(|pid| port_index.get(pid).copied().unwrap_or(u32::MAX));
        hyper.compute_ports_per_side(graph);
    }

    let mut penalties: Option<Vec<i32>> = None;
    for hyper_idx in 0..holder.sl_hyper_loops.len() {
        match holder.sl_hyper_loops[hyper_idx].self_loop_type {
            Some(SelfLoopType::OneSide) => {
                determine_one_side(graph, &mut holder.sl_hyper_loops[hyper_idx]);
            }
            Some(SelfLoopType::TwoSidesCorner) => {
                determine_two_sides_corner(&mut holder.sl_hyper_loops[hyper_idx], &port_index);
            }
            Some(SelfLoopType::TwoSidesOpposing) => {
                if penalties.is_none() {
                    penalties = Some(compute_port_penalties(graph, node));
                }
                let SelfLoopHolder { sl_hyper_loops, .. } = &mut *holder;
                determine_two_sides_opposing(
                    &mut sl_hyper_loops[hyper_idx],
                    &port_index,
                    penalties.as_deref().unwrap(),
                    side_order_before_sort[hyper_idx],
                );
            }
            Some(SelfLoopType::ThreeSides) => {
                determine_three_sides(&mut holder.sl_hyper_loops[hyper_idx], &port_index);
            }
            Some(SelfLoopType::FourSides) => {
                if penalties.is_none() {
                    penalties = Some(compute_port_penalties(graph, node));
                }
                determine_four_sides(
                    &mut holder.sl_hyper_loops[hyper_idx],
                    &port_index,
                    penalties.as_deref().unwrap(),
                );
            }
            None => {
                // Unclassified loop: leave leftmost/rightmost unset so the
                // router can detect and skip the loop gracefully.
            }
        }

        compute_occupied_port_sides(graph, &mut holder.sl_hyper_loops[hyper_idx]);
    }
}

fn determine_one_side(graph: &LGraph, hyper: &mut SelfHyperLoop) {
    let Some(first) = hyper.sl_ports.first().copied() else {
        return;
    };
    let side = graph.port(first).side;
    if let Some(bucket_first) = hyper.sl_ports_on_side(side).first().copied() {
        hyper.leftmost_port = Some(bucket_first);
    } else {
        hyper.leftmost_port = Some(first);
    }
    hyper.rightmost_port = hyper.sl_ports_on_side(side).last().copied().or(hyper.leftmost_port);
}

fn determine_two_sides_corner(hyper: &mut SelfHyperLoop, port_index: &HashMap<PortId, u32>) {
    let sides = sorted_two_side_corner_sides(hyper);
    assign_leftmost_rightmost_by_side(hyper, sides[0], sides[1], port_index);
}

fn determine_two_sides_opposing(
    hyper: &mut SelfHyperLoop,
    port_index: &HashMap<PortId, u32>,
    penalties: &[i32],
    side_order: (PortSide, PortSide),
) {
    let (side_a, side_b) = side_order;
    if side_a == PortSide::Undefined || side_b == PortSide::Undefined {
        return;
    }

    let opt1_left = lowest_port_on_side(hyper, side_a, port_index);
    let opt1_right = highest_port_on_side(hyper, side_b, port_index);
    let opt1_penalty = compute_edge_penalty(opt1_left, opt1_right, port_index, penalties);

    let opt2_left = lowest_port_on_side(hyper, side_b, port_index);
    let opt2_right = highest_port_on_side(hyper, side_a, port_index);
    let opt2_penalty = compute_edge_penalty(opt2_left, opt2_right, port_index, penalties);

    if opt1_penalty <= opt2_penalty {
        hyper.leftmost_port = opt1_left;
        hyper.rightmost_port = opt1_right;
    } else {
        hyper.leftmost_port = opt2_left;
        hyper.rightmost_port = opt2_right;
    }
}

fn determine_three_sides(hyper: &mut SelfHyperLoop, port_index: &HashMap<PortId, u32>) {
    let missing = compute_missing_port_side(hyper);
    let (left, right) = match missing {
        PortSide::North => (PortSide::East, PortSide::West),
        PortSide::East => (PortSide::South, PortSide::North),
        PortSide::South => (PortSide::West, PortSide::East),
        PortSide::West => (PortSide::North, PortSide::South),
        PortSide::Undefined => return,
    };
    assign_leftmost_rightmost_by_side(hyper, left, right, port_index);
}

fn determine_four_sides(
    hyper: &mut SelfHyperLoop,
    port_index: &HashMap<PortId, u32>,
    penalties: &[i32],
) {
    if hyper.sl_ports.is_empty() {
        return;
    }
    let sorted = hyper.sl_ports.clone();
    let mut worst_left = sorted[sorted.len() - 1];
    let mut worst_right = sorted[0];
    let mut worst_penalty =
        compute_edge_penalty(Some(worst_left), Some(worst_right), port_index, penalties);

    for i in 1..sorted.len() {
        let curr_left = sorted[i - 1];
        let curr_right = sorted[i];
        let curr_penalty =
            compute_edge_penalty(Some(curr_left), Some(curr_right), port_index, penalties);
        if curr_penalty > worst_penalty {
            worst_left = curr_left;
            worst_right = curr_right;
            worst_penalty = curr_penalty;
        }
    }

    // Split at the worst pair: the trunk wraps the other way around the node,
    // so worst_right becomes leftmost and worst_left becomes rightmost.
    hyper.leftmost_port = Some(worst_right);
    hyper.rightmost_port = Some(worst_left);
}

fn compute_occupied_port_sides(graph: &LGraph, hyper: &mut SelfHyperLoop) {
    let Some(left) = hyper.leftmost_port else {
        return;
    };
    let Some(right) = hyper.rightmost_port else {
        return;
    };
    let mut curr = graph.port(left).side;
    let target = graph.port(right).side;

    let mut occupied = PortSideSet::empty();
    for _ in 0..5 {
        occupied |= portside_set_of(curr);
        if curr == target {
            break;
        }
        curr = portside_right(curr);
    }
    hyper.occupied_port_sides = occupied;
}

fn sorted_two_side_corner_sides(hyper: &SelfHyperLoop) -> [PortSide; 2] {
    let (a, b) = two_distinct_sides(hyper);
    // Sort the two sides and flip NORTH/WEST so clockwise wrap-around
    // produces a valid (leftmost, rightmost) pair.
    let sides = [a, b];
    let mut sorted = sides;
    sorted.sort_by_key(|s| portside_ordinal(*s));
    if sorted[0] == PortSide::North && sorted[1] == PortSide::West {
        [PortSide::West, PortSide::North]
    } else {
        sorted
    }
}

fn compute_missing_port_side(hyper: &SelfHyperLoop) -> PortSide {
    for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
        if hyper.sl_ports_on_side(side).is_empty() {
            return side;
        }
    }
    PortSide::Undefined
}

fn two_distinct_sides(hyper: &SelfHyperLoop) -> (PortSide, PortSide) {
    let mut collected: [PortSide; 2] = [PortSide::Undefined; 2];
    let mut idx = 0;
    for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
        if !hyper.sl_ports_on_side(side).is_empty() && idx < 2 {
            collected[idx] = side;
            idx += 1;
        }
    }
    (collected[0], collected[1])
}

fn two_distinct_sides_in_hyper_port_order(
    hyper: &SelfHyperLoop,
    graph: &LGraph,
) -> (PortSide, PortSide) {
    let mut collected: [PortSide; 2] = [PortSide::Undefined; 2];
    let mut idx = 0;
    let mut seen = PortSideSet::empty();
    for &pid in &hyper.sl_ports {
        let side = graph.port(pid).side;
        let bit = portside_set_of(side);
        if !seen.contains(bit) {
            seen |= bit;
            if idx < 2 {
                collected[idx] = side;
                idx += 1;
                if idx >= 2 {
                    break;
                }
            }
        }
    }
    (collected[0], collected[1])
}

fn assign_leftmost_rightmost_by_side(
    hyper: &mut SelfHyperLoop,
    left_side: PortSide,
    right_side: PortSide,
    port_index: &HashMap<PortId, u32>,
) {
    hyper.leftmost_port = lowest_port_on_side(hyper, left_side, port_index);
    hyper.rightmost_port = highest_port_on_side(hyper, right_side, port_index);
}

fn lowest_port_on_side(
    hyper: &SelfHyperLoop,
    side: PortSide,
    port_index: &HashMap<PortId, u32>,
) -> Option<PortId> {
    hyper
        .sl_ports_on_side(side)
        .iter()
        .copied()
        .min_by_key(|pid| port_index.get(pid).copied().unwrap_or(u32::MAX))
}

fn highest_port_on_side(
    hyper: &SelfHyperLoop,
    side: PortSide,
    port_index: &HashMap<PortId, u32>,
) -> Option<PortId> {
    hyper
        .sl_ports_on_side(side)
        .iter()
        .copied()
        .max_by_key(|pid| port_index.get(pid).copied().unwrap_or(0))
}

fn compute_edge_penalty(
    left: Option<PortId>,
    right: Option<PortId>,
    port_index: &HashMap<PortId, u32>,
    penalties: &[i32],
) -> i32 {
    let (Some(l), Some(r)) = (left, right) else {
        return 0;
    };
    let port_count = penalties.len();
    if port_count == 0 {
        return 0;
    }

    let left_id = port_index.get(&l).copied().unwrap_or(0) as usize;
    let right_id = port_index.get(&r).copied().unwrap_or(0) as usize;
    let left_of_right_id = if right_id == 0 { port_count - 1 } else { right_id - 1 };

    if left_id <= left_of_right_id {
        penalties[left_of_right_id] - penalties[left_id]
    } else {
        penalties[port_count - 1] - penalties[left_id] + penalties[left_of_right_id]
    }
}

fn compute_port_penalties(graph: &LGraph, node: NodeId) -> Vec<i32> {
    let ports = &graph.node(node).ports;
    let hierarchy_segments = hierarchy_segment_edges(graph);
    let mut out = Vec::with_capacity(ports.len());
    let mut sum = 0_i32;
    for &pid in ports {
        let connected = graph
            .port(pid)
            .incoming_edges
            .iter()
            .chain(graph.port(pid).outgoing_edges.iter())
            .any(|eid| !hierarchy_segments.contains(eid));
        sum += if connected { CONNECTED_PORT_PENALTY } else { UNCONNECTED_PORT_PENALTY };
        out.push(sum);
    }
    out
}

fn hierarchy_segment_edges(graph: &LGraph) -> HashSet<crate::graph::index::EdgeId> {
    let mut edges = HashSet::new();
    let map = graph.properties.get(&crate::intermediate::compound_graph::CROSS_HIERARCHY_MAP);
    for segments in map.values() {
        for segment in segments {
            edges.insert(segment.new_edge);
        }
    }
    edges
}

fn portside_ordinal(side: PortSide) -> u8 {
    match side {
        PortSide::Undefined => 0,
        PortSide::North => 1,
        PortSide::East => 2,
        PortSide::South => 3,
        PortSide::West => 4,
    }
}
