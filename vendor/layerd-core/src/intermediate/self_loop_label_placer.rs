//! Self-loop label side, alignment, and primary-axis placement.
//!
//! Runs between `RoutingDirector` and `RoutingSlotAssigner`. For every
//! hyper-loop that carries labels, pick a side, an alignment on that side, an
//! alignment reference port, and the primary-axis coordinate (x for
//! north/south labels, y for east/west labels). The perpendicular coordinate
//! is set later by the orthogonal router once the routing slot is known.

use hashbrown::HashMap;

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        port::{PortSide, PortSideSet},
    },
    intermediate::{
        self_hyper_loop_labels::{Alignment, SelfHyperLoopLabels},
        self_loop_holder::{SelfHyperLoop, SelfLoopHolder, SelfLoopType, portside_set_of},
    },
    options::enums::SelfLoopOrdering,
    properties::internal::{EDGE_LABELS_INLINE, SELF_LOOP_ORDERING_OVERRIDE},
};

/// Set `side`, `alignment`, `alignment_reference_port`, and the primary-axis
/// component of `position` on every hyper-loop's `SelfHyperLoopLabels`.
pub fn place_labels(graph: &mut LGraph, holder: &mut SelfLoopHolder, node: NodeId) {
    let ordering = graph
        .node(node)
        .properties
        .get(&SELF_LOOP_ORDERING_OVERRIDE)
        .unwrap_or(graph.options.self_loop_ordering);
    let node_size = graph.node(node).size;

    let mut port_index: HashMap<PortId, u32> = HashMap::new();
    for (i, &pid) in graph.node(node).ports.iter().enumerate() {
        port_index.insert(pid, i as u32);
    }

    // Phase 1: assign side and alignment. One-sided sequenced loops are
    // deferred so they can be paired inward from the outside.
    let mut northern_sequenced: Vec<usize> = Vec::new();
    let mut southern_sequenced: Vec<usize> = Vec::new();

    for hyper_idx in 0..holder.sl_hyper_loops.len() {
        let Some(loop_type) = holder.sl_hyper_loops[hyper_idx].self_loop_type else {
            continue;
        };
        if holder.sl_hyper_loops[hyper_idx].sl_labels.is_none() {
            continue;
        }

        match loop_type {
            SelfLoopType::OneSide => {
                let side = one_sided_loop_side(&holder.sl_hyper_loops[hyper_idx]);
                if ordering == SelfLoopOrdering::Sequenced && side == PortSide::North {
                    northern_sequenced.push(hyper_idx);
                } else if ordering == SelfLoopOrdering::Sequenced && side == PortSide::South {
                    southern_sequenced.push(hyper_idx);
                } else {
                    assign_one_sided_simple(graph, holder, hyper_idx, side);
                }
            }
            SelfLoopType::TwoSidesCorner => {
                assign_two_sides_corner(graph, holder, hyper_idx);
            }
            SelfLoopType::TwoSidesOpposing | SelfLoopType::ThreeSides => {
                assign_two_sides_opposing_or_three(graph, holder, hyper_idx);
            }
            SelfLoopType::FourSides => {
                assign_four_sides(graph, holder, hyper_idx);
            }
        }
    }

    if !northern_sequenced.is_empty() {
        assign_sequenced_side(holder, &northern_sequenced, PortSide::North, &port_index);
    }
    if !southern_sequenced.is_empty() {
        assign_sequenced_side(holder, &southern_sequenced, PortSide::South, &port_index);
    }

    // Phase 2: compute primary-axis coordinates for every labelled hyper-loop.
    for hyper in &mut holder.sl_hyper_loops {
        if let Some(slabels) = hyper.sl_labels.as_mut() {
            compute_coordinates(graph, slabels, node_size);
        }
    }
}

fn one_sided_loop_side(hyper: &SelfHyperLoop) -> PortSide {
    // Iterate `occupied_port_sides`; for ONE_SIDE there is exactly one.
    for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
        if hyper.occupied_port_sides.contains(portside_set_of(side)) {
            return side;
        }
    }
    PortSide::Undefined
}

fn assign_one_sided_simple(
    graph: &mut LGraph,
    holder: &mut SelfLoopHolder,
    hyper_idx: usize,
    side: PortSide,
) {
    clear_inline_property(graph, &holder.sl_hyper_loops[hyper_idx]);
    match side {
        PortSide::East | PortSide::West => {
            // Align relative to the topmost port (smallest y) among leftmost
            // / rightmost.
            let left = holder.sl_hyper_loops[hyper_idx].leftmost_port;
            let right = holder.sl_hyper_loops[hyper_idx].rightmost_port;
            let topmost = topmost_of_two(graph, left, right);
            set_labels(holder, hyper_idx, side, Alignment::Top, topmost);
        }
        PortSide::North | PortSide::South => {
            set_labels(holder, hyper_idx, side, Alignment::Center, None);
        }
        PortSide::Undefined => {}
    }
}

fn assign_two_sides_corner(graph: &mut LGraph, holder: &mut SelfLoopHolder, hyper_idx: usize) {
    clear_inline_property(graph, &holder.sl_hyper_loops[hyper_idx]);
    let left = holder.sl_hyper_loops[hyper_idx].leftmost_port;
    let right = holder.sl_hyper_loops[hyper_idx].rightmost_port;
    let left_side = left.map(|p| graph.port(p).side).unwrap_or(PortSide::Undefined);
    let right_side = right.map(|p| graph.port(p).side).unwrap_or(PortSide::Undefined);

    if left_side == PortSide::North {
        set_labels(holder, hyper_idx, PortSide::North, Alignment::Left, left);
    } else if right_side == PortSide::North {
        set_labels(holder, hyper_idx, PortSide::North, Alignment::Right, right);
    } else if left_side == PortSide::South {
        set_labels(holder, hyper_idx, PortSide::South, Alignment::Right, left);
    } else if right_side == PortSide::South {
        set_labels(holder, hyper_idx, PortSide::South, Alignment::Left, right);
    }
}

fn assign_two_sides_opposing_or_three(
    graph: &LGraph,
    holder: &mut SelfLoopHolder,
    hyper_idx: usize,
) {
    let occupied = holder.sl_hyper_loops[hyper_idx].occupied_port_sides;
    let left = holder.sl_hyper_loops[hyper_idx].leftmost_port;
    let right = holder.sl_hyper_loops[hyper_idx].rightmost_port;
    let has_inline = hyper_has_inline_labels(graph, &holder.sl_hyper_loops[hyper_idx]);

    let assigned = if !occupied.contains(PortSideSet::NORTH) {
        Some((PortSide::South, Alignment::Center, None))
    } else if !occupied.contains(PortSideSet::SOUTH) {
        Some((PortSide::North, Alignment::Center, None))
    } else if !occupied.contains(PortSideSet::WEST) {
        if has_inline {
            Some((PortSide::East, Alignment::Center, None))
        } else {
            Some((PortSide::North, Alignment::Left, left))
        }
    } else if !occupied.contains(PortSideSet::EAST) {
        if has_inline {
            Some((PortSide::West, Alignment::Center, None))
        } else {
            Some((PortSide::North, Alignment::Right, right))
        }
    } else {
        None
    };

    if let Some((side, alignment, reference)) = assigned {
        set_labels(holder, hyper_idx, side, alignment, reference);
    }
}

fn assign_four_sides(graph: &mut LGraph, holder: &mut SelfLoopHolder, hyper_idx: usize) {
    clear_inline_property(graph, &holder.sl_hyper_loops[hyper_idx]);
    let left = holder.sl_hyper_loops[hyper_idx].leftmost_port;
    let right = holder.sl_hyper_loops[hyper_idx].rightmost_port;
    let leftmost_side = left.map(|p| graph.port(p).side).unwrap_or(PortSide::Undefined);
    let rightmost_side = right.map(|p| graph.port(p).side).unwrap_or(PortSide::Undefined);

    if leftmost_side == PortSide::North || rightmost_side == PortSide::North {
        set_labels(holder, hyper_idx, PortSide::South, Alignment::Center, None);
    } else {
        set_labels(holder, hyper_idx, PortSide::North, Alignment::Center, None);
    }
}

fn assign_sequenced_side(
    holder: &mut SelfLoopHolder,
    loop_indices: &[usize],
    side: PortSide,
    port_index: &HashMap<PortId, u32>,
) {
    let mut sorted: Vec<usize> = loop_indices.to_vec();
    // Sort by leftmost port id; southern side reverses so leftmost ends up
    // first in the list.
    if side == PortSide::North {
        sorted.sort_by_key(|&idx| {
            holder.sl_hyper_loops[idx]
                .leftmost_port
                .and_then(|p| port_index.get(&p).copied())
                .unwrap_or(u32::MAX)
        });
    } else {
        sorted.sort_by(|&a, &b| {
            let ia = holder.sl_hyper_loops[a]
                .leftmost_port
                .and_then(|p| port_index.get(&p).copied())
                .unwrap_or(u32::MAX);
            let ib = holder.sl_hyper_loops[b]
                .leftmost_port
                .and_then(|p| port_index.get(&p).copied())
                .unwrap_or(u32::MAX);
            ib.cmp(&ia)
        });
    }

    let mut left_idx = 0_usize;
    let mut right_idx = sorted.len().saturating_sub(1);
    while left_idx < right_idx {
        let left_loop = sorted[left_idx];
        let right_loop = sorted[right_idx];
        let (left_ref, right_ref) = if side == PortSide::North {
            (
                holder.sl_hyper_loops[left_loop].rightmost_port,
                holder.sl_hyper_loops[right_loop].leftmost_port,
            )
        } else {
            (
                holder.sl_hyper_loops[left_loop].leftmost_port,
                holder.sl_hyper_loops[right_loop].rightmost_port,
            )
        };
        set_labels(holder, left_loop, side, Alignment::Right, left_ref);
        set_labels(holder, right_loop, side, Alignment::Left, right_ref);
        left_idx += 1;
        right_idx -= 1;
    }
    if left_idx == right_idx {
        set_labels(holder, sorted[left_idx], side, Alignment::Center, None);
    }
}

fn set_labels(
    holder: &mut SelfLoopHolder,
    hyper_idx: usize,
    side: PortSide,
    alignment: Alignment,
    reference: Option<PortId>,
) {
    if let Some(slabels) = holder.sl_hyper_loops[hyper_idx].sl_labels.as_mut() {
        slabels.side = Some(side);
        slabels.alignment = Some(alignment);
        slabels.alignment_reference_port = reference;
    }
}

fn clear_inline_property(graph: &mut LGraph, hyper: &SelfHyperLoop) {
    // Explicitly set EDGE_LABELS_INLINE to false to override any manual
    // marking for labels that do not support inline placement.
    let Some(slabels) = &hyper.sl_labels else { return };
    for &lid in &slabels.labels {
        graph.label_mut(lid).properties.set(&EDGE_LABELS_INLINE, false);
    }
}

fn hyper_has_inline_labels(graph: &LGraph, hyper: &SelfHyperLoop) -> bool {
    let Some(slabels) = &hyper.sl_labels else { return false };
    slabels
        .labels
        .iter()
        .any(|&lid| graph.label(lid).properties.get(&EDGE_LABELS_INLINE))
}

fn topmost_of_two(graph: &LGraph, a: Option<PortId>, b: Option<PortId>) -> Option<PortId> {
    match (a, b) {
        (Some(pa), Some(pb)) =>
            if graph.port(pb).position.y < graph.port(pa).position.y {
                Some(pb)
            } else {
                Some(pa)
            },
        (Some(p), None) | (None, Some(p)) => Some(p),
        (None, None) => None,
    }
}

/// Primary-axis placement. For north/south labels this sets `position.x`; for
/// east/west labels it sets `position.y`. Matches
/// `LabelPlacer.computeCoordinates`.
fn compute_coordinates(
    graph: &LGraph,
    slabels: &mut SelfHyperLoopLabels,
    node_size: crate::math::Vec2,
) {
    let Some(alignment) = slabels.alignment else { return };
    let reference = slabels
        .alignment_reference_port
        .map(|p| (graph.port(p).position, graph.port(p).anchor))
        .map(|(pos, anch)| crate::math::Vec2::new(pos.x + anch.x, pos.y + anch.y));

    match alignment {
        Alignment::Center => {
            slabels.position.x = (node_size.x - slabels.size.x) / 2.0;
        }
        Alignment::Left =>
            if let Some(r) = reference {
                slabels.position.x = r.x;
            },
        Alignment::Right =>
            if let Some(r) = reference {
                slabels.position.x = r.x - slabels.size.x;
            },
        Alignment::Top =>
            if let Some(r) = reference {
                slabels.position.y = r.y;
            },
    }
}
