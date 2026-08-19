//! Self-loop port restorer.
//!
//! Re-attaches hidden self-loop ports, picks a side for them depending on the
//! `EDGE_ROUTING_SELF_LOOP_DISTRIBUTION` option, and then splices them into
//! the node's port list at the correct PortSideArea (Start / Middle / End)
//! slot for the loop's SelfLoopType.

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    intermediate::self_loop_holder::{
        SELF_LOOP_HOLDER, SelfHyperLoop, SelfLoopHolder, SelfLoopType,
    },
    options::enums::{PortConstraints, SelfLoopDistribution, SelfLoopOrdering},
    properties::internal::{
        ORIGINAL_PORT_CONSTRAINTS, SELF_LOOP_DISTRIBUTION_OVERRIDE, SELF_LOOP_ORDERING_OVERRIDE,
    },
};

/// Slot within a single port side, matching `PortSideArea`.
/// Used to place restored self-loop ports at the start, middle, or end of a
/// side's port list.
#[derive(Clone, Copy, PartialEq, Eq)]
enum PortSideArea {
    Start = 0,
    Middle = 1,
    End = 2,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum AddMode {
    Prepend,
    Append,
}

/// Restore hidden self-loop ports, assign port sides based on the self-loop
/// distribution strategy, and refresh hyper-loop type classifications.
pub fn restore(graph: &mut LGraph) {
    let mut targets: Vec<NodeId> = Vec::new();
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            if graph.node(nid).node_type == NodeType::Normal
                && graph.node(nid).properties.has(&SELF_LOOP_HOLDER)
            {
                targets.push(nid);
            }
        }
    }

    for node in targets {
        let Some(mut holder) = graph.node(node).properties.get(&SELF_LOOP_HOLDER) else {
            continue;
        };
        let distribution = graph
            .node(node)
            .properties
            .get(&SELF_LOOP_DISTRIBUTION_OVERRIDE)
            .unwrap_or(graph.options.self_loop_distribution);
        let ordering = graph
            .node(node)
            .properties
            .get(&SELF_LOOP_ORDERING_OVERRIDE)
            .unwrap_or(graph.options.self_loop_ordering);

        if holder.ports_hidden {
            let original = graph.node(node).properties.get(&ORIGINAL_PORT_CONSTRAINTS);
            match original {
                PortConstraints::Undefined | PortConstraints::Free => {
                    assign_port_sides(graph, &mut holder, distribution);
                    apply_port_sides(graph, &holder);
                    restore_hidden_ports(graph, &mut holder, node, ordering);
                }
                PortConstraints::FixedSide => {
                    restore_hidden_ports(graph, &mut holder, node, ordering);
                }
                _ => {
                    // Order-fixed constraints never hide ports during
                    // pre-processing; treat as a no-op so misconfigured
                    // graphs do not panic.
                }
            }
        }

        holder.compute_ports_per_side_all(graph);
        graph.node_mut(node).properties.set(&SELF_LOOP_HOLDER, Some(holder));
    }
}

/// Dispatch on the self-loop distribution strategy to pick sides for hidden
/// ports.
fn assign_port_sides(
    _graph: &LGraph,
    holder: &mut SelfLoopHolder,
    distribution: SelfLoopDistribution,
) {
    match distribution {
        SelfLoopDistribution::North => assign_to_single_side(holder, PortSide::North),
        SelfLoopDistribution::South => assign_to_single_side(holder, PortSide::South),
        SelfLoopDistribution::NorthSouth => assign_to_north_or_south(holder),
        SelfLoopDistribution::EquallyDistributed => assign_to_all_sides(holder),
    }
}

fn assign_to_single_side(holder: &mut SelfLoopHolder, side: PortSide) {
    let ports: Vec<PortId> = hidden_ports(holder);
    for pid in ports {
        if let Some(slot) = holder.sl_port_map.get_mut(&pid) {
            slot.side = side;
        }
    }
}

/// Greedily balance hidden ports between north and south so the two sides end
/// up with a similar port count.
fn assign_to_north_or_south(holder: &mut SelfLoopHolder) {
    let mut north_count: usize = 0;
    let mut south_count: usize = 0;

    // Iterate by hyper-loop so every hidden port in a loop lands on the same
    // side.
    for hyper_idx in 0..holder.sl_hyper_loops.len() {
        let hidden_for_loop = hidden_ports_for_loop(holder, hyper_idx);
        if hidden_for_loop.is_empty() {
            continue;
        }

        let side = if north_count <= south_count {
            north_count += hidden_for_loop.len();
            PortSide::North
        } else {
            south_count += hidden_for_loop.len();
            PortSide::South
        };

        for pid in hidden_for_loop {
            if let Some(slot) = holder.sl_port_map.get_mut(&pid) {
                slot.side = side;
            }
        }
    }
}

/// Cycle through all four sides and the four corners, matching
/// `PortSideAssigner.assignToAllSides`. Larger hyper-loops come first so they
/// land on a single side rather than a corner.
fn assign_to_all_sides(holder: &mut SelfLoopHolder) {
    let mut loop_order: Vec<usize> = (0..holder.sl_hyper_loops.len()).collect();
    loop_order.sort_by(|&a, &b| {
        let la = holder.sl_hyper_loops[a].sl_ports.len();
        let lb = holder.sl_hyper_loops[b].sl_ports.len();
        lb.cmp(&la)
    });

    const TARGETS: [(PortSide, PortSide); 8] = [
        (PortSide::North, PortSide::North),
        (PortSide::South, PortSide::South),
        (PortSide::East, PortSide::East),
        (PortSide::West, PortSide::West),
        (PortSide::West, PortSide::North),
        (PortSide::North, PortSide::East),
        (PortSide::South, PortSide::West),
        (PortSide::East, PortSide::South),
    ];

    for (i, hyper_idx) in loop_order.iter().enumerate() {
        let (first_side, second_side) = TARGETS[i % TARGETS.len()];
        assign_loop_to_target(holder, *hyper_idx, first_side, second_side);
    }
}

/// Split the ports of a single hyper-loop between two target sides. For
/// corner targets, ports are sorted by net flow so mostly-outgoing ports land
/// on the first side and mostly-incoming ports on the second, nudging edges
/// to run clockwise.
fn assign_loop_to_target(
    holder: &mut SelfLoopHolder,
    hyper_idx: usize,
    first_side: PortSide,
    second_side: PortSide,
) {
    if hyper_idx >= holder.sl_hyper_loops.len() {
        return;
    }

    // Determine whether this is a corner target (two different sides).
    let is_corner = first_side != second_side;

    // Snapshot the hyper-loop's port list and sort by net flow for corner
    // targets. The hyper-loop's port list is replaced with the sorted order
    // so downstream assignment sees the same ordering.
    let mut ports: Vec<PortId> = holder.sl_hyper_loops[hyper_idx].sl_ports.clone();
    if is_corner {
        ports.sort_by_key(|pid| holder.sl_port_map.get(pid).map(|p| p.sl_net_flow()).unwrap_or(0));
    }
    holder.sl_hyper_loops[hyper_idx].sl_ports = ports.clone();

    let half = ports.len() / 2;
    for (i, pid) in ports.iter().enumerate() {
        if let Some(slot) = holder.sl_port_map.get_mut(pid) {
            if !slot.is_hidden {
                continue;
            }
            slot.side = if i < half { first_side } else { second_side };
        }
    }
}

fn hidden_ports(holder: &SelfLoopHolder) -> Vec<PortId> {
    holder.sl_port_map.values().filter(|p| p.is_hidden).map(|p| p.port_id).collect()
}

fn hidden_ports_for_loop(holder: &SelfLoopHolder, hyper_idx: usize) -> Vec<PortId> {
    let hyper: &SelfHyperLoop = &holder.sl_hyper_loops[hyper_idx];
    hyper
        .sl_ports
        .iter()
        .copied()
        .filter(|pid| holder.sl_port_map.get(pid).map(|p| p.is_hidden).unwrap_or(false))
        .collect()
}

/// Push the holder's port-side assignment back onto the underlying ports.
fn apply_port_sides(graph: &mut LGraph, holder: &SelfLoopHolder) {
    let updates: Vec<(PortId, PortSide)> = holder
        .sl_port_map
        .values()
        .filter(|p| p.is_hidden)
        .map(|p| (p.port_id, p.side))
        .collect();
    for (pid, side) in updates {
        graph.port_mut(pid).side = side;
    }
}

/// Re-attach previously hidden ports to their owning node, placing each into
/// the correct PortSideArea slot based on the SelfLoopType of its hyper-loop,
/// then splicing the per-area lists into the remaining (non-self-loop) ports.
fn restore_hidden_ports(
    graph: &mut LGraph,
    holder: &mut SelfLoopHolder,
    node: NodeId,
    ordering: SelfLoopOrdering,
) {
    // Rebuild the hyper-loop → ports_by_side cache before bucketising so the
    // Start/Middle/End placement sees the freshly-assigned sides.
    holder.compute_ports_per_side_all(graph);

    // 4 sides × 3 areas of hidden self-loop ports.
    let mut target_areas: [[Vec<PortId>; 3]; 4] = Default::default();
    bucket_hyper_loops(holder, ordering, &mut target_areas);

    // Build the new port list by interleaving per-side Start / Middle / End
    // slots with the existing non-self-loop ports.
    let old_ports: Vec<PortId> = graph.node(node).ports.iter().copied().collect();
    let hidden_set: hashbrown::HashSet<PortId> =
        holder.sl_port_map.values().filter(|p| p.is_hidden).map(|p| p.port_id).collect();
    // Separate the non-hidden "original" ports — they act as the regular
    // backbone the hidden self-loop ports are interleaved into.
    let old_visible: Vec<PortId> =
        old_ports.into_iter().filter(|pid| !hidden_set.contains(pid)).collect();

    let mut new_ports: Vec<PortId> = Vec::with_capacity(old_visible.len() + hidden_set.len());
    let mut cursor: usize = 0;

    // NORTH side.
    extend_from_area(&target_areas, PortSide::North, PortSideArea::Start, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::North
            && is_north_south_port_with_west_or_west_east(graph, pid)
    });
    extend_from_area(&target_areas, PortSide::North, PortSideArea::Middle, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::North
    });
    extend_from_area(&target_areas, PortSide::North, PortSideArea::End, &mut new_ports);

    // EAST side.
    extend_from_area(&target_areas, PortSide::East, PortSideArea::Start, &mut new_ports);
    extend_from_area(&target_areas, PortSide::East, PortSideArea::Middle, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::East
    });
    extend_from_area(&target_areas, PortSide::East, PortSideArea::End, &mut new_ports);

    // SOUTH side.
    extend_from_area(&target_areas, PortSide::South, PortSideArea::Start, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::South && is_north_south_port_with_east(graph, pid)
    });
    extend_from_area(&target_areas, PortSide::South, PortSideArea::Middle, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::South
    });
    extend_from_area(&target_areas, PortSide::South, PortSideArea::End, &mut new_ports);

    // WEST side.
    extend_from_area(&target_areas, PortSide::West, PortSideArea::Start, &mut new_ports);
    cursor = consume_while(&old_visible, cursor, &mut new_ports, |pid| {
        graph.port(pid).side == PortSide::West
    });
    extend_from_area(&target_areas, PortSide::West, PortSideArea::Middle, &mut new_ports);
    extend_from_area(&target_areas, PortSide::West, PortSideArea::End, &mut new_ports);

    // Append any leftover visible ports (e.g. Undefined side); ordering covers
    // the four main sides while any residual port is added at the end.
    for pid in &old_visible[cursor..] {
        new_ports.push(*pid);
    }

    graph.node_mut(node).ports = new_ports.into_iter().collect();

    // Clear the hidden flag now that the ports are re-attached.
    for slot in holder.sl_port_map.values_mut() {
        if slot.is_hidden {
            slot.is_hidden = false;
        }
    }
    holder.ports_hidden = false;
}

/// Classify each hyper-loop by `SelfLoopType` and push its hidden ports into
/// the right (side, area) bucket.
fn bucket_hyper_loops(
    holder: &SelfLoopHolder,
    ordering: SelfLoopOrdering,
    target_areas: &mut [[Vec<PortId>; 3]; 4],
) {
    // Group hyper-loop indices by type.
    let mut by_type: [Vec<usize>; 5] = Default::default();
    for (idx, hyper) in holder.sl_hyper_loops.iter().enumerate() {
        let Some(ty) = hyper.self_loop_type else { continue };
        by_type[self_loop_type_index(ty)].push(idx);
    }

    let mut one_side = by_type[self_loop_type_index(SelfLoopType::OneSide)].clone();
    if ordering == SelfLoopOrdering::ReverseStacked {
        one_side.reverse();
    }
    for idx in one_side {
        let hyper = &holder.sl_hyper_loops[idx];
        let Some(&first_pid) = hyper.sl_ports.first() else { continue };
        let Some(first_slot) = holder.sl_port_map.get(&first_pid) else { continue };
        let side = first_slot.side;
        // Sort this loop's ports by net flow ascending so outflow-heavy ports
        // land first.
        let mut sorted: Vec<PortId> = hyper.sl_ports.clone();
        sorted.sort_by_key(|pid| holder.sl_port_map.get(pid).map(|s| s.sl_net_flow()).unwrap_or(0));
        match ordering {
            SelfLoopOrdering::Sequenced => {
                push_hidden(
                    holder,
                    &sorted,
                    side,
                    PortSideArea::Middle,
                    AddMode::Append,
                    target_areas,
                );
            }
            SelfLoopOrdering::Stacked | SelfLoopOrdering::ReverseStacked => {
                let split = compute_port_list_split_index(holder, &sorted);
                push_hidden(
                    holder,
                    &sorted[..split],
                    side,
                    PortSideArea::Middle,
                    AddMode::Prepend,
                    target_areas,
                );
                push_hidden(
                    holder,
                    &sorted[split..],
                    side,
                    PortSideArea::Middle,
                    AddMode::Append,
                    target_areas,
                );
            }
        }
    }

    for idx in &by_type[self_loop_type_index(SelfLoopType::TwoSidesCorner)] {
        let hyper = &holder.sl_hyper_loops[*idx];
        let sides = sorted_two_side_port_sides(hyper);
        let [s0, s1] = sides;
        let ports_s0 = ports_of_hyper_on_side(holder, hyper, s0);
        let ports_s1 = ports_of_hyper_on_side(holder, hyper, s1);
        push_hidden(holder, &ports_s0, s0, PortSideArea::End, AddMode::Prepend, target_areas);
        push_hidden(holder, &ports_s1, s1, PortSideArea::Start, AddMode::Append, target_areas);
    }

    for idx in &by_type[self_loop_type_index(SelfLoopType::ThreeSides)] {
        let hyper = &holder.sl_hyper_loops[*idx];
        let sides = three_side_constellation(hyper);
        let ports_0 = ports_of_hyper_on_side(holder, hyper, sides[0]);
        let ports_1 = ports_of_hyper_on_side(holder, hyper, sides[1]);
        let ports_2 = ports_of_hyper_on_side(holder, hyper, sides[2]);
        push_hidden(holder, &ports_0, sides[0], PortSideArea::End, AddMode::Prepend, target_areas);
        push_hidden(
            holder,
            &ports_1,
            sides[1],
            PortSideArea::Middle,
            AddMode::Append,
            target_areas,
        );
        push_hidden(holder, &ports_2, sides[2], PortSideArea::Start, AddMode::Append, target_areas);
    }

    for idx in &by_type[self_loop_type_index(SelfLoopType::FourSides)] {
        let hyper = &holder.sl_hyper_loops[*idx];
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            let ports = ports_of_hyper_on_side(holder, hyper, side);
            if !ports.is_empty() {
                push_hidden(
                    holder,
                    &ports,
                    side,
                    PortSideArea::Middle,
                    AddMode::Append,
                    target_areas,
                );
            }
        }
    }

    for idx in &by_type[self_loop_type_index(SelfLoopType::TwoSidesOpposing)] {
        let hyper = &holder.sl_hyper_loops[*idx];
        let sides = sorted_two_side_port_sides(hyper);
        let [s0, s1] = sides;
        let ports_s0 = ports_of_hyper_on_side(holder, hyper, s0);
        let ports_s1 = ports_of_hyper_on_side(holder, hyper, s1);
        push_hidden(holder, &ports_s0, s0, PortSideArea::End, AddMode::Prepend, target_areas);
        push_hidden(holder, &ports_s1, s1, PortSideArea::Start, AddMode::Append, target_areas);
    }
}

/// Find the first port with positive / non-negative net flow and use that as
/// the split point, falling back to the centre of the list when neither
/// qualifies.
fn compute_port_list_split_index(holder: &SelfLoopHolder, sorted_ports: &[PortId]) -> usize {
    if sorted_ports.is_empty() {
        return 0;
    }
    let net_flow =
        |pid: &PortId| -> i32 { holder.sl_port_map.get(pid).map(|s| s.sl_net_flow()).unwrap_or(0) };

    let mut positive_idx = 0;
    while positive_idx < sorted_ports.len() {
        if net_flow(&sorted_ports[positive_idx]) > 0 {
            break;
        }
        positive_idx += 1;
    }
    if positive_idx > 0 && positive_idx < sorted_ports.len().saturating_sub(1) {
        return positive_idx;
    }
    // The second scan uses a `> 0` condition matching the first loop's
    // observed behaviour.
    let mut non_negative_idx = 0;
    while non_negative_idx < sorted_ports.len() {
        if net_flow(&sorted_ports[non_negative_idx]) > 0 {
            break;
        }
        non_negative_idx += 1;
    }
    if non_negative_idx > 0 && positive_idx < sorted_ports.len().saturating_sub(1) {
        return non_negative_idx;
    }
    sorted_ports.len() / 2
}

/// Return the two occupied sides in clockwise order, with `(West, North)`
/// swapped from the sort's natural `(North, West)`.
fn sorted_two_side_port_sides(hyper: &SelfHyperLoop) -> [PortSide; 2] {
    let mut sides: Vec<PortSide> = Vec::with_capacity(2);
    for (i, bucket) in hyper.sl_ports_by_side.iter().enumerate() {
        if !bucket.is_empty() {
            let s = match i {
                0 => PortSide::North,
                1 => PortSide::East,
                2 => PortSide::South,
                3 => PortSide::West,
                _ => PortSide::Undefined,
            };
            sides.push(s);
        }
    }
    if sides.len() != 2 {
        return [
            sides.first().copied().unwrap_or(PortSide::North),
            sides.get(1).copied().unwrap_or(PortSide::North),
        ];
    }
    // Sort PortSide by ordinal (N=0, E=1, S=2, W=3) then swap
    // (N, W) → (W, N) because the natural ordinal order breaks clockwise.
    sides.sort_by_key(|s| match s {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => 4,
    });
    if sides[0] == PortSide::North && sides[1] == PortSide::West {
        return [PortSide::West, PortSide::North];
    }
    [sides[0], sides[1]]
}

/// For a 3-side loop, return the sides in clockwise order starting at the one
/// following the missing side.
fn three_side_constellation(hyper: &SelfHyperLoop) -> [PortSide; 3] {
    let has = |s: PortSide| {
        !hyper.sl_ports_by_side[match s {
            PortSide::North => 0,
            PortSide::East => 1,
            PortSide::South => 2,
            PortSide::West => 3,
            PortSide::Undefined => 4,
        }]
        .is_empty()
    };
    if !has(PortSide::North) {
        return [PortSide::East, PortSide::South, PortSide::West];
    }
    if !has(PortSide::East) {
        return [PortSide::South, PortSide::West, PortSide::North];
    }
    if !has(PortSide::South) {
        return [PortSide::West, PortSide::North, PortSide::East];
    }
    [PortSide::North, PortSide::East, PortSide::South]
}

/// Return the hidden ports of `hyper` that sit on `side`.
fn ports_of_hyper_on_side(
    holder: &SelfLoopHolder,
    hyper: &SelfHyperLoop,
    side: PortSide,
) -> Vec<PortId> {
    let idx = match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => return Vec::new(),
    };
    hyper.sl_ports_by_side[idx]
        .iter()
        .copied()
        .filter(|pid| holder.sl_port_map.get(pid).map(|s| s.is_hidden).unwrap_or(false))
        .collect()
}

fn self_loop_type_index(ty: SelfLoopType) -> usize {
    match ty {
        SelfLoopType::OneSide => 0,
        SelfLoopType::TwoSidesCorner => 1,
        SelfLoopType::TwoSidesOpposing => 2,
        SelfLoopType::ThreeSides => 3,
        SelfLoopType::FourSides => 4,
    }
}

/// Filter a collection to hidden ports, reverse, and prepend / append to the
/// target bucket.
fn push_hidden(
    _holder: &SelfLoopHolder,
    ports: &[PortId],
    side: PortSide,
    area: PortSideArea,
    add_mode: AddMode,
    target_areas: &mut [[Vec<PortId>; 3]; 4],
) {
    let side_idx = match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => return,
    };
    let area_idx = area as usize;
    let mut reversed: Vec<PortId> = ports.iter().rev().copied().collect();
    match add_mode {
        AddMode::Append => target_areas[side_idx][area_idx].extend(reversed),
        AddMode::Prepend => {
            reversed.append(&mut target_areas[side_idx][area_idx]);
            target_areas[side_idx][area_idx] = reversed;
        }
    }
}

fn extend_from_area(
    target_areas: &[[Vec<PortId>; 3]; 4],
    side: PortSide,
    area: PortSideArea,
    out: &mut Vec<PortId>,
) {
    let side_idx = match side {
        PortSide::North => 0,
        PortSide::East => 1,
        PortSide::South => 2,
        PortSide::West => 3,
        PortSide::Undefined => return,
    };
    out.extend(target_areas[side_idx][area as usize].iter().copied());
}

/// Copy ports from `from` starting at `cursor` while the predicate matches,
/// writing them into `out`. Returns the first index that did not match.
fn consume_while<F: FnMut(PortId) -> bool>(
    from: &[PortId],
    cursor: usize,
    out: &mut Vec<PortId>,
    mut pred: F,
) -> usize {
    let mut i = cursor;
    while i < from.len() && pred(from[i]) {
        out.push(from[i]);
        i += 1;
    }
    i
}

/// True when a N/S port's dummy carries at least one west connection
/// (optionally an additional east).
fn is_north_south_port_with_west_or_west_east(graph: &LGraph, port_id: PortId) -> bool {
    let sides = north_south_port_connection_sides(graph, port_id);
    sides.contains(&PortSide::West)
}

/// True when a N/S port's dummy has an east connection.
fn is_north_south_port_with_east(graph: &LGraph, port_id: PortId) -> bool {
    let sides = north_south_port_connection_sides(graph, port_id);
    sides.contains(&PortSide::East)
}

fn north_south_port_connection_sides(
    graph: &LGraph,
    port_id: PortId,
) -> smallvec::SmallVec<PortSide, 2> {
    let mut sides: smallvec::SmallVec<PortSide, 2> = smallvec::SmallVec::new();
    let Some(dummy) = graph.port(port_id).port_dummy else {
        return sides;
    };
    let dummy_ports: smallvec::SmallVec<PortId, 4> =
        graph.node(dummy).ports.iter().copied().collect();
    for dp in dummy_ports {
        let origin = graph.port(dp).properties.get(&crate::properties::internal::ORIGIN_PORT);
        if origin != Some(port_id) {
            continue;
        }
        let has_edges =
            !graph.port(dp).incoming_edges.is_empty() || !graph.port(dp).outgoing_edges.is_empty();
        if !has_edges {
            continue;
        }
        let side = graph.port(dp).side;
        if !sides.contains(&side) {
            sides.push(side);
        }
    }
    sides
}
