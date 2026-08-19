//! Port distribution helpers shared by the layer sweep orchestrator.
//!
//! The crossing minimizer orchestrator lives in `layer_sweep_crossing_minimizer.rs`.
//! The orchestrator entry point and `CrossMinType` enum are kept here for
//! internal call sites that import from `layer_sweep::*`.

use std::{cmp::Ordering, mem};

use smallvec::SmallVec;

pub(crate) use crate::p3_crossing_min::layer_sweep_crossing_minimizer::{
    CrossMinType, minimize_crossings, minimize_crossings_with_graph_rngs,
};
use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        port::PortSide,
    },
    options::enums::PortConstraints,
    p3_crossing_min::{
        greedy_port_distributor,
        scratch_stats::{self, PortDistributionFootprint},
    },
    properties::internal::{ORIGIN_PORT, P3_IGNORE_NESTED_GRAPHS},
};

type PortBuf = SmallVec<PortId, 6>;

/// Port rank calculation strategy.
#[derive(Clone, Copy)]
pub enum PortRankMode {
    NodeRelative,
    LayerTotal,
}

/// Port type direction for rank calculation.
#[derive(Clone, Copy)]
pub enum PortType {
    Input,
    Output,
}

pub(crate) struct PortRanks {
    values: Vec<f32>,
    touched_indices: Vec<usize>,
}

impl PortRanks {
    pub(crate) fn new() -> Self {
        Self { values: Vec::new(), touched_indices: Vec::new() }
    }

    fn clear(&mut self) {
        for &idx in &self.touched_indices {
            self.values[idx] = f32::NAN;
        }
        self.touched_indices.clear();
    }

    #[inline]
    pub(crate) fn get(&self, port_id: PortId) -> Option<f64> {
        let value = *self.values.get(port_id.0.index() as usize)?;
        if value.is_nan() { None } else { Some(value as f64) }
    }

    #[inline]
    fn insert(&mut self, port_id: PortId, value: f64) {
        let idx = port_id.0.index() as usize;
        if idx >= self.values.len() {
            self.values.resize(idx + 1, f32::NAN);
        }
        if self.values[idx].is_nan() {
            self.touched_indices.push(idx);
        }
        self.values[idx] = value as f32;
    }

    fn retained_bytes(&self) -> usize {
        self.values.capacity() * mem::size_of::<f32>()
            + self.touched_indices.capacity() * mem::size_of::<usize>()
    }
}

struct PortBarycenters {
    values: Vec<f32>,
    touched_indices: Vec<usize>,
}

impl PortBarycenters {
    fn new() -> Self {
        Self { values: Vec::new(), touched_indices: Vec::new() }
    }

    fn clear(&mut self) {
        for &idx in &self.touched_indices {
            self.values[idx] = f32::NAN;
        }
        self.touched_indices.clear();
    }

    #[inline]
    fn get_or_zero(&self, port_id: PortId) -> f64 {
        let Some(&value) = self.values.get(port_id.0.index() as usize) else {
            return 0.0;
        };
        if value.is_nan() { 0.0 } else { value as f64 }
    }

    #[inline]
    fn insert(&mut self, port_id: PortId, value: f64) {
        let idx = port_id.0.index() as usize;
        if idx >= self.values.len() {
            self.values.resize(idx + 1, f32::NAN);
        }
        if self.values[idx].is_nan() {
            self.touched_indices.push(idx);
        }
        self.values[idx] = value as f32;
    }

    fn retained_bytes(&self) -> usize {
        self.values.capacity() * mem::size_of::<f32>()
            + self.touched_indices.capacity() * mem::size_of::<usize>()
    }
}

struct NodeLayerPositions {
    index_to_pos: Vec<u32>,
    touched_indices: Vec<usize>,
    nodes: Vec<NodeId>,
}

impl NodeLayerPositions {
    const MISSING: u32 = u32::MAX;

    fn new() -> Self {
        Self { index_to_pos: Vec::new(), touched_indices: Vec::new(), nodes: Vec::new() }
    }

    fn reset_with_nodes(&mut self, nodes: &[NodeId]) {
        self.clear();
        self.nodes.reserve(nodes.len());
        self.touched_indices.reserve(nodes.len());

        for &node_id in nodes {
            let index = node_id.0.index() as usize;
            if index >= self.index_to_pos.len() {
                self.index_to_pos.resize(index + 1, Self::MISSING);
            }
            let pos = self.nodes.len() as u32;
            self.index_to_pos[index] = pos;
            self.touched_indices.push(index);
            self.nodes.push(node_id);
        }
    }

    fn clear(&mut self) {
        for &index in &self.touched_indices {
            self.index_to_pos[index] = Self::MISSING;
        }
        self.touched_indices.clear();
        self.nodes.clear();
    }

    #[inline]
    fn len(&self) -> usize {
        self.nodes.len()
    }

    #[inline]
    fn get(&self, node_id: NodeId) -> Option<usize> {
        let index = node_id.0.index() as usize;
        let &pos = self.index_to_pos.get(index)?;
        if pos == Self::MISSING {
            return None;
        }
        let pos = pos as usize;
        if self.nodes[pos] == node_id { Some(pos) } else { None }
    }

    fn retained_bytes(&self) -> usize {
        self.index_to_pos.capacity() * mem::size_of::<u32>()
            + self.touched_indices.capacity() * mem::size_of::<usize>()
            + self.nodes.capacity() * mem::size_of::<NodeId>()
    }
}

pub(crate) struct PortDistributionScratch {
    free_ranks: PortRanks,
    barycenters: PortBarycenters,
    current_layer_positions: NodeLayerPositions,
    fixed_layer_nodes: Vec<NodeId>,
    current_layer_nodes: Vec<NodeId>,
    greedy: greedy_port_distributor::GreedyPortDistributorScratch,
}

impl PortDistributionScratch {
    pub(crate) fn new() -> Self {
        Self {
            free_ranks: PortRanks::new(),
            barycenters: PortBarycenters::new(),
            current_layer_positions: NodeLayerPositions::new(),
            fixed_layer_nodes: Vec::new(),
            current_layer_nodes: Vec::new(),
            greedy: greedy_port_distributor::GreedyPortDistributorScratch::new(),
        }
    }

    fn footprint(&self) -> PortDistributionFootprint {
        PortDistributionFootprint {
            retained_bytes: self.free_ranks.retained_bytes()
                + self.barycenters.retained_bytes()
                + self.current_layer_positions.retained_bytes()
                + (self.fixed_layer_nodes.capacity() + self.current_layer_nodes.capacity())
                    * mem::size_of::<NodeId>(),
            free_rank_slots: self.free_ranks.values.len(),
            free_rank_capacity: self.free_ranks.values.capacity(),
            barycenter_slots: self.barycenters.values.len(),
            barycenter_capacity: self.barycenters.values.capacity(),
            node_position_slots: self.current_layer_positions.index_to_pos.len(),
            node_position_capacity: self.current_layer_positions.index_to_pos.capacity(),
        }
    }
}

impl Drop for PortDistributionScratch {
    fn drop(&mut self) {
        if scratch_stats::enabled() {
            scratch_stats::record_port_distribution(self.footprint());
        }
    }
}

pub(crate) fn distribute_ports_while_sweeping_with_fixed_ranks_and_scratch(
    graph: &mut LGraph,
    current_index: usize,
    forward: bool,
    mode: PortRankMode,
    cross_min_type: CrossMinType,
    fixed_ranks: Option<&PortRanks>,
    scratch: &mut PortDistributionScratch,
) -> bool {
    if current_index >= graph.layers.len() {
        return false;
    }

    match cross_min_type {
        CrossMinType::TwoSidedGreedySwitch =>
            if !layer_has_reorderable_ports(graph, current_index) {
                false
            } else {
                greedy_port_distributor::distribute_ports_while_sweeping_with_scratch(
                    graph,
                    current_index,
                    forward,
                    &mut scratch.greedy,
                )
            },
        _ => barycenter_distribute_ports_in_layer_with_fixed_ranks_and_scratch(
            graph,
            current_index,
            forward,
            mode,
            fixed_ranks,
            scratch,
        ),
    }
}

pub(crate) fn barycenter_distribute_ports_in_layer_with_fixed_ranks_and_scratch(
    graph: &mut LGraph,
    current_index: usize,
    forward: bool,
    mode: PortRankMode,
    fixed_ranks: Option<&PortRanks>,
    scratch: &mut PortDistributionScratch,
) -> bool {
    if current_index >= graph.layers.len() {
        return false;
    }

    if !layer_has_reorderable_ports(graph, current_index)
        && (is_first_layer(graph, current_index, forward)
            || !layer_has_reorderable_ports(
                graph,
                if forward { current_index - 1 } else { current_index + 1 },
            ))
    {
        return false;
    }

    let sweep_side = if forward { PortSide::West } else { PortSide::East };
    scratch
        .current_layer_positions
        .reset_with_nodes(&graph.layers[current_index].nodes);
    if !is_first_layer(graph, current_index, forward) {
        let fixed_index = if forward { current_index - 1 } else { current_index + 1 };
        scratch.fixed_layer_nodes.clear();
        scratch.fixed_layer_nodes.extend_from_slice(&graph.layers[fixed_index].nodes);
        let computed_fixed_ranks;
        let fixed_ranks = match fixed_ranks {
            Some(ranks) => ranks,
            None => {
                computed_fixed_ranks = calculate_port_ranks(
                    graph,
                    fixed_index,
                    if forward { PortType::Output } else { PortType::Input },
                    mode,
                );
                &computed_fixed_ranks
            }
        };
        scratch.current_layer_nodes.clear();
        scratch
            .current_layer_nodes
            .extend_from_slice(&graph.layers[current_index].nodes);
        for i in 0..scratch.current_layer_nodes.len() {
            let node_id = scratch.current_layer_nodes[i];
            distribute_ports_of_node(
                graph,
                node_id,
                sweep_side,
                fixed_ranks,
                &mut scratch.barycenters,
                &scratch.current_layer_positions,
            );
        }

        calculate_port_ranks_into(
            graph,
            current_index,
            if forward { PortType::Input } else { PortType::Output },
            mode,
            &mut scratch.free_ranks,
        );
        scratch
            .current_layer_positions
            .reset_with_nodes(&graph.layers[fixed_index].nodes);
        for i in 0..scratch.fixed_layer_nodes.len() {
            let node_id = scratch.fixed_layer_nodes[i];
            if !has_sweepable_nested_graph(graph, node_id) {
                distribute_ports_of_node(
                    graph,
                    node_id,
                    sweep_side.opposed(),
                    &scratch.free_ranks,
                    &mut scratch.barycenters,
                    &scratch.current_layer_positions,
                );
            }
        }
    } else {
        let empty_ranks = PortRanks::new();
        scratch.current_layer_nodes.clear();
        scratch
            .current_layer_nodes
            .extend_from_slice(&graph.layers[current_index].nodes);
        for i in 0..scratch.current_layer_nodes.len() {
            let node_id = scratch.current_layer_nodes[i];
            distribute_ports_of_node(
                graph,
                node_id,
                sweep_side,
                &empty_ranks,
                &mut scratch.barycenters,
                &scratch.current_layer_positions,
            );
        }
    }
    false
}

pub(crate) fn barycenter_distribute_ports_in_layer_with_node_order(
    graph: &mut LGraph,
    node_order: &[Vec<NodeId>],
    current_index: usize,
    forward: bool,
    mode: PortRankMode,
    scratch: &mut PortDistributionScratch,
) -> bool {
    if current_index >= node_order.len() {
        return false;
    }

    let sweep_side = if forward { PortSide::West } else { PortSide::East };
    scratch.current_layer_positions.reset_with_nodes(&node_order[current_index]);
    if !is_first_node_order_layer(node_order.len(), current_index, forward) {
        let fixed_index = if forward { current_index - 1 } else { current_index + 1 };
        let mut fixed_ranks = PortRanks::new();
        calculate_port_ranks_for_nodes_into(
            graph,
            &node_order[fixed_index],
            if forward { PortType::Output } else { PortType::Input },
            mode,
            &mut fixed_ranks,
        );
        for &node_id in &node_order[current_index] {
            distribute_ports_of_node(
                graph,
                node_id,
                sweep_side,
                &fixed_ranks,
                &mut scratch.barycenters,
                &scratch.current_layer_positions,
            );
        }

        calculate_port_ranks_for_nodes_into(
            graph,
            &node_order[current_index],
            if forward { PortType::Input } else { PortType::Output },
            mode,
            &mut scratch.free_ranks,
        );
        scratch.current_layer_positions.reset_with_nodes(&node_order[fixed_index]);
        for &node_id in &node_order[fixed_index] {
            if !has_sweepable_nested_graph(graph, node_id) {
                distribute_ports_of_node(
                    graph,
                    node_id,
                    sweep_side.opposed(),
                    &scratch.free_ranks,
                    &mut scratch.barycenters,
                    &scratch.current_layer_positions,
                );
            }
        }
    } else {
        let empty_ranks = PortRanks::new();
        for &node_id in &node_order[current_index] {
            distribute_ports_of_node(
                graph,
                node_id,
                sweep_side,
                &empty_ranks,
                &mut scratch.barycenters,
                &scratch.current_layer_positions,
            );
        }
    }
    false
}

fn is_first_node_order_layer(length: usize, current_index: usize, forward: bool) -> bool {
    if forward { current_index == 0 } else { current_index + 1 == length }
}

fn has_sweepable_nested_graph(graph: &LGraph, node_id: NodeId) -> bool {
    if graph.properties.get(&P3_IGNORE_NESTED_GRAPHS) {
        return false;
    }
    graph.nested(node_id).is_some_and(|child| !child.layers.is_empty())
}

fn layer_has_reorderable_ports(graph: &LGraph, layer_idx: usize) -> bool {
    graph.layers[layer_idx].nodes.iter().copied().any(|node_id| {
        !node_port_constraints(graph, node_id).is_order_fixed()
            && node_has_reorderable_ports(graph, node_id)
    })
}

fn node_has_reorderable_ports(graph: &LGraph, node_id: NodeId) -> bool {
    let mut seen_sides = [false; 5];
    let mut previous_side = None;

    for &port_id in &graph.node(node_id).ports {
        let side = graph.port(port_id).side as usize;
        if seen_sides[side] {
            return true;
        }
        if let Some(previous) = previous_side
            && side < previous
        {
            return true;
        }
        seen_sides[side] = true;
        previous_side = Some(side);
    }

    false
}

fn distribute_ports_of_node(
    graph: &mut LGraph,
    node_id: NodeId,
    sweep_side: PortSide,
    port_ranks: &PortRanks,
    barycenters: &mut PortBarycenters,
    layer_positions: &NodeLayerPositions,
) {
    if node_port_constraints(graph, node_id).is_order_fixed()
        || !node_has_reorderable_ports(graph, node_id)
    {
        return;
    }

    barycenters.clear();
    distribute_ports_on_side(graph, node_id, sweep_side, port_ranks, barycenters, layer_positions);
    distribute_ports_on_side(
        graph,
        node_id,
        PortSide::South,
        port_ranks,
        barycenters,
        layer_positions,
    );
    distribute_ports_on_side(
        graph,
        node_id,
        PortSide::North,
        port_ranks,
        barycenters,
        layer_positions,
    );
    sort_ports_by_barycenter(graph, node_id, barycenters);
}

fn distribute_ports_on_side(
    graph: &mut LGraph,
    node_id: NodeId,
    side: PortSide,
    port_ranks: &PortRanks,
    barycenters: &mut PortBarycenters,
    layer_positions: &NodeLayerPositions,
) {
    let side_ports = ports_on_side(graph, node_id, side);
    if side_ports.is_empty() {
        return;
    }

    let layer_size = layer_positions.len() + 1;
    let north_south_sort_span = 2.0 * layer_positions.len() as f64 + 1.0;
    let node_index_in_layer = layer_positions.get(node_id).unwrap_or(0) + 1;

    let mut in_layer_ports = PortBuf::new();
    let mut min_barycenter: f64 = 0.0;
    let mut max_barycenter: f64 = 0.0;

    for port_id in &side_ports {
        let port = graph.port(*port_id);
        let is_north_south = matches!(port.side, PortSide::North | PortSide::South);
        if is_north_south {
            if let Some(value) =
                north_south_port_sort_key(graph, *port_id, port.side, north_south_sort_span)
            {
                barycenters.insert(*port_id, value);
            }
            continue;
        }

        let mut sum = 0.0;
        let mut in_layer = false;
        for &edge_id in &port.outgoing_edges {
            let connected_port = graph.edge(edge_id).target;
            if graph.node(graph.port(connected_port).owner).layer == graph.node(node_id).layer {
                in_layer = true;
                break;
            }
            // The underlying ranks table is graph-wide and unranked ports
            // contribute 0 to the sum. Treat a missing entry as a 0-rank
            // contribution.
            if let Some(rank) = port_ranks.get(connected_port) {
                sum += rank;
            }
        }
        if in_layer {
            in_layer_ports.push(*port_id);
            continue;
        }
        for &edge_id in &port.incoming_edges {
            let connected_port = graph.edge(edge_id).source;
            if graph.node(graph.port(connected_port).owner).layer == graph.node(node_id).layer {
                in_layer = true;
                break;
            }
            if let Some(rank) = port_ranks.get(connected_port) {
                sum -= rank;
            }
        }
        if in_layer {
            in_layer_ports.push(*port_id);
            continue;
        }

        // Divide by `port.degree()` (total adjacent edges, including those
        // whose ranks weren't populated and therefore contribute 0 to
        // `sum`), not by the count of populated ranks.
        let degree = port.outgoing_edges.len() + port.incoming_edges.len();
        if degree > 0 {
            let bary = sum / degree as f64;
            min_barycenter = min_barycenter.min(bary);
            max_barycenter = max_barycenter.max(bary);
            barycenters.insert(*port_id, bary);
        }
    }

    for port_id in in_layer_ports {
        let mut sum = 0.0;
        let mut count = 0usize;
        for connected_port in graph
            .port(port_id)
            .incoming_edges
            .iter()
            .chain(graph.port(port_id).outgoing_edges.iter())
            .map(|&edge_id| {
                let edge = graph.edge(edge_id);
                if edge.source == port_id { edge.target } else { edge.source }
            })
        {
            let connected_node = graph.port(connected_port).owner;
            if graph.node(connected_node).layer != graph.node(node_id).layer {
                continue;
            }
            let idx = layer_positions.get(connected_node).unwrap_or(0) + 1;
            sum += idx as f64;
            count += 1;
        }
        if count == 0 {
            continue;
        }
        let bary = sum / count as f64;
        let port_side = graph.port(port_id).side;
        let value = match port_side {
            PortSide::East =>
                if bary < node_index_in_layer as f64 {
                    min_barycenter - bary
                } else {
                    max_barycenter + (layer_size as f64 - bary)
                },
            PortSide::West =>
                if bary < node_index_in_layer as f64 {
                    max_barycenter + bary
                } else {
                    min_barycenter - (layer_size as f64 - bary)
                },
            _ => bary,
        };
        barycenters.insert(port_id, value);
    }
}

fn north_south_port_sort_key(
    graph: &LGraph,
    port_id: PortId,
    side: PortSide,
    absurdly_large_float: f64,
) -> Option<f64> {
    let port_dummy = graph.port(port_id).port_dummy?;
    // For NORTH/SOUTH ports introduced by `north_south_port` the dummy lives
    // in the same LGraph as the port. For compound parents whose external
    // ports were registered by `compound_graph::preprocess`, the dummy lives
    // in the nested LGraph one level deeper. Resolve through the registry so
    // both paths share the same logic.
    let dummy_graph = graph.find_graph_containing(port_dummy)?;
    let layer_idx = dummy_graph.node(port_dummy).layer.get()?;
    let port_dummy_pos = dummy_graph.layers[layer_idx]
        .nodes
        .iter()
        .position(|&n| n == port_dummy)
        .unwrap_or(0) as f64;

    let mut input = false;
    let mut output = false;
    for &dummy_port in &dummy_graph.node(port_dummy).ports {
        if dummy_graph.port(dummy_port).properties.get(&ORIGIN_PORT) == Some(port_id) {
            if !dummy_graph.port(dummy_port).outgoing_edges.is_empty() {
                output = true;
            } else if !dummy_graph.port(dummy_port).incoming_edges.is_empty() {
                input = true;
            }
        }
    }

    let value = if input && (input ^ output) {
        if side == PortSide::North {
            -port_dummy_pos
        } else {
            absurdly_large_float - port_dummy_pos
        }
    } else if output && (input ^ output) {
        port_dummy_pos + 1.0
    } else if input && output {
        if side == PortSide::North { 0.0 } else { absurdly_large_float / 2.0 }
    } else {
        0.0
    };

    Some(value)
}

/// Sort ports clockwise by side (via `PortSide as u8`), then within each side
/// by precomputed barycenter. Zero barycenters are less than any nonzero
/// value and equal among themselves.
fn sort_ports_by_barycenter(graph: &mut LGraph, node_id: NodeId, barycenters: &PortBarycenters) {
    let node_ports: PortBuf = graph.node(node_id).ports.iter().copied().collect();
    let mut merged: SmallVec<(PortId, f64), 6> = node_ports
        .iter()
        .copied()
        .map(|port_id| (port_id, barycenters.get_or_zero(port_id)))
        .collect();

    merged.sort_by(|a, b| {
        let side_a = graph.port(a.0).side;
        let side_b = graph.port(b.0).side;
        if side_a != side_b {
            return (side_a as u8).cmp(&(side_b as u8));
        }
        if a.1 == 0.0 && b.1 == 0.0 {
            Ordering::Equal
        } else if a.1 == 0.0 {
            Ordering::Less
        } else if b.1 == 0.0 {
            Ordering::Greater
        } else {
            a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal)
        }
    });

    let ordered: SmallVec<PortId, 2> = merged.into_iter().map(|(port_id, _)| port_id).collect();
    if graph.node(node_id).ports.as_slice() != ordered.as_slice() {
        graph.node_mut(node_id).ports = ordered;
        graph.bump_node_order_version(node_id);
    }
}

fn ports_on_side(graph: &LGraph, node_id: NodeId, side: PortSide) -> PortBuf {
    let node = graph.node(node_id);
    if node.is_port_side_cached() {
        node.ports_on_side(side).iter().copied().collect()
    } else {
        node.ports
            .iter()
            .copied()
            .filter(|&port_id| graph.port(port_id).side == side)
            .collect()
    }
}

pub(crate) fn calculate_port_ranks(
    graph: &LGraph,
    layer_idx: usize,
    port_type: PortType,
    mode: PortRankMode,
) -> PortRanks {
    let mut ranks = PortRanks::new();
    calculate_port_ranks_into(graph, layer_idx, port_type, mode, &mut ranks);
    ranks
}

pub(crate) fn calculate_port_ranks_into(
    graph: &LGraph,
    layer_idx: usize,
    port_type: PortType,
    mode: PortRankMode,
    ranks: &mut PortRanks,
) {
    calculate_port_ranks_for_nodes_into(
        graph,
        &graph.layers[layer_idx].nodes,
        port_type,
        mode,
        ranks,
    );
}

fn calculate_port_ranks_for_nodes_into(
    graph: &LGraph,
    nodes: &[NodeId],
    port_type: PortType,
    mode: PortRankMode,
    ranks: &mut PortRanks,
) {
    ranks.clear();
    let mut rank_sum = 0.0;
    for &node_id in nodes {
        rank_sum += calculate_port_ranks_for_node(graph, node_id, rank_sum, port_type, mode, ranks);
    }
}

fn calculate_port_ranks_for_node(
    graph: &LGraph,
    node_id: NodeId,
    rank_sum: f64,
    port_type: PortType,
    mode: PortRankMode,
    values: &mut PortRanks,
) -> f64 {
    match port_type {
        PortType::Input => {
            let mut input_count = 0usize;
            let mut north_input_count = 0usize;
            for &port_id in &graph.node(node_id).ports {
                if !graph.port(port_id).incoming_edges.is_empty() {
                    input_count += 1;
                    if graph.port(port_id).side == PortSide::North {
                        north_input_count += 1;
                    }
                }
            }

            match mode {
                PortRankMode::NodeRelative => {
                    let incr = 1.0 / (input_count as f64 + 1.0);
                    let mut north_pos = rank_sum + north_input_count as f64 * incr;
                    let mut rest_pos = rank_sum + 1.0 - incr;
                    for_each_input_port_in_order(graph, node_id, |port_id| {
                        if graph.port(port_id).side == PortSide::North {
                            values.insert(port_id, north_pos);
                            north_pos -= incr;
                        } else {
                            values.insert(port_id, rest_pos);
                            rest_pos -= incr;
                        }
                    });
                    1.0
                }
                PortRankMode::LayerTotal => {
                    let mut north_pos = rank_sum + north_input_count as f64;
                    let mut rest_pos = rank_sum + input_count as f64;
                    for_each_input_port_in_order(graph, node_id, |port_id| {
                        if graph.port(port_id).side == PortSide::North {
                            values.insert(port_id, north_pos);
                            north_pos -= 1.0;
                        } else {
                            values.insert(port_id, rest_pos);
                            rest_pos -= 1.0;
                        }
                    });
                    input_count as f64
                }
            }
        }
        PortType::Output => match mode {
            PortRankMode::NodeRelative => {
                let output_count = graph
                    .node(node_id)
                    .ports
                    .iter()
                    .filter(|&&port_id| !graph.port(port_id).outgoing_edges.is_empty())
                    .count();
                let incr = 1.0 / (output_count as f64 + 1.0);
                let mut pos = rank_sum + incr;
                for_each_output_port_in_order(graph, node_id, |port_id| {
                    values.insert(port_id, pos);
                    pos += incr;
                });
                1.0
            }
            PortRankMode::LayerTotal => {
                let mut pos = 0usize;
                for_each_output_port_in_order(graph, node_id, |port_id| {
                    pos += 1;
                    values.insert(port_id, rank_sum + pos as f64);
                });
                pos as f64
            }
        },
    }
}

#[inline]
fn for_each_input_port_in_order(graph: &LGraph, node_id: NodeId, mut visit: impl FnMut(PortId)) {
    // `node.ports` is already ordered N → E → S → W after `PortListSorter`;
    // walk that side order here so the rank assignments match.
    let node = graph.node(node_id);
    if node.is_port_side_cached() {
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            for &port_id in node.ports_on_side(side) {
                if !graph.port(port_id).incoming_edges.is_empty() {
                    visit(port_id);
                }
            }
        }
    } else {
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            for &port_id in &node.ports {
                let port = graph.port(port_id);
                if port.side == side && !port.incoming_edges.is_empty() {
                    visit(port_id);
                }
            }
        }
    }
}

#[inline]
fn for_each_output_port_in_order(graph: &LGraph, node_id: NodeId, mut visit: impl FnMut(PortId)) {
    let node = graph.node(node_id);
    if node.is_port_side_cached() {
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            for &port_id in node.ports_on_side(side) {
                if !graph.port(port_id).outgoing_edges.is_empty() {
                    visit(port_id);
                }
            }
        }
    } else {
        for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
            for &port_id in &node.ports {
                let port = graph.port(port_id);
                if port.side == side && !port.outgoing_edges.is_empty() {
                    visit(port_id);
                }
            }
        }
    }
}

pub(crate) fn is_first_layer(graph: &LGraph, current_index: usize, forward: bool) -> bool {
    if forward {
        current_index == 0
    } else {
        current_index + 1 == graph.layers.len()
    }
}

pub(crate) fn reorder_parent_ports_on_side(
    graph: &mut LGraph,
    parent_node: NodeId,
    side: PortSide,
    ordered_side_ports: &[PortId],
) {
    let current_ports: Vec<PortId> = graph.node(parent_node).ports.to_vec();
    let mut side_iter = ordered_side_ports.iter().copied();
    let reordered: Vec<PortId> = current_ports
        .iter()
        .copied()
        .map(|port_id| {
            // Only swap hierarchical ports (those backed by an external-port
            // dummy via `port_dummy`) on the target side. Non-hierarchical
            // ports keep their slot, so a node mixing user-defined ports with
            // hierarchical ones preserves stable ordering for the former.
            let on_side = graph.port(port_id).side == side;
            let is_hierarchical = graph.port(port_id).port_dummy.is_some();
            if on_side && is_hierarchical {
                side_iter.next().unwrap_or(port_id)
            } else {
                port_id
            }
        })
        .collect();
    let changed = reordered != current_ports;
    if changed {
        graph.node_mut(parent_node).ports = reordered.into();
        graph.bump_node_order_version(parent_node);
    }
}

pub(crate) fn node_port_constraints(graph: &LGraph, node_id: NodeId) -> PortConstraints {
    let node_constraints = graph.node(node_id).port_constraints();
    if node_constraints == PortConstraints::Undefined {
        graph.options.port_constraints
    } else {
        node_constraints
    }
}
