use crate::{
    graph::{
        LGraph,
        edge::EdgeFlags,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{Alignment, CenterEdgeLabelPlacementStrategy},
    properties::internal::{
        ALIGNMENT, CENTER_LABEL_PLACEMENT_STRATEGY, LONG_EDGE_BEFORE_LABEL_DUMMY,
        REPRESENTED_LABELS,
    },
};

/// Moves each center edge label dummy into the layer chosen by the configured
/// placement strategy, swapping it with a long-edge dummy already in that
/// layer.
///
/// Marks the long-edge dummies that precede a label dummy with
/// `LONG_EDGE_BEFORE_LABEL_DUMMY = true` so `HyperedgeDummyMerger` does not
/// cross the label boundary when merging.
pub fn switch(graph: &mut LGraph) {
    let default_strategy = graph.options.center_label_placement;

    let infos = gather_label_dummy_infos(graph, default_strategy);
    if infos.is_empty() {
        return;
    }

    let mut layer_widths: Vec<f64> = vec![0.0; graph.layers.len()];
    if infos.iter().any(|info| info.placement_strategy.uses_label_size_information()) {
        compute_layer_widths(graph, &mut layer_widths);
    }

    let mut non_width_based: Vec<LabelDummyInfo> = Vec::new();
    let mut width_based: Vec<LabelDummyInfo> = Vec::new();
    for info in infos {
        if info.placement_strategy.uses_label_size_information() {
            width_based.push(info);
        } else {
            non_width_based.push(info);
        }
    }

    process_infos(graph, non_width_based, &mut layer_widths);
    process_infos(graph, width_based, &mut layer_widths);
}

struct LabelDummyInfo {
    label_dummy: NodeId,
    placement_strategy: CenterEdgeLabelPlacementStrategy,
    left_dummies: Vec<NodeId>,
    right_dummies: Vec<NodeId>,
    leftmost_layer_id: usize,
    rightmost_layer_id: usize,
}

impl LabelDummyInfo {
    fn total_dummy_count(&self) -> usize {
        self.rightmost_layer_id - self.leftmost_layer_id + 1
    }

    fn ith_dummy_node(&self, i: usize) -> NodeId {
        if i < self.left_dummies.len() {
            self.left_dummies[i]
        } else if i == self.left_dummies.len() {
            self.label_dummy
        } else {
            self.right_dummies[i - self.left_dummies.len() - 1]
        }
    }
}

fn gather_label_dummy_infos(
    graph: &LGraph,
    default_strategy: CenterEdgeLabelPlacementStrategy,
) -> Vec<LabelDummyInfo> {
    let mut infos: Vec<LabelDummyInfo> = Vec::new();
    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            if graph.node(node_id).node_type != NodeType::Label {
                continue;
            }
            let (left, right) = gather_dummy_chain(graph, node_id);
            let leftmost = match left.first() {
                Some(&n) => graph.node(n).layer.unwrap_or(0),
                None => graph.node(node_id).layer.unwrap_or(0),
            };
            let rightmost = match right.last() {
                Some(&n) => graph.node(n).layer.unwrap_or(0),
                None => graph.node(node_id).layer.unwrap_or(0),
            };
            let mut placement_strategy = default_strategy;
            let represented_labels: Vec<_> =
                graph.node(node_id).properties.get(&REPRESENTED_LABELS);
            for label_id in represented_labels {
                if let Some(strategy) =
                    graph.label(label_id).properties.get(&CENTER_LABEL_PLACEMENT_STRATEGY)
                {
                    placement_strategy = strategy;
                    break;
                }
            }
            infos.push(LabelDummyInfo {
                label_dummy: node_id,
                placement_strategy,
                left_dummies: left,
                right_dummies: right,
                leftmost_layer_id: leftmost,
                rightmost_layer_id: rightmost,
            });
        }
    }
    infos
}

fn gather_dummy_chain(graph: &LGraph, label_dummy: NodeId) -> (Vec<NodeId>, Vec<NodeId>) {
    let mut left: Vec<NodeId> = Vec::new();
    let mut current = label_dummy;
    loop {
        let next = first_incoming_source_node(graph, current);
        match next {
            Some(n) if graph.node(n).node_type == NodeType::LongEdge => {
                left.push(n);
                current = n;
            }
            _ => break,
        }
    }
    left.reverse();

    let mut right: Vec<NodeId> = Vec::new();
    let mut current = label_dummy;
    loop {
        let next = first_outgoing_target_node(graph, current);
        match next {
            Some(n) if graph.node(n).node_type == NodeType::LongEdge => {
                right.push(n);
                current = n;
            }
            _ => break,
        }
    }

    (left, right)
}

fn first_incoming_source_node(graph: &LGraph, node: NodeId) -> Option<NodeId> {
    for &port_id in &graph.node(node).ports {
        if let Some(&edge_id) = (&graph.port(port_id).incoming_edges).into_iter().next() {
            let src_port = graph.edge(edge_id).source;
            return Some(graph.port(src_port).owner);
        }
    }
    None
}

fn first_outgoing_target_node(graph: &LGraph, node: NodeId) -> Option<NodeId> {
    for &port_id in &graph.node(node).ports {
        if let Some(&edge_id) = (&graph.port(port_id).outgoing_edges).into_iter().next() {
            let tgt_port = graph.edge(edge_id).target;
            return Some(graph.port(tgt_port).owner);
        }
    }
    None
}

fn compute_layer_widths(graph: &LGraph, widths: &mut [f64]) {
    for (i, layer) in graph.layers.iter().enumerate() {
        let mut max = 0.0f64;
        for &node_id in &layer.nodes {
            if graph.node(node_id).node_type == NodeType::Normal {
                max = max.max(graph.node(node_id).size.x);
            }
        }
        widths[i] = max;
    }
}

fn process_infos(graph: &mut LGraph, mut infos: Vec<LabelDummyInfo>, layer_widths: &mut [f64]) {
    if infos.is_empty() {
        return;
    }
    if matches!(
        infos[0].placement_strategy,
        CenterEdgeLabelPlacementStrategy::SpaceEfficientLayer
    ) {
        compute_space_efficient_assignment(graph, infos, layer_widths);
        return;
    }

    for info in &mut infos {
        let target = match info.placement_strategy {
            CenterEdgeLabelPlacementStrategy::CenterLayer =>
                find_center_layer(graph, info, layer_widths),
            CenterEdgeLabelPlacementStrategy::MedianLayer => find_median_layer(info),
            CenterEdgeLabelPlacementStrategy::WidestLayer => find_widest_layer(info, layer_widths),
            CenterEdgeLabelPlacementStrategy::HeadLayer => {
                set_end_layer_alignment(graph, info);
                find_end_layer(graph, info, true)
            }
            CenterEdgeLabelPlacementStrategy::TailLayer => {
                set_end_layer_alignment(graph, info);
                find_end_layer(graph, info, false)
            }
            CenterEdgeLabelPlacementStrategy::SpaceEfficientLayer => {
                continue;
            }
        };
        assign_layer(graph, info, target, layer_widths);
        update_long_edge_before_label_dummy(graph, info.label_dummy);
    }
}

fn find_median_layer(info: &LabelDummyInfo) -> usize {
    let layers = info.total_dummy_count();
    let lower_median = (layers - 1) / 2;
    info.leftmost_layer_id + lower_median
}

fn find_widest_layer(info: &LabelDummyInfo, layer_widths: &[f64]) -> usize {
    let mut widest = info.leftmost_layer_id;
    for idx in (info.leftmost_layer_id + 1)..=info.rightmost_layer_id {
        if layer_widths[idx] > layer_widths[widest] {
            widest = idx;
        }
    }
    widest
}

fn find_center_layer(graph: &LGraph, info: &LabelDummyInfo, layer_widths: &[f64]) -> usize {
    let edge_node = graph.options.spacing.edge_node_between_layers * 2.0;
    let node_node = graph.options.spacing.node_node_between_layers;
    let min_between = edge_node.max(node_node);

    let count = info.total_dummy_count();
    let mut sums: Vec<f64> = vec![0.0; count];
    let mut current = -min_between;
    let mut idx = 0;
    for &left in &info.left_dummies {
        let lid = graph.node(left).layer.unwrap_or(0);
        current += layer_widths[lid] + min_between;
        sums[idx] = current;
        idx += 1;
    }
    {
        let lid = graph.node(info.label_dummy).layer.unwrap_or(0);
        current += layer_widths[lid] + min_between;
        sums[idx] = current;
        idx += 1;
    }
    for &right in &info.right_dummies {
        let lid = graph.node(right).layer.unwrap_or(0);
        current += layer_widths[lid] + min_between;
        sums[idx] = current;
        idx += 1;
    }

    let threshold = sums[count - 1] / 2.0;
    if let Some(i) = sums.iter().take(count).position(|&sum| sum >= threshold) {
        return info.leftmost_layer_id + i;
    }
    info.leftmost_layer_id + info.left_dummies.len()
}

fn find_end_layer(graph: &LGraph, info: &LabelDummyInfo, head_layer: bool) -> usize {
    let reversed = is_part_of_reversed_edge(graph, info.label_dummy);
    if (head_layer && !reversed) || (!head_layer && reversed) {
        info.rightmost_layer_id
    } else {
        info.leftmost_layer_id
    }
}

fn set_end_layer_alignment(graph: &mut LGraph, info: &LabelDummyInfo) {
    let is_head = matches!(info.placement_strategy, CenterEdgeLabelPlacementStrategy::HeadLayer);
    let reversed = is_part_of_reversed_edge(graph, info.label_dummy);
    let alignment = if (is_head && !reversed) || (!is_head && reversed) {
        Alignment::Right
    } else {
        Alignment::Left
    };
    graph.node_mut(info.label_dummy).properties.set(&ALIGNMENT, alignment);
}

fn is_part_of_reversed_edge(graph: &LGraph, label_dummy: NodeId) -> bool {
    for &port_id in &graph.node(label_dummy).ports {
        for &edge_id in &graph.port(port_id).incoming_edges {
            if graph.edge(edge_id).flags.contains(EdgeFlags::REVERSED) {
                return true;
            }
        }
        for &edge_id in &graph.port(port_id).outgoing_edges {
            if graph.edge(edge_id).flags.contains(EdgeFlags::REVERSED) {
                return true;
            }
        }
    }
    false
}

fn assign_layer(
    graph: &mut LGraph,
    info: &mut LabelDummyInfo,
    target_layer_idx: usize,
    layer_widths: &mut [f64],
) {
    let current_slot = info.leftmost_layer_id + info.left_dummies.len();
    if target_layer_idx != current_slot {
        let other = info.ith_dummy_node(target_layer_idx - info.leftmost_layer_id);
        swap_nodes(graph, info.label_dummy, other);

        // After the swap `left_dummies` / `right_dummies` describe layer-id
        // positions; rebuild them so subsequent work uses the correct chain.
        let (left, right) = gather_dummy_chain(graph, info.label_dummy);
        info.left_dummies = left;
        info.right_dummies = right;
        info.leftmost_layer_id = match info.left_dummies.first() {
            Some(&n) => graph.node(n).layer.unwrap_or(0),
            None => graph.node(info.label_dummy).layer.unwrap_or(0),
        };
        info.rightmost_layer_id = match info.right_dummies.last() {
            Some(&n) => graph.node(n).layer.unwrap_or(0),
            None => graph.node(info.label_dummy).layer.unwrap_or(0),
        };
    }

    let new_layer = graph.node(info.label_dummy).layer.unwrap_or(0);
    let width = graph.node(info.label_dummy).size.x;
    if new_layer < layer_widths.len() {
        layer_widths[new_layer] = layer_widths[new_layer].max(width);
    }
}

fn swap_nodes(graph: &mut LGraph, label_dummy: NodeId, other_dummy: NodeId) {
    let layer1 = graph.node(label_dummy).layer.unwrap_or(0);
    let layer2 = graph.node(other_dummy).layer.unwrap_or(0);

    let pos1 = graph.layers[layer1].nodes.iter().position(|&n| n == label_dummy).unwrap_or(0);
    let pos2 = graph.layers[layer2].nodes.iter().position(|&n| n == other_dummy).unwrap_or(0);

    let (input1, output1) = input_output_ports(graph, label_dummy);
    let (input2, output2) = input_output_ports(graph, other_dummy);

    let incoming1: Vec<EdgeId> = graph.port(input1).incoming_edges.to_vec();
    let outgoing1: Vec<EdgeId> = graph.port(output1).outgoing_edges.to_vec();
    let incoming2: Vec<EdgeId> = graph.port(input2).incoming_edges.to_vec();
    let outgoing2: Vec<EdgeId> = graph.port(output2).outgoing_edges.to_vec();

    // Move label_dummy into other_dummy's layer slot.
    graph.layers[layer1].nodes.remove(pos1);
    // Adjust pos2 if we removed from the same layer at an earlier index.
    let effective_pos2 = if layer1 == layer2 && pos1 < pos2 { pos2 - 1 } else { pos2 };
    graph.layers[layer2].nodes.remove(effective_pos2);

    graph.layers[layer2].nodes.insert(effective_pos2, label_dummy);
    graph.layers[layer1].nodes.insert(pos1, other_dummy);

    graph.node_mut(label_dummy).layer = Some(layer2).into();
    graph.node_mut(other_dummy).layer = Some(layer1).into();

    for edge in incoming2 {
        graph.reroute_edge_target(edge, input1);
    }
    for edge in outgoing2 {
        graph.reroute_edge_source(edge, output1);
    }
    for edge in incoming1 {
        graph.reroute_edge_target(edge, input2);
    }
    for edge in outgoing1 {
        graph.reroute_edge_source(edge, output2);
    }
}

fn input_output_ports(graph: &LGraph, node: NodeId) -> (PortId, PortId) {
    let mut input = None;
    let mut output = None;
    for &port_id in &graph.node(node).ports {
        match graph.port(port_id).side {
            PortSide::West => input = Some(port_id),
            PortSide::East => output = Some(port_id),
            _ => {}
        }
    }
    (
        input.expect("label/long-edge dummy missing west port"),
        output.expect("label/long-edge dummy missing east port"),
    )
}

fn update_long_edge_before_label_dummy(graph: &mut LGraph, label_dummy: NodeId) {
    let mut current = label_dummy;
    loop {
        let prev = first_incoming_source_node(graph, current);
        match prev {
            Some(n) if graph.node(n).node_type == NodeType::LongEdge => {
                graph.node_mut(n).properties.set(&LONG_EDGE_BEFORE_LABEL_DUMMY, true);
                current = n;
            }
            _ => break,
        }
    }
}

fn compute_space_efficient_assignment(
    graph: &mut LGraph,
    mut infos: Vec<LabelDummyInfo>,
    layer_widths: &mut [f64],
) {
    let mut remaining: Vec<LabelDummyInfo> = Vec::new();
    for mut info in infos.drain(..) {
        if info.leftmost_layer_id == info.rightmost_layer_id {
            let only_layer = info.leftmost_layer_id;
            assign_layer(graph, &mut info, only_layer, layer_widths);
            update_long_edge_before_label_dummy(graph, info.label_dummy);
            continue;
        }
        let width = graph.node(info.label_dummy).size.x;
        let mut placed = false;
        for layer_idx in info.leftmost_layer_id..=info.rightmost_layer_id {
            if layer_widths[layer_idx] >= width {
                assign_layer(graph, &mut info, layer_idx, layer_widths);
                update_long_edge_before_label_dummy(graph, info.label_dummy);
                placed = true;
                break;
            }
        }
        if !placed {
            remaining.push(info);
        }
    }

    if remaining.is_empty() {
        return;
    }

    remaining.sort_by(|a, b| {
        let av = graph.node(a.label_dummy).size.x;
        let bv = graph.node(b.label_dummy).size.x;
        bv.partial_cmp(&av).unwrap_or(std::cmp::Ordering::Equal)
    });

    let count = remaining.len();
    let widths_snapshot: Vec<f64> =
        remaining.iter().map(|i| graph.node(i.label_dummy).size.x).collect();

    for label_idx in 0..count {
        let target =
            find_potentially_widest_layer(&remaining, label_idx, &widths_snapshot, layer_widths);
        let info = &mut remaining[label_idx];
        assign_layer(graph, info, target, layer_widths);
        update_long_edge_before_label_dummy(graph, info.label_dummy);
    }
}

fn find_potentially_widest_layer(
    infos: &[LabelDummyInfo],
    label_index: usize,
    widths: &[f64],
    layer_widths: &[f64],
) -> usize {
    let info = &infos[label_index];
    let dummy_width = widths[label_index];

    let mut widest_idx = info.leftmost_layer_id;
    let mut widest_w = 0.0f64;

    for (layer, &layer_width) in layer_widths
        .iter()
        .enumerate()
        .take(info.rightmost_layer_id + 1)
        .skip(info.leftmost_layer_id)
    {
        if dummy_width <= layer_width {
            return layer;
        }

        let mut potential = layer_width;
        let mut largest_unassigned: Option<f64> = None;
        for other in (label_index + 1)..infos.len() {
            let curr = &infos[other];
            if curr.leftmost_layer_id <= layer && curr.rightmost_layer_id >= layer {
                largest_unassigned = Some(widths[other]);
            }
        }
        if let Some(w) = largest_unassigned {
            potential = potential.max(w);
        }

        if potential > widest_w {
            widest_idx = layer;
            widest_w = potential;
        }
    }

    widest_idx
}
