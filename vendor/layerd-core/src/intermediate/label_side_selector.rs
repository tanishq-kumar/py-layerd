use std::collections::VecDeque;

use crate::{
    graph::{
        LGraph,
        edge::EdgeFlags,
        index::{EdgeId, LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{EdgeLabelPlacement, EdgeLabelSideSelection, LabelSide},
    properties::internal::{
        EDGE_LABEL_PLACEMENT, EDGE_LABELS_INLINE, EDGE_THICKNESS, LABEL_DUMMY_EDGE, LABEL_SIDE,
        REPRESENTED_LABELS,
    },
};

/// Chooses which side (above, below, or inline) each edge label sits on,
/// annotates the labels with `LABEL_SIDE`, and moves the ports of label
/// dummy nodes so they sit on the correct edge side.
pub fn select(graph: &mut LGraph) {
    match graph.options.edge_labels_side_selection {
        EdgeLabelSideSelection::AlwaysUp => same_side(graph, LabelSide::Above),
        EdgeLabelSideSelection::AlwaysDown => same_side(graph, LabelSide::Below),
        EdgeLabelSideSelection::DirectionUp => based_on_direction(graph, LabelSide::Above),
        EdgeLabelSideSelection::DirectionDown => based_on_direction(graph, LabelSide::Below),
        EdgeLabelSideSelection::SmartUp => smart(graph, LabelSide::Above),
        EdgeLabelSideSelection::SmartDown => smart(graph, LabelSide::Below),
    }
}

fn same_side(graph: &mut LGraph, side: LabelSide) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            if graph.node(node_id).node_type == NodeType::Label {
                apply_label_side_to_dummy(graph, node_id, side);
            }
            let out_edges: Vec<EdgeId> = collect_outgoing_edges(graph, node_id);
            for edge_id in out_edges {
                apply_label_side_to_edge(graph, edge_id, side);
            }
        }
    }
}

fn based_on_direction(graph: &mut LGraph, side_for_right: LabelSide) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            if graph.node(node_id).node_type == NodeType::Label {
                let points_right = does_label_dummy_point_right(graph, node_id);
                let side = if points_right { side_for_right } else { side_for_right.opposite() };
                apply_label_side_to_dummy(graph, node_id, side);
            }
            let out_edges: Vec<EdgeId> = collect_outgoing_edges(graph, node_id);
            for edge_id in out_edges {
                let points_right = does_edge_point_right(graph, edge_id);
                let side = if points_right { side_for_right } else { side_for_right.opposite() };
                apply_label_side_to_edge(graph, edge_id, side);
            }
        }
    }
}

fn smart(graph: &mut LGraph, default_side: LabelSide) {
    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let mut queue: VecDeque<NodeId> = VecDeque::new();
        let mut top_group = true;
        let mut label_dummies_in_queue: usize = 0;

        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            let node_type = graph.node(node_id).node_type;
            match node_type {
                NodeType::Label => {
                    label_dummies_in_queue += 1;
                    queue.push_back(node_id);
                }
                NodeType::LongEdge => {
                    queue.push_back(node_id);
                }
                NodeType::Normal => {
                    smart_for_regular_node(graph, node_id, default_side);
                    if !queue.is_empty() {
                        smart_for_dummy_run(
                            graph,
                            &mut queue,
                            label_dummies_in_queue,
                            top_group,
                            false,
                            default_side,
                        );
                    }
                    top_group = false;
                    label_dummies_in_queue = 0;
                }
                _ => {
                    if !queue.is_empty() {
                        smart_for_dummy_run(
                            graph,
                            &mut queue,
                            label_dummies_in_queue,
                            top_group,
                            false,
                            default_side,
                        );
                    }
                    top_group = false;
                    label_dummies_in_queue = 0;
                }
            }
        }

        if !queue.is_empty() {
            smart_for_dummy_run(
                graph,
                &mut queue,
                label_dummies_in_queue,
                top_group,
                true,
                default_side,
            );
        }
    }
}

fn smart_for_dummy_run(
    graph: &mut LGraph,
    queue: &mut VecDeque<NodeId>,
    label_dummy_count: usize,
    top_group: bool,
    bottom_group: bool,
    default_side: LabelSide,
) {
    assert!(!queue.is_empty());

    let first_is_label = queue.front().map(|&n| graph.node(n).node_type) == Some(NodeType::Label);
    let last_is_label = queue.back().map(|&n| graph.node(n).node_type) == Some(NodeType::Label);

    if top_group && (!bottom_group || queue.len() > 1) && label_dummy_count == 1 && first_is_label {
        if let Some(&n) = queue.front() {
            apply_label_side_to_dummy(graph, n, LabelSide::Above);
        }
    } else if bottom_group
        && (!top_group || queue.len() > 1)
        && label_dummy_count == 1
        && last_is_label
    {
        if let Some(&n) = queue.back() {
            apply_label_side_to_dummy(graph, n, LabelSide::Below);
        }
    } else if queue.len() == 2 {
        if let (Some(first), Some(second)) = (queue.pop_front(), queue.pop_front()) {
            apply_label_side_to_dummy(graph, first, LabelSide::Above);
            apply_label_side_to_dummy(graph, second, LabelSide::Below);
        }
    } else {
        apply_for_dummy_run_with_simple_loops(graph, queue, default_side);
    }

    queue.clear();
}

fn apply_for_dummy_run_with_simple_loops(
    graph: &mut LGraph,
    queue: &VecDeque<NodeId>,
    default_side: LabelSide,
) {
    let mut run: Vec<NodeId> = Vec::new();
    let mut prev_source: Option<NodeId> = None;
    let mut prev_target: Option<NodeId> = None;

    for &current in queue.iter() {
        let source = long_edge_end_node(graph, current, true);
        let target = long_edge_end_node(graph, current, false);

        if prev_source != source || prev_target != target {
            apply_run(graph, &mut run, default_side);
            prev_source = source;
            prev_target = target;
        }
        run.push(current);
    }

    apply_run(graph, &mut run, default_side);
}

fn apply_run(graph: &mut LGraph, run: &mut Vec<NodeId>, default_side: LabelSide) {
    if run.is_empty() {
        return;
    }
    if run.len() == 2 {
        apply_label_side_to_dummy(graph, run[0], LabelSide::Above);
        apply_label_side_to_dummy(graph, run[1], LabelSide::Below);
    } else {
        for &n in run.iter() {
            apply_label_side_to_dummy(graph, n, default_side);
        }
    }
    run.clear();
}

fn smart_for_regular_node(graph: &mut LGraph, node: NodeId, default_side: LabelSide) {
    let ports: Vec<PortId> = graph.node(node).ports.to_vec();

    let mut per_side: Vec<(PortSide, Vec<Vec<LabelId>>)> = Vec::new();
    let mut current_side: Option<PortSide> = None;

    for port_id in ports {
        let side = graph.port(port_id).side;
        if Some(side) != current_side {
            if !per_side.is_empty() {
                let (_, _) = per_side.last().expect("non-empty by check");
            }
            per_side.push((side, Vec::new()));
            current_side = Some(side);
        }
        if let Some(labels) = gather_end_labels(graph, port_id) {
            per_side.last_mut().expect("pushed above").1.push(labels);
        }
    }

    for (side, queue) in per_side {
        handle_port_end_labels(graph, &queue, side, default_side);
    }
}

fn handle_port_end_labels(
    graph: &mut LGraph,
    queue: &[Vec<LabelId>],
    port_side: PortSide,
    default_side: LabelSide,
) {
    if queue.is_empty() {
        return;
    }

    if queue.len() == 2 {
        let (first_side, second_side) = match port_side {
            PortSide::North | PortSide::East => (LabelSide::Above, LabelSide::Below),
            _ => (LabelSide::Below, LabelSide::Above),
        };
        for &label in &queue[0] {
            graph.label_mut(label).properties.set(&LABEL_SIDE, first_side);
        }
        for &label in &queue[1] {
            graph.label_mut(label).properties.set(&LABEL_SIDE, second_side);
        }
    } else {
        for labels in queue {
            for &label in labels {
                graph.label_mut(label).properties.set(&LABEL_SIDE, default_side);
            }
        }
    }
}

fn apply_label_side_to_dummy(graph: &mut LGraph, node: NodeId, side: LabelSide) {
    if graph.node(node).node_type != NodeType::Label {
        return;
    }
    let effective_side = if is_inline_edge_label(graph, node) { LabelSide::Inline } else { side };
    graph.node_mut(node).properties.set(&LABEL_SIDE, effective_side);

    if effective_side == LabelSide::Below {
        return;
    }

    let origin_edge = match graph.node(node).properties.get(&LABEL_DUMMY_EDGE) {
        Some(e) => e,
        None => return,
    };
    let thickness = graph.edge(origin_edge).properties.get(&EDGE_THICKNESS);

    let mut dummy_size_y = graph.node(node).size.y;
    let port_pos = match effective_side {
        LabelSide::Above => dummy_size_y - (thickness / 2.0).ceil(),
        LabelSide::Inline => {
            let spacing = graph.options.spacing.edge_label;
            let pos = (dummy_size_y - spacing - thickness).ceil() / 2.0;
            dummy_size_y -= spacing;
            dummy_size_y -= thickness;
            pos
        }
        _ => 0.0,
    };

    if effective_side == LabelSide::Inline {
        graph.node_mut(node).size.y = dummy_size_y;
    }

    let ports: Vec<PortId> = graph.node(node).ports.to_vec();
    for port_id in ports {
        graph.port_mut(port_id).position.y = port_pos;
    }
}

fn apply_label_side_to_edge(graph: &mut LGraph, edge: EdgeId, side: LabelSide) {
    let labels: Vec<LabelId> = graph.edge(edge).labels.to_vec();
    for label in labels {
        graph.label_mut(label).properties.set(&LABEL_SIDE, side);
    }
}

fn collect_outgoing_edges(graph: &LGraph, node: NodeId) -> Vec<EdgeId> {
    let mut edges = Vec::new();
    for &port_id in &graph.node(node).ports {
        edges.extend(graph.port(port_id).outgoing_edges.iter().copied());
    }
    edges
}

fn does_edge_point_right(graph: &LGraph, edge: EdgeId) -> bool {
    !graph.edge(edge).flags.contains(EdgeFlags::REVERSED)
}

fn does_label_dummy_point_right(graph: &LGraph, node: NodeId) -> bool {
    let mut any_right = false;
    for &port_id in &graph.node(node).ports {
        for &e in &graph.port(port_id).incoming_edges {
            if does_edge_point_right(graph, e) {
                any_right = true;
            }
        }
        for &e in &graph.port(port_id).outgoing_edges {
            if does_edge_point_right(graph, e) {
                any_right = true;
            }
        }
    }
    any_right
}

fn long_edge_end_node(graph: &LGraph, dummy: NodeId, use_source: bool) -> Option<NodeId> {
    let port = if use_source {
        graph.node(dummy).long_edge_source
    } else {
        graph.node(dummy).long_edge_target
    };
    port.map(|port| graph.port(port).owner)
}

fn is_inline_edge_label(graph: &LGraph, dummy: NodeId) -> bool {
    if graph.node(dummy).node_type != NodeType::Label {
        return false;
    }
    let label_ids: Vec<LabelId> = graph.node(dummy).properties.get(&REPRESENTED_LABELS);
    label_ids.iter().all(|&id| graph.label(id).properties.get(&EDGE_LABELS_INLINE))
}

/// Collects the HEAD and TAIL labels attached to edges incident on `port`.
/// Returns `None` if the port has no incident edges at all (the
/// "no-incident-edge-thickness" sentinel).
fn gather_end_labels(graph: &LGraph, port: PortId) -> Option<Vec<LabelId>> {
    let mut any_edge = false;
    let mut labels = Vec::new();

    for &edge_id in &graph.port(port).outgoing_edges {
        any_edge = true;
        for &label_id in &graph.edge(edge_id).labels {
            if graph.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT)
                == EdgeLabelPlacement::Tail
            {
                labels.push(label_id);
            }
        }
    }
    for &edge_id in &graph.port(port).incoming_edges {
        any_edge = true;
        for &label_id in &graph.edge(edge_id).labels {
            if graph.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT)
                == EdgeLabelPlacement::Head
            {
                labels.push(label_id);
            }
        }
    }

    if any_edge { Some(labels) } else { None }
}
