use crate::{
    graph::{LGraph, index::NodeId, node::NodeType, port::PortSide},
    math::Vec2,
    options::enums::{
        Alignment, DirectionCongruency, InLayerConstraint, LayerConstraint, LayoutDirection,
        NodeLabelPlacement,
    },
    properties::internal::{
        ALIGNMENT, EXT_PORT_SIDE, IN_LAYER_CONSTRAINT, JUNCTION_POINTS, LAYER_CONSTRAINT,
        NODE_LABEL_PLACEMENT, NODE_SIZE_MINIMUM, PORT_INDEX, POSITION,
    },
};

/// Applies the input-direction transform that runs before P1.
///
/// The matching `postprocess` call applies the opposite transform after P5.
pub fn preprocess(graph: &mut LGraph) {
    let direction = graph.options.direction;
    let congruency = graph.options.direction_congruency;
    let nodes = collect_all_nodes(graph);

    match congruency {
        DirectionCongruency::ReadingDirection => match direction {
            LayoutDirection::Right | LayoutDirection::Undefined => {}
            LayoutDirection::Left => {
                mirror_x(graph, &nodes);
            }
            LayoutDirection::Down => {
                transpose(graph, &nodes);
            }
            LayoutDirection::Up => {
                mirror_y(graph, &nodes);
                transpose(graph, &nodes);
            }
        },
        DirectionCongruency::Rotation => match direction {
            LayoutDirection::Right | LayoutDirection::Undefined => {}
            LayoutDirection::Left => {
                // Reflect across both axes.
                mirror_x(graph, &nodes);
                mirror_y(graph, &nodes);
            }
            LayoutDirection::Down => {
                // Rotate 90 degrees counterclockwise.
                mirror_x(graph, &nodes);
                transpose(graph, &nodes);
            }
            LayoutDirection::Up => {
                // Rotate 90 degrees clockwise.
                transpose(graph, &nodes);
                mirror_x(graph, &nodes);
            }
        },
    }
}

/// Applies the internal left-to-right transform after P5.
pub fn postprocess(graph: &mut LGraph) {
    let direction = graph.options.direction;
    let congruency = graph.options.direction_congruency;
    let nodes = collect_all_nodes(graph);

    match congruency {
        DirectionCongruency::ReadingDirection => match direction {
            LayoutDirection::Right | LayoutDirection::Undefined => {}
            LayoutDirection::Left => {
                mirror_x(graph, &nodes);
            }
            LayoutDirection::Down => {
                transpose(graph, &nodes);
            }
            LayoutDirection::Up => {
                transpose(graph, &nodes);
                mirror_y(graph, &nodes);
            }
        },
        DirectionCongruency::Rotation => match direction {
            LayoutDirection::Right | LayoutDirection::Undefined => {}
            LayoutDirection::Left => {
                // Same pair, just in the same order as preprocess (self-inverse).
                mirror_x(graph, &nodes);
                mirror_y(graph, &nodes);
            }
            LayoutDirection::Down => {
                // Rotate 90 degrees clockwise.
                transpose(graph, &nodes);
                mirror_x(graph, &nodes);
            }
            LayoutDirection::Up => {
                // Rotate 90 degrees counterclockwise.
                mirror_x(graph, &nodes);
                transpose(graph, &nodes);
            }
        },
    }
}

/// Collect all node IDs (both layerless and in layers).
fn collect_all_nodes(graph: &LGraph) -> Vec<NodeId> {
    let mut nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    for layer in &graph.layers {
        nodes.extend(&layer.nodes);
    }
    nodes
}

/// Reflect an edge's `JUNCTION_POINTS` x coordinates around `offset`.
fn mirror_junction_points_x(graph: &mut LGraph, edge_id: crate::graph::index::EdgeId, offset: f64) {
    let mut jps = graph.edge(edge_id).properties.get(&JUNCTION_POINTS);
    if jps.is_empty() {
        return;
    }
    for jp in jps.iter_mut() {
        jp.x = offset - jp.x;
    }
    graph.edge_mut(edge_id).properties.set(&JUNCTION_POINTS, jps);
}

fn mirror_junction_points_y(graph: &mut LGraph, edge_id: crate::graph::index::EdgeId, offset: f64) {
    let mut jps = graph.edge(edge_id).properties.get(&JUNCTION_POINTS);
    if jps.is_empty() {
        return;
    }
    for jp in jps.iter_mut() {
        jp.y = offset - jp.y;
    }
    graph.edge_mut(edge_id).properties.set(&JUNCTION_POINTS, jps);
}

fn transpose_junction_points(graph: &mut LGraph, edge_id: crate::graph::index::EdgeId) {
    let mut jps = graph.edge(edge_id).properties.get(&JUNCTION_POINTS);
    if jps.is_empty() {
        return;
    }
    for jp in jps.iter_mut() {
        std::mem::swap(&mut jp.x, &mut jp.y);
    }
    graph.edge_mut(edge_id).properties.set(&JUNCTION_POINTS, jps);
}

/// Reflect all coordinates across the vertical axis.
fn mirror_x(graph: &mut LGraph, nodes: &[NodeId]) {
    // Compute offset: the width of the graph or the maximum x extent
    let offset = if graph.size.x > 0.0 {
        graph.size.x - graph.offset.x
    } else {
        let mut max_x = 0.0_f64;
        for &nid in nodes {
            let n = graph.node(nid);
            max_x = max_x.max(n.position.x + n.size.x + n.margin.right);
        }
        max_x
    } - graph.offset.x;

    for &nid in nodes {
        let node_w = graph.node(nid).size.x;
        graph.node_mut(nid).position.x = offset - node_w - graph.node(nid).position.x;

        // Mirror port sides and positions
        let ports: Vec<_> = graph.node(nid).ports.to_vec();
        let node_w = graph.node(nid).size.x;
        for &pid in &ports {
            let port_w = graph.port(pid).size.x;
            graph.port_mut(pid).position.x = node_w - port_w - graph.port(pid).position.x;
            graph.port_mut(pid).anchor.x = graph.port(pid).size.x - graph.port(pid).anchor.x;
            let new_side = mirror_port_side_x(graph.port(pid).side);
            graph.port_mut(pid).side = new_side;

            // Reverse PORT_INDEX.
            let port_idx = graph.port(pid).properties.get(&PORT_INDEX);
            if port_idx != 0 {
                graph.port_mut(pid).properties.set(&PORT_INDEX, -port_idx);
            }

            // Mirror outgoing edge bend points, junction points, and labels.
            let outgoing: Vec<_> = graph.port(pid).outgoing_edges.to_vec();
            for &eid in &outgoing {
                for bp in &mut graph.edge_mut(eid).bend_points {
                    bp.x = offset - bp.x;
                }
                mirror_junction_points_x(graph, eid, offset);
                let label_ids: Vec<_> = graph.edge(eid).labels.to_vec();
                for &lid in &label_ids {
                    let lw = graph.label(lid).size.x;
                    graph.label_mut(lid).position.x = offset - lw - graph.label(lid).position.x;
                }
            }

            // Mirror port labels
            let label_ids: Vec<_> = graph.port(pid).labels.to_vec();
            for &lid in &label_ids {
                let lw = graph.label(lid).size.x;
                graph.label_mut(lid).position.x = port_w - lw - graph.label(lid).position.x;
            }
        }

        // External port dummy? Mirror EXT_PORT_SIDE and layer constraint.
        if graph.node(nid).node_type == NodeType::ExternalPort {
            let ext_side = graph.node(nid).properties.get(&EXT_PORT_SIDE);
            graph.node_mut(nid).properties.set(&EXT_PORT_SIDE, mirror_port_side_x(ext_side));
            let lc = graph.node(nid).properties.get(&LAYER_CONSTRAINT);
            graph
                .node_mut(nid)
                .properties
                .set(&LAYER_CONSTRAINT, mirror_layer_constraint_x(lc));
        }

        // Mirror node labels and their placement.
        mirror_node_label_placement_x(graph, nid);
        let label_ids: Vec<_> = graph.node(nid).labels.to_vec();
        let node_w = graph.node(nid).size.x;
        for &lid in &label_ids {
            let lw = graph.label(lid).size.x;
            graph.label_mut(lid).position.x = node_w - lw - graph.label(lid).position.x;
        }
    }

    // Mirror padding
    let old_left = graph.padding.left;
    let old_right = graph.padding.right;
    graph.padding.left = old_right;
    graph.padding.right = old_left;

    // Mirror NODE_LABELS_PADDING.left ↔ .right alongside the layout swap.
    mirror_node_labels_padding_x(graph);
}

fn mirror_node_labels_padding_x(graph: &mut LGraph) {
    use crate::properties::internal::NODE_LABELS_PADDING;
    let mut p = graph.properties.get(&NODE_LABELS_PADDING);
    std::mem::swap(&mut p.left, &mut p.right);
    graph.properties.set(&NODE_LABELS_PADDING, p);
}

/// Mirror a `LayerConstraint` along the X axis.
fn mirror_layer_constraint_x(lc: LayerConstraint) -> LayerConstraint {
    match lc {
        LayerConstraint::First => LayerConstraint::Last,
        LayerConstraint::FirstSeparate => LayerConstraint::LastSeparate,
        LayerConstraint::Last => LayerConstraint::First,
        LayerConstraint::LastSeparate => LayerConstraint::FirstSeparate,
        other => other,
    }
}

/// Mirror node-label placement along the X axis. Only swaps the H_LEFT /
/// H_RIGHT bits.
fn mirror_node_label_placement_x(graph: &mut LGraph, nid: NodeId) {
    let placement = graph.node(nid).properties.get(&NODE_LABEL_PLACEMENT);
    if placement.is_empty() {
        return;
    }
    let mut new_placement = placement;
    if new_placement.contains(NodeLabelPlacement::H_LEFT) {
        new_placement.remove(NodeLabelPlacement::H_LEFT);
        new_placement.insert(NodeLabelPlacement::H_RIGHT);
    } else if new_placement.contains(NodeLabelPlacement::H_RIGHT) {
        new_placement.remove(NodeLabelPlacement::H_RIGHT);
        new_placement.insert(NodeLabelPlacement::H_LEFT);
    }
    graph.node_mut(nid).properties.set(&NODE_LABEL_PLACEMENT, new_placement);
}

/// Reflect all coordinates across the horizontal axis.
fn mirror_y(graph: &mut LGraph, nodes: &[NodeId]) {
    let offset = if graph.size.y > 0.0 {
        graph.size.y - graph.offset.y
    } else {
        let mut max_y = 0.0_f64;
        for &nid in nodes {
            let n = graph.node(nid);
            max_y = max_y.max(n.position.y + n.size.y + n.margin.bottom);
        }
        max_y
    } - graph.offset.y;

    for &nid in nodes {
        let node_h = graph.node(nid).size.y;
        graph.node_mut(nid).position.y = offset - node_h - graph.node(nid).position.y;

        let ports: Vec<_> = graph.node(nid).ports.to_vec();
        let node_h = graph.node(nid).size.y;
        for &pid in &ports {
            let port_h = graph.port(pid).size.y;
            graph.port_mut(pid).position.y = node_h - port_h - graph.port(pid).position.y;
            graph.port_mut(pid).anchor.y = graph.port(pid).size.y - graph.port(pid).anchor.y;
            let new_side = mirror_port_side_y(graph.port(pid).side);
            graph.port_mut(pid).side = new_side;

            // Reverse PORT_INDEX.
            let port_idx = graph.port(pid).properties.get(&PORT_INDEX);
            if port_idx != 0 {
                graph.port_mut(pid).properties.set(&PORT_INDEX, -port_idx);
            }

            let outgoing: Vec<_> = graph.port(pid).outgoing_edges.to_vec();
            for &eid in &outgoing {
                for bp in &mut graph.edge_mut(eid).bend_points {
                    bp.y = offset - bp.y;
                }
                mirror_junction_points_y(graph, eid, offset);
                let label_ids: Vec<_> = graph.edge(eid).labels.to_vec();
                for &lid in &label_ids {
                    let lh = graph.label(lid).size.y;
                    graph.label_mut(lid).position.y = offset - lh - graph.label(lid).position.y;
                }
            }

            let label_ids: Vec<_> = graph.port(pid).labels.to_vec();
            for &lid in &label_ids {
                let lh = graph.label(lid).size.y;
                graph.label_mut(lid).position.y = port_h - lh - graph.label(lid).position.y;
            }
        }

        // External port dummy? Mirror EXT_PORT_SIDE (Y axis) and in-layer constraint.
        if graph.node(nid).node_type == NodeType::ExternalPort {
            let ext_side = graph.node(nid).properties.get(&EXT_PORT_SIDE);
            graph.node_mut(nid).properties.set(&EXT_PORT_SIDE, mirror_port_side_y(ext_side));
            let ilc = graph.node(nid).properties.get(&IN_LAYER_CONSTRAINT);
            graph
                .node_mut(nid)
                .properties
                .set(&IN_LAYER_CONSTRAINT, mirror_in_layer_constraint_y(ilc));
        }

        // Mirror ALIGNMENT: TOP <-> BOTTOM.
        let align = graph.node(nid).properties.get(&ALIGNMENT);
        if align == Alignment::Top {
            graph.node_mut(nid).properties.set(&ALIGNMENT, Alignment::Bottom);
        } else if align == Alignment::Bottom {
            graph.node_mut(nid).properties.set(&ALIGNMENT, Alignment::Top);
        }

        // Mirror node labels / placement V bits.
        mirror_node_label_placement_y(graph, nid);
        let label_ids: Vec<_> = graph.node(nid).labels.to_vec();
        let node_h = graph.node(nid).size.y;
        for &lid in &label_ids {
            let lh = graph.label(lid).size.y;
            graph.label_mut(lid).position.y = node_h - lh - graph.label(lid).position.y;
        }
    }

    // Mirror padding
    let old_top = graph.padding.top;
    let old_bottom = graph.padding.bottom;
    graph.padding.top = old_bottom;
    graph.padding.bottom = old_top;

    // Also flip NODE_LABELS_PADDING.top <-> .bottom.
    mirror_node_labels_padding_y(graph);
}

fn mirror_node_labels_padding_y(graph: &mut LGraph) {
    use crate::properties::internal::NODE_LABELS_PADDING;
    let mut p = graph.properties.get(&NODE_LABELS_PADDING);
    std::mem::swap(&mut p.top, &mut p.bottom);
    graph.properties.set(&NODE_LABELS_PADDING, p);
}

/// Mirror an `InLayerConstraint` along the Y axis.
fn mirror_in_layer_constraint_y(ilc: InLayerConstraint) -> InLayerConstraint {
    match ilc {
        InLayerConstraint::Top => InLayerConstraint::Bottom,
        InLayerConstraint::Bottom => InLayerConstraint::Top,
        other => other,
    }
}

/// Mirror node-label placement along the Y axis. Only swaps V_TOP / V_BOTTOM.
fn mirror_node_label_placement_y(graph: &mut LGraph, nid: NodeId) {
    let placement = graph.node(nid).properties.get(&NODE_LABEL_PLACEMENT);
    if placement.is_empty() {
        return;
    }
    let mut new_placement = placement;
    if new_placement.contains(NodeLabelPlacement::V_TOP) {
        new_placement.remove(NodeLabelPlacement::V_TOP);
        new_placement.insert(NodeLabelPlacement::V_BOTTOM);
    } else if new_placement.contains(NodeLabelPlacement::V_BOTTOM) {
        new_placement.remove(NodeLabelPlacement::V_BOTTOM);
        new_placement.insert(NodeLabelPlacement::V_TOP);
    }
    graph.node_mut(nid).properties.set(&NODE_LABEL_PLACEMENT, new_placement);
}

/// Transpose all coordinates (swap x and y).
fn transpose(graph: &mut LGraph, nodes: &[NodeId]) {
    for &nid in nodes {
        transpose_vec2(&mut graph.node_mut(nid).position);
        transpose_vec2(&mut graph.node_mut(nid).size);
        // Transpose padding alongside position / size.
        let pad = *graph.node(nid).padding;
        graph.node_mut(nid).padding.top = pad.left;
        graph.node_mut(nid).padding.bottom = pad.right;
        graph.node_mut(nid).padding.left = pad.top;
        graph.node_mut(nid).padding.right = pad.bottom;

        // Transpose node-level properties: MIN_SIZE / ALIGNMENT / POSITION.
        transpose_node_properties(graph, nid);
        // Transpose node label placement bitflags.
        transpose_node_label_placement(graph, nid);

        let ports: Vec<_> = graph.node(nid).ports.to_vec();
        for &pid in &ports {
            transpose_vec2(&mut graph.port_mut(pid).position);
            transpose_vec2(&mut graph.port_mut(pid).anchor);
            transpose_vec2(&mut graph.port_mut(pid).size);
            let new_side = transpose_port_side(graph.port(pid).side);
            graph.port_mut(pid).side = new_side;

            // Reverse PORT_INDEX.
            let port_idx = graph.port(pid).properties.get(&PORT_INDEX);
            if port_idx != 0 {
                graph.port_mut(pid).properties.set(&PORT_INDEX, -port_idx);
            }

            let outgoing: Vec<_> = graph.port(pid).outgoing_edges.to_vec();
            for &eid in &outgoing {
                for bp in &mut graph.edge_mut(eid).bend_points {
                    transpose_vec2(bp);
                }
                transpose_junction_points(graph, eid);
                let label_ids: Vec<_> = graph.edge(eid).labels.to_vec();
                for &lid in &label_ids {
                    transpose_vec2(&mut graph.label_mut(lid).position);
                    transpose_vec2(&mut graph.label_mut(lid).size);
                }
            }

            let label_ids: Vec<_> = graph.port(pid).labels.to_vec();
            for &lid in &label_ids {
                transpose_vec2(&mut graph.label_mut(lid).position);
                transpose_vec2(&mut graph.label_mut(lid).size);
            }
        }

        // External port dummy? Transpose EXT_PORT_SIDE and layer constraint.
        if graph.node(nid).node_type == NodeType::ExternalPort {
            let ext_side = graph.node(nid).properties.get(&EXT_PORT_SIDE);
            graph
                .node_mut(nid)
                .properties
                .set(&EXT_PORT_SIDE, transpose_port_side(ext_side));
            transpose_layer_constraint(graph, nid);
        }

        let label_ids: Vec<_> = graph.node(nid).labels.to_vec();
        for &lid in &label_ids {
            transpose_vec2(&mut graph.label_mut(lid).position);
            transpose_vec2(&mut graph.label_mut(lid).size);
        }
    }

    // Transpose graph-level properties
    transpose_vec2(&mut graph.offset);
    transpose_vec2(&mut graph.size);
    let p = graph.padding;
    graph.padding.top = p.left;
    graph.padding.bottom = p.right;
    graph.padding.left = p.top;
    graph.padding.right = p.bottom;

    // Also transpose the edge-label side selection + graph-level
    // NODE_LABELS_PADDING so rotated layouts keep labels on the intended
    // side of the edges and respect the node's padding budget.
    // (Diff items 15.3 and 15.4.)
    transpose_edge_label_side_selection(graph);
    transpose_graph_node_labels_padding(graph);
}

/// Swap Up ↔ Down on the graph-level `EdgeLabelSideSelection` option.
fn transpose_edge_label_side_selection(graph: &mut LGraph) {
    graph.options.edge_labels_side_selection = graph.options.edge_labels_side_selection.transpose();
}

/// Transpose `NODE_LABELS_PADDING` on the graph. Swap the `top ↔ left` and
/// `bottom ↔ right` pairs (same mapping as the graph's own padding
/// transpose a few lines above).
fn transpose_graph_node_labels_padding(graph: &mut LGraph) {
    use crate::properties::internal::NODE_LABELS_PADDING;
    let p = graph.properties.get(&NODE_LABELS_PADDING);
    let swapped =
        crate::math::Padding { top: p.left, right: p.bottom, bottom: p.right, left: p.top };
    graph.properties.set(&NODE_LABELS_PADDING, swapped);
}

/// Transpose node properties: swap MIN_SIZE.x/.y, map ALIGNMENT between
/// axis-aware variants, transpose POSITION.
fn transpose_node_properties(graph: &mut LGraph, nid: NodeId) {
    // NODE_SIZE_MINIMUM.
    let mut min_size = graph.node(nid).properties.get(&NODE_SIZE_MINIMUM);
    std::mem::swap(&mut min_size.x, &mut min_size.y);
    graph.node_mut(nid).properties.set(&NODE_SIZE_MINIMUM, min_size);

    // ALIGNMENT: LEFT↔TOP, RIGHT↔BOTTOM.
    let align = graph.node(nid).properties.get(&ALIGNMENT);
    let new_align = match align {
        Alignment::Left => Alignment::Top,
        Alignment::Right => Alignment::Bottom,
        Alignment::Top => Alignment::Left,
        Alignment::Bottom => Alignment::Right,
        other => other,
    };
    graph.node_mut(nid).properties.set(&ALIGNMENT, new_align);

    // Swap the x/y components of `POSITION` so processors that order nodes
    // by the user-supplied interactive POSITION (e.g. semi-interactive
    // crossmin) see the rotated coordinate after an UP/DOWN transform.
    if let Some(pos) = graph.node(nid).properties.get(&POSITION) {
        graph
            .node_mut(nid)
            .properties
            .set(&POSITION, Some(crate::math::Vec2 { x: pos.y, y: pos.x }));
    }
}

/// Transpose node-label placement: map placement bits between horizontal and
/// vertical axes.
fn transpose_node_label_placement(graph: &mut LGraph, nid: NodeId) {
    let new_node_placement =
        transpose_placement_value(graph.node(nid).properties.get(&NODE_LABEL_PLACEMENT));
    if let Some(p) = new_node_placement {
        graph.node_mut(nid).properties.set(&NODE_LABEL_PLACEMENT, p);
    }
    // Per-label `NODE_LABELS_PLACEMENT` is honoured by the cell layout, so
    // those per-label placements need the same transpose as the node-level
    // value to keep DOWN/UP layouts coming back in the original orientation.
    let label_ids: smallvec::SmallVec<crate::graph::index::LabelId, 2> =
        graph.node(nid).labels.iter().copied().collect();
    for lid in label_ids {
        if !graph.label(lid).properties.has(&NODE_LABEL_PLACEMENT) {
            continue;
        }
        let placement = graph.label(lid).properties.get(&NODE_LABEL_PLACEMENT);
        if let Some(p) = transpose_placement_value(placement) {
            graph.label_mut(lid).properties.set(&NODE_LABEL_PLACEMENT, p);
        }
    }
}

fn transpose_placement_value(placement: NodeLabelPlacement) -> Option<NodeLabelPlacement> {
    if placement.is_empty() {
        return None;
    }
    let mut new_placement = NodeLabelPlacement::empty();
    if placement.contains(NodeLabelPlacement::INSIDE) {
        new_placement.insert(NodeLabelPlacement::INSIDE);
    } else {
        new_placement.insert(NodeLabelPlacement::OUTSIDE);
    }
    if !placement.contains(NodeLabelPlacement::H_PRIORITY) {
        new_placement.insert(NodeLabelPlacement::H_PRIORITY);
    }
    if placement.contains(NodeLabelPlacement::H_LEFT) {
        new_placement.insert(NodeLabelPlacement::V_TOP);
    } else if placement.contains(NodeLabelPlacement::H_CENTER) {
        new_placement.insert(NodeLabelPlacement::V_CENTER);
    } else if placement.contains(NodeLabelPlacement::H_RIGHT) {
        new_placement.insert(NodeLabelPlacement::V_BOTTOM);
    }
    if placement.contains(NodeLabelPlacement::V_TOP) {
        new_placement.insert(NodeLabelPlacement::H_LEFT);
    } else if placement.contains(NodeLabelPlacement::V_CENTER) {
        new_placement.insert(NodeLabelPlacement::H_CENTER);
    } else if placement.contains(NodeLabelPlacement::V_BOTTOM) {
        new_placement.insert(NodeLabelPlacement::H_RIGHT);
    }
    Some(new_placement)
}

/// Transpose layer constraints: convert between FIRST_SEPARATE/LAST_SEPARATE
/// layer constraints and TOP/BOTTOM in-layer constraints.
fn transpose_layer_constraint(graph: &mut LGraph, nid: NodeId) {
    let lc = graph.node(nid).properties.get(&LAYER_CONSTRAINT);
    let ilc = graph.node(nid).properties.get(&IN_LAYER_CONSTRAINT);
    if lc == LayerConstraint::FirstSeparate {
        graph.node_mut(nid).properties.set(&LAYER_CONSTRAINT, LayerConstraint::None);
        graph.node_mut(nid).properties.set(&IN_LAYER_CONSTRAINT, InLayerConstraint::Top);
    } else if lc == LayerConstraint::LastSeparate {
        graph.node_mut(nid).properties.set(&LAYER_CONSTRAINT, LayerConstraint::None);
        graph
            .node_mut(nid)
            .properties
            .set(&IN_LAYER_CONSTRAINT, InLayerConstraint::Bottom);
    } else if ilc == InLayerConstraint::Top {
        graph
            .node_mut(nid)
            .properties
            .set(&LAYER_CONSTRAINT, LayerConstraint::FirstSeparate);
        graph
            .node_mut(nid)
            .properties
            .set(&IN_LAYER_CONSTRAINT, InLayerConstraint::None);
    } else if ilc == InLayerConstraint::Bottom {
        graph
            .node_mut(nid)
            .properties
            .set(&LAYER_CONSTRAINT, LayerConstraint::LastSeparate);
        graph
            .node_mut(nid)
            .properties
            .set(&IN_LAYER_CONSTRAINT, InLayerConstraint::None);
    }
}

fn transpose_vec2(v: &mut Vec2) {
    std::mem::swap(&mut v.x, &mut v.y);
}

fn mirror_port_side_x(side: PortSide) -> PortSide {
    match side {
        PortSide::East => PortSide::West,
        PortSide::West => PortSide::East,
        other => other,
    }
}

fn mirror_port_side_y(side: PortSide) -> PortSide {
    match side {
        PortSide::North => PortSide::South,
        PortSide::South => PortSide::North,
        other => other,
    }
}

fn transpose_port_side(side: PortSide) -> PortSide {
    match side {
        PortSide::North => PortSide::West,
        PortSide::West => PortSide::North,
        PortSide::South => PortSide::East,
        PortSide::East => PortSide::South,
        PortSide::Undefined => PortSide::Undefined,
    }
}
