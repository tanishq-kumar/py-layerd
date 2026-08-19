use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{EdgeId, LabelId, NodeId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::{
        EdgeLabelPlacement, EdgeRoutingStrategy, LabelSide, LayoutDirection, PortConstraints,
    },
    properties::internal::{
        EDGE_LABEL_PLACEMENT, EDGE_LABELS_INLINE, EDGE_THICKNESS, END_LABEL_EDGE, JUNCTION_POINTS,
        LABEL_DUMMY_EDGE, LABEL_SIDE, REPRESENTED_LABELS,
    },
};

/// Inserts dummy nodes for edges with CENTER labels, reserving space for them.
///
/// For every non-self-loop edge that carries at least one `CENTER`-placed
/// label, create a dummy of type `Label`, set its width/height from the
/// CENTER labels' sizes (stacked along the layout direction), and split
/// the edge through it (copy properties, clear JUNCTION_POINTS,
/// port.y = floor(thickness/2)). Moved labels are transferred to the
/// dummy node's `REPRESENTED_LABELS` slot and removed from the edge's
/// own label list.
pub fn insert(graph: &mut LGraph) {
    let edge_label_spacing = graph.options.spacing.edge_label;
    let label_label_spacing = graph.options.spacing.label_label;
    let vertical_layout =
        matches!(graph.options.direction, LayoutDirection::Up | LayoutDirection::Down);

    // Collect edges that need processing (have CENTER labels, are not self-loops).
    let mut edges_to_process: Vec<EdgeId> = Vec::new();
    let node_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    for &nid in &node_ids {
        for &pid in &graph.node(nid).ports {
            for &eid in &graph.port(pid).outgoing_edges {
                if edge_needs_processing(graph, eid) {
                    edges_to_process.push(eid);
                }
            }
        }
    }

    let mut new_dummies: Vec<NodeId> = Vec::new();
    for edge_id in edges_to_process {
        let dummy = create_label_dummy(
            graph,
            edge_id,
            edge_label_spacing,
            label_label_spacing,
            vertical_layout,
        );
        new_dummies.push(dummy);
    }
    graph.layerless_nodes.extend(new_dummies);
}

fn edge_needs_processing(graph: &LGraph, edge_id: EdgeId) -> bool {
    let edge = graph.edge(edge_id);
    let src_node = graph.port(edge.source).owner;
    let tgt_node = graph.port(edge.target).owner;
    if src_node == tgt_node {
        return false;
    }
    edge.labels.iter().any(|&lid| {
        graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Center
    })
}

/// Creates a label dummy node for `edge_id`, moves CENTER labels onto it, and
/// splits the edge. Returns the dummy node id.
fn create_label_dummy(
    graph: &mut LGraph,
    edge_id: EdgeId,
    edge_label_spacing: f64,
    label_label_spacing: f64,
    vertical_layout: bool,
) -> NodeId {
    // Negative EDGE_THICKNESS clamps to 0 and is written back to the edge.
    let mut thickness = graph.edge(edge_id).properties.get(&EDGE_THICKNESS);
    if thickness < 0.0 {
        thickness = 0.0;
        graph.edge_mut(edge_id).properties.set(&EDGE_THICKNESS, 0.0);
    }

    // Snapshot endpoints before any rewire.
    let source_port = graph.edge(edge_id).source;
    let target_port = graph.edge(edge_id).target;

    let dummy_node = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy_node).node_type = NodeType::Label;
    graph.layerless_nodes.retain(|&n| n != dummy_node);

    // Set ORIGIN, REPRESENTED_LABELS (empty — filled below),
    // PORT_CONSTRAINTS=FIXED_POS/TARGET.
    graph.node_mut(dummy_node).origin_edge = Some(edge_id);
    graph
        .node_mut(dummy_node)
        .properties
        .set(&REPRESENTED_LABELS, Vec::<LabelId>::new());
    graph.node_mut(dummy_node).node_port_constraints = Some(PortConstraints::FixedPos);
    graph.node_mut(dummy_node).long_edge_source = Some(source_port);
    graph.node_mut(dummy_node).long_edge_target = Some(target_port);
    graph.node_mut(dummy_node).properties.set(&LABEL_DUMMY_EDGE, Some(edge_id));

    // Split the edge through the dummy. Height = thickness; ports sit at
    // floor(thickness / 2).
    graph.node_mut(dummy_node).size.y = thickness;
    let port_pos_y = (thickness / 2.0).floor();

    let dummy_in = graph.add_port(dummy_node, PortSide::West);
    let dummy_out = graph.add_port(dummy_node, PortSide::East);
    graph.port_mut(dummy_in).position.y = port_pos_y;
    graph.port_mut(dummy_out).position.y = port_pos_y;

    // Clone edge's cold properties + flags for the new dummy edge.
    let cloned_props = graph.edge(edge_id).properties.clone();
    let cloned_flags = graph.edge(edge_id).flags;

    graph.port_mut(target_port).incoming_edges.retain(|e| *e != edge_id);
    let dummy_owner = graph.port_owner(dummy_in);
    let edge = graph.edge_mut(edge_id);
    edge.target = dummy_in;
    edge.target_owner = dummy_owner;
    graph.port_mut(dummy_in).incoming_edges.push(edge_id);

    let new_edge = graph.add_edge(dummy_out, target_port);
    graph.edge_mut(new_edge).properties = cloned_props;
    graph.edge_mut(new_edge).flags = cloned_flags;
    graph
        .edge_mut(new_edge)
        .properties
        .set(&JUNCTION_POINTS, smallvec::SmallVec::new());
    move_head_labels(graph, edge_id, new_edge);

    // Move CENTER labels from the old edge onto the dummy's REPRESENTED_LABELS,
    // accumulating the dummy's x/y extent according to layout direction.
    let old_labels: SmallVec<LabelId, 3> = graph.edge(edge_id).labels.iter().copied().collect();
    let mut represented: Vec<LabelId> = Vec::new();
    let mut remaining: SmallVec<LabelId, 2> = SmallVec::new();
    let mut dummy_w = 0.0_f64;
    // Long-edge splitter initialises dummy height to `EDGE_THICKNESS`;
    // `LabelDummyInserter` then mutates the live size below.
    let mut dummy_h = thickness;
    for lid in old_labels {
        let placement = graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT);
        if placement == EdgeLabelPlacement::Center {
            let size = graph.label(lid).size;
            if vertical_layout {
                dummy_w += size.x + label_label_spacing;
                dummy_h = dummy_h.max(size.y);
            } else {
                dummy_w = dummy_w.max(size.x);
                dummy_h += size.y + label_label_spacing;
            }
            represented.push(lid);
        } else {
            remaining.push(lid);
        }
    }
    graph.edge_mut(edge_id).labels = remaining;

    // Post-accumulation adjustment.
    if vertical_layout {
        dummy_w -= label_label_spacing;
        dummy_h += edge_label_spacing + thickness;
    } else {
        dummy_h += edge_label_spacing - label_label_spacing + thickness;
    }
    graph.node_mut(dummy_node).size.x = dummy_w;
    graph.node_mut(dummy_node).size.y = dummy_h;

    graph.node_mut(dummy_node).properties.set(&REPRESENTED_LABELS, represented);

    dummy_node
}

fn move_head_labels(graph: &mut LGraph, old_edge: EdgeId, new_edge: EdgeId) {
    let old_labels: SmallVec<LabelId, 3> = graph.edge(old_edge).labels.iter().copied().collect();
    let mut kept: SmallVec<LabelId, 2> = SmallVec::new();
    let mut moved: SmallVec<LabelId, 2> = SmallVec::new();
    for label_id in old_labels {
        if graph.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT) == EdgeLabelPlacement::Head {
            if graph.label(label_id).properties.get(&END_LABEL_EDGE).is_none() {
                graph.label_mut(label_id).properties.set(&END_LABEL_EDGE, Some(old_edge));
            }
            moved.push(label_id);
        } else {
            kept.push(label_id);
        }
    }
    graph.edge_mut(old_edge).labels = kept;
    for label_id in moved {
        graph.edge_mut(new_edge).labels.push(label_id);
    }
}

/// Removes label dummy nodes, places labels at the dummy position, and
/// reconnects the original edges.
/// Runs after P5.
pub fn remove(graph: &mut LGraph) {
    // Find all Label dummy nodes in layers
    let mut dummies: Vec<NodeId> = Vec::new();
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            if graph.node(nid).node_type == NodeType::Label {
                dummies.push(nid);
            }
        }
    }

    for dummy_node in &dummies {
        let dummy = *dummy_node;
        let dummy_pos = graph.node(dummy).position;
        let dummy_size = graph.node(dummy).size;

        // Labels were moved to the dummy's REPRESENTED_LABELS during insertion;
        // place them at the dummy position and migrate them back onto the
        // restored edge. Compute `labels_below_edge`, adjust start y by
        // `thickness + edge_label_spacing` if below, compute label space
        // with inline-edge-label shortening, then stack labels using the
        // spacing between successive entries.
        let original_edge_id = graph.node(dummy).properties.get(&LABEL_DUMMY_EDGE);
        let represented_labels: Vec<LabelId> =
            graph.node(dummy).properties.get(&REPRESENTED_LABELS);
        if let Some(orig_eid) = original_edge_id {
            let edge_label_spacing = graph.options.spacing.edge_label;
            let label_label_spacing = graph.options.spacing.label_label;
            let thickness = graph.edge(orig_eid).properties.get(&EDGE_THICKNESS);
            let labels_below_edge =
                graph.node(dummy).properties.get(&LABEL_SIDE) == LabelSide::Below;

            let mut curr_pos = dummy_pos;
            if labels_below_edge {
                curr_pos.y += thickness + edge_label_spacing;
            }

            let all_inline = !represented_labels.is_empty()
                && represented_labels
                    .iter()
                    .all(|&lid| graph.label(lid).properties.get(&EDGE_LABELS_INLINE));
            let label_space_x = dummy_size.x;
            let label_space_y =
                dummy_size.y + if all_inline { 0.0 } else { -thickness - edge_label_spacing };

            let direction = graph.options.direction;
            let is_vertical = matches!(direction, LayoutDirection::Up | LayoutDirection::Down);

            if is_vertical {
                // Vertical-layout label placement: labels stack horizontally
                // (advance x by label.size.x + spacing); y picks among
                // (a) vertical centering inside `label_space_y` for inline
                // labels, (b) top-aligned for `left_aligned == true`
                // (i.e. labels above the edge), or (c) bottom-aligned
                // otherwise. UP reverses the label order so the visible
                // sequence matches DOWN.
                let order: Vec<LabelId> = if matches!(direction, LayoutDirection::Up) {
                    represented_labels.iter().rev().copied().collect()
                } else {
                    represented_labels.clone()
                };
                let left_aligned = !labels_below_edge;
                let mut label_x = curr_pos.x;
                for (i, &lid) in order.iter().enumerate() {
                    let label_w = graph.label(lid).size.x;
                    let label_h = graph.label(lid).size.y;
                    let label_y = if all_inline {
                        curr_pos.y + (label_space_y - label_h) / 2.0
                    } else if left_aligned {
                        curr_pos.y
                    } else {
                        curr_pos.y + label_space_y - label_h
                    };
                    graph.label_mut(lid).position = Vec2::new(label_x, label_y);
                    label_x += label_w;
                    if i + 1 < order.len() {
                        label_x += label_label_spacing;
                    }
                }
            } else {
                let _ = label_space_x;
                let _ = label_space_y;
                let mut label_y = curr_pos.y;
                for (i, &lid) in represented_labels.iter().enumerate() {
                    let label_w = graph.label(lid).size.x;
                    graph.label_mut(lid).position =
                        Vec2::new(curr_pos.x + (dummy_size.x - label_w) / 2.0, label_y);
                    label_y += graph.label(lid).size.y;
                    if i + 1 < represented_labels.len() {
                        label_y += label_label_spacing;
                    }
                }
            }
            // Return the labels to the restored edge.
            for lid in &represented_labels {
                graph.edge_mut(orig_eid).labels.push(*lid);
            }
        }

        let add_unnecessary = graph.options.edge_routing == EdgeRoutingStrategy::Polyline;
        super::long_edge_joiner::join_at(graph, dummy, add_unnecessary);
    }

    // Remove dummy nodes from layers
    for layer_idx in 0..graph.layers.len() {
        let kept: Vec<_> = graph.layers[layer_idx]
            .nodes
            .iter()
            .copied()
            .filter(|&n| graph.node(n).node_type != NodeType::Label)
            .collect();
        graph.layers[layer_idx].nodes = kept;
    }
}
