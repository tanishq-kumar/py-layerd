use crate::{
    graph::{
        LGraph, LayerData,
        index::{LabelId, NodeId},
        node::NodeType,
    },
    math::Vec2,
    options::enums::LayoutDirection,
    properties::internal::REPRESENTED_LABELS,
};

const MIN_WIDTH_EDGE_LABELS: f64 = 60.0;

/// Applies label-management sizing to center edge labels held by label dummy
/// nodes. Sets each label dummy's size to the space required to stack its
/// labels plus an `EDGE_THICKNESS + edge-label-spacing` vertical margin.
///
/// Label sizes are treated as already-final (no pluggable label manager
/// is wired up), so the space-required calculation simplifies to stacking
/// existing sizes with label-label spacing.
pub fn process(graph: &mut LGraph) {
    let edge_label_spacing = graph.options.spacing.edge_label;
    let label_label_spacing = graph.options.spacing.label_label;
    let vertical_layout = is_vertical(graph.options.direction);

    let layer_count = graph.layers.len();
    for layer_idx in 0..layer_count {
        let max_width = find_max_non_dummy_node_width(&graph.layers[layer_idx], graph)
            .max(MIN_WIDTH_EDGE_LABELS);

        let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
        for node_id in nodes {
            if graph.node(node_id).node_type != NodeType::Label {
                continue;
            }

            let edge_thickness = 1.0; // EDGE_THICKNESS default

            let label_ids: Vec<LabelId> = graph.node(node_id).properties.get(&REPRESENTED_LABELS);
            let labels: Vec<(f64, f64)> = label_ids
                .iter()
                .map(|id| {
                    let size = graph.label(*id).size;
                    (size.x, size.y)
                })
                .collect();

            let space =
                compute_required_space(&labels, max_width, label_label_spacing, vertical_layout);

            graph.node_mut(node_id).size.x = space.x;
            graph.node_mut(node_id).size.y = space.y + edge_thickness + edge_label_spacing;
        }
    }
}

fn is_vertical(direction: LayoutDirection) -> bool {
    matches!(direction, LayoutDirection::Up | LayoutDirection::Down)
}

/// Largest width among non-dummy normal nodes in a layer.
fn find_max_non_dummy_node_width(layer: &LayerData, graph: &LGraph) -> f64 {
    let mut max = 0.0f64;
    for &node_id in &layer.nodes {
        if graph.node(node_id).node_type == NodeType::Normal {
            max = max.max(graph.node(node_id).size.x);
        }
    }
    max
}

/// Stack all label sizes along the primary axis with `label_label_spacing`
/// between them, taking the max on the secondary axis. Returns the bounding
/// box required to place them. A vertical layout rotates this 90°.
fn compute_required_space(
    labels: &[(f64, f64)],
    _target_width: f64,
    label_label_spacing: f64,
    vertical_layout: bool,
) -> Vec2 {
    if labels.is_empty() {
        return Vec2::ZERO;
    }

    let mut req = Vec2::ZERO;
    for &(lx, ly) in labels {
        if vertical_layout {
            req.x += label_label_spacing + lx;
            req.y = req.y.max(ly);
        } else {
            req.x = req.x.max(lx);
            req.y += label_label_spacing + ly;
        }
    }

    if vertical_layout {
        req.x -= label_label_spacing;
    } else {
        req.y -= label_label_spacing;
    }

    req
}
