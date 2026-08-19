//! Alternating-layer unzipper for compact layouts.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph, LayerData,
        index::{EdgeId, NodeId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::PortConstraints,
    properties::internal::{
        LAYER_UNZIPPING_LAYER_SPLIT, LAYER_UNZIPPING_MINIMIZE_EDGE_LENGTH,
        LAYER_UNZIPPING_RESET_ON_LONG_EDGES,
    },
};

/// Divides nodes between layers to create a more compact final layout.
///
/// Reads `LAYER_UNZIPPING_LAYER_SPLIT` on each node to determine how many
/// sub-layers its containing layer should be split into. Nodes are distributed
/// round-robin across the sub-layers and long-edge dummies are inserted for
/// edges that cross the newly introduced sub-layer boundaries.
pub fn unzip(graph: &mut LGraph) {
    // Pass 1: determine per-layer split parameters and insert N-1 empty sub-layers.
    let mut groups: Vec<SplitGroup> = Vec::new();
    let mut i = 0;
    while i < graph.layers.len() {
        let n = layer_split_for_layer(graph, i);
        let reset = reset_on_long_edges_for_layer(graph, i);
        let minimize_edge_length = minimize_edge_length_for_layer(graph, i);

        // When minimize_edge_length is enabled, skip the split if the
        // width/height ratio crosses the heuristic threshold: splitting
        // becomes favourable only when `box_width / box_height < n / 4`,
        // where `n` is the layer's node count.
        if minimize_edge_length && skip_layer_by_minimize_edge_length(graph, i) {
            i += 1;
            continue;
        }

        if n > 1 && graph.layers[i].nodes.len() > n {
            let shift = n - 1;
            for offset in 0..shift {
                graph.layers.insert(i + 1 + offset, LayerData::new());
            }
            // Shift all downstream nodes' layer index by `shift`.
            let downstream_start = i + 1 + shift;
            for l in downstream_start..graph.layers.len() {
                let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[l].nodes);
                for node_id in nodes {
                    graph.node_mut(node_id).layer = Some(l).into();
                }
            }
            groups.push(SplitGroup { primary: i, n, reset_on_long_edges: reset });
            i += n;
        } else {
            i += 1;
        }
    }

    // Pass 2: shift nodes into their assigned sub-layers and create dummies.
    for group in &groups {
        process_split_group(graph, group.primary, group.n, group.reset_on_long_edges);
    }

    // Pass 3: remove placeholder dummies.
    let placeholder_nodes: Vec<NodeId> = graph
        .nodes_iter()
        .filter(|(_, n)| {
            matches!(n.node_type, NodeType::Placeholder | NodeType::NonShiftingPlaceholder)
        })
        .map(|(id, _)| id)
        .collect();
    for node_id in placeholder_nodes {
        graph.remove_node(node_id);
    }
}

struct SplitGroup {
    primary: usize,
    n: usize,
    reset_on_long_edges: bool,
}

/// Returns the smallest explicit `LAYER_UNZIPPING_LAYER_SPLIT` among the
/// layer's nodes, or the property default (1) if no node sets it.
fn layer_split_for_layer(graph: &LGraph, layer_idx: usize) -> usize {
    let mut min = usize::MAX;
    let mut found = false;
    for &node_id in &graph.layers[layer_idx].nodes {
        let props = &graph.node(node_id).properties;
        if props.has(&LAYER_UNZIPPING_LAYER_SPLIT) {
            found = true;
            let v = props.get(&LAYER_UNZIPPING_LAYER_SPLIT);
            if v < min {
                min = v;
            }
        }
    }
    if !found { 1 } else { min }
}

/// Returns false if any node explicitly sets
/// `LAYER_UNZIPPING_RESET_ON_LONG_EDGES` to false, otherwise true.
fn reset_on_long_edges_for_layer(graph: &LGraph, layer_idx: usize) -> bool {
    for &node_id in &graph.layers[layer_idx].nodes {
        let props = &graph.node(node_id).properties;
        if props.has(&LAYER_UNZIPPING_RESET_ON_LONG_EDGES)
            && !props.get(&LAYER_UNZIPPING_RESET_ON_LONG_EDGES)
        {
            return false;
        }
    }
    true
}

/// Returns true if any node in the layer explicitly enables
/// `LAYER_UNZIPPING_MINIMIZE_EDGE_LENGTH`.
fn minimize_edge_length_for_layer(graph: &LGraph, layer_idx: usize) -> bool {
    for &node_id in &graph.layers[layer_idx].nodes {
        let props = &graph.node(node_id).properties;
        if props.has(&LAYER_UNZIPPING_MINIMIZE_EDGE_LENGTH)
            && props.get(&LAYER_UNZIPPING_MINIMIZE_EDGE_LENGTH)
        {
            return true;
        }
    }
    false
}

/// Apply the layer width/height heuristic. Returns `true` when the layer
/// should be skipped (i.e. not split).
fn skip_layer_by_minimize_edge_length(graph: &LGraph, layer_idx: usize) -> bool {
    let nodes = &graph.layers[layer_idx].nodes;
    if nodes.is_empty() {
        return false;
    }
    let mut max_width = 0.0_f64;
    let mut sum_height = 0.0_f64;
    for &nid in nodes {
        let size = graph.node(nid).size;
        max_width = max_width.max(size.x);
        sum_height += size.y;
    }
    let count = nodes.len();
    let mut average_height = sum_height / count as f64;

    // max_width += max(2 * SPACING_EDGE_NODE_BETWEEN_LAYERS,
    //   max(n * SPACING_EDGE_EDGE_BETWEEN_LAYERS,
    //       SPACING_NODE_NODE_BETWEEN_LAYERS)).
    let s = &graph.options.spacing;
    let term_a = 2.0 * s.edge_node_between_layers;
    let term_b = (count as f64) * s.edge_edge_between_layers;
    let term_c = s.node_node_between_layers;
    max_width += term_a.max(term_b.max(term_c));

    // average_height += max(SPACING_NODE_NODE, SPACING_EDGE_NODE).
    average_height += s.node_node.max(s.edge_node);

    // Skip when max_width / average_height >= n / 4.
    if average_height <= 0.0 {
        return false;
    }
    max_width / average_height >= (count as f64) / 4.0
}

fn process_split_group(graph: &mut LGraph, primary: usize, n: usize, reset_on_long_edges: bool) {
    let nodes_in_layer = graph.layers[primary].nodes.len() as i32;
    let n_i32 = n as i32;

    let mut j: i32 = 0;
    let mut node_index: i32 = 0;
    let mut target_layer: i32 = 0;

    while j < nodes_in_layer {
        let node_id = graph.layers[primary].nodes[node_index as usize];
        let node_type = graph.node(node_id).node_type;

        if !matches!(node_type, NodeType::NonShiftingPlaceholder) {
            let wrapped = target_layer.rem_euclid(n_i32) as usize;
            let shifted = shift_node(graph, primary, n, wrapped, node_index as usize);
            node_index += shifted as i32;
        } else {
            j -= 1;
            target_layer -= 1;
        }

        if reset_on_long_edges && matches!(node_type, NodeType::LongEdge) {
            target_layer = -1;
        }

        j += 1;
        node_index += 1;
        target_layer += 1;
    }
}

/// Shifts a single node into the `target_layer`-th sub-layer and inserts
/// dummy nodes for any incoming or outgoing edges that must cross the newly
/// introduced sub-layer boundaries. Returns `edge_count - 1`, i.e. the
/// number of net nodes added to the primary sub-layer at or around
/// `node_index`, so the caller can advance past them.
fn shift_node(
    graph: &mut LGraph,
    primary: usize,
    n: usize,
    target_layer: usize,
    node_index: usize,
) -> usize {
    let node_id = graph.layers[primary].nodes[node_index];

    if target_layer > 0 {
        let dest = primary + target_layer;
        let dest_len = graph.layers[dest].nodes.len();
        graph.insert_node_in_layer(node_id, dest, dest_len);
    }

    let mut edge_count: usize = 0;

    // Incoming edges, iterated in reverse order.
    let incoming: SmallVec<EdgeId, 4> = {
        let mut v: SmallVec<EdgeId, 4> = graph.incoming_edges(node_id).collect();
        v.reverse();
        v
    };
    let has_incoming = !incoming.is_empty();
    for incoming_edge in incoming {
        let mut next_edge = incoming_edge;
        for layer_offset in 0..target_layer {
            let dummy = create_long_edge_dummy(graph);
            place_node(graph, dummy, primary + layer_offset, node_index + edge_count);
            next_edge = split_edge_on_dummy(graph, next_edge, dummy);
        }
        if target_layer > 0 {
            edge_count += 1;
        }
    }

    // No incoming edges: create unconnected PLACEHOLDER dummies to pad.
    if !has_incoming {
        for layer_offset in 0..target_layer {
            let dummy = create_placeholder(graph, NodeType::Placeholder);
            place_node(graph, dummy, primary + layer_offset, node_index + edge_count);
        }
        if target_layer > 0 {
            edge_count += 1;
        }
    }

    // Outgoing edges: first edge creates long-edge dummies in following sub-layers;
    // subsequent edges additionally drop NONSHIFTING_PLACEHOLDER pads before/at target.
    let outgoing: SmallVec<EdgeId, 4> = graph.outgoing_edges(node_id).collect();
    let mut extra_edge = false;
    for outgoing_edge in outgoing {
        let mut next_edge = outgoing_edge;
        for layer_offset in (target_layer + 1)..n {
            let dummy = create_long_edge_dummy(graph);
            append_node(graph, dummy, primary + layer_offset);
            next_edge = split_edge_on_dummy(graph, next_edge, dummy);
        }

        if extra_edge {
            for layer_offset in 0..=target_layer {
                let ph = create_placeholder(graph, NodeType::NonShiftingPlaceholder);
                place_node(graph, ph, primary + layer_offset, node_index + 1);
            }
            edge_count += 1;
        }

        extra_edge = true;
    }

    if edge_count > 0 { edge_count - 1 } else { 0 }
}

/// Inserts `node_id` into `layer_idx` at `position`, clamping to the layer's
/// current length if the requested position is past the end.
fn place_node(graph: &mut LGraph, node_id: NodeId, layer_idx: usize, position: usize) {
    let len = graph.layers[layer_idx].nodes.len();
    let actual = if position > len { len } else { position };
    graph.insert_node_in_layer(node_id, layer_idx, actual);
}

fn append_node(graph: &mut LGraph, node_id: NodeId, layer_idx: usize) {
    let len = graph.layers[layer_idx].nodes.len();
    graph.insert_node_in_layer(node_id, layer_idx, len);
}

/// Creates an unconnected dummy of the given placeholder type.
fn create_placeholder(graph: &mut LGraph, kind: NodeType) -> NodeId {
    let dummy = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy).node_type = kind;
    dummy
}

/// Creates a long-edge dummy with `FixedPos` port constraints.
fn create_long_edge_dummy(graph: &mut LGraph) -> NodeId {
    let dummy = graph.add_node(Vec2::ZERO);
    let n = graph.node_mut(dummy);
    n.node_type = NodeType::LongEdge;
    n.node_port_constraints = Some(PortConstraints::FixedPos);
    dummy
}

/// Splits `edge` on `dummy_node`: rewires `edge.target` to a new west port on
/// the dummy, creates a new east-side output edge to the original target, and
/// sets `LONG_EDGE_SOURCE` / `LONG_EDGE_TARGET` on the dummy. Returns the new
/// outgoing edge.
fn split_edge_on_dummy(graph: &mut LGraph, edge_id: EdgeId, dummy_node: NodeId) -> EdgeId {
    let dummy_in = graph.add_port(dummy_node, PortSide::West);
    let dummy_out = graph.add_port(dummy_node, PortSide::East);

    let old_target = graph.edge(edge_id).target;
    graph.port_mut(old_target).incoming_edges.retain(|e| *e != edge_id);
    let dummy_owner = graph.port_owner(dummy_in);
    let edge = graph.edge_mut(edge_id);
    edge.target = dummy_in;
    edge.target_owner = dummy_owner;
    graph.port_mut(dummy_in).incoming_edges.push(edge_id);

    let new_edge = graph.add_edge(dummy_out, old_target);

    let in_src_port = graph.edge(edge_id).source;
    let in_src_node = graph.port(in_src_port).owner;
    let in_src_type = graph.node(in_src_node).node_type;
    if matches!(in_src_type, NodeType::LongEdge) {
        let src = graph.node(in_src_node).long_edge_source;
        let tgt = graph.node(in_src_node).long_edge_target;
        graph.node_mut(dummy_node).long_edge_source = src;
        graph.node_mut(dummy_node).long_edge_target = tgt;
    } else {
        graph.node_mut(dummy_node).long_edge_source = Some(in_src_port);
        graph.node_mut(dummy_node).long_edge_target = Some(old_target);
    }

    new_edge
}
