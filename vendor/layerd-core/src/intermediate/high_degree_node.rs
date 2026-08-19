use smallvec::SmallVec;

use crate::graph::{LGraph, LayerData, index::NodeId};

/// Handles nodes with very many connections by moving their tree-like
/// subtrees into dedicated layers.
///
/// For each high-degree node, identifies incoming and outgoing trees
/// (subtrees that connect to the high-degree node through a single edge).
/// Trees below a certain height threshold are moved to newly created
/// layers adjacent to the layer containing the high-degree node.
///
/// High-degree-node tree layering preprocessor.
pub fn process(graph: &mut LGraph) {
    let degree_threshold = graph.options.high_degree_threshold;
    let mut tree_height_threshold = graph.options.high_degree_tree_height;
    if tree_height_threshold == 0 {
        tree_height_threshold = usize::MAX;
    }

    // We process layers one by one, inserting new layers as needed
    let mut layer_idx = 0;
    while layer_idx < graph.layers.len() {
        let node_ids: SmallVec<NodeId, 32> =
            SmallVec::from_slice_copy(&graph.layers[layer_idx].nodes);

        let mut high_degree_nodes: Vec<(NodeId, HighDegreeInfo)> = Vec::new();

        for &node_id in &node_ids {
            let degree = node_degree(graph, node_id);
            if degree >= degree_threshold {
                let info =
                    calculate_tree_info(graph, node_id, degree_threshold, tree_height_threshold);
                high_degree_nodes.push((node_id, info));
            }
        }

        if high_degree_nodes.is_empty() {
            layer_idx += 1;
            continue;
        }

        // Determine max incoming and outgoing tree heights
        let inc_max =
            high_degree_nodes.iter().map(|(_, info)| info.inc_max_height).max().unwrap_or(0);
        let out_max =
            high_degree_nodes.iter().map(|(_, info)| info.out_max_height).max().unwrap_or(0);

        // Insert layers before the current layer for incoming trees
        let mut pre_layers: Vec<usize> = Vec::new();
        for _ in 0..inc_max {
            graph.layers.insert(layer_idx, LayerData::new());
            update_layer_assignments(graph, layer_idx);
            pre_layers.push(layer_idx);
            layer_idx += 1;
        }
        pre_layers.reverse();

        // Move incoming tree roots to pre-layers
        for (_, info) in &high_degree_nodes {
            for &root in &info.inc_roots {
                move_tree_incoming(graph, root, &pre_layers, 0);
            }
        }

        // Insert layers after the current layer for outgoing trees
        let mut after_layers: Vec<usize> = Vec::new();
        for i in 0..out_max {
            let insert_pos = layer_idx + 1 + i;
            graph.layers.insert(insert_pos, LayerData::new());
            update_layer_assignments(graph, insert_pos);
            after_layers.push(insert_pos);
        }

        // Move outgoing tree roots to after-layers
        for (_, info) in &high_degree_nodes {
            for &root in &info.out_roots {
                move_tree_outgoing(graph, root, &after_layers, 0);
            }
        }

        layer_idx += 1 + out_max;
    }

    // Remove empty layers
    let mut i = 0;
    while i < graph.layers.len() {
        if graph.layers[i].nodes.is_empty() {
            graph.layers.remove(i);
            update_layer_assignments(graph, i);
        } else {
            i += 1;
        }
    }
}

fn update_layer_assignments(graph: &mut LGraph, from: usize) {
    for idx in from..graph.layers.len() {
        let nodes: SmallVec<NodeId, 32> = SmallVec::from_slice_copy(&graph.layers[idx].nodes);
        for node_id in nodes {
            graph.node_mut(node_id).layer = Some(idx).into();
        }
    }
}

struct HighDegreeInfo {
    inc_max_height: usize,
    inc_roots: Vec<NodeId>,
    out_max_height: usize,
    out_roots: Vec<NodeId>,
}

fn node_degree(graph: &LGraph, node_id: NodeId) -> usize {
    graph.incoming_edges(node_id).count() + graph.outgoing_edges(node_id).count()
}

fn calculate_tree_info(
    graph: &LGraph,
    hdn: NodeId,
    degree_threshold: usize,
    tree_height_threshold: usize,
) -> HighDegreeInfo {
    let mut info = HighDegreeInfo {
        inc_max_height: 0,
        inc_roots: Vec::new(),
        out_max_height: 0,
        out_roots: Vec::new(),
    };

    // Check incoming trees
    for edge_id in graph.incoming_edges(hdn) {
        let src_port = graph.edge(edge_id).source;
        let src_node = graph.port(src_port).owner;
        if src_node == hdn {
            continue; // self-loop
        }

        // src_node must have single outgoing connection
        if !has_single_connection_outgoing(graph, src_node) {
            continue;
        }

        let height = tree_height_incoming(graph, src_node, degree_threshold, tree_height_threshold);
        if height < 0 {
            continue;
        }

        info.inc_max_height = info.inc_max_height.max(height as usize);
        info.inc_roots.push(src_node);
    }

    // Check outgoing trees
    for edge_id in graph.outgoing_edges(hdn) {
        let tgt_port = graph.edge(edge_id).target;
        let tgt_node = graph.port(tgt_port).owner;
        if tgt_node == hdn {
            continue; // self-loop
        }

        if !has_single_connection_incoming(graph, tgt_node) {
            continue;
        }

        let height = tree_height_outgoing(graph, tgt_node, degree_threshold, tree_height_threshold);
        if height < 0 {
            continue;
        }

        info.out_max_height = info.out_max_height.max(height as usize);
        info.out_roots.push(tgt_node);
    }

    info
}

fn has_single_connection_outgoing(graph: &LGraph, node: NodeId) -> bool {
    let mut target: Option<NodeId> = None;
    for edge_id in graph.outgoing_edges(node) {
        let tgt = graph.port(graph.edge(edge_id).target).owner;
        match target {
            None => target = Some(tgt),
            Some(existing) =>
                if existing != tgt {
                    return false;
                },
        }
    }
    true
}

fn has_single_connection_incoming(graph: &LGraph, node: NodeId) -> bool {
    let mut source: Option<NodeId> = None;
    for edge_id in graph.incoming_edges(node) {
        let src = graph.port(graph.edge(edge_id).source).owner;
        match source {
            None => source = Some(src),
            Some(existing) =>
                if existing != src {
                    return false;
                },
        }
    }
    true
}

fn tree_height_incoming(
    graph: &LGraph,
    root: NodeId,
    degree_threshold: usize,
    threshold: usize,
) -> i32 {
    tree_height_iterative(graph, root, degree_threshold, threshold, true)
}

fn tree_height_outgoing(
    graph: &LGraph,
    root: NodeId,
    degree_threshold: usize,
    threshold: usize,
) -> i32 {
    tree_height_iterative(graph, root, degree_threshold, threshold, false)
}

fn tree_height_iterative(
    graph: &LGraph,
    root: NodeId,
    degree_threshold: usize,
    threshold: usize,
    incoming: bool,
) -> i32 {
    let mut stack = vec![(root, false)];
    let mut heights: hashbrown::HashMap<NodeId, i32> = hashbrown::HashMap::new();
    while let Some((node, finish)) = stack.pop() {
        if heights.contains_key(&node) {
            continue;
        }
        if node_degree(graph, node) >= degree_threshold {
            return -1;
        }
        let single_connection = if incoming {
            has_single_connection_outgoing(graph, node)
        } else {
            has_single_connection_incoming(graph, node)
        };
        if !single_connection {
            return -1;
        }

        let children: Vec<NodeId> = if incoming {
            graph
                .incoming_edges(node)
                .map(|edge_id| graph.port(graph.edge(edge_id).source).owner)
                .collect()
        } else {
            graph
                .outgoing_edges(node)
                .map(|edge_id| graph.port(graph.edge(edge_id).target).owner)
                .collect()
        };
        if finish {
            let mut max_h = 0;
            for child in children {
                let Some(&h) = heights.get(&child) else {
                    return -1;
                };
                if h < 0 {
                    return -1;
                }
                max_h = max_h.max(h);
                if max_h >= threshold as i32 {
                    return -1;
                }
            }
            heights.insert(node, max_h + 1);
        } else if children.is_empty() {
            heights.insert(node, 1);
        } else {
            stack.push((node, true));
            for child in children.into_iter().rev() {
                if !heights.contains_key(&child) {
                    stack.push((child, false));
                }
            }
        }
    }
    heights.get(&root).copied().unwrap_or(-1)
}

fn move_tree_incoming(graph: &mut LGraph, root: NodeId, layers: &[usize], depth: usize) {
    move_tree_iterative(graph, root, layers, depth, true);
}

fn move_tree_outgoing(graph: &mut LGraph, root: NodeId, layers: &[usize], depth: usize) {
    move_tree_iterative(graph, root, layers, depth, false);
}

fn move_tree_iterative(
    graph: &mut LGraph,
    root: NodeId,
    layers: &[usize],
    depth: usize,
    incoming: bool,
) {
    let mut stack = vec![(root, depth)];
    while let Some((node, depth)) = stack.pop() {
        if depth >= layers.len() {
            continue;
        }

        if let Some(old_layer) = graph.node(node).layer.get() {
            graph.layers[old_layer].nodes.retain(|&n| n != node);
        }

        let target_layer = layers[depth];
        graph.node_mut(node).layer = Some(target_layer).into();
        graph.layers[target_layer].nodes.push(node);

        let children: Vec<NodeId> = if incoming {
            graph
                .incoming_edges(node)
                .map(|edge_id| graph.port(graph.edge(edge_id).source).owner)
                .collect()
        } else {
            graph
                .outgoing_edges(node)
                .map(|edge_id| graph.port(graph.edge(edge_id).target).owner)
                .collect()
        };
        for child in children.into_iter().rev() {
            stack.push((child, depth + 1));
        }
    }
}
