use std::collections::VecDeque;

use crate::{
    graph::{LGraph, LayerData, index::NodeId, node::NodeType},
    options::enums::NodePromotionStrategy,
    properties::internal::MODEL_ORDER,
};

/// Promotes nodes to earlier layers to reduce the number of long-edge dummy nodes.
///
/// After layering, checks whether moving a node one layer earlier would reduce
/// the total count of dummy nodes needed. If yes, promotes the node (and
/// recursively promotes predecessors that end up in the same layer).
///
/// Implements a simplified version of the Nikolov heuristic. Also dispatches
/// to the model-order path for the `ModelOrderLeftToRight` /
/// `ModelOrderRightToLeft` strategies.
pub fn promote_nodes(graph: &mut LGraph) {
    match graph.options.node_promotion {
        NodePromotionStrategy::None => return,
        NodePromotionStrategy::ModelOrderLeftToRight => {
            promote_nodes_model_order(graph, true);
            return;
        }
        NodePromotionStrategy::ModelOrderRightToLeft => {
            promote_nodes_model_order(graph, false);
            return;
        }
        _ => {}
    }

    if graph.layers.len() <= 1 {
        return;
    }

    // Dispatch to the specific Nikolov flavour.
    //
    // Stop-criterion semantics:
    //   - Plain Nikolov / NoBoundary / Improved: stop only when an outer
    //     iteration produces zero promotions.
    //   - NODECOUNT_PERCENTAGE: stop when iteration counter reaches
    //     `ceil(layers * pct / 100)`.
    //   - DUMMYNODE_PERCENTAGE: stop when cumulative reducedDummies reach
    //     `ceil(dummy_count * pct / 100)`.
    //
    // Width-guard semantics:
    //   - NIKOLOV / NIKOLOV_PIXEL: reject promotions whose new layer width
    //     exceeds the accepted `maxWidth` baseline.
    //   - NoBoundary / Improved first pass: do not reject — only the
    //     after-pass `newMaxWidth > maxWidth` check decides whether the
    //     improved fallback runs.
    match graph.options.node_promotion {
        NodePromotionStrategy::NikolovImproved => {
            let original_max_width = max_layer_width_in_nodes(graph);
            run_nikolov(graph, NikolovOpts::default());
            let new_max_width = max_layer_width_in_nodes(graph);
            if new_max_width > original_max_width {
                run_nikolov(
                    graph,
                    NikolovOpts { nikolov_width_guard: true, ..NikolovOpts::default() },
                );
            }
            return;
        }
        NodePromotionStrategy::NikolovImprovedPixel => {
            let original_max_pixel = max_layer_width_in_pixels(graph);
            run_nikolov(graph, NikolovOpts { pixel_weighting: true, ..NikolovOpts::default() });
            let new_max_pixel = max_layer_width_in_pixels(graph);
            if new_max_pixel > original_max_pixel {
                run_nikolov(
                    graph,
                    NikolovOpts {
                        pixel_weighting: true,
                        nikolov_width_guard: true,
                        ..NikolovOpts::default()
                    },
                );
            }
            return;
        }
        NodePromotionStrategy::NikolovPixel => {
            run_nikolov(
                graph,
                NikolovOpts {
                    pixel_weighting: true,
                    nikolov_width_guard: true,
                    ..NikolovOpts::default()
                },
            );
            return;
        }
        NodePromotionStrategy::Nikolov => {
            run_nikolov(graph, NikolovOpts { nikolov_width_guard: true, ..NikolovOpts::default() });
            return;
        }
        NodePromotionStrategy::NoBoundary => {
            run_nikolov(graph, NikolovOpts::default());
            return;
        }
        NodePromotionStrategy::NodecountPercentage => {
            let n_layers = graph.layers.len() as u32;
            let pct = graph.options.node_promotion_max_iterations;
            // promote_until_n = ceil(layers.length * promote_until / 100).
            let promote_until_n = ((n_layers as f64 * pct as f64) / 100.0).ceil() as usize;
            run_nikolov(
                graph,
                NikolovOpts { iteration_cap: Some(promote_until_n), ..NikolovOpts::default() },
            );
            return;
        }
        NodePromotionStrategy::DummynodePercentage => {
            let dummy_count = count_dummy_nodes(graph) as u32;
            let pct = graph.options.node_promotion_max_iterations;
            // promote_until_d = ceil(dummy_node_count * promote_until / 100).
            let promote_until_d = ((dummy_count as f64 * pct as f64) / 100.0).ceil() as usize;
            run_nikolov(
                graph,
                NikolovOpts { reduced_dummy_cap: Some(promote_until_d), ..NikolovOpts::default() },
            );
            return;
        }
        _ => {}
    }

    // Fall through: other strategies reuse the default Nikolov path.
    run_nikolov(graph, NikolovOpts::default());
}

/// Per-call configuration for `run_nikolov`. Captures stop-criterion plus the
/// `promoteNode` width guard variants.
#[derive(Default)]
struct NikolovOpts {
    /// Approximate node width with `node.size.y + SPACING_NODE_NODE` and
    /// dummies with `SPACING_EDGE_NODE_BETWEEN_LAYERS`.
    pixel_weighting: bool,
    /// When set, reject any single promotion that would make either the
    /// source or target layer wider than the accepted `maxWidth` baseline.
    /// `NIKOLOV` and `NIKOLOV_PIXEL` apply this guard; `NO_BOUNDARY` skips it.
    nikolov_width_guard: bool,
    /// When `Some(n)` (NODECOUNT_PERCENTAGE), stop when the outer iteration
    /// counter reaches `n`.
    iteration_cap: Option<usize>,
    /// When `Some(n)` (DUMMYNODE_PERCENTAGE), stop when the cumulative
    /// `reducedDummies` counter reaches `n`.
    reduced_dummy_cap: Option<usize>,
}

/// Parameterised Nikolov promotion. Factored out of `promote_nodes` so every
/// flavour can share it.
fn run_nikolov(graph: &mut LGraph, opts: NikolovOpts) {
    let pixel_weighting = opts.pixel_weighting;
    let nikolov_width_guard = opts.nikolov_width_guard;
    let iteration_cap = opts.iteration_cap;
    let reduced_dummy_cap = opts.reduced_dummy_cap;
    if graph.layers.len() <= 1 {
        return;
    }

    // Collect all nodes with their current layer assignments and degree info
    let mut all_nodes: Vec<NodeId> = Vec::new();
    let mut node_layers: Vec<usize> = Vec::new();
    let mut node_in_degree: Vec<i32> = Vec::new();
    let mut node_out_degree: Vec<i32> = Vec::new();
    let mut node_id_map: hashbrown::HashMap<NodeId, usize> = hashbrown::HashMap::new();

    let max_height = graph.layers.len();
    // Reversed IDs: layer 0 (leftmost) gets ID max_height-1.

    for layer_idx in 0..max_height {
        let rev_id = max_height - 1 - layer_idx;
        for &node_id in &graph.layers[layer_idx].nodes {
            let idx = all_nodes.len();
            all_nodes.push(node_id);
            node_layers.push(rev_id);
            node_id_map.insert(node_id, idx);

            let in_deg = graph.incoming_edges(node_id).count() as i32;
            let out_deg = graph.outgoing_edges(node_id).count() as i32;
            node_in_degree.push(in_deg);
            node_out_degree.push(out_deg);
        }
    }

    // Collect nodes with incoming edges
    let nodes_with_incoming: Vec<usize> =
        (0..all_nodes.len()).filter(|&i| node_in_degree[i] > 0).collect();

    // Precompute initial per-layer widths so the NIKOLOV / NIKOLOV_PIXEL
    // width guard can reject promotions that grow the widest layer past
    // the accepted baseline. `compute_layer_widths_with_dummies` accumulates
    // a `dummyBaggage` running total per layer.
    let (initial_max_width, initial_per_layer_width, node_size_affix, dummy_size) =
        compute_layer_widths_with_dummies(graph, pixel_weighting);
    let _ = &initial_per_layer_width;

    let mut layers = node_layers.clone();
    let mut current_height = max_height;
    let mut iterations: usize = 0;
    let mut reduced_dummies: usize = 0;
    let max_iterations = if graph.options.node_promotion_max_iterations > 0 {
        graph.options.node_promotion_max_iterations as usize
    } else {
        usize::MAX
    };

    loop {
        // The outer loop terminates when `promotions == 0` OR when the
        // configured stop predicate returns false (NODECOUNT_PERCENTAGE
        // caps the iteration counter; DUMMYNODE_PERCENTAGE caps cumulative
        // `reducedDummies`). The `node_promotion_max_iterations` hard cap
        // is a safety net for the non-percentage paths.
        if iterations >= max_iterations {
            break;
        }
        if let Some(cap) = iteration_cap
            && iterations >= cap
        {
            break;
        }
        if let Some(cap) = reduced_dummy_cap
            && reduced_dummies >= cap
        {
            break;
        }

        let mut promotions = 0usize;

        for &node_idx in &nodes_with_incoming {
            if let Some(cap) = reduced_dummy_cap
                && reduced_dummies >= cap
            {
                break;
            }

            // Take backup BEFORE each individual promotion so a revert
            // only undoes this node's changes, not earlier successes.
            let backup = layers.clone();
            let height_backup = current_height;

            let diff = try_promote(
                &all_nodes,
                &mut layers,
                &node_in_degree,
                &node_out_degree,
                &node_id_map,
                graph,
                node_idx,
                &mut current_height,
            );

            let mut keep = diff < 0;
            if keep && nikolov_width_guard {
                // NIKOLOV / NIKOLOV_PIXEL reject any promotion whose new
                // layer width exceeds the accepted baseline. NoBoundary /
                // Improved skip the check; the after-pass
                // `new_max_width > max_width` comparison decides whether the
                // improved fallback runs.
                let new_max = current_max_layer_width(
                    &all_nodes,
                    &layers,
                    current_height,
                    max_height,
                    pixel_weighting,
                    &*graph,
                    &node_id_map,
                    node_size_affix,
                    dummy_size,
                );
                if new_max > initial_max_width {
                    keep = false;
                }
            }

            if keep {
                promotions += 1;
                if diff < 0 {
                    reduced_dummies = reduced_dummies.saturating_add((-diff) as usize);
                }
            } else {
                layers.copy_from_slice(&backup);
                current_height = height_backup;
            }
        }

        iterations += 1;
        if promotions == 0 {
            break;
        }
    }

    apply_layering(graph, &all_nodes, &layers, current_height, max_height);
}

/// Current max layer width across the candidate layering. Full per-iteration
/// recomputation: sum per-layer node contributions (either unit count or
/// pixel-weighted `size.y + node_size_affix`) then add one dummy per edge
/// segment strictly between `src_rev_layer` and `tgt_rev_layer`.
#[allow(clippy::too_many_arguments)]
fn current_max_layer_width(
    all_nodes: &[NodeId],
    layers: &[usize],
    current_height: usize,
    _original_max_height: usize,
    pixel_weighting: bool,
    graph: &LGraph,
    _id_map: &hashbrown::HashMap<NodeId, usize>,
    node_size_affix: f64,
    dummy_size: f64,
) -> f64 {
    let h = current_height.max(1);
    let mut per_layer = vec![0.0f64; h];
    for (idx, &rev_layer) in layers.iter().enumerate() {
        let layer_idx = h.saturating_sub(rev_layer + 1).min(h - 1);
        let nid = all_nodes[idx];
        if pixel_weighting {
            per_layer[layer_idx] += graph.node(nid).size.y + node_size_affix;
        } else {
            per_layer[layer_idx] += 1.0;
        }
    }
    // Add approximate dummy contributions: every edge that spans across
    // layers contributes one dummy per intermediate layer.
    for (idx, &rev_layer) in layers.iter().enumerate() {
        let src_layer = rev_layer;
        let nid = all_nodes[idx];
        for eid in graph.outgoing_edges(nid) {
            let tgt = graph.port(graph.edge(eid).target).owner;
            if let Some(&tgt_idx) = _id_map.get(&tgt) {
                let tgt_rev = layers[tgt_idx];
                if tgt_rev > src_layer + 1 {
                    // Dummies occupy layers strictly between the two
                    // reversed ids, exclusive.
                    for r in (src_layer + 1)..tgt_rev {
                        let actual = h.saturating_sub(r + 1).min(h - 1);
                        if pixel_weighting {
                            per_layer[actual] += dummy_size;
                        } else {
                            per_layer[actual] += 1.0;
                        }
                    }
                }
            }
        }
    }
    per_layer.iter().copied().fold(0.0_f64, f64::max)
}

/// Ahead-of-loop layer width snapshot used as the NIKOLOV / NIKOLOV_PIXEL
/// width-guard baseline. Accumulates a per-layer `dummyBaggage = running
/// outgoing - incoming so far` total: a layer's effective width is
/// `layerSize + dummyBaggage` (or `layerSizePixel + dummyBaggage * dummySize`
/// in pixel mode), so a candidate promotion is rejected only relative to a
/// baseline that already includes those implied dummy contributions.
fn compute_layer_widths_with_dummies(
    graph: &LGraph,
    pixel_weighting: bool,
) -> (f64, Vec<f64>, f64, f64) {
    let node_size_affix = graph.options.spacing.node_node;
    let dummy_size = graph.options.spacing.edge_node_between_layers;
    let mut per_layer = vec![0.0_f64; graph.layers.len()];
    let mut dummy_baggage_count: i64 = 0;
    let mut dummy_baggage_pixels: f64 = 0.0;
    for (i, layer) in graph.layers.iter().enumerate() {
        let mut incoming: i64 = 0;
        let mut outgoing: i64 = 0;
        let mut layer_pixel: f64 = 0.0;
        let layer_size = layer.nodes.len() as i64;
        for &nid in &layer.nodes {
            if pixel_weighting {
                layer_pixel += graph.node(nid).size.y + node_size_affix;
            }
            incoming += graph.incoming_edges(nid).count() as i64;
            outgoing += graph.outgoing_edges(nid).count() as i64;
        }
        // `dummy_baggage -= incoming` first, *then* compute
        // `nodes_n_dummies = layer_size + dummy_baggage`, then
        // `dummy_baggage += outgoing` for the next layer.
        dummy_baggage_count -= incoming;
        dummy_baggage_pixels -= incoming as f64 * dummy_size;
        let total = if pixel_weighting {
            layer_pixel + dummy_baggage_pixels
        } else {
            (layer_size + dummy_baggage_count).max(0) as f64
        };
        per_layer[i] = total;
        dummy_baggage_count += outgoing;
        dummy_baggage_pixels += outgoing as f64 * dummy_size;
    }
    let max = per_layer.iter().copied().fold(0.0_f64, f64::max);
    (max, per_layer, node_size_affix, dummy_size)
}

/// Sum of dummy-node count implied by current layer-graph edges.
fn count_dummy_nodes(graph: &LGraph) -> usize {
    let mut count = 0usize;
    for (_, node) in graph.nodes_iter() {
        let node_layer = node.layer;
        if node_layer.is_none() {
            continue;
        }
    }
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            for eid in graph.outgoing_edges(nid) {
                let src_layer = graph.node(nid).layer.unwrap_or(0) as i32;
                let tgt_port = graph.edge(eid).target;
                let tgt_node = graph.port(tgt_port).owner;
                let tgt_layer = graph.node(tgt_node).layer.map(|l| l as i32).unwrap_or(src_layer);
                if tgt_layer > src_layer + 1 {
                    count += (tgt_layer - src_layer - 1) as usize;
                }
            }
        }
    }
    count
}

/// Returns the current max width (node count) across all layers.
fn max_layer_width_in_nodes(graph: &LGraph) -> usize {
    graph.layers.iter().map(|l| l.nodes.len()).max().unwrap_or(0)
}

/// Returns the current max width (approximated pixels) across all layers.
fn max_layer_width_in_pixels(graph: &LGraph) -> f64 {
    let node_size_affix = graph.options.spacing.node_node;
    graph
        .layers
        .iter()
        .map(|l| l.nodes.iter().map(|&nid| graph.node(nid).size.y + node_size_affix).sum::<f64>())
        .fold(0.0_f64, f64::max)
}

fn try_promote(
    all_nodes: &[NodeId],
    layers: &mut [usize],
    in_degree: &[i32],
    out_degree: &[i32],
    id_map: &hashbrown::HashMap<NodeId, usize>,
    graph: &LGraph,
    node_idx: usize,
    current_height: &mut usize,
) -> i32 {
    let mut diff = 0;
    let mut stack = vec![node_idx];

    while let Some(node_idx) = stack.pop() {
        let node_id = all_nodes[node_idx];
        let old_layer = layers[node_idx];
        let new_layer = old_layer + 1;

        if new_layer >= *current_height {
            *current_height = new_layer + 1;
        }

        layers[node_idx] = new_layer;
        diff += out_degree[node_idx] - in_degree[node_idx];

        let predecessors: Vec<usize> = graph
            .incoming_edges(node_id)
            .filter_map(|edge_id| {
                let src_port = graph.edge(edge_id).source;
                let src_node = graph.port(src_port).owner;
                id_map.get(&src_node).copied().filter(|&src_idx| layers[src_idx] == new_layer)
            })
            .collect();
        for src_idx in predecessors.into_iter().rev() {
            stack.push(src_idx);
        }
    }

    diff
}

fn apply_layering(
    graph: &mut LGraph,
    all_nodes: &[NodeId],
    layers: &[usize],
    current_height: usize,
    _original_max_height: usize,
) {
    // Create new layers
    let mut new_layers: Vec<Vec<NodeId>> = vec![Vec::new(); current_height + 1];

    for (idx, &node_id) in all_nodes.iter().enumerate() {
        let rev_layer = layers[idx];
        // Convert back from reversed ID to normal layer index
        let layer_idx = current_height.saturating_sub(rev_layer + 1);
        // Clamp to avoid out-of-bounds
        let layer_idx = layer_idx.min(current_height);
        new_layers[layer_idx].push(node_id);
    }

    // Remove empty layers and rebuild
    graph.layers.clear();
    for nodes in new_layers {
        if nodes.is_empty() {
            continue;
        }
        let mut layer = LayerData::new();
        for &node_id in &nodes {
            graph.node_mut(node_id).layer = Some(graph.layers.len()).into();
            layer.nodes.push(node_id);
        }
        graph.layers.push(layer);
    }
}

// Model-order node promotion

/// Effective model order for a node.
///
/// Falls back to the node's insertion id when `MODEL_ORDER` has never been
/// set (still at the default `-1`). The importer seeds `MODEL_ORDER` with
/// the child index; test graphs built via `LGraph::add_node` do not go
/// through the importer, so the id fallback reproduces the same order.
fn effective_model_order(graph: &LGraph, nid: NodeId) -> i32 {
    let explicit = graph.node(nid).properties.get(&MODEL_ORDER);
    if explicit == -1 { graph.node(nid).id as i32 } else { explicit }
}

/// Promotes nodes using the model-order heuristic.
///
/// The label-layer special handling for `NodeType::LABEL` dummies inserted
/// for edge labels is intentionally skipped — none of the current promotion
/// tests exercise that path.
fn promote_nodes_model_order(graph: &mut LGraph, left_to_right: bool) {
    let n_layers = graph.layers.len();
    if n_layers < 2 {
        return;
    }

    let layer_map = compute_model_order_layering(graph, left_to_right);
    apply_model_order_layering(graph, &layer_map);
}

/// Bidirectional ordered multi-map.
///
/// Maintains a key->ordered values map alongside a value->key reverse index.
/// `put` removes the value from its prior key list before appending it to
/// the new key. Keys are never removed even after their value list empties —
/// the outer promotion loop relies on `keys` size growing monotonically as
/// cascades push values past the original key range.
struct BiLinkedHashMultiMap<K, V> {
    key_to_values: hashbrown::HashMap<K, Vec<V>>,
    value_to_key: hashbrown::HashMap<V, K>,
}

impl<K, V> BiLinkedHashMultiMap<K, V>
where
    K: Copy + Eq + std::hash::Hash + Ord,
    V: Copy + Eq + std::hash::Hash,
{
    fn new() -> Self {
        Self { key_to_values: hashbrown::HashMap::new(), value_to_key: hashbrown::HashMap::new() }
    }

    /// Insert `(key, value)`. If `value` was already present under another
    /// key, remove it from that prior list before appending here. The
    /// reverse map is updated unconditionally last.
    fn put(&mut self, key: K, value: V) {
        if let Some(&old_key) = self.value_to_key.get(&value)
            && let Some(values) = self.key_to_values.get_mut(&old_key)
        {
            values.retain(|&v| v != value);
        }
        self.key_to_values.entry(key).or_default().push(value);
        self.value_to_key.insert(value, key);
    }

    /// Bulk insert under `key`.
    fn put_all(&mut self, key: K, values: impl IntoIterator<Item = V>) {
        for value in values {
            self.put(key, value);
        }
    }

    /// Reverse lookup: which key currently holds `value`.
    fn get_key(&self, value: V) -> Option<K> {
        self.value_to_key.get(&value).copied()
    }

    /// Forward lookup: the values stored under `key`. Empty when unknown.
    fn get_values(&self, key: K) -> &[V] {
        self.key_to_values.get(&key).map(|v| v.as_slice()).unwrap_or(&[])
    }

    /// Sorted key view. The only consumer that relies on iteration order
    /// sorts the set itself, so a sorted view is exposed directly.
    fn keys_sorted(&self) -> Vec<K> {
        let mut ks: Vec<K> = self.key_to_values.keys().copied().collect();
        ks.sort();
        ks
    }

    /// True when `key` is the largest known key.
    fn is_maximal_key(&self, key: K) -> bool {
        self.key_to_values.keys().all(|&other| key >= other)
    }

    /// True when `key` is the smallest known key.
    fn is_minimal_key(&self, key: K) -> bool {
        self.key_to_values.keys().all(|&other| key <= other)
    }

    fn key_count(&self) -> usize {
        self.key_to_values.len()
    }
}

/// Pure computation of the new layering, operating on `&LGraph` only so
/// the borrow checker lets the main loop coexist with the final write-back.
///
/// Keys are `i64` so a cascade that crosses the original layer 0 (RTL) or
/// final layer (LTR) can extend the multimap by a fresh negative or
/// `n_layers+k` key without bounds checks. The final pass
/// (`apply_model_order_layering`) sorts the key set and emits a
/// contiguous `Vec` with empty layers filtered out.
fn compute_model_order_layering(graph: &LGraph, left_to_right: bool) -> Vec<Vec<NodeId>> {
    let n_layers = graph.layers.len();

    let mut layer_map: BiLinkedHashMultiMap<i64, NodeId> = BiLinkedHashMultiMap::new();
    // Seed each layer's nodes sorted by effective model order. For LTR sort
    // descending so the largest-order node leads each layer; for RTL
    // ascending.
    for (idx, layer) in graph.layers.iter().enumerate() {
        let mut nodes: Vec<NodeId> = layer.nodes.clone();
        nodes.sort_by(|&a, &b| {
            if !has_explicit_model_order(graph, a) || !has_explicit_model_order(graph, b) {
                return std::cmp::Ordering::Equal;
            }
            let mo_a = effective_model_order(graph, a);
            let mo_b = effective_model_order(graph, b);
            if left_to_right { mo_b.cmp(&mo_a) } else { mo_a.cmp(&mo_b) }
        });
        layer_map.put_all(idx as i64, nodes);
    }

    loop {
        let mut something_changed = false;

        // Iterate keys in the direction of promotion. `current_layer_id`
        // walks the integer range `0..size-1` (LTR reversed, RTL forward)
        // treating the integer as the key, not the sorted key set. This
        // matters when the cascade extends the multimap by a negative key
        // — RTL still iterates `1..size`, so the (now non-extremal) key 0
        // stays unvisited as a seed even though `is_minimal_key(0)` would
        // now return false.
        let size = layer_map.key_count() as i64;
        let layer_order: Vec<i64> = if left_to_right {
            (0..size.saturating_sub(1)).rev().collect()
        } else {
            (1..size).collect()
        };

        for current in layer_order {
            let mut i = 0;
            while i < layer_map.get_values(current).len() {
                let node = layer_map.get_values(current)[i];

                // Nodes without an explicit `MODEL_ORDER` never enter the
                // promotion loop, so label dummies can never be the
                // "current candidate" — they are promoted only as cascade
                // companions.
                if !has_explicit_model_order(graph, node) {
                    i += 1;
                    continue;
                }
                let my_order = effective_model_order(graph, node);

                // Skip seeds at the extremal key.
                let at_boundary = if left_to_right {
                    layer_map.is_maximal_key(current)
                } else {
                    layer_map.is_minimal_key(current)
                };
                if at_boundary {
                    i += 1;
                    continue;
                }

                // Condition 1: node must have the extremal model order in
                // its current layer. Left-to-right wants the largest;
                // right-to-left wants the smallest. Skip nodes that never
                // had an explicit MO seeded (e.g. label dummies).
                let mut shall_be_promoted = true;
                for &other in layer_map.get_values(current) {
                    if other == node {
                        continue;
                    }
                    if !has_explicit_model_order(graph, other) {
                        continue;
                    }
                    let their = effective_model_order(graph, other);
                    if left_to_right && my_order < their {
                        shall_be_promoted = false;
                        break;
                    }
                    if !left_to_right && my_order > their {
                        shall_be_promoted = false;
                        break;
                    }
                }
                if !shall_be_promoted {
                    i += 1;
                    continue;
                }

                // Condition 2: either the adjacent layer contains a node
                // with strictly worse model order (so the swap is justified
                // by model order) OR the adjacent layer is purely made of
                // label dummies that the node can jump across.
                let next = if left_to_right { current + 1 } else { current - 1 };
                let mut model_order_allows = false;
                let mut promote_through_dummy_layer = true;
                let mut contains_labels = false;

                for &next_node in layer_map.get_values(next) {
                    if next_node == node {
                        continue;
                    }
                    if has_explicit_model_order(graph, next_node) {
                        let their = effective_model_order(graph, next_node);
                        let allows =
                            if left_to_right { their < my_order } else { their > my_order };
                        model_order_allows |= allows;
                        promote_through_dummy_layer = false;
                    } else if !model_order_allows && promote_through_dummy_layer {
                        // The label-layer promotion path is allowed only for
                        // `NodeType::Label` dummies produced by the label
                        // dummy inserter; any other untyped neighbor would
                        // signal real structure we cannot jump across.
                        if graph.node(next_node).node_type != NodeType::Label {
                            continue;
                        }
                        contains_labels = true;
                        let upstream = label_opposite_end(graph, next_node, !left_to_right);
                        if upstream == Some(node)
                            && let Some(downstream) =
                                label_opposite_end(graph, next_node, left_to_right)
                        {
                            let distance =
                                layer_distance(&layer_map, node, downstream, left_to_right);
                            if distance <= 2 {
                                promote_through_dummy_layer = false;
                            }
                        }
                    }
                }

                // If we would promote across a label layer, require that
                // the node has a long enough edge to justify spanning it.
                if contains_labels && promote_through_dummy_layer {
                    let connected = if left_to_right {
                        first_adjacent(graph, node, true)
                    } else {
                        first_adjacent(graph, node, false)
                    };
                    if let Some(conn) = connected {
                        let distance = layer_distance(&layer_map, node, conn, left_to_right);
                        if distance <= 2 && graph.node(conn).node_type == NodeType::Normal {
                            promote_through_dummy_layer = false;
                        }
                    }
                }

                if !(model_order_allows || promote_through_dummy_layer) {
                    i += 1;
                    continue;
                }

                // Promote the node and cascade to any connected nodes that
                // end up co-located with it. Use FIFO order + membership
                // dedup to keep each cascade step single-promotion: without
                // the dedup, the same node can be promoted multiple times
                // in a single outer pass, leaving connected siblings at
                // the wrong layer and producing intra-layer edges.
                let seed = promote_node_by_model_order(&mut layer_map, graph, node, left_to_right);
                let mut queue: VecDeque<NodeId> = VecDeque::new();
                let mut in_queue: hashbrown::HashSet<NodeId> = hashbrown::HashSet::new();
                for n in seed {
                    if in_queue.insert(n) {
                        queue.push_back(n);
                    }
                }
                while let Some(next_node) = queue.pop_front() {
                    in_queue.remove(&next_node);
                    let more = promote_node_by_model_order(
                        &mut layer_map,
                        graph,
                        next_node,
                        left_to_right,
                    );
                    for m in more {
                        if in_queue.insert(m) {
                            queue.push_back(m);
                        }
                    }
                }

                something_changed = true;
                // Do not increment `i`: the current slot now holds the
                // next node, since the promoted one was removed from this
                // layer.
            }
        }

        if !something_changed {
            break;
        }
    }

    // Materialise the final layer sequence by walking the sorted key set
    // and skipping empty layers. Keep `n_layers` referenced so both
    // direction-specific builds compile without dead-code noise.
    let _ = n_layers;
    layer_map
        .keys_sorted()
        .into_iter()
        .map(|k| layer_map.get_values(k).to_vec())
        .filter(|v| !v.is_empty())
        .collect()
}

/// Moves `node` one layer in the promotion direction and returns any
/// connected nodes that end up in the same layer (candidates for cascading
/// promotion).
fn promote_node_by_model_order(
    layer_map: &mut BiLinkedHashMultiMap<i64, NodeId>,
    graph: &LGraph,
    node: NodeId,
    left_to_right: bool,
) -> Vec<NodeId> {
    let old_layer = layer_map.get_key(node).expect("promoted node has no current layer");
    let new_layer = if left_to_right { old_layer + 1 } else { old_layer - 1 };

    layer_map.put(new_layer, node);

    let mut nodes_to_promote = Vec::new();
    if left_to_right {
        for eid in graph.outgoing_edges(node) {
            let tgt_port = graph.edge(eid).target;
            let tgt_node = graph.port(tgt_port).owner;
            if layer_map.get_key(tgt_node) == Some(new_layer) && tgt_node != node {
                nodes_to_promote.push(tgt_node);
            }
        }
    } else {
        for eid in graph.incoming_edges(node) {
            let src_port = graph.edge(eid).source;
            let src_node = graph.port(src_port).owner;
            if layer_map.get_key(src_node) == Some(new_layer) && src_node != node {
                nodes_to_promote.push(src_node);
            }
        }
    }

    nodes_to_promote
}

/// Whether the node carries an explicit `MODEL_ORDER` property, i.e. an
/// importer (or a test builder that sets the key) asserted a model order.
/// Nodes that never had it set fall back to their scratch id inside
/// `effective_model_order`, so this distinction is required to tell a
/// user-defined ordering apart from a synthetic one.
fn has_explicit_model_order(graph: &LGraph, node: NodeId) -> bool {
    graph.node(node).properties.get(&MODEL_ORDER) != -1
}

/// Returns the node on the opposite end of the first connected edge of a
/// label dummy. `downstream` selects which end: `true` follows the outgoing
/// edge (target side), `false` follows the incoming edge (source side).
fn label_opposite_end(graph: &LGraph, label_node: NodeId, downstream: bool) -> Option<NodeId> {
    if downstream {
        let eid = graph.outgoing_edges(label_node).next()?;
        Some(graph.port(graph.edge(eid).target).owner)
    } else {
        let eid = graph.incoming_edges(label_node).next()?;
        Some(graph.port(graph.edge(eid).source).owner)
    }
}

/// Returns the node on the other end of the first outgoing (or incoming)
/// edge of `node`. Used to pick the neighbor consulted when deciding whether
/// a current node's edge is long enough to justify crossing a label layer.
fn first_adjacent(graph: &LGraph, node: NodeId, outgoing: bool) -> Option<NodeId> {
    if outgoing {
        let eid = graph.outgoing_edges(node).next()?;
        Some(graph.port(graph.edge(eid).target).owner)
    } else {
        let eid = graph.incoming_edges(node).next()?;
        Some(graph.port(graph.edge(eid).source).owner)
    }
}

/// Absolute difference in layer indices between `base` and `other`, signed
/// according to the promotion direction so that "downstream" always reads as
/// a positive distance.
fn layer_distance(
    layer_map: &BiLinkedHashMultiMap<i64, NodeId>,
    base: NodeId,
    other: NodeId,
    left_to_right: bool,
) -> i32 {
    let base_layer = layer_map.get_key(base).unwrap_or(0) as i32;
    let other_layer = layer_map.get_key(other).unwrap_or(0) as i32;
    if left_to_right { other_layer - base_layer } else { base_layer - other_layer }
}

/// Writes the computed per-layer node lists back to `graph.layers`.
///
/// Empty layers are skipped and node `layer` fields are updated to match
/// their new index.
fn apply_model_order_layering(graph: &mut LGraph, layer_map: &[Vec<NodeId>]) {
    graph.layers.clear();
    for layer_nodes in layer_map {
        if layer_nodes.is_empty() {
            continue;
        }
        let mut layer = LayerData::new();
        for &nid in layer_nodes {
            graph.node_mut(nid).layer = Some(graph.layers.len()).into();
            layer.nodes.push(nid);
        }
        graph.layers.push(layer);
    }
}
