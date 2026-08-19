//! Stretch-width layerer.
//!
//! StretchWidth (Nikolov, Tarassov, Branke) is a heuristic for the minimum-
//! width layering problem with consideration of dummy nodes. It is a
//! bottom-up algorithm: layers are built starting from the sinks and the
//! final list is reversed at the end.

use crate::graph::{LGraph, LayerData, index::NodeId, node::NodeType};

/// Assign layers using the StretchWidth heuristic.
pub fn assign_layers(graph: &mut LGraph) {
    let nodes: Vec<NodeId> = graph.layerless_nodes.clone();
    if nodes.is_empty() {
        return;
    }
    let mut state = StretchWidthState::build(graph, &nodes);
    state.run(graph);
}

struct StretchWidthState {
    /// Number of layerless nodes.
    n: usize,
    /// Dense-index → original `NodeId`.
    idx_to_node: Vec<NodeId>,
    /// `NodeData.id` → dense index.
    id_to_idx: Vec<usize>,
    /// Nodes sorted descending by rank (computed once in `build`).
    ///
    /// Rank is `max(d+(v), max_{u in pred(v)}(d+(u)))`.
    sorted_layerless: Vec<usize>,
    /// `remainingOutGoing[idx]`. Initialized to `out_degree[idx]`; decremented
    /// by `update_out_going` each time a layer is closed.
    remaining_outgoing: Vec<i32>,
    /// Pristine copy of the out-degree — source for the reset path
    /// (`remainingOutGoing = copyOf(outDegree)`).
    out_degree: Vec<i32>,
    /// In-degree (no self-loops excluded).
    in_degree: Vec<i32>,
    /// Normalized per-node size: `node.size.y / minimumNodeSize`.
    norm_size: Vec<f64>,
    /// Normalized dummy size: `SPACING_EDGE_EDGE / minimumNodeSize`.
    dummy_size: f64,
    /// Normalized max real-node size (initial `maxWidth`).
    max_real: f64,
    /// Average outgoing degree of the graph.
    upper_layer_influence: f64,
    /// Whether a node has been placed into a layer in the current pass.
    placed: Vec<bool>,
}

impl StretchWidthState {
    fn build(graph: &LGraph, nodes: &[NodeId]) -> Self {
        let n = nodes.len();
        let max_id = nodes.iter().map(|&nid| graph.node(nid).id).max().unwrap_or(0) as usize;
        let mut id_to_idx = vec![usize::MAX; max_id + 1];
        let mut idx_to_node = Vec::with_capacity(n);
        for (i, &nid) in nodes.iter().enumerate() {
            id_to_idx[graph.node(nid).id as usize] = i;
            idx_to_node.push(nid);
        }

        let mut out_degree = vec![0i32; n];
        let mut in_degree = vec![0i32; n];
        for (i, &nid) in nodes.iter().enumerate() {
            for _ in graph.outgoing_edges(nid) {
                out_degree[i] += 1;
            }
            for _ in graph.incoming_edges(nid) {
                in_degree[i] += 1;
            }
        }

        // Rank = max(d+(v), max d+(u) for u in pred(v)).
        let mut rank = vec![0i32; n];
        for (i, &nid) in nodes.iter().enumerate() {
            let mut max_rank = out_degree[i];
            for eid in graph.incoming_edges(nid) {
                let src_node = graph.port(graph.edge(eid).source).owner;
                let src_id = graph.node(src_node).id as usize;
                if src_id < id_to_idx.len() {
                    let j = id_to_idx[src_id];
                    if j != usize::MAX && out_degree[j] > max_rank {
                        max_rank = out_degree[j];
                    }
                }
            }
            rank[i] = max_rank;
        }

        let mut sorted_layerless: Vec<usize> = (0..n).collect();
        sorted_layerless.sort_by(|&a, &b| rank[b].cmp(&rank[a]).then_with(|| a.cmp(&b)));

        let mut min_real = f64::INFINITY;
        let mut max_real = f64::NEG_INFINITY;
        for &nid in nodes {
            if graph.node(nid).node_type != NodeType::Normal {
                continue;
            }
            let s = graph.node(nid).size.y;
            if s < min_real {
                min_real = s;
            }
            if s > max_real {
                max_real = s;
            }
        }
        let raw_min_real = min_real;
        if !min_real.is_finite() {
            min_real = 1.0;
        }
        let min_real = min_real.max(1.0);
        let max_real = max_real.max(1.0);

        // Preserve `0 / 0 = NaN` behaviour for all-zero fixtures, but do
        // not let `positive / 0` become infinity and reset forever.
        let mut norm_size = vec![0.0f64; n];
        for (i, &nid) in nodes.iter().enumerate() {
            let raw = graph.node(nid).size.y / raw_min_real;
            norm_size[i] = if raw.is_infinite() { graph.node(nid).size.y / min_real } else { raw };
        }

        let dummy_size = graph.options.spacing.edge_edge / min_real;
        let normalized_max_real = max_real / min_real;

        let total_out: f64 = out_degree.iter().map(|&d| d as f64).sum();
        let upper_layer_influence = if n == 0 { 0.0 } else { total_out / n as f64 };

        StretchWidthState {
            n,
            idx_to_node,
            id_to_idx,
            sorted_layerless,
            remaining_outgoing: out_degree.clone(),
            out_degree,
            in_degree,
            norm_size,
            dummy_size,
            max_real: normalized_max_real,
            upper_layer_influence,
            placed: vec![false; n],
        }
    }

    fn run(&mut self, graph: &mut LGraph) {
        let mut width_current: f64 = 0.0;
        let mut width_up: f64 = 0.0;
        let mut max_width: f64 = self.max_real.max(1.0);

        let mut layers: Vec<Vec<usize>> = Vec::new();
        let mut current: Vec<usize> = Vec::new();
        let mut remaining = self.n;

        loop {
            if remaining == 0 {
                break;
            }

            let selected = self.select_node();

            let advance_layer = match selected {
                None => true,
                Some(idx) =>
                    self.condition_go_up(idx, width_current, width_up, max_width)
                        && !current.is_empty(),
            };
            if advance_layer {
                // Close the current layer, start the next layer above.
                self.update_out_going(graph, &current);
                layers.push(std::mem::take(&mut current));
                width_current = width_up;
                width_up = 0.0;
                continue;
            }

            let idx = selected.expect("selected is Some when we reach this branch");

            if self.condition_go_up(idx, width_current, width_up, max_width) {
                // Reset path: widen `maxWidth` and start over.
                layers.clear();
                current.clear();
                width_current = 0.0;
                width_up = 0.0;
                max_width += 1.0;
                for slot in &mut self.placed {
                    *slot = false;
                }
                self.remaining_outgoing.copy_from_slice(&self.out_degree);
                remaining = self.n;
                continue;
            }

            // Place `idx` into the current layer.
            current.push(idx);
            self.placed[idx] = true;
            remaining -= 1;
            width_current =
                width_current - self.out_degree[idx] as f64 * self.dummy_size + self.norm_size[idx];
            width_up += self.in_degree[idx] as f64 * self.dummy_size;
        }

        if !current.is_empty() {
            layers.push(current);
        }

        // Bottom-up → reverse so layer 0 is the top-most (source) layer.
        layers.reverse();

        graph.layers.clear();
        for _ in 0..layers.len() {
            graph.layers.push(LayerData::new());
        }
        for (li, layer) in layers.iter().enumerate() {
            for &idx in layer {
                let nid = self.idx_to_node[idx];
                graph.layers[li].nodes.push(nid);
                graph.node_mut(nid).layer = Some(li).into();
            }
        }
        graph.layerless_nodes.clear();
    }

    /// Return the first sorted node whose `remaining_outgoing <= 0`. `None`
    /// when no candidate remains.
    fn select_node(&self) -> Option<usize> {
        for &idx in &self.sorted_layerless {
            if self.placed[idx] {
                continue;
            }
            if self.remaining_outgoing[idx] <= 0 {
                return Some(idx);
            }
        }
        None
    }

    /// Predict whether placing `idx` into the current layer would overflow
    /// either the current-layer width bound (a) or the in-edge estimate for
    /// the next layer (b).
    fn condition_go_up(
        &self,
        idx: usize,
        width_current: f64,
        width_up: f64,
        max_width: f64,
    ) -> bool {
        let a = (width_current - self.out_degree[idx] as f64 * self.dummy_size
            + self.norm_size[idx])
            > max_width;
        let b = (width_up + self.in_degree[idx] as f64 * self.dummy_size)
            > (max_width * self.upper_layer_influence * self.dummy_size);
        a || b
    }

    /// For every node in the just-closed layer, decrement
    /// `remaining_outgoing` at each of its predecessors.
    fn update_out_going(&mut self, graph: &LGraph, current_layer: &[usize]) {
        for &idx in current_layer {
            let nid = self.idx_to_node[idx];
            for eid in graph.incoming_edges(nid) {
                let src_node = graph.port(graph.edge(eid).source).owner;
                let src_id = graph.node(src_node).id as usize;
                if src_id < self.id_to_idx.len() {
                    let j = self.id_to_idx[src_id];
                    if j != usize::MAX {
                        self.remaining_outgoing[j] -= 1;
                    }
                }
            }
        }
    }
}
