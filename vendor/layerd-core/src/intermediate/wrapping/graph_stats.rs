//! Per-layer width / height statistics consumed by the cut-index heuristics
//! and the single-edge graph wrapper. Fields are populated lazily on first
//! access.

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    options::enums::LayoutDirection,
    properties::internal::ORIGIN_NODE,
};

/// Lazily-computed width / height statistics for a layered graph.
pub struct GraphStats<'g> {
    pub graph: &'g LGraph,
    /// Desired aspect ratio (width / height), direction-corrected.
    pub dar: f64,
    /// Number of layers in the graph.
    pub longest_path: usize,

    spacing: f64,
    in_layer_spacing: f64,

    max_width: Option<f64>,
    max_height: Option<f64>,
    sum_width: Option<f64>,

    widths: Option<Vec<f64>>,
    heights: Option<Vec<f64>>,

    cuts_allowed: Option<Vec<bool>>,
}

impl<'g> GraphStats<'g> {
    /// Build a fresh stats object for `graph`. Only direction / aspect data
    /// is computed eagerly; width and height tables are lazy.
    pub fn new(graph: &'g LGraph) -> Self {
        let options = &graph.options;
        let aspect_ratio = options.aspect_ratio;
        let correction = options.wrapping_correction_factor;
        let dar = match options.direction {
            LayoutDirection::Left | LayoutDirection::Right | LayoutDirection::Undefined =>
                aspect_ratio * correction,
            _ => 1.0 / (aspect_ratio * correction),
        };

        let spacing = options.spacing.node_node_between_layers;
        let in_layer_spacing = options.spacing.node_node;
        let longest_path = graph.layers.len();

        Self {
            graph,
            dar,
            longest_path,
            spacing,
            in_layer_spacing,
            max_width: None,
            max_height: None,
            sum_width: None,
            widths: None,
            heights: None,
            cuts_allowed: None,
        }
    }

    /// Per-layer widths (cached).
    pub fn widths(&mut self) -> &[f64] {
        self.ensure_widths_and_heights();
        self.widths.as_deref().unwrap()
    }

    /// Per-layer heights (cached).
    pub fn heights(&mut self) -> &[f64] {
        self.ensure_widths_and_heights();
        self.heights.as_deref().unwrap()
    }

    /// Max width across all layers (cached).
    pub fn max_width(&mut self) -> f64 {
        if self.max_width.is_none() {
            let m = (0..self.graph.layers.len())
                .map(|i| self.determine_layer_width(i))
                .fold(f64::NEG_INFINITY, f64::max);
            self.max_width = Some(m);
        }
        self.max_width.unwrap()
    }

    /// Sum of per-layer widths (cached).
    pub fn sum_width(&mut self) -> f64 {
        if self.sum_width.is_none() {
            self.sum_width = Some(self.graph_sum_width());
        }
        self.sum_width.unwrap()
    }

    /// Max height across all layers (cached).
    pub fn max_height(&mut self) -> f64 {
        if self.max_height.is_none() {
            let m = (0..self.graph.layers.len())
                .map(|i| self.determine_layer_height(i))
                .fold(f64::NEG_INFINITY, f64::max);
            self.max_height = Some(m);
        }
        self.max_height.unwrap()
    }

    /// Whether it is legal to cut immediately before `layer_index`.
    pub fn is_cut_allowed(&mut self, layer_index: usize) -> bool {
        self.ensure_cuts_allowed();
        self.cuts_allowed.as_ref().unwrap()[layer_index]
    }

    fn graph_sum_width(&self) -> f64 {
        (0..self.graph.layers.len()).map(|i| self.determine_layer_width(i)).sum()
    }

    fn determine_layer_width(&self, layer_idx: usize) -> f64 {
        let layer = &self.graph.layers[layer_idx];
        let mut max_w = 0.0_f64;
        for &node_id in &layer.nodes {
            let node = self.graph.node(node_id);
            let n_w = node.size.x + node.margin.right + node.margin.left + self.spacing;
            max_w = max_w.max(n_w);
        }
        max_w
    }

    fn determine_layer_height(&self, layer_idx: usize) -> f64 {
        let layer = &self.graph.layers[layer_idx];
        let mut total = 0.0_f64;
        for &node_id in &layer.nodes {
            let node = self.graph.node(node_id);
            total += node.size.y + node.margin.bottom + node.margin.top + self.in_layer_spacing;

            let incoming: Vec<_> = self.graph.incoming_edges(node_id).collect();
            for eid in incoming {
                let src_port = self.graph.edge(eid).source;
                let src_owner = self.graph.port(src_port).owner;
                if self.graph.node(src_owner).node_type == NodeType::NorthSouthPort {
                    let origin: Option<NodeId> =
                        self.graph.node(src_owner).properties.get(&ORIGIN_NODE);
                    if let Some(origin_id) = origin {
                        let origin_node = self.graph.node(origin_id);
                        total +=
                            origin_node.size.y + origin_node.margin.bottom + origin_node.margin.top;
                    }
                }
            }
        }
        total
    }

    fn ensure_widths_and_heights(&mut self) {
        if self.widths.is_some() && self.heights.is_some() {
            return;
        }
        let n = self.longest_path;
        let mut widths = Vec::with_capacity(n);
        let mut heights = Vec::with_capacity(n);
        for i in 0..n {
            widths.push(self.determine_layer_width(i));
            heights.push(self.determine_layer_height(i));
        }
        self.widths = Some(widths);
        self.heights = Some(heights);
    }

    fn ensure_cuts_allowed(&mut self) {
        if self.cuts_allowed.is_some() {
            return;
        }
        let n = self.graph.layers.len();
        let mut allowed = vec![false; n];
        if let Some(forbidden) = &self.graph.options.wrapping_validify_forbidden_indices {
            // When forbidden indices are specified explicitly, every other
            // index is implicitly allowed; only the flagged ones are
            // forbidden. The reference initialises the array with `false`,
            // then the loop only writes `false` on forbidden entries. That
            // looks redundant but is bug-compat for `cuts_allowed[0] = false`
            // plus the loop never setting anything `true`. Replicate verbatim.
            for &f in forbidden {
                if f > 0 && (f as usize) < allowed.len() {
                    allowed[f as usize] = false;
                }
            }
        } else {
            // Default behaviour: cut before a layer only when that layer is
            // connected to the previous one by exactly one source-target pair.
            allowed[0] = false;
            for (i, allowed_cut) in allowed.iter_mut().enumerate().take(n).skip(1) {
                *allowed_cut = self.is_cut_allowed_layer(i);
            }
        }
        self.cuts_allowed = Some(allowed);
    }

    fn is_cut_allowed_layer(&self, layer_idx: usize) -> bool {
        let layer = &self.graph.layers[layer_idx];
        let mut n1: Option<NodeId> = None;
        let mut n2: Option<NodeId> = None;
        for &tgt_id in &layer.nodes {
            let incoming: Vec<_> = self.graph.incoming_edges(tgt_id).collect();
            for eid in incoming {
                if let Some(existing) = n1
                    && existing != tgt_id
                {
                    return false;
                }
                n1 = Some(tgt_id);
                let src_port = self.graph.edge(eid).source;
                let src_node = self.graph.port(src_port).owner;
                if let Some(existing) = n2
                    && existing != src_node
                {
                    return false;
                }
                n2 = Some(src_node);
            }
        }
        true
    }
}
