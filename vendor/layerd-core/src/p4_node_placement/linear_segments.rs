//! Linear segments node placement.
//!
//! Reference: Georg Sander, "A fast heuristic for hierarchical Manhattan
//! layout", GD'95, LNCS 1027, pp. 447-458.

use hashbrown::HashMap;

use crate::{
    graph::{LGraph, index::NodeId, node::NodeType},
    properties::internal::PRIORITY_STRAIGHTNESS,
};

type SegIdx = u32;

/// Default priority seed used before examining per-edge `PRIORITY_STRAIGHTNESS`.
///
/// Set to the minimum signed value when no edges exist on a node.
const NO_EDGE_PRIO: i32 = i32::MIN;

/// Factor for threshold after which balancing is aborted.
const THRESHOLD_FACTOR: f64 = 20.0;
/// Minimum pendulum iterations.
const PENDULUM_ITERS: i32 = 4;
/// Additional iterations after the abort condition fires.
const FINAL_ITERS: i32 = 3;
/// Factor for threshold within which node overlapping is detected.
const OVERLAP_DETECT: f64 = 0.0001;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    ForwPendulum,
    BackwPendulum,
    Rubber,
}

struct LinearSegment {
    nodes: Vec<NodeId>,
    id: SegIdx,
    index_in_last_layer: i32,
    last_layer: i32,
    deflection: f64,
    weight: i32,
    ref_segment: Option<SegIdx>,
}

impl LinearSegment {
    fn new(id: SegIdx) -> Self {
        Self {
            nodes: Vec::new(),
            id,
            index_in_last_layer: -1,
            last_layer: -1,
            deflection: 0.0,
            weight: 0,
            ref_segment: None,
        }
    }
}

/// Entry point for the linear segments placement algorithm.
pub fn place_nodes(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }
    let mut state = State::new(graph);
    state.sort_linear_segments(graph);
    state.create_unbalanced_placement(graph);
    state.balance_placement(graph);
    state.post_process(graph);
    state.finalize_graph_height(graph);
}

struct State {
    segments: Vec<LinearSegment>,
    /// Maps each node to the index of its owning segment.
    node_to_seg: HashMap<NodeId, SegIdx>,
    /// Per-node `INPUT_PRIO`: max `PRIORITY_STRAIGHTNESS` across incoming edges.
    input_prio: HashMap<NodeId, i32>,
    /// Per-node `OUTPUT_PRIO`: max `PRIORITY_STRAIGHTNESS` across outgoing edges.
    output_prio: HashMap<NodeId, i32>,
}

impl State {
    fn new(_graph: &LGraph) -> Self {
        Self {
            segments: Vec::new(),
            node_to_seg: HashMap::new(),
            input_prio: HashMap::new(),
            output_prio: HashMap::new(),
        }
    }

    fn sort_linear_segments(&mut self, graph: &LGraph) {
        // Compute per-node input/output priorities.
        self.compute_priorities(graph);

        // Create linear segments by walking successors for LONG_EDGE and
        // NORTH_SOUTH_PORT nodes.
        for layer in &graph.layers {
            for &nid in &layer.nodes {
                if self.node_to_seg.contains_key(&nid) {
                    continue;
                }
                let seg_id = self.segments.len() as SegIdx;
                let mut seg = LinearSegment::new(seg_id);
                self.fill_segment(graph, nid, &mut seg);
                self.segments.push(seg);
            }
        }

        // Build and topologically sort the segment dependency graph.
        let n = self.segments.len();
        let mut outgoing: Vec<Vec<SegIdx>> = vec![Vec::new(); n];
        let mut incoming_count: Vec<i32> = vec![0; n];

        self.create_dependency_graph_edges(graph, &mut outgoing, &mut incoming_count);

        let n = self.segments.len();
        if outgoing.len() < n {
            outgoing.resize(n, Vec::new());
        }
        if incoming_count.len() < n {
            incoming_count.resize(n, 0);
        }

        // Kahn's algorithm.
        let mut no_incoming: Vec<SegIdx> =
            (0..n as SegIdx).filter(|&i| incoming_count[i as usize] == 0).collect();
        let mut new_ranks: Vec<i32> = vec![-1; n];
        let mut next_rank = 0i32;
        while let Some(sid) = no_incoming.pop() {
            new_ranks[sid as usize] = next_rank;
            next_rank += 1;
            let nexts = std::mem::take(&mut outgoing[sid as usize]);
            for tgt in nexts {
                let tgt_u = tgt as usize;
                incoming_count[tgt_u] -= 1;
                if incoming_count[tgt_u] == 0 {
                    no_incoming.push(tgt);
                }
            }
        }

        // Apply the new ordering.
        let mut reordered: Vec<LinearSegment> = Vec::with_capacity(n);
        reordered.resize_with(n, || LinearSegment::new(0));
        let mut taken: Vec<Option<LinearSegment>> = self.segments.drain(..).map(Some).collect();
        for (old_id, seg_opt) in taken.iter_mut().enumerate() {
            let rank = new_ranks[old_id];
            assert!(rank >= 0, "segment {old_id} not ranked");
            let mut seg = seg_opt.take().expect("segment already consumed");
            seg.id = rank as SegIdx;
            for &nid in &seg.nodes {
                self.node_to_seg.insert(nid, rank as SegIdx);
            }
            reordered[rank as usize] = seg;
        }
        self.segments = reordered;
    }

    fn compute_priorities(&mut self, graph: &LGraph) {
        for layer in &graph.layers {
            for &nid in &layer.nodes {
                let mut inprio = NO_EDGE_PRIO;
                let mut outprio = NO_EDGE_PRIO;
                for eid in graph.incoming_edges(nid) {
                    let prio = graph.edge(eid).properties.get(&PRIORITY_STRAIGHTNESS);
                    inprio = inprio.max(prio);
                }
                for eid in graph.outgoing_edges(nid) {
                    let prio = graph.edge(eid).properties.get(&PRIORITY_STRAIGHTNESS);
                    outprio = outprio.max(prio);
                }
                self.input_prio.insert(nid, inprio);
                self.output_prio.insert(nid, outprio);
            }
        }
    }

    fn fill_segment(&mut self, graph: &LGraph, nid: NodeId, seg: &mut LinearSegment) -> bool {
        let mut current = nid;
        loop {
            if self.node_to_seg.contains_key(&current) {
                return false;
            }
            self.node_to_seg.insert(current, seg.id);
            seg.nodes.push(current);
            let node_type = graph.node(current).node_type;

            if matches!(node_type, NodeType::LongEdge | NodeType::NorthSouthPort) {
                let cur_layer = graph.node(current).layer;
                let mut next = None;
                'ports: for &pid in graph.node(current).ports.iter() {
                    for &eid in graph.port(pid).outgoing_edges.iter() {
                        let tgt_port = graph.edge(eid).target;
                        let tgt_node = graph.port(tgt_port).owner;
                        let tgt_type = graph.node(tgt_node).node_type;
                        if graph.node(tgt_node).layer == cur_layer {
                            continue;
                        }
                        if matches!(tgt_type, NodeType::LongEdge | NodeType::NorthSouthPort)
                            && !self.node_to_seg.contains_key(&tgt_node)
                        {
                            next = Some(tgt_node);
                            break 'ports;
                        }
                    }
                }
                if let Some(next_node) = next {
                    current = next_node;
                    continue;
                }
            }
            return true;
        }
    }

    fn create_dependency_graph_edges(
        &mut self,
        graph: &LGraph,
        outgoing: &mut Vec<Vec<SegIdx>>,
        incoming_count: &mut Vec<i32>,
    ) {
        for (layer_index, layer) in graph.layers.iter().enumerate() {
            let nodes = &layer.nodes;
            if nodes.is_empty() {
                continue;
            }
            let layer_index_i = layer_index as i32;
            let mut index_in_layer = 0i32;
            let mut previous_node: Option<NodeId> = None;
            let mut cursor = 0usize;
            let mut current_node = Some(nodes[cursor]);

            while let Some(current) = current_node {
                let current_seg_id = self.node_to_seg[&current];
                let current_seg = &self.segments[current_seg_id as usize];

                // Cycle detection: scan upcoming nodes in this layer for any
                // segment that appeared after `current` in its `last_layer`
                // but before `current` in the current layer.
                let mut cycle_seg: Option<SegIdx> = None;
                if current_seg.index_in_last_layer >= 0 {
                    for &ahead in &nodes[cursor + 1..] {
                        let ahead_seg_id = self.node_to_seg[&ahead];
                        let ahead_seg = &self.segments[ahead_seg_id as usize];
                        if ahead_seg.last_layer == current_seg.last_layer
                            && ahead_seg.index_in_last_layer < current_seg.index_in_last_layer
                        {
                            cycle_seg = Some(ahead_seg_id);
                            break;
                        }
                    }
                }

                // Split if needed.
                let active_seg_id = if cycle_seg.is_some() {
                    let old_id = current_seg_id;
                    let new_id = self.segments.len() as SegIdx;

                    // Move all nodes from `current` onward into a new segment.
                    let split_pos = self.segments[old_id as usize]
                        .nodes
                        .iter()
                        .position(|&n| n == current)
                        .expect("split anchor not found in segment");
                    let moved: Vec<NodeId> =
                        self.segments[old_id as usize].nodes.drain(split_pos..).collect();
                    let mut new_seg = LinearSegment::new(new_id);
                    for &nid in &moved {
                        self.node_to_seg.insert(nid, new_id);
                    }
                    new_seg.nodes = moved;
                    self.segments.push(new_seg);
                    outgoing.push(Vec::new());
                    if let Some(prev) = previous_node {
                        let prev_seg_id = self.node_to_seg[&prev];
                        let prev_out = &mut outgoing[prev_seg_id as usize];
                        if let Some(pos) = prev_out.iter().position(|&s| s == old_id) {
                            prev_out.remove(pos);
                        }
                        incoming_count[current_seg_id as usize] -= 1;
                        outgoing[prev_seg_id as usize].push(new_id);
                        incoming_count.push(1);
                    } else {
                        incoming_count.push(0);
                    }
                    new_id
                } else {
                    current_seg_id
                };

                // Add dependency current → next (if next exists in layer).
                let next_cursor = cursor + 1;
                let next_node = nodes.get(next_cursor).copied();
                if let Some(next_nid) = next_node {
                    let next_seg_id = self.node_to_seg[&next_nid];
                    outgoing[active_seg_id as usize].push(next_seg_id);
                    incoming_count[next_seg_id as usize] += 1;
                }

                // Update segment's layer tracking.
                {
                    let active_seg = &mut self.segments[active_seg_id as usize];
                    active_seg.last_layer = layer_index_i;
                    active_seg.index_in_last_layer = index_in_layer;
                }
                index_in_layer += 1;

                previous_node = Some(current);
                cursor = next_cursor;
                current_node = next_node;
            }
        }
    }

    fn create_unbalanced_placement(&self, graph: &mut LGraph) {
        let num_layers = graph.layers.len();
        let mut node_count = vec![0i32; num_layers];
        let mut recent_node: Vec<Option<NodeId>> = vec![None; num_layers];

        // Zero layer heights so we can grow them as we place segments.
        for layer in &mut graph.layers {
            layer.size.y = 0.0;
        }

        let edge_edge = graph.options.spacing.edge_edge;

        // Iterate segments in their ranked order (which is already 0..n).
        for seg_idx in 0..self.segments.len() {
            // Determine uppermost placement across all layers the segment visits.
            let mut uppermost = 0.0f64;
            for node_pos in 0..self.segments[seg_idx].nodes.len() {
                let nid = self.segments[seg_idx].nodes[node_pos];
                let layer_idx = graph.node(nid).layer.expect("node missing layer");
                node_count[layer_idx] += 1;

                let mut spacing = edge_edge;
                if node_count[layer_idx] > 0
                    && let Some(prev_nid) = recent_node[layer_idx]
                {
                    spacing = vertical_spacing_pair(graph, prev_nid, nid);
                }
                let candidate = graph.layers[layer_idx].size.y + spacing;
                if candidate > uppermost {
                    uppermost = candidate;
                }
            }

            // Apply uppermost placement.
            for node_pos in 0..self.segments[seg_idx].nodes.len() {
                let nid = self.segments[seg_idx].nodes[node_pos];
                let node = graph.node(nid);
                let layer_idx = node.layer.expect("node missing layer");
                let margin_top = node.margin.top;
                let size_y = node.size.y;
                let margin_bottom = node.margin.bottom;

                graph.node_mut(nid).position.y = uppermost + margin_top;
                graph.layers[layer_idx].size.y = uppermost + margin_top + size_y + margin_bottom;
                recent_node[layer_idx] = Some(nid);
            }
        }
    }

    fn balance_placement(&mut self, graph: &mut LGraph) {
        let dampening = graph.options.node_placement_linear_segments_deflection_dampening;
        let thoroughness = graph.options.thoroughness.max(1) as f64;
        let mut pendulum_iters = PENDULUM_ITERS;
        let final_iters_base = FINAL_ITERS;
        let threshold = THRESHOLD_FACTOR / thoroughness;

        let mut mode = Mode::ForwPendulum;
        let mut last_total: f64 = i32::MAX as f64;
        let mut ready = false;
        let mut final_iters = final_iters_base;

        loop {
            let incoming = mode != Mode::BackwPendulum;
            let outgoing = mode != Mode::ForwPendulum;

            // Clear regions, recompute deflections.
            let mut total = 0.0f64;
            for idx in 0..self.segments.len() {
                self.segments[idx].ref_segment = None;
            }
            for idx in 0..self.segments.len() {
                self.calc_deflection(graph, idx, incoming, outgoing, dampening);
                total += self.segments[idx].deflection.abs();
            }

            // Merge overlapping regions until stable.
            while self.merge_regions(graph) {}

            // Apply each region's deflection to its member segments. We index
            // into `self.segments[idx].nodes` each iteration so the inner loop
            // keeps using an immutable borrow of `self` while `graph.node_mut`
            // takes a disjoint mutable borrow of the graph.
            for idx in 0..self.segments.len() {
                let defl = self.region_deflection(idx);
                if defl == 0.0 {
                    continue;
                }
                let nn = self.segments[idx].nodes.len();
                for ni in 0..nn {
                    let nid = self.segments[idx].nodes[ni];
                    graph.node_mut(nid).position.y += defl;
                }
            }

            // Mode transition.
            match mode {
                Mode::ForwPendulum | Mode::BackwPendulum => {
                    pendulum_iters -= 1;
                    if pendulum_iters <= 0
                        && (total < last_total || -pendulum_iters as f64 > thoroughness)
                    {
                        mode = Mode::Rubber;
                        last_total = i32::MAX as f64;
                    } else if mode == Mode::ForwPendulum {
                        mode = Mode::BackwPendulum;
                        last_total = total;
                    } else {
                        mode = Mode::ForwPendulum;
                        last_total = total;
                    }
                }
                Mode::Rubber => {
                    ready = total >= last_total || last_total - total < threshold;
                    last_total = total;
                    if ready {
                        final_iters -= 1;
                    }
                }
            }
            if ready && final_iters <= 0 {
                break;
            }
        }
    }

    fn calc_deflection(
        &mut self,
        graph: &LGraph,
        seg_idx: usize,
        incoming: bool,
        outgoing: bool,
        dampening: f64,
    ) {
        let mut segment_deflection = 0.0f64;
        let mut node_weight_sum = 0i32;
        let segment_id = self.segments[seg_idx].id;

        // Index instead of cloning the node list: `self.segments[seg_idx].nodes`
        // is not mutated inside this loop (only `self.segments[seg_idx]` fields
        // `deflection`/`weight` are written after the loop exits), so we can
        // keep borrowing it through `&self.segments[seg_idx]`.
        let nn = self.segments[seg_idx].nodes.len();
        for ni in 0..nn {
            let nid = self.segments[seg_idx].nodes[ni];
            let node_pos_y = graph.node(nid).position.y;
            let input_prio = if incoming {
                *self.input_prio.get(&nid).unwrap_or(&NO_EDGE_PRIO)
            } else {
                NO_EDGE_PRIO
            };
            let output_prio = if outgoing {
                *self.output_prio.get(&nid).unwrap_or(&NO_EDGE_PRIO)
            } else {
                NO_EDGE_PRIO
            };
            let min_prio = input_prio.max(output_prio);

            let mut node_deflection = 0.0f64;
            let mut edge_weight_sum = 0i32;

            for &pid in graph.node(nid).ports.iter() {
                let port = graph.port(pid);
                let portpos = node_pos_y + port.position.y + port.anchor.y;

                if outgoing {
                    for &eid in port.outgoing_edges.iter() {
                        let edge = graph.edge(eid);
                        let other_pid = edge.target;
                        let other_port = graph.port(other_pid);
                        let other_nid = other_port.owner;
                        let other_seg_id = self.node_to_seg[&other_nid];
                        if other_seg_id == segment_id {
                            continue;
                        }
                        let other_in = *self.input_prio.get(&other_nid).unwrap_or(&NO_EDGE_PRIO);
                        let other_out = *self.output_prio.get(&other_nid).unwrap_or(&NO_EDGE_PRIO);
                        let other_prio = other_in.max(other_out);
                        let prio = edge.properties.get(&PRIORITY_STRAIGHTNESS);
                        if prio >= min_prio && prio >= other_prio {
                            let other_node_y = graph.node(other_nid).position.y;
                            node_deflection +=
                                other_node_y + other_port.position.y + other_port.anchor.y
                                    - portpos;
                            edge_weight_sum += 1;
                        }
                    }
                }

                if incoming {
                    for &eid in port.incoming_edges.iter() {
                        let edge = graph.edge(eid);
                        let other_pid = edge.source;
                        let other_port = graph.port(other_pid);
                        let other_nid = other_port.owner;
                        let other_seg_id = self.node_to_seg[&other_nid];
                        if other_seg_id == segment_id {
                            continue;
                        }
                        let other_in = *self.input_prio.get(&other_nid).unwrap_or(&NO_EDGE_PRIO);
                        let other_out = *self.output_prio.get(&other_nid).unwrap_or(&NO_EDGE_PRIO);
                        let other_prio = other_in.max(other_out);
                        let prio = edge.properties.get(&PRIORITY_STRAIGHTNESS);
                        if prio >= min_prio && prio >= other_prio {
                            let other_node_y = graph.node(other_nid).position.y;
                            node_deflection +=
                                other_node_y + other_port.position.y + other_port.anchor.y
                                    - portpos;
                            edge_weight_sum += 1;
                        }
                    }
                }
            }

            if edge_weight_sum > 0 {
                segment_deflection += node_deflection / edge_weight_sum as f64;
                node_weight_sum += 1;
            }
        }

        let seg = &mut self.segments[seg_idx];
        if node_weight_sum > 0 {
            seg.deflection = dampening * segment_deflection / node_weight_sum as f64;
            seg.weight = node_weight_sum;
        } else {
            seg.deflection = 0.0;
            seg.weight = 0;
        }
    }

    fn region_deflection(&self, seg_idx: usize) -> f64 {
        let mut cur = seg_idx as SegIdx;
        while let Some(nxt) = self.segments[cur as usize].ref_segment {
            cur = nxt;
        }
        self.segments[cur as usize].deflection
    }

    fn region_of(&self, seg_idx: SegIdx) -> SegIdx {
        let mut cur = seg_idx;
        while let Some(nxt) = self.segments[cur as usize].ref_segment {
            cur = nxt;
        }
        cur
    }

    fn merge_regions(&mut self, graph: &LGraph) -> bool {
        let mut changed = false;
        let node_node = graph.options.spacing.node_node;
        let threshold = OVERLAP_DETECT * node_node;

        for layer in &graph.layers {
            let nodes = &layer.nodes;
            if nodes.len() < 2 {
                continue;
            }
            for pair_idx in 0..nodes.len() - 1 {
                let n1 = nodes[pair_idx];
                let n2 = nodes[pair_idx + 1];
                let seg1 = self.node_to_seg[&n1];
                let seg2 = self.node_to_seg[&n2];
                let region1 = self.region_of(seg1);
                let region2 = self.region_of(seg2);
                if region1 == region2 {
                    continue;
                }

                let spacing = vertical_spacing_pair(graph, n1, n2);
                let n1_data = graph.node(n1);
                let n2_data = graph.node(n2);
                let r1_deflection = self.segments[region1 as usize].deflection;
                let r2_deflection = self.segments[region2 as usize].deflection;

                let n1_extent = n1_data.position.y
                    + n1_data.size.y
                    + n1_data.margin.bottom
                    + r1_deflection
                    + spacing;
                let n2_extent = n2_data.position.y - n2_data.margin.top + r2_deflection;

                if n1_extent > n2_extent + threshold {
                    let w1 = self.segments[region1 as usize].weight;
                    let w2 = self.segments[region2 as usize].weight;
                    let weight_sum = w1 + w2;
                    debug_assert!(weight_sum > 0);
                    let new_deflection =
                        (w2 as f64 * r2_deflection + w1 as f64 * r1_deflection) / weight_sum as f64;
                    let r2 = &mut self.segments[region2 as usize];
                    r2.deflection = new_deflection;
                    r2.weight = weight_sum;
                    self.segments[region1 as usize].ref_segment = Some(region2);
                    changed = true;
                }
            }
        }
        changed
    }

    fn post_process(&self, graph: &mut LGraph) {
        for seg in &self.segments {
            if seg.nodes.is_empty() {
                continue;
            }
            let (min_room_above, min_room_below) = self.segment_room(graph, seg);

            let mut min_displacement = i32::MAX as f64;
            let mut found = false;

            // Incoming edges on the first node.
            let first = seg.nodes[0];
            for &pid in graph.node(first).ports.iter() {
                let port = graph.port(pid);
                let pos = graph.node(first).position.y + port.position.y + port.anchor.y;
                for &eid in port.incoming_edges.iter() {
                    let edge = graph.edge(eid);
                    let src_pid = edge.source;
                    let src_port = graph.port(src_pid);
                    let src_nid = src_port.owner;
                    let d =
                        graph.node(src_nid).position.y + src_port.position.y + src_port.anchor.y
                            - pos;
                    if d.abs() < min_displacement.abs()
                        && d.abs() < (if d < 0.0 { min_room_above } else { min_room_below })
                    {
                        min_displacement = d;
                        found = true;
                    }
                }
            }

            // Outgoing edges on the last node.
            let last = *seg.nodes.last().unwrap();
            for &pid in graph.node(last).ports.iter() {
                let port = graph.port(pid);
                let pos = graph.node(last).position.y + port.position.y + port.anchor.y;
                for &eid in port.outgoing_edges.iter() {
                    let edge = graph.edge(eid);
                    let tgt_pid = edge.target;
                    let tgt_port = graph.port(tgt_pid);
                    let tgt_nid = tgt_port.owner;
                    let d =
                        graph.node(tgt_nid).position.y + tgt_port.position.y + tgt_port.anchor.y
                            - pos;
                    if d.abs() < min_displacement.abs()
                        && d.abs() < (if d < 0.0 { min_room_above } else { min_room_below })
                    {
                        min_displacement = d;
                        found = true;
                    }
                }
            }

            if found && min_displacement != 0.0 {
                for &nid in &seg.nodes {
                    graph.node_mut(nid).position.y += min_displacement;
                }
            }
        }
    }

    fn segment_room(&self, graph: &LGraph, seg: &LinearSegment) -> (f64, f64) {
        let mut min_above = i32::MAX as f64;
        let mut min_below = i32::MAX as f64;
        for &nid in &seg.nodes {
            let node = graph.node(nid);
            let layer_idx = node.layer.expect("node missing layer");
            let layer = &graph.layers[layer_idx];
            let pos_in_layer =
                layer.nodes.iter().position(|&n| n == nid).expect("node missing from layer");

            let room_above = if pos_in_layer > 0 {
                let neighbor = graph.node(layer.nodes[pos_in_layer - 1]);
                let spacing = vertical_spacing_pair(graph, nid, layer.nodes[pos_in_layer - 1]);
                node.position.y
                    - node.margin.top
                    - (neighbor.position.y + neighbor.size.y + neighbor.margin.bottom + spacing)
            } else {
                node.position.y - node.margin.top
            };
            if room_above < min_above {
                min_above = room_above;
            }

            let room_below = if pos_in_layer + 1 < layer.nodes.len() {
                let neighbor = graph.node(layer.nodes[pos_in_layer + 1]);
                let spacing = vertical_spacing_pair(graph, nid, layer.nodes[pos_in_layer + 1]);
                neighbor.position.y
                    - neighbor.margin.top
                    - (node.position.y + node.size.y + node.margin.bottom + spacing)
            } else {
                2.0 * node.position.y
            };
            if room_below < min_below {
                min_below = room_below;
            }
        }
        (min_above, min_below)
    }

    fn finalize_graph_height(&self, graph: &mut LGraph) {
        let mut max_height = 0.0f64;
        for layer_idx in 0..graph.layers.len() {
            let mut layer_bottom = 0.0f64;
            let nodes: Vec<NodeId> = graph.layers[layer_idx].nodes.clone();
            for nid in nodes {
                let node = graph.node(nid);
                let bottom = node.position.y + node.size.y + node.margin.bottom;
                if bottom > layer_bottom {
                    layer_bottom = bottom;
                }
            }
            graph.layers[layer_idx].size.y = layer_bottom;
            if layer_bottom > max_height {
                max_height = layer_bottom;
            }
        }
        graph.size.y = max_height;
    }
}

/// Return the vertical spacing between two nodes based on their types,
/// covering the NodeType pairs reachable during layered layout.
fn vertical_spacing_pair(graph: &LGraph, n1: NodeId, n2: NodeId) -> f64 {
    let t1 = graph.node(n1).node_type;
    let t2 = graph.node(n2).node_type;
    let sp = &graph.options.spacing;
    use NodeType::*;
    match (t1, t2) {
        (Normal, Normal) => sp.node_node,
        (Normal, LongEdge) | (LongEdge, Normal) => sp.edge_node,
        (Normal, NorthSouthPort) | (NorthSouthPort, Normal) => sp.edge_node,
        (Normal, ExternalPort) | (ExternalPort, Normal) => sp.edge_node,
        (Normal, Label) | (Label, Normal) => sp.node_node,
        (LongEdge, LongEdge) => sp.edge_edge,
        (LongEdge, NorthSouthPort) | (NorthSouthPort, LongEdge) => sp.edge_edge,
        (LongEdge, ExternalPort) | (ExternalPort, LongEdge) => sp.edge_edge,
        (LongEdge, Label) | (Label, LongEdge) => sp.edge_node,
        (NorthSouthPort, NorthSouthPort) => sp.edge_edge,
        (NorthSouthPort, ExternalPort) | (ExternalPort, NorthSouthPort) => sp.edge_edge,
        (NorthSouthPort, Label) | (Label, NorthSouthPort) => sp.label_node,
        (ExternalPort, ExternalPort) => sp.port_port,
        (ExternalPort, Label) | (Label, ExternalPort) => sp.label_port_vertical,
        (Label, Label) => sp.edge_edge,
        (BreakingPoint, BreakingPoint) => sp.edge_edge,
        (BreakingPoint, Normal) | (Normal, BreakingPoint) => sp.edge_node,
        (BreakingPoint, LongEdge) | (LongEdge, BreakingPoint) => sp.edge_node,
        (BreakingPoint, Label) | (Label, BreakingPoint) => sp.edge_node,
        _ => sp.edge_edge,
    }
}
