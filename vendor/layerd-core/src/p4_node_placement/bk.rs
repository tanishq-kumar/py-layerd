//! Brandes-Köpf node placement.
//!
//! Reference: Brandes & Köpf, "Fast and Simple Horizontal Coordinate
//! Assignment", GD 2001.

use std::collections::VecDeque;

use smallvec::SmallVec;

use super::threshold_strategy::ThresholdStrategy;
use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
    },
    properties::internal::{
        PRIORITY_STRAIGHTNESS, SPACING_EDGE_EDGE_OVERRIDE, SPACING_EDGE_NODE_OVERRIDE,
        SPACING_NODE_NODE_OVERRIDE,
    },
};

type NeighborList = SmallVec<(usize, EdgeId), 2>;
const MISSING_NODE_IDX: usize = usize::MAX;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum VDir {
    Down,
    Up,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum HDir {
    Right,
    Left,
}

/// Precomputed neighbor/index data used by all four alignment passes.
pub(super) struct Neighborhood {
    /// Total number of nodes (length of all index-keyed arrays).
    pub(super) n: usize,
    /// Maps the `ArenaId` slot of a `NodeId` to its sequential index (0..n).
    ///
    /// Lookups still validate against `idx_to_node` so reused arena slots and
    /// cross-graph ids do not alias.
    pub(super) node_to_idx: Vec<usize>,
    /// Inverse of `node_to_idx`.
    pub(super) idx_to_node: Vec<NodeId>,
    /// Position of each node within its layer.
    pub(super) node_index: Vec<usize>,
    /// Layer index of each node.
    pub(super) layer_index: Vec<usize>,
    upper_neighbor: Vec<usize>,
    lower_neighbor: Vec<usize>,
    node_type: Vec<NodeType>,
    node_margin_top: Vec<f64>,
    node_bottom_extent: Vec<f64>,
    /// Vertical spacing to the previous node in the same layer. Entry is zero
    /// for the first node in each layer.
    vertical_spacing_before: Vec<f64>,
    /// Left (incoming-side) neighbors sorted by position. Each entry is
    /// `(neighbor_idx, connecting_edge_id)`.
    pub(super) left_neighbors: Vec<NeighborList>,
    /// Right (outgoing-side) neighbors sorted by position.
    pub(super) right_neighbors: Vec<NeighborList>,
}

#[inline]
fn lookup_node_idx(node_to_idx: &[usize], idx_to_node: &[NodeId], nid: NodeId) -> Option<usize> {
    let slot = nid.arena_id().index() as usize;
    let idx = *node_to_idx.get(slot)?;
    if idx != MISSING_NODE_IDX && idx_to_node.get(idx).copied() == Some(nid) {
        Some(idx)
    } else {
        None
    }
}

impl Neighborhood {
    #[inline]
    pub(super) fn lookup_idx(&self, nid: NodeId) -> Option<usize> {
        lookup_node_idx(&self.node_to_idx, &self.idx_to_node, nid)
    }

    #[inline]
    pub(super) fn idx(&self, nid: NodeId) -> usize {
        self.lookup_idx(nid).expect("node missing from P4 neighborhood")
    }
}

fn build_neighborhood(graph: &LGraph) -> Neighborhood {
    let mut idx_to_node: Vec<NodeId> = Vec::new();
    let mut max_node_slot = 0usize;

    // Assign sequential indices following layer order.
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            max_node_slot = max_node_slot.max(nid.arena_id().index() as usize);
            idx_to_node.push(nid);
        }
    }
    let n = idx_to_node.len();
    let mut node_to_idx =
        if n == 0 { Vec::new() } else { vec![MISSING_NODE_IDX; max_node_slot + 1] };
    for (idx, nid) in idx_to_node.iter().copied().enumerate() {
        node_to_idx[nid.arena_id().index() as usize] = idx;
    }

    let mut node_type = Vec::with_capacity(n);
    let mut node_margin_top = Vec::with_capacity(n);
    let mut node_bottom_extent = Vec::with_capacity(n);
    for &nid in &idx_to_node {
        let node = graph.node(nid);
        node_type.push(node.node_type);
        node_margin_top.push(node.margin.top);
        node_bottom_extent.push(node.size.y + node.margin.bottom);
    }

    // Node position within layer and layer index.
    let mut node_index = vec![0usize; n];
    let mut layer_index_arr = vec![0usize; n];
    for (li, layer) in graph.layers.iter().enumerate() {
        for (pos, &nid) in layer.nodes.iter().enumerate() {
            if let Some(idx) = lookup_node_idx(&node_to_idx, &idx_to_node, nid) {
                node_index[idx] = pos;
                layer_index_arr[idx] = li;
            }
        }
    }

    let mut upper_neighbor = vec![MISSING_NODE_IDX; n];
    let mut lower_neighbor = vec![MISSING_NODE_IDX; n];
    let mut vertical_spacing_before = vec![0.0; n];
    for layer in &graph.layers {
        for pair in layer.nodes.windows(2) {
            let previous = pair[0];
            let current = pair[1];
            let Some(previous_idx) = lookup_node_idx(&node_to_idx, &idx_to_node, previous) else {
                continue;
            };
            let Some(current_idx) = lookup_node_idx(&node_to_idx, &idx_to_node, current) else {
                continue;
            };
            lower_neighbor[previous_idx] = current_idx;
            upper_neighbor[current_idx] = previous_idx;
            {
                vertical_spacing_before[current_idx] =
                    vertical_spacing_between(graph, previous, current);
            }
        }
    }

    // Left neighbors: for each node, the nodes in the previous layer connected
    // via incoming edges, sorted by their position in that layer.
    let mut left_neighbors: Vec<NeighborList> = vec![NeighborList::new(); n];
    let mut right_neighbors: Vec<NeighborList> = vec![NeighborList::new(); n];

    // For each node, collect only the neighbors reachable via non-self-loop,
    // non-in-layer edges with the maximum `PRIORITY_STRAIGHTNESS` among that
    // node's incoming (resp. outgoing) edges.
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            let idx = lookup_node_idx(&node_to_idx, &idx_to_node, nid)
                .expect("layer node missing from P4 neighborhood");
            let node_layer = graph.node(nid).layer;

            let mut left_max_prio = 0i32;
            let mut left_buf = NeighborList::new();
            for eid in graph.incoming_edges(nid) {
                let edge = graph.edge(eid);
                let src_node = edge.source_owner;
                if src_node == nid {
                    continue;
                }
                if graph.node(src_node).layer == node_layer {
                    continue;
                }
                let Some(src_idx) = lookup_node_idx(&node_to_idx, &idx_to_node, src_node) else {
                    continue;
                };
                let prio = edge.properties.get(&PRIORITY_STRAIGHTNESS);
                if prio > left_max_prio {
                    left_max_prio = prio;
                    left_buf.clear();
                }
                if prio == left_max_prio {
                    left_buf.push((src_idx, eid));
                }
            }
            left_neighbors[idx] = left_buf;

            let mut right_max_prio = 0i32;
            let mut right_buf = NeighborList::new();
            for eid in graph.outgoing_edges(nid) {
                let edge = graph.edge(eid);
                let tgt_node = edge.target_owner;
                if tgt_node == nid {
                    continue;
                }
                if graph.node(tgt_node).layer == node_layer {
                    continue;
                }
                let Some(tgt_idx) = lookup_node_idx(&node_to_idx, &idx_to_node, tgt_node) else {
                    continue;
                };
                let prio = edge.properties.get(&PRIORITY_STRAIGHTNESS);
                if prio > right_max_prio {
                    right_max_prio = prio;
                    right_buf.clear();
                }
                if prio == right_max_prio {
                    right_buf.push((tgt_idx, eid));
                }
            }
            right_neighbors[idx] = right_buf;
        }
    }

    // Sort neighbors by their position within their layer.
    for list in &mut left_neighbors {
        list.sort_by_key(|&(ni, _)| node_index[ni]);
    }
    for list in &mut right_neighbors {
        list.sort_by_key(|&(ni, _)| node_index[ni]);
    }

    Neighborhood {
        n,
        node_to_idx,
        idx_to_node,
        node_index,
        layer_index: layer_index_arr,
        upper_neighbor,
        lower_neighbor,
        node_type,
        node_margin_top,
        node_bottom_extent,
        vertical_spacing_before,
        left_neighbors,
        right_neighbors,
    }
}

/// State for one of the four BK aligned layouts. The `sink`/`shift` fields
/// drive class-graph compaction; `su`/`od` track straightened and
/// dummy-only blocks for `ThresholdStrategy`.
pub(super) struct AlignedLayout {
    pub(super) root: Vec<usize>,
    pub(super) align: Vec<usize>,
    pub(super) inner_shift: Vec<f64>,
    pub(super) block_size: Vec<f64>,
    pub(super) sink: Vec<usize>,
    pub(super) shift: Vec<f64>,
    pub(super) y: Vec<f64>,
    pub(super) su: Vec<bool>,
    pub(super) od: Vec<bool>,
    pub(super) placed: Vec<bool>,
    pub(super) vdir: VDir,
    pub(super) hdir: HDir,
}

impl AlignedLayout {
    fn new(n: usize, vdir: VDir, hdir: HDir) -> Self {
        let shift_init = if vdir == VDir::Up { f64::NEG_INFINITY } else { f64::INFINITY };
        // Allocate sink and shift but do not fill them — `horizontal_compaction`
        // initializes both. `usize::MAX` is used as a sentinel so any accidental
        // read before compaction panics with an obvious out-of-bounds.
        AlignedLayout {
            root: (0..n).collect(),
            align: (0..n).collect(),
            inner_shift: vec![0.0; n],
            block_size: vec![0.0; n],
            sink: vec![usize::MAX; n],
            shift: vec![shift_init; n],
            y: vec![0.0; n],
            su: vec![false; n],
            od: vec![true; n],
            placed: vec![false; n],
            vdir,
            hdir,
        }
    }
}

/// Returns `y[tgt] - y[src]`, i.e. a positive value means `tgt` must shift up
/// (under vdir-down) to straighten the edge.
pub(super) fn calculate_delta(
    bal: &AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    src_pid: PortId,
    tgt_pid: PortId,
) -> f64 {
    let src_port = graph.port(src_pid);
    let tgt_port = graph.port(tgt_pid);
    let src_idx = ni.idx(src_port.owner);
    let tgt_idx = ni.idx(tgt_port.owner);
    let src_pos =
        bal.y[src_idx] + bal.inner_shift[src_idx] + src_port.position.y + src_port.anchor.y;
    let tgt_pos =
        bal.y[tgt_idx] + bal.inner_shift[tgt_idx] + tgt_port.position.y + tgt_port.anchor.y;
    tgt_pos - src_pos
}

/// Shifts every node of the block anchored at `root_idx` by `delta`.
pub(super) fn shift_block(bal: &mut AlignedLayout, root_idx: usize, delta: f64) {
    let mut current = root_idx;
    loop {
        bal.y[current] += delta;
        current = bal.align[current];
        if current == root_idx {
            break;
        }
    }
}

pub(super) fn get_min_y(
    bal: &AlignedLayout,
    _graph: &LGraph,
    ni: &Neighborhood,
    idx: usize,
) -> f64 {
    let root_idx = bal.root[idx];
    bal.y[root_idx] + bal.inner_shift[idx] - ni.node_margin_top[idx]
}

pub(super) fn get_max_y(
    bal: &AlignedLayout,
    _graph: &LGraph,
    ni: &Neighborhood,
    idx: usize,
) -> f64 {
    let root_idx = bal.root[idx];
    bal.y[root_idx] + bal.inner_shift[idx] + ni.node_bottom_extent[idx]
}

fn upper_neighbor_idx(ni: &Neighborhood, _graph: &LGraph, idx: usize) -> Option<usize> {
    let nbr = ni.upper_neighbor[idx];
    if nbr == MISSING_NODE_IDX { None } else { Some(nbr) }
}

fn lower_neighbor_idx(ni: &Neighborhood, _graph: &LGraph, idx: usize) -> Option<usize> {
    let nbr = ni.lower_neighbor[idx];
    if nbr == MISSING_NODE_IDX { None } else { Some(nbr) }
}

/// Returns the maximum distance (up to `delta`) the block with root `root_idx`
/// can be shifted upward without overlapping any block's upper neighbor.
///
/// Per-pair spacing comes from `vertical_spacing(current, neighbor)`, not
/// a uniform constant. A uniform spacing over-spaces LongEdge /
/// NorthSouthPort / Label dummy pairs (the table maps them to
/// `SPACING_EDGE_NODE`/`SPACING_EDGE_EDGE`, both 10 by default, vs
/// `SPACING_NODE_NODE` 20). The mismatch left the threshold-strategy
/// `available_space` short, so deferred blocks were not shifted enough
/// and ended up 10 px below the correct position.
pub(super) fn check_space_above(
    bal: &AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    root_idx: usize,
    delta: f64,
) -> f64 {
    let mut available = delta;
    let mut current = root_idx;
    loop {
        current = bal.align[current];
        let min_y_current = get_min_y(bal, graph, ni, current);
        if let Some(nbr_idx) = upper_neighbor_idx(ni, graph, current) {
            let max_y_neighbor = get_max_y(bal, graph, ni, nbr_idx);
            let pair_spacing = adjacent_vertical_spacing(ni, current, nbr_idx);
            let candidate = min_y_current - (max_y_neighbor + pair_spacing);
            if candidate < available {
                available = candidate;
            }
        }
        if current == root_idx {
            break;
        }
    }
    available
}

/// Returns the maximum distance (up to `delta`) the block with root `root_idx`
/// can be shifted downward without overlapping any block's lower neighbor.
///
/// See `check_space_above` for the rationale behind the per-pair spacing
/// lookup.
pub(super) fn check_space_below(
    bal: &AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    root_idx: usize,
    delta: f64,
) -> f64 {
    let mut available = delta;
    let mut current = root_idx;
    loop {
        current = bal.align[current];
        let max_y_current = get_max_y(bal, graph, ni, current);
        if let Some(nbr_idx) = lower_neighbor_idx(ni, graph, current) {
            let min_y_neighbor = get_min_y(bal, graph, ni, nbr_idx);
            let pair_spacing = adjacent_vertical_spacing(ni, current, nbr_idx);
            let candidate = min_y_neighbor - (max_y_current + pair_spacing);
            if candidate < available {
                available = candidate;
            }
        }
        if current == root_idx {
            break;
        }
    }
    available
}

/// Returns true if `idx` is a long-edge dummy with an incoming edge from
/// another long-edge dummy in the specified layer.
fn incident_to_inner_segment(
    idx: usize,
    node_layer: usize,
    prev_layer: usize,
    ni: &Neighborhood,
    _graph: &LGraph,
) -> bool {
    if ni.node_type[idx] != NodeType::LongEdge {
        return false;
    }
    for &(nbr_idx, _eid) in &ni.left_neighbors[idx] {
        if ni.node_type[nbr_idx] == NodeType::LongEdge
            && ni.layer_index[nbr_idx] == prev_layer
            && ni.layer_index[idx] == node_layer
        {
            return true;
        }
    }
    false
}

fn mark_conflicts(graph: &LGraph, ni: &Neighborhood) -> hashbrown::HashSet<EdgeId> {
    let mut marked = hashbrown::HashSet::new();
    let num_layers = graph.layers.len();
    if num_layers < 3 {
        return marked;
    }

    for i in 1..num_layers - 1 {
        let layer_above = &graph.layers[i];
        let layer_below = &graph.layers[i + 1];
        let layer_above_size = layer_above.nodes.len();

        let mut k0: usize = 0;
        let mut l: usize = 0;

        for l1 in 0..layer_below.nodes.len() {
            let v_nid = layer_below.nodes[l1];
            let v_idx = ni.idx(v_nid);

            let v_inner_segment = incident_to_inner_segment(v_idx, i + 1, i, ni, graph);
            if l1 == layer_below.nodes.len() - 1 || v_inner_segment {
                let mut k1 = layer_above_size - 1;
                if v_inner_segment {
                    // k1 = position of the first left neighbor (inner segment source)
                    if let Some(&(nbr_idx, _)) = ni.left_neighbors[v_idx].first() {
                        k1 = ni.node_index[nbr_idx];
                    }
                }

                while l <= l1 {
                    let vl_nid = layer_below.nodes[l];
                    let vl_idx = ni.idx(vl_nid);

                    if !incident_to_inner_segment(vl_idx, i + 1, i, ni, graph) {
                        for &(nbr_idx, eid) in &ni.left_neighbors[vl_idx] {
                            let k = ni.node_index[nbr_idx];
                            if k < k0 || k > k1 {
                                marked.insert(eid);
                            }
                        }
                    }
                    l += 1;
                }
                k0 = k1;
            }
        }
    }

    marked
}

fn vertical_alignment(
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    marked: &hashbrown::HashSet<EdgeId>,
) {
    // Reset
    for i in 0..ni.n {
        bal.root[i] = i;
        bal.align[i] = i;
        bal.inner_shift[i] = 0.0;
    }

    if bal.hdir == HDir::Left {
        for li in (0..graph.layers.len()).rev() {
            vertical_alignment_layer(bal, graph, ni, marked, li);
        }
    } else {
        for li in 0..graph.layers.len() {
            vertical_alignment_layer(bal, graph, ni, marked, li);
        }
    }
}

fn vertical_alignment_layer(
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    marked: &hashbrown::HashSet<EdgeId>,
    li: usize,
) {
    let mut r: i64 = if bal.vdir == VDir::Up { i64::MAX } else { -1 };
    let layer = &graph.layers[li];
    if bal.vdir == VDir::Up {
        for &nid in layer.nodes.iter().rev() {
            let Some(v) = ni.lookup_idx(nid) else { continue };
            vertical_alignment_node(bal, graph, ni, marked, v, &mut r);
        }
    } else {
        for &nid in &layer.nodes {
            let Some(v) = ni.lookup_idx(nid) else { continue };
            vertical_alignment_node(bal, graph, ni, marked, v, &mut r);
        }
    }
}

fn vertical_alignment_node(
    bal: &mut AlignedLayout,
    _graph: &LGraph,
    ni: &Neighborhood,
    marked: &hashbrown::HashSet<EdgeId>,
    v: usize,
    r: &mut i64,
) {
    let neighbors: &[(usize, EdgeId)] = if bal.hdir == HDir::Left {
        &ni.right_neighbors[v]
    } else {
        &ni.left_neighbors[v]
    };

    if neighbors.is_empty() {
        return;
    }

    let d = neighbors.len();
    let low = (((d + 1) as f64 / 2.0).floor() as usize).saturating_sub(1);
    let high = (((d + 1) as f64 / 2.0).ceil() as usize).saturating_sub(1);

    let v_is_long_edge = ni.node_type[v] == NodeType::LongEdge;
    if bal.vdir == VDir::Up {
        for m in (low..=high).rev() {
            let (u, eid) = neighbors[m];
            if bal.align[v] == v && !marked.contains(&eid) && (ni.node_index[u] as i64) < *r {
                bal.align[u] = v;
                bal.root[v] = bal.root[u];
                bal.align[v] = bal.root[v];
                let root = bal.root[v];
                bal.od[root] &= v_is_long_edge;
                *r = ni.node_index[u] as i64;
            }
        }
    } else {
        for &(u, eid) in neighbors.iter().take(high + 1).skip(low) {
            if bal.align[v] == v && !marked.contains(&eid) && (ni.node_index[u] as i64) > *r {
                bal.align[u] = v;
                bal.root[v] = bal.root[u];
                bal.align[v] = bal.root[v];
                let root = bal.root[v];
                bal.od[root] &= v_is_long_edge;
                *r = ni.node_index[u] as i64;
            }
        }
    }
}

fn inside_block_shift(bal: &mut AlignedLayout, graph: &LGraph, ni: &Neighborhood) {
    // Identify all block roots.
    let mut visited = vec![false; ni.n];
    for i in 0..ni.n {
        let root = bal.root[i];
        if visited[root] {
            continue;
        }
        visited[root] = true;

        let mut space_above = ni.node_margin_top[root];
        let mut space_below = ni.node_bottom_extent[root];
        bal.inner_shift[root] = 0.0;

        // Walk the ring: align[root] -> align[...] -> ... -> root
        let mut current = root;
        loop {
            let next = bal.align[current];
            if next == root {
                break;
            }

            // Find the connecting edge between current and next.
            let current_nid = ni.idx_to_node[current];
            let next_nid = ni.idx_to_node[next];
            let port_pos_diff = find_port_diff(graph, current_nid, next_nid, bal.hdir);

            let next_inner_shift = bal.inner_shift[current] + port_pos_diff;
            bal.inner_shift[next] = next_inner_shift;

            space_above = space_above.max(ni.node_margin_top[next] - next_inner_shift);
            space_below = space_below.max(next_inner_shift + ni.node_bottom_extent[next]);

            current = next;
        }

        // Adjust inner shifts so they're relative to the block's top border.
        let mut cur = root;
        loop {
            bal.inner_shift[cur] += space_above;
            cur = bal.align[cur];
            if cur == root {
                break;
            }
        }

        bal.block_size[root] = space_above + space_below;
    }
}

/// Find the port-position difference for the edge connecting two aligned nodes.
///
/// Iterates `source` ports yielding incoming edges before outgoing ones.
/// When two aligned block members are joined by more than one edge
/// (common with long-edge chains after long-edge splitting), the
/// edge incident on `from`'s incoming-port side wins. Walking outgoing
/// first picks a different port pair, giving a different `port_pos_diff`
/// and a different `inner_shift`.
fn find_port_diff(graph: &LGraph, from_nid: NodeId, to_nid: NodeId, hdir: HDir) -> f64 {
    for &port_id in &graph.node(from_nid).ports {
        let port = graph.port(port_id);
        for &eid in port.incoming_edges.iter().chain(port.outgoing_edges.iter()) {
            let edge = graph.edge(eid);
            let other_owner =
                if edge.source == port_id { edge.target_owner } else { edge.source_owner };
            if other_owner != to_nid {
                continue;
            }
            let src_port = graph.port(edge.source);
            let tgt_port = graph.port(edge.target);
            let diff = if hdir == HDir::Left {
                (tgt_port.position.y + tgt_port.anchor.y)
                    - (src_port.position.y + src_port.anchor.y)
            } else {
                (src_port.position.y + src_port.anchor.y)
                    - (tgt_port.position.y + tgt_port.anchor.y)
            };
            return diff;
        }
    }
    0.0
}

/// Node of the class graph. Each class is identified by a block sink (see
/// `bal.sink`); `node_idx` stores that sink's node index so the resolved
/// shift can be written back to `bal.shift[node_idx]`.
struct ClassNode {
    node_idx: usize,
    out_edges: Vec<ClassEdge>,
    /// Resolved per-class shift; `None` until a longest-path pass touches it.
    class_shift: Option<f64>,
    indegree: i32,
}

/// Directed edge in the class graph carrying the minimum required separation
/// between two classes.
struct ClassEdge {
    target: usize,
    separation: f64,
}

/// Look up the class node for a sink, creating it lazily.
fn get_or_create_class_node(
    sink_idx: usize,
    class_nodes: &mut Vec<ClassNode>,
    class_lookup: &mut [i32],
) -> usize {
    let cached = class_lookup[sink_idx];
    if cached >= 0 {
        return cached as usize;
    }
    let idx = class_nodes.len();
    class_nodes.push(ClassNode {
        node_idx: sink_idx,
        out_edges: Vec::new(),
        class_shift: None,
        indegree: 0,
    });
    class_lookup[sink_idx] = idx as i32;
    idx
}

/// Append a class graph edge and increment the target's indegree.
fn add_class_edge(class_nodes: &mut [ClassNode], source: usize, target: usize, separation: f64) {
    class_nodes[source].out_edges.push(ClassEdge { target, separation });
    class_nodes[target].indegree += 1;
}

/// Resolve per-class shifts via longest-path layering on the class graph.
///
/// Iterates class nodes in topological order starting from those with
/// `indegree == 0`. Under `VDir::Down` the separation is minimised; under
/// `VDir::Up` it is maximised.
fn place_classes(bal: &mut AlignedLayout, class_nodes: &mut [ClassNode]) {
    let mut queue: VecDeque<usize> = VecDeque::new();
    for (i, cn) in class_nodes.iter().enumerate() {
        if cn.indegree == 0 {
            queue.push_back(i);
        }
    }

    while let Some(idx) = queue.pop_front() {
        // Anchor the class at zero the first time it is touched.
        if class_nodes[idx].class_shift.is_none() {
            class_nodes[idx].class_shift = Some(0.0);
        }
        let n_shift = class_nodes[idx].class_shift.expect("class_shift was just set");

        // Snapshot the outgoing edges to side-step the mutable borrow on class_nodes.
        let edges: Vec<(usize, f64)> =
            class_nodes[idx].out_edges.iter().map(|e| (e.target, e.separation)).collect();

        for (target, separation) in edges {
            let candidate = n_shift + separation;
            let merged = match class_nodes[target].class_shift {
                Some(existing) if bal.vdir == VDir::Down => existing.min(candidate),
                Some(existing) => existing.max(candidate),
                _ => candidate,
            };
            class_nodes[target].class_shift = Some(merged);
            class_nodes[target].indegree -= 1;
            if class_nodes[target].indegree == 0 {
                queue.push_back(target);
            }
        }
    }

    // Commit every resolved class shift to the layout's per-sink shift array.
    for cn in class_nodes.iter() {
        if let Some(shift) = cn.class_shift {
            bal.shift[cn.node_idx] = shift;
        }
    }
}

/// Returns the vertical spacing required between two adjacent nodes in the
/// same layer. Maps every (NodeType, NodeType) pair to one of the
/// `SPACING_*` graph properties:
///
///   NORMAL × NORMAL          → SPACING_NODE_NODE         (default 20)
///   NORMAL × LONG_EDGE       → SPACING_EDGE_NODE         (default 10)
///   NORMAL × NORTH_SOUTH_PORT→ SPACING_EDGE_NODE
///   NORMAL × EXTERNAL_PORT   → SPACING_EDGE_NODE
///   NORMAL × LABEL           → SPACING_NODE_NODE
///   LONG_EDGE × LONG_EDGE    → SPACING_EDGE_EDGE         (default 10)
///   LONG_EDGE × NORTH_SOUTH_PORT → SPACING_EDGE_EDGE
///   LONG_EDGE × EXTERNAL_PORT → SPACING_EDGE_EDGE
///   LONG_EDGE × LABEL        → SPACING_EDGE_NODE
///   NORTH_SOUTH × NORTH_SOUTH → SPACING_EDGE_EDGE
///   NORTH_SOUTH × EXTERNAL_PORT → SPACING_EDGE_EDGE
///   NORTH_SOUTH × LABEL       → SPACING_LABEL_NODE
///   EXTERNAL × EXTERNAL       → SPACING_PORT_PORT
///   LABEL × LABEL             → SPACING_EDGE_EDGE
///   BREAKING_POINT × BREAKING_POINT → SPACING_EDGE_EDGE
///   BREAKING_POINT × NORMAL   → SPACING_EDGE_NODE
///   BREAKING_POINT × LABEL    → SPACING_EDGE_NODE
///   BREAKING_POINT × LONG_EDGE→ SPACING_EDGE_NODE
fn vertical_spacing_between(graph: &LGraph, n1: NodeId, n2: NodeId) -> f64 {
    let t1 = graph.node(n1).node_type;
    let t2 = graph.node(n2).node_type;
    let (base, override_key) = vertical_spacing_with_override_key(graph, t1, t2);
    let Some(key) = override_key else {
        return base;
    };

    let s1 = graph.node(n1).properties.get_copy(key).unwrap_or(base);
    let s2 = graph.node(n2).properties.get_copy(key).unwrap_or(base);
    s1.max(s2)
}

fn vertical_spacing_with_override_key(
    graph: &LGraph,
    t1: NodeType,
    t2: NodeType,
) -> (f64, Option<&'static crate::properties::PropertyKey<Option<f64>>>) {
    use NodeType as NT;
    let s = &graph.options.spacing;
    let (a, b) = if (t1 as u8) <= (t2 as u8) { (t1, t2) } else { (t2, t1) };
    match (a, b) {
        (NT::Normal, NT::Normal) | (NT::Normal, NT::Label) =>
            (s.node_node, Some(&SPACING_NODE_NODE_OVERRIDE)),
        (NT::Normal, NT::LongEdge)
        | (NT::Normal, NT::NorthSouthPort)
        | (NT::Normal, NT::ExternalPort)
        | (NT::LongEdge, NT::Label)
        | (NT::Normal, NT::BreakingPoint)
        | (NT::LongEdge, NT::BreakingPoint)
        | (NT::Label, NT::BreakingPoint) => (s.edge_node, Some(&SPACING_EDGE_NODE_OVERRIDE)),
        (NT::LongEdge, NT::LongEdge)
        | (NT::LongEdge, NT::NorthSouthPort)
        | (NT::LongEdge, NT::ExternalPort)
        | (NT::NorthSouthPort, NT::NorthSouthPort)
        | (NT::NorthSouthPort, NT::ExternalPort)
        | (NT::Label, NT::Label)
        | (NT::BreakingPoint, NT::BreakingPoint) =>
            (s.edge_edge, Some(&SPACING_EDGE_EDGE_OVERRIDE)),
        (NT::NorthSouthPort, NT::Label) => (s.label_node, None),
        (NT::ExternalPort, NT::ExternalPort) => (s.port_port, None),
        (NT::ExternalPort, NT::Label) => (s.label_port_vertical, None),
        _ => (s.node_node, None),
    }
}

fn adjacent_vertical_spacing(ni: &Neighborhood, idx: usize, neighbor_idx: usize) -> f64 {
    if ni.node_index[idx] > ni.node_index[neighbor_idx] {
        ni.vertical_spacing_before[idx]
    } else {
        ni.vertical_spacing_before[neighbor_idx]
    }
}

#[derive(Clone, Copy)]
struct PlaceBlockState {
    root: usize,
    is_initial: bool,
    cur_thresh: f64,
    current: usize,
}

#[derive(Clone, Copy)]
enum PlaceBlockFrame {
    Enter(usize),
    Step(PlaceBlockState),
    ResumeAfterNeighbor { state: PlaceBlockState, neighbor_idx: usize, neighbor_root: usize },
}

fn process_placed_neighbor_for_block(
    state: &mut PlaceBlockState,
    neighbor_idx: usize,
    neighbor_root: usize,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    spacing: f64,
    thresh: &mut ThresholdStrategy,
    class_nodes: &mut Vec<ClassNode>,
    class_lookup: &mut [i32],
) {
    let root = state.root;
    let current = state.current;
    state.cur_thresh = thresh.calculate_threshold(state.cur_thresh, root, current, bal, graph, ni);

    // Sink inheritance: an unclassed block adopts the neighbor's class.
    if bal.sink[root] == root {
        bal.sink[root] = bal.sink[neighbor_root];
    }

    if bal.sink[root] == bal.sink[neighbor_root] {
        // Same class: compact the block against its predecessor.
        // Spacing is read per (current_node, neighbor) NodeType pair
        // — using the constant `SPACING_NODE_NODE` for every pair
        // would over-spread blocks that contain long-edge / label
        // dummies.
        let same_class_spacing = adjacent_vertical_spacing(ni, current, neighbor_idx);

        if bal.vdir == VDir::Down {
            let new_pos = bal.y[neighbor_root]
                + bal.inner_shift[neighbor_idx]
                + ni.node_bottom_extent[neighbor_idx]
                + same_class_spacing
                + ni.node_margin_top[current]
                - bal.inner_shift[current];

            if state.is_initial {
                state.is_initial = false;
                bal.y[root] = new_pos.max(state.cur_thresh);
            } else {
                bal.y[root] = bal.y[root].max(new_pos.max(state.cur_thresh));
            }
        } else {
            let new_pos = bal.y[neighbor_root] + bal.inner_shift[neighbor_idx]
                - ni.node_margin_top[neighbor_idx]
                - same_class_spacing
                - ni.node_bottom_extent[current]
                - bal.inner_shift[current];

            if state.is_initial {
                state.is_initial = false;
                bal.y[root] = new_pos.min(state.cur_thresh);
            } else {
                bal.y[root] = bal.y[root].min(new_pos.min(state.cur_thresh));
            }
        }
    } else {
        // Cross class: record the required separation in the class graph.
        let sink_root = bal.sink[root];
        let sink_neighbor = bal.sink[neighbor_root];
        let sink_class = get_or_create_class_node(sink_root, class_nodes, class_lookup);
        let neighbor_class = get_or_create_class_node(sink_neighbor, class_nodes, class_lookup);

        let required_space = if bal.vdir == VDir::Up {
            bal.y[root] + bal.inner_shift[current] + ni.node_bottom_extent[current] + spacing
                - (bal.y[neighbor_root] + bal.inner_shift[neighbor_idx]
                    - ni.node_margin_top[neighbor_idx])
        } else {
            bal.y[root] + bal.inner_shift[current]
                - ni.node_margin_top[current]
                - bal.y[neighbor_root]
                - bal.inner_shift[neighbor_idx]
                - ni.node_bottom_extent[neighbor_idx]
                - spacing
        };

        add_class_edge(class_nodes, sink_class, neighbor_class, required_space);
    }
}

fn push_next_place_block_step(
    mut state: PlaceBlockState,
    bal: &AlignedLayout,
    thresh: &mut ThresholdStrategy,
    stack: &mut Vec<PlaceBlockFrame>,
) {
    state.current = bal.align[state.current];
    if state.current == state.root {
        thresh.finish_block(state.root);
    } else {
        stack.push(PlaceBlockFrame::Step(state));
    }
}

fn place_block(
    root: usize,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    spacing: f64,
    thresh: &mut ThresholdStrategy,
    class_nodes: &mut Vec<ClassNode>,
    class_lookup: &mut [i32],
) {
    let mut stack = vec![PlaceBlockFrame::Enter(root)];
    while let Some(frame) = stack.pop() {
        match frame {
            PlaceBlockFrame::Enter(root) => {
                if bal.placed[root] {
                    continue;
                }
                bal.placed[root] = true;
                bal.y[root] = 0.0;
                let cur_thresh =
                    if bal.vdir == VDir::Down { f64::NEG_INFINITY } else { f64::INFINITY };
                stack.push(PlaceBlockFrame::Step(PlaceBlockState {
                    root,
                    is_initial: true,
                    cur_thresh,
                    current: root,
                }));
            }
            PlaceBlockFrame::Step(mut state) => {
                let neighbor_idx = if bal.vdir == VDir::Down {
                    ni.upper_neighbor[state.current]
                } else {
                    ni.lower_neighbor[state.current]
                };

                if neighbor_idx == MISSING_NODE_IDX {
                    state.cur_thresh = thresh.calculate_threshold(
                        state.cur_thresh,
                        state.root,
                        state.current,
                        bal,
                        graph,
                        ni,
                    );
                    push_next_place_block_step(state, bal, thresh, &mut stack);
                    continue;
                }

                let neighbor_root = bal.root[neighbor_idx];
                if bal.placed[neighbor_root] {
                    process_placed_neighbor_for_block(
                        &mut state,
                        neighbor_idx,
                        neighbor_root,
                        bal,
                        graph,
                        ni,
                        spacing,
                        thresh,
                        class_nodes,
                        class_lookup,
                    );
                    push_next_place_block_step(state, bal, thresh, &mut stack);
                } else {
                    // Ensure neighbor's block is placed first; resume this node
                    // after the dependency frame has finished.
                    stack.push(PlaceBlockFrame::ResumeAfterNeighbor {
                        state,
                        neighbor_idx,
                        neighbor_root,
                    });
                    stack.push(PlaceBlockFrame::Enter(neighbor_root));
                }
            }
            PlaceBlockFrame::ResumeAfterNeighbor { mut state, neighbor_idx, neighbor_root } => {
                process_placed_neighbor_for_block(
                    &mut state,
                    neighbor_idx,
                    neighbor_root,
                    bal,
                    graph,
                    ni,
                    spacing,
                    thresh,
                    class_nodes,
                    class_lookup,
                );
                push_next_place_block_step(state, bal, thresh, &mut stack);
            }
        }
    }
}

fn compact(
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    spacing: f64,
    thresh: &mut ThresholdStrategy,
) {
    // Reset placement state.
    for i in 0..ni.n {
        bal.placed[i] = false;
    }

    // Sink per block is re-initialized to self, and `shift` receives the
    // vdir-aware sentinel.
    for i in 0..ni.n {
        bal.sink[i] = i;
        bal.shift[i] = if bal.vdir == VDir::Up { f64::NEG_INFINITY } else { f64::INFINITY };
    }
    thresh.init();

    // Class graph state — scoped to this compaction pass.
    let mut class_nodes: Vec<ClassNode> = Vec::new();
    let mut class_lookup: Vec<i32> = vec![-1; ni.n];

    if bal.hdir == HDir::Left {
        for li in (0..graph.layers.len()).rev() {
            compact_layer_roots(
                bal,
                graph,
                ni,
                spacing,
                thresh,
                &mut class_nodes,
                &mut class_lookup,
                li,
            );
        }
    } else {
        for li in 0..graph.layers.len() {
            compact_layer_roots(
                bal,
                graph,
                ni,
                spacing,
                thresh,
                &mut class_nodes,
                &mut class_lookup,
                li,
            );
        }
    }

    // Resolve per-class shifts via a longest-path pass over the class graph.
    place_classes(bal, &mut class_nodes);

    // Apply final block coordinates in layer/node order. Every node first
    // copies its root coordinate, then root nodes apply the resolved class
    // shift when their turn is reached.
    if bal.hdir == HDir::Left {
        for li in (0..graph.layers.len()).rev() {
            apply_final_block_coordinates(bal, graph, ni, li);
        }
    } else {
        for li in 0..graph.layers.len() {
            apply_final_block_coordinates(bal, graph, ni, li);
        }
    }

    // Flush any blocks that `SimpleThresholdStrategy` deferred.
    thresh.post_process(bal, graph, ni);
}

fn apply_final_block_coordinates(
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    li: usize,
) {
    for &nid in &graph.layers[li].nodes {
        let Some(i) = ni.lookup_idx(nid) else { continue };
        bal.y[i] = bal.y[bal.root[i]];
        if bal.root[i] == i {
            let sink_shift = bal.shift[bal.sink[i]];
            let has_shift = if bal.vdir == VDir::Up {
                sink_shift > f64::NEG_INFINITY
            } else {
                sink_shift < f64::INFINITY
            };
            if has_shift {
                bal.y[i] += sink_shift;
            }
        }
    }
}

fn compact_layer_roots(
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    spacing: f64,
    thresh: &mut ThresholdStrategy,
    class_nodes: &mut Vec<ClassNode>,
    class_lookup: &mut [i32],
    li: usize,
) {
    let layer = &graph.layers[li];
    if bal.vdir == VDir::Up {
        for &nid in layer.nodes.iter().rev() {
            let Some(v) = ni.lookup_idx(nid) else { continue };
            compact_root(v, bal, graph, ni, spacing, thresh, class_nodes, class_lookup);
        }
    } else {
        for &nid in &layer.nodes {
            let Some(v) = ni.lookup_idx(nid) else { continue };
            compact_root(v, bal, graph, ni, spacing, thresh, class_nodes, class_lookup);
        }
    }
}

fn compact_root(
    v: usize,
    bal: &mut AlignedLayout,
    graph: &LGraph,
    ni: &Neighborhood,
    spacing: f64,
    thresh: &mut ThresholdStrategy,
    class_nodes: &mut Vec<ClassNode>,
    class_lookup: &mut [i32],
) {
    if bal.root[v] == v {
        place_block(v, bal, graph, ni, spacing, thresh, class_nodes, class_lookup);
    }
}

fn layout_size(bal: &AlignedLayout, _graph: &LGraph, ni: &Neighborhood) -> f64 {
    //   yMin = y[n.id]
    //   yMax = yMin + block_size[root[n.id].id]
    //
    // Use the *block-top* y (without `inner_shift`) and the *block height*,
    // so two nodes that share a block get the same `yMax` and contribute the
    // block bounds exactly once. A per-node form (`y + inner_shift + size.y`)
    // under-counts block height when `inner_shift` shifts any node upward
    // inside a block, leading to wrong winners in `pick_smallest_valid` and
    // ultimately wrong y coordinates on graphs whose 4-alignment candidates
    // tie on per-node extents.
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for i in 0..ni.n {
        let root_idx = bal.root[i];
        let y_min = bal.y[i];
        let y_max = y_min + bal.block_size[root_idx];
        min = min.min(y_min);
        max = max.max(y_max);
    }
    if max < min { 0.0 } else { max - min }
}

/// Returns true if the layout does not violate node ordering in any layer.
fn check_order(bal: &AlignedLayout, graph: &LGraph, ni: &Neighborhood) -> bool {
    for layer in &graph.layers {
        let mut pos = f64::NEG_INFINITY;
        for &nid in &layer.nodes {
            let idx = ni.idx(nid);
            let top = bal.y[idx] + bal.inner_shift[idx] - ni.node_margin_top[idx];
            let bottom = bal.y[idx] + bal.inner_shift[idx] + ni.node_bottom_extent[idx];
            if top <= pos || bottom <= pos {
                return false;
            }
            pos = bottom;
        }
    }
    true
}

fn create_balanced(layouts: &[AlignedLayout], graph: &LGraph, ni: &Neighborhood) -> AlignedLayout {
    let count = layouts.len();
    let mut balanced = AlignedLayout::new(ni.n, VDir::Down, HDir::Right);

    // Find min/max per layout.
    let mut mins = vec![f64::INFINITY; count];
    let mut maxs = vec![f64::NEG_INFINITY; count];
    let mut widths = vec![0.0f64; count];
    let mut min_width_idx = 0usize;

    for (li, bal) in layouts.iter().enumerate() {
        for i in 0..ni.n {
            let nid = ni.idx_to_node[i];
            let node_y = bal.y[i] + bal.inner_shift[i];
            mins[li] = mins[li].min(node_y);
            maxs[li] = maxs[li].max(node_y + graph.node(nid).size.y);
        }
        widths[li] = layout_size(bal, graph, ni);
        if widths[li] < widths[min_width_idx] {
            min_width_idx = li;
        }
    }

    // Shift to align with the smallest layout.
    let mut shift = vec![0.0f64; count];
    for i in 0..count {
        if layouts[i].vdir == VDir::Down {
            shift[i] = mins[min_width_idx] - mins[i];
        } else {
            shift[i] = maxs[min_width_idx] - maxs[i];
        }
    }

    // For each node, take the median of the four shifted y-coordinates.
    // BK always produces exactly 4 layouts (Down/Up × Right/Left), so the
    // median is `(sorted[1] + sorted[2]) / 2`. Using a dynamic `vals[0]` /
    // `vals[n-1]` fallback would silently drift if a future caller passed
    // fewer layouts — the assert below pins the invariant.
    assert_eq!(count, 4, "BK balanced layout expects exactly 4 layouts");
    let mut vals = [0.0f64; 4];
    for idx in 0..ni.n {
        for (li, bal) in layouts.iter().enumerate() {
            vals[li] = bal.y[idx] + bal.inner_shift[idx] + shift[li];
        }
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        balanced.y[idx] = (vals[1] + vals[2]) / 2.0;
        // inner shift is already folded in.
        balanced.inner_shift[idx] = 0.0;
    }

    balanced
}

fn build_aligned_layout(
    graph: &LGraph,
    ni: &Neighborhood,
    marked: &hashbrown::HashSet<EdgeId>,
    spacing: f64,
    improve_straightness: bool,
    vd: VDir,
    hd: HDir,
) -> AlignedLayout {
    let mut bal = AlignedLayout::new(ni.n, vd, hd);
    let mut thresh = if improve_straightness {
        ThresholdStrategy::simple()
    } else {
        ThresholdStrategy::null()
    };
    vertical_alignment(&mut bal, graph, ni, marked);
    inside_block_shift(&mut bal, graph, ni);
    compact(&mut bal, graph, ni, spacing, &mut thresh);
    bal
}

/// Place nodes using the Brandes-Koepf algorithm.
pub fn place_nodes(graph: &mut LGraph) {
    if graph.layers.is_empty() {
        return;
    }

    let ni = build_neighborhood(graph);
    if ni.n == 0 {
        return;
    }

    let spacing = graph.options.spacing.node_node;
    let marked = mark_conflicts(graph, &ni);

    use crate::options::EdgeStraighteningStrategy;
    let improve_straightness =
        graph.options.edge_straightening == EdgeStraighteningStrategy::ImproveStraightness;

    // Build the layout set. The fixed-alignment option drives the choice:
    // explicit LEFTUP / LEFTDOWN / RIGHTUP / RIGHTDOWN produce a single
    // layout; the default branch produces all four.
    use crate::options::{EdgeRoutingStrategy, FixedAlignment};
    let alignment = graph.options.fixed_alignment;
    let dir_list: &[(VDir, HDir)] = match alignment {
        FixedAlignment::LeftDown => &[(VDir::Down, HDir::Left)],
        FixedAlignment::LeftUp => &[(VDir::Up, HDir::Left)],
        FixedAlignment::RightDown => &[(VDir::Down, HDir::Right)],
        FixedAlignment::RightUp => &[(VDir::Up, HDir::Right)],
        // `None`, `Balanced`, `Leftmost`, `Rightmost` all go through the
        // default branch that builds all four candidates. `Balanced` adds
        // an extra synthesised layout downstream.
        //
        // The four layouts are added in this exact order:
        //   (DOWN, RIGHT), (UP, RIGHT), (DOWN, LEFT), (UP, LEFT)
        // Order matters because `pick_smallest_valid` uses strict `>` when
        // comparing layout size, so on ties the first iterator entry wins.
        _ => &[
            (VDir::Down, HDir::Right),
            (VDir::Up, HDir::Right),
            (VDir::Down, HDir::Left),
            (VDir::Up, HDir::Left),
        ],
    };

    // Choose layout: balanced if requested, else smallest valid.
    //
    // A balanced layout is produced only when alignment is explicitly
    // Balanced, or when alignment is None AND `favor_straight_edges` is
    // false. `favor_straight_edges` defaults to `(edge_routing ==
    // ORTHOGONAL)`, so orthogonal routing (the default) skips balanced and
    // picks the smallest valid layout from the four.
    let favor_straight_edges = graph.options.edge_routing == EdgeRoutingStrategy::Orthogonal;
    let use_balanced = alignment == FixedAlignment::Balanced
        || (alignment == FixedAlignment::None && !favor_straight_edges);

    let chosen_layout = if use_balanced {
        let mut layouts: SmallVec<AlignedLayout, 4> = SmallVec::new();
        for &(vd, hd) in dir_list {
            layouts.push(build_aligned_layout(
                graph,
                &ni,
                &marked,
                spacing,
                improve_straightness,
                vd,
                hd,
            ));
        }

        let balanced = create_balanced(&layouts, graph, &ni);
        if check_order(&balanced, graph, &ni) {
            layouts.push(balanced);
            layouts.swap_remove(layouts.len() - 1)
        } else {
            let chosen_idx = pick_smallest_valid(&layouts, graph, &ni);
            layouts.swap_remove(chosen_idx)
        }
    } else {
        let mut fallback_layout = None;
        let mut best_layout = None;
        let mut best_size = f64::INFINITY;

        for &(vd, hd) in dir_list {
            let bal =
                build_aligned_layout(graph, &ni, &marked, spacing, improve_straightness, vd, hd);
            if check_order(&bal, graph, &ni) {
                let size = layout_size(&bal, graph, &ni);
                if size < best_size {
                    best_size = size;
                    best_layout = Some(bal);
                }
            } else if fallback_layout.is_none() {
                fallback_layout = Some(bal);
            }
        }
        best_layout
            .or(fallback_layout)
            .expect("BK node placement must produce at least one layout")
    };

    // Apply calculated positions to nodes: `node.position.y = y + inner_shift`.
    // Normalization and graph.size.y are handled by the downstream
    // `LayerSizeAndGraphHeightCalculator` via `graph.offset.y`.
    for i in 0..ni.n {
        let nid = ni.idx_to_node[i];
        graph.node_mut(nid).position.y = chosen_layout.y[i] + chosen_layout.inner_shift[i];
    }
}

fn pick_smallest_valid(layouts: &[AlignedLayout], graph: &LGraph, ni: &Neighborhood) -> usize {
    let mut best: Option<usize> = None;
    let mut best_size = f64::INFINITY;
    for (i, bal) in layouts.iter().enumerate() {
        if check_order(bal, graph, ni) {
            let sz = layout_size(bal, graph, ni);
            if sz < best_size {
                best_size = sz;
                best = Some(i);
            }
        }
    }
    // Fall back to first layout if none is valid (should not happen).
    best.unwrap_or(0)
}
