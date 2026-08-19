//! Gansner network simplex solver core.

use std::collections::{HashMap, VecDeque};

use super::graph::{NEdge, NGraph, NNode};

/// Numerical tolerance for cut values: values greater than this are treated
/// as non-negative to avoid spurious pivots from floating-point noise.
const FUZZY_ZERO: f64 = -1e-10;

/// Node-count threshold above which the subtree pre/post pass runs.
const SUBTREE_OPT_THRESHOLD: usize = 40;

/// Builder configuration for one Gansner network-simplex solve.
///
/// Each `with_*` method consumes `self` and returns a new `Solver` with the
/// option applied (`forGraph(g).withIterationLimit(...).withBalancing(true).execute(monitor)`
/// style chain).
pub struct Solver {
    graph: NGraph,
    iter_limit: Option<usize>,
    do_balance: bool,
    previous_counts: Option<Vec<usize>>,
    subtree_opt: bool,
}

/// Outcome of [`Solver::solve`].
pub struct SolverResult {
    /// Graph with `NNode::layer` assigned. Callers map back to their domain
    /// using [`NNode::stable_id`].
    pub graph: NGraph,
}

impl Solver {
    pub fn new(graph: NGraph) -> Self {
        Self {
            graph,
            iter_limit: None,
            do_balance: false,
            previous_counts: None,
            subtree_opt: false,
        }
    }

    /// Cap the number of simplex pivot iterations. `None` (the default) means
    /// "run until no negative cut-value tree edge remains".
    pub fn with_iter_limit(mut self, limit: usize) -> Self {
        self.iter_limit = Some(limit);
        self
    }

    /// Enable the post-normalize balancing pass. Default: `false`.
    pub fn with_balancing(mut self, do_balance: bool) -> Self {
        self.do_balance = do_balance;
        self
    }

    /// Seed the normalize/balance passes with per-layer node counts from a
    /// previously solved connected component.
    pub fn with_previous_counts(mut self, counts: Vec<usize>) -> Self {
        self.previous_counts = Some(counts);
        self
    }

    /// Enable the leaf-removal pre/post pass (`removeSubtrees` /
    /// `reattachSubtrees`). When enabled, nodes with a single connected edge
    /// are peeled off before the main simplex loop and reattached afterwards,
    /// which can turn a deep graph
    /// into a much smaller one for the pivot phase.
    ///
    /// Callers that cache `NNode` / `NEdge` array indices outside the
    /// solver must leave this `false` — the pre-pass rebuilds both vectors
    /// and invalidates any such cache. Use [`NNode::stable_id`] instead.
    /// Default: `false`.
    pub fn with_subtree_optimization(mut self, enabled: bool) -> Self {
        self.subtree_opt = enabled;
        self
    }

    /// Run the solver. Consumes `self` and returns the solved graph plus
    /// per-layer filling counts.
    pub fn solve(self) -> SolverResult {
        let limit = self.iter_limit.unwrap_or(usize::MAX);
        run_network_simplex(
            self.graph,
            limit,
            self.do_balance,
            self.previous_counts.as_deref(),
            self.subtree_opt,
        )
    }
}

/// Run the full pipeline on `g`: reset, subtree pre-pass, initial feasible
/// layering, spanning tree, simplex iteration, subtree post-pass, normalize,
/// balance.
fn run_network_simplex(
    mut g: NGraph,
    iter_limit: usize,
    do_balance: bool,
    previous_counts: Option<&[usize]>,
    subtree_opt: bool,
) -> SolverResult {
    if g.nodes.is_empty() {
        return SolverResult { graph: g };
    }

    for n in &mut g.nodes {
        n.layer = 0;
    }

    let removed = if subtree_opt && g.nodes.len() >= SUBTREE_OPT_THRESHOLD {
        remove_subtrees(&mut g)
    } else {
        Vec::new()
    };

    if g.nodes.is_empty() {
        reattach_subtrees(&mut g, &removed);
        let mut filling = normalize(&mut g, previous_counts);
        if do_balance {
            balance(&mut g, &mut filling);
        }
        return SolverResult { graph: g };
    }

    let sources: Vec<usize> =
        (0..g.nodes.len()).filter(|&i| g.nodes[i].incoming.is_empty()).collect();
    topological_layering(&mut g, &sources);

    let num_edges = g.edges.len();
    if num_edges > 0 {
        let mut edge_visited = vec![false; num_edges];
        let node_count = g.nodes.len();

        while tight_tree_dfs(&mut g, 0, &mut edge_visited) < node_count {
            if let Some(e_idx) = minimal_slack(&g) {
                let source = g.edges[e_idx].source;
                let target = g.edges[e_idx].target;
                let delta = g.edges[e_idx].delta;
                let mut slack = g.nodes[target].layer - g.nodes[source].layer - delta;
                if g.nodes[target].tree_node {
                    slack = -slack;
                }
                for n in &mut g.nodes {
                    if n.tree_node {
                        n.layer += slack;
                    }
                }
            } else {
                break;
            }
            edge_visited.fill(false);
        }

        let num_nodes = g.nodes.len();
        let mut po_id = vec![0i32; num_nodes];
        let mut lowest_po_id = vec![0i32; num_nodes];
        let mut cutvalue = vec![0.0f64; g.edges.len()];
        let mut postorder_stack = Vec::with_capacity(num_nodes);

        edge_visited.fill(false);
        let mut post_order = 1i32;
        postorder_traversal(
            &g,
            0,
            &mut po_id,
            &mut lowest_po_id,
            &mut edge_visited,
            &mut post_order,
            &mut postorder_stack,
        );
        compute_cutvalues(&g, &mut cutvalue);

        // Main simplex loop:
        //   NEdge e = leaveEdge(); int iter = 0;
        //   while (e != null && iter < iterationLimit) {
        //       exchange(e, enterEdge(e)); e = leaveEdge(); iter++;
        //   }
        // The `iter == limit - 1` exchange must run before the next
        // leave-edge probe. An earlier ordering (`if iter >= iter_limit {
        // break }` before the exchange) skipped that final exchange and
        // produced a 1-iteration deficit on graphs that exhausted the budget.
        let mut iter = 0;
        while iter < iter_limit {
            let Some(leave_idx) = find_leave_edge(&g, &cutvalue, FUZZY_ZERO) else { break };
            let Some(enter_idx) = find_enter_edge(&g, leave_idx, &po_id, &lowest_po_id) else {
                break;
            };
            exchange(
                &mut g,
                leave_idx,
                enter_idx,
                &mut po_id,
                &mut lowest_po_id,
                &mut cutvalue,
                &mut edge_visited,
                &mut postorder_stack,
            );
            iter += 1;
        }
    }

    reattach_subtrees(&mut g, &removed);

    let mut filling = normalize(&mut g, previous_counts);
    if do_balance {
        balance(&mut g, &mut filling);
    }

    SolverResult { graph: g }
}

// Topological layering

fn topological_layering(g: &mut NGraph, sources: &[usize]) {
    let mut incident: Vec<usize> = g.nodes.iter().map(|n| n.incoming.len()).collect();

    let mut queue: VecDeque<usize> = VecDeque::new();
    for &s in sources {
        queue.push_back(s);
    }

    while let Some(node_idx) = queue.pop_front() {
        let outgoing_len = g.nodes[node_idx].outgoing.len();
        for pos in 0..outgoing_len {
            let edge_idx = g.nodes[node_idx].outgoing[pos];
            let target = g.edges[edge_idx].target;
            let delta = g.edges[edge_idx].delta;
            let new_layer = g.nodes[node_idx].layer + delta;
            if new_layer > g.nodes[target].layer {
                g.nodes[target].layer = new_layer;
            }
            incident[target] -= 1;
            if incident[target] == 0 {
                queue.push_back(target);
            }
        }
    }
}

// Tight tree DFS

struct TightTreeFrame {
    node_idx: usize,
    incoming_pos: usize,
    outgoing_pos: usize,
}

fn tight_tree_dfs(g: &mut NGraph, node_idx: usize, edge_visited: &mut [bool]) -> usize {
    let mut node_count = 1;
    g.nodes[node_idx].tree_node = true;

    let mut stack = vec![TightTreeFrame { node_idx, incoming_pos: 0usize, outgoing_pos: 0usize }];
    while let Some(frame_idx) = stack.len().checked_sub(1) {
        let node_idx = stack[frame_idx].node_idx;
        let edge_idx = if stack[frame_idx].incoming_pos < g.nodes[node_idx].incoming.len() {
            let edge_idx = g.nodes[node_idx].incoming[stack[frame_idx].incoming_pos];
            stack[frame_idx].incoming_pos += 1;
            edge_idx
        } else if stack[frame_idx].outgoing_pos < g.nodes[node_idx].outgoing.len() {
            let edge_idx = g.nodes[node_idx].outgoing[stack[frame_idx].outgoing_pos];
            stack[frame_idx].outgoing_pos += 1;
            edge_idx
        } else {
            stack.pop();
            continue;
        };

        if let Some(opposite) = tight_tree_dfs_edge_descendant(g, node_idx, edge_idx, edge_visited)
        {
            g.nodes[opposite].tree_node = true;
            node_count += 1;
            stack.push(TightTreeFrame {
                node_idx: opposite,
                incoming_pos: 0usize,
                outgoing_pos: 0usize,
            });
        }
    }

    node_count
}

fn tight_tree_dfs_edge_descendant(
    g: &mut NGraph,
    node_idx: usize,
    edge_idx: usize,
    edge_visited: &mut [bool],
) -> Option<usize> {
    if edge_visited[edge_idx] {
        return None;
    }
    edge_visited[edge_idx] = true;

    let source = g.edges[edge_idx].source;
    let target = g.edges[edge_idx].target;
    let opposite = if source == node_idx { target } else { source };

    if g.edges[edge_idx].tree_edge {
        return Some(opposite);
    }

    if !g.nodes[opposite].tree_node {
        let slack = g.nodes[target].layer - g.nodes[source].layer - g.edges[edge_idx].delta;
        if slack == 0 {
            g.mark_tree_edge(edge_idx);
            return Some(opposite);
        }
    }

    None
}

// Minimal slack

fn minimal_slack(g: &NGraph) -> Option<usize> {
    let mut min_slack = i32::MAX;
    let mut min_edge: Option<usize> = None;

    for (idx, edge) in g.edges.iter().enumerate() {
        if g.nodes[edge.source].tree_node ^ g.nodes[edge.target].tree_node {
            let slack = g.nodes[edge.target].layer - g.nodes[edge.source].layer - edge.delta;
            if slack < min_slack {
                min_slack = slack;
                min_edge = Some(idx);
            }
        }
    }
    min_edge
}

// Postorder traversal

struct PostorderFrame {
    node_idx: usize,
    outgoing_pos: u32,
    incoming_pos: u32,
    lowest: i32,
}

fn postorder_traversal(
    g: &NGraph,
    node_idx: usize,
    po_id: &mut [i32],
    lowest_po_id: &mut [i32],
    edge_visited: &mut [bool],
    post_order: &mut i32,
    stack: &mut Vec<PostorderFrame>,
) -> i32 {
    stack.clear();
    stack.push(PostorderFrame { node_idx, outgoing_pos: 0, incoming_pos: 0, lowest: i32::MAX });

    while let Some(frame_idx) = stack.len().checked_sub(1) {
        let node_idx = stack[frame_idx].node_idx;

        let mut descend = None;
        while (stack[frame_idx].incoming_pos as usize) < g.nodes[node_idx].incoming.len() {
            let edge_idx = g.nodes[node_idx].incoming[stack[frame_idx].incoming_pos as usize];
            stack[frame_idx].incoming_pos += 1;
            if g.edges[edge_idx].tree_edge && !edge_visited[edge_idx] {
                edge_visited[edge_idx] = true;
                let source = g.edges[edge_idx].source;
                let target = g.edges[edge_idx].target;
                descend = Some(if source == node_idx { target } else { source });
                break;
            }
        }
        if let Some(opposite) = descend {
            stack.push(PostorderFrame {
                node_idx: opposite,
                outgoing_pos: 0,
                incoming_pos: 0,
                lowest: i32::MAX,
            });
            continue;
        }

        let mut descend = None;
        while (stack[frame_idx].outgoing_pos as usize) < g.nodes[node_idx].outgoing.len() {
            let edge_idx = g.nodes[node_idx].outgoing[stack[frame_idx].outgoing_pos as usize];
            stack[frame_idx].outgoing_pos += 1;
            if g.edges[edge_idx].tree_edge && !edge_visited[edge_idx] {
                edge_visited[edge_idx] = true;
                let source = g.edges[edge_idx].source;
                let target = g.edges[edge_idx].target;
                descend = Some(if source == node_idx { target } else { source });
                break;
            }
        }
        if let Some(opposite) = descend {
            stack.push(PostorderFrame {
                node_idx: opposite,
                outgoing_pos: 0,
                incoming_pos: 0,
                lowest: i32::MAX,
            });
            continue;
        }

        let frame = stack.pop().unwrap();
        let lowest = frame.lowest.min(*post_order);
        po_id[frame.node_idx] = *post_order;
        lowest_po_id[frame.node_idx] = lowest;
        *post_order += 1;

        if let Some(parent) = stack.last_mut() {
            parent.lowest = parent.lowest.min(lowest);
        } else {
            return lowest;
        }
    }

    i32::MAX
}

// Compute cut values

fn compute_cutvalues(g: &NGraph, cutvalue: &mut [f64]) {
    let num_nodes = g.nodes.len();

    let mut unknown_count = vec![0usize; num_nodes];
    let mut remaining_edge = vec![0usize; num_nodes];

    for (edge_idx, edge) in g.edges.iter().enumerate() {
        if edge.tree_edge {
            unknown_count[edge.source] += 1;
            remaining_edge[edge.source] ^= edge_idx;
            unknown_count[edge.target] += 1;
            remaining_edge[edge.target] ^= edge_idx;
        }
    }

    let mut leafs: Vec<usize> = Vec::new();
    for (i, &count) in unknown_count.iter().enumerate().take(num_nodes) {
        if count == 1 {
            leafs.push(i);
        }
    }

    for leaf_start in leafs {
        let mut node = leaf_start;

        while unknown_count[node] == 1 {
            let to_determine = remaining_edge[node];
            let td_source = g.edges[to_determine].source;
            let td_target = g.edges[to_determine].target;

            cutvalue[to_determine] = g.edges[to_determine].weight;

            for &edge_idx in &g.nodes[node].incoming {
                update_cutvalue_from_connected_edge(
                    g,
                    cutvalue,
                    node,
                    to_determine,
                    td_source,
                    td_target,
                    edge_idx,
                );
            }
            for &edge_idx in &g.nodes[node].outgoing {
                update_cutvalue_from_connected_edge(
                    g,
                    cutvalue,
                    node,
                    to_determine,
                    td_source,
                    td_target,
                    edge_idx,
                );
            }

            remove_unknown_cutvalue(
                &mut unknown_count,
                &mut remaining_edge,
                td_source,
                to_determine,
            );
            remove_unknown_cutvalue(
                &mut unknown_count,
                &mut remaining_edge,
                td_target,
                to_determine,
            );

            node = if node == td_source { td_target } else { td_source };
        }
    }
}

#[inline]
fn remove_unknown_cutvalue(
    unknown_count: &mut [usize],
    remaining_edge: &mut [usize],
    node: usize,
    edge_idx: usize,
) {
    debug_assert!(unknown_count[node] > 0);
    unknown_count[node] -= 1;
    remaining_edge[node] ^= edge_idx;
}

fn update_cutvalue_from_connected_edge(
    g: &NGraph,
    cutvalue: &mut [f64],
    node: usize,
    to_determine: usize,
    td_source: usize,
    td_target: usize,
    edge_idx: usize,
) {
    if edge_idx == to_determine {
        return;
    }

    let e_source = g.edges[edge_idx].source;
    let e_target = g.edges[edge_idx].target;

    if g.edges[edge_idx].tree_edge {
        if e_source == td_source || e_target == td_target {
            cutvalue[to_determine] -= cutvalue[edge_idx] - g.edges[edge_idx].weight;
        } else {
            cutvalue[to_determine] += cutvalue[edge_idx] - g.edges[edge_idx].weight;
        }
    } else if node == td_source {
        if e_source == node {
            cutvalue[to_determine] += g.edges[edge_idx].weight;
        } else {
            cutvalue[to_determine] -= g.edges[edge_idx].weight;
        }
    } else if e_source == node {
        cutvalue[to_determine] -= g.edges[edge_idx].weight;
    } else {
        cutvalue[to_determine] += g.edges[edge_idx].weight;
    }
}

// Find leave edge

fn find_leave_edge(g: &NGraph, cutvalue: &[f64], fuzzy_zero: f64) -> Option<usize> {
    g.tree_edge_order.iter().find(|&&idx| cutvalue[idx] < fuzzy_zero).copied()
}

// Is in head component

fn is_in_head(
    node_idx: usize,
    edge_source: usize,
    edge_target: usize,
    po_id: &[i32],
    lowest_po_id: &[i32],
) -> bool {
    let n = po_id[node_idx];
    if lowest_po_id[edge_source] <= n
        && n <= po_id[edge_source]
        && lowest_po_id[edge_target] <= n
        && n <= po_id[edge_target]
    {
        po_id[edge_source] >= po_id[edge_target]
    } else {
        po_id[edge_source] < po_id[edge_target]
    }
}

// Find enter edge

fn find_enter_edge(
    g: &NGraph,
    leave_idx: usize,
    po_id: &[i32],
    lowest_po_id: &[i32],
) -> Option<usize> {
    let leave_source = g.edges[leave_idx].source;
    let leave_target = g.edges[leave_idx].target;

    let mut replace: Option<usize> = None;
    let mut rep_slack = i32::MAX;

    for (idx, edge) in g.edges.iter().enumerate() {
        let source = edge.source;
        let target = edge.target;
        if is_in_head(source, leave_source, leave_target, po_id, lowest_po_id)
            && !is_in_head(target, leave_source, leave_target, po_id, lowest_po_id)
        {
            let slack = g.nodes[target].layer - g.nodes[source].layer - edge.delta;
            if slack < rep_slack {
                rep_slack = slack;
                replace = Some(idx);
            }
        }
    }
    replace
}

// Exchange

fn exchange(
    g: &mut NGraph,
    leave_idx: usize,
    enter_idx: usize,
    po_id: &mut [i32],
    lowest_po_id: &mut [i32],
    cutvalue: &mut [f64],
    edge_visited: &mut [bool],
    postorder_stack: &mut Vec<PostorderFrame>,
) {
    let leave_source = g.edges[leave_idx].source;
    let leave_target = g.edges[leave_idx].target;

    g.unmark_tree_edge(leave_idx);
    g.mark_tree_edge(enter_idx);

    let enter_source = g.edges[enter_idx].source;
    let enter_target = g.edges[enter_idx].target;
    let enter_delta = g.edges[enter_idx].delta;
    let mut delta = g.nodes[enter_target].layer - g.nodes[enter_source].layer - enter_delta;

    if !is_in_head(enter_target, leave_source, leave_target, po_id, lowest_po_id) {
        delta = -delta;
    }

    for i in 0..g.nodes.len() {
        if !is_in_head(i, leave_source, leave_target, po_id, lowest_po_id) {
            g.nodes[i].layer += delta;
        }
    }

    edge_visited.fill(false);
    let mut post_order = 1i32;
    postorder_traversal(g, 0, po_id, lowest_po_id, edge_visited, &mut post_order, postorder_stack);
    compute_cutvalues(g, cutvalue);
}

// Remove subtrees

/// Record of one leaf node removed during the subtree optimization.
struct RemovedNode {
    stable_id: u32,
    other_stable_id: u32,
    is_source: bool,
    edge_delta: i32,
    edge_weight: f64,
}

fn remove_subtrees(g: &mut NGraph) -> Vec<RemovedNode> {
    let mut removed: Vec<RemovedNode> = Vec::new();

    let mut queue: VecDeque<usize> = VecDeque::new();
    let mut node_alive = vec![true; g.nodes.len()];

    for i in 0..g.nodes.len() {
        if count_connected(g, i, &node_alive) == 1 {
            queue.push_back(i);
        }
    }

    while let Some(node_idx) = queue.pop_front() {
        if !node_alive[node_idx] {
            continue;
        }

        let connected = count_connected(g, node_idx, &node_alive);
        if connected != 1 {
            continue;
        }

        let Some(edge_idx) = find_single_connected_edge(g, node_idx, &node_alive) else {
            continue;
        };

        let edge_source = g.edges[edge_idx].source;
        let edge_target = g.edges[edge_idx].target;
        let edge_delta = g.edges[edge_idx].delta;
        let edge_weight = g.edges[edge_idx].weight;
        let is_source = edge_source == node_idx;
        let other = if is_source { edge_target } else { edge_source };

        node_alive[node_idx] = false;

        removed.push(RemovedNode {
            stable_id: g.nodes[node_idx].stable_id,
            other_stable_id: g.nodes[other].stable_id,
            is_source,
            edge_delta,
            edge_weight,
        });

        if node_alive[other] && count_connected(g, other, &node_alive) == 1 {
            queue.push_back(other);
        }
    }

    if removed.is_empty() {
        return removed;
    }

    // Rebuild the graph keeping only alive nodes.
    let mut old_to_new = vec![usize::MAX; g.nodes.len()];
    let mut new_idx = 0;
    for (i, alive) in node_alive.iter().copied().enumerate().take(g.nodes.len()) {
        if alive {
            old_to_new[i] = new_idx;
            new_idx += 1;
        }
    }

    let mut new_nodes: Vec<NNode> = Vec::new();
    for (i, alive) in node_alive.iter().copied().enumerate().take(g.nodes.len()) {
        if alive {
            new_nodes.push(NNode {
                layer: g.nodes[i].layer,
                tree_node: false,
                outgoing: Vec::new(),
                incoming: Vec::new(),
                stable_id: g.nodes[i].stable_id,
            });
        }
    }

    let mut new_edges: Vec<NEdge> = Vec::new();
    for edge in &g.edges {
        if node_alive[edge.source] && node_alive[edge.target] {
            let new_src = old_to_new[edge.source];
            let new_tgt = old_to_new[edge.target];
            let eidx = new_edges.len();
            new_nodes[new_src].outgoing.push(eidx);
            new_nodes[new_tgt].incoming.push(eidx);
            new_edges.push(NEdge {
                source: new_src,
                target: new_tgt,
                weight: edge.weight,
                delta: edge.delta,
                tree_edge: false,
            });
        }
    }

    g.nodes = new_nodes;
    g.edges = new_edges;
    g.tree_edge_order.clear();

    removed
}

fn count_connected(g: &NGraph, node_idx: usize, node_alive: &[bool]) -> usize {
    let mut count = 0;
    for &eidx in &g.nodes[node_idx].outgoing {
        if node_alive[g.edges[eidx].target] {
            count += 1;
        }
    }
    for &eidx in &g.nodes[node_idx].incoming {
        if node_alive[g.edges[eidx].source] {
            count += 1;
        }
    }
    count
}

fn find_single_connected_edge(g: &NGraph, node_idx: usize, node_alive: &[bool]) -> Option<usize> {
    for &eidx in &g.nodes[node_idx].incoming {
        if node_alive[g.edges[eidx].source] {
            return Some(eidx);
        }
    }
    g.nodes[node_idx]
        .outgoing
        .iter()
        .find(|&&eidx| node_alive[g.edges[eidx].target])
        .copied()
}

// Reattach subtrees

fn reattach_subtrees(g: &mut NGraph, removed: &[RemovedNode]) {
    if removed.is_empty() {
        return;
    }

    let mut sid_to_idx: HashMap<u32, usize> = HashMap::with_capacity(g.nodes.len());
    for (i, node) in g.nodes.iter().enumerate() {
        sid_to_idx.insert(node.stable_id, i);
    }

    for rem in removed.iter().rev() {
        if let Some(&other_idx) = sid_to_idx.get(&rem.other_stable_id) {
            let new_node_idx = g.nodes.len();
            let other_layer = g.nodes[other_idx].layer;

            let node_layer = if rem.is_source {
                other_layer - rem.edge_delta
            } else {
                other_layer + rem.edge_delta
            };

            g.nodes.push(NNode {
                layer: node_layer,
                tree_node: false,
                outgoing: Vec::new(),
                incoming: Vec::new(),
                stable_id: rem.stable_id,
            });

            let edge_idx = g.edges.len();
            let (src, tgt) = if rem.is_source {
                (new_node_idx, other_idx)
            } else {
                (other_idx, new_node_idx)
            };
            g.edges.push(NEdge {
                source: src,
                target: tgt,
                weight: rem.edge_weight,
                delta: rem.edge_delta,
                tree_edge: false,
            });
            g.nodes[src].outgoing.push(edge_idx);
            g.nodes[tgt].incoming.push(edge_idx);

            sid_to_idx.insert(rem.stable_id, new_node_idx);
        }
    }
}

// Normalize

/// Shift layers so the lowest assigned layer is 0 and return per-layer node
/// counts.
///
/// `previous_counts` are folded into the returned `filling` array so a
/// subsequent balance pass on the next connected component sees
/// previously-layered nodes.
fn normalize(g: &mut NGraph, previous_counts: Option<&[usize]>) -> Vec<usize> {
    if g.nodes.is_empty() {
        return Vec::new();
    }
    let highest = g.nodes.iter().map(|n| n.layer).max().unwrap_or(0);
    let lowest = g.nodes.iter().map(|n| n.layer).min().unwrap_or(0);
    let layer_count = (highest - lowest + 1) as usize;
    let mut filling = vec![0usize; layer_count];
    for n in &mut g.nodes {
        n.layer -= lowest;
        filling[n.layer as usize] += 1;
    }
    if let Some(prev) = previous_counts {
        for (idx, &cnt) in prev.iter().enumerate() {
            if idx >= filling.len() {
                break;
            }
            filling[idx] += cnt;
        }
    }
    filling
}

// Balance

/// Move nodes whose in- and out-degree match toward the least-populated
/// legal layer. `filling` is read and mutated in place.
fn balance(g: &mut NGraph, filling: &mut [usize]) {
    if g.nodes.is_empty() || filling.is_empty() {
        return;
    }

    for i in 0..g.nodes.len() {
        let in_deg = g.nodes[i].incoming.len();
        let out_deg = g.nodes[i].outgoing.len();
        if in_deg != out_deg {
            continue;
        }

        let (min_span_in, min_span_out) = minimal_span(g, i);
        let cur_layer = g.nodes[i].layer;
        let mut new_layer = cur_layer;

        // Balance bounds are intentionally unconditional. When `minimalSpan`
        // returns -1 for either side (no incoming or no outgoing edges), the
        // candidate range collapses to empty. Treating that case specially
        // changes downstream long-edge splitting.
        let low = cur_layer - min_span_in + 1;
        let high = cur_layer + min_span_out;

        for l in low..high {
            if l >= 0
                && (l as usize) < filling.len()
                && filling[l as usize] < filling[new_layer as usize]
            {
                new_layer = l;
            }
        }

        if filling[new_layer as usize] < filling[cur_layer as usize] {
            filling[cur_layer as usize] -= 1;
            filling[new_layer as usize] += 1;
            g.nodes[i].layer = new_layer;
        }
    }
}

fn minimal_span(g: &NGraph, node_idx: usize) -> (i32, i32) {
    let mut min_span_in = i32::MAX;
    let mut min_span_out = i32::MAX;

    // Keep a single pass over incoming and outgoing edges. When an incoming
    // edge's span is >= `minSpanIn`, it falls into the second arm and may
    // lower `minSpanOut`; that bleed-through defines the balance range.
    for &eidx in g.nodes[node_idx].incoming.iter().chain(g.nodes[node_idx].outgoing.iter()) {
        let edge = &g.edges[eidx];
        let span = g.nodes[edge.target].layer - g.nodes[edge.source].layer;
        if edge.target == node_idx && span < min_span_in {
            min_span_in = span;
        } else if span < min_span_out {
            min_span_out = span;
        }
    }

    if min_span_in == i32::MAX {
        min_span_in = -1;
    }
    if min_span_out == i32::MAX {
        min_span_out = -1;
    }

    (min_span_in, min_span_out)
}
