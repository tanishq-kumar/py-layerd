//! Auxiliary graph data structures for the Gansner network simplex.

/// Auxiliary node in a network simplex graph.
pub struct NNode {
    /// Rank assigned by the solver. Callers interpret this as a layer index
    /// (P2) or a y-position (P4).
    pub layer: i32,
    /// True while this node is inside the current feasible spanning tree.
    pub tree_node: bool,
    /// Indices into [`NGraph::edges`] for edges leaving this node.
    pub outgoing: Vec<usize>,
    /// Indices into [`NGraph::edges`] for edges entering this node.
    pub incoming: Vec<usize>,
    /// Caller-assigned identifier that survives the internal subtree
    /// optimization. After [`super::Solver::solve`] returns, the resulting
    /// `NGraph` may have nodes in a different order or slightly different
    /// count; `stable_id` is the only reliable back-pointer to the caller's
    /// domain entity.
    pub stable_id: u32,
}

/// Auxiliary edge in a network simplex graph.
pub struct NEdge {
    /// Index of the source node in [`NGraph::nodes`].
    pub source: usize,
    /// Index of the target node in [`NGraph::nodes`].
    pub target: usize,
    /// Priority/strength of the edge. Higher values pull endpoints tighter.
    pub weight: f64,
    /// Minimum rank difference this edge enforces (`target.layer -
    /// source.layer >= delta`). Always 1 for P2 layering; varies for P4.
    pub delta: i32,
    /// True while this edge is inside the current feasible spanning tree.
    pub tree_edge: bool,
}

/// Auxiliary graph used by the Gansner network simplex solver.
pub struct NGraph {
    pub nodes: Vec<NNode>,
    pub edges: Vec<NEdge>,
    /// Indices into `edges` that are currently tree edges, kept in the order
    /// they were promoted. Insertion-ordered set semantics: `leaveEdge` picks
    /// the first tree edge with a negative cut value, so order controls the
    /// pivot sequence.
    pub tree_edge_order: Vec<usize>,
}

impl Default for NGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl NGraph {
    pub fn new() -> Self {
        Self { nodes: Vec::new(), edges: Vec::new(), tree_edge_order: Vec::new() }
    }

    pub fn with_capacity(node_cap: usize, edge_cap: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(node_cap),
            edges: Vec::with_capacity(edge_cap),
            tree_edge_order: Vec::with_capacity(node_cap.saturating_sub(1)),
        }
    }

    /// Add a node with the given caller-assigned stable identifier. Returns
    /// the new node's index in [`Self::nodes`].
    pub fn add_node(&mut self, stable_id: u32) -> usize {
        let idx = self.nodes.len();
        self.nodes.push(NNode {
            layer: 0,
            tree_node: false,
            outgoing: Vec::new(),
            incoming: Vec::new(),
            stable_id,
        });
        idx
    }

    /// Add a directed edge with the given `weight` and minimum span `delta`.
    /// Returns the new edge's index in [`Self::edges`].
    pub fn add_edge(&mut self, source: usize, target: usize, weight: f64, delta: i32) -> usize {
        let idx = self.edges.len();
        self.edges.push(NEdge { source, target, weight, delta, tree_edge: false });
        self.nodes[source].outgoing.push(idx);
        self.nodes[target].incoming.push(idx);
        idx
    }

    /// Promote `edge_idx` to a tree edge. No-op if already a tree edge.
    #[inline]
    pub(super) fn mark_tree_edge(&mut self, edge_idx: usize) {
        if !self.edges[edge_idx].tree_edge {
            self.edges[edge_idx].tree_edge = true;
            self.tree_edge_order.push(edge_idx);
        }
    }

    /// Demote `edge_idx` from a tree edge. No-op if not a tree edge.
    #[inline]
    pub(super) fn unmark_tree_edge(&mut self, edge_idx: usize) {
        if self.edges[edge_idx].tree_edge {
            self.edges[edge_idx].tree_edge = false;
            self.tree_edge_order.retain(|&e| e != edge_idx);
        }
    }
}
