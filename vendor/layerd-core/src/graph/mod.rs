pub mod arena;
pub mod edge;
pub mod hierarchical_edge;
pub mod index;
pub mod label;
pub mod node;
pub mod port;

use std::sync::Mutex;

use arena::Arena;
use edge::EdgeData;
use hashbrown::HashMap;
use index::{EdgeId, LabelId, NodeId, PortId};
use label::LabelData;
use node::{NodeData, NodeType};
use port::{PortData, PortEdges, PortSide, PortSideSet};
use smallvec::SmallVec;

type SplitAllMut<'a> = (
    &'a mut Arena<NodeData>,
    &'a mut Arena<PortData>,
    &'a mut Arena<EdgeData>,
    &'a mut Arena<LabelData>,
    &'a mut Vec<LayerData>,
);

/// Global allocator + registry of `LGraph::graph_id`.
///
/// Each `LGraph` instance is tagged with a unique 16-bit `graph_id`. The id
/// is encoded into every `ArenaId` produced inside that LGraph, which lets
/// `Arena::get` reject any cross-LGraph id at the arena layer (no more
/// silent "wrong slot" reads). The registry also serves as a graph_id →
/// `*mut LGraph` lookup, replacing the recursive DFS in
/// `find_graph_containing`.
///
/// IDs are recycled via a free-list when an `LGraph` is dropped. With 16
/// bits we support 65535 concurrently live LGraphs — far above any realistic
/// nesting depth + sibling fan-out.
struct GraphRegistry {
    slots: Vec<Option<std::ptr::NonNull<LGraph>>>,
    free_list: Vec<u16>,
}

// SAFETY: `*mut LGraph` is `!Send` by default, but the registry only stores
// raw pointers for read-only lookup; ownership stays with the `Box<LGraph>`
// and `LGraph::Drop` clears the slot before the box is freed. The `Mutex`
// serializes all access.
unsafe impl Send for GraphRegistry {}

static REGISTRY: Mutex<GraphRegistry> =
    Mutex::new(GraphRegistry { slots: Vec::new(), free_list: Vec::new() });

fn allocate_graph_id() -> u16 {
    let mut reg = REGISTRY.lock().expect("graph registry poisoned");
    if let Some(id) = reg.free_list.pop() {
        // Slot starts cleared; the LGraph::new caller will populate it after
        // `self` lands at its final address (set_self_ptr).
        return id;
    }
    let id = reg.slots.len();
    if id >= u16::MAX as usize {
        panic!("graph_id overflow: more than 65535 concurrently live LGraphs");
    }
    reg.slots.push(None);
    id as u16
}

fn release_graph_id(id: u16) {
    let mut reg = REGISTRY.lock().expect("graph registry poisoned");
    if let Some(slot) = reg.slots.get_mut(id as usize) {
        *slot = None;
    }
    reg.free_list.push(id);
}

fn register_graph_ptr(id: u16, ptr: std::ptr::NonNull<LGraph>) {
    let mut reg = REGISTRY.lock().expect("graph registry poisoned");
    if let Some(slot) = reg.slots.get_mut(id as usize) {
        *slot = Some(ptr);
    }
}

/// Resolve a `graph_id` to the LGraph reference. `O(1)`. Returns `None` if
/// the id is unknown, the slot has been cleared (LGraph already dropped), or
/// the LGraph hasn't yet finished moving to its final address.
///
/// SAFETY: callers must ensure no `&mut LGraph` borrow on the returned graph
/// is currently live. The registry stores raw pointers and does not enforce
/// this; cross-arena lookups in compound graph code respect this contract by
/// only reading immutable state.
pub fn graph_by_id(id: u16) -> Option<*const LGraph> {
    let reg = REGISTRY.lock().expect("graph registry poisoned");
    reg.slots
        .get(id as usize)
        .copied()
        .flatten()
        .map(|p| p.as_ptr() as *const LGraph)
}

pub fn graph_by_id_mut(id: u16) -> Option<*mut LGraph> {
    let reg = REGISTRY.lock().expect("graph registry poisoned");
    reg.slots.get(id as usize).copied().flatten().map(|p| p.as_ptr())
}

use crate::{
    math::{Padding, Vec2},
    options::{LayoutOptions, enums::EdgeLabelPlacement},
    properties::{
        PropertyMap,
        internal::{EDGE_LABEL_PLACEMENT, EXT_PORT_CONNECTIONS, EXT_PORT_SIDE},
    },
    rng::SeededRng,
};

/// Data for a single layer in the layered graph.
pub struct LayerData {
    pub nodes: Vec<NodeId>,
    pub size: Vec2,
    order_version: u64,
}

impl LayerData {
    pub fn new() -> Self {
        LayerData { nodes: Vec::new(), size: Vec2::ZERO, order_version: 0 }
    }
}

impl Default for LayerData {
    fn default() -> Self {
        Self::new()
    }
}

/// The core layered graph data structure.
///
/// Stores nodes, ports, edges, and labels in generational arenas, along with
/// layer assignments and graph-level metadata.
pub struct LGraph {
    nodes: Arena<NodeData>,
    ports: Arena<PortData>,
    edges: Arena<EdgeData>,
    labels: Arena<LabelData>,

    pub layers: Vec<LayerData>,
    pub layerless_nodes: Vec<NodeId>,

    pub size: Vec2,
    pub padding: Padding,
    pub offset: Vec2,
    pub parent_node: Option<NodeId>,

    pub options: LayoutOptions,
    pub properties: PropertyMap,

    /// Shared random source for the pipeline.
    ///
    /// One generator is constructed per `layout()` invocation and threaded
    /// through every phase so that consumption is monotonic across P1, P3,
    /// and any RNG-using intermediate processor. The generator is owned on
    /// `LGraph` and mutated in-place by phases that need it.
    /// and exposing take/put borrow-split helpers for processors that also
    /// need `&mut LGraph`.
    pub rng: SeededRng,

    pub graph_ports: SmallVec<PortId, 4>,
    pub graph_labels: SmallVec<LabelId, 2>,

    /// Cross-hierarchy edges awaiting preprocessing into local dummy edges.
    ///
    /// Populated only on the root `LGraph`. Drained by the compound preprocessor
    /// during Pre-P1; empty for downstream phases. See `hierarchical_edge` module.
    pub hierarchical_edges: Vec<hierarchical_edge::HierarchicalEdgeData>,

    /// Identifier-to-NodeId lookup table populated by source-graph importers.
    /// Empty for programmatically constructed graphs (zero allocation).
    pub identifier_map: HashMap<String, NodeId>,

    /// Globally unique tag identifying this LGraph instance. Encoded into
    /// every `ArenaId` minted by this graph's arenas, allowing the arena
    /// layer to reject cross-LGraph id leakage at lookup time.
    graph_id: u16,
    order_version: u64,

    next_node_id: u32,
}

impl LGraph {
    pub fn new() -> Self {
        let graph_id = allocate_graph_id();
        LGraph {
            nodes: Arena::new(graph_id),
            ports: Arena::new(graph_id),
            edges: Arena::new(graph_id),
            labels: Arena::new(graph_id),
            layers: Vec::new(),
            layerless_nodes: Vec::new(),
            size: Vec2::ZERO,
            padding: Padding::default(),
            offset: Vec2::ZERO,
            parent_node: None,
            options: LayoutOptions::default(),
            properties: PropertyMap::new(),
            rng: SeededRng::new(LayoutOptions::default().random_seed),
            graph_ports: SmallVec::new(),
            graph_labels: SmallVec::new(),
            hierarchical_edges: Vec::new(),
            identifier_map: HashMap::new(),
            graph_id,
            order_version: 0,
            next_node_id: 0,
        }
    }

    /// Returns this graph's unique 16-bit identifier. Used by the
    /// `find_graph_containing`-style helpers to dispatch by id rather than
    /// DFS-walking the nested tree.
    pub fn graph_id(&self) -> u16 {
        self.graph_id
    }

    /// Monotonic version for P3-relevant node and port order changes.
    ///
    /// Crossing minimization score caches key off this value instead of
    /// fingerprinting every layer and port list. Callers must bump it only
    /// after a real order mutation, and parent graphs must bump when a nested
    /// graph's version changes.
    pub fn order_version(&self) -> u64 {
        self.order_version
    }

    pub fn layer_order_version(&self, layer_idx: usize) -> u64 {
        self.layers.get(layer_idx).map(|layer| layer.order_version).unwrap_or(0)
    }

    pub fn bump_order_version(&mut self) {
        self.order_version = self.order_version.wrapping_add(1);
    }

    pub fn bump_layer_order_version(&mut self, layer_idx: usize) {
        self.bump_order_version();
        if let Some(layer) = self.layers.get_mut(layer_idx) {
            layer.order_version = layer.order_version.wrapping_add(1);
        }
    }

    pub fn bump_node_order_version(&mut self, node_id: NodeId) {
        self.bump_order_version();
        let layer_idx = self.node(node_id).layer.get();
        if let Some(layer_idx) = layer_idx
            && let Some(layer) = self.layers.get_mut(layer_idx)
        {
            layer.order_version = layer.order_version.wrapping_add(1);
        }
    }

    pub fn bump_all_layer_order_versions(&mut self) {
        self.bump_order_version();
        for layer in &mut self.layers {
            layer.order_version = layer.order_version.wrapping_add(1);
        }
    }

    /// Register this graph's heap address with the global registry. Must be
    /// called by any code that boxes an LGraph and wants `graph_by_id` to
    /// resolve it (i.e. cross-LGraph lookups via `find_graph_containing`).
    /// The address must remain stable for the LGraph's lifetime — `set_nested`
    /// uses `Box::into_raw` which guarantees this; `LGraph::new()` itself
    /// returns a stack value that the caller may move, so the registry is
    /// updated lazily by `set_nested`/`set_nested_boxed` and by an explicit
    /// `register_self_ptr` for root-level graphs (they live in the caller's
    /// stack/box for the duration of `layout`).
    pub fn register_self_ptr(&mut self) {
        // SAFETY: `self` is a valid reference; we just type-cast the pointer
        // for storage. The registry stores it as `*mut LGraph` and the
        // `Drop` impl unregisters before the storage goes away.
        let ptr = unsafe { std::ptr::NonNull::new_unchecked(self as *mut LGraph) };
        register_graph_ptr(self.graph_id, ptr);
    }

    /// Look up a node by its source-graph identifier string.
    pub fn node_by_identifier(&self, identifier: &str) -> Option<NodeId> {
        self.identifier_map.get(identifier).copied()
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    pub fn node(&self, id: NodeId) -> &NodeData {
        self.nodes.get(id.arena_id()).expect("invalid NodeId")
    }

    pub fn node_mut(&mut self, id: NodeId) -> &mut NodeData {
        self.nodes.get_mut(id.arena_id()).expect("invalid NodeId")
    }

    /// Non-panicking variant of [`node`]: returns `None` when the NodeId
    /// belongs to a different LGraph's arena.
    ///
    /// Compound-graph state (`dummy_node_map`, `BARYCENTER_ASSOCIATES`, and
    /// similar side-channels) can carry NodeIds that were minted inside a
    /// nested graph's arena. Callers that might be handed such a cross-arena
    /// NodeId should branch on this method rather than `node` to avoid a
    /// panic; `find_graph_containing` (or `walk_nested` with a known path)
    /// can route the lookup to the owning LGraph when the caller needs the
    /// real data.
    pub fn try_node(&self, id: NodeId) -> Option<&NodeData> {
        self.nodes.get(id.arena_id())
    }

    /// Mutable companion of [`Self::try_node`].
    pub fn try_node_mut(&mut self, id: NodeId) -> Option<&mut NodeData> {
        self.nodes.get_mut(id.arena_id())
    }

    pub fn port(&self, id: PortId) -> &PortData {
        self.ports.get(id.arena_id()).expect("invalid PortId")
    }

    pub fn port_owner(&self, id: PortId) -> NodeId {
        if id.0.graph_id() == self.graph_id {
            return self.port(id).owner;
        }
        let ptr = graph_by_id(id.0.graph_id()).expect("invalid cross-graph PortId");
        // SAFETY: the registry pointer is cleared by `LGraph::Drop`; callers
        // only use this for ids reachable from the current graph hierarchy.
        unsafe { (&*ptr).port(id).owner }
    }

    pub fn port_mut(&mut self, id: PortId) -> &mut PortData {
        self.ports.get_mut(id.arena_id()).expect("invalid PortId")
    }

    /// Non-panicking variant of [`Self::port`]: see [`Self::try_node`] for the
    /// motivation.
    pub fn try_port(&self, id: PortId) -> Option<&PortData> {
        self.ports.get(id.arena_id())
    }

    /// Mutable companion of [`Self::try_port`].
    pub fn try_port_mut(&mut self, id: PortId) -> Option<&mut PortData> {
        self.ports.get_mut(id.arena_id())
    }

    pub fn edge(&self, id: EdgeId) -> &EdgeData {
        self.edges.get(id.arena_id()).expect("invalid EdgeId")
    }

    pub fn try_edge(&self, id: EdgeId) -> Option<&EdgeData> {
        self.edges.get(id.arena_id())
    }

    pub fn edge_mut(&mut self, id: EdgeId) -> &mut EdgeData {
        self.edges.get_mut(id.arena_id()).expect("invalid EdgeId")
    }

    pub fn label(&self, id: LabelId) -> &LabelData {
        self.labels.get(id.arena_id()).expect("invalid LabelId")
    }

    pub fn label_mut(&mut self, id: LabelId) -> &mut LabelData {
        self.labels.get_mut(id.arena_id()).expect("invalid LabelId")
    }

    /// Group `node_id`'s ports into N→E→S→W→Undefined order (stable within each
    /// side) and record cumulative end indices in `NodeData.port_side_ends`.
    /// After this returns, P3 cross-min hot paths can call
    /// `node.ports_on_side(side)` for an O(1) slice instead of filtering.
    ///
    /// Caches per-side port indices in the node's properties. Idempotent.
    ///
    /// Fast-path: a single linear scan computes per-side counts and detects
    /// whether the existing port order is already grouped (the post-PortListSorter
    /// case for side-fixed nodes). When grouped, we skip the re-sort and the
    /// write-back entirely.
    pub fn cache_port_sides(&mut self, node_id: NodeId) {
        let n = self.node(node_id).ports.len();
        debug_assert!(n < u16::MAX as usize, "port count exceeds cache range");

        let mut counts = [0u16; 5];
        let mut already_grouped = true;
        let mut last_side_idx = 0usize;
        for &port_id in &self.node(node_id).ports {
            let s = node::port_side_table_index(self.port(port_id).side);
            counts[s] += 1;
            if s < last_side_idx {
                already_grouped = false;
            }
            last_side_idx = s;
        }

        let mut ends = [0u16; 5];
        let mut acc: u16 = 0;
        for i in 0..5 {
            acc += counts[i];
            ends[i] = acc;
        }
        debug_assert_eq!(ends[4] as usize, n);

        if !already_grouped {
            let mut keyed: SmallVec<(PortId, u8), 6> = SmallVec::with_capacity(n);
            for &port_id in &self.node(node_id).ports {
                let s = node::port_side_table_index(self.port(port_id).side) as u8;
                keyed.push((port_id, s));
            }
            keyed.sort_by_key(|&(_, s)| s);
            let node = self.node_mut(node_id);
            node.ports.clear();
            for &(p, _) in &keyed {
                node.ports.push(p);
            }
            node.port_side_ends = ends;
        } else {
            self.node_mut(node_id).port_side_ends = ends;
        }
    }

    //
    // `NodeData.nested_graph` stores a raw owning pointer (via `Box::into_raw`) rather
    // than `Option<Box<LGraph>>` so that `&mut self` borrows of the parent do not lock
    // the child. These helpers encapsulate the `unsafe` dereference. The pointer is
    // guaranteed to remain valid until `NodeData::drop` or `take_nested` releases it.

    /// Walk a chain of nested graphs, one `NodeId` per step. Each
    /// `path[i]` must identify a node in the LGraph reached so far whose
    /// `nested_graph` slot is populated. The empty path yields `self`.
    /// Returns `None` the first time a step fails.
    pub fn walk_nested(&self, path: &[NodeId]) -> Option<&LGraph> {
        let mut g = self;
        for &nid in path {
            g = g.nested(nid)?;
        }
        Some(g)
    }

    /// Mutable variant of [`Self::walk_nested`].
    pub fn walk_nested_mut(&mut self, path: &[NodeId]) -> Option<&mut LGraph> {
        let mut graph = self as *mut LGraph;
        for &nid in path {
            // SAFETY: `graph` always points to the current unique mutable graph
            // in the path. Each loop iteration releases the previous temporary
            // borrow before descending into the next nested graph.
            graph = unsafe { (&mut *graph).nested_mut(nid)? as *mut LGraph };
        }
        // SAFETY: the final pointer is inside the unique mutable subtree rooted
        // at `self`, and no intermediate borrow is still live.
        Some(unsafe { &mut *graph })
    }

    /// Return a shared reference to the nested graph attached to `id`, if any.
    pub fn nested(&self, id: NodeId) -> Option<&LGraph> {
        // SAFETY: the pointer was created by `Box::into_raw` in `set_nested` and remains
        // valid until `take_nested` or `NodeData::drop` releases it. The `&self` borrow
        // proves no exclusive access is outstanding, and we return a borrow tied to `&self`.
        self.node(id).nested_graph.map(|p| unsafe { p.as_ref() })
    }

    /// Return a mutable reference to the nested graph attached to `id`, if any.
    pub fn nested_mut(&mut self, id: NodeId) -> Option<&mut LGraph> {
        // SAFETY: same lifetime invariant as `nested`. The `&mut self` borrow proves
        // exclusive access to this `NodeData`, which transitively means exclusive
        // access to the child allocation through the stored pointer.
        self.node_mut(id).nested_graph.map(|mut p| unsafe { p.as_mut() })
    }

    /// Attach `inner` as the nested graph of `id`, dropping any existing nested graph.
    ///
    /// Records the parent NodeId on the nested graph so that downstream code
    /// (configurator gates, hierarchical resizer write-back) can detect the
    /// enclosing scope.
    pub fn set_nested(&mut self, id: NodeId, mut inner: LGraph) {
        inner.parent_node = Some(id);
        self.take_nested(id);
        let inner_graph_id = inner.graph_id;
        let ptr = Box::into_raw(Box::new(inner));
        // SAFETY: `Box::into_raw` never returns null.
        let nn = unsafe { std::ptr::NonNull::new_unchecked(ptr) };
        // Now that the LGraph lives at a stable heap address, register it
        // in the global graph_id → ptr table. The registry is consulted by
        // `find_graph_containing` for O(1) cross-LGraph lookups.
        register_graph_ptr(inner_graph_id, nn);
        self.node_mut(id).nested_graph = Some(nn);
    }

    /// Detach and return the nested graph owned by `id`, if any.
    pub fn take_nested(&mut self, id: NodeId) -> Option<LGraph> {
        let ptr = self.node_mut(id).nested_graph.take()?;
        // SAFETY: after `take()` the slot is `None`, so `Drop` will not re-free the
        // allocation. Reconstructing the `Box` and moving its contents out is sound.
        Some(*unsafe { Box::from_raw(ptr.as_ptr()) })
    }

    /// Detach the nested graph owned by `id` without moving it out of its `Box`.
    ///
    /// Preserves the original `Box::into_raw` pointer so callers who cached a
    /// `NonNull<LGraph>` at that address remain valid after a matching
    /// `set_nested_boxed` restore.
    pub fn take_nested_boxed(&mut self, id: NodeId) -> Option<Box<LGraph>> {
        let ptr = self.node_mut(id).nested_graph.take()?;
        // SAFETY: pointer came from `Box::into_raw` in `set_nested{,_boxed}`; the
        // slot is now `None`, so `Drop` will not double-free.
        Some(unsafe { Box::from_raw(ptr.as_ptr()) })
    }

    /// Re-attach a previously `take_nested_boxed`-ed graph without reallocating.
    ///
    /// The `Box`'s heap pointer is preserved, so a `NonNull<LGraph>` that the
    /// caller cached at the original address remains valid.
    pub fn set_nested_boxed(&mut self, id: NodeId, mut inner: Box<LGraph>) {
        inner.parent_node = Some(id);
        // Drop any currently attached nested graph at `id`.
        let _ = self.take_nested_boxed(id);
        let inner_graph_id = inner.graph_id;
        let ptr = Box::into_raw(inner);
        // SAFETY: `Box::into_raw` never returns null.
        let nn = unsafe { std::ptr::NonNull::new_unchecked(ptr) };
        register_graph_ptr(inner_graph_id, nn);
        self.node_mut(id).nested_graph = Some(nn);
    }

    /// Returns `true` if `id` has a nested graph attached.
    pub fn has_nested(&self, id: NodeId) -> bool {
        self.node(id).nested_graph.is_some()
    }

    /// Returns `true` if `descendant` is nested inside `ancestor` (direct or indirect),
    /// or if `descendant == ancestor`.
    ///
    /// Walks the `parent_node` chain through nested graphs via the global
    /// `LGraph::parent_node` field.
    ///
    /// NOTE: only compares against ancestors reachable via `parent_node`. A node whose
    /// owning graph does not record a `parent_node` (i.e. the root graph) terminates
    /// the walk.
    pub fn is_descendant(&self, descendant: NodeId, ancestor: NodeId) -> bool {
        // `ancestor` may live in a different LGraph's arena. Resolve via the
        // global registry rather than panicking through `self.node(ancestor)`.
        let ancestor_graph = match self.find_graph_containing(ancestor) {
            Some(g) => g,
            None => return false,
        };
        if ancestor_graph.try_node(ancestor).and_then(|n| n.nested_graph).is_none() {
            return false;
        }
        // Walk graph upward from the graph containing `descendant` until we find one
        // whose parent_node equals `ancestor`, or we exhaust the chain.
        let mut current_graph: &LGraph = match self.find_graph_containing(descendant) {
            Some(g) => g,
            None => return false,
        };
        loop {
            match current_graph.parent_node {
                Some(p) if p == ancestor => return true,
                Some(p) => match self.find_graph_containing(p) {
                    Some(g) => current_graph = g,
                    None => return false,
                },
                None => return false,
            }
        }
    }

    /// Locate the LGraph (self or any transitive nested graph) that owns `node`.
    ///
    /// Every `NodeId` carries the `graph_id` of its owning LGraph, so we can
    /// dispatch directly via the global registry — `O(1)` table lookup
    /// instead of the previous `O(total_nodes)` DFS. The fallback DFS
    /// remains for pre-Drop pointer validity edge cases.
    pub fn find_graph_containing(&self, node: NodeId) -> Option<&LGraph> {
        let g_id = node.0.graph_id();
        if g_id == self.graph_id {
            return self.try_node(node).map(|_| self);
        }
        // Registry hit: jump straight to the owning LGraph in O(1).
        let ptr = graph_by_id(g_id)?;
        // SAFETY: the registry holds a pointer cleared by `LGraph::Drop`
        // before the storage goes away. We return a borrow tied to `&self`
        // because all reachable LGraphs in a single `layerd::layout` call
        // outlive `self` (they hang off `self.nested_graph` via `Box`).
        let g = unsafe { &*ptr };
        g.try_node(node).map(|_| g)
    }

    /// Translate `point` from the local coordinate system of `from_graph` into the
    /// local coordinate system of `to_graph`, walking up via `parent_node` through
    /// `root`.
    ///
    /// The walk is a two-step transform: first lift `point` to `root`'s
    /// coordinate frame (absolute), then descend from `root` into `to_graph`
    /// (relative). Both graphs must be reachable from `root` via the
    /// `nested_graph` / `parent_node` spine.
    pub(crate) fn change_coord_system(
        root: &LGraph,
        point: &mut Vec2,
        from_graph: *const LGraph,
        to_graph: *const LGraph,
    ) {
        if std::ptr::eq(from_graph, to_graph) {
            return;
        }
        // SAFETY: callers pass pointers obtained via `LGraph::nested` or `root` itself,
        // both of which remain valid for the lifetime of `root`.
        let from = unsafe { &*from_graph };
        let to = unsafe { &*to_graph };
        // Step 1: accumulate offsets walking `from` toward the root.
        let mut graph: &LGraph = from;
        loop {
            point.x += graph.offset.x;
            point.y += graph.offset.y;
            match graph.parent_node {
                Some(p) => {
                    point.x += graph.padding.left;
                    point.y += graph.padding.top;
                    let parent_graph = match root.find_graph_containing(p) {
                        Some(g) => g,
                        None => break,
                    };
                    let parent_pos = parent_graph.node(p).position;
                    point.x += parent_pos.x;
                    point.y += parent_pos.y;
                    graph = parent_graph;
                }
                None => break,
            }
        }
        // Step 2: subtract offsets walking `to` toward the root.
        let mut graph: &LGraph = to;
        loop {
            point.x -= graph.offset.x;
            point.y -= graph.offset.y;
            match graph.parent_node {
                Some(p) => {
                    point.x -= graph.padding.left;
                    point.y -= graph.padding.top;
                    let parent_graph = match root.find_graph_containing(p) {
                        Some(g) => g,
                        None => break,
                    };
                    let parent_pos = parent_graph.node(p).position;
                    point.x -= parent_pos.x;
                    point.y -= parent_pos.y;
                    graph = parent_graph;
                }
                None => break,
            }
        }
    }

    /// Returns the absolute anchor position of `port` relative to its containing graph.
    ///
    /// Returns `owner.position + port.position + port.anchor`.
    pub fn absolute_anchor(&self, port: PortId) -> Vec2 {
        let p = self.port(port);
        let owner_pos = self.node(p.owner).position;
        Vec2 {
            x: owner_pos.x + p.position.x + p.anchor.x,
            y: owner_pos.y + p.position.y + p.anchor.y,
        }
    }

    pub fn add_node(&mut self, size: Vec2) -> NodeId {
        let id = self.next_node_id;
        self.next_node_id += 1;
        let node_data = NodeData::new(size, id);
        let arena_id = self.nodes.insert(node_data);
        let node_id = NodeId(arena_id);
        self.layerless_nodes.push(node_id);
        node_id
    }

    pub fn add_port(&mut self, node: NodeId, side: PortSide) -> PortId {
        let port_data = PortData::new(node, side);
        let arena_id = self.ports.insert(port_data);
        let port_id = PortId(arena_id);
        self.node_mut(node).ports.push(port_id);
        port_id
    }

    pub fn add_edge(&mut self, source: PortId, target: PortId) -> EdgeId {
        let edge_data =
            EdgeData::new(source, target, self.port_owner(source), self.port_owner(target));
        let arena_id = self.edges.insert(edge_data);
        let edge_id = EdgeId(arena_id);
        self.port_mut(source).outgoing_edges.push(edge_id);
        self.port_mut(target).incoming_edges.push(edge_id);
        edge_id
    }

    /// Insert an `EdgeData` that is *not* registered on its source / target
    /// ports' adjacency lists. Used by the compound preprocessor to stage a
    /// "proxy" bookkeeping edge whose endpoints may live in a different
    /// LGraph arena (cross-hierarchy edges); the proxy carries labels and
    /// the eventual reassembled bend-point chain, but it should not double
    /// up on any port's outgoing / incoming lists nor attempt a cross-arena
    /// `port_mut` call.
    ///
    /// The `source` / `target` fields on the resulting `EdgeData` still
    /// hold the caller-supplied `PortId`s for later inspection, but the
    /// caller is responsible for treating them as metadata rather than
    /// live adjacency. Pair with `relink_edge` (walking through
    /// `find_graph_containing`) when the proxy should be connected to real
    /// ports again.
    pub fn add_edge_orphan(&mut self, source: PortId, target: PortId) -> EdgeId {
        let edge_data =
            EdgeData::new(source, target, self.port_owner(source), self.port_owner(target));
        let arena_id = self.edges.insert(edge_data);
        EdgeId(arena_id)
    }

    pub fn add_node_label(&mut self, node: NodeId, text: impl Into<String>, size: Vec2) -> LabelId {
        let label_data = LabelData::new(text, size);
        let arena_id = self.labels.insert(label_data);
        let label_id = LabelId(arena_id);
        self.node_mut(node).labels.push(label_id);
        label_id
    }

    pub fn add_port_label(&mut self, port: PortId, text: impl Into<String>, size: Vec2) -> LabelId {
        let label_data = LabelData::new(text, size);
        let arena_id = self.labels.insert(label_data);
        let label_id = LabelId(arena_id);
        self.port_mut(port).labels.push(label_id);
        label_id
    }

    pub fn add_edge_label(&mut self, edge: EdgeId, text: impl Into<String>, size: Vec2) -> LabelId {
        let label_data = LabelData::new(text, size);
        let arena_id = self.labels.insert(label_data);
        let label_id = LabelId(arena_id);
        self.edge_mut(edge).labels.push(label_id);
        label_id
    }

    //
    // Stored only on the root `LGraph`. The compound preprocessor drains this
    // list during Pre-P1 and replaces each entry with one or more local dummy
    // edges plus external-port dummies via `transform_hierarchy_edges`. Downstream
    // phases never see hierarchical edges directly.

    /// Append a hierarchical edge whose endpoints may live in different
    /// hierarchy levels. Returns the slot index for callers that need to
    /// re-acquire the edge before the preprocessor runs.
    ///
    /// Source/target endpoints use [`hierarchical_edge::HierarchicalPortRef`]
    /// which qualifies a `PortId` with the `Option<NodeId>` of the parent node
    /// whose `nested_graph` arena owns that port (`None` = this graph itself).
    pub fn add_hierarchical_edge(
        &mut self,
        source: hierarchical_edge::HierarchicalPortRef,
        target: hierarchical_edge::HierarchicalPortRef,
    ) -> index::HierarchicalEdgeId {
        let id = index::HierarchicalEdgeId(self.hierarchical_edges.len() as u32);
        self.hierarchical_edges
            .push(hierarchical_edge::HierarchicalEdgeData::new(source, target));
        id
    }

    /// Iterate hierarchical edges currently stored on this graph.
    pub fn hierarchical_edges_iter(
        &self,
    ) -> impl Iterator<Item = (index::HierarchicalEdgeId, &hierarchical_edge::HierarchicalEdgeData)> + '_
    {
        self.hierarchical_edges
            .iter()
            .enumerate()
            .map(|(i, e)| (index::HierarchicalEdgeId(i as u32), e))
    }

    /// Detach all hierarchical edges, returning ownership to the caller.
    /// Used by the compound preprocessor at Pre-P1 time.
    pub fn take_hierarchical_edges(&mut self) -> Vec<hierarchical_edge::HierarchicalEdgeData> {
        std::mem::take(&mut self.hierarchical_edges)
    }

    /// Resolve a `HierarchicalPortRef` to the `&LGraph` containing that port,
    /// or `None` if the parent graph cannot be located.
    pub fn resolve_hierarchical_port_graph(
        &self,
        r: hierarchical_edge::HierarchicalPortRef,
    ) -> Option<&LGraph> {
        match r.graph_parent {
            None => Some(self),
            Some(parent) => self.find_graph_containing(parent).and_then(|g| g.nested(parent)),
        }
    }

    /// Mutable variant of `resolve_hierarchical_port_graph`.
    ///
    /// Walks the hierarchy from the root via raw pointer to avoid the
    /// parent/child borrow chain.
    pub fn resolve_hierarchical_port_graph_mut(
        &mut self,
        r: hierarchical_edge::HierarchicalPortRef,
    ) -> Option<&mut LGraph> {
        match r.graph_parent {
            None => Some(self),
            Some(parent) => {
                let containing_ptr: *mut LGraph = find_containing_graph_ptr(self, parent)?;
                // SAFETY: pointer obtained synchronously, no aliasing borrow held.
                let containing = unsafe { &mut *containing_ptr };
                containing.nested_mut(parent)
            }
        }
    }

    /// Add a root-level (graph-owned) port that is not attached to any node.
    ///
    /// The port is stored in the arena and registered in `graph_ports`.
    /// Its `owner` field is set to a sentinel NodeId that must not be dereferenced.
    pub fn add_graph_port(&mut self, side: PortSide) -> PortId {
        // Create a sentinel NodeId. This port has no owning node.
        let sentinel = NodeId(arena::ArenaId::sentinel());
        let port_data = PortData::new(sentinel, side);
        let arena_id = self.ports.insert(port_data);
        let port_id = PortId(arena_id);
        self.graph_ports.push(port_id);
        port_id
    }

    /// Add a root-level (graph-owned) label.
    ///
    /// The label is stored in the arena and registered in `graph_labels`.
    pub fn add_graph_label(&mut self, text: impl Into<String>, size: Vec2) -> LabelId {
        let label_data = LabelData::new(text, size);
        let arena_id = self.labels.insert(label_data);
        let label_id = LabelId(arena_id);
        self.graph_labels.push(label_id);
        label_id
    }

    /// Set the layout options for this graph.
    pub fn set_options(&mut self, options: LayoutOptions) {
        self.options = options;
    }

    /// Reinitialize the shared RNG from `options.random_seed`.
    ///
    /// Called by the `layout()` entry to honour the one-RNG-per-invocation
    /// contract, and by test helpers that simulate a top-level layout entry.
    pub fn reseed_from_options(&mut self) {
        self.rng = SeededRng::new(self.options.random_seed);
    }

    /// Move the shared RNG out of the graph for exclusive use by a processor.
    ///
    /// Pair with [`LGraph::put_rng`]. The placeholder left behind must not be
    /// observed by any code running before `put_rng` restores the real state.
    pub fn take_rng(&mut self) -> SeededRng {
        std::mem::replace(&mut self.rng, SeededRng::new(0))
    }

    /// Restore the shared RNG after a processor finished consuming it.
    pub fn put_rng(&mut self, rng: SeededRng) {
        self.rng = rng;
    }

    /// Reverse an edge's direction, swapping its source and target ports.
    ///
    /// Updates port adjacency lists and toggles the REVERSED flag.
    ///
    /// Hierarchical edges (see [`hierarchical_edge`]) are not affected — they
    /// have no local `EdgeId` until the compound preprocessor materialises them
    /// at Pre-P1 time, after which the resulting local segments behave like any
    /// other edge.
    pub fn reverse_edge(&mut self, edge_id: EdgeId) {
        self.reverse_edge_inner(edge_id, false);
    }

    /// Like [`Self::reverse_edge`] but respects `INPUT_COLLECT` / `OUTPUT_COLLECT`
    /// collector ports: when the old target is an input-collector, the new
    /// source is rerouted to the same node's output-collector (creating one if
    /// it does not exist); symmetric for `OUTPUT_COLLECT` on the old source.
    ///
    /// Cycle breakers (P1), `partition`, and `edge_and_layer_constraint`
    /// should use this variant so collector ports are respected.
    pub fn reverse_edge_adapt_ports(&mut self, edge_id: EdgeId) {
        self.reverse_edge_inner(edge_id, true);
    }

    fn reverse_edge_inner(&mut self, edge_id: EdgeId, adapt_ports: bool) {
        use crate::properties::internal::{INPUT_COLLECT, OUTPUT_COLLECT};
        let edge = self.edge(edge_id);
        let old_source = edge.source;
        let old_target = edge.target;

        // Resolve collector ports on the old endpoints before detaching. Only
        // meaningful when adapt_ports is true and the old port actually has
        // the collector property set; otherwise the fallback is the old
        // opposite endpoint (standard swap).
        let new_source = if adapt_ports && self.port(old_target).properties.get(&INPUT_COLLECT) {
            let node = self.port(old_target).owner;
            self.provide_collector_port_output(node)
        } else {
            old_target
        };
        let new_target = if adapt_ports && self.port(old_source).properties.get(&OUTPUT_COLLECT) {
            let node = self.port(old_source).owner;
            self.provide_collector_port_input(node)
        } else {
            old_source
        };

        // Remove from old port lists
        self.port_mut(old_source).outgoing_edges.retain(|e| *e != edge_id);
        self.port_mut(old_target).incoming_edges.retain(|e| *e != edge_id);

        // Apply new source / target on the edge.
        let new_source_owner = self.port_owner(new_source);
        let new_target_owner = self.port_owner(new_target);
        let edge = self.edge_mut(edge_id);
        edge.source = new_source;
        edge.target = new_target;
        edge.source_owner = new_source_owner;
        edge.target_owner = new_target_owner;
        std::mem::swap(&mut edge.start_point, &mut edge.end_point);
        edge.bend_points.reverse();

        // Switch end labels: edge reversal flips HEAD<->TAIL on
        // EDGE_LABEL_PLACEMENT for each of the edge's labels.
        let label_ids: SmallVec<LabelId, 3> = self.edge(edge_id).labels.iter().copied().collect();
        for label_id in label_ids {
            let placement = self.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT);
            match placement {
                EdgeLabelPlacement::Head => {
                    self.label_mut(label_id)
                        .properties
                        .set(&EDGE_LABEL_PLACEMENT, EdgeLabelPlacement::Tail);
                }
                EdgeLabelPlacement::Tail => {
                    self.label_mut(label_id)
                        .properties
                        .set(&EDGE_LABEL_PLACEMENT, EdgeLabelPlacement::Head);
                }
                _ => {}
            }
        }

        // Add to new port lists.
        self.port_mut(new_source).outgoing_edges.push(edge_id);
        self.port_mut(new_target).incoming_edges.push(edge_id);

        // Toggle REVERSED flag
        self.edge_mut(edge_id).flags.toggle(edge::EdgeFlags::REVERSED);
    }

    /// Return an existing `OUTPUT_COLLECT` port on `node` or create one on its
    /// east side.
    fn provide_collector_port_output(&mut self, node: NodeId) -> PortId {
        use crate::properties::internal::OUTPUT_COLLECT;
        for &pid in &self.node(node).ports.clone() {
            if self.port(pid).properties.get(&OUTPUT_COLLECT) {
                return pid;
            }
        }
        let new_port = self.add_port(node, port::PortSide::East);
        self.port_mut(new_port).properties.set(&OUTPUT_COLLECT, true);
        new_port
    }

    /// Return an existing `INPUT_COLLECT` port on `node` or create one on its
    /// west side.
    fn provide_collector_port_input(&mut self, node: NodeId) -> PortId {
        use crate::properties::internal::INPUT_COLLECT;
        for &pid in &self.node(node).ports.clone() {
            if self.port(pid).properties.get(&INPUT_COLLECT) {
                return pid;
            }
        }
        let new_port = self.add_port(node, port::PortSide::West);
        self.port_mut(new_port).properties.set(&INPUT_COLLECT, true);
        new_port
    }

    //
    // These methods decompose `&mut LGraph` into disjoint borrows of its fields,
    // allowing simultaneous read access to layer node lists while mutating arena
    // contents. This eliminates the need for `Vec<NodeId>` clones that exist
    // solely to work around the borrow checker.

    /// Split-borrow: mutable node arena + immutable layer slice.
    ///
    /// Allows iterating `layers[i].nodes` while calling `nodes.get_mut(id)`.
    pub fn split_nodes_layers(&mut self) -> (&mut Arena<NodeData>, &[LayerData]) {
        (&mut self.nodes, &self.layers)
    }

    /// Split-borrow: mutable node arena + mutable layer slice.
    pub fn split_nodes_layers_mut(&mut self) -> (&mut Arena<NodeData>, &mut [LayerData]) {
        (&mut self.nodes, &mut self.layers)
    }

    /// Split-borrow: mutable port arena + immutable node arena.
    ///
    /// Allows iterating a node's ports while mutating port data.
    pub fn split_ports_nodes(&mut self) -> (&mut Arena<PortData>, &Arena<NodeData>) {
        (&mut self.ports, &self.nodes)
    }

    /// Split-borrow: mutable node arena + mutable port arena.
    ///
    /// Allows mutating both nodes and ports simultaneously.
    pub fn split_nodes_ports(&mut self) -> (&mut Arena<NodeData>, &mut Arena<PortData>) {
        (&mut self.nodes, &mut self.ports)
    }

    /// Split-borrow: mutable edge arena + immutable port/node arenas + layers.
    pub fn split_edges(
        &mut self,
    ) -> (&mut Arena<EdgeData>, &Arena<PortData>, &Arena<NodeData>, &[LayerData]) {
        (&mut self.edges, &self.ports, &self.nodes, &self.layers)
    }

    /// Split-borrow: all four arenas + layers, fully decomposed.
    ///
    /// Use when an algorithm needs simultaneous access to multiple arenas.
    pub fn split_all(&mut self) -> SplitAllMut<'_> {
        (
            &mut self.nodes,
            &mut self.ports,
            &mut self.edges,
            &mut self.labels,
            &mut self.layers,
        )
    }

    /// Returns an iterator over all incoming edge IDs for a given node.
    ///
    /// Iterates over all ports of the node and collects their incoming edges.
    pub fn incoming_edges(&self, node: NodeId) -> impl Iterator<Item = EdgeId> + '_ {
        self.node(node)
            .ports
            .iter()
            .flat_map(|port_id| self.port(*port_id).incoming_edges.iter().copied())
    }

    /// Returns an iterator over all outgoing edge IDs for a given node.
    ///
    /// Iterates over all ports of the node and collects their outgoing edges.
    pub fn outgoing_edges(&self, node: NodeId) -> impl Iterator<Item = EdgeId> + '_ {
        self.node(node)
            .ports
            .iter()
            .flat_map(|port_id| self.port(*port_id).outgoing_edges.iter().copied())
    }

    /// Returns an iterator over all nodes and their data.
    pub fn nodes_iter(&self) -> impl Iterator<Item = (NodeId, &NodeData)> {
        self.nodes.iter().map(|(id, data)| (NodeId(id), data))
    }

    /// Returns an iterator over all edges and their data.
    pub fn edges_iter(&self) -> impl Iterator<Item = (EdgeId, &EdgeData)> {
        self.edges.iter().map(|(id, data)| (EdgeId(id), data))
    }

    /// Returns an iterator over all labels and their data.
    pub fn labels_iter(&self) -> impl Iterator<Item = (LabelId, &LabelData)> {
        self.labels.iter().map(|(id, data)| (LabelId(id), data))
    }

    /// Returns a mutable iterator over all nodes.
    pub fn nodes_iter_mut(&mut self) -> impl Iterator<Item = (NodeId, &mut NodeData)> {
        self.nodes.iter_mut().map(|(id, data)| (NodeId(id), data))
    }

    /// Yield `(NodeId, &mut nested_graph)` for every node that has a nested
    /// graph attached, without allocating an intermediate `Vec<NodeId>`.
    ///
    /// Each nested graph is a separate `Box`-allocated `LGraph`, so mutating
    /// it through the yielded `&mut LGraph` does not alias with any field of
    /// `self` and does not alias with any other nested graph. The iterator
    /// holds an exclusive borrow of `self.nodes`, so while iterating no
    /// `set_nested`/`take_nested`/`nodes_iter_mut` call on `self` is possible.
    pub fn nested_graphs_mut(&mut self) -> impl Iterator<Item = (NodeId, &mut LGraph)> {
        self.nodes.iter_mut().filter_map(|(id, data)| {
            let ptr = data.nested_graph?;
            // SAFETY: `ptr` was installed by `set_nested` via `Box::into_raw`
            // and remains valid until `take_nested`/drop. The nested allocation
            // is disjoint from `self.nodes` entries, and `iter_mut` gives us
            // exclusive access to this slot, so we can safely hand out a
            // `&mut LGraph` reborrowed from the pointer.
            Some((NodeId(id), unsafe { &mut *ptr.as_ptr() }))
        })
    }

    /// Yield raw `(NodeId, NonNull<LGraph>)` for every node that owns a
    /// nested graph, without producing a `&mut LGraph`.
    ///
    /// Used by callers that need to build child→parent maps across the
    /// hierarchy and cannot hold concurrent `&mut LGraph` to multiple
    /// nested levels (the `nested_graphs_mut` iterator borrows `self`).
    pub fn nested_node_pointers(
        &self,
    ) -> impl Iterator<Item = (NodeId, std::ptr::NonNull<LGraph>)> + '_ {
        self.nodes
            .iter()
            .filter_map(|(id, data)| Some((NodeId(id), data.nested_graph?)))
    }

    /// Returns a mutable iterator over all ports.
    pub fn ports_iter_mut(&mut self) -> impl Iterator<Item = (PortId, &mut port::PortData)> {
        self.ports.iter_mut().map(|(id, data)| (PortId(id), data))
    }

    /// Returns an iterator over all ports.
    pub fn ports_iter(&self) -> impl Iterator<Item = (PortId, &port::PortData)> {
        self.ports.iter().map(|(id, data)| (PortId(id), data))
    }

    /// Split `self` into one `LGraph` per weakly connected component of the
    /// layerless node set, draining the original's arenas.
    ///
    /// Operates directly on the private arenas rather than going through
    /// `remove_node`, which does not cascade to the node's ports and edges.
    /// After this returns, `self.nodes`, `self.ports`, `self.edges`, `self.labels`, and
    /// `self.layerless_nodes` are empty; `self.options`, `self.padding`, and
    /// `self.properties` remain in place so the caller can clone them into
    /// the per-component graphs and later re-populate `self` from the
    /// combined result.
    ///
    /// `self.identifier_map` is drained across the returned components so
    /// each component's local `node_by_identifier` lookups continue to
    /// resolve, at the cost of the root losing its map until `combine`
    /// reassembles it.
    ///
    /// Callers must uphold the gate: no split on compound graphs and no split
    /// when external ports with order-fixed port constraints are present.
    pub fn extract_component_graphs(&mut self) -> Vec<LGraph> {
        // Build adjacency via outgoing + incoming edges so DFS walks the
        // undirected connectivity (predecessor + successor ports).
        let roots: Vec<NodeId> = std::mem::take(&mut self.layerless_nodes);
        let mut visited: HashMap<NodeId, bool> = HashMap::with_capacity(roots.len());
        for &n in &roots {
            visited.insert(n, false);
        }

        let mut components: Vec<Vec<NodeId>> = Vec::new();
        let mut stack: Vec<NodeId> = Vec::new();
        let mut neighbours: Vec<NodeId> = Vec::new();
        for &root in &roots {
            if let Some(&true) = visited.get(&root) {
                continue;
            }
            let mut component: Vec<NodeId> = Vec::new();
            stack.clear();
            stack.push(root);
            while let Some(node_id) = stack.pop() {
                match visited.get_mut(&node_id) {
                    Some(flag) if !*flag => *flag = true,
                    _ => continue,
                }
                component.push(node_id);
                let node = self.node(node_id);
                // Connected-ports walk: predecessors (incoming-edge sources)
                // first then successors (outgoing-edge targets) per port.
                // Then push in reverse so `stack.pop()` yields nodes in
                // recursive DFS pre-order.
                let ports: SmallVec<PortId, 6> = node.ports.iter().copied().collect();
                neighbours.clear();
                for port_id in ports {
                    let port = self.port(port_id);
                    for &edge_id in &port.incoming_edges {
                        let other = self.edge(edge_id).source;
                        let neighbour = self.port(other).owner;
                        if visited.contains_key(&neighbour) {
                            neighbours.push(neighbour);
                        }
                    }
                    for &edge_id in &port.outgoing_edges {
                        let other = self.edge(edge_id).target;
                        let neighbour = self.port(other).owner;
                        if visited.contains_key(&neighbour) {
                            neighbours.push(neighbour);
                        }
                    }
                }
                for &n in neighbours.iter().rev() {
                    stack.push(n);
                }
            }
            components.push(component);
        }

        // Fast-path: zero- or single-component graphs skip the split / combine
        // dance because migrating to fresh arenas would invalidate every
        // `NodeId` / `PortId` / `EdgeId` / `LabelId` the caller is holding.
        // The split always runs DFS even for the single-component case, so
        // the resulting `layerless_nodes` order is the DFS pre-order rather
        // than the importer's insertion order. Write the DFS ordering back,
        // even when we keep the original arenas. Without this, P1
        // GreedyCycleBreaker would read a different layerless order and
        // reverse a different edge set.
        if components.is_empty() {
            self.layerless_nodes = Vec::new();
            return Vec::new();
        }
        if components.len() == 1
            && !components[0]
                .iter()
                .any(|&node_id| self.node(node_id).node_type == NodeType::ExternalPort)
        {
            self.layerless_nodes = components.into_iter().next().unwrap();
            return Vec::new();
        }

        // Invert the root's identifier map so each component picks up only
        // the names that belong to it. The map is reconstructed on the root
        // in `absorb_graph` once the components are combined back.
        let identifier_entries: Vec<(String, NodeId)> =
            std::mem::take(&mut self.identifier_map).into_iter().collect();
        let mut identifier_reverse: HashMap<NodeId, SmallVec<String, 1>> = HashMap::new();
        for (name, node_id) in identifier_entries {
            identifier_reverse.entry(node_id).or_default().push(name);
        }

        let mut result: Vec<LGraph> = Vec::with_capacity(components.len());
        for component_nodes in components {
            let ext_port_connections = self.component_ext_port_connections(&component_nodes);
            let mut target = LGraph::new();
            target.options = self.options.clone();
            target.padding = self.padding;
            target.properties = self.properties.clone();
            // Store the set of external port sides reached by this component
            // for `ComponentGroupGraphPlacer`.
            target.properties.set(&EXT_PORT_CONNECTIONS, ext_port_connections);
            // Drop `NODE_SIZE_MINIMUM` from each component so per-component
            // bounds do not inherit the root graph's combined minimum.
            target
                .properties
                .set(&crate::properties::internal::NODE_SIZE_MINIMUM, Vec2::ZERO);

            let mut node_map: HashMap<NodeId, NodeId> =
                HashMap::with_capacity(component_nodes.len());
            let mut port_map: HashMap<PortId, PortId> = HashMap::new();
            let mut edge_map: HashMap<EdgeId, EdgeId> = HashMap::new();
            let mut label_map: HashMap<LabelId, LabelId> = HashMap::new();

            // Collect this component's ports/edges/labels while migrating
            // nodes into the target arena. We record arena keys first, then
            // migrate ports/edges/labels in a second pass so `self` stays
            // readable during node drain.
            let mut port_ids: Vec<PortId> = Vec::new();
            let mut edge_ids: Vec<EdgeId> = Vec::new();

            for &old_node_id in &component_nodes {
                let mut data = self
                    .nodes
                    .remove(old_node_id.arena_id())
                    .expect("component DFS referenced a missing node");
                // Clear cache indices that do not survive a fresh arena.
                data.layer = None.into();
                data.port_side_ends = [u16::MAX; 5];
                port_ids.extend(data.ports.iter().copied());
                let new_arena_id = target.nodes.insert(data);
                let new_node_id = NodeId(new_arena_id);
                node_map.insert(old_node_id, new_node_id);
                target.layerless_nodes.push(new_node_id);
            }

            for &old_port_id in &port_ids {
                let port_data = self
                    .ports
                    .remove(old_port_id.arena_id())
                    .expect("component port already migrated");
                edge_ids.extend(port_data.outgoing_edges.iter().copied());
                // Note: incoming_edges are represented by some other node's
                // outgoing_edges in the same component, so collecting only
                // outgoing avoids double migration without missing any edge.
                let label_ids: SmallVec<LabelId, 2> = port_data.labels.iter().copied().collect();
                let new_arena_id = target.ports.insert(port_data);
                let new_port_id = PortId(new_arena_id);
                port_map.insert(old_port_id, new_port_id);
                for old_label_id in label_ids {
                    migrate_label(self, &mut target, old_label_id, &mut label_map);
                }
            }

            for &old_edge_id in &edge_ids {
                let edge_data = self
                    .edges
                    .remove(old_edge_id.arena_id())
                    .expect("component edge already migrated");
                let label_ids: SmallVec<LabelId, 3> = edge_data.labels.iter().copied().collect();
                let new_arena_id = target.edges.insert(edge_data);
                let new_edge_id = EdgeId(new_arena_id);
                edge_map.insert(old_edge_id, new_edge_id);
                for old_label_id in label_ids {
                    migrate_label(self, &mut target, old_label_id, &mut label_map);
                }
            }

            // Node labels come last; ports/edges above may have registered them.
            let node_label_ids: Vec<(NodeId, SmallVec<LabelId, 2>)> = target
                .nodes
                .iter()
                .map(|(id, data)| (NodeId(id), data.labels.iter().copied().collect()))
                .collect();
            for (_new_node, labels) in &node_label_ids {
                for &old_label_id in labels {
                    if !label_map.contains_key(&old_label_id) {
                        migrate_label(self, &mut target, old_label_id, &mut label_map);
                    }
                }
            }

            remap_component_ids(&mut target, &node_map, &port_map, &edge_map, &label_map);
            remap_nested_origin_refs(&mut target, &node_map, &port_map);

            // Hand the subset of the root's identifier map that maps to this
            // component's nodes over to the component.
            for (&old_node_id, &new_node_id) in &node_map {
                if let Some(names) = identifier_reverse.get(&old_node_id) {
                    for name in names {
                        target.identifier_map.insert(name.clone(), new_node_id);
                    }
                }
            }

            result.push(target);
        }

        result
    }

    fn component_ext_port_connections(&self, component_nodes: &[NodeId]) -> PortSideSet {
        let mut sides = PortSideSet::SIDES_NONE;
        for &node_id in component_nodes {
            let node = self.node(node_id);
            if node.node_type != NodeType::ExternalPort {
                continue;
            }
            match node.properties.get(&EXT_PORT_SIDE) {
                PortSide::North => sides.insert(PortSideSet::NORTH),
                PortSide::East => sides.insert(PortSideSet::EAST),
                PortSide::South => sides.insert(PortSideSet::SOUTH),
                PortSide::West => sides.insert(PortSideSet::WEST),
                PortSide::Undefined => {}
            }
        }
        sides
    }

    /// Drain every arena of `source` into `self`, translating node positions,
    /// edge bend points, edge end points, and edge / port / node label
    /// positions by `graph_offset`.
    ///
    /// Iterates the source arenas directly instead of `source.layerless_nodes`.
    /// The pipeline drains `layerless_nodes` into `layers` during P2 and does
    /// not restore it at the end, so a layerless-only drain would miss every
    /// laid-out node. Dummy nodes added during the pipeline also live in
    /// the arena (often never referenced from `layerless_nodes`), and they
    /// must move with the real nodes so edge / port references stay valid.
    ///
    /// All migrated nodes land in `self.layerless_nodes` after translation.
    /// The original `source.layers` vector is discarded because `combine`
    /// reports the final layout via `self.layerless_nodes`: the post-combine
    /// state has every node sitting under the target graph's layerless
    /// collection.
    pub fn absorb_graph(&mut self, mut source: LGraph, graph_offset: Vec2) {
        let mut node_map: HashMap<NodeId, NodeId> = HashMap::new();
        let mut port_map: HashMap<PortId, PortId> = HashMap::new();
        let mut edge_map: HashMap<EdgeId, EdgeId> = HashMap::new();
        let mut label_map: HashMap<LabelId, LabelId> = HashMap::new();

        // Classify every label by its owner before arena migration starts,
        // so edge labels (graph-local frame) translate by `graph_offset`
        // while node / port / graph labels (owner-local frame) stay put.
        let mut edge_label_ids: std::collections::HashSet<LabelId> =
            std::collections::HashSet::new();
        for (_, edge) in source.edges.iter() {
            for &label_id in &edge.labels {
                edge_label_ids.insert(label_id);
            }
        }

        // Harvest every occupied arena id up front. The arena's `iter`
        // returns a borrow-tied iterator; collecting keeps the later
        // `remove` / `insert` loop free of that borrow.
        let source_node_ids: Vec<NodeId> = source.nodes.iter().map(|(id, _)| NodeId(id)).collect();
        let source_port_ids: Vec<PortId> = source.ports.iter().map(|(id, _)| PortId(id)).collect();
        let source_edge_ids: Vec<EdgeId> = source.edges.iter().map(|(id, _)| EdgeId(id)).collect();
        let source_label_ids: Vec<LabelId> =
            source.labels.iter().map(|(id, _)| LabelId(id)).collect();

        for old_node_id in source_node_ids {
            let mut data = source
                .nodes
                .remove(old_node_id.arena_id())
                .expect("absorb_graph: node already migrated");
            data.position.x += graph_offset.x;
            data.position.y += graph_offset.y;
            // The destination has no business inheriting the source's layer
            // assignment (the source's layers vector is dropped by this call).
            data.layer = None.into();
            data.port_side_ends = [u16::MAX; 5];
            let new_arena_id = self.nodes.insert(data);
            let new_id = NodeId(new_arena_id);
            node_map.insert(old_node_id, new_id);
            self.layerless_nodes.push(new_id);
        }

        for old_port_id in source_port_ids {
            let port_data = source
                .ports
                .remove(old_port_id.arena_id())
                .expect("absorb_graph: port already migrated");
            let new_arena_id = self.ports.insert(port_data);
            port_map.insert(old_port_id, PortId(new_arena_id));
        }

        for old_edge_id in source_edge_ids {
            let mut edge_data = source
                .edges
                .remove(old_edge_id.arena_id())
                .expect("absorb_graph: edge already migrated");
            for bp in edge_data.bend_points.iter_mut() {
                bp.x += graph_offset.x;
                bp.y += graph_offset.y;
            }
            if let Some(ref mut start) = edge_data.start_point {
                start.x += graph_offset.x;
                start.y += graph_offset.y;
            }
            if let Some(ref mut end) = edge_data.end_point {
                end.x += graph_offset.x;
                end.y += graph_offset.y;
            }
            let new_arena_id = self.edges.insert(edge_data);
            edge_map.insert(old_edge_id, EdgeId(new_arena_id));
        }

        for old_label_id in source_label_ids {
            let mut label_data = source
                .labels
                .remove(old_label_id.arena_id())
                .expect("absorb_graph: label already migrated");
            if edge_label_ids.contains(&old_label_id) {
                label_data.position.x += graph_offset.x;
                label_data.position.y += graph_offset.y;
            }
            let new_arena_id = self.labels.insert(label_data);
            label_map.insert(old_label_id, LabelId(new_arena_id));
        }

        remap_absorbed_ids(self, &node_map, &port_map, &edge_map, &label_map);

        // Merge the source's identifier_map into self, translating through
        // `node_map`. Duplicate identifiers would indicate a bug in the
        // split logic; we overwrite silently and let the last write win.
        let source_identifiers = std::mem::take(&mut source.identifier_map);
        for (name, old_node_id) in source_identifiers {
            if let Some(&new_node_id) = node_map.get(&old_node_id) {
                self.identifier_map.insert(name, new_node_id);
            }
        }
    }

    /// Unlink a node from its layer, layerless list, and parent/child links.
    ///
    /// Detaches `node_id` from `layer.nodes`, clears its `layer`, and removes
    /// its parent/child references. The arena entry is **not** freed, so any
    /// `NodeId` already captured by other parts of the layout pipeline
    /// (`SplineSegment.source_node`, the dependency graph in
    /// `node_promotion`, …) keeps resolving to the now-orphan
    /// `NodeData`. The orphan stays alive in the arena because downstream
    /// consumers (`SplineSegment` etc.) continue to read its last-set
    /// `position`, `size`, and `id`.
    /// those references when it needs them.
    ///
    /// The arena entries are reclaimed when the `LGraph` is dropped.
    pub fn remove_node(&mut self, node_id: NodeId) {
        // Remove from layer (LNode.setLayer(null) semantic).
        if let Some(layer_idx) = self.node(node_id).layer.get()
            && layer_idx < self.layers.len()
        {
            self.layers[layer_idx].nodes.retain(|&n| n != node_id);
        }
        // Clear `layer` so subsequent reads see "not in any layer".
        self.node_mut(node_id).layer = None.into();
        // Remove from layerless nodes.
        self.layerless_nodes.retain(|&n| n != node_id);
        // Clean up parent-child links.
        if let Some(parent_id) = self.node(node_id).parent {
            self.node_mut(parent_id).children.retain(|c| *c != node_id);
        }
        let children = std::mem::take(&mut self.node_mut(node_id).children);
        for child_id in children {
            self.node_mut(child_id).parent = None;
        }
        self.node_mut(node_id).parent = None;
        // Keep the LNode arena entry alive even after detachment. Subsequent
        // `try_node` / `nodes_iter` callers still see it as an orphan with
        // `layer == None`.
    }

    /// Insert a node into a specific position within a layer.
    ///
    /// Handles all three cases: layerless → layer, layer A → layer B, and
    /// re-insertion within the same layer.
    pub fn insert_node_in_layer(&mut self, node_id: NodeId, layer_idx: usize, position: usize) {
        // Remove from old layer if already assigned
        if let Some(old_layer) = self.nodes.get(node_id.arena_id()).and_then(|n| n.layer.get())
            && old_layer < self.layers.len()
        {
            self.layers[old_layer].nodes.retain(|&n| n != node_id);
        }
        // Remove from layerless
        self.layerless_nodes.retain(|&n| n != node_id);
        self.node_mut(node_id).layer = Some(layer_idx).into();
        let pos = position.min(self.layers[layer_idx].nodes.len());
        self.layers[layer_idx].nodes.insert(pos, node_id);
    }

    /// Disconnect an edge from its ports and remove it from the edge arena.
    ///
    /// Hierarchical edges are not affected.
    pub fn remove_edge(&mut self, edge_id: EdgeId) {
        let src = self.edge(edge_id).source;
        let tgt = self.edge(edge_id).target;
        self.port_mut(src).outgoing_edges.retain(|e| *e != edge_id);
        self.port_mut(tgt).incoming_edges.retain(|e| *e != edge_id);
        self.edges.remove(edge_id.arena_id());
    }

    /// Reroute an edge's target to a different port.
    ///
    /// Hierarchical edges are not affected.
    pub fn reroute_edge_target(&mut self, edge_id: EdgeId, new_target: PortId) {
        let old_target = self.edge(edge_id).target;
        let new_target_owner = self.port_owner(new_target);
        self.port_mut(old_target).incoming_edges.retain(|e| *e != edge_id);
        let edge = self.edge_mut(edge_id);
        edge.target = new_target;
        edge.target_owner = new_target_owner;
        self.port_mut(new_target).incoming_edges.push(edge_id);
    }

    /// Move every incoming edge from one target port to another.
    ///
    /// This is the batch form of repeated [`Self::reroute_edge_target`] calls for
    /// code paths that intentionally drain an entire port. It preserves the
    /// old port edge order and appends moved edges after any edges already on
    /// `new_target`, while avoiding a `retain` scan per edge.
    pub fn move_incoming_edges(&mut self, old_target: PortId, new_target: PortId) -> PortEdges {
        if old_target == new_target {
            return self.port(old_target).incoming_edges.clone();
        }

        let moved = std::mem::take(&mut self.port_mut(old_target).incoming_edges);
        let new_target_owner = self.port_owner(new_target);
        for &edge_id in &moved {
            let edge = self.edge_mut(edge_id);
            edge.target = new_target;
            edge.target_owner = new_target_owner;
        }
        self.port_mut(new_target).incoming_edges.extend(moved.iter().copied());
        moved
    }

    /// Reroute an edge's source to a different port.
    ///
    /// Hierarchical edges are not affected.
    pub fn reroute_edge_source(&mut self, edge_id: EdgeId, new_source: PortId) {
        let old_source = self.edge(edge_id).source;
        let new_source_owner = self.port_owner(new_source);
        self.port_mut(old_source).outgoing_edges.retain(|e| *e != edge_id);
        let edge = self.edge_mut(edge_id);
        edge.source = new_source;
        edge.source_owner = new_source_owner;
        self.port_mut(new_source).outgoing_edges.push(edge_id);
    }

    /// Move every outgoing edge from one source port to another.
    ///
    /// This is the batch form of repeated [`Self::reroute_edge_source`] calls for
    /// code paths that intentionally drain an entire port. It preserves the
    /// old port edge order and appends moved edges after any edges already on
    /// `new_source`, while avoiding a `retain` scan per edge.
    pub fn move_outgoing_edges(&mut self, old_source: PortId, new_source: PortId) -> PortEdges {
        if old_source == new_source {
            return self.port(old_source).outgoing_edges.clone();
        }

        let moved = std::mem::take(&mut self.port_mut(old_source).outgoing_edges);
        let new_source_owner = self.port_owner(new_source);
        for &edge_id in &moved {
            let edge = self.edge_mut(edge_id);
            edge.source = new_source;
            edge.source_owner = new_source_owner;
        }
        self.port_mut(new_source).outgoing_edges.extend(moved.iter().copied());
        moved
    }
}

impl Default for LGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for LGraph {
    fn drop(&mut self) {
        // Return the graph_id to the registry's free list. The arena's own
        // Drop impl runs after this and frees the per-LGraph contents.
        release_graph_id(self.graph_id);
    }
}

/// Move a single label from `source` to `target`, recording the id remap.
///
/// `label_map` is idempotent — re-calling for an already-migrated label is a
/// no-op so callers can opportunistically call from multiple owner paths
/// (port, node, edge) without double-migrating.
fn migrate_label(
    source: &mut LGraph,
    target: &mut LGraph,
    old_label_id: LabelId,
    label_map: &mut HashMap<LabelId, LabelId>,
) {
    migrate_label_with_offset(source, target, old_label_id, label_map, Vec2::ZERO);
}

/// Move a single label from `source` to `target`, applying `position_offset`.
///
/// `position_offset` lets edge labels (which share the edge bend-point frame)
/// translate with the component move, while port/node labels use their
/// enclosing owner's frame and stay at zero offset.
fn migrate_label_with_offset(
    source: &mut LGraph,
    target: &mut LGraph,
    old_label_id: LabelId,
    label_map: &mut HashMap<LabelId, LabelId>,
    position_offset: Vec2,
) {
    if label_map.contains_key(&old_label_id) {
        return;
    }
    if let Some(mut label_data) = source.labels.remove(old_label_id.arena_id()) {
        label_data.position.x += position_offset.x;
        label_data.position.y += position_offset.y;
        let new_arena_id = target.labels.insert(label_data);
        label_map.insert(old_label_id, LabelId(new_arena_id));
    }
}

/// Reuses `remap_component_ids` against an absorbed graph; both split and
/// absorb-back operations re-walk the same arena and rewrite internal ids.
fn remap_absorbed_ids(
    graph: &mut LGraph,
    node_map: &HashMap<NodeId, NodeId>,
    port_map: &HashMap<PortId, PortId>,
    edge_map: &HashMap<EdgeId, EdgeId>,
    label_map: &HashMap<LabelId, LabelId>,
) {
    remap_component_ids(graph, node_map, port_map, edge_map, label_map);
}

/// Rewrite every intra-component `NodeId` / `PortId` / `EdgeId` / `LabelId`
/// reference in `graph` through the supplied maps.
///
/// Called once per component after the arena migration has finished. The
/// target arenas hold freshly inserted data, so every stored id still points
/// at the source graph's arena coordinates; this pass swaps them for the
/// new ids.
fn remap_component_ids(
    graph: &mut LGraph,
    node_map: &HashMap<NodeId, NodeId>,
    port_map: &HashMap<PortId, PortId>,
    edge_map: &HashMap<EdgeId, EdgeId>,
    label_map: &HashMap<LabelId, LabelId>,
) {
    for (_, node) in graph.nodes.iter_mut() {
        for port in node.ports.iter_mut() {
            if let Some(new_port) = port_map.get(port) {
                *port = *new_port;
            }
        }
        for label in node.labels.iter_mut() {
            if let Some(new_label) = label_map.get(label) {
                *label = *new_label;
            }
        }
        if let Some(parent) = node.parent.as_mut()
            && let Some(new_parent) = node_map.get(parent)
        {
            *parent = *new_parent;
        }
        for child in node.children.iter_mut() {
            if let Some(new_child) = node_map.get(child) {
                *child = *new_child;
            }
        }
        if let Some(origin_edge) = node.origin_edge.as_mut()
            && let Some(new_edge) = edge_map.get(origin_edge)
        {
            *origin_edge = *new_edge;
        }
        if let Some(long_edge_source) = node.long_edge_source.as_mut()
            && let Some(new_port) = port_map.get(long_edge_source)
        {
            *long_edge_source = *new_port;
        }
        if let Some(long_edge_target) = node.long_edge_target.as_mut()
            && let Some(new_port) = port_map.get(long_edge_target)
        {
            *long_edge_target = *new_port;
        }
    }
    for (_, port) in graph.ports.iter_mut() {
        if let Some(new_owner) = node_map.get(&port.owner) {
            port.owner = *new_owner;
        }
        for edge in port.incoming_edges.iter_mut() {
            if let Some(new_edge) = edge_map.get(edge) {
                *edge = *new_edge;
            }
        }
        for edge in port.outgoing_edges.iter_mut() {
            if let Some(new_edge) = edge_map.get(edge) {
                *edge = *new_edge;
            }
        }
        for label in port.labels.iter_mut() {
            if let Some(new_label) = label_map.get(label) {
                *label = *new_label;
            }
        }
    }
    for (_, edge) in graph.edges.iter_mut() {
        if let Some(new_source) = port_map.get(&edge.source) {
            edge.source = *new_source;
        }
        if let Some(new_target) = port_map.get(&edge.target) {
            edge.target = *new_target;
        }
        if let Some(new_owner) = node_map.get(&edge.source_owner) {
            edge.source_owner = *new_owner;
        }
        if let Some(new_owner) = node_map.get(&edge.target_owner) {
            edge.target_owner = *new_owner;
        }
        for label in edge.labels.iter_mut() {
            if let Some(new_label) = label_map.get(label) {
                *label = *new_label;
            }
        }
    }
}

/// Recursively remap cross-graph PortId / NodeId references stored in
/// nested LGraph properties when a parent graph's arena is split into
/// per-component LGraphs.
///
/// Reaches inside every nested graph attached to the target's compound
/// nodes, performing the same id remap that `remap_component_ids` does for
/// the target arena. The cross-graph refs that matter here:
///
/// - EP dummy `ORIGIN_PORT` / `ORIGIN_NODE` (set by
///   `compound_graph::transform_external_port_in_nested`) — these
///   reference a port / node that lived in the parent arena and now
///   lives in the target arena under a fresh id.
///
/// Without this remap, EP dummies left over from a pre-split
/// `install_external_ports_for_separate_hierarchy` pass continue
/// pointing at PortIds that encode the old parent graph_id; later
/// `HierarchicalPortConstraintProcessor` N/S replacement + P3
/// hierarchical sweep would feed these stale ids into
/// `connected_edges` and trigger an "invalid PortId" panic when the
/// parent ctx graph cannot resolve them (issue
/// `bf_model_order_no_crash` on `aspect_cartrackingattackmodeling`).
///
/// Recursion is necessary because grandchild EP dummies' ORIGIN_PORT
/// references the immediate parent compound's port — the immediate
/// parent of a grandchild is itself a nested graph, but the chain of
/// remaps must follow the descent so the per-level port_map applies
/// at the right depth. We rely on the fact that `port_map` covers
/// every PortId that ever lived in the source arena; nested graphs
/// owning their own private ports stay untouched.
fn remap_nested_origin_refs(
    graph: &mut LGraph,
    node_map: &HashMap<NodeId, NodeId>,
    port_map: &HashMap<PortId, PortId>,
) {
    use crate::properties::internal::{ORIGIN_NODE, ORIGIN_PORT};
    let mut stack: Vec<*mut LGraph> =
        graph.nested_graphs_mut().map(|(_, nested)| nested as *mut LGraph).collect();
    stack.reverse();
    while let Some(nested_ptr) = stack.pop() {
        // SAFETY: pointers come from unique nested graph boxes and are only
        // borrowed one at a time.
        let nested = unsafe { &mut *nested_ptr };
        let inner_node_ids: Vec<NodeId> = nested.nodes_iter().map(|(id, _)| id).collect();
        for n_id in inner_node_ids {
            if let Some(origin_port) = nested.node(n_id).properties.get(&ORIGIN_PORT)
                && let Some(&new_port) = port_map.get(&origin_port)
            {
                nested.node_mut(n_id).properties.set(&ORIGIN_PORT, Some(new_port));
            }
            if let Some(origin_node) = nested.node(n_id).properties.get(&ORIGIN_NODE)
                && let Some(&new_node) = node_map.get(&origin_node)
            {
                nested.node_mut(n_id).properties.set(&ORIGIN_NODE, Some(new_node));
            }
        }
        let inner_port_ids: Vec<PortId> = nested.ports_iter().map(|(id, _)| id).collect();
        for p_id in inner_port_ids {
            if let Some(origin_port) = nested.port(p_id).properties.get(&ORIGIN_PORT)
                && let Some(&new_port) = port_map.get(&origin_port)
            {
                nested.port_mut(p_id).properties.set(&ORIGIN_PORT, Some(new_port));
            }
        }
        let mut children: Vec<*mut LGraph> =
            nested.nested_graphs_mut().map(|(_, child)| child as *mut LGraph).collect();
        while let Some(child) = children.pop() {
            stack.push(child);
        }
    }
}

/// Walk the nested-graph hierarchy from `graph` to find the LGraph that owns `node`.
///
/// Returns a raw `*mut LGraph` so the caller can re-borrow the result without
/// holding an outstanding `&mut` on intermediate parent graphs.
fn find_containing_graph_ptr(graph: &mut LGraph, node: NodeId) -> Option<*mut LGraph> {
    let mut stack = vec![graph as *mut LGraph];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: pointers come from the nested ownership tree and are only
        // borrowed one at a time.
        let graph = unsafe { &mut *graph_ptr };
        if graph.nodes.get(node.arena_id()).is_some() {
            return Some(graph_ptr);
        }
        let child_ids: Vec<NodeId> = graph.nodes_iter().map(|(nid, _)| nid).collect();
        for child in child_ids.into_iter().rev() {
            if graph.has_nested(child) {
                stack.push(graph.nested_mut(child).unwrap() as *mut LGraph);
            }
        }
    }
    None
}
