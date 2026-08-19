//! Layer sweep crossing minimizer (P3 main orchestrator).

use std::{
    collections::{HashMap, VecDeque},
    marker::PhantomData,
    ptr::NonNull,
    sync::OnceLock,
};

use crate::{
    graph::{
        LGraph,
        index::{NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    options::enums::{
        CrossingMinimizationStrategy, HierarchyHandling, OrderingStrategy, PortConstraints,
    },
    p3_crossing_min::{
        barycenter_state::BarycenterStateMap,
        counting,
        crossing_count_side::CrossingCountSide,
        graph_info::GraphInfoHolder,
        layer_sweep::{
            self, PortRankMode, PortRanks, PortType, calculate_port_ranks,
            calculate_port_ranks_into, is_first_layer,
        },
        switch_decider::{ParentContext, SwitchDecider},
    },
    properties::internal::{
        FIRST_TRY_WITH_INITIAL_ORDER, ORIGIN_PORT, P3_IGNORE_NESTED_GRAPHS, P3_INITIAL_LAYER_ORDER,
        SECOND_TRY_WITH_INITIAL_ORDER,
    },
    rng::{Rng, SeededRng},
};

/// Read both `FIRST_TRY_WITH_INITIAL_ORDER` and `SECOND_TRY_WITH_INITIAL_ORDER`
/// graph properties.
fn read_try_flags(graph: &LGraph) -> (bool, bool) {
    (
        graph.properties.get(&FIRST_TRY_WITH_INITIAL_ORDER),
        graph.properties.get(&SECOND_TRY_WITH_INITIAL_ORDER),
    )
}

/// Heuristic type for the layer sweep crossing minimizer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossMinType {
    Barycenter,
    Median,
    OneSidedGreedySwitch,
    TwoSidedGreedySwitch,
}

#[derive(Clone, Default)]
struct GraphSnapshot {
    layers: Vec<Vec<NodeId>>,
    ports: Vec<(NodeId, Vec<PortId>)>,
    nested: Vec<(NodeId, GraphSnapshot)>,
}

impl GraphSnapshot {
    /// Capture `graph`'s layer / port / nested state into `self`, reusing the
    /// existing inner `Vec` allocations via `clone_from`. Removes the heap
    /// churn that the thoroughness-7 outer loop would otherwise pay on every
    /// score-improving sweep iteration (3-5 captures per iteration × 7 iterations).
    fn capture_from(&mut self, graph: &LGraph) {
        if self.layers.len() < graph.layers.len() {
            self.layers.resize(graph.layers.len(), Vec::new());
        } else {
            self.layers.truncate(graph.layers.len());
        }
        for (slot, layer) in self.layers.iter_mut().zip(graph.layers.iter()) {
            slot.clone_from(&layer.nodes);
        }

        let node_count = graph.nodes_iter().count();
        if self.ports.len() < node_count {
            let placeholder = NodeId(crate::graph::arena::ArenaId::sentinel());
            self.ports.resize_with(node_count, || (placeholder, Vec::new()));
        } else {
            self.ports.truncate(node_count);
        }
        for ((slot_id, slot_ports), (node_id, node)) in
            self.ports.iter_mut().zip(graph.nodes_iter())
        {
            *slot_id = node_id;
            slot_ports.clear();
            slot_ports.extend(node.ports.iter().copied());
        }

        self.nested.clear();
        for (node_id, node) in graph.nodes_iter() {
            if let Some(child) = node.nested_graph_ref() {
                let mut child_snap = GraphSnapshot::default();
                child_snap.capture_from(child);
                self.nested.push((node_id, child_snap));
            }
        }
    }
}

struct ScoreCache {
    scratch: counting::CountingScratch,
    crossing_cache: LayerCrossingCache,
    version: u64,
    fingerprint: u64,
    score: f64,
    valid: bool,
    has_active_nested_score: Option<bool>,
}

impl ScoreCache {
    fn new() -> Self {
        Self {
            scratch: counting::CountingScratch::new(),
            crossing_cache: LayerCrossingCache::new(),
            version: 0,
            fingerprint: 0,
            score: 0.0,
            valid: false,
            has_active_nested_score: None,
        }
    }

    fn effective_score(&mut self, graph: &LGraph) -> f64 {
        let version = graph.order_version();
        if self.valid && self.version == version {
            if score_version_assertion_enabled() {
                let fingerprint = score_fingerprint(graph);
                assert_eq!(
                    self.fingerprint, fingerprint,
                    "P3 score cache version hit but graph order fingerprint changed"
                );
            }
            return self.score;
        }

        let score = if self.can_use_incremental_crossing_score(graph) {
            let cached = self.crossing_cache.count_all_crossings(graph) as f64;
            if score_version_assertion_enabled() {
                let fresh = counting::count_all_crossings_with_scratch(graph, &mut self.scratch);
                assert_eq!(
                    cached, fresh as f64,
                    "P3 incremental crossing score diverged from fresh full count"
                );
            }
            cached
        } else {
            effective_score(graph, &mut self.scratch)
        };
        self.version = version;
        if score_version_assertion_enabled() {
            self.fingerprint = score_fingerprint(graph);
        }
        self.score = score;
        self.valid = true;
        score
    }

    fn can_use_incremental_crossing_score(&mut self, graph: &LGraph) -> bool {
        let node_inf = graph.options.consider_model_order_crossing_counter_node_influence;
        let port_inf = graph.options.consider_model_order_crossing_counter_port_influence;
        if node_inf != 0.0 || port_inf != 0.0 {
            return false;
        }

        let has_nested = *self
            .has_active_nested_score
            .get_or_insert_with(|| graph_has_active_nested_score(graph));
        !has_nested
    }
}

#[derive(Clone, Copy, Default)]
struct CachedBoundaryCrossings {
    layer_idx: usize,
    layer_version: u64,
    value: usize,
    valid: bool,
}

#[derive(Clone, Copy, Default)]
struct CachedLayerCrossingsAt {
    layer_version: u64,
    next_layer_version: u64,
    value: usize,
    valid: bool,
}

#[derive(Clone, Copy, Default)]
struct CachedHyperedgeBoundary {
    // P3 sweeps mutate node/port order but not graph topology, port sides, or
    // edge incidence. Cache this topology-only predicate across score checks.
    value: bool,
    valid: bool,
}

struct LayerCrossingCache {
    scratch: counting::CountingScratch,
    layer_count: usize,
    west_boundary: CachedBoundaryCrossings,
    east_boundary: CachedBoundaryCrossings,
    at_layers: Vec<CachedLayerCrossingsAt>,
    hyperedge_boundaries: Vec<CachedHyperedgeBoundary>,
}

impl LayerCrossingCache {
    fn new() -> Self {
        Self {
            scratch: counting::CountingScratch::new(),
            layer_count: 0,
            west_boundary: CachedBoundaryCrossings::default(),
            east_boundary: CachedBoundaryCrossings::default(),
            at_layers: Vec::new(),
            hyperedge_boundaries: Vec::new(),
        }
    }

    fn count_all_crossings(&mut self, graph: &LGraph) -> usize {
        let layer_count = graph.layers.len();
        if layer_count == 0 {
            self.invalidate_for_layer_count(0);
            return 0;
        }
        if self.layer_count != layer_count {
            self.invalidate_for_layer_count(layer_count);
        }

        let LayerCrossingCache {
            scratch,
            west_boundary,
            east_boundary,
            at_layers,
            hyperedge_boundaries,
            ..
        } = self;

        let last_layer_idx = layer_count - 1;
        let mut total = boundary_crossings(west_boundary, graph, 0, PortSide::West, scratch);
        total += boundary_crossings(east_boundary, graph, last_layer_idx, PortSide::East, scratch);

        for (layer_idx, entry) in at_layers.iter_mut().enumerate() {
            total += layer_crossings_at(entry, graph, layer_idx, scratch, hyperedge_boundaries);
        }
        total
    }

    fn invalidate_for_layer_count(&mut self, layer_count: usize) {
        self.layer_count = layer_count;
        self.west_boundary.valid = false;
        self.east_boundary.valid = false;
        self.at_layers.clear();
        self.at_layers.resize(layer_count, CachedLayerCrossingsAt::default());
        self.hyperedge_boundaries.clear();
        self.hyperedge_boundaries
            .resize(layer_count, CachedHyperedgeBoundary::default());
    }
}

fn boundary_crossings(
    entry: &mut CachedBoundaryCrossings,
    graph: &LGraph,
    layer_idx: usize,
    side: PortSide,
    scratch: &mut counting::CountingScratch,
) -> usize {
    let layer_version = graph.layer_order_version(layer_idx);
    if entry.valid && entry.layer_idx == layer_idx && entry.layer_version == layer_version {
        return entry.value;
    }

    let value =
        counting::count_in_layer_crossings_on_side_with_scratch(graph, layer_idx, side, scratch);
    *entry = CachedBoundaryCrossings { layer_idx, layer_version, value, valid: true };
    value
}

fn layer_crossings_at(
    entry: &mut CachedLayerCrossingsAt,
    graph: &LGraph,
    layer_idx: usize,
    scratch: &mut counting::CountingScratch,
    hyperedge_boundaries: &mut [CachedHyperedgeBoundary],
) -> usize {
    let layer_version = graph.layer_order_version(layer_idx);
    let next_layer_version = if layer_idx + 1 < graph.layers.len() {
        graph.layer_order_version(layer_idx + 1)
    } else {
        0
    };
    if entry.valid
        && entry.layer_version == layer_version
        && entry.next_layer_version == next_layer_version
    {
        return entry.value;
    }

    let has_hyperedges = cached_hyperedge_boundary(hyperedge_boundaries, graph, layer_idx);
    let value =
        counting::count_crossings_at_with_hyperedge_hint(graph, layer_idx, scratch, has_hyperedges);
    *entry = CachedLayerCrossingsAt { layer_version, next_layer_version, value, valid: true };
    value
}

fn cached_hyperedge_boundary(
    entries: &mut [CachedHyperedgeBoundary],
    graph: &LGraph,
    layer_idx: usize,
) -> bool {
    if layer_idx + 1 >= graph.layers.len() {
        return false;
    }
    let Some(entry) = entries.get_mut(layer_idx) else {
        return counting::has_hyperedges_between_layers(graph, layer_idx, layer_idx + 1);
    };
    if entry.valid {
        return entry.value;
    }

    let value = counting::has_hyperedges_between_layers(graph, layer_idx, layer_idx + 1);
    *entry = CachedHyperedgeBoundary { value, valid: true };
    value
}

fn graph_has_active_nested_score(graph: &LGraph) -> bool {
    let mut stack = vec![graph];
    while let Some(graph) = stack.pop() {
        if !p3_uses_nested_graphs(graph) {
            continue;
        }
        for (_, node) in graph.nodes_iter() {
            let Some(child) = node.nested_graph_ref() else {
                continue;
            };
            let child_info = GraphInfoHolder::from_graph(child);
            if !child_info.dont_sweep_into() {
                return true;
            }
            stack.push(child);
        }
    }
    false
}

fn score_version_assertion_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        cfg!(debug_assertions) || std::env::var_os("LAYERD_ASSERT_SCORE_VERSION").is_some()
    })
}

struct SweepScratch {
    bary_state: BarycenterStateMap,
    fixed_ranks: PortRanks,
    ordered_nodes: Vec<NodeId>,
    port_distribution: layer_sweep::PortDistributionScratch,
}

impl SweepScratch {
    fn new() -> Self {
        Self {
            bary_state: BarycenterStateMap::new(),
            fixed_ranks: PortRanks::new(),
            ordered_nodes: Vec::new(),
            port_distribution: layer_sweep::PortDistributionScratch::new(),
        }
    }
}

fn score_fingerprint(graph: &LGraph) -> u64 {
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    fingerprint_graph_into(graph, &mut state);
    state
}

fn fingerprint_graph_into(graph: &LGraph, state: &mut u64) {
    let mut stack = vec![graph];
    while let Some(graph) = stack.pop() {
        mix_fingerprint(state, graph.graph_id() as u64);
        mix_fingerprint(state, graph.layers.len() as u64);
        for layer in &graph.layers {
            mix_fingerprint(state, layer.nodes.len() as u64);
            for &node_id in &layer.nodes {
                mix_fingerprint(state, node_id.0.index() as u64);
                let node = graph.node(node_id);
                mix_fingerprint(state, node.ports.len() as u64);
                for &port_id in &node.ports {
                    mix_fingerprint(state, port_id.0.index() as u64);
                }
            }
        }

        let mut children = Vec::new();
        for (node_id, node) in graph.nodes_iter() {
            if let Some(child) = node.nested_graph_ref() {
                mix_fingerprint(state, node_id.0.index() as u64);
                children.push(child);
            }
        }
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }
}

#[inline]
fn mix_fingerprint(state: &mut u64, value: u64) {
    *state ^= value
        .wrapping_add(0x9e37_79b9_7f4a_7c15)
        .wrapping_add(*state << 6)
        .wrapping_add(*state >> 2);
}

/// Derive the effective `CrossMinType` from layout options.
fn cross_min_type_from_options(opts: &crate::options::LayoutOptions) -> CrossMinType {
    match opts.crossing_minimization {
        CrossingMinimizationStrategy::MedianLayerSweep => CrossMinType::Median,
        _ => CrossMinType::Barycenter,
    }
}

/// Returns whether the heuristic is deterministic (never consults the RNG once
/// the node order is fixed).
///
/// `Median` is treated as deterministic: it consults the RNG once to seed
/// first-layer weights, but from there on only propagates medians.
fn is_deterministic(cross_min_type: CrossMinType) -> bool {
    matches!(
        cross_min_type,
        CrossMinType::Median
            | CrossMinType::OneSidedGreedySwitch
            | CrossMinType::TwoSidedGreedySwitch
    )
}

/// Returns whether the heuristic is guaranteed to reduce or preserve crossings
/// on every sweep.
fn always_improves(cross_min_type: CrossMinType) -> bool {
    matches!(cross_min_type, CrossMinType::TwoSidedGreedySwitch)
}

/// Minimize crossings using the layer sweep algorithm.
///
/// Production entry point. Consumes the shared RNG on `graph` so that its
/// state continues from whatever the earlier phases left behind, satisfying
/// the one-Random-per-layout contract.
pub fn minimize_crossings(graph: &mut LGraph) {
    let cmt = cross_min_type_from_options(&graph.options);
    let mut rng = graph.take_rng();
    minimize_crossings_with_rng_internal(graph, cmt, &mut rng, true);
    graph.put_rng(rng);
}

pub(crate) fn minimize_crossings_with_graph_rngs(
    graph: &mut LGraph,
    cross_min_type: CrossMinType,
    rng: &mut impl Rng,
) {
    minimize_crossings_with_rng_internal(graph, cross_min_type, rng, true);
}

fn minimize_crossings_with_rng_internal(
    graph: &mut LGraph,
    cross_min_type: CrossMinType,
    rng: &mut impl Rng,
    use_graph_rng_for_nested_heuristics: bool,
) {
    let hierarchical_layout = graph.options.hierarchy_handling == HierarchyHandling::Include;
    let empty_graph =
        graph.layers.is_empty() || graph.layers.iter().all(|layer| layer.nodes.is_empty());
    let single_node = graph.layers.len() == 1 && graph.layers[0].nodes.len() == 1;
    if empty_graph || (single_node && !hierarchical_layout) {
        return;
    }

    let use_median = matches!(cross_min_type, CrossMinType::Median);

    // Capture a seed snapshot once per layout, so every randomized-layouts
    // invocation can replay the same sequence. Without this, a subgraph that
    // appears under multiple parents gets different orderings across copies.
    let random_seed = rng.next_long();

    // One port-rank mode per LGraph, in inclusion-BFS order. Each holder
    // creates its own port distributor, consuming one `nextBoolean()` for
    // barycenter/median graphs. Keep per-graph modes instead of sharing the
    // root graph's choice with every nested graph.
    let port_rank_modes = initialize_port_rank_modes(graph, cross_min_type, rng);

    let _graph_info = GraphInfoHolder::from_graph(graph);
    let graphs_to_sweep_on = collect_graphs_to_sweep_on_paths(graph);
    for path in graphs_to_sweep_on {
        minimize_graph_at_path(
            graph,
            &path,
            rng,
            cross_min_type,
            use_median,
            &port_rank_modes,
            random_seed,
            use_graph_rng_for_nested_heuristics,
        );
    }

    store_graph_info_for_hierarchy(graph);
}

type PortRankModes = HashMap<u16, PortRankMode>;

fn collect_graphs_to_sweep_on_paths(graph: &LGraph) -> Vec<Vec<NodeId>> {
    let mut graphs_to_sweep_on = Vec::new();
    let mut queue = VecDeque::from([Vec::new()]);

    while let Some(path) = queue.pop_front() {
        let Some(current) = graph_at_path(graph, &path) else {
            continue;
        };

        if GraphInfoHolder::from_graph(current).dont_sweep_into() {
            graphs_to_sweep_on.insert(0, path.clone());
        }

        if p3_uses_nested_graphs(current) {
            let child_ids: Vec<NodeId> = current
                .layers
                .iter()
                .flat_map(|layer| layer.nodes.iter())
                .copied()
                .filter(|&node_id| current.has_nested(node_id))
                .collect();
            for node_id in child_ids {
                let mut child_path = path.clone();
                child_path.push(node_id);
                queue.push_back(child_path);
            }
        }
    }

    graphs_to_sweep_on
}

fn graph_at_path<'a>(mut graph: &'a LGraph, path: &[NodeId]) -> Option<&'a LGraph> {
    for &node_id in path {
        graph = graph.nested(node_id)?;
    }
    Some(graph)
}

fn restore_taken_nested_path(
    graph: &mut LGraph,
    mut parents: Vec<(NodeId, Box<LGraph>, u64)>,
    current_node_id: NodeId,
    current_box: Box<LGraph>,
) {
    let mut child_node_id = current_node_id;
    let mut child_box = current_box;
    while let Some((parent_node_id, mut parent_box, _version_before)) = parents.pop() {
        parent_box.set_nested_boxed(child_node_id, child_box);
        child_node_id = parent_node_id;
        child_box = parent_box;
    }
    graph.set_nested_boxed(child_node_id, child_box);
}

fn minimize_graph_at_path(
    graph: &mut LGraph,
    path: &[NodeId],
    rng: &mut impl Rng,
    cross_min_type: CrossMinType,
    use_median: bool,
    port_rank_modes: &PortRankModes,
    random_seed: i64,
    use_graph_rng_for_nested_heuristics: bool,
) {
    let Some((&node_id, remaining)) = path.split_first() else {
        minimize_graph_only(
            graph,
            rng,
            cross_min_type,
            use_median,
            port_rank_modes,
            true,
            None,
            random_seed,
            use_graph_rng_for_nested_heuristics,
        );
        return;
    };

    let Some(mut current_box) = graph.take_nested_boxed(node_id) else {
        return;
    };
    let mut current_node_id = node_id;
    let mut current_version_before = current_box.order_version();
    let mut parents: Vec<(NodeId, Box<LGraph>, u64)> = Vec::new();

    for &next_node_id in remaining {
        let Some(next_box) = current_box.take_nested_boxed(next_node_id) else {
            restore_taken_nested_path(graph, parents, current_node_id, current_box);
            return;
        };
        parents.push((current_node_id, current_box, current_version_before));
        current_node_id = next_node_id;
        current_box = next_box;
        current_version_before = current_box.order_version();
    }

    {
        let parent_graph =
            if let Some((_parent_node_id, parent_box, _version_before)) = parents.last_mut() {
                &mut **parent_box
            } else {
                &mut *graph
            };
        let parent_layer_idx = parent_graph.node(current_node_id).layer.unwrap_or(0);
        let parent_ctx = ParentContext {
            graph: parent_graph,
            parent_node_id: current_node_id,
            parent_layer_idx,
        };
        minimize_graph_only(
            &mut current_box,
            rng,
            cross_min_type,
            use_median,
            port_rank_modes,
            false,
            Some(parent_ctx),
            random_seed,
            use_graph_rng_for_nested_heuristics,
        );
    }

    let mut child_node_id = current_node_id;
    let mut child_box = current_box;
    let mut child_version_before = current_version_before;
    let mut transfer_leaf_order = true;
    while let Some((parent_node_id, mut parent_box, parent_version_before)) = parents.pop() {
        let child_changed = child_box.order_version() != child_version_before;
        parent_box.set_nested_boxed(child_node_id, child_box);
        if child_changed {
            parent_box.bump_order_version();
        }
        if transfer_leaf_order {
            transfer_child_dummy_order_to_parent_ports(&mut parent_box, child_node_id);
            transfer_leaf_order = false;
        }
        child_node_id = parent_node_id;
        child_box = parent_box;
        child_version_before = parent_version_before;
    }

    let child_changed = child_box.order_version() != child_version_before;
    graph.set_nested_boxed(child_node_id, child_box);
    if child_changed {
        graph.bump_order_version();
    }
    if transfer_leaf_order {
        transfer_child_dummy_order_to_parent_ports(graph, child_node_id);
    }
}

fn minimize_graph_only(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    cross_min_type: CrossMinType,
    use_median: bool,
    port_rank_modes: &PortRankModes,
    is_root: bool,
    mut parent_context: Option<ParentContext<'_>>,
    random_seed: i64,
    use_graph_rng_for_nested_heuristics: bool,
) {
    if graph.layers.is_empty() {
        return;
    }

    initialize_p3_initial_layer_order(graph);

    let port_rank_mode = port_rank_mode_for_graph(graph, port_rank_modes);
    let use_current_graph_rng = use_graph_rng_for_nested_heuristics && !is_root;

    // Dispatch on the heuristic's deterministic / always-improves flags:
    // Barycenter and Median are randomized and benefit from a thoroughness
    // outer loop; one-sided greedy switch is deterministic but not monotone
    // (single counter pass); two-sided greedy switch is monotone (no-counter
    // sweep).
    match (is_deterministic(cross_min_type), always_improves(cross_min_type)) {
        (false, _) => compare_different_randomized_layouts(
            graph,
            rng,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            random_seed,
            use_current_graph_rng,
            use_graph_rng_for_nested_heuristics,
        ),
        (true, true) => minimize_no_counter(
            graph,
            rng,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            random_seed,
            use_current_graph_rng,
            use_graph_rng_for_nested_heuristics,
        ),
        (true, false) => minimize_with_counter_single_pass(
            graph,
            rng,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            random_seed,
            use_current_graph_rng,
            use_graph_rng_for_nested_heuristics,
        ),
    }
}

fn initialize_port_rank_modes(
    graph: &mut LGraph,
    cross_min_type: CrossMinType,
    rng: &mut impl Rng,
) -> PortRankModes {
    let mut modes = HashMap::new();
    let mut graphs = vec![std::ptr::NonNull::from(&mut *graph)];
    let mut idx = 0;
    while idx < graphs.len() {
        let current_ptr = graphs[idx];
        // SAFETY: every pointer is the unique root graph or one nested graph
        // owned by a distinct Box. We do not keep child borrows live across
        // loop iterations.
        let current = unsafe { &mut *current_ptr.as_ptr() };
        let is_root_graph = idx == 0;
        idx += 1;

        let mode = if is_root_graph {
            create_port_rank_mode(cross_min_type, rng)
        } else {
            // Each nested LGraph constructs its port distributor with its
            // own RNG, not the root crossing minimizer's RNG. This
            // consumption is observable later when child
            // barycenter heuristics randomize otherwise unconstrained layers.
            let mut graph_rng = current.take_rng();
            let mode = create_port_rank_mode(cross_min_type, &mut graph_rng);
            current.put_rng(graph_rng);
            mode
        };
        modes.insert(current.graph_id(), mode);

        if p3_uses_nested_graphs(current) {
            let child_ids: Vec<NodeId> = current
                .layers
                .iter()
                .flat_map(|layer| layer.nodes.iter())
                .copied()
                .filter(|&node_id| current.has_nested(node_id))
                .collect();
            for node_id in child_ids {
                if let Some(child) = current.nested_mut(node_id) {
                    graphs.push(std::ptr::NonNull::from(child));
                }
            }
        }
    }
    modes
}

fn create_port_rank_mode(cross_min_type: CrossMinType, rng: &mut impl Rng) -> PortRankMode {
    if matches!(cross_min_type, CrossMinType::TwoSidedGreedySwitch) || rng.next_bool() {
        PortRankMode::NodeRelative
    } else {
        PortRankMode::LayerTotal
    }
}

fn port_rank_mode_for_graph(graph: &LGraph, port_rank_modes: &PortRankModes) -> PortRankMode {
    port_rank_modes
        .get(&graph.graph_id())
        .copied()
        .unwrap_or(PortRankMode::NodeRelative)
}

fn p3_uses_nested_graphs(graph: &LGraph) -> bool {
    !graph.properties.get(&P3_IGNORE_NESTED_GRAPHS)
}

fn store_graph_info_for_hierarchy(graph: &mut LGraph) {
    use crate::p3_crossing_min::graph_info::P3_GRAPH_INFO;
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers are unique nested graph boxes.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let info = GraphInfoHolder::from_graph(graph);
        graph.properties.set(&P3_GRAPH_INFO, info);
        let node_ids: Vec<NodeId> = graph
            .layerless_nodes
            .iter()
            .chain(graph.layers.iter().flat_map(|l| l.nodes.iter()))
            .copied()
            .collect();
        for nid in node_ids.into_iter().rev() {
            if let Some(nested) = graph.nested_mut(nid) {
                stack.push(std::ptr::NonNull::from(nested));
            }
        }
    }
}

fn initialize_p3_initial_layer_order(graph: &mut LGraph) {
    let assignments: Vec<(NodeId, i32)> = graph
        .layers
        .iter()
        .flat_map(|layer| {
            layer.nodes.iter().enumerate().map(|(index, &node_id)| {
                let capped = index.min(i32::MAX as usize) as i32;
                (node_id, capped)
            })
        })
        .collect();

    for (node_id, index) in assignments {
        graph.node_mut(node_id).properties.set(&P3_INITIAL_LAYER_ORDER, index);
    }
}

/// Sweep forward and backward while the node order keeps changing.
///
/// Appropriate when the chosen heuristic is known to always improve or
/// preserve crossing count, so the sweep is iterated without counting
/// crossings. Convergence is detected by comparing the flat layer node order
/// before and after each pair of sweeps.
fn minimize_no_counter(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    use_median: bool,
    port_rank_mode: PortRankMode,
    port_rank_modes: &PortRankModes,
    cross_min_type: CrossMinType,
    mut parent_context: Option<ParentContext<'_>>,
    random_seed: i64,
    use_graph_rng_for_heuristic: bool,
    use_graph_rng_for_nested_heuristics: bool,
) {
    // Drive the loop on the OR-combined `improved` bool returned by
    // `set_first_layer_order` and `sweep_reducing_crossings`. Stops as soon
    // as a full pass produces no change. Used for always-improves heuristics
    // (two-sided greedy switch).
    let mut is_forward = rng.next_bool();
    let mut improved = true;
    let mut sweep_scratch = SweepScratch::new();
    let mut graph_rng = use_graph_rng_for_heuristic.then(|| graph.take_rng());
    while improved {
        improved = false;
        if let Some(heuristic_rng) = graph_rng.as_mut() {
            improved |= set_first_layer_order_with_state_scratch(
                graph,
                heuristic_rng,
                is_forward,
                use_median,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                &mut sweep_scratch.bary_state,
                &mut sweep_scratch.ordered_nodes,
            );
            improved |= sweep_reducing_crossings(
                graph,
                heuristic_rng,
                is_forward,
                false,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        } else {
            improved |= set_first_layer_order_with_state_scratch(
                graph,
                rng,
                is_forward,
                use_median,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                &mut sweep_scratch.bary_state,
                &mut sweep_scratch.ordered_nodes,
            );
            improved |= sweep_reducing_crossings(
                graph,
                rng,
                is_forward,
                false,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        }
        is_forward = !is_forward;
    }
    if let Some(graph_rng) = graph_rng {
        graph.put_rng(graph_rng);
    }
}

/// Restart the counter-based pass up to `thoroughness` times and keep the
/// snapshot with the fewest crossings.
///
/// Randomized outer loop used by Barycenter and Median. Each iteration either
/// keeps the initial node order (first and second try when
/// `CONSIDER_MODEL_ORDER_STRATEGY` is active) or randomizes the first layer,
/// then performs one full back-and-forth sweep while counting crossings. The
/// graph is restored to the best snapshot at the end.
fn compare_different_randomized_layouts(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    use_median: bool,
    port_rank_mode: PortRankMode,
    port_rank_modes: &PortRankModes,
    cross_min_type: CrossMinType,
    mut parent_context: Option<ParentContext<'_>>,
    random_seed: i64,
    use_graph_rng_for_heuristic: bool,
    use_graph_rng_for_nested_heuristics: bool,
) {
    // Reset the RNG seed so two copies of the same hierarchical subgraph
    // under different parents end up with the same randomized layouts.
    rng.set_seed(random_seed);
    let max_iter = graph.options.thoroughness.max(1) as usize;
    let mut score_cache = ScoreCache::new();
    let mut sweep_scratch = SweepScratch::new();
    let mut best_snapshot = GraphSnapshot::default();
    let mut graph_rng = use_graph_rng_for_heuristic.then(|| graph.take_rng());
    // `best_score` starts at +infinity so every first iteration's `crossings`
    // value is strictly less and therefore promotes its captured snapshot to
    // the global best. Pre-capturing the initial graph state would seed
    // `best_score = effective_score(initial)`, and a sweep that produces an
    // equal-score state (e.g. when the initial layout already has 0 crossings
    // yet better port-side ordering than initial) would fail the `<` test and
    // get rolled back by the trailing `restore_graph_snapshot`.
    let mut best_score = f64::INFINITY;
    let mut best_captured = false;

    // These flags live on the graph so nested sub-sweeps
    // (`sweep_reducing_crossings` / `minimize_layer`) can read them and
    // suppress the usual "randomize on first sweep" behaviour.
    graph.properties.set(
        &FIRST_TRY_WITH_INITIAL_ORDER,
        graph.options.ordering_strategy != OrderingStrategy::None,
    );

    for _ in 0..max_iter {
        let (first_try, second_try) = read_try_flags(graph);
        let initial_score = score_cache.effective_score(graph);
        let mut forward = rng.next_bool();
        if initial_score == 0.0 && first_try {
            best_snapshot.capture_from(graph);
            best_captured = true;
            break;
        }

        if !first_try && !second_try {
            if let Some(heuristic_rng) = graph_rng.as_mut() {
                set_first_layer_order_with_state_scratch(
                    graph,
                    heuristic_rng,
                    forward,
                    use_median,
                    cross_min_type,
                    parent_context.as_mut().map(|p| p.reborrow()),
                    &mut sweep_scratch.bary_state,
                    &mut sweep_scratch.ordered_nodes,
                );
            } else {
                set_first_layer_order_with_state_scratch(
                    graph,
                    rng,
                    forward,
                    use_median,
                    cross_min_type,
                    parent_context.as_mut().map(|p| p.reborrow()),
                    &mut sweep_scratch.bary_state,
                    &mut sweep_scratch.ordered_nodes,
                );
            }
        } else {
            forward = first_try;
        }

        if let Some(heuristic_rng) = graph_rng.as_mut() {
            sweep_reducing_crossings(
                graph,
                heuristic_rng,
                forward,
                true,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        } else {
            sweep_reducing_crossings(
                graph,
                rng,
                forward,
                true,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        }

        if graph.properties.get(&SECOND_TRY_WITH_INITIAL_ORDER) {
            graph.properties.set(&SECOND_TRY_WITH_INITIAL_ORDER, false);
        }
        if graph.properties.get(&FIRST_TRY_WITH_INITIAL_ORDER) {
            graph.properties.set(&FIRST_TRY_WITH_INITIAL_ORDER, false);
            graph.properties.set(&SECOND_TRY_WITH_INITIAL_ORDER, true);
        }

        let mut current_score = score_cache.effective_score(graph);
        if current_score < best_score {
            best_score = current_score;
            best_snapshot.capture_from(graph);
            best_captured = true;
        }

        loop {
            if current_score == 0.0 {
                break;
            }
            forward = !forward;
            let old_score = current_score;
            if let Some(heuristic_rng) = graph_rng.as_mut() {
                sweep_reducing_crossings(
                    graph,
                    heuristic_rng,
                    forward,
                    false,
                    use_median,
                    port_rank_mode,
                    port_rank_modes,
                    cross_min_type,
                    parent_context.as_mut().map(|p| p.reborrow()),
                    random_seed,
                    &mut sweep_scratch,
                    use_graph_rng_for_nested_heuristics,
                );
            } else {
                sweep_reducing_crossings(
                    graph,
                    rng,
                    forward,
                    false,
                    use_median,
                    port_rank_mode,
                    port_rank_modes,
                    cross_min_type,
                    parent_context.as_mut().map(|p| p.reborrow()),
                    random_seed,
                    &mut sweep_scratch,
                    use_graph_rng_for_nested_heuristics,
                );
            }
            current_score = score_cache.effective_score(graph);
            if current_score < best_score {
                best_score = current_score;
                best_snapshot.capture_from(graph);
                best_captured = true;
            }
            if old_score <= current_score {
                break;
            }
        }

        // Do NOT restore the graph at the end of each outer iteration —
        // keep the last sweep's state and rely on `best_captured` to track
        // the best for a final transfer at the very end. Restoring here
        // would undo the work of the next iteration's `set_first_layer_order`,
        // shifting the subsequent random starting point.
        if best_score == 0.0 {
            break;
        }
    }

    // Snapshots are saved only after each thoroughness iteration, never
    // seeding "best" from the initial graph state. With
    // `crossing_minimization_force_node_model_order=true` every post-sweep
    // state is MO-sorted (the model-order insertion sort runs unconditionally
    // inside `minimize_layer`), but a non-sorted initial state can still have
    // fewer raw crossings; if we kept the initial-state snapshot under that
    // flag, `restore_graph_snapshot` would un-sort the layer at the very end.
    // Skip the restore in that case so the MO-respect invariant holds while
    // leaving normal barycenter sweeps unchanged.
    if best_captured {
        restore_graph_snapshot(graph, &best_snapshot);
    }
    if let Some(graph_rng) = graph_rng {
        graph.put_rng(graph_rng);
    }
}

/// Run a single deterministic counter-based pass without the thoroughness
/// outer loop.
///
/// Used when the heuristic is deterministic but not monotone (one-sided
/// greedy switch). After one initial sweep, keep sweeping in the opposite
/// direction while the crossing count keeps falling, then restore the best
/// snapshot.
fn minimize_with_counter_single_pass(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    use_median: bool,
    port_rank_mode: PortRankMode,
    port_rank_modes: &PortRankModes,
    cross_min_type: CrossMinType,
    mut parent_context: Option<ParentContext<'_>>,
    random_seed: i64,
    use_graph_rng_for_heuristic: bool,
    use_graph_rng_for_nested_heuristics: bool,
) {
    let mut forward = rng.next_bool();
    let mut sweep_scratch = SweepScratch::new();
    let mut graph_rng = use_graph_rng_for_heuristic.then(|| graph.take_rng());
    if let Some(heuristic_rng) = graph_rng.as_mut() {
        set_first_layer_order_with_state_scratch(
            graph,
            heuristic_rng,
            forward,
            use_median,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            &mut sweep_scratch.bary_state,
            &mut sweep_scratch.ordered_nodes,
        );

        sweep_reducing_crossings(
            graph,
            heuristic_rng,
            forward,
            true,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            random_seed,
            &mut sweep_scratch,
            use_graph_rng_for_nested_heuristics,
        );
    } else {
        set_first_layer_order_with_state_scratch(
            graph,
            rng,
            forward,
            use_median,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            &mut sweep_scratch.bary_state,
            &mut sweep_scratch.ordered_nodes,
        );

        sweep_reducing_crossings(
            graph,
            rng,
            forward,
            true,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context.as_mut().map(|p| p.reborrow()),
            random_seed,
            &mut sweep_scratch,
            use_graph_rng_for_nested_heuristics,
        );
    }

    let mut score_cache = ScoreCache::new();
    let mut current_score = score_cache.effective_score(graph);
    let mut best_snapshot = GraphSnapshot::default();
    best_snapshot.capture_from(graph);

    loop {
        if current_score == 0.0 {
            break;
        }
        forward = !forward;
        let old_score = current_score;
        if let Some(heuristic_rng) = graph_rng.as_mut() {
            sweep_reducing_crossings(
                graph,
                heuristic_rng,
                forward,
                false,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        } else {
            sweep_reducing_crossings(
                graph,
                rng,
                forward,
                false,
                use_median,
                port_rank_mode,
                port_rank_modes,
                cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                random_seed,
                &mut sweep_scratch,
                use_graph_rng_for_nested_heuristics,
            );
        }
        current_score = score_cache.effective_score(graph);
        if current_score < old_score {
            best_snapshot.capture_from(graph);
        } else {
            break;
        }
    }

    restore_graph_snapshot(graph, &best_snapshot);
    if let Some(graph_rng) = graph_rng {
        graph.put_rng(graph_rng);
    }
}

#[derive(Clone, Copy)]
struct ParentContextInfo {
    graph: NonNull<LGraph>,
    parent_node_id: NodeId,
    parent_layer_idx: usize,
}

impl ParentContextInfo {
    fn from_context(ctx: ParentContext<'_>) -> Self {
        Self {
            graph: NonNull::from(&mut *ctx.graph),
            parent_node_id: ctx.parent_node_id,
            parent_layer_idx: ctx.parent_layer_idx,
        }
    }

    fn as_context<'b>(mut self) -> ParentContext<'b> {
        // SAFETY: the pointer targets an ancestor graph that is inactive while
        // the descendant sweep frame runs. The explicit stack preserves the
        // same parent-before-child borrow discipline as the previous call stack.
        unsafe {
            ParentContext {
                graph: self.graph.as_mut(),
                parent_node_id: self.parent_node_id,
                parent_layer_idx: self.parent_layer_idx,
            }
        }
    }
}

enum SweepGraph<'a> {
    Root(NonNull<LGraph>, PhantomData<&'a mut LGraph>),
    Owned(Box<LGraph>),
}

impl SweepGraph<'_> {
    fn as_mut(&mut self) -> &mut LGraph {
        match self {
            SweepGraph::Root(ptr, _) => {
                // SAFETY: root is active only in its top stack frame.
                unsafe { ptr.as_mut() }
            }
            SweepGraph::Owned(graph) => graph,
        }
    }

    fn ptr(&mut self) -> NonNull<LGraph> {
        NonNull::from(&mut *self.as_mut())
    }
}

enum SweepFrameRng<'a, R: Rng> {
    External(NonNull<R>, PhantomData<&'a mut R>),
    Owned(Box<SeededRng>),
}

impl<'a, R: Rng> SweepFrameRng<'a, R> {
    fn external_for_child(&self) -> Self {
        match self {
            SweepFrameRng::External(ptr, _) => SweepFrameRng::External(*ptr, PhantomData),
            SweepFrameRng::Owned(_) => {
                unreachable!("nested graph RNGs are owned only when descendants take their own RNG")
            }
        }
    }
}

impl<R: Rng> Rng for SweepFrameRng<'_, R> {
    fn next_bool(&mut self) -> bool {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame reads the shared RNG.
                unsafe { ptr.as_mut().next_bool() }
            }
            SweepFrameRng::Owned(rng) => rng.next_bool(),
        }
    }

    fn next_f32(&mut self) -> f32 {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame reads the shared RNG.
                unsafe { ptr.as_mut().next_f32() }
            }
            SweepFrameRng::Owned(rng) => rng.next_f32(),
        }
    }

    fn next_f64(&mut self) -> f64 {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame reads the shared RNG.
                unsafe { ptr.as_mut().next_f64() }
            }
            SweepFrameRng::Owned(rng) => rng.next_f64(),
        }
    }

    fn next_int(&mut self, bound: i32) -> i32 {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame reads the shared RNG.
                unsafe { ptr.as_mut().next_int(bound) }
            }
            SweepFrameRng::Owned(rng) => rng.next_int(bound),
        }
    }

    fn next_long(&mut self) -> i64 {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame reads the shared RNG.
                unsafe { ptr.as_mut().next_long() }
            }
            SweepFrameRng::Owned(rng) => rng.next_long(),
        }
    }

    fn set_seed(&mut self, seed: i64) {
        match self {
            SweepFrameRng::External(ptr, _) => {
                // SAFETY: only the active frame mutates the shared RNG.
                unsafe { ptr.as_mut().set_seed(seed) }
            }
            SweepFrameRng::Owned(rng) => rng.set_seed(seed),
        }
    }
}

struct SweepReturnInfo {
    parent_node_id: NodeId,
    child_version_before: u64,
    child_version_before_sweep: u64,
}

enum SweepStage {
    Init,
    Layer(usize),
    Children { layer_idx: usize, node_ids: Vec<NodeId>, next: usize },
    Done,
}

struct SweepFrame<'a, R: Rng> {
    graph: SweepGraph<'a>,
    rng: SweepFrameRng<'a, R>,
    forward: bool,
    first_sweep: bool,
    use_median: bool,
    port_rank_mode: PortRankMode,
    port_rank_modes: &'a PortRankModes,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContextInfo>,
    random_seed: i64,
    use_graph_rng_for_nested_heuristics: bool,
    scratch: SweepScratch,
    len: usize,
    pre_ordered: bool,
    improved: bool,
    stage: SweepStage,
    return_info: Option<SweepReturnInfo>,
}

struct CompletedSweep {
    graph: Option<Box<LGraph>>,
    scratch: SweepScratch,
    improved: bool,
    return_info: Option<SweepReturnInfo>,
}

// Keep `SweepFrame` inline; boxing here would allocate on nested-graph descent.
#[allow(clippy::large_enum_variant)]
enum SweepAction<'a, R: Rng> {
    Continue,
    PushChild(SweepFrame<'a, R>),
    Finish,
}

impl<'a, R: Rng> SweepFrame<'a, R> {
    fn new_root(
        graph: &'a mut LGraph,
        rng: &'a mut R,
        forward: bool,
        first_sweep: bool,
        use_median: bool,
        port_rank_mode: PortRankMode,
        port_rank_modes: &'a PortRankModes,
        cross_min_type: CrossMinType,
        parent_context: Option<ParentContext<'_>>,
        random_seed: i64,
        scratch: SweepScratch,
        use_graph_rng_for_nested_heuristics: bool,
    ) -> Self {
        Self {
            graph: SweepGraph::Root(NonNull::from(graph), PhantomData),
            rng: SweepFrameRng::External(NonNull::from(rng), PhantomData),
            forward,
            first_sweep,
            use_median,
            port_rank_mode,
            port_rank_modes,
            cross_min_type,
            parent_context: parent_context.map(ParentContextInfo::from_context),
            random_seed,
            use_graph_rng_for_nested_heuristics,
            scratch,
            len: 0,
            pre_ordered: false,
            improved: false,
            stage: SweepStage::Init,
            return_info: None,
        }
    }

    fn into_completed(self) -> CompletedSweep {
        let SweepFrame { mut graph, rng, scratch, improved, return_info, .. } = self;
        if let SweepFrameRng::Owned(rng) = rng {
            graph.as_mut().put_rng(*rng);
        }
        let graph = match graph {
            SweepGraph::Root(_, _) => None,
            SweepGraph::Owned(graph) => Some(graph),
        };
        CompletedSweep { graph, scratch, improved, return_info }
    }
}

fn next_sweep_stage_after_children(forward: bool, len: usize, layer_idx: usize) -> SweepStage {
    if forward {
        let next = layer_idx + 1;
        if next < len { SweepStage::Layer(next) } else { SweepStage::Done }
    } else if layer_idx > 0 {
        SweepStage::Layer(layer_idx - 1)
    } else {
        SweepStage::Done
    }
}

fn sweep_reducing_crossings<R: Rng>(
    graph: &mut LGraph,
    rng: &mut R,
    forward: bool,
    first_sweep: bool,
    use_median: bool,
    port_rank_mode: PortRankMode,
    port_rank_modes: &PortRankModes,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContext<'_>>,
    random_seed: i64,
    scratch: &mut SweepScratch,
    use_graph_rng_for_nested_heuristics: bool,
) -> bool {
    let root_scratch = std::mem::replace(scratch, SweepScratch::new());
    let root_frame = SweepFrame::new_root(
        graph,
        rng,
        forward,
        first_sweep,
        use_median,
        port_rank_mode,
        port_rank_modes,
        cross_min_type,
        parent_context,
        random_seed,
        root_scratch,
        use_graph_rng_for_nested_heuristics,
    );
    let mut stack = vec![root_frame];

    while !stack.is_empty() {
        let action = {
            let frame = stack.last_mut().unwrap();
            step_sweep_frame(frame)
        };

        match action {
            SweepAction::Continue => {}
            SweepAction::PushChild(child) => stack.push(child),
            SweepAction::Finish => {
                let completed = stack.pop().unwrap().into_completed();
                if let Some(parent) = stack.last_mut() {
                    apply_completed_child_sweep(parent, completed);
                } else {
                    *scratch = completed.scratch;
                    return completed.improved;
                }
            }
        }
    }

    false
}

fn step_sweep_frame<'a, R: Rng>(frame: &mut SweepFrame<'a, R>) -> SweepAction<'a, R> {
    let stage = std::mem::replace(&mut frame.stage, SweepStage::Done);
    match stage {
        SweepStage::Init => {
            let graph = frame.graph.as_mut();
            frame.len = graph.layers.len();
            if frame.len == 0 {
                return SweepAction::Finish;
            }

            let start_idx = if frame.forward { 0 } else { frame.len - 1 };
            let (first_try, second_try) = read_try_flags(graph);
            frame.pre_ordered = !frame.first_sweep || first_try || second_try;
            frame.improved |=
                layer_sweep::distribute_ports_while_sweeping_with_fixed_ranks_and_scratch(
                    graph,
                    start_idx,
                    frame.forward,
                    frame.port_rank_mode,
                    frame.cross_min_type,
                    None,
                    &mut frame.scratch.port_distribution,
                );
            let node_ids = graph.layers[start_idx].nodes.to_vec();
            frame.stage = SweepStage::Children { layer_idx: start_idx, node_ids, next: 0 };
            SweepAction::Continue
        }
        SweepStage::Layer(free_layer_idx) => {
            let graph = frame.graph.as_mut();
            let mut parent_context = frame.parent_context.map(ParentContextInfo::as_context);
            let (layer_improved, fixed_ranks) = minimize_layer_with_reusable_fixed_ranks(
                graph,
                free_layer_idx,
                frame.forward,
                frame.pre_ordered,
                &mut frame.rng,
                frame.use_median,
                frame.port_rank_mode,
                frame.cross_min_type,
                parent_context.as_mut().map(|p| p.reborrow()),
                &mut frame.scratch.bary_state,
                &mut frame.scratch.fixed_ranks,
                &mut frame.scratch.ordered_nodes,
            );
            frame.improved |= layer_improved;
            frame.improved |=
                layer_sweep::distribute_ports_while_sweeping_with_fixed_ranks_and_scratch(
                    graph,
                    free_layer_idx,
                    frame.forward,
                    frame.port_rank_mode,
                    frame.cross_min_type,
                    fixed_ranks,
                    &mut frame.scratch.port_distribution,
                );
            let node_ids = graph.layers[free_layer_idx].nodes.to_vec();
            frame.stage = SweepStage::Children { layer_idx: free_layer_idx, node_ids, next: 0 };
            SweepAction::Continue
        }
        SweepStage::Children { layer_idx, node_ids, mut next } => {
            while next < node_ids.len() {
                let node_id = node_ids[next];
                next += 1;

                let graph = frame.graph.as_mut();
                if !p3_uses_nested_graphs(graph) {
                    break;
                }
                let Some(child_graph) = graph.nested(node_id) else {
                    continue;
                };
                if child_graph.layers.is_empty() {
                    continue;
                }
                let child_info = GraphInfoHolder::from_graph(child_graph);
                if child_info.dont_sweep_into() {
                    continue;
                }

                let child_version_before = child_graph.order_version();
                let dummies_transferred =
                    transfer_parent_port_order_to_child_dummies(graph, node_id, frame.forward);

                let Some(mut child_box) = graph.take_nested_boxed(node_id) else {
                    continue;
                };
                let child_version_before_sweep = child_box.order_version();
                let child_port_rank_mode =
                    port_rank_mode_for_graph(&child_box, frame.port_rank_modes);
                let mut parent_graph = frame.graph.ptr();

                let child_rng = if frame.use_graph_rng_for_nested_heuristics {
                    SweepFrameRng::Owned(Box::new(child_box.take_rng()))
                } else {
                    frame.rng.external_for_child()
                };
                let mut child_frame = SweepFrame {
                    graph: SweepGraph::Owned(child_box),
                    rng: child_rng,
                    forward: frame.forward,
                    first_sweep: frame.first_sweep,
                    use_median: frame.use_median,
                    port_rank_mode: child_port_rank_mode,
                    port_rank_modes: frame.port_rank_modes,
                    cross_min_type: frame.cross_min_type,
                    parent_context: Some(ParentContextInfo {
                        graph: parent_graph,
                        parent_node_id: node_id,
                        parent_layer_idx: layer_idx,
                    }),
                    random_seed: frame.random_seed,
                    use_graph_rng_for_nested_heuristics: frame.use_graph_rng_for_nested_heuristics,
                    scratch: SweepScratch::new(),
                    len: 0,
                    pre_ordered: false,
                    improved: false,
                    stage: SweepStage::Init,
                    return_info: Some(SweepReturnInfo {
                        parent_node_id: node_id,
                        child_version_before,
                        child_version_before_sweep,
                    }),
                };

                if !dummies_transferred {
                    let parent_context = ParentContext {
                        // SAFETY: the parent frame is inactive until this child frame completes.
                        graph: unsafe { parent_graph.as_mut() },
                        parent_node_id: node_id,
                        parent_layer_idx: layer_idx,
                    };
                    set_first_layer_order(
                        child_frame.graph.as_mut(),
                        &mut child_frame.rng,
                        frame.forward,
                        frame.use_median,
                        frame.cross_min_type,
                        Some(parent_context),
                    );
                }

                frame.stage = SweepStage::Children { layer_idx, node_ids, next };
                return SweepAction::PushChild(child_frame);
            }

            frame.stage = next_sweep_stage_after_children(frame.forward, frame.len, layer_idx);
            SweepAction::Continue
        }
        SweepStage::Done => SweepAction::Finish,
    }
}

fn apply_completed_child_sweep<R: Rng>(
    parent: &mut SweepFrame<'_, R>,
    mut completed: CompletedSweep,
) {
    let info = completed.return_info.take().expect("nested sweep frame missing return info");
    let child_box = completed.graph.take().expect("nested sweep frame missing graph");
    let child_changed_by_sweep = child_box.order_version() != info.child_version_before_sweep;
    parent.improved |= completed.improved || child_changed_by_sweep;

    let child_changed = child_box.order_version() != info.child_version_before;
    let graph = parent.graph.as_mut();
    graph.set_nested_boxed(info.parent_node_id, child_box);
    if child_changed {
        graph.bump_order_version();
    }
    transfer_child_dummy_order_to_parent_ports(graph, info.parent_node_id);
}

/// Returns true if the free layer's first node is an external-port dummy.
fn first_node_is_external_port_dummy(graph: &LGraph, free_layer_idx: usize) -> bool {
    use crate::graph::node::NodeType;
    if free_layer_idx >= graph.layers.len() {
        return false;
    }
    let Some(&first_nid) = graph.layers[free_layer_idx].nodes.first() else {
        return false;
    };
    graph.node(first_nid).node_type == NodeType::ExternalPort
}

fn minimize_layer_with_reusable_fixed_ranks<'a>(
    graph: &mut LGraph,
    free_layer_idx: usize,
    forward: bool,
    pre_ordered: bool,
    rng: &mut impl Rng,
    use_median: bool,
    port_rank_mode: PortRankMode,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContext<'_>>,
    bary_state_scratch: &mut BarycenterStateMap,
    fixed_rank_scratch: &'a mut PortRanks,
    ordered_scratch: &mut Vec<NodeId>,
) -> (bool, Option<&'a PortRanks>) {
    if matches!(cross_min_type, CrossMinType::Barycenter)
        && !is_first_layer(graph, free_layer_idx, forward)
    {
        let fixed_idx = if forward { free_layer_idx - 1 } else { free_layer_idx + 1 };
        calculate_port_ranks_into(
            graph,
            fixed_idx,
            if forward { PortType::Output } else { PortType::Input },
            port_rank_mode,
            fixed_rank_scratch,
        );
        // Safe to reuse for the immediately following port-distribution step:
        // ordering this free layer does not mutate the fixed layer's node or
        // port order, which are the only inputs to these ranks.
        let effective_pre_ordered =
            pre_ordered || first_node_is_external_port_dummy(graph, free_layer_idx);
        let improved = crate::p3_crossing_min::barycenter_heuristic::order_free_layer_by_heuristic_with_state_scratch(
            graph,
            free_layer_idx,
            forward,
            effective_pre_ordered,
            rng,
            fixed_rank_scratch,
            use_median,
            bary_state_scratch,
            ordered_scratch,
        );
        return (improved, Some(fixed_rank_scratch));
    }

    (
        minimize_layer(
            graph,
            free_layer_idx,
            forward,
            pre_ordered,
            rng,
            use_median,
            port_rank_mode,
            cross_min_type,
            parent_context,
            bary_state_scratch,
            ordered_scratch,
        ),
        None,
    )
}

fn minimize_layer(
    graph: &mut LGraph,
    free_layer_idx: usize,
    forward: bool,
    pre_ordered: bool,
    rng: &mut impl Rng,
    use_median: bool,
    port_rank_mode: PortRankMode,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContext<'_>>,
    bary_state_scratch: &mut BarycenterStateMap,
    ordered_scratch: &mut Vec<NodeId>,
) -> bool {
    use crate::p3_crossing_min::barycenter_heuristic::order_free_layer_by_heuristic_with_state_scratch;

    match cross_min_type {
        CrossMinType::OneSidedGreedySwitch =>
            greedy_switch_layer(graph, free_layer_idx, forward, true, parent_context),
        CrossMinType::TwoSidedGreedySwitch =>
            greedy_switch_layer(graph, free_layer_idx, forward, false, parent_context),
        CrossMinType::Median =>
            if !is_first_layer(graph, free_layer_idx, forward) {
                crate::p3_crossing_min::median_heuristic::minimize_layer(
                    graph,
                    free_layer_idx,
                    forward,
                )
            } else {
                crate::p3_crossing_min::median_heuristic::set_first_layer_order(graph, rng, forward)
            },
        CrossMinType::Barycenter =>
            if !is_first_layer(graph, free_layer_idx, forward) {
                let fixed_idx = if forward { free_layer_idx - 1 } else { free_layer_idx + 1 };
                let port_ranks = calculate_port_ranks(
                    graph,
                    fixed_idx,
                    if forward { PortType::Output } else { PortType::Input },
                    port_rank_mode,
                );
                let effective_pre_ordered =
                    pre_ordered || first_node_is_external_port_dummy(graph, free_layer_idx);
                order_free_layer_by_heuristic_with_state_scratch(
                    graph,
                    free_layer_idx,
                    forward,
                    effective_pre_ordered,
                    rng,
                    &port_ranks,
                    use_median,
                    bary_state_scratch,
                    ordered_scratch,
                )
            } else {
                set_first_layer_order_with_state_scratch(
                    graph,
                    rng,
                    forward,
                    use_median,
                    cross_min_type,
                    parent_context,
                    bary_state_scratch,
                    ordered_scratch,
                )
            },
    }
}

fn set_first_layer_order(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    forward: bool,
    use_median: bool,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContext<'_>>,
) -> bool {
    let mut bary_state_scratch = BarycenterStateMap::new();
    let mut ordered_scratch = Vec::new();
    set_first_layer_order_with_state_scratch(
        graph,
        rng,
        forward,
        use_median,
        cross_min_type,
        parent_context,
        &mut bary_state_scratch,
        &mut ordered_scratch,
    )
}

fn set_first_layer_order_with_state_scratch(
    graph: &mut LGraph,
    rng: &mut impl Rng,
    forward: bool,
    _use_median: bool,
    cross_min_type: CrossMinType,
    parent_context: Option<ParentContext<'_>>,
    bary_state_scratch: &mut BarycenterStateMap,
    ordered_scratch: &mut Vec<NodeId>,
) -> bool {
    let start_idx = if forward { 0 } else { graph.layers.len().saturating_sub(1) };
    if start_idx >= graph.layers.len() {
        return false;
    }

    match cross_min_type {
        CrossMinType::OneSidedGreedySwitch | CrossMinType::TwoSidedGreedySwitch =>
            greedy_switch_layer(
                graph,
                start_idx,
                forward,
                matches!(cross_min_type, CrossMinType::OneSidedGreedySwitch),
                parent_context,
            ),
        CrossMinType::Median =>
            crate::p3_crossing_min::median_heuristic::set_first_layer_order(graph, rng, forward),
        _ => {
            // Barycenter `setFirstLayerOrder` →
            // `minimizeCrossings(nodes, false, true, forward)`:
            // 1. Randomize barycenters: write a fresh `next_f64()` into every
            //    node's `BarycenterState.barycenter` and leave the layer list
            //    order unchanged.
            // 2. With `crossing_minimization_force_node_model_order=true`,
            //    sort via `insertion_sort` using the model-order-aware
            //    barycenter comparator; otherwise plain comparator sort.
            //
            // The states map is populated below so that
            // `compare_based_on_barycenter` does not return `Ordering::Equal`
            // for unset entries — equality there blocks model-order
            // propagation and produces wrong boundary-layer states for
            // dummies sandwiched between real nodes.
            ordered_scratch.clear();
            ordered_scratch.extend_from_slice(&graph.layers[start_idx].nodes);
            bary_state_scratch.reset_with_nodes(ordered_scratch);
            for &node_id in ordered_scratch.iter() {
                let value = rng.next_f64();
                let state = bary_state_scratch
                    .get_mut(node_id)
                    .expect("first-layer node must have bary state");
                state.barycenter = Some(value);
                state.summed_weight = value;
                state.degree = 1;
            }
            if graph.options.crossing_minimization_force_node_model_order {
                use crate::p3_crossing_min::model_order_barycenter_heuristic::ModelOrderSorter;
                let mut sorter = ModelOrderSorter::new();
                sorter.insertion_sort(ordered_scratch, graph, bary_state_scratch);
            } else {
                bary_state_scratch.sort_nodes_by_barycenter(ordered_scratch);
            }
            // Regardless of the `randomize` flag, constraint resolution runs
            // after the sort to honour `IN_LAYER_SUCCESSOR_CONSTRAINTS` and
            // the layout-unit cross-product. Without this call, north/south
            // port dummies drift away from their parent normal node when the
            // first-layer randomizer scatters them, producing visual edge
            // intersections.
            crate::p3_crossing_min::forster_constraint_resolver::apply_constraint_resolution(
                graph,
                start_idx,
                ordered_scratch,
                bary_state_scratch,
            );
            if graph.layers[start_idx].nodes.as_slice() != ordered_scratch.as_slice() {
                graph.layers[start_idx].nodes.clear();
                graph.layers[start_idx].nodes.extend_from_slice(ordered_scratch);
                graph.bump_layer_order_version(start_idx);
            }
            false
        }
    }
}

pub(crate) fn greedy_switch_layer(
    graph: &mut LGraph,
    free_layer_idx: usize,
    forward: bool,
    one_sided: bool,
    parent_context: Option<ParentContext<'_>>,
) -> bool {
    if graph.layers[free_layer_idx].nodes.len() < 2 {
        return false;
    }

    let side = if forward { CrossingCountSide::West } else { CrossingCountSide::East };
    let mut decider = SwitchDecider::new(graph, free_layer_idx, side, one_sided, parent_context);

    // Sweep the layer until a full pass produces no swap, returning whether
    // any swap ever happened (drives the no-counter outer while-loop).
    let mut any_improved = false;
    loop {
        let mut improved = false;
        let len = decider.free_layer_len();
        for upper_idx in 0..len - 1 {
            let lower_idx = upper_idx + 1;
            if decider.does_switch_reduce_crossings(upper_idx, lower_idx) {
                decider.notify_of_switch(upper_idx, lower_idx);
                improved = true;
            }
        }
        if !improved {
            break;
        }
        any_improved = true;
    }

    decider.apply_to_graph(graph, free_layer_idx);
    any_improved
}

/// Reorder the child's external-port dummy layer to match the parent's port
/// positions on the relevant side.
///
/// Returns `true` if the child's first layer was an external-port dummy
/// layer and got reordered; `false` when it was not (in which case the
/// caller should fall back to `set_first_layer_order`).
fn transfer_parent_port_order_to_child_dummies(
    graph: &mut LGraph,
    parent_node: NodeId,
    forward: bool,
) -> bool {
    let side = if forward { PortSide::West } else { PortSide::East };
    let child_has_dummy_layer = {
        let child = graph.nested(parent_node).unwrap();
        let target_layer_idx = if forward { 0 } else { child.layers.len().saturating_sub(1) };
        child.layers.get(target_layer_idx).is_some_and(|layer| {
            layer
                .nodes
                .first()
                .is_some_and(|&node_id| child.node(node_id).node_type == NodeType::ExternalPort)
        })
    };
    if !child_has_dummy_layer {
        return false;
    }

    // Iterate parent ports in N→S→E→W order, reversing the per-side stored
    // order for SOUTH/WEST. Parent ports are stored clockwise (E top→bottom,
    // W bottom→top); the child's external-port dummy layer is laid out
    // top→bottom. Without the reversal, a parent whose WEST ports were just
    // reordered top-most-first lands its dummies in bottom-most-first order
    // in the child, leaving every hierarchical sweep unable to propagate
    // parent port reorderings into the nested graph.
    let mut hierarchical_ports: Vec<PortId> = graph
        .node(parent_node)
        .ports
        .iter()
        .copied()
        .filter(|&port_id| graph.port(port_id).side == side)
        .filter(|&port_id| graph.port(port_id).port_dummy.is_some())
        .collect();
    if matches!(side, PortSide::West | PortSide::South) {
        hierarchical_ports.reverse();
    }
    if hierarchical_ports.is_empty() {
        return false;
    }

    let dummy_nodes: Vec<NodeId> = hierarchical_ports
        .into_iter()
        .filter_map(|port_id| graph.port(port_id).port_dummy)
        .collect();

    let child = graph.nested_mut(parent_node).unwrap();
    let layer_idx = if forward { 0 } else { child.layers.len().saturating_sub(1) };
    if child.layers[layer_idx].nodes != dummy_nodes {
        child.layers[layer_idx].nodes = dummy_nodes;
        child.bump_layer_order_version(layer_idx);
    }
    true
}

fn transfer_child_dummy_order_to_parent_ports(graph: &mut LGraph, parent_node: NodeId) {
    let sides = [PortSide::West, PortSide::East];
    for side in sides {
        let Some(new_order) = child_dummy_port_order(graph, parent_node, side) else {
            continue;
        };
        if new_order.is_empty() {
            continue;
        }
        layer_sweep::reorder_parent_ports_on_side(graph, parent_node, side, &new_order);
        let current_constraints = graph.node(parent_node).port_constraints();
        if current_constraints.is_weaker_than(PortConstraints::FixedOrder) {
            graph.node_mut(parent_node).node_port_constraints = Some(PortConstraints::FixedOrder);
        }
    }
}

fn child_dummy_port_order(
    graph: &LGraph,
    parent_node: NodeId,
    side: PortSide,
) -> Option<Vec<PortId>> {
    let child = graph.nested(parent_node)?;
    if child.layers.is_empty() {
        return None;
    }
    let layer_idx = if side == PortSide::West { 0 } else { child.layers.len().saturating_sub(1) };
    let layer = child.layers.get(layer_idx)?;
    let first_dummy = *layer.nodes.first()?;
    if child.node(first_dummy).node_type != NodeType::ExternalPort {
        return None;
    }

    // Walk the dummy layer top→bottom for East, bottom→top for West, so the
    // assignment matches the parent port list's clockwise stored order
    // (East top→bottom, West bottom→top). Returning a top→bottom list for
    // West would swap port origins relative to their physical position when
    // paired with the parent port iteration in `reorder_parent_ports_on_side`.
    let dummy_iter: Box<dyn Iterator<Item = &NodeId>> = if matches!(side, PortSide::West) {
        Box::new(layer.nodes.iter().rev())
    } else {
        Box::new(layer.nodes.iter())
    };

    // `ORIGIN_PORT` lives on the dummy NODE, not on one of its ports —
    // `create_external_port_dummy_in_nested` sets it on the dummy's node
    // (only NS dummies set it on the port). Reading via
    // `child.port(...).properties` here would silently miss every
    // external-port dummy that was correctly set on the node.
    let mut ports = Vec::new();
    for &dummy_id in dummy_iter {
        let Some(origin_port) = child.node(dummy_id).properties.get(&ORIGIN_PORT) else {
            continue;
        };
        ports.push(origin_port);
    }
    Some(ports)
}

fn restore_graph_snapshot(graph: &mut LGraph, snapshot: &GraphSnapshot) {
    enum RestoreFrame<'a> {
        Enter {
            graph: *mut LGraph,
            snapshot: &'a GraphSnapshot,
        },
        Finish {
            graph: *mut LGraph,
            snapshot: &'a GraphSnapshot,
            changed: bool,
            children: Vec<*mut LGraph>,
        },
    }

    let mut changed_by_graph: HashMap<*mut LGraph, bool> = HashMap::new();
    let mut stack = vec![RestoreFrame::Enter { graph, snapshot }];
    while let Some(frame) = stack.pop() {
        match frame {
            RestoreFrame::Enter { graph, snapshot } => {
                // SAFETY: graph pointers are unique nested graph boxes.
                let graph_ref = unsafe { &mut *graph };
                let mut changed = false;
                for layer_idx in 0..graph_ref.layers.len().min(snapshot.layers.len()) {
                    if graph_ref.layers[layer_idx].nodes.as_slice()
                        != snapshot.layers[layer_idx].as_slice()
                    {
                        graph_ref.layers[layer_idx].nodes = snapshot.layers[layer_idx].clone();
                        graph_ref.bump_layer_order_version(layer_idx);
                        changed = true;
                    }
                }

                for (node_id, ports) in &snapshot.ports {
                    if graph_ref.node(*node_id).ports.as_slice() != ports.as_slice() {
                        graph_ref.node_mut(*node_id).ports = ports.clone().into();
                        graph_ref.bump_node_order_version(*node_id);
                        changed = true;
                    }
                }

                let mut children = Vec::new();
                let mut child_frames = Vec::new();
                for (node_id, child_snapshot) in &snapshot.nested {
                    if let Some(child) = graph_ref.nested_mut(*node_id) {
                        let child_ptr = child as *mut LGraph;
                        children.push(child_ptr);
                        child_frames.push(RestoreFrame::Enter {
                            graph: child_ptr,
                            snapshot: child_snapshot,
                        });
                    }
                }
                stack.push(RestoreFrame::Finish { graph, snapshot, changed, children });
                for child_frame in child_frames.into_iter().rev() {
                    stack.push(child_frame);
                }
            }
            RestoreFrame::Finish { graph, snapshot, mut changed, children } => {
                for child in children {
                    changed |= changed_by_graph.remove(&child).unwrap_or(false);
                }
                // SAFETY: child frames have finished.
                let graph_ref = unsafe { &mut *graph };
                for layer in &snapshot.layers {
                    for &node_id in layer {
                        let current_constraints = graph_ref.node(node_id).port_constraints();
                        if current_constraints.is_weaker_than(PortConstraints::FixedOrder) {
                            graph_ref.node_mut(node_id).node_port_constraints =
                                Some(PortConstraints::FixedOrder);
                            changed = true;
                        }
                    }
                }

                if changed {
                    graph_ref.bump_order_version();
                }
                correct_north_south_port_sides(graph_ref);
                changed_by_graph.insert(graph, changed);
            }
        }
    }
}

/// Walks every layer and corrects the side of any `NORTH_SOUTH_PORT`
/// dummy's origin port whose post-sweep position has crossed its origin
/// node.
fn correct_north_south_port_sides(graph: &mut LGraph) {
    use crate::properties::internal::{IN_LAYER_LAYOUT_UNIT, ORIGIN_PORT};
    for layer_idx in 0..graph.layers.len() {
        let mut dummies: Vec<(NodeId, usize)> = Vec::new();
        for (idx, &nid) in graph.layers[layer_idx].nodes.iter().enumerate() {
            if graph.node(nid).node_type == crate::graph::node::NodeType::NorthSouthPort {
                dummies.push((nid, idx));
            }
        }
        if dummies.is_empty() {
            continue;
        }

        // Position-in-layer table: NodeId -> index. Build it only for layers
        // that actually contain NORTH_SOUTH_PORT dummies; most flat graphs
        // have none and do not need the table at all.
        let mut id_of: std::collections::HashMap<NodeId, usize> =
            std::collections::HashMap::with_capacity(graph.layers[layer_idx].nodes.len());
        for (idx, &nid) in graph.layers[layer_idx].nodes.iter().enumerate() {
            id_of.insert(nid, idx);
        }

        for (dummy_id, dummy_pos) in dummies {
            let Some(origin_id) = graph.node(dummy_id).properties.get(&IN_LAYER_LAYOUT_UNIT) else {
                continue;
            };
            let Some(&origin_pos) = id_of.get(&origin_id) else { continue };
            let Some(&dummy_port) = graph.node(dummy_id).ports.first() else {
                continue;
            };
            let Some(target_port) = graph.port(dummy_port).properties.get(&ORIGIN_PORT) else {
                continue;
            };

            let current_side = graph.port(target_port).side;
            let port_height = graph.port(target_port).size.y;
            let explicit_anchor = graph.port(target_port).explicitly_supplied_anchor;
            match current_side {
                PortSide::North if dummy_pos > origin_pos => {
                    let port = graph.port_mut(target_port);
                    port.side = PortSide::South;
                    if explicit_anchor {
                        port.anchor.y = port_height - port.anchor.y;
                    }
                }
                PortSide::South if origin_pos > dummy_pos => {
                    let port = graph.port_mut(target_port);
                    port.side = PortSide::North;
                    if explicit_anchor {
                        port.anchor.y = -(port_height - port.anchor.y);
                    }
                }
                _ => {}
            }
        }
    }
}

/// Pick the right crossing-score function: when either
/// `consider_model_order_crossing_counter_node_influence` or
/// `_port_influence` is non-zero, use the weighted score; otherwise use the
/// plain crossing count (cast to `f64`).
fn effective_score(graph: &LGraph, scratch: &mut counting::CountingScratch) -> f64 {
    let node_inf = graph.options.consider_model_order_crossing_counter_node_influence;
    let port_inf = graph.options.consider_model_order_crossing_counter_port_influence;
    if node_inf != 0.0 || port_inf != 0.0 {
        total_crossing_score_weighted(graph, scratch)
    } else {
        total_crossing_score(graph, scratch) as f64
    }
}

fn total_crossing_score(graph: &LGraph, scratch: &mut counting::CountingScratch) -> usize {
    let mut total = 0;
    let mut stack = vec![graph];
    while let Some(graph) = stack.pop() {
        total += counting::count_all_crossings_with_scratch(graph, scratch);
        if p3_uses_nested_graphs(graph) {
            let mut children = Vec::new();
            for (_, node) in graph.nodes_iter() {
                if let Some(child) = node.nested_graph_ref() {
                    let child_info = GraphInfoHolder::from_graph(child);
                    if !child_info.dont_sweep_into() {
                        children.push(child);
                    }
                }
            }
            for child in children.into_iter().rev() {
                stack.push(child);
            }
        }
    }
    total
}

/// Weighted crossing score, applied when
/// `CONSIDER_MODEL_ORDER_CROSSING_COUNTER_NODE_INFLUENCE` or
/// `_PORT_INFLUENCE` is non-zero.
///
/// score = base_crossings
///       + NODE_INFLUENCE * node_model_order_conflicts
///       + PORT_INFLUENCE * port_model_order_conflicts
///
/// Node conflicts count layer-internal inversions of `MODEL_ORDER` (pairs
/// where `i < j` but `MODEL_ORDER[layer[i]] > MODEL_ORDER[layer[j]]`). This
/// is a lighter approximation of running a full model-order comparator over
/// every pair (full comparator port is a future improvement).
///
/// Port conflicts count, per node, inversions over the "effective port
/// model order" — the `MODEL_ORDER` of the target node that each port's
/// first outgoing edge reaches (following long-edge dummies). Ports
/// without any outgoing edge contribute `i32::MAX`, so they sort to the
/// end stably.
fn total_crossing_score_weighted(graph: &LGraph, scratch: &mut counting::CountingScratch) -> f64 {
    use crate::properties::internal::MODEL_ORDER;
    let mut total = 0.0;
    let mut stack = vec![graph];
    while let Some(graph) = stack.pop() {
        total += counting::count_all_crossings_with_scratch(graph, scratch) as f64;

        let node_inf = graph.options.consider_model_order_crossing_counter_node_influence;
        if node_inf != 0.0 {
            let mut conflicts = 0usize;
            for layer in &graph.layers {
                conflicts +=
                    scratch.count_i32_inversions(layer.nodes.iter().filter_map(|&node_id| {
                        let model_order = graph.node(node_id).properties.get_copy(&MODEL_ORDER);
                        (model_order != -1).then_some(model_order)
                    }));
            }
            total += node_inf * conflicts as f64;
        }

        let port_inf = graph.options.consider_model_order_crossing_counter_port_influence;
        if port_inf != 0.0 {
            let mut conflicts = 0usize;
            for layer in &graph.layers {
                for &nid in &layer.nodes {
                    conflicts += scratch.count_i32_inversions(
                        graph
                            .node(nid)
                            .ports
                            .iter()
                            .copied()
                            .map(|port_id| port_effective_model_order(graph, port_id)),
                    );
                }
            }
            total += port_inf * conflicts as f64;
        }

        if p3_uses_nested_graphs(graph) {
            let mut children = Vec::new();
            for (_, node) in graph.nodes_iter() {
                if let Some(child) = node.nested_graph_ref() {
                    let child_info = GraphInfoHolder::from_graph(child);
                    if !child_info.dont_sweep_into() {
                        children.push(child);
                    }
                }
            }
            for child in children.into_iter().rev() {
                stack.push(child);
            }
        }
    }
    total
}

/// Return the effective model order for `port`: the minimum MODEL_ORDER
/// across the target nodes reached by its outgoing edges (long-edge
/// aware). Ports without any outgoing edge return `i32::MAX` so they
/// sort last and never trigger an inversion against a real port.
fn port_effective_model_order(graph: &LGraph, port_id: crate::graph::index::PortId) -> i32 {
    use crate::{graph::node::NodeType, properties::internal::MODEL_ORDER};
    let mut best = i32::MAX;
    for &eid in &graph.port(port_id).outgoing_edges {
        // Follow long-edge chains to the terminal normal node.
        let mut cursor_port = graph.edge(eid).target;
        loop {
            let owner = graph.port(cursor_port).owner;
            let node = graph.node(owner);
            if node.node_type == NodeType::Normal {
                if let Some(&model_order) = node.properties.get_ref(&MODEL_ORDER) {
                    best = best.min(model_order);
                }
                break;
            }
            // For long-edge dummies points at the
            // original terminal port; stop there and read its owner.
            if let Some(long_target) = node.long_edge_target {
                let target_owner = graph.port(long_target).owner;
                if let Some(&model_order) =
                    graph.node(target_owner).properties.get_ref(&MODEL_ORDER)
                {
                    best = best.min(model_order);
                }
                break;
            }
            // Otherwise walk forward one more layer through the dummy's
            // first outgoing edge.
            let Some(&next_eid) =
                node.ports.iter().flat_map(|&p| graph.port(p).outgoing_edges.iter()).next()
            else {
                break;
            };
            cursor_port = graph.edge(next_eid).target;
        }
    }
    best
}
