//! Cross-hierarchy edges are edges whose source and target live in different levels of
//! the nested graph tree. The preprocessor walks the hierarchy depth-first and replaces
//! each such edge with a sequence of hierarchy-local dummy edge segments plus external
//! port dummy nodes. Dummy segments accumulate in a per-edge multimap attached to the
//! root graph via `CROSS_HIERARCHY_MAP`; the postprocessor consumes that map to restore
//! the original edge layout.
//!
//! The postprocessor (`postprocess`) reassembles original edges from their segments
//! using `change_coord_system` to unify coordinate frames, stitches bend-point chains
//! under the `UNNECESSARY_BENDPOINTS` tolerance rule, and restores each original edge
//! to its source/target ports.

use hashbrown::HashMap;
use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        hierarchical_edge::HierarchicalEdgeData,
        index::{EdgeId, LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    options::enums::{
        EdgeConstraint, EdgeLabelPlacement, InLayerConstraint, LayerConstraint, LayoutDirection,
        PortConstraints, PortLabelPlacement, PortType,
    },
    properties::{
        PropertyKey, PropertyMap,
        graph_properties::GraphProperties,
        internal::{
            EDGE_LABEL_PLACEMENT, EXT_PORT_SIDE, EXT_PORT_SIZE, GRAPH_PROPERTIES,
            IN_LAYER_CONSTRAINT, INSIDE_CONNECTIONS, INSIDE_SELF_LOOPS_ACTIVATE,
            INSIDE_SELF_LOOPS_YO, LAYER_CONSTRAINT, MERGE_HIERARCHY_EDGES, NODE_EDGE_CONSTRAINT,
            ORIGIN_NODE, ORIGIN_PORT, ORIGINAL_LABEL_EDGE, PORT_ANCHOR, PORT_BORDER_OFFSET,
            PORT_INDEX, PORT_LABEL_PLACEMENT, PORT_RATIO_OR_POSITION,
        },
    },
};

/// Records a single hierarchy-local edge segment created when breaking a cross-hierarchy
/// edge.
///
/// `graph_parent_node` identifies the enclosing graph by its parent `NodeId` (or `None`
/// for the root graph). This is a stable identity the postprocessor can use to locate
/// the graph and compute coordinate-system transforms without holding a raw `&LGraph`.
#[derive(Debug, Clone, Copy)]
pub struct CrossHierarchyEdge {
    /// Dummy edge used to compute layout for this segment.
    pub new_edge: EdgeId,
    /// Identity of the graph containing `new_edge`: either the root (`None`) or the
    /// parent node whose `nested_graph` points at that LGraph.
    pub graph_parent_node: Option<NodeId>,
    /// Segment direction: `Input` for edges moving into deeper hierarchy, `Output`
    /// for edges moving out.
    pub port_type: PortType,
}

impl CrossHierarchyEdge {
    fn new(new_edge: EdgeId, graph_parent_node: Option<NodeId>, port_type: PortType) -> Self {
        CrossHierarchyEdge { new_edge, graph_parent_node, port_type }
    }

    /// Returns the actual source port of this segment. If the dummy edge's source lives
    /// on an `ExternalPort` dummy node, the original port (stored in `ORIGIN_PORT`) is
    /// returned instead.
    pub fn actual_source(&self, root: &LGraph) -> Option<PortId> {
        let graph = self.segment_graph(root)?;
        let edge = graph.edge(self.new_edge);
        let src_port = graph.port(edge.source);
        let owner = graph.node(src_port.owner);
        if owner.node_type == NodeType::ExternalPort {
            owner.properties.get(&ORIGIN_PORT)
        } else {
            Some(edge.source)
        }
    }

    /// Returns the actual target port (see `actual_source`).
    pub fn actual_target(&self, root: &LGraph) -> Option<PortId> {
        let graph = self.segment_graph(root)?;
        let edge = graph.edge(self.new_edge);
        let tgt_port = graph.port(edge.target);
        let owner = graph.node(tgt_port.owner);
        if owner.node_type == NodeType::ExternalPort {
            owner.properties.get(&ORIGIN_PORT)
        } else {
            Some(edge.target)
        }
    }

    /// Resolve `graph_parent_node` to the `&LGraph` that holds this segment.
    pub fn segment_graph<'a>(&self, root: &'a LGraph) -> Option<&'a LGraph> {
        match self.graph_parent_node {
            None => Some(root),
            Some(parent) => root.find_graph_containing(parent).and_then(|g| g.nested(parent)),
        }
    }
}

struct CrossHierarchyMapMarker;

/// Root-graph property storing the cross-hierarchy map built by the preprocessor
/// and consumed by the postprocessor.
///
/// Keyed by the original edge's `EdgeId`. Each entry holds the list of
/// dummy-edge segments (outer + inner) that replaced it, unordered; the
/// postprocessor sorts via `CrossHierarchyEdgeComparator`.
pub static CROSS_HIERARCHY_MAP: std::sync::LazyLock<
    PropertyKey<HashMap<EdgeId, SmallVec<CrossHierarchyEdge, 4>>>,
> = std::sync::LazyLock::new(|| PropertyKey::of::<CrossHierarchyMapMarker>(HashMap::new));

/// Orders cross-hierarchy edge segments from source (outermost on the output side) to
/// target (outermost on the input side).
///
/// Output segments precede input segments. Within the same `PortType`, deeper graphs
/// come later for `Input` (diving down) and earlier for `Output` (rising up). The
/// depth is the number of parent nodes between the segment's graph and the root.
pub fn compare_cross_hierarchy_edges(
    root: &LGraph,
    a: &CrossHierarchyEdge,
    b: &CrossHierarchyEdge,
) -> std::cmp::Ordering {
    if a.port_type == PortType::Output && b.port_type == PortType::Input {
        return std::cmp::Ordering::Less;
    }
    if a.port_type == PortType::Input && b.port_type == PortType::Output {
        return std::cmp::Ordering::Greater;
    }
    let level_a = hierarchy_level(root, a.graph_parent_node);
    let level_b = hierarchy_level(root, b.graph_parent_node);
    if a.port_type == PortType::Output {
        level_b.cmp(&level_a)
    } else {
        level_a.cmp(&level_b)
    }
}

fn hierarchy_level(root: &LGraph, graph_parent: Option<NodeId>) -> u32 {
    // Root graph is level 0; each nesting step adds 1.
    let mut current = match graph_parent {
        None => return 0,
        Some(p) => p,
    };
    let mut level = 1u32;
    loop {
        let containing = match root.find_graph_containing(current) {
            Some(g) => g,
            None => return level,
        };
        match containing.parent_node {
            None => return level,
            Some(p) => {
                current = p;
                level += 1;
            }
        }
    }
}

/// Carries information about a newly-created external port from a child
/// graph up to its parent.
#[derive(Debug, Clone)]
struct ExternalPort {
    /// Original cross-hierarchy edges that this external port represents.
    orig_edges: SmallVec<EdgeId, 2>,
    /// New (dummy) edge created in the containing graph.
    new_edge: EdgeId,
    /// Dummy node inserted on the containing graph's layerless list for this port.
    dummy_node: NodeId,
    /// Sole port on `dummy_node` that the new edge attaches to.
    dummy_port: PortId,
    /// Flow direction of this external port.
    port_type: PortType,
    /// True when this port should be exposed to the next level up (`false` for
    /// segments that terminate at the parent node itself).
    exported: bool,
}

/// Preprocess a compound graph: split cross-hierarchy edges into hierarchy-local
/// segments and publish the resulting map under `CROSS_HIERARCHY_MAP`.
///
/// Must be invoked exactly once on the root graph; `transform_hierarchy_edges`
/// traverses the full nested tree from there.
/// Threading this stage into nested-graph pipelines as well would walk every
/// level twice and accumulate cross-arena `dummy_node_map` entries keyed by
/// ports minted in the nested's arena, breaking
/// `set_sides_of_ports_to_sides_of_dummy_nodes`, `create_dummy_edge`, and
/// `connect_child`.
pub fn preprocess(graph: &mut LGraph) {
    let mut state = PreprocessorState::default();
    convert_hierarchical_edges_to_local(graph, &mut state);
    transform_hierarchy_edges(graph, None, &mut state);
    ensure_declared_port_dummies_for_external_nested(graph, &mut state);
    move_labels_and_remove_original_edges(graph, &state);
    set_sides_of_ports_to_sides_of_dummy_nodes(graph, &state);
    graph.properties.set(&CROSS_HIERARCHY_MAP, state.cross_hierarchy_map);
}

/// Install external-port dummies for the SEPARATE_CHILDREN dispatch path.
///
/// Source importers pre-build nested LGraphs at parse time, so we do the
/// external-port transformation here: walk every compound top-down,
/// run `check_external_ports_for_compound` to decide whether the level needs
/// external-port dummies, and transform each port into an EP dummy in the
/// nested LGraph. A second pass then drains `root.hierarchical_edges` and
/// adds the local dummy->child / child->dummy edges those records map to.
/// Idempotent: if the nested LGraph already contains an EP dummy referencing
/// one of the compound's ports, the per-compound work is skipped (so
/// repeated SEPARATE layout passes re-entering this function are no-ops).
/// The cross_hierarchy_map is not populated since SEPARATE has
/// no `postprocess` step.
pub fn install_external_ports_for_separate_hierarchy(graph: &mut LGraph) {
    let root_ptr: *mut LGraph = graph as *mut LGraph;
    install_external_ports_in_subtree(graph, root_ptr);
    convert_hierarchical_edges_to_local_for_separate(graph);
    // Source import creates external-port dummies BEFORE iterating children,
    // so `layerlessNodes` order is `[ep_dummy_for_each_port, child_1,
    // child_2, ...]`. `ComponentsProcessor.split` iterates in that order and
    // starts DFS from the first node, so the first connected component
    // dumped is rooted at an EP dummy and visits the dummy's neighbour
    // child before the rest. Source import creates children first and only
    // later inserts EP dummies, so without this reorder dummies
    // append to the end of `layerless_nodes` — giving a different
    // DFS-pre-order for `ComponentsProcessor.split` and propagating into
    // a different P3 layer-internal order (and therefore a different P4
    // BK output).
    reorder_eps_first_in_subtree(graph);
}

/// Move all `NodeType::ExternalPort` nodes to the front of `layerless_nodes`
/// while preserving relative order within each group.
fn reorder_eps_first_in_subtree(graph: &mut LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: each pointer is a unique nested graph box.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let mut eps: Vec<NodeId> = Vec::new();
        let mut others: Vec<NodeId> = Vec::new();
        for &nid in &graph.layerless_nodes {
            if graph.node(nid).node_type == NodeType::ExternalPort {
                eps.push(nid);
            } else {
                others.push(nid);
            }
        }
        if !eps.is_empty() {
            graph.layerless_nodes.clear();
            graph.layerless_nodes.extend(eps);
            graph.layerless_nodes.extend(others);
        }
        let children: Vec<std::ptr::NonNull<LGraph>> = graph
            .nested_graphs_mut()
            .map(|(_, nested)| std::ptr::NonNull::from(nested))
            .collect();
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }
}

/// Walk every compound under `graph`, running `process_compound_for_separate`
/// for each. Order is top-down (process compound, then recurse into its nested
/// subtree).
fn install_external_ports_in_subtree(graph: &mut LGraph, root_ptr: *mut LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: each pointer is a unique nested graph box.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let compound_ids: Vec<NodeId> = graph
            .nodes_iter()
            .filter_map(|(id, n)| if n.nested_graph.is_some() { Some(id) } else { None })
            .collect();
        for compound in compound_ids {
            process_compound_for_separate(graph, compound, root_ptr);
        }
        let children: Vec<std::ptr::NonNull<LGraph>> = graph
            .nested_graphs_mut()
            .map(|(_, nested)| std::ptr::NonNull::from(nested))
            .collect();
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }
}

/// Process one compound's external ports, gated on `checkExternalPorts`.
/// Idempotent: returns early if the nested LGraph already has any EP dummy
/// whose `ORIGIN_PORT` matches one of the compound's ports.
fn process_compound_for_separate(
    parent_graph: &mut LGraph,
    compound: NodeId,
    root_ptr: *mut LGraph,
) {
    let port_ids: Vec<PortId> = parent_graph.node(compound).ports.iter().copied().collect();
    if port_ids.is_empty() {
        return;
    }

    // Idempotency: if a previous install run already created EP dummies for
    // this compound, skip. Repeated SEPARATE hierarchy passes should not
    // double-create dummies on later visits.
    let already_installed = if let Some(nested) = parent_graph.nested(compound) {
        nested.layerless_nodes.iter().any(|&nid| {
            let n = nested.node(nid);
            if n.node_type != NodeType::ExternalPort {
                return false;
            }
            n.properties.get(&ORIGIN_PORT).is_some_and(|op| port_ids.contains(&op))
        })
    } else {
        false
    };
    if already_installed {
        return;
    }

    let port_constraints = match parent_graph.node(compound).port_constraints() {
        PortConstraints::Undefined => PortConstraints::Free,
        other => other,
    };

    // SAFETY: `root_ptr` was registered by the caller (live `&mut LGraph` at
    // the entry of `install_external_ports_for_separate_hierarchy`). The
    // shared `&LGraph` we expose here aliases the caller's mutable borrow on
    // root, but `parent_graph` is either `root` itself (in which case we'd
    // alias) or a nested graph reached through `Box::into_raw`. For the root
    // case we only read `root.hierarchical_edges`; the install pass does not
    // mutate that vector until phase 2.
    let root = unsafe { &*root_ptr };
    let (has_external_ports, _has_hyperedges) =
        check_external_ports_for_compound(parent_graph, compound, &port_ids, root);
    if !has_external_ports {
        return;
    }

    for port_id in port_ids {
        transform_external_port_in_nested(
            parent_graph,
            compound,
            port_id,
            port_constraints,
            root_ptr,
        );
    }
}

/// Decide whether a compound's children expose external ports and whether any
/// port carries hyperedges.
///
/// Returns `(has_external_ports, has_hyperedges)`. `has_external_ports` is
/// true when at least one port of the compound has cross-hierarchy edges to
/// children of the compound, an inside-self-loop, or inside-placed labels.
/// `has_hyperedges` is true when at least one port has more than one such edge.
fn check_external_ports_for_compound(
    parent_graph: &LGraph,
    compound: NodeId,
    port_ids: &[PortId],
    root: &LGraph,
) -> (bool, bool) {
    let inside_loops_active =
        parent_graph.node(compound).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);
    let port_label_placement = parent_graph.node(compound).properties.get(&PORT_LABEL_PLACEMENT);

    let mut has_external_ports = false;
    let mut has_hyperedges = false;

    for &port_id in port_ids {
        let mut external_port_edges = 0i32;

        // Local edges: count inside-self-loops as external port edges. For
        // non-self-loop local edges, the target is at the same level as
        // `compound` (a sibling), which does not satisfy the `connectsToChild`
        // predicate, so they don't contribute to the external-port count here.
        for &edge_id in parent_graph.port(port_id).outgoing_edges.iter() {
            let edge = parent_graph.edge(edge_id);
            let target_owner = parent_graph.port(edge.target).owner;
            let is_self_loop = target_owner == compound;
            let is_inside_self_loop =
                is_self_loop && inside_loops_active && edge.properties.get(&INSIDE_SELF_LOOPS_YO);
            if is_inside_self_loop {
                external_port_edges += 1;
                if external_port_edges > 1 {
                    break;
                }
            }
        }
        if external_port_edges <= 1 {
            for &edge_id in parent_graph.port(port_id).incoming_edges.iter() {
                let edge = parent_graph.edge(edge_id);
                let source_owner = parent_graph.port(edge.source).owner;
                let is_self_loop = source_owner == compound;
                let is_inside_self_loop = is_self_loop
                    && inside_loops_active
                    && edge.properties.get(&INSIDE_SELF_LOOPS_YO);
                if is_inside_self_loop {
                    external_port_edges += 1;
                    if external_port_edges > 1 {
                        break;
                    }
                }
            }
        }

        // Cross-hierarchy edges: a hierarchical_edges entry whose other end
        // sits in a graph owned by `compound` (i.e. the deeper end is a
        // child of `compound`'s nested LGraph). Matches the `connectsToChild`
        // predicate.
        for h_edge in root.hierarchical_edges.iter() {
            if external_port_edges > 1 {
                break;
            }
            let connects_to_child = if h_edge.source.port == port_id {
                h_edge.target.graph_parent == Some(compound)
            } else if h_edge.target.port == port_id {
                h_edge.source.graph_parent == Some(compound)
            } else {
                false
            };
            if connects_to_child {
                external_port_edges += 1;
            }
        }

        if external_port_edges > 0
            || (port_label_placement.contains(PortLabelPlacement::INSIDE)
                && !parent_graph.port(port_id).labels.is_empty())
        {
            has_external_ports = true;
        }

        if external_port_edges > 1 {
            has_hyperedges = true;
        }

        if has_external_ports && has_hyperedges {
            break;
        }
    }

    (has_external_ports, has_hyperedges)
}

/// Transform an external port into a dummy node inside the nested LGraph.
///
/// Computes net flow over both local and cross-hierarchy edges, infers
/// `PORT_BORDER_OFFSET` if missing, creates the EP dummy via
/// `create_external_port_dummy_in_nested`, then sets dummy properties,
/// per-side `connected_to_external_nodes`, `PORT_LABELS_PLACEMENT` outside,
/// and transfers labels (with size adjustments per the outside-non-fixed
/// rules) onto the dummy port.
fn transform_external_port_in_nested(
    parent_graph: &mut LGraph,
    compound: NodeId,
    compound_port: PortId,
    port_constraints: PortConstraints,
    root_ptr: *mut LGraph,
) -> NodeId {
    let net_flow = calculate_net_flow_root_aware(parent_graph, compound_port, compound, root_ptr);

    let port_pos = parent_graph.port(compound_port).position;
    let port_size = parent_graph.port(compound_port).size;
    let port_node_size = parent_graph.node(compound).size;
    let port_side = parent_graph.port(compound_port).side;

    // PORT_BORDER_OFFSET inference:
    // - If `(0, 0)` position, use 0.
    // - Else, compute via `calc_port_offset`.
    if !parent_graph.port(compound_port).properties.has(&PORT_BORDER_OFFSET) {
        let port_offset = if port_pos.x == 0.0 && port_pos.y == 0.0 {
            0.0
        } else {
            calc_port_offset(port_pos, port_size, port_node_size, port_side)
        };
        parent_graph
            .port_mut(compound_port)
            .properties
            .set(&PORT_BORDER_OFFSET, port_offset);
    }

    // Read port-label placement on the compound (used to size labels).
    let port_label_placement = parent_graph.node(compound).properties.get(&PORT_LABEL_PLACEMENT);
    let inside_port_labels = port_label_placement.contains(PortLabelPlacement::INSIDE);
    let port_labels_fixed = port_label_placement.is_fixed();

    let connected = is_connected_to_external_nodes(parent_graph, compound_port, compound, root_ptr);
    if is_connected_to_child_nodes(parent_graph, compound_port, compound, root_ptr) {
        parent_graph.port_mut(compound_port).properties.set(&INSIDE_CONNECTIONS, true);
    }

    let dummy_node = create_external_port_dummy_in_nested(
        parent_graph,
        compound,
        compound_port,
        port_constraints,
        net_flow,
    );
    let final_side = parent_graph.port(compound_port).side;

    // Snapshot label data from `parent_graph` before grabbing nested mut.
    let label_count = parent_graph.port(compound_port).labels.len();
    let labels: Vec<(String, Vec2, Vec2)> = (0..label_count)
        .map(|i| {
            let lid = parent_graph.port(compound_port).labels[i];
            let l = parent_graph.label(lid);
            (l.text.clone(), l.size, l.position)
        })
        .collect();

    let nested = parent_graph.nested_mut(compound).unwrap();
    let dummy_port = *nested.node(dummy_node).ports.first().unwrap();
    nested.port_mut(dummy_port).connected_to_external_nodes = connected;

    // The dummy gets PORT_LABELS_PLACEMENT outside.
    nested
        .node_mut(dummy_node)
        .properties
        .set(&PORT_LABEL_PLACEMENT, PortLabelPlacement::OUTSIDE);

    // Transfer labels. For OUTSIDE non-fixed labels, zero out the dimension
    // that does not project inward so they don't push the node bigger.
    for (text, mut size, pos) in labels {
        if !inside_port_labels && !port_labels_fixed {
            match final_side {
                PortSide::East | PortSide::West => size.x = 0.0,
                PortSide::North | PortSide::South => size.y = 0.0,
                _ => {}
            }
        }
        let new_label = nested.add_port_label(dummy_port, text, size);
        nested.label_mut(new_label).position = pos;
    }

    dummy_node
}

/// Compute the port border offset along its assigned side. The offset is the
/// amount by which the port is moved outside the node along that side: 0 means
/// the port touches the outside border. Returns 0 for `Undefined` side.
fn calc_port_offset(port_pos: Vec2, port_size: Vec2, owner_size: Vec2, side: PortSide) -> f64 {
    match side {
        PortSide::North => -(port_pos.y + port_size.y),
        PortSide::East => port_pos.x - owner_size.x,
        PortSide::South => port_pos.y - owner_size.y,
        PortSide::West => -(port_pos.x + port_size.x),
        PortSide::Undefined => 0.0,
    }
}

/// Net-flow calculation for a compound port. Counts both local edges (in
/// `parent_graph`) and cross-hierarchy edges (`root.hierarchical_edges`).
fn calculate_net_flow_root_aware(
    parent_graph: &LGraph,
    port_id: PortId,
    compound: NodeId,
    root_ptr: *mut LGraph,
) -> i32 {
    let inside_loops_active =
        parent_graph.node(compound).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);
    let mut output_vote = 0i32;
    let mut input_vote = 0i32;

    // Outgoing local edges.
    for &edge_id in parent_graph.port(port_id).outgoing_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let target_owner = parent_graph.port(edge.target).owner;
        let is_self_loop = target_owner == compound;
        let is_inside_self_loop =
            is_self_loop && inside_loops_active && edge.properties.get(&INSIDE_SELF_LOOPS_YO);
        if is_self_loop && is_inside_self_loop {
            input_vote += 1;
        } else if is_self_loop {
            output_vote += 1;
        } else {
            // Local non-self-loop: target is at the same level as the
            // compound, so the default branch increments output.
            output_vote += 1;
        }
    }

    // Incoming local edges.
    for &edge_id in parent_graph.port(port_id).incoming_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let source_owner = parent_graph.port(edge.source).owner;
        let is_self_loop = source_owner == compound;
        let is_inside_self_loop =
            is_self_loop && inside_loops_active && edge.properties.get(&INSIDE_SELF_LOOPS_YO);
        if is_self_loop && is_inside_self_loop {
            output_vote += 1;
        } else {
            input_vote += 1;
        }
    }

    // Cross-hierarchy edges (in root).
    // SAFETY: `root_ptr` was registered by `install_external_ports_for_separate_hierarchy`
    // which holds the live `&mut LGraph`. The reference here aliases that mut borrow
    // for read-only access to `hierarchical_edges`; no mutations cross.
    let root = unsafe { &*root_ptr };
    for h_edge in root.hierarchical_edges.iter() {
        if h_edge.source.port == port_id {
            // Outgoing.
            if h_edge.target.graph_parent == Some(compound) {
                input_vote += 1; // target is a child of compound
            } else {
                output_vote += 1;
            }
        } else if h_edge.target.port == port_id {
            // Incoming.
            if h_edge.source.graph_parent == Some(compound) {
                output_vote += 1; // source is a child of compound
            } else {
                input_vote += 1;
            }
        }
    }

    output_vote - input_vote
}

/// Returns true if any edge incident to `compound_port` connects to a node
/// outside `compound`'s subtree. Local non-self-loop edges (in `parent_graph`)
/// always satisfy this — the other end is a sibling of `compound`, which is
/// not a descendant. Cross-hierarchy edges connect to nodes in some nested
/// LGraph; the check is whether that nested is a descendant of `compound`.
fn is_connected_to_external_nodes(
    parent_graph: &LGraph,
    compound_port: PortId,
    compound: NodeId,
    root_ptr: *mut LGraph,
) -> bool {
    // Local edges (excluding self-loops on `compound`): the other end is a
    // sibling of `compound`, hence non-descendant.
    for &edge_id in parent_graph.port(compound_port).outgoing_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let target_owner = parent_graph.port(edge.target).owner;
        if target_owner != compound {
            return true;
        }
    }
    for &edge_id in parent_graph.port(compound_port).incoming_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let source_owner = parent_graph.port(edge.source).owner;
        if source_owner != compound {
            return true;
        }
    }

    // Cross-hierarchy edges. The other end's graph_parent identifies the
    // immediate compound owning the deeper LGraph; that compound must be
    // `compound` or a descendant of `compound` for the connection to count
    // as internal.
    // SAFETY: see `calculate_net_flow_root_aware`.
    let root = unsafe { &*root_ptr };
    for h_edge in root.hierarchical_edges.iter() {
        let other_graph_parent = if h_edge.source.port == compound_port {
            h_edge.target.graph_parent
        } else if h_edge.target.port == compound_port {
            h_edge.source.graph_parent
        } else {
            continue;
        };
        let Some(other_parent) = other_graph_parent else {
            // Other end is on root — definitely not a descendant of compound.
            return true;
        };
        if other_parent == compound {
            continue; // child of compound — internal.
        }
        // Check if other_parent is a descendant of compound by walking up
        // through `parent_node` chains via the global registry.
        if !is_node_descendant_of(root, other_parent, compound) {
            return true;
        }
    }
    false
}

fn is_connected_to_child_nodes(
    parent_graph: &LGraph,
    compound_port: PortId,
    compound: NodeId,
    root_ptr: *mut LGraph,
) -> bool {
    let inside_loops_active =
        parent_graph.node(compound).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);

    for &edge_id in parent_graph.port(compound_port).outgoing_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let target_owner = parent_graph.port(edge.target).owner;
        if target_owner == compound
            && inside_loops_active
            && edge.properties.get(&INSIDE_SELF_LOOPS_YO)
        {
            return true;
        }
    }
    for &edge_id in parent_graph.port(compound_port).incoming_edges.iter() {
        let edge = parent_graph.edge(edge_id);
        let source_owner = parent_graph.port(edge.source).owner;
        if source_owner == compound
            && inside_loops_active
            && edge.properties.get(&INSIDE_SELF_LOOPS_YO)
        {
            return true;
        }
    }

    // SAFETY: see `calculate_net_flow_root_aware`.
    let root = unsafe { &*root_ptr };
    for h_edge in root.hierarchical_edges.iter() {
        let other_graph_parent = if h_edge.source.port == compound_port {
            h_edge.target.graph_parent
        } else if h_edge.target.port == compound_port {
            h_edge.source.graph_parent
        } else {
            continue;
        };
        let Some(other_parent) = other_graph_parent else {
            continue;
        };
        if other_parent == compound || is_node_descendant_of(root, other_parent, compound) {
            return true;
        }
    }

    false
}

/// True when `descendant` is an indirect / direct child of `ancestor` (or
/// equal). Walks up `descendant`'s containing-LGraph chain via
/// `parent_node` links until we hit `ancestor` or run out.
fn is_node_descendant_of(root: &LGraph, descendant: NodeId, ancestor: NodeId) -> bool {
    if descendant == ancestor {
        return true;
    }
    let mut current = descendant;
    loop {
        let Some(containing) = root.find_graph_containing(current) else {
            return false;
        };
        match containing.parent_node {
            Some(p) if p == ancestor => return true,
            Some(p) => current = p,
            None => return false,
        }
    }
}

/// Phase 2 of `install_external_ports_for_separate_hierarchy`: drain
/// `root.hierarchical_edges` and add corresponding local edges in the
/// appropriate nested LGraphs, using EP dummies created in phase 1.
///
/// For each cross-hierarchy edge whose endpoints span levels, identifies the
/// outer port (shallower side) and the inner port (deeper side), looks up
/// the EP dummy that represents the outer port in the inner LGraph (created
/// by `transform_external_port_in_nested`), and adds a local edge between the
/// dummy port and the inner port. If no dummy was created (because
/// `check_external_ports_for_compound` returned false for that compound), the
/// edge is dropped — flat-import would not pick it up either, since it was
/// queued by the converter without ever crossing back into flat-import scope.
fn convert_hierarchical_edges_to_local_for_separate(graph: &mut LGraph) {
    let pending: Vec<HierarchicalEdgeData> = graph.take_hierarchical_edges();
    for h_edge in pending {
        let source_owner = h_edge.source.graph_parent;
        let target_owner = h_edge.target.graph_parent;

        // Both ends in the same LGraph → promote to a local edge.
        if source_owner == target_owner {
            let target_graph = graph.resolve_hierarchical_port_graph_mut(h_edge.source);
            if let Some(g) = target_graph {
                let edge = g.add_edge(h_edge.source.port, h_edge.target.port);
                g.edge_mut(edge).order = h_edge.order;
                attach_hierarchical_edge_metadata(g, edge, &h_edge);
                sort_port_edges_by_order(g, h_edge.source.port, true);
                sort_port_edges_by_order(g, h_edge.target.port, false);
            }
            continue;
        }

        let src_level = hierarchy_level(graph, source_owner);
        let tgt_level = hierarchy_level(graph, target_owner);

        if src_level < tgt_level {
            // Source is shallower. Inner edge goes from EP_for(source) to target,
            // inside `target_parent`'s nested LGraph.
            let outer_port = h_edge.source.port;
            let target_parent = match target_owner {
                Some(t) => t,
                None => continue,
            };
            let parent_graph_id = target_parent.0.graph_id();
            let parent_graph_ptr =
                crate::graph::graph_by_id_mut(parent_graph_id).unwrap_or(graph as *mut LGraph);
            // SAFETY: registry pointer is valid until LGraph::Drop runs;
            // no other live mut borrow exists on this LGraph here.
            let parent_graph = unsafe { &mut *parent_graph_ptr };
            let nested = parent_graph.nested_mut(target_parent).unwrap();
            if let Some(dp) = find_ep_dummy_port_in_nested(nested, outer_port) {
                let edge = nested.add_edge(dp, h_edge.target.port);
                nested.edge_mut(edge).order = h_edge.order;
                attach_hierarchical_edge_metadata(nested, edge, &h_edge);
                sort_port_edges_by_order(nested, dp, true);
                sort_port_edges_by_order(nested, h_edge.target.port, false);
            }
        } else {
            let outer_port = h_edge.target.port;
            let source_parent = match source_owner {
                Some(s) => s,
                None => continue,
            };
            let parent_graph_id = source_parent.0.graph_id();
            let parent_graph_ptr =
                crate::graph::graph_by_id_mut(parent_graph_id).unwrap_or(graph as *mut LGraph);
            // SAFETY: same as above.
            let parent_graph = unsafe { &mut *parent_graph_ptr };
            let nested = parent_graph.nested_mut(source_parent).unwrap();
            if let Some(dp) = find_ep_dummy_port_in_nested(nested, outer_port) {
                let edge = nested.add_edge(h_edge.source.port, dp);
                nested.edge_mut(edge).order = h_edge.order;
                attach_hierarchical_edge_metadata(nested, edge, &h_edge);
                sort_port_edges_by_order(nested, h_edge.source.port, true);
                sort_port_edges_by_order(nested, dp, false);
            }
        }
    }
}

fn attach_hierarchical_edge_metadata(
    graph: &mut LGraph,
    edge: EdgeId,
    h_edge: &HierarchicalEdgeData,
) {
    graph.edge_mut(edge).properties = h_edge.properties.clone();
    if h_edge.labels.is_empty() {
        return;
    }

    let mut graph_properties = graph.properties.get(&GRAPH_PROPERTIES);
    for label in &h_edge.labels {
        let label_id = graph.add_edge_label(edge, label.text.clone(), label.size);
        graph.label_mut(label_id).position = label.position;
        graph.label_mut(label_id).properties = label.properties.clone();
        match graph.label(label_id).properties.get(&EDGE_LABEL_PLACEMENT) {
            EdgeLabelPlacement::Center => {
                graph_properties.insert(GraphProperties::CENTER_LABELS);
            }
            EdgeLabelPlacement::Head | EdgeLabelPlacement::Tail => {
                graph_properties.insert(GraphProperties::END_LABELS);
            }
            EdgeLabelPlacement::Undefined => {}
        }
    }
    graph.properties.set(&GRAPH_PROPERTIES, graph_properties);
}

/// Locate an EP dummy in `nested` whose `ORIGIN_PORT` matches `outer_port`,
/// returning its sole port (the inner-side connection point). Returns `None`
/// when no such dummy exists in this LGraph.
pub(crate) fn find_ep_dummy_port_in_nested(nested: &LGraph, outer_port: PortId) -> Option<PortId> {
    for &nid in nested.layerless_nodes.iter() {
        let n = nested.node(nid);
        if n.node_type == NodeType::ExternalPort
            && n.properties.get(&ORIGIN_PORT) == Some(outer_port)
        {
            return n.ports.first().copied();
        }
    }
    None
}

/// Drain `LGraph::hierarchical_edges` and materialise each entry as a local
/// edge chain through every crossed hierarchy boundary. Cross-hierarchy edges
/// are staged in `LGraph::hierarchical_edges` until preprocessing time.
fn convert_hierarchical_edges_to_local(graph: &mut LGraph, state: &mut PreprocessorState) {
    let pending: Vec<HierarchicalEdgeData> = graph.take_hierarchical_edges();
    for h_edge in pending {
        let source_owner = h_edge.source.graph_parent;
        let target_owner = h_edge.target.graph_parent;

        // Degenerate case: both ports live in the same graph; promote to a local edge.
        if source_owner == target_owner {
            let target_graph = graph.resolve_hierarchical_port_graph_mut(h_edge.source);
            if let Some(g) = target_graph {
                let edge = g.add_edge(h_edge.source.port, h_edge.target.port);
                g.edge_mut(edge).order = h_edge.order;
                sort_port_edges_by_order(g, h_edge.source.port, true);
                sort_port_edges_by_order(g, h_edge.target.port, false);
            }
            continue;
        }

        convert_multi_level_hierarchical_edge(graph, h_edge, state);
    }
}

fn convert_multi_level_hierarchical_edge(
    graph: &mut LGraph,
    h_edge: HierarchicalEdgeData,
    state: &mut PreprocessorState,
) {
    let source_path = hierarchy_path_from_root(graph, h_edge.source.graph_parent);
    let target_path = hierarchy_path_from_root(graph, h_edge.target.graph_parent);
    let mut common = 0usize;
    while common < source_path.len()
        && common < target_path.len()
        && source_path[common] == target_path[common]
    {
        common += 1;
    }

    let original_proxy = graph.add_edge_orphan(h_edge.source.port, h_edge.target.port);
    graph.edge_mut(original_proxy).order = h_edge.order;
    attach_hierarchical_edge_metadata(graph, original_proxy, &h_edge);
    let mut current_port = h_edge.source.port;
    let mut target_consumed_at_parent_port = false;

    for &compound in source_path[common..].iter().rev() {
        let Some(parent_graph_ptr) = crate::graph::graph_by_id_mut(compound.0.graph_id()) else {
            return;
        };
        // SAFETY: graph registry pointers remain valid for the layout call; every
        // borrow below is scoped before the next registry lookup.
        let parent_graph = unsafe { &mut *parent_graph_ptr };

        if port_is_on_node(parent_graph, h_edge.target.port, compound) {
            let dummy_port = ensure_parent_port_dummy(
                parent_graph,
                compound,
                h_edge.target.port,
                PortType::Output,
                state,
            );
            let local_edge = {
                let nested = parent_graph.nested_mut(compound).expect("compound owns nested graph");
                create_synthetic_segment_edge(
                    nested,
                    current_port,
                    dummy_port,
                    h_edge.order,
                    PortType::Output,
                    state,
                )
            };
            state
                .cross_hierarchy_map
                .entry(original_proxy)
                .or_default()
                .push(CrossHierarchyEdge::new(local_edge, Some(compound), PortType::Output));
            current_port = h_edge.target.port;
            target_consumed_at_parent_port = true;
            break;
        }

        let (parent_port, dummy_port) = create_synthetic_boundary_port(
            parent_graph,
            compound,
            PortType::Output,
            current_port,
            state,
        );
        let local_edge = {
            let nested = parent_graph.nested_mut(compound).expect("compound owns nested graph");
            create_synthetic_segment_edge(
                nested,
                current_port,
                dummy_port,
                h_edge.order,
                PortType::Output,
                state,
            )
        };
        state
            .cross_hierarchy_map
            .entry(original_proxy)
            .or_default()
            .push(CrossHierarchyEdge::new(local_edge, Some(compound), PortType::Output));
        current_port = parent_port;
    }

    for &compound in &target_path[common..] {
        let Some(parent_graph_ptr) = crate::graph::graph_by_id_mut(compound.0.graph_id()) else {
            return;
        };
        // SAFETY: see the output-side loop above.
        let parent_graph = unsafe { &mut *parent_graph_ptr };

        if port_is_on_node(parent_graph, current_port, compound) {
            current_port = ensure_parent_port_dummy(
                parent_graph,
                compound,
                current_port,
                PortType::Input,
                state,
            );
            continue;
        }

        let (parent_port, dummy_port) = create_synthetic_boundary_port(
            parent_graph,
            compound,
            PortType::Input,
            h_edge.target.port,
            state,
        );
        let local_edge = create_synthetic_segment_edge(
            parent_graph,
            current_port,
            parent_port,
            h_edge.order,
            PortType::Input,
            state,
        );
        state
            .cross_hierarchy_map
            .entry(original_proxy)
            .or_default()
            .push(CrossHierarchyEdge::new(local_edge, parent_graph.parent_node, PortType::Input));
        current_port = dummy_port;
    }

    if target_consumed_at_parent_port {
        return;
    }

    let segment_parent = target_path
        .last()
        .copied()
        .or_else(|| if common == 0 { None } else { target_path.get(common - 1).copied() });
    let final_port_type =
        if target_path.len() > common { PortType::Input } else { PortType::Output };
    let target_graph_ptr: *mut LGraph = match segment_parent {
        Some(parent) => {
            let Some(parent_graph_ptr) = crate::graph::graph_by_id_mut(parent.0.graph_id()) else {
                return;
            };
            // SAFETY: registry pointer is valid for the layout call.
            let parent_graph = unsafe { &mut *parent_graph_ptr };
            match parent_graph.nested_mut(parent) {
                Some(nested) => nested as *mut LGraph,
                None => return,
            }
        }
        None => graph as *mut LGraph,
    };
    // SAFETY: target graph pointer is rooted in `graph`'s nested tree.
    let target_graph = unsafe { &mut *target_graph_ptr };
    let final_edge = create_synthetic_segment_edge(
        target_graph,
        current_port,
        h_edge.target.port,
        h_edge.order,
        final_port_type,
        state,
    );
    state
        .cross_hierarchy_map
        .entry(original_proxy)
        .or_default()
        .push(CrossHierarchyEdge::new(final_edge, segment_parent, final_port_type));
}

fn port_is_on_node(graph: &LGraph, port: PortId, node: NodeId) -> bool {
    graph.try_port(port).is_some_and(|p| p.owner == node)
}

fn ensure_parent_port_dummy(
    parent_graph: &mut LGraph,
    compound: NodeId,
    parent_port: PortId,
    port_type: PortType,
    state: &mut PreprocessorState,
) -> PortId {
    if let Some(dummy) = state.dummy_node_map.get(&parent_port).copied() {
        let nested = parent_graph.nested(compound).expect("compound owns nested graph");
        return *nested.node(dummy).ports.first().expect("dummy has port");
    }

    let port_constraints = parent_graph.node(compound).port_constraints();
    let net_flow = match port_type {
        PortType::Input => -1,
        PortType::Output => 1,
        PortType::Undefined => 0,
    };
    let dummy = create_external_port_dummy_in_nested(
        parent_graph,
        compound,
        parent_port,
        port_constraints,
        net_flow,
    );
    state.dummy_node_map.insert(parent_port, dummy);
    let nested = parent_graph.nested(compound).expect("compound owns nested graph");
    *nested.node(dummy).ports.first().expect("dummy has port")
}

fn hierarchy_path_from_root(root: &LGraph, graph_parent: Option<NodeId>) -> Vec<NodeId> {
    let Some(mut current) = graph_parent else {
        return Vec::new();
    };
    let mut path = Vec::new();
    loop {
        path.push(current);
        let Some(containing) = root.find_graph_containing(current) else {
            break;
        };
        let Some(parent) = containing.parent_node else {
            break;
        };
        current = parent;
    }
    path.reverse();
    path
}

fn create_synthetic_boundary_port(
    parent_graph: &mut LGraph,
    compound: NodeId,
    port_type: PortType,
    merge_key_port: PortId,
    state: &mut PreprocessorState,
) -> (PortId, PortId) {
    let merge = port_type == PortType::Input
        && parent_graph
            .nested(compound)
            .map(|nested| nested.properties.get(&MERGE_HIERARCHY_EDGES))
            .unwrap_or_else(|| parent_graph.properties.get(&MERGE_HIERARCHY_EDGES));
    let key = (compound, port_type, merge_key_port);
    if merge && let Some(ports) = state.synthetic_boundary_ports.get(&key).copied() {
        return ports;
    }

    let layout_direction = parent_graph.options.direction;
    let parent_port_side = match port_type {
        PortType::Input => PortSide::from_direction(layout_direction).opposed(),
        PortType::Output => PortSide::from_direction(layout_direction),
        PortType::Undefined => PortSide::Undefined,
    };
    let parent_port = parent_graph.add_port(compound, parent_port_side);
    let border_offset = parent_graph.options.spacing.edge_edge / 2.0;
    parent_graph
        .port_mut(parent_port)
        .properties
        .set(&PORT_BORDER_OFFSET, border_offset);
    let net_flow = if port_type == PortType::Input { -1 } else { 1 };
    let dummy_node = create_external_port_dummy_in_nested(
        parent_graph,
        compound,
        parent_port,
        PortConstraints::Free,
        net_flow,
    );
    state.dummy_node_map.insert(parent_port, dummy_node);
    let dummy_port = {
        let nested = parent_graph.nested(compound).expect("compound owns nested graph");
        *nested.node(dummy_node).ports.first().expect("dummy has port")
    };
    let ports = (parent_port, dummy_port);
    if merge {
        state.synthetic_boundary_ports.insert(key, ports);
    }
    ports
}

fn create_synthetic_segment_edge(
    graph: &mut LGraph,
    source: PortId,
    target: PortId,
    order: i32,
    port_type: PortType,
    state: &mut PreprocessorState,
) -> EdgeId {
    let merge = port_type == PortType::Input && graph.properties.get(&MERGE_HIERARCHY_EDGES);
    let key = (source, target, port_type);
    if merge && let Some(edge) = state.synthetic_segment_edges.get(&key).copied() {
        return edge;
    }

    let edge = graph.add_edge(source, target);
    graph.edge_mut(edge).order = order;
    sort_port_edges_by_order(graph, source, true);
    sort_port_edges_by_order(graph, target, false);
    if merge {
        state.synthetic_segment_edges.insert(key, edge);
    }
    edge
}

/// Postprocess a compound graph: reassemble original cross-hierarchy edges from the
/// segments produced by `preprocess`.
///
/// For each original edge, sort its segments source->target, walk them to reconstruct
/// a bend-point chain in a common reference coordinate system, copy labels back, then
/// restore the original edge's source/target ports so downstream consumers see the
/// edge re-attached to the original endpoints.
pub fn postprocess(graph: &mut LGraph) {
    // Take ownership of the map by cloning via get() and resetting the stored value
    // to an empty map. Subsequent edits on `graph` won't alias the local copy.
    let map: HashMap<EdgeId, SmallVec<CrossHierarchyEdge, 4>> =
        graph.properties.get(&CROSS_HIERARCHY_MAP);
    if map.is_empty() {
        return;
    }
    graph.properties.set(&CROSS_HIERARCHY_MAP, HashMap::new());
    let add_unnecessary_bendpoints =
        graph.properties.get(&crate::properties::internal::UNNECESSARY_BENDPOINTS);

    // Collect dummy edges to remove after the reassembly pass.
    let mut dummy_edges_to_unlink: Vec<(Option<NodeId>, EdgeId)> = Vec::new();

    for (orig_edge, segments_sv) in map.iter() {
        // Sort from source to target.
        let mut segments: SmallVec<CrossHierarchyEdge, 4> = segments_sv.iter().copied().collect();
        segments.sort_by(|a, b| compare_cross_hierarchy_edges(graph, a, b));

        // Figure out original source/target ports from first/last segments.
        let Some(first) = segments.first() else {
            continue;
        };
        let Some(last) = segments.last() else {
            continue;
        };
        let Some(source_port) = first.actual_source(graph) else {
            continue;
        };
        let Some(target_port) = last.actual_target(graph) else {
            continue;
        };

        // Determine the reference graph: if target is descendant of source node, the
        // reference graph is the source node's nested graph; otherwise it is the graph
        // containing the source node.
        // `actual_source` / `actual_target` may return ports from a deep nested
        // arena (the orig_edge proxy was added via `add_edge_orphan` with
        // cross-arena PortIds). Resolve through the global graph_id registry
        // rather than calling `graph.port` directly, which would panic.
        let source_node = match resolve_port_owner(graph, source_port) {
            Some(n) => n,
            None => continue,
        };
        let target_node = match resolve_port_owner(graph, target_port) {
            Some(n) => n,
            None => continue,
        };
        // `graph.nested(source_node)` panics if source_node lives in a
        // nested arena. Look it up via the registry instead.
        let source_node_owner_ptr =
            crate::graph::graph_by_id(source_node.0.graph_id()).unwrap_or(graph as *const LGraph);
        // SAFETY: registry pointer valid until LGraph::Drop.
        let source_node_owner = unsafe { &*source_node_owner_ptr };
        let reference_graph_ptr: *const LGraph = if graph.is_descendant(target_node, source_node) {
            match source_node_owner.nested(source_node) {
                Some(g) => g as *const LGraph,
                None => match graph.find_graph_containing(source_node) {
                    Some(g) => g as *const LGraph,
                    None => graph as *const LGraph,
                },
            }
        } else {
            match graph.find_graph_containing(source_node) {
                Some(g) => g as *const LGraph,
                None => graph as *const LGraph,
            }
        };

        // Junction point clearing — gated on whether any segment has JP's.
        let any_has_jp = segments.iter().any(|seg| {
            seg.segment_graph(graph)
                .map(|g| {
                    !g.edge(seg.new_edge)
                        .properties
                        .get(&crate::properties::internal::JUNCTION_POINTS)
                        .is_empty()
                })
                .unwrap_or(false)
        });

        // Reset original edge state before reassembly.
        graph.edge_mut(*orig_edge).bend_points.clear();
        graph
            .edge_mut(*orig_edge)
            .properties
            .set(&crate::properties::internal::JUNCTION_POINTS, SmallVec::new());

        // Walk segments and stitch bend points in reference-graph coordinates.
        let mut last_point: Option<Vec2> = None;
        let mut accumulated_junction_points: SmallVec<Vec2, 4> = SmallVec::new();
        let mut new_bend_points: SmallVec<Vec2, 4> = SmallVec::new();

        for seg in &segments {
            let Some(segment_graph_ptr) = seg.segment_graph(graph).map(|g| g as *const LGraph)
            else {
                continue;
            };
            let mut offset = Vec2::ZERO;
            LGraph::change_coord_system(graph, &mut offset, segment_graph_ptr, reference_graph_ptr);

            // SAFETY: segment_graph_ptr is alive for the lifetime of `graph`.
            let seg_graph = unsafe { &*segment_graph_ptr };
            let seg_bends_snapshot: SmallVec<Vec2, 4> =
                seg_graph.edge(seg.new_edge).bend_points.iter().copied().collect();
            let mut seg_bends: SmallVec<Vec2, 4> = seg_bends_snapshot
                .iter()
                .map(|p| Vec2 { x: p.x + offset.x, y: p.y + offset.y })
                .collect();

            // Compute absolute anchors of segment endpoints (in reference coords).
            let src_port = seg_graph.edge(seg.new_edge).source;
            let tgt_port = seg_graph.edge(seg.new_edge).target;
            let mut src_anchor = seg_graph.absolute_anchor(src_port);
            let mut tgt_anchor = seg_graph.absolute_anchor(tgt_port);
            src_anchor.x += offset.x;
            src_anchor.y += offset.y;
            tgt_anchor.x += offset.x;
            tgt_anchor.y += offset.y;

            if let Some(last) = last_point {
                let next = seg_bends.first().copied().unwrap_or(tgt_anchor);
                let x_diff = (last.x - next.x).abs() > TOLERANCE;
                let y_diff = (last.y - next.y).abs() > TOLERANCE;
                let add =
                    if add_unnecessary_bendpoints { x_diff || y_diff } else { x_diff && y_diff };
                if add {
                    new_bend_points.push(src_anchor);
                }
            }

            last_point = seg_bends.last().copied().or(Some(src_anchor));
            new_bend_points.append(&mut seg_bends);

            // Accumulate junction points from the segment's dummy edge.
            let jps: SmallVec<Vec2, 4> = seg_graph
                .edge(seg.new_edge)
                .properties
                .get(&crate::properties::internal::JUNCTION_POINTS);
            for jp in jps {
                accumulated_junction_points.push(Vec2 { x: jp.x + offset.x, y: jp.y + offset.y });
            }

            // Copy labels back: any label on this segment whose ORIGINAL_LABEL_EDGE
            // points at `orig_edge` is moved back to the original.
            copy_labels_back(
                graph,
                seg.new_edge,
                seg.graph_parent_node,
                *orig_edge,
                reference_graph_ptr,
            );

            dummy_edges_to_unlink.push((seg.graph_parent_node, seg.new_edge));
        }

        // Cross-hierarchy bend points are computed in a reference graph here
        // and then translated to the root-owned original proxy edge's
        // coordinate system before persisting, so downstream consumers
        // looking at the post-layout `LGraph` directly see consistent
        // root-frame coordinates.
        let root_graph_ptr = graph as *const LGraph;
        if !std::ptr::eq(reference_graph_ptr, root_graph_ptr) {
            for point in new_bend_points.iter_mut() {
                LGraph::change_coord_system(graph, point, reference_graph_ptr, root_graph_ptr);
            }
            for point in accumulated_junction_points.iter_mut() {
                LGraph::change_coord_system(graph, point, reference_graph_ptr, root_graph_ptr);
            }
        }

        // Persist reassembled bend points + junction points on the original edge.
        graph.edge_mut(*orig_edge).bend_points = new_bend_points.into_iter().collect();
        graph
            .edge_mut(*orig_edge)
            .properties
            .set(&crate::properties::internal::JUNCTION_POINTS, accumulated_junction_points);
        // Drop the JP property entirely when no segment had any.
        if !any_has_jp {
            graph
                .edge_mut(*orig_edge)
                .properties
                .set(&crate::properties::internal::JUNCTION_POINTS, SmallVec::new());
        }

        // Re-attach the original edge to its original ports.
        relink_original_edge(graph, *orig_edge, source_port, target_port);
    }

    // Unlink dummy edges — their source/target lists are cleared so downstream phases
    // ignore them. Actual arena removal is out of scope (segments may be shared).
    for (graph_parent, dummy_edge) in dummy_edges_to_unlink {
        let target_graph_ptr: *mut LGraph = match graph_parent {
            None => graph as *mut LGraph,
            Some(p) => {
                // `p` may live in a deep arena (convert routed dummies into
                // nested LGraphs via the registry). Look up the parent
                // graph by `p`'s graph_id, then call `nested_mut(p)` on it.
                let parent_graph_id = p.0.graph_id();
                let parent_graph_ptr = match crate::graph::graph_by_id_mut(parent_graph_id) {
                    Some(ptr) => ptr,
                    None => continue,
                };
                // SAFETY: registry pointer valid until LGraph::Drop.
                let parent_graph = unsafe { &mut *parent_graph_ptr };
                match parent_graph.nested_mut(p) {
                    Some(g) => g as *mut LGraph,
                    None => continue,
                }
            }
        };
        // SAFETY: pointer alive for lifetime of `graph`; no aliasing borrow held.
        let g = unsafe { &mut *target_graph_ptr };
        unlink_edge_local(g, dummy_edge);
    }
}

const TOLERANCE: f64 = 1e-3;

/// Re-attach an original edge to its endpoints after reassembly.
fn relink_original_edge(graph: &mut LGraph, edge: EdgeId, source: PortId, target: PortId) {
    // Ensure the edge's own record points at the right ports.
    let source_owner = graph.port_owner(source);
    let target_owner = graph.port_owner(target);
    let edge_data = graph.edge_mut(edge);
    edge_data.source = source;
    edge_data.target = target;
    edge_data.source_owner = source_owner;
    edge_data.target_owner = target_owner;
    // Append to the ports' adjacency lists if not already present. The
    // source/target may live in nested arenas (cross-hierarchy edges where
    // `actual_source`/`actual_target` returned ports from a deeper LGraph),
    // in which case the proxy edge stays "orphan" — it is never iterated
    // through this graph's port adjacency list, only via
    // `cross_hierarchy_map`. Skip the cross-arena case rather than panic.
    if let Some(p) = graph.try_port(source)
        && !p.outgoing_edges.contains(&edge)
    {
        graph.port_mut(source).outgoing_edges.push(edge);
    }
    if let Some(p) = graph.try_port(target)
        && !p.incoming_edges.contains(&edge)
    {
        graph.port_mut(target).incoming_edges.push(edge);
    }
}

fn unlink_edge_local(graph: &mut LGraph, edge: EdgeId) {
    let src = graph.edge(edge).source;
    let tgt = graph.edge(edge).target;
    graph.port_mut(src).outgoing_edges.retain(|e| *e != edge);
    graph.port_mut(tgt).incoming_edges.retain(|e| *e != edge);
}

fn copy_labels_back(
    graph: &mut LGraph,
    segment_edge: EdgeId,
    segment_parent: Option<NodeId>,
    orig_edge: EdgeId,
    reference_graph: *const LGraph,
) {
    // Resolve the segment's graph. `segment_parent` may live in a deep
    // arena (for cross-hierarchy edges where convert_hierarchical_edges_to_local
    // installed dummies inside a deeper LGraph), so we route the
    // `nested_mut(p)` lookup through the registry rather than assuming
    // `graph` directly contains `p`.
    let seg_graph_ptr: *mut LGraph = match segment_parent {
        None => graph as *mut LGraph,
        Some(p) => {
            let parent_graph_id = p.0.graph_id();
            let parent_graph_ptr = match crate::graph::graph_by_id_mut(parent_graph_id) {
                Some(ptr) => ptr,
                None => return,
            };
            // SAFETY: registry guarantees pointer validity until LGraph::Drop.
            let parent_graph = unsafe { &mut *parent_graph_ptr };
            match parent_graph.nested_mut(p) {
                Some(g) => g as *mut LGraph,
                None => return,
            }
        }
    };

    if std::ptr::eq(seg_graph_ptr as *const LGraph, graph as *const LGraph) {
        let labels: SmallVec<LabelId, 2> =
            graph.edge(segment_edge).labels.iter().copied().collect();
        for lbl in labels {
            let back_ref: Option<EdgeId> = graph.label(lbl).properties.get(&ORIGINAL_LABEL_EDGE);
            if back_ref != Some(orig_edge) {
                continue;
            }
            let mut position = graph.label(lbl).position;
            LGraph::change_coord_system(
                graph,
                &mut position,
                graph as *const LGraph,
                reference_graph,
            );
            graph.label_mut(lbl).position = position;
            graph.edge_mut(segment_edge).labels.retain(|l| *l != lbl);
            graph.edge_mut(orig_edge).labels.push(lbl);
        }
        return;
    }

    struct LabelCopy {
        text: String,
        size: Vec2,
        position: Vec2,
        properties: PropertyMap,
    }

    let moved_labels: Vec<LabelCopy> = {
        // SAFETY: pointer valid for `graph`'s lifetime; this scoped borrow ends
        // before labels are inserted into the root graph below.
        let seg_graph = unsafe { &mut *seg_graph_ptr };
        let labels: SmallVec<LabelId, 2> =
            seg_graph.edge(segment_edge).labels.iter().copied().collect();
        let mut moved = Vec::new();
        for lbl in labels {
            let source_label = seg_graph.label(lbl);
            let back_ref: Option<EdgeId> = source_label.properties.get(&ORIGINAL_LABEL_EDGE);
            if back_ref != Some(orig_edge) {
                continue;
            }
            let mut position = source_label.position;
            LGraph::change_coord_system(
                graph,
                &mut position,
                seg_graph as *const LGraph,
                reference_graph,
            );
            moved.push(LabelCopy {
                text: source_label.text.clone(),
                size: source_label.size,
                position,
                properties: source_label.properties.clone(),
            });
            seg_graph.edge_mut(segment_edge).labels.retain(|l| *l != lbl);
        }
        moved
    };

    for label in moved_labels {
        let new_label = graph.add_edge_label(orig_edge, label.text, label.size);
        graph.label_mut(new_label).position = label.position;
        graph.label_mut(new_label).properties = label.properties;
    }
}

/// Mutable state threaded through every preprocessor helper.
///
/// Lives in a dedicated struct passed by `&mut` so the helper functions stay
/// free-standing.
#[derive(Default)]
struct PreprocessorState {
    /// Accumulates CrossHierarchyEdge segments keyed by original EdgeId.
    cross_hierarchy_map: HashMap<EdgeId, SmallVec<CrossHierarchyEdge, 4>>,
    /// Maps outside ports to their external-port dummy nodes, deduplicating dummy
    /// creation for ports shared across multiple edges.
    dummy_node_map: HashMap<PortId, NodeId>,
    /// Synthetic boundary ports created while materialising importer-provided
    /// hierarchical edges. Reused when hierarchy-edge merging is enabled so
    /// shared segments do not widen nested graphs.
    synthetic_boundary_ports: HashMap<(NodeId, PortType, PortId), (PortId, PortId)>,
    /// Dummy edge segments shared by merged hierarchy edges.
    synthetic_segment_edges: HashMap<(PortId, PortId, PortType), EdgeId>,
}

/// Split cross-hierarchy edges throughout the nested hierarchy.
///
/// `parent_node` is the node whose `nested_graph` is `graph`, or `None` if `graph` is
/// the root. Returns the list of external ports exposed by `graph` to its parent.
fn transform_hierarchy_edges(
    graph: &mut LGraph,
    parent_node: Option<NodeId>,
    state: &mut PreprocessorState,
) -> Vec<ExternalPort> {
    let root_ptr = std::ptr::NonNull::from(&mut *graph);
    let mut frames: Vec<(std::ptr::NonNull<LGraph>, Option<NodeId>)> = Vec::new();
    let mut stack = vec![(root_ptr, parent_node)];

    while let Some((graph_ptr, parent_node)) = stack.pop() {
        frames.push((graph_ptr, parent_node));
        // SAFETY: each pointer is a unique nested graph box.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let child_ids: Vec<NodeId> = graph.layerless_nodes.clone();
        for node_id in child_ids.into_iter().rev() {
            let nested_has_pending =
                graph.nested(node_id).map(|g| !g.hierarchical_edges.is_empty()).unwrap_or(false);
            if nested_has_pending && let Some(nested) = graph.nested_mut(node_id) {
                stack.push((std::ptr::NonNull::from(nested), Some(node_id)));
            }
        }
    }

    let mut exported_by_graph: HashMap<*mut LGraph, Vec<ExternalPort>> = HashMap::new();
    for (graph_ptr, parent_node) in frames.into_iter().rev() {
        // SAFETY: children are processed before parents, and each graph pointer
        // is unique within the nested ownership tree.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let mut contained_external_ports: Vec<ExternalPort> = Vec::new();
        let child_ids: Vec<NodeId> = graph.layerless_nodes.clone();
        for node_id in child_ids {
            if !graph.has_nested(node_id) {
                continue;
            }
            if let Some(nested) = graph.nested_mut(node_id) {
                let nested_ptr = nested as *mut LGraph;
                if let Some(child_ports) = exported_by_graph.remove(&nested_ptr) {
                    contained_external_ports.extend(child_ports);
                }
            }

            // Inside self-loops that opted in to `INSIDE_SELF_LOOPS_YO`.
            process_inside_self_loops(graph, node_id, state);

            // Materialise dummy nodes for declared ports of the compound node when the
            // child graph exports external ports.
            let child_has_external = graph
                .nested(node_id)
                .map(|g| {
                    g.properties.get(&GRAPH_PROPERTIES).contains(GraphProperties::EXTERNAL_PORTS)
                })
                .unwrap_or(false);
            if child_has_external {
                ensure_port_dummies_for_compound_node(graph, node_id, state);
            }
        }

        // Inner segments: edges emanating from the contained external ports connect
        // to direct children, siblings, or the outside world.
        let mut exported: Vec<ExternalPort> = Vec::new();
        process_inner_hierarchical_edge_segments(
            graph,
            parent_node,
            &contained_external_ports,
            &mut exported,
            state,
        );

        // Outer segments: edges emanating from the graph's own child nodes that leave
        // the parent's boundary. Only when the current graph is nested under a parent.
        if let Some(pnode) = parent_node {
            process_outer_hierarchical_edge_segments(graph, pnode, &mut exported, state);
        }
        exported_by_graph.insert(graph_ptr.as_ptr(), exported);
    }

    exported_by_graph.remove(&root_ptr.as_ptr()).unwrap_or_default()
}

/// Ensures that every declared port of a compound node has an associated external-port
/// dummy inside its nested graph, and transfers port labels so the nested graph can
/// reserve space for them.
fn ensure_port_dummies_for_compound_node(
    graph: &mut LGraph,
    node_id: NodeId,
    state: &mut PreprocessorState,
) {
    let port_constraints = graph.node(node_id).port_constraints();
    let port_label_placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    let inside_port_labels = port_label_placement.contains(PortLabelPlacement::INSIDE);
    let port_ids: SmallVec<PortId, 6> = graph.node(node_id).ports.iter().copied().collect();
    for port_id in port_ids {
        // `calculate_net_flow` returns `outgoing - incoming` directly; passing
        // it as-is gets the dummy on the correct side for compound self-loop
        // ports.
        let net_flow = calculate_net_flow(graph, port_id);
        // Idempotency: convert_hierarchical_edges_to_local may have already
        // created a dummy for this port (and stashed it in
        // state.dummy_node_map). Also scan the nested LGraph for an existing
        // ExternalPort dummy whose ORIGIN_PORT matches, needed when the
        // dummy was created during a different preprocess pass that bypassed
        // dummy_node_map.
        let dummy_node = if let Some(existing) = state.dummy_node_map.get(&port_id).copied() {
            existing
        } else {
            let already = if let Some(nested) = graph.nested(node_id) {
                nested.layerless_nodes.iter().copied().find(|&nid| {
                    let n = nested.node(nid);
                    n.node_type == NodeType::ExternalPort
                        && n.properties.get(&ORIGIN_PORT) == Some(port_id)
                })
            } else {
                None
            };
            if let Some(existing) = already {
                state.dummy_node_map.insert(port_id, existing);
                existing
            } else {
                let dummy = create_external_port_dummy_in_nested(
                    graph,
                    node_id,
                    port_id,
                    port_constraints,
                    net_flow,
                );
                state.dummy_node_map.insert(port_id, dummy);
                dummy
            }
        };
        // Transfer port labels onto dummy's first port so space is reserved.
        copy_port_labels_onto_dummy(
            graph,
            node_id,
            port_id,
            dummy_node,
            port_constraints,
            inside_port_labels,
            port_label_placement,
        );
    }
}

/// Reserves space for every declared port label of a parent compound after a
/// child graph has exposed external ports.
///
/// Cross-arena hierarchical-edge conversion may create the dummies earlier, so
/// this dedicated post-pass walks every nested graph that now contains
/// external-port dummies and applies the same label-reservation contract.
fn ensure_declared_port_dummies_for_external_nested(
    graph: &mut LGraph,
    state: &mut PreprocessorState,
) {
    let mut graphs = Vec::new();
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        graphs.push(graph_ptr);
        // SAFETY: each pointer is a unique nested graph box.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let children: Vec<std::ptr::NonNull<LGraph>> = graph
            .nested_graphs_mut()
            .map(|(_, nested)| std::ptr::NonNull::from(nested))
            .collect();
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }

    for graph_ptr in graphs.into_iter().rev() {
        // SAFETY: children are processed before parents.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let node_ids: SmallVec<NodeId, 16> = graph
            .nodes_iter()
            .filter_map(|(id, node)| node.nested_graph.is_some().then_some(id))
            .collect();

        for node_id in node_ids {
            let child_has_external = graph
                .nested(node_id)
                .map(|nested| {
                    nested
                        .properties
                        .get(&GRAPH_PROPERTIES)
                        .contains(GraphProperties::EXTERNAL_PORTS)
                        || nested.nodes_iter().any(|(_, n)| n.node_type == NodeType::ExternalPort)
                })
                .unwrap_or(false);
            if child_has_external {
                ensure_port_dummies_for_compound_node(graph, node_id, state);
            }
        }
    }
}

fn copy_port_labels_onto_dummy(
    graph: &mut LGraph,
    compound: NodeId,
    port_id: PortId,
    dummy_node: NodeId,
    port_constraints: PortConstraints,
    inside_port_labels: bool,
    port_label_placement: PortLabelPlacement,
) {
    // The nested graph holding the dummy lives under this compound via `nested_mut`.
    let port_label_ids: SmallVec<LabelId, 2> = graph.port(port_id).labels.iter().copied().collect();
    if port_label_ids.is_empty() {
        return;
    }
    let port_side = graph.port(port_id).side;
    let port_size = graph.port(port_id).size;
    // Snapshot label sizes from the parent port. For non-inside labels,
    // collapse the dummy-label dimension that should not reserve inside
    // space; fixed labels keep only the part that overlaps the compound
    // interior.
    let label_sizes: Vec<Vec2> = port_label_ids
        .iter()
        .map(|lid| {
            let label = graph.label(*lid);
            let mut size = label.size;
            if !inside_port_labels {
                let inside_part = if port_label_placement.is_fixed() {
                    compute_label_inside_part(label.position, label.size, port_size, port_side)
                } else {
                    0.0
                };
                if port_constraints == PortConstraints::Free
                    || matches!(port_side, PortSide::East | PortSide::West)
                {
                    size.x = inside_part;
                } else {
                    size.y = inside_part;
                }
            }
            size
        })
        .collect();
    let nested = graph.nested_mut(compound).unwrap();
    let first_port = *nested.node(dummy_node).ports.first().expect("dummy has a port");
    if nested.port(first_port).labels.len() >= port_label_ids.len() {
        return;
    }
    for size in label_sizes {
        // Create a placeholder label of equal dimensions (text ignored — only
        // size matters for the layered algorithm).
        let _label_id = nested.add_port_label(first_port, String::new(), size);
    }
}

fn compute_label_inside_part(
    label_position: Vec2,
    label_size: Vec2,
    port_size: Vec2,
    port_side: PortSide,
) -> f64 {
    match port_side {
        PortSide::North => (label_size.y + label_position.y - port_size.y).max(0.0),
        PortSide::South => (-label_position.y).max(0.0),
        PortSide::East => (-label_position.x).max(0.0),
        PortSide::West => (label_size.x + label_position.x - port_size.x).max(0.0),
        PortSide::Undefined => 0.0,
    }
}

/// Creates external-port dummies for both endpoints of each self-loop that opted in
/// to inside-routing, and records the self-loop as an inner cross-hierarchy segment.
fn process_inside_self_loops(
    graph: &mut LGraph,
    compound_node: NodeId,
    state: &mut PreprocessorState,
) {
    if !graph.node(compound_node).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE) {
        return;
    }
    let port_ids: SmallVec<PortId, 6> = graph.node(compound_node).ports.iter().copied().collect();
    for src_port in port_ids {
        let out_edges: SmallVec<EdgeId, 2> =
            graph.port(src_port).outgoing_edges.iter().copied().collect();
        for edge_id in out_edges {
            let tgt_port = graph.edge(edge_id).target;
            let is_self_loop = graph.port(tgt_port).owner == compound_node;
            let inside_loop_opt_in = graph.edge(edge_id).properties.get(&INSIDE_SELF_LOOPS_YO);
            if !(is_self_loop && inside_loop_opt_in) {
                continue;
            }
            // Materialise dummies on both ends.
            let source_dummy =
                ensure_inside_loop_endpoint_dummy(graph, compound_node, src_port, state, -1);
            let target_dummy =
                ensure_inside_loop_endpoint_dummy(graph, compound_node, tgt_port, state, 1);

            // Snapshot the original edge properties and flags before we grab
            // the nested graph borrow, so we can faithfully copy them onto
            // the dummy edge — properties are cloned then JUNCTION_POINTS
            // is cleared.
            let orig_properties = graph.edge(edge_id).properties.clone();
            let orig_flags = graph.edge(edge_id).flags;

            let dummy_edge_id = {
                let nested = graph.nested_mut(compound_node).unwrap();
                let src_dummy_port =
                    *nested.node(source_dummy).ports.first().expect("dummy has port");
                let tgt_dummy_port =
                    *nested.node(target_dummy).ports.first().expect("dummy has port");
                let eid = nested.add_edge(src_dummy_port, tgt_dummy_port);
                nested.edge_mut(eid).properties = orig_properties;
                nested.edge_mut(eid).flags = orig_flags;
                nested
                    .edge_mut(eid)
                    .properties
                    .set(&crate::properties::internal::JUNCTION_POINTS, smallvec::SmallVec::new());
                eid
            };
            state
                .cross_hierarchy_map
                .entry(edge_id)
                .or_default()
                .push(CrossHierarchyEdge::new(
                    dummy_edge_id,
                    Some(compound_node),
                    PortType::Output,
                ));
            // Mark the nested graph as owning external ports.
            let nested = graph.nested_mut(compound_node).unwrap();
            let mut props = nested.properties.get(&GRAPH_PROPERTIES);
            props.insert(GraphProperties::EXTERNAL_PORTS);
            nested.properties.set(&GRAPH_PROPERTIES, props);
        }
    }
}

fn ensure_inside_loop_endpoint_dummy(
    graph: &mut LGraph,
    compound: NodeId,
    port: PortId,
    state: &mut PreprocessorState,
    net_flow: i32,
) -> NodeId {
    if let Some(existing) = state.dummy_node_map.get(&port).copied() {
        return existing;
    }
    let dummy = create_external_port_dummy_in_nested(
        graph,
        compound,
        port,
        PortConstraints::Free,
        net_flow,
    );
    state.dummy_node_map.insert(port, dummy);
    dummy
}

/// Processes inner hierarchical edge segments.
fn process_inner_hierarchical_edge_segments(
    graph: &mut LGraph,
    parent_node: Option<NodeId>,
    contained_external_ports: &[ExternalPort],
    exported: &mut Vec<ExternalPort>,
    state: &mut PreprocessorState,
) {
    let mut created: Vec<ExternalPort> = Vec::new();
    for external_port in contained_external_ports {
        let mut current: Option<ExternalPort> = None;
        match external_port.port_type {
            PortType::Output =>
                for out_edge in external_port.orig_edges.iter().copied() {
                    let target_node = graph.port(graph.edge(out_edge).target).owner;
                    let case = classify_inner_segment(graph, target_node, parent_node);
                    match case {
                        InnerSegmentCase::Child => {
                            connect_child(
                                graph,
                                external_port,
                                out_edge,
                                external_port.dummy_port,
                                graph.edge(out_edge).target,
                                state,
                            );
                        }
                        InnerSegmentCase::Sibling => {
                            connect_siblings(
                                graph,
                                external_port,
                                contained_external_ports,
                                out_edge,
                                state,
                            );
                        }
                        InnerSegmentCase::Outside => {
                            let new_port = introduce_hierarchical_edge_segment(
                                graph,
                                parent_node,
                                out_edge,
                                external_port.dummy_port,
                                PortType::Output,
                                current.as_ref(),
                                state,
                            );
                            if !same_external_port(&current, &new_port) {
                                created.push(new_port.clone());
                            }
                            if new_port.exported {
                                current = Some(new_port);
                            }
                        }
                    }
                },
            PortType::Input | PortType::Undefined => {
                for in_edge in external_port.orig_edges.iter().copied() {
                    let source_node = graph.port(graph.edge(in_edge).source).owner;
                    let case = classify_inner_segment(graph, source_node, parent_node);
                    match case {
                        InnerSegmentCase::Child => {
                            connect_child(
                                graph,
                                external_port,
                                in_edge,
                                graph.edge(in_edge).source,
                                external_port.dummy_port,
                                state,
                            );
                        }
                        InnerSegmentCase::Sibling => {
                            // Handled by the output-side pass above.
                            continue;
                        }
                        InnerSegmentCase::Outside => {
                            let new_port = introduce_hierarchical_edge_segment(
                                graph,
                                parent_node,
                                in_edge,
                                external_port.dummy_port,
                                PortType::Input,
                                current.as_ref(),
                                state,
                            );
                            if !same_external_port(&current, &new_port) {
                                created.push(new_port.clone());
                            }
                            if new_port.exported {
                                current = Some(new_port);
                            }
                        }
                    }
                }
            }
        }
    }
    // Commit created external ports to the graph's layerless list (deduplicated)
    // and export those marked `exported`.
    for ext in created {
        if !graph.layerless_nodes.contains(&ext.dummy_node) {
            graph.layerless_nodes.push(ext.dummy_node);
        }
        if ext.exported {
            exported.push(ext);
        }
    }
}

#[derive(PartialEq, Eq)]
enum InnerSegmentCase {
    Child,
    Sibling,
    Outside,
}

fn classify_inner_segment(
    graph: &LGraph,
    opposite_node: NodeId,
    parent_node: Option<NodeId>,
) -> InnerSegmentCase {
    // `opposite_node` is the endpoint that is NOT the external dummy. Case 1: it is a
    // direct child of `graph`. Case 2: it is a descendant of `parent_node` (or any
    // node if `parent_node == None`). Case 3: otherwise.
    if graph.nodes_iter().any(|(nid, _)| nid == opposite_node) {
        return InnerSegmentCase::Child;
    }
    match parent_node {
        None => InnerSegmentCase::Sibling,
        Some(p) =>
            if graph.is_descendant(opposite_node, p) {
                InnerSegmentCase::Sibling
            } else {
                InnerSegmentCase::Outside
            },
    }
}

fn same_external_port(lhs: &Option<ExternalPort>, rhs: &ExternalPort) -> bool {
    match lhs {
        None => false,
        Some(l) => l.new_edge == rhs.new_edge && l.dummy_node == rhs.dummy_node,
    }
}

fn connect_child(
    graph: &mut LGraph,
    external_port: &ExternalPort,
    orig_edge: EdgeId,
    source_port: PortId,
    target_port: PortId,
    state: &mut PreprocessorState,
) {
    let dummy_edge = create_dummy_edge(graph, orig_edge, source_port, target_port);
    state
        .cross_hierarchy_map
        .entry(orig_edge)
        .or_default()
        .push(CrossHierarchyEdge::new(dummy_edge, graph.parent_node, external_port.port_type));
}

fn connect_siblings(
    graph: &mut LGraph,
    source_external: &ExternalPort,
    contained: &[ExternalPort],
    orig_edge: EdgeId,
    state: &mut PreprocessorState,
) {
    // Find the counterpart external port that shares this original edge and is an Input.
    let target = contained.iter().find(|other| {
        !std::ptr::eq(*other, source_external) && other.orig_edges.contains(&orig_edge)
    });
    let Some(target) = target else {
        return;
    };
    let dummy_edge =
        create_dummy_edge(graph, orig_edge, source_external.dummy_port, target.dummy_port);
    state
        .cross_hierarchy_map
        .entry(orig_edge)
        .or_default()
        .push(CrossHierarchyEdge::new(
            dummy_edge,
            graph.parent_node,
            source_external.port_type,
        ));
}

fn process_outer_hierarchical_edge_segments(
    graph: &mut LGraph,
    parent_node: NodeId,
    exported: &mut Vec<ExternalPort>,
    state: &mut PreprocessorState,
) {
    let mut created: Vec<ExternalPort> = Vec::new();
    let child_ids: Vec<NodeId> = graph.layerless_nodes.clone();
    for child in child_ids {
        let child_ports: SmallVec<PortId, 6> = graph.node(child).ports.iter().copied().collect();
        for child_port in child_ports {
            // Outgoing
            let mut current_out: Option<ExternalPort> = None;
            let out_edges: SmallVec<EdgeId, 2> =
                graph.port(child_port).outgoing_edges.iter().copied().collect();
            for edge in out_edges {
                let target_node = graph.port(graph.edge(edge).target).owner;
                if !graph.is_descendant(target_node, parent_node) && target_node != parent_node {
                    let new_port = introduce_hierarchical_edge_segment(
                        graph,
                        Some(parent_node),
                        edge,
                        graph.edge(edge).source,
                        PortType::Output,
                        current_out.as_ref(),
                        state,
                    );
                    if !same_external_port(&current_out, &new_port) {
                        created.push(new_port.clone());
                    }
                    if new_port.exported {
                        current_out = Some(new_port);
                    }
                }
            }
            // Incoming
            let mut current_in: Option<ExternalPort> = None;
            let in_edges: SmallVec<EdgeId, 2> =
                graph.port(child_port).incoming_edges.iter().copied().collect();
            for edge in in_edges {
                let source_node = graph.port(graph.edge(edge).source).owner;
                if !graph.is_descendant(source_node, parent_node) && source_node != parent_node {
                    let new_port = introduce_hierarchical_edge_segment(
                        graph,
                        Some(parent_node),
                        edge,
                        graph.edge(edge).target,
                        PortType::Input,
                        current_in.as_ref(),
                        state,
                    );
                    if !same_external_port(&current_in, &new_port) {
                        created.push(new_port.clone());
                    }
                    if new_port.exported {
                        current_in = Some(new_port);
                    }
                }
            }
        }
    }
    for ext in created {
        if !graph.layerless_nodes.contains(&ext.dummy_node) {
            graph.layerless_nodes.push(ext.dummy_node);
        }
        if ext.exported {
            exported.push(ext);
        }
    }
}

/// Creates (or reuses) a dummy edge + external port dummy for one segment of a
/// cross-hierarchy edge.
fn introduce_hierarchical_edge_segment(
    graph: &mut LGraph,
    parent_node: Option<NodeId>,
    orig_edge: EdgeId,
    opposite_port: PortId,
    port_type: PortType,
    default_external: Option<&ExternalPort>,
    state: &mut PreprocessorState,
) -> ExternalPort {
    let merge = graph.properties.get(&MERGE_HIERARCHY_EDGES);
    let parent_end_port: Option<PortId> = match (parent_node, port_type) {
        (Some(p), PortType::Input) if graph.port(graph.edge(orig_edge).source).owner == p =>
            Some(graph.edge(orig_edge).source),
        (Some(p), PortType::Output) if graph.port(graph.edge(orig_edge).target).owner == p =>
            Some(graph.edge(orig_edge).target),
        _ => None,
    };

    if let Some(existing) = default_external.filter(|_| merge && parent_end_port.is_none()) {
        // Reuse default: append orig edge to its origEdges list and record mapping.
        let mut updated = existing.clone();
        updated.orig_edges.push(orig_edge);
        state
            .cross_hierarchy_map
            .entry(orig_edge)
            .or_default()
            .push(CrossHierarchyEdge::new(existing.new_edge, graph.parent_node, port_type));
        return updated;
    }

    // Create a fresh dummy node + dummy edge.
    let external_port_side = match parent_end_port {
        Some(p) => graph.port(p).side,
        None => {
            let pc = parent_node
                .map(|p| graph.node(p).port_constraints())
                .unwrap_or(PortConstraints::Free);
            if pc.is_side_fixed() {
                if port_type == PortType::Input { PortSide::West } else { PortSide::East }
            } else {
                PortSide::Undefined
            }
        }
    };
    let (dummy_node, dummy_node_port) = create_external_port_dummy_in_graph(
        graph,
        parent_node,
        port_type,
        external_port_side,
        orig_edge,
        state,
    );
    let (src, tgt) = match port_type {
        PortType::Input => (dummy_node_port, opposite_port),
        _ => (opposite_port, dummy_node_port),
    };
    let dummy_edge = create_dummy_edge(graph, orig_edge, src, tgt);

    let exported = parent_end_port.is_none();
    let external_port = ExternalPort {
        orig_edges: {
            let mut v = SmallVec::new();
            v.push(orig_edge);
            v
        },
        new_edge: dummy_edge,
        dummy_node,
        dummy_port: dummy_node_port,
        port_type,
        exported,
    };

    state
        .cross_hierarchy_map
        .entry(orig_edge)
        .or_default()
        .push(CrossHierarchyEdge::new(dummy_edge, graph.parent_node, port_type));

    external_port
}

/// Creates a dummy node inside `compound`'s nested graph that represents the outer
/// port `compound_port`.
///
/// Reads geometry / anchor / order / border-offset from `compound_port` (in
/// `parent_graph`) and from `compound` (the compound node), so callers don't
/// have to thread eight parameters by hand. Side-effect: when
/// `port_constraints` does not fix the side, writes the resolved
/// `final_external_port_side` back onto `compound_port.side`.
fn create_external_port_dummy_in_nested(
    parent_graph: &mut LGraph,
    compound: NodeId,
    compound_port: PortId,
    port_constraints: PortConstraints,
    net_flow: i32,
) -> NodeId {
    let layout_direction = parent_graph.options.direction;

    // Read everything we need from `compound_port` + `compound` before grabbing
    // the nested mut borrow (the nested LGraph lives behind a raw pointer in
    // `compound`'s `nested_graph` slot, but reading from `parent_graph` after
    // taking `nested_mut` is still possible — the nested allocation is
    // disjoint — but it reads cleaner to snapshot once).
    let port_size = parent_graph.port(compound_port).size;
    let port_position = parent_graph.port(compound_port).position;
    let port_node_size = parent_graph.node(compound).size;
    let port_anchor_explicit = parent_graph.port(compound_port).properties.get(&PORT_ANCHOR);
    let has_explicit_anchor = port_anchor_explicit.is_some();
    let port_index = parent_graph.port(compound_port).properties.get(&PORT_INDEX);
    let port_border_offset = parent_graph.port(compound_port).properties.get(&PORT_BORDER_OFFSET);
    let raw_port_side = parent_graph.port(compound_port).side;

    // Resolve final port side from net flow when not pinned by constraints.
    let final_external_port_side = if !port_constraints.is_side_fixed() {
        if net_flow >= 0 {
            PortSide::from_direction(layout_direction)
        } else {
            PortSide::from_direction(layout_direction).opposed()
        }
    } else if raw_port_side == PortSide::Undefined {
        PortSide::from_direction(layout_direction)
    } else {
        raw_port_side
    };
    if !port_constraints.is_side_fixed() || raw_port_side == PortSide::Undefined {
        parent_graph.port_mut(compound_port).side = final_external_port_side;
    }

    // Anchor: explicit value if `PORT_ANCHOR` was set, else port-size/2.
    let mut anchor = match port_anchor_explicit {
        Some(v) => v,
        None => Vec2::new(port_size.x / 2.0, port_size.y / 2.0),
    };

    // Compute dummy size:
    //   WEST/EAST: dummy.size.y = portSize.y; if portBorderOffset < 0 { x = -portBorderOffset }
    //   NORTH/SOUTH: dummy.size.x = portSize.x; if portBorderOffset < 0 { y = -portBorderOffset }
    let dummy_size = match final_external_port_side {
        PortSide::West | PortSide::East =>
            Vec2::new(if port_border_offset < 0.0 { -port_border_offset } else { 0.0 }, port_size.y),
        PortSide::North | PortSide::South =>
            Vec2::new(port_size.x, if port_border_offset < 0.0 { -port_border_offset } else { 0.0 }),
        PortSide::Undefined => Vec2::ZERO,
    };

    // Per-side anchor adjustments.
    match final_external_port_side {
        PortSide::West => {
            if !has_explicit_anchor {
                anchor.x = port_size.x;
            }
            anchor.x -= port_size.x;
        }
        PortSide::East =>
            if !has_explicit_anchor {
                anchor.x = 0.0;
            },
        PortSide::North => {
            if !has_explicit_anchor {
                anchor.y = port_size.y;
            }
            anchor.y -= port_size.y;
        }
        PortSide::South =>
            if !has_explicit_anchor {
                anchor.y = 0.0;
            },
        PortSide::Undefined => {}
    }

    // Dummy port lives on the inside of the compound — opposite of the compound's port side.
    let dummy_port_side = match final_external_port_side {
        PortSide::West => PortSide::East,
        PortSide::East => PortSide::West,
        PortSide::North => PortSide::South,
        PortSide::South => PortSide::North,
        PortSide::Undefined => PortSide::Undefined,
    };

    // PORT_RATIO_OR_POSITION when port order is fixed
    // (`LGraphUtil.createExternalPortDummy:910-955`).
    let mut ratio_or_position: Option<f64> = None;
    if port_constraints.is_order_fixed() {
        let info = if port_constraints == PortConstraints::FixedOrder
            && parent_graph.port(compound_port).properties.has(&PORT_INDEX)
        {
            // PORT_INDEX is reversed for SOUTH/WEST sides
            // (`LGraphUtil.createExternalPortDummy:920-929`).
            match final_external_port_side {
                PortSide::North | PortSide::East => port_index as f64,
                PortSide::South | PortSide::West => -(port_index as f64),
                PortSide::Undefined => 0.0,
            }
        } else {
            let mut value = match final_external_port_side {
                PortSide::West | PortSide::East => port_position.y + port_size.y / 2.0,
                PortSide::North | PortSide::South => port_position.x + port_size.x / 2.0,
                PortSide::Undefined => 0.0,
            };
            if port_constraints.is_ratio_fixed() {
                let denom = match final_external_port_side {
                    PortSide::West | PortSide::East => port_node_size.y,
                    PortSide::North | PortSide::South => port_node_size.x,
                    PortSide::Undefined => 1.0,
                };
                if denom > 0.0 {
                    value /= denom;
                }
            }
            value
        };
        ratio_or_position = Some(info);
    }

    let nested = parent_graph.nested_mut(compound).unwrap();
    // `LGraph::add_node` already pushes the new id to `layerless_nodes`.
    // Don't push again — doing so leaves every external-port dummy listed
    // twice, which P2NetworkSimplex then flushes into `layers[0]` as
    // duplicate entries, ultimately corrupting the parent compound's
    // `ports` list when `transfer_child_dummy_order_to_parent_ports` reads
    // those duplicate dummies and maps them back to ORIGIN_PORT.
    let dummy_node = nested.add_node(dummy_size);
    nested.node_mut(dummy_node).node_type = NodeType::ExternalPort;

    {
        let n = nested.node_mut(dummy_node);
        // Every external port dummy ships with `PORT_CONSTRAINTS = FIXED_POS`.
        // Without this, downstream port sorters (`PortListSorter`) rearrange
        // the dummy's single port and break the `EXT_PORT_SIDE` invariant
        // the rest of the pipeline relies on.
        n.node_port_constraints = Some(PortConstraints::FixedPos);
        n.properties.set(&PORT_BORDER_OFFSET, port_border_offset);
        n.properties.set(&EXT_PORT_SIZE, port_size);
        n.properties.set(&EXT_PORT_SIDE, final_external_port_side);
        n.properties.set(&ORIGIN_PORT, Some(compound_port));
        n.properties.set(&PORT_ANCHOR, Some(anchor));
        if let Some(rop) = ratio_or_position {
            n.properties.set(&PORT_RATIO_OR_POSITION, rop);
        }

        // Per-side layer/edge constraints for EP dummies. Without these,
        // P2NetworkSimplex treats them as ordinary nodes and lays them out
        // wherever their single edge points, instead of pinning W->layer-0,
        // E->last-layer, N/S->top/bottom of their layer.
        match final_external_port_side {
            PortSide::West => {
                n.properties.set(&LAYER_CONSTRAINT, LayerConstraint::FirstSeparate);
                n.properties.set(&NODE_EDGE_CONSTRAINT, EdgeConstraint::OutgoingOnly);
            }
            PortSide::East => {
                n.properties.set(&LAYER_CONSTRAINT, LayerConstraint::LastSeparate);
                n.properties.set(&NODE_EDGE_CONSTRAINT, EdgeConstraint::IncomingOnly);
            }
            PortSide::North => {
                n.properties.set(&IN_LAYER_CONSTRAINT, InLayerConstraint::Top);
            }
            PortSide::South => {
                n.properties.set(&IN_LAYER_CONSTRAINT, InLayerConstraint::Bottom);
            }
            PortSide::Undefined => {}
        }
    }

    let dummy_port = nested.add_port(dummy_node, dummy_port_side);
    // Construct a fresh port with default `(0, 0)` size. Setting a non-zero
    // `port_size` here propagates 8px through downstream port placement /
    // margin / spacing on the inner side, which manifests as an extra 8px
    // in the parent compound's width on every fixture that hits this path.
    nested.port_mut(dummy_port).size = Vec2::ZERO;
    // Apply the anchor by setting the dummy port position.
    nested.port_mut(dummy_port).position = anchor;

    let mut gp = nested.properties.get(&GRAPH_PROPERTIES);
    gp.insert(GraphProperties::EXTERNAL_PORTS);
    nested.properties.set(&GRAPH_PROPERTIES, gp);
    dummy_node
}

/// Creates a dummy inside `graph` itself (the parent level) used as an endpoint for a
/// new cross-hierarchy segment. If `parent_node` is Some, also creates or reuses a
/// matching outside port on that parent node.
fn create_external_port_dummy_in_graph(
    graph: &mut LGraph,
    parent_node: Option<NodeId>,
    port_type: PortType,
    port_side: PortSide,
    orig_edge: EdgeId,
    state: &mut PreprocessorState,
) -> (NodeId, PortId) {
    let layout_direction = graph.options.direction;
    // Outside port: the end of `orig_edge` that lives in the parent graph (i.e. on the
    // outside of this level). For `PortType::Input`, the source port is outside.
    let outside_port = match port_type {
        PortType::Input => graph.edge(orig_edge).source,
        _ => graph.edge(orig_edge).target,
    };
    let outside_owner = graph.port(outside_port).owner;
    let attaches_to_parent = match parent_node {
        Some(p) => outside_owner == p,
        None => false,
    };

    if attaches_to_parent {
        if let Some(existing) = state.dummy_node_map.get(&outside_port).copied() {
            let port = *graph.node(existing).ports.first().expect("dummy has port");
            return (existing, port);
        }
        let net_flow = calculate_net_flow(graph, outside_port);
        let dummy = add_external_port_dummy_raw(
            graph,
            None,
            port_side,
            layout_direction,
            graph.port(outside_port).size,
            Some(outside_port),
            net_flow,
        );
        let port = *graph.node(dummy).ports.first().expect("dummy has port");
        state.dummy_node_map.insert(outside_port, dummy);
        graph.layerless_nodes.retain(|&nid| nid != dummy);
        (dummy, port)
    } else {
        let net_flow = if port_type == PortType::Input { -1 } else { 1 };
        let dummy = add_external_port_dummy_raw(
            graph,
            None,
            port_side,
            layout_direction,
            Vec2::ZERO,
            None,
            net_flow,
        );
        // When the edge does not attach to the parent, also create a port on the
        // parent itself so the outside world sees a sink/source. Record that new
        // outside port as the origin so later lookups route through it.
        let dummy_port = *graph.node(dummy).ports.first().expect("dummy has port");
        if let Some(pnode) = parent_node {
            let parent_port_side = match port_type {
                PortType::Input => PortSide::from_direction(layout_direction).opposed(),
                _ => PortSide::from_direction(layout_direction),
            };
            let parent_port = graph.add_port(pnode, parent_port_side);
            let border_offset = graph.node(dummy).properties.get(&PORT_BORDER_OFFSET);
            graph.port_mut(parent_port).properties.set(&PORT_BORDER_OFFSET, border_offset);
            graph.node_mut(dummy).properties.set(&ORIGIN_PORT, Some(parent_port));
            state.dummy_node_map.insert(parent_port, dummy);
        }
        graph.layerless_nodes.retain(|&nid| nid != dummy);
        (dummy, dummy_port)
    }
}

fn add_external_port_dummy_raw(
    graph: &mut LGraph,
    existing_dummy: Option<NodeId>,
    port_side: PortSide,
    layout_direction: LayoutDirection,
    port_size: Vec2,
    origin_port: Option<PortId>,
    net_flow: i32,
) -> NodeId {
    if let Some(id) = existing_dummy {
        return id;
    }
    let dummy_node = graph.add_node(Vec2::ZERO);
    graph.node_mut(dummy_node).node_type = NodeType::ExternalPort;
    let dummy_side =
        resolve_external_dummy_side(PortConstraints::Free, port_side, net_flow, layout_direction);
    let dummy_port = graph.add_port(dummy_node, dummy_side);
    graph.port_mut(dummy_port).size = port_size;
    graph.node_mut(dummy_node).properties.set(&EXT_PORT_SIDE, dummy_side);
    if let Some(p) = origin_port {
        graph.node_mut(dummy_node).properties.set(&ORIGIN_PORT, Some(p));
    }
    let mut gp = graph.properties.get(&GRAPH_PROPERTIES);
    gp.insert(GraphProperties::EXTERNAL_PORTS);
    graph.properties.set(&GRAPH_PROPERTIES, gp);
    dummy_node
}

fn resolve_external_dummy_side(
    port_constraints: PortConstraints,
    port_side: PortSide,
    net_flow: i32,
    layout_direction: LayoutDirection,
) -> PortSide {
    if port_side != PortSide::Undefined {
        return port_side;
    }
    if port_constraints.is_side_fixed() {
        return port_side;
    }
    let output_side = PortSide::from_direction(layout_direction);
    if net_flow >= 0 { output_side } else { output_side.opposed() }
}

/// Creates a new edge inside `graph` that mimics `orig_edge`'s properties. The dummy
/// edge does NOT preserve labels — those are moved in `move_labels_...`.
fn create_dummy_edge(
    graph: &mut LGraph,
    orig_edge: EdgeId,
    source: PortId,
    target: PortId,
) -> EdgeId {
    // A faithful port would copy the PropertyMap; the current `LGraph` API has no
    // property-copy helper, so we leave the dummy with default properties. Consumers
    // read edge thickness / label placement via the original edge where still present.
    let edge = graph.add_edge(source, target);
    if orig_edge.0.graph_id() == graph.graph_id()
        && let Some(original) = graph.try_edge(orig_edge)
    {
        let order = original.order;
        graph.edge_mut(edge).order = order;
        sort_port_edges_by_order(graph, source, true);
        sort_port_edges_by_order(graph, target, false);
    }
    edge
}

fn sort_port_edges_by_order(graph: &mut LGraph, port: PortId, outgoing: bool) {
    let mut edges: Vec<EdgeId> = if outgoing {
        graph.port(port).outgoing_edges.iter().copied().collect()
    } else {
        graph.port(port).incoming_edges.iter().copied().collect()
    };
    edges.sort_by_key(|&edge| graph.edge(edge).order);
    if outgoing {
        graph.port_mut(port).outgoing_edges = edges.into_iter().collect();
    } else {
        graph.port_mut(port).incoming_edges = edges.into_iter().collect();
    }
}

/// Net flow calculation for a compound port. Positive values indicate the
/// port should be treated as an output of the parent compound node.
fn calculate_net_flow(graph: &LGraph, port: PortId) -> i32 {
    let owner = graph.port(port).owner;
    let inside_loops = graph.node(owner).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);
    let mut output_vote = 0i32;
    let mut input_vote = 0i32;
    for edge in graph.port(port).outgoing_edges.iter().copied() {
        let tgt_node = graph.port(graph.edge(edge).target).owner;
        let is_self_loop = tgt_node == owner;
        let is_inside_loop =
            is_self_loop && inside_loops && graph.edge(edge).properties.get(&INSIDE_SELF_LOOPS_YO);
        if is_self_loop && is_inside_loop {
            input_vote += 1;
        } else if is_self_loop {
            output_vote += 1;
        } else if tgt_node_parent_is(graph, tgt_node, owner) {
            input_vote += 1;
        } else {
            output_vote += 1;
        }
    }
    for edge in graph.port(port).incoming_edges.iter().copied() {
        let src_node = graph.port(graph.edge(edge).source).owner;
        let is_self_loop = src_node == owner;
        let is_inside_loop =
            is_self_loop && inside_loops && graph.edge(edge).properties.get(&INSIDE_SELF_LOOPS_YO);
        if is_self_loop && is_inside_loop {
            output_vote += 1;
        } else if is_self_loop {
            input_vote += 1;
        } else if tgt_node_parent_is(graph, src_node, owner) {
            output_vote += 1;
        } else {
            input_vote += 1;
        }
    }
    output_vote - input_vote
}

/// True when `node`'s containing graph has `owner` as its parent node — i.e. `node` is
/// a direct child of `owner`'s nested graph.
fn tgt_node_parent_is(graph: &LGraph, node: NodeId, owner: NodeId) -> bool {
    let Some(containing) = graph.find_graph_containing(node) else {
        return false;
    };
    containing.parent_node == Some(owner)
}

/// Moves labels from each original edge onto one of its dummy segments, then removes
/// the original edge's port connections.
fn move_labels_and_remove_original_edges(graph: &mut LGraph, state: &PreprocessorState) {
    let orig_edges: Vec<EdgeId> = state.cross_hierarchy_map.keys().copied().collect();
    for orig_edge in orig_edges {
        let label_ids: SmallVec<LabelId, 2> =
            graph.edge(orig_edge).labels.iter().copied().collect();
        if !label_ids.is_empty() {
            // Sort segments source -> target.
            let mut segments: SmallVec<CrossHierarchyEdge, 4> = state
                .cross_hierarchy_map
                .get(&orig_edge)
                .map(|segments| segments.iter().copied().collect())
                .unwrap_or_default();
            segments.sort_by(|a, b| compare_cross_hierarchy_edges(graph, a, b));
            for label in label_ids {
                let placement = graph.label(label).properties.get(&EDGE_LABEL_PLACEMENT);
                let target_index: Option<usize> = match placement {
                    EdgeLabelPlacement::Head => segments.len().checked_sub(1),
                    EdgeLabelPlacement::Center => shallowest_segment_index(graph, &segments),
                    EdgeLabelPlacement::Tail => Some(0),
                    _ => None,
                };
                let Some(idx) = target_index else { continue };
                let Some(seg) = segments.get(idx) else { continue };
                // Find the segment's graph and transfer the label on its dummy edge.
                let seg_parent = seg.graph_parent_node;
                let seg_edge = seg.new_edge;
                transfer_label_to_segment(graph, label, orig_edge, seg_edge, seg_parent);
            }
        }
        // Remove original edge from its ports so subsequent phases do not see it.
        unlink_original_edge(graph, orig_edge);
    }
}

fn shallowest_segment_index(root: &LGraph, segments: &[CrossHierarchyEdge]) -> Option<usize> {
    segments
        .iter()
        .enumerate()
        .min_by_key(|(_, seg)| hierarchy_level(root, seg.graph_parent_node))
        .map(|(index, _)| index)
}

fn transfer_label_to_segment(
    graph: &mut LGraph,
    label: LabelId,
    orig_edge: EdgeId,
    seg_edge: EdgeId,
    seg_parent: Option<NodeId>,
) {
    // Locate the graph holding `seg_edge`.
    let target_graph_ref: *mut LGraph = match seg_parent {
        None => graph as *mut LGraph,
        Some(p) => {
            let parent_graph_id = p.0.graph_id();
            let parent_graph_ptr = match crate::graph::graph_by_id_mut(parent_graph_id) {
                Some(ptr) => ptr,
                None => return,
            };
            // SAFETY: registry pointer is valid until LGraph::Drop.
            let parent_graph = unsafe { &mut *parent_graph_ptr };
            match parent_graph.nested_mut(p) {
                Some(g) => g as *mut LGraph,
                None => return,
            }
        }
    };
    // SAFETY: `target_graph_ref` points either to `graph` itself or to a nested graph
    // obtained through `nested_mut`, both of which remain valid for the duration of
    // this call. We don't alias the borrow.
    let target: &mut LGraph = unsafe { &mut *target_graph_ref };
    // Update label back-reference then append to the segment's label list.
    // When the original proxy edge lives in another graph arena, clone the
    // label data into the segment graph; label ids are arena-local.
    if label.arena_id().graph_id() == target.graph_id() {
        target.label_mut(label).properties.set(&ORIGINAL_LABEL_EDGE, Some(orig_edge));
        target.edge_mut(seg_edge).labels.push(label);
    } else {
        let source_label = graph.label(label);
        let new_label =
            target.add_edge_label(seg_edge, source_label.text.clone(), source_label.size);
        target.label_mut(new_label).position = source_label.position;
        target.label_mut(new_label).properties = source_label.properties.clone();
        target
            .label_mut(new_label)
            .properties
            .set(&ORIGINAL_LABEL_EDGE, Some(orig_edge));
    }
    // Record graph-level label property flags: END_LABELS + CENTER_LABELS.
    let mut gp = target.properties.get(&GRAPH_PROPERTIES);
    gp.insert(GraphProperties::END_LABELS);
    gp.insert(GraphProperties::CENTER_LABELS);
    target.properties.set(&GRAPH_PROPERTIES, gp);
    // Remove the label from the original edge's label list.
    graph.edge_mut(orig_edge).labels.retain(|l| *l != label);
}

fn unlink_original_edge(graph: &mut LGraph, orig_edge: EdgeId) {
    let src = graph.edge(orig_edge).source;
    let tgt = graph.edge(orig_edge).target;
    // `orig_edge` can be a cross-hierarchy proxy edge installed via
    // `add_edge_orphan`: its source / target PortIds may live in nested
    // LGraphs' arenas, not in `graph`'s. `try_port_mut` silently skips
    // when the port is not in `graph.ports`; the adjacency list of the
    // real port never had `orig_edge` anyway (orphan inserts do not
    // register), so the retain is a no-op on that side.
    if let Some(port) = graph.try_port_mut(src) {
        port.outgoing_edges.retain(|e| *e != orig_edge);
    }
    if let Some(port) = graph.try_port_mut(tgt) {
        port.incoming_edges.retain(|e| *e != orig_edge);
    }
}

fn set_sides_of_ports_to_sides_of_dummy_nodes(graph: &mut LGraph, state: &PreprocessorState) {
    // For each (port, dummy) in `dummy_node_map`, record:
    //   dummy.set(ORIGIN, port)
    //   port.set(PORT_DUMMY, dummy)
    //   port.set(INSIDE_CONNECTIONS, true)
    //   port.side = dummy.EXT_PORT_SIDE
    //   port.node.set(PORT_CONSTRAINTS, FIXED_SIDE)
    //   port.node.graph.GRAPH_PROPERTIES.add(NON_FREE_PORTS)
    //
    // `state.dummy_node_map` accumulates entries across every LGraph visited
    // by the `transform_hierarchy_edges` hierarchy walk. The corresponding
    // `port_id` therefore lives in whichever LGraph created the dummy —
    // often a nested graph, not the `graph` handed to this function. Route
    // every mutation to the owning LGraph via `find_graph_owning_port_ptr`
    // so we never call `port_mut` / `node_mut` on the wrong arena.
    //
    // The `dummy_node` on the other hand is always inside `graph`'s nested
    // tree (it is the external-port dummy created by
    // `ensure_port_dummies_for_compound_node`). Its side property is read
    // through `locate_dummy_side`, which walks `find_graph_containing`.
    for (&port_id, &dummy_node) in state.dummy_node_map.iter() {
        let dummy_side: PortSide =
            locate_dummy_side(graph, dummy_node).unwrap_or(PortSide::Undefined);

        let Some(owning_ptr) = find_graph_owning_port_ptr(graph, port_id) else {
            continue;
        };
        // SAFETY: `owning_ptr` was obtained synchronously from
        // `find_graph_owning_port_ptr` walking `graph`'s nested subtree; the
        // pointer is valid for the lifetime of `graph`'s `&mut` borrow, and
        // no other borrow aliases it while we mutate through it here.
        let owning = unsafe { &mut *owning_ptr };
        let owner_node = owning.port(port_id).owner;

        owning.node_mut(owner_node).properties.set(&ORIGIN_NODE, Some(owner_node));
        owning.port_mut(port_id).port_dummy = Some(dummy_node);
        owning.port_mut(port_id).properties.set(&INSIDE_CONNECTIONS, true);
        owning.port_mut(port_id).side = dummy_side;
        owning.node_mut(owner_node).node_port_constraints = Some(PortConstraints::FixedSide);

        let mut gp = owning.properties.get(&GRAPH_PROPERTIES);
        gp.insert(GraphProperties::NON_FREE_PORTS);
        owning.properties.set(&GRAPH_PROPERTIES, gp);
    }
}

fn locate_dummy_side(graph: &LGraph, dummy: NodeId) -> Option<PortSide> {
    graph
        .find_graph_containing(dummy)
        .map(|g| g.node(dummy).properties.get(&EXT_PORT_SIDE))
}

/// Resolve a `PortId` to the owning node id by routing through the global
/// graph registry: every PortId encodes the `graph_id` of the LGraph whose
/// arena holds it, so we can dispatch directly to the right LGraph in O(1)
/// without DFS-walking the nested subtree.
///
/// `root` is only used as a fallback when the encoded graph_id is no longer
/// in the registry (the LGraph has been dropped, or it's the same graph_id
/// as the local one). Returns `None` if the port cannot be resolved.
fn resolve_port_owner(root: &LGraph, port: PortId) -> Option<NodeId> {
    if root.graph_id() == port.0.graph_id() {
        return root.try_port(port).map(|p| p.owner);
    }
    let ptr = crate::graph::graph_by_id(port.0.graph_id())?;
    // SAFETY: `ptr` was registered by `LGraph::set_nested` and is unregistered
    // by `LGraph::Drop`. We hold no mutable borrow into the target LGraph
    // here (the caller's `&LGraph` on `root` doesn't alias deeper nested
    // graphs in any read path that takes `resolve_port_owner`).
    let g = unsafe { &*ptr };
    g.try_port(port).map(|p| p.owner)
}

/// Walk `graph` and its nested subtree to find the LGraph whose `ports`
/// arena holds `port`. Returns a raw pointer so the caller can mutate
/// through it without the borrow checker tracking intermediate sub-borrows.
fn find_graph_owning_port_ptr(graph: &mut LGraph, port: PortId) -> Option<*mut LGraph> {
    let mut stack = vec![graph as *mut LGraph];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers come from the nested ownership tree rooted at
        // the input graph and are only used one at a time.
        let graph = unsafe { &mut *graph_ptr };
        if graph.try_port(port).is_some() {
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
