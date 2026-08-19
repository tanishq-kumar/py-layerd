// Phase internals pass graph state, options, and scratch buffers explicitly.
#![allow(clippy::too_many_arguments)]

//! Layered graph layout engine.

mod algorithms;
mod components;
mod intermediate;
mod nodespacing;
mod p1_cycle_breaking;
mod p2_layering;
mod p3_crossing_min;
mod p4_node_placement;
mod p5_edge_routing;
mod pipeline;

pub mod graph;
pub mod math;
pub mod options;
pub mod properties;
pub mod rng;

use graph::{
    LGraph,
    index::{EdgeId, NodeId, PortId},
    node::NodeType,
    port::PortSide,
};
use math::Vec2;
use options::enums::{
    EdgeLabelPlacement, HierarchyHandling, NodeLabelPlacement, PortConstraints, SizeConstraint,
};
use properties::{
    graph_properties::GraphProperties,
    internal::{
        COMMENT_BOX, EDGE_LABEL_PLACEMENT, EXT_PORT_REPLACED_DUMMY, EXT_PORT_SIDE,
        GRAPH_PROPERTIES, HYPERNODE, INSIDE_SELF_LOOPS_ACTIVATE, INSIDE_SELF_LOOPS_YO,
        NODE_LABEL_PLACEMENT, NODE_LABELS_PADDING, NODE_SIZE_CONSTRAINTS,
        NODE_SIZE_FIXED_GRAPH_SIZE, NODE_SIZE_MINIMUM, ORIGIN_PORT, P3_IGNORE_NESTED_GRAPHS,
        PORT_BORDER_OFFSET,
    },
};

/// Diagnostic hooks used by local workspace tools.
#[cfg(feature = "devtools")]
pub mod diagnostics {
    use crate::graph::LGraph;
    pub use crate::{properties, rng};

    pub fn resolve_hierarchy_handling(graph: &mut LGraph) {
        crate::resolve_hierarchy_handling(graph);
    }

    pub fn prepare_graph_for_layout(graph: &mut LGraph) {
        crate::prepare_graph_for_layout(graph);
    }

    pub mod components {
        use crate::graph::LGraph;

        pub fn split(graph: &mut LGraph) -> Vec<LGraph> {
            crate::components::split(graph)
        }

        pub fn combine(components: Vec<LGraph>, target: &mut LGraph) {
            crate::components::combine(components, target);
        }
    }

    pub mod pipeline {
        pub use crate::pipeline::{PhaseImpl, PhaseSlot, PipelineStage};

        pub mod configurator {
            use crate::{graph::LGraph, pipeline::PipelineStage};

            pub fn build_pipeline(graph: &LGraph) -> Vec<PipelineStage> {
                crate::pipeline::configurator::build_pipeline(graph)
            }
        }
    }

    pub mod p3_crossing_min {
        pub mod scratch_stats {
            pub use crate::p3_crossing_min::scratch_stats::{
                P3ScratchStats, enable_global_stats, reset_global_stats, take_global_stats,
            };
        }
    }
}

/// Run the complete layered layout algorithm on the given graph.
///
/// Build a pipeline of processors based on the graph's options and
/// execute them in order: cycle breaking, layering, crossing minimization,
/// node placement, and edge routing, along with intermediate processors.
pub fn layout(graph: &mut LGraph) {
    let root = std::ptr::NonNull::from(&mut *graph);
    let mut stack = vec![LayoutFrame::Enter(root)];

    while let Some(frame) = stack.pop() {
        match frame {
            LayoutFrame::Enter(graph_ptr) => {
                // SAFETY: every frame contains a unique LGraph pointer from the
                // nested ownership tree. Children are processed only after their
                // parent borrow has ended.
                let graph = unsafe { &mut *graph_ptr.as_ptr() };
                // Register the current LGraph's address with the global registry.
                // `set_nested` registers nested children at creation time, but the
                // explicit call preserves per-`layout()` entry behavior and refreshes
                // the pointer after callers move a graph between runs.
                graph.register_self_ptr();

                // Apply each compound's inside-label padding to its nested LGraph's
                // padding before `prepare_graph_for_layout` copies options into the
                // runtime padding field.
                apply_inside_label_padding_to_hierarchy(graph);

                // INCLUDE_CHILDREN must be explicit on the input; SEPARATE drops
                // queued cross-hierarchy edges into external-port dummies.
                resolve_hierarchy_handling(graph);
                match graph.options.hierarchy_handling {
                    options::enums::HierarchyHandling::Include => {
                        do_compound_layout(graph);
                    }
                    _ => {
                        let inside_loop_compounds =
                            materialize_inside_self_loop_nested_for_leaves(graph);
                        intermediate::compound_graph::install_external_ports_for_separate_hierarchy(
                            graph,
                        );
                        let metas =
                            move_inside_self_loops_into_nested(graph, &inside_loop_compounds);
                        begin_separate_layout(graph);

                        let mut children: Vec<std::ptr::NonNull<LGraph>> = graph
                            .nested_graphs_mut()
                            .map(|(_, nested)| std::ptr::NonNull::from(&mut *nested))
                            .collect();
                        stack.push(LayoutFrame::FinishSeparate { graph: graph_ptr, metas });
                        while let Some(child) = children.pop() {
                            stack.push(LayoutFrame::Enter(child));
                        }
                    }
                }
            }
            LayoutFrame::FinishSeparate { graph, metas } => {
                // SAFETY: child frames have completed and no other live borrow targets
                // this graph pointer.
                let graph = unsafe { &mut *graph.as_ptr() };
                finish_separate_layout(graph);
                // Self-loops routed inside a nested LGraph have coordinates relative
                // to the compound's top-left corner; offset them back to the outer
                // coordinate system.
                apply_inside_self_loop_writeback(graph, &metas);
            }
        }
    }
}

enum LayoutFrame {
    Enter(std::ptr::NonNull<LGraph>),
    FinishSeparate { graph: std::ptr::NonNull<LGraph>, metas: Vec<InsideLoopEdgeMeta> },
}

/// Resolve `HIERARCHY_HANDLING::Inherit` top-down across the whole hierarchy.
///
/// Root `Inherit` becomes `Separate`; every nested level with `Inherit`
/// picks up its parent's value. LGraph stores `hierarchy_handling` per
/// graph with no fallback, so the resolution must land on every level
/// explicitly: otherwise nested graphs that stay on `Inherit` are not
/// recognised as hierarchical and `HierarchicalNodeResizer` is omitted
/// from their pipeline, leaving the parent compound at size `(0, 0)`.
fn resolve_hierarchy_handling(graph: &mut LGraph) {
    use options::enums::HierarchyHandling;
    if graph.options.hierarchy_handling == HierarchyHandling::Inherit {
        graph.options.hierarchy_handling = HierarchyHandling::Separate;
    }
    propagate_hierarchy_handling(graph);
}

fn propagate_hierarchy_handling(graph: &mut LGraph) {
    use options::enums::HierarchyHandling;
    let mut stack = vec![(std::ptr::NonNull::from(&mut *graph), graph.options.hierarchy_handling)];
    while let Some((graph_ptr, parent_value)) = stack.pop() {
        // SAFETY: each pointer is visited after its parent borrow has ended.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let mut children: Vec<(std::ptr::NonNull<LGraph>, HierarchyHandling)> = Vec::new();
        for (_id, nested) in graph.nested_graphs_mut() {
            if nested.options.hierarchy_handling == HierarchyHandling::Inherit {
                nested.options.hierarchy_handling = parent_value;
            }
            children
                .push((std::ptr::NonNull::from(&mut *nested), nested.options.hierarchy_handling));
        }
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }
}

/// Compound-graph layout entry.
///
/// - No connected-components split.
/// - `CompoundGraphPreprocessor` / `Postprocessor` run once on the root graph
///   out of band; they are NOT part of `build_pipeline` under this path.
/// - `hierarchical_layout` drives the per-graph interleaved pipeline for every
///   level in the hierarchy.
fn do_compound_layout(graph: &mut LGraph) {
    let root_ptr = graph as *const LGraph;
    apply_hierarchical_import_minimum_sizes(graph, root_ptr);
    intermediate::compound_graph::preprocess(graph);
    hierarchical_layout(graph);
    intermediate::compound_graph::postprocess(graph);

    // Apply the root LGraph's `(offset + padding)`, then walk every nested
    // LGraph. Without this hierarchy pass, nested children stay at
    // local-LGraph-origin and never get shifted by the nested LGraph's own
    // `(offset + padding)`.
    apply_layout_padding_to_hierarchy(graph);
}

/// Compute each compound node's label/port-driven minimum size and store it
/// on the nested LGraph before the layered pipeline starts.
///
/// `HierarchicalNodeResizer` later grows the nested graph to at least that
/// minimum before writing the size back to the compound node. Only runs
/// under `HIERARCHY_HANDLING=INCLUDE_CHILDREN`.
fn apply_hierarchical_import_minimum_sizes(graph: &mut LGraph, root_ptr: *const LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers are unique nested graph boxes.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let node_ids: Vec<NodeId> = graph
            .nodes_iter()
            .filter_map(|(id, node)| node.nested_graph.is_some().then_some(id))
            .collect();

        for node_id in node_ids.iter().copied() {
            let node_constraints = graph.node(node_id).properties.get(&NODE_SIZE_CONSTRAINTS);
            let nested_constraints = graph
                .nested(node_id)
                .map(|nested| nested.properties.get(&NODE_SIZE_CONSTRAINTS))
                .unwrap_or_else(SizeConstraint::empty);

            if !node_constraints.is_empty() || !nested_constraints.is_empty() {
                ensure_defined_port_sides_for_minimum_size(graph, node_id, root_ptr);
                let minimum = intermediate::node_dimension_calculation::calculate_node_minimum_size(
                    graph, node_id,
                );

                if let Some(nested) = graph.nested_mut(node_id) {
                    let mut constraints = nested.properties.get(&NODE_SIZE_CONSTRAINTS);
                    constraints.insert(SizeConstraint::MINIMUM_SIZE);
                    nested.properties.set(&NODE_SIZE_CONSTRAINTS, constraints);

                    let mut configured_min = nested.properties.get(&NODE_SIZE_MINIMUM);
                    configured_min.x = configured_min.x.max(minimum.x);
                    configured_min.y = configured_min.y.max(minimum.y);
                    nested.properties.set(&NODE_SIZE_MINIMUM, configured_min);
                }
            }
        }

        for node_id in node_ids.into_iter().rev() {
            if let Some(nested) = graph.nested_mut(node_id) {
                stack.push(std::ptr::NonNull::from(nested));
            }
        }
    }
}

fn ensure_defined_port_sides_for_minimum_size(
    graph: &mut LGraph,
    node_id: NodeId,
    root_ptr: *const LGraph,
) {
    let mut port_constraints = node_port_constraints_for_minimum_size(graph, node_id);
    if matches!(port_constraints, PortConstraints::Undefined) {
        port_constraints = PortConstraints::Free;
        graph.node_mut(node_id).node_port_constraints = Some(PortConstraints::Free);
    }

    let ports: Vec<PortId> = graph.node(node_id).ports.iter().copied().collect();
    for port_id in ports {
        if !port_constraints.is_side_fixed() {
            let output_side = PortSide::from_direction(graph.options.direction);
            let net_flow = calculate_import_net_flow(graph, port_id, node_id, root_ptr);
            graph.port_mut(port_id).side =
                if net_flow > 0 { output_side } else { output_side.opposed() };
        } else if graph.port(port_id).side == PortSide::Undefined {
            let inferred = infer_port_side_from_position(graph, node_id, port_id)
                .unwrap_or_else(|| PortSide::from_direction(graph.options.direction));
            graph.port_mut(port_id).side = inferred;
        }
    }
}

fn node_port_constraints_for_minimum_size(graph: &LGraph, node_id: NodeId) -> PortConstraints {
    let node_constraints = graph.node(node_id).port_constraints();
    if matches!(node_constraints, PortConstraints::Undefined) {
        graph.options.port_constraints
    } else {
        node_constraints
    }
}

fn infer_port_side_from_position(
    graph: &LGraph,
    node_id: NodeId,
    port_id: PortId,
) -> Option<PortSide> {
    let node_size = graph.node(node_id).size;
    let port = graph.port(port_id);
    let candidates = [
        (PortSide::North, (port.position.y + port.size.y).abs()),
        (PortSide::East, (port.position.x - node_size.x).abs()),
        (PortSide::South, (port.position.y - node_size.y).abs()),
        (PortSide::West, (port.position.x + port.size.x).abs()),
    ];
    candidates.into_iter().min_by(|a, b| a.1.total_cmp(&b.1)).map(|(side, _)| side)
}

/// Compute the import-time net flow at a compound's port: outgoing votes
/// minus incoming votes across local and cross-hierarchy edges.
fn calculate_import_net_flow(
    graph: &LGraph,
    port_id: PortId,
    compound: NodeId,
    root_ptr: *const LGraph,
) -> i32 {
    let mut output_vote = 0i32;
    let mut input_vote = 0i32;

    for &edge_id in graph.port(port_id).outgoing_edges.iter() {
        let target_owner = graph.port(graph.edge(edge_id).target).owner;
        if target_owner == compound {
            input_vote += 1;
        } else {
            output_vote += 1;
        }
    }
    for &edge_id in graph.port(port_id).incoming_edges.iter() {
        let source_owner = graph.port(graph.edge(edge_id).source).owner;
        if source_owner == compound {
            output_vote += 1;
        } else {
            input_vote += 1;
        }
    }

    // SAFETY: `root_ptr` points at the live root graph for this layout call.
    // We only read the cross-hierarchy edge side channel here, before the
    // compound preprocessor drains it.
    let root = unsafe { &*root_ptr };
    for edge in root.hierarchical_edges.iter() {
        if edge.source.port == port_id {
            if edge.target.graph_parent == Some(compound) {
                input_vote += 1;
            } else {
                output_vote += 1;
            }
        } else if edge.target.port == port_id {
            if edge.source.graph_parent == Some(compound) {
                output_vote += 1;
            } else {
                input_vote += 1;
            }
        }
    }

    output_vote - input_vote
}

fn apply_layout_padding_to_hierarchy(graph: &mut LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers are unique nested graph boxes.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        apply_layout_padding(graph);
        let children: Vec<std::ptr::NonNull<LGraph>> = graph
            .nested_graphs_mut()
            .map(|(_, nested)| std::ptr::NonNull::from(nested))
            .collect();
        for child in children.into_iter().rev() {
            stack.push(child);
        }
    }
}

/// Compute the padding required to fit `node`'s INSIDE-positioned labels
/// at the top/bottom/left/right of its bounds.
///
/// Labels are placed in a 3×3 grid where rows are `V_TOP/V_CENTER/V_BOTTOM`
/// and columns are `H_LEFT/H_CENTER/H_RIGHT`. Each border cell contributes
/// its `minHeight` to `padding.top/bottom` or `minWidth` to
/// `padding.left/right`. When a side is non-zero, the container's own
/// padding (`NODE_LABELS_PADDING`) and gap (`2 * SPACING_LABEL_LABEL`)
/// are added on top.
///
/// Multi-label cells use `cellMinHeight = sum_heights + (n-1) *
/// label_label_spacing` and `cellMinWidth = max_widths`.
fn compute_inside_node_label_padding(graph: &LGraph, node_id: NodeId) -> math::Padding {
    let label_label_spacing = graph.options.spacing.label_label;
    let cell_gap = 2.0 * label_label_spacing;
    let container_padding = graph.node(node_id).properties.get(&NODE_LABELS_PADDING);

    // Per-cell aggregates indexed by [row][col] where 0=BEGIN, 1=CENTER, 2=END.
    let mut cell_h = [[0.0_f64; 3]; 3];
    let mut cell_w = [[0.0_f64; 3]; 3];
    let mut cell_n = [[0_usize; 3]; 3];

    let labels: smallvec::SmallVec<graph::index::LabelId, 4> =
        graph.node(node_id).labels.iter().copied().collect();
    for label_id in labels {
        let label = graph.label(label_id);
        let placement = label.properties.get(&NODE_LABEL_PLACEMENT);
        // If the label has its own placement, use it; otherwise fall back to
        // the node's `nodeLabels.placement` property.
        let placement = if placement.is_empty() {
            graph.node(node_id).properties.get(&NODE_LABEL_PLACEMENT)
        } else {
            placement
        };
        if !placement.contains(NodeLabelPlacement::INSIDE) {
            continue;
        }
        let row = if placement.contains(NodeLabelPlacement::V_TOP) {
            0
        } else if placement.contains(NodeLabelPlacement::V_BOTTOM) {
            2
        } else {
            1
        };
        let col = if placement.contains(NodeLabelPlacement::H_LEFT) {
            0
        } else if placement.contains(NodeLabelPlacement::H_RIGHT) {
            2
        } else {
            1
        };
        let h = label.size.y;
        let w = label.size.x;
        if cell_n[row][col] == 0 {
            cell_h[row][col] = h;
        } else {
            cell_h[row][col] += label_label_spacing + h;
        }
        cell_w[row][col] = cell_w[row][col].max(w);
        cell_n[row][col] += 1;
    }

    let mut padding = math::Padding::default();
    for &height in &cell_h[0] {
        padding.top = padding.top.max(height);
    }
    for &height in &cell_h[2] {
        padding.bottom = padding.bottom.max(height);
    }
    for row in &cell_w {
        padding.left = padding.left.max(row[0]);
        padding.right = padding.right.max(row[2]);
    }
    if padding.top > 0.0 {
        padding.top += container_padding.top + cell_gap;
    }
    if padding.bottom > 0.0 {
        padding.bottom += container_padding.bottom + cell_gap;
    }
    if padding.left > 0.0 {
        padding.left += container_padding.left + cell_gap;
    }
    if padding.right > 0.0 {
        padding.right += container_padding.right + cell_gap;
    }
    padding
}

/// Walk the LGraph hierarchy and add each compound node's inside-label
/// padding to its nested LGraph's `options.padding`. Source importers skip
/// this step at parse time, so it lands here instead.
///
/// Idempotent in practice when called once at layout entry — the
/// computed padding is a function of the (immutable at this point) node
/// labels.
fn apply_inside_label_padding_to_hierarchy(graph: &mut LGraph) {
    let mut stack = vec![std::ptr::NonNull::from(&mut *graph)];
    while let Some(graph_ptr) = stack.pop() {
        // SAFETY: graph pointers are unique nested graph boxes.
        let graph = unsafe { &mut *graph_ptr.as_ptr() };
        let mut to_apply: smallvec::SmallVec<(NodeId, math::Padding), 4> =
            smallvec::SmallVec::new();
        for (id, node) in graph.nodes_iter() {
            if node.nested_graph_ref().is_some() {
                let padding = compute_inside_node_label_padding(graph, id);
                if padding.top != 0.0
                    || padding.bottom != 0.0
                    || padding.left != 0.0
                    || padding.right != 0.0
                {
                    to_apply.push((id, padding));
                }
            }
        }
        for (id, pad) in to_apply.iter() {
            if let Some(nested) = graph.nested_mut(*id) {
                nested.options.padding.top += pad.top;
                nested.options.padding.bottom += pad.bottom;
                nested.options.padding.left += pad.left;
                nested.options.padding.right += pad.right;
            }
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

/// Collect every LGraph in the hierarchy rooted at `root` in reverse BFS
/// order (deepest first, `root` last).
///
/// Each returned `NonNull<LGraph>` aliases the `Box::into_raw` pointer
/// stored in the owning `NodeData.nested_graph`; uniqueness is guaranteed
/// by construction (each `NodeData` owns at most one nested graph).
/// Callers must never keep two live `&mut LGraph` to the same pointer
/// simultaneously.
fn collect_all_graphs_bottom_up(root: &mut LGraph) -> Vec<std::ptr::NonNull<LGraph>> {
    let mut collected: Vec<std::ptr::NonNull<LGraph>> = vec![std::ptr::NonNull::from(&mut *root)];
    let mut frontier = collected.clone();
    while let Some(ptr) = frontier.pop() {
        // SAFETY: unique nested-graph pointer (see function doc).
        let g = unsafe { &mut *ptr.as_ptr() };
        for (_nid, nested) in g.nested_graphs_mut() {
            let np = std::ptr::NonNull::from(&mut *nested);
            collected.push(np);
            frontier.push(np);
        }
    }
    collected.reverse(); // deepest first, root last
    collected
}

/// Enforce cross-hierarchy consistency for hierarchy-aware processors
/// before the interleaved loop runs.
///
/// - CMS must match the root's value — only one
///   `CrossingMinimizationStrategy` is supported across the whole
///   hierarchy. A mismatch is a user configuration error and panics here.
/// - `greedy_switch_hierarchical_type` on the root is copied to every
///   child, overwriting any child-level override.
fn review_and_correct_hierarchical_processors(graphs: &[std::ptr::NonNull<LGraph>]) {
    let Some((&root_ptr, children)) = graphs.split_last() else {
        return;
    };
    // SAFETY: `root_ptr` is the last element in the bottom-up list, a unique
    // pointer to the root graph. We only read root options, no overlapping write.
    let root = unsafe { root_ptr.as_ref() };
    let root_cms = root.options.crossing_minimization;
    let root_gs_hier = root.options.greedy_switch_hierarchical_type;
    for &child_ptr in children {
        // SAFETY: child pointers are distinct from `root_ptr` and distinct from
        // each other (each points to a unique nested graph).
        let child = unsafe { &mut *child_ptr.as_ptr() };
        assert_eq!(
            child.options.crossing_minimization, root_cms,
            "Hierarchy processor mismatch: child CMS={:?} root={:?} under Include",
            child.options.crossing_minimization, root_cms,
        );
        child.options.greedy_switch_hierarchical_type = root_gs_hier;
    }
}

/// Per-graph prelude shared between the compound and separate paths.
///
/// Runs padding seeding, RNG reseed, ordering-strategy sync, graph-property
/// cache, and model-order assignment.
fn prepare_graph_for_layout(graph: &mut LGraph) {
    resolve_layout_direction(graph);
    graph.padding = graph.options.padding;
    graph.reseed_from_options();
    sync_ordering_strategy_property(graph);
    cache_graph_properties(graph);
    if needs_model_order(graph) {
        assign_model_order_from_insertion(graph);
    }
    // Decide greedy switch activation on the parent graph (whose
    // `layerless_nodes.len()` is the full pre-split count) and stamp the
    // boolean on the property so each post-split component inherits the
    // parent's decision instead of re-deciding from its own (smaller)
    // layerless count.
    let activate = pipeline::configurator::compute_greedy_switch_activation(graph);
    graph.properties.set(&properties::internal::GREEDY_SWITCH_ACTIVATE, activate);
}

fn resolve_layout_direction(graph: &mut LGraph) {
    use options::enums::LayoutDirection;
    if graph.options.direction == LayoutDirection::Undefined {
        graph.options.direction = if graph.options.aspect_ratio >= 1.0 {
            LayoutDirection::Right
        } else {
            LayoutDirection::Down
        };
    }
}

/// Whether a pipeline stage must run only on the root graph during
/// compound layout, with its effect extending into all nested levels.
///
/// Only `P3LayerSweep` is hierarchy-aware: its internal sweep frame stack
/// descends into nested graphs via `take_nested_boxed` /
/// `set_nested_boxed`, so the interleaved loop must consume this slot on
/// non-root graphs without running it and run it exactly once on root.
fn is_hierarchy_aware_stage(stage: pipeline::PipelineStage) -> bool {
    matches!(stage, pipeline::PipelineStage::Phase(pipeline::PhaseImpl::P3LayerSweep))
}

/// Compound layout driver.
///
/// - Collects every LGraph in the hierarchy bottom-up.
/// - Validates cross-hierarchy processor configuration.
/// - Runs `prepare_graph_for_layout` per graph.
/// - Builds a per-graph `Vec<PipelineStage>` and an index cursor, then
///   steps cursors in the interleaved outer/inner loop: each outer
///   iteration walks all graphs bottom-up; each graph consumes
///   non-hierarchy-aware stages in order; a hierarchy-aware stage
///   breaks to the next graph, running on the root and being skipped
///   (consumed without running) on non-root graphs. Exits when root's
///   cursor reaches the end of its pipeline.
fn hierarchical_layout(root: &mut LGraph) {
    let graphs = collect_all_graphs_bottom_up(root);
    review_and_correct_hierarchical_processors(&graphs);
    for &g in &graphs {
        // SAFETY: each `NonNull` is a unique nested-graph pointer; no overlap.
        prepare_graph_for_layout(unsafe { &mut *g.as_ptr() });
    }
    // Build a child-graph -> parent-graph map (by `*mut LGraph` identity)
    // so the post-resizer hook below can propagate the nested graph's
    // final size into the parent arena's compound node. Without this,
    // the parent's P4 sees the compound node at its declared size rather
    // than the freshly-resized one.
    use std::collections::HashMap;
    let mut parent_by_child: HashMap<*mut LGraph, std::ptr::NonNull<LGraph>> = HashMap::new();
    for &g in &graphs {
        // SAFETY: read-only borrow released before reborrows below.
        let g_ref = unsafe { g.as_ref() };
        for (_nid, nested) in g_ref.nested_node_pointers() {
            parent_by_child.insert(nested.as_ptr(), g);
        }
    }
    let mut plans: Vec<(std::ptr::NonNull<LGraph>, Vec<pipeline::PipelineStage>, usize)> = graphs
        .iter()
        .map(|&g| {
            // SAFETY: read-only borrow for pipeline construction; released
            // before `stage.run` later reborrows mutably.
            let pipeline = pipeline::configurator::build_pipeline(unsafe { g.as_ref() });
            (g, pipeline, 0usize)
        })
        .collect();
    let has_hierarchical_resizer: Vec<bool> = plans
        .iter()
        .map(|(_, pipeline, _)| pipeline.iter().copied().any(is_hierarchical_resizer_stage))
        .collect();
    let mut pending_size_mirrors: HashMap<*mut LGraph, Vec<std::ptr::NonNull<LGraph>>> =
        HashMap::new();
    let root_idx = plans.len() - 1;
    loop {
        if plans[root_idx].2 >= plans[root_idx].1.len() {
            break;
        }
        for idx in 0..plans.len() {
            let is_root = idx == root_idx;
            loop {
                let cursor = plans[idx].2;
                if should_flush_pending_size_mirrors(&plans[idx].1, cursor)
                    && let Some(children) = pending_size_mirrors.remove(&plans[idx].0.as_ptr())
                {
                    for child_ptr in children {
                        mirror_nested_size_to_parent(child_ptr, plans[idx].0);
                    }
                }
                if cursor >= plans[idx].1.len() {
                    break;
                }
                let stage = plans[idx].1[cursor];
                if is_hierarchy_aware_stage(stage) {
                    plans[idx].2 = cursor + 1;
                    if is_root {
                        let ptr = plans[idx].0;
                        // SAFETY: `ptr` is the unique root pointer; `plans`
                        // holds no live `&mut LGraph` across this reborrow.
                        stage.run(unsafe { &mut *ptr.as_ptr() });
                    }
                    // Non-root graphs consume the hierarchy-aware slot
                    // without running it. Break to next graph / outer
                    // iteration.
                    break;
                }
                let ptr = plans[idx].0;
                // SAFETY: same — `ptr` is unique and disjoint from other plans.
                stage.run(unsafe { &mut *ptr.as_ptr() });
                plans[idx].2 = cursor + 1;

                // After the nested graph's resizer runs, immediately write
                // the new size back to the parent compound node. Arenas are
                // not shared across hierarchy levels, so this write-back
                // has to happen here — during the interleaved loop — so
                // the parent's later P4/P5 sees the corrected size.
                if matches!(
                    stage,
                    pipeline::PipelineStage::Intermediate(
                        intermediate::IntermediateProcessorId::HierarchicalNodeResizer
                    )
                ) && let Some(&parent_ptr) = parent_by_child.get(&ptr.as_ptr())
                {
                    mirror_nested_size_to_parent(ptr, parent_ptr);
                }
                if plans[idx].2 >= plans[idx].1.len()
                    && !has_hierarchical_resizer[idx]
                    && let Some(&parent_ptr) = parent_by_child.get(&ptr.as_ptr())
                {
                    pending_size_mirrors.entry(parent_ptr.as_ptr()).or_default().push(ptr);
                }
            }
        }
    }
}

fn is_hierarchical_resizer_stage(stage: pipeline::PipelineStage) -> bool {
    matches!(
        stage,
        pipeline::PipelineStage::Intermediate(
            intermediate::IntermediateProcessorId::HierarchicalNodeResizer
        )
    )
}

fn should_flush_pending_size_mirrors(pipeline: &[pipeline::PipelineStage], cursor: usize) -> bool {
    cursor > 0 && pipeline[..cursor].iter().copied().any(is_hierarchy_aware_stage)
}

fn mirror_nested_size_to_parent(
    nested_ptr: std::ptr::NonNull<LGraph>,
    parent_ptr: std::ptr::NonNull<LGraph>,
) {
    // SAFETY: `nested_ptr` and `parent_ptr` are distinct graph pointers from
    // the bottom-up collection, each owned by a different `Box` via
    // `set_nested`. The interleaved driver holds no other live borrow here.
    let nested = unsafe { nested_ptr.as_ref() };
    let parent_node_id = nested.parent_node.expect("nested graph must have parent_node");
    let actual_size = Vec2 {
        x: nested.size.x + nested.padding.left + nested.padding.right,
        y: nested.size.y + nested.padding.top + nested.padding.bottom,
    };
    // Only switch the parent compound to `FIXED_POS` when the nested graph
    // actually contains `EXTERNAL_PORT` dummies. The graph-properties cache
    // flags any nested graph as carrying EXTERNAL_PORTS so early gates fire
    // correctly, so we count dummies directly here.
    let has_ext_dummies = nested.nodes_iter().any(|(_id, n)| n.node_type == NodeType::ExternalPort);
    let mut port_writes: Vec<(PortId, Vec2, PortSide)> = Vec::new();
    collect_external_port_writes(nested, &mut port_writes);
    let parent = unsafe { &mut *parent_ptr.as_ptr() };
    for (pid, pos, side) in port_writes {
        let port = parent.port_mut(pid);
        port.position = pos;
        port.side = side;
    }
    if has_ext_dummies {
        parent.node_mut(parent_node_id).node_port_constraints = Some(PortConstraints::FixedPos);
        let mut gp = parent.properties.get(&GRAPH_PROPERTIES);
        gp.insert(GraphProperties::NON_FREE_PORTS);
        parent.properties.set(&GRAPH_PROPERTIES, gp);
    }
    // EXTERNAL_PORTS branch: keep ports pinned (ext-port positions are
    // already authoritative). Otherwise ports move with the resize.
    resize_parent_from_nested(parent, parent_node_id, actual_size, !has_ext_dummies);
}

/// Separate-hierarchy layout: connected components split + per-component
/// pipeline + combine. Used whenever `HIERARCHY_HANDLING != Include`.
fn begin_separate_layout(graph: &mut LGraph) {
    let root_ptr = graph as *const LGraph;
    apply_hierarchical_import_minimum_sizes(graph, root_ptr);
}

fn finish_separate_layout(graph: &mut LGraph) {
    // Lay out every nested graph bottom-up before the top-level pipeline
    // runs. Nested children are laid out first so their final sizes are
    // known when the preprocessor reads them during hierarchical edge
    // splitting.
    //
    // Collect each nested graph's final `actual_size` as we recurse; after
    // the iterator drops we write the sizes back to the enclosing parent
    // nodes. Arenas are not shared across hierarchy levels, so a
    // processor running inside a nested pipeline cannot reach the parent's
    // arena — the write-back has to live here.
    //
    // External-port dummies update the parent port position but do not
    // overwrite the parent port side; the outer pipeline decides any
    // side change later.
    let mut port_writes: Vec<(PortId, Vec2, PortSide)> = Vec::new();
    let mut nested_meta: Vec<(NodeId, Vec2, bool)> = Vec::new();
    for (node_id, nested) in graph.nested_graphs_mut() {
        let has_ext = nested.nodes_iter().any(|(_id, n)| n.node_type == NodeType::ExternalPort);
        let actual_size = nested.size;
        collect_external_port_writes_after_padding(nested, &mut port_writes);
        nested_meta.push((node_id, actual_size, has_ext));
    }
    for (parent_port_id, new_pos, _side) in port_writes {
        let port = graph.port_mut(parent_port_id);
        port.position = new_pos;
    }
    for &(node_id, _, has_ext) in &nested_meta {
        if has_ext {
            graph.node_mut(node_id).node_port_constraints = Some(PortConstraints::FixedPos);
            let mut gp = graph.properties.get(&GRAPH_PROPERTIES);
            gp.insert(GraphProperties::NON_FREE_PORTS);
            graph.properties.set(&GRAPH_PROPERTIES, gp);
        }
    }
    for (node_id, new_size, has_ext) in nested_meta {
        resize_parent_from_nested(graph, node_id, new_size, !has_ext);
    }

    // The flat-import shape carries no parent-attached nested LGraphs.
    // Nested graphs are kept around here for hierarchy write-back
    // bookkeeping, so P3 must explicitly ignore them on this path.
    graph.properties.set(&P3_IGNORE_NESTED_GRAPHS, true);

    prepare_graph_for_layout(graph);

    // Split into connected components, run the pipeline on each, then
    // combine. `components::split` returns an empty Vec when the gate
    // disallows splitting (compound graph, external ports with order-fixed
    // constraints, or `separate_connected_components` disabled), in
    // which case we fall back to running the pipeline directly.
    let components = components::split(graph);
    if components.is_empty() {
        run_pipeline(graph);
    } else {
        // Components must share a single RNG state across the split:
        // c1 consumes the state c0 left behind, c2 sees c1's, and so on.
        // `extract_component_graphs` clones properties (so each component
        // would otherwise start at a fresh `SeededRng::new(1)`), so we
        // thread the parent's RNG through `take_rng` / `put_rng`. Without
        // this, any phase that pulls from `LGraph::rng` (LayerSweep,
        // GreedyCycleBreaker, OrthogonalRoutingGenerator, etc.) diverges
        // on multi-component fixtures.
        let mut shared_rng = graph.take_rng();
        let mut laid_out: Vec<LGraph> = Vec::with_capacity(components.len());
        for mut component in components {
            component.put_rng(shared_rng);
            run_pipeline(&mut component);
            shared_rng = component.take_rng();
            laid_out.push(component);
        }
        graph.put_rng(shared_rng);
        components::combine(laid_out, graph);
    }

    intermediate::hierarchical_node_resizer::resize_graph(graph);
    apply_layout_padding(graph);
}

/// Translate every node / edge / edge-label position by `graph.offset +
/// (padding.left, padding.top)`, grow `graph.size` to the full bounding
/// box, and zero out `graph.offset`.
///
/// `LayerSizeAndGraphHeightCalculator` writes `graph.offset.y -= minY` so
/// the topmost node sits at y=0; this function adds the padding on top of
/// that offset and applies the combined offset to every node, port dummy,
/// and edge segment. Compound layout applies this to the whole nested
/// hierarchy through `apply_layout_padding_to_hierarchy`.
fn apply_layout_padding(graph: &mut LGraph) {
    let pad = graph.padding;
    let off = graph.offset;
    let off_x = off.x + pad.left;
    let off_y = off.y + pad.top;
    if off_x == 0.0 && off_y == 0.0 {
        return;
    }

    use std::collections::HashSet;
    let mut edge_label_ids: HashSet<graph::index::LabelId> = HashSet::new();
    let edge_ids: Vec<graph::index::EdgeId> = graph.edges_iter().map(|(eid, _)| eid).collect();
    for &eid in &edge_ids {
        for &lid in &graph.edge(eid).labels {
            edge_label_ids.insert(lid);
        }
    }

    for (_id, node) in graph.nodes_iter_mut() {
        node.position.x += off_x;
        node.position.y += off_y;
    }

    for eid in edge_ids {
        let edge = graph.edge_mut(eid);
        for bp in edge.bend_points.iter_mut() {
            bp.x += off_x;
            bp.y += off_y;
        }
        if let Some(start) = edge.start_point.as_mut() {
            start.x += off_x;
            start.y += off_y;
        }
        if let Some(end) = edge.end_point.as_mut() {
            end.x += off_x;
            end.y += off_y;
        }
    }

    let label_ids: Vec<graph::index::LabelId> = edge_label_ids.into_iter().collect();
    for lid in label_ids {
        let label = graph.label_mut(lid);
        label.position.x += off_x;
        label.position.y += off_y;
    }

    graph.size.x += pad.left + pad.right;
    graph.size.y += pad.top + pad.bottom;
    // The accumulated offset has been baked into the node / edge / label
    // positions; reset so a second call (or a nested layout pass) does
    // not re-apply it.
    graph.offset = Vec2::ZERO;
}

/// Seed `GraphProperties::*` flags on the graph by inspecting the final
/// node / port / edge state. There is no central importer step, so the
/// inference happens at the top of `layout()` before the pipeline
/// configurator reads the flags.
///
/// Preserves any flags already set (e.g. EXTERNAL_PORTS flagged by the
/// compound graph preprocessor, PARTITIONS set elsewhere).
fn cache_graph_properties(graph: &mut LGraph) {
    let mut props = graph.properties.get(&GRAPH_PROPERTIES);

    // Scan nodes: comments, hypernodes, external ports, N/S ports,
    // non-free port constraints. EXTERNAL_PORTS is set only when a
    // `NodeType::ExternalPort` dummy exists in the graph. The compound
    // preprocessor is the canonical setter; nested graphs without actual
    // hierarchical-port edges must keep the flag false so downstream
    // gates (self-loop hierarchy mode, post-resizer FIXED_POS lock,
    // HierarchicalPortOrthogonalEdgeRouter) behave correctly.
    let node_ids: Vec<NodeId> = graph
        .layerless_nodes
        .iter()
        .copied()
        .chain(graph.layers.iter().flat_map(|l| l.nodes.iter().copied()))
        .collect();
    for nid in &node_ids {
        let node = graph.node(*nid);
        if node.properties.get(&COMMENT_BOX) {
            props.insert(GraphProperties::COMMENTS);
        }
        if node.properties.get(&HYPERNODE) {
            props.insert(GraphProperties::HYPERNODES);
            props.insert(GraphProperties::HYPEREDGES);
        }
        if node.node_type == NodeType::ExternalPort {
            props.insert(GraphProperties::EXTERNAL_PORTS);
        }
        let pc = node.port_constraints();
        if !matches!(pc, PortConstraints::Undefined | PortConstraints::Free) {
            props.insert(GraphProperties::NON_FREE_PORTS);
        }
        for &port_id in &node.ports {
            let port = graph.port(port_id);
            if matches!(port.side, PortSide::North | PortSide::South) {
                props.insert(GraphProperties::NORTH_SOUTH_PORTS);
            }
            // >1 incoming or outgoing on a single port counts as a
            // hyperedge.
            if port.incoming_edges.len() > 1 || port.outgoing_edges.len() > 1 {
                props.insert(GraphProperties::HYPEREDGES);
            }
        }
    }

    // Scan edges: self-loops and center/end labels.
    for (_eid, edge) in graph.edges_iter() {
        // Cross-hierarchy proxy edges (added by `compound_graph::preprocess`
        // via `add_edge_orphan` to keep a root-level handle for the
        // postprocessor) carry raw `PortId` values from nested arenas. The
        // arena-tag check on `LGraph::port` rejects these, so we skip
        // them — they are pure bookkeeping and contribute no self-loops
        // or labels at this level.
        let Some(src_port) = graph.try_port(edge.source) else { continue };
        let Some(tgt_port) = graph.try_port(edge.target) else { continue };
        let src_node = src_port.owner;
        let tgt_node = tgt_port.owner;
        if src_node == tgt_node {
            props.insert(GraphProperties::SELF_LOOPS);
        }
        for &lid in &edge.labels {
            let placement = graph.label(lid).properties.get(&EDGE_LABEL_PLACEMENT);
            match placement {
                EdgeLabelPlacement::Center => {
                    props.insert(GraphProperties::CENTER_LABELS);
                }
                EdgeLabelPlacement::Head | EdgeLabelPlacement::Tail => {
                    props.insert(GraphProperties::END_LABELS);
                }
                EdgeLabelPlacement::Undefined => {}
            }
        }
    }

    graph.properties.set(&GRAPH_PROPERTIES, props);
}

/// For every external-port dummy in the nested graph, compute the final
/// port position in the parent's coordinate system and record a
/// `(parent_port_id, position, side)` tuple.
fn collect_external_port_writes(nested: &LGraph, out: &mut Vec<(PortId, Vec2, PortSide)>) {
    let nested_size = nested.size;
    let padding = nested.padding;
    let offset = nested.offset;
    for (_nid, node) in nested.nodes_iter() {
        if node.node_type != NodeType::ExternalPort {
            continue;
        }
        if node.properties.get(&EXT_PORT_REPLACED_DUMMY).is_some() {
            continue;
        }
        let Some(parent_port_id) = node.properties.get(&ORIGIN_PORT) else {
            continue;
        };
        let side = node.properties.get(&EXT_PORT_SIDE);
        let port_offset = node.properties.get(&PORT_BORDER_OFFSET);
        let dummy_pos = node.position;
        let dummy_size = node.size;
        // `portPosition = dummy.position + dummy.size / 2`.
        let mut port_position =
            Vec2 { x: dummy_pos.x + dummy_size.x / 2.0, y: dummy_pos.y + dummy_size.y / 2.0 };
        let port_width = dummy_size.x;
        let port_height = dummy_size.y;
        match side {
            PortSide::North => {
                port_position.x += padding.left + offset.x - port_width / 2.0;
                port_position.y = -port_height - port_offset;
            }
            PortSide::East => {
                port_position.x = nested_size.x + padding.left + padding.right + port_offset;
                port_position.y += padding.top + offset.y - port_height / 2.0;
            }
            PortSide::South => {
                port_position.x += padding.left + offset.x - port_width / 2.0;
                port_position.y = nested_size.y + padding.top + padding.bottom + port_offset;
            }
            PortSide::West => {
                port_position.x = -port_width - port_offset;
                port_position.y += padding.top + offset.y - port_height / 2.0;
            }
            PortSide::Undefined => continue,
        }
        out.push((parent_port_id, port_position, side));
    }
}

/// Compute parent-port write-back from a nested graph after
/// `apply_layout_padding` has already translated node coordinates and folded
/// padding into `nested.size`.
fn collect_external_port_writes_after_padding(
    nested: &LGraph,
    out: &mut Vec<(PortId, Vec2, PortSide)>,
) {
    let nested_size = nested.size;
    let inside_self_loop_nested = nested.properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);
    for (_nid, node) in nested.nodes_iter() {
        if node.node_type != NodeType::ExternalPort {
            continue;
        }
        if node.properties.get(&EXT_PORT_REPLACED_DUMMY).is_some() {
            continue;
        }
        let Some(parent_port_id) = node.properties.get(&ORIGIN_PORT) else {
            continue;
        };
        let side = node.properties.get(&EXT_PORT_SIDE);
        let port_offset = node.properties.get(&PORT_BORDER_OFFSET);
        let dummy_pos = node.position;
        let dummy_size = node.size;
        let port_position = match side {
            PortSide::North => Vec2 {
                x: if inside_self_loop_nested {
                    dummy_pos.x + dummy_size.x / 2.0
                } else {
                    dummy_pos.x
                },
                y: -dummy_size.y - port_offset,
            },
            PortSide::East => Vec2 { x: nested_size.x + port_offset, y: dummy_pos.y },
            PortSide::South => Vec2 {
                x: if inside_self_loop_nested {
                    dummy_pos.x + dummy_size.x / 2.0
                } else {
                    dummy_pos.x
                },
                y: nested_size.y + port_offset,
            },
            PortSide::West => Vec2 { x: -dummy_size.x - port_offset, y: dummy_pos.y },
            PortSide::Undefined => continue,
        };
        out.push((parent_port_id, port_position, side));
    }
}

/// Run the assembled pipeline on a single graph. Extracted so both the
/// direct path and the split / combine path use the same phase sequence.
fn run_pipeline(graph: &mut LGraph) {
    let pipeline = pipeline::configurator::build_pipeline(graph);
    for stage in pipeline {
        stage.run(graph);
    }
}

/// Write the newly computed nested-graph size back to its enclosing
/// composite node.
///
/// When `move_ports` is true, ports are shifted along their side
/// (east/south translate, north/south scale horizontally, east/west scale
/// vertically unless pinned by `FIXED_POS`); when false, port positions
/// are left untouched (the EXTERNAL_PORTS path passes `false` because
/// ext-port positions are already authoritative). Labels near the
/// right/bottom edges are translated proportionally.
fn resize_parent_from_nested(
    graph: &mut LGraph,
    node_id: NodeId,
    new_size: Vec2,
    move_ports: bool,
) {
    if move_ports && graph.node(node_id).properties.get(&NODE_SIZE_FIXED_GRAPH_SIZE) {
        return;
    }

    let old_size = graph.node(node_id).size;
    let preserve_explicit_minimum = {
        let node = graph.node(node_id);
        let constraints = node.properties.get(&NODE_SIZE_CONSTRAINTS);
        let minimum = node.properties.get(&NODE_SIZE_MINIMUM);
        constraints.contains(SizeConstraint::MINIMUM_SIZE)
            && ((minimum.x > 0.0 && minimum.x > new_size.x)
                || (minimum.y > 0.0 && minimum.y > new_size.y))
    };
    let width_diff = new_size.x - old_size.x;
    let height_diff = new_size.y - old_size.y;
    let width_ratio = if old_size.x > 0.0 { new_size.x / old_size.x } else { 1.0 };
    let height_ratio = if old_size.y > 0.0 { new_size.y / old_size.y } else { 1.0 };

    if move_ports {
        let fixed_ports = graph.node(node_id).port_constraints() == PortConstraints::FixedPos;

        let port_ids: smallvec::SmallVec<graph::index::PortId, 6> =
            graph.node(node_id).ports.iter().copied().collect();
        for port_id in port_ids {
            let side = graph.port(port_id).side;
            let port = graph.port_mut(port_id);
            match side {
                PortSide::North =>
                    if !fixed_ports {
                        port.position.x *= width_ratio;
                    },
                PortSide::East => {
                    port.position.x += width_diff;
                    if !fixed_ports {
                        port.position.y *= height_ratio;
                    }
                }
                PortSide::South => {
                    if !fixed_ports {
                        port.position.x *= width_ratio;
                    }
                    port.position.y += height_diff;
                }
                PortSide::West =>
                    if !fixed_ports {
                        port.position.y *= height_ratio;
                    },
                PortSide::Undefined => {}
            }
        }
    }

    if old_size.x > 0.0 && old_size.y > 0.0 {
        let label_ids: smallvec::SmallVec<graph::index::LabelId, 2> =
            graph.node(node_id).labels.iter().copied().collect();
        for label_id in label_ids {
            let label = graph.label(label_id);
            let midx = label.position.x + label.size.x / 2.0;
            let midy = label.position.y + label.size.y / 2.0;
            let width_pct = midx / old_size.x;
            let height_pct = midy / old_size.y;
            if width_pct + height_pct >= 1.0 {
                if width_pct - height_pct > 0.0 && midy >= 0.0 {
                    let label = graph.label_mut(label_id);
                    label.position.x += width_diff;
                    label.position.y += height_diff * height_pct;
                } else if width_pct - height_pct < 0.0 && midx >= 0.0 {
                    let label = graph.label_mut(label_id);
                    label.position.x += width_diff * width_pct;
                    label.position.y += height_diff;
                }
            }
        }
    }

    let node = graph.node_mut(node_id);
    node.size.x = new_size.x;
    node.size.y = new_size.y;
    if !preserve_explicit_minimum {
        node.properties.set(&NODE_SIZE_CONSTRAINTS, SizeConstraint::empty());
    }
}

/// Mirror `graph.options.ordering_strategy != None` into the graph-level
/// `CONSIDER_MODEL_ORDER_STRATEGY` property so that downstream gates have
/// a single source of truth.
fn sync_ordering_strategy_property(graph: &mut LGraph) {
    use options::enums::OrderingStrategy;
    if graph.options.ordering_strategy != OrderingStrategy::None {
        graph.properties.set(&properties::internal::CONSIDER_MODEL_ORDER_STRATEGY, true);
    }
}

/// Whether the current layout options require per-node `MODEL_ORDER`
/// values.
fn needs_model_order(graph: &LGraph) -> bool {
    use options::enums::{
        ComponentOrderingStrategy, CycleBreakingStrategy, LayeringStrategy, NodePromotionStrategy,
        OrderingStrategy,
    };
    let model_order_cycle_breaking = matches!(
        graph.options.cycle_breaking,
        CycleBreakingStrategy::ModelOrder
            | CycleBreakingStrategy::BfsNodeOrder
            | CycleBreakingStrategy::DfsNodeOrder
            | CycleBreakingStrategy::GreedyModelOrder
            | CycleBreakingStrategy::SccConnectivity
            | CycleBreakingStrategy::SccNodeType
    );
    let model_order_layering = matches!(
        graph.options.layering,
        LayeringStrategy::BfModelOrder | LayeringStrategy::DfModelOrder
    ) || matches!(
        graph.options.node_promotion,
        NodePromotionStrategy::ModelOrderLeftToRight | NodePromotionStrategy::ModelOrderRightToLeft
    );
    let model_order_crossing_minimization = graph.options.ordering_strategy
        != OrderingStrategy::None
        || graph.options.crossing_minimization_force_node_model_order
        || graph.options.consider_model_order_crossing_counter_node_influence != 0.0
        || graph.options.consider_model_order_crossing_counter_port_influence != 0.0;
    let model_order_components =
        graph.options.consider_model_order_components != ComponentOrderingStrategy::None;
    model_order_cycle_breaking
        || model_order_layering
        || model_order_crossing_minimization
        || model_order_components
}

/// Assign sequential `MODEL_ORDER` values to all nodes (and edges) based
/// on their insertion order, and update `MAX_MODEL_ORDER_NODES` and
/// `CB_NUM_MODEL_ORDER_GROUPS` on the graph.
///
/// Edges get a separate 0-based index. `SortByInputModelProcessor` and
/// `ModelOrderNodeComparator` both rely on `edge.MODEL_ORDER` when
/// sorting nodes that have no previous-layer connection.
fn assign_model_order_from_insertion(graph: &mut LGraph) {
    use std::collections::HashSet;
    let n = graph.layerless_nodes.len();
    let mut cb_groups: HashSet<i32> = HashSet::new();
    for i in 0..n {
        let node_id = graph.layerless_nodes[i];
        let node = graph.node_mut(node_id);
        node.properties.set(&properties::internal::MODEL_ORDER, i as i32);
        if node.properties.has(&properties::internal::CB_CYCLE_BREAKING_ID) {
            let id = node.properties.get(&properties::internal::CB_CYCLE_BREAKING_ID);
            cb_groups.insert(id);
        }
    }
    graph.properties.set(&properties::internal::MAX_MODEL_ORDER_NODES, n as i32);
    graph
        .properties
        .set(&properties::internal::CB_NUM_MODEL_ORDER_GROUPS, cb_groups.len() as i32);

    // Edges get a 0-based MODEL_ORDER following their insertion order.
    // Without this, `getModelOrderFromConnectedEdges` reads the
    // property's default (-1) for every edge, collapsing the
    // dummy-vs-normal ordering decision.
    let edge_ids: Vec<crate::graph::index::EdgeId> = graph.edges_iter().map(|(id, _)| id).collect();
    for (i, edge_id) in edge_ids.into_iter().enumerate() {
        graph
            .edge_mut(edge_id)
            .properties
            .set(&properties::internal::MODEL_ORDER, i as i32);
    }
}

/// Bookkeeping for one inside-self-loop edge that was moved into a virtual
/// nested LGraph during layout setup. Used by
/// [`apply_inside_self_loop_writeback`] to copy the routed bend points back
/// onto the original outer-graph edge after the outer pipeline finishes.
#[derive(Clone, Copy, Debug)]
struct InsideLoopEdgeMeta {
    /// Stable id of the compound node owning the nested LGraph.
    compound_stable_id: u32,
    /// The compound node owning the nested LGraph that holds the twin edge.
    compound: NodeId,
    /// Edge id in the outer (compound's parent) LGraph.
    original_edge: EdgeId,
    /// Edge id in the nested LGraph before any connected-component split.
    twin_edge: EdgeId,
    /// Source port's source-order index on the compound node.
    source_port_index: u32,
    /// Target port's source-order index on the compound node.
    target_port_index: u32,
    /// Outer source port (re-attached during writeback).
    source_port: PortId,
    /// Outer target port (re-attached during writeback).
    target_port: PortId,
}

/// Find every leaf node (nodes without a pre-built nested LGraph) that
/// has `INSIDE_SELF_LOOPS_ACTIVATE=true` and at least one outgoing
/// self-loop edge with `INSIDE_SELF_LOOPS_YO=true`, materialise a virtual
/// nested LGraph for it, and return the list of qualifying compound
/// NodeIds.
///
/// Compound nodes that already have a nested LGraph because they have actual
/// children are
/// returned in the list so [`move_inside_self_loops_into_nested`] still
/// processes their inside-self-loops, but their existing nested LGraph
/// is preserved.
fn materialize_inside_self_loop_nested_for_leaves(graph: &mut LGraph) -> Vec<NodeId> {
    let candidates: Vec<NodeId> = collect_inside_self_loop_compounds(graph);
    let mut compounds = Vec::with_capacity(candidates.len());
    for node_id in candidates {
        if !graph.has_nested(node_id) {
            let mut inner = build_inner_lgraph_for_compound(graph, node_id);
            seed_import_minimum_size_for_inner(graph, node_id, &mut inner);
            graph.set_nested(node_id, inner);
        }
        compounds.push(node_id);
    }
    compounds
}

/// Walk the LGraph's nodes (both `layerless_nodes` and any already-layered
/// nodes) and return those whose properties + outgoing edges qualify for
/// inside-self-loop processing.
fn collect_inside_self_loop_compounds(graph: &LGraph) -> Vec<NodeId> {
    let mut out = Vec::new();
    let visit = |nid: NodeId, out: &mut Vec<NodeId>| {
        let n = graph.node(nid);
        if n.node_type != NodeType::Normal {
            return;
        }
        if !n.properties.get(&INSIDE_SELF_LOOPS_ACTIVATE) {
            return;
        }
        for &pid in &n.ports {
            for &eid in &graph.port(pid).outgoing_edges {
                let edge = graph.edge(eid);
                let tgt_owner = graph.port(edge.target).owner;
                if tgt_owner == nid && edge.properties.get(&INSIDE_SELF_LOOPS_YO) {
                    out.push(nid);
                    return;
                }
            }
        }
    };
    for &nid in &graph.layerless_nodes {
        visit(nid, &mut out);
    }
    for layer in &graph.layers {
        for &nid in &layer.nodes {
            visit(nid, &mut out);
        }
    }
    out
}

/// Build a fresh LGraph that will host the virtual layout for
/// `compound`'s inside-self-loop processing.
///
/// Every layout option configured on the parent flows to the inner
/// LGraph: clone the parent's `LayoutOptions` as the inner-LGraph
/// default, then override `port_constraints` from the compound's
/// `NODE_PORT_CONSTRAINTS` when set; source importers store node-level
/// overrides there as a per-element fallback.
fn build_inner_lgraph_for_compound(parent_graph: &LGraph, compound: NodeId) -> LGraph {
    let mut inner = LGraph::new();
    inner.options = parent_graph.options.clone();
    inner.properties = parent_graph.node(compound).properties.clone();

    let node_padding = *parent_graph.node(compound).padding;
    let same_side_inside_loops = inside_self_loops_are_same_fixed_side(parent_graph, compound);
    let mut padding = if parent_graph.parent_node.is_some() || same_side_inside_loops {
        if node_padding != Default::default() {
            node_padding
        } else {
            inner.options.padding
        }
    } else {
        math::Padding {
            top: inner.options.padding.top,
            right: 2.0,
            bottom: inner.options.padding.bottom,
            left: 2.0,
        }
    };
    let label_padding = compute_inside_node_label_padding(parent_graph, compound);
    padding.top += label_padding.top;
    padding.bottom += label_padding.bottom;
    padding.left += label_padding.left;
    padding.right += label_padding.right;
    inner.options.padding = padding;
    inner.padding = padding;

    let pc = parent_graph.node(compound).port_constraints();
    if pc != PortConstraints::Undefined {
        inner.options.port_constraints = pc;
    }
    inner.options.hierarchy_handling = HierarchyHandling::Include;
    inner.properties.set(&GRAPH_PROPERTIES, GraphProperties::empty());
    inner
}

fn inside_self_loops_are_same_fixed_side(parent_graph: &LGraph, compound: NodeId) -> bool {
    let mut saw_inside_loop = false;
    for &src_port in &parent_graph.node(compound).ports {
        for &edge_id in &parent_graph.port(src_port).outgoing_edges {
            let edge = parent_graph.edge(edge_id);
            let target_owner = parent_graph.port(edge.target).owner;
            if target_owner != compound || !edge.properties.get(&INSIDE_SELF_LOOPS_YO) {
                continue;
            }
            saw_inside_loop = true;
            let source_side = parent_graph.port(src_port).side;
            let target_side = parent_graph.port(edge.target).side;
            if source_side == PortSide::Undefined
                || target_side == PortSide::Undefined
                || source_side != target_side
            {
                return false;
            }
        }
    }
    saw_inside_loop
}

fn seed_import_minimum_size_for_inner(
    parent_graph: &mut LGraph,
    compound: NodeId,
    inner: &mut LGraph,
) {
    let node_constraints = parent_graph.node(compound).properties.get(&NODE_SIZE_CONSTRAINTS);
    let inner_constraints = inner.properties.get(&NODE_SIZE_CONSTRAINTS);
    if node_constraints.is_empty() && inner_constraints.is_empty() {
        return;
    }

    let root_ptr = parent_graph as *const LGraph;
    ensure_defined_port_sides_for_minimum_size(parent_graph, compound, root_ptr);
    let minimum = intermediate::node_dimension_calculation::calculate_node_minimum_size(
        parent_graph,
        compound,
    );

    let mut constraints = inner.properties.get(&NODE_SIZE_CONSTRAINTS);
    constraints.insert(SizeConstraint::MINIMUM_SIZE);
    inner.properties.set(&NODE_SIZE_CONSTRAINTS, constraints);

    let mut configured_min = inner.properties.get(&NODE_SIZE_MINIMUM);
    configured_min.x = configured_min.x.max(minimum.x);
    configured_min.y = configured_min.y.max(minimum.y);
    inner.properties.set(&NODE_SIZE_MINIMUM, configured_min);
}

/// For every compound returned by
/// [`materialize_inside_self_loop_nested_for_leaves`], move each
/// inside-self-loop edge from the outer LGraph into the compound's
/// nested LGraph as a twin edge between the corresponding EP dummies
/// (created earlier by
/// [`intermediate::compound_graph::install_external_ports_for_separate_hierarchy`]).
///
/// The original edge is retained in the outer LGraph's edge arena so the
/// final dump can rehydrate it during
/// [`apply_inside_self_loop_writeback`], but it is removed from the outer
/// port adjacency lists so no outer-pipeline processor (cycle breaker,
/// self-loop pre-processor, edge router, …) can see it.
fn move_inside_self_loops_into_nested(
    graph: &mut LGraph,
    compounds: &[NodeId],
) -> Vec<InsideLoopEdgeMeta> {
    let mut metas = Vec::new();
    for &compound in compounds {
        let port_ids: smallvec::SmallVec<PortId, 6> =
            graph.node(compound).ports.iter().copied().collect();
        for src_port in port_ids {
            let outgoing: smallvec::SmallVec<EdgeId, 2> =
                graph.port(src_port).outgoing_edges.iter().copied().collect();
            for edge_id in outgoing {
                let tgt_port = graph.edge(edge_id).target;
                let tgt_owner = graph.port(tgt_port).owner;
                let is_self_loop = tgt_owner == compound;
                let is_yo = graph.edge(edge_id).properties.get(&INSIDE_SELF_LOOPS_YO);
                if !(is_self_loop && is_yo) {
                    continue;
                }
                let src_dummy_port = {
                    let nested = graph.nested(compound).expect("nested materialised above");
                    intermediate::compound_graph::find_ep_dummy_port_in_nested(nested, src_port)
                        .expect("EP dummy must exist for inside-self-loop source port")
                };
                let tgt_dummy_port = {
                    let nested = graph.nested(compound).expect("nested materialised above");
                    intermediate::compound_graph::find_ep_dummy_port_in_nested(nested, tgt_port)
                        .expect("EP dummy must exist for inside-self-loop target port")
                };

                let orig_props = graph.edge(edge_id).properties.clone();
                let orig_flags = graph.edge(edge_id).flags;

                let twin_edge = {
                    let nested = graph.nested_mut(compound).expect("nested materialised above");
                    let eid = nested.add_edge(src_dummy_port, tgt_dummy_port);
                    nested.edge_mut(eid).properties = orig_props;
                    nested.edge_mut(eid).flags = orig_flags;
                    // Clear junction points on the freshly imported edge so
                    // source hints do not bleed into the inner pipeline as
                    // initial bend points.
                    nested
                        .edge_mut(eid)
                        .properties
                        .set(&properties::internal::JUNCTION_POINTS, smallvec::SmallVec::new());
                    let mut gp = nested.properties.get(&GRAPH_PROPERTIES);
                    gp.insert(GraphProperties::SELF_LOOPS);
                    nested.properties.set(&GRAPH_PROPERTIES, gp);
                    eid
                };

                graph.port_mut(src_port).outgoing_edges.retain(|&e| e != edge_id);
                graph.port_mut(tgt_port).incoming_edges.retain(|&e| e != edge_id);

                metas.push(InsideLoopEdgeMeta {
                    compound_stable_id: graph.node(compound).id,
                    compound,
                    original_edge: edge_id,
                    twin_edge,
                    source_port_index: graph.port(src_port).original_index,
                    target_port_index: graph.port(tgt_port).original_index,
                    source_port: src_port,
                    target_port: tgt_port,
                });
            }
        }
    }
    metas
}

/// Restore each inside-self-loop edge to its outer-graph port adjacency
/// and copy the routed bend points + endpoints from the corresponding
/// twin edge in the nested LGraph, offsetting by the compound's final
/// outer-graph position.
fn apply_inside_self_loop_writeback(graph: &mut LGraph, metas: &[InsideLoopEdgeMeta]) {
    for meta in metas {
        let compound = resolve_inside_loop_compound(graph, meta).unwrap_or(meta.compound);
        let source_port = resolve_inside_loop_port(graph, compound, meta.source_port_index)
            .unwrap_or(meta.source_port);
        let target_port = resolve_inside_loop_port(graph, compound, meta.target_port_index)
            .unwrap_or(meta.target_port);
        let (twin_bps, twin_start, twin_end, horizontal_padding_delta) = {
            let nested = graph.nested(compound).expect("compound retains nested through layout");
            let edge = nested
                .try_edge(meta.twin_edge)
                .or_else(|| resolve_inside_loop_twin_edge(nested, source_port, target_port))
                .unwrap_or_else(|| {
                    panic!(
                        "inside self-loop twin edge not found for ports {:?} -> {:?}",
                        source_port, target_port
                    )
                });
            let has_north_south_ext_port = nested.nodes_iter().any(|(_node_id, node)| {
                node.node_type == NodeType::ExternalPort
                    && matches!(
                        node.properties.get(&EXT_PORT_SIDE),
                        PortSide::North | PortSide::South
                    )
            });
            let horizontal_padding_delta =
                if nested.properties.get(&INSIDE_SELF_LOOPS_ACTIVATE) && has_north_south_ext_port {
                    graph.padding.left - nested.padding.left
                } else {
                    0.0
                };
            (
                edge.bend_points.clone(),
                edge.start_point,
                edge.end_point,
                horizontal_padding_delta,
            )
        };
        let node_pos = graph.node(compound).position;

        if !graph.port(source_port).outgoing_edges.contains(&meta.original_edge) {
            graph.port_mut(source_port).outgoing_edges.push(meta.original_edge);
        }
        if !graph.port(target_port).incoming_edges.contains(&meta.original_edge) {
            graph.port_mut(target_port).incoming_edges.push(meta.original_edge);
        }

        let source_owner = graph.port_owner(source_port);
        let target_owner = graph.port_owner(target_port);
        let edge = graph.edge_mut(meta.original_edge);
        edge.source = source_port;
        edge.target = target_port;
        edge.source_owner = source_owner;
        edge.target_owner = target_owner;
        edge.bend_points = twin_bps
            .iter()
            .map(|bp| Vec2 {
                x: bp.x + node_pos.x + horizontal_padding_delta,
                y: bp.y + node_pos.y,
            })
            .collect();
        edge.start_point = twin_start
            .map(|p| Vec2 { x: p.x + node_pos.x + horizontal_padding_delta, y: p.y + node_pos.y });
        edge.end_point = twin_end
            .map(|p| Vec2 { x: p.x + node_pos.x + horizontal_padding_delta, y: p.y + node_pos.y });
    }
}

fn resolve_inside_loop_twin_edge(
    nested: &LGraph,
    source_port: PortId,
    target_port: PortId,
) -> Option<&graph::edge::EdgeData> {
    let source_dummy_port =
        intermediate::compound_graph::find_ep_dummy_port_in_nested(nested, source_port)?;
    let target_dummy_port =
        intermediate::compound_graph::find_ep_dummy_port_in_nested(nested, target_port)?;

    nested.edges_iter().find_map(|(_edge_id, edge)| {
        let forward = edge.source == source_dummy_port && edge.target == target_dummy_port;
        let reversed = edge.source == target_dummy_port && edge.target == source_dummy_port;
        (forward || reversed).then_some(edge)
    })
}

fn resolve_inside_loop_compound(graph: &LGraph, meta: &InsideLoopEdgeMeta) -> Option<NodeId> {
    graph
        .nodes_iter()
        .find_map(|(node_id, node)| (node.id == meta.compound_stable_id).then_some(node_id))
}

fn resolve_inside_loop_port(
    graph: &LGraph,
    compound: NodeId,
    original_index: u32,
) -> Option<PortId> {
    graph
        .node(compound)
        .ports
        .iter()
        .copied()
        .find(|&port_id| graph.port(port_id).original_index == original_index)
}
