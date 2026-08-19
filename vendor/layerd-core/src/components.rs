//! Connected-component split / combine for the layered pipeline.
//!
//! Split isolates weakly connected components so the main pipeline runs once
//! per independent layout problem. Combine places each laid-out component
//! into a row-major grid bounded by the target aspect ratio and drains arena
//! contents back into `target` with the correct offset.
//!
//! Gating:
//! - Compound graphs (`parent_node.is_some()` or any node with a nested
//!   graph) are not split.
//! - Graphs carrying external ports with order-fixed port constraints fall
//!   through unsplit; compatible external-port graphs use the component-group
//!   placer.
//! - `LayoutOptions.separate_connected_components` toggles the behaviour
//!   entirely.

use crate::{
    graph::{LGraph, node::NodeType, port::PortSideSet},
    math::Vec2,
    options::enums::{ComponentOrderingStrategy, EdgeRoutingStrategy, HierarchyHandling},
    properties::internal::{EXT_PORT_CONNECTIONS, MODEL_ORDER, PRIORITY},
};

/// Split `graph` into one `LGraph` per weakly connected component, or return
/// an empty `Vec` and leave `graph` untouched when the guard rules say
/// splitting is not safe. Each returned component owns a migrated copy of
/// `graph`'s arenas; `graph` retains only its options / padding / properties,
/// ready for `combine` to re-populate it.
pub fn split(graph: &mut LGraph) -> Vec<LGraph> {
    if !should_split(graph) {
        return Vec::new();
    }
    let component_ordering = graph.options.consider_model_order_components;
    let mut components = graph.extract_component_graphs();
    sort_model_order_components(&mut components, component_ordering);
    components
}

/// Combine previously laid-out components back into `target`.
///
/// - Empty input: reset `target.size` to zero, clear `layerless_nodes`.
/// - Single component: absorb it with zero offset.
/// - Multi-component: preserve model-order component order when requested;
///   otherwise sort by (priority desc, area asc). Components are then fitted
///   into rows whose width is bounded by
///   `max(max_component_width, sqrt(total_area) * aspect_ratio)`, offset, and
///   drained back into `target`.
pub fn combine(mut components: Vec<LGraph>, target: &mut LGraph) {
    target.layerless_nodes.clear();
    if components.is_empty() {
        target.size = Vec2::ZERO;
        return;
    }
    if components_have_external_ports(&components) {
        combine_component_groups(components, target);
        return;
    }
    if components.len() == 1 {
        let mut source = components.pop().expect("len == 1");
        let offset = source.offset;
        target.options = source.options.clone();
        target.padding = source.padding;
        target.properties = source.properties.clone();
        target.size = source.size;
        source.offset = Vec2::ZERO;
        target.absorb_graph(source, offset);
        return;
    }

    if target.options.consider_model_order_components == ComponentOrderingStrategy::None {
        let priorities = compute_priorities(&components);
        sort_components(&mut components, &priorities);
    }

    let first_graph = &components[0];
    target.options = first_graph.options.clone();
    target.padding = first_graph.padding;
    target.properties = first_graph.properties.clone();

    let aspect_ratio = target.options.aspect_ratio;
    let component_spacing = target.options.spacing_component_component;

    let mut max_row_width = 0.0f64;
    let mut total_area = 0.0f64;
    for graph in &components {
        max_row_width = max_row_width.max(graph.size.x);
        total_area += graph.size.x * graph.size.y;
    }
    let area_bound = total_area.max(0.0).sqrt() * aspect_ratio;
    max_row_width = max_row_width.max(area_bound);

    place_row_major(&mut components, max_row_width, component_spacing, target);
    for component in components {
        let offset = component.offset;
        let mut migrated = component;
        migrated.offset = Vec2::ZERO;
        target.absorb_graph(migrated, offset);
    }
}

fn sort_model_order_components(
    components: &mut [LGraph],
    component_ordering: ComponentOrderingStrategy,
) {
    if component_ordering == ComponentOrderingStrategy::None {
        return;
    }
    components.sort_by_key(minimal_model_order);
}

fn minimal_model_order(graph: &LGraph) -> i32 {
    graph
        .layerless_nodes
        .iter()
        .filter_map(|&node_id| {
            let node = graph.node(node_id);
            node.properties.has(&MODEL_ORDER).then(|| node.properties.get(&MODEL_ORDER))
        })
        .min()
        .unwrap_or(i32::MAX)
}

/// Decide whether splitting is safe for this graph.
///
/// Splitting happens whenever `HIERARCHY_HANDLING != INCLUDE_CHILDREN`:
/// nested structure is moved with the NodeData into the per-component
/// LGraph, so flat and compound-aware converters both get the same
/// split/combine treatment. The gates that remain are:
/// - Option opt-out (`separate_connected_components == false`).
/// - `INCLUDE_CHILDREN` mode is not split-safe; when active on a nested
///   LGraph (`parent_node.is_some()`) we cannot split because
///   `hierarchical_layout` interleaves processors across the whole
///   hierarchy with shared cursors.
/// - External ports with order-fixed port constraints; the component-group
///   placer is not implemented yet.
///
/// Nested LGraphs under SEPARATE_CHILDREN *are* split. Without that,
/// isolated external-port dummies (e.g. `ptides_tte_TTE` N1's P5/P7 which
/// have no inner edges) stay glued to the main component and inflate
/// layer-0 height instead of becoming trivial sibling components.
fn should_split(graph: &LGraph) -> bool {
    if !graph.options.separate_connected_components {
        return false;
    }
    if matches!(graph.options.hierarchy_handling, HierarchyHandling::Include)
        && graph.parent_node.is_some()
    {
        return false;
    }
    if has_order_fixed_external_ports(graph) {
        return false;
    }
    true
}

/// True when the graph has external ports and the user's port constraints
/// require the component-group placer, which is not implemented yet. Without
/// that placer, splitting would scramble the external port layout.
///
/// Splitting is allowed when port constraints are not order-fixed (FREE,
/// FIXED_SIDE, or unset/Undefined). For nested LGraphs the constraint lives
/// on the compound node's `NODE_PORT_CONSTRAINTS`, so we walk up to the
/// compound to read it.
///
/// `Undefined` resolves to `FREE` (matches the importer fallback).
fn has_order_fixed_external_ports(graph: &LGraph) -> bool {
    let has_external = !graph.graph_ports.is_empty()
        || graph.nodes_iter().any(|(_, n)| matches!(n.node_type, NodeType::ExternalPort));
    if !has_external {
        return false;
    }
    use crate::options::enums::PortConstraints;
    let pc = effective_port_constraints(graph);
    !matches!(
        pc,
        PortConstraints::Free | PortConstraints::FixedSide | PortConstraints::Undefined,
    )
}

/// Resolve the LGraph's effective port constraints. For root LGraphs the
/// answer is `graph.options.port_constraints`; for nested LGraphs we read
/// the compound node's `NODE_PORT_CONSTRAINTS`, because source importers
/// store `portConstraints: X` from a node body on the node itself, never on
/// the nested LGraph's options.
fn effective_port_constraints(graph: &LGraph) -> crate::options::enums::PortConstraints {
    if graph.options.port_constraints != crate::options::enums::PortConstraints::Undefined {
        return graph.options.port_constraints;
    }
    if let Some(parent) = graph.parent_node
        && let Some(parent_graph) = graph.find_graph_containing(parent)
        && let Some(node) = parent_graph.try_node(parent)
    {
        let pc = node.port_constraints();
        return pc;
    }
    graph.options.port_constraints
}

/// Sum per-node `PRIORITY` for each component so the sort comparator can
/// read each value by index without re-scanning every component.
///
/// Walks `nodes_iter()` rather than `layerless_nodes` so the sum still
/// reflects the component's nodes after the pipeline has drained them into
/// layers (P2 empties `layerless_nodes` and does not restore it). Dummy
/// nodes added by the pipeline all default to `PRIORITY = 0`, so including
/// them here is harmless.
fn compute_priorities(components: &[LGraph]) -> Vec<i32> {
    components
        .iter()
        .map(|graph| {
            graph
                .nodes_iter()
                .map(|(_, node)| node.properties.get(&PRIORITY))
                .fold(0i32, i32::saturating_add)
        })
        .collect()
}

/// Sort components by (priority desc, area asc) when model-order component
/// ordering is disabled.
fn sort_components(components: &mut [LGraph], priorities: &[i32]) {
    // `indices` holds the sorted permutation; we reorder `components` in
    // place using an indirect swap so we do not disturb `priorities`'
    // ordering (which is keyed to the original slot).
    let mut indices: Vec<usize> = (0..components.len()).collect();
    indices.sort_by(|&a, &b| {
        let prio_a = priorities[a];
        let prio_b = priorities[b];
        if prio_b != prio_a {
            return prio_b.cmp(&prio_a);
        }
        let area_a = components[a].size.x * components[a].size.y;
        let area_b = components[b].size.x * components[b].size.y;
        area_a.partial_cmp(&area_b).unwrap_or(std::cmp::Ordering::Equal)
    });
    apply_permutation(components, &indices);
}

/// Rearrange `values` in place according to `indices`, where
/// `indices[i]` is the original slot that should end up at position `i`.
fn apply_permutation<T>(values: &mut [T], indices: &[usize]) {
    debug_assert_eq!(values.len(), indices.len());
    let n = values.len();
    let mut visited = vec![false; n];
    for start in 0..n {
        if visited[start] || indices[start] == start {
            visited[start] = true;
            continue;
        }
        let mut cur = start;
        while !visited[cur] {
            visited[cur] = true;
            let next = indices[cur];
            if next == start {
                break;
            }
            values.swap(cur, next);
            cur = next;
        }
    }
}

/// Assign per-component offsets so they fit inside the bounded row width,
/// and write the target graph's final size.
fn place_row_major(
    components: &mut [LGraph],
    max_row_width: f64,
    component_spacing: f64,
    target: &mut LGraph,
) {
    let mut xpos = 0.0f64;
    let mut ypos = 0.0f64;
    let mut highest_box = 0.0f64;
    let mut broadest_row = component_spacing;

    for graph in components.iter_mut() {
        let size = graph.size;
        if xpos + size.x > max_row_width {
            xpos = 0.0;
            ypos += highest_box + component_spacing;
            highest_box = 0.0;
        }
        // Add the incoming graph offset to `(xpos, ypos)` before applying.
        // Components coming out of `split` have zero offset; the compaction
        // hand-off could set it, but compaction is disabled here
        // (`compaction_connected_components == false` by default).
        graph.offset.x += xpos;
        graph.offset.y += ypos;
        broadest_row = broadest_row.max(xpos + size.x);
        highest_box = highest_box.max(size.y);
        xpos += size.x + component_spacing;
    }

    target.size = Vec2 { x: broadest_row, y: ypos + highest_box };
}

fn components_have_external_ports(components: &[LGraph]) -> bool {
    components
        .iter()
        .any(|graph| graph.nodes_iter().any(|(_, n)| n.node_type == NodeType::ExternalPort))
}

/// Combine components by external-port group placement.
fn combine_component_groups(components: Vec<LGraph>, target: &mut LGraph) {
    target.layerless_nodes.clear();
    if components.is_empty() {
        target.size = Vec2::ZERO;
        return;
    }

    let first = &components[0];
    target.options = first.options.clone();
    target.padding = first.padding;
    target.properties = first.properties.clone();

    let mut groups: Vec<ComponentGroupPlacement> = Vec::new();
    for component in components {
        add_component_to_groups(component, &mut groups);
    }

    let spacing = target.options.spacing_component_component;
    let mut offset = Vec2::ZERO;
    for group in groups.iter_mut() {
        let group_size = place_component_group(group, spacing);
        group.offset_all(offset);
        offset.x += group_size.x;
        offset.y += group_size.y;
    }

    target.size = Vec2 { x: offset.x - spacing, y: offset.y - spacing };

    if target.options.compaction_connected_components
        && target.options.edge_routing == EdgeRoutingStrategy::Orthogonal
    {
        compact_connected_component_groups(&mut groups, target);
    }

    for group in groups {
        for component in group.into_components() {
            let offset = component.offset;
            let mut migrated = component;
            migrated.offset = Vec2::ZERO;
            target.absorb_graph(migrated, offset);
        }
    }
}

fn compact_connected_component_groups(groups: &mut [ComponentGroupPlacement], target: &mut LGraph) {
    let mut width = 0.0f64;
    for group in groups {
        for component in group.all_components_mut() {
            component.offset.x = 0.0;
            width = width.max(component.size.x);
        }
    }
    target.size.x = width;
}

fn add_component_to_groups(component: LGraph, groups: &mut Vec<ComponentGroupPlacement>) {
    let mut pending = Some(component);
    for group in groups.iter_mut() {
        let component_ref = pending.as_ref().expect("component still pending");
        if group.can_add(component_ref) {
            group.add(pending.take().expect("component still pending"));
            return;
        }
    }
    groups.push(ComponentGroupPlacement::with_component(
        pending.expect("component still pending"),
    ));
}

struct ComponentGroupPlacement {
    none: Vec<LGraph>,
    north: Vec<LGraph>,
    south: Vec<LGraph>,
    west: Vec<LGraph>,
    east: Vec<LGraph>,
    north_west: Vec<LGraph>,
    north_east: Vec<LGraph>,
    south_west: Vec<LGraph>,
    east_south: Vec<LGraph>,
    east_west: Vec<LGraph>,
    north_south: Vec<LGraph>,
    north_east_west: Vec<LGraph>,
    east_south_west: Vec<LGraph>,
    north_south_west: Vec<LGraph>,
    north_east_south: Vec<LGraph>,
    north_east_south_west: Vec<LGraph>,
}

impl ComponentGroupPlacement {
    fn new() -> Self {
        Self {
            none: Vec::new(),
            north: Vec::new(),
            south: Vec::new(),
            west: Vec::new(),
            east: Vec::new(),
            north_west: Vec::new(),
            north_east: Vec::new(),
            south_west: Vec::new(),
            east_south: Vec::new(),
            east_west: Vec::new(),
            north_south: Vec::new(),
            north_east_west: Vec::new(),
            east_south_west: Vec::new(),
            north_south_west: Vec::new(),
            north_east_south: Vec::new(),
            north_east_south_west: Vec::new(),
        }
    }

    fn with_component(component: LGraph) -> Self {
        let mut group = Self::new();
        group.add(component);
        group
    }

    fn can_add(&self, component: &LGraph) -> bool {
        let candidate = component.properties.get(&EXT_PORT_CONNECTIONS);
        for &constraint in constraints_for(candidate) {
            if !self.components_for(constraint).is_empty() {
                return false;
            }
        }
        true
    }

    fn add(&mut self, component: LGraph) {
        let sides = component.properties.get(&EXT_PORT_CONNECTIONS);
        self.components_for_mut(sides).push(component);
    }

    fn components_for(&self, sides: PortSideSet) -> &[LGraph] {
        use PortSideSet as S;
        if sides == S::SIDES_NONE {
            &self.none
        } else if sides == S::SIDES_NORTH {
            &self.north
        } else if sides == S::SIDES_SOUTH {
            &self.south
        } else if sides == S::SIDES_WEST {
            &self.west
        } else if sides == S::SIDES_EAST {
            &self.east
        } else if sides == S::SIDES_NORTH_WEST {
            &self.north_west
        } else if sides == S::SIDES_NORTH_EAST {
            &self.north_east
        } else if sides == S::SIDES_SOUTH_WEST {
            &self.south_west
        } else if sides == S::SIDES_EAST_SOUTH {
            &self.east_south
        } else if sides == S::SIDES_EAST_WEST {
            &self.east_west
        } else if sides == S::SIDES_NORTH_SOUTH {
            &self.north_south
        } else if sides == S::SIDES_NORTH_EAST_WEST {
            &self.north_east_west
        } else if sides == S::SIDES_EAST_SOUTH_WEST {
            &self.east_south_west
        } else if sides == S::SIDES_NORTH_SOUTH_WEST {
            &self.north_south_west
        } else if sides == S::SIDES_NORTH_EAST_SOUTH {
            &self.north_east_south
        } else if sides == S::SIDES_NORTH_EAST_SOUTH_WEST {
            &self.north_east_south_west
        } else {
            &self.none
        }
    }

    fn components_for_mut(&mut self, sides: PortSideSet) -> &mut Vec<LGraph> {
        use PortSideSet as S;
        if sides == S::SIDES_NONE {
            &mut self.none
        } else if sides == S::SIDES_NORTH {
            &mut self.north
        } else if sides == S::SIDES_SOUTH {
            &mut self.south
        } else if sides == S::SIDES_WEST {
            &mut self.west
        } else if sides == S::SIDES_EAST {
            &mut self.east
        } else if sides == S::SIDES_NORTH_WEST {
            &mut self.north_west
        } else if sides == S::SIDES_NORTH_EAST {
            &mut self.north_east
        } else if sides == S::SIDES_SOUTH_WEST {
            &mut self.south_west
        } else if sides == S::SIDES_EAST_SOUTH {
            &mut self.east_south
        } else if sides == S::SIDES_EAST_WEST {
            &mut self.east_west
        } else if sides == S::SIDES_NORTH_SOUTH {
            &mut self.north_south
        } else if sides == S::SIDES_NORTH_EAST_WEST {
            &mut self.north_east_west
        } else if sides == S::SIDES_EAST_SOUTH_WEST {
            &mut self.east_south_west
        } else if sides == S::SIDES_NORTH_SOUTH_WEST {
            &mut self.north_south_west
        } else if sides == S::SIDES_NORTH_EAST_SOUTH {
            &mut self.north_east_south
        } else if sides == S::SIDES_NORTH_EAST_SOUTH_WEST {
            &mut self.north_east_south_west
        } else {
            &mut self.none
        }
    }

    fn offset_all(&mut self, offset: Vec2) {
        for component in self.all_components_mut() {
            offset_graph(component, offset.x, offset.y);
        }
    }

    fn all_components_mut(&mut self) -> impl Iterator<Item = &mut LGraph> {
        self.none
            .iter_mut()
            .chain(self.north.iter_mut())
            .chain(self.south.iter_mut())
            .chain(self.west.iter_mut())
            .chain(self.east.iter_mut())
            .chain(self.north_west.iter_mut())
            .chain(self.north_east.iter_mut())
            .chain(self.south_west.iter_mut())
            .chain(self.east_south.iter_mut())
            .chain(self.east_west.iter_mut())
            .chain(self.north_south.iter_mut())
            .chain(self.north_east_west.iter_mut())
            .chain(self.east_south_west.iter_mut())
            .chain(self.north_south_west.iter_mut())
            .chain(self.north_east_south.iter_mut())
            .chain(self.north_east_south_west.iter_mut())
    }

    fn into_components(self) -> Vec<LGraph> {
        let mut out = Vec::new();
        out.extend(self.none);
        out.extend(self.north);
        out.extend(self.south);
        out.extend(self.west);
        out.extend(self.east);
        out.extend(self.north_west);
        out.extend(self.north_east);
        out.extend(self.south_west);
        out.extend(self.east_south);
        out.extend(self.east_west);
        out.extend(self.north_south);
        out.extend(self.north_east_west);
        out.extend(self.east_south_west);
        out.extend(self.north_south_west);
        out.extend(self.north_east_south);
        out.extend(self.north_east_south_west);
        out
    }
}

fn place_component_group(group: &mut ComponentGroupPlacement, spacing: f64) -> Vec2 {
    use PortSideSet as S;

    let size_c = place_components_in_rows(group.components_for_mut(S::SIDES_NONE), spacing);
    let size_n = place_components_horizontally(group.components_for_mut(S::SIDES_NORTH), spacing);
    let size_s = place_components_horizontally(group.components_for_mut(S::SIDES_SOUTH), spacing);
    let size_w = place_components_vertically(group.components_for_mut(S::SIDES_WEST), spacing);
    let size_e = place_components_vertically(group.components_for_mut(S::SIDES_EAST), spacing);
    let size_nw =
        place_components_horizontally(group.components_for_mut(S::SIDES_NORTH_WEST), spacing);
    let size_ne =
        place_components_horizontally(group.components_for_mut(S::SIDES_NORTH_EAST), spacing);
    let size_sw =
        place_components_horizontally(group.components_for_mut(S::SIDES_SOUTH_WEST), spacing);
    let size_se =
        place_components_horizontally(group.components_for_mut(S::SIDES_EAST_SOUTH), spacing);
    let size_we =
        place_components_vertically(group.components_for_mut(S::SIDES_EAST_WEST), spacing);
    let size_ns =
        place_components_horizontally(group.components_for_mut(S::SIDES_NORTH_SOUTH), spacing);
    let size_nwe =
        place_components_horizontally(group.components_for_mut(S::SIDES_NORTH_EAST_WEST), spacing);
    let size_swe =
        place_components_horizontally(group.components_for_mut(S::SIDES_EAST_SOUTH_WEST), spacing);
    let size_wns =
        place_components_vertically(group.components_for_mut(S::SIDES_NORTH_SOUTH_WEST), spacing);
    let size_ens =
        place_components_vertically(group.components_for_mut(S::SIDES_NORTH_EAST_SOUTH), spacing);
    let size_nesw = place_components_horizontally(
        group.components_for_mut(S::SIDES_NORTH_EAST_SOUTH_WEST),
        spacing,
    );

    let col_left_width = maxd(&[size_nw.x, size_w.x, size_sw.x, size_wns.x]);
    let col_mid_width = maxd(&[size_n.x, size_c.x, size_s.x, size_nesw.x]);
    let col_ns_width = size_ns.x;
    let col_right_width = maxd(&[size_ne.x, size_e.x, size_se.x, size_ens.x]);
    let row_top_height = maxd(&[size_nw.y, size_n.y, size_ne.y, size_nwe.y]);
    let row_mid_height = maxd(&[size_w.y, size_c.y, size_e.y, size_nesw.y]);
    let row_we_height = size_we.y;
    let row_bottom_height = maxd(&[size_sw.y, size_s.y, size_se.y, size_swe.y]);

    offset_graphs(
        group.components_for_mut(S::SIDES_NONE),
        col_left_width + col_ns_width,
        row_top_height + row_we_height,
    );
    offset_graphs(
        group.components_for_mut(S::SIDES_NORTH_EAST_SOUTH_WEST),
        col_left_width + col_ns_width,
        row_top_height + row_we_height,
    );
    offset_graphs(group.components_for_mut(S::SIDES_NORTH), col_left_width + col_ns_width, 0.0);
    offset_graphs(
        group.components_for_mut(S::SIDES_SOUTH),
        col_left_width + col_ns_width,
        row_top_height + row_we_height + row_mid_height,
    );
    offset_graphs(group.components_for_mut(S::SIDES_WEST), 0.0, row_top_height + row_we_height);
    offset_graphs(
        group.components_for_mut(S::SIDES_EAST),
        col_left_width + col_ns_width + col_mid_width,
        row_top_height + row_we_height,
    );
    offset_graphs(
        group.components_for_mut(S::SIDES_NORTH_EAST),
        col_left_width + col_ns_width + col_mid_width,
        0.0,
    );
    offset_graphs(
        group.components_for_mut(S::SIDES_SOUTH_WEST),
        0.0,
        row_top_height + row_we_height + row_mid_height,
    );
    offset_graphs(
        group.components_for_mut(S::SIDES_EAST_SOUTH),
        col_left_width + col_ns_width + col_mid_width,
        row_top_height + row_we_height + row_mid_height,
    );
    offset_graphs(group.components_for_mut(S::SIDES_EAST_WEST), 0.0, row_top_height);
    offset_graphs(group.components_for_mut(S::SIDES_NORTH_SOUTH), col_left_width, 0.0);
    offset_graphs(
        group.components_for_mut(S::SIDES_EAST_SOUTH_WEST),
        0.0,
        row_top_height + row_we_height + row_mid_height,
    );
    offset_graphs(
        group.components_for_mut(S::SIDES_NORTH_EAST_SOUTH),
        col_left_width + col_ns_width + col_mid_width,
        0.0,
    );

    Vec2 {
        x: maxd(&[
            col_left_width + col_mid_width + col_ns_width + col_right_width,
            size_we.x,
            size_nwe.x,
            size_swe.x,
        ]),
        y: maxd(&[
            row_top_height + row_mid_height + row_we_height + row_bottom_height,
            size_ns.y,
            size_wns.y,
            size_ens.y,
        ]),
    }
}

fn place_components_horizontally(components: &mut [LGraph], spacing: f64) -> Vec2 {
    let mut size = Vec2::ZERO;
    for component in components {
        offset_graph(component, size.x, 0.0);
        size.x += component.size.x + spacing;
        size.y = size.y.max(component.size.y);
    }
    if size.y > 0.0 {
        size.y += spacing;
    }
    size
}

fn place_components_vertically(components: &mut [LGraph], spacing: f64) -> Vec2 {
    let mut size = Vec2::ZERO;
    for component in components {
        offset_graph(component, 0.0, size.y);
        size.y += component.size.y + spacing;
        size.x = size.x.max(component.size.x);
    }
    if size.x > 0.0 {
        size.x += spacing;
    }
    size
}

fn place_components_in_rows(components: &mut [LGraph], spacing: f64) -> Vec2 {
    if components.is_empty() {
        return Vec2::ZERO;
    }

    let mut max_row_width = 0.0f64;
    let mut total_area = 0.0f64;
    for component in components.iter() {
        max_row_width = max_row_width.max(component.size.x);
        total_area += component.size.x * component.size.y;
    }
    max_row_width =
        max_row_width.max(total_area.max(0.0).sqrt() * components[0].options.aspect_ratio);

    let mut xpos = 0.0f64;
    let mut ypos = 0.0f64;
    let mut highest_box = 0.0f64;
    let mut broadest_row = spacing;
    for graph in components {
        let size = graph.size;
        if xpos + size.x > max_row_width {
            xpos = 0.0;
            ypos += highest_box + spacing;
            highest_box = 0.0;
        }
        offset_graph(graph, xpos, ypos);
        broadest_row = broadest_row.max(xpos + size.x);
        highest_box = highest_box.max(size.y);
        xpos += size.x + spacing;
    }

    Vec2 { x: broadest_row + spacing, y: ypos + highest_box + spacing }
}

fn offset_graphs(components: &mut [LGraph], x: f64, y: f64) {
    for component in components {
        offset_graph(component, x, y);
    }
}

fn offset_graph(component: &mut LGraph, x: f64, y: f64) {
    component.offset.x += x;
    component.offset.y += y;
}

fn maxd(values: &[f64]) -> f64 {
    values.iter().copied().fold(0.0, f64::max)
}

fn constraints_for(sides: PortSideSet) -> &'static [PortSideSet] {
    use PortSideSet as S;
    if sides == S::SIDES_NONE {
        &[S::SIDES_NORTH_EAST_SOUTH_WEST]
    } else if sides == S::SIDES_WEST {
        &[S::SIDES_NORTH_EAST_SOUTH_WEST, S::SIDES_NORTH_SOUTH_WEST]
    } else if sides == S::SIDES_EAST {
        &[S::SIDES_NORTH_EAST_SOUTH, S::SIDES_NORTH_EAST_SOUTH_WEST]
    } else if sides == S::SIDES_NORTH {
        &[S::SIDES_NORTH_EAST_SOUTH_WEST, S::SIDES_NORTH_EAST_WEST]
    } else if sides == S::SIDES_SOUTH {
        &[S::SIDES_EAST_SOUTH_WEST, S::SIDES_NORTH_EAST_SOUTH_WEST]
    } else if sides == S::SIDES_NORTH_SOUTH {
        &[
            S::SIDES_EAST_WEST,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
            S::SIDES_NORTH_EAST_WEST,
            S::SIDES_EAST_SOUTH_WEST,
        ]
    } else if sides == S::SIDES_EAST_WEST {
        &[
            S::SIDES_NORTH_SOUTH,
            S::SIDES_NORTH_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
        ]
    } else if sides == S::SIDES_NORTH_WEST {
        &[S::SIDES_NORTH_WEST, S::SIDES_NORTH_EAST_WEST, S::SIDES_NORTH_SOUTH_WEST]
    } else if sides == S::SIDES_NORTH_EAST {
        &[S::SIDES_NORTH_EAST, S::SIDES_NORTH_EAST_WEST, S::SIDES_NORTH_EAST_SOUTH]
    } else if sides == S::SIDES_SOUTH_WEST {
        &[S::SIDES_SOUTH_WEST, S::SIDES_EAST_SOUTH_WEST, S::SIDES_NORTH_SOUTH_WEST]
    } else if sides == S::SIDES_EAST_SOUTH {
        &[S::SIDES_EAST_SOUTH, S::SIDES_EAST_SOUTH_WEST, S::SIDES_NORTH_EAST_SOUTH]
    } else if sides == S::SIDES_NORTH_EAST_WEST {
        &[
            S::SIDES_NORTH,
            S::SIDES_NORTH_SOUTH,
            S::SIDES_NORTH_WEST,
            S::SIDES_NORTH_EAST,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
            S::SIDES_NORTH_EAST_WEST,
            S::SIDES_NORTH_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH,
        ]
    } else if sides == S::SIDES_EAST_SOUTH_WEST {
        &[
            S::SIDES_SOUTH,
            S::SIDES_NORTH_SOUTH,
            S::SIDES_SOUTH_WEST,
            S::SIDES_EAST_SOUTH,
            S::SIDES_EAST_SOUTH_WEST,
            S::SIDES_NORTH_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
        ]
    } else if sides == S::SIDES_NORTH_SOUTH_WEST {
        &[
            S::SIDES_WEST,
            S::SIDES_EAST_WEST,
            S::SIDES_NORTH_WEST,
            S::SIDES_SOUTH_WEST,
            S::SIDES_NORTH_EAST_WEST,
            S::SIDES_EAST_SOUTH_WEST,
            S::SIDES_NORTH_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
        ]
    } else if sides == S::SIDES_NORTH_EAST_SOUTH {
        &[
            S::SIDES_EAST,
            S::SIDES_EAST_WEST,
            S::SIDES_NORTH_EAST,
            S::SIDES_EAST_SOUTH,
            S::SIDES_NORTH_EAST_WEST,
            S::SIDES_EAST_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
        ]
    } else if sides == S::SIDES_NORTH_EAST_SOUTH_WEST {
        &[
            S::SIDES_NONE,
            S::SIDES_WEST,
            S::SIDES_EAST,
            S::SIDES_NORTH,
            S::SIDES_SOUTH,
            S::SIDES_NORTH_SOUTH,
            S::SIDES_EAST_WEST,
            S::SIDES_NORTH_EAST_WEST,
            S::SIDES_EAST_SOUTH_WEST,
            S::SIDES_NORTH_SOUTH_WEST,
            S::SIDES_NORTH_EAST_SOUTH,
            S::SIDES_NORTH_EAST_SOUTH_WEST,
        ]
    } else {
        &[]
    }
}
