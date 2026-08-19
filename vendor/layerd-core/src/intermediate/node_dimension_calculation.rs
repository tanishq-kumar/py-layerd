//! Node dimension calculation backed by the cell system.
//!
//! Computes node label bounds, port label bounds, content-area size, and final
//! port positions for the layered pipeline.
//!
//! The public entry point is [`calculate`], called once per pipeline pass, and
//! [`calculate_node`] for driving a single node through the same flow.
//!
//! The module builds, for each node:
//! * an inside 3x3 grid of `LabelCell`s indexed by `NodeLabelLocation`, so
//!   inside labels can be stacked and positioned by horizontal/vertical
//!   alignment;
//! * four `AtomicCell`s for the inside port-label strips (one per side) whose
//!   minimum content area represents the room the ports need along each side;
//! * four optional outside `StripContainerCell`s (NORTH/SOUTH `Strip::Horizontal`,
//!   WEST/EAST `Strip::Vertical`) for outside labels, kept alongside the
//!   main grid the way an `outsideNodeLabelContainers` map would. They are
//!   sized and positioned after the node body, then their label cells write
//!   each label's final coordinate via the cell system. Margins for outside
//!   labels are derived later by `InnermostNodeMarginCalculator` via a
//!   bounding-box union of the node and its outside-label containers.
//!
//! The grid's minimum size drives the node's final size (subject to
//! `SizeConstraint`/`NODE_SIZE_MINIMUM` overrides). Once the final size is
//! known the grid is laid out; label coordinates are copied back onto the
//! graph, and ports are positioned based on the resulting inside-port-label
//! rectangles. Port anchors default to the node side for side-fixed ports,
//! or to the port center when the side is still free.

use smallvec::SmallVec;

use crate::{
    graph::{
        LGraph,
        index::{LabelId, NodeId, PortId},
        node::NodeType,
        port::PortSide,
    },
    math::Vec2,
    nodespacing::cell_system::{
        AtomicCell, CellKind, ContainerArea, GridContainerCell, HorizontalLabelAlignment,
        LabelCell, Rect, Strip, StripContainerCell, VerticalLabelAlignment,
    },
    options::enums::{
        LayoutDirection, NodeLabelPlacement, PortAlignment, PortConstraints, PortLabelPlacement,
        SizeConstraint, SizeOptions,
    },
    properties::internal::{
        INSIDE_CONNECTIONS, INSIDE_SELF_LOOPS_ACTIVATE, NODE_LABEL_PLACEMENT, NODE_LABELS_PADDING,
        NODE_SIZE_CONSTRAINTS, NODE_SIZE_MINIMUM, NODE_SIZE_OPTIONS, PORT_ALIGNMENT_DEFAULT,
        PORT_ALIGNMENT_EAST, PORT_ALIGNMENT_NORTH, PORT_ALIGNMENT_SOUTH, PORT_ALIGNMENT_WEST,
        PORT_ANCHOR, PORT_BORDER_OFFSET, PORT_LABEL_PLACEMENT, SPACING_PORTS_SURROUNDING,
    },
};

/// Entry point invoked by `LabelAndNodeSizeProcessor` for every non-dummy node.
pub fn calculate(graph: &mut LGraph) {
    let ids: Vec<NodeId> = graph.nodes_iter().map(|(id, _)| id).collect();
    for id in ids {
        if graph.node(id).node_type != NodeType::Normal {
            continue;
        }
        calculate_node(graph, id);
    }
}

/// Calculates label cells, node size, and port positions for a single node.
pub fn calculate_node(graph: &mut LGraph, node_id: NodeId) {
    calculate_node_with_options(graph, node_id, false);
}

fn calculate_node_with_options(
    graph: &mut LGraph,
    node_id: NodeId,
    ignore_inside_port_labels: bool,
) {
    let buckets = collect_label_buckets(graph, node_id);
    let port_block = compute_port_block_sizes(graph, node_id, ignore_inside_port_labels);

    let label_node_gap = graph.options.spacing.label_node;
    let label_label_gap = graph.options.spacing.label_label;
    // NODE_LABELS_PADDING uses individual-or-inherited resolution: check
    // SPACING_INDIVIDUAL on the node first, fall back to the parent graph's
    // property when the node has no override. Without the fallback, a graph
    // that sets NODE_LABELS_PADDING expecting nodes to inherit it gets a
    // node read returning the default (zero padding) and the label lands
    // at (0, 0) instead of (left, top).
    let node_labels_padding = if graph.node(node_id).properties.has(&NODE_LABELS_PADDING) {
        graph.node(node_id).properties.get(&NODE_LABELS_PADDING)
    } else {
        graph.properties.get(&NODE_LABELS_PADDING)
    };
    let size_constraints = graph.node(node_id).properties.get(&NODE_SIZE_CONSTRAINTS);
    let size_options = graph.node(node_id).properties.get(&NODE_SIZE_OPTIONS);
    let min_size_property = graph.node(node_id).properties.get(&NODE_SIZE_MINIMUM);

    // Minimum size is honoured only when SizeConstraint::MINIMUM_SIZE is set.
    // With SizeOptions::DEFAULT_MINIMUM_SIZE, fall back to the default min size.
    let explicit_min = if size_constraints.contains(SizeConstraint::MINIMUM_SIZE) {
        let mut m = min_size_property;
        if size_options.contains(SizeOptions::DEFAULT_MINIMUM_SIZE) {
            if m.x <= 0.0 {
                m.x = ELK_DEFAULT_MIN_SIZE.x;
            }
            if m.y <= 0.0 {
                m.y = ELK_DEFAULT_MIN_SIZE.y;
            }
        }
        Some(m)
    } else {
        None
    };

    // When MINIMUM_SIZE_ACCOUNTS_FOR_PADDING is set, the minimum applies to
    // the inside grid's centre cell (the "client area"). Otherwise it floors
    // the whole node.
    let (client_area_min, whole_node_min) = match explicit_min {
        Some(m) if size_options.contains(SizeOptions::MINIMUM_SIZE_ACCOUNTS_FOR_PADDING) =>
            (Some(m), Vec2::ZERO),
        Some(m) => (None, m),
        None => (None, Vec2::ZERO),
    };

    // Symmetry: `!sizeOptions.contains(ASYMMETRICAL)`. The default for
    // `NODE_SIZE_OPTIONS` is `EnumSet.of(DEFAULT_MINIMUM_SIZE)`, so symmetry
    // is on by default.
    let symmetrical = !size_options.contains(SizeOptions::ASYMMETRICAL);
    let tabular_node_labels = size_options.contains(SizeOptions::FORCE_TABULAR_NODE_LABELS);
    let node_labels_contribute = size_constraints.contains(SizeConstraint::NODE_LABELS);
    let center_min_contributes = client_area_min.is_some();
    // When port labels are placed inside, padding around the label cells is
    // `portLabelSpacingVertical` (N/S) or `portLabelSpacingHorizontal` (E/W).
    // Negative values are skipped to avoid growing the node.
    let placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    let inside_padding = if placement.contains(PortLabelPlacement::INSIDE) {
        let v = graph.options.spacing.label_port_vertical.max(0.0);
        let h = graph.options.spacing.label_port_horizontal.max(0.0);
        SidePadding { top: v, bottom: v, left: h, right: h }
    } else {
        SidePadding::default()
    };
    // Derive `horizontalLayoutMode` from the graph direction. UNDEFINED or any
    // horizontal direction -> true; vertical direction (DOWN/UP) -> false.
    // After the internal transpose, vertical-direction graphs are running in
    // horizontally-laid-out form, so the flag passed to the cell system stays
    // original-direction-flavoured: this lets stacking match the original
    // intent once `direction::postprocess` rotates back.
    let direction = graph.options.direction;
    let horizontal_layout_mode = matches!(
        direction,
        LayoutDirection::Right | LayoutDirection::Left | LayoutDirection::Undefined
    );
    let label_cell_spacing = 2.0 * label_label_gap;
    let mut node_container = build_node_container(
        &buckets.inside,
        port_block,
        label_label_gap,
        label_cell_spacing,
        label_node_gap,
        symmetrical,
        tabular_node_labels,
        node_labels_contribute,
        center_min_contributes,
        inside_padding,
        horizontal_layout_mode,
    );
    let mut outside_containers = build_outside_containers(
        &buckets.outside,
        label_label_gap,
        label_cell_spacing,
        label_node_gap,
        symmetrical,
        horizontal_layout_mode,
    );
    if let Some(padding) = node_labels_padding_padding(node_labels_padding)
        && let Some(grid) = inner_inside_grid_mut(&mut node_container)
    {
        grid.base.padding = padding;
    }
    if let Some(client) = client_area_min
        && let Some(grid) = inner_inside_grid_mut(&mut node_container)
    {
        grid.set_center_min_size(client);
        if !node_labels_contribute {
            grid.set_only_center_contributes(true);
        }
    }

    let current_size = graph.node(node_id).size;
    let grid_min = Vec2::new(node_container.min_width(), node_container.min_height());

    // Empty constraints (or only `PORT_LABELS`) leave the node's current size
    // alone. The cell-min derivation is short-circuited in that case, so a
    // node configured without size constraints keeps its caller-supplied
    // width/height even when ports or labels would otherwise demand more
    // space.
    let size_constraints_fixed =
        size_constraints.is_empty() || size_constraints == SizeConstraint::PORT_LABELS;
    let final_size = if size_constraints_fixed {
        // Honour the explicit minimum when one is set, but never grow past
        // the caller-supplied size from the cell system's minima.
        Vec2::new(current_size.x.max(whole_node_min.x), current_size.y.max(whole_node_min.y))
    } else {
        resolve_node_size(current_size, grid_min, whole_node_min)
    };

    {
        let node = graph.node_mut(node_id);
        node.size = final_size;
    }

    // Lay out the outer strip; cascades through the middle row + inside grid
    // so each label cell gets a final rectangle.
    node_container.base.rect = Rect::new(0.0, 0.0, final_size.x, final_size.y);
    node_container.layout_children_horizontally();
    node_container.layout_children_vertically();

    // Position outside containers relative to the node body (using the node's
    // final size) and then layout their children. The four containers are
    // independent of the main node container.
    let outside_overhang = size_options.contains(SizeOptions::OUTSIDE_NODE_LABELS_OVERHANG);
    place_outside_containers(&mut outside_containers, final_size, outside_overhang);

    apply_label_positions(graph, &node_container, &outside_containers);

    // Port placement happens once the node size is final. The effective
    // placement span is the node body; when size constraints are effectively
    // fixed, the cell system may compute larger port-label cells without
    // moving free side ports away from the body span.
    place_ports(graph, node_id, final_size, body_port_side_layouts(final_size));
    place_port_labels(graph, node_id);
    assign_port_anchors(graph, node_id);
}

/// Calculates the size the node/label/port cell system would request without
/// keeping the layout writes that [`calculate_node`] normally applies.
///
/// Used by the hierarchical importer to derive a nested graph's
/// `NODE_SIZE_MINIMUM`. Inside port-label cells are intentionally left out
/// of this import-side lower bound.
pub(crate) fn calculate_node_minimum_size(graph: &mut LGraph, node_id: NodeId) -> Vec2 {
    let original_size = graph.node(node_id).size;
    let label_ids: SmallVec<LabelId, 4> = graph.node(node_id).labels.iter().copied().collect();
    let label_positions: SmallVec<(LabelId, Vec2), 4> =
        label_ids.iter().map(|&id| (id, graph.label(id).position)).collect();
    let port_ids: SmallVec<PortId, 6> = graph.node(node_id).ports.iter().copied().collect();
    let port_state: SmallVec<(PortId, Vec2, Vec2), 6> = port_ids
        .iter()
        .map(|&id| {
            let port = graph.port(id);
            (id, port.position, port.anchor)
        })
        .collect();

    calculate_node_with_options(graph, node_id, true);
    let minimum = graph.node(node_id).size;

    graph.node_mut(node_id).size = original_size;
    for (label_id, position) in label_positions {
        graph.label_mut(label_id).position = position;
    }
    for (port_id, position, anchor) in port_state {
        let port = graph.port_mut(port_id);
        port.position = position;
        port.anchor = anchor;
    }

    minimum
}

// Node-label locations

/// Node-label location enum: 21 valid placements + UNDEFINED.
///
/// Outside locations split between four standalone strip containers (NORTH /
/// SOUTH / WEST / EAST). Inside locations live in a 3x3 grid in the centre of
/// the main node container. Selection is driven by exact-set matching against
/// `NodeLabelPlacement` flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeLabelLocation {
    OutTopLeft,
    OutTopCenter,
    OutTopRight,
    OutBottomLeft,
    OutBottomCenter,
    OutBottomRight,
    OutLeftTop,
    OutLeftCenter,
    OutLeftBottom,
    OutRightTop,
    OutRightCenter,
    OutRightBottom,
    InTopLeft,
    InTopCenter,
    InTopRight,
    InCenterLeft,
    InCenter,
    InCenterRight,
    InBottomLeft,
    InBottomCenter,
    InBottomRight,
    Undefined,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OutsideSide {
    North,
    South,
    West,
    East,
}

impl OutsideSide {
    fn ordinal(self) -> usize {
        match self {
            Self::North => 0,
            Self::South => 1,
            Self::West => 2,
            Self::East => 3,
        }
    }
    const ALL: [OutsideSide; 4] = [Self::North, Self::South, Self::West, Self::East];
}

impl NodeLabelLocation {
    /// Resolves a `NodeLabelPlacement` set to its location via exact-set match.
    /// Each location stores one or two valid flag combinations; the requested
    /// flags are looked up against them exactly. Empty / unrecognised flags
    /// collapse to `Undefined`.
    fn from_placement(flags: NodeLabelPlacement) -> Self {
        use NodeLabelPlacement as P;
        // Input flag-set validation is the caller's responsibility; this is a
        // pure exact lookup against the allowed combinations.
        let priority = P::H_PRIORITY;
        let combos: [(Self, P, bool); 21] = [
            // Outside corners (top / bottom rows): no H_PRIORITY variant.
            (Self::OutTopLeft, P::OUTSIDE | P::V_TOP | P::H_LEFT, false),
            (Self::OutTopRight, P::OUTSIDE | P::V_TOP | P::H_RIGHT, false),
            (Self::OutBottomLeft, P::OUTSIDE | P::V_BOTTOM | P::H_LEFT, false),
            (Self::OutBottomRight, P::OUTSIDE | P::V_BOTTOM | P::H_RIGHT, false),
            // Outside centres (top / bottom): with or without H_PRIORITY.
            (Self::OutTopCenter, P::OUTSIDE | P::V_TOP | P::H_CENTER, true),
            (Self::OutBottomCenter, P::OUTSIDE | P::V_BOTTOM | P::H_CENTER, true),
            // Outside left / right corners: H_PRIORITY required to flip from
            // top/bottom row to left/right column.
            (Self::OutLeftTop, P::OUTSIDE | P::H_LEFT | P::V_TOP | priority, false),
            (Self::OutLeftBottom, P::OUTSIDE | P::H_LEFT | P::V_BOTTOM | priority, false),
            (Self::OutRightTop, P::OUTSIDE | P::H_RIGHT | P::V_TOP | priority, false),
            (Self::OutRightBottom, P::OUTSIDE | P::H_RIGHT | P::V_BOTTOM | priority, false),
            // Outside left / right centres: with or without H_PRIORITY.
            (Self::OutLeftCenter, P::OUTSIDE | P::H_LEFT | P::V_CENTER, true),
            (Self::OutRightCenter, P::OUTSIDE | P::H_RIGHT | P::V_CENTER, true),
            // Inside grid (9 cells): every cell accepts H_PRIORITY too.
            (Self::InTopLeft, P::INSIDE | P::V_TOP | P::H_LEFT, true),
            (Self::InTopCenter, P::INSIDE | P::V_TOP | P::H_CENTER, true),
            (Self::InTopRight, P::INSIDE | P::V_TOP | P::H_RIGHT, true),
            (Self::InCenterLeft, P::INSIDE | P::V_CENTER | P::H_LEFT, true),
            (Self::InCenter, P::INSIDE | P::V_CENTER | P::H_CENTER, true),
            (Self::InCenterRight, P::INSIDE | P::V_CENTER | P::H_RIGHT, true),
            (Self::InBottomLeft, P::INSIDE | P::V_BOTTOM | P::H_LEFT, true),
            (Self::InBottomCenter, P::INSIDE | P::V_BOTTOM | P::H_CENTER, true),
            (Self::InBottomRight, P::INSIDE | P::V_BOTTOM | P::H_RIGHT, true),
        ];
        for &(loc, base, allow_priority) in &combos {
            if flags == base {
                return loc;
            }
            if allow_priority && flags == base | priority {
                return loc;
            }
        }
        Self::Undefined
    }

    fn is_inside(self) -> bool {
        matches!(
            self,
            Self::InTopLeft
                | Self::InTopCenter
                | Self::InTopRight
                | Self::InCenterLeft
                | Self::InCenter
                | Self::InCenterRight
                | Self::InBottomLeft
                | Self::InBottomCenter
                | Self::InBottomRight
        )
    }

    fn outside_side(self) -> Option<OutsideSide> {
        match self {
            Self::OutTopLeft | Self::OutTopCenter | Self::OutTopRight => Some(OutsideSide::North),
            Self::OutBottomLeft | Self::OutBottomCenter | Self::OutBottomRight =>
                Some(OutsideSide::South),
            Self::OutLeftTop | Self::OutLeftCenter | Self::OutLeftBottom => Some(OutsideSide::West),
            Self::OutRightTop | Self::OutRightCenter | Self::OutRightBottom =>
                Some(OutsideSide::East),
            _ => None,
        }
    }

    /// Inside grid `(row, column)`.
    fn inside_grid_position(self) -> Option<(ContainerArea, ContainerArea)> {
        use ContainerArea as A;
        Some(match self {
            Self::InTopLeft => (A::Begin, A::Begin),
            Self::InTopCenter => (A::Begin, A::Center),
            Self::InTopRight => (A::Begin, A::End),
            Self::InCenterLeft => (A::Center, A::Begin),
            Self::InCenter => (A::Center, A::Center),
            Self::InCenterRight => (A::Center, A::End),
            Self::InBottomLeft => (A::End, A::Begin),
            Self::InBottomCenter => (A::End, A::Center),
            Self::InBottomRight => (A::End, A::End),
            _ => return None,
        })
    }

    /// Slot inside the outside container — column for NORTH/SOUTH (`HORIZONTAL`
    /// strip), row for WEST/EAST (`VERTICAL` strip).
    fn outside_slot(self) -> Option<ContainerArea> {
        use ContainerArea as A;
        Some(match self {
            // NORTH / SOUTH containers: pick column (`OUT_T_L` → BEGIN, etc.).
            Self::OutTopLeft | Self::OutBottomLeft => A::Begin,
            Self::OutTopCenter | Self::OutBottomCenter => A::Center,
            Self::OutTopRight | Self::OutBottomRight => A::End,
            // WEST / EAST containers: pick row (`OUT_L_T` → BEGIN, etc.).
            Self::OutLeftTop | Self::OutRightTop => A::Begin,
            Self::OutLeftCenter | Self::OutRightCenter => A::Center,
            Self::OutLeftBottom | Self::OutRightBottom => A::End,
            _ => return None,
        })
    }

    /// Horizontal and vertical alignment for this label location.
    fn alignment(self) -> (HorizontalLabelAlignment, VerticalLabelAlignment) {
        use HorizontalLabelAlignment as H;
        use VerticalLabelAlignment as V;
        match self {
            // Outside top row: vertical alignment is BOTTOM (the label sits
            // above the node, anchored at the cell's bottom).
            Self::OutTopLeft => (H::Left, V::Bottom),
            Self::OutTopCenter => (H::Center, V::Bottom),
            Self::OutTopRight => (H::Right, V::Bottom),
            // Outside bottom row: vertical alignment TOP.
            Self::OutBottomLeft => (H::Left, V::Top),
            Self::OutBottomCenter => (H::Center, V::Top),
            Self::OutBottomRight => (H::Right, V::Top),
            // Outside left column: horizontal alignment RIGHT (label sits to
            // the left of the node, anchored at the cell's right edge).
            Self::OutLeftTop => (H::Right, V::Top),
            Self::OutLeftCenter => (H::Right, V::Center),
            Self::OutLeftBottom => (H::Right, V::Bottom),
            // Outside right column: horizontal alignment LEFT.
            Self::OutRightTop => (H::Left, V::Top),
            Self::OutRightCenter => (H::Left, V::Center),
            Self::OutRightBottom => (H::Left, V::Bottom),
            // Inside grid alignments follow the cell's position in the grid.
            Self::InTopLeft => (H::Left, V::Top),
            Self::InTopCenter => (H::Center, V::Top),
            Self::InTopRight => (H::Right, V::Top),
            Self::InCenterLeft => (H::Left, V::Center),
            Self::InCenter => (H::Center, V::Center),
            Self::InCenterRight => (H::Right, V::Center),
            Self::InBottomLeft => (H::Left, V::Bottom),
            Self::InBottomCenter => (H::Center, V::Bottom),
            Self::InBottomRight => (H::Right, V::Bottom),
            Self::Undefined => (H::Center, V::Center),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct LabelEntry {
    id: LabelId,
    size: Vec2,
    location: NodeLabelLocation,
}

#[derive(Debug, Default)]
struct LabelBuckets {
    inside: Vec<LabelEntry>,
    /// Outside labels keyed by `OutsideSide::ordinal()`.
    outside: [Vec<LabelEntry>; 4],
}

fn collect_label_buckets(graph: &LGraph, node_id: NodeId) -> LabelBuckets {
    // Resolve placement per-label first (`label.has(NODE_LABEL_PLACEMENT)`),
    // falling back to the node's property when the label is silent.
    let node_placement = graph.node(node_id).properties.get(&NODE_LABEL_PLACEMENT);
    let label_ids: Vec<LabelId> = graph.node(node_id).labels.to_vec();
    let mut buckets = LabelBuckets::default();
    for id in label_ids {
        let label_placement = if graph.label(id).properties.has(&NODE_LABEL_PLACEMENT) {
            graph.label(id).properties.get(&NODE_LABEL_PLACEMENT)
        } else {
            node_placement
        };
        let location = NodeLabelLocation::from_placement(label_placement);
        if matches!(location, NodeLabelLocation::Undefined) {
            // Skip labels whose placement does not resolve to any location.
            continue;
        }
        let size = graph.label(id).size;
        let entry = LabelEntry { id, size, location };
        if location.is_inside() {
            buckets.inside.push(entry);
        } else if let Some(side) = location.outside_side() {
            buckets.outside[side.ordinal()].push(entry);
        }
    }
    buckets
}

fn node_labels_padding_padding(padding: crate::math::Padding) -> Option<crate::math::Padding> {
    if padding == crate::math::Padding::default() { None } else { Some(padding) }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<LabelEntry>();
    }
}

// Port block sizes

/// Per-side port-strip extents computed from port and port-label sizes.
///
/// For each side we track a `main` extent (the strip's main axis: width for
/// N/S, height for E/W) and a `cross` extent (cross axis: height for N/S,
/// width for E/W). The cross axis lets the inside port label cells push
/// the node taller (N/S) or wider (E/W) so labels never overflow the node
/// rectangle.
#[derive(Debug, Clone, Copy, Default)]
struct PortBlock {
    north: f64,
    south: f64,
    east: f64,
    west: f64,
    north_cross: f64,
    south_cross: f64,
    east_cross: f64,
    west_cross: f64,
    pad_top: f64,
    pad_right: f64,
    pad_bottom: f64,
    pad_left: f64,
    north_padding: SidePadding,
    south_padding: SidePadding,
    east_padding: SidePadding,
    west_padding: SidePadding,
    include_ports: bool,
    include_port_labels: bool,
    free_port_placement: bool,
}

fn compute_port_block_sizes(
    graph: &LGraph,
    node_id: NodeId,
    ignore_inside_port_labels: bool,
) -> PortBlock {
    let port_spacing = graph.options.spacing.port_port;
    let size_constraints = graph.node(node_id).properties.get(&NODE_SIZE_CONSTRAINTS);
    let size_options = graph.node(node_id).properties.get(&NODE_SIZE_OPTIONS);
    let symmetrical = !size_options.contains(SizeOptions::ASYMMETRICAL);
    let include_ports = size_constraints.contains(SizeConstraint::PORTS);
    let include_port_labels =
        include_ports && size_constraints.contains(SizeConstraint::PORT_LABELS);
    let free_port_placement = !matches!(
        graph.node(node_id).port_constraints(),
        PortConstraints::FixedPos | PortConstraints::FixedRatio
    );
    let default_align =
        resolve_alignment(graph.node(node_id).properties.get(&PORT_ALIGNMENT_DEFAULT));
    let surrounding = if graph.node(node_id).properties.has(&SPACING_PORTS_SURROUNDING) {
        graph.node(node_id).properties.get(&SPACING_PORTS_SURROUNDING)
    } else {
        graph.properties.get(&SPACING_PORTS_SURROUNDING)
    };

    // Read `PORT_LABEL_PLACEMENT` from the node first (Element-level
    // property), falling back to the property's intrinsic default; the
    // `properties.get` accessor already supplies the default when unset.
    let placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    let labels_inside = placement.contains(PortLabelPlacement::INSIDE);
    let ignore_port_label_cells = ignore_inside_port_labels && labels_inside;
    let labels_fixed = placement.is_fixed();
    let label_label = graph.options.spacing.label_label;

    // Per-side cell widths (N/S) and heights (E/W). For non-PORT_LABELS
    // nodes we fall back to the historical `count * port_spacing`
    // shorthand to preserve existing test outcomes.
    let mut north_ports: SmallVec<PortId, 4> = SmallVec::new();
    let mut south_ports: SmallVec<PortId, 4> = SmallVec::new();
    let mut east_ports: SmallVec<PortId, 4> = SmallVec::new();
    let mut west_ports: SmallVec<PortId, 4> = SmallVec::new();

    let mut north_cross = 0.0_f64;
    let mut south_cross = 0.0_f64;
    let mut east_cross = 0.0_f64;
    let mut west_cross = 0.0_f64;
    let mut pad_top = 0.0_f64;
    let mut pad_right = 0.0_f64;
    let mut pad_bottom = 0.0_f64;
    let mut pad_left = 0.0_f64;

    for &pid in &graph.node(node_id).ports {
        let port = graph.port(pid);
        let port_border_offset = port.properties.get(&PORT_BORDER_OFFSET);
        if port_border_offset < 0.0 {
            let inset = -port_border_offset;
            match port.side {
                PortSide::North => pad_top = pad_top.max(inset),
                PortSide::East => pad_right = pad_right.max(inset),
                PortSide::South => pad_bottom = pad_bottom.max(inset),
                PortSide::West => pad_left = pad_left.max(inset),
                PortSide::Undefined => {}
            }
        }
        if labels_fixed {
            // Fixed port labels become node-container padding rather than
            // inside port-label cell content. After reserving negative
            // `PORT_BORDER_OFFSET`, compute the label part that lies inside
            // the node and, under symmetric sizing, apply the same padding
            // to the opposite side with the border offset folded back in.
            let mut minx = f64::INFINITY;
            let mut miny = f64::INFINITY;
            let mut maxx = f64::NEG_INFINITY;
            let mut maxy = f64::NEG_INFINITY;
            let mut have_label = false;
            for &lid in &port.labels {
                let l = graph.label(lid);
                minx = minx.min(l.position.x);
                miny = miny.min(l.position.y);
                maxx = maxx.max(l.position.x + l.size.x);
                maxy = maxy.max(l.position.y + l.size.y);
                have_label = true;
            }
            if have_label {
                let inside_part = match port.side {
                    PortSide::North => (maxy - (port.size.y + port_border_offset)).max(0.0),
                    PortSide::South => (-miny - port_border_offset).max(0.0),
                    PortSide::East => (-minx - port_border_offset).max(0.0),
                    PortSide::West => (maxx - (port.size.x + port_border_offset)).max(0.0),
                    PortSide::Undefined => 0.0,
                };
                match port.side {
                    PortSide::North => {
                        let bigger = inside_part > pad_top;
                        pad_top = pad_top.max(inside_part);
                        if symmetrical && bigger {
                            pad_top = pad_top.max(pad_bottom);
                            pad_bottom = pad_top + port_border_offset;
                        }
                    }
                    PortSide::South => {
                        let bigger = inside_part > pad_bottom;
                        pad_bottom = pad_bottom.max(inside_part);
                        if symmetrical && bigger {
                            pad_bottom = pad_bottom.max(pad_top);
                            pad_top = pad_bottom + port_border_offset;
                        }
                    }
                    PortSide::East => {
                        let bigger = inside_part > pad_right;
                        pad_right = pad_right.max(inside_part);
                        if symmetrical && bigger {
                            pad_right = pad_right.max(pad_left);
                            pad_left = pad_right + port_border_offset;
                        }
                    }
                    PortSide::West => {
                        let bigger = inside_part > pad_left;
                        pad_left = pad_left.max(inside_part);
                        if symmetrical && bigger {
                            pad_left = pad_left.max(pad_right);
                            pad_right = pad_left + port_border_offset;
                        }
                    }
                    PortSide::Undefined => {}
                }
            }
        }
        let label_box = if include_port_labels && !ignore_port_label_cells {
            port_label_box_size(graph, pid, label_label)
        } else {
            Vec2::ZERO
        };
        // Cross-axis contribution from this port:
        // * `INSIDE` labels — the cell needs to host the label (label-axis
        //   size).
        //   Driven by `simpleInsidePortLabelPlacement` (N/S) and
        //   `calculateWidthDueToLabels` (E/W) helpers in this module.
        // * `OUTSIDE` labels contribute nothing to cross-axis size — they
        //   sit outside the node.
        let (cross_n, cross_s, cross_e, cross_w) = if include_port_labels {
            if labels_inside {
                match port.side {
                    PortSide::North => (label_box.y, 0.0, 0.0, 0.0),
                    PortSide::South => (0.0, label_box.y, 0.0, 0.0),
                    PortSide::East => (0.0, 0.0, label_box.x, 0.0),
                    PortSide::West => (0.0, 0.0, 0.0, label_box.x),
                    PortSide::Undefined => (0.0, 0.0, 0.0, 0.0),
                }
            } else {
                (0.0, 0.0, 0.0, 0.0)
            }
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };
        north_cross = north_cross.max(cross_n);
        south_cross = south_cross.max(cross_s);
        east_cross = east_cross.max(cross_e);
        west_cross = west_cross.max(cross_w);
        match port.side {
            PortSide::North => north_ports.push(pid),
            PortSide::South => south_ports.push(pid),
            PortSide::East => east_ports.push(pid),
            PortSide::West => west_ports.push(pid),
            PortSide::Undefined => {}
        }
    }

    let side_cells = |ports: &[PortId], axis: Axis| -> SmallVec<f64, 4> {
        let margins = port_margins_for_side(graph, node_id, ports, axis, include_port_labels);
        ports
            .iter()
            .zip(margins)
            .map(|(&pid, margin)| {
                let port = graph.port(pid);
                let size = match axis {
                    Axis::Horizontal => port.size.x,
                    Axis::Vertical => port.size.y,
                };
                size + margin.before + margin.after
            })
            .collect()
    };

    let north_cells = side_cells(&north_ports, Axis::Horizontal);
    let south_cells = side_cells(&south_ports, Axis::Horizontal);
    let east_cells = side_cells(&east_ports, Axis::Vertical);
    let west_cells = side_cells(&west_ports, Axis::Vertical);

    let label_port_v = graph.options.spacing.label_port_vertical.max(0.0);
    let label_port_h = graph.options.spacing.label_port_horizontal.max(0.0);
    let label_cell_spacing = 2.0 * label_label;

    let north_padding = SidePadding {
        top: label_port_v,
        bottom: 0.0,
        left: if north_ports.is_empty() { 0.0 } else { surrounding.left },
        right: if north_ports.is_empty() { 0.0 } else { surrounding.right },
    };
    let south_padding = SidePadding {
        top: 0.0,
        bottom: label_port_v,
        left: if south_ports.is_empty() { 0.0 } else { surrounding.left },
        right: if south_ports.is_empty() { 0.0 } else { surrounding.right },
    };

    let north_min_height = north_cross + north_padding.top + north_padding.bottom;
    let south_min_height = south_cross + south_padding.top + south_padding.bottom;
    let east_initial_top = if east_ports.is_empty() { 0.0 } else { surrounding.top };
    let east_initial_bottom = if east_ports.is_empty() { 0.0 } else { surrounding.bottom };
    let west_initial_top = if west_ports.is_empty() { 0.0 } else { surrounding.top };
    let west_initial_bottom = if west_ports.is_empty() { 0.0 } else { surrounding.bottom };
    let vertical_top_padding = if free_port_placement {
        (east_initial_top.max(west_initial_top) - (pad_top + north_min_height + label_cell_spacing))
            .max(0.0)
    } else {
        east_initial_top.max(west_initial_top)
    };
    let vertical_bottom_padding = if free_port_placement {
        (east_initial_bottom.max(west_initial_bottom)
            - (pad_bottom + south_min_height + label_cell_spacing))
            .max(0.0)
    } else {
        east_initial_bottom.max(west_initial_bottom)
    };
    let west_padding = SidePadding {
        top: if west_ports.is_empty() { 0.0 } else { vertical_top_padding },
        bottom: if west_ports.is_empty() { 0.0 } else { vertical_bottom_padding },
        left: if west_cross > 0.0 { label_port_h } else { 0.0 },
        right: 0.0,
    };
    let east_padding = SidePadding {
        top: if east_ports.is_empty() { 0.0 } else { vertical_top_padding },
        bottom: if east_ports.is_empty() { 0.0 } else { vertical_bottom_padding },
        left: 0.0,
        right: if east_cross > 0.0 { label_port_h } else { 0.0 },
    };

    // Block extent for one side: the ports themselves are always summed when
    // `SizeConstraint.PORTS` is active, with inter-port gaps between them.
    // Distributed alignment adds one surrounding port-port spacing on both
    // ends.
    let block_size = |cells: &[f64], align: PortAlignment| -> f64 {
        if !include_ports || cells.is_empty() {
            return 0.0;
        }
        let mut size = cells.iter().sum::<f64>() + ((cells.len() - 1) as f64) * port_spacing;
        if matches!(align, PortAlignment::Distributed | PortAlignment::Undefined) {
            size += 2.0 * port_spacing;
        }
        size
    };

    PortBlock {
        north: block_size(
            &north_cells,
            side_alignment(graph, node_id, &PORT_ALIGNMENT_NORTH, default_align),
        ),
        south: block_size(
            &south_cells,
            side_alignment(graph, node_id, &PORT_ALIGNMENT_SOUTH, default_align),
        ),
        east: block_size(
            &east_cells,
            side_alignment(graph, node_id, &PORT_ALIGNMENT_EAST, default_align),
        ),
        west: block_size(
            &west_cells,
            side_alignment(graph, node_id, &PORT_ALIGNMENT_WEST, default_align),
        ),
        north_cross,
        south_cross,
        east_cross,
        west_cross,
        pad_top,
        pad_right,
        pad_bottom,
        pad_left,
        north_padding,
        south_padding,
        east_padding,
        west_padding,
        include_ports,
        include_port_labels,
        free_port_placement,
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct PortAxisMargins {
    before: f64,
    after: f64,
}

fn port_margins_for_side(
    graph: &LGraph,
    node_id: NodeId,
    ports: &[PortId],
    axis: Axis,
    include_port_labels: bool,
) -> SmallVec<PortAxisMargins, 4> {
    let mut margins: SmallVec<PortAxisMargins, 4> = ports
        .iter()
        .map(|&pid| port_margins_for_axis(graph, node_id, pid, axis, include_port_labels))
        .collect();

    let placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    let port_labels_outside = placement.contains(PortLabelPlacement::OUTSIDE);
    if port_labels_outside && !margins.is_empty() {
        margins[0].before = 0.0;
        let last = margins.len() - 1;
        margins[last].after = 0.0;

        let always_same_side = placement.contains(PortLabelPlacement::ALWAYS_SAME_SIDE);
        let always_other_same_side = placement.contains(PortLabelPlacement::ALWAYS_OTHER_SAME_SIDE);
        let space_efficient = placement.contains(PortLabelPlacement::SPACE_EFFICIENT);
        let space_efficient_port_labels =
            !always_same_side && !always_other_same_side && (space_efficient || ports.len() == 2);
        if space_efficient_port_labels && !labels_next_to_port(graph, node_id, ports[0], placement)
        {
            margins[0].after = 0.0;
        }
    }

    let size_options = graph.node(node_id).properties.get(&NODE_SIZE_OPTIONS);
    if size_options.contains(SizeOptions::UNIFORM_PORT_SPACING) && !margins.is_empty() {
        let max_before = margins.iter().map(|m| m.before).fold(0.0_f64, f64::max);
        let max_after = margins.iter().map(|m| m.after).fold(0.0_f64, f64::max);
        for margin in &mut margins {
            margin.before = max_before;
            margin.after = max_after;
        }
        if port_labels_outside {
            margins[0].before = 0.0;
            let last = margins.len() - 1;
            margins[last].after = 0.0;
        }
    }

    margins
}

fn port_label_box_size(graph: &LGraph, port_id: PortId, label_label: f64) -> Vec2 {
    let mut width = 0.0_f64;
    let mut height = 0.0_f64;
    for (count, &label_id) in graph.port(port_id).labels.iter().enumerate() {
        let label = graph.label(label_id);
        width = width.max(label.size.x);
        if count > 0 {
            height += label_label;
        }
        height += label.size.y;
    }
    Vec2::new(width, height)
}

fn port_margins_for_axis(
    graph: &LGraph,
    node_id: NodeId,
    port_id: PortId,
    axis: Axis,
    include_port_labels: bool,
) -> PortAxisMargins {
    if !include_port_labels {
        return PortAxisMargins::default();
    }

    let placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    if placement.is_fixed() {
        return fixed_port_label_margins_for_axis(graph, port_id, axis);
    }

    let label_box = port_label_box_size(graph, port_id, graph.options.spacing.label_label);
    if label_box == Vec2::ZERO {
        return PortAxisMargins::default();
    }

    let port = graph.port(port_id);
    let labels_next_to_port = labels_next_to_port(graph, node_id, port_id, placement);
    match axis {
        Axis::Horizontal =>
            if !labels_next_to_port {
                PortAxisMargins {
                    before: 0.0,
                    after: graph.options.spacing.label_port_horizontal + label_box.x,
                }
            } else if label_box.x > port.size.x {
                let overhang = (label_box.x - port.size.x) / 2.0;
                PortAxisMargins { before: overhang, after: overhang }
            } else {
                PortAxisMargins::default()
            },
        Axis::Vertical =>
            if !labels_next_to_port {
                PortAxisMargins {
                    before: 0.0,
                    after: graph.options.spacing.label_port_vertical + label_box.y,
                }
            } else if label_box.y > port.size.y {
                if graph.options.port_labels_treat_as_group || port.labels.len() == 1 {
                    let overhang = (label_box.y - port.size.y) / 2.0;
                    PortAxisMargins { before: overhang, after: overhang }
                } else {
                    let first_label_height = graph.label(port.labels[0]).size.y;
                    let first_overhang = (first_label_height - port.size.y) / 2.0;
                    PortAxisMargins {
                        before: first_overhang.max(0.0),
                        after: label_box.y - first_overhang - port.size.y,
                    }
                }
            } else {
                PortAxisMargins::default()
            },
    }
}

fn fixed_port_label_margins_for_axis(
    graph: &LGraph,
    port_id: PortId,
    axis: Axis,
) -> PortAxisMargins {
    let port = graph.port(port_id);
    if port.labels.is_empty() {
        return PortAxisMargins::default();
    }

    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for &label_id in &port.labels {
        let label = graph.label(label_id);
        let (start, size) = match axis {
            Axis::Horizontal => (label.position.x, label.size.x),
            Axis::Vertical => (label.position.y, label.size.y),
        };
        min = min.min(start);
        max = max.max(start + size);
    }

    let port_size = match axis {
        Axis::Horizontal => port.size.x,
        Axis::Vertical => port.size.y,
    };
    PortAxisMargins { before: (-min).max(0.0), after: (max - port_size).max(0.0) }
}

fn labels_next_to_port(
    graph: &LGraph,
    node_id: NodeId,
    port_id: PortId,
    placement: PortLabelPlacement,
) -> bool {
    if placement.contains(PortLabelPlacement::INSIDE) {
        let treat_as_compound = graph.has_nested(node_id)
            || graph.node(node_id).properties.get(&INSIDE_SELF_LOOPS_ACTIVATE);
        if treat_as_compound {
            placement.contains(PortLabelPlacement::NEXT_TO_PORT_IF_POSSIBLE)
                && !graph.port(port_id).properties.get(&INSIDE_CONNECTIONS)
        } else {
            true
        }
    } else if placement.contains(PortLabelPlacement::OUTSIDE) {
        placement.contains(PortLabelPlacement::NEXT_TO_PORT_IF_POSSIBLE)
            && !port_has_incident_edges(graph, port_id)
    } else {
        false
    }
}

fn port_has_incident_edges(graph: &LGraph, port_id: PortId) -> bool {
    let port = graph.port(port_id);
    !port.incoming_edges.is_empty() || !port.outgoing_edges.is_empty()
}

// Grid construction

/// Build the inside-only 3x3 grid of node-label cells.
///
/// 9 grid slots, one `LabelCell` per used `NodeLabelLocation`. The grid sits
/// at the CENTER cell of `nodeContainerMiddleRow` (see `build_node_container`).
fn build_inside_grid(
    labels: &[LabelEntry],
    label_label_gap: f64,
    label_cell_spacing: f64,
    symmetrical: bool,
    tabular_node_labels: bool,
    node_labels_contribute: bool,
    horizontal_layout_mode: bool,
) -> GridContainerCell {
    let mut grid = GridContainerCell::new(tabular_node_labels, symmetrical, label_cell_spacing);

    let mut slots: [[Option<LabelCell>; 3]; 3] =
        [[None, None, None], [None, None, None], [None, None, None]];
    for entry in labels {
        let Some((row, col)) = entry.location.inside_grid_position() else {
            continue;
        };
        let (h, v) = entry.location.alignment();
        let slot = &mut slots[row.ordinal()][col.ordinal()];
        let cell = slot.get_or_insert_with(|| {
            let mut cell = LabelCell::with_layout_mode(label_label_gap, horizontal_layout_mode);
            cell.set_horizontal_alignment(h);
            cell.set_vertical_alignment(v);
            cell.base.contributes_to_min_width = node_labels_contribute;
            cell.base.contributes_to_min_height = node_labels_contribute;
            cell
        });
        cell.add_label(entry.id, entry.size);
    }
    for (r_idx, row) in slots.into_iter().enumerate() {
        let row_area = ContainerArea::ALL[r_idx];
        for (c_idx, cell) in row.into_iter().enumerate() {
            if let Some(cell) = cell {
                let col_area = ContainerArea::ALL[c_idx];
                grid.set_cell(row_area, col_area, Some(cell.into()));
            }
        }
    }

    grid
}

/// Build the four outside-label `StripContainerCell`s. Returns
/// `[north, south, west, east]` (indexed by `OutsideSide::ordinal`).
///
/// NORTH/SOUTH containers run as `Strip::Horizontal` with their
/// `padding.bottom` / `padding.top` set to the node-label spacing; WEST/EAST
/// run as `Strip::Vertical` with `padding.right` / `padding.left`. Side-
/// specific contribution flags (height for N/S, width for W/E) restrict each
/// container's contribution to its primary axis.
fn build_outside_containers(
    bucketed: &[Vec<LabelEntry>; 4],
    label_label_gap: f64,
    label_cell_spacing: f64,
    label_node_spacing: f64,
    symmetrical: bool,
    horizontal_layout_mode: bool,
) -> [Option<StripContainerCell>; 4] {
    let mut containers: [Option<StripContainerCell>; 4] = [None, None, None, None];
    for &side in &OutsideSide::ALL {
        let labels = &bucketed[side.ordinal()];
        if labels.is_empty() {
            continue;
        }
        let strip = match side {
            OutsideSide::North | OutsideSide::South => Strip::Horizontal,
            OutsideSide::West | OutsideSide::East => Strip::Vertical,
        };
        let mut container = StripContainerCell::new(strip, symmetrical, label_cell_spacing);
        // Use the container's padding to keep the labels visually away from
        // the node body, so the gap collapses cleanly into the layout when
        // there is no label on a given side.
        match side {
            OutsideSide::North => container.base.padding.bottom = label_node_spacing,
            OutsideSide::South => container.base.padding.top = label_node_spacing,
            OutsideSide::West => container.base.padding.right = label_node_spacing,
            OutsideSide::East => container.base.padding.left = label_node_spacing,
        }
        // Outside containers are sized using `getMinimumWidth/Height`; the
        // cell-system gates that on `contributes_to_min_*`, so flag the
        // container itself and each child cell on the side-appropriate axis.
        container.base.contributes_to_min_width = true;
        container.base.contributes_to_min_height = true;
        // Group labels by slot and build one LabelCell per used slot.
        let mut slot_cells: [Option<LabelCell>; 3] = [None, None, None];
        for entry in labels {
            let Some(slot) = entry.location.outside_slot() else {
                continue;
            };
            let (h, v) = entry.location.alignment();
            let cell = slot_cells[slot.ordinal()].get_or_insert_with(|| {
                let mut cell = LabelCell::with_layout_mode(label_label_gap, horizontal_layout_mode);
                cell.set_horizontal_alignment(h);
                cell.set_vertical_alignment(v);
                match side {
                    OutsideSide::North | OutsideSide::South => {
                        cell.base.contributes_to_min_height = true;
                    }
                    OutsideSide::West | OutsideSide::East => {
                        cell.base.contributes_to_min_width = true;
                    }
                }
                cell
            });
            cell.add_label(entry.id, entry.size);
        }
        for (idx, slot_cell) in slot_cells.into_iter().enumerate() {
            if let Some(cell) = slot_cell {
                container.set_cell(ContainerArea::ALL[idx], Some(cell.into()));
            }
        }
        containers[side.ordinal()] = Some(container);
    }
    containers
}

/// Position the four outside containers around the final node rectangle.
///
/// Each container takes its minimum size, optionally clamps to the node size
/// when `OUTSIDE_NODE_LABELS_OVERHANG` is off, and then sits flush against
/// the corresponding node side at negative coordinates (NORTH/WEST) or just
/// past the node size (SOUTH/EAST).
fn place_outside_containers(
    containers: &mut [Option<StripContainerCell>; 4],
    node_size: Vec2,
    overhang: bool,
) {
    for &side in &OutsideSide::ALL {
        let Some(container) = containers[side.ordinal()].as_mut() else {
            continue;
        };
        let mut width = container.min_width();
        let mut height = container.min_height();
        match side {
            OutsideSide::North | OutsideSide::South => {
                width = width.max(node_size.x);
                if width > node_size.x && !overhang {
                    width = node_size.x;
                }
            }
            OutsideSide::West | OutsideSide::East => {
                height = height.max(node_size.y);
                if height > node_size.y && !overhang {
                    height = node_size.y;
                }
            }
        }
        let (x, y) = match side {
            OutsideSide::North => (-(width - node_size.x) / 2.0, -height),
            OutsideSide::South => (-(width - node_size.x) / 2.0, node_size.y),
            OutsideSide::West => (-width, -(height - node_size.y) / 2.0),
            OutsideSide::East => (node_size.x, -(height - node_size.y) / 2.0),
        };
        container.base.rect = Rect::new(x, y, width, height);
        container.layout_children_horizontally();
        container.layout_children_vertically();
    }
}

/// Build the outer node container with full nested-strip structure:
///
/// - The outer container is a vertical strip.
/// - The BEGIN slot stores the NORTH inside-port-label cell.
/// - The CENTER slot stores a horizontal strip containing WEST, the inside
///   3x3 label grid, and EAST.
/// - The END slot stores the SOUTH inside-port-label cell.
///
/// Each `AtomicCell` carries the per-side minimum content area required to
/// fit the side's ports (port_count x port_spacing). Strip cascading then
/// makes them contribute to outer min width/height — instead of a
/// single-grid simplification that collapses the four sides into a single
/// centre cell.
fn build_node_container(
    labels: &[LabelEntry],
    ports: PortBlock,
    label_label_gap: f64,
    label_cell_spacing: f64,
    _label_node_gap: f64,
    symmetrical: bool,
    tabular_node_labels: bool,
    node_labels_contribute: bool,
    center_min_contributes: bool,
    inside_padding: SidePadding,
    horizontal_layout_mode: bool,
) -> StripContainerCell {
    let inside_grid = build_inside_grid(
        labels,
        label_label_gap,
        label_cell_spacing,
        symmetrical,
        tabular_node_labels,
        node_labels_contribute,
        horizontal_layout_mode,
    );

    let mut middle_row = StripContainerCell::new(Strip::Horizontal, symmetrical, 0.0);
    middle_row.base.contributes_to_min_width = (ports.include_ports && ports.include_port_labels)
        || node_labels_contribute
        || center_min_contributes;
    middle_row.base.contributes_to_min_height = (ports.include_ports && ports.free_port_placement)
        || node_labels_contribute
        || center_min_contributes;

    // E/W cells carry vertical surrounding-port padding. For free port
    // placement the algorithm later subtracts the N/S port-label cells'
    // heights from that padding; `compute_port_block_sizes` stores the
    // adjusted values.
    let mut west_padding = ports.west_padding;
    let mut east_padding = ports.east_padding;
    west_padding.left = west_padding.left.max(inside_padding.left);
    east_padding.right = east_padding.right.max(inside_padding.right);
    let mut west_cell = port_label_cell(Vec2::new(ports.west_cross, ports.west), west_padding);
    west_cell.base.contributes_to_min_width = ports.include_ports && ports.include_port_labels;
    west_cell.base.contributes_to_min_height = ports.include_ports && ports.free_port_placement;
    let mut east_cell = port_label_cell(Vec2::new(ports.east_cross, ports.east), east_padding);
    east_cell.base.contributes_to_min_width = ports.include_ports && ports.include_port_labels;
    east_cell.base.contributes_to_min_height = ports.include_ports && ports.free_port_placement;
    middle_row.set_cell(ContainerArea::Begin, Some(west_cell.into()));

    let mut grid_kind: CellKind = inside_grid.into();
    grid_kind.base_mut().contributes_to_min_width =
        node_labels_contribute || center_min_contributes;
    grid_kind.base_mut().contributes_to_min_height =
        node_labels_contribute || center_min_contributes;
    middle_row.set_cell(ContainerArea::Center, Some(grid_kind));
    middle_row.set_cell(ContainerArea::End, Some(east_cell.into()));

    let mut outer = StripContainerCell::new(Strip::Vertical, symmetrical, 0.0);
    outer.base.contributes_to_min_width = true;
    outer.base.contributes_to_min_height = true;
    // Reserve node-container padding for ports that extend into the node
    // via negative `PORT_BORDER_OFFSET`.
    outer.base.padding.top = ports.pad_top;
    outer.base.padding.right = ports.pad_right;
    outer.base.padding.bottom = ports.pad_bottom;
    outer.base.padding.left = ports.pad_left;

    // N/S cells keep label-port padding for the vertical-padding update,
    // but under plain PORTS constraints they contribute width only.
    let mut north_padding = ports.north_padding;
    let mut south_padding = ports.south_padding;
    north_padding.top = north_padding.top.max(inside_padding.top);
    south_padding.bottom = south_padding.bottom.max(inside_padding.bottom);
    let mut north_cell = port_label_cell(Vec2::new(ports.north, ports.north_cross), north_padding);
    north_cell.base.contributes_to_min_width = ports.include_ports;
    north_cell.base.contributes_to_min_height = ports.include_ports && ports.include_port_labels;
    let mut south_cell = port_label_cell(Vec2::new(ports.south, ports.south_cross), south_padding);
    south_cell.base.contributes_to_min_width = ports.include_ports;
    south_cell.base.contributes_to_min_height = ports.include_ports && ports.include_port_labels;
    outer.set_cell(ContainerArea::Begin, Some(north_cell.into()));
    outer.set_cell(ContainerArea::Center, Some(middle_row.into()));
    outer.set_cell(ContainerArea::End, Some(south_cell.into()));

    outer
}

#[derive(Debug, Clone, Copy, Default)]
struct SidePadding {
    top: f64,
    bottom: f64,
    left: f64,
    right: f64,
}

fn port_label_cell(min_size: Vec2, padding: SidePadding) -> AtomicCell {
    let mut cell = AtomicCell::new();
    cell.set_min_content_area_size(min_size, false);
    cell.base.padding.top = padding.top;
    cell.base.padding.bottom = padding.bottom;
    cell.base.padding.left = padding.left;
    cell.base.padding.right = padding.right;
    cell.base.contributes_to_min_width = true;
    cell.base.contributes_to_min_height = true;
    cell
}

/// Walk the outer node container's nesting (Vertical → CENTER → Horizontal
/// → CENTER) to reach the inside-grid `GridContainerCell` and return a
/// mutable reference. Returns `None` if the structure has been altered
/// unexpectedly (defensive).
fn inner_inside_grid_mut(outer: &mut StripContainerCell) -> Option<&mut GridContainerCell> {
    let middle = outer.cell_mut(ContainerArea::Center)?;
    let CellKind::Strip(middle_strip) = middle else { return None };
    let centre = middle_strip.cell_mut(ContainerArea::Center)?;
    if let CellKind::Grid(grid) = centre { Some(&mut **grid) } else { None }
}

fn inner_inside_grid(outer: &StripContainerCell) -> Option<&GridContainerCell> {
    let middle = outer.cell(ContainerArea::Center)?;
    let CellKind::Strip(middle_strip) = middle else { return None };
    let centre = middle_strip.cell(ContainerArea::Center)?;
    if let CellKind::Grid(grid) = centre { Some(&**grid) } else { None }
}

// Node-size resolution

/// Default node minimum size, applied when `SizeOptions::DEFAULT_MINIMUM_SIZE`
/// is set and the stored value is non-positive.
const ELK_DEFAULT_MIN_SIZE: Vec2 = Vec2 { x: 20.0, y: 20.0 };

fn resolve_node_size(current: Vec2, grid_min: Vec2, whole_node_min: Vec2) -> Vec2 {
    let effective_min =
        Vec2::new(grid_min.x.max(whole_node_min.x), grid_min.y.max(whole_node_min.y));
    // Overwrite node size with the cell-system computed minimum when
    // constraints are not fixed. Fall back to current only when the cell
    // system says nothing about that axis (effective_min == 0).
    let final_x = if effective_min.x > 0.0 { effective_min.x } else { current.x };
    let final_y = if effective_min.y > 0.0 { effective_min.y } else { current.y };
    Vec2::new(final_x, final_y)
}

// Writing label positions back

/// Walks the inside grid and the four outside containers, copying each label
/// cell's computed coordinate back onto the graph's label arena.
fn apply_label_positions(
    graph: &mut LGraph,
    node_container: &StripContainerCell,
    outside: &[Option<StripContainerCell>; 4],
) {
    if let Some(grid) = inner_inside_grid(node_container) {
        for row in ContainerArea::ALL {
            for column in ContainerArea::ALL {
                let Some(cell) = grid.cell(row, column) else { continue };
                if let CellKind::Label(label_cell) = cell {
                    for (label_id, position) in label_cell.apply_label_layout() {
                        graph.label_mut(label_id).position = position;
                    }
                }
            }
        }
    }
    for slot in outside {
        let Some(container) = slot.as_ref() else { continue };
        for area in ContainerArea::ALL {
            let Some(cell) = container.cell(area) else { continue };
            if let CellKind::Label(label_cell) = cell {
                for (label_id, position) in label_cell.apply_label_layout() {
                    graph.label_mut(label_id).position = position;
                }
            }
        }
    }
}

// Port placement

fn place_ports(
    graph: &mut LGraph,
    node_id: NodeId,
    node_size: Vec2,
    port_layouts: PortSideLayouts,
) {
    let constraints = graph.node(node_id).port_constraints();
    let port_spacing = graph.options.spacing.port_port;

    match constraints {
        PortConstraints::FixedPos => apply_fixed_pos_positions(graph, node_id, node_size),
        PortConstraints::FixedRatio => apply_fixed_ratio_positions(graph, node_id, node_size),
        _ => distribute_ports_on_sides(graph, node_id, node_size, port_spacing, port_layouts),
    }
}

fn place_port_labels(graph: &mut LGraph, node_id: NodeId) {
    let placement = graph.node(node_id).properties.get(&PORT_LABEL_PLACEMENT);
    if placement.is_fixed() {
        return;
    }
    let inside = placement.contains(PortLabelPlacement::INSIDE);
    let outside = placement.contains(PortLabelPlacement::OUTSIDE);
    if !inside && !outside {
        return;
    }
    let first_outside_label_other_side = if outside {
        first_outside_label_other_side_ports(graph, node_id, placement)
    } else {
        SmallVec::new()
    };
    let always_other_same_side = placement.contains(PortLabelPlacement::ALWAYS_OTHER_SAME_SIDE);

    // The port-label border offset includes the node-container padding
    // introduced by negative port border offsets plus port-label spacing.
    let mut pad_top = 0.0_f64;
    let mut pad_right = 0.0_f64;
    let mut pad_bottom = 0.0_f64;
    let mut pad_left = 0.0_f64;
    for &pid in &graph.node(node_id).ports {
        let port = graph.port(pid);
        let border = port.properties.get(&PORT_BORDER_OFFSET);
        if border < 0.0 {
            let inset = -border;
            match port.side {
                PortSide::North => pad_top = pad_top.max(inset),
                PortSide::East => pad_right = pad_right.max(inset),
                PortSide::South => pad_bottom = pad_bottom.max(inset),
                PortSide::West => pad_left = pad_left.max(inset),
                PortSide::Undefined => {}
            }
        }
    }

    let port_ids: SmallVec<PortId, 8> = graph.node(node_id).ports.iter().copied().collect();
    for pid in port_ids {
        let label_ids: SmallVec<LabelId, 4> = graph.port(pid).labels.iter().copied().collect();
        if label_ids.is_empty() {
            continue;
        }

        let port = graph.port(pid);
        let side = port.side;
        let port_size = port.size;
        let border = port.properties.get(&PORT_BORDER_OFFSET);
        let next_to_port = labels_next_to_port(graph, node_id, pid, placement);
        let outside_other_side = outside
            && !next_to_port
            && (always_other_same_side || first_outside_label_other_side.contains(&pid));

        let mut box_w = 0.0_f64;
        let mut box_h = 0.0_f64;
        for (idx, &lid) in label_ids.iter().enumerate() {
            let label = graph.label(lid);
            box_w = box_w.max(label.size.x);
            if idx > 0 {
                box_h += graph.options.spacing.label_label;
            }
            box_h += label.size.y;
        }

        let label_h_for_center = if graph.options.port_labels_treat_as_group {
            box_h
        } else {
            graph.label(label_ids[0]).size.y
        };
        let h_spacing = graph.options.spacing.label_port_horizontal;
        let v_spacing = graph.options.spacing.label_port_vertical;
        let (box_x, box_y, h_align) = if inside {
            match side {
                PortSide::North => {
                    let x = if next_to_port {
                        (port_size.x - box_w) / 2.0
                    } else {
                        port_size.x + h_spacing
                    };
                    (x, port_size.y + border + pad_top + v_spacing, LabelHAlign::Center)
                }
                PortSide::South => {
                    let x = if next_to_port {
                        (port_size.x - box_w) / 2.0
                    } else {
                        port_size.x + h_spacing
                    };
                    (x, -border - pad_bottom - v_spacing - box_h, LabelHAlign::Center)
                }
                PortSide::East => {
                    let y = if next_to_port {
                        (port_size.y - label_h_for_center) / 2.0
                    } else {
                        port_size.y + v_spacing
                    };
                    (-border - pad_right - h_spacing - box_w, y, LabelHAlign::Right)
                }
                PortSide::West => {
                    let y = if next_to_port {
                        (port_size.y - label_h_for_center) / 2.0
                    } else {
                        port_size.y + v_spacing
                    };
                    (port_size.x + border + pad_left + h_spacing, y, LabelHAlign::Left)
                }
                PortSide::Undefined => continue,
            }
        } else {
            match side {
                PortSide::North => {
                    let x = if next_to_port {
                        (port_size.x - box_w) / 2.0
                    } else if outside_other_side {
                        -box_w - h_spacing
                    } else {
                        port_size.x + h_spacing
                    };
                    (x, -box_h - v_spacing, LabelHAlign::Left)
                }
                PortSide::South => {
                    let x = if next_to_port {
                        (port_size.x - box_w) / 2.0
                    } else if outside_other_side {
                        -box_w - h_spacing
                    } else {
                        port_size.x + h_spacing
                    };
                    (x, port_size.y + v_spacing, LabelHAlign::Left)
                }
                PortSide::East => {
                    let y = if next_to_port {
                        (port_size.y - label_h_for_center) / 2.0
                    } else if outside_other_side {
                        -box_h - v_spacing
                    } else {
                        port_size.y + v_spacing
                    };
                    (port_size.x + h_spacing, y, LabelHAlign::Left)
                }
                PortSide::West => {
                    let y = if next_to_port {
                        (port_size.y - label_h_for_center) / 2.0
                    } else if outside_other_side {
                        -box_h - v_spacing
                    } else {
                        port_size.y + v_spacing
                    };
                    (-box_w - h_spacing, y, LabelHAlign::Right)
                }
                PortSide::Undefined => continue,
            }
        };

        let mut y = box_y;
        for lid in label_ids {
            let size = graph.label(lid).size;
            let x = match h_align {
                LabelHAlign::Left => box_x,
                LabelHAlign::Center => box_x + (box_w - size.x) / 2.0,
                LabelHAlign::Right => box_x + box_w - size.x,
            };
            graph.label_mut(lid).position = Vec2::new(x, y);
            y += size.y + graph.options.spacing.label_label;
        }
    }
}

fn first_outside_label_other_side_ports(
    graph: &LGraph,
    node_id: NodeId,
    placement: PortLabelPlacement,
) -> SmallVec<PortId, 4> {
    let always_same_side = placement.contains(PortLabelPlacement::ALWAYS_SAME_SIDE);
    let space_efficient = placement.contains(PortLabelPlacement::SPACE_EFFICIENT);
    if always_same_side {
        return SmallVec::new();
    }

    let mut result = SmallVec::new();
    for side in [PortSide::North, PortSide::East, PortSide::South, PortSide::West] {
        let mut ports: Vec<PortId> = graph
            .node(node_id)
            .ports
            .iter()
            .copied()
            .filter(|&pid| graph.port(pid).side == side)
            .collect();
        if ports.len() < 2 || !(ports.len() == 2 || space_efficient) {
            continue;
        }
        match side {
            PortSide::North | PortSide::South => ports
                .sort_by(|&a, &b| graph.port(a).position.x.total_cmp(&graph.port(b).position.x)),
            PortSide::East | PortSide::West => ports
                .sort_by(|&a, &b| graph.port(a).position.y.total_cmp(&graph.port(b).position.y)),
            PortSide::Undefined => {}
        }
        let first = ports[0];
        if !labels_next_to_port(graph, node_id, first, placement) {
            result.push(first);
        }
    }

    result
}

#[derive(Debug, Clone, Copy)]
enum LabelHAlign {
    Left,
    Center,
    Right,
}

/// Snap each `FixedPos` port to its node border, preserving the user-supplied
/// along-axis coordinate. SOUTH ports get the `+nodeHeight` shift applied
/// after the snap.
fn apply_fixed_pos_positions(graph: &mut LGraph, node_id: NodeId, node_size: Vec2) {
    let port_ids: Vec<PortId> = graph.node(node_id).ports.to_vec();
    for pid in port_ids {
        let port = graph.port(pid);
        let side = port.side;
        let offset = port.properties.get(&PORT_BORDER_OFFSET);
        let port_size = port.size;
        let stored = port.position;
        let new_pos = match side {
            // calculateHorizontalPortYCoordinate: NORTH -> -size.y - offset.
            PortSide::North => Vec2::new(stored.x, -port_size.y - offset),
            // calculateHorizontalPortYCoordinate: SOUTH -> offset; then
            // offsetSouthernPortsByNodeSize adds nodeHeight.
            PortSide::South => Vec2::new(stored.x, node_size.y + offset),
            // calculateVerticalPortXCoordinate: EAST -> nodeWidth + offset.
            PortSide::East => Vec2::new(node_size.x + offset, stored.y),
            // calculateVerticalPortXCoordinate: WEST -> -size.x - offset.
            PortSide::West => Vec2::new(-port_size.x - offset, stored.y),
            PortSide::Undefined => stored,
        };
        graph.port_mut(pid).position = new_pos;
    }
}

fn distribute_ports_on_sides(
    graph: &mut LGraph,
    node_id: NodeId,
    node_size: Vec2,
    port_spacing: f64,
    port_layouts: PortSideLayouts,
) {
    let port_ids: Vec<PortId> = graph.node(node_id).ports.to_vec();
    let mut north: SmallVec<PortId, 4> = SmallVec::new();
    let mut south: SmallVec<PortId, 4> = SmallVec::new();
    let mut east: SmallVec<PortId, 4> = SmallVec::new();
    let mut west: SmallVec<PortId, 4> = SmallVec::new();
    for pid in port_ids {
        match graph.port(pid).side {
            PortSide::North => north.push(pid),
            PortSide::South => south.push(pid),
            PortSide::East => east.push(pid),
            PortSide::West => west.push(pid),
            PortSide::Undefined => {}
        }
    }

    let size_options = graph.node(node_id).properties.get(&NODE_SIZE_OPTIONS);
    let overhang = size_options.contains(SizeOptions::PORTS_OVERHANG);
    let default_align =
        resolve_alignment(graph.node(node_id).properties.get(&PORT_ALIGNMENT_DEFAULT));

    let north_align = side_alignment(graph, node_id, &PORT_ALIGNMENT_NORTH, default_align);
    let south_align = side_alignment(graph, node_id, &PORT_ALIGNMENT_SOUTH, default_align);
    let east_align = side_alignment(graph, node_id, &PORT_ALIGNMENT_EAST, default_align);
    let west_align = side_alignment(graph, node_id, &PORT_ALIGNMENT_WEST, default_align);

    // Per-side cross-axis port coordinates:
    //   WEST  -> -port.size.x - offset
    //   EAST  -> nodeWidth + offset
    //   NORTH -> -port.size.y - offset
    //   SOUTH -> nodeHeight + offset after the southern-size shift.
    // `offset` is zero when `PORT_BORDER_OFFSET` is not explicitly set.
    distribute_along(
        graph,
        node_id,
        north.as_slice(),
        port_layouts.north,
        port_spacing,
        north_align,
        overhang,
        Axis::Horizontal,
        false,
        |pos, port_sz, offset| Vec2::new(pos, -port_sz.y - offset),
    );
    distribute_along(
        graph,
        node_id,
        south.as_slice(),
        port_layouts.south,
        port_spacing,
        south_align,
        overhang,
        Axis::Horizontal,
        // SOUTH iterates the port-context list by volatileId descending,
        // where volatileId is assigned in `LNode.ports` order. The net effect
        // is that horizontal-free placement walks the SOUTH segment of the
        // node's port list in reverse.
        true,
        |pos, _port_sz, offset| Vec2::new(pos, node_size.y + offset),
    );
    distribute_along(
        graph,
        node_id,
        east.as_slice(),
        port_layouts.east,
        port_spacing,
        east_align,
        overhang,
        Axis::Vertical,
        false,
        |pos, _port_sz, offset| Vec2::new(node_size.x + offset, pos),
    );
    distribute_along(
        graph,
        node_id,
        west.as_slice(),
        port_layouts.west,
        port_spacing,
        west_align,
        overhang,
        Axis::Vertical,
        true,
        |pos, port_sz, offset| Vec2::new(-port_sz.x - offset, pos),
    );
}

#[derive(Debug, Clone, Copy)]
enum Axis {
    Horizontal,
    Vertical,
}

#[derive(Debug, Clone, Copy)]
struct PortSideLayouts {
    north: PortSideLayout,
    south: PortSideLayout,
    east: PortSideLayout,
    west: PortSideLayout,
}

#[derive(Debug, Clone, Copy, Default)]
struct PortSideLayout {
    start: f64,
    length: f64,
    padding_before: f64,
    padding_after: f64,
}

fn body_port_side_layouts(node_size: Vec2) -> PortSideLayouts {
    let horizontal =
        PortSideLayout { start: 0.0, length: node_size.x, padding_before: 0.0, padding_after: 0.0 };
    let vertical =
        PortSideLayout { start: 0.0, length: node_size.y, padding_before: 0.0, padding_after: 0.0 };
    PortSideLayouts { north: horizontal, south: horizontal, east: vertical, west: vertical }
}

/// Resolves the per-side `PortAlignment` override by reading the side-specific
/// property and substituting the node-level default when unset.
fn side_alignment(
    graph: &LGraph,
    node_id: NodeId,
    key: &crate::properties::PropertyKey<PortAlignment>,
    default_align: PortAlignment,
) -> PortAlignment {
    let v = graph.node(node_id).properties.get(key);
    if matches!(v, PortAlignment::Undefined) { default_align } else { v }
}

/// Resolves the node-level `PORT_ALIGNMENT_DEFAULT` value, substituting
/// `Distributed` if it is left at `Undefined`.
fn resolve_alignment(value: PortAlignment) -> PortAlignment {
    if matches!(value, PortAlignment::Undefined) {
        PortAlignment::Distributed
    } else {
        value
    }
}

/// Distributes ports along a single side of the node according to the
/// requested `PortAlignment` and `PORTS_OVERHANG` flag.
///
/// When the row would not fit and overhang is disabled, `gap` becomes the
/// squeezed value `(axis_length - sum_widths) / (n - 1)` and no alignment
/// offset is applied.
#[allow(clippy::too_many_arguments)]
fn distribute_along(
    graph: &mut LGraph,
    node_id: NodeId,
    ports: &[PortId],
    side_layout: PortSideLayout,
    port_spacing: f64,
    align: PortAlignment,
    overhang: bool,
    axis: Axis,
    reverse: bool,
    make_pos: impl Fn(f64, Vec2, f64) -> Vec2,
) {
    let n = ports.len();
    if n == 0 {
        return;
    }

    // The node side is sorted by `PortListSorter` clockwise on N/E and
    // counter-clockwise on S/W; the `reverse` flag re-orders the W side so
    // placement always proceeds from low coordinate to high.
    let order: SmallVec<PortId, 4> = if reverse {
        ports.iter().rev().copied().collect()
    } else {
        ports.iter().copied().collect()
    };
    let sizes: SmallVec<Vec2, 4> = order.iter().map(|&pid| graph.port(pid).size).collect();
    let offsets: SmallVec<f64, 4> = order
        .iter()
        .map(|&pid| graph.port(pid).properties.get(&PORT_BORDER_OFFSET))
        .collect();
    let size_constraints = graph.node(node_id).properties.get(&NODE_SIZE_CONSTRAINTS);
    let include_port_labels = size_constraints.contains(SizeConstraint::PORT_LABELS);
    let margins = port_margins_for_side(graph, node_id, &order, axis, include_port_labels);
    let widths: SmallVec<f64, 4> = sizes
        .iter()
        .zip(margins.iter())
        .map(|(s, margin)| match axis {
            Axis::Horizontal => s.x + margin.before + margin.after,
            Axis::Vertical => s.y + margin.before + margin.after,
        })
        .collect();
    let sum_widths: f64 = widths.iter().sum();
    // When there's exactly one port and the requested alignment is
    // Distributed or Justified, fall back to Center, and for Distributed
    // strip the two surrounding gaps that the cell-minimum formula adds.
    // Without this, an EAST/WEST side carrying a single port yields a
    // `calculated = size + 2 * port_spacing` that overflows the cell and
    // the fallback path picks `current = port_spacing` instead of
    // `(axis - size) / 2`, leaving the port off-centre — and that small
    // shift propagates through `find_port_diff` into BK `inner_shift` and
    // pulls every dependent block down by the same amount.
    let mut effective_align = align;
    if n == 1 && matches!(align, PortAlignment::Distributed | PortAlignment::Justified) {
        effective_align = PortAlignment::Center;
    }
    // Cell minimum content area along the side: the base size sums port
    // sizes plus `(n-1)` inter-port gaps, and `Distributed` alignment
    // additionally adds two surrounding gaps (one before the first port,
    // one after the last) so ports never sit on the cell boundary.
    let base_calc = sum_widths + (n.saturating_sub(1) as f64) * port_spacing;
    // The `calculated` size is the cell-minimum the layout engine reserved
    // for this port row. Compute it from the *original* alignment so the
    // surrounding-gap addition for Distributed cells survives the n==1 ->
    // Center fallback above, except that Distributed strips the two added
    // gaps after the fallback. Justified doesn't get the gap addition in
    // the first place, so the fallback leaves it untouched.
    let mut calculated = match align {
        PortAlignment::Distributed | PortAlignment::Undefined => base_calc + 2.0 * port_spacing,
        _ => base_calc,
    };
    if n == 1 && matches!(align, PortAlignment::Distributed) {
        calculated -= 2.0 * port_spacing;
    }
    let align = effective_align;
    let axis_start = side_layout.start + side_layout.padding_before;
    let raw_axis_length =
        side_layout.length - side_layout.padding_before - side_layout.padding_after;
    let axis_length = if n == 1 { raw_axis_length.max(calculated) } else { raw_axis_length };

    let (current_offset, gap) = if calculated > axis_length && !overhang && n >= 2 {
        // Overflow + !PORTS_OVERHANG path. The squeeze redistribution depends
        // on alignment because each alignment owns a different `calculated`
        // and a different "where do we start" offset. `Distributed` consumes
        // the two surrounding gaps it added to `calculated` and bumps the
        // cursor one bumped gap in; every other alignment squeezes only the
        // inter-port gaps.
        match align {
            PortAlignment::Distributed | PortAlignment::Undefined => {
                let additional = (axis_length - calculated) / (n as f64 + 1.0);
                let space_between = port_spacing + additional;
                (space_between, space_between)
            }
            _ => (0.0, port_spacing + (axis_length - calculated) / (n as f64 - 1.0)),
        }
    } else {
        match align {
            PortAlignment::Begin => (0.0, port_spacing),
            PortAlignment::Center => ((axis_length - calculated) / 2.0, port_spacing),
            PortAlignment::End => (axis_length - calculated, port_spacing),
            PortAlignment::Distributed => {
                // Distributed bumps `spaceBetweenPorts` by
                // `(avail - calculated) / (n + 1)`, floor-clamped to zero,
                // and starts the cursor one full spacing in. With
                // `calculated` already including the two surrounding gaps
                // (cell minimumContentAreaSize formula), the "additional"
                // expression is computed directly.
                let additional = ((axis_length - calculated) / (n as f64 + 1.0)).max(0.0);
                let space_between = port_spacing + additional;
                (space_between, space_between)
            }
            PortAlignment::Justified if n >= 2 => {
                let extra = (axis_length - calculated) / (n as f64 - 1.0);
                (0.0, port_spacing + extra.max(0.0))
            }
            // n == 1 collapses to centring (divide-by-zero guard).
            PortAlignment::Justified => ((axis_length - calculated) / 2.0, port_spacing),
            // Caller already substituted Undefined → default; treat as
            // Distributed defensively.
            PortAlignment::Undefined => {
                let additional = ((axis_length - calculated) / (n as f64 + 1.0)).max(0.0);
                let space_between = port_spacing + additional;
                (space_between, space_between)
            }
        }
    };
    let mut current = axis_start + current_offset;

    for (i, &pid) in order.iter().enumerate() {
        graph.port_mut(pid).position = make_pos(current + margins[i].before, sizes[i], offsets[i]);
        current += widths[i] + gap;
    }
}

fn apply_fixed_ratio_positions(graph: &mut LGraph, node_id: NodeId, node_size: Vec2) {
    let port_ids: Vec<PortId> = graph.node(node_id).ports.to_vec();
    for pid in port_ids {
        let port = graph.port(pid);
        let side = port.side;
        let stored = port.position;
        let offset = port.properties.get(&PORT_BORDER_OFFSET);
        let port_size = port.size;
        let new_pos = match side {
            PortSide::North => Vec2::new(stored.x * node_size.x, -port_size.y - offset),
            PortSide::South => Vec2::new(stored.x * node_size.x, node_size.y + offset),
            PortSide::East => Vec2::new(node_size.x + offset, stored.y * node_size.y),
            PortSide::West => Vec2::new(-port_size.x - offset, stored.y * node_size.y),
            PortSide::Undefined => stored,
        };
        graph.port_mut(pid).position = new_pos;
    }
}

fn assign_port_anchors(graph: &mut LGraph, node_id: NodeId) {
    let constraints = {
        let node_constraints = graph.node(node_id).port_constraints();
        if node_constraints == PortConstraints::Undefined {
            graph.options.port_constraints
        } else {
            node_constraints
        }
    };
    let port_ids: Vec<PortId> = graph.node(node_id).ports.to_vec();
    for pid in port_ids {
        let port = graph.port(pid);
        if port.explicitly_supplied_anchor {
            continue;
        }
        let anchor = match port.properties.get(&PORT_ANCHOR) {
            Some(v) => v,
            None if constraints.is_side_fixed() => match port.side {
                PortSide::North => Vec2::new(port.size.x / 2.0, 0.0),
                PortSide::East => Vec2::new(port.size.x, port.size.y / 2.0),
                PortSide::South => Vec2::new(port.size.x / 2.0, port.size.y),
                PortSide::West => Vec2::new(0.0, port.size.y / 2.0),
                PortSide::Undefined => Vec2::new(port.size.x / 2.0, port.size.y / 2.0),
            },
            None => Vec2::new(port.size.x / 2.0, port.size.y / 2.0),
        };
        graph.port_mut(pid).anchor = anchor;
    }
}
