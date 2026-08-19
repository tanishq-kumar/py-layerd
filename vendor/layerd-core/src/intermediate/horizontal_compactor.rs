//! Horizontal graph compactor.

use std::{cmp::Ordering, collections::VecDeque};

use hashbrown::{HashMap, HashSet};

use crate::{
    algorithms::network_simplex::{NGraph, Solver},
    graph::{
        LGraph,
        index::{EdgeId, NodeId, PortId},
        node::NodeType,
        port::{PortSide, PortSideSet},
    },
    math::Vec2,
    options::enums::{ConstraintCalculationStrategy, EdgeRoutingStrategy, GraphCompactionStrategy},
    p5_edge_routing::splines::segment::{
        SPLINE_ROUTE_START, SPLINE_SEGMENT_STORE, SegmentId, SplineSegment,
    },
};

const FUZZY_TOLERANCE: f64 = 0.0001;
const EDGE_AWARE_EPSILON: f64 = 0.5;
const NETWORK_SIMPLEX_SEPARATION_WEIGHT: f64 = 1.0;
const NETWORK_SIMPLEX_EDGE_WEIGHT: f64 = 100.0;

#[derive(Debug, Clone, Copy)]
struct Rect {
    x: f64,
    y: f64,
    width: f64,
    height: f64,
}

impl Rect {
    fn right(self) -> f64 {
        self.x + self.width
    }

    fn bottom(self) -> f64 {
        self.y + self.height
    }
}

#[derive(Debug, Clone)]
struct VerticalSegment {
    hitbox: Rect,
    represented_edges: Vec<EdgeId>,
    affected_segments: Vec<SegmentId>,
    affected_bends: Vec<(EdgeId, usize)>,
    group_parents: Vec<usize>,
    ignore_up: bool,
    ignore_down: bool,
}

impl VerticalSegment {
    fn join_with(&mut self, other: &VerticalSegment) {
        self.represented_edges.extend(other.represented_edges.iter().copied());
        self.affected_segments.extend(other.affected_segments.iter().copied());
        self.affected_bends.extend(other.affected_bends.iter().copied());
        self.group_parents.extend(other.group_parents.iter().copied());
        self.ignore_up |= other.ignore_up;
        self.ignore_down |= other.ignore_down;

        let new_x = self.hitbox.x.min(other.hitbox.x);
        let new_y = self.hitbox.y.min(other.hitbox.y);
        let max_x = self.hitbox.right().max(other.hitbox.right());
        let max_y = self.hitbox.bottom().max(other.hitbox.bottom());
        self.hitbox = Rect { x: new_x, y: new_y, width: max_x - new_x, height: max_y - new_y };
    }

    fn intersects(&self, other: &VerticalSegment) -> bool {
        fuzzy_eq(self.hitbox.x, other.hitbox.x)
            && !(fuzzy_lt(self.hitbox.bottom(), other.hitbox.y)
                || fuzzy_lt(other.hitbox.bottom(), self.hitbox.y))
    }
}

#[derive(Debug, Clone, Copy)]
enum Origin {
    Node(NodeId),
    VerticalSegment(usize),
}

#[derive(Debug, Clone)]
struct CNode {
    origin: Origin,
    hitbox: Rect,
    pre_compaction: Rect,
    constraints: Vec<usize>,
    group: usize,
    group_offset_x: f64,
    group_offset_y: f64,
    start_pos: f64,
    lock_left: bool,
    lock_right: bool,
}

#[derive(Debug, Clone)]
struct CGroup {
    nodes: Vec<usize>,
    reference: usize,
    start_pos: f64,
    out_degree: i32,
    out_degree_real: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy)]
enum LockMode {
    None,
    UnconstrainedGroups,
    ConnectionDirection,
}

struct CGraph {
    nodes: Vec<CNode>,
    groups: Vec<CGroup>,
    predefined_horizontal_constraints: Vec<(usize, usize)>,
    direction: Option<Direction>,
    lock_mode: LockMode,
}

impl CGraph {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            groups: Vec::new(),
            predefined_horizontal_constraints: Vec::new(),
            direction: None,
            lock_mode: LockMode::None,
        }
    }

    fn add_node(
        &mut self,
        origin: Origin,
        hitbox: Rect,
        lock_left: bool,
        lock_right: bool,
    ) -> usize {
        let node_idx = self.nodes.len();
        let group_idx = self.groups.len();
        self.nodes.push(CNode {
            origin,
            hitbox,
            pre_compaction: hitbox,
            constraints: Vec::new(),
            group: group_idx,
            group_offset_x: 0.0,
            group_offset_y: 0.0,
            start_pos: f64::NEG_INFINITY,
            lock_left,
            lock_right,
        });
        self.groups.push(CGroup {
            nodes: vec![node_idx],
            reference: node_idx,
            start_pos: f64::NEG_INFINITY,
            out_degree: 0,
            out_degree_real: 0,
        });
        node_idx
    }

    fn add_node_to_group(
        &mut self,
        origin: Origin,
        hitbox: Rect,
        lock_left: bool,
        lock_right: bool,
        group_idx: usize,
    ) -> usize {
        let node_idx = self.nodes.len();
        self.nodes.push(CNode {
            origin,
            hitbox,
            pre_compaction: hitbox,
            constraints: Vec::new(),
            group: group_idx,
            group_offset_x: 0.0,
            group_offset_y: 0.0,
            start_pos: f64::NEG_INFINITY,
            lock_left,
            lock_right,
        });
        self.groups[group_idx].nodes.push(node_idx);
        node_idx
    }

    fn calculate_group_offsets(&mut self) {
        for group_idx in 0..self.groups.len() {
            let nodes = self.groups[group_idx].nodes.clone();
            let mut reference = nodes[0];
            for &node_idx in &nodes {
                if self.nodes[node_idx].hitbox.x < self.nodes[reference].hitbox.x {
                    reference = node_idx;
                }
            }
            self.groups[group_idx].reference = reference;
            let ref_x = self.nodes[reference].hitbox.x;
            let ref_y = self.nodes[reference].hitbox.y;
            for &node_idx in &nodes {
                self.nodes[node_idx].group_offset_x = self.nodes[node_idx].hitbox.x - ref_x;
                self.nodes[node_idx].group_offset_y = self.nodes[node_idx].hitbox.y - ref_y;
            }
        }
    }

    fn change_direction(
        &mut self,
        graph: &LGraph,
        vertical_segments: &[VerticalSegment],
        dir: Direction,
    ) {
        if self.direction == Some(dir) {
            return;
        }

        match (self.direction, dir) {
            (None, Direction::Left) => {
                self.direction = Some(Direction::Left);
                self.calculate_constraints(graph, vertical_segments);
            }
            (None, Direction::Right) => {
                self.direction = Some(Direction::Right);
                self.mirror_hitboxes();
                self.calculate_constraints(graph, vertical_segments);
            }
            (Some(Direction::Left), Direction::Right)
            | (Some(Direction::Right), Direction::Left) => {
                self.direction = Some(dir);
                self.mirror_hitboxes();
                self.reverse_constraints();
            }
            (Some(_), _) => {}
        }
    }

    fn finish(&mut self, graph: &LGraph, vertical_segments: &[VerticalSegment]) {
        self.change_direction(graph, vertical_segments, Direction::Left);
    }

    fn mirror_hitboxes(&mut self) {
        for node in &mut self.nodes {
            node.hitbox.x = -node.hitbox.x - node.hitbox.width;
        }
        self.calculate_group_offsets();
    }

    fn calculate_constraints(&mut self, graph: &LGraph, vertical_segments: &[VerticalSegment]) {
        for node in &mut self.nodes {
            node.constraints.clear();
        }

        let dir = self.direction.unwrap_or(Direction::Left);
        for &(left, right) in &self.predefined_horizontal_constraints {
            match dir {
                Direction::Left => self.nodes[left].constraints.push(right),
                Direction::Right => self.nodes[right].constraints.push(left),
            }
        }

        match graph.options.post_compaction_constraints {
            ConstraintCalculationStrategy::Scanline => match graph.options.edge_routing {
                EdgeRoutingStrategy::Splines => self.calculate_spline_scanline_constraints(graph),
                EdgeRoutingStrategy::Orthogonal =>
                    self.calculate_quadratic_constraints(graph, vertical_segments),
                EdgeRoutingStrategy::Polyline => {}
            },
            ConstraintCalculationStrategy::Quadratic => {
                self.calculate_quadratic_constraints(graph, vertical_segments);
            }
        }

        self.calculate_constraints_for_groups();
    }

    fn calculate_spline_scanline_constraints(&mut self, graph: &LGraph) {
        self.sweep_constraints(|node| matches!(node.origin, Origin::VerticalSegment(_)));

        let edge_spacing = (graph.options.spacing.edge_edge / 2.0 - EDGE_AWARE_EPSILON).max(0.0);
        let node_spacing = (graph.options.spacing.node_node / 2.0 - EDGE_AWARE_EPSILON).max(0.0);
        let min_spacing = self
            .nodes
            .iter()
            .filter_map(|node| match node.origin {
                Origin::VerticalSegment(_) => Some(edge_spacing),
                Origin::Node(node_id) => {
                    if graph.node(node_id).node_type == NodeType::ExternalPort {
                        None
                    } else {
                        Some(node_spacing)
                    }
                }
            })
            .fold(f64::INFINITY, f64::min);

        let min_spacing = if min_spacing.is_infinite() { 0.0 } else { min_spacing };
        for node in &mut self.nodes {
            if matches!(node.origin, Origin::Node(_)) {
                node.hitbox.y -= min_spacing;
                node.hitbox.height += 2.0 * min_spacing;
            }
        }

        self.sweep_constraints(|_| true);

        for node in &mut self.nodes {
            if matches!(node.origin, Origin::Node(_)) {
                node.hitbox.y += min_spacing;
                node.hitbox.height -= 2.0 * min_spacing;
            }
        }
    }

    fn calculate_quadratic_constraints(
        &mut self,
        graph: &LGraph,
        vertical_segments: &[VerticalSegment],
    ) {
        let len = self.nodes.len();
        for a in 0..len {
            for b in 0..len {
                if a == b || self.nodes[a].group == self.nodes[b].group {
                    continue;
                }
                let spacing =
                    vertical_spacing(graph, vertical_segments, &self.nodes[a], &self.nodes[b]);
                let a_box = self.nodes[a].hitbox;
                let b_box = self.nodes[b].hitbox;
                if (b_box.x > a_box.x || (b_box.x == a_box.x && a_box.width < b_box.width))
                    && fuzzy_gt(b_box.y + b_box.height + spacing, a_box.y)
                    && fuzzy_lt(b_box.y, a_box.y + a_box.height + spacing)
                {
                    self.nodes[a].constraints.push(b);
                }
            }
        }
    }

    fn sweep_constraints<F>(&mut self, mut filter: F)
    where
        F: FnMut(&CNode) -> bool,
    {
        #[derive(Clone, Copy)]
        struct Timestamp {
            node: usize,
            low: bool,
            order: usize,
        }

        let mut points = Vec::new();
        for idx in 0..self.nodes.len() {
            if filter(&self.nodes[idx]) {
                let order = points.len();
                points.push(Timestamp { node: idx, low: true, order });
                points.push(Timestamp { node: idx, low: false, order: order + 1 });
            }
        }

        points.sort_by(|a, b| {
            let ya = if a.low {
                self.nodes[a.node].hitbox.y
            } else {
                self.nodes[a.node].hitbox.bottom()
            };
            let yb = if b.low {
                self.nodes[b.node].hitbox.y
            } else {
                self.nodes[b.node].hitbox.bottom()
            };
            let cmp = ya.partial_cmp(&yb).unwrap_or(Ordering::Equal);
            if cmp != Ordering::Equal {
                return cmp;
            }
            match (a.low, b.low) {
                (false, true) => Ordering::Less,
                (true, false) => Ordering::Greater,
                _ => a.order.cmp(&b.order),
            }
        });

        let mut intervals: Vec<usize> = Vec::new();
        let mut cand: Vec<Option<usize>> = vec![None; self.nodes.len()];

        for point in points {
            if point.low {
                let pos = interval_insert_pos(&self.nodes, &intervals, point.node);
                intervals.insert(pos, point.node);
                cand[point.node] = pos.checked_sub(1).map(|p| intervals[p]);
                if pos + 1 < intervals.len() {
                    let right = intervals[pos + 1];
                    cand[right] = Some(point.node);
                }
            } else if let Some(pos) = intervals.iter().position(|&idx| idx == point.node) {
                let left = pos.checked_sub(1).map(|p| intervals[p]);
                if let Some(left) = left
                    && cand[point.node] == Some(left)
                    && self.nodes[left].group != self.nodes[point.node].group
                {
                    self.nodes[left].constraints.push(point.node);
                }

                if pos + 1 < intervals.len() {
                    let right = intervals[pos + 1];
                    if cand[right] == Some(point.node)
                        && self.nodes[right].group != self.nodes[point.node].group
                    {
                        self.nodes[point.node].constraints.push(right);
                    }
                }

                intervals.remove(pos);
            }
        }
    }

    fn calculate_constraints_for_groups(&mut self) {
        for group in &mut self.groups {
            group.out_degree = 0;
            group.out_degree_real = 0;
        }

        for group_idx in 0..self.groups.len() {
            let nodes = self.groups[group_idx].nodes.clone();
            for node_idx in nodes {
                let constraints = self.nodes[node_idx].constraints.clone();
                for inc in constraints {
                    let inc_group = self.nodes[inc].group;
                    if inc_group != group_idx {
                        self.groups[inc_group].out_degree += 1;
                        self.groups[inc_group].out_degree_real += 1;
                    }
                }
            }
        }
    }

    fn reverse_constraints(&mut self) {
        let mut incoming: Vec<Vec<usize>> = vec![Vec::new(); self.nodes.len()];
        for node_idx in 0..self.nodes.len() {
            self.nodes[node_idx].start_pos = f64::NEG_INFINITY;
            for &inc in &self.nodes[node_idx].constraints {
                incoming[inc].push(node_idx);
            }
        }
        for (node_idx, constraints) in incoming.into_iter().enumerate() {
            self.nodes[node_idx].constraints = constraints;
        }
        self.calculate_constraints_for_groups();
    }

    fn compact_longest_path(&mut self, graph: &LGraph, vertical_segments: &[VerticalSegment]) {
        if self.direction.is_none() {
            self.change_direction(graph, vertical_segments, Direction::Left);
        }
        let direction = self.direction.unwrap_or(Direction::Left);

        let mut min_start_pos = f64::INFINITY;
        for node in &self.nodes {
            min_start_pos = min_start_pos
                .min(self.nodes[self.groups[node.group].reference].hitbox.x + node.group_offset_x);
        }

        let mut sinks = VecDeque::new();
        for group_idx in 0..self.groups.len() {
            self.groups[group_idx].start_pos = min_start_pos;
            self.groups[group_idx].out_degree = self.groups[group_idx].out_degree_real;
            if self.groups[group_idx].out_degree == 0 {
                sinks.push_back(group_idx);
            }
        }
        for node in &mut self.nodes {
            node.start_pos = f64::NEG_INFINITY;
        }

        while let Some(group_idx) = sinks.pop_front() {
            let group_nodes = self.groups[group_idx].nodes.clone();
            for &node_idx in &group_nodes {
                let suggested =
                    self.groups[group_idx].start_pos + self.nodes[node_idx].group_offset_x;
                if !self.is_locked_group(group_idx, direction)
                    || self.nodes[node_idx].hitbox.x < suggested
                {
                    self.nodes[node_idx].start_pos = suggested;
                } else {
                    self.nodes[node_idx].start_pos = self.nodes[node_idx].hitbox.x;
                }
            }

            for &node_idx in &group_nodes {
                let constraints = self.nodes[node_idx].constraints.clone();
                for inc in constraints {
                    let spacing = horizontal_spacing(
                        graph,
                        vertical_segments,
                        &self.nodes[node_idx],
                        &self.nodes[inc],
                    );
                    let inc_group = self.nodes[inc].group;
                    let required = self.nodes[node_idx].start_pos
                        + self.nodes[node_idx].hitbox.width
                        + spacing
                        - self.nodes[inc].group_offset_x;
                    self.groups[inc_group].start_pos =
                        self.groups[inc_group].start_pos.max(required);

                    if self.is_locked_node(inc, direction) {
                        let locked = self.nodes[inc].hitbox.x - self.nodes[inc].group_offset_x;
                        self.groups[inc_group].start_pos =
                            self.groups[inc_group].start_pos.max(locked);
                    }

                    self.groups[inc_group].out_degree -= 1;
                    if self.groups[inc_group].out_degree == 0 {
                        sinks.push_back(inc_group);
                    }
                }
            }
        }

        for node in &mut self.nodes {
            node.hitbox.x = node.start_pos;
        }
    }

    fn compact_network_simplex(&mut self, graph: &LGraph, vertical_segments: &[VerticalSegment]) {
        if self.direction.is_none() {
            self.change_direction(graph, vertical_segments, Direction::Left);
        }

        let group_count = self.groups.len();
        let mut ngraph = NGraph::with_capacity(group_count + 1, self.nodes.len() * 4);
        for group_idx in 0..group_count {
            ngraph.add_node(group_idx as u32);
        }
        let mut next_stable_id = group_count as u32;

        self.add_network_separation_constraints(
            graph,
            vertical_segments,
            &mut ngraph,
            &mut next_stable_id,
        );
        self.add_network_edge_constraints(graph, vertical_segments, &mut ngraph);
        add_artificial_source_node(&mut ngraph, &mut next_stable_id);

        let result = Solver::new(ngraph).solve();
        for node in &result.graph.nodes {
            let group_idx = node.stable_id as usize;
            if group_idx >= group_count {
                continue;
            }
            let group_nodes = self.groups[group_idx].nodes.clone();
            for cnode_idx in group_nodes {
                self.nodes[cnode_idx].hitbox.x =
                    node.layer as f64 + self.nodes[cnode_idx].group_offset_x;
            }
        }
    }

    fn add_network_separation_constraints(
        &self,
        graph: &LGraph,
        vertical_segments: &[VerticalSegment],
        ngraph: &mut NGraph,
        next_stable_id: &mut u32,
    ) {
        for cnode_idx in 0..self.nodes.len() {
            for &inc_idx in &self.nodes[cnode_idx].constraints {
                let source_group = self.nodes[cnode_idx].group;
                let target_group = self.nodes[inc_idx].group;
                if source_group == target_group {
                    continue;
                }

                let spacing = horizontal_spacing(
                    graph,
                    vertical_segments,
                    &self.nodes[cnode_idx],
                    &self.nodes[inc_idx],
                );
                let delta = self.nodes[cnode_idx].group_offset_x
                    + self.nodes[cnode_idx].hitbox.width
                    + spacing
                    - self.nodes[inc_idx].group_offset_x;
                let delta = delta.ceil().max(0.0) as i32;

                if !vertical_segments_of_same_edge(
                    vertical_segments,
                    &self.nodes[cnode_idx],
                    &self.nodes[inc_idx],
                ) {
                    let weight = match (self.nodes[cnode_idx].origin, self.nodes[inc_idx].origin) {
                        (Origin::VerticalSegment(_), Origin::Node(_))
                        | (Origin::Node(_), Origin::VerticalSegment(_)) => 2.0,
                        _ => NETWORK_SIMPLEX_SEPARATION_WEIGHT,
                    };
                    ngraph.add_edge(source_group, target_group, weight, delta);
                } else {
                    let helper = ngraph.add_node(*next_stable_id);
                    *next_stable_id += 1;
                    let offset_delta = (self.nodes[inc_idx].group_offset_x
                        - self.nodes[cnode_idx].group_offset_x)
                        .ceil() as i32;
                    ngraph.add_edge(
                        helper,
                        source_group,
                        NETWORK_SIMPLEX_SEPARATION_WEIGHT,
                        offset_delta.max(0),
                    );
                    ngraph.add_edge(
                        helper,
                        target_group,
                        NETWORK_SIMPLEX_SEPARATION_WEIGHT,
                        (-offset_delta).max(0),
                    );
                }
            }
        }
    }

    fn add_network_edge_constraints(
        &self,
        graph: &LGraph,
        vertical_segments: &[VerticalSegment],
        ngraph: &mut NGraph,
    ) {
        let mut node_map: HashMap<NodeId, usize> = HashMap::new();
        let mut edge_map: HashMap<EdgeId, Vec<usize>> = HashMap::new();
        for (cnode_idx, cnode) in self.nodes.iter().enumerate() {
            match cnode.origin {
                Origin::Node(node_id) => {
                    node_map.insert(node_id, cnode_idx);
                }
                Origin::VerticalSegment(vs_idx) => {
                    if let Some(segment) = vertical_segments.get(vs_idx) {
                        for &edge_id in &segment.represented_edges {
                            edge_map.entry(edge_id).or_default().push(cnode_idx);
                        }
                    }
                }
            }
        }

        for (cnode_idx, cnode) in self.nodes.iter().enumerate() {
            let Origin::Node(node_id) = cnode.origin else {
                continue;
            };
            for edge_id in graph.outgoing_edges(node_id) {
                let edge = graph.edge(edge_id);
                let source_node = graph.port(edge.source).owner;
                let target_node = graph.port(edge.target).owner;
                if source_node == target_node {
                    continue;
                }
                let source_side = graph.port(edge.source).side;
                let target_side = graph.port(edge.target).side;
                if PortSideSet::SIDES_NORTH_SOUTH.contains(port_side_flag(source_side))
                    && PortSideSet::SIDES_NORTH_SOUTH.contains(port_side_flag(target_side))
                {
                    continue;
                }

                let Some(&target_cnode) = node_map.get(&target_node) else {
                    continue;
                };
                let source_group = self.nodes[cnode_idx].group;
                let target_group = self.nodes[target_cnode].group;
                if source_group != target_group {
                    ngraph.add_edge(source_group, target_group, NETWORK_SIMPLEX_EDGE_WEIGHT, 0);
                }

                if source_side == PortSide::West
                    && !graph.port(edge.source).outgoing_edges.is_empty()
                    && let Some(edge_nodes) = edge_map.get(&edge_id)
                {
                    for &segment_node in edge_nodes {
                        if self.nodes[segment_node].hitbox.x < cnode.hitbox.x {
                            let segment_group = self.nodes[segment_node].group;
                            if segment_group != source_group {
                                ngraph.add_edge(
                                    segment_group,
                                    source_group,
                                    NETWORK_SIMPLEX_EDGE_WEIGHT,
                                    1,
                                );
                            }
                        }
                    }
                }

                if target_side == PortSide::East
                    && !graph.port(edge.target).incoming_edges.is_empty()
                    && let Some(edge_nodes) = edge_map.get(&edge_id)
                {
                    for &segment_node in edge_nodes {
                        if self.nodes[segment_node].hitbox.x > cnode.hitbox.x {
                            let segment_group = self.nodes[segment_node].group;
                            if source_group != segment_group {
                                ngraph.add_edge(
                                    source_group,
                                    segment_group,
                                    NETWORK_SIMPLEX_EDGE_WEIGHT,
                                    1,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn is_locked_group(&self, group_idx: usize, direction: Direction) -> bool {
        self.groups[group_idx]
            .nodes
            .iter()
            .any(|&node_idx| self.is_locked_node(node_idx, direction))
    }

    fn is_locked_node(&self, node_idx: usize, direction: Direction) -> bool {
        match self.lock_mode {
            LockMode::None => false,
            LockMode::UnconstrainedGroups =>
                self.groups[self.nodes[node_idx].group].out_degree_real == 0,
            LockMode::ConnectionDirection => match direction {
                Direction::Left => self.nodes[node_idx].lock_left,
                Direction::Right => self.nodes[node_idx].lock_right,
            },
        }
    }
}

/// Compacts the graph horizontally after edge routing.
pub fn compact(graph: &mut LGraph) {
    if graph.options.post_compaction_strategy == GraphCompactionStrategy::None {
        return;
    }
    if graph.options.edge_routing == EdgeRoutingStrategy::Polyline {
        return;
    }

    let (mut cgraph, mut vertical_segments) = transform_to_cgraph(graph);
    if cgraph.nodes.is_empty() {
        return;
    }

    match graph.options.post_compaction_strategy {
        GraphCompactionStrategy::None => return,
        GraphCompactionStrategy::Left => {
            cgraph.change_direction(graph, &vertical_segments, Direction::Left);
            cgraph.compact_longest_path(graph, &vertical_segments);
        }
        GraphCompactionStrategy::Right => {
            cgraph.change_direction(graph, &vertical_segments, Direction::Right);
            cgraph.compact_longest_path(graph, &vertical_segments);
        }
        GraphCompactionStrategy::LeftRightConstraintLocking => {
            cgraph.change_direction(graph, &vertical_segments, Direction::Left);
            cgraph.compact_longest_path(graph, &vertical_segments);
            cgraph.change_direction(graph, &vertical_segments, Direction::Right);
            cgraph.lock_mode = LockMode::UnconstrainedGroups;
            cgraph.compact_longest_path(graph, &vertical_segments);
        }
        GraphCompactionStrategy::LeftRightConnectionLocking => {
            cgraph.change_direction(graph, &vertical_segments, Direction::Left);
            cgraph.compact_longest_path(graph, &vertical_segments);
            cgraph.change_direction(graph, &vertical_segments, Direction::Right);
            cgraph.lock_mode = LockMode::ConnectionDirection;
            cgraph.compact_longest_path(graph, &vertical_segments);
        }
        GraphCompactionStrategy::EdgeLength => {
            cgraph.change_direction(graph, &vertical_segments, Direction::Left);
            cgraph.compact_network_simplex(graph, &vertical_segments);
        }
    }

    cgraph.finish(graph, &vertical_segments);
    apply_layout(graph, &cgraph, &mut vertical_segments);
}

fn add_artificial_source_node(ngraph: &mut NGraph, next_stable_id: &mut u32) {
    let sources: Vec<usize> = ngraph
        .nodes
        .iter()
        .enumerate()
        .filter_map(|(idx, node)| node.incoming.is_empty().then_some(idx))
        .collect();
    if sources.len() <= 1 {
        return;
    }

    let source = ngraph.add_node(*next_stable_id);
    *next_stable_id += 1;
    for target in sources {
        ngraph.add_edge(source, target, 0.0, 1);
    }
}

fn port_side_flag(side: PortSide) -> PortSideSet {
    match side {
        PortSide::North => PortSideSet::NORTH,
        PortSide::East => PortSideSet::EAST,
        PortSide::South => PortSideSet::SOUTH,
        PortSide::West => PortSideSet::WEST,
        PortSide::Undefined => PortSideSet::SIDES_NONE,
    }
}

fn transform_to_cgraph(graph: &LGraph) -> (CGraph, Vec<VerticalSegment>) {
    let mut cgraph = CGraph::new();
    let mut node_map: HashMap<NodeId, usize> = HashMap::new();

    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            let node = graph.node(node_id);
            let hitbox = Rect {
                x: node.position.x - node.margin.left,
                y: node.position.y - node.margin.top,
                width: node.size.x + node.margin.left + node.margin.right,
                height: node.size.y + node.margin.top + node.margin.bottom,
            };
            let (lock_left, lock_right) = connection_locks_for_node(graph, node_id);
            let idx = cgraph.add_node(Origin::Node(node_id), hitbox, lock_left, lock_right);
            node_map.insert(node_id, idx);
        }
    }

    let (vertical_segments, raw_constraints, raw_to_merged) = match graph.options.edge_routing {
        EdgeRoutingStrategy::Splines => collect_spline_vertical_segments(graph),
        EdgeRoutingStrategy::Orthogonal => collect_orthogonal_vertical_segments(graph, &node_map),
        EdgeRoutingStrategy::Polyline => (Vec::new(), Vec::new(), Vec::new()),
    };

    let mut raw_to_cnode = vec![usize::MAX; raw_to_merged.len()];
    for (merged_idx, segment) in vertical_segments.iter().enumerate() {
        let (lock_left, lock_right) = connection_locks_for_segment(graph, segment);
        let cnode = if let Some(&parent_idx) = segment.group_parents.first() {
            let parent_group = cgraph.nodes[parent_idx].group;
            cgraph.add_node_to_group(
                Origin::VerticalSegment(merged_idx),
                segment.hitbox,
                lock_left,
                lock_right,
                parent_group,
            )
        } else {
            cgraph.add_node(
                Origin::VerticalSegment(merged_idx),
                segment.hitbox,
                lock_left,
                lock_right,
            )
        };
        for (raw_idx, &mapped) in raw_to_merged.iter().enumerate() {
            if mapped == merged_idx {
                raw_to_cnode[raw_idx] = cnode;
            }
        }
    }

    for (raw_a, raw_b) in raw_constraints {
        if raw_a < raw_to_cnode.len() && raw_b < raw_to_cnode.len() {
            let a = raw_to_cnode[raw_a];
            let b = raw_to_cnode[raw_b];
            if a != usize::MAX && b != usize::MAX {
                cgraph.predefined_horizontal_constraints.push((a, b));
            }
        }
    }

    cgraph.calculate_group_offsets();
    drop(node_map);
    (cgraph, vertical_segments)
}

fn collect_orthogonal_vertical_segments(
    graph: &LGraph,
    node_map: &HashMap<NodeId, usize>,
) -> (Vec<VerticalSegment>, Vec<(usize, usize)>, Vec<usize>) {
    let mut raw = Vec::new();

    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            let Some(&cnode_idx) = node_map.get(&node_id) else {
                continue;
            };
            let node_hitbox = Rect {
                x: graph.node(node_id).position.x - graph.node(node_id).margin.left,
                y: graph.node(node_id).position.y - graph.node(node_id).margin.top,
                width: graph.node(node_id).size.x
                    + graph.node(node_id).margin.left
                    + graph.node(node_id).margin.right,
                height: graph.node(node_id).size.y
                    + graph.node(node_id).margin.top
                    + graph.node(node_id).margin.bottom,
            };

            for edge_id in graph.outgoing_edges(node_id) {
                let edge = graph.edge(edge_id);
                if edge.bend_points.is_empty() {
                    continue;
                }

                let first_bend = edge.bend_points[0];
                match graph.port(edge.source).side {
                    PortSide::North => raw.push(orthogonal_vertical_segment(
                        graph,
                        edge_id,
                        first_bend,
                        Vec2::new(first_bend.x, node_hitbox.y),
                        Some(cnode_idx),
                        vec![(edge_id, 0)],
                        false,
                        true,
                    )),
                    PortSide::South => raw.push(orthogonal_vertical_segment(
                        graph,
                        edge_id,
                        first_bend,
                        Vec2::new(first_bend.x, node_hitbox.bottom()),
                        Some(cnode_idx),
                        vec![(edge_id, 0)],
                        true,
                        false,
                    )),
                    _ => {}
                }

                let mut first_regular = true;
                let mut last_segment: Option<usize> = None;
                let mut last_segment_start_y = first_bend.y;
                let mut bend1 = first_bend;
                for bend_idx in 1..edge.bend_points.len() {
                    let bend2 = edge.bend_points[bend_idx];
                    if !fuzzy_eq(bend1.y, bend2.y) {
                        let raw_idx = raw.len();
                        raw.push(orthogonal_vertical_segment(
                            graph,
                            edge_id,
                            bend1,
                            bend2,
                            None,
                            vec![(edge_id, bend_idx - 1), (edge_id, bend_idx)],
                            false,
                            false,
                        ));

                        if first_regular {
                            first_regular = false;
                            if bend2.y < node_hitbox.y {
                                raw[raw_idx].ignore_down = true;
                            } else if bend2.y > node_hitbox.bottom() {
                                raw[raw_idx].ignore_up = true;
                            } else {
                                raw[raw_idx].ignore_up = true;
                                raw[raw_idx].ignore_down = true;
                            }
                        }

                        last_segment = Some(raw_idx);
                        last_segment_start_y = bend1.y;
                    }
                    if bend_idx + 1 < edge.bend_points.len() {
                        bend1 = bend2;
                    }
                }

                if let Some(raw_idx) = last_segment {
                    let target_node = graph.port(edge.target).owner;
                    if let Some(&target_cnode) = node_map.get(&target_node) {
                        let target = &graph.node(target_node);
                        let target_hitbox = Rect {
                            x: target.position.x - target.margin.left,
                            y: target.position.y - target.margin.top,
                            width: target.size.x + target.margin.left + target.margin.right,
                            height: target.size.y + target.margin.top + target.margin.bottom,
                        };
                        if last_segment_start_y < target_hitbox.y {
                            raw[raw_idx].ignore_down = true;
                        } else if last_segment_start_y > target_hitbox.bottom() {
                            raw[raw_idx].ignore_up = true;
                        } else {
                            raw[raw_idx].ignore_up = true;
                            raw[raw_idx].ignore_down = true;
                        }
                        let _ = target_cnode;
                    }
                }
            }

            for edge_id in graph.incoming_edges(node_id) {
                let edge = graph.edge(edge_id);
                if edge.bend_points.is_empty() {
                    continue;
                }
                let bend_idx = edge.bend_points.len() - 1;
                let last_bend = edge.bend_points[bend_idx];
                match graph.port(edge.target).side {
                    PortSide::North => raw.push(orthogonal_vertical_segment(
                        graph,
                        edge_id,
                        last_bend,
                        Vec2::new(last_bend.x, node_hitbox.y),
                        Some(cnode_idx),
                        vec![(edge_id, bend_idx)],
                        false,
                        true,
                    )),
                    PortSide::South => raw.push(orthogonal_vertical_segment(
                        graph,
                        edge_id,
                        last_bend,
                        Vec2::new(last_bend.x, node_hitbox.bottom()),
                        Some(cnode_idx),
                        vec![(edge_id, bend_idx)],
                        true,
                        false,
                    )),
                    _ => {}
                }
            }
        }
    }

    merge_vertical_segments(raw, Vec::new())
}

fn orthogonal_vertical_segment(
    graph: &LGraph,
    edge_id: EdgeId,
    p1: Vec2,
    p2: Vec2,
    group_parent: Option<usize>,
    affected_bends: Vec<(EdgeId, usize)>,
    ignore_up: bool,
    ignore_down: bool,
) -> VerticalSegment {
    let junctions = graph
        .edge(edge_id)
        .properties
        .get(&crate::properties::internal::JUNCTION_POINTS);
    let represented_edges = vec![edge_id];
    let mut group_parents = Vec::new();
    if let Some(parent) = group_parent {
        group_parents.push(parent);
    }

    let segment = VerticalSegment {
        hitbox: Rect {
            x: p1.x.min(p2.x),
            y: p1.y.min(p2.y),
            width: (p1.x - p2.x).abs(),
            height: (p1.y - p2.y).abs(),
        },
        represented_edges,
        affected_segments: Vec::new(),
        affected_bends,
        group_parents,
        ignore_up,
        ignore_down,
    };

    // Keep the same x-only membership test as the segment constructor for
    // junction points; the current write-back does not move them
    // separately yet, but preserving represented edge membership keeps the
    // same merge and spacing behavior.
    let _ = junctions;
    segment
}

fn collect_spline_vertical_segments(
    graph: &LGraph,
) -> (Vec<VerticalSegment>, Vec<(usize, usize)>, Vec<usize>) {
    let segments: Vec<SplineSegment> =
        graph.properties.get_ref(&SPLINE_SEGMENT_STORE).cloned().unwrap_or_default();
    if segments.is_empty() {
        return (Vec::new(), Vec::new(), Vec::new());
    }

    let mut raw = Vec::new();
    let mut raw_constraints = Vec::new();
    for route in collect_spline_routes(graph) {
        let mut last_vs: Option<usize> = None;
        for seg_id in route {
            let idx = seg_id.0 as usize;
            if idx >= segments.len() || segments[idx].is_straight {
                continue;
            }
            let Some(&edge_id) = segments[idx].edges.first() else {
                continue;
            };
            let raw_idx = raw.len();
            raw.push(VerticalSegment {
                hitbox: Rect {
                    x: segments[idx].bbox_x,
                    y: segments[idx].bbox_y,
                    width: segments[idx].bbox_width,
                    height: segments[idx].bbox_height,
                },
                represented_edges: vec![edge_id],
                affected_segments: vec![seg_id],
                affected_bends: Vec::new(),
                group_parents: Vec::new(),
                ignore_up: false,
                ignore_down: false,
            });
            if let Some(prev) = last_vs {
                raw_constraints.push((prev, raw_idx));
            }
            last_vs = Some(raw_idx);
        }
    }

    merge_vertical_segments(raw, raw_constraints)
}

fn merge_vertical_segments(
    raw: Vec<VerticalSegment>,
    raw_constraints: Vec<(usize, usize)>,
) -> (Vec<VerticalSegment>, Vec<(usize, usize)>, Vec<usize>) {
    if raw.is_empty() {
        return (Vec::new(), raw_constraints, Vec::new());
    }

    let mut order: Vec<usize> = (0..raw.len()).collect();
    order.sort_by(|&a, &b| {
        fuzzy_cmp(raw[a].hitbox.x, raw[b].hitbox.x)
            .then_with(|| raw[a].hitbox.y.partial_cmp(&raw[b].hitbox.y).unwrap_or(Ordering::Equal))
    });

    let mut merged = Vec::new();
    let mut raw_to_merged = vec![usize::MAX; raw.len()];
    let mut survivor = raw[order[0]].clone();
    let mut survivor_raws = vec![order[0]];

    for &raw_idx in order.iter().skip(1) {
        if survivor.intersects(&raw[raw_idx]) {
            survivor.join_with(&raw[raw_idx]);
            survivor_raws.push(raw_idx);
        } else {
            let merged_idx = merged.len();
            for raw_id in survivor_raws.drain(..) {
                raw_to_merged[raw_id] = merged_idx;
            }
            merged.push(survivor);
            survivor = raw[raw_idx].clone();
            survivor_raws.push(raw_idx);
        }
    }

    let merged_idx = merged.len();
    for raw_id in survivor_raws {
        raw_to_merged[raw_id] = merged_idx;
    }
    merged.push(survivor);

    (merged, raw_constraints, raw_to_merged)
}

fn collect_spline_routes(graph: &LGraph) -> Vec<Vec<SegmentId>> {
    let mut routes = Vec::new();
    for layer in &graph.layers {
        for &node_id in &layer.nodes {
            for &port_id in &graph.node(node_id).ports {
                for &edge_id in &graph.port(port_id).outgoing_edges {
                    let edge = graph.edge(edge_id);
                    if graph.port(edge.source).owner == graph.port(edge.target).owner {
                        continue;
                    }
                    if let Some(route) = graph.edge(edge_id).properties.get_ref(&SPLINE_ROUTE_START)
                        && !route.is_empty()
                    {
                        routes.push(route.clone());
                    }
                }
            }
        }
    }
    routes
}

fn apply_layout(graph: &mut LGraph, cgraph: &CGraph, vertical_segments: &mut [VerticalSegment]) {
    let mut segment_store: Vec<SplineSegment> =
        graph.properties.get_ref(&SPLINE_SEGMENT_STORE).cloned().unwrap_or_default();

    let mut node_hitboxes: HashMap<NodeId, Rect> = HashMap::new();
    let mut node_delta_x: HashMap<NodeId, f64> = HashMap::new();
    let mut top_left = Vec2::new(f64::INFINITY, f64::INFINITY);
    let mut bottom_right = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);

    for cnode in &cgraph.nodes {
        top_left.x = top_left.x.min(cnode.hitbox.x);
        top_left.y = top_left.y.min(cnode.hitbox.y);
        bottom_right.x = bottom_right.x.max(cnode.hitbox.right());
        bottom_right.y = bottom_right.y.max(cnode.hitbox.bottom());

        match cnode.origin {
            Origin::Node(node_id) => {
                let margin_left = graph.node(node_id).margin.left;
                node_delta_x.insert(node_id, cnode.hitbox.x - cnode.pre_compaction.x);
                graph.node_mut(node_id).position.x = cnode.hitbox.x + margin_left;
                node_hitboxes.insert(node_id, cnode.hitbox);
            }
            Origin::VerticalSegment(vs_idx) => {
                let delta_x = cnode.hitbox.x - cnode.pre_compaction.x;
                if let Some(vs) = vertical_segments.get_mut(vs_idx) {
                    for &seg_id in &vs.affected_segments {
                        if let Some(seg) = segment_store.get_mut(seg_id.0 as usize) {
                            seg.bbox_x += delta_x;
                        }
                    }
                    for &(edge_id, bend_idx) in &vs.affected_bends {
                        if let Some(bend) = graph.edge_mut(edge_id).bend_points.get_mut(bend_idx) {
                            bend.x += delta_x;
                        }
                    }
                    vs.hitbox.x = cnode.hitbox.x;
                }
            }
        }
    }

    if top_left.x.is_finite() {
        graph.offset = Vec2::new(-top_left.x, -top_left.y);
        graph.size = Vec2::new(bottom_right.x - top_left.x, bottom_right.y - top_left.y);
    }

    if graph.options.edge_routing == EdgeRoutingStrategy::Splines {
        offset_spline_self_loop_bend_points(graph, &node_delta_x);
        adjust_spline_control_points(graph, &mut segment_store, &node_hitboxes);
    }

    offset_self_loop_labels(graph, &node_delta_x);

    graph.properties.set(&SPLINE_SEGMENT_STORE, segment_store);
}

fn collect_self_loop_edges(
    graph: &LGraph,
    node_delta_x: &HashMap<NodeId, f64>,
) -> Vec<(EdgeId, f64)> {
    let mut edges = Vec::new();
    for (&node_id, &delta_x) in node_delta_x {
        if delta_x == 0.0 {
            continue;
        }
        for edge_id in graph.outgoing_edges(node_id) {
            let edge = graph.edge(edge_id);
            if edge.source_owner == node_id && edge.target_owner == node_id {
                edges.push((edge_id, delta_x));
            }
        }
    }
    edges
}

fn offset_spline_self_loop_bend_points(graph: &mut LGraph, node_delta_x: &HashMap<NodeId, f64>) {
    for (edge_id, delta_x) in collect_self_loop_edges(graph, node_delta_x) {
        for bend in &mut graph.edge_mut(edge_id).bend_points {
            bend.x += delta_x;
        }
    }
}

fn offset_self_loop_labels(graph: &mut LGraph, node_delta_x: &HashMap<NodeId, f64>) {
    for (edge_id, delta_x) in collect_self_loop_edges(graph, node_delta_x) {
        let labels = graph.edge(edge_id).labels.clone();
        for label_id in labels {
            graph.label_mut(label_id).position.x += delta_x;
        }
    }
}

fn adjust_spline_control_points(
    graph: &LGraph,
    segments: &mut [SplineSegment],
    node_hitboxes: &HashMap<NodeId, Rect>,
) {
    for route in collect_spline_routes(graph) {
        if route.is_empty() {
            continue;
        }
        let route_indices: Vec<usize> = route
            .iter()
            .map(|sid| sid.0 as usize)
            .filter(|&idx| idx < segments.len())
            .collect();
        if route_indices.is_empty() {
            continue;
        }

        let mut last_idx_pos = 0usize;
        let first_idx = route_indices[0];
        if route_indices.len() == 1 {
            adjust_control_point_between_segments(
                segments,
                node_hitboxes,
                &route_indices,
                first_idx,
                first_idx,
                1,
                0,
            );
            continue;
        }

        let mut i = 1usize;
        while i < route_indices.len() {
            let last_seg_idx = route_indices[last_idx_pos];
            if segments[last_seg_idx].initial_segment || !segments[last_seg_idx].is_straight {
                if let Some((j, next_idx)) = first_non_straight_segment(segments, &route_indices, i)
                {
                    adjust_control_point_between_segments(
                        segments,
                        node_hitboxes,
                        &route_indices,
                        last_seg_idx,
                        next_idx,
                        i,
                        j,
                    );
                    i = j + 1;
                    last_idx_pos = j;
                } else {
                    break;
                }
            } else {
                i += 1;
            }
        }
    }
}

fn first_non_straight_segment(
    segments: &[SplineSegment],
    route_indices: &[usize],
    index: usize,
) -> Option<(usize, usize)> {
    if index >= route_indices.len() {
        return None;
    }
    for i in index..route_indices.len() {
        let seg_idx = route_indices[i];
        if i == route_indices.len() - 1 || !segments[seg_idx].is_straight {
            return Some((i, seg_idx));
        }
    }
    None
}

fn adjust_control_point_between_segments(
    segments: &mut [SplineSegment],
    node_hitboxes: &HashMap<NodeId, Rect>,
    route_indices: &[usize],
    left_seg_idx: usize,
    right_seg_idx: usize,
    left_idx: usize,
    right_idx: usize,
) {
    let mut idx1 = left_idx;
    let start_x = if segments[left_seg_idx].initial_segment && segments[left_seg_idx].is_straight {
        idx1 = idx1.saturating_sub(1);
        segments[left_seg_idx]
            .source_node
            .and_then(|node| node_hitboxes.get(&node).copied())
            .map(|hitbox| hitbox.x + hitbox.width)
            .unwrap_or_else(|| segments[left_seg_idx].bbox_x + segments[left_seg_idx].bbox_width)
    } else {
        segments[left_seg_idx].bbox_x + segments[left_seg_idx].bbox_width
    };

    let mut idx2 = right_idx;
    let end_x = if segments[right_seg_idx].last_segment && segments[right_seg_idx].is_straight {
        idx2 += 1;
        segments[right_seg_idx]
            .target_node
            .and_then(|node| node_hitboxes.get(&node).copied())
            .map(|hitbox| hitbox.x)
            .unwrap_or(segments[right_seg_idx].bbox_x)
    } else {
        segments[right_seg_idx].bbox_x
    };

    let strip = end_x - start_x;
    let chunks = 2usize.max(idx2.saturating_sub(idx1)) as f64;
    let chunk = strip / chunks;
    let mut new_pos = start_x + chunk;

    for route_pos in idx1..idx2 {
        let Some(&seg_idx) = route_indices.get(route_pos) else {
            continue;
        };
        let width = segments[seg_idx].bbox_width;
        segments[seg_idx].bbox_x = new_pos - width / 2.0;
        new_pos += chunk;
    }
}

fn connection_locks_for_node(graph: &LGraph, node_id: NodeId) -> (bool, bool) {
    if graph.node(node_id).node_type == NodeType::ExternalPort {
        return (false, false);
    }
    let mut incoming = 0usize;
    let mut outgoing = 0usize;
    for &port_id in &graph.node(node_id).ports {
        incoming += graph.port(port_id).incoming_edges.len();
        outgoing += graph.port(port_id).outgoing_edges.len();
    }
    if incoming < outgoing {
        (true, false)
    } else if incoming > outgoing {
        (false, true)
    } else {
        (false, false)
    }
}

fn connection_locks_for_segment(graph: &LGraph, segment: &VerticalSegment) -> (bool, bool) {
    let mut inc: HashSet<PortId> = HashSet::new();
    let mut out: HashSet<PortId> = HashSet::new();
    for &edge_id in &segment.represented_edges {
        let edge = graph.edge(edge_id);
        inc.insert(edge.source);
        out.insert(edge.target);
    }
    if inc.len() < out.len() {
        (true, false)
    } else if inc.len() > out.len() {
        (false, true)
    } else {
        (false, false)
    }
}

fn horizontal_spacing(
    graph: &LGraph,
    vertical_segments: &[VerticalSegment],
    a: &CNode,
    b: &CNode,
) -> f64 {
    if vertical_segments_of_same_edge(vertical_segments, a, b) {
        return 0.0;
    }
    if node_type(graph, a) == Some(NodeType::ExternalPort)
        || node_type(graph, b) == Some(NodeType::ExternalPort)
    {
        return 0.0;
    }
    spacing_for_types_horizontal(graph, spacing_type(graph, a), spacing_type(graph, b))
}

fn vertical_spacing(
    graph: &LGraph,
    vertical_segments: &[VerticalSegment],
    a: &CNode,
    b: &CNode,
) -> f64 {
    if vertical_segments_of_same_edge(vertical_segments, a, b) {
        return 1.0;
    }
    spacing_for_types_vertical(graph, spacing_type(graph, a), spacing_type(graph, b))
}

fn vertical_segments_of_same_edge(
    vertical_segments: &[VerticalSegment],
    a: &CNode,
    b: &CNode,
) -> bool {
    let (Origin::VerticalSegment(a_idx), Origin::VerticalSegment(b_idx)) = (a.origin, b.origin)
    else {
        return false;
    };
    let Some(a_seg) = vertical_segments.get(a_idx) else {
        return false;
    };
    let Some(b_seg) = vertical_segments.get(b_idx) else {
        return false;
    };
    a_seg
        .represented_edges
        .iter()
        .any(|edge| b_seg.represented_edges.iter().any(|other| edge == other))
}

fn node_type(graph: &LGraph, node: &CNode) -> Option<NodeType> {
    match node.origin {
        Origin::Node(node_id) => Some(graph.node(node_id).node_type),
        Origin::VerticalSegment(_) => None,
    }
}

fn spacing_type(graph: &LGraph, node: &CNode) -> NodeType {
    node_type(graph, node).unwrap_or(NodeType::LongEdge)
}

fn spacing_for_types_horizontal(graph: &LGraph, a: NodeType, b: NodeType) -> f64 {
    use NodeType::*;
    match (a, b) {
        (Normal, Normal) | (Normal, Label) | (Label, Normal) =>
            graph.options.spacing.node_node_between_layers,
        (Normal, LongEdge)
        | (LongEdge, Normal)
        | (Normal, BreakingPoint)
        | (BreakingPoint, Normal)
        | (LongEdge, Label)
        | (Label, LongEdge)
        | (BreakingPoint, Label)
        | (Label, BreakingPoint)
        | (BreakingPoint, LongEdge)
        | (LongEdge, BreakingPoint) => graph.options.spacing.edge_node_between_layers,
        (ExternalPort, ExternalPort) => graph.options.spacing.port_port,
        (ExternalPort, Label) | (Label, ExternalPort) =>
            graph.options.spacing.label_port_horizontal,
        _ => graph.options.spacing.edge_edge_between_layers,
    }
}

fn spacing_for_types_vertical(graph: &LGraph, a: NodeType, b: NodeType) -> f64 {
    use NodeType::*;
    match (a, b) {
        (Normal, Normal) | (Normal, Label) | (Label, Normal) => graph.options.spacing.node_node,
        (Normal, LongEdge)
        | (LongEdge, Normal)
        | (Normal, BreakingPoint)
        | (BreakingPoint, Normal)
        | (LongEdge, Label)
        | (Label, LongEdge)
        | (BreakingPoint, Label)
        | (Label, BreakingPoint)
        | (BreakingPoint, LongEdge)
        | (LongEdge, BreakingPoint) => graph.options.spacing.edge_node,
        (ExternalPort, ExternalPort) => graph.options.spacing.port_port,
        (ExternalPort, Label) | (Label, ExternalPort) => graph.options.spacing.label_port_vertical,
        _ => graph.options.spacing.edge_edge,
    }
}

fn interval_insert_pos(nodes: &[CNode], intervals: &[usize], node_idx: usize) -> usize {
    let center = nodes[node_idx].hitbox.x + nodes[node_idx].hitbox.width / 2.0;
    intervals
        .iter()
        .position(|&idx| nodes[idx].hitbox.x + nodes[idx].hitbox.width / 2.0 > center)
        .unwrap_or(intervals.len())
}

fn fuzzy_eq(a: f64, b: f64) -> bool {
    (a - b).abs() <= FUZZY_TOLERANCE
}

fn fuzzy_cmp(a: f64, b: f64) -> Ordering {
    if fuzzy_eq(a, b) {
        Ordering::Equal
    } else {
        a.partial_cmp(&b).unwrap_or(Ordering::Equal)
    }
}

fn fuzzy_lt(a: f64, b: f64) -> bool {
    !fuzzy_eq(a, b) && a < b
}

fn fuzzy_gt(a: f64, b: f64) -> bool {
    !fuzzy_eq(a, b) && a > b
}
