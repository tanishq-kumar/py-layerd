use std::{
    ops::{Deref, DerefMut},
    slice,
};

use bitflags::bitflags;

use super::index::{EdgeId, LabelId, NodeId};
use crate::{math::Vec2, properties::PropertyMap};

/// The side of a node on which a port is located.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PortSide {
    #[default]
    Undefined,
    North,
    East,
    South,
    West,
}

impl PortSide {
    /// Returns the opposed port side (East<->West, North<->South).
    pub fn opposed(self) -> Self {
        match self {
            PortSide::North => PortSide::South,
            PortSide::South => PortSide::North,
            PortSide::East => PortSide::West,
            PortSide::West => PortSide::East,
            PortSide::Undefined => PortSide::Undefined,
        }
    }

    /// Returns the port side associated with outgoing flow under a given layout direction.
    /// Map a layout direction to the canonical "downstream" port side.
    pub fn from_direction(direction: crate::options::enums::LayoutDirection) -> Self {
        use crate::options::enums::LayoutDirection;
        match direction {
            LayoutDirection::Right => PortSide::East,
            LayoutDirection::Left => PortSide::West,
            LayoutDirection::Down => PortSide::South,
            LayoutDirection::Up => PortSide::North,
            LayoutDirection::Undefined => PortSide::East,
        }
    }
}

bitflags! {
    /// A set of port sides, exposed via the `SIDES_*` named constants below.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct PortSideSet: u8 {
        const NORTH = 1 << 0;
        const EAST  = 1 << 1;
        const SOUTH = 1 << 2;
        const WEST  = 1 << 3;
    }
}

impl PortSideSet {
    pub const SIDES_NONE: Self = Self::empty();
    pub const SIDES_NORTH: Self = Self::NORTH;
    pub const SIDES_EAST: Self = Self::EAST;
    pub const SIDES_SOUTH: Self = Self::SOUTH;
    pub const SIDES_WEST: Self = Self::WEST;

    pub const SIDES_NORTH_SOUTH: Self = Self::NORTH.union(Self::SOUTH);
    pub const SIDES_EAST_WEST: Self = Self::EAST.union(Self::WEST);
    pub const SIDES_NORTH_WEST: Self = Self::NORTH.union(Self::WEST);
    pub const SIDES_NORTH_EAST: Self = Self::NORTH.union(Self::EAST);
    pub const SIDES_SOUTH_WEST: Self = Self::SOUTH.union(Self::WEST);
    pub const SIDES_EAST_SOUTH: Self = Self::EAST.union(Self::SOUTH);

    pub const SIDES_NORTH_EAST_WEST: Self = Self::NORTH.union(Self::EAST).union(Self::WEST);
    pub const SIDES_EAST_SOUTH_WEST: Self = Self::EAST.union(Self::SOUTH).union(Self::WEST);
    pub const SIDES_NORTH_SOUTH_WEST: Self = Self::NORTH.union(Self::SOUTH).union(Self::WEST);
    pub const SIDES_NORTH_EAST_SOUTH: Self = Self::NORTH.union(Self::EAST).union(Self::SOUTH);

    pub const SIDES_NORTH_EAST_SOUTH_WEST: Self =
        Self::NORTH.union(Self::EAST).union(Self::SOUTH).union(Self::WEST);
}

// Keep sparse label lists out of line so the common unlabeled port stays small.
#[allow(clippy::box_collection)]
#[derive(Default)]
pub struct PortLabels {
    labels: Option<Box<Vec<LabelId>>>,
}

impl PortLabels {
    #[inline]
    pub fn new() -> Self {
        Self { labels: None }
    }

    #[inline]
    pub fn push(&mut self, label: LabelId) {
        self.labels.get_or_insert_with(|| Box::new(Vec::new())).push(label);
    }

    #[inline]
    pub fn as_slice(&self) -> &[LabelId] {
        self.labels.as_deref().map(Vec::as_slice).unwrap_or(&[])
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [LabelId] {
        self.labels.as_deref_mut().map(Vec::as_mut_slice).unwrap_or(&mut [])
    }
}

impl Deref for PortLabels {
    type Target = [LabelId];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for PortLabels {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<'a> IntoIterator for &'a PortLabels {
    type IntoIter = slice::Iter<'a, LabelId>;
    type Item = &'a LabelId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

#[derive(Clone, Debug, Default)]
pub enum PortEdges {
    #[default]
    Empty,
    One(EdgeId),
    Many(Box<Vec<EdgeId>>),
}

impl PortEdges {
    #[inline]
    pub fn new() -> Self {
        Self::Empty
    }

    #[inline]
    pub fn push(&mut self, edge: EdgeId) {
        match self {
            Self::Empty => *self = Self::One(edge),
            Self::One(first) => {
                let edges = vec![*first, edge];
                *self = Self::Many(Box::new(edges));
            }
            Self::Many(edges) => edges.push(edge),
        }
    }

    #[inline]
    pub fn insert(&mut self, index: usize, edge: EdgeId) {
        match self {
            Self::Empty => {
                assert!(index == 0, "insertion index (is {index}) should be <= len (is 0)");
                *self = Self::One(edge);
            }
            Self::One(first) => {
                assert!(index <= 1, "insertion index (is {index}) should be <= len (is 1)");
                let mut edges = Vec::with_capacity(2);
                if index == 0 {
                    edges.push(edge);
                    edges.push(*first);
                } else {
                    edges.push(*first);
                    edges.push(edge);
                }
                *self = Self::Many(Box::new(edges));
            }
            Self::Many(edges) => edges.insert(index, edge),
        }
    }

    #[inline]
    pub fn retain<F>(&mut self, mut keep: F)
    where
        F: FnMut(&EdgeId) -> bool,
    {
        match self {
            Self::Empty => {}
            Self::One(edge) =>
                if !keep(edge) {
                    *self = Self::Empty;
                },
            Self::Many(edges) => {
                edges.retain(|edge| keep(edge));
                match edges.len() {
                    0 => *self = Self::Empty,
                    1 => *self = Self::One(edges[0]),
                    _ => {}
                }
            }
        }
    }

    #[inline]
    pub fn extend<I>(&mut self, edges: I)
    where
        I: IntoIterator<Item = EdgeId>,
    {
        for edge in edges {
            self.push(edge);
        }
    }

    #[inline]
    pub fn as_slice(&self) -> &[EdgeId] {
        match self {
            Self::Empty => &[],
            Self::One(edge) => slice::from_ref(edge),
            Self::Many(edges) => edges.as_slice(),
        }
    }

    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [EdgeId] {
        match self {
            Self::Empty => &mut [],
            Self::One(edge) => slice::from_mut(edge),
            Self::Many(edges) => edges.as_mut_slice(),
        }
    }
}

impl FromIterator<EdgeId> for PortEdges {
    fn from_iter<T: IntoIterator<Item = EdgeId>>(iter: T) -> Self {
        let mut edges = Self::new();
        edges.extend(iter);
        edges
    }
}

impl Deref for PortEdges {
    type Target = [EdgeId];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for PortEdges {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<'a> IntoIterator for &'a PortEdges {
    type IntoIter = slice::Iter<'a, EdgeId>;
    type Item = &'a EdgeId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

impl<'a> IntoIterator for &'a mut PortEdges {
    type IntoIter = slice::IterMut<'a, EdgeId>;
    type Item = &'a mut EdgeId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.as_mut_slice().iter_mut()
    }
}

pub enum PortEdgesIntoIter {
    Empty,
    One(Option<EdgeId>),
    Many(std::vec::IntoIter<EdgeId>),
}

impl Iterator for PortEdgesIntoIter {
    type Item = EdgeId;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Empty => None,
            Self::One(edge) => edge.take(),
            Self::Many(edges) => edges.next(),
        }
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        match self {
            Self::Empty => (0, Some(0)),
            Self::One(edge) => {
                let len = usize::from(edge.is_some());
                (len, Some(len))
            }
            Self::Many(edges) => edges.size_hint(),
        }
    }
}

impl ExactSizeIterator for PortEdgesIntoIter {}

impl IntoIterator for PortEdges {
    type IntoIter = PortEdgesIntoIter;
    type Item = EdgeId;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        match self {
            Self::Empty => PortEdgesIntoIter::Empty,
            Self::One(edge) => PortEdgesIntoIter::One(Some(edge)),
            Self::Many(edges) => PortEdgesIntoIter::Many((*edges).into_iter()),
        }
    }
}

/// Data stored for each port in the graph.
pub struct PortData {
    pub position: Vec2,
    pub size: Vec2,
    pub side: PortSide,
    pub anchor: Vec2,
    pub explicitly_supplied_anchor: bool,
    pub owner: NodeId,
    pub incoming_edges: PortEdges,
    pub outgoing_edges: PortEdges,
    pub labels: PortLabels,
    pub connected_to_external_nodes: bool,
    pub properties: PropertyMap,
    /// User-supplied identifier from the source graph. Empty for ports
    /// synthesised programmatically.
    pub identifier: Option<Box<String>>,
    /// Scratch ID for algorithms, assigned by `set_up_ids()`.
    pub id: u32,
    /// Promoted from `properties[PORT_DUMMY]`. The dummy node created for this
    /// port by N/S port preprocessing. Promoted out of `PropertyMap` because
    /// `SwitchDecider::constraints_prevent_switch` iterates every port of the
    /// pair on each n² query, and the per-port `PropertyMap` lookup (`Option`
    /// deref → HashMap probe → `dyn AnyClone` downcast → `Option<NodeId>` clone)
    /// dominated the inner sweep on external-port-heavy fixtures.
    pub port_dummy: Option<NodeId>,
    /// True if the importer synthesised this port for an edge endpoint that
    /// was attached directly to a node rather than to an explicit source-model
    /// port. Diagnostic dumps emit `i32::MAX` (`2147483647`) for these ports.
    /// Left at `false` for layout-internal dummies inserted by intermediate
    /// processors, since those are long-edge / N-S / self-loop scaffolding
    /// rather than original-model node endpoints.
    pub is_synthetic_for_node_endpoint: bool,
    /// Position of this port in the user-supplied source graph's port list
    /// at parse time, before any pipeline reordering by `PortListSorter` /
    /// `port_side_processor`. The original position is recorded here so
    /// diagnostic dumps can report the source-model port index even after the
    /// internal list has been sorted. `u32::MAX` for layout-internal dummies
    /// that have no source-graph counterpart.
    pub original_index: u32,
}

impl PortData {
    pub fn new(owner: NodeId, side: PortSide) -> Self {
        PortData {
            position: Vec2::ZERO,
            size: Vec2::ZERO,
            side,
            anchor: Vec2::ZERO,
            explicitly_supplied_anchor: false,
            owner,
            incoming_edges: PortEdges::new(),
            outgoing_edges: PortEdges::new(),
            labels: PortLabels::new(),
            connected_to_external_nodes: true,
            properties: PropertyMap::new(),
            identifier: None,
            id: 0,
            port_dummy: None,
            is_synthetic_for_node_endpoint: false,
            original_index: u32::MAX,
        }
    }
}
