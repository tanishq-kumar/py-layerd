use bitflags::bitflags;
use smallvec::SmallVec;

use super::index::{LabelId, NodeId, PortId};
use crate::{math::Vec2, properties::PropertyMap};

bitflags! {
    /// Internal edge state flags.
    ///
    /// Hot, internal-only edge state that is accessed on algorithm paths.
    /// Contrast with `EdgeData.properties`, which holds cold configurable values.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct EdgeFlags: u8 {
        /// The edge has been reversed during cycle breaking.
        const REVERSED = 1 << 0;
    }
}

/// Data stored for each edge in the graph.
pub struct EdgeData {
    pub source: PortId,
    pub target: PortId,
    pub source_owner: NodeId,
    pub target_owner: NodeId,
    pub order: i32,
    pub start_point: Option<Vec2>,
    pub end_point: Option<Vec2>,
    pub bend_points: Vec<Vec2>,
    pub labels: SmallVec<LabelId, 2>,
    pub flags: EdgeFlags,
    pub properties: PropertyMap,
}

impl EdgeData {
    pub fn new(source: PortId, target: PortId, source_owner: NodeId, target_owner: NodeId) -> Self {
        EdgeData {
            source,
            target,
            source_owner,
            target_owner,
            order: i32::MAX,
            start_point: None,
            end_point: None,
            bend_points: Vec::new(),
            labels: SmallVec::new(),
            flags: EdgeFlags::empty(),
            properties: PropertyMap::new(),
        }
    }
}
