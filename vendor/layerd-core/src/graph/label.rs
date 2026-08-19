use smallvec::SmallVec;

use crate::{graph::index::LabelId, math::Vec2, properties::PropertyMap};

/// Data stored for each label in the graph.
pub struct LabelData {
    pub position: Vec2,
    pub size: Vec2,
    pub text: String,
    pub properties: PropertyMap,
}

impl LabelData {
    pub fn new(text: impl Into<String>, size: Vec2) -> Self {
        LabelData { position: Vec2::ZERO, size, text: text.into(), properties: PropertyMap::new() }
    }
}

/// Container for grouped edge labels at a node's end.
///
/// Groups labels for sorting and positioning by the `EndLabelSorter` and
/// `EndLabelPreprocessor`.
#[derive(Debug, Clone, Default)]
pub struct LabelCell {
    pub labels: SmallVec<LabelId, 4>,
}
