//! Labels associated with a self-hyper-loop.
//!
//! Gathers edge labels while a self-hyper-loop is being constructed and later
//! places them relative to the loop's trunk. The actual placement decision is
//! made by `LabelPlacer` during routing; this module owns the storage and the
//! bounding-box arithmetic.

use crate::{
    graph::{
        LGraph,
        index::{LabelId, PortId},
        port::PortSide,
    },
    math::Vec2,
    options::enums::LayoutDirection,
};

/// Describes how a label is aligned relative to its host side. Most alignments
/// apply to northern and southern labels; eastern and western labels are
/// restricted to `Top`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alignment {
    /// Northern or southern centered label.
    Center,
    /// Northern or southern left-aligned label.
    Left,
    /// Northern or southern right-aligned label.
    Right,
    /// Eastern or western top-aligned label.
    Top,
}

/// Labels attached to a `SelfHyperLoop`. Collected as edges are added to the
/// loop and placed later by the routing stage.
#[derive(Debug, Clone)]
pub struct SelfHyperLoopLabels {
    /// Arbitrary identifier, used by label placement passes for tie-breaking.
    pub id: u32,
    /// Labels represented by this instance.
    pub labels: Vec<LabelId>,
    /// Size required to place all labels.
    pub size: Vec2,
    /// Top-left corner of the bounding box once placement has been computed.
    pub position: Vec2,
    /// Layout direction inherited from the owning graph. Decides whether the
    /// labels stack vertically (horizontal layout) or horizontally (vertical
    /// layout).
    pub layout_direction: LayoutDirection,
    /// Space to leave between adjacent labels (`SPACING_LABEL_LABEL`).
    pub label_label_spacing: f64,
    /// Side the label is placed on. `None` until determined by `LabelPlacer`.
    pub side: Option<PortSide>,
    /// Horizontal or vertical alignment on the chosen side.
    pub alignment: Option<Alignment>,
    /// Port that a non-centered alignment is measured against.
    pub alignment_reference_port: Option<PortId>,
}

impl SelfHyperLoopLabels {
    /// Create an empty container for the given layout direction and spacing.
    pub fn new(layout_direction: LayoutDirection, label_label_spacing: f64) -> Self {
        SelfHyperLoopLabels {
            id: 0,
            labels: Vec::new(),
            size: Vec2::ZERO,
            position: Vec2::ZERO,
            layout_direction,
            label_label_spacing,
            side: None,
            alignment: None,
            alignment_reference_port: None,
        }
    }

    /// Append a label and grow the bounding box accordingly.
    pub fn add_label(&mut self, label_id: LabelId, label_size: Vec2) {
        self.labels.push(label_id);
        self.update_size(label_size);
    }

    /// Grow the bounding box under the assumption that `new_label_size` was
    /// just appended.
    fn update_size(&mut self, new_label_size: Vec2) {
        if is_horizontal(self.layout_direction) {
            // Labels stack vertically.
            self.size.x = self.size.x.max(new_label_size.x);
            self.size.y += new_label_size.y;
            if self.labels.len() > 1 {
                self.size.y += self.label_label_spacing;
            }
        } else {
            // Labels stack horizontally.
            self.size.x += new_label_size.x;
            self.size.y = self.size.y.max(new_label_size.y);
            if self.labels.len() > 1 {
                self.size.x += self.label_label_spacing;
            }
        }
    }

    /// Apply the stored bounding-box placement to the individual labels.
    pub fn apply_placement(&self, graph: &mut LGraph, offset: Vec2) {
        if is_horizontal(self.layout_direction) {
            self.apply_placement_horizontal(graph, offset);
        } else {
            self.apply_placement_vertical(graph, offset);
        }
    }

    fn apply_placement_horizontal(&self, graph: &mut LGraph, offset: Vec2) {
        let x = self.position.x;
        let mut y = self.position.y;

        for &lid in &self.labels {
            let label_size = graph.label(lid).size;
            let label_pos = {
                let lx = match (self.alignment, self.side) {
                    (Some(Alignment::Left), _) | (_, Some(PortSide::East)) => x,
                    (Some(Alignment::Right), _) | (_, Some(PortSide::West)) =>
                        x + self.size.x - label_size.x,
                    _ => x + (self.size.x - label_size.x) / 2.0,
                };
                Vec2::new(lx + offset.x, y + offset.y)
            };
            graph.label_mut(lid).position = label_pos;
            y += label_size.y + self.label_label_spacing;
        }
    }

    fn apply_placement_vertical(&self, graph: &mut LGraph, offset: Vec2) {
        let mut x = self.position.x;
        let y = self.position.y;

        for &lid in &self.labels {
            let label_size = graph.label(lid).size;
            // Top-align everywhere except the northern side, which bottom-aligns
            // so the labels sit against the node edge.
            let ly = if self.side == Some(PortSide::North) {
                y + self.size.y - label_size.y
            } else {
                y
            };
            graph.label_mut(lid).position = Vec2::new(x + offset.x, ly + offset.y);
            x += label_size.x + self.label_label_spacing;
        }
    }
}

/// True when the layout direction is along the x axis.
#[inline]
pub fn is_horizontal(direction: LayoutDirection) -> bool {
    matches!(
        direction,
        LayoutDirection::Right | LayoutDirection::Left | LayoutDirection::Undefined
    )
}
