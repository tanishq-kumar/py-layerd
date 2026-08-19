use smallvec::SmallVec;

use super::cell::{CellBase, HorizontalLabelAlignment, VerticalLabelAlignment};
use crate::{graph::index::LabelId, math::Vec2};

/// A label paired with its size, stored inside a `LabelCell`.
#[derive(Debug, Clone, Copy)]
pub struct LabelEntry {
    pub id: LabelId,
    pub size: Vec2,
}

/// Positions a stack of labels with a configurable gap.
///
/// Cell that holds one or more labels.
/// Layout is driven by `apply_label_layout`, which produces the final
/// coordinate for every contained label. Callers are expected to write those
/// coordinates back onto their own label storage (graph arena etc.).
#[derive(Debug)]
pub struct LabelCell {
    pub base: CellBase,
    horizontal_layout_mode: bool,
    horizontal_alignment: HorizontalLabelAlignment,
    vertical_alignment: VerticalLabelAlignment,
    gap: f64,
    labels: SmallVec<LabelEntry, 2>,
    min_content_area_size: Vec2,
}

impl LabelCell {
    pub fn new(gap: f64) -> Self {
        LabelCell {
            base: CellBase::default(),
            horizontal_layout_mode: true,
            horizontal_alignment: HorizontalLabelAlignment::Center,
            vertical_alignment: VerticalLabelAlignment::Center,
            gap,
            labels: SmallVec::new(),
            min_content_area_size: Vec2::ZERO,
        }
    }

    pub fn with_layout_mode(gap: f64, horizontal_layout_mode: bool) -> Self {
        let mut cell = Self::new(gap);
        cell.horizontal_layout_mode = horizontal_layout_mode;
        cell
    }

    pub fn set_horizontal_alignment(&mut self, alignment: HorizontalLabelAlignment) -> &mut Self {
        self.horizontal_alignment = alignment;
        self
    }

    pub fn set_vertical_alignment(&mut self, alignment: VerticalLabelAlignment) -> &mut Self {
        self.vertical_alignment = alignment;
        self
    }

    /// Appends a label and grows the cell's minimum content area size
    /// accordingly.
    pub fn add_label(&mut self, id: LabelId, size: Vec2) {
        let first = self.labels.is_empty();
        self.labels.push(LabelEntry { id, size });
        if self.horizontal_layout_mode {
            if self.min_content_area_size.x < size.x {
                self.min_content_area_size.x = size.x;
            }
            self.min_content_area_size.y += size.y;
            if !first {
                self.min_content_area_size.y += self.gap;
            }
        } else {
            self.min_content_area_size.x += size.x;
            if self.min_content_area_size.y < size.y {
                self.min_content_area_size.y = size.y;
            }
            if !first {
                self.min_content_area_size.x += self.gap;
            }
        }
    }

    pub fn min_width(&self) -> f64 {
        let p = self.base.padding;
        self.min_content_area_size.x + p.left + p.right
    }

    pub fn min_height(&self) -> f64 {
        let p = self.base.padding;
        self.min_content_area_size.y + p.top + p.bottom
    }

    /// Computes final positions for each label inside this cell.
    pub fn apply_label_layout(&self) -> SmallVec<(LabelId, Vec2), 2> {
        if self.horizontal_layout_mode {
            self.layout_horizontal()
        } else {
            self.layout_vertical()
        }
    }

    fn layout_horizontal(&self) -> SmallVec<(LabelId, Vec2), 2> {
        let rect = self.base.rect;
        let padding = self.base.padding;

        let mut y_pos = rect.y;
        match self.vertical_alignment {
            VerticalLabelAlignment::Top => {}
            VerticalLabelAlignment::Center => {
                y_pos += (rect.height - self.min_content_area_size.y) / 2.0;
            }
            VerticalLabelAlignment::Bottom => {
                y_pos += rect.height - self.min_content_area_size.y;
            }
        }

        let mut out = SmallVec::with_capacity(self.labels.len());
        for entry in &self.labels {
            let x = match self.horizontal_alignment {
                HorizontalLabelAlignment::Left => rect.x + padding.left,
                HorizontalLabelAlignment::Center =>
                    rect.x + padding.left + (rect.width - entry.size.x) / 2.0,
                HorizontalLabelAlignment::Right =>
                    rect.x + rect.width - padding.right - entry.size.x,
            };
            out.push((entry.id, Vec2::new(x, y_pos)));
            y_pos += entry.size.y + self.gap;
        }
        out
    }

    fn layout_vertical(&self) -> SmallVec<(LabelId, Vec2), 2> {
        let rect = self.base.rect;
        let padding = self.base.padding;

        let mut x_pos = rect.x;
        match self.horizontal_alignment {
            HorizontalLabelAlignment::Left => {}
            HorizontalLabelAlignment::Center => {
                x_pos += (rect.width - self.min_content_area_size.x) / 2.0;
            }
            HorizontalLabelAlignment::Right => {
                x_pos += rect.width - self.min_content_area_size.x;
            }
        }

        let mut out = SmallVec::with_capacity(self.labels.len());
        for entry in &self.labels {
            let y = match self.vertical_alignment {
                VerticalLabelAlignment::Top => rect.y + padding.top,
                VerticalLabelAlignment::Center =>
                    rect.y + padding.top + (rect.height - entry.size.y) / 2.0,
                VerticalLabelAlignment::Bottom =>
                    rect.y + rect.height - padding.bottom - entry.size.y,
            };
            out.push((entry.id, Vec2::new(x_pos, y)));
            x_pos += entry.size.x + self.gap;
        }
        out
    }
}
