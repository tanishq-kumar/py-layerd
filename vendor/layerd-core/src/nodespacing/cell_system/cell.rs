use super::{AtomicCell, GridContainerCell, LabelCell, StripContainerCell};
use crate::math::{Padding, Vec2};

/// Axis-aligned rectangle with origin (top-left) and size.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl Rect {
    pub fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        Rect { x, y, width, height }
    }
}

/// Row or column position inside a 3-slot strip or a 3x3 grid container.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContainerArea {
    Begin,
    Center,
    End,
}

impl ContainerArea {
    pub const ALL: [ContainerArea; 3] = [Self::Begin, Self::Center, Self::End];

    pub fn ordinal(self) -> usize {
        match self {
            Self::Begin => 0,
            Self::Center => 1,
            Self::End => 2,
        }
    }
}

/// Strip container orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Strip {
    Vertical,
    Horizontal,
}

/// Horizontal alignment of labels within a `LabelCell`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HorizontalLabelAlignment {
    Left,
    #[default]
    Center,
    Right,
}

/// Vertical alignment of labels within a `LabelCell`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VerticalLabelAlignment {
    Top,
    #[default]
    Center,
    Bottom,
}

/// Common fields shared by every cell variant.
#[derive(Debug, Clone, Copy, Default)]
pub struct CellBase {
    pub padding: Padding,
    pub rect: Rect,
    pub contributes_to_min_width: bool,
    pub contributes_to_min_height: bool,
}

/// Static dispatch over every supported cell variant.
#[derive(Debug)]
pub enum CellKind {
    Atomic(Box<AtomicCell>),
    Label(Box<LabelCell>),
    Strip(Box<StripContainerCell>),
    Grid(Box<GridContainerCell>),
}

impl CellKind {
    pub fn base(&self) -> &CellBase {
        match self {
            Self::Atomic(c) => &c.base,
            Self::Label(c) => &c.base,
            Self::Strip(c) => &c.base,
            Self::Grid(c) => &c.base,
        }
    }

    pub fn base_mut(&mut self) -> &mut CellBase {
        match self {
            Self::Atomic(c) => &mut c.base,
            Self::Label(c) => &mut c.base,
            Self::Strip(c) => &mut c.base,
            Self::Grid(c) => &mut c.base,
        }
    }

    pub fn min_width(&self) -> f64 {
        match self {
            Self::Atomic(c) => c.min_width(),
            Self::Label(c) => c.min_width(),
            Self::Strip(c) => c.min_width(),
            Self::Grid(c) => c.min_width(),
        }
    }

    pub fn min_height(&self) -> f64 {
        match self {
            Self::Atomic(c) => c.min_height(),
            Self::Label(c) => c.min_height(),
            Self::Strip(c) => c.min_height(),
            Self::Grid(c) => c.min_height(),
        }
    }

    /// Applies a new horizontal position and width to the cell.
    pub fn apply_horizontal_layout(&mut self, x: f64, width: f64) {
        let rect = &mut self.base_mut().rect;
        rect.x = x;
        rect.width = width;
    }

    /// Applies a new vertical position and height to the cell.
    pub fn apply_vertical_layout(&mut self, y: f64, height: f64) {
        let rect = &mut self.base_mut().rect;
        rect.y = y;
        rect.height = height;
    }

    /// Returns the minimum width a cell contributes to a container's size,
    /// honoring the contribution flag and the "empty atomic" special case.
    pub fn min_width_contribution(cell: Option<&CellKind>, respect_flag: bool) -> f64 {
        let Some(cell) = cell else {
            return 0.0;
        };
        if respect_flag && !cell.base().contributes_to_min_width {
            return 0.0;
        }
        if let Self::Atomic(atomic) = cell
            && atomic.min_content_area_size.x == 0.0
        {
            return 0.0;
        }
        cell.min_width()
    }

    /// Returns the minimum height a cell contributes to a container's size,
    /// honoring the contribution flag and the "empty atomic" special case.
    pub fn min_height_contribution(cell: Option<&CellKind>, respect_flag: bool) -> f64 {
        let Some(cell) = cell else {
            return 0.0;
        };
        if respect_flag && !cell.base().contributes_to_min_height {
            return 0.0;
        }
        if let Self::Atomic(atomic) = cell
            && atomic.min_content_area_size.y == 0.0
        {
            return 0.0;
        }
        cell.min_height()
    }

    /// Propagates horizontal layout to container children after this cell's
    /// own rectangle has been fixed.
    pub fn layout_children_horizontally(&mut self) {
        match self {
            Self::Strip(c) => c.layout_children_horizontally(),
            Self::Grid(c) => c.layout_children_horizontally(),
            _ => {}
        }
    }

    /// Propagates vertical layout to container children after this cell's
    /// own rectangle has been fixed.
    pub fn layout_children_vertically(&mut self) {
        match self {
            Self::Strip(c) => c.layout_children_vertically(),
            Self::Grid(c) => c.layout_children_vertically(),
            _ => {}
        }
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<CellBase>();
    }
}

/// Convenience factories for wrapping a concrete cell into a `CellKind`.
impl From<AtomicCell> for CellKind {
    fn from(c: AtomicCell) -> Self {
        CellKind::Atomic(Box::new(c))
    }
}

impl From<LabelCell> for CellKind {
    fn from(c: LabelCell) -> Self {
        CellKind::Label(Box::new(c))
    }
}

impl From<StripContainerCell> for CellKind {
    fn from(c: StripContainerCell) -> Self {
        CellKind::Strip(Box::new(c))
    }
}

impl From<GridContainerCell> for CellKind {
    fn from(c: GridContainerCell) -> Self {
        CellKind::Grid(Box::new(c))
    }
}

/// Pads the given content size by the cell's padding.
pub(super) fn pad_width(content: f64, padding: Padding) -> f64 {
    if content > 0.0 { content + padding.horizontal() } else { 0.0 }
}

pub(super) fn pad_height(content: f64, padding: Padding) -> f64 {
    if content > 0.0 { content + padding.vertical() } else { 0.0 }
}

/// Sum a triplet with a gap between every pair of non-zero entries.
pub(super) fn sum_with_gaps(values: [f64; 3], gap: f64) -> f64 {
    let mut sum = 0.0;
    let mut active = 0;
    for v in values {
        if v > 0.0 {
            sum += v;
            active += 1;
        }
    }
    if active > 1 {
        sum += gap * (active as f64 - 1.0);
    }
    sum
}

#[allow(dead_code)]
pub(super) fn zero_vec() -> Vec2 {
    Vec2::ZERO
}
