use super::cell::CellBase;
use crate::math::{Padding, Vec2};

/// A leaf cell carrying an explicit minimum content-area size.
///
/// Atomic (leaf) cell used by the cell system.
#[derive(Debug, Default)]
pub struct AtomicCell {
    pub base: CellBase,
    pub min_content_area_size: Vec2,
}

impl AtomicCell {
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the cell's minimum content area size. If `includes_padding` is
    /// `true` the supplied size already bakes in this cell's padding, so the
    /// padding is subtracted before storing.
    pub fn set_min_content_area_size(&mut self, size: Vec2, includes_padding: bool) {
        if includes_padding {
            let p = self.base.padding;
            self.min_content_area_size =
                Vec2::new(size.x - p.left - p.right, size.y - p.top - p.bottom);
        } else {
            self.min_content_area_size = size;
        }
    }

    pub fn min_width(&self) -> f64 {
        pad(self.min_content_area_size.x, self.base.padding, true)
    }

    pub fn min_height(&self) -> f64 {
        pad(self.min_content_area_size.y, self.base.padding, false)
    }
}

fn pad(content: f64, padding: Padding, horizontal: bool) -> f64 {
    if horizontal {
        content + padding.left + padding.right
    } else {
        content + padding.top + padding.bottom
    }
}
