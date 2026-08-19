use super::cell::{CellBase, CellKind, ContainerArea, Strip, pad_height, pad_width, sum_with_gaps};
use crate::math::Vec2;

/// Strip container: three child cells laid out as either rows or columns.
///
/// Strip container cell: a row or column of child cells.
#[derive(Debug)]
pub struct StripContainerCell {
    pub base: CellBase,
    mode: Strip,
    symmetrical: bool,
    gap: f64,
    cells: [Option<CellKind>; 3],
}

impl StripContainerCell {
    pub fn new(mode: Strip, symmetrical: bool, gap: f64) -> Self {
        StripContainerCell {
            base: CellBase::default(),
            mode,
            symmetrical,
            gap,
            cells: [None, None, None],
        }
    }

    pub fn cell(&self, area: ContainerArea) -> Option<&CellKind> {
        self.cells[area.ordinal()].as_ref()
    }

    pub fn cell_mut(&mut self, area: ContainerArea) -> Option<&mut CellKind> {
        self.cells[area.ordinal()].as_mut()
    }

    pub fn set_cell(&mut self, area: ContainerArea, cell: Option<CellKind>) {
        self.cells[area.ordinal()] = cell;
    }

    pub fn min_width(&self) -> f64 {
        let width = if self.mode == Strip::Vertical {
            // All children laid out as full-width rows.
            self.cells
                .iter()
                .filter_map(|c| {
                    let cell = c.as_ref()?;
                    if cell.base().contributes_to_min_width { Some(cell.min_width()) } else { None }
                })
                .fold(0.0f64, f64::max)
        } else {
            let widths = self.cell_widths(true);
            sum_with_gaps(widths, self.gap)
        };
        pad_width(width, self.base.padding)
    }

    pub fn min_height(&self) -> f64 {
        let height = if self.mode == Strip::Vertical {
            let heights = self.cell_heights(true);
            sum_with_gaps(heights, self.gap)
        } else {
            self.cells
                .iter()
                .filter_map(|c| {
                    let cell = c.as_ref()?;
                    if cell.base().contributes_to_min_height {
                        Some(cell.min_height())
                    } else {
                        None
                    }
                })
                .fold(0.0f64, f64::max)
        };
        pad_height(height, self.base.padding)
    }

    pub fn layout_children_horizontally(&mut self) {
        let rect = self.base.rect;
        let padding = self.base.padding;

        if self.mode == Strip::Vertical {
            let x_pos = rect.x + padding.left;
            let width = rect.width - padding.left - padding.right;
            for slot in &mut self.cells {
                if let Some(child) = slot.as_mut() {
                    child.apply_horizontal_layout(x_pos, width);
                }
            }
        } else {
            let mut cell_widths = self.cell_widths(false);

            // Left-aligned Begin.
            if self.cells[0].is_some() {
                let x = rect.x + padding.left;
                self.cells[0].as_mut().unwrap().apply_horizontal_layout(x, cell_widths[0]);
            }
            // Right-aligned End.
            if self.cells[2].is_some() {
                let x = rect.x + rect.width - padding.right - cell_widths[2];
                self.cells[2].as_mut().unwrap().apply_horizontal_layout(x, cell_widths[2]);
            }

            let mut free = rect.width - padding.left - padding.right;
            if cell_widths[0] > 0.0 {
                free -= cell_widths[0] + self.gap;
                cell_widths[0] += self.gap;
            }
            if cell_widths[2] > 0.0 {
                free -= cell_widths[2] + self.gap;
            }
            cell_widths[1] = cell_widths[1].max(free);

            if self.cells[1].is_some() {
                let x = rect.x + padding.left + cell_widths[0] - (cell_widths[1] - free) / 2.0;
                self.cells[1].as_mut().unwrap().apply_horizontal_layout(x, cell_widths[1]);
            }
        }

        for slot in &mut self.cells {
            if let Some(child) = slot.as_mut() {
                child.layout_children_horizontally();
            }
        }
    }

    pub fn layout_children_vertically(&mut self) {
        let rect = self.base.rect;
        let padding = self.base.padding;

        if self.mode == Strip::Vertical {
            let mut cell_heights = self.cell_heights(false);

            if self.cells[0].is_some() {
                let y = rect.y + padding.top;
                self.cells[0].as_mut().unwrap().apply_vertical_layout(y, cell_heights[0]);
            }
            if self.cells[2].is_some() {
                let y = rect.y + rect.height - padding.bottom - cell_heights[2];
                self.cells[2].as_mut().unwrap().apply_vertical_layout(y, cell_heights[2]);
            }

            let content_height = rect.height - padding.top - padding.bottom;
            let mut free = content_height;
            if cell_heights[0] > 0.0 {
                cell_heights[0] += self.gap;
                free -= cell_heights[0];
            }
            if cell_heights[2] > 0.0 {
                free -= cell_heights[2] + self.gap;
            }
            cell_heights[1] = cell_heights[1].max(free);

            if self.cells[1].is_some() {
                let y = rect.y + padding.top + cell_heights[0] - (cell_heights[1] - free) / 2.0;
                self.cells[1].as_mut().unwrap().apply_vertical_layout(y, cell_heights[1]);
            }
        } else {
            let y_pos = rect.y + padding.top;
            let height = rect.height - padding.top - padding.bottom;
            for slot in &mut self.cells {
                if let Some(child) = slot.as_mut() {
                    child.apply_vertical_layout(y_pos, height);
                }
            }
        }

        for slot in &mut self.cells {
            if let Some(child) = slot.as_mut() {
                child.layout_children_vertically();
            }
        }
    }

    fn cell_widths(&self, respect_flag: bool) -> [f64; 3] {
        let mut widths = [
            CellKind::min_width_contribution(self.cells[0].as_ref(), respect_flag),
            CellKind::min_width_contribution(self.cells[1].as_ref(), respect_flag),
            CellKind::min_width_contribution(self.cells[2].as_ref(), respect_flag),
        ];
        if self.symmetrical {
            let outer = widths[0].max(widths[2]);
            widths[0] = outer;
            widths[2] = outer;
        }
        widths
    }

    fn cell_heights(&self, respect_flag: bool) -> [f64; 3] {
        let mut heights = [
            CellKind::min_height_contribution(self.cells[0].as_ref(), respect_flag),
            CellKind::min_height_contribution(self.cells[1].as_ref(), respect_flag),
            CellKind::min_height_contribution(self.cells[2].as_ref(), respect_flag),
        ];
        if self.symmetrical {
            let outer = heights[0].max(heights[2]);
            heights[0] = outer;
            heights[2] = outer;
        }
        heights
    }
}

/// Grid container: 3x3 cells.
///
/// Grid container cell: a 2D grid of child cells.
#[derive(Debug)]
pub struct GridContainerCell {
    pub base: CellBase,
    gap: f64,
    tabular: bool,
    symmetrical: bool,
    cells: [[Option<CellKind>; 3]; 3],
    center_min_size: Option<Vec2>,
    only_center_contributes: bool,
    center_rect: super::Rect,
}

impl GridContainerCell {
    pub fn new(tabular: bool, symmetrical: bool, gap: f64) -> Self {
        GridContainerCell {
            base: CellBase::default(),
            gap,
            tabular,
            symmetrical,
            cells: [[None, None, None], [None, None, None], [None, None, None]],
            center_min_size: None,
            only_center_contributes: false,
            center_rect: super::Rect::default(),
        }
    }

    pub fn cell(&self, row: ContainerArea, column: ContainerArea) -> Option<&CellKind> {
        self.cells[row.ordinal()][column.ordinal()].as_ref()
    }

    pub fn set_cell(&mut self, row: ContainerArea, column: ContainerArea, cell: Option<CellKind>) {
        self.cells[row.ordinal()][column.ordinal()] = cell;
    }

    pub fn set_center_min_size(&mut self, size: Vec2) {
        self.center_min_size = Some(size);
    }

    pub fn set_only_center_contributes(&mut self, flag: bool) {
        self.only_center_contributes = flag;
    }

    pub fn min_width(&self) -> f64 {
        let width = if self.only_center_contributes {
            if let Some(size) = self.center_min_size {
                size.x
            } else if let Some(center) = self.cells[1][1].as_ref() {
                center.min_width()
            } else {
                0.0
            }
        } else if self.tabular {
            sum_with_gaps(self.min_column_widths(None, true), self.gap)
        } else {
            ContainerArea::ALL
                .iter()
                .map(|area| sum_with_gaps(self.min_column_widths(Some(*area), true), self.gap))
                .fold(0.0f64, f64::max)
        };
        pad_width(width, self.base.padding)
    }

    pub fn min_height(&self) -> f64 {
        let height = if self.only_center_contributes {
            if let Some(size) = self.center_min_size {
                size.y
            } else if let Some(center) = self.cells[1][1].as_ref() {
                center.min_height()
            } else {
                0.0
            }
        } else {
            sum_with_gaps(self.min_row_heights(true), self.gap)
        };
        pad_height(height, self.base.padding)
    }

    pub fn layout_children_horizontally(&mut self) {
        if self.tabular {
            let col_widths = self.min_column_widths(None, false);
            for row in ContainerArea::ALL {
                self.apply_widths_to_row(row, col_widths);
            }
        } else {
            for row in ContainerArea::ALL {
                let col_widths = self.min_column_widths(Some(row), false);
                self.apply_widths_to_row(row, col_widths);
            }
        }

        for row in &mut self.cells {
            for slot in row {
                if let Some(child) = slot.as_mut() {
                    child.layout_children_horizontally();
                }
            }
        }
    }

    pub fn layout_children_vertically(&mut self) {
        let rect = self.base.rect;
        let padding = self.base.padding;

        let mut row_heights = self.min_row_heights(false);

        // Top row.
        let top_y = rect.y + padding.top;
        self.apply_heights_to_row(ContainerArea::Begin, top_y, row_heights);
        // Bottom row.
        let bottom_y = rect.y + rect.height - padding.bottom - row_heights[2];
        self.apply_heights_to_row(ContainerArea::End, bottom_y, row_heights);

        let mut free = rect.height - padding.top - padding.bottom;
        if row_heights[0] > 0.0 {
            row_heights[0] += self.gap;
            free -= row_heights[0];
        }
        if row_heights[2] > 0.0 {
            row_heights[2] += self.gap;
            free -= row_heights[2];
        }

        self.center_rect.height = free.max(0.0);
        self.center_rect.y = rect.y + padding.top + (self.center_rect.height - free) / 2.0;

        row_heights[1] = row_heights[1].max(free);

        let center_y = rect.y + padding.top + row_heights[0] - (row_heights[1] - free) / 2.0;
        self.apply_heights_to_row(ContainerArea::Center, center_y, row_heights);

        for row in &mut self.cells {
            for slot in row {
                if let Some(child) = slot.as_mut() {
                    child.layout_children_vertically();
                }
            }
        }
    }

    fn min_column_widths(&self, row: Option<ContainerArea>, respect_flag: bool) -> [f64; 3] {
        let mut widths = [
            self.min_width_of_column(ContainerArea::Begin, row, respect_flag),
            self.min_width_of_column(ContainerArea::Center, row, respect_flag),
            self.min_width_of_column(ContainerArea::End, row, respect_flag),
        ];
        if self.symmetrical {
            let outer = widths[0].max(widths[2]);
            widths[0] = outer;
            widths[2] = outer;
        }
        widths
    }

    fn min_width_of_column(
        &self,
        column: ContainerArea,
        row: Option<ContainerArea>,
        respect_flag: bool,
    ) -> f64 {
        let mut max = 0.0f64;
        match row {
            None =>
                for r in 0..3 {
                    let value = CellKind::min_width_contribution(
                        self.cells[r][column.ordinal()].as_ref(),
                        respect_flag,
                    );
                    if value > max {
                        max = value;
                    }
                },
            Some(r) => {
                max = CellKind::min_width_contribution(
                    self.cells[r.ordinal()][column.ordinal()].as_ref(),
                    respect_flag,
                );
            }
        }
        if column == ContainerArea::Center
            && let Some(size) = self.center_min_size
            && max < size.x
        {
            max = size.x;
        }
        max
    }

    fn min_row_heights(&self, respect_flag: bool) -> [f64; 3] {
        let mut heights = [
            self.min_height_of_row(ContainerArea::Begin, respect_flag),
            self.min_height_of_row(ContainerArea::Center, respect_flag),
            self.min_height_of_row(ContainerArea::End, respect_flag),
        ];
        if self.symmetrical {
            let outer = heights[0].max(heights[2]);
            heights[0] = outer;
            heights[2] = outer;
        }
        heights
    }

    fn min_height_of_row(&self, row: ContainerArea, respect_flag: bool) -> f64 {
        let mut max = 0.0f64;
        for c in 0..3 {
            let value = CellKind::min_height_contribution(
                self.cells[row.ordinal()][c].as_ref(),
                respect_flag,
            );
            if value > max {
                max = value;
            }
        }
        if row == ContainerArea::Center
            && let Some(size) = self.center_min_size
            && max < size.y
        {
            max = size.y;
        }
        max
    }

    fn apply_widths_to_row(&mut self, row: ContainerArea, mut col_widths: [f64; 3]) {
        let rect = self.base.rect;
        let padding = self.base.padding;

        // Begin column is left-aligned with content area.
        let begin_x = rect.x + padding.left;
        self.apply_width_to_column(ContainerArea::Begin, begin_x, col_widths);
        // End column is right-aligned.
        let end_x = rect.x + rect.width - padding.right - col_widths[2];
        self.apply_width_to_column(ContainerArea::End, end_x, col_widths);

        let mut free = rect.width - padding.left - padding.right;
        if col_widths[0] > 0.0 {
            col_widths[0] += self.gap;
            free -= col_widths[0];
        }
        if col_widths[2] > 0.0 {
            col_widths[2] += self.gap;
            free -= col_widths[2];
        }

        let center_width = free.max(0.0);
        col_widths[1] = col_widths[1].max(free);

        let center_x = rect.x + padding.left + col_widths[0] - (col_widths[1] - free) / 2.0;
        self.apply_width_to_column(ContainerArea::Center, center_x, col_widths);

        if row == ContainerArea::Center {
            self.center_rect.width = center_width;
            self.center_rect.x = rect.x + padding.left + (center_width - free) / 2.0;
        }
    }

    fn apply_width_to_column(&mut self, column: ContainerArea, x: f64, col_widths: [f64; 3]) {
        let width = col_widths[column.ordinal()];
        for row in 0..3 {
            if let Some(child) = self.cells[row][column.ordinal()].as_mut() {
                child.apply_horizontal_layout(x, width);
            }
        }
    }

    fn apply_heights_to_row(&mut self, row: ContainerArea, y: f64, row_heights: [f64; 3]) {
        let height = row_heights[row.ordinal()];
        for column in 0..3 {
            if let Some(child) = self.cells[row.ordinal()][column].as_mut() {
                child.apply_vertical_layout(y, height);
            }
        }
    }
}
