//! Cut-index calculators pick the layer indexes at which the breaking-point
//! processor should split a wide-and-narrow layering. The manual variant reads explicit cuts from
//! `LayoutOptions.wrapping_cutting_cuts`. ARD solves for the chunk count
//! that matches the desired aspect ratio. MSD minimizes the maximum scale.

use super::graph_stats::GraphStats;
use crate::{graph::LGraph, options::enums::WrappingCuttingStrategy};

/// Result returned by a cut-index strategy.
pub struct CutIndexes {
    pub indexes: Vec<i32>,
    pub guaranteed_valid: bool,
}

/// Compute cut indexes for `strategy`.
pub fn calculate(
    strategy: WrappingCuttingStrategy,
    graph: &LGraph,
    stats: &mut GraphStats<'_>,
) -> CutIndexes {
    let indexes = match strategy {
        WrappingCuttingStrategy::Manual => manual_cut_indexes(graph),
        WrappingCuttingStrategy::Ard => ard_cut_indexes(stats),
        WrappingCuttingStrategy::Msd => msd_cut_indexes(graph, stats),
    };
    CutIndexes { indexes, guaranteed_valid: false }
}

/// Returns the explicit cut list from the graph options.
pub fn manual_cut_indexes(graph: &LGraph) -> Vec<i32> {
    graph.options.wrapping_cutting_cuts.clone().unwrap_or_default()
}

/// Aspect-ratio-driven chunk-count heuristic.
pub fn ard_cut_indexes(stats: &mut GraphStats<'_>) -> Vec<i32> {
    let rows = chunk_count(stats);
    if rows <= 1 {
        return Vec::new();
    }
    let step = stats.longest_path as f64 / rows as f64;
    let mut cuts = Vec::with_capacity(rows.saturating_sub(1));
    for idx in 1..rows {
        cuts.push((idx as f64 * step).round() as i32);
    }
    cuts
}

/// Returns the chunk count implied by the graph stats, clamped to the number
/// of layers. Shared with the MSD heuristic.
pub fn chunk_count(stats: &mut GraphStats<'_>) -> usize {
    let sum_width = stats.sum_width();
    let max_height = stats.max_height();
    if max_height <= 0.0 {
        return 0;
    }
    let denom = stats.dar * max_height;
    if denom <= 0.0 {
        return 0;
    }
    let rowsd = (sum_width / denom).sqrt();
    let rows = rowsd.round() as isize;
    let rows = rows.max(0) as usize;
    rows.min(stats.longest_path)
}

/// Mean-standard-deviation heuristic.
pub fn msd_cut_indexes(graph: &LGraph, stats: &mut GraphStats<'_>) -> Vec<i32> {
    let (widths, heights) = {
        let _ = stats.widths();
        (stats.widths().to_vec(), stats.heights().to_vec())
    };
    let n = widths.len();
    if n == 0 {
        return Vec::new();
    }

    // width_at_index[i] = sum(widths[0..=i])
    let mut width_at_index = vec![0.0_f64; n];
    width_at_index[0] = widths[0];
    let mut total = widths[0];
    for i in 1..n {
        width_at_index[i] = width_at_index[i - 1] + widths[i];
        total += widths[i];
    }

    let initial_cuts = chunk_count(stats).saturating_sub(1) as i32;
    let freedom = graph.options.wrapping_cutting_msd_freedom;

    let mut best_max_scale = f64::NEG_INFINITY;
    let mut best_cuts: Vec<i32> = Vec::new();
    let longest_path = stats.longest_path as i32;

    let m_lo = (initial_cuts - freedom).max(0);
    let m_hi = (initial_cuts + freedom).min(longest_path - 1).max(0);

    for m in m_lo..=m_hi {
        let mut width = f64::NEG_INFINITY;
        let height;
        let mut cuts: Vec<i32> = Vec::new();

        if m == 0 {
            width = total;
            height = stats.max_height();
        } else {
            let row_sum = total / (m as f64 + 1.0);
            let mut sum_so_far = 0.0_f64;
            let mut last_cut_width = 0.0_f64;
            let mut row_height_max = heights[0];
            let mut running_height = 0.0_f64;

            let mut index: i32 = 1;
            while index < longest_path {
                let idx = index as usize;
                if width_at_index[idx - 1] - sum_so_far >= row_sum {
                    cuts.push(index);
                    width = width.max(width_at_index[idx - 1] - last_cut_width);
                    running_height += row_height_max;
                    sum_so_far += width_at_index[idx - 1] - sum_so_far;
                    last_cut_width = width_at_index[idx - 1];
                    row_height_max = heights[idx];
                }
                row_height_max = row_height_max.max(heights[idx]);
                index += 1;
            }
            running_height += row_height_max;
            height = running_height;
        }

        let scale_by_width = if width > 0.0 { 1.0 / width } else { f64::INFINITY };
        let dar_h = stats.dar * height;
        let scale_by_height = if dar_h > 0.0 { 1.0 / dar_h } else { f64::INFINITY };
        let max_scale = scale_by_width.min(scale_by_height);
        if max_scale > best_max_scale {
            best_max_scale = max_scale;
            best_cuts = cuts;
        }
    }

    best_cuts
}
