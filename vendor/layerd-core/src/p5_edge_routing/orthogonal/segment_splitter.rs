//! Segment splitter for orthogonal hyperedge routing.
//!
//! When the cycle detector reports feedback critical dependencies (those that
//! cannot simply be reversed because reversal would cause edge overlaps), the
//! splitter picks one segment per cycle and splits it at a free area. The
//! new segment plus a linking connection resolve the critical cycle while
//! only introducing a small detour for the routed edge.

use super::{
    hyper_edge_dependency::{DependencyId, HyperEdgeSegmentDependency},
    hyper_edge_segment::{
        HyperEdgeId, HyperEdgeSegment, SimulatedSplit, segment_center, simulate_split,
        split_segment_at,
    },
    routing_generator::{count_crossings, create_dependency_if_necessary},
};

/// Free area between two existing horizontal connection coordinates.
///
/// Large enough to host a new linking segment without introducing additional
/// conflicts.
#[derive(Debug, Clone, Copy)]
struct FreeArea {
    start: f64,
    end: f64,
}

impl FreeArea {
    fn size(&self) -> f64 {
        self.end - self.start
    }

    fn center(&self) -> f64 {
        (self.start + self.end) / 2.0
    }
}

/// Breaks the given feedback dependencies by splitting one segment per cycle.
///
/// Returns the number of segments that were actually split. The input
/// dependency ids are the feedback arcs reported by
/// `detect_cycles_and_break`; the caller guarantees they are critical.
///
/// Candidate split positions are scored with a three-tier rating —
/// crossings first, dependencies second, free-area size last.
pub fn split_segments(
    segments: &mut Vec<HyperEdgeSegment>,
    deps: &mut Vec<HyperEdgeSegmentDependency>,
    feedback: &[DependencyId],
    conflict_threshold: f64,
    critical_conflict_threshold: f64,
) -> usize {
    if feedback.is_empty() {
        return 0;
    }

    let mut free_areas = find_free_areas(segments, critical_conflict_threshold);
    let segments_to_split = decide_which_segments_to_split(segments, deps, feedback);

    // Sort by ascending segment length — the shorter, the fewer placement
    // options, so split them first.
    let mut ordered: Vec<HyperEdgeId> = segments_to_split;
    ordered.sort_by(|&a, &b| {
        segments[a.index()]
            .length()
            .partial_cmp(&segments[b.index()].length())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut split_count = 0usize;
    for seg_id in ordered {
        let split_position = choose_split_position(
            segments,
            deps,
            seg_id,
            &mut free_areas,
            critical_conflict_threshold,
        );
        clear_dependencies_for_segment(segments, deps, seg_id);
        let partner_id = split_segment_at(segments, seg_id.index(), split_position);
        rebuild_dependencies_after_split(
            segments,
            deps,
            seg_id,
            partner_id,
            conflict_threshold,
            critical_conflict_threshold,
        );
        split_count += 1;
    }
    split_count
}

fn clear_dependencies_for_segment(
    segments: &mut [HyperEdgeSegment],
    deps: &mut [HyperEdgeSegmentDependency],
    segment: HyperEdgeId,
) {
    while let Some(dep_id) = segments[segment.index()].incoming_dependencies.first().copied() {
        HyperEdgeSegmentDependency::remove(dep_id, deps, segments);
    }
    while let Some(dep_id) = segments[segment.index()].outgoing_dependencies.first().copied() {
        HyperEdgeSegmentDependency::remove(dep_id, deps, segments);
    }
}

/// Cost of using a given free area to split a segment.
#[derive(Debug, Clone, Copy)]
struct AreaRating {
    crossings: i32,
    dependencies: i32,
}

/// Collects every free area between two connection coordinates that is at
/// least `2 * threshold` wide.
fn find_free_areas(segments: &[HyperEdgeSegment], threshold: f64) -> Vec<FreeArea> {
    let mut coords: Vec<f64> = Vec::new();
    for seg in segments {
        coords.extend_from_slice(&seg.incoming_connection_coordinates);
        coords.extend_from_slice(&seg.outgoing_connection_coordinates);
    }
    coords.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let mut areas: Vec<FreeArea> = Vec::new();
    for w in coords.windows(2) {
        let gap = w[1] - w[0];
        if gap >= 2.0 * threshold {
            areas.push(FreeArea { start: w[0] + threshold, end: w[1] - threshold });
        }
    }
    areas
}

/// For each feedback dependency, picks one of the two incident segments to be
/// split and marks it with `split_by` pointing to the other one.
fn decide_which_segments_to_split(
    segments: &mut [HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    feedback: &[DependencyId],
) -> Vec<HyperEdgeId> {
    let mut selected: Vec<HyperEdgeId> = Vec::new();
    for &dep_id in feedback {
        let dep = &deps[dep_id.index()];
        let Some(source) = dep.source else { continue };
        let Some(target) = dep.target else { continue };

        if selected.contains(&source) || selected.contains(&target) {
            continue;
        }

        // Default: split the source and let the target remain between the
        // two halves. Reverse when the source represents a hyperedge but the
        // target does not — splitting a hyperedge segment tends to introduce
        // more crossings than splitting a regular edge.
        let source_is_hyper = segments[source.index()].represents_hyperedge();
        let target_is_hyper = segments[target.index()].represents_hyperedge();
        let (to_split, causing) = if source_is_hyper && !target_is_hyper {
            (target, source)
        } else {
            (source, target)
        };

        segments[to_split.index()].split_by = Some(causing);
        selected.push(to_split);
    }
    selected
}

/// Chooses the split position for a segment.
///
/// Scan the free-area list for the index range overlapping the segment's
/// vertical extent, delegate to `choose_best_area_index` for the actual
/// three-tier rating, consume that area, and return its centre. Falls back
/// to the segment's own centre when no area overlaps.
fn choose_split_position(
    segments: &[HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    seg_id: HyperEdgeId,
    free_areas: &mut Vec<FreeArea>,
    threshold: f64,
) -> f64 {
    let segment = &segments[seg_id.index()];
    let start = segment.start_position;
    let end = segment.end_position;

    // Collect `[first, last]` index range of free areas that overlap the
    // segment's vertical extent.
    let mut first: Option<usize> = None;
    let mut last: Option<usize> = None;
    for (i, area) in free_areas.iter().enumerate() {
        if area.end < start {
            continue;
        }
        if area.start > end {
            break;
        }
        if first.is_none() {
            first = Some(i);
        }
        last = Some(i);
    }

    match (first, last) {
        (Some(f), Some(l)) => {
            let best_idx = choose_best_area_index(segments, deps, seg_id, free_areas, f, l);
            let position = free_areas[best_idx].center();
            consume_area(free_areas, best_idx, threshold);
            position
        }
        _ => segment_center(segment),
    }
}

/// Picks the best free area inside `[from, to]` (both inclusive) using a
/// three-tier ordering: crossings → dependencies → size.
fn choose_best_area_index(
    segments: &[HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    seg_id: HyperEdgeId,
    free_areas: &[FreeArea],
    from: usize,
    to: usize,
) -> usize {
    if from >= to {
        return from;
    }
    // Simulate the split once and reuse the two synthetic segments for every
    // rate_area call.
    let simulated = simulate_split(&segments[seg_id.index()]);

    let mut best_idx = from;
    let mut best_area = free_areas[from];
    let mut best_rating = rate_area(segments, deps, seg_id, &simulated, &best_area);

    for (i, &curr_area) in free_areas.iter().enumerate().take(to + 1).skip(from + 1) {
        let curr_rating = rate_area(segments, deps, seg_id, &simulated, &curr_area);
        if is_better(curr_area, curr_rating, best_area, best_rating) {
            best_area = curr_area;
            best_rating = curr_rating;
            best_idx = i;
        }
    }
    best_idx
}

/// Rates the outcome of linking the two split halves via `area`.
///
/// Clones the simulated halves, populates their outgoing/incoming coordinate
/// lists with the area's centre, then counts crossings and dependencies
/// against every neighbour of the original segment plus the splitBy segment
/// itself.
fn rate_area(
    segments: &[HyperEdgeSegment],
    deps: &[HyperEdgeSegmentDependency],
    seg_id: HyperEdgeId,
    simulated: &SimulatedSplit,
    area: &FreeArea,
) -> AreaRating {
    let area_centre = area.center();

    // Local mutable copies of the simulated halves with the link coordinate
    // inserted. The immutable borrow of `segments` means we can't reuse the
    // simulated segments across calls, so we clone here.
    let mut split = simulated.split.clone();
    let mut partner = simulated.partner.clone();
    split.outgoing_connection_coordinates.clear();
    split.outgoing_connection_coordinates.push(area_centre);
    split.recompute_extent();
    partner.incoming_connection_coordinates.clear();
    partner.incoming_connection_coordinates.push(area_centre);
    partner.recompute_extent();

    let mut rating = AreaRating { crossings: 0, dependencies: 0 };

    let segment = &segments[seg_id.index()];

    // Every neighbour currently joined to `segment` becomes a neighbour of
    // both halves after the split; tally crossings/dependencies for each.
    for &dep_id in &segment.incoming_dependencies {
        let Some(source) = deps[dep_id.index()].source else { continue };
        let other = &segments[source.index()];
        update_considering_both_orderings(&mut rating, &split, other);
        update_considering_both_orderings(&mut rating, &partner, other);
    }
    for &dep_id in &segment.outgoing_dependencies {
        let Some(target) = deps[dep_id.index()].target else { continue };
        let other = &segments[target.index()];
        update_considering_both_orderings(&mut rating, &split, other);
        update_considering_both_orderings(&mut rating, &partner, other);
    }

    // Two more critical dependencies are introduced —
    // `split → splitBy → partner` — with a fixed ordering, so count
    // crossings for that single order.
    rating.dependencies += 2;
    if let Some(split_by_id) = segment.split_by {
        let split_by = &segments[split_by_id.index()];
        rating.crossings += count_crossings_for_single_ordering(&split, split_by);
        rating.crossings += count_crossings_for_single_ordering(split_by, &partner);
    }

    rating
}

/// Counts the crossings for both orderings (`s1` left of `s2` and vice
/// versa) and charges the rating with the cheaper side plus the number of
/// dependencies that ordering would force.
fn update_considering_both_orderings(
    rating: &mut AreaRating,
    s1: &HyperEdgeSegment,
    s2: &HyperEdgeSegment,
) {
    let c_s1_left = count_crossings_for_single_ordering(s1, s2);
    let c_s2_left = count_crossings_for_single_ordering(s2, s1);
    if c_s1_left == c_s2_left {
        if c_s1_left > 0 {
            // Two-cycle: both orders cost the same positive number of
            // crossings, so there will be a pair of dependencies.
            rating.dependencies += 2;
            rating.crossings += c_s1_left;
        }
    } else {
        rating.dependencies += 1;
        rating.crossings += c_s1_left.min(c_s2_left);
    }
}

/// Counts crossings between two segments under the assumption that `left`
/// is placed in a lower routing slot than `right`.
fn count_crossings_for_single_ordering(left: &HyperEdgeSegment, right: &HyperEdgeSegment) -> i32 {
    let a = count_crossings(
        &left.outgoing_connection_coordinates,
        right.start_position,
        right.end_position,
    );
    let b = count_crossings(
        &right.incoming_connection_coordinates,
        left.start_position,
        left.end_position,
    );
    a + b
}

/// Three-tier ranking used by `choose_best_area_index`.
fn is_better(
    curr_area: FreeArea,
    curr_rating: AreaRating,
    best_area: FreeArea,
    best_rating: AreaRating,
) -> bool {
    if curr_rating.crossings < best_rating.crossings {
        return true;
    }
    if curr_rating.crossings == best_rating.crossings {
        if curr_rating.dependencies < best_rating.dependencies {
            return true;
        }
        if curr_rating.dependencies == best_rating.dependencies
            && curr_area.size() > best_area.size()
        {
            return true;
        }
    }
    false
}

/// Replaces the chosen area with up to two smaller areas that flank the newly
/// used centre point.
fn consume_area(free_areas: &mut Vec<FreeArea>, used_idx: usize, threshold: f64) {
    let old = free_areas.remove(used_idx);
    if old.size() / 2.0 < threshold {
        return;
    }
    let centre = old.center();
    let mut insert_at = used_idx;
    let left_end = centre - threshold;
    if old.start <= left_end {
        free_areas.insert(insert_at, FreeArea { start: old.start, end: left_end });
        insert_at += 1;
    }
    let right_start = centre + threshold;
    if right_start <= old.end {
        free_areas.insert(insert_at, FreeArea { start: right_start, end: old.end });
    }
}

/// Installs the critical dependencies that wire the three segments after a
/// split: `segment → split_causing_segment → partner`. Also re-issues regular
/// dependencies between the two halves and every other segment whose vertical
/// range still overlaps.
///
/// The regular re-registration uses the overlap-based recipe shared with
/// the router's dependency builder.
fn rebuild_dependencies_after_split(
    segments: &mut [HyperEdgeSegment],
    deps: &mut Vec<HyperEdgeSegmentDependency>,
    segment: HyperEdgeId,
    partner: HyperEdgeId,
    conflict_threshold: f64,
    critical_conflict_threshold: f64,
) {
    let Some(split_causing) = segments[segment.index()].split_by else {
        return;
    };

    HyperEdgeSegmentDependency::create_and_add_critical(segments, deps, segment, split_causing);
    HyperEdgeSegmentDependency::create_and_add_critical(segments, deps, split_causing, partner);

    let n = segments.len();
    for other_idx in 0..n {
        let other = HyperEdgeId(other_idx as u32);
        if other == segment || other == partner || other == split_causing {
            continue;
        }
        create_dependency_if_necessary(
            segments,
            deps,
            other,
            segment,
            conflict_threshold,
            critical_conflict_threshold,
        );
        create_dependency_if_necessary(
            segments,
            deps,
            other,
            partner,
            conflict_threshold,
            critical_conflict_threshold,
        );
    }
}
