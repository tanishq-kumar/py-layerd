//! Hyperedge segment (trunk) data structure for orthogonal edge routing.
//!
//! A segment represents the shared portion (trunk) of a hyperedge between two
//! adjacent layers. In horizontal layouts the trunk is vertical and its
//! coordinates are y values; in vertical layouts the trunk is horizontal and
//! its coordinates are x values. Phase A only builds the data structure and
//! populates it from an `LGraph`; splitting and cycle breaking arrive in
//! Phase B.

use smallvec::SmallVec;

use super::hyper_edge_dependency::DependencyId;
use crate::graph::index::PortId;

/// Newtype id referring to a `HyperEdgeSegment` within a `Vec` arena.
///
/// Provides hashing, equality, and total ordering for segments stored by index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct HyperEdgeId(pub u32);

impl HyperEdgeId {
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Trunk of a hyperedge between two adjacent layers.
#[derive(Debug, Clone)]
pub struct HyperEdgeSegment {
    /// Identifier within the segment arena.
    pub id: HyperEdgeId,

    /// Ports represented by this segment (source side + target side).
    pub ports: SmallVec<PortId, 2>,

    /// Mark value used by the cycle breaker for linear ordering.
    pub mark: i32,

    /// Slot index; determines the distance from the preceding layer once the
    /// routing generator assigns coordinates.
    pub routing_slot: i32,

    /// Top coordinate of the trunk; `NaN` until a connection is added.
    pub start_position: f64,
    /// Bottom coordinate of the trunk; `NaN` until a connection is added.
    pub end_position: f64,

    /// Sorted list of coordinates where incoming connections enter.
    pub incoming_connection_coordinates: SmallVec<f64, 2>,
    /// Sorted list of coordinates where outgoing connections leave.
    pub outgoing_connection_coordinates: SmallVec<f64, 2>,

    /// Indices into the dependency arena for outgoing dependencies.
    pub outgoing_dependencies: SmallVec<DependencyId, 2>,
    /// Indices into the dependency arena for incoming dependencies.
    pub incoming_dependencies: SmallVec<DependencyId, 2>,

    /// Combined weight of all outgoing dependencies.
    pub out_weight: i32,
    /// Combined weight of critical outgoing dependencies.
    pub critical_out_weight: i32,
    /// Combined weight of all incoming dependencies.
    pub in_weight: i32,
    /// Combined weight of critical incoming dependencies.
    pub critical_in_weight: i32,

    /// Split partner (the other segment produced by a split), if any.
    pub split_partner: Option<HyperEdgeId>,
    /// The segment that caused this one to be split; only set on one partner.
    pub split_by: Option<HyperEdgeId>,
}

/// Values whose absolute difference is below this threshold are treated as
/// equal when inserting into a sorted coordinate list.
const COORDINATE_TOLERANCE: f64 = 1.0e-6;

impl HyperEdgeSegment {
    /// Creates an empty segment with the given id.
    pub fn new(id: HyperEdgeId) -> Self {
        Self {
            id,
            ports: SmallVec::new(),
            mark: 0,
            routing_slot: 0,
            start_position: f64::NAN,
            end_position: f64::NAN,
            incoming_connection_coordinates: SmallVec::new(),
            outgoing_connection_coordinates: SmallVec::new(),
            outgoing_dependencies: SmallVec::new(),
            incoming_dependencies: SmallVec::new(),
            out_weight: 0,
            critical_out_weight: 0,
            in_weight: 0,
            critical_in_weight: 0,
            split_partner: None,
            split_by: None,
        }
    }

    /// Adds an incoming connection at the given coordinate and updates extent.
    pub fn add_incoming_connection(&mut self, coord: f64) {
        insert_sorted(&mut self.incoming_connection_coordinates, coord);
        self.recompute_extent();
    }

    /// Adds an outgoing connection at the given coordinate and updates extent.
    pub fn add_outgoing_connection(&mut self, coord: f64) {
        insert_sorted(&mut self.outgoing_connection_coordinates, coord);
        self.recompute_extent();
    }

    /// Recomputes `start_position` and `end_position` from both coordinate lists.
    pub fn recompute_extent(&mut self) {
        self.start_position = f64::NAN;
        self.end_position = f64::NAN;
        fold_extent(
            &self.incoming_connection_coordinates,
            &mut self.start_position,
            &mut self.end_position,
        );
        fold_extent(
            &self.outgoing_connection_coordinates,
            &mut self.start_position,
            &mut self.end_position,
        );
    }

    /// Length of the trunk along its primary axis.
    pub fn length(&self) -> f64 {
        self.end_position - self.start_position
    }

    /// Returns whether the segment connects three or more ports.
    pub fn represents_hyperedge(&self) -> bool {
        self.incoming_connection_coordinates.len() + self.outgoing_connection_coordinates.len() > 2
    }

    /// Returns whether the segment was introduced while splitting another.
    pub fn is_dummy(&self) -> bool {
        self.split_partner.is_some() && self.split_by.is_none()
    }
}

fn fold_extent(positions: &[f64], start: &mut f64, end: &mut f64) {
    if positions.is_empty() {
        return;
    }
    let first = positions[0];
    let last = positions[positions.len() - 1];
    *start = if start.is_nan() { first } else { start.min(first) };
    *end = if end.is_nan() { last } else { end.max(last) };
}

/// Inserts `value` into a sorted coordinate list, skipping near-duplicates.
///
/// If an approximately equal value is already present the insert is dropped.
pub fn insert_sorted(list: &mut SmallVec<f64, 2>, value: f64) {
    let mut idx = 0;
    while idx < list.len() {
        let next = list[idx];
        if (next - value).abs() < COORDINATE_TOLERANCE {
            return;
        }
        if next > value {
            break;
        }
        idx += 1;
    }
    list.insert(idx, value);
}

/// Pair of segments produced by `simulate_split`.
pub struct SimulatedSplit {
    /// Synthetic segment carrying the original incoming connections.
    pub split: HyperEdgeSegment,
    /// Synthetic segment carrying the original outgoing connections.
    pub partner: HyperEdgeSegment,
}

/// Simulates the result of splitting a segment without mutating the arena.
///
/// Returns two fresh segments (without ids assigned yet) that carry the
/// incoming and outgoing connections of the original segment. Used by the
/// splitter to rate candidate split positions before committing.
pub fn simulate_split(segment: &HyperEdgeSegment) -> SimulatedSplit {
    let placeholder = HyperEdgeId(u32::MAX);
    let mut split = HyperEdgeSegment::new(placeholder);
    let mut partner = HyperEdgeSegment::new(placeholder);

    split.incoming_connection_coordinates = segment.incoming_connection_coordinates.clone();
    split.split_by = segment.split_by;
    split.recompute_extent();

    partner.outgoing_connection_coordinates = segment.outgoing_connection_coordinates.clone();
    partner.recompute_extent();

    split.split_partner = Some(partner.id);
    partner.split_partner = Some(split.id);

    SimulatedSplit { split, partner }
}

/// Splits `segments[seg_idx]` at `split_position` and appends the new partner.
///
/// Retains incoming coordinates on the original, moves outgoing coordinates
/// to the partner, and records a link position on each side so the
/// connection through the free area is drawn correctly. Clears all
/// dependencies on the original segment since they must be rebuilt around
/// the new topology; the caller is responsible for updating the dependency
/// arena.
pub fn split_segment_at(
    segments: &mut Vec<HyperEdgeSegment>,
    seg_idx: usize,
    split_position: f64,
) -> HyperEdgeId {
    let partner_id = HyperEdgeId(segments.len() as u32);
    let original_id = segments[seg_idx].id;

    let mut partner = HyperEdgeSegment::new(partner_id);
    partner.split_partner = Some(original_id);
    partner.outgoing_connection_coordinates =
        std::mem::take(&mut segments[seg_idx].outgoing_connection_coordinates);

    segments[seg_idx].split_partner = Some(partner_id);

    // Link position on both sides.
    insert_sorted(&mut segments[seg_idx].outgoing_connection_coordinates, split_position);
    insert_sorted(&mut partner.incoming_connection_coordinates, split_position);

    segments[seg_idx].recompute_extent();
    partner.recompute_extent();

    // The original's dependencies are stale after the split; the splitter
    // will rebuild them, so clear both sides now. Incoming dependencies are
    // referenced by other segments' outgoing lists, so we only clear the
    // local lists here — the dependency arena cleanup happens in the splitter.
    segments[seg_idx].outgoing_dependencies.clear();
    segments[seg_idx].incoming_dependencies.clear();

    segments.push(partner);
    partner_id
}

/// Centre of a segment's range along the routing axis.
#[inline]
pub fn segment_center(segment: &HyperEdgeSegment) -> f64 {
    (segment.start_position + segment.end_position) / 2.0
}

impl Ord for HyperEdgeSegment {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.mark.cmp(&other.mark)
    }
}

impl PartialOrd for HyperEdgeSegment {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for HyperEdgeSegment {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for HyperEdgeSegment {}

impl std::hash::Hash for HyperEdgeSegment {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}
