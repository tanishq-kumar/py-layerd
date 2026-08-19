//! Dependency between two hyperedge segments.
//!
//! A dependency expresses that its source segment wants to be placed in a
//! lower routing slot than its target segment. Ignoring a `Regular`
//! dependency increases crossings; ignoring a `Critical` one produces edge
//! overlaps.

use super::hyper_edge_segment::{HyperEdgeId, HyperEdgeSegment};

/// Non-zero weight reserved for critical dependencies.
pub const CRITICAL_DEPENDENCY_WEIGHT: i32 = 1;

/// Kind of dependency between two hyperedge segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DependencyType {
    /// Ignoring this dependency causes additional crossings.
    Regular,
    /// Ignoring this dependency causes edge overlaps.
    Critical,
}

/// Newtype id referring to a dependency within a `Vec` arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DependencyId(pub u32);

impl DependencyId {
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Directed dependency between two hyperedge segments.
#[derive(Debug, Clone, Copy)]
pub struct HyperEdgeSegmentDependency {
    /// Kind of dependency.
    pub dep_type: DependencyType,
    /// Source segment (wants to be in lower routing slot than target).
    ///
    /// `None` once the dependency has been removed.
    pub source: Option<HyperEdgeId>,
    /// Target segment.
    ///
    /// `None` once the dependency has been removed.
    pub target: Option<HyperEdgeId>,
    /// Weight of the dependency.
    pub weight: i32,
}

impl HyperEdgeSegmentDependency {
    /// Creates a new regular dependency and registers it on both segments.
    pub fn create_and_add_regular(
        segments: &mut [HyperEdgeSegment],
        deps: &mut Vec<HyperEdgeSegmentDependency>,
        source: HyperEdgeId,
        target: HyperEdgeId,
        weight: i32,
    ) -> DependencyId {
        create_and_add(segments, deps, DependencyType::Regular, source, target, weight)
    }

    /// Creates a new critical dependency with `CRITICAL_DEPENDENCY_WEIGHT`.
    pub fn create_and_add_critical(
        segments: &mut [HyperEdgeSegment],
        deps: &mut Vec<HyperEdgeSegmentDependency>,
        source: HyperEdgeId,
        target: HyperEdgeId,
    ) -> DependencyId {
        create_and_add(
            segments,
            deps,
            DependencyType::Critical,
            source,
            target,
            CRITICAL_DEPENDENCY_WEIGHT,
        )
    }

    /// Reverses the dependency: swap source and target and re-register on the
    /// segments. The incident lists on the previous endpoints are updated so
    /// old entries do not linger.
    pub fn reverse(
        dep_id: DependencyId,
        deps: &mut [HyperEdgeSegmentDependency],
        segments: &mut [HyperEdgeSegment],
    ) {
        let (old_source, old_target) = {
            let d = &deps[dep_id.index()];
            (d.source, d.target)
        };
        set_source(dep_id, deps, segments, old_target);
        set_target(dep_id, deps, segments, old_source);
    }

    /// Unregisters the dependency from both endpoints.
    pub fn remove(
        dep_id: DependencyId,
        deps: &mut [HyperEdgeSegmentDependency],
        segments: &mut [HyperEdgeSegment],
    ) {
        set_source(dep_id, deps, segments, None);
        set_target(dep_id, deps, segments, None);
    }
}

fn create_and_add(
    segments: &mut [HyperEdgeSegment],
    deps: &mut Vec<HyperEdgeSegmentDependency>,
    dep_type: DependencyType,
    source: HyperEdgeId,
    target: HyperEdgeId,
    weight: i32,
) -> DependencyId {
    let id = DependencyId(deps.len() as u32);
    deps.push(HyperEdgeSegmentDependency { dep_type, source: None, target: None, weight });
    set_source(id, deps, segments, Some(source));
    set_target(id, deps, segments, Some(target));
    id
}

fn set_source(
    dep_id: DependencyId,
    deps: &mut [HyperEdgeSegmentDependency],
    segments: &mut [HyperEdgeSegment],
    new_source: Option<HyperEdgeId>,
) {
    let old_source = deps[dep_id.index()].source;
    if let Some(old) = old_source {
        segments[old.index()].outgoing_dependencies.retain(|&d| d != dep_id);
    }
    deps[dep_id.index()].source = new_source;
    if let Some(new) = new_source {
        segments[new.index()].outgoing_dependencies.push(dep_id);
    }
}

fn set_target(
    dep_id: DependencyId,
    deps: &mut [HyperEdgeSegmentDependency],
    segments: &mut [HyperEdgeSegment],
    new_target: Option<HyperEdgeId>,
) {
    let old_target = deps[dep_id.index()].target;
    if let Some(old) = old_target {
        segments[old.index()].incoming_dependencies.retain(|&d| d != dep_id);
    }
    deps[dep_id.index()].target = new_target;
    if let Some(new) = new_target {
        segments[new.index()].incoming_dependencies.push(dep_id);
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<HyperEdgeSegmentDependency>();
    }
}
