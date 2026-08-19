//! Model-order aware barycenter heuristic for crossing minimization.
//!
//! Uses insertion sort (not quicksort) to preserve partial ordering from model
//! order while allowing dummy nodes to be placed by barycenter values. Tracks
//! transitive ordering dependencies to ensure consistency.

use std::cmp::Ordering;

use hashbrown::{HashMap, HashSet};

use crate::{
    graph::{LGraph, index::NodeId},
    options::enums::LayerConstraint,
    p3_crossing_min::{
        barycenter_heuristic::compare_barycenters, barycenter_state::BarycenterStateMap,
    },
    properties::internal::{LAYER_CONSTRAINT, MODEL_ORDER},
};

/// Transitive ordering state for model-order-aware sorting.
///
/// Real struct with state — not a ZST. The `bigger_than`/`smaller_than` maps
/// track transitive ordering decisions made during insertion sort so that later
/// comparisons respect earlier ones.
pub struct ModelOrderSorter {
    bigger_than: HashMap<NodeId, HashSet<NodeId>>,
    smaller_than: HashMap<NodeId, HashSet<NodeId>>,
}

impl ModelOrderSorter {
    pub fn new() -> Self {
        Self { bigger_than: HashMap::new(), smaller_than: HashMap::new() }
    }

    /// Sort a layer using insertion sort with model-order-aware comparison.
    ///
    /// Insertion sort is used instead of quicksort because the partial ordering
    /// defined by model order combined with barycenter values is prone to
    /// creating conflicting orderings. Insertion sort ensures no real node is
    /// moved past another real node if the model order forbids it.
    pub fn insertion_sort(
        &mut self,
        nodes: &mut [NodeId],
        graph: &LGraph,
        states: &BarycenterStateMap,
    ) {
        for i in 1..nodes.len() {
            let temp = nodes[i];
            let mut j = i;
            while j > 0 && self.compare(nodes[j - 1], temp, graph, states) == Ordering::Greater {
                nodes[j] = nodes[j - 1];
                j -= 1;
            }
            nodes[j] = temp;
        }
        self.clear_transitive_ordering();
    }

    fn compare(
        &mut self,
        n1: NodeId,
        n2: NodeId,
        graph: &LGraph,
        states: &BarycenterStateMap,
    ) -> Ordering {
        let n1_constraint = graph.node(n1).properties.get(&LAYER_CONSTRAINT);
        let n2_constraint = graph.node(n2).properties.get(&LAYER_CONSTRAINT);
        if matches!(n1_constraint, LayerConstraint::FirstSeparate | LayerConstraint::LastSeparate)
            || matches!(
                n2_constraint,
                LayerConstraint::FirstSeparate | LayerConstraint::LastSeparate
            )
        {
            return Ordering::Equal;
        }

        let transitive = self.compare_based_on_transitive_dependencies(n1, n2);
        if transitive != Ordering::Equal {
            return transitive;
        }

        let n1_has_mo = graph.node(n1).properties.has(&MODEL_ORDER);
        let n2_has_mo = graph.node(n2).properties.has(&MODEL_ORDER);
        if n1_has_mo && n2_has_mo {
            let n1_mo = graph.node(n1).properties.get(&MODEL_ORDER);
            let n2_mo = graph.node(n2).properties.get(&MODEL_ORDER);
            let value = n1_mo.cmp(&n2_mo);
            if value == Ordering::Less {
                self.update_bigger_and_smaller_associations(n1, n2);
                return value;
            } else if value == Ordering::Greater {
                self.update_bigger_and_smaller_associations(n2, n1);
                return value;
            }
        }

        self.compare_based_on_barycenter(n1, n2, states)
    }

    fn compare_based_on_transitive_dependencies(&mut self, n1: NodeId, n2: NodeId) -> Ordering {
        if !self.bigger_than.contains_key(&n1) {
            self.bigger_than.insert(n1, HashSet::new());
        } else if self.bigger_than[&n1].contains(&n2) {
            return Ordering::Greater;
        }
        if !self.bigger_than.contains_key(&n2) {
            self.bigger_than.insert(n2, HashSet::new());
        } else if self.bigger_than[&n2].contains(&n1) {
            return Ordering::Less;
        }
        if !self.smaller_than.contains_key(&n1) {
            self.smaller_than.insert(n1, HashSet::new());
        } else if self.smaller_than[&n1].contains(&n2) {
            return Ordering::Less;
        }
        if !self.smaller_than.contains_key(&n2) {
            self.smaller_than.insert(n2, HashSet::new());
        } else if self.smaller_than[&n2].contains(&n1) {
            return Ordering::Greater;
        }
        Ordering::Equal
    }

    fn compare_based_on_barycenter(
        &mut self,
        n1: NodeId,
        n2: NodeId,
        states: &BarycenterStateMap,
    ) -> Ordering {
        let result = compare_barycenters(states.get(n1), states.get(n2));
        match result {
            Ordering::Less => self.update_bigger_and_smaller_associations(n1, n2),
            Ordering::Greater => self.update_bigger_and_smaller_associations(n2, n1),
            Ordering::Equal => {}
        }
        result
    }

    fn update_bigger_and_smaller_associations(&mut self, bigger: NodeId, smaller: NodeId) {
        self.bigger_than.entry(bigger).or_default().insert(smaller);
        self.smaller_than.entry(smaller).or_default().insert(bigger);

        let smaller_bigger_than: Vec<NodeId> = self
            .bigger_than
            .get(&smaller)
            .cloned()
            .unwrap_or_default()
            .into_iter()
            .collect();
        let bigger_smaller_than: Vec<NodeId> = self
            .smaller_than
            .get(&bigger)
            .cloned()
            .unwrap_or_default()
            .into_iter()
            .collect();

        for very_small in &smaller_bigger_than {
            self.bigger_than.entry(bigger).or_default().insert(*very_small);
            self.smaller_than.entry(*very_small).or_default().insert(bigger);
            for very_big in &bigger_smaller_than {
                self.smaller_than.entry(*very_small).or_default().insert(*very_big);
            }
        }

        for very_big in &bigger_smaller_than {
            self.smaller_than.entry(smaller).or_default().insert(*very_big);
            self.bigger_than.entry(*very_big).or_default().insert(smaller);
            for very_small in &smaller_bigger_than {
                self.bigger_than.entry(*very_big).or_default().insert(*very_small);
            }
        }
    }

    fn clear_transitive_ordering(&mut self) {
        self.bigger_than.clear();
        self.smaller_than.clear();
    }
}
