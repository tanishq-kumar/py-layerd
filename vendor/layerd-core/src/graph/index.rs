use super::arena::ArenaId;

macro_rules! define_id {
    ($name:ident) => {
        #[derive(Clone, Copy, PartialEq, Eq, Hash)]
        pub struct $name(pub ArenaId);

        impl $name {
            pub fn arena_id(self) -> ArenaId {
                self.0
            }
        }

        impl std::fmt::Debug for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(
                    f,
                    "{}(g{}/{}g{})",
                    stringify!($name),
                    self.0.graph_id(),
                    self.0.index(),
                    self.0.generation()
                )
            }
        }
    };
}

define_id!(NodeId);
define_id!(PortId);
define_id!(EdgeId);
define_id!(LabelId);

/// Identifier for a [`HierarchicalEdgeData`] slot stored on the root `LGraph`.
///
/// Hierarchical edges live in a parallel `Vec<HierarchicalEdgeData>` on the
/// root graph, distinct from the local edge arena. The compound preprocessor
/// drains this list at Pre-P1 time and materialises each entry as one or more
/// local dummy edges plus external-port dummies; downstream phases never
/// observe `HierarchicalEdgeId` values directly.
///
/// [`HierarchicalEdgeData`]: super::hierarchical_edge::HierarchicalEdgeData
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct HierarchicalEdgeId(pub u32);
