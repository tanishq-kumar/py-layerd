use std::{
    hash::{Hash, Hasher},
    mem::MaybeUninit,
    num::NonZeroU64,
};

/// A generational, graph-scoped index into an `Arena`.
///
/// Packs three fields into a single `u64`:
///
/// ```text
///   bits  | 63..48 | 47..16 | 15..0  |
///   field | graph  | index  | gen    |
/// ```
///
/// - **graph** (16 bits) — `LGraph::graph_id`, identifying the LGraph instance
///   that owns this id. Two different LGraphs always produce disjoint
///   `ArenaId` values, eliminating the cross-arena collisions that plagued
///   the per-graph-only encoding.
/// - **index** (32 bits) — slot index inside the owning arena.
/// - **gen** (16 bits) — generation counter for slot reuse safety.
///
/// `Arena::get` checks that `id.graph_id() == self.arena_tag` so a NodeId
/// from one LGraph never silently resolves into a different LGraph's slot.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ArenaId(NonZeroU64);

impl ArenaId {
    fn new(graph_id: u16, index: u32, generation: u16) -> Self {
        let bits = ((graph_id as u64) << 48)
            | ((index as u64 & 0xFFFF_FFFF) << 16)
            | (generation as u64 & 0xFFFF);
        ArenaId(NonZeroU64::new(bits + 1).expect("encoded arena id must be non-zero"))
    }

    fn bits(self) -> u64 {
        self.0.get() - 1
    }

    pub fn graph_id(self) -> u16 {
        (self.bits() >> 48) as u16
    }

    pub fn index(self) -> u32 {
        ((self.bits() >> 16) & 0xFFFF_FFFF) as u32
    }

    pub fn generation(self) -> u16 {
        (self.bits() & 0xFFFF) as u16
    }

    /// A sentinel ArenaId that does not correspond to any valid arena entry.
    ///
    /// Used for graph-level ports that have no owning node. `Arena::get` on
    /// the sentinel always returns `None` (graph_id mismatch and gen mismatch
    /// both fail the check independently).
    pub fn sentinel() -> Self {
        ArenaId(NonZeroU64::MAX)
    }
}

impl std::fmt::Debug for ArenaId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ArenaId(g{}/{}g{})", self.graph_id(), self.index(), self.generation())
    }
}

impl Hash for ArenaId {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.bits().hash(state);
    }
}

struct Entry<T> {
    generation: u16,
    occupied: bool,
    value: MaybeUninit<T>,
}

/// A generational arena that provides stable indices even after removals.
///
/// Each `Arena` is tagged with the `graph_id` of the owning `LGraph`. All
/// `ArenaId`s produced by `insert` carry that tag; `get`/`get_mut`/`remove`
/// reject any `ArenaId` whose tag does not match. This catches cross-LGraph
/// id leakage at the lookup layer and prevents the silent "wrong slot"
/// reads that previously corrupted compound graph state.
pub struct Arena<T> {
    arena_tag: u16,
    entries: Vec<Entry<T>>,
    free_list: Vec<u32>,
    len: usize,
}

impl<T> Arena<T> {
    pub fn new(arena_tag: u16) -> Self {
        Arena { arena_tag, entries: Vec::new(), free_list: Vec::new(), len: 0 }
    }

    pub fn with_capacity(arena_tag: u16, capacity: usize) -> Self {
        Arena { arena_tag, entries: Vec::with_capacity(capacity), free_list: Vec::new(), len: 0 }
    }

    pub fn reserve(&mut self, additional: usize) {
        self.entries.reserve(additional.saturating_sub(self.free_list.len()));
    }

    /// The graph-id tag this arena was constructed with. Every `ArenaId`
    /// returned by `insert` and accepted by `get`/`get_mut`/`remove` carries
    /// this same tag in its high 16 bits.
    pub fn arena_tag(&self) -> u16 {
        self.arena_tag
    }

    pub fn insert(&mut self, value: T) -> ArenaId {
        self.len += 1;

        if let Some(index) = self.free_list.pop() {
            let entry = &mut self.entries[index as usize];
            entry.generation = entry.generation.wrapping_add(1);
            entry.occupied = true;
            entry.value = MaybeUninit::new(value);
            ArenaId::new(self.arena_tag, index, entry.generation)
        } else {
            let index = self.entries.len() as u32;
            let generation: u16 = 0;
            self.entries
                .push(Entry { generation, occupied: true, value: MaybeUninit::new(value) });
            ArenaId::new(self.arena_tag, index, generation)
        }
    }

    pub fn remove(&mut self, id: ArenaId) -> Option<T> {
        if id.graph_id() != self.arena_tag {
            return None;
        }
        let index = id.index() as usize;
        if index >= self.entries.len() {
            return None;
        }
        let entry = &mut self.entries[index];
        if !entry.occupied || entry.generation != id.generation() {
            return None;
        }
        entry.occupied = false;
        self.len -= 1;
        self.free_list.push(id.index());
        // SAFETY: We just verified the entry is occupied and the value is initialized.
        let value = unsafe { entry.value.assume_init_read() };
        Some(value)
    }

    pub fn get(&self, id: ArenaId) -> Option<&T> {
        if id.graph_id() != self.arena_tag {
            return None;
        }
        let index = id.index() as usize;
        let entry = self.entries.get(index)?;
        if !entry.occupied || entry.generation != id.generation() {
            return None;
        }
        // SAFETY: We verified the entry is occupied, so the value is initialized.
        Some(unsafe { entry.value.assume_init_ref() })
    }

    pub fn get_mut(&mut self, id: ArenaId) -> Option<&mut T> {
        if id.graph_id() != self.arena_tag {
            return None;
        }
        let index = id.index() as usize;
        let entry = self.entries.get_mut(index)?;
        if !entry.occupied || entry.generation != id.generation() {
            return None;
        }
        // SAFETY: We verified the entry is occupied, so the value is initialized.
        Some(unsafe { entry.value.assume_init_mut() })
    }

    pub fn iter(&self) -> impl Iterator<Item = (ArenaId, &T)> {
        let arena_tag = self.arena_tag;
        self.entries.iter().enumerate().filter(|(_, e)| e.occupied).map(move |(i, e)| {
            let id = ArenaId::new(arena_tag, i as u32, e.generation);
            // SAFETY: We verified the entry is occupied.
            let value = unsafe { e.value.assume_init_ref() };
            (id, value)
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ArenaId, &mut T)> {
        let arena_tag = self.arena_tag;
        self.entries
            .iter_mut()
            .enumerate()
            .filter(|(_, e)| e.occupied)
            .map(move |(i, e)| {
                let id = ArenaId::new(arena_tag, i as u32, e.generation);
                // SAFETY: We verified the entry is occupied.
                let value = unsafe { e.value.assume_init_mut() };
                (id, value)
            })
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<T> Drop for Arena<T> {
    fn drop(&mut self) {
        for entry in &mut self.entries {
            if entry.occupied {
                // SAFETY: We verified the entry is occupied, so the value is initialized.
                unsafe {
                    entry.value.assume_init_drop();
                }
                entry.occupied = false;
            }
        }
    }
}
