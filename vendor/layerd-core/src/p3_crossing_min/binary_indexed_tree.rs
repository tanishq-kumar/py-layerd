//! Binary indexed tree for crossing-count accumulation.
//!
//! Sorted list of integers storing values from 0 up to the maxNumber passed
//! on creation. Adding, removing and rank (prefix sum) operations run in
//! O(log maxNumber). Implemented as a Fenwick tree where each leaf stores
//! the number of integers at the leaf index and each internal node stores
//! the number of values in its left branch.

pub struct BinaryIndexedTree {
    binary_sums: Vec<i32>,
    nums_per_index: Vec<i32>,
    size: usize,
    max_num: usize,
}

impl BinaryIndexedTree {
    /// Construct tree given maximum number of elements.
    pub fn new(max_num: usize) -> Self {
        BinaryIndexedTree {
            binary_sums: vec![0; max_num + 1],
            nums_per_index: vec![0; max_num],
            size: 0,
            max_num,
        }
    }

    /// Construct an empty tree with no allocations. Pair with `reset` before
    /// the first use. Used by `CountingScratch` so the tree can be pooled
    /// across tens of thousands of crossing-counter calls per layout sweep.
    pub fn empty() -> Self {
        BinaryIndexedTree {
            binary_sums: Vec::new(),
            nums_per_index: Vec::new(),
            size: 0,
            max_num: 0,
        }
    }

    /// Resize and zero the buffers in place so the tree can be reused for a
    /// new `max_num`. Keeps existing capacity when it already covers the
    /// requested size, avoiding the per-call `Vec::resize` that the previous
    /// `clear` path forced.
    pub fn reset(&mut self, max_num: usize) {
        let sums_len = max_num + 1;
        if self.binary_sums.len() < sums_len {
            self.binary_sums.resize(sums_len, 0);
        } else {
            for slot in &mut self.binary_sums[..sums_len] {
                *slot = 0;
            }
        }
        if self.nums_per_index.len() < max_num {
            self.nums_per_index.resize(max_num, 0);
        } else {
            for slot in &mut self.nums_per_index[..max_num] {
                *slot = 0;
            }
        }
        self.size = 0;
        self.max_num = max_num;
    }

    /// Increment given index.
    pub fn add(&mut self, index: usize) {
        self.size += 1;
        self.nums_per_index[index] += 1;
        let mut i = index + 1;
        while i < self.binary_sums.len() {
            self.binary_sums[i] += 1;
            i += i & i.wrapping_neg();
        }
    }

    /// Sum of all entries before the given index (exclusive).
    pub fn rank(&self, index: usize) -> usize {
        let mut sum = 0i32;
        let mut i = index;
        while i > 0 {
            sum += self.binary_sums[i];
            i -= i & i.wrapping_neg();
        }
        sum as usize
    }

    /// Current number of elements stored in the tree.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Remove all entries for one index.
    ///
    /// Uses `saturating_sub` to preserve the no-crash invariant when
    /// `nums_per_index[index]` and `size` get out of sync (e.g. stale
    /// `binary_sums` slots beyond the current `max_num` left behind by a
    /// smaller `reset` call). The underlying corruption that lets
    /// `num_entries > size` is a P3 counter follow-up: it produces slightly
    /// wrong crossing counts in pathological inputs but never crashes.
    pub fn remove_all(&mut self, index: usize) {
        let num_entries = self.nums_per_index[index];
        if num_entries == 0 {
            return;
        }
        self.nums_per_index[index] = 0;
        self.size = self.size.saturating_sub(num_entries as usize);
        let mut i = index + 1;
        while i < self.binary_sums.len() {
            self.binary_sums[i] -= num_entries;
            i += i & i.wrapping_neg();
        }
    }

    pub(crate) fn capacity(&self) -> usize {
        self.max_num
    }

    pub(crate) fn retained_bytes(&self) -> usize {
        self.binary_sums.capacity() * std::mem::size_of::<i32>()
            + self.nums_per_index.capacity() * std::mem::size_of::<i32>()
    }
}
