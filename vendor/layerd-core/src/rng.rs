//! 48-bit linear congruential PRNG with the `0x5DEECE66D` multiplier.
//!
//! Constructs one generator per layout invocation from the graph's
//! `RANDOM_SEED` and threads it through every phase that consumes random
//! bits (P1 cycle breaker random tiebreak, P3 layer-sweep barycenter
//! randomization, etc.). The bit-level operations (`next(bits)` shift,
//! rejection-sampling `nextInt(bound)`, and the `nextDouble` /
//! `nextFloat` / `nextBoolean` derivations) are byte-for-byte stable so
//! the same seed produces the same sequence across runs.

/// Trait for random number generation in layout algorithms.
///
/// Production code uses [`SeededRng`]; tests use a mock RNG for
/// deterministic, controllable behaviour.
pub trait Rng {
    fn next_bool(&mut self) -> bool;
    fn next_f32(&mut self) -> f32;
    fn next_f64(&mut self) -> f64;
    fn next_int(&mut self, bound: i32) -> i32 {
        assert!(bound > 0, "bound must be positive");
        ((self.next_f64() * bound as f64) as i32).min(bound - 1)
    }
    /// 64-bit sample used by P3 layer-sweep to capture a stable seed that is
    /// later replayed before each hierarchical sub-graph pass, so copies of
    /// the same subgraph under different parents get the same ordering.
    ///
    /// Default `0`. `MockRng` uses the default; its deterministic sequence
    /// does not depend on reseed.
    fn next_long(&mut self) -> i64 {
        0
    }
    /// Reset the generator state to a fresh seed. Default: no-op, matching
    /// `MockRng`'s intent (a deterministic sequence that ignores reseed).
    fn set_seed(&mut self, _seed: i64) {}
}

/// Linear congruential PRNG: 48-bit state, multiplier `0x5DEECE66D`,
/// addend `0xB`, shift-by-`(48 - bits)` in `next(bits)`, rejection-sampling
/// `nextInt(bound)`, and the `(next(26) << 27 | next(27)) / 2^53` formula
/// for `nextDouble`.
///
/// A seed of 0 maps to a fixed seed of 42 for deterministic test output;
/// any non-zero seed is used as-is.
pub struct SeededRng {
    seed: u64,
}

impl SeededRng {
    const MULTIPLIER: u64 = 0x5DEECE66D;
    const ADDEND: u64 = 0xB;
    const MASK: u64 = (1u64 << 48) - 1;

    pub fn new(seed: u64) -> Self {
        let effective_seed = if seed == 0 { 42 } else { seed };
        Self { seed: (effective_seed ^ Self::MULTIPLIER) & Self::MASK }
    }

    /// Advance the state and return the top `bits` bits of the new state.
    #[inline]
    fn next(&mut self, bits: u32) -> i32 {
        self.seed =
            self.seed.wrapping_mul(Self::MULTIPLIER).wrapping_add(Self::ADDEND) & Self::MASK;
        (self.seed >> (48 - bits)) as i32
    }

    /// Uniform integer in `[0, bound)`.
    ///
    /// Panics on `bound <= 0`.
    pub fn next_int(&mut self, bound: i32) -> i32 {
        assert!(bound > 0, "bound must be positive");

        let m = bound - 1;
        if (bound & m) == 0 {
            // Power-of-two fast path: scale a 31-bit sample to [0, bound).
            return ((bound as i64).wrapping_mul(self.next(31) as i64) >> 31) as i32;
        }

        // Rejection sampling: discard the upper biased range to keep the
        // distribution uniform when `bound` is not a divisor of 2^31.
        let mut u = self.next(31);
        loop {
            let r = u % bound;
            if u.wrapping_sub(r).wrapping_add(m) >= 0 {
                return r;
            }
            u = self.next(31);
        }
    }
}

impl Rng for SeededRng {
    fn next_bool(&mut self) -> bool {
        self.next(1) != 0
    }

    fn next_f32(&mut self) -> f32 {
        // `nextFloat`: `next(24) / (1 << 24)`.
        (self.next(24) as f32) / ((1u32 << 24) as f32)
    }

    fn next_f64(&mut self) -> f64 {
        // `nextDouble`: `((next(26) << 27) + next(27)) / (1L << 53)`.
        ((((self.next(26) as i64) << 27) + self.next(27) as i64) as f64) / ((1i64 << 53) as f64)
    }

    fn next_int(&mut self, bound: i32) -> i32 {
        SeededRng::next_int(self, bound)
    }

    fn next_long(&mut self) -> i64 {
        // `nextLong`: `((long)next(32) << 32) + next(32)`.
        ((self.next(32) as i64) << 32) + self.next(32) as i64
    }

    fn set_seed(&mut self, seed: i64) {
        // `setSeed(long)`: `this.seed = (seed ^ 0x5DEECE66DL) & ((1L << 48) - 1)`.
        self.seed = (seed as u64 ^ Self::MULTIPLIER) & Self::MASK;
    }
}
