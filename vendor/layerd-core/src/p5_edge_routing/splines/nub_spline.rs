//! Non-uniform B-spline (NUB) implementation.
//!
//! Non-Uniform B-Spline (NUBS) in polar form (Sederberg notation). Every
//! control point carries a knot sliding-window (`polar_coordinate`) that
//! tracks its position in the knot vector; knot insertion then advances the
//! window by blossoming neighbouring control points.
//!
//! The de Boor recursion is intentionally implemented verbatim (no external
//! `nurbs` crate) because standard de Boor implementations drift at the
//! `~1e-6` level in interior control points, which compounds through
//! `FinalSplineBendpointsCalculator` and changes final bend coordinates.

use crate::math::Vec2;

/// Default dimension of the spline.
pub const DIM: i32 = 3;

/// Doubles with a difference less than this value are treated as equal.
const EPSILON: f64 = 0.000001;

/// A control point paired with its polar coordinate (Sederberg notation).
#[derive(Debug, Clone)]
pub struct PolarCP {
    pub cp: Vec2,
    pub polar_coordinate: Vec<f64>,
}

impl PolarCP {
    /// Fresh polar CP from a raw control point and its polar window.
    pub fn new(control_point: Vec2, polar_coordinate: Vec<f64>) -> Self {
        PolarCP { cp: control_point, polar_coordinate }
    }

    /// Blossom two adjacent polar CPs at `new_knot`.
    pub fn blossom(first: &PolarCP, second: &PolarCP, new_knot: f64) -> Self {
        let first_factor = *first.polar_coordinate.first().expect("polar coord empty");
        let second_factor = *second.polar_coordinate.last().expect("polar coord empty");

        // vectorC = ((b-c) * vectorA + (c-a) * vectorB) / (b-a)
        let a_scaled = Vec2::new(
            first.cp.x * (second_factor - new_knot),
            first.cp.y * (second_factor - new_knot),
        );
        let b_scaled = Vec2::new(
            second.cp.x * (new_knot - first_factor),
            second.cp.y * (new_knot - first_factor),
        );
        let mut total = Vec2::new(a_scaled.x + b_scaled.x, a_scaled.y + b_scaled.y);
        let scale = 1.0 / (second_factor - first_factor);
        total.x *= scale;
        total.y *= scale;

        let mut polar_coordinate: Vec<f64> = Vec::new();
        let mut needs_to_be_added = true;

        // Copy the tail of first's polar coord (dropping the first element),
        // inserting new_knot in sorted position.
        let mut iter = first.polar_coordinate.iter();
        iter.next();
        for &next_knot in iter {
            if needs_to_be_added && next_knot - new_knot > EPSILON {
                polar_coordinate.push(new_knot);
                needs_to_be_added = false;
            }
            polar_coordinate.push(next_knot);
        }
        if needs_to_be_added {
            polar_coordinate.push(new_knot);
        }

        PolarCP { cp: total, polar_coordinate }
    }
}

/// Non-Uniform B-Spline in polar form.
#[derive(Debug, Clone)]
pub struct NubSpline {
    knot_vector: Vec<f64>,
    control_points: Vec<PolarCP>,
    dim_nubs: i32,
    is_clamped: bool,
    is_bezier: bool,
    min_knot: f64,
    max_knot: f64,
}

impl NubSpline {
    /// Create a uniform NubSpline with the specified control points.
    ///
    /// Dimension must be `>= 1`. Panics on invalid inputs.
    pub fn new(clamped: bool, dimension: i32, k_vectors: Vec<Vec2>) -> Self {
        if dimension < 1 {
            panic!("The dimension must be at least 1!");
        }

        // Pre-pad the control-point list to `>= dimension + 1` by duplicating
        // the first element at the front.
        let mut k_vectors = k_vectors;
        let mut i = k_vectors.len() as i32 - 1;
        while i < dimension {
            let first = k_vectors[0];
            k_vectors.insert(0, first);
            i += 1;
        }

        assert!(
            k_vectors.len() as i32 > dimension,
            "At (least dimension + 1) control points are necessary!"
        );

        let mut spline = NubSpline {
            knot_vector: Vec::new(),
            control_points: Vec::new(),
            dim_nubs: dimension,
            is_clamped: clamped,
            is_bezier: false,
            min_knot: 0.0,
            max_knot: 0.0,
        };

        // Create the knot vector.
        spline.create_uniform_knot_vector(clamped, k_vectors.len() as i32 + dimension - 1);

        // Create the PolarCPs using a sliding window over the knot vector.
        let mut polar_coordinate: Vec<f64> = Vec::new();
        let mut knot_iter = spline.knot_vector.iter().copied();

        for _ in 0..(spline.dim_nubs - 1) {
            polar_coordinate.push(knot_iter.next().expect("knot vector shorter than dim - 1"));
        }

        for kv in k_vectors {
            polar_coordinate.push(knot_iter.next().expect("knot vector exhausted"));
            spline.control_points.push(PolarCP::new(kv, polar_coordinate.clone()));
            polar_coordinate.remove(0);
        }

        spline.min_knot = *spline.knot_vector.first().expect("empty knot vector");
        spline.max_knot = *spline.knot_vector.last().expect("empty knot vector");

        spline
    }

    /// Multiplicity of `knot_to_check` inside the knot vector.
    fn get_multiplicity(&self, knot_to_check: f64) -> i32 {
        let mut count = 0;
        for &current in &self.knot_vector {
            let diff = current - knot_to_check;
            if diff > EPSILON {
                return count;
            } else if diff > -EPSILON {
                count += 1;
            }
        }
        count
    }

    /// Build a uniform knot vector in `0.0 ..= 1.0`.
    fn create_uniform_knot_vector(&mut self, clamped: bool, size: i32) {
        assert!(
            size >= 2 * self.dim_nubs,
            "The knot vector must have at least two times the dimension elements."
        );
        let my_size;
        if clamped {
            self.min_knot = 0.0;
            self.max_knot = 1.0;
            for _ in 0..self.dim_nubs {
                self.knot_vector.push(0.0);
            }
            my_size = (size + 1 - 2 * self.dim_nubs) as f64;
        } else {
            my_size = (size + 1) as f64;
            let ddim = self.dim_nubs as f64;
            self.min_knot = ddim / (my_size + 1.0);
            self.max_knot = (my_size - ddim) / my_size;
        }

        let fraction = my_size;
        let count = my_size as i32;
        for i in 1..count {
            self.knot_vector.push(i as f64 / fraction);
        }

        if self.is_clamped {
            for _ in 0..self.dim_nubs {
                self.knot_vector.push(1.0);
            }
        }
    }

    /// Insert new knots + matching blossomed control points.
    ///
    /// `cp_cursor` and `knot_cursor` follow `ListIterator` semantics: the
    /// index of the element `next()` would return. `previous()` means
    /// `cursor -= 1`, `next()` means `cursor += 1`, `add(x)` inserts at
    /// `cursor` and advances past, `remove()` deletes at `cursor - 1` (the
    /// last returned element) and leaves `cursor` unchanged in element terms
    /// but the underlying index shifts.
    fn insert_knot_at_current_position(
        &mut self,
        insertions: i32,
        knot_to_insert: f64,
        cp_cursor: &mut usize,
        knot_cursor: &mut usize,
    ) {
        let multiplicity = self.get_multiplicity(knot_to_insert);
        for i in 0..insertions {
            // iterKnot.add(knot_to_insert) inserts before cursor and advances cursor.
            self.knot_vector.insert(*knot_cursor, knot_to_insert);
            *knot_cursor += 1;

            // Collect the new CPs derived from the blossom of adjacent old CPs.
            let mut new_cps: Vec<PolarCP> = Vec::new();

            let mut second = self.control_points[*cp_cursor].clone();
            *cp_cursor += 1;

            for _j in (multiplicity + i)..self.dim_nubs {
                let first = second.clone();
                second = self.control_points[*cp_cursor].clone();
                *cp_cursor += 1;
                new_cps.push(PolarCP::blossom(&first, &second, knot_to_insert));
            }

            // Walk back; skip the first previous() (no remove), then remove
            // one per step starting at `j > multiplicity + i`.
            for j in (multiplicity + i)..self.dim_nubs {
                *cp_cursor -= 1;
                if j > multiplicity + i {
                    self.control_points.remove(*cp_cursor);
                }
            }

            // Insert the new CPs at the cursor.
            for cp in new_cps {
                self.control_points.insert(*cp_cursor, cp);
                *cp_cursor += 1;
            }

            // Rewind for the next insertion iteration, if any.
            if i < insertions - 1 {
                for _j in (multiplicity + i)..self.dim_nubs {
                    *cp_cursor -= 1;
                }
            }
        }
    }

    /// Convert this NubSpline into bezier form. Every interior knot is raised
    /// to multiplicity `dim`.
    pub fn convert_to_bezier(&mut self) {
        // Remove/skip the leading knots that do not require replication.
        let skip_count = if self.is_clamped {
            self.dim_nubs as usize
        } else {
            (self.dim_nubs - 1) as usize
        };

        if !self.is_clamped {
            // Remove the leading (dim-1) knots.
            for _ in 0..skip_count {
                self.knot_vector.remove(0);
            }
        }

        let mut knot_cursor: usize = if self.is_clamped { self.dim_nubs as usize } else { 0 };
        let mut cp_cursor: usize = 0;

        let mut current_knot = self.knot_vector[knot_cursor];
        knot_cursor += 1;

        while self.max_knot - current_knot > EPSILON {
            let knot_to_count = current_knot;
            let mut occurrence = 0i32;

            while (current_knot - knot_to_count).abs() < EPSILON {
                occurrence += 1;
                current_knot = self.knot_vector[knot_cursor];
                knot_cursor += 1;
                cp_cursor += 1;
            }

            if occurrence < self.dim_nubs {
                // iterKnot.previous(); step back before inserting.
                knot_cursor -= 1;
                self.insert_knot_at_current_position(
                    self.dim_nubs - occurrence,
                    knot_to_count,
                    &mut cp_cursor,
                    &mut knot_cursor,
                );
                // iterKnot.next(); advance past the block we just inserted.
                knot_cursor += 1;
            }

            cp_cursor -= 1;
        }

        if !self.is_clamped {
            // Remove trailing (dim-1) knots.
            let trailing = (self.dim_nubs - 1) as usize;
            let len = self.knot_vector.len();
            self.knot_vector.truncate(len.saturating_sub(trailing));
        }
        self.is_clamped = true;
        self.is_bezier = true;
    }

    /// Return an equivalent list of bezier control points. Converts to bezier
    /// form first when needed.
    pub fn get_bezier_cp(
        &mut self,
        with_source_vector: bool,
        with_target_vector: bool,
    ) -> Vec<Vec2> {
        if !self.is_bezier {
            self.convert_to_bezier();
        }
        let mut result: Vec<Vec2> = Vec::with_capacity(self.control_points.len());
        let mut iter = self.control_points.iter();

        if !with_source_vector {
            iter.next();
        }
        for cp in iter {
            result.push(cp.cp);
        }
        if !with_target_vector {
            result.pop();
        }
        result
    }

    /// Bezier control points without source and target vectors.
    pub fn get_bezier_cp_default(&mut self) -> Vec<Vec2> {
        self.get_bezier_cp(false, false)
    }
}
