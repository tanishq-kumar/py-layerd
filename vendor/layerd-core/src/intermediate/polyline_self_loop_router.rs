use crate::math::Vec2;

/// Signum that maps zero to zero (vs `f64::signum` which returns `1.0` for
/// `+0.0` and `-1.0` for `-0.0`).
#[inline]
fn zero_preserving_signum(v: f64) -> f64 {
    if v == 0.0 { 0.0 } else { v.signum() }
}

/// Cut corners of an orthogonal polyline by replacing each corner with two
/// points offset by `distance` along the adjacent segments.
pub fn cut_corners(bend_points: &[Vec2], distance: f64) -> Vec<Vec2> {
    assert!(bend_points.len() > 2, "need more than 2 bend points");

    let mut result = Vec::new();

    for i in 1..bend_points.len() - 1 {
        let previous = &bend_points[i - 1];
        let corner = &bend_points[i];
        let next = &bend_points[i + 1];

        let mut offset1 = Vec2::new(previous.x - corner.x, previous.y - corner.y);
        let mut offset2 = Vec2::new(next.x - corner.x, next.y - corner.y);

        // Near-zero snapping
        if offset1.x.abs() < 1e-6 {
            offset1.x = 0.0;
        }
        if offset1.y.abs() < 1e-6 {
            offset1.y = 0.0;
        }
        if offset2.x.abs() < 1e-6 {
            offset2.x = 0.0;
        }
        if offset2.y.abs() < 1e-6 {
            offset2.y = 0.0;
        }

        let mut effective_distance = distance;
        effective_distance = effective_distance.min((offset1.x + offset1.y).abs() / 2.0);
        effective_distance = effective_distance.min((offset2.x + offset2.y).abs() / 2.0);

        // Use the local zero-preserving signum (Rust signum returns 1.0 for 0.0).
        offset1.x = zero_preserving_signum(offset1.x) * effective_distance;
        offset1.y = zero_preserving_signum(offset1.y) * effective_distance;
        offset2.x = zero_preserving_signum(offset2.x) * effective_distance;
        offset2.y = zero_preserving_signum(offset2.y) * effective_distance;

        result.push(Vec2::new(corner.x + offset1.x, corner.y + offset1.y));
        result.push(Vec2::new(corner.x + offset2.x, corner.y + offset2.y));
    }

    result
}
