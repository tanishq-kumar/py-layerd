//! Pure vector / angle utilities.
//!
//! Pure vector / angle utilities consumed by the spline router and the final
//! bend-point calculator. Kept as free functions because there is no per-call
//! state.

use crate::graph::port::PortSide;

/// Differences below this value are treated as zero.
pub const EPSILON: f64 = 0.00000001;

/// 1/2 * Pi.
pub const HALF_PI: f64 = std::f64::consts::FRAC_PI_2;

/// 3/2 * Pi.
pub const THREE_HALF_PI: f64 = 3.0 * std::f64::consts::FRAC_PI_2;

/// Direction (radians) pointing from a node's center to the given port side.
pub fn port_side_to_direction(side: PortSide) -> f64 {
    match side {
        PortSide::North => THREE_HALF_PI,
        PortSide::East => 0.0,
        PortSide::South => HALF_PI,
        PortSide::West => std::f64::consts::PI,
        PortSide::Undefined => 0.0,
    }
}

/// Whether `value` lies between the two boundaries (inclusive). Double overload
/// with `EPSILON` tolerance, using a tri-state comparison so the spline
/// router's slot-overlap test stays consistent across boundary orderings.
pub fn is_between_f64(value: f64, boundary0: f64, boundary1: f64) -> bool {
    if (boundary0 - value).abs() < EPSILON || (boundary1 - value).abs() < EPSILON {
        return true;
    }
    if boundary0 - value > EPSILON {
        value - boundary1 > EPSILON
    } else {
        boundary1 - value > EPSILON
    }
}
