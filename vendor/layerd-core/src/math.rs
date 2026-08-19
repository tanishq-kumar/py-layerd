use std::ops::{Add, AddAssign, Sub};

/// A 2D vector with f64 components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }
}

impl Default for Vec2 {
    fn default() -> Self {
        Vec2::ZERO
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;

    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

/// Padding around a node or graph.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Padding {
    pub top: f64,
    pub right: f64,
    pub bottom: f64,
    pub left: f64,
}

impl Padding {
    pub fn uniform(value: f64) -> Self {
        Padding { top: value, right: value, bottom: value, left: value }
    }

    pub fn horizontal(&self) -> f64 {
        self.left + self.right
    }

    pub fn vertical(&self) -> f64 {
        self.top + self.bottom
    }
}

impl Default for Padding {
    fn default() -> Self {
        Padding { top: 0.0, right: 0.0, bottom: 0.0, left: 0.0 }
    }
}

/// Margin around a node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Margin {
    pub top: f64,
    pub right: f64,
    pub bottom: f64,
    pub left: f64,
}

impl Margin {
    pub fn horizontal(&self) -> f64 {
        self.left + self.right
    }

    pub fn vertical(&self) -> f64 {
        self.top + self.bottom
    }
}

impl Default for Margin {
    fn default() -> Self {
        Margin { top: 0.0, right: 0.0, bottom: 0.0, left: 0.0 }
    }
}
