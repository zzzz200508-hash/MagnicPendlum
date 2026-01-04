use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub(crate) fn scale(&self, scalar: f64) -> Vector3D {
        Vector3D {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self{
            x: x,
            y: y,
            z: z,
        }
    }
}

impl Vector3D {
    pub(crate) fn length_squared(self) -> f64 { self.x * self.x + self.y * self.y + self.z * self.z}
    pub(crate) fn length(self) -> f64 { self.length_squared().sqrt() }
}
impl std::ops::Add for Vector3D {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output { Self { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z } }
}
impl std::ops::Sub for Vector3D {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output { Self { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z } }
}
impl std::ops::Mul<f64> for Vector3D {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output { Self { x: self.x * rhs, y: self.y * rhs, z: self.z * rhs } }
}
impl std::ops::Div<f64> for Vector3D {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output { Self { x: self.x / rhs, y: self.y / rhs, z: self.z / rhs } }
}

impl std::ops::Mul<Vector3D> for Vector3D {
    type Output = Self;
    fn mul(self, rhs: Vector3D) -> Self::Output { Self { x: self.x * rhs.x, y: self.y * rhs.y, z: self.z * rhs.z } }
}
impl std::ops::Div<Vector3D> for Vector3D {
    type Output = Self;
    fn div(self, rhs: Vector3D) -> Self::Output { Self { x: self.x / rhs.x, y: self.y / rhs.y, z: self.z / rhs.z } }
}

impl Vector3D {
    fn dot(self, rhs: Vector3D) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    fn times(self, rhs: Vector3D) -> Self {
        Self{
            x: self.y * rhs.z - self.z * rhs.y,
            y: - self.x * rhs.z + self.z * rhs.x,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}
