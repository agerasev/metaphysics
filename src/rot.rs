use crate::Param;
use core::f32::consts::PI;
use glam::{Mat2, Mat3, Quat, Vec2, Vec3};

/// 2D Rotation.
#[derive(Clone, Copy, Default, Debug)]
pub struct Rot2(f32);

/// 3D Rotation.
#[derive(Clone, Copy, Debug)]
pub struct Rot3(Quat);

impl From<f32> for Rot2 {
    fn from(value: f32) -> Self {
        Self(value)
    }
}
impl From<Rot2> for f32 {
    fn from(value: Rot2) -> Self {
        value.0
    }
}

impl From<Quat> for Rot3 {
    fn from(value: Quat) -> Self {
        Self(value)
    }
}
impl From<Rot3> for Quat {
    fn from(value: Rot3) -> Self {
        value.0
    }
}

impl Default for Rot3 {
    fn default() -> Self {
        Self(Quat::IDENTITY)
    }
}

impl Rot2 {
    /// Create a 2D rotation from an angle in radians.
    pub fn from_angle(angle: f32) -> Self {
        Self(angle % (2.0 * PI))
    }

    /// Get the angle in radians, in the range [0, 2π)
    pub fn angle(self) -> f32 {
        self.0
    }

    /// Get the angle in degrees, in the range [0, 360)
    pub fn angle_degrees(self) -> f32 {
        (180.0 / PI) * self.angle()
    }

    /// Get the 2D rotation matrix.
    pub fn matrix(self) -> Mat2 {
        Mat2::from_angle(self.0)
    }

    /// Transform a 2D vector by this rotation.
    pub fn transform(&self, v: Vec2) -> Vec2 {
        self.matrix().mul_vec2(v)
    }

    /// Chain this rotation with another rotation.
    pub fn chain(self, other: Self) -> Self {
        Self((self.0 + other.0) % (2.0 * PI))
    }

    /// Get the inverse rotation.
    pub fn inverse(self) -> Self {
        Self(-self.0)
    }
}

impl Rot3 {
    /// Create a 3D rotation from an axis-angle representation.
    ///
    /// # Arguments
    /// + `v` - A vector where the direction represents the rotation axis
    ///   and the magnitude represents the rotation angle in radians.
    pub fn from_scaled_axis(v: Vec3) -> Self {
        Self(Quat::from_scaled_axis(v))
    }

    /// Get the 3D rotation matrix.
    pub fn matrix(self) -> Mat3 {
        Mat3::from_quat(self.0)
    }

    /// Transform a 3D vector by this rotation.
    pub fn transform(self, v: Vec3) -> Vec3 {
        self.0.mul_vec3(v)
    }

    /// Chain this rotation with another rotation.
    ///
    /// # Arguments
    /// + `other` - The other rotation to apply after this one.
    pub fn chain(self, other: Self) -> Self {
        Self(other.0.mul_quat(self.0).normalize())
    }

    /// Get the inverse rotation.
    pub fn inverse(self) -> Self {
        Self(self.0.inverse())
    }
}

impl Param for Rot2 {
    /// Angular speed
    type Deriv = f32;
    fn step(self, dp: f32, dt: f32) -> Self {
        self.chain(Rot2::from_angle(dp * dt))
    }
}
impl Param for Rot3 {
    /// Direction is an axis of rotation.
    /// Length is angular speed around this axis.
    type Deriv = Vec3;
    fn step(self, dp: Vec3, dt: f32) -> Self {
        self.chain(Rot3::from_scaled_axis(dp * dt))
    }
}

/// Compute the moment of force in 2D.
///
/// # Arguments
/// + `pos` - Position the force applied at.
///   Relative to the axis of rotation.
/// + `force` - Force vector.
///
/// # Returns
/// The moment of force value.
pub fn torque2(pos: Vec2, force: Vec2) -> f32 {
    pos.perp_dot(force)
}

/// Compute the moment of force in 3D.
///
/// # Arguments
/// + `pos` - Position the force applied at.
///   Relative to the point the rotation is performed around.
/// + `force` - Force vector.
///
/// # Returns
/// The torque vector which direction is an axis of rotation acceleration,
/// and length is an absolute value of moment of force.
pub fn torque3(pos: Vec3, force: Vec3) -> Vec3 {
    pos.cross(force)
}

/// Compute linear velocity at the point of body having angular velocity in 2D.
///
/// This computes the linear velocity at a given position due to angular velocity.
///
/// # Arguments
/// + `angular` - The angular velocity (scalar)
/// + `pos` - Position vector
///
/// # Returns
/// The linear velocity vector
pub fn angular_to_linear2(angular: f32, pos: Vec2) -> Vec2 {
    angular * pos.perp()
}

/// Compute linear velocity at the point of body having angular velocity in 3D.
///
/// This computes the linear velocity at a given position due to angular velocity.
///
/// # Arguments
/// + `angular` - The angular velocity vector
/// + `pos` - Position vector
///
/// # Returns
/// The linear velocity vector
pub fn angular_to_linear3(angular: Vec3, pos: Vec3) -> Vec3 {
    angular.cross(pos)
}
