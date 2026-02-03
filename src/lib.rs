#![no_std]

mod euler;
mod rk4;
mod rot;
mod var;

pub use crate::{euler::Euler, rk4::Rk4, rot::*, var::Var};

use core::ops::{Add, Mul};
use glam::{Vec2, Vec3};

/// System parameter (degrees of freedom).
///
/// This trait represents a parameter that can be integrated over time.
/// It defines how to advance the parameter given its derivative and a time step.
pub trait Param: Sized + Copy + Default {
    /// The type of derivative for this parameter.
    type Deriv: Sized + Copy + Add<Output = Self::Deriv> + Mul<f32, Output = Self::Deriv> + Default;

    /// Advance the parameter by one time step.
    ///
    /// # Arguments
    /// + `deriv` - The derivative of the parameter
    /// + `dt` - The time step
    ///
    /// # Returns
    /// The parameter advanced by the derivative times the time step
    fn step(self, deriv: Self::Deriv, dt: f32) -> Self;
}

/// Visitor pattern for applying operations to variables.
///
/// This trait allows systems to apply operations to all variables in a uniform way.
pub trait Visitor {
    /// The solver type this visitor works with.
    type Solver: Solver;

    /// Apply the visitor's operation to a variable.
    fn apply<P: Param>(&mut self, v: &mut Var<P, Self::Solver>);
}

/// A system which evolution in time we want to simulate.
///
/// It consists of variables accessible by [`Self::visit_vars`] and
/// computes derivatives of these variables using [`Self::compute_derivs`].
pub trait System<S: Solver + ?Sized> {
    /// Compute the derivatives for all variables in the system.
    ///
    /// In general, time step `dt` is not required for derivative computation
    /// but in some cases (e.g. static frtion simulation) it may be used
    /// to increase the stability of solution.
    fn compute_derivs(&mut self, dt: f32);

    /// Visit all variables in the system with the provided visitor.
    fn visit_vars<V: Visitor<Solver = S>>(&mut self, visitor: &mut V);
}

impl Param for f32 {
    type Deriv = f32;
    fn step(self, deriv: f32, dt: f32) -> Self {
        self + deriv * dt
    }
}
impl Param for Vec2 {
    type Deriv = Vec2;
    fn step(self, deriv: Vec2, dt: f32) -> Self {
        self + deriv * dt
    }
}
impl Param for Vec3 {
    type Deriv = Vec3;
    fn step(self, deriv: Vec3, dt: f32) -> Self {
        self + deriv * dt
    }
}

/// Temporal differential equation solver.
///
/// This trait defines the interface for numerical solvers that advance systems of
/// differential equations over time.
pub trait Solver {
    /// The storage type needed for this solver for each parameter type.
    type Storage<P: Param>: Sized + Clone + Copy + Default;

    /// Perform a single integration step for the system.
    ///
    /// # Arguments
    /// + `system` - The system to integrate
    /// + `dt` - The time step for this integration step
    fn solve_step<S: System<Self>>(&self, system: &mut S, dt: f32);
}
