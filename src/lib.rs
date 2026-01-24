#![no_std]

#[cfg(feature = "std")]
extern crate std;

mod euler;
mod rk4;
mod rot;

use core::ops::{Add, Deref, DerefMut, Mul};

use glam::{Vec2, Vec3};

pub use crate::{euler::Euler, rk4::Rk4, rot::*};

/// System parameter (degrees of freedom).
pub trait Param: Sized + Copy + Default {
    type Deriv: Sized + Copy + Add<Output = Self::Deriv> + Mul<f32, Output = Self::Deriv> + Default;
    fn step(self, deriv: Self::Deriv, dt: f32) -> Self;
}

/// Independent variable.
#[derive(Clone, Copy, Default, Debug)]
pub struct Var<P: Param> {
    pub(crate) value: (P, P),
    pub(crate) deriv: (P::Deriv, P::Deriv),
}

impl<P: Param> Var<P> {
    pub fn new(value: P) -> Self {
        Var {
            value: (value, Default::default()),
            deriv: Default::default(),
        }
    }
    pub fn into_value(self) -> P {
        self.value.0
    }
    pub fn value(&self) -> &P {
        &self.value.0
    }
    pub fn value_mut(&mut self) -> &mut P {
        &mut self.value.0
    }

    pub fn add_deriv(&mut self, d: P::Deriv) {
        self.deriv.0 = self.deriv.0 + d;
    }
    pub fn deriv(&self) -> &P::Deriv {
        &self.deriv.0
    }
    pub fn deriv_mut(&mut self) -> &mut P::Deriv {
        &mut self.deriv.0
    }
}

impl<P: Param> Deref for Var<P> {
    type Target = P;
    fn deref(&self) -> &Self::Target {
        self.value()
    }
}
impl<P: Param> DerefMut for Var<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.value_mut()
    }
}

pub trait Visitor {
    fn apply<P: Param>(&mut self, v: &mut Var<P>);
}

pub trait System {
    fn compute_derivs(&mut self, dt: f32);
    fn visit_vars<V: Visitor>(&mut self, visitor: &mut V);
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
pub trait Solver {
    fn solve_step<S: System>(&self, system: &mut S, dt: f32);
}
