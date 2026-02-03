# phy

A generic extendable first-order differential equation solver.

## Overview

The crate provides a framework for solving first-order differential equations using various numerical integration methods. It's designed to be generic and extensible, supporting different parameter types and solvers.

## Features

- Different parameter types and their derivatives
- Generic solvers. For now there are Euler's method and Runge-Kutta 4th order (RK4)
- Built-in support for 2D and 3D rotations with proper angular mathematics
- Easy to add new solvers and parameter types
- Uses `#![no_std]` and `glam` crate for math operations

## Usage

Example usage with a simple system:

```rust
use phy::{Euler, Rk4, Var, System, Visitor};

// Define your system parameters
struct MySystem {
    position: Var<f32, Euler>,
    velocity: Var<f32, Euler>,
}

impl System<Euler> for MySystem {
    fn compute_derivs(&mut self, dt: f32) {
        // Compute derivatives for the system
        self.position.deriv = self.velocity.value;
        self.velocity.deriv = -9.81; // Gravity
    }

    fn visit_vars<V: Visitor<Solver = Euler>>(&mut self, visitor: &mut V) {
        visitor.apply(&mut self.position);
        visitor.apply(&mut self.velocity);
    }
}
```
