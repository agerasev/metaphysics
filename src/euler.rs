use crate::{Param, Solver, System, Var, Visitor};

/// Euler's method for solving differential equations.
///
/// This is the simplest numerical integration method, using the derivative at the
/// current time step to advance the solution.
pub struct Euler;

/// Internal step implementation for Euler method.
struct EulerStep {
    dt: f32,
}

impl Visitor for EulerStep {
    type Solver = Euler;
    fn apply<P: Param>(&mut self, p: &mut Var<P, Euler>) {
        p.value = p.value.step(p.deriv, self.dt);
        p.deriv = Default::default();
    }
}

impl Solver for Euler {
    type Storage<P: Param> = ();
    fn solve_step<S: System<Self>>(&self, system: &mut S, dt: f32) {
        system.compute_derivs(dt);
        system.visit_vars(&mut EulerStep { dt });
    }
}
