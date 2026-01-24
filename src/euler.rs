use crate::{Param, Solver, System, Var, Visitor};

/// Euler's method.
pub struct Euler;

struct EulerStep {
    dt: f32,
}

impl Visitor for EulerStep {
    fn apply<P: Param>(&mut self, p: &mut Var<P>) {
        let x = &mut p.value;
        let dx = &mut p.deriv;
        let dt = self.dt;

        x.0 = x.0.step(dx.0, dt);
        dx.0 = Default::default();
    }
}

impl Solver for Euler {
    fn solve_step<S: System>(&self, system: &mut S, dt: f32) {
        system.compute_derivs(dt);
        system.visit_vars(&mut EulerStep { dt });
    }
}
