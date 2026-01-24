use crate::{Param, Solver, System, Var, Visitor};

/// The Runge–Kutta method (RK4).
pub struct Rk4;

struct Rk4Step {
    stage: u32,
    dt: f32,
}

impl Rk4Step {
    fn dt(&self) -> f32 {
        match self.stage {
            0 => self.dt / 2.0,
            1 => self.dt / 2.0,
            2 => self.dt,
            3 => self.dt,
            _ => unreachable!(),
        }
    }
}

impl Visitor for Rk4Step {
    fn apply<P: Param>(&mut self, p: &mut Var<P>) {
        let x = &mut p.value;
        let dx = &mut p.deriv;
        let dt = self.dt();

        match self.stage {
            0 => {
                (x.1, dx.1) = (x.0, dx.0);
                x.0 = x.1.step(dx.0, dt);
            }
            1 => {
                dx.1 = dx.1 + dx.0 * 2.0;
                x.0 = x.1.step(dx.0, dt);
            }
            2 => {
                dx.1 = dx.1 + dx.0 * 2.0;
                x.0 = x.1.step(dx.0, dt);
            }
            3 => {
                dx.1 = (dx.1 + dx.0) * (1.0 / 6.0);
                x.0 = x.1.step(dx.1, dt);
            }
            _ => unreachable!(),
        };

        dx.0 = Default::default();
    }
}

impl Solver for Rk4 {
    fn solve_step<S: System>(&self, system: &mut S, dt: f32) {
        for stage in 0..4 {
            let mut step = Rk4Step { stage, dt };
            system.compute_derivs(step.dt());
            system.visit_vars(&mut step);
        }
    }
}
