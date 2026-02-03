use crate::{Param, Solver, System, Var, Visitor};

/// The Runge–Kutta method (RK4) for solving differential equations.
///
/// This is a fourth-order numerical integration method that provides better accuracy
/// and stability compared to Euler's method by using multiple evaluations of the
/// derivative within each time step.
pub struct Rk4;

/// Storage type for RK4 solver.
#[derive(Clone, Copy, Default, Debug)]
pub struct Rk4Storage<P: Param> {
    /// The current value of the parameter
    value: P,
    /// The current derivative of the parameter
    deriv: P::Deriv,
}

/// Internal step implementation for RK4 method.
struct Rk4Step {
    stage: u32,
    dt: f32,
}

impl Rk4Step {
    /// Calculate the time step for the current stage
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
    type Solver = Rk4;
    fn apply<P: Param>(&mut self, p: &mut Var<P, Self::Solver>) {
        let x = &mut p.value;
        let dx = &mut p.deriv;
        let sx = &mut p.storage.value;
        let sdx = &mut p.storage.deriv;
        let dt = self.dt();

        match self.stage {
            0 => {
                (*sx, *sdx) = (*x, *dx);
                *x = sx.step(*dx, dt);
            }
            1 => {
                *sdx = *sdx + *dx * 2.0;
                *x = sx.step(*dx, dt);
            }
            2 => {
                *sdx = *sdx + *dx * 2.0;
                *x = sx.step(*dx, dt);
            }
            3 => {
                *sdx = (*sdx + *dx) * (1.0 / 6.0);
                *x = sx.step(*sdx, dt);
            }
            _ => unreachable!(),
        };

        *dx = Default::default();
    }
}

impl Solver for Rk4 {
    type Storage<P: Param> = Rk4Storage<P>;
    fn solve_step<S: System<Self>>(&self, system: &mut S, dt: f32) {
        for stage in 0..4 {
            let mut step = Rk4Step { stage, dt };
            system.compute_derivs(step.dt());
            system.visit_vars(&mut step);
        }
    }
}
