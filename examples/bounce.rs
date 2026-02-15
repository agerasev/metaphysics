use phy::{Rk4, Solver, System, Var, Visitor};
use std::fmt::{self, Display, Formatter};

struct BouncingBall<S: Solver> {
    pos: Var<f32, S>,
    vel: Var<f32, S>,
}

const G: f32 = 9.8;
const K: f32 = 1000.0;
const F: f32 = 100.0;

impl<S: Solver> System<S> for BouncingBall<S> {
    fn compute_derivs(&mut self, _dt: f32) {
        self.pos.deriv = *self.vel;
        self.vel.deriv += -G + (K - F * *self.vel) * (-self.pos.min(0.0));
    }
    fn visit_vars<V: Visitor<Solver = S>>(&mut self, visitor: &mut V) {
        visitor.apply(&mut self.pos);
        visitor.apply(&mut self.vel);
    }
}

impl<S: Solver> Display for BouncingBall<S> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        for i in 0..64 {
            if i as f32 * 0.16 < *self.pos {
                write!(f, " ")?;
            } else {
                write!(f, "*")?;
                break;
            }
        }
        Ok(())
    }
}

fn main() {
    let solver = Rk4;

    let mut system = BouncingBall {
        pos: Var::new(10.0),
        vel: Var::new(0.0),
    };

    for _ in 0..40 {
        for _ in 0..10 {
            solver.solve_step(&mut system, 0.01);
        }
        println!("{}", system);
    }
}
