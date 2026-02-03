use crate::{Param, Solver};
use core::{
    fmt::{self, Debug, Formatter},
    ops::{Deref, DerefMut},
};

/// Independent variable for use in differential equation systems.
///
/// This struct represents a variable that can be integrated over time in a system
/// of differential equations.
pub struct Var<P: Param, S: Solver> {
    /// The current value of the variable
    pub value: P,
    /// The derivative of the variable
    pub deriv: P::Deriv,
    /// Solver-specific storage for intermediate computations
    pub storage: S::Storage<P>,
}

impl<P: Param, S: Solver> Clone for Var<P, S> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<P: Param, S: Solver> Copy for Var<P, S> {}
impl<P: Param, S: Solver> Default for Var<P, S> {
    fn default() -> Self {
        Self {
            value: P::default(),
            deriv: P::Deriv::default(),
            storage: S::Storage::<P>::default(),
        }
    }
}

impl<P: Param, S: Solver> Var<P, S> {
    /// Create a new variable with the specified initial value.
    ///
    /// # Arguments
    /// + `value` - The initial value of the variable
    ///
    /// # Returns
    /// A new variable with the specified value and empty derivative/storage
    pub fn new(value: P) -> Self {
        Var {
            value,
            deriv: Default::default(),
            storage: Default::default(),
        }
    }
}

impl<P: Param, S: Solver> Deref for Var<P, S> {
    type Target = P;
    fn deref(&self) -> &Self::Target {
        &self.value
    }
}
impl<P: Param, S: Solver> DerefMut for Var<P, S> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

impl<P: Param, S: Solver> Debug for Var<P, S>
where
    P: Debug,
    P::Deriv: Debug,
    S::Storage<P>: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Var {{ value: {:?}, deriv: {:?}, storage: {:?} }}",
            &self.value, &self.deriv, &self.storage
        )
    }
}
