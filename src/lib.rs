#![feature(coroutines)]
#![feature(iter_from_coroutine)]
// #![feature(generic_const_exprs)]
// #![feature(non_lifetime_binders)]

use motor_math::FloatType;

pub mod heuristic;
pub mod optimize;

pub const WIDTH: FloatType = 0.19 * 2.0;
pub const LENGTH: FloatType = 0.22 * 2.0;
pub const HEIGHT: FloatType = 0.09 * 2.0;
