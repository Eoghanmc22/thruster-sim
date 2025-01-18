#![allow(incomplete_features)]
#![feature(lazy_type_alias)]

use std::fs;

use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
};
use nalgebra::{vector, Const, SVector};
use num_dual::{gradient, DualVec};
use thruster_sim::{
    heuristic::{ScoreResult, ScoreSettings},
    optimize,
};

pub const STEP_SIZE: FloatType = 0.000001;
pub const DIMENSIONALITY: usize = 3;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.01;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

fn main() {
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");

    let vector = vector![0.254, -0.571, 0.781];

    let config = motor_config(vector);
    println!("config: {config:#.04?}");
    println!("matrix: {:.04}", config.matrix);
    println!("inverse: {:.04}", config.pseudo_inverse);
    let maximums = reverse::axis_maximums(&config, &motor_data, 25.0, 0.00001);
    println!("maximums: {maximums:#.04?}");
    let score = optimize::evaluate(&config, &Default::default(), &motor_data);
    println!("score: {score:#.04?}");
}

pub fn motor_config<D: Number>(point: Point<D>) -> MotorConfig<ErasedMotorId, D> {
    MotorConfig::<X3dMotorId, _>::new(
        Motor {
            position: vector![0.325, 0.355, 0.241].map(D::from),
            orientation: point,
            direction: Direction::Clockwise,
        },
        vector![0.0, 0.0, 0.0].map(D::from),
    )
    .erase()
}
