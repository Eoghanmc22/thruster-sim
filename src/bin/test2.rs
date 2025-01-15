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

pub fn initial_points(count: usize) -> Vec<Point<FloatType>> {
    let mut points = vec![];

    for _ in 0..count {
        points.push(Point::from_fn(|_, _| rand::random()));
    }

    points
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

pub fn normalise_point<D: Number>(point: Point<D>) -> Point<D> {
    point.normalize()
}

fn gradient_ascent<'a>(
    points: &'a [Point<FloatType>],
    heuristic: &'a ScoreSettings,
    motor_data: &'a MotorData,
    step_size: FloatType,
) -> impl Iterator<Item = Ascent> + use<'a> {
    points.iter().map(move |&old_point| {
        let mut result = None;

        let (score, grad) = gradient(
            |point| {
                let motor_config = motor_config(point);
                let (score, score_breakdown) =
                    optimize::evaluate(&motor_config, heuristic, motor_data);
                result = Some(score_breakdown);

                score
            },
            old_point,
        );

        let delta = step_size * grad;
        let new_point = normalise_point(old_point + delta);
        let delta = new_point - old_point;

        Ascent {
            old_point,
            new_point,
            old_score: score,
            est_new_score: score + grad.dot(&delta),
            gradient: grad,
            score_breakdown: result.unwrap(),
        }
    })
}

#[derive(Debug, Clone)]
struct Ascent {
    old_point: Point<FloatType>,
    new_point: Point<FloatType>,

    old_score: FloatType,
    est_new_score: FloatType,

    gradient: Point<FloatType>,
    score_breakdown: ScoreResult<DualVec<FloatType, FloatType, Const<DIMENSIONALITY>>>,
}
