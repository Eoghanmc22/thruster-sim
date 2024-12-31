#![allow(incomplete_features)]
#![feature(lazy_type_alias)]

use core::f64;

use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
};
use nalgebra::{vector, SVector};
use num_dual::gradient;
use thruster_sim::{
    heuristic::ScoreSettings,
    optimize::{self, fibonacci_sphere},
};

pub const STEP_SIZE: FloatType = 0.1;
pub const DIMENSIONALITY: usize = 3;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.01;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

fn main() {
    let motor_data = motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

    let mut points = initial_points(100);
    let mut completed_points = vec![];
    let heuristic = ScoreSettings::default();

    let mut counter = 0;
    while !points.is_empty() {
        println!("{counter}");
        let results = gradient_ascent(&points, &heuristic, &motor_data, STEP_SIZE);

        let mut remaining_points = vec![];
        for result in results {
            // println!("{}", result.gradient);
            println!("{:?}", result);
            if result.gradient.norm_squared() < CRITICAL_POINT_EPSILON * CRITICAL_POINT_EPSILON {
                // if (result.est_new_score - result.old_score) / STEP_SIZE < CRITICAL_POINT_EPSILON {
                completed_points.push(result);
            } else {
                remaining_points.push(result.new_point);
            }
        }

        points = remaining_points;
        counter += 1;
    }

    completed_points.sort_by(|a, b| FloatType::total_cmp(&a.old_score, &b.old_score).reverse());

    for result in completed_points.iter().take(10) {
        println!("point: {:.04}", result.new_point);
        println!(
            "score: {:.02}, performance: {:?}",
            result.old_score,
            reverse::axis_maximums(&motor_config(result.new_point), &motor_data, 1.0, 0.01)
        );
    }
}

pub fn initial_points(count: usize) -> Vec<Point<FloatType>> {
    optimize::fibonacci_sphere(count)
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
        let (score, grad) = gradient(
            |point| {
                let motor_config = motor_config(point);
                optimize::evaluate(&motor_config, heuristic, motor_data)
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
        }
    })
}

#[derive(Debug)]
struct Ascent {
    old_point: Point<FloatType>,
    new_point: Point<FloatType>,

    old_score: FloatType,
    est_new_score: FloatType,

    gradient: Point<FloatType>,
}
