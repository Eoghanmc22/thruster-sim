#![allow(incomplete_features)]
#![feature(lazy_type_alias)]

use std::fs;

use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    utils::VectorTransform,
    Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
};
use nalgebra::{vector, ComplexField, Const, Normed, SVector};
use num_dual::{gradient, DualNum, DualVec};
use thruster_sim::{
    heuristic::{ScoreResult, ScoreSettings},
    optimize::{
        self, symetrical::SymerticalOptimization, AsyncOptimizationArena, OptimizationArena,
    },
};

pub const LEARN_RATE: FloatType = 0.01;

// pub const THRUSTER_COUNT: usize = 3;
pub const THRUSTER_COUNT: usize = 6;
pub const DIMENSIONALITY: usize = THRUSTER_COUNT * 6;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.1;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

fn main() {
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");

    let mut arena = AsyncOptimizationArena::new(SymerticalOptimization::<3>);
    arena.reset(25, ScoreSettings::default());

    for _ in 0..500 {
        arena.step(&motor_data);
    }

    // let mut counter = 0;
    // while !points.is_empty() {
    //     if counter % 10 == 0 {
    //         println!("{counter}");
    //     }
    //     let results = gradient_ascent(&points, &heuristic, &motor_data, LEARN_RATE);
    //
    //     let mut remaining_points = vec![];
    //     for mut result in results {
    //         if result.new_point.fountier_threshold.0 * 1.01 < result.old_score {
    //             result.new_point.fountier_threshold = (result.old_score, counter);
    //         }
    //
    //         if result.gradient.norm_squared() < CRITICAL_POINT_EPSILON * CRITICAL_POINT_EPSILON
    //             || counter - result.new_point.fountier_threshold.1 > 25
    //         {
    //             completed_points.push(result);
    //         } else {
    //             remaining_points.push(result.new_point);
    //         }
    //     }
    //
    //     points = remaining_points;
    //     counter += 1;
    // }
    //
    // completed_points.sort_by(|a, b| FloatType::total_cmp(&a.old_score, &b.old_score).reverse());
    //
    // for result in completed_points.iter().take(1) {
    //     println!("point: {:.04?}", result.new_point);
    //     println!(
    //         "score: {:.02}, performance: {:?}",
    //         result.old_score,
    //         reverse::axis_maximums(
    //             &motor_config(result.new_point.point),
    //             &motor_data,
    //             1.0,
    //             0.01
    //         )
    //     );
    //     let _ = fs::write("logs/current.log", format!("{result:#?}"));
    // }
}
