use motor_math::{motor_preformance::MotorData, solve::reverse, FloatType, MotorConfig, Number};
use nalgebra::{vector, Vector3};
use std::{fmt::Debug, hash::Hash};

use crate::heuristic::{score, ScoreSettings};

pub fn fibonacci_sphere(samples: usize) -> Vec<Vector3<FloatType>> {
    let mut points = Vec::new();
    let phi = (core::f64::consts::PI * (5f64.sqrt() - 1.0)) as FloatType; // golden angle in radians

    for i in 0..samples {
        let y = 1.0 - (i as FloatType / (samples as FloatType - 1.0)) * 2.0; // y goes from 1 to -1
        let radius = FloatType::sqrt(1.0 - y * y); // radius at y

        let theta = phi * i as FloatType; // golden angle increment

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;

        points.push(vector![x, y, z]);
    }

    points
}

pub fn evaluate<MotorId: Debug + Ord + Hash + Clone, D: Number>(
    motor_config: &MotorConfig<MotorId, D>,
    settings: &ScoreSettings,
    motor_data: &MotorData,
) -> D {
    let result = reverse::axis_maximums(motor_config, motor_data, 1.0, 0.05);
    score(&result, settings)
}
