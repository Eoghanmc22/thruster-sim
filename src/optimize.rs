use motor_math::{
    motor_preformance::MotorData, x3d::X3dMotorId, Direction, FloatType, Motor, MotorConfig,
};
use nalgebra::{vector, ComplexField, Matrix, Vector3};
use num_dual::{gradient, DualNum};

use crate::{
    heuristic::{score, ScoreSettings},
    physics, HEIGHT, LENGTH, WIDTH,
};

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

pub fn accent_sphere(
    step: FloatType,
    point: Vector3<FloatType>,
    settings: &ScoreSettings,
    motor_data: &MotorData,
) -> (Vector3<FloatType>, bool) {
    let g = |point| {
        let mut motor = Motor {
            orientation: point,
            position: vector![
                (WIDTH / 2.0).into(),
                (LENGTH / 2.0).into(),
                (HEIGHT / 2.0).into()
            ],
            direction: Direction::Clockwise,
        };
        motor.orientation.normalize_mut();

        let motor_config =
            MotorConfig::<X3dMotorId, _>::new(motor, vector![0.0.into(), 0.0.into(), 0.0.into()]);

        let result = physics(&motor_config, motor_data, false);
        score(&result, settings)
    };

    let (old_score, grad) = gradient(g, point);

    // let constrained_grad = grad - grad.dot(&point) * point;
    // println!("cg {:?}", step * constrained_grad);
    let constrained_grad = grad;

    let new_point = point + step * constrained_grad;
    let new_point = new_point.normalize();

    let norm_2 = constrained_grad.norm_squared();

    const SANITY_CHECK: bool = true;
    if SANITY_CHECK {
        let (new_score, _grad) = gradient(g, new_point);

        let secant = (new_score - old_score) / (new_point - point).norm();
        let grad_mag = norm_2.sqrt();
        let derv_diff = (secant - grad_mag).abs();
        let diff = new_point - point;

        // if derv_diff > 0.005 {
        if derv_diff > 5.0 {
            println!("New: {new_score}, old: {old_score}, secant: {secant}, gradient: {grad_mag}, derv diff: {derv_diff}, point diff: {diff}, grad vec: {constrained_grad}");
        } else {
            println!("good");
        }
    }

    // (new_point, norm_2.is_nan() || norm_2 < 0.000001)
    (new_point, norm_2.is_nan() || norm_2 < 0.01)
}
