use motor_math::{motor_preformance::MotorData, x3d::X3dMotorId, Direction, Motor, MotorConfig};
use nalgebra::{vector, Vector3};
use num_dual::gradient;

use crate::{
    heuristic::{score, ScoreSettings},
    physics, HEIGHT, LENGTH, WIDTH,
};

pub fn fibonacci_sphere(samples: usize) -> Vec<Vector3<f32>> {
    let mut points = Vec::new();
    let phi = std::f32::consts::PI * (5f32.sqrt() - 1.0); // golden angle in radians

    for i in 0..samples {
        let y = 1.0 - (i as f32 / (samples as f32 - 1.0)) * 2.0; // y goes from 1 to -1
        let radius = (1.0 - y * y).sqrt(); // radius at y

        let theta = phi * i as f32; // golden angle increment

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;

        points.push(vector![x, y, z]);
    }

    points
}

pub fn accent_sphere(
    step: f32,
    point: Vector3<f32>,
    settings: &ScoreSettings,
    motor_data: &MotorData,
) -> (Vector3<f32>, bool) {
    let (_old_score, grad) = gradient(
        |point| {
            let motor_config = MotorConfig::<X3dMotorId, _>::new(
                Motor {
                    orientation: point,
                    position: vector![
                        (WIDTH / 2.0).into(),
                        (LENGTH / 2.0).into(),
                        (HEIGHT / 2.0).into()
                    ],
                    direction: Direction::Clockwise,
                },
                vector![0.0.into(), 0.0.into(), 0.0.into()],
            );

            let result = physics(&motor_config, motor_data, true);
            score(&result, settings)
        },
        point,
    );

    let constrained_grad = grad - grad.dot(&point) * point;
    // println!("cg {:?}", step * constrained_grad);

    let new_point = point + step * constrained_grad;
    let new_point = new_point.normalize();

    let norm_2 = constrained_grad.norm_squared();
    // (new_point, norm_2.is_nan() || norm_2 < 0.000001)
    (new_point, norm_2.is_nan() || norm_2 < 0.01)
}
