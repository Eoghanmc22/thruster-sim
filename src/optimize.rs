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
    // let steps = [
    //     Quat::IDENTITY,
    //     Quat::from_rotation_x(step),
    //     Quat::from_rotation_x(-step),
    //     Quat::from_rotation_y(step),
    //     Quat::from_rotation_y(-step),
    //     Quat::from_rotation_z(step),
    //     Quat::from_rotation_z(-step),
    //     // Quat::from_rotation_x(scale * 10.0),
    //     // Quat::from_rotation_x(-scale * 10.0),
    //     // Quat::from_rotation_y(scale * 10.0),
    //     // Quat::from_rotation_y(-scale * 10.0),
    //     // Quat::from_rotation_z(scale * 10.0),
    //     // Quat::from_rotation_z(-scale * 10.0),
    //     // Quat::from_rotation_x(scale / 10.0),
    //     // Quat::from_rotation_x(-scale / 10.0),
    //     // Quat::from_rotation_y(scale / 10.0),
    //     // Quat::from_rotation_y(-scale / 10.0),
    //     // Quat::from_rotation_z(scale / 10.0),
    //     // Quat::from_rotation_z(-scale / 10.0),
    //     // Quat::from_rotation_x(scale / 100.0),
    //     // Quat::from_rotation_x(-scale / 100.0),
    //     // Quat::from_rotation_y(scale / 100.0),
    //     // Quat::from_rotation_y(-scale / 100.0),
    //     // Quat::from_rotation_z(scale / 100.0),
    //     // Quat::from_rotation_z(-scale / 100.0),
    // ];
    //
    // let mut best = (0, f32::NEG_INFINITY, Vec3A::ZERO);
    //
    // for (idx, step) in steps.into_iter().enumerate() {
    //     let new_point = step * point;
    //
    //     let motor_config = MotorConfig::<X3dMotorId>::new(
    //         Motor {
    //             orientation: new_point,
    //             position: vec3a(WIDTH / 2.0, LENGTH / 2.0, HEIGHT / 2.0),
    //             direction: Direction::Clockwise,
    //         },
    //         vec3a(0.0, 0.0, 0.0),
    //     );
    //
    //     let result = physics(&motor_config, motor_data, true);
    //     let score = score(&result, settings);
    //
    //     if score > best.1 {
    //         best = (idx, score, new_point);
    //     }
    //
    //     // Only reachable on first iteration, used to cull search space
    //     if idx == 0 && score == f32::NEG_INFINITY {
    //         return (Vec3A::ZERO, true);
    //     }
    // }
    //
    // (best.2, best.0 == 0 || best.1 == f32::NEG_INFINITY)

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

    // let new_point = point - step * grad;
    let new_point = point + step * constrained_grad;
    let new_point = new_point.normalize();
    // println!("{point:.4?} -> {new_point:.4?}, {constrained_grad:.4?}");

    (new_point, constrained_grad.norm_squared() < 0.0001)
}
