use bevy::math::{vec3a, Quat, Vec3A};
use motor_math::{x3d::X3dMotorId, Direction, Motor, MotorConfig};

use crate::{
    heuristic::{score, ScoreSettings},
    physics, HEIGHT, LENGTH, WIDTH,
};

pub fn fibonacci_sphere(samples: usize) -> Vec<Vec3A> {
    let mut points = Vec::new();
    let phi = std::f32::consts::PI * (5f32.sqrt() - 1.0); // golden angle in radians

    for i in 0..samples {
        let y = 1.0 - (i as f32 / (samples as f32 - 1.0)) * 2.0; // y goes from 1 to -1
        let radius = (1.0 - y * y).sqrt(); // radius at y

        let theta = phi * i as f32; // golden angle increment

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;

        points.push(vec3a(x, y, z));
    }

    points
}

pub fn accent_sphere(scale: f32, point: Vec3A, settings: &ScoreSettings) -> (Vec3A, bool) {
    let steps = [
        Quat::IDENTITY,
        Quat::from_rotation_x(scale),
        Quat::from_rotation_x(-scale),
        Quat::from_rotation_y(scale),
        Quat::from_rotation_y(-scale),
        Quat::from_rotation_z(scale),
        Quat::from_rotation_z(-scale),
        // Quat::from_rotation_x(scale * 10.0),
        // Quat::from_rotation_x(-scale * 10.0),
        // Quat::from_rotation_y(scale * 10.0),
        // Quat::from_rotation_y(-scale * 10.0),
        // Quat::from_rotation_z(scale * 10.0),
        // Quat::from_rotation_z(-scale * 10.0),
        // Quat::from_rotation_x(scale / 10.0),
        // Quat::from_rotation_x(-scale / 10.0),
        // Quat::from_rotation_y(scale / 10.0),
        // Quat::from_rotation_y(-scale / 10.0),
        // Quat::from_rotation_z(scale / 10.0),
        // Quat::from_rotation_z(-scale / 10.0),
        // Quat::from_rotation_x(scale / 100.0),
        // Quat::from_rotation_x(-scale / 100.0),
        // Quat::from_rotation_y(scale / 100.0),
        // Quat::from_rotation_y(-scale / 100.0),
        // Quat::from_rotation_z(scale / 100.0),
        // Quat::from_rotation_z(-scale / 100.0),
    ];

    let mut best = (0, f32::NEG_INFINITY, Vec3A::ZERO);

    for (idx, step) in steps.into_iter().enumerate() {
        let new_point = step * point;

        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            orientation: new_point,
            position: vec3a(WIDTH / 2.0, LENGTH / 2.0, HEIGHT / 2.0),
            direction: Direction::Clockwise,
        });

        let result = physics(&motor_config);
        let score = score(&result, settings);

        if score > best.1 {
            best = (idx, score, new_point);
        }

        // Only reachable on first iteration, used to cull search space
        if idx == 0 && score == f32::NEG_INFINITY {
            return (Vec3A::ZERO, true);
        }
    }

    (best.2, best.0 == 0 || best.1 == f32::NEG_INFINITY)
}
