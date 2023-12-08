use std::collections::BTreeMap;

use bevy::math::{vec3a, Quat, Vec3A};
use itertools::Itertools;
use motor_math::{x3d::X3dMotorId, Direction, Motor, MotorConfig};
use random_math_test::{heuristic::score, physics};

const STEP: f32 = 0.001;
// const STEP: f64 = 0.001;
const SIMILARITY: f32 = 0.05;
const WIDTH: f32 = 0.325;
const LENGTH: f32 = 0.355;
const HEIGHT: f32 = 0.241;

fn main() {
    let mut points = fibonacci_sphere(1000);
    let mut solved_points = Vec::new();
    let mut counter = 0;

    while !points.is_empty() {
        points.retain_mut(|point| {
            let (new_point, solved) = accent_sphere(STEP, *point);

            if solved {
                solved_points.push(new_point);
                false
            } else {
                *point = new_point;
                true
            }
        });

        if counter % 100 == 0 {
            dbg!(counter);
        }
        counter += 1;
    }

    let mut scored_points = solved_points
        .into_iter()
        .map(|point| {
            (
                point,
                score(&physics(&MotorConfig::<X3dMotorId>::new(Motor {
                    orientation: point,
                    position: vec3a(WIDTH / 2.0, LENGTH / 2.0, HEIGHT / 2.0),
                    direction: Direction::Clockwise,
                }))),
            )
        })
        .filter(|(_, score)| *score != f32::NEG_INFINITY)
        .collect_vec();

    scored_points.sort_by(|(_, score_a), (_, score_b)| f32::total_cmp(score_a, score_b).reverse());

    let mut deduped_points: Vec<(Vec3A, f32)> = Vec::new();
    'outer: for (new_point, score) in scored_points {
        for (existing_point, _score) in &deduped_points {
            let delta = (new_point - *existing_point).length();
            if delta < SIMILARITY {
                continue 'outer;
            }

            let delta = (-new_point - *existing_point).length();
            if delta < SIMILARITY {
                continue 'outer;
            }
        }

        deduped_points.push((new_point, score));
    }

    for point in deduped_points.into_iter().take(5) {
        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            orientation: point.0,
            position: vec3a(WIDTH / 2.0, LENGTH / 2.0, HEIGHT / 2.0),
            direction: Direction::Clockwise,
        });

        let result = physics(&motor_config);
        let result: BTreeMap<_, _> = result.into_iter().collect();

        println!("{point:+.3?}, {result:+.3?}");
    }
}

fn fibonacci_sphere(samples: usize) -> Vec<Vec3A> {
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

fn accent_sphere(scale: f32, point: Vec3A) -> (Vec3A, bool) {
    let steps = [
        Quat::IDENTITY,
        Quat::from_rotation_x(scale),
        Quat::from_rotation_x(-scale),
        Quat::from_rotation_y(scale),
        Quat::from_rotation_y(-scale),
        Quat::from_rotation_z(scale),
        Quat::from_rotation_z(-scale),
        Quat::from_rotation_x(scale * 10.0),
        Quat::from_rotation_x(-scale * 10.0),
        Quat::from_rotation_y(scale * 10.0),
        Quat::from_rotation_y(-scale * 10.0),
        Quat::from_rotation_z(scale * 10.0),
        Quat::from_rotation_z(-scale * 10.0),
        Quat::from_rotation_x(scale / 10.0),
        Quat::from_rotation_x(-scale / 10.0),
        Quat::from_rotation_y(scale / 10.0),
        Quat::from_rotation_y(-scale / 10.0),
        Quat::from_rotation_z(scale / 10.0),
        Quat::from_rotation_z(-scale / 10.0),
        Quat::from_rotation_x(scale / 100.0),
        Quat::from_rotation_x(-scale / 100.0),
        Quat::from_rotation_y(scale / 100.0),
        Quat::from_rotation_y(-scale / 100.0),
        Quat::from_rotation_z(scale / 100.0),
        Quat::from_rotation_z(-scale / 100.0),
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
        let score = score(&result);

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

// fn score(result: &HashMap<PhysicsAxis, PhysicsResult>) -> f64 {
//     let results: HashMap<PhysicsAxis, f64> = result
//         .into_iter()
//         .map(|(axis, result)| match result {
//             PhysicsResult::Linear(value) | PhysicsResult::Torque(value) => (*axis, value.abs()),
//         })
//         .collect();
//
//     let mut kinda_sucks = 0.0;
//
//     for (axis, force) in &results {
//         let min = match axis {
//             PhysicsAxis::X | PhysicsAxis::Y | PhysicsAxis::Z => 2.0,
//             PhysicsAxis::XRot | PhysicsAxis::YRot | PhysicsAxis::ZRot => 1.0,
//         };
//
//         if *force < min {
//             kinda_sucks -= f64::INFINITY;
//         }
//     }
//
//     // Importance constraints
//     results[&PhysicsAxis::Y] * 2.0
//         + results[&PhysicsAxis::Z] * 1.5
//         + results[&PhysicsAxis::X] * 0.75
//         + results[&PhysicsAxis::XRot] * 0.8
//         + results[&PhysicsAxis::YRot] * 0.5
//         + results[&PhysicsAxis::ZRot] * 0.75
//
//         // Similar constraints
//         // - 100.0 * (results[&PhysicsAxis::Y] * 1.5 - results[&PhysicsAxis::Z]).abs()
//         - 2.0 * (results[&PhysicsAxis::XRot] - results[&PhysicsAxis::YRot]).abs()
//
//         // Comparison constraints
//         - 30.0 * (results[&PhysicsAxis::X] - results[&PhysicsAxis::Y]).max(0.0)
//         - 30.0 * (results[&PhysicsAxis::Y] - results[&PhysicsAxis::Z]).max(0.0)
//         - 30.0 * (results[&PhysicsAxis::ZRot] - results[&PhysicsAxis::XRot]).max(0.0)
//         - 30.0 * (results[&PhysicsAxis::ZRot] - results[&PhysicsAxis::YRot]).max(0.0)
//
//         // Doesnt totally suck constraint
//         + kinda_sucks
// }
