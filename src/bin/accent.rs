use std::collections::BTreeMap;

use bevy::math::{dvec3, DQuat, DVec3};
use fxhash::FxHashMap as HashMap;
use itertools::Itertools;
use random_math_test::{
    motor_code::{self, MotorData},
    physics, MotorConfig, PhysicsAxis, PhysicsResult, SeedAngle,
};

const STEP: f64 = 0.001;
// const STEP: f64 = 0.001;
const SIMILARITY: f64 = 0.05;
const WIDTH: f64 = 0.325;
const LENGTH: f64 = 0.355;
const HEIGHT: f64 = 0.241;

fn main() {
    let motor_data = motor_code::read_motor_data().unwrap();

    let mut points = fibonacci_sphere(10000);
    let mut solved_points = Vec::new();
    let mut counter = 0;

    while !points.is_empty() {
        points.retain_mut(|point| {
            let (new_point, solved) = accent_sphere(STEP, *point, &motor_data);

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
                score(&physics(
                    &MotorConfig {
                        seed: SeedAngle::Vec(point),
                        width: WIDTH,
                        length: LENGTH,
                        height: HEIGHT,
                    },
                    &motor_data,
                )),
            )
        })
        .filter(|(_, score)| *score != f64::NEG_INFINITY)
        .collect_vec();

    scored_points.sort_by(|(_, score_a), (_, score_b)| f64::total_cmp(score_a, score_b).reverse());

    let mut deduped_points: Vec<(DVec3, f64)> = Vec::new();
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
        let motor_config = MotorConfig {
            seed: SeedAngle::Vec(point.0),
            width: WIDTH,
            length: LENGTH,
            height: HEIGHT,
        };

        let result = physics(&motor_config, &motor_data);
        let result: BTreeMap<_, _> = result.into_iter().collect();

        println!("{point:+.3?}, {result:+.3?}");
    }
}

fn fibonacci_sphere(samples: usize) -> Vec<DVec3> {
    let mut points = Vec::new();
    let phi = std::f64::consts::PI * (5f64.sqrt() - 1.0); // golden angle in radians

    for i in 0..samples {
        let y = 1.0 - (i as f64 / (samples as f64 - 1.0)) * 2.0; // y goes from 1 to -1
        let radius = (1.0 - y * y).sqrt(); // radius at y

        let theta = phi * i as f64; // golden angle increment

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;

        points.push(dvec3(x, y, z));
    }

    points
}

fn accent_sphere(scale: f64, point: DVec3, motor_data: &MotorData) -> (DVec3, bool) {
    let steps = [
        DQuat::IDENTITY,
        DQuat::from_rotation_x(scale),
        DQuat::from_rotation_x(-scale),
        DQuat::from_rotation_y(scale),
        DQuat::from_rotation_y(-scale),
        DQuat::from_rotation_z(scale),
        DQuat::from_rotation_z(-scale),
        DQuat::from_rotation_x(scale * 10.0),
        DQuat::from_rotation_x(-scale * 10.0),
        DQuat::from_rotation_y(scale * 10.0),
        DQuat::from_rotation_y(-scale * 10.0),
        DQuat::from_rotation_z(scale * 10.0),
        DQuat::from_rotation_z(-scale * 10.0),
        DQuat::from_rotation_x(scale / 10.0),
        DQuat::from_rotation_x(-scale / 10.0),
        DQuat::from_rotation_y(scale / 10.0),
        DQuat::from_rotation_y(-scale / 10.0),
        DQuat::from_rotation_z(scale / 10.0),
        DQuat::from_rotation_z(-scale / 10.0),
        DQuat::from_rotation_x(scale / 100.0),
        DQuat::from_rotation_x(-scale / 100.0),
        DQuat::from_rotation_y(scale / 100.0),
        DQuat::from_rotation_y(-scale / 100.0),
        DQuat::from_rotation_z(scale / 100.0),
        DQuat::from_rotation_z(-scale / 100.0),
    ];

    let mut best = (0, f64::NEG_INFINITY, DVec3::ZERO);

    for (idx, step) in steps.into_iter().enumerate() {
        let new_point = step * point;

        let motor_config = MotorConfig {
            seed: SeedAngle::Vec(new_point),
            width: WIDTH,
            length: LENGTH,
            height: HEIGHT,
        };

        let result = physics(&motor_config, motor_data);
        let score = score(&result);

        if score > best.1 {
            best = (idx, score, new_point);
        }

        if idx == 0 && score == f64::NEG_INFINITY {
            return (DVec3::ZERO, true);
        }
    }

    (best.2, best.0 == 0 || best.1 == f64::NEG_INFINITY)
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

fn score(result: &HashMap<PhysicsAxis, PhysicsResult>) -> f64 {
    let mut avg_linear = 0.0;
    let mut avg_torque = 0.0;
    let mut min_linear = f64::INFINITY;
    let mut min_torque = f64::INFINITY;

    for result in result.values() {
        match result {
            PhysicsResult::Linear(val) => {
                avg_linear += val / 3.0;
                min_linear = min_linear.min(*val);
            }
            PhysicsResult::Torque(val) => {
                avg_torque += val / 3.0;
                min_torque = min_torque.min(*val);
            }
        }
    }

    let mut score_linear = 0.0;
    let mut score_torque = 0.0;

    for result in result.values() {
        match result {
            PhysicsResult::Linear(val) => score_linear += (val - avg_linear) * (val - avg_linear),
            PhysicsResult::Torque(val) => score_torque += (val - avg_torque) * (val - avg_torque),
        }
    }

    let torque_sucks = {
        if min_torque < 0.5 {
            f64::NEG_INFINITY
        } else {
            0.0
        }
    };

    let linear_sucks = {
        if min_linear < 2.0 {
            f64::NEG_INFINITY
        } else {
            0.0
        }
    };

    let results: HashMap<PhysicsAxis, f64> = result
        .into_iter()
        .map(|(axis, result)| match result {
            PhysicsResult::Linear(value) | PhysicsResult::Torque(value) => (*axis, value.abs()),
        })
        .collect();

    // -0.2 * (score_linear + score_torque).sqrt()
    // results[&PhysicsAxis::Y] * 6.0
    //     + results[&PhysicsAxis::Z] * 12.0
    //     + results[&PhysicsAxis::X] * -1.0
    //     + results[&PhysicsAxis::XRot] * 4.0
    //     + results[&PhysicsAxis::YRot] * 4.0
    //     + results[&PhysicsAxis::ZRot] * -1.0
    0.0 + linear_sucks
        + torque_sucks
        // + min_linear * 1.0
        // + min_torque * 1.0
        + avg_linear * 1.0
        // + avg_torque * 1.0
        + results[&PhysicsAxis::XRot] * 2.0
        + results[&PhysicsAxis::YRot] * 2.0
        - 30.0 * (results[&PhysicsAxis::X] - results[&PhysicsAxis::Y]).max(0.0)
        - 30.0 * (results[&PhysicsAxis::Y] - results[&PhysicsAxis::Z]).max(0.0)
        + 30.0 * (results[&PhysicsAxis::Y] - 4.5)
    // + 30.0 * (results[&PhysicsAxis::Z] - 7.0)
    // + 30.0 * (results[&PhysicsAxis::XRot] - 4.0)
    // + 30.0 * (results[&PhysicsAxis::YRot] - 2.0)
}
