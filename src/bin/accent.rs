use std::collections::BTreeMap;

use bevy::math::{vec3a, Vec3A};
use itertools::Itertools;
use motor_math::{x3d::X3dMotorId, Direction, Motor, MotorConfig};
use random_math_test::{
    heuristic::score,
    optimize::{accent_sphere, fibonacci_sphere},
    physics, HEIGHT, LENGTH, WIDTH,
};

const STEP: f32 = 0.001;
// const STEP: f64 = 0.001;
const SIMILARITY: f32 = 0.05;

fn main() {
    let mut points = fibonacci_sphere(1000);
    let mut solved_points = Vec::new();
    let mut counter = 0;

    while !points.is_empty() {
        points.retain_mut(|point| {
            let (new_point, solved) = accent_sphere(STEP, *point, &Default::default());

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
                score(
                    &physics(&MotorConfig::<X3dMotorId>::new(Motor {
                        orientation: point,
                        position: vec3a(WIDTH / 2.0, LENGTH / 2.0, HEIGHT / 2.0),
                        direction: Direction::Clockwise,
                    })),
                    &Default::default(),
                ),
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
