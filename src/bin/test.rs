use std::fmt::Debug;
use std::hash::Hash;
use std::{collections::HashMap, hash::BuildHasher};

use bevy::gizmos::config;
use motor_math::FloatType;
use motor_math::{
    motor_preformance::{self, Interpolation, MotorData, MotorRecord},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement, Number,
};
use nalgebra::{vector, ArrayStorage, Const, Matrix, Vector, Vector2, Vector3};
use num_dual::gradient;
use stable_hashmap::StableState;

const STEP: FloatType = 0.1;

fn main() {
    let motor_data = motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

    let point = vector![0.3, -0.5, -0.2];
    // let point = vector![3.5, 2.3];

    let (old_score, grad) = gradient(|point| func(point, &motor_data), point);

    let new_point = point + STEP * grad;
    // let new_point = new_point.normalize();

    let new_score = func(new_point, &motor_data);

    let secant = (new_score - old_score) / (new_point - point).norm();
    let grad_mag = grad.norm();
    let derv_diff = (secant - grad_mag).abs();
    let diff = new_point - point;
    let p_grad = old_score + grad_mag * STEP;

    println!("New: {new_score} p_grad: {p_grad}, old: {old_score}, secant: {secant}, gradient: {grad_mag}, derv diff: {derv_diff}, point diff: {diff}, grad vec: {grad}");
}

fn func<D: Number>(point: Vector3<D>, motor_data: &MotorData) -> D {
    let mut motor = Motor {
        // orientation: point,
        orientation: vector![0.2.into(), 0.4.into(), 0.1.into()].normalize(),
        position: point,
        // position: vector![(0.2 / 2.0).into(), (0.3 / 2.0).into(), (0.4 / 2.0).into()],
        direction: Direction::Clockwise,
    };
    // motor.orientation.normalize_mut();

    let motor_config =
        MotorConfig::<X3dMotorId, _>::new(motor, vector![0.0.into(), 0.0.into(), 0.0.into()]);

    // println!("config: {motor_config:#?}");
    println!("goob inverse: {:?}", motor_config.pseudo_inverse);
    // println!(
    //     "Normal inverse: {:?}",
    //     motor_config.matrix.rows(0, 6).try_inverse().unwrap()
    // );

    println!();

    let forces = reverse::reverse_solve(
        Movement {
            // force: vector![D::from(0.0), D::from(0.0), D::from(0.0),],
            force: vector![D::from(0.5), D::from(0.2), D::from(0.35),],
            // torque: vector![D::from(0.7), D::from(0.9), D::from(0.1),],
            torque: vector![D::from(0.0), D::from(0.0), D::from(0.0),],
        },
        &motor_config,
    );

    println!("forces: {forces:?}");
    println!();

    let motor_cmds = reverse::forces_to_cmds(forces, &motor_config, motor_data);

    println!("cmds: {motor_cmds:?}");

    reverse::binary_search_force_ratio(&motor_cmds, &motor_config, motor_data, 25.0, 0.0001)
}

// pub fn binary_search_force_ratio<D: Number, MotorId: Hash + Ord + Clone + Debug>(
//     motor_cmds: &HashMap<MotorId, MotorRecord<D>, impl BuildHasher>,
//     motor_config: &MotorConfig<MotorId, D>,
//     motor_data: &MotorData,
//     amperage_cap: f32,
//     epsilon: f32,
// ) -> D {
//     let (mut lower_bound, mut lower_current) = (D::zero(), D::zero());
//     let (mut upper_bound, mut upper_current) = (D::from(f32::INFINITY), D::from(f32::INFINITY));
//     let mut mid = D::one();
//
//     loop {
//         let mid_current = motor_cmds
//             .iter()
//             .map(|(motor_id, data)| {
//                 let direction = motor_config
//                     .motor(motor_id)
//                     .map(|it| it.direction)
//                     .unwrap_or(crate::Direction::Clockwise);
//
//                 // FIXME: old code of copying force's sign to its self is a no-op could be a bug
//                 // let adjusted_force = data.force.copysign(data.force) * mid;
//                 let adjusted_force = data.force * mid;
//                 let data = motor_data
//                     // .lookup_by_force(adjusted_force, Interpolation::LerpDirection(direction));
//                     .lookup_by_force(adjusted_force /*.abs()*/, Interpolation::Lerp);
//
//                 data.current.abs()
//             })
//             .sum::<D>();
//
//         if mid_current.re() == 0.0 {
//             return D::one();
//         }
//         if (mid_current.re() - amperage_cap).abs() < epsilon {
//             return mid;
//         }
//
//         if mid_current.re() >= amperage_cap {
//             upper_bound = mid;
//             upper_current = mid_current;
//         } else {
//             lower_bound = mid;
//             lower_current = mid_current;
//         }
//
//         if upper_bound.re() == f32::INFINITY {
//             mid *= D::from(amperage_cap) / mid_current;
//             // mid *= 2.0;
//         } else {
//             let alpha = (D::from(amperage_cap) - lower_current) / (upper_current - lower_current);
//             mid = upper_bound * alpha + lower_bound * (D::one() - alpha)
//             // mid = upper_bound / 2.0 + lower_bound / 2.0
//         }
//     }
// }

// fn simple<D: Number>(point: Vector2<D>, motor_data: &MotorData) -> D {
//     let record_1 = motor_data.lookup_by_force(point.x, Interpolation::Lerp);
//     let record_2 = motor_data.lookup_by_force(point.y, Interpolation::Lerp);
//
//     binary_search_force_ratio_simple(&[record_1, record_2], motor_data, 1.0, 0.0001)
// }
//
// pub fn binary_search_force_ratio_simple<D: Number>(
//     motor_cmds: &[MotorRecord<D>],
//     motor_data: &MotorData,
//     amperage_cap: f32,
//     epsilon: f32,
// ) -> D {
//     let (mut lower_bound, mut lower_current) = (D::zero(), D::zero());
//     let (mut upper_bound, mut upper_current) = (D::from(f32::INFINITY), D::from(f32::INFINITY));
//     let mut mid = D::one();
//
//     loop {
//         let mid_current = motor_cmds
//             .iter()
//             .map(|data| {
//                 let adjusted_force = data.force * mid;
//                 let data = motor_data.lookup_by_force(adjusted_force, Interpolation::Lerp);
//
//                 data.current
//             })
//             .sum::<D>();
//
//         if mid_current.re() == 0.0 {
//             return D::one();
//         }
//         if (mid_current.re() - amperage_cap).abs() < epsilon {
//             return mid;
//         }
//
//         if mid_current.re() >= amperage_cap {
//             upper_bound = mid;
//             upper_current = mid_current;
//         } else {
//             lower_bound = mid;
//             lower_current = mid_current;
//         }
//
//         if upper_bound.re() == f32::INFINITY {
//             mid *= D::from(amperage_cap) / mid_current;
//             // mid *= 2.0;
//         } else {
//             let alpha = (D::from(amperage_cap) - lower_current) / (upper_current - lower_current);
//             mid = upper_bound * alpha + lower_bound * (D::one() - alpha)
//             // mid = upper_bound / 2.0 + lower_bound / 2.0
//         }
//     }
// }
