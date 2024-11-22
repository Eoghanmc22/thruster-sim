pub mod heuristic;
pub mod optimize;

use std::{collections::HashMap, hash::BuildHasherDefault};

use motor_math::{
    motor_preformance::MotorData,
    solve::{forward, reverse},
    x3d::X3dMotorId,
    FloatType, MotorConfig, Movement, Number,
};
use nalgebra::vector;
use stable_hashmap::StableHashMap;

pub const WIDTH: FloatType = 0.19 * 2.0;
pub const LENGTH: FloatType = 0.22 * 2.0;
pub const HEIGHT: FloatType = 0.09 * 2.0;
// pub const WIDTH: f32 = 0.2 * 2.0;
// pub const LENGTH: f32 = 0.2 * 2.0;
// pub const HEIGHT: f32 = 0.2 * 2.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum PhysicsAxis {
    X,
    Y,
    Z,
    XRot,
    YRot,
    ZRot,
}

#[derive(Clone, Copy, Debug)]
pub enum PhysicsResult<D> {
    Linear(D),
    Torque(D),
}

pub fn physics<D: Number>(
    motor_config: &MotorConfig<X3dMotorId, D>,
    motor_data: &MotorData,
    fast: bool,
) -> StableHashMap<PhysicsAxis, PhysicsResult<D>> {
    // Why is axes the plural of axis
    let axes = [
        (
            PhysicsAxis::X,
            Movement {
                force: vector![D::one(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
        ),
        (
            PhysicsAxis::Y,
            Movement {
                force: vector![D::zero(), D::one(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
        ),
        (
            PhysicsAxis::Z,
            Movement {
                force: vector![D::zero(), D::zero(), D::one()],
                torque: vector![D::zero(), D::zero(), D::zero()],
            },
        ),
        (
            PhysicsAxis::XRot,
            Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::one(), D::zero(), D::zero()],
            },
        ),
        (
            PhysicsAxis::YRot,
            Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::one(), D::zero()],
            },
        ),
        (
            PhysicsAxis::ZRot,
            Movement {
                force: vector![D::zero(), D::zero(), D::zero()],
                torque: vector![D::zero(), D::zero(), D::one()],
            },
        ),
    ];

    let mut results = HashMap::default();

    for (axis, movement) in axes {
        let value = if fast {
            let mut forces = reverse::reverse_solve(movement, motor_config);

            let force_length_2 = forces.values().map(|it| *it * *it).sum::<D>().sqrt();
            let adjustment = if force_length_2.re() > 0.01 {
                D::one() / force_length_2
            } else {
                D::zero()
            };
            forces.values_mut().for_each(|it| *it *= adjustment);

            let adjusted_movement = forward::forward_solve(motor_config, &forces);

            adjusted_movement.force.dot(&movement.force).abs()
                + adjusted_movement.torque.dot(&movement.torque).abs()
        } else {
            let forces = reverse::reverse_solve(movement, motor_config);

            let motor_cmds = reverse::forces_to_cmds(forces, motor_config, motor_data);
            let ratio = reverse::binary_search_force_ratio(
                &motor_cmds,
                motor_config,
                motor_data,
                25.0,
                0.05,
            );

            if ratio.re() > 100.0 {
                D::zero()
            } else {
                ratio
            }
        };

        match axis {
            PhysicsAxis::X | PhysicsAxis::Y | PhysicsAxis::Z => {
                results.insert(axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::XRot | PhysicsAxis::YRot | PhysicsAxis::ZRot => {
                results.insert(axis, PhysicsResult::Torque(value));
            }
        }
    }

    results
}
