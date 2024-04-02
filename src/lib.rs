pub mod heuristic;
pub mod optimize;

use ahash::HashMap;
use bevy::math::Vec3A;
use motor_math::{
    motor_preformance::MotorData,
    solve::{forward, reverse},
    x3d::X3dMotorId,
    MotorConfig, Movement,
};

pub const WIDTH: f32 = 0.19 * 2.0;
pub const LENGTH: f32 = 0.22 * 2.0;
pub const HEIGHT: f32 = 0.09 * 2.0;

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
pub enum PhysicsResult {
    Linear(f32),
    Torque(f32),
}

pub fn physics(
    motor_config: &MotorConfig<X3dMotorId>,
    motor_data: &MotorData,
    fast: bool,
) -> HashMap<PhysicsAxis, PhysicsResult> {
    // Why is axes the plural of axis
    let axes = [
        (
            PhysicsAxis::X,
            Movement {
                force: Vec3A::X,
                torque: Vec3A::ZERO,
            },
        ),
        (
            PhysicsAxis::Y,
            Movement {
                force: Vec3A::Y,
                torque: Vec3A::ZERO,
            },
        ),
        (
            PhysicsAxis::Z,
            Movement {
                force: Vec3A::Z,
                torque: Vec3A::ZERO,
            },
        ),
        (
            PhysicsAxis::XRot,
            Movement {
                force: Vec3A::ZERO,
                torque: Vec3A::X,
            },
        ),
        (
            PhysicsAxis::YRot,
            Movement {
                force: Vec3A::ZERO,
                torque: Vec3A::Y,
            },
        ),
        (
            PhysicsAxis::ZRot,
            Movement {
                force: Vec3A::ZERO,
                torque: Vec3A::Z,
            },
        ),
    ];

    let mut results = HashMap::default();

    for (axis, movement) in axes {
        let value = if fast {
            let mut forces = reverse::reverse_solve(movement, motor_config);

            let force_length = forces.values().map(|it| it * it).sum::<f32>().sqrt();
            let adjustment = if force_length > 0.01 {
                1.0 / force_length
            } else {
                0.0
            };
            forces.values_mut().for_each(|it| *it *= adjustment);

            let adjusted_movement = forward::forward_solve(motor_config, &forces);

            adjusted_movement.force.dot(movement.force).abs()
                + adjusted_movement.torque.dot(movement.torque).abs()
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

            if ratio > 100.0 {
                0.0
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
