pub mod heuristic;
pub mod optimize;

use ahash::HashMap;
use bevy::math::Vec3A;
use motor_math::{
    solve::{forward, reverse},
    x3d::X3dMotorId,
    MotorConfig, Movement,
};

pub const WIDTH: f32 = 0.325 * 2.0;
pub const LENGTH: f32 = 0.355 * 2.0;
pub const HEIGHT: f32 = 0.241 * 2.0;

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

pub fn physics(motor_config: &MotorConfig<X3dMotorId>) -> HashMap<PhysicsAxis, PhysicsResult> {
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
        let mut forces = reverse::reverse_solve(movement, motor_config);

        let force_length = forces.values().map(|it| it * it).sum::<f32>();
        let adjustment = 1.0 / force_length.sqrt();
        forces.values_mut().for_each(|it| *it *= adjustment);

        let adjusted_movement = forward::forward_solve(motor_config, &forces);

        match axis {
            PhysicsAxis::X | PhysicsAxis::Y | PhysicsAxis::Z => {
                let value = adjusted_movement.force.dot(movement.force).abs();
                results.insert(axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::XRot | PhysicsAxis::YRot | PhysicsAxis::ZRot => {
                let value = adjusted_movement.torque.dot(movement.torque).abs();
                results.insert(axis, PhysicsResult::Torque(value));
            }
        }
    }

    results
}
