use bevy::math::{dvec3, DVec3};
use fxhash::FxHashMap as HashMap;
use motor_code::{MotorData, MotorId};

pub mod lines;
pub mod motor_code;

// +X: Right, +Y: Forwards, +Z: Up
// +XR: Pitch Up, +YR: Roll Clockwise, +ZR: Yaw Clockwise (top view)

/// Models a possible varient of the symmetrical 3d X motor config
#[derive(Clone, Copy, Debug)]
pub struct MotorConfig {
    pub seed: SeedAngle,

    /// Meters
    pub width: f64,
    /// Meters
    pub length: f64,
    /// Meters
    pub height: f64,
}

#[derive(Clone, Copy, Debug)]
pub enum SeedAngle {
    VecByTwoAngles {
        /// FrontLeftTop
        /// Radians
        angle_xy: f64,
        /// FrontLeftTop
        /// Radians
        angle_yz: f64,
    },
    Vec(DVec3),
}

#[derive(Clone, Copy, Debug)]
pub struct Motor {
    /// Offset from origin
    pub position: DVec3,
    /// Unit vector
    pub orientation: DVec3,
}

impl MotorConfig {
    pub fn compute_motors(&self) -> HashMap<MotorId, Motor> {
        let MotorConfig {
            seed,

            width,
            length,
            height,
        } = *self;

        // Half dimensions
        let h_w = width / 2.0;
        let h_l = length / 2.0;
        let h_h = height / 2.0;

        #[rustfmt::skip]
        let motors = [
            (MotorId::FrontRightTop, [].as_slice()),

            (MotorId::FrontRightBottom, [VectorTransform::ReflectXY].as_slice()),
            (MotorId::FrontLeftTop, [VectorTransform::ReflectYZ].as_slice()),
            (MotorId::BackRightTop, [VectorTransform::ReflectXZ].as_slice()),

            (MotorId::FrontLeftBottom, [VectorTransform::ReflectXY, VectorTransform::ReflectYZ].as_slice()),
            (MotorId::BackLeftTop, [VectorTransform::ReflectYZ, VectorTransform::ReflectXZ].as_slice()),
            (MotorId::BackRightBottom, [VectorTransform::ReflectXZ, VectorTransform::ReflectXY].as_slice()),

            (MotorId::BackLeftBottom, [VectorTransform::ReflectXY, VectorTransform::ReflectYZ, VectorTransform::ReflectXZ].as_slice()),
        ];

        let frt_position = dvec3(h_w, h_l, h_h);
        let frt_orientation = match seed {
            SeedAngle::VecByTwoAngles { angle_xy, angle_yz } => vec_from_angles(angle_xy, angle_yz),
            SeedAngle::Vec(vec) => vec,
        };

        motors
            .into_iter()
            .map(|(motor, transforms)| {
                let (position, orientation) = transforms.into_iter().fold(
                    (frt_position, frt_orientation),
                    |(position, orientation), transform| {
                        (
                            transform.transform(position),
                            transform.transform(orientation),
                        )
                    },
                );

                (
                    motor,
                    Motor {
                        position,
                        orientation,
                    },
                )
            })
            .collect()
    }
}

#[derive(Clone, Copy, Debug)]
pub enum VectorTransform {
    ReflectXY,
    ReflectYZ,
    ReflectXZ,
}

impl VectorTransform {
    pub fn transform(&self, vec: DVec3) -> DVec3 {
        let DVec3 { x, y, z } = vec;

        match self {
            VectorTransform::ReflectXY => dvec3(x, y, -z),
            VectorTransform::ReflectYZ => dvec3(-x, y, z),
            VectorTransform::ReflectXZ => dvec3(x, -y, z),
        }
    }
}

// https://stackoverflow.com/questions/30011741/3d-vector-defined-by-2-angles
pub fn vec_from_angles(angle_xy: f64, angle_yz: f64) -> DVec3 {
    let x = angle_xy.cos() * angle_yz.cos();
    let y = angle_xy.sin() * angle_yz.cos();
    let z = angle_yz.sin();

    dvec3(x, y, z)
}

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
    Linear(f64),
    Torque(f64),
}

pub fn physics(
    motor_config: &MotorConfig,
    motor_preformance_data: &MotorData,
) -> HashMap<PhysicsAxis, PhysicsResult> {
    // TODO: Derive from motor config
    // Laterial is signum(orientation dot axis)
    // Torque is signum((position cross orientation) dot axis)
    // signum may not be correct for arbituary motor configs
    let motor_relations = &[
        (
            MotorId::FrontLeftBottom,
            [-1.0f64, -1.0, 1.0, 1.0, 1.0, -1.0].as_slice(),
        ),
        (
            MotorId::FrontLeftTop,
            [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0].as_slice(),
        ),
        (
            MotorId::FrontRightBottom,
            [1.0, -1.0, 1.0, 1.0, -1.0, 1.0].as_slice(),
        ),
        (
            MotorId::FrontRightTop,
            [1.0, -1.0, -1.0, -1.0, 1.0, 1.0].as_slice(),
        ),
        (
            MotorId::BackLeftBottom,
            [-1.0, 1.0, 1.0, -1.0, 1.0, 1.0].as_slice(),
        ),
        (
            MotorId::BackLeftTop,
            [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0].as_slice(),
        ),
        (
            MotorId::BackRightBottom,
            [1.0, 1.0, 1.0, -1.0, -1.0, -1.0].as_slice(),
        ),
        (
            MotorId::BackRightTop,
            [1.0, 1.0, -1.0, 1.0, 1.0, -1.0].as_slice(),
        ),
    ];

    // Why is axes the plural of axis
    let axes = &[
        (PhysicsAxis::X, DVec3::X),
        (PhysicsAxis::Y, DVec3::Y),
        (PhysicsAxis::Z, DVec3::Z),
        (PhysicsAxis::XRot, DVec3::X),
        (PhysicsAxis::YRot, DVec3::Y),
        (PhysicsAxis::ZRot, DVec3::Z),
    ];

    let motor_conf_data = motor_config.compute_motors();
    // TODO: Could just assume unit force
    let (forwards, backwards) = motor_code::calculate_thrust_limits(motor_preformance_data);

    let mut results = HashMap::default();

    for (axis_idx, (axis, vector)) in axes.iter().enumerate() {
        let mut forces = HashMap::default();

        for (motor_id, relations) in motor_relations {
            let motor = motor_conf_data.get(motor_id).unwrap();

            let thrust = if relations[axis_idx].is_sign_positive() {
                forwards
            } else {
                backwards
            };

            let force = motor.orientation * thrust;
            forces.insert(*motor_id, force);
        }

        match axis {
            PhysicsAxis::X | PhysicsAxis::Y | PhysicsAxis::Z => {
                let linear = physics_linear(forces);
                // FIXME: abs call isnt needed once motor relations is calculated procedurally
                let value = linear.dot(*vector).abs();

                results.insert(*axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::XRot | PhysicsAxis::YRot | PhysicsAxis::ZRot => {
                let torque = physics_torque(&motor_conf_data, forces);
                // FIXME: abs call isnt needed once motor relations is calculated procedurally
                let value = torque.dot(*vector).abs();

                results.insert(*axis, PhysicsResult::Torque(value));
            }
        }
    }

    results
}

fn physics_linear(forces: HashMap<MotorId, DVec3>) -> DVec3 {
    let mut force_sum = DVec3::ZERO;

    for (motor, force) in forces {
        force_sum += force;
    }

    force_sum
}

fn physics_torque(
    motor_conf_data: &HashMap<MotorId, Motor>,
    forces: HashMap<MotorId, DVec3>,
) -> DVec3 {
    let mut torque_sum = DVec3::ZERO;

    for (motor, force) in forces {
        let position = motor_conf_data.get(&motor).unwrap().position;

        torque_sum += position.cross(force);
    }

    torque_sum
}
