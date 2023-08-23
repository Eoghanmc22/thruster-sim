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
        PhysicsAxis::X,
        PhysicsAxis::Y,
        PhysicsAxis::Z,
        PhysicsAxis::XRot,
        PhysicsAxis::YRot,
        PhysicsAxis::ZRot,
    ];

    let motor_conf_data = motor_config.compute_motors();
    let (forwards, backwards) =
        motor_code::calculate_relevant_thrust_limits(motor_preformance_data);

    let mut results = HashMap::default();

    for (axis_idx, axis) in axes.iter().enumerate() {
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
            PhysicsAxis::X => {
                let value = physics_linear(DVec3::X, forces);

                results.insert(*axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::Y => {
                let value = physics_linear(DVec3::Y, forces);

                results.insert(*axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::Z => {
                let value = physics_linear(DVec3::Z, forces);

                results.insert(*axis, PhysicsResult::Linear(value));
            }
            PhysicsAxis::XRot | PhysicsAxis::YRot | PhysicsAxis::ZRot => {
                let value = physics_torque(*axis, &motor_conf_data, forces);

                results.insert(*axis, PhysicsResult::Torque(value));
            }
        }
    }

    results
}

fn physics_linear(reference_vec: DVec3, forces: HashMap<MotorId, DVec3>) -> f64 {
    let sum: DVec3 = forces.into_values().sum();

    // FIXME: Get rid of abs call
    sum.dot(reference_vec).abs()
}

fn physics_torque(
    axis: PhysicsAxis,
    motor_conf_data: &HashMap<MotorId, Motor>,
    forces: HashMap<MotorId, DVec3>,
) -> f64 {
    // TODO: Analyse the plane in the middle of the body perpendicular to the rotational axis
    // Consider oppsite corners of the rectangle created by the intersection of this plane and the
    // robot
    // torque = position cross force (right hand rule)

    // TODO generate?
    // Is this even right?
    let data = match axis {
        PhysicsAxis::XRot => [
            (
                [MotorId::FrontLeftTop, MotorId::FrontRightTop],
                [MotorId::BackLeftBottom, MotorId::BackRightBottom],
            ),
            (
                [MotorId::FrontLeftBottom, MotorId::FrontRightBottom],
                [MotorId::BackLeftTop, MotorId::BackRightTop],
            ),
        ],
        PhysicsAxis::YRot => [
            (
                [MotorId::FrontRightTop, MotorId::BackRightTop],
                [MotorId::FrontLeftBottom, MotorId::BackLeftBottom],
            ),
            (
                [MotorId::FrontLeftTop, MotorId::BackLeftTop],
                [MotorId::FrontRightBottom, MotorId::BackRightBottom],
            ),
        ],
        PhysicsAxis::ZRot => [
            (
                [MotorId::FrontRightTop, MotorId::FrontRightBottom],
                [MotorId::BackLeftTop, MotorId::BackLeftBottom],
            ),
            (
                [MotorId::FrontLeftTop, MotorId::FrontLeftBottom],
                [MotorId::BackRightTop, MotorId::BackRightBottom],
            ),
        ],
        _ => unreachable!(),
    };

    let mut total_torque = DVec3::default();

    for (corner_a, _corner_b) in data {
        let force_sum_a: DVec3 = corner_a
            .into_iter()
            .map(|motor| forces.get(&motor).unwrap())
            .sum();
        let corner_a_pos: DVec3 = corner_a
            .into_iter()
            .map(|motor| motor_conf_data.get(&motor).unwrap().position)
            .sum();

        // We dont care about corner_b
        // let force_sum_b: DVec3 = corner_b
        //     .into_iter()
        //     .map(|motor| forces.get(&motor).unwrap())
        //     .sum();
        // let corner_b_pos: DVec3 = corner_b
        //     .into_iter()
        //     .map(|motor| motor_conf_data.get(&motor).unwrap().position)
        //     .sum();

        let force = force_sum_a * 2.0;
        let torque = corner_a_pos.cross(force);

        total_torque += torque;
    }

    total_torque.length()
}
