use motor_math::{solve::reverse::Axis, FloatType, MotorConfig, Number};
use nalgebra::{vector, SVector};
use stable_hashmap::StableHashMap;
use std::fmt::Debug;
use std::hash::Hash;

#[derive(Clone)]
pub struct ScoreSettings {
    pub mes_linear: FloatType,
    pub mes_x_off: FloatType,
    pub mes_y_off: FloatType,
    pub mes_z_off: FloatType,

    pub mes_torque: FloatType,
    pub mes_x_rot_off: FloatType,
    pub mes_y_rot_off: FloatType,
    pub mes_z_rot_off: FloatType,

    pub avg_linear: FloatType,
    pub avg_torque: FloatType,

    pub min_linear: FloatType,
    pub min_torque: FloatType,

    pub x: FloatType,
    pub y: FloatType,
    pub z: FloatType,

    pub x_rot: FloatType,
    pub y_rot: FloatType,
    pub z_rot: FloatType,

    pub center_of_mass_loss: FloatType,
    pub center_loss: FloatType,
    pub surface_area_loss: FloatType,
    pub dimension_loss: FloatType,

    pub tube_exclusion_radius: FloatType,
    pub tube_exclusion_loss: FloatType,

    pub thruster_exclusion_radius: FloatType,
    pub thruster_exclusion_loss: FloatType,
}

impl Default for ScoreSettings {
    fn default() -> Self {
        Self {
            mes_linear: 0.0,
            mes_x_off: 0.0,
            mes_y_off: 0.0,
            mes_z_off: 0.0,
            mes_torque: 0.0,
            mes_x_rot_off: 0.0,
            mes_y_rot_off: 0.0,
            mes_z_rot_off: 0.0,
            avg_linear: 0.0,
            avg_torque: 0.9,
            min_linear: 0.0,
            min_torque: 7.0,
            x: 0.5,
            y: 1.5,
            z: 0.75,
            x_rot: 0.35,
            y_rot: 0.2,
            z_rot: 0.25,
            center_of_mass_loss: -100.0,
            center_loss: -50.0,
            surface_area_loss: 1e-5,
            dimension_loss: -10.0,
            tube_exclusion_radius: 0.08,
            thruster_exclusion_radius: 0.08,
            tube_exclusion_loss: -90.0,
            thruster_exclusion_loss: -90.0,
            // distance_loss: -3.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ScoreResult<D> {
    pub mes_linear: D,
    pub mes_torque: D,

    pub avg_linear: D,
    pub avg_torque: D,

    pub min_linear: D,
    pub min_torque: D,

    pub x: D,
    pub y: D,
    pub z: D,

    pub x_rot: D,
    pub y_rot: D,
    pub z_rot: D,

    pub center_of_mass_loss: D,
    pub center_loss: D,
    pub surface_area_loss: D,
    pub dimension_loss: D,
    pub tube_exclusion_loss: D,
    pub thruster_exclusion_loss: D,
}

pub fn score<MotorId: Debug + Ord + Hash + Clone, D: Number>(
    result: &StableHashMap<Axis, D>,
    motor_config: &MotorConfig<MotorId, D>,
    settings: &ScoreSettings,
) -> (D, ScoreResult<D>) {
    // Average and min
    let mut avg_linear = D::from(0.0);
    let mut avg_torque = D::from(0.0);
    let mut min_linear = D::from(FloatType::INFINITY);
    let mut min_torque = D::from(FloatType::INFINITY);

    for (axis, val) in result {
        match axis {
            Axis::X | Axis::Y | Axis::Z => {
                avg_linear += *val / 3.0;
                min_linear = min_linear.min(*val);
            }
            Axis::XRot | Axis::YRot | Axis::ZRot => {
                avg_torque += *val / 3.0;
                min_torque = min_torque.min(*val);
            }
        }
    }

    // Mean error squared
    let mut mes_linear = D::from(0.0);
    let mut mes_torque = D::from(0.0);

    for (axis, result) in result {
        let offset = match axis {
            Axis::X => settings.mes_x_off,
            Axis::Y => settings.mes_y_off,
            Axis::Z => settings.mes_z_off,
            Axis::XRot => settings.mes_x_rot_off,
            Axis::YRot => settings.mes_y_rot_off,
            Axis::ZRot => settings.mes_z_rot_off,
        };
        let offset = D::from(offset);

        let (val, avg) = match axis {
            Axis::X | Axis::Y | Axis::Z => (*result, avg_linear),
            Axis::XRot | Axis::YRot | Axis::ZRot => (*result, avg_torque),
        };

        let goal = if offset.re() > 0.0 {
            offset
        } else if offset.re() == 0.0 {
            avg
        } else {
            val
        };

        match axis {
            Axis::X | Axis::Y | Axis::Z => mes_linear += (val - goal) * (val - goal),
            Axis::XRot | Axis::YRot | Axis::ZRot => mes_torque += (val - goal) * (val - goal),
        }
    }

    let thruster_count = motor_config.motors().count();

    let mut position_sum = SVector::<D, 3>::zeros();
    let mut min: SVector<D, 3> = vector![
        D::from(-settings.tube_exclusion_radius),
        D::from(-0.17),
        D::from(-settings.tube_exclusion_radius)
    ];
    let mut max: SVector<D, 3> = vector![
        D::from(settings.tube_exclusion_radius),
        D::from(0.17),
        D::from(settings.tube_exclusion_radius)
    ];
    let mut tube_exclusion_loss = D::zero();
    let mut thruster_exclusion_loss = D::zero();

    for (id, motor) in motor_config.motors() {
        let pos = motor.position;

        min = vector![min.x.min(pos.x), min.y.min(pos.y), min.z.min(pos.z)];
        max = vector![max.x.max(pos.x), max.y.max(pos.y), max.z.max(pos.z)];

        position_sum += pos / D::from(thruster_count as FloatType);

        for (other_id, other_motor) in motor_config.motors() {
            if id == other_id {
                continue;
            }

            let delta = pos - other_motor.position;
            let space_between = delta.norm() - D::from(2.0 * settings.thruster_exclusion_radius);
            if space_between < D::zero() {
                thruster_exclusion_loss += space_between * space_between;
            }
        }

        let pos_2d = motor.position.xz();
        let space_between = pos_2d.norm()
            - D::from(settings.tube_exclusion_radius + settings.thruster_exclusion_radius);
        if space_between < D::zero() {
            tube_exclusion_loss += space_between * space_between;
        }
    }

    let center = (max + min) / D::from(2.0);
    let half_extent = (max - min) / D::from(2.0);

    // let center_of_mass = position_sum.dot(&position_sum).powi(4);
    let center_of_mass = position_sum
        .dot(&position_sum)
        .max(position_sum.norm() * 10.0);
    let center = center.dot(&center);
    // let surface_area = D::from(8.0)
    //     * (half_extent.x * (half_extent.y + half_extent.z) + half_extent.y * half_extent.z);

    let surface_area = D::from(4.0)
        * (result[&Axis::X] / (half_extent.y * half_extent.z)
            + result[&Axis::Y] / (half_extent.z * half_extent.x)
            + result[&Axis::Z] / (half_extent.x * half_extent.y));

    let dimension = D::from(4.0)
        * (half_extent.x * half_extent.x
            + half_extent.y * half_extent.y
            + half_extent.z * half_extent.z);

    (
        D::from(settings.x) * result[&Axis::X]
            + D::from(settings.y) * result[&Axis::Y]
            + D::from(settings.z) * result[&Axis::Z]
            + D::from(settings.x_rot) * result[&Axis::XRot]
            + D::from(settings.y_rot) * result[&Axis::YRot]
            + D::from(settings.z_rot) * result[&Axis::ZRot]
            + D::from(settings.mes_linear) * mes_linear
            + D::from(settings.mes_torque) * mes_torque
            + D::from(settings.min_linear) * min_linear
            + D::from(settings.min_torque) * min_torque
            + D::from(settings.avg_linear) * avg_linear
            + D::from(settings.avg_torque) * avg_torque
            + D::from(settings.center_of_mass_loss) * center_of_mass
            + D::from(settings.center_loss) * center
            + D::from(settings.surface_area_loss) * surface_area
            + D::from(settings.dimension_loss) * dimension
            + D::from(settings.tube_exclusion_loss) * tube_exclusion_loss
            + D::from(settings.thruster_exclusion_loss) * thruster_exclusion_loss,
        ScoreResult {
            mes_linear,
            mes_torque,
            avg_linear,
            avg_torque,
            min_linear,
            min_torque,
            x: result[&Axis::X],
            y: result[&Axis::Y],
            z: result[&Axis::Z],
            x_rot: result[&Axis::XRot],
            y_rot: result[&Axis::YRot],
            z_rot: result[&Axis::ZRot],
            center_of_mass_loss: center_of_mass,
            center_loss: center,
            surface_area_loss: surface_area,
            dimension_loss: dimension,
            tube_exclusion_loss,
            thruster_exclusion_loss,
        },
    )
}
