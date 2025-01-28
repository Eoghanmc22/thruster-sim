use motor_math::{solve::reverse::Axis, FloatType, MotorConfig, Number};
use nalgebra::{vector, SVector};
use stable_hashmap::StableHashMap;
use std::fmt::Debug;
use std::hash::Hash;
use std::marker::PhantomData;

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
    pub surface_area_score: FloatType,
    pub dimension_loss: FloatType,
    pub cardinality_loss: FloatType,

    pub tube_exclusion_radius: FloatType,
    pub tube_exclusion_loss: FloatType,

    pub thruster_exclusion_radius: FloatType,
    pub thruster_exclusion_loss: FloatType,
    pub thruster_flow_exclusion_loss: FloatType,
}

impl Default for ScoreSettings {
    fn default() -> Self {
        Self {
            mes_linear: -2.0,
            mes_x_off: 0.0,
            mes_y_off: 65.0,
            mes_z_off: 0.0,
            mes_torque: 0.0,
            mes_x_rot_off: 0.0,
            mes_y_rot_off: 0.0,
            mes_z_rot_off: 0.0,
            avg_linear: 0.05,
            avg_torque: 0.8,
            min_linear: 0.02,
            min_torque: 0.36,
            x: 0.2,
            y: 0.55,
            z: 0.4,
            x_rot: 0.35,
            y_rot: 0.2,
            z_rot: 0.25,
            center_of_mass_loss: -500.0,
            center_loss: 0.0,
            surface_area_score: 0.0,
            dimension_loss: -1000.0,
            tube_exclusion_radius: 0.08,
            thruster_exclusion_radius: 0.08,
            tube_exclusion_loss: -500.0,
            thruster_exclusion_loss: -500.0,
            thruster_flow_exclusion_loss: -10.0,
            cardinality_loss: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub enum Scaled {}
#[derive(Debug, Clone)]
pub enum Unscaled {}

#[derive(Debug, Clone)]
pub struct ScoreResult<D, Type> {
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
    pub surface_area_score: D,
    pub dimension_loss: D,
    pub tube_exclusion_loss: D,
    pub thruster_exclusion_loss: D,
    pub thruster_flow_exclusion_loss: D,
    pub cardinality_loss: D,

    phantom: PhantomData<Type>,
}

impl<D: Number> ScoreResult<D, Unscaled> {
    pub fn scale(&self, settings: &ScoreSettings) -> ScoreResult<D, Scaled> {
        ScoreResult::<D, Scaled> {
            x: D::from(settings.x) * self.x,
            y: D::from(settings.y) * self.y,
            z: D::from(settings.z) * self.z,
            x_rot: D::from(settings.x_rot) * self.x_rot,
            y_rot: D::from(settings.y_rot) * self.y_rot,
            z_rot: D::from(settings.z_rot) * self.z_rot,
            mes_linear: D::from(settings.mes_linear) * self.mes_linear,
            mes_torque: D::from(settings.mes_torque) * self.mes_torque,
            min_linear: D::from(settings.min_linear) * self.min_linear,
            min_torque: D::from(settings.min_torque) * self.min_torque,
            avg_linear: D::from(settings.avg_linear) * self.avg_linear,
            avg_torque: D::from(settings.avg_torque) * self.avg_torque,
            center_of_mass_loss: D::from(settings.center_of_mass_loss) * self.center_of_mass_loss,
            center_loss: D::from(settings.center_loss) * self.center_loss,
            surface_area_score: D::from(settings.surface_area_score) * self.surface_area_score,
            dimension_loss: D::from(settings.dimension_loss) * self.dimension_loss,
            tube_exclusion_loss: D::from(settings.tube_exclusion_loss) * self.tube_exclusion_loss,
            thruster_exclusion_loss: D::from(settings.thruster_exclusion_loss)
                * self.thruster_exclusion_loss,
            thruster_flow_exclusion_loss: D::from(settings.thruster_flow_exclusion_loss)
                * self.thruster_flow_exclusion_loss,
            cardinality_loss: D::from(settings.cardinality_loss) * self.cardinality_loss,
            phantom: PhantomData::default(),
        }
    }

    pub fn score(&self, settings: &ScoreSettings) -> D {
        self.scale(settings).score()
    }
}

impl<D: Number> ScoreResult<D, Scaled> {
    pub fn score(&self) -> D {
        self.x
            + self.y
            + self.z
            + self.x_rot
            + self.y_rot
            + self.z_rot
            + self.mes_linear
            + self.mes_torque
            + self.min_linear
            + self.min_torque
            + self.avg_linear
            + self.avg_torque
            + self.center_of_mass_loss
            + self.center_loss
            + self.surface_area_score
            + self.dimension_loss
            + self.tube_exclusion_loss
            + self.thruster_exclusion_loss
            + self.thruster_flow_exclusion_loss
            + self.cardinality_loss
    }
}

impl<D: Number, Type> ScoreResult<D, Type> {
    pub fn to_float(&self) -> ScoreResult<FloatType, Type> {
        ScoreResult::<FloatType, Type> {
            x: self.x.re(),
            y: self.y.re(),
            z: self.z.re(),
            x_rot: self.x_rot.re(),
            y_rot: self.y_rot.re(),
            z_rot: self.z_rot.re(),
            mes_linear: self.mes_linear.re(),
            mes_torque: self.mes_torque.re(),
            min_linear: self.min_linear.re(),
            min_torque: self.min_torque.re(),
            avg_linear: self.avg_linear.re(),
            avg_torque: self.avg_torque.re(),
            center_of_mass_loss: self.center_of_mass_loss.re(),
            center_loss: self.center_loss.re(),
            surface_area_score: self.surface_area_score.re(),
            dimension_loss: self.dimension_loss.re(),
            tube_exclusion_loss: self.tube_exclusion_loss.re(),
            thruster_exclusion_loss: self.thruster_exclusion_loss.re(),
            thruster_flow_exclusion_loss: self.thruster_flow_exclusion_loss.re(),
            cardinality_loss: self.cardinality_loss.re(),
            phantom: PhantomData::default(),
        }
    }
}

impl<D: Number + Default, Type> Default for ScoreResult<D, Type> {
    fn default() -> Self {
        Self {
            mes_linear: Default::default(),
            mes_torque: Default::default(),
            avg_linear: Default::default(),
            avg_torque: Default::default(),
            min_linear: Default::default(),
            min_torque: Default::default(),
            x: Default::default(),
            y: Default::default(),
            z: Default::default(),
            x_rot: Default::default(),
            y_rot: Default::default(),
            z_rot: Default::default(),
            center_of_mass_loss: Default::default(),
            center_loss: Default::default(),
            surface_area_score: Default::default(),
            dimension_loss: Default::default(),
            tube_exclusion_loss: Default::default(),
            thruster_exclusion_loss: Default::default(),
            thruster_flow_exclusion_loss: Default::default(),
            cardinality_loss: Default::default(),
            phantom: Default::default(),
        }
    }
}

pub fn score<MotorId: Debug + Ord + Hash + Clone, D: Number>(
    result: &StableHashMap<Axis, D>,
    motor_config: &MotorConfig<MotorId, D>,
    settings: &ScoreSettings,
) -> (D, ScoreResult<D, Unscaled>) {
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

        let (val, _avg) = match axis {
            Axis::X | Axis::Y | Axis::Z => (*result, avg_linear),
            Axis::XRot | Axis::YRot | Axis::ZRot => (*result, avg_torque),
        };

        let goal = if offset.re() > 0.0 { offset } else { val };

        match axis {
            Axis::X | Axis::Y | Axis::Z => mes_linear += (val - goal) * (val - goal),
            Axis::XRot | Axis::YRot | Axis::ZRot => mes_torque += (val - goal) * (val - goal),
        }
    }

    let thruster_count = motor_config.motors().count();
    let thruster_count = D::from(thruster_count as FloatType);

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
    let mut thruster_flow_exclusion_loss = D::zero();

    let mut average_direction = SVector::<D, 3>::zeros();

    for (id, motor) in motor_config.motors() {
        let pos = motor.position;

        min = vector![min.x.min(pos.x), min.y.min(pos.y), min.z.min(pos.z)];
        max = vector![max.x.max(pos.x), max.y.max(pos.y), max.z.max(pos.z)];

        position_sum += pos / thruster_count;

        let positive_canidate = average_direction + motor.orientation / thruster_count;
        let negative_canidate = average_direction - motor.orientation / thruster_count;

        if positive_canidate.norm_squared() >= negative_canidate.norm_squared() {
            average_direction = positive_canidate;
        } else {
            average_direction = negative_canidate;
        }

        for (other_id, other_motor) in motor_config.motors() {
            if id == other_id {
                continue;
            }

            let delta = pos - other_motor.position;

            // Intersection loss
            let space_between = delta.norm() - D::from(2.0 * settings.thruster_exclusion_radius);
            if space_between < D::zero() {
                thruster_exclusion_loss += space_between * space_between;
            }

            // Parallel distance/flow loss
            let dot = delta.dot(&other_motor.orientation);
            let proj = other_motor.position + other_motor.orientation * dot;
            let perp_dist = (pos - proj).norm();
            thruster_flow_exclusion_loss +=
                (D::from(settings.thruster_exclusion_radius * settings.thruster_exclusion_radius)
                    / (perp_dist + D::from(0.001)))
                    / D::from(2.0);
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
        * (result[&Axis::X] / (half_extent.y * half_extent.z * D::from(100.0 * 100.0))
            + result[&Axis::Y] / (half_extent.z * half_extent.x * D::from(100.0 * 100.0))
            + result[&Axis::Z] / (half_extent.x * half_extent.y * D::from(100.0 * 100.0)));

    let dimension = D::from(4.0)
        * (half_extent.x * half_extent.x * half_extent.x * half_extent.x
            + half_extent.y * half_extent.y * half_extent.y * half_extent.y
            + half_extent.z * half_extent.z * half_extent.z * half_extent.z);

    let strongest_dir = average_direction.normalize();
    let cardinality_loss = strongest_dir.norm() - strongest_dir.abs().max();
    let cardinality_loss = cardinality_loss * cardinality_loss;

    let result = ScoreResult::<_, Unscaled> {
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
        surface_area_score: surface_area,
        dimension_loss: dimension,
        tube_exclusion_loss,
        thruster_exclusion_loss,
        thruster_flow_exclusion_loss,
        cardinality_loss,
        phantom: Default::default(),
    };

    (result.score(settings), result)
}
