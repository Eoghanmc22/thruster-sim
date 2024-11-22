use std::{collections::HashMap, hash::BuildHasherDefault};

use motor_math::{FloatType, Number};
use stable_hashmap::StableHashMap;

use crate::{PhysicsAxis, PhysicsResult};

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
            avg_torque: 0.0,
            min_linear: 0.0,
            min_torque: 0.0,
            x: 0.5,
            y: 0.5,
            z: 0.5,
            x_rot: 0.35,
            y_rot: 0.2,
            z_rot: 0.25,
        }
    }
}

pub fn score<D: Number>(
    result: &StableHashMap<PhysicsAxis, PhysicsResult<D>>,
    settings: &ScoreSettings,
) -> D {
    // Average and min
    let mut avg_linear = D::from(0.0);
    let mut avg_torque = D::from(0.0);
    let mut min_linear = D::from(FloatType::INFINITY);
    let mut min_torque = D::from(FloatType::INFINITY);

    for result in result.values() {
        match result {
            PhysicsResult::Linear(val) => {
                avg_linear += *val / 3.0;
                min_linear = min_linear.min(*val);
            }
            PhysicsResult::Torque(val) => {
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
            PhysicsAxis::X => settings.mes_x_off,
            PhysicsAxis::Y => settings.mes_y_off,
            PhysicsAxis::Z => settings.mes_z_off,
            PhysicsAxis::XRot => settings.mes_x_rot_off,
            PhysicsAxis::YRot => settings.mes_y_rot_off,
            PhysicsAxis::ZRot => settings.mes_z_rot_off,
        };
        let offset = D::from(offset);

        let (val, avg) = match *result {
            PhysicsResult::Linear(val) => (val, avg_linear),
            PhysicsResult::Torque(val) => (val, avg_torque),
        };

        let goal = if offset.re() > 0.0 {
            offset
        } else if offset.re() == 0.0 {
            avg
        } else {
            val
        };

        match *result {
            PhysicsResult::Linear(val) => mes_linear += (val - goal) * (val - goal),
            PhysicsResult::Torque(val) => mes_torque += (val - goal) * (val - goal),
        }
    }

    // // Minimums to cull search space
    // let torque_sucks = {
    //     if min_torque < 0.2 {
    //         f32::NEG_INFINITY
    //     } else {
    //         0.0
    //     }
    // };
    //
    // let linear_sucks = {
    //     if min_linear < 0.5 {
    //         f32::NEG_INFINITY
    //     } else {
    //         0.0
    //     }
    // };

    // Per axis values
    let results: StableHashMap<PhysicsAxis, D> = result
        .iter()
        .map(|(axis, result)| match result {
            &PhysicsResult::Linear(value) | &PhysicsResult::Torque(value) => (*axis, value.abs()),
        })
        .collect();

    D::from(settings.x) * results[&PhysicsAxis::X]
        + D::from(settings.y) * results[&PhysicsAxis::Y]
        + D::from(settings.z) * results[&PhysicsAxis::Z]
        + D::from(settings.x_rot) * results[&PhysicsAxis::XRot]
        + D::from(settings.y_rot) * results[&PhysicsAxis::YRot]
        + D::from(settings.z_rot) * results[&PhysicsAxis::ZRot]
        + D::from(settings.mes_linear) * mes_linear
        + D::from(settings.mes_torque) * mes_torque
        + D::from(settings.min_linear) * min_linear
        + D::from(settings.min_torque) * min_torque
        + D::from(settings.avg_linear) * avg_linear
        + D::from(settings.avg_torque) * avg_torque
}
