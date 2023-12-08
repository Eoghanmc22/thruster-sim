use ahash::HashMap;

use crate::{PhysicsAxis, PhysicsResult};

#[derive(Clone)]
pub struct ScoreSettings {
    pub mes_linear: f32,
    pub mes_torque: f32,
    pub avg_linear: f32,
    pub avg_torque: f32,
    pub min_linear: f32,
    pub min_torque: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub x_rot: f32,
    pub y_rot: f32,
    pub z_rot: f32,
}

impl Default for ScoreSettings {
    fn default() -> Self {
        Self {
            mes_linear: -1.0,
            mes_torque: 0.0,
            avg_linear: 0.0,
            avg_torque: 0.0,
            min_linear: 0.0,
            min_torque: 0.0,
            x: 1.0,
            y: 1.0,
            z: 1.0,
            x_rot: 1.0,
            y_rot: 1.0,
            z_rot: 1.0,
        }
    }
}

pub fn score(result: &HashMap<PhysicsAxis, PhysicsResult>, settings: &ScoreSettings) -> f32 {
    // Average and min
    let mut avg_linear = 0.0;
    let mut avg_torque = 0.0;
    let mut min_linear = f32::INFINITY;
    let mut min_torque = f32::INFINITY;

    for result in result.values() {
        match result {
            PhysicsResult::Linear(val) => {
                avg_linear += val / 3.0;
                min_linear = min_linear.min(*val);
            }
            PhysicsResult::Torque(val) => {
                avg_torque += val / 3.0;
                min_torque = min_torque.min(*val);
            }
        }
    }

    // Mean error squared
    let mut mes_linear = 0.0;
    let mut mes_torque = 0.0;

    for result in result.values() {
        match result {
            PhysicsResult::Linear(val) => mes_linear += (val - avg_linear) * (val - avg_linear),
            PhysicsResult::Torque(val) => mes_torque += (val - avg_torque) * (val - avg_torque),
        }
    }

    // Minimums to cull search space
    let torque_sucks = {
        if min_torque < 0.2 {
            f32::NEG_INFINITY
        } else {
            0.0
        }
    };

    let linear_sucks = {
        if min_linear < 0.5 {
            f32::NEG_INFINITY
        } else {
            0.0
        }
    };

    // Per axis values
    let results: HashMap<PhysicsAxis, f32> = result
        .iter()
        .map(|(axis, result)| match result {
            PhysicsResult::Linear(value) | PhysicsResult::Torque(value) => (*axis, value.abs()),
        })
        .collect();

    0.0 + mes_linear * settings.mes_linear
        + mes_torque * settings.mes_torque
        + min_linear * settings.min_linear
        + min_torque * settings.min_torque
        + avg_linear * settings.avg_linear
        + avg_torque * settings.avg_torque
        + results[&PhysicsAxis::X] * settings.x
        + results[&PhysicsAxis::Z] * settings.y
        + results[&PhysicsAxis::Y] * settings.z
        + results[&PhysicsAxis::XRot] * settings.x_rot
        + results[&PhysicsAxis::ZRot] * settings.y_rot
        + results[&PhysicsAxis::YRot] * settings.z_rot
    // - 30.0 * (results[&PhysicsAxis::X] - results[&PhysicsAxis::Y]).max(0.0)
    // - 30.0 * (results[&PhysicsAxis::Y] - results[&PhysicsAxis::Z]).max(0.0)
    // + 30.0 * (results[&PhysicsAxis::Y] - 4.5)
}
