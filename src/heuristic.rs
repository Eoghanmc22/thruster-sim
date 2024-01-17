use ahash::HashMap;

use crate::{PhysicsAxis, PhysicsResult};

#[derive(Clone)]
pub struct ScoreSettings {
    pub mes_linear: f32,
    pub mes_x_off: f32,
    pub mes_y_off: f32,
    pub mes_z_off: f32,

    pub mes_torque: f32,
    pub mes_x_rot_off: f32,
    pub mes_y_rot_off: f32,
    pub mes_z_rot_off: f32,

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

    for (axis, result) in result {
        let offset = match axis {
            PhysicsAxis::X => settings.mes_x_off,
            PhysicsAxis::Y => settings.mes_y_off,
            PhysicsAxis::Z => settings.mes_z_off,
            PhysicsAxis::XRot => settings.mes_x_rot_off,
            PhysicsAxis::YRot => settings.mes_y_rot_off,
            PhysicsAxis::ZRot => settings.mes_z_rot_off,
        };

        let (val, avg) = match result {
            &PhysicsResult::Linear(val) => (val, avg_linear),
            &PhysicsResult::Torque(val) => (val, avg_torque),
        };

        let goal = if offset > 0.0 {
            offset
        } else if offset == 0.0 {
            avg
        } else {
            val
        };

        match result {
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
    let results: HashMap<PhysicsAxis, f32> = result
        .iter()
        .map(|(axis, result)| match result {
            PhysicsResult::Linear(value) | PhysicsResult::Torque(value) => (*axis, value.abs()),
        })
        .collect();

    0.0 + settings.x * results[&PhysicsAxis::X]
        + settings.y * results[&PhysicsAxis::Y]
        + settings.z * results[&PhysicsAxis::Z]
        + settings.x_rot * results[&PhysicsAxis::XRot]
        + settings.y_rot * results[&PhysicsAxis::YRot]
        + settings.z_rot * results[&PhysicsAxis::ZRot]
        + settings.mes_linear * mes_linear
        + settings.mes_torque * mes_torque
        + settings.min_linear * min_linear
        + settings.min_torque * min_torque
        + settings.avg_linear * avg_linear
        + settings.avg_torque * avg_torque
}
