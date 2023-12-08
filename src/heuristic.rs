use ahash::HashMap;

use crate::{PhysicsAxis, PhysicsResult};

pub fn score(result: &HashMap<PhysicsAxis, PhysicsResult>) -> f32 {
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
        .into_iter()
        .map(|(axis, result)| match result {
            PhysicsResult::Linear(value) | PhysicsResult::Torque(value) => (*axis, value.abs()),
        })
        .collect();

    0.0 
        - mes_linear
        - mes_torque
        // + min_linear * 0.5
        // + min_torque * 2.5
        // + avg_linear * 1.0
        // + avg_torque * 2.0
        + results[&PhysicsAxis::X] * 1.0
        + results[&PhysicsAxis::Z] * 1.0
        + results[&PhysicsAxis::Y] * 1.0
        // + results[&PhysicsAxis::XRot] * 4.0
        // + results[&PhysicsAxis::ZRot] * 4.0
        // + results[&PhysicsAxis::YRot] * -3.0
    // - 30.0 * (results[&PhysicsAxis::X] - results[&PhysicsAxis::Y]).max(0.0)
    // - 30.0 * (results[&PhysicsAxis::Y] - results[&PhysicsAxis::Z]).max(0.0)
    // + 30.0 * (results[&PhysicsAxis::Y] - 4.5)
}
