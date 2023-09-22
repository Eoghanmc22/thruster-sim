use anyhow::Context;
use fxhash::FxHashMap as HashMap;

use itertools::Itertools;
use serde::{Deserialize, Serialize};

const DEFAULT_MOTOR_CW: MotorSettings = MotorSettings { sign: 1.0 };
const DEFAULT_MOTOR_CCW: MotorSettings = MotorSettings { sign: -1.0 };

// ---------- Thrusters ----------
pub const MOTOR_FLB: MotorSettings = DEFAULT_MOTOR_CCW;
pub const MOTOR_FLT: MotorSettings = DEFAULT_MOTOR_CW;
pub const MOTOR_FRB: MotorSettings = DEFAULT_MOTOR_CW;
pub const MOTOR_FRT: MotorSettings = DEFAULT_MOTOR_CCW;
pub const MOTOR_BLB: MotorSettings = DEFAULT_MOTOR_CW;
pub const MOTOR_BLT: MotorSettings = DEFAULT_MOTOR_CCW;
pub const MOTOR_BRB: MotorSettings = DEFAULT_MOTOR_CCW;
pub const MOTOR_BRT: MotorSettings = DEFAULT_MOTOR_CW;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct MotorSettings {
    pub sign: f64,
}

impl From<MotorId> for MotorSettings {
    #[rustfmt::skip]
    fn from(value: MotorId) -> Self {
        match value {
            MotorId::FrontLeftBottom =>   MOTOR_FLB,
            MotorId::FrontLeftTop =>      MOTOR_FLT,
            MotorId::FrontRightBottom =>  MOTOR_FRB,
            MotorId::FrontRightTop =>     MOTOR_FRT,
            MotorId::BackLeftBottom =>    MOTOR_BLB,
            MotorId::BackLeftTop =>       MOTOR_BLT,
            MotorId::BackRightBottom =>   MOTOR_BRB,
            MotorId::BackRightTop =>      MOTOR_BRT,
        }
    }
}

/// +X: Right, +Y: Forwards, +Z: Up
/// +XR: Pitch Up, +YR: Roll Clockwise, +ZR: Yaw Clockwise (top view)
#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct Movement {
    /// Right
    pub x: f64,
    /// Forwards
    pub y: f64,
    /// Up
    pub z: f64,

    /// Pitch Up
    pub x_rot: f64,
    /// Roll Clockwise
    pub y_rot: f64,
    /// Yaw Clockwise (top view)
    pub z_rot: f64,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum MotorId {
    FrontLeftBottom,
    FrontLeftTop,
    FrontRightBottom,
    FrontRightTop,
    BackLeftBottom,
    BackLeftTop,
    BackRightBottom,
    BackRightTop,
}

/// Calculates the max forwards and reverse motor thrusts
/// when all motors are moving at max allowed speed
///
/// Returns (forward, reverse) in kgf
pub fn calculate_thrust_limits(motor_data: &MotorData) -> (f64, f64) {
    // max_amperage / total_desired
    let scale = 20.0 / (1.0 * 4.0 + 1.25 * 4.0);

    [1.0, -1.25]
        .into_iter()
        .map(|value| value * scale)
        .map(|current| motor_data.lookup_by_current(current).force)
        .collect_tuple()
        .unwrap()
}

pub fn mix_movement<'a>(
    mov: Movement,
    motor_data: &MotorData,
    motor_mixer: impl Fn(MotorId, &Movement) -> f64,
) -> HashMap<MotorId, f64> {
    const MAX_AMPERAGE: f64 = 20.0;

    let drive_ids = [
        MotorId::FrontLeftBottom,
        MotorId::FrontLeftTop,
        MotorId::FrontRightBottom,
        MotorId::FrontRightTop,
        MotorId::BackLeftBottom,
        MotorId::BackLeftTop,
        MotorId::BackRightBottom,
        MotorId::BackRightTop,
    ];

    let mut raw_mix = HashMap::default();

    for motor_id in drive_ids {
        let motor = MotorSettings::from(motor_id);

        // #[rustfmt::skip]
        // let speed = match motor_id {
        //     MotorId::FrontLeftBottom =>   -x - y + z + x_rot + y_rot - z_rot,
        //     MotorId::FrontLeftTop =>      -x - y - z - x_rot - y_rot - z_rot,
        //     MotorId::FrontRightBottom =>   x - y + z + x_rot - y_rot + z_rot,
        //     MotorId::FrontRightTop =>      x - y - z - x_rot + y_rot + z_rot,
        //     MotorId::BackLeftBottom =>    -x + y + z - x_rot + y_rot + z_rot,
        //     MotorId::BackLeftTop =>       -x + y - z + x_rot - y_rot + z_rot,
        //     MotorId::BackRightBottom =>    x + y + z - x_rot - y_rot - z_rot,
        //     MotorId::BackRightTop =>       x + y - z + x_rot + y_rot - z_rot,
        // };

        let speed = (motor_mixer)(motor_id, &mov);

        let skew = if speed >= 0.0 { 1.0 } else { 1.25 };
        let direction = motor.sign;

        raw_mix.insert(motor_id, speed * skew * direction);
    }

    let max_raw = raw_mix.len() as f64;
    let total_raw: f64 = raw_mix.values().map(|it| it.abs()).sum();
    let scale_raw = if total_raw > max_raw {
        max_raw / total_raw
    } else {
        // Handle cases where we dont want to go max speed
        1.0
    };

    let motor_amperage = MAX_AMPERAGE / max_raw;
    raw_mix
        .into_iter()
        .map(|(motor, value)| (motor, value * scale_raw * motor_amperage))
        .map(|(motor, current)| (motor, motor_data.lookup_by_current(current).force))
        .collect()
}

pub struct MotorData {
    forward: Vec<MotorRecord>,
    backward: Vec<MotorRecord>,
}

impl MotorData {
    pub fn sort(&mut self) {
        self.forward
            .sort_by(|a, b| f64::total_cmp(&a.current, &b.current));
        self.backward
            .sort_by(|a, b| f64::total_cmp(&a.current, &b.current));
    }

    // TODO: Interpolate
    pub fn lookup_by_current(&self, signed_current: f64) -> MotorRecord {
        let current = signed_current.abs();

        let data_set = if signed_current >= 0.0 {
            &self.forward
        } else {
            &self.backward
        };

        let idx = data_set.partition_point(|x| x.current < current);
        data_set[idx]
    }
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct MotorRecord {
    pwm: f64,
    rpm: f64,
    current: f64,
    voltage: f64,
    power: f64,
    force: f64,
    efficiency: f64,
}

pub fn read_motor_data() -> anyhow::Result<MotorData> {
    let forward = csv::Reader::from_path("forward_motor_data.csv").context("Read forward data")?;
    let reverse = csv::Reader::from_path("reverse_motor_data.csv").context("Read reverse data")?;

    let mut forward_data = Vec::default();
    for result in forward.into_deserialize() {
        let record: MotorRecord = result.context("Parse motor record")?;
        forward_data.push(record);
    }

    let mut reverse_data = Vec::default();
    for result in reverse.into_deserialize() {
        let record: MotorRecord = result.context("Parse motor record")?;
        reverse_data.push(record);
    }

    let mut motor_data = MotorData {
        forward: forward_data,
        backward: reverse_data,
    };
    motor_data.sort();

    Ok(motor_data)
}
