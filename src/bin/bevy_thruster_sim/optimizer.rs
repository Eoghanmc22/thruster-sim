use bevy::prelude::*;
use motor_math::FloatType;
use settings::ToggleableScoreSettings;
use thruster_sim::optimize::OptimizationArena;

use crate::{motor_config::MotorConfigRes, MotorDataRes};

pub mod gui;
pub mod settings;

#[derive(Resource)]
pub struct ScoreSettingsRes(pub ToggleableScoreSettings);

#[derive(Resource)]
pub struct OptimizerArenaRes(pub Box<dyn OptimizationArena + Send + Sync + 'static>);

pub fn step_accent_points(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    mut optimizer: ResMut<OptimizerArenaRes>,
) {
    let best = optimizer.0.step(&motor_data.0).next();
    if let Some(best) = best {
        let current_score = motor_conf.0.score;

        if best.score - current_score > 0.001 {
            commands.insert_resource(MotorConfigRes(best));
        }
    }
}

pub fn handle_heuristic_change(
    score_settings: Res<ScoreSettingsRes>,
    mut motor_conf: ResMut<MotorConfigRes>,
    mut optimizer: ResMut<OptimizerArenaRes>,
) {
    if score_settings.is_changed() {
        info!("Heuristic changed");

        optimizer.0.reset(100, score_settings.0.flatten());
        motor_conf.0.score = FloatType::NEG_INFINITY;
    }
}
