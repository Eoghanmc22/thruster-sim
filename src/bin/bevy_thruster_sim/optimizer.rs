use bevy::prelude::*;
use motor_math::FloatType;
use settings::ToggleableScoreSettings;
use thruster_sim::optimize::{OptimizationArena, OptimizationOutput};

use crate::{motor_config::MotorConfigRes, MotorDataRes};

pub mod gui;
pub mod settings;

#[derive(Resource)]
pub struct ScoreSettingsRes(pub ToggleableScoreSettings);

#[derive(Resource)]
pub struct OptimizerArenaRes(pub Box<dyn OptimizationArena + Send + Sync + 'static>);

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShownConfig {
    Best,
    Index(usize),
}

#[derive(Resource, Debug, Clone)]
pub struct TopConfigs {
    pub configs: Vec<OptimizationOutput>,
}

pub fn step_accent_points(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    shown_config: Res<ShownConfig>,
    mut optimizer: ResMut<OptimizerArenaRes>,
    mut best: ResMut<TopConfigs>,
) {
    best.configs.clear();
    for config in optimizer.0.step(&motor_data.0).take(10) {
        best.configs.push(config);
    }

    let current_score = motor_conf.0.score;
    match *shown_config {
        ShownConfig::Best => {
            if let Some(best) = best.configs.first() {
                if shown_config.is_changed() || best.score - current_score > 0.001 {
                    commands.insert_resource(MotorConfigRes(best.clone()));
                }
            }
        }
        ShownConfig::Index(idx) => {
            if let Some(idx) = optimizer.0.lookup_index(idx) {
                if shown_config.is_changed() || idx.score - current_score > 0.001 {
                    commands.insert_resource(MotorConfigRes(idx));
                }
            }
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
