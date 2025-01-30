use bevy::prelude::*;
use motor_math::FloatType;
use settings::ToggleableScoreSettings;
use thruster_sim::{
    optimize::{
        full::FullOptimization, symetrical::SymerticalOptimization,
        x3d_fixed::FixedX3dOptimization, AsyncOptimizationArena, OptimizationArena,
        OptimizationOutput, SyncOptimizationArena,
    },
    HEIGHT, LENGTH, WIDTH,
};

use crate::{motor_config::MotorConfigRes, MotorDataRes};

pub mod gui;
pub mod settings;

#[derive(Resource, Clone, Copy, PartialEq, Eq)]
pub struct ArenaMode {
    pub arena_type: ArenaType,
    pub is_async: bool,
    pub point_count: usize,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ArenaType {
    X3d,
    Symmetrical3,
    Unconstrained6,
}

#[derive(Resource)]
pub struct ScoreSettingsRes(pub ToggleableScoreSettings);

#[derive(Resource)]
pub struct OptimizerArenaRes(pub Box<dyn OptimizationArena + Send + Sync + 'static>);

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShownConfig {
    Best,
    Index(usize),
}

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum OptimizerStatus {
    Running,
    Paused,
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
    status: Res<OptimizerStatus>,
    mut optimizer: ResMut<OptimizerArenaRes>,
    mut best: ResMut<TopConfigs>,
) {
    if let OptimizerStatus::Running = *status {
        best.configs.clear();
        for config in optimizer.0.step(&motor_data.0).take(10) {
            best.configs.push(config);
        }
    }

    let current_score = motor_conf.0.score;
    match *shown_config {
        ShownConfig::Best => {
            if let Some(best) = best.configs.first() {
                // if shown_config.is_changed() || best.score - current_score > 0.001 {
                commands.insert_resource(MotorConfigRes(best.clone()));
                // }
            }
        }
        ShownConfig::Index(idx) => {
            if let Some(idx) = optimizer.0.lookup_index(idx) {
                // if shown_config.is_changed() || idx.score - current_score > 0.001 {
                commands.insert_resource(MotorConfigRes(idx));
                // }
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

        optimizer.0.set_heuristic(score_settings.0.flatten());
        motor_conf.0.score = FloatType::NEG_INFINITY;
    }
}

#[derive(Event)]
pub struct ResetEvent;

pub fn handle_reset(
    mut commands: Commands,
    score_settings: Res<ScoreSettingsRes>,
    arena_mode: Res<ArenaMode>,
    mut motor_conf: ResMut<MotorConfigRes>,
    mut optimizer: ResMut<OptimizerArenaRes>,
    mut reset_event: EventReader<ResetEvent>,
) {
    if arena_mode.is_changed() {
        let arena: Box<dyn OptimizationArena + Send + Sync + 'static> =
            match (arena_mode.arena_type, arena_mode.is_async) {
                (ArenaType::X3d, true) => {
                    Box::new(AsyncOptimizationArena::new(FixedX3dOptimization {
                        width: WIDTH / 2.0,
                        length: LENGTH / 2.0,
                        height: HEIGHT / 2.0,
                    }))
                }
                (ArenaType::X3d, false) => {
                    Box::new(SyncOptimizationArena::new(FixedX3dOptimization {
                        width: WIDTH / 2.0,
                        length: LENGTH / 2.0,
                        height: HEIGHT / 2.0,
                    }))
                }
                (ArenaType::Symmetrical3, true) => {
                    Box::new(AsyncOptimizationArena::new(SymerticalOptimization::<3>))
                }
                (ArenaType::Symmetrical3, false) => {
                    Box::new(SyncOptimizationArena::new(SymerticalOptimization::<3>))
                }
                (ArenaType::Unconstrained6, true) => {
                    Box::new(AsyncOptimizationArena::new(FullOptimization::<6>))
                }
                (ArenaType::Unconstrained6, false) => {
                    Box::new(SyncOptimizationArena::new(FullOptimization::<6>))
                }
            };

        commands.insert_resource(OptimizerArenaRes(arena));
        commands.add(|world: &mut World| {
            world.send_event(ResetEvent);
        });
    }

    if !reset_event.is_empty() {
        reset_event.clear();
        info!("Reset Optimizer");

        optimizer
            .0
            .reset(arena_mode.point_count, score_settings.0.flatten());

        motor_conf.0.score = FloatType::NEG_INFINITY;
    }
}
