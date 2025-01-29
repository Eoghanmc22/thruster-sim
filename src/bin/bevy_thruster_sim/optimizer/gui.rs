use std::collections::BTreeMap;

use bevy::prelude::*;
use bevy_egui::{
    egui::{self, Slider},
    EguiContexts,
};
use bevy_panorbit_camera::PanOrbitCamera;
use motor_math::solve::reverse;
use thruster_sim::heuristic::MesType;

use crate::{motor_config::MotorConfigRes, MotorDataRes};

use super::{OptimizerStatus, ResetEvent, ScoreSettingsRes, ShownConfig, TopConfigs};

pub fn render_gui(
    mut commands: Commands,
    mut contexts: EguiContexts,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    solver: Res<ScoreSettingsRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
    mut shown_config: ResMut<ShownConfig>,
    best: Res<TopConfigs>,
    mut status: ResMut<OptimizerStatus>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        ui.set_width(250.0);

        ui.collapsing("Instances", |ui| {
            let mut shown = *shown_config;

            ui.horizontal(|ui| {
                if ui.button("Reset").clicked() {
                    commands.add(|world: &mut World| {
                        world.send_event(ResetEvent);
                    });
                }

                match *status {
                    OptimizerStatus::Running => {
                        if ui.button("Pause").clicked() {
                            *status = OptimizerStatus::Paused;
                        }
                    }
                    OptimizerStatus::Paused => {
                        if ui.button("Resume").clicked() {
                            *status = OptimizerStatus::Running;
                        }
                    }
                }
            });

            ui.selectable_value(&mut shown, ShownConfig::Best, "Always Best");
            for config in &best.configs {
                ui.selectable_value(
                    &mut shown,
                    ShownConfig::Index(config.idx),
                    format!("{}, {:.02}", config.idx, config.score),
                );
            }

            if shown != *shown_config {
                *shown_config = shown;
            }
        });

        ui.collapsing("Optimization Goals", |ui| {
            let mut settings = solver.0.clone();

            let mut updated = false;

            let text_width = 200.0;

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.mes_linear.0, "MES Linear");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.mes_linear.0,
                        Slider::new(&mut settings.mes_linear.1, -5.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.mes_torque.0, "MES Torque");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.mes_torque.0,
                        Slider::new(&mut settings.mes_torque.1, -5.0..=1.0),
                    )
                    .changed();
            });

            ui.collapsing("MES Linear Goals", |ui| {
                match settings.mes_linear_type {
                    MesType::AtLeast => {
                        if ui.button("At least").clicked() {
                            settings.mes_linear_type = MesType::Equal;
                            updated = true;
                        }
                    }
                    MesType::Equal => {
                        if ui.button("Equal").clicked() {
                            settings.mes_linear_type = MesType::AtLeast;
                            updated = true;
                        }
                    }
                }

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_x_off.0, "X");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_x_off.0,
                            Slider::new(&mut settings.mes_x_off.1, 0.0..=70.0),
                        )
                        .changed();
                });

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_y_off.0, "Y");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_y_off.0,
                            Slider::new(&mut settings.mes_y_off.1, 0.0..=70.0),
                        )
                        .changed();
                });

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_z_off.0, "Z");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_z_off.0,
                            Slider::new(&mut settings.mes_z_off.1, 0.0..=70.0),
                        )
                        .changed();
                });
            });

            ui.collapsing("MES Torque Goals", |ui| {
                match settings.mes_torque_type {
                    MesType::AtLeast => {
                        if ui.button("At least").clicked() {
                            settings.mes_torque_type = MesType::Equal;
                            updated = true;
                        }
                    }
                    MesType::Equal => {
                        if ui.button("Equal").clicked() {
                            settings.mes_torque_type = MesType::AtLeast;
                            updated = true;
                        }
                    }
                }

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_x_rot_off.0, "X");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_x_rot_off.0,
                            Slider::new(&mut settings.mes_x_rot_off.1, 0.0..=20.0),
                        )
                        .changed();
                });

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_y_rot_off.0, "Y");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_y_rot_off.0,
                            Slider::new(&mut settings.mes_y_rot_off.1, 0.0..=20.0),
                        )
                        .changed();
                });

                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_z_rot_off.0, "Z");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_z_rot_off.0,
                            Slider::new(&mut settings.mes_z_rot_off.1, 0.0..=20.0),
                        )
                        .changed();
                });
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.min_linear.0, "Min Linear");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.min_linear.0,
                        Slider::new(&mut settings.min_linear.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.min_torque.0, "Min Torque");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.min_torque.0,
                        Slider::new(&mut settings.min_torque.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.avg_linear.0, "Avg Linear");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.avg_linear.0,
                        Slider::new(&mut settings.avg_linear.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.avg_torque.0, "Avg Torque");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.avg_torque.0,
                        Slider::new(&mut settings.avg_torque.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.x.0, "X");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(settings.x.0, Slider::new(&mut settings.x.1, 0.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.y.0, "Y");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(settings.y.0, Slider::new(&mut settings.y.1, 0.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.z.0, "Z");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(settings.z.0, Slider::new(&mut settings.z.1, 0.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.x_rot.0, "X ROT");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.x_rot.0,
                        Slider::new(&mut settings.x_rot.1, 0.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.y_rot.0, "Y ROT");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.y_rot.0,
                        Slider::new(&mut settings.y_rot.1, 0.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.z_rot.0, "Z ROT");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.z_rot.0,
                        Slider::new(&mut settings.z_rot.1, 0.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(
                    &mut settings.center_of_mass_loss.0,
                    "Center of Mass offset loss",
                );
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.center_of_mass_loss.0,
                        Slider::new(&mut settings.center_of_mass_loss.1, -1000.0..=0.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.center_loss.0, "AABB center offset loss");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.center_loss.0,
                        Slider::new(&mut settings.center_loss.1, -100.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(
                    &mut settings.surface_area_loss.0,
                    "Force/surface area score",
                );
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.surface_area_loss.0,
                        Slider::new(&mut settings.surface_area_loss.1, 0.0..=1.5),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.dimension_loss.0, "Linear size loss");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.dimension_loss.0,
                        Slider::new(&mut settings.dimension_loss.1, -1000.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.tube_exclusion_radius.0, "Tube radius");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.tube_exclusion_radius.0,
                        Slider::new(&mut settings.tube_exclusion_radius.1, -1.0..=1.0),
                    )
                    .changed();
            });

            // ui.horizontal(|ui| {
            //     let check = ui.checkbox(&mut settings.tube_exclusion_loss.0, "Tube exclusion loss");
            //     let width = check.rect.width();
            //     ui.allocate_space((text_width - width, 0.0).into());
            //
            //     updated |= check.changed();
            //     updated |= ui
            //         .add_enabled(
            //             settings.tube_exclusion_loss.0,
            //             Slider::new(&mut settings.tube_exclusion_loss.1, -100.0..=1.0),
            //         )
            //         .changed();
            // });

            ui.horizontal(|ui| {
                let check =
                    ui.checkbox(&mut settings.thruster_exclusion_radius.0, "Thruster radius");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.thruster_exclusion_radius.0,
                        Slider::new(&mut settings.thruster_exclusion_radius.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(
                    &mut settings.thruster_flow_exclusion_loss.0,
                    "Thruster flow exclusion loss",
                );
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.thruster_flow_exclusion_loss.0,
                        Slider::new(&mut settings.thruster_flow_exclusion_loss.1, -100.0..=0.0),
                    )
                    .changed();
            });

            // ui.horizontal(|ui| {
            //     let check = ui.checkbox(
            //         &mut settings.thruster_exclusion_loss.0,
            //         "Thruster exclusion loss",
            //     );
            //     let width = check.rect.width();
            //     ui.allocate_space((text_width - width, 0.0).into());
            //
            //     updated |= check.changed();
            //     updated |= ui
            //         .add_enabled(
            //             settings.thruster_exclusion_loss.0,
            //             Slider::new(&mut settings.thruster_exclusion_loss.1, -100.0..=1.0),
            //         )
            //         .changed();
            // });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.cardinality_loss.0, "Cardinality loss");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.cardinality_loss.0,
                        Slider::new(&mut settings.cardinality_loss.1, -100.0..=1.0),
                    )
                    .changed();
            });

            if updated {
                commands.insert_resource(ScoreSettingsRes(settings));
            }

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("Physics Result", |ui| {
            let physics_result =
                reverse::axis_maximums(&motor_conf.0.motor_config, &motor_data.0, 25.0, 0.001);
            let physics_result: BTreeMap<_, _> = physics_result.into_iter().collect();
            ui.label(format!("{physics_result:#.2?}"));

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("Unscaled Score Result", |ui| {
            ui.label(format!("{:#.02?}", motor_conf.0.score_result_unscaled));

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("Scaled Score Result", |ui| {
            ui.label(format!(
                "Score: {:.02}",
                motor_conf.0.score_result_scaled.score()
            ));
            ui.label(format!("{:#.02?}", motor_conf.0.score_result_scaled));

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        if ui.button("Print Parameters").clicked() {
            println!("{}", motor_conf.0.parameters);
        }
        // ui.collapsing("Parameters", |ui| {
        //     ui.label(format!("{}", motor_conf.0.parameters));
        //
        //     ui.allocate_space((ui.available_width(), 0.0).into());
        // });
    });

    let enable_cameras = if let Some(response) = response {
        !response.response.contains_pointer()
    } else {
        true
    };

    cameras.iter_mut().for_each(|mut camera| {
        camera.enabled = enable_cameras;
    })
}
