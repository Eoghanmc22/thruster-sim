use std::{collections::BTreeMap, time::Duration};

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{vec3, vec3a, Vec3A},
    prelude::*,
    render::{
        camera::{self, ScalingMode, Viewport},
        mesh::{shape::Cylinder, Indices},
        render_asset::RenderAssetUsages,
        render_resource::PrimitiveTopology,
        view::{screenshot::ScreenshotManager, RenderLayers},
    },
    window::{PresentMode, PrimaryWindow, Window, WindowResized, WindowResolution},
};
use bevy_egui::{
    egui::{self, Sense, Slider},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hexasphere::shapes::IcoSphere;
use motor_math::{
    motor_preformance::{self, MotorData},
    solve::{forward, reverse},
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement,
};
use thruster_sim::{
    heuristic::{self, ScoreSettings},
    optimize::{accent_sphere, fibonacci_sphere},
    physics, HEIGHT, LENGTH, WIDTH,
};

fn main() {
    let motor_data = motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoNoVsync, // Reduces input lag.
                    // fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            PanOrbitCameraPlugin,
            EguiPlugin,
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
        ))
        .insert_gizmo_group(
            AxisGizmo,
            GizmoConfig {
                render_layers: RenderLayers::all(),
                ..default()
            },
        )
        .init_gizmo_group::<ThrustGizmo>()
        .insert_resource(ScoreSettingsRes(ToggleableScoreSettings::default()))
        .insert_resource(MotorDataRes(motor_data))
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(PowerCompensated(true))
        .insert_resource(AutoGenerate::Off)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                render_gui,
                update_motor_conf,
                set_camera_viewports,
                sync_cameras,
                handle_heuristic_change,
                step_accent_points,
                screenshot_on_tab,
                auto_generate_constraints.before(sync_cameras),
                toggle_power_compensated_on_enter,
                toggle_auto_gen_on_space,
            ),
        )
        .run();
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct ThrustGizmo;
#[derive(Default, Reflect, GizmoConfigGroup)]
struct AxisGizmo;

#[derive(Resource)]
struct MotorConfigRes(MotorConfig<X3dMotorId>);
#[derive(Resource)]
pub struct MotorDataRes(pub MotorData);
#[derive(Resource)]
pub struct PowerCompensated(pub bool);
#[derive(Resource)]
pub enum AutoGenerate {
    Off,
    Randomize,
    Solve(Duration),
    // In bevy time
    Show(Duration),
}
#[derive(Resource)]
struct ScoreSettingsRes(ToggleableScoreSettings);
#[derive(Component)]
struct MotorMarker(X3dMotorId, bool);
#[derive(Component)]
enum HeuristicMesh {
    Positive,
    Negative,
}
#[derive(Component, Clone, Copy)]
enum StrengthMesh {
    Force,
    Torque,
}

#[derive(Component)]
struct AccentPoint(Vec3A, bool, f32);

#[derive(Component)]
struct CurrentConfig;

#[derive(Component)]
enum CameraPos {
    LeftTop,
    LeftBottom,
    RightTop,
    RightBottom,
}

impl CameraPos {
    pub fn view(&self, window: &WindowResolution) -> Viewport {
        let half_width = window.physical_width() / 2;
        let half_height = window.physical_height() / 2;

        match self {
            CameraPos::LeftTop => Viewport {
                physical_position: UVec2::new(0, 0),
                physical_size: UVec2::new(half_width, half_height),
                ..default()
            },
            CameraPos::LeftBottom => Viewport {
                physical_position: UVec2::new(0, half_height),
                physical_size: UVec2::new(half_width, half_height),
                ..default()
            },
            CameraPos::RightTop => Viewport {
                physical_position: UVec2::new(half_width, 0),
                physical_size: UVec2::new(half_width, half_height),
                ..default()
            },
            CameraPos::RightBottom => Viewport {
                physical_position: UVec2::new(half_width, half_height),
                physical_size: UVec2::new(half_width, half_height),
                ..default()
            },
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut ambiant: ResMut<AmbientLight>,
    motor_data: Res<MotorDataRes>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    score_settings: Res<ScoreSettingsRes>,
    power_compensated: Res<PowerCompensated>,
) {
    let motor_conf = MotorConfig::<X3dMotorId>::new(
        Motor {
            position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
            orientation: vec3a(-0.254, 0.571, -0.781).normalize(),
            direction: Direction::Clockwise,
        },
        vec3a(0.0, 0.0, 0.0),
    );

    add_motor_conf(
        &motor_conf,
        &motor_data,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
        power_compensated.0,
    );
    commands.insert_resource(MotorConfigRes(motor_conf.clone()));

    // light
    ambiant.brightness = 150.0;
    commands.spawn((
        PointLightBundle {
            point_light: PointLight {
                intensity: 3_000_000.0,
                shadows_enabled: false,
                ..default()
            },
            transform: Transform::from_xyz(4.0, 8.0, 4.0),
            ..default()
        },
        RenderLayers::all(),
    ));

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-0.375, 0.25, 0.625).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(0),
        CameraPos::LeftTop,
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-0.375, 0.25, 0.625).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 1,
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(1),
        CameraPos::RightTop,
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-0.375, 0.25, 0.625).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 2,
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(2),
        CameraPos::RightBottom,
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-0.375, 0.25, 0.625).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 3,
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(3),
        CameraPos::LeftBottom,
    ));

    let (positive, negative) = make_heuristic_meshes(&score_settings.0.flatten(), &motor_data.0);

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(positive),
            material: materials_pbr.add(Color::rgb(0.4, 0.8, 0.3)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        HeuristicMesh::Positive,
        RenderLayers::layer(3),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(negative),
            material: materials_pbr.add(Color::rgb(0.8, 0.4, 0.3)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        HeuristicMesh::Negative,
        RenderLayers::layer(3),
    ));
}

fn render_gui(
    mut commands: Commands,
    mut contexts: EguiContexts,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    solver: Res<ScoreSettingsRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        ui.set_width(250.0);

        ui.collapsing("Optimization Goals", |ui| {
            let mut settings = solver.0.clone();

            let mut updated = false;

            let text_width = 100.0;

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.mes_linear.0, "MES Linear");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(
                        settings.mes_linear.0,
                        Slider::new(&mut settings.mes_linear.1, -1.0..=1.0),
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
                        Slider::new(&mut settings.mes_torque.1, -1.0..=1.0),
                    )
                    .changed();
            });

            ui.collapsing("MES Linear Goals", |ui| {
                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_x_off.0, "X");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_x_off.0,
                            Slider::new(&mut settings.mes_x_off.1, -3.0..=3.0),
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
                            Slider::new(&mut settings.mes_y_off.1, -3.0..=3.0),
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
                            Slider::new(&mut settings.mes_z_off.1, -3.0..=3.0),
                        )
                        .changed();
                });
            });

            ui.collapsing("MES Torque Goals", |ui| {
                ui.horizontal(|ui| {
                    let check = ui.checkbox(&mut settings.mes_x_rot_off.0, "X");
                    let width = check.rect.width();
                    ui.allocate_space((text_width - width, 0.0).into());

                    updated |= check.changed();
                    updated |= ui
                        .add_enabled(
                            settings.mes_x_rot_off.0,
                            Slider::new(&mut settings.mes_x_rot_off.1, -2.0..=2.0),
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
                            Slider::new(&mut settings.mes_y_rot_off.1, -2.0..=2.0),
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
                            Slider::new(&mut settings.mes_z_rot_off.1, -2.0..=2.0),
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
                    .add_enabled(settings.x.0, Slider::new(&mut settings.x.1, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.y.0, "Y");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(settings.y.0, Slider::new(&mut settings.y.1, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let check = ui.checkbox(&mut settings.z.0, "Z");
                let width = check.rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= check.changed();
                updated |= ui
                    .add_enabled(settings.z.0, Slider::new(&mut settings.z.1, -1.0..=1.0))
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
                        Slider::new(&mut settings.x_rot.1, -1.0..=1.0),
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
                        Slider::new(&mut settings.y_rot.1, -1.0..=1.0),
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
                        Slider::new(&mut settings.z_rot.1, -1.0..=1.0),
                    )
                    .changed();
            });

            if updated {
                commands.insert_resource(ScoreSettingsRes(settings));
            }

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("Physics Result", |ui| {
            ui.collapsing("Fast Physics Result", |ui| {
                let physics_result = physics(&motor_conf.0, &motor_data.0, true);
                let physics_result: BTreeMap<_, _> = physics_result.into_iter().collect();
                ui.label(format!("{physics_result:#.2?}"));

                ui.allocate_space((ui.available_width(), 0.0).into());
            });

            let physics_result = physics(&motor_conf.0, &motor_data.0, false);
            let physics_result: BTreeMap<_, _> = physics_result.into_iter().collect();
            ui.label(format!("{physics_result:#.2?}"));

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("FRT Thruster Data", |ui| {
            let frt = motor_conf.0.motor(&X3dMotorId::FrontRightTop).unwrap();
            ui.label(format!("{frt:#.3?}"));

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.interact(
            ui.clip_rect(),
            "I dont know what im doing".into(),
            Sense::hover(),
        )
    });

    let enable_cameras = if let Some(response) = response {
        if let Some(response) = response.inner {
            !response.hovered()
        } else {
            true
        }
    } else {
        true
    };

    cameras.for_each_mut(|mut camera| {
        camera.enabled = enable_cameras;
    })
}

fn update_motor_conf(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    score_settings: Res<ScoreSettingsRes>,
    power_compensated: Res<PowerCompensated>,
    motors_query: Query<(Entity, &MotorMarker)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mesh_query: Query<(&Handle<Mesh>, &StrengthMesh)>,
    highlight_query: Query<Entity, With<CurrentConfig>>,
    mut gizmos_axis: Gizmos<AxisGizmo>,
    mut gizmos_thrust: Gizmos<ThrustGizmo>,
) {
    if motor_conf.is_changed() || power_compensated.is_changed() {
        for (entity, motor_id) in motors_query.iter() {
            let motor = motor_conf.0.motor(&motor_id.0).unwrap();

            if motor_id.1 {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation(
                        (motor.position * 2.0 + motor.orientation / 2.0).into(),
                    )
                    .looking_to(motor.orientation.into(), (-motor.position).into())
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            } else {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation((motor.position * 2.0).into())
                        .looking_to(motor.orientation.into(), (-motor.position).into())
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            }
        }

        for (mesh, mesh_type) in mesh_query.iter() {
            *meshes.get_mut(mesh).unwrap() = make_strength_mesh(
                &motor_conf.0,
                &motor_data.0,
                *mesh_type,
                power_compensated.0,
            );
        }

        let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
            * Transform::from_translation(
                (motor_conf
                    .0
                    .motor(&X3dMotorId::FrontRightTop)
                    .unwrap()
                    .orientation
                    * heuristic::score(
                        &physics(&motor_conf.0, &motor_data.0, true),
                        &score_settings.0.flatten(),
                    )
                    * 0.3)
                    .into(),
            );
        commands.entity(highlight_query.single()).insert(transform);
    }

    gizmos_axis.rect(
        Vec3::ZERO,
        Quat::from_rotation_x(90f32.to_radians()),
        Vec2::splat(5.0),
        Color::GRAY,
    );

    for i in 1..=9 {
        let y = i as f32 / 2.0 - 2.5;

        gizmos_axis.line(
            vec3(-2.5, 0.0, y),
            vec3(2.5, 0.0, y),
            if y != 0.0 { Color::GRAY } else { Color::RED },
        );
    }

    for i in 1..=9 {
        let x = i as f32 / 2.0 - 2.5;

        gizmos_axis.line(
            vec3(x, 0.0, -2.5),
            vec3(x, 0.0, 2.5),
            if x != 0.0 { Color::GRAY } else { Color::GREEN },
        );
    }

    gizmos_axis.line(vec3(0.0, -2.5, 0.0), vec3(0.0, 2.5, 0.0), Color::BLUE);
}

fn add_motor_conf(
    motor_conf: &MotorConfig<X3dMotorId>,
    motor_data: &Res<MotorDataRes>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,

    power_compensated: bool,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(shape::Box::new(WIDTH * 2.0, LENGTH * 2.0, HEIGHT * 2.0)),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(
                Sphere::new(0.05), // shape::Icosphere {
                                   //     radius: 0.05,
                                   //     subdivisions: 20,
                                   // }
                                   // .try_into()
                                   // .unwrap(),
            ),
            material: materials_pbr.add(Color::GRAY),
            ..default()
        },
        CurrentConfig,
        RenderLayers::layer(3),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(
                motor_conf,
                &motor_data.0,
                StrengthMesh::Force,
                power_compensated,
            )),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        StrengthMesh::Force,
        RenderLayers::layer(1),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(
                motor_conf,
                &motor_data.0,
                StrengthMesh::Torque,
                power_compensated,
            )),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        StrengthMesh::Torque,
        RenderLayers::layer(2),
    ));

    for (motor_id, _) in motor_conf.motors() {
        add_motor(*motor_id, commands, meshes, materials_pbr);
    }
}

fn add_motor(
    motor_id: X3dMotorId,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.005,
                height: 1.0,
                ..default()
            }),
            material: materials_pbr.add(Color::GREEN),
            ..default()
        },
        MotorMarker(motor_id, true),
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.15,
                height: 0.15,
                ..default()
            }),
            material: materials_pbr.add(Color::DARK_GRAY),
            ..default()
        },
        MotorMarker(motor_id, false),
        RenderLayers::layer(0),
    ));
}

fn set_camera_viewports(
    windows: Query<&Window>,
    mut resize_events: EventReader<WindowResized>,
    mut cameras: Query<(&mut Camera, &CameraPos)>,
) {
    // We need to dynamically resize the camera's viewports whenever the window size changes
    // so then each camera always takes up half the screen.
    // A resize_event is sent when the window is first created, allowing us to reuse this system for initial setup.
    for resize_event in resize_events.read() {
        let window = windows.get(resize_event.window).unwrap();

        for (mut camera, view) in cameras.iter_mut() {
            camera.viewport = Some(view.view(&window.resolution));
        }
    }
}

fn make_strength_mesh(
    motor_config: &MotorConfig<X3dMotorId>,
    motor_data: &MotorData,
    mesh_type: StrengthMesh,
    power_compensated: bool,
) -> Mesh {
    let generated = IcoSphere::new(20, |point| {
        let movement = match mesh_type {
            StrengthMesh::Force => Movement {
                force: point.normalize(),
                torque: Vec3A::ZERO,
            },
            StrengthMesh::Torque => Movement {
                force: Vec3A::ZERO,
                torque: point.normalize(),
            },
        };

        if power_compensated {
            let forces = reverse::reverse_solve(movement, motor_config);
            let motor_cmds = reverse::forces_to_cmds(forces, motor_config, motor_data);
            let ratio = reverse::binary_search_force_ratio(
                &motor_cmds,
                motor_config,
                motor_data,
                25.0,
                0.01,
            );

            let type_ratio = match mesh_type {
                StrengthMesh::Force => 1.0,
                StrengthMesh::Torque => 3.5,
            };
            let ratio = if ratio > 300.0 / type_ratio {
                0.0
            } else {
                ratio
            };

            ratio * 0.015 * type_ratio
        } else {
            let mut forces = reverse::reverse_solve(movement, motor_config);

            let type_ratio = match mesh_type {
                StrengthMesh::Force => 1.0,
                StrengthMesh::Torque => 3.5,
            };

            let force_length = forces.values().map(|it| it * it).sum::<f32>();
            let adjustment = type_ratio * 0.458 / force_length.sqrt();
            forces.values_mut().for_each(|it| *it *= adjustment);

            let adjusted_movement = forward::forward_solve(motor_config, &forces);

            adjusted_movement.force.dot(movement.force).abs()
                + adjusted_movement.torque.dot(movement.torque).abs()
        }
    });

    iso_sphere_to_mesh(generated)
}

fn make_heuristic_meshes(score_settings: &ScoreSettings, motor_data: &MotorData) -> (Mesh, Mesh) {
    let positive = IcoSphere::new(20, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(
            Motor {
                position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                orientation: point.normalize(),
                direction: Direction::Clockwise,
            },
            vec3a(0.0, 0.0, 0.0),
        );

        let physics = physics(&motor_config, motor_data, true);
        let score = heuristic::score(&physics, score_settings);

        score.clamp(0.0, 10.0) * 0.3
    });

    let negative = IcoSphere::new(20, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(
            Motor {
                position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                orientation: -point.normalize(),
                direction: Direction::Clockwise,
            },
            vec3a(0.0, 0.0, 0.0),
        );

        let physics = physics(&motor_config, motor_data, true);
        let score = heuristic::score(&physics, score_settings);

        score.clamp(-10.0, 0.0).abs() * 0.3
    });

    (iso_sphere_to_mesh(positive), iso_sphere_to_mesh(negative))
}

fn iso_sphere_to_mesh(obj: IcoSphere<f32>) -> Mesh {
    let raw_points = obj.raw_points();
    let raw_data = obj.raw_data();

    let points = raw_points
        .iter()
        .zip(raw_data.iter())
        .map(|(&p, &scale)| (p * scale).into())
        .collect::<Vec<[f32; 3]>>();

    let mut indices = Vec::with_capacity(obj.indices_per_main_triangle() * 20);

    for i in 0..20 {
        obj.get_indices(i, &mut indices);
    }

    let indices = Indices::U32(indices);

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    );
    mesh.insert_indices(indices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, points);
    mesh.duplicate_vertices();
    mesh.compute_flat_normals();
    mesh
}

fn sync_cameras(
    mut cameras: Query<(&mut Transform, &mut PanOrbitCamera, &Camera)>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    let mut update = None;

    for (transform, camera, view) in cameras.iter_mut() {
        if let (Some(view_port), Some(position)) = (
            view.logical_viewport_rect(),
            windows.single().cursor_position(),
        ) {
            if transform.is_changed() && view_port.contains(position) {
                update = Some((*transform, *camera));
            }
        }
    }

    if let Some((trans, cam)) = update {
        for mut camera in cameras.iter_mut() {
            *camera.0 = trans;
            *camera.1 = cam;
        }
    }
}

fn handle_heuristic_change(
    mut commands: Commands,
    score_settings: Res<ScoreSettingsRes>,
    motor_data: Res<MotorDataRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(&Handle<Mesh>, &HeuristicMesh)>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    points: Query<Entity, With<AccentPoint>>,
) {
    if score_settings.is_changed() {
        let (positive, negative) =
            make_heuristic_meshes(&score_settings.0.flatten(), &motor_data.0);

        for (mesh, mesh_type) in query.iter() {
            match mesh_type {
                HeuristicMesh::Positive => *meshes.get_mut(mesh).unwrap() = positive.clone(),
                HeuristicMesh::Negative => *meshes.get_mut(mesh).unwrap() = negative.clone(),
            }
        }

        for point in &points {
            commands.entity(point).despawn();
        }

        let sphere_points = fibonacci_sphere(100);
        for point in sphere_points {
            commands.spawn((
                PbrBundle {
                    mesh: meshes.add(
                        Sphere::new(0.01),
                        // shape::Icosphere {
                        //     radius: 0.01,
                        //     subdivisions: 1,
                        // }
                        // .try_into()
                        // .unwrap(),
                    ),
                    material: materials_pbr.add(Color::WHITE),
                    ..default()
                },
                AccentPoint(point, false, 0.0),
                RenderLayers::layer(3),
            ));
        }
    }
}

fn step_accent_points(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    mut points: Query<(Entity, &mut AccentPoint)>,
    score_settings: Res<ScoreSettingsRes>,
) {
    points.par_iter_mut().for_each(|(_, mut point)| {
        if !point.1 {
            let (next_point, done) =
                accent_sphere(0.005, point.0, &score_settings.0.flatten(), &motor_data.0);

            let config = MotorConfig::<X3dMotorId>::new(
                Motor {
                    position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                    orientation: point.0.normalize(),
                    direction: Direction::Clockwise,
                },
                vec3a(0.0, 0.0, 0.0),
            );

            let score = heuristic::score(
                &physics(&config, &motor_data.0, true),
                &score_settings.0.flatten(),
            );

            point.0 = next_point;
            point.1 = done;
            point.2 = score
        }
    });

    let mut best: Option<(MotorConfig<X3dMotorId>, f32)> = None;

    for (entity, point) in points.iter() {
        if !point.1 {
            let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                * Transform::from_translation((point.0.normalize() * point.2 * 0.3).into());

            commands.entity(entity).try_insert(transform);
        }

        if Some(point.2) > best.as_ref().map(|it| it.1) {
            let config = MotorConfig::<X3dMotorId>::new(
                Motor {
                    position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                    orientation: point.0.normalize(),
                    direction: Direction::Clockwise,
                },
                vec3a(0.0, 0.0, 0.0),
            );

            best = Some((config, point.2));
        }
    }

    if let Some((best, best_score)) = best {
        let current_score = heuristic::score(
            &physics(&motor_conf.0, &motor_data.0, true),
            &score_settings.0.flatten(),
        );

        if (best_score - current_score).abs() > 0.001 {
            commands.insert_resource(MotorConfigRes(best));
        }
    }
}

#[derive(Clone)]
pub struct ToggleableScoreSettings {
    pub mes_linear: (bool, f32),
    pub mes_x_off: (bool, f32),
    pub mes_y_off: (bool, f32),
    pub mes_z_off: (bool, f32),

    pub mes_torque: (bool, f32),
    pub mes_x_rot_off: (bool, f32),
    pub mes_y_rot_off: (bool, f32),
    pub mes_z_rot_off: (bool, f32),

    pub avg_linear: (bool, f32),
    pub avg_torque: (bool, f32),

    pub min_linear: (bool, f32),
    pub min_torque: (bool, f32),

    pub x: (bool, f32),
    pub y: (bool, f32),
    pub z: (bool, f32),

    pub x_rot: (bool, f32),
    pub y_rot: (bool, f32),
    pub z_rot: (bool, f32),
}

impl ToggleableScoreSettings {
    pub fn flatten(&self) -> ScoreSettings {
        ScoreSettings {
            mes_linear: if self.mes_linear.0 {
                self.mes_linear.1
            } else {
                0.0
            },
            mes_x_off: if self.mes_x_off.0 {
                self.mes_x_off.1
            } else {
                -1.0
            },
            mes_y_off: if self.mes_y_off.0 {
                self.mes_y_off.1
            } else {
                -1.0
            },
            mes_z_off: if self.mes_z_off.0 {
                self.mes_z_off.1
            } else {
                -1.0
            },
            mes_torque: if self.mes_torque.0 {
                self.mes_torque.1
            } else {
                0.0
            },
            mes_x_rot_off: if self.mes_x_rot_off.0 {
                self.mes_x_rot_off.1
            } else {
                -1.0
            },
            mes_y_rot_off: if self.mes_y_rot_off.0 {
                self.mes_y_rot_off.1
            } else {
                -1.0
            },
            mes_z_rot_off: if self.mes_z_rot_off.0 {
                self.mes_z_rot_off.1
            } else {
                -1.0
            },
            avg_linear: if self.avg_linear.0 {
                self.avg_linear.1
            } else {
                0.0
            },
            avg_torque: if self.avg_torque.0 {
                self.avg_torque.1
            } else {
                0.0
            },
            min_linear: if self.min_linear.0 {
                self.min_linear.1
            } else {
                0.0
            },
            min_torque: if self.min_torque.0 {
                self.min_torque.1
            } else {
                0.0
            },
            x: if self.x.0 { self.x.1 } else { 0.0 },
            y: if self.y.0 { self.y.1 } else { 0.0 },
            z: if self.z.0 { self.z.1 } else { 0.0 },
            x_rot: if self.x_rot.0 { self.x_rot.1 } else { 0.0 },
            y_rot: if self.y_rot.0 { self.y_rot.1 } else { 0.0 },
            z_rot: if self.z_rot.0 { self.z_rot.1 } else { 0.0 },
        }
    }
}

impl Default for ToggleableScoreSettings {
    fn default() -> Self {
        let base = ScoreSettings::default();

        Self {
            mes_linear: (true, base.mes_linear),
            mes_x_off: (true, base.mes_x_off),
            mes_y_off: (true, base.mes_y_off),
            mes_z_off: (true, base.mes_z_off),
            mes_torque: (true, base.mes_torque),
            mes_x_rot_off: (true, base.mes_x_rot_off),
            mes_y_rot_off: (true, base.mes_y_rot_off),
            mes_z_rot_off: (true, base.mes_z_rot_off),
            avg_linear: (true, base.avg_linear),
            avg_torque: (true, base.avg_torque),
            min_linear: (true, base.min_linear),
            min_torque: (true, base.min_torque),
            x: (true, base.x),
            y: (true, base.y),
            z: (true, base.z),
            x_rot: (true, base.x_rot),
            y_rot: (true, base.y_rot),
            z_rot: (true, base.z_rot),
        }
    }
}

fn screenshot_on_tab(
    input: Res<ButtonInput<KeyCode>>,
    main_window: Query<Entity, With<PrimaryWindow>>,
    mut screenshot_manager: ResMut<ScreenshotManager>,
) {
    if input.just_pressed(KeyCode::Tab) {
        let time =
            time::OffsetDateTime::now_local().unwrap_or_else(|_| time::OffsetDateTime::now_utc());

        let (year, month, day) = (time.year(), time.month() as u8, time.day());
        let (hour, minute, second) = (time.hour(), time.minute(), time.second());

        let path = format!(
            "./screenshot-{year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{second:02}.png",
        );
        screenshot_manager
            .save_screenshot_to_disk(main_window.single(), path)
            .unwrap();
    }
}

fn auto_generate_constraints(
    mut auto_generate: ResMut<AutoGenerate>,
    mut score_settings: ResMut<ScoreSettingsRes>,
    time: Res<Time>,
    points: Query<(Entity, &AccentPoint)>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    match *auto_generate {
        AutoGenerate::Randomize => {
            score_settings.0 = ToggleableScoreSettings {
                mes_linear: (rand::random(), rand::random::<f32>() - 0.5),
                mes_x_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                mes_y_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                mes_z_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                mes_torque: (rand::random(), rand::random::<f32>() - 0.5),
                mes_x_rot_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                mes_y_rot_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                mes_z_rot_off: (rand::random(), rand::random::<f32>() * 2.0 - 1.0),
                avg_linear: (rand::random(), rand::random::<f32>() / 2.0 + 0.15),
                avg_torque: (rand::random(), rand::random()),
                min_linear: (rand::random(), rand::random::<f32>() / 2.0 + 0.15),
                min_torque: (rand::random(), rand::random()),
                x: (rand::random(), rand::random::<f32>() / 2.0 + 0.15),
                y: (rand::random(), rand::random::<f32>() / 2.0 + 0.15),
                z: (rand::random(), rand::random::<f32>() / 2.0 + 0.15),
                x_rot: (rand::random(), rand::random::<f32>() / 2.0 + 0.25),
                y_rot: (rand::random(), rand::random::<f32>() / 2.0 + 0.25),
                z_rot: (rand::random(), rand::random::<f32>() / 2.0 + 0.25),
            };

            *auto_generate = AutoGenerate::Solve(time.elapsed());
        }
        AutoGenerate::Solve(start) => {
            let min_duration = Duration::from_secs_f32(0.75);

            let done = points
                .iter()
                .all(|it| it.1 .1 || rand::random::<f32>() < 0.927);

            if done && time.elapsed() > start + min_duration {
                *auto_generate = AutoGenerate::Show(time.elapsed());
            }
        }
        AutoGenerate::Show(start) => {
            let rotate_duration = Duration::from_secs_f32(5.0);
            let total_duration = rotate_duration + Duration::from_secs_f32(0.5);

            if time.elapsed() < start + rotate_duration {
                for mut camera in &mut cameras {
                    camera.target_yaw +=
                        360f32.to_radians() / rotate_duration.as_secs_f32() * time.delta_seconds();
                    camera.target_yaw %= 360.0
                }
            }

            if time.elapsed() > start + total_duration {
                *auto_generate = AutoGenerate::Randomize;
            }
        }
        AutoGenerate::Off => {
            // Do Nothing
        }
    }
}

fn toggle_power_compensated_on_enter(
    mut power_compensated: ResMut<PowerCompensated>,
    input: Res<ButtonInput<KeyCode>>,
) {
    if input.just_pressed(KeyCode::Enter) {
        power_compensated.0 = !power_compensated.0;
    }
}

fn toggle_auto_gen_on_space(
    mut auto_generate: ResMut<AutoGenerate>,
    input: Res<ButtonInput<KeyCode>>,
) {
    if input.just_pressed(KeyCode::Space) {
        match *auto_generate {
            AutoGenerate::Off => {
                *auto_generate = AutoGenerate::Randomize;
            }
            AutoGenerate::Randomize | AutoGenerate::Solve(_) | AutoGenerate::Show(_) => {
                *auto_generate = AutoGenerate::Off;
            }
        }
    }
}
