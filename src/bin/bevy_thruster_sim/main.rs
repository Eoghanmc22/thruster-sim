pub mod camera;
pub mod mesh;
pub mod motor_config;
pub mod optimizer;

#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
use std::panic;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    render::{camera::ScalingMode, view::RenderLayers},
    window::{PresentMode, Window},
};
use bevy_egui::EguiPlugin;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use camera::{set_camera_viewports, sync_cameras, CameraPos};
use motor_config::{add_motor_conf, update_motor_conf, AxisGizmo, MotorConfigRes, ThrustGizmo};
use motor_math::{
    motor_preformance::{self, MotorData},
    x3d::X3dMotorId,
    Direction, FloatType, Motor, MotorConfig,
};
use nalgebra::{vector, DMatrix};
use optimizer::{gui::render_gui, handle_reset, OptimizerStatus, ShownConfig, TopConfigs};
use optimizer::{handle_heuristic_change, step_accent_points, OptimizerArenaRes, ScoreSettingsRes};
use optimizer::{settings::ToggleableScoreSettings, ResetEvent};
use thruster_sim::optimize::symetrical::SymerticalOptimization;
use thruster_sim::optimize::{AsyncOptimizationArena, OptimizationOutput};
use thruster_sim::{HEIGHT, LENGTH, WIDTH};

#[derive(Resource)]
pub struct MotorDataRes(pub MotorData);

fn main() {
    #[cfg(not(all(target_arch = "wasm32", target_os = "unknown")))]
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");
    #[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
    let motor_data = {
        panic::set_hook(Box::new(console_error_panic_hook::hook));
        motor_preformance::read_motor_data_from_string(include_str!("../../../motor_data.csv"))
            .expect("Read motor data")
    };

    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoNoVsync, // Reduces input lag.
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            PanOrbitCameraPlugin,
            EguiPlugin,
            // LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin,
        ))
        .insert_gizmo_config(
            AxisGizmo,
            GizmoConfig {
                render_layers: RenderLayers::from_layers(&[0, 1, 2, 3]),
                ..default()
            },
        )
        .init_gizmo_group::<ThrustGizmo>()
        .insert_resource(ScoreSettingsRes(ToggleableScoreSettings::default()))
        .insert_resource(OptimizerArenaRes(Box::new(AsyncOptimizationArena::new(
            // FullOptimization::<6>,
            SymerticalOptimization::<3>,
            // FixedX3dOptimization {
            //     width: WIDTH / 2.0,
            //     length: LENGTH / 2.0,
            //     height: HEIGHT / 2.0,
            // },
        ))))
        .insert_resource(MotorDataRes(motor_data))
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(ShownConfig::Best)
        .insert_resource(OptimizerStatus::Running)
        .insert_resource(TopConfigs { configs: vec![] })
        .add_event::<ResetEvent>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                render_gui,
                update_motor_conf,
                set_camera_viewports,
                sync_cameras,
                handle_heuristic_change,
                handle_reset,
                step_accent_points,
                // screenshot_on_tab,
                // auto_generate_constraints.before(sync_cameras),
                // toggle_auto_gen_on_space,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut ambiant: ResMut<AmbientLight>,
    motor_data: Res<MotorDataRes>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
) {
    let motor_conf = MotorConfig::<X3dMotorId, FloatType>::new(
        Motor {
            position: vector![WIDTH, LENGTH, HEIGHT] / 2.0,
            orientation: vector![-0.254, 0.571, -0.781].normalize(),
            direction: Direction::Clockwise,
        },
        vector![0.0, 0.0, 0.0],
    )
    .erase();

    add_motor_conf(
        &motor_conf,
        &motor_data,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
    );
    commands.insert_resource(MotorConfigRes(OptimizationOutput {
        idx: 0,
        motor_config: motor_conf.clone(),
        score: 0.0,
        parameters: DMatrix::default(),
        score_result_unscaled: Default::default(),
        score_result_scaled: Default::default(),
    }));

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
        RenderLayers::from_layers(&[0, 1, 2, 3]),
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

    // let (positive, negative) = make_heuristic_meshes(&score_settings.0.flatten(), &motor_data.0);
    //
    // commands.spawn((
    //     PbrBundle {
    //         mesh: meshes.add(positive),
    //         material: materials_pbr.add(Color::srgb(0.4, 0.8, 0.3)),
    //         transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
    //         ..default()
    //     },
    //     HeuristicMesh::Positive,
    //     RenderLayers::layer(3),
    // ));
    //
    // commands.spawn((
    //     PbrBundle {
    //         mesh: meshes.add(negative),
    //         material: materials_pbr.add(Color::srgb(0.8, 0.4, 0.3)),
    //         transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
    //         ..default()
    //     },
    //     HeuristicMesh::Negative,
    //     RenderLayers::layer(3),
    // ));

    commands.add(|world: &mut World| {
        world.send_event(ResetEvent);
    });
}
