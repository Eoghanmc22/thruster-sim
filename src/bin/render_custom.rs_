use std::{collections::BTreeMap, time::Duration};

use bevy::color;
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::vec3,
    prelude::*,
    render::{
        camera::{ScalingMode, Viewport},
        mesh::Indices,
        render_asset::RenderAssetUsages,
        render_resource::PrimitiveTopology,
        view::RenderLayers,
    },
    window::{PresentMode, Window, WindowResized, WindowResolution},
};
use bevy_egui::{
    egui::{self, Sense, Slider},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hexasphere::shapes::IcoSphere;
use motor_math::utils::VectorTransform;
use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, FloatType, Motor, MotorConfig, Movement,
};
use motor_math::{ErasedMotorId, Number};
use nalgebra::{vector, SVector, Vector3};
use thruster_sim::{heuristic::ScoreSettings, optimize, HEIGHT, LENGTH, WIDTH};

fn main() {
    #[cfg(not(all(target_arch = "wasm32", target_os = "unknown")))]
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");
    #[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
    let motor_data = {
        panic::set_hook(Box::new(console_error_panic_hook::hook));
        motor_preformance::read_motor_data_from_string(include_str!("../motor_data.csv"))
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
            LogDiagnosticsPlugin::default(),
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
        .insert_resource(MotorDataRes(motor_data))
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(AutoGenerate::Off)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (render_gui, update_motor_conf, set_camera_viewports),
        )
        .run();
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct ThrustGizmo;
#[derive(Default, Reflect, GizmoConfigGroup)]
struct AxisGizmo;

#[derive(Resource)]
struct MotorConfigRes(MotorConfig<ErasedMotorId, FloatType>);
#[derive(Resource)]
pub struct MotorDataRes(pub MotorData);
#[derive(Resource)]
pub enum AutoGenerate {
    Off,
    Randomize,
    Solve(Duration),
    // In bevy time
    Show(Duration),
}
#[derive(Component)]
struct MotorMarker(ErasedMotorId, bool);
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
) {
    let vector = vector![
        -0.08411990799490855,
        -0.18783187621711067,
        -0.20972502686797223,
        0.9322531685223766,
        0.08575171235180115,
        -0.35149775761549806,
        -0.231881086311625,
        0.29723897420004863,
        -0.0057789112048616245,
        0.2610274925260214,
        0.96525411322843,
        -0.012212495288635333,
        -0.23315361333868193,
        -0.23494563162684876,
        0.038256920330250646,
        0.3655902114005249,
        0.055977593075181076,
        0.9290911184602019,
        0.28885392553829514,
        0.06303859727388472,
        -0.019994632199575983,
        -0.2351876482859487,
        0.9704939554479132,
        -0.053181317516480246,
        -0.030301818091313885,
        0.2976593090356167,
        0.1264479232374007,
        -0.01842271135426574,
        -0.030917417175738333,
        0.9993521486550867,
        0.28914757902742405,
        -0.23475827066480578,
        0.06732347072449481,
        -0.2693148573494942,
        0.02675872054104416,
        0.9626803615353479,
    ];

    let motor_conf = motor_config(vector);

    add_motor_conf(
        &motor_conf,
        &motor_data,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
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
}

fn render_gui(
    mut contexts: EguiContexts,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        ui.set_width(250.0);

        ui.collapsing("Physics Result", |ui| {
            let physics_result = reverse::axis_maximums(&motor_conf.0, &motor_data.0, 25.0, 0.001);
            let physics_result: BTreeMap<_, _> = physics_result.into_iter().collect();
            ui.label(format!("{physics_result:#.2?}"));

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

    cameras.iter_mut().for_each(|mut camera| {
        camera.enabled = enable_cameras;
    })
}

fn update_motor_conf(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    motors_query: Query<(Entity, &MotorMarker)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mesh_query: Query<(&Handle<Mesh>, &StrengthMesh)>,
    mut gizmos_axis: Gizmos<AxisGizmo>,
) {
    if motor_conf.is_changed() {
        for (entity, motor_id) in motors_query.iter() {
            let motor = motor_conf.0.motor(&motor_id.0).unwrap();

            if motor_id.1 {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation(
                        (motor.position * 2.0 + motor.orientation / 2.0)
                            .cast::<f32>()
                            .into(),
                    )
                    .looking_to(
                        Vec3::from(motor.orientation.cast::<f32>()),
                        Vec3::from((-motor.position).cast::<f32>()),
                    )
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            } else {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation((motor.position * 2.0).cast::<f32>().into())
                        .looking_to(
                            Vec3::from(motor.orientation.cast::<f32>()),
                            Vec3::from((-motor.position).cast::<f32>()),
                        )
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            }
        }

        for (mesh, mesh_type) in mesh_query.iter() {
            *meshes.get_mut(mesh).unwrap() =
                make_strength_mesh(&motor_conf.0, &motor_data.0, *mesh_type);
        }
    }

    gizmos_axis.rect(
        Vec3::ZERO,
        Quat::from_rotation_x(90f32.to_radians()),
        Vec2::splat(5.0),
        color::palettes::css::GRAY,
    );

    for i in 1..=9 {
        let y = i as f32 / 2.0 - 2.5;

        gizmos_axis.line(
            vec3(-2.5, 0.0, y),
            vec3(2.5, 0.0, y),
            if y != 0.0 {
                color::palettes::css::GRAY
            } else {
                color::palettes::css::RED
            },
        );
    }

    for i in 1..=9 {
        let x = i as f32 / 2.0 - 2.5;

        gizmos_axis.line(
            vec3(x, 0.0, -2.5),
            vec3(x, 0.0, 2.5),
            if x != 0.0 {
                color::palettes::css::GRAY
            } else {
                color::palettes::css::GREEN
            },
        );
    }

    gizmos_axis.line(
        vec3(0.0, -2.5, 0.0),
        vec3(0.0, 2.5, 0.0),
        color::palettes::css::BLUE,
    );
}

fn add_motor_conf(
    motor_conf: &MotorConfig<ErasedMotorId, FloatType>,
    motor_data: &Res<MotorDataRes>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(
                motor_conf,
                &motor_data.0,
                StrengthMesh::Force,
            )),
            material: materials_pbr.add(Color::srgb(0.8, 0.7, 0.6)),
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
            )),
            material: materials_pbr.add(Color::srgb(0.8, 0.7, 0.6)),
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
    motor_id: ErasedMotorId,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.005,
                half_height: 1.0 / 2.0,
            }),
            material: materials_pbr.add(Color::from(color::palettes::css::GREEN)),
            ..default()
        },
        MotorMarker(motor_id, true),
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.15,
                half_height: 0.15 / 2.0,
            }),
            material: materials_pbr.add(Color::from(color::palettes::css::BLACK)),
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
    motor_config: &MotorConfig<ErasedMotorId, FloatType>,
    motor_data: &MotorData,
    mesh_type: StrengthMesh,
) -> Mesh {
    let generated = IcoSphere::new(10, |point| {
        let movement = match mesh_type {
            StrengthMesh::Force => Movement {
                force: Vector3::from(point.normalize()).cast::<FloatType>(),
                torque: vector![0.0, 0.0, 0.0],
            },
            StrengthMesh::Torque => Movement {
                force: vector![0.0, 0.0, 0.0],
                torque: Vector3::from(point.normalize()).cast::<FloatType>(),
            },
        };

        let forces = reverse::reverse_solve(movement, motor_config);
        let motor_cmds = reverse::forces_to_cmds(forces, motor_config, motor_data);
        let ratio =
            reverse::binary_search_force_ratio(&motor_cmds, motor_config, motor_data, 25.0, 0.001);

        let type_ratio = match mesh_type {
            StrengthMesh::Force => 1.0,
            StrengthMesh::Torque => 3.5,
        };
        let ratio = if ratio > 300.0 / type_ratio {
            0.0
        } else {
            ratio
        };

        (ratio * 0.015 * type_ratio) as f32
    });

    iso_sphere_to_mesh(generated)
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
    // mesh.duplicate_vertices();
    // mesh.compute_flat_normals();
    mesh.compute_smooth_normals();
    mesh
}

// pub const THRUSTER_COUNT: usize = 3;
pub const THRUSTER_COUNT: usize = 6;
pub const DIMENSIONALITY: usize = THRUSTER_COUNT * 6;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

pub fn motor_config<D: Number>(point: Point<D>) -> MotorConfig<ErasedMotorId, D> {
    MotorConfig::<ErasedMotorId, _>::new_raw(
        (0..THRUSTER_COUNT).map(|idx| {
            (
                idx as _,
                Motor {
                    position: point.fixed_rows::<3>(idx * 6).into(),
                    orientation: point.fixed_rows::<3>(idx * 6 + 3).into(),
                    direction: Direction::Clockwise,
                },
            )
        }),
        vector![0.0, 0.0, 0.0].map(D::from),
    )
}

// pub fn motor_config<D: Number>(point: Point<D>) -> MotorConfig<ErasedMotorId, D> {
//     MotorConfig::<ErasedMotorId, _>::new_raw(
//         (0..THRUSTER_COUNT * 2).map(|idx| {
//             if idx < THRUSTER_COUNT {
//                 (
//                     idx as _,
//                     Motor {
//                         position: point.fixed_rows::<3>(idx * 6).into(),
//                         orientation: point.fixed_rows::<3>(idx * 6 + 3).into(),
//                         direction: Direction::Clockwise,
//                     },
//                 )
//             } else {
//                 let vec_idx = idx % THRUSTER_COUNT;
//                 (
//                     idx as _,
//                     Motor {
//                         position: VectorTransform::ReflectYZ
//                             .transform(point.fixed_rows::<3>(vec_idx * 6).into()),
//                         orientation: VectorTransform::ReflectYZ
//                             .transform(point.fixed_rows::<3>(vec_idx * 6 + 3).into()),
//                         direction: Direction::Clockwise,
//                     },
//                 )
//             }
//         }),
//         vector![0.0, 0.0, 0.0].map(D::from),
//     )
// }
