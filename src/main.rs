use std::collections::BTreeMap;

use bevy::{
    core_pipeline::clear_color::ClearColorConfig,
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{vec3, vec3a, Vec3A},
    prelude::*,
    render::{
        camera::{ScalingMode, Viewport},
        mesh::{shape::Cylinder, Indices},
        render_resource::PrimitiveTopology,
        view::RenderLayers,
    },
    window::{PrimaryWindow, WindowResized, WindowResolution},
};
use bevy_egui::{
    egui::{self, Sense},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hexasphere::shapes::IcoSphere;
use motor_math::{
    solve::{forward, reverse},
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement,
};
use random_math_test::{heuristic, physics};

const WIDTH: f32 = 0.325 * 2.0;
const LENGTH: f32 = 0.355 * 2.0;
const HEIGHT: f32 = 0.241 * 2.0;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PanOrbitCameraPlugin,
            EguiPlugin,
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
        ))
        .insert_resource(GizmoConfig {
            render_layers: RenderLayers::all(),
            ..default()
        })
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                render_gui,
                update_motor_conf,
                set_camera_viewports,
                sync_cameras,
            ),
        )
        .run();
}

#[derive(Resource)]
struct MotorConfigRes(MotorConfig<X3dMotorId>);
#[derive(Component)]
struct MotorMarker(X3dMotorId);

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
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let motor_handle = asset_server.load("t200.gltf#Scene0");

    let motor_conf = MotorConfig::<X3dMotorId>::new(Motor {
        position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
        orientation: vec3a(0.254, -0.571, 0.781).normalize(),
        direction: Direction::Clockwise,
    });

    add_motor_conf(
        &motor_conf,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
        &motor_handle,
    );
    commands.insert_resource(MotorConfigRes(motor_conf.clone()));

    // light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: false,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-4.5, 3.0, 7.5).looking_at(Vec3::ZERO, Vec3::Z),
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
            transform: Transform::from_xyz(-4.5, 3.0, 7.5).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 1,
                ..default()
            },
            camera_3d: Camera3d {
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
            transform: Transform::from_xyz(-4.5, 3.0, 7.5).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 2,
                ..default()
            },
            camera_3d: Camera3d {
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
            transform: Transform::from_xyz(-4.5, 3.0, 7.5).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            camera: Camera {
                order: 3,
                ..default()
            },
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(3),
        CameraPos::LeftBottom,
    ));

    let (positive, negative) = make_heuristic_meshes();

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(positive),
            material: materials_pbr.add(Color::rgb(0.4, 0.8, 0.3).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(3),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(negative),
            material: materials_pbr.add(Color::rgb(0.8, 0.4, 0.3).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(3),
    ));

    let result = physics(&motor_conf);
    let result: BTreeMap<_, _> = result.into_iter().collect();

    println!(
        "{:+.3?}, {result:+.3?}",
        motor_conf.motor(&X3dMotorId::FrontRightTop)
    );
}

fn render_gui(
    mut contexts: EguiContexts,
    mut motor_conf: ResMut<MotorConfigRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        // if let SeedAngle::VecByTwoAngles { angle_xy, angle_yz } = &mut motor_conf.0.seed {
        //     ui.horizontal(|ui| {
        //         let mut angle = angle_xy.to_degrees();
        //
        //         ui.label("angle_xy");
        //         if ui.add(Slider::new(&mut angle, 0.0..=360.0)).changed() {
        //             *angle_xy = angle.to_radians();
        //         }
        //     });
        //     ui.horizontal(|ui| {
        //         let mut angle = angle_yz.to_degrees();
        //
        //         ui.label("angle_yz");
        //         if ui.add(Slider::new(&mut angle, 0.0..=360.0)).changed() {
        //             *angle_yz = angle.to_radians();
        //         }
        //     });
        // }

        let physics_result = physics(&motor_conf.0);
        let physics_result: BTreeMap<_, _> = physics_result.into_iter().collect();
        ui.label(format!("{physics_result:#.2?}"));

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
    motor_conf: ResMut<MotorConfigRes>,
    mut motors_query: Query<(&MotorMarker)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut gizmos: Gizmos,
    time: Res<Time>,
) {
    // if motor_conf.is_changed() {
    for (motor_id) in motors_query.iter_mut() {
        let motor = motor_conf.0.motor(&motor_id.0).unwrap();

        // *mesh = meshes.add(Mesh::from(LineList {
        //     lines: vec![(
        //         motor.position.into(),
        //         (motor.position + motor.orientation).into(),
        //     )],
        // }));

        // let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));
        // gizmos.line(
        //     transform * Vec3::from(motor.position),
        //     transform * Vec3::from(motor.position + motor.orientation),
        //     Color::GREEN,
        // );
    }

    gizmos.rect(
        Vec3::ZERO,
        Quat::from_rotation_x(90f32.to_radians()),
        Vec2::splat(5.0),
        Color::DARK_GRAY,
    );

    for i in 1..=9 {
        let y = i as f32 / 2.0 - 2.5;

        gizmos.line(
            vec3(-2.5, 0.0, y),
            vec3(2.5, 0.0, y),
            if y != 0.0 {
                Color::DARK_GRAY
            } else {
                Color::RED
            },
        );
    }

    for i in 1..=9 {
        let x = i as f32 / 2.0 - 2.5;

        gizmos.line(
            vec3(x, 0.0, -2.5),
            vec3(x, 0.0, 2.5),
            if x != 0.0 {
                Color::DARK_GRAY
            } else {
                Color::GREEN
            },
        );
    }

    gizmos.line(vec3(0.0, -2.5, 0.0), vec3(0.0, 2.5, 0.0), Color::BLUE);
}

fn add_motor_conf(
    motor_conf: &MotorConfig<X3dMotorId>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,

    motor_handle: &Handle<Scene>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(shape::Box::new(WIDTH, LENGTH, HEIGHT).into()),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(motor_conf, StrengthMeshType::Force)),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(1),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(motor_conf, StrengthMeshType::Torque)),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(2),
    ));

    for (motor_id, motor) in motor_conf.motors() {
        add_motor(
            *motor_id,
            *motor,
            commands,
            meshes,
            materials_pbr,
            motor_handle,
        );
    }
}

fn add_motor(
    motor_id: X3dMotorId,
    motor: Motor,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,

    motor_handle: &Handle<Scene>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(
                Cylinder {
                    radius: 0.005,
                    height: 1.0,
                    ..default()
                }
                .into(),
            ),
            material: materials_pbr.add(Color::GREEN.into()),
            // transform: Transform::from_rotation(
            //     Quat::from_rotation_x(90f32.to_radians())
            //         * Quat::from_rotation_arc(Vec3::X, motor.orientation.into()),
            // )
            // .with_translation(
            //     Quat::from_rotation_x(90f32.to_radians())
            //         * Vec3::from(motor.position + motor.orientation / 2.0),
            // ),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                * Transform::from_translation((motor.position + motor.orientation / 2.0).into())
                    .looking_to(motor.orientation.into(), (-motor.position).into())
                * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        MotorMarker(motor_id),
        RenderLayers::layer(0),
    ));

    commands.spawn((
        SceneBundle {
            scene: motor_handle.clone(),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                * Transform::from_translation(motor.position.into())
                    .looking_to(motor.orientation.into(), (-motor.position).into())
                    .with_scale(Vec3::splat(2.5)),

            ..default()
        },
        MotorMarker(motor_id),
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

enum StrengthMeshType {
    Force,
    Torque,
}

fn make_strength_mesh(motor_config: &MotorConfig<X3dMotorId>, mesh_type: StrengthMeshType) -> Mesh {
    let generated = IcoSphere::new(50, |point| {
        let movement = match mesh_type {
            StrengthMeshType::Force => Movement {
                force: point.normalize(),
                torque: Vec3A::ZERO,
            },
            StrengthMeshType::Torque => Movement {
                force: Vec3A::ZERO,
                torque: point.normalize(),
            },
        };

        let mut forces = reverse::reverse_solve(movement, motor_config);

        let force_length = forces.values().map(|it| it * it).sum::<f32>();
        let adjustment = 0.5 / force_length.sqrt();
        forces.values_mut().for_each(|it| *it *= adjustment);

        let adjusted_movement = forward::forward_solve(motor_config, &forces);

        match mesh_type {
            StrengthMeshType::Force => adjusted_movement.force.dot(movement.force).abs(),
            StrengthMeshType::Torque => adjusted_movement.torque.dot(movement.torque).abs(),
        }
    });

    iso_sphere_to_mesh(generated)
}

fn make_heuristic_meshes() -> (Mesh, Mesh) {
    let positive = IcoSphere::new(60, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
            orientation: point.normalize(),
            direction: Direction::Clockwise,
        });

        let physics = physics(&motor_config);
        let score = heuristic::score(&physics);

        score.clamp(0.0, 10.0) * 0.3
    });

    let negative = IcoSphere::new(60, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
            orientation: -point.normalize(),
            direction: Direction::Clockwise,
        });

        let physics = physics(&motor_config);
        let score = heuristic::score(&physics);

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

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_indices(Some(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, points);
    mesh.duplicate_vertices();
    mesh.compute_flat_normals();
    mesh
}

fn sync_cameras(
    mut cameras: Query<(&mut Transform, &mut PanOrbitCamera, &Camera), With<Camera3d>>,
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
