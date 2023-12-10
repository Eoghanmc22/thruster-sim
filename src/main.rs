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
    window::{PresentMode, PrimaryWindow, Window, WindowResized, WindowResolution},
};
use bevy_egui::{
    egui::{self, Sense, Slider},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hexasphere::shapes::IcoSphere;
use motor_math::{
    solve::{forward, reverse},
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement,
};
use random_math_test::{
    heuristic::{self, ScoreSettings},
    optimize::{accent_sphere, fibonacci_sphere},
    physics, HEIGHT, LENGTH, WIDTH,
};

fn main() {
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
            FrameTimeDiagnosticsPlugin::default(),
        ))
        .insert_resource(GizmoConfig {
            render_layers: RenderLayers::all(),
            ..default()
        })
        .insert_resource(ScoreSettingsRes(ScoreSettings::default()))
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
            ),
        )
        .run();
}

#[derive(Resource)]
struct MotorConfigRes(MotorConfig<X3dMotorId>);
#[derive(Resource)]
struct ScoreSettingsRes(ScoreSettings);
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
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    score_settings: Res<ScoreSettingsRes>,
) {
    let motor_conf = MotorConfig::<X3dMotorId>::new(Motor {
        position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
        orientation: vec3a(0.254, -0.571, 0.781).normalize(),
        direction: Direction::Clockwise,
    });

    add_motor_conf(&motor_conf, &mut commands, &mut meshes, &mut materials_pbr);
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

    let (positive, negative) = make_heuristic_meshes(&score_settings.0);

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(positive),
            material: materials_pbr.add(Color::rgb(0.4, 0.8, 0.3).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        HeuristicMesh::Positive,
        RenderLayers::layer(3),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(negative),
            material: materials_pbr.add(Color::rgb(0.8, 0.4, 0.3).into()),
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
    solver: Res<ScoreSettingsRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        ui.set_width(250.0);

        ui.collapsing("Optimization Goals", |ui| {
            let mut settings = solver.0.clone();

            let mut updated = false;

            let text_width = 70.0;

            ui.horizontal(|ui| {
                let width = ui.label("MES Linear").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.mes_linear, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("MES Torque").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.mes_torque, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Min Linear").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.min_linear, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Min Torque").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.min_torque, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Avg Linear").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.avg_linear, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Avg Torque").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.avg_torque, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("X").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui.add(Slider::new(&mut settings.x, -1.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Y").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui.add(Slider::new(&mut settings.y, -1.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Z").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui.add(Slider::new(&mut settings.z, -1.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("X ROT").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.x_rot, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Y ROT").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.y_rot, -1.0..=1.0))
                    .changed();
            });

            ui.horizontal(|ui| {
                let width = ui.label("Z ROT").rect.width();
                ui.allocate_space((text_width - width, 0.0).into());

                updated |= ui
                    .add(Slider::new(&mut settings.z_rot, -1.0..=1.0))
                    .changed();
            });

            if updated {
                commands.insert_resource(ScoreSettingsRes(settings));
            }

            ui.allocate_space((ui.available_width(), 0.0).into());
        });

        ui.collapsing("Physics Result", |ui| {
            let physics_result = physics(&motor_conf.0);
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
    score_settings: Res<ScoreSettingsRes>,
    motors_query: Query<(Entity, &MotorMarker)>,
    mut meshes: ResMut<Assets<Mesh>>,
    mesh_query: Query<(&Handle<Mesh>, &StrengthMesh)>,
    highlight_query: Query<Entity, With<CurrentConfig>>,
    mut gizmos: Gizmos,
) {
    if motor_conf.is_changed() {
        for (entity, motor_id) in motors_query.iter() {
            let motor = motor_conf.0.motor(&motor_id.0).unwrap();

            if motor_id.1 {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation(
                        (motor.position + motor.orientation / 2.0).into(),
                    )
                    .looking_to(motor.orientation.into(), (-motor.position).into())
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            } else {
                let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
                    * Transform::from_translation((motor.position).into())
                        .looking_to(motor.orientation.into(), (-motor.position).into())
                    * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

                commands.entity(entity).insert(transform);
            }
        }

        for (mesh, mesh_type) in mesh_query.iter() {
            *meshes.get_mut(mesh).unwrap() = make_strength_mesh(&motor_conf.0, *mesh_type);
        }

        let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
            * Transform::from_translation(
                (motor_conf
                    .0
                    .motor(&X3dMotorId::FrontRightTop)
                    .unwrap()
                    .orientation
                    * heuristic::score(&physics(&motor_conf.0), &score_settings.0)
                    * 0.3)
                    .into(),
            );
        commands.entity(highlight_query.single()).insert(transform);
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
            mesh: meshes.add(
                shape::Icosphere {
                    radius: 0.05,
                    subdivisions: 20,
                }
                .try_into()
                .unwrap(),
            ),
            material: materials_pbr.add(Color::GRAY.into()),
            ..default()
        },
        CurrentConfig,
        RenderLayers::layer(3),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(motor_conf, StrengthMesh::Force)),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        StrengthMesh::Force,
        RenderLayers::layer(1),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(make_strength_mesh(motor_conf, StrengthMesh::Torque)),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
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
            mesh: meshes.add(
                Cylinder {
                    radius: 0.005,
                    height: 1.0,
                    ..default()
                }
                .into(),
            ),
            material: materials_pbr.add(Color::GREEN.into()),
            ..default()
        },
        MotorMarker(motor_id, true),
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(
                Cylinder {
                    radius: 0.1,
                    height: 0.1,
                    ..default()
                }
                .into(),
            ),
            material: materials_pbr.add(Color::DARK_GRAY.into()),
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

fn make_strength_mesh(motor_config: &MotorConfig<X3dMotorId>, mesh_type: StrengthMesh) -> Mesh {
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

        let mut forces = reverse::reverse_solve(movement, motor_config);

        let force_length = forces.values().map(|it| it * it).sum::<f32>();
        let adjustment = 0.5 / force_length.sqrt();
        forces.values_mut().for_each(|it| *it *= adjustment);

        let adjusted_movement = forward::forward_solve(motor_config, &forces);

        match mesh_type {
            StrengthMesh::Force => adjusted_movement.force.dot(movement.force).abs(),
            StrengthMesh::Torque => adjusted_movement.torque.dot(movement.torque).abs(),
        }
    });

    iso_sphere_to_mesh(generated)
}

fn make_heuristic_meshes(score_settings: &ScoreSettings) -> (Mesh, Mesh) {
    let positive = IcoSphere::new(20, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
            orientation: point.normalize(),
            direction: Direction::Clockwise,
        });

        let physics = physics(&motor_config);
        let score = heuristic::score(&physics, score_settings);

        score.clamp(0.0, 10.0) * 0.3
    });

    let negative = IcoSphere::new(20, |point| {
        let motor_config = MotorConfig::<X3dMotorId>::new(Motor {
            position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
            orientation: -point.normalize(),
            direction: Direction::Clockwise,
        });

        let physics = physics(&motor_config);
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

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_indices(Some(indices));
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
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(&Handle<Mesh>, &HeuristicMesh)>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    points: Query<Entity, With<AccentPoint>>,
) {
    if score_settings.is_changed() {
        let (positive, negative) = make_heuristic_meshes(&score_settings.0);

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
                        shape::Icosphere {
                            radius: 0.01,
                            subdivisions: 1,
                        }
                        .try_into()
                        .unwrap(),
                    ),
                    material: materials_pbr.add(Color::WHITE.into()),
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
    mut points: Query<(Entity, &mut AccentPoint)>,
    score_settings: Res<ScoreSettingsRes>,
) {
    points.par_iter_mut().for_each(|(_, mut point)| {
        if !point.1 {
            let (next_point, done) = accent_sphere(0.005, point.0, &score_settings.0);

            let config = MotorConfig::<X3dMotorId>::new(Motor {
                position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                orientation: point.0.normalize(),
                direction: Direction::Clockwise,
            });

            let score = heuristic::score(&physics(&config), &score_settings.0);

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
            let config = MotorConfig::<X3dMotorId>::new(Motor {
                position: vec3a(WIDTH, LENGTH, HEIGHT) / 2.0,
                orientation: point.0.normalize(),
                direction: Direction::Clockwise,
            });

            best = Some((config, point.2));
        }
    }

    if let Some((best, best_score)) = best {
        let current_score = heuristic::score(&physics(&motor_conf.0), &score_settings.0);

        if (best_score - current_score).abs() > 0.001 {
            commands.insert_resource(MotorConfigRes(best));
        }
    }
}
