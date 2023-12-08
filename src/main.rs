use std::collections::BTreeMap;

use bevy::{
    core_pipeline::clear_color::{self, ClearColorConfig},
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    ecs::system::SystemChangeTick,
    math::{dvec3, vec3a, DQuat, DVec3, Vec3A},
    prelude::*,
    render::{
        camera::{ScalingMode, Viewport},
        mesh::Indices,
        render_resource::PrimitiveTopology,
        view::RenderLayers,
    },
    window::{PrimaryWindow, WindowResized},
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
    lines::{LineList, LineMaterial},
    physics,
};

const WIDTH: f32 = 0.325;
const LENGTH: f32 = 0.355;
const HEIGHT: f32 = 0.241;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            MaterialPlugin::<LineMaterial>::default(),
            PanOrbitCameraPlugin,
            EguiPlugin,
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
        ))
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
struct LeftCamera;
#[derive(Component)]
struct RightTopCamera;
#[derive(Component)]
struct RightBottomCamera;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    mut materials_line: ResMut<Assets<LineMaterial>>,
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
        &mut materials_line,
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
            transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Z),
            projection: Projection::Orthographic(OrthographicProjection {
                scaling_mode: ScalingMode::WindowSize(250.0),
                ..default()
            }),
            ..default()
        },
        PanOrbitCamera::default(),
        RenderLayers::layer(0),
        LeftCamera,
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Z),
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
        RightTopCamera,
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Z),
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
        RightBottomCamera,
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
    mut motors_query: Query<(&MotorMarker, &mut Handle<Mesh>)>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if motor_conf.is_changed() {
        for (motor_id, mut mesh) in motors_query.iter_mut() {
            let motor = motor_conf.0.motor(&motor_id.0).unwrap();

            *mesh = meshes.add(Mesh::from(LineList {
                lines: vec![(
                    motor.position.into(),
                    (motor.position + motor.orientation).into(),
                )],
            }));
        }
    }
}

fn add_motor_conf(
    motor_conf: &MotorConfig<X3dMotorId>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
    materials_line: &mut ResMut<Assets<LineMaterial>>,

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
            materials_line,
            motor_handle,
        );
    }
}

fn add_motor(
    motor_id: X3dMotorId,
    motor: Motor,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_line: &mut ResMut<Assets<LineMaterial>>,

    motor_handle: &Handle<Scene>,
) {
    commands.spawn((
        MaterialMeshBundle {
            mesh: meshes.add(Mesh::from(LineList {
                lines: vec![(
                    motor.position.into(),
                    (motor.position + motor.orientation).into(),
                )],
            })),
            material: materials_line.add(LineMaterial {
                color: Color::GREEN,
            }),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        MotorMarker(motor_id),
        RenderLayers::layer(0),
    ));

    // commands.spawn((
    //     SceneBundle {
    //         scene: motor_handle.clone(),
    //         transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
    //             * Transform::from_translation(motor.position.as_vec3())
    //                 .looking_to(motor.orientation.as_vec3(), -motor.position.as_vec3()),
    //
    //         ..default()
    //     },
    //     MotorMarker(motor_id),
    //     RenderLayers::layer(0),
    // ));
}

fn set_camera_viewports(
    windows: Query<&Window>,
    mut resize_events: EventReader<WindowResized>,
    mut left_camera: Query<&mut Camera, With<LeftCamera>>,
    mut right_top_camera: Query<&mut Camera, (With<RightTopCamera>, Without<LeftCamera>)>,
    mut right_bottom_camera: Query<
        &mut Camera,
        (
            With<RightBottomCamera>,
            Without<LeftCamera>,
            Without<RightTopCamera>,
        ),
    >,
) {
    // We need to dynamically resize the camera's viewports whenever the window size changes
    // so then each camera always takes up half the screen.
    // A resize_event is sent when the window is first created, allowing us to reuse this system for initial setup.
    for resize_event in resize_events.iter() {
        let window = windows.get(resize_event.window).unwrap();
        let mut left_camera = left_camera.single_mut();
        left_camera.viewport = Some(Viewport {
            physical_position: UVec2::new(0, 0),
            physical_size: UVec2::new(
                window.resolution.physical_width() / 2,
                window.resolution.physical_height(),
            ),
            ..default()
        });

        let mut right_top_camera = right_top_camera.single_mut();
        right_top_camera.viewport = Some(Viewport {
            physical_position: UVec2::new(window.resolution.physical_width() / 2, 0),
            physical_size: UVec2::new(
                window.resolution.physical_width() / 2,
                window.resolution.physical_height() / 2,
            ),
            ..default()
        });

        let mut right_bottom_camera = right_bottom_camera.single_mut();
        right_bottom_camera.viewport = Some(Viewport {
            physical_position: UVec2::new(
                window.resolution.physical_width() / 2,
                window.resolution.physical_height() / 2,
            ),
            physical_size: UVec2::new(
                window.resolution.physical_width() / 2,
                window.resolution.physical_height() / 2,
            ),
            ..default()
        });
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
                force: point,
                torque: Vec3A::ZERO,
            },
            StrengthMeshType::Torque => Movement {
                force: Vec3A::ZERO,
                torque: point,
            },
        };

        let mut forces = reverse::reverse_solve(movement, motor_config);

        let force_length = forces.values().map(|it| it * it).sum::<f32>();
        let adjustment = 1.0 / force_length;
        forces.values_mut().for_each(|it| *it *= adjustment);

        let adjusted_movement = forward::forward_solve(motor_config, &forces);

        match mesh_type {
            StrengthMeshType::Force => adjusted_movement.force.dot(movement.force).abs(),
            StrengthMeshType::Torque => adjusted_movement.torque.dot(movement.torque).abs(),
        }
    });

    let raw_points = generated.raw_points();
    let raw_data = generated.raw_data();

    let points = raw_points
        .iter()
        .zip(raw_data.iter())
        .map(|(&p, &scale)| (p * scale).into())
        .collect::<Vec<[f32; 3]>>();

    let mut indices = Vec::with_capacity(generated.indices_per_main_triangle() * 20);

    for i in 0..20 {
        generated.get_indices(i, &mut indices);
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
