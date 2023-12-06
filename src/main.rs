use std::collections::BTreeMap;

use bevy::{
    core_pipeline::clear_color::{self, ClearColorConfig},
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{dvec3, DQuat, DVec3},
    prelude::*,
    render::{
        camera::{ScalingMode, Viewport},
        mesh::Indices,
        render_resource::PrimitiveTopology,
        view::RenderLayers,
    },
    window::WindowResized,
};
use bevy_egui::{
    egui::{self, Sense, Slider},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use hexasphere::shapes::IcoSphere;
use random_math_test::{
    lines::{LineList, LineMaterial},
    motor_code::{self, MotorData, MotorId},
    physics, Motor, MotorConfig, SeedAngle,
};

fn main() {
    let motor_data = motor_code::read_motor_data().unwrap();

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
            (render_gui, update_motor_conf, set_camera_viewports),
        )
        .insert_resource(MotorDataRes(motor_data))
        .run();
}

#[derive(Resource)]
struct MotorConfigRes(MotorConfig);
#[derive(Resource)]
struct MotorDataRes(MotorData);
#[derive(Component)]
struct MotorMarker(MotorId);

#[derive(Component)]
struct LeftCamera;
#[derive(Component)]
struct RightCamera;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    mut materials_line: ResMut<Assets<LineMaterial>>,
    mut motor_data: Res<MotorDataRes>,
    asset_server: Res<AssetServer>,
) {
    let motor_handle = asset_server.load("t200.gltf#Scene0");

    let motor_conf = MotorConfig {
        seed: SeedAngle::Vec(dvec3(0.254, -0.571, 0.781).normalize()),
        // seed: SeedAngle::Vec(
        //     DQuat::from_euler(EulerRot::XZY, 45f64.to_radians(), -45f64.to_radians(), 0.0)
        //         * DVec3::X,
        // ),
        // seed: SeedAngle::VecByTwoAngles {
        //     angle_xy: 135f64.to_radians(),
        //     angle_yz: 45f64.to_radians(),
        // },
        width: 0.37,
        length: 0.45,
        height: 0.19,
    };
    add_motor_conf(
        &motor_conf,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
        &mut materials_line,
        &motor_handle,
    );
    commands.insert_resource(MotorConfigRes(motor_conf));

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
        RightCamera,
    ));

    let result = physics(&motor_conf, &motor_data.0);
    let result: BTreeMap<_, _> = result.into_iter().collect();

    println!("{:+.3?}, {result:+.3?}", motor_conf.seed);
}

fn render_gui(
    mut contexts: EguiContexts,
    mut motor_conf: ResMut<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    mut cameras: Query<&mut PanOrbitCamera>,
) {
    let response = egui::Window::new("Motor Config").show(contexts.ctx_mut(), |ui| {
        if let SeedAngle::VecByTwoAngles { angle_xy, angle_yz } = &mut motor_conf.0.seed {
            ui.horizontal(|ui| {
                let mut angle = angle_xy.to_degrees();

                ui.label("angle_xy");
                if ui.add(Slider::new(&mut angle, 0.0..=360.0)).changed() {
                    *angle_xy = angle.to_radians();
                }
            });
            ui.horizontal(|ui| {
                let mut angle = angle_yz.to_degrees();

                ui.label("angle_yz");
                if ui.add(Slider::new(&mut angle, 0.0..=360.0)).changed() {
                    *angle_yz = angle.to_radians();
                }
            });
        }

        let physics_result = physics(&motor_conf.0, &motor_data.0);
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
        let motors = motor_conf.0.compute_motors();

        for (motor_id, mut mesh) in motors_query.iter_mut() {
            let motor = motors.get(&motor_id.0).unwrap();

            *mesh = meshes.add(Mesh::from(LineList {
                lines: vec![(
                    motor.position.as_vec3(),
                    (motor.position + motor.orientation).as_vec3(),
                )],
            }));
        }
    }
}

fn add_motor_conf(
    motor_conf: &MotorConfig,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
    materials_line: &mut ResMut<Assets<LineMaterial>>,

    motor_handle: &Handle<Scene>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(
                shape::Box::new(
                    motor_conf.width as f32,
                    motor_conf.length as f32,
                    motor_conf.height as f32,
                )
                .into(),
            ),
            material: materials_pbr.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(0),
    ));

    let motors = motor_conf.compute_motors();
    for (motor_id, motor) in motors {
        add_motor(
            motor_id,
            motor,
            commands,
            meshes,
            materials_line,
            motor_handle,
        );
    }
}

fn add_motor(
    motor_id: MotorId,
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
                    motor.position.as_vec3(),
                    (motor.position + motor.orientation).as_vec3(),
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
    mut left_camera: Query<&mut Camera, (With<LeftCamera>, Without<RightCamera>)>,
    mut right_camera: Query<&mut Camera, With<RightCamera>>,
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

        let mut right_camera = right_camera.single_mut();
        right_camera.viewport = Some(Viewport {
            physical_position: UVec2::new(window.resolution.physical_width() / 2, 0),
            physical_size: UVec2::new(
                window.resolution.physical_width() / 2,
                window.resolution.physical_height(),
            ),
            ..default()
        });
    }
}

fn make_strength_mesh() -> Mesh {
    let generated = IcoSphere::new(20, |point| {
        let inclination = point.y.acos();
        let azimuth = point.z.atan2(point.x);

        let norm_inclination = inclination / std::f32::consts::PI;
        let norm_azimuth = 0.5 - (azimuth / std::f32::consts::TAU);

        [norm_azimuth, norm_inclination]
    });

    let raw_points = generated.raw_points();

    let points = raw_points
        .iter()
        .map(|&p| (p * 1.0).into())
        .collect::<Vec<[f32; 3]>>();

    let normals = raw_points
        .iter()
        .copied()
        .map(Into::into)
        .collect::<Vec<[f32; 3]>>();

    let uvs = generated.raw_data().to_owned();

    let mut indices = Vec::with_capacity(generated.indices_per_main_triangle() * 20);

    for i in 0..20 {
        generated.get_indices(i, &mut indices);
    }

    let indices = Indices::U32(indices);

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_indices(Some(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, points);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh
}
