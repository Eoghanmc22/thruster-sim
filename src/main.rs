use std::collections::BTreeMap;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{dvec3, DQuat, DVec3},
    prelude::*,
    render::camera::ScalingMode,
};
use bevy_egui::{
    egui::{self, Sense, Slider},
    EguiContexts, EguiPlugin,
};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
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
        .add_systems(Update, (render_gui, update_motor_conf))
        .insert_resource(MotorDataRes(motor_data))
        .run();
}

#[derive(Resource)]
struct MotorConfigRes(MotorConfig);
#[derive(Resource)]
struct MotorDataRes(MotorData);
#[derive(Component)]
struct MotorMarker(MotorId);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
    mut materials_line: ResMut<Assets<LineMaterial>>,
    mut motor_data: Res<MotorDataRes>,
) {
    let motor_conf = MotorConfig {
        seed: SeedAngle::Vec(dvec3(0.381, -0.530, 0.758).normalize()),
        // seed: SeedAngle::Vec(
        //     DQuat::from_euler(EulerRot::XZY, 45f64.to_radians(), -45f64.to_radians(), 0.0)
        //         * DVec3::X,
        // ),
        // seed: SeedAngle::VecByTwoAngles {
        //     angle_xy: 135f64.to_radians(),
        //     angle_yz: 45f64.to_radians(),
        // },
        width: 0.325,
        length: 0.355,
        height: 0.241,
    };
    add_motor_conf(
        &motor_conf,
        &mut commands,
        &mut meshes,
        &mut materials_pbr,
        &mut materials_line,
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
) {
    commands.spawn(PbrBundle {
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
    });

    let motors = motor_conf.compute_motors();
    for (motor_id, motor) in motors {
        add_motor(motor_id, motor, commands, meshes, materials_line);
    }
}

fn add_motor(
    motor_id: MotorId,
    motor: Motor,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_line: &mut ResMut<Assets<LineMaterial>>,
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
    ));
}
