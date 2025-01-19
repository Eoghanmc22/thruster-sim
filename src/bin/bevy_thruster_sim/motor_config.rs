use bevy::{color, math::vec3, prelude::*, render::view::RenderLayers};
use motor_math::{ErasedMotorId, FloatType, Motor, MotorConfig};
use thruster_sim::optimize::OptimizationOutput;

use crate::{
    mesh::{make_strength_mesh, StrengthMesh},
    MotorDataRes,
};

#[derive(Resource)]
pub struct MotorConfigRes(pub OptimizationOutput);
#[derive(Component)]
pub struct MotorMarker(pub ErasedMotorId, pub bool);

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct ThrustGizmo;
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct AxisGizmo;

// TODO: Seperate gizmos into seperate file
pub fn update_motor_conf(
    mut commands: Commands,
    motor_conf: Res<MotorConfigRes>,
    motor_data: Res<MotorDataRes>,
    motors_query: Query<Entity, With<MotorMarker>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mesh_query: Query<(&Handle<Mesh>, &StrengthMesh)>,
    mut gizmos_axis: Gizmos<AxisGizmo>,
    mut materials_pbr: ResMut<Assets<StandardMaterial>>,
) {
    if motor_conf.is_changed() {
        for entity in motors_query.iter() {
            commands.entity(entity).despawn();
        }
        for (motor_id, motor) in motor_conf.0.motor_config.motors() {
            add_motor(
                *motor_id,
                motor,
                &mut commands,
                &mut meshes,
                &mut materials_pbr,
            );
        }

        for (mesh, mesh_type) in mesh_query.iter() {
            *meshes.get_mut(mesh).unwrap() =
                make_strength_mesh(&motor_conf.0.motor_config, &motor_data.0, *mesh_type);
        }

        // let transform = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
        //     * Transform::from_translation(
        //         (motor_conf
        //             .0
        //             .motor_config
        //             .motor(&X3dMotorId::FrontRightTop)
        //             .unwrap()
        //             .orientation
        //             * optimize::evaluate(
        //                 &motor_conf.0.motor_config,
        //                 &score_settings.0.flatten(),
        //                 &motor_data.0,
        //             )
        //             .0
        //             * 0.3)
        //             .cast::<f32>()
        //             .into(),
        //     );
        // commands.entity(highlight_query.single()).insert(transform);
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

pub fn add_motor_conf(
    motor_conf: &MotorConfig<ErasedMotorId, FloatType>,
    motor_data: &Res<MotorDataRes>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder::new(0.1, 0.76)),
            material: materials_pbr.add(Color::srgb(0.8, 0.7, 0.6)),
            transform: Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians())),
            ..default()
        },
        RenderLayers::layer(0),
    ));

    // commands.spawn((
    //     PbrBundle {
    //         mesh: meshes.add(Sphere::new(0.05)),
    //         material: materials_pbr.add(Color::from(color::palettes::css::GRAY)),
    //         ..default()
    //     },
    //     CurrentConfig,
    //     RenderLayers::layer(3),
    // ));

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

    // for (motor_id, _) in motor_conf.motors() {
    //     add_motor(*motor_id, commands, meshes, materials_pbr);
    // }
}

pub fn add_motor(
    motor_id: ErasedMotorId,
    motor: &Motor<FloatType>,

    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials_pbr: &mut ResMut<Assets<StandardMaterial>>,
) {
    let transform_normal = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
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

    let transform_thruster = Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()))
        * Transform::from_translation((motor.position * 2.0).cast::<f32>().into()).looking_to(
            Vec3::from(motor.orientation.cast::<f32>()),
            Vec3::from((-motor.position).cast::<f32>()),
        )
        * Transform::from_rotation(Quat::from_rotation_x(90f32.to_radians()));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.005,
                half_height: 1.0 / 2.0,
            }),
            material: materials_pbr.add(Color::from(color::palettes::css::GREEN)),
            transform: transform_normal,
            ..default()
        },
        MotorMarker(motor_id, true),
        RenderLayers::layer(0),
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder {
                radius: 0.1,
                half_height: 0.05,
            }),
            material: materials_pbr.add(Color::from(color::palettes::css::BLACK)),
            transform: transform_thruster,

            ..default()
        },
        MotorMarker(motor_id, false),
        RenderLayers::layer(0),
    ));
}
