use bevy::{
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use hexasphere::shapes::IcoSphere;
use motor_math::{
    motor_preformance::MotorData, solve::reverse, ErasedMotorId, FloatType, MotorConfig, Movement,
};
use nalgebra::{vector, Vector3};

#[derive(Component)]
pub enum HeuristicMesh {
    Positive,
    Negative,
}
#[derive(Component, Clone, Copy)]
pub enum StrengthMesh {
    Force,
    Torque,
}

pub fn make_strength_mesh(
    motor_config: &MotorConfig<ErasedMotorId, FloatType>,
    motor_data: &MotorData,
    mesh_type: StrengthMesh,
) -> Mesh {
    let generated = IcoSphere::new(5, |point| {
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
        // let ratio = 1.0;

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

// fn make_heuristic_meshes(score_settings: &ScoreSettings, motor_data: &MotorData) -> (Mesh, Mesh) {
//     let positive = IcoSphere::new(20, |point| {
//         let motor_config = MotorConfig::<X3dMotorId, FloatType>::new(
//             Motor {
//                 position: vector![WIDTH, LENGTH, HEIGHT] / 2.0,
//                 orientation: Vector3::from(point.normalize()).cast::<FloatType>(),
//                 direction: Direction::Clockwise,
//             },
//             vector![0.0, 0.0, 0.0],
//         );
//
//         let score = optimize::evaluate(&motor_config, score_settings, motor_data).0;
//
//         score.clamp(0.0, 10.0) as f32 * 0.3
//     });
//
//     let negative = IcoSphere::new(20, |point| {
//         let motor_config = MotorConfig::<X3dMotorId, FloatType>::new(
//             Motor {
//                 position: vector![WIDTH, LENGTH, HEIGHT] / 2.0,
//                 orientation: Vector3::from(point.normalize()).cast::<FloatType>(),
//                 direction: Direction::Clockwise,
//             },
//             vector![0.0, 0.0, 0.0],
//         );
//
//         let score = optimize::evaluate(&motor_config, score_settings, motor_data).0;
//
//         score.clamp(-10.0, 0.0).abs() as f32 * 0.3
//     });
//
//     (iso_sphere_to_mesh(positive), iso_sphere_to_mesh(negative))
// }

pub fn iso_sphere_to_mesh(obj: IcoSphere<f32>) -> Mesh {
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
