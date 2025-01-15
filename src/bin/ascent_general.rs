#![allow(incomplete_features)]
#![feature(lazy_type_alias)]

use std::fs;

use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    utils::VectorTransform,
    Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
};
use nalgebra::{vector, ComplexField, Const, Normed, SVector};
use num_dual::{gradient, DualNum, DualVec};
use thruster_sim::{
    heuristic::{ScoreResult, ScoreSettings},
    optimize,
};

pub const LEARN_RATE: FloatType = 0.01;

// pub const THRUSTER_COUNT: usize = 3;
pub const THRUSTER_COUNT: usize = 6;
pub const DIMENSIONALITY: usize = THRUSTER_COUNT * 6;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.1;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

fn main() {
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");

    let mut points = initial_points(25);
    let mut completed_points = vec![];
    let heuristic = ScoreSettings::default();

    let mut counter = 0;
    while !points.is_empty() {
        if counter % 10 == 0 {
            println!("{counter}");
        }
        let results = gradient_ascent(&points, &heuristic, &motor_data, LEARN_RATE);

        let mut remaining_points = vec![];
        for mut result in results {
            if result.new_point.fountier_threshold.0 * 1.01 < result.old_score {
                result.new_point.fountier_threshold = (result.old_score, counter);
            }

            if result.gradient.norm_squared() < CRITICAL_POINT_EPSILON * CRITICAL_POINT_EPSILON
                || counter - result.new_point.fountier_threshold.1 > 25
            {
                completed_points.push(result);
            } else {
                remaining_points.push(result.new_point);
            }
        }

        points = remaining_points;
        counter += 1;
    }

    completed_points.sort_by(|a, b| FloatType::total_cmp(&a.old_score, &b.old_score).reverse());

    for result in completed_points.iter().take(1) {
        println!("point: {:.04?}", result.new_point);
        println!(
            "score: {:.02}, performance: {:?}",
            result.old_score,
            reverse::axis_maximums(
                &motor_config(result.new_point.point),
                &motor_data,
                1.0,
                0.01
            )
        );
        let _ = fs::write("logs/current.log", format!("{result:#?}"));
    }
}

pub fn initial_points(count: usize) -> Vec<OptimizationPoint> {
    let mut points = vec![];

    for _ in 0..count {
        points.push(OptimizationPoint::new(Point::from_fn(|_, _| {
            rand::random()
        })));
    }

    points
}

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

pub fn normalise_point<D: Number>(mut point: Point<D>) -> Point<D> {
    for idx in 0..THRUSTER_COUNT {
        point.fixed_rows_mut::<3>(idx * 6 + 3).normalize_mut();
    }

    point
}

// Adam without weight decay
fn gradient_ascent<'a>(
    points: &'a [OptimizationPoint],
    heuristic: &'a ScoreSettings,
    motor_data: &'a MotorData,
    step_size: FloatType,
) -> impl Iterator<Item = Ascent> + use<'a> {
    points.iter().map(move |old_point| {
        let old_point = old_point.clone();

        let mut result = None;

        let (score, grad) = gradient(
            |point| {
                let motor_config = motor_config(point);
                let (score, score_breakdown) =
                    optimize::evaluate(&motor_config, heuristic, motor_data);
                result = Some(score_breakdown);

                score
            },
            old_point.point,
        );

        let beta_1 = 0.9;
        let beta_2 = 0.999;
        let epsilon = 1e-10;

        let new_time = old_point.time + 1;
        let new_first_moment = beta_1 * old_point.first_moment + (1.0 - beta_1) * grad;
        let new_second_moment =
            beta_2 * old_point.second_moment + (1.0 - beta_2) * grad.component_mul(&grad);

        let first_moment_hat = new_first_moment / (1.0 - beta_1.powi(new_time));
        let second_moment_hat = new_second_moment / (1.0 - beta_2.powi(new_time));

        let new_point = old_point.point
            + step_size
                * first_moment_hat
                    .component_div(&second_moment_hat.map(|it| it.sqrt()).add_scalar(epsilon));

        let new_point = OptimizationPoint {
            point: normalise_point(new_point),
            first_moment: new_first_moment,
            second_moment: new_second_moment,
            time: new_time,
            ..old_point.clone()
        };

        let delta = new_point.point - old_point.point;

        Ascent {
            old_point,
            new_point,
            old_score: score,
            est_new_score: score + grad.dot(&delta),
            gradient: grad,
            score_breakdown: result.unwrap(),
        }
    })
}

#[derive(Debug, Clone)]
pub struct Ascent {
    pub old_point: OptimizationPoint,
    pub new_point: OptimizationPoint,

    pub old_score: FloatType,
    pub est_new_score: FloatType,

    pub gradient: Point<FloatType>,
    pub score_breakdown: ScoreResult<DualVec<FloatType, FloatType, Const<DIMENSIONALITY>>>,
}

#[derive(Debug, Clone)]
pub struct OptimizationPoint {
    pub point: Point<FloatType>,

    pub first_moment: Point<FloatType>,
    pub second_moment: Point<FloatType>,

    pub fountier_threshold: (FloatType, usize),

    pub time: i32,
}

impl OptimizationPoint {
    pub fn new(point: Point<FloatType>) -> Self {
        Self {
            point,
            first_moment: Point::zeros(),
            second_moment: Point::zeros(),
            time: 0,
            fountier_threshold: (FloatType::NEG_INFINITY, 0),
        }
    }
}
