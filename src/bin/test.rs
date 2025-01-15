#![allow(incomplete_features)]
#![feature(lazy_type_alias)]

use std::fs;

use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
};
use nalgebra::{vector, Const, SVector};
use num_dual::{gradient, DualVec};
use thruster_sim::{
    heuristic::{ScoreResult, ScoreSettings},
    optimize,
};

pub const STEP_SIZE: FloatType = 0.000001;
pub const THRUSTER_COUNT: usize = 6;
pub const DIMENSIONALITY: usize = THRUSTER_COUNT * 6;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.01;

pub type Point<D: Number> = SVector<D, DIMENSIONALITY>;

fn main() {
    let motor_data =
        motor_preformance::read_motor_data_from_path("motor_data.csv").expect("Read motor data");

    let vector = vector![
        -0.18274243598960593,
        0.3419019654744739,
        -0.026983697696884124,
        0.43228202708653546,
        0.8992952006254792,
        0.06633544444665322,
        0.283788586396575,
        -0.16999458967593373,
        0.278053643273787,
        -0.5382437292102611,
        0.3668571251518362,
        0.7587552554619712,
        0.11272695517575863,
        0.3415552692494902,
        -0.09373975832339038,
        -0.09709131335043157,
        0.22124459734197205,
        0.9703731782252005,
        0.27583734844192775,
        0.229018374314737,
        -0.09955407686014464,
        -0.28134585009824703,
        0.9088187732304506,
        -0.30804699325978435,
        0.27146601791162234,
        -0.16436025262976178,
        -0.09192875758887374,
        0.8005457838617138,
        0.22363710171218162,
        0.5559792214453787,
        -0.18759702311845758,
        -0.15092452145820776,
        0.27518958108566366,
        0.24447456629088463,
        0.0595979775712273,
        0.9678224359386919,
    ];

    let config = motor_config(vector);
    println!("config: {config:#.04?}");
    println!("matrix: {:.04}", config.matrix);
    println!("inverse: {:.04}", config.pseudo_inverse);
    let maximums = reverse::axis_maximums(&config, &motor_data, 0.5, 0.00001);
    println!("maximums: {maximums:#.04?}");
    let score = optimize::evaluate(&config, &Default::default(), &motor_data);
    println!("score: {score:#.04?}");
}

pub fn initial_points(count: usize) -> Vec<Point<FloatType>> {
    let mut points = vec![];

    for _ in 0..count {
        points.push(Point::from_fn(|_, _| rand::random()));
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

pub fn normalise_point<D: Number>(mut point: Point<D>) -> Point<D> {
    for idx in 0..THRUSTER_COUNT {
        point.fixed_rows_mut::<3>(idx * 6 + 3).normalize_mut();
    }

    point
}

fn gradient_ascent<'a>(
    points: &'a [Point<FloatType>],
    heuristic: &'a ScoreSettings,
    motor_data: &'a MotorData,
    step_size: FloatType,
) -> impl Iterator<Item = Ascent> + use<'a> {
    points.iter().map(move |&old_point| {
        let mut result = None;

        let (score, grad) = gradient(
            |point| {
                let motor_config = motor_config(point);
                let (score, score_breakdown) =
                    optimize::evaluate(&motor_config, heuristic, motor_data);
                result = Some(score_breakdown);

                score
            },
            old_point,
        );

        let delta = step_size * grad;
        let new_point = normalise_point(old_point + delta);
        let delta = new_point - old_point;

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
struct Ascent {
    old_point: Point<FloatType>,
    new_point: Point<FloatType>,

    old_score: FloatType,
    est_new_score: FloatType,

    gradient: Point<FloatType>,
    score_breakdown: ScoreResult<DualVec<FloatType, FloatType, Const<DIMENSIONALITY>>>,
}
