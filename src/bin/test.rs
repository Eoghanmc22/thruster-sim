use motor_math::FloatType;
use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement, Number,
};
use nalgebra::{vector, SVector};
use num_dual::gradient;
use thruster_sim::heuristic::ScoreSettings;
use thruster_sim::{optimize, HEIGHT, LENGTH, WIDTH};

// psuedo inverse explodes at idx 16 for the next 32 indices
fn main() {
    let motor_data = motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

    let point = vector![
        -0.25972495316993033,
        0.9521023058438949,
        -0.16138199375276022
    ];

    let result = gradient_ascent(&point, &Default::default(), &motor_data, STEP_SIZE);

    // let new_point = point + STEP * grad;
    // // let new_point = new_point.normalize();
    //
    // let new_score = func(new_point, &motor_data);
    //
    // let secant = (new_score - old_score) / (new_point - point).norm();
    // let grad_mag = grad.norm();
    // let derv_diff = (secant - grad_mag).abs();
    // let diff = new_point - point;
    // let p_grad = old_score + grad_mag * STEP;
    //
    // println!("New: {new_score} p_grad: {p_grad}, old: {old_score}, secant: {secant}, gradient: {grad_mag}, derv diff: {derv_diff}, point diff: {diff}, grad vec: {grad}");
}

pub const STEP_SIZE: FloatType = 0.001;
pub const DIMENSIONALITY: usize = 3;
pub const CRITICAL_POINT_EPSILON: FloatType = 0.01;
pub type Point<D> = SVector<D, DIMENSIONALITY>;

pub fn initial_points(count: usize) -> Vec<Point<FloatType>> {
    optimize::fibonacci_sphere(count)
}

pub fn motor_config<D: Number>(point: Point<D>) -> MotorConfig<X3dMotorId, D> {
    MotorConfig::<X3dMotorId, _>::new(
        Motor {
            position: (vector![WIDTH, LENGTH, HEIGHT] / 2.0).map(D::from),
            orientation: point,
            direction: Direction::Clockwise,
        },
        vector![0.0, 0.0, 0.0].map(D::from),
    )
}

pub fn normalise_point<D: Number>(point: Point<D>) -> Point<D> {
    point.normalize()
}

fn gradient_ascent(
    &old_point: &Point<FloatType>,
    heuristic: &ScoreSettings,
    motor_data: &MotorData,
    step_size: FloatType,
) -> Ascent {
    let (score, grad) = gradient(
        |point| {
            let motor_config = motor_config(point);
            println!("{motor_config:#?}");
            optimize::evaluate(&motor_config, heuristic, motor_data)
        },
        old_point,
    );

    if grad.norm() > 70.0 {
        println!("point: {old_point}, grad: {grad}");
    }

    let delta = step_size * grad;
    let new_point = normalise_point(old_point + delta);
    let delta = new_point - old_point;

    Ascent {
        old_point,
        new_point,
        old_score: score,
        est_new_score: score + grad.dot(&delta),
        gradient: grad,
    }
}

#[derive(Debug)]
struct Ascent {
    old_point: Point<FloatType>,
    new_point: Point<FloatType>,

    old_score: FloatType,
    est_new_score: FloatType,

    gradient: Point<FloatType>,
}
