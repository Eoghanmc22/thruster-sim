use motor_math::FloatType;
use motor_math::{
    motor_preformance::{self, MotorData},
    solve::reverse,
    x3d::X3dMotorId,
    Direction, Motor, MotorConfig, Movement, Number,
};
use nalgebra::{vector, Vector3, Vector6};
use num_dual::gradient;
use thruster_sim::heuristic::ScoreSettings;
use thruster_sim::optimize;

const STEP: FloatType = 0.00001;

fn main() {
    let motor_data = motor_preformance::read_motor_data("motor_data.csv").expect("Read motor data");

    let point = vector![0.3, -0.5, -0.2, 0.1, -0.7, 0.4];
    // let point = vector![3.5, 2.3];

    let (old_score, grad) = gradient(|point| func(point, &motor_data), point);

    let new_point = point + STEP * grad;
    // let new_point = new_point.normalize();

    let new_score = func(new_point, &motor_data);

    let secant = (new_score - old_score) / (new_point - point).norm();
    let grad_mag = grad.norm();
    let derv_diff = (secant - grad_mag).abs();
    let diff = new_point - point;
    let p_grad = old_score + grad_mag * STEP;

    println!("New: {new_score} p_grad: {p_grad}, old: {old_score}, secant: {secant}, gradient: {grad_mag}, derv diff: {derv_diff}, point diff: {diff}, grad vec: {grad}");
}

fn func<D: Number>(point: Vector6<D>, motor_data: &MotorData) -> D {
    let motor = Motor {
        orientation: Vector3::from(point.fixed_rows(0)),
        position: Vector3::from(point.fixed_rows(3)),
        // orientation: vector![D::from(0.3), D::from(-0.5), D::from(-0.2)],
        // position: vector![D::from(0.1), D::from(0.7), D::from(0.4)],
        direction: Direction::Clockwise,
    };

    let motor_config =
        MotorConfig::<X3dMotorId, _>::new(motor, vector![0.0.into(), 0.0.into(), 0.0.into()]);

    optimize::evaluate(&motor_config, &ScoreSettings::default(), motor_data)
}
