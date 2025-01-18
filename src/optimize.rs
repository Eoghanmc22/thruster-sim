use itertools::Itertools;
use motor_math::{
    motor_preformance::MotorData, solve::reverse, ErasedMotorId, FloatType, MotorConfig, Number,
};
use nalgebra::{vector, Const, DMatrix, SMatrix, Vector3};
use num_dual::{gradient, DualVec};
use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
use std::{fmt::Debug, hash::Hash, iter};

use crate::heuristic::{score, Scaled, ScoreResult, ScoreSettings, Unscaled};

pub fn fibonacci_sphere(samples: usize) -> impl Iterator<Item = Vector3<FloatType>> {
    iter::from_coroutine(
        #[coroutine]
        move || {
            let phi = (core::f64::consts::PI * (5f64.sqrt() - 1.0)) as FloatType; // golden angle in radians

            for i in 0..samples {
                let y = 1.0 - (i as FloatType / (samples as FloatType - 1.0)) * 2.0; // y goes from 1 to -1
                let radius = FloatType::sqrt(1.0 - y * y); // radius at y

                let theta = phi * i as FloatType; // golden angle increment

                let x = theta.cos() * radius;
                let z = theta.sin() * radius;

                yield vector![x, y, z];
            }
        },
    )
}

pub fn evaluate<MotorId: Debug + Ord + Hash + Clone, D: Number>(
    motor_config: &MotorConfig<MotorId, D>,
    settings: &ScoreSettings,
    motor_data: &MotorData,
) -> (D, ScoreResult<D, Unscaled>) {
    // TODO: Use 25 amps / make consistant with other calls
    let result = reverse::axis_maximums(motor_config, motor_data, 25.0, 0.001);
    score(&result, motor_config, settings)
}

// Adam without weight decay
pub fn adam_optimizer<const DIM1: usize, const DIM2: usize, Config>(
    old_point: &OptimizationState<Config::Point<FloatType>>,
    config: &Config,
    heuristic: &ScoreSettings,
    motor_data: &MotorData,
    step_size: FloatType,
    frontier_ratio_threshold: FloatType,
) -> Ascent<DIM1, DIM2>
where
    // This feels wrong
    Config: OptimizableConfig<Point<FloatType> = SMatrix<FloatType, DIM1, DIM2>>
        + OptimizableConfig<
            Point<DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>> = SMatrix<
                DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>,
                DIM1,
                DIM2,
            >,
        > + 'static,
{
    let old_point = old_point.clone();

    let mut result = None;

    let (score, grad) = gradient(
        |point| {
            let motor_config = config.motor_config(point);
            let (score, score_breakdown) = evaluate(&motor_config, heuristic, motor_data);
            result = Some(score_breakdown);

            score
        },
        old_point.point,
    );

    let grad = Config::Point::<FloatType>::from_column_slice(grad.as_slice());

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

    let mut frontier_threshold = old_point.frontier_threshold;
    if (frontier_threshold.0 * frontier_ratio_threshold).abs() < score.abs() {
        frontier_threshold = (score, new_time);
    }

    let new_point = OptimizationState {
        point: config.normalise_point::<FloatType>(new_point),
        first_moment: new_first_moment,
        second_moment: new_second_moment,
        time: new_time,
        frontier_threshold,
        ..old_point
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
}

#[derive(Debug, Clone)]
pub struct Ascent<const DIM1: usize, const DIM2: usize> {
    pub old_point: OptimizationState<SMatrix<FloatType, DIM1, DIM2>>,
    pub new_point: OptimizationState<SMatrix<FloatType, DIM1, DIM2>>,

    pub old_score: FloatType,
    pub est_new_score: FloatType,

    pub gradient: SMatrix<FloatType, DIM1, DIM2>,
    pub score_breakdown:
        ScoreResult<DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>, Unscaled>,
}

#[derive(Debug, Clone)]
pub struct OptimizationState<Point> {
    pub point: Point,

    pub first_moment: Point,
    pub second_moment: Point,

    pub frontier_threshold: (FloatType, i32),
    pub time: i32,
    pub done: bool,
}

impl<const DIM1: usize, const DIM2: usize> OptimizationState<SMatrix<FloatType, DIM1, DIM2>> {
    pub fn new(point: SMatrix<FloatType, DIM1, DIM2>) -> Self {
        Self {
            point,
            first_moment: SMatrix::zeros(),
            second_moment: SMatrix::zeros(),
            time: 0,
            frontier_threshold: (FloatType::NEG_INFINITY, 0),
            done: false,
        }
    }
}

pub trait OptimizableConfig {
    const DIMENSIONALITY: usize;

    type MotorId: Ord + Debug + Clone + Hash;
    type Point<D: Number>;

    fn initial_points(
        &self,
        count: usize,
    ) -> impl Iterator<Item = OptimizationState<Self::Point<FloatType>>>;
    fn motor_config<D: Number>(&self, point: Self::Point<D>) -> MotorConfig<Self::MotorId, D>;
    fn normalise_point<D: Number>(&self, point: Self::Point<D>) -> Self::Point<D>;
}

pub mod x3d_fixed {
    use motor_math::{x3d::X3dMotorId, Direction, FloatType, Motor, MotorConfig, Number};
    use nalgebra::{vector, SVector};

    use super::{OptimizableConfig, OptimizationState};

    pub struct FixedX3dOptimization {
        pub width: FloatType,
        pub length: FloatType,
        pub height: FloatType,
    }

    impl OptimizableConfig for FixedX3dOptimization {
        const DIMENSIONALITY: usize = 3;

        type MotorId = X3dMotorId;
        type Point<D: Number> = SVector<D, { Self::DIMENSIONALITY }>;

        fn initial_points(
            &self,
            count: usize,
        ) -> impl Iterator<Item = OptimizationState<Self::Point<FloatType>>> {
            super::fibonacci_sphere(count).map(OptimizationState::new)
        }

        fn motor_config<D: Number>(&self, point: Self::Point<D>) -> MotorConfig<Self::MotorId, D> {
            MotorConfig::<Self::MotorId, _>::new(
                Motor {
                    position: vector![self.width, self.length, self.height].map(D::from),
                    orientation: point,
                    direction: Direction::Clockwise,
                },
                vector![0.0, 0.0, 0.0].map(D::from),
            )
        }

        fn normalise_point<D: Number>(&self, point: Self::Point<D>) -> Self::Point<D> {
            point.normalize()
        }
    }
}

pub mod x3d_dyn {
    use motor_math::{x3d::X3dMotorId, Direction, FloatType, Motor, MotorConfig, Number};
    use nalgebra::{vector, Const, Matrix3x2, SVector, U1};

    use super::{OptimizableConfig, OptimizationState};

    pub struct DynamicX3dOptimization;

    impl OptimizableConfig for DynamicX3dOptimization {
        const DIMENSIONALITY: usize = 6;

        type MotorId = X3dMotorId;
        type Point<D: Number> = SVector<D, { Self::DIMENSIONALITY }>;

        fn initial_points(
            &self,
            count: usize,
        ) -> impl Iterator<Item = OptimizationState<Self::Point<FloatType>>> {
            super::fibonacci_sphere(count)
                .map(|dir| {
                    let pos = SVector::<FloatType, 3>::from_fn(|_, _| rand::random());
                    Matrix3x2::from_columns(&[pos, dir])
                        .reshape_generic(Const::<{ Self::DIMENSIONALITY }>, U1)
                })
                .map(OptimizationState::new)
        }

        fn motor_config<D: Number>(&self, point: Self::Point<D>) -> MotorConfig<Self::MotorId, D> {
            MotorConfig::<Self::MotorId, _>::new(
                Motor {
                    position: point.fixed_rows::<3>(0).into(),
                    orientation: point.fixed_rows::<3>(3).into(),
                    direction: Direction::Clockwise,
                },
                vector![0.0, 0.0, 0.0].map(D::from),
            )
        }

        fn normalise_point<D: Number>(&self, point: Self::Point<D>) -> Self::Point<D> {
            point.normalize()
        }
    }
}

pub mod symetrical {
    use motor_math::{
        utils::VectorTransform, Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
    };
    use nalgebra::{vector, SMatrix};

    use super::{OptimizableConfig, OptimizationState};

    pub struct SymerticalOptimization<const HALF_THRUSTER_COUNT: usize>;

    impl<const HALF_THRUSTER_COUNT: usize> OptimizableConfig
        for SymerticalOptimization<{ HALF_THRUSTER_COUNT }>
    {
        const DIMENSIONALITY: usize = HALF_THRUSTER_COUNT * 6;

        type MotorId = ErasedMotorId;
        type Point<D: Number> = SMatrix<D, 6, { HALF_THRUSTER_COUNT }>;

        fn initial_points(
            &self,
            count: usize,
        ) -> impl Iterator<Item = OptimizationState<Self::Point<FloatType>>> {
            (0..count)
                .map(|_| Self::Point::<FloatType>::from_fn(|_, _| rand::random()))
                .map(OptimizationState::new)
        }

        fn motor_config<D: Number>(&self, point: Self::Point<D>) -> MotorConfig<Self::MotorId, D> {
            MotorConfig::<ErasedMotorId, _>::new_raw(
                (0..HALF_THRUSTER_COUNT * 2).map(|idx| {
                    if idx < HALF_THRUSTER_COUNT {
                        (
                            idx as _,
                            Motor {
                                position: point.fixed_view::<3, 1>(0, idx).into(),
                                orientation: point.fixed_view::<3, 1>(3, idx).into(),
                                direction: Direction::Clockwise,
                            },
                        )
                    } else {
                        let vec_idx = idx % HALF_THRUSTER_COUNT;
                        (
                            idx as _,
                            Motor {
                                position: VectorTransform::ReflectYZ
                                    .transform(point.fixed_view::<3, 1>(0, vec_idx).into()),
                                orientation: VectorTransform::ReflectYZ
                                    .transform(point.fixed_view::<3, 1>(3, vec_idx).into()),
                                direction: Direction::Clockwise,
                            },
                        )
                    }
                }),
                vector![0.0, 0.0, 0.0].map(D::from),
            )
        }

        fn normalise_point<D: Number>(&self, mut point: Self::Point<D>) -> Self::Point<D> {
            for idx in 0..HALF_THRUSTER_COUNT {
                point.fixed_view_mut::<3, 1>(3, idx).normalize_mut();
            }

            point
        }
    }
}

pub mod full {
    use motor_math::{
        utils::VectorTransform, Direction, ErasedMotorId, FloatType, Motor, MotorConfig, Number,
    };
    use nalgebra::{vector, SMatrix};

    use super::{OptimizableConfig, OptimizationState};

    pub struct FullOptimization<const THRUSTER_COUNT: usize>;

    impl<const THRUSTER_COUNT: usize> OptimizableConfig for FullOptimization<{ THRUSTER_COUNT }> {
        const DIMENSIONALITY: usize = THRUSTER_COUNT * 6;

        type MotorId = ErasedMotorId;
        type Point<D: Number> = SMatrix<D, 6, { THRUSTER_COUNT }>;

        fn initial_points(
            &self,
            count: usize,
        ) -> impl Iterator<Item = OptimizationState<Self::Point<FloatType>>> {
            (0..count)
                .map(|_| Self::Point::<FloatType>::from_fn(|_, _| rand::random()))
                .map(OptimizationState::new)
        }

        fn motor_config<D: Number>(&self, point: Self::Point<D>) -> MotorConfig<Self::MotorId, D> {
            MotorConfig::<ErasedMotorId, _>::new_raw(
                (0..THRUSTER_COUNT).map(|idx| {
                    if idx < THRUSTER_COUNT {
                        (
                            idx as _,
                            Motor {
                                position: point.fixed_view::<3, 1>(0, idx).into(),
                                orientation: point.fixed_view::<3, 1>(3, idx).into(),
                                direction: Direction::Clockwise,
                            },
                        )
                    } else {
                        let vec_idx = idx % THRUSTER_COUNT;
                        (
                            idx as _,
                            Motor {
                                position: VectorTransform::ReflectYZ
                                    .transform(point.fixed_view::<3, 1>(0, vec_idx).into()),
                                orientation: VectorTransform::ReflectYZ
                                    .transform(point.fixed_view::<3, 1>(3, vec_idx).into()),
                                direction: Direction::Clockwise,
                            },
                        )
                    }
                }),
                vector![0.0, 0.0, 0.0].map(D::from),
            )
        }

        fn normalise_point<D: Number>(&self, mut point: Self::Point<D>) -> Self::Point<D> {
            for idx in 0..THRUSTER_COUNT {
                point.fixed_view_mut::<3, 1>(3, idx).normalize_mut();
            }

            point
        }
    }
}

pub trait OptimizationArena {
    fn reset(&mut self, point_count: usize, heuristic: ScoreSettings);
    fn step<'a>(
        &'a mut self,
        motor_data: &MotorData,
    ) -> Box<dyn Iterator<Item = OptimizationOutput> + 'a>;
}

pub struct OptimizationOutput {
    pub score: FloatType,
    pub motor_config: MotorConfig<ErasedMotorId, FloatType>,
    pub parameters: DMatrix<FloatType>,
    pub score_result_unscaled: ScoreResult<FloatType, Unscaled>,
    pub score_result_scaled: ScoreResult<FloatType, Scaled>,
}

pub struct SyncOptimizationArena<Config: OptimizableConfig> {
    config: Config,
    heuristic: ScoreSettings,
    points: Vec<(
        FloatType,
        OptimizationState<Config::Point<FloatType>>,
        ScoreResult<FloatType, Unscaled>,
    )>,

    /// The step size/learn rate
    step_size: FloatType,
    /// The ratio by which a points score must improve to be considered an improvement
    frontier_ratio_threshold: FloatType,
    /// The number of time steps a point must not improve for it to be considered done
    frontier_time_limit: i32,
}

impl<Config: OptimizableConfig> SyncOptimizationArena<Config> {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            heuristic: ScoreSettings::default(),
            points: vec![],
            step_size: 0.01,
            frontier_ratio_threshold: 1.01,
            frontier_time_limit: 25,
        }
    }
}

impl<const DIM1: usize, const DIM2: usize, Config: OptimizableConfig> OptimizationArena
    for SyncOptimizationArena<Config>
where
    Config: OptimizableConfig<Point<FloatType> = SMatrix<FloatType, DIM1, DIM2>>
        + OptimizableConfig<
            Point<DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>> = SMatrix<
                DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>,
                DIM1,
                DIM2,
            >,
        > + 'static,
{
    fn reset(&mut self, point_count: usize, heuristic: ScoreSettings) {
        self.points = self
            .config
            .initial_points(point_count)
            .map(|it| (FloatType::NEG_INFINITY, it, Default::default()))
            .collect_vec();
        self.heuristic = heuristic;
    }

    fn step<'a>(
        &'a mut self,
        motor_data: &MotorData,
    ) -> Box<dyn Iterator<Item = OptimizationOutput> + 'a> {
        for (score, point, breakdown) in &mut self.points {
            if !point.done {
                let ascent = adam_optimizer(
                    point,
                    &self.config,
                    &self.heuristic,
                    motor_data,
                    self.step_size,
                    self.frontier_ratio_threshold,
                );
                *point = ascent.new_point;
                *score = ascent.old_score;
                *breakdown = ascent.score_breakdown.to_float();

                if point.time - point.frontier_threshold.1 > self.frontier_time_limit {
                    point.done = true;
                }
            }
        }

        self.points.sort_by(|a, b| FloatType::total_cmp(&a.0, &b.0));

        Box::new(
            self.points
                .iter()
                .map(|(score, point, breakdown)| OptimizationOutput {
                    score: *score,
                    motor_config: self.config.motor_config(point.point).erase_lossy(),
                    parameters: DMatrix::from_column_slice(DIM1, DIM2, point.point.as_slice()),
                    score_result_unscaled: breakdown.clone(),
                    score_result_scaled: breakdown.scale(&self.heuristic),
                }),
        )
    }
}

pub struct AsyncOptimizationArena<Config: OptimizableConfig> {
    config: Config,
    heuristic: ScoreSettings,
    points: Vec<(
        FloatType,
        OptimizationState<Config::Point<FloatType>>,
        ScoreResult<FloatType, Unscaled>,
    )>,

    /// The step size/learn rate
    step_size: FloatType,
    /// The ratio by which a points score must improve to be considered an improvement
    frontier_ratio_threshold: FloatType,
    /// The number of time steps a point must not improve for it to be considered done
    frontier_time_limit: i32,
}

impl<Config: OptimizableConfig> AsyncOptimizationArena<Config> {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            heuristic: ScoreSettings::default(),
            points: vec![],
            step_size: 0.01,
            frontier_ratio_threshold: 1.01,
            frontier_time_limit: 25,
        }
    }
}

impl<const DIM1: usize, const DIM2: usize, Config: OptimizableConfig> OptimizationArena
    for AsyncOptimizationArena<Config>
where
    Config: OptimizableConfig<Point<FloatType> = SMatrix<FloatType, DIM1, DIM2>>
        + OptimizableConfig<
            Point<DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>> = SMatrix<
                DualVec<FloatType, FloatType, Const<DIM1>, Const<DIM2>>,
                DIM1,
                DIM2,
            >,
        > + Send
        + Sync
        + 'static,
{
    fn reset(&mut self, point_count: usize, heuristic: ScoreSettings) {
        self.points = self
            .config
            .initial_points(point_count)
            .map(|it| (FloatType::NEG_INFINITY, it, Default::default()))
            .collect_vec();
        self.heuristic = heuristic;
    }

    fn step<'a>(
        &'a mut self,
        motor_data: &MotorData,
    ) -> Box<dyn Iterator<Item = OptimizationOutput> + 'a> {
        self.points
            .par_iter_mut()
            .for_each(|(score, point, breakdown)| {
                if !point.done {
                    let ascent = adam_optimizer(
                        point,
                        &self.config,
                        &self.heuristic,
                        motor_data,
                        self.step_size,
                        self.frontier_ratio_threshold,
                    );

                    *point = ascent.new_point;
                    *score = ascent.old_score;
                    *breakdown = ascent.score_breakdown.to_float();

                    if point.time - point.frontier_threshold.1 > self.frontier_time_limit {
                        // point.done = true;
                    }
                }
            });

        self.points
            .sort_by(|a, b| FloatType::total_cmp(&a.0, &b.0).reverse());

        Box::new(
            self.points
                .iter()
                .map(|(score, point, breakdown)| OptimizationOutput {
                    score: *score,
                    motor_config: self.config.motor_config(point.point).erase_lossy(),
                    parameters: DMatrix::from_column_slice(DIM1, DIM2, point.point.as_slice()),
                    score_result_unscaled: breakdown.clone(),
                    score_result_scaled: breakdown.scale(&self.heuristic),
                }),
        )
    }
}
