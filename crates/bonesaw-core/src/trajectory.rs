use nalgebra::{DVector, UnitQuaternion};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::math::{ControlTime, Motion6, SpatialAcceleration6, Transform3, Vec3};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActuatorSample {
    pub time_ns: ControlTime,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub acceleration: Vec<f64>,
}

/// Sample-major structure-of-arrays block. The four flat buffers replace one
/// heap allocation per field per sample and map directly to batch transports.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActuatorSampleBlock {
    pub times_ns: Vec<ControlTime>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub acceleration: Vec<f64>,
    pub dof: usize,
}

impl ActuatorSampleBlock {
    pub fn with_capacity(samples: usize, dof: usize) -> Self {
        Self {
            times_ns: Vec::with_capacity(samples),
            position: Vec::with_capacity(samples.saturating_mul(dof)),
            velocity: Vec::with_capacity(samples.saturating_mul(dof)),
            acceleration: Vec::with_capacity(samples.saturating_mul(dof)),
            dof,
        }
    }

    pub fn len(&self) -> usize {
        self.times_ns.len()
    }

    pub fn is_empty(&self) -> bool {
        self.times_ns.is_empty()
    }

    pub fn time(&self, sample: usize) -> Option<ControlTime> {
        self.times_ns.get(sample).copied()
    }

    pub fn position(&self, sample: usize) -> Option<&[f64]> {
        self.sample_slice(&self.position, sample)
    }

    pub fn velocity(&self, sample: usize) -> Option<&[f64]> {
        self.sample_slice(&self.velocity, sample)
    }

    pub fn acceleration(&self, sample: usize) -> Option<&[f64]> {
        self.sample_slice(&self.acceleration, sample)
    }

    fn sample_slice<'a>(&self, values: &'a [f64], sample: usize) -> Option<&'a [f64]> {
        let start = sample.checked_mul(self.dof)?;
        values.get(start..start.checked_add(self.dof)?)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SegmentLimits {
    pub min_position: f64,
    pub max_position: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_jerk: f64,
}

impl Default for SegmentLimits {
    fn default() -> Self {
        Self {
            min_position: f64::NEG_INFINITY,
            max_position: f64::INFINITY,
            max_velocity: f64::INFINITY,
            max_acceleration: f64::INFINITY,
            max_jerk: f64::INFINITY,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SegmentExtrema {
    pub min_position: Vec<f64>,
    pub max_position: Vec<f64>,
    pub max_abs_velocity: Vec<f64>,
    pub max_abs_acceleration: Vec<f64>,
    pub max_abs_jerk: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct QuinticSegment {
    pub start_time_ns: ControlTime,
    pub duration_ns: i64,
    /// Per-actuator coefficients in normalized time `u ∈ [0, 1]`.
    pub coefficients: Vec<[f64; 6]>,
}

/// Deterministic radial error-growth bounds for a floating-root prediction.
///
/// Translation and attitude are bounded separately because a root attitude
/// error moves each world probe in proportion to its compiled reach from the
/// root. Bounds are isotropic Euclidean radii and grow as
/// `initial + velocity * t + 0.5 * acceleration * t^2`. They are admission
/// evidence supplied by an estimator or experiment contract, not covariance
/// and not a probabilistic confidence claim.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RootPredictionErrorGrowth {
    pub initial_translation_radius_m: f64,
    pub translation_velocity_error_bound_mps: f64,
    pub translation_acceleration_error_bound_mps2: f64,
    pub initial_rotation_radius_rad: f64,
    pub angular_velocity_error_bound_radps: f64,
    pub angular_acceleration_error_bound_radps2: f64,
}

impl RootPredictionErrorGrowth {
    pub fn is_valid(self) -> bool {
        [
            self.initial_translation_radius_m,
            self.translation_velocity_error_bound_mps,
            self.translation_acceleration_error_bound_mps2,
            self.initial_rotation_radius_rad,
            self.angular_velocity_error_bound_radps,
            self.angular_acceleration_error_bound_radps2,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
    }

    pub fn radii_at_seconds(self, seconds_from_start: f64) -> Option<(f64, f64)> {
        if !self.is_valid() || !seconds_from_start.is_finite() || seconds_from_start < 0.0 {
            return None;
        }
        let translation = self.initial_translation_radius_m
            + self.translation_velocity_error_bound_mps * seconds_from_start
            + 0.5 * self.translation_acceleration_error_bound_mps2 * seconds_from_start.powi(2);
        let rotation = self.initial_rotation_radius_rad
            + self.angular_velocity_error_bound_radps * seconds_from_start
            + 0.5 * self.angular_acceleration_error_bound_radps2 * seconds_from_start.powi(2);
        Some((translation, rotation))
    }

    pub fn maximum_radius_growth_rates_at_seconds(
        self,
        seconds_from_start: f64,
    ) -> Option<(f64, f64)> {
        if !self.is_valid() || !seconds_from_start.is_finite() || seconds_from_start < 0.0 {
            return None;
        }
        Some((
            self.translation_velocity_error_bound_mps
                + self.translation_acceleration_error_bound_mps2 * seconds_from_start,
            self.angular_velocity_error_bound_radps
                + self.angular_acceleration_error_bound_radps2 * seconds_from_start,
        ))
    }
}

/// A bounded short-horizon floating-root prediction expressed as a quintic
/// tangent displacement from one pose in the smooth `control_world` frame.
///
/// This is deliberately prediction evidence, not an actuator command.  The
/// six tangent coordinates and their derivatives use the canonical
/// `[angular xyz; linear xyz]` world convention.  Rotation reconstruction is
/// a local Lie-group update, which is appropriate for one WBC horizon and
/// avoids component-wise quaternion interpolation.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct RootPosePredictionSegment {
    pub start_time_ns: ControlTime,
    pub duration_ns: i64,
    #[serde(with = "crate::math::serde_transform3")]
    pub control_world_from_root_at_start: Transform3,
    /// Per-tangent-coordinate coefficients in normalized time `u ∈ [0, 1]`.
    coefficients: [[f64; 6]; 6],
    #[serde(default)]
    pub error_growth: RootPredictionErrorGrowth,
}

impl RootPosePredictionSegment {
    pub fn stationary() -> Self {
        Self {
            start_time_ns: 0,
            duration_ns: 1,
            control_world_from_root_at_start: Transform3::identity(),
            coefficients: [[0.0; 6]; 6],
            error_growth: RootPredictionErrorGrowth::default(),
        }
    }

    pub fn set_error_growth(
        &mut self,
        error_growth: RootPredictionErrorGrowth,
    ) -> Result<(), TrajectoryError> {
        if !error_growth.is_valid() {
            return Err(TrajectoryError::NonFinite);
        }
        self.error_growth = error_growth;
        Ok(())
    }

    pub fn error_radii_at(&self, time_ns: ControlTime) -> Result<(f64, f64), TrajectoryError> {
        let segment_end = self.start_time_ns.saturating_add(self.duration_ns);
        if time_ns < self.start_time_ns || time_ns > segment_end {
            return Err(TrajectoryError::Interval);
        }
        self.error_growth
            .radii_at_seconds((time_ns - self.start_time_ns) as f64 * 1e-9)
            .ok_or(TrajectoryError::NonFinite)
    }

    pub fn maximum_error_radius_growth_rates_between(
        &self,
        start_time_ns: ControlTime,
        end_time_ns: ControlTime,
    ) -> Result<(f64, f64), TrajectoryError> {
        let segment_end = self.start_time_ns.saturating_add(self.duration_ns);
        if start_time_ns < self.start_time_ns
            || end_time_ns > segment_end
            || start_time_ns > end_time_ns
        {
            return Err(TrajectoryError::Interval);
        }
        self.error_growth
            .maximum_radius_growth_rates_at_seconds(
                (end_time_ns - self.start_time_ns) as f64 * 1e-9,
            )
            .ok_or(TrajectoryError::NonFinite)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn set_boundary_conditions(
        &mut self,
        start_time_ns: ControlTime,
        duration_ns: i64,
        control_world_from_root_at_start: Transform3,
        start_twist_world: Motion6,
        start_acceleration_world: SpatialAcceleration6,
        end_tangent_displacement_world: Motion6,
        end_twist_world: Motion6,
        end_acceleration_world: SpatialAcceleration6,
    ) -> Result<(), TrajectoryError> {
        if duration_ns <= 0 {
            return Err(TrajectoryError::Duration);
        }
        if !control_world_from_root_at_start
            .translation
            .vector
            .iter()
            .chain(control_world_from_root_at_start.rotation.coords.iter())
            .chain(start_twist_world.0.iter())
            .chain(start_acceleration_world.0.iter())
            .chain(end_tangent_displacement_world.0.iter())
            .chain(end_twist_world.0.iter())
            .chain(end_acceleration_world.0.iter())
            .all(|value| value.is_finite())
        {
            return Err(TrajectoryError::NonFinite);
        }
        let duration = duration_ns as f64 * 1e-9;
        self.start_time_ns = start_time_ns;
        self.duration_ns = duration_ns;
        self.control_world_from_root_at_start = control_world_from_root_at_start;
        for coordinate in 0..6 {
            let c0 = 0.0;
            let c1 = start_twist_world.0[coordinate] * duration;
            let c2 = 0.5 * start_acceleration_world.0[coordinate] * duration * duration;
            let p = end_tangent_displacement_world.0[coordinate] - (c0 + c1 + c2);
            let v = end_twist_world.0[coordinate] * duration - (c1 + 2.0 * c2);
            let a = end_acceleration_world.0[coordinate] * duration * duration - 2.0 * c2;
            self.coefficients[coordinate] = [
                c0,
                c1,
                c2,
                10.0 * p - 4.0 * v + 0.5 * a,
                -15.0 * p + 7.0 * v - a,
                6.0 * p - 3.0 * v + 0.5 * a,
            ];
        }
        Ok(())
    }

    pub fn evaluate_into(
        &self,
        time_ns: ControlTime,
        control_world_from_root: &mut Transform3,
        twist_world: &mut Motion6,
        acceleration_world: &mut SpatialAcceleration6,
    ) -> Result<(), TrajectoryError> {
        if self.duration_ns <= 0 {
            return Err(TrajectoryError::Duration);
        }
        let duration = self.duration_ns as f64 * 1e-9;
        let u = ((time_ns - self.start_time_ns) as f64 / self.duration_ns as f64).clamp(0.0, 1.0);
        let mut tangent = Motion6::default();
        for coordinate in 0..6 {
            let [c0, c1, c2, c3, c4, c5] = self.coefficients[coordinate];
            tangent.0[coordinate] = ((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u + c0;
            twist_world.0[coordinate] =
                ((((5.0 * c5 * u + 4.0 * c4) * u + 3.0 * c3) * u + 2.0 * c2) * u + c1) / duration;
            acceleration_world.0[coordinate] =
                (((20.0 * c5 * u + 12.0 * c4) * u + 6.0 * c3) * u + 2.0 * c2) / duration.powi(2);
        }
        control_world_from_root.rotation =
            UnitQuaternion::from_scaled_axis(Vec3::new(tangent.0[0], tangent.0[1], tangent.0[2]))
                * self.control_world_from_root_at_start.rotation;
        control_world_from_root.translation.vector =
            self.control_world_from_root_at_start.translation.vector
                + Vec3::new(tangent.0[3], tangent.0[4], tangent.0[5]);
        Ok(())
    }

    pub fn maximum_abs_twist_between_into(
        &self,
        start_time_ns: ControlTime,
        end_time_ns: ControlTime,
        maximum_abs_twist: &mut Motion6,
    ) -> Result<(), TrajectoryError> {
        let segment_end = self.start_time_ns.saturating_add(self.duration_ns);
        if start_time_ns < self.start_time_ns
            || end_time_ns > segment_end
            || start_time_ns > end_time_ns
        {
            return Err(TrajectoryError::Interval);
        }
        let duration = self.duration_ns as f64 * 1e-9;
        let u_start = (start_time_ns - self.start_time_ns) as f64 / self.duration_ns as f64;
        let u_end = (end_time_ns - self.start_time_ns) as f64 / self.duration_ns as f64;
        for coordinate in 0..6 {
            let [_, c1, c2, c3, c4, c5] = self.coefficients[coordinate];
            let velocity = [c1, 2.0 * c2, 3.0 * c3, 4.0 * c4, 5.0 * c5];
            let acceleration = [2.0 * c2, 6.0 * c3, 12.0 * c4, 20.0 * c5, 0.0];
            let roots = roots_unit_interval_fixed(&acceleration, 3);
            let mut maximum = poly_eval_fixed(&velocity, 4, u_start)
                .abs()
                .max(poly_eval_fixed(&velocity, 4, u_end).abs());
            for point in &roots.values[..roots.len] {
                if *point >= u_start && *point <= u_end {
                    maximum = maximum.max(poly_eval_fixed(&velocity, 4, *point).abs());
                }
            }
            maximum_abs_twist.0[coordinate] = maximum / duration;
        }
        Ok(())
    }
}

impl Clone for QuinticSegment {
    fn clone(&self) -> Self {
        Self {
            start_time_ns: self.start_time_ns,
            duration_ns: self.duration_ns,
            coefficients: self.coefficients.clone(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.start_time_ns = source.start_time_ns;
        self.duration_ns = source.duration_ns;
        self.coefficients.clone_from(&source.coefficients);
    }
}

#[derive(Debug, Error)]
pub enum TrajectoryError {
    #[error("trajectory boundary vectors have different dimensions")]
    Dimension,
    #[error("trajectory duration must be positive")]
    Duration,
    #[error("trajectory interval is reversed or outside the segment")]
    Interval,
    #[error("trajectory boundary contains NaN or infinity")]
    NonFinite,
    #[error("sample period must divide the trajectory duration exactly")]
    SamplePeriod,
    #[error(
        "actuator {actuator} violates segment {quantity} limit: observed {observed}, limit {limit}"
    )]
    Limit {
        actuator: usize,
        quantity: &'static str,
        observed: f64,
        limit: f64,
    },
}

impl QuinticSegment {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        start_time_ns: ControlTime,
        duration_ns: i64,
        q0: &DVector<f64>,
        v0: &DVector<f64>,
        a0: &DVector<f64>,
        q1: &DVector<f64>,
        v1: &DVector<f64>,
        a1: &DVector<f64>,
    ) -> Result<Self, TrajectoryError> {
        let mut segment = Self {
            start_time_ns,
            duration_ns,
            coefficients: Vec::with_capacity(q0.len()),
        };
        segment.set_boundary_conditions(start_time_ns, duration_ns, q0, v0, a0, q1, v1, a1)?;
        Ok(segment)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn set_boundary_conditions(
        &mut self,
        start_time_ns: ControlTime,
        duration_ns: i64,
        q0: &DVector<f64>,
        v0: &DVector<f64>,
        a0: &DVector<f64>,
        q1: &DVector<f64>,
        v1: &DVector<f64>,
        a1: &DVector<f64>,
    ) -> Result<(), TrajectoryError> {
        let n = q0.len();
        if [v0.len(), a0.len(), q1.len(), v1.len(), a1.len()]
            .iter()
            .any(|length| *length != n)
        {
            return Err(TrajectoryError::Dimension);
        }
        if duration_ns <= 0 {
            return Err(TrajectoryError::Duration);
        }
        if !q0
            .iter()
            .chain(v0.iter())
            .chain(a0.iter())
            .chain(q1.iter())
            .chain(v1.iter())
            .chain(a1.iter())
            .all(|x| x.is_finite())
        {
            return Err(TrajectoryError::NonFinite);
        }
        let duration = duration_ns as f64 * 1e-9;
        self.start_time_ns = start_time_ns;
        self.duration_ns = duration_ns;
        self.coefficients.clear();
        self.coefficients.reserve(n);
        for index in 0..n {
            // Convert endpoint derivatives from seconds to normalized time.
            let c0 = q0[index];
            let c1 = v0[index] * duration;
            let c2 = 0.5 * a0[index] * duration * duration;
            let p = q1[index] - (c0 + c1 + c2);
            let v = v1[index] * duration - (c1 + 2.0 * c2);
            let a = a1[index] * duration * duration - 2.0 * c2;
            let c3 = 10.0 * p - 4.0 * v + 0.5 * a;
            let c4 = -15.0 * p + 7.0 * v - a;
            let c5 = 6.0 * p - 3.0 * v + 0.5 * a;
            self.coefficients.push([c0, c1, c2, c3, c4, c5]);
        }
        Ok(())
    }

    pub fn evaluate(&self, time_ns: ControlTime) -> ActuatorSample {
        let mut position = Vec::with_capacity(self.coefficients.len());
        let mut velocity = Vec::with_capacity(self.coefficients.len());
        let mut acceleration = Vec::with_capacity(self.coefficients.len());
        position.resize(self.coefficients.len(), 0.0);
        velocity.resize(self.coefficients.len(), 0.0);
        acceleration.resize(self.coefficients.len(), 0.0);
        self.evaluate_into(time_ns, &mut position, &mut velocity, &mut acceleration)
            .expect("fresh evaluation buffers have the segment dimension");
        ActuatorSample {
            time_ns,
            position,
            velocity,
            acceleration,
        }
    }

    /// Evaluate into caller-owned flat buffers. This is the trajectory primitive
    /// used by validation paths that must not allocate once their scratch
    /// buffers have been constructed.
    pub fn evaluate_into(
        &self,
        time_ns: ControlTime,
        position: &mut [f64],
        velocity: &mut [f64],
        acceleration: &mut [f64],
    ) -> Result<(), TrajectoryError> {
        let dof = self.coefficients.len();
        if position.len() != dof || velocity.len() != dof || acceleration.len() != dof {
            return Err(TrajectoryError::Dimension);
        }
        let duration = self.duration_ns as f64 * 1e-9;
        let u = ((time_ns - self.start_time_ns) as f64 / self.duration_ns as f64).clamp(0.0, 1.0);
        for (index, coefficients) in self.coefficients.iter().enumerate() {
            let [c0, c1, c2, c3, c4, c5] = *coefficients;
            position[index] = ((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u + c0;
            velocity[index] =
                ((((5.0 * c5 * u + 4.0 * c4) * u + 3.0 * c3) * u + 2.0 * c2) * u + c1) / duration;
            acceleration[index] =
                (((20.0 * c5 * u + 12.0 * c4) * u + 6.0 * c3) * u + 2.0 * c2) / duration.powi(2);
        }
        Ok(())
    }

    pub fn sample_block(
        &self,
        sample_period_ns: i64,
    ) -> Result<ActuatorSampleBlock, TrajectoryError> {
        if sample_period_ns <= 0 || self.duration_ns % sample_period_ns != 0 {
            return Err(TrajectoryError::SamplePeriod);
        }
        let samples = (self.duration_ns / sample_period_ns) as usize;
        let mut block = ActuatorSampleBlock::with_capacity(samples, self.coefficients.len());
        self.sample_block_into(sample_period_ns, &mut block)?;
        Ok(block)
    }

    pub fn sample_block_into(
        &self,
        sample_period_ns: i64,
        block: &mut ActuatorSampleBlock,
    ) -> Result<(), TrajectoryError> {
        if sample_period_ns <= 0 || self.duration_ns % sample_period_ns != 0 {
            return Err(TrajectoryError::SamplePeriod);
        }
        let samples = (self.duration_ns / sample_period_ns) as usize;
        let dof = self.coefficients.len();
        block.times_ns.clear();
        block.position.clear();
        block.velocity.clear();
        block.acceleration.clear();
        block.times_ns.reserve(samples);
        block.position.reserve(samples.saturating_mul(dof));
        block.velocity.reserve(samples.saturating_mul(dof));
        block.acceleration.reserve(samples.saturating_mul(dof));
        block.dof = dof;
        let duration = self.duration_ns as f64 * 1e-9;
        for sample in 1..=samples {
            let time_ns = self.start_time_ns + sample as i64 * sample_period_ns;
            let u =
                ((time_ns - self.start_time_ns) as f64 / self.duration_ns as f64).clamp(0.0, 1.0);
            block.times_ns.push(time_ns);
            for coefficients in &self.coefficients {
                let [c0, c1, c2, c3, c4, c5] = *coefficients;
                block
                    .position
                    .push(((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u + c0);
                block.velocity.push(
                    ((((5.0 * c5 * u + 4.0 * c4) * u + 3.0 * c3) * u + 2.0 * c2) * u + c1)
                        / duration,
                );
                block.acceleration.push(
                    (((20.0 * c5 * u + 12.0 * c4) * u + 6.0 * c3) * u + 2.0 * c2)
                        / duration.powi(2),
                );
            }
        }
        Ok(())
    }

    pub fn extrema(&self) -> SegmentExtrema {
        let mut extrema = SegmentExtrema {
            min_position: Vec::with_capacity(self.coefficients.len()),
            max_position: Vec::with_capacity(self.coefficients.len()),
            max_abs_velocity: Vec::with_capacity(self.coefficients.len()),
            max_abs_acceleration: Vec::with_capacity(self.coefficients.len()),
            max_abs_jerk: Vec::with_capacity(self.coefficients.len()),
        };
        self.extrema_into(&mut extrema);
        extrema
    }

    pub fn extrema_into(&self, extrema: &mut SegmentExtrema) {
        let duration = self.duration_ns as f64 * 1e-9;
        extrema.min_position.clear();
        extrema.max_position.clear();
        extrema.max_abs_velocity.clear();
        extrema.max_abs_acceleration.clear();
        extrema.max_abs_jerk.clear();
        extrema.min_position.reserve(self.coefficients.len());
        extrema.max_position.reserve(self.coefficients.len());
        extrema.max_abs_velocity.reserve(self.coefficients.len());
        extrema
            .max_abs_acceleration
            .reserve(self.coefficients.len());
        extrema.max_abs_jerk.reserve(self.coefficients.len());
        for coefficients in &self.coefficients {
            let [minimum_position, maximum_position] = position_range_fixed(coefficients);
            extrema.min_position.push(minimum_position);
            extrema.max_position.push(maximum_position);
            let [_, c1, c2, c3, c4, c5] = *coefficients;
            let v = [c1, 2.0 * c2, 3.0 * c3, 4.0 * c4, 5.0 * c5];
            let a = [2.0 * c2, 6.0 * c3, 12.0 * c4, 20.0 * c5, 0.0];
            let j = [6.0 * c3, 24.0 * c4, 60.0 * c5, 0.0, 0.0];
            let snap = [24.0 * c4, 120.0 * c5, 0.0, 0.0, 0.0];
            extrema
                .max_abs_velocity
                .push(extreme_abs_fixed(&v, 4, &a, 3) / duration);
            extrema
                .max_abs_acceleration
                .push(extreme_abs_fixed(&a, 3, &j, 2) / duration.powi(2));
            extrema
                .max_abs_jerk
                .push(extreme_abs_fixed(&j, 2, &snap, 1) / duration.powi(3));
        }
    }

    /// Analytic maximum absolute coordinate velocity on a closed subinterval.
    /// The caller owns the output buffer; roots are isolated with the same
    /// fixed-capacity polynomial machinery used by whole-segment admission.
    pub fn maximum_abs_velocity_between_into(
        &self,
        start_time_ns: ControlTime,
        end_time_ns: ControlTime,
        maximum_abs_velocity: &mut [f64],
    ) -> Result<(), TrajectoryError> {
        let segment_end = self.start_time_ns.saturating_add(self.duration_ns);
        if maximum_abs_velocity.len() != self.coefficients.len() {
            return Err(TrajectoryError::Dimension);
        }
        if start_time_ns < self.start_time_ns
            || end_time_ns > segment_end
            || start_time_ns > end_time_ns
        {
            return Err(TrajectoryError::Interval);
        }
        let duration = self.duration_ns as f64 * 1e-9;
        let u_start = (start_time_ns - self.start_time_ns) as f64 / self.duration_ns as f64;
        let u_end = (end_time_ns - self.start_time_ns) as f64 / self.duration_ns as f64;
        for (coordinate, coefficients) in self.coefficients.iter().enumerate() {
            let [_, c1, c2, c3, c4, c5] = *coefficients;
            let velocity = [c1, 2.0 * c2, 3.0 * c3, 4.0 * c4, 5.0 * c5];
            let acceleration = [2.0 * c2, 6.0 * c3, 12.0 * c4, 20.0 * c5, 0.0];
            let roots = roots_unit_interval_fixed(&acceleration, 3);
            let mut maximum = poly_eval_fixed(&velocity, 4, u_start)
                .abs()
                .max(poly_eval_fixed(&velocity, 4, u_end).abs());
            for point in &roots.values[..roots.len] {
                if *point >= u_start && *point <= u_end {
                    maximum = maximum.max(poly_eval_fixed(&velocity, 4, *point).abs());
                }
            }
            maximum_abs_velocity[coordinate] = maximum / duration;
        }
        Ok(())
    }

    pub fn validate(&self, limits: &[SegmentLimits]) -> Result<SegmentExtrema, TrajectoryError> {
        let mut extrema = SegmentExtrema {
            min_position: Vec::with_capacity(self.coefficients.len()),
            max_position: Vec::with_capacity(self.coefficients.len()),
            max_abs_velocity: Vec::with_capacity(self.coefficients.len()),
            max_abs_acceleration: Vec::with_capacity(self.coefficients.len()),
            max_abs_jerk: Vec::with_capacity(self.coefficients.len()),
        };
        self.validate_into(limits, &mut extrema)?;
        Ok(extrema)
    }

    pub fn validate_into(
        &self,
        limits: &[SegmentLimits],
        extrema: &mut SegmentExtrema,
    ) -> Result<(), TrajectoryError> {
        if limits.len() != self.coefficients.len() {
            return Err(TrajectoryError::Dimension);
        }
        self.extrema_into(extrema);
        for (index, limit) in limits.iter().enumerate() {
            check_lower_limit(
                index,
                "position",
                extrema.min_position[index],
                limit.min_position,
            )?;
            check_upper_limit(
                index,
                "position",
                extrema.max_position[index],
                limit.max_position,
            )?;
            check_limit(
                index,
                "velocity",
                extrema.max_abs_velocity[index],
                limit.max_velocity,
            )?;
            check_limit(
                index,
                "acceleration",
                extrema.max_abs_acceleration[index],
                limit.max_acceleration,
            )?;
            check_limit(index, "jerk", extrema.max_abs_jerk[index], limit.max_jerk)?;
        }
        Ok(())
    }
}

fn check_lower_limit(
    actuator: usize,
    quantity: &'static str,
    observed: f64,
    limit: f64,
) -> Result<(), TrajectoryError> {
    let tolerance = 1e-12 * limit.abs().max(1.0);
    if observed < limit - tolerance {
        Err(TrajectoryError::Limit {
            actuator,
            quantity,
            observed,
            limit,
        })
    } else {
        Ok(())
    }
}

fn check_upper_limit(
    actuator: usize,
    quantity: &'static str,
    observed: f64,
    limit: f64,
) -> Result<(), TrajectoryError> {
    let tolerance = 1e-12 * limit.abs().max(1.0);
    if observed > limit + tolerance {
        Err(TrajectoryError::Limit {
            actuator,
            quantity,
            observed,
            limit,
        })
    } else {
        Ok(())
    }
}

fn check_limit(
    actuator: usize,
    quantity: &'static str,
    observed: f64,
    limit: f64,
) -> Result<(), TrajectoryError> {
    if observed > limit * (1.0 + 1e-12) {
        Err(TrajectoryError::Limit {
            actuator,
            quantity,
            observed,
            limit,
        })
    } else {
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct FixedRoots {
    values: [f64; 4],
    len: usize,
}

impl FixedRoots {
    fn new() -> Self {
        Self {
            values: [0.0; 4],
            len: 0,
        }
    }

    fn push_unique(&mut self, value: f64) {
        let value = value.clamp(0.0, 1.0);
        if self.values[..self.len]
            .iter()
            .any(|existing| (*existing - value).abs() < 1e-9)
        {
            return;
        }
        if self.len < self.values.len() {
            self.values[self.len] = value;
            self.len += 1;
        }
    }

    fn sort(&mut self) {
        self.values[..self.len].sort_by(f64::total_cmp);
    }
}

fn derivative_fixed(coefficients: &[f64; 5], degree: usize) -> [f64; 5] {
    let mut derivative = [0.0; 5];
    for power in 1..=degree {
        derivative[power - 1] = coefficients[power] * power as f64;
    }
    derivative
}

fn poly_eval_fixed(coefficients: &[f64; 5], degree: usize, value: f64) -> f64 {
    (0..=degree)
        .rev()
        .fold(0.0, |result, power| result * value + coefficients[power])
}

fn position_range_fixed(coefficients: &[f64; 6]) -> [f64; 2] {
    let [c0, c1, c2, c3, c4, c5] = *coefficients;
    let velocity = [c1, 2.0 * c2, 3.0 * c3, 4.0 * c4, 5.0 * c5];
    let roots = roots_unit_interval_fixed(&velocity, 4);
    let evaluate = |u: f64| ((((c5 * u + c4) * u + c3) * u + c2) * u + c1) * u + c0;
    let start = evaluate(0.0);
    let end = evaluate(1.0);
    let mut minimum = start.min(end);
    let mut maximum = start.max(end);
    for point in &roots.values[..roots.len] {
        let value = evaluate(*point);
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }
    [minimum, maximum]
}

/// Deterministically isolate polynomial roots in `[0, 1]` by recursively
/// splitting at derivative roots. This is analytic polynomial extrema
/// evaluation (no time-grid sampling) with bisection only for root refinement.
fn roots_unit_interval_fixed(coefficients: &[f64; 5], requested_degree: usize) -> FixedRoots {
    let mut degree = requested_degree;
    while degree > 0 && coefficients[degree].abs() < 1e-14 {
        degree -= 1;
    }
    if degree == 0 {
        return FixedRoots::new();
    }
    if degree == 1 {
        let root = -coefficients[0] / coefficients[1];
        let mut roots = FixedRoots::new();
        if (-1e-12..=1.0 + 1e-12).contains(&root) {
            roots.push_unique(root);
        }
        return roots;
    }

    let derivative_coefficients = derivative_fixed(coefficients, degree);
    let derivative_roots =
        roots_unit_interval_fixed(&derivative_coefficients, degree.saturating_sub(1));
    let mut boundaries = [0.0; 6];
    let mut boundary_count = 1;
    for root in &derivative_roots.values[..derivative_roots.len] {
        boundaries[boundary_count] = *root;
        boundary_count += 1;
    }
    boundaries[boundary_count] = 1.0;
    boundary_count += 1;
    boundaries[..boundary_count].sort_by(f64::total_cmp);

    let mut roots = FixedRoots::new();
    for boundary in &boundaries[..boundary_count] {
        if poly_eval_fixed(coefficients, degree, *boundary).abs() < 1e-10 {
            roots.push_unique(*boundary);
        }
    }
    for interval in boundaries[..boundary_count].windows(2) {
        let mut left = interval[0];
        let mut right = interval[1];
        let mut left_value = poly_eval_fixed(coefficients, degree, left);
        let right_value = poly_eval_fixed(coefficients, degree, right);
        if left_value * right_value >= 0.0 {
            continue;
        }
        for _ in 0..60 {
            let middle = 0.5 * (left + right);
            let middle_value = poly_eval_fixed(coefficients, degree, middle);
            if left_value * middle_value <= 0.0 {
                right = middle;
            } else {
                left = middle;
                left_value = middle_value;
            }
        }
        roots.push_unique(0.5 * (left + right));
    }
    roots.sort();
    roots
}

fn extreme_abs_fixed(
    polynomial: &[f64; 5],
    degree: usize,
    derivative_polynomial: &[f64; 5],
    derivative_degree: usize,
) -> f64 {
    let roots = roots_unit_interval_fixed(derivative_polynomial, derivative_degree);
    let mut maximum = poly_eval_fixed(polynomial, degree, 0.0)
        .abs()
        .max(poly_eval_fixed(polynomial, degree, 1.0).abs());
    for point in &roots.values[..roots.len] {
        maximum = maximum.max(poly_eval_fixed(polynomial, degree, *point).abs());
    }
    maximum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_and_sample_count_are_exact() {
        let q0 = DVector::from_vec(vec![0.0]);
        let q1 = DVector::from_vec(vec![1.0]);
        let zero = DVector::zeros(1);
        let segment =
            QuinticSegment::new(100, 20_000_000, &q0, &zero, &zero, &q1, &zero, &zero).unwrap();
        let start = segment.evaluate(100);
        let end = segment.evaluate(20_000_100);
        assert!(start.position[0].abs() < 1e-12);
        assert!((end.position[0] - 1.0).abs() < 1e-12);
        assert!(end.velocity[0].abs() < 1e-10);
        assert_eq!(segment.sample_block(1_000_000).unwrap().len(), 20);
        let direct = segment.evaluate(10_000_000);
        let mut position = vec![0.0; 1];
        let mut velocity = vec![0.0; 1];
        let mut acceleration = vec![0.0; 1];
        segment
            .evaluate_into(10_000_000, &mut position, &mut velocity, &mut acceleration)
            .unwrap();
        assert_eq!(direct.position, position);
        assert_eq!(direct.velocity, velocity);
        assert_eq!(direct.acceleration, acceleration);
    }

    #[test]
    fn floating_root_prediction_reconstructs_pose_and_bounds_twist() {
        let start = Transform3::from_parts(
            nalgebra::Translation3::new(1.0, -2.0, 0.5),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        );
        let twist = Motion6(nalgebra::SVector::<f64, 6>::new(
            0.0, 0.0, 0.4, 1.0, -0.5, 0.2,
        ));
        let acceleration = SpatialAcceleration6(nalgebra::SVector::<f64, 6>::new(
            0.0, 0.0, 0.2, -0.4, 0.0, 0.6,
        ));
        let duration = 0.02;
        let displacement =
            Motion6(twist.0 * duration + acceleration.0 * (0.5 * duration * duration));
        let end_twist = Motion6(twist.0 + acceleration.0 * duration);
        let mut segment = RootPosePredictionSegment::stationary();
        segment
            .set_boundary_conditions(
                10,
                20_000_000,
                start,
                twist,
                acceleration,
                displacement,
                end_twist,
                acceleration,
            )
            .unwrap();

        let mut pose = Transform3::identity();
        let mut evaluated_twist = Motion6::default();
        let mut evaluated_acceleration = SpatialAcceleration6::default();
        segment
            .evaluate_into(
                20_000_010,
                &mut pose,
                &mut evaluated_twist,
                &mut evaluated_acceleration,
            )
            .unwrap();
        let expected_translation = start.translation.vector
            + Vec3::new(displacement.0[3], displacement.0[4], displacement.0[5]);
        assert!((pose.translation.vector - expected_translation).norm() < 1e-12);
        assert!((evaluated_twist.0 - end_twist.0).norm() < 1e-11);
        assert!((evaluated_acceleration.0 - acceleration.0).norm() < 1e-9);

        let mut bound = Motion6::default();
        segment
            .maximum_abs_twist_between_into(10, 20_000_010, &mut bound)
            .unwrap();
        for sample in 0..=200 {
            let time = 10 + sample * 100_000;
            segment
                .evaluate_into(
                    time,
                    &mut pose,
                    &mut evaluated_twist,
                    &mut evaluated_acceleration,
                )
                .unwrap();
            for coordinate in 0..6 {
                assert!(evaluated_twist.0[coordinate].abs() <= bound.0[coordinate] + 1e-12);
            }
        }

        segment
            .set_error_growth(RootPredictionErrorGrowth {
                initial_translation_radius_m: 0.001,
                translation_velocity_error_bound_mps: 0.02,
                translation_acceleration_error_bound_mps2: 0.5,
                initial_rotation_radius_rad: 0.002,
                angular_velocity_error_bound_radps: 0.03,
                angular_acceleration_error_bound_radps2: 0.4,
            })
            .unwrap();
        let (translation_radius, rotation_radius) = segment.error_radii_at(20_000_010).unwrap();
        assert!((translation_radius - 0.0015).abs() < 1e-15);
        assert!((rotation_radius - 0.00268).abs() < 1e-15);
        let (translation_rate, rotation_rate) = segment
            .maximum_error_radius_growth_rates_between(10, 20_000_010)
            .unwrap();
        assert!((translation_rate - 0.03).abs() < 1e-15);
        assert!((rotation_rate - 0.038).abs() < 1e-15);
    }

    #[test]
    fn interval_velocity_extrema_bound_dense_samples_without_global_overreach() {
        let q0 = DVector::from_element(1, 0.0);
        let q1 = DVector::from_element(1, 1.0);
        let zero = DVector::zeros(1);
        let segment =
            QuinticSegment::new(1_000, 1_000_000_000, &q0, &zero, &zero, &q1, &zero, &zero)
                .unwrap();
        let mut local = [0.0];
        segment
            .maximum_abs_velocity_between_into(1_000, 100_001_000, &mut local)
            .unwrap();
        let global = segment.extrema().max_abs_velocity[0];
        assert!(local[0] < 0.3 * global);

        let mut position = [0.0];
        let mut velocity = [0.0];
        let mut acceleration = [0.0];
        for sample in 0..=100 {
            let time = 1_000 + sample * 1_000_000;
            segment
                .evaluate_into(time, &mut position, &mut velocity, &mut acceleration)
                .unwrap();
            assert!(velocity[0].abs() <= local[0] + 1e-12);
        }
    }

    #[test]
    fn extrema_bound_dense_observations() {
        let q0 = DVector::from_vec(vec![0.0]);
        let q1 = DVector::from_vec(vec![1.0]);
        let zero = DVector::zeros(1);
        let segment =
            QuinticSegment::new(0, 1_000_000_000, &q0, &zero, &zero, &q1, &zero, &zero).unwrap();
        let extrema = segment.extrema();
        for index in 0..=10_000 {
            let sample = segment.evaluate(index * 100_000);
            assert!(sample.velocity[0].abs() <= extrema.max_abs_velocity[0] + 1e-10);
            assert!(sample.acceleration[0].abs() <= extrema.max_abs_acceleration[0] + 1e-10);
            assert!(sample.position[0] >= extrema.min_position[0] - 1e-10);
            assert!(sample.position[0] <= extrema.max_position[0] + 1e-10);
        }
    }

    #[test]
    fn analytic_position_extrema_reject_interior_overshoot() {
        let position = DVector::zeros(1);
        let start_velocity = DVector::from_vec(vec![4.0]);
        let end_velocity = DVector::from_vec(vec![-4.0]);
        let zero = DVector::zeros(1);
        let segment = QuinticSegment::new(
            0,
            1_000_000_000,
            &position,
            &start_velocity,
            &zero,
            &position,
            &end_velocity,
            &zero,
        )
        .unwrap();
        let extrema = segment.extrema();
        assert!(extrema.max_position[0] > 0.5);
        let error = segment
            .validate(&[SegmentLimits {
                min_position: -0.5,
                max_position: 0.5,
                ..SegmentLimits::default()
            }])
            .unwrap_err();
        assert!(matches!(
            error,
            TrajectoryError::Limit {
                quantity: "position",
                ..
            }
        ));
    }
}
