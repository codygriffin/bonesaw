//! Immutable dense-SDF world geometry and floating acceleration barriers.
//!
//! The field is part of the compiled query object: runtime evaluation performs
//! no topology discovery or allocation.  Every grid node must be finite.
//! Outside-map space is either rejected or conservatively represented by the
//! occupied boundary of the known grid; it can never silently become free.

use nalgebra::{DMatrix, Point3, RowDVector};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    collision::{
        CollisionAccelerationBarrierConfig, CompiledCollisionModel, DistanceQuality, SphereProxy,
    },
    math::{ControlTime, Motion6, SpatialAcceleration6, Transform3, Vec3},
    model::{BodyId, CompiledModel, DynamicsCache, ModelCache, ModelError},
    solver::ConstraintBuffer,
    trajectory::{QuinticSegment, RootPosePredictionSegment, TrajectoryError},
};

const GRADIENT_EPSILON: f64 = 1e-10;

/// Version and temporal validity of one immutable world-scene snapshot.
///
/// The field transform is already expressed in smooth `control_world`.
/// `scene_epoch` identifies replacement of scene content; it is deliberately
/// independent of localization/map frame corrections and the MotionProgram
/// epoch. The validity interval is closed and must cover the full command
/// horizon when that policy is enabled.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct WorldSceneStamp {
    pub scene_epoch: u64,
    pub source_time_ns: ControlTime,
    pub valid_from_ns: ControlTime,
    pub valid_until_ns: ControlTime,
}

impl Default for WorldSceneStamp {
    fn default() -> Self {
        Self::timeless(0)
    }
}

impl WorldSceneStamp {
    pub const fn timeless(scene_epoch: u64) -> Self {
        Self {
            scene_epoch,
            source_time_ns: ControlTime::MIN,
            valid_from_ns: ControlTime::MIN,
            valid_until_ns: ControlTime::MAX,
        }
    }

    pub fn validate(self) -> bool {
        self.valid_from_ns <= self.valid_until_ns && self.source_time_ns <= self.valid_until_ns
    }

    pub fn evidence(
        self,
        tick_time_ns: ControlTime,
        horizon_ns: i64,
        expected_scene_epoch: Option<u64>,
        maximum_age_ns: Option<i64>,
        require_full_horizon: bool,
    ) -> WorldSceneEvidence {
        let horizon_end_ns = tick_time_ns.saturating_add(horizon_ns.max(0));
        let age_ns = tick_time_ns.checked_sub(self.source_time_ns);
        let validity = if !self.validate() || horizon_ns < 0 {
            WorldSceneValidity::InvalidStamp
        } else if expected_scene_epoch.is_some_and(|expected| expected != self.scene_epoch) {
            WorldSceneValidity::EpochMismatch
        } else if self.source_time_ns > tick_time_ns {
            WorldSceneValidity::SourceFromFuture
        } else if tick_time_ns < self.valid_from_ns {
            WorldSceneValidity::NotYetValid
        } else if tick_time_ns > self.valid_until_ns {
            WorldSceneValidity::ExpiredAtTick
        } else if require_full_horizon && horizon_end_ns > self.valid_until_ns {
            WorldSceneValidity::HorizonExpired
        } else if maximum_age_ns
            .is_some_and(|maximum| maximum < 0 || age_ns.is_none_or(|age| age > maximum))
        {
            WorldSceneValidity::TooOld
        } else {
            WorldSceneValidity::Valid
        };
        WorldSceneEvidence {
            stamp: self,
            tick_time_ns,
            horizon_end_ns,
            age_ns,
            validity,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum WorldSceneValidity {
    Valid,
    EpochMismatch,
    SourceFromFuture,
    NotYetValid,
    ExpiredAtTick,
    HorizonExpired,
    TooOld,
    InvalidStamp,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct WorldSceneEvidence {
    pub stamp: WorldSceneStamp,
    pub tick_time_ns: ControlTime,
    pub horizon_end_ns: ControlTime,
    pub age_ns: Option<i64>,
    pub validity: WorldSceneValidity,
}

impl WorldSceneEvidence {
    pub const fn timeless_valid() -> Self {
        Self {
            stamp: WorldSceneStamp::timeless(0),
            tick_time_ns: 0,
            horizon_end_ns: 0,
            age_ns: None,
            validity: WorldSceneValidity::Valid,
        }
    }

    pub fn is_valid(self) -> bool {
        self.validity == WorldSceneValidity::Valid
    }
}

/// Explicit policy for queries outside the finite known SDF volume.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum SdfOutsidePolicy {
    /// Reject the control query. The caller must withhold or replace the
    /// command rather than interpreting unavailable geometry as free space.
    Reject,
    /// Treat the exterior as occupied. The grid AABB becomes an additional
    /// signed-distance surface whose positive side is the known interior.
    OccupiedBoundary,
}

/// Provenance of one local field linearization.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum SdfSampleSource {
    TrilinearGrid,
    OccupiedBoundary,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SdfSample {
    pub signed_distance_m: f64,
    /// Analytic derivative of the trilinear/boundary scalar field. It is not
    /// normalized before row emission, so it remains consistent with the
    /// reported signed-distance scalar even when a sampled field is imperfect.
    pub gradient_in_control_world: Vec3,
    pub source: SdfSampleSource,
}

/// Immutable, fully-known dense signed-distance grid.
///
/// Samples use x-fastest indexing at grid nodes. `control_world_from_grid`
/// freezes the grid frame in the controller's smooth control world.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DenseSdfGrid {
    #[serde(with = "crate::math::serde_transform3")]
    control_world_from_grid: Transform3,
    dimensions: [usize; 3],
    spacing_m: Vec3,
    samples_m: Vec<f64>,
    outside_policy: SdfOutsidePolicy,
}

impl DenseSdfGrid {
    pub fn new(
        control_world_from_grid: Transform3,
        dimensions: [usize; 3],
        spacing_m: Vec3,
        samples_m: Vec<f64>,
        outside_policy: SdfOutsidePolicy,
    ) -> Result<Self, WorldCollisionError> {
        let grid = Self {
            control_world_from_grid,
            dimensions,
            spacing_m,
            samples_m,
            outside_policy,
        };
        grid.validate()?;
        Ok(grid)
    }

    pub fn validate(&self) -> Result<(), WorldCollisionError> {
        if self.dimensions.iter().any(|dimension| *dimension < 2)
            || !self
                .spacing_m
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
            || !self
                .control_world_from_grid
                .translation
                .vector
                .iter()
                .all(|value| value.is_finite())
            || !self
                .control_world_from_grid
                .rotation
                .coords
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(WorldCollisionError::InvalidGrid);
        }
        let expected = self.dimensions[0]
            .checked_mul(self.dimensions[1])
            .and_then(|value| value.checked_mul(self.dimensions[2]))
            .ok_or(WorldCollisionError::InvalidGrid)?;
        if self.samples_m.len() != expected
            || !self.samples_m.iter().all(|sample| sample.is_finite())
        {
            return Err(WorldCollisionError::InvalidGrid);
        }
        Ok(())
    }

    pub fn dimensions(&self) -> [usize; 3] {
        self.dimensions
    }

    pub fn spacing_m(&self) -> Vec3 {
        self.spacing_m
    }

    pub fn outside_policy(&self) -> SdfOutsidePolicy {
        self.outside_policy
    }

    pub fn control_world_from_grid(&self) -> Transform3 {
        self.control_world_from_grid
    }

    pub fn samples(&self) -> &[f64] {
        &self.samples_m
    }

    /// Conservative global spatial Lipschitz bound for the sampled scalar
    /// field. Each trilinear derivative is an interpolation of axis-edge
    /// slopes, so the Euclidean norm is bounded by the three maximum absolute
    /// edge slopes. `OccupiedBoundary` additionally contributes a unit-SDF
    /// boundary.
    pub fn maximum_gradient_norm_bound(&self) -> f64 {
        let mut maximum_axis_slope = Vec3::zeros();
        for z in 0..self.dimensions[2] {
            for y in 0..self.dimensions[1] {
                for x in 0..self.dimensions[0] {
                    if x + 1 < self.dimensions[0] {
                        maximum_axis_slope.x = maximum_axis_slope.x.max(
                            ((self.samples_m[self.index(x + 1, y, z)]
                                - self.samples_m[self.index(x, y, z)])
                                / self.spacing_m.x)
                                .abs(),
                        );
                    }
                    if y + 1 < self.dimensions[1] {
                        maximum_axis_slope.y = maximum_axis_slope.y.max(
                            ((self.samples_m[self.index(x, y + 1, z)]
                                - self.samples_m[self.index(x, y, z)])
                                / self.spacing_m.y)
                                .abs(),
                        );
                    }
                    if z + 1 < self.dimensions[2] {
                        maximum_axis_slope.z = maximum_axis_slope.z.max(
                            ((self.samples_m[self.index(x, y, z + 1)]
                                - self.samples_m[self.index(x, y, z)])
                                / self.spacing_m.z)
                                .abs(),
                        );
                    }
                }
            }
        }
        let trilinear = maximum_axis_slope.norm();
        if self.outside_policy == SdfOutsidePolicy::OccupiedBoundary {
            trilinear.max(1.0)
        } else {
            trilinear
        }
    }

    pub fn query(&self, point_in_control_world: Vec3) -> Result<SdfSample, WorldCollisionError> {
        if !point_in_control_world.iter().all(|value| value.is_finite()) {
            return Err(WorldCollisionError::InvalidQuery);
        }
        let point_grid = self
            .control_world_from_grid
            .inverse()
            .transform_point(&Point3::from(point_in_control_world))
            .coords;
        let maximum = Vec3::new(
            (self.dimensions[0] - 1) as f64 * self.spacing_m.x,
            (self.dimensions[1] - 1) as f64 * self.spacing_m.y,
            (self.dimensions[2] - 1) as f64 * self.spacing_m.z,
        );
        let inside =
            (0..3).all(|axis| point_grid[axis] >= 0.0 && point_grid[axis] <= maximum[axis]);
        if !inside {
            return match self.outside_policy {
                SdfOutsidePolicy::Reject => Err(WorldCollisionError::OutsideKnownField),
                SdfOutsidePolicy::OccupiedBoundary => Ok(self.boundary_sample(point_grid, maximum)),
            };
        }

        let trilinear = self.trilinear_sample(point_grid);
        if self.outside_policy == SdfOutsidePolicy::OccupiedBoundary {
            let boundary = self.boundary_sample(point_grid, maximum);
            if boundary.signed_distance_m < trilinear.signed_distance_m {
                return Ok(boundary);
            }
        }
        Ok(trilinear)
    }

    fn trilinear_sample(&self, point_grid: Vec3) -> SdfSample {
        let scaled = Vec3::new(
            point_grid.x / self.spacing_m.x,
            point_grid.y / self.spacing_m.y,
            point_grid.z / self.spacing_m.z,
        );
        let mut lower = [0usize; 3];
        let mut fraction = [0.0; 3];
        for axis in 0..3 {
            let maximum_lower = self.dimensions[axis] - 2;
            lower[axis] = (scaled[axis].floor() as usize).min(maximum_lower);
            fraction[axis] = scaled[axis] - lower[axis] as f64;
        }
        let [tx, ty, tz] = fraction;
        let mut values = [[[0.0; 2]; 2]; 2];
        for dz in 0..2 {
            for dy in 0..2 {
                for dx in 0..2 {
                    values[dz][dy][dx] =
                        self.samples_m[self.index(lower[0] + dx, lower[1] + dy, lower[2] + dz)];
                }
            }
        }

        let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
        let x00 = lerp(values[0][0][0], values[0][0][1], tx);
        let x10 = lerp(values[0][1][0], values[0][1][1], tx);
        let x01 = lerp(values[1][0][0], values[1][0][1], tx);
        let x11 = lerp(values[1][1][0], values[1][1][1], tx);
        let y0 = lerp(x00, x10, ty);
        let y1 = lerp(x01, x11, ty);
        let signed_distance_m = lerp(y0, y1, tz);

        let dx00 = values[0][0][1] - values[0][0][0];
        let dx10 = values[0][1][1] - values[0][1][0];
        let dx01 = values[1][0][1] - values[1][0][0];
        let dx11 = values[1][1][1] - values[1][1][0];
        let derivative_x = lerp(lerp(dx00, dx10, ty), lerp(dx01, dx11, ty), tz) / self.spacing_m.x;

        let dy00 = values[0][1][0] - values[0][0][0];
        let dy10 = values[0][1][1] - values[0][0][1];
        let dy01 = values[1][1][0] - values[1][0][0];
        let dy11 = values[1][1][1] - values[1][0][1];
        let derivative_y = lerp(lerp(dy00, dy10, tx), lerp(dy01, dy11, tx), tz) / self.spacing_m.y;

        let dz00 = values[1][0][0] - values[0][0][0];
        let dz10 = values[1][0][1] - values[0][0][1];
        let dz01 = values[1][1][0] - values[0][1][0];
        let dz11 = values[1][1][1] - values[0][1][1];
        let derivative_z = lerp(lerp(dz00, dz10, tx), lerp(dz01, dz11, tx), ty) / self.spacing_m.z;
        let gradient_grid = Vec3::new(derivative_x, derivative_y, derivative_z);
        SdfSample {
            signed_distance_m,
            gradient_in_control_world: self
                .control_world_from_grid
                .transform_vector(&gradient_grid),
            source: SdfSampleSource::TrilinearGrid,
        }
    }

    fn boundary_sample(&self, point_grid: Vec3, maximum: Vec3) -> SdfSample {
        let clamped = Vec3::new(
            point_grid.x.clamp(0.0, maximum.x),
            point_grid.y.clamp(0.0, maximum.y),
            point_grid.z.clamp(0.0, maximum.z),
        );
        let outside_delta = point_grid - clamped;
        let outside_distance = outside_delta.norm();
        let (signed_distance_m, gradient_grid) = if outside_distance > 0.0 {
            (-outside_distance, -outside_delta / outside_distance)
        } else {
            let candidates = [
                (point_grid.x, Vec3::x()),
                (maximum.x - point_grid.x, -Vec3::x()),
                (point_grid.y, Vec3::y()),
                (maximum.y - point_grid.y, -Vec3::y()),
                (point_grid.z, Vec3::z()),
                (maximum.z - point_grid.z, -Vec3::z()),
            ];
            candidates
                .into_iter()
                .min_by(|left, right| left.0.total_cmp(&right.0))
                .unwrap_or((0.0, Vec3::x()))
        };
        SdfSample {
            signed_distance_m,
            gradient_in_control_world: self
                .control_world_from_grid
                .transform_vector(&gradient_grid),
            source: SdfSampleSource::OccupiedBoundary,
        }
    }

    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        x + self.dimensions[0] * (y + self.dimensions[1] * z)
    }
}

/// One deterministic body-attached sphere sampled against the world field.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorldSphereProbe {
    pub stable_id: u32,
    pub body: BodyId,
    #[serde(with = "crate::math::serde_transform3")]
    pub body_from_sphere: Transform3,
    pub radius_m: f64,
    pub proxy_quality: DistanceQuality,
}

impl From<&SphereProxy> for WorldSphereProbe {
    fn from(proxy: &SphereProxy) -> Self {
        Self {
            stable_id: proxy.stable_id,
            body: proxy.body,
            body_from_sphere: proxy.body_from_sphere,
            radius_m: proxy.radius,
            proxy_quality: proxy.quality,
        }
    }
}

/// Fully compiled world field and deterministic robot probes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompiledWorldCollisionModel {
    #[serde(default)]
    pub scene_stamp: WorldSceneStamp,
    pub field: DenseSdfGrid,
    pub probes: Vec<WorldSphereProbe>,
    pub unsupported_shape_count: usize,
    /// Flat `[probe][coordinate]` conservative center-speed coefficients for
    /// a fixed control root. The field Lipschitz bound converts center travel
    /// into a conservative scalar-distance rate.
    pub probe_speed_coefficients: Vec<f64>,
    pub speed_coordinate_count: usize,
    /// Conservative maximum center radius from the floating root origin for
    /// each probe over the authored joint ranges. This converts predicted
    /// root angular speed into a world-linear probe speed bound.
    pub probe_root_angular_radius_bounds_m: Vec<f64>,
    pub field_lipschitz_bound: f64,
}

impl CompiledWorldCollisionModel {
    /// Build conservative sphere probes from every supported authored robot
    /// collision shape. This intentionally reuses the canonical sphere-cover
    /// compiler; unsupported authored meshes stay explicit.
    pub fn compile(
        model: &CompiledModel,
        field: DenseSdfGrid,
    ) -> Result<Self, WorldCollisionError> {
        let collision = CompiledCollisionModel::compile(model);
        Self::new(
            model,
            field,
            collision
                .spheres
                .iter()
                .map(WorldSphereProbe::from)
                .collect(),
            collision.unsupported_shape_count,
        )
    }

    pub fn new(
        model: &CompiledModel,
        field: DenseSdfGrid,
        probes: Vec<WorldSphereProbe>,
        unsupported_shape_count: usize,
    ) -> Result<Self, WorldCollisionError> {
        field.validate()?;
        let mut previous = None;
        for probe in &probes {
            if probe.body.0 >= model.bodies.len()
                || !probe.radius_m.is_finite()
                || probe.radius_m < 0.0
                || previous.is_some_and(|stable_id| stable_id >= probe.stable_id)
            {
                return Err(WorldCollisionError::InvalidProbe);
            }
            previous = Some(probe.stable_id);
        }
        let motion_bounds: Vec<_> = probes
            .iter()
            .map(|probe| world_probe_motion_bounds(model, probe))
            .collect();
        let probe_speed_coefficients = motion_bounds
            .iter()
            .flat_map(|(coefficients, _)| coefficients.iter().copied())
            .collect();
        let probe_root_angular_radius_bounds_m =
            motion_bounds.iter().map(|(_, radius)| *radius).collect();
        let field_lipschitz_bound = field.maximum_gradient_norm_bound();
        Ok(Self {
            scene_stamp: WorldSceneStamp::default(),
            field,
            probes,
            unsupported_shape_count,
            probe_speed_coefficients,
            speed_coordinate_count: model.dof,
            probe_root_angular_radius_bounds_m,
            field_lipschitz_bound,
        })
    }

    pub fn with_scene_stamp(
        mut self,
        scene_stamp: WorldSceneStamp,
    ) -> Result<Self, WorldCollisionError> {
        if !scene_stamp.validate() {
            return Err(WorldCollisionError::InvalidSceneStamp);
        }
        self.scene_stamp = scene_stamp;
        Ok(self)
    }

    pub fn scene_evidence(
        &self,
        tick_time_ns: ControlTime,
        horizon_ns: i64,
        expected_scene_epoch: Option<u64>,
        maximum_age_ns: Option<i64>,
        require_full_horizon: bool,
    ) -> WorldSceneEvidence {
        self.scene_stamp.evidence(
            tick_time_ns,
            horizon_ns,
            expected_scene_epoch,
            maximum_age_ns,
            require_full_horizon,
        )
    }

    pub fn probe_body(&self, stable_id: u32) -> Option<BodyId> {
        self.probes
            .binary_search_by_key(&stable_id, |probe| probe.stable_id)
            .ok()
            .map(|index| self.probes[index].body)
    }

    /// Validate every world probe at both endpoints and every deterministic
    /// servo-grid knot. Leaving a Reject-policy field is retained as typed
    /// unknown-space evidence instead of aborting before the independently
    /// valid contingency segment can be checked.
    #[allow(clippy::too_many_arguments)]
    pub fn validate_segment_on_grid_buffered(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        control_world_from_root: Transform3,
        sample_period_ns: i64,
        required_clearance_m: f64,
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sweep_scratch: &mut WorldCollisionSweepScratch,
    ) -> Result<WorldCollisionSweepReport, WorldCollisionError> {
        let mut root_prediction = RootPosePredictionSegment::stationary();
        root_prediction.set_boundary_conditions(
            segment.start_time_ns,
            segment.duration_ns,
            control_world_from_root,
            Motion6::default(),
            SpatialAcceleration6::default(),
            Motion6::default(),
            Motion6::default(),
            SpatialAcceleration6::default(),
        )?;
        self.validate_segment_on_grid_with_root_prediction_buffered(
            model,
            segment,
            &root_prediction,
            sample_period_ns,
            required_clearance_m,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            sweep_scratch,
        )
    }

    /// Validate a joint segment against the field while evaluating an explicit
    /// floating-root prediction at the same deterministic time knots.
    #[allow(clippy::too_many_arguments)]
    pub fn validate_segment_on_grid_with_root_prediction_buffered(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        root_prediction: &RootPosePredictionSegment,
        sample_period_ns: i64,
        required_clearance_m: f64,
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sweep_scratch: &mut WorldCollisionSweepScratch,
    ) -> Result<WorldCollisionSweepReport, WorldCollisionError> {
        if !required_clearance_m.is_finite() {
            return Err(WorldCollisionError::InvalidClearance);
        }
        if sample_period_ns <= 0 || segment.duration_ns % sample_period_ns != 0 {
            return Err(TrajectoryError::SamplePeriod.into());
        }
        if state_scratch.q.len() != model.dof
            || state_scratch.v.len() != model.dof
            || acceleration_scratch.len() != model.dof
            || segment.coefficients.len() != model.dof
            || sweep_scratch.probe_minimum_distances.len() != self.probes.len()
            || sweep_scratch.interval_max_abs_velocity.len() != model.dof
        {
            return Err(WorldCollisionError::Dimension);
        }

        sweep_scratch.probe_minimum_distances.fill(f64::INFINITY);
        let intervals = (segment.duration_ns / sample_period_ns) as usize;
        let probe_count = self.probes.len();
        let grid_value_count = (intervals + 1).saturating_mul(probe_count);
        let retain_grid = sweep_scratch.probe_grid_distances.len() >= grid_value_count;
        let mut report = WorldCollisionSweepReport::empty(self.field.outside_policy());
        report.samples_evaluated = intervals + 1;
        report.represented_probe_count = probe_count;
        report.unsupported_shape_count = self.unsupported_shape_count;
        for sample_index in 0..=intervals {
            let time_ns = segment.start_time_ns + sample_index as i64 * sample_period_ns;
            let mut root_twist = Motion6::default();
            let mut root_acceleration = SpatialAcceleration6::default();
            root_prediction.evaluate_into(
                time_ns,
                &mut state_scratch.control_world_from_root,
                &mut root_twist,
                &mut root_acceleration,
            )?;
            segment.evaluate_into(
                time_ns,
                state_scratch.q.as_mut_slice(),
                state_scratch.v.as_mut_slice(),
                acceleration_scratch.as_mut_slice(),
            )?;
            model.forward_kinematics(state_scratch, model_scratch)?;
            for (probe_index, probe) in self.probes.iter().enumerate() {
                report.probe_samples_evaluated += 1;
                match self.probe_distance_at_cache(model_scratch, probe)? {
                    WorldProbeDistance::Known { distance_m, source } => {
                        let prediction_erosion_m = self
                            .prediction_clearance_erosion_bound(
                                root_prediction,
                                probe_index,
                                time_ns,
                            )
                            .ok_or(WorldCollisionError::Dimension)?;
                        let distance_m = distance_m - prediction_erosion_m;
                        report.maximum_prediction_clearance_erosion_m = report
                            .maximum_prediction_clearance_erosion_m
                            .max(prediction_erosion_m);
                        sweep_scratch.probe_minimum_distances[probe_index] =
                            sweep_scratch.probe_minimum_distances[probe_index].min(distance_m);
                        if retain_grid {
                            sweep_scratch.probe_grid_distances
                                [sample_index * probe_count + probe_index] = distance_m;
                        }
                        record_world_sweep_distance(
                            &mut report,
                            probe,
                            source,
                            time_ns,
                            distance_m,
                            required_clearance_m,
                        );
                    }
                    WorldProbeDistance::Unknown => {
                        if retain_grid {
                            sweep_scratch.probe_grid_distances
                                [sample_index * probe_count + probe_index] = f64::NEG_INFINITY;
                        }
                        record_world_sweep_unknown(&mut report, probe, time_ns);
                    }
                }
            }
        }
        Ok(report)
    }

    pub fn continuous_clearance_bound(
        &self,
        sweep_scratch: &WorldCollisionSweepScratch,
        maximum_joint_velocity: &[f64],
        half_interval_seconds: f64,
    ) -> Option<WorldCollisionContinuityReport> {
        self.continuous_clearance_bound_with_root_prediction(
            sweep_scratch,
            maximum_joint_velocity,
            &RootPosePredictionSegment::stationary(),
            &Motion6::default(),
            half_interval_seconds,
        )
    }

    pub fn continuous_clearance_bound_with_root_prediction(
        &self,
        sweep_scratch: &WorldCollisionSweepScratch,
        maximum_joint_velocity: &[f64],
        root_prediction: &RootPosePredictionSegment,
        maximum_abs_root_twist_world: &Motion6,
        half_interval_seconds: f64,
    ) -> Option<WorldCollisionContinuityReport> {
        if maximum_joint_velocity.len() != self.speed_coordinate_count
            || !half_interval_seconds.is_finite()
            || half_interval_seconds < 0.0
            || self.probe_speed_coefficients.len()
                != self
                    .probes
                    .len()
                    .saturating_mul(self.speed_coordinate_count)
            || sweep_scratch.probe_minimum_distances.len() != self.probes.len()
            || self.probe_root_angular_radius_bounds_m.len() != self.probes.len()
            || !maximum_abs_root_twist_world
                .0
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0)
        {
            return None;
        }
        let mut report = WorldCollisionContinuityReport::empty();
        for (probe_index, probe) in self.probes.iter().enumerate() {
            let speed = self.probe_speed_bound(
                probe_index,
                maximum_joint_velocity,
                maximum_abs_root_twist_world,
            )?;
            let prediction_error_rate = self.prediction_clearance_error_rate_bound_between(
                root_prediction,
                probe_index,
                root_prediction.start_time_ns,
                root_prediction
                    .start_time_ns
                    .saturating_add(root_prediction.duration_ns),
            )?;
            let distance_rate = self.field_lipschitz_bound * speed + prediction_error_rate;
            let lower = sweep_scratch.probe_minimum_distances[probe_index]
                - distance_rate * half_interval_seconds;
            if lower < report.minimum_clearance_lower_bound_m {
                report.minimum_clearance_lower_bound_m = lower;
                report.limiting_probe_id = Some(probe.stable_id);
                report.limiting_body = Some(probe.body);
                report.limiting_distance_rate_bound_mps = distance_rate;
            }
        }
        Some(report)
    }

    /// Bounded pair-free adaptive refinement for a robot-probe/static-field
    /// segment. A conservative field Lipschitz constant and analytic local
    /// joint-velocity extrema bound each leaf. Midpoints are real SDF samples;
    /// sampled violations and unknown-space encounters retain exact time and
    /// stable probe provenance.
    #[allow(clippy::too_many_arguments)]
    pub fn adaptive_continuous_clearance_bound(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        control_world_from_root: Transform3,
        sample_period_ns: i64,
        required_clearance_m: f64,
        maximum_subdivision_depth: u8,
        maximum_joint_velocity: &[f64],
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sweep_scratch: &mut WorldCollisionSweepScratch,
        sweep: &mut WorldCollisionSweepReport,
    ) -> Result<WorldCollisionContinuityReport, WorldCollisionError> {
        let mut root_prediction = RootPosePredictionSegment::stationary();
        root_prediction.set_boundary_conditions(
            segment.start_time_ns,
            segment.duration_ns,
            control_world_from_root,
            Motion6::default(),
            SpatialAcceleration6::default(),
            Motion6::default(),
            Motion6::default(),
            SpatialAcceleration6::default(),
        )?;
        self.adaptive_continuous_clearance_bound_with_root_prediction(
            model,
            segment,
            &root_prediction,
            sample_period_ns,
            required_clearance_m,
            maximum_subdivision_depth,
            maximum_joint_velocity,
            &Motion6::default(),
            state_scratch,
            acceleration_scratch,
            model_scratch,
            sweep_scratch,
            sweep,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn adaptive_continuous_clearance_bound_with_root_prediction(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        root_prediction: &RootPosePredictionSegment,
        sample_period_ns: i64,
        required_clearance_m: f64,
        maximum_subdivision_depth: u8,
        maximum_joint_velocity: &[f64],
        maximum_abs_root_twist_world: &Motion6,
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sweep_scratch: &mut WorldCollisionSweepScratch,
        sweep: &mut WorldCollisionSweepReport,
    ) -> Result<WorldCollisionContinuityReport, WorldCollisionError> {
        let intervals = (segment.duration_ns / sample_period_ns) as usize;
        let probe_count = self.probes.len();
        if !required_clearance_m.is_finite()
            || sample_period_ns <= 0
            || segment.duration_ns % sample_period_ns != 0
            || maximum_subdivision_depth > 12
            || maximum_joint_velocity.len() != model.dof
            || !maximum_abs_root_twist_world
                .0
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0)
            || sweep_scratch.probe_grid_distances.len()
                < (intervals + 1).saturating_mul(probe_count)
            || sweep_scratch.interval_max_abs_velocity.len() != model.dof
        {
            return Err(WorldCollisionError::Dimension);
        }
        let mut continuity = WorldCollisionContinuityReport::empty();
        let base_half_interval_seconds = 0.5 * sample_period_ns as f64 * 1e-9;
        for (probe_index, probe) in self.probes.iter().enumerate() {
            let broad_speed = self
                .probe_speed_bound(
                    probe_index,
                    maximum_joint_velocity,
                    maximum_abs_root_twist_world,
                )
                .ok_or(WorldCollisionError::Dimension)?;
            let prediction_error_rate = self
                .prediction_clearance_error_rate_bound_between(
                    root_prediction,
                    probe_index,
                    segment.start_time_ns,
                    segment.start_time_ns.saturating_add(segment.duration_ns),
                )
                .ok_or(WorldCollisionError::Dimension)?;
            let broad_rate = self.field_lipschitz_bound * broad_speed + prediction_error_rate;
            let broad_lower = sweep_scratch.probe_minimum_distances[probe_index]
                - broad_rate * base_half_interval_seconds;
            if broad_lower >= required_clearance_m {
                continuity.leaf_interval_count += intervals;
                update_world_continuity_minimum(&mut continuity, probe, broad_lower, broad_rate);
                continue;
            }
            for interval in 0..intervals {
                let left_time_ns = segment.start_time_ns + interval as i64 * sample_period_ns;
                let right_time_ns = left_time_ns + sample_period_ns;
                let left_distance =
                    sweep_scratch.probe_grid_distances[interval * probe_count + probe_index];
                let right_distance =
                    sweep_scratch.probe_grid_distances[(interval + 1) * probe_count + probe_index];
                self.refine_world_probe_interval(
                    model,
                    segment,
                    root_prediction,
                    required_clearance_m,
                    maximum_subdivision_depth,
                    probe_index,
                    probe,
                    left_time_ns,
                    right_time_ns,
                    left_distance,
                    right_distance,
                    0,
                    state_scratch,
                    acceleration_scratch,
                    model_scratch,
                    sweep_scratch,
                    sweep,
                    &mut continuity,
                )?;
                if !sweep.is_known() || sweep.has_sampled_violation() {
                    return Ok(continuity);
                }
            }
        }
        Ok(continuity)
    }

    #[allow(clippy::too_many_arguments)]
    fn refine_world_probe_interval(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        root_prediction: &RootPosePredictionSegment,
        required_clearance_m: f64,
        maximum_subdivision_depth: u8,
        probe_index: usize,
        probe: &WorldSphereProbe,
        left_time_ns: ControlTime,
        right_time_ns: ControlTime,
        left_distance_m: f64,
        right_distance_m: f64,
        depth: u8,
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sweep_scratch: &mut WorldCollisionSweepScratch,
        sweep: &mut WorldCollisionSweepReport,
        continuity: &mut WorldCollisionContinuityReport,
    ) -> Result<(), WorldCollisionError> {
        segment.maximum_abs_velocity_between_into(
            left_time_ns,
            right_time_ns,
            &mut sweep_scratch.interval_max_abs_velocity,
        )?;
        let mut maximum_abs_root_twist = Motion6::default();
        root_prediction.maximum_abs_twist_between_into(
            left_time_ns,
            right_time_ns,
            &mut maximum_abs_root_twist,
        )?;
        let center_speed = self
            .probe_speed_bound(
                probe_index,
                &sweep_scratch.interval_max_abs_velocity,
                &maximum_abs_root_twist,
            )
            .ok_or(WorldCollisionError::Dimension)?;
        let prediction_error_rate = self
            .prediction_clearance_error_rate_bound_between(
                root_prediction,
                probe_index,
                left_time_ns,
                right_time_ns,
            )
            .ok_or(WorldCollisionError::Dimension)?;
        let distance_rate = self.field_lipschitz_bound * center_speed + prediction_error_rate;
        let half_interval_seconds = 0.5 * (right_time_ns - left_time_ns) as f64 * 1e-9;
        let lower = left_distance_m.min(right_distance_m) - distance_rate * half_interval_seconds;
        let midpoint_time_ns = left_time_ns + (right_time_ns - left_time_ns) / 2;
        let may_refine = lower < required_clearance_m
            && depth < maximum_subdivision_depth
            && midpoint_time_ns > left_time_ns
            && midpoint_time_ns < right_time_ns;
        if !may_refine {
            continuity.leaf_interval_count += 1;
            continuity.maximum_subdivision_depth_reached =
                continuity.maximum_subdivision_depth_reached.max(depth);
            if lower < required_clearance_m {
                continuity.unresolved_interval_count += 1;
            }
            update_world_continuity_minimum(continuity, probe, lower, distance_rate);
            return Ok(());
        }

        let midpoint = self.probe_distance_at_time(
            model,
            segment,
            root_prediction,
            probe,
            midpoint_time_ns,
            state_scratch,
            acceleration_scratch,
            model_scratch,
        )?;
        continuity.refinement_probe_samples_evaluated += 1;
        sweep.probe_samples_evaluated += 1;
        let (midpoint_distance_m, midpoint_source) = match midpoint {
            WorldProbeDistance::Known { distance_m, source } => (distance_m, source),
            WorldProbeDistance::Unknown => {
                record_world_sweep_unknown(sweep, probe, midpoint_time_ns);
                continuity.leaf_interval_count += 1;
                continuity.unresolved_interval_count += 1;
                continuity.maximum_subdivision_depth_reached =
                    continuity.maximum_subdivision_depth_reached.max(depth + 1);
                update_world_continuity_minimum(continuity, probe, lower, distance_rate);
                return Ok(());
            }
        };
        let prediction_erosion_m = self
            .prediction_clearance_erosion_bound(root_prediction, probe_index, midpoint_time_ns)
            .ok_or(WorldCollisionError::Dimension)?;
        sweep.maximum_prediction_clearance_erosion_m = sweep
            .maximum_prediction_clearance_erosion_m
            .max(prediction_erosion_m);
        record_world_sweep_distance(
            sweep,
            probe,
            midpoint_source,
            midpoint_time_ns,
            midpoint_distance_m,
            required_clearance_m,
        );
        if midpoint_distance_m < required_clearance_m {
            continuity.leaf_interval_count += 1;
            continuity.unresolved_interval_count += 1;
            continuity.maximum_subdivision_depth_reached =
                continuity.maximum_subdivision_depth_reached.max(depth + 1);
            update_world_continuity_minimum(continuity, probe, lower, distance_rate);
            return Ok(());
        }
        self.refine_world_probe_interval(
            model,
            segment,
            root_prediction,
            required_clearance_m,
            maximum_subdivision_depth,
            probe_index,
            probe,
            left_time_ns,
            midpoint_time_ns,
            left_distance_m,
            midpoint_distance_m,
            depth + 1,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            sweep_scratch,
            sweep,
            continuity,
        )?;
        if !sweep.is_known() || sweep.has_sampled_violation() {
            return Ok(());
        }
        self.refine_world_probe_interval(
            model,
            segment,
            root_prediction,
            required_clearance_m,
            maximum_subdivision_depth,
            probe_index,
            probe,
            midpoint_time_ns,
            right_time_ns,
            midpoint_distance_m,
            right_distance_m,
            depth + 1,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            sweep_scratch,
            sweep,
            continuity,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn probe_distance_at_time(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        root_prediction: &RootPosePredictionSegment,
        probe: &WorldSphereProbe,
        time_ns: ControlTime,
        state_scratch: &mut crate::model::RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
    ) -> Result<WorldProbeDistance, WorldCollisionError> {
        let mut root_twist = Motion6::default();
        let mut root_acceleration = SpatialAcceleration6::default();
        root_prediction.evaluate_into(
            time_ns,
            &mut state_scratch.control_world_from_root,
            &mut root_twist,
            &mut root_acceleration,
        )?;
        segment.evaluate_into(
            time_ns,
            state_scratch.q.as_mut_slice(),
            state_scratch.v.as_mut_slice(),
            acceleration_scratch.as_mut_slice(),
        )?;
        model.forward_kinematics(state_scratch, model_scratch)?;
        match self.probe_distance_at_cache(model_scratch, probe)? {
            WorldProbeDistance::Known { distance_m, source } => {
                let probe_index = self
                    .probes
                    .binary_search_by_key(&probe.stable_id, |candidate| candidate.stable_id)
                    .map_err(|_| WorldCollisionError::Dimension)?;
                let erosion = self
                    .prediction_clearance_erosion_bound(root_prediction, probe_index, time_ns)
                    .ok_or(WorldCollisionError::Dimension)?;
                Ok(WorldProbeDistance::Known {
                    distance_m: distance_m - erosion,
                    source,
                })
            }
            WorldProbeDistance::Unknown => Ok(WorldProbeDistance::Unknown),
        }
    }

    fn prediction_clearance_erosion_bound(
        &self,
        root_prediction: &RootPosePredictionSegment,
        probe_index: usize,
        time_ns: ControlTime,
    ) -> Option<f64> {
        let reach = *self.probe_root_angular_radius_bounds_m.get(probe_index)?;
        let (translation_radius, rotation_radius) = root_prediction.error_radii_at(time_ns).ok()?;
        Some(self.field_lipschitz_bound * (translation_radius + reach * rotation_radius))
    }

    fn prediction_clearance_error_rate_bound_between(
        &self,
        root_prediction: &RootPosePredictionSegment,
        probe_index: usize,
        start_time_ns: ControlTime,
        end_time_ns: ControlTime,
    ) -> Option<f64> {
        let reach = *self.probe_root_angular_radius_bounds_m.get(probe_index)?;
        let (translation_rate, rotation_rate) = root_prediction
            .maximum_error_radius_growth_rates_between(start_time_ns, end_time_ns)
            .ok()?;
        Some(self.field_lipschitz_bound * (translation_rate + reach * rotation_rate))
    }

    fn probe_distance_at_cache(
        &self,
        cache: &ModelCache,
        probe: &WorldSphereProbe,
    ) -> Result<WorldProbeDistance, WorldCollisionError> {
        let center_world = (cache.world_from_body[probe.body.0] * probe.body_from_sphere)
            .translation
            .vector;
        match self.field.query(center_world) {
            Ok(sample) => Ok(WorldProbeDistance::Known {
                distance_m: sample.signed_distance_m - probe.radius_m,
                source: sample.source,
            }),
            Err(WorldCollisionError::OutsideKnownField) => Ok(WorldProbeDistance::Unknown),
            Err(error) => Err(error.with_probe(probe.stable_id)),
        }
    }

    fn probe_speed_bound(
        &self,
        probe_index: usize,
        maximum_joint_velocity: &[f64],
        maximum_abs_root_twist_world: &Motion6,
    ) -> Option<f64> {
        if maximum_joint_velocity.len() != self.speed_coordinate_count {
            return None;
        }
        let start = probe_index.checked_mul(self.speed_coordinate_count)?;
        let coefficients = self
            .probe_speed_coefficients
            .get(start..start + self.speed_coordinate_count)?;
        let joint_speed: f64 = coefficients
            .iter()
            .zip(maximum_joint_velocity)
            .map(|(coefficient, velocity)| coefficient * velocity)
            .sum();
        let radius = *self.probe_root_angular_radius_bounds_m.get(probe_index)?;
        let angular_speed_l1 = maximum_abs_root_twist_world.0.as_slice()[..3]
            .iter()
            .sum::<f64>();
        let linear_speed_l1 = maximum_abs_root_twist_world.0.as_slice()[3..]
            .iter()
            .sum::<f64>();
        Some(joint_speed + radius * angular_speed_l1 + linear_speed_l1)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn evaluate_probe_floating_into(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        dynamics: &DynamicsCache,
        root_twist_world: Motion6,
        joint_velocity: &nalgebra::DVector<f64>,
        probe: &WorldSphereProbe,
        sample: &mut WorldDistanceSample,
        scratch: &mut WorldCollisionEvaluationScratch,
    ) -> Result<(), WorldCollisionError> {
        let generalized_dof = model.dof + 6;
        if joint_velocity.len() != model.dof
            || sample.jacobian_row.len() != generalized_dof
            || scratch.point_jacobian.ncols() != generalized_dof
        {
            return Err(WorldCollisionError::Dimension);
        }
        let world_from_sphere = cache.world_from_body[probe.body.0] * probe.body_from_sphere;
        let center_world = world_from_sphere.translation.vector;
        let field_sample = self
            .field
            .query(center_world)
            .map_err(|error| error.with_probe(probe.stable_id))?;
        let gradient = field_sample.gradient_in_control_world;
        let gradient_norm = gradient.norm();
        let normal = if gradient_norm > GRADIENT_EPSILON {
            gradient / gradient_norm
        } else {
            Vec3::x()
        };
        model.floating_point_jacobian_into(
            cache,
            model.bodies[probe.body.0].body_frame,
            probe.body_from_sphere.translation.vector,
            &mut scratch.point_jacobian,
        )?;
        for column in 0..generalized_dof {
            sample.jacobian_row[column] = (0..3)
                .map(|axis| gradient[axis] * scratch.point_jacobian[(axis, column)])
                .sum();
        }
        let relative_normal_velocity_mps = (0..6)
            .map(|coordinate| sample.jacobian_row[coordinate] * root_twist_world.0[coordinate])
            .sum::<f64>()
            + (0..model.dof)
                .map(|coordinate| sample.jacobian_row[6 + coordinate] * joint_velocity[coordinate])
                .sum::<f64>();
        let point_bias = model.point_bias_acceleration_world(
            model.bodies[probe.body.0].body_frame,
            probe.body_from_sphere.translation.vector,
            cache,
            dynamics,
        )?;
        sample.probe_id = probe.stable_id;
        sample.body = probe.body;
        sample.signed_distance_m = field_sample.signed_distance_m - probe.radius_m;
        sample.gradient_in_control_world = gradient;
        sample.gradient_norm = gradient_norm;
        sample.normal_in_control_world = normal;
        sample.point_robot_in_control_world = center_world - normal * probe.radius_m;
        sample.point_world_in_control_world =
            center_world - normal * field_sample.signed_distance_m;
        sample.relative_normal_velocity_mps = relative_normal_velocity_mps;
        // This is the current field-gradient projection of rigid-point Jdot-v.
        // SDF Hessian curvature and voxel-feature switching are intentionally
        // excluded and retained in the quality semantics.
        sample.relative_normal_acceleration_bias_mps2 = gradient.dot(&point_bias);
        sample.field_source = field_sample.source;
        sample.proxy_quality = probe.proxy_quality;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn emit_floating_acceleration_barriers_into(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        dynamics: &DynamicsCache,
        root_twist_world: Motion6,
        joint_velocity: &nalgebra::DVector<f64>,
        config: CollisionAccelerationBarrierConfig,
        hard_rows: &mut ConstraintBuffer,
        scratch: &mut WorldCollisionBarrierScratch,
        evidence: &mut WorldCollisionBarrierEvidence,
    ) -> Result<(), WorldCollisionError> {
        if !config.validate() {
            return Err(WorldCollisionError::InvalidConfig);
        }
        if scratch.capacity() < self.probes.len() || scratch.active_rows.ncols() != model.dof + 6 {
            return Err(WorldCollisionError::ConstraintCapacity);
        }
        *evidence = WorldCollisionBarrierEvidence {
            enabled: true,
            represented_probe_count: self.probes.len(),
            unsupported_shape_count: self.unsupported_shape_count,
            outside_policy: self.field.outside_policy(),
            ..WorldCollisionBarrierEvidence::disabled()
        };
        scratch.active_len = 0;
        let omega = config.natural_frequency_rad_s;
        let damping = 2.0 * config.damping_ratio * omega;
        for probe in &self.probes {
            self.evaluate_probe_floating_into(
                model,
                cache,
                dynamics,
                root_twist_world,
                joint_velocity,
                probe,
                &mut scratch.sample,
                &mut scratch.evaluation,
            )?;
            let margin = scratch.sample.signed_distance_m - config.hard_margin;
            if scratch.sample.signed_distance_m < evidence.minimum_signed_distance_m {
                evidence.minimum_signed_distance_m = scratch.sample.signed_distance_m;
                evidence.minimum_margin_m = margin;
                evidence.closest_probe_id = Some(probe.stable_id);
                evidence.closest_body = Some(probe.body);
                evidence.closest_field_source = Some(scratch.sample.field_source);
                evidence.closest_proxy_quality = Some(probe.proxy_quality);
                evidence.closest_gradient_norm = scratch.sample.gradient_norm;
            }
            if scratch.sample.signed_distance_m >= config.influence_margin {
                continue;
            }
            if scratch.sample.gradient_norm <= GRADIENT_EPSILON {
                return Err(WorldCollisionError::DegenerateGradient {
                    probe_id: probe.stable_id,
                });
            }
            let required_normal_acceleration =
                -damping * scratch.sample.relative_normal_velocity_mps - omega * omega * margin;
            let controllable_lower = required_normal_acceleration
                - scratch.sample.relative_normal_acceleration_bias_mps2;
            let row = hard_rows
                .push()
                .ok_or(WorldCollisionError::ConstraintCapacity)?;
            row.stable_id = config.hard_stable_id_base.saturating_add(probe.stable_id);
            for coordinate in 0..model.dof + 6 {
                row.coefficients[coordinate] = scratch.sample.jacobian_row[coordinate];
            }
            row.lower = controllable_lower;
            row.upper = f64::INFINITY;

            let active = scratch.active_len;
            scratch
                .active_rows
                .row_mut(active)
                .copy_from(&scratch.sample.jacobian_row);
            scratch.active_probe_ids[active] = probe.stable_id;
            scratch.active_bodies[active] = probe.body;
            scratch.active_field_sources[active] = scratch.sample.field_source;
            scratch.active_proxy_qualities[active] = probe.proxy_quality;
            scratch.active_gradient_norms[active] = scratch.sample.gradient_norm;
            scratch.active_relative_normal_velocities[active] =
                scratch.sample.relative_normal_velocity_mps;
            scratch.active_bias_accelerations[active] =
                scratch.sample.relative_normal_acceleration_bias_mps2;
            scratch.active_required_normal_accelerations[active] = required_normal_acceleration;
            scratch.active_len += 1;
        }
        evidence.active_probe_count = scratch.active_len;
        Ok(())
    }

    pub fn finalize_floating_acceleration_barrier_evidence(
        &self,
        generalized_acceleration: &nalgebra::DVector<f64>,
        scratch: &WorldCollisionBarrierScratch,
        evidence: &mut WorldCollisionBarrierEvidence,
    ) -> Result<(), WorldCollisionError> {
        if generalized_acceleration.len() != scratch.active_rows.ncols() {
            return Err(WorldCollisionError::Dimension);
        }
        for active in 0..scratch.active_len {
            let controllable = scratch
                .active_rows
                .row(active)
                .iter()
                .zip(generalized_acceleration.iter())
                .map(|(coefficient, acceleration)| coefficient * acceleration)
                .sum::<f64>();
            let achieved = controllable + scratch.active_bias_accelerations[active];
            let residual = achieved - scratch.active_required_normal_accelerations[active];
            if residual < evidence.minimum_barrier_residual_mps2 {
                evidence.minimum_barrier_residual_mps2 = residual;
                evidence.limiting_active_probe_id = Some(scratch.active_probe_ids[active]);
                evidence.limiting_active_body = Some(scratch.active_bodies[active]);
                evidence.limiting_field_source = Some(scratch.active_field_sources[active]);
                evidence.limiting_proxy_quality = Some(scratch.active_proxy_qualities[active]);
                evidence.limiting_gradient_norm = scratch.active_gradient_norms[active];
                evidence.limiting_relative_normal_velocity_mps =
                    scratch.active_relative_normal_velocities[active];
                evidence.limiting_normal_bias_acceleration_mps2 =
                    scratch.active_bias_accelerations[active];
                evidence.limiting_required_normal_acceleration_mps2 =
                    scratch.active_required_normal_accelerations[active];
                evidence.limiting_achieved_normal_acceleration_mps2 = achieved;
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
enum WorldProbeDistance {
    Known {
        distance_m: f64,
        source: SdfSampleSource,
    },
    Unknown,
}

/// Sampled commanded-segment evidence for robot probes against one immutable
/// world field. Unknown-space provenance remains distinct from a measured
/// clearance violation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldCollisionSweepReport {
    pub scene: WorldSceneEvidence,
    pub samples_evaluated: usize,
    pub probe_samples_evaluated: usize,
    pub represented_probe_count: usize,
    pub unsupported_shape_count: usize,
    /// Largest deterministic root-prediction error erosion applied to any
    /// represented probe sample. Reported distance is already robustified.
    pub maximum_prediction_clearance_erosion_m: f64,
    pub outside_policy: SdfOutsidePolicy,
    pub minimum_signed_distance_m: f64,
    pub minimum_probe_id: Option<u32>,
    pub minimum_body: Option<BodyId>,
    pub minimum_time_ns: Option<ControlTime>,
    pub minimum_field_source: Option<SdfSampleSource>,
    pub minimum_proxy_quality: Option<DistanceQuality>,
    pub first_violation_probe_id: Option<u32>,
    pub first_violation_body: Option<BodyId>,
    pub first_violation_time_ns: Option<ControlTime>,
    pub first_unknown_probe_id: Option<u32>,
    pub first_unknown_body: Option<BodyId>,
    pub first_unknown_time_ns: Option<ControlTime>,
}

impl WorldCollisionSweepReport {
    pub const fn empty(outside_policy: SdfOutsidePolicy) -> Self {
        Self {
            scene: WorldSceneEvidence::timeless_valid(),
            samples_evaluated: 0,
            probe_samples_evaluated: 0,
            represented_probe_count: 0,
            unsupported_shape_count: 0,
            maximum_prediction_clearance_erosion_m: 0.0,
            outside_policy,
            minimum_signed_distance_m: f64::INFINITY,
            minimum_probe_id: None,
            minimum_body: None,
            minimum_time_ns: None,
            minimum_field_source: None,
            minimum_proxy_quality: None,
            first_violation_probe_id: None,
            first_violation_body: None,
            first_violation_time_ns: None,
            first_unknown_probe_id: None,
            first_unknown_body: None,
            first_unknown_time_ns: None,
        }
    }

    pub fn is_known(&self) -> bool {
        self.scene.is_valid() && self.first_unknown_probe_id.is_none()
    }

    pub fn has_sampled_violation(&self) -> bool {
        self.first_violation_probe_id.is_some()
    }

    pub fn is_clear(&self) -> bool {
        self.is_known() && !self.has_sampled_violation()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldCollisionContinuityReport {
    pub minimum_clearance_lower_bound_m: f64,
    pub limiting_probe_id: Option<u32>,
    pub limiting_body: Option<BodyId>,
    pub limiting_distance_rate_bound_mps: f64,
    pub leaf_interval_count: usize,
    pub refinement_probe_samples_evaluated: usize,
    pub unresolved_interval_count: usize,
    pub maximum_subdivision_depth_reached: u8,
}

impl WorldCollisionContinuityReport {
    pub const fn empty() -> Self {
        Self {
            minimum_clearance_lower_bound_m: f64::INFINITY,
            limiting_probe_id: None,
            limiting_body: None,
            limiting_distance_rate_bound_mps: 0.0,
            leaf_interval_count: 0,
            refinement_probe_samples_evaluated: 0,
            unresolved_interval_count: 0,
            maximum_subdivision_depth_reached: 0,
        }
    }
}

/// Caller-owned fixed-capacity storage for base-grid and adaptive world scans.
#[derive(Clone, Debug)]
pub struct WorldCollisionSweepScratch {
    probe_minimum_distances: Vec<f64>,
    probe_grid_distances: Vec<f64>,
    interval_max_abs_velocity: Vec<f64>,
}

impl WorldCollisionSweepScratch {
    pub fn with_grid_capacity(dof: usize, probe_count: usize, grid_sample_capacity: usize) -> Self {
        Self {
            probe_minimum_distances: vec![f64::INFINITY; probe_count],
            probe_grid_distances: vec![
                f64::INFINITY;
                probe_count.saturating_mul(grid_sample_capacity)
            ],
            interval_max_abs_velocity: vec![0.0; dof],
        }
    }
}

fn record_world_sweep_distance(
    report: &mut WorldCollisionSweepReport,
    probe: &WorldSphereProbe,
    source: SdfSampleSource,
    time_ns: ControlTime,
    signed_distance_m: f64,
    required_clearance_m: f64,
) {
    if signed_distance_m < report.minimum_signed_distance_m
        || (signed_distance_m == report.minimum_signed_distance_m
            && (time_ns, probe.stable_id)
                < (
                    report.minimum_time_ns.unwrap_or(ControlTime::MAX),
                    report.minimum_probe_id.unwrap_or(u32::MAX),
                ))
    {
        report.minimum_signed_distance_m = signed_distance_m;
        report.minimum_probe_id = Some(probe.stable_id);
        report.minimum_body = Some(probe.body);
        report.minimum_time_ns = Some(time_ns);
        report.minimum_field_source = Some(source);
        report.minimum_proxy_quality = Some(probe.proxy_quality);
    }
    if signed_distance_m < required_clearance_m
        && report.first_violation_time_ns.is_none_or(|first_time| {
            time_ns < first_time
                || (time_ns == first_time
                    && probe.stable_id < report.first_violation_probe_id.unwrap_or(u32::MAX))
        })
    {
        report.first_violation_probe_id = Some(probe.stable_id);
        report.first_violation_body = Some(probe.body);
        report.first_violation_time_ns = Some(time_ns);
    }
}

fn record_world_sweep_unknown(
    report: &mut WorldCollisionSweepReport,
    probe: &WorldSphereProbe,
    time_ns: ControlTime,
) {
    if report.first_unknown_time_ns.is_none_or(|first_time| {
        time_ns < first_time
            || (time_ns == first_time
                && probe.stable_id < report.first_unknown_probe_id.unwrap_or(u32::MAX))
    }) {
        report.first_unknown_probe_id = Some(probe.stable_id);
        report.first_unknown_body = Some(probe.body);
        report.first_unknown_time_ns = Some(time_ns);
    }
}

fn update_world_continuity_minimum(
    report: &mut WorldCollisionContinuityReport,
    probe: &WorldSphereProbe,
    lower_bound_m: f64,
    distance_rate_bound_mps: f64,
) {
    if lower_bound_m < report.minimum_clearance_lower_bound_m {
        report.minimum_clearance_lower_bound_m = lower_bound_m;
        report.limiting_probe_id = Some(probe.stable_id);
        report.limiting_body = Some(probe.body);
        report.limiting_distance_rate_bound_mps = distance_rate_bound_mps;
    }
}

fn world_probe_motion_bounds(model: &CompiledModel, probe: &WorldSphereProbe) -> (Vec<f64>, f64) {
    let mut body = probe.body;
    let mut reach = probe.body_from_sphere.translation.vector.norm();
    let mut coefficients = vec![0.0; model.dof];
    while let Some(joint_id) = model.bodies[body.0].parent_joint {
        let joint = &model.joints[joint_id.0];
        if let Some(coordinate) = joint.coordinate {
            match joint.kind {
                crate::model::JointKind::Revolute | crate::model::JointKind::Continuous => {
                    coefficients[coordinate] = reach;
                }
                crate::model::JointKind::Prismatic => coefficients[coordinate] = 1.0,
                crate::model::JointKind::Fixed => {}
            }
        }
        reach += joint.parent_from_joint.translation.vector.norm();
        if joint.kind == crate::model::JointKind::Prismatic {
            reach += joint.limit.lower.abs().max(joint.limit.upper.abs());
        }
        body = joint.parent;
    }
    (coefficients, reach)
}

#[derive(Clone, Debug)]
pub struct WorldDistanceSample {
    pub probe_id: u32,
    pub body: BodyId,
    pub signed_distance_m: f64,
    pub gradient_in_control_world: Vec3,
    pub gradient_norm: f64,
    pub normal_in_control_world: Vec3,
    pub point_robot_in_control_world: Vec3,
    pub point_world_in_control_world: Vec3,
    pub relative_normal_velocity_mps: f64,
    pub relative_normal_acceleration_bias_mps2: f64,
    pub jacobian_row: RowDVector<f64>,
    pub field_source: SdfSampleSource,
    pub proxy_quality: DistanceQuality,
}

impl WorldDistanceSample {
    pub fn workspace(generalized_dof: usize) -> Self {
        Self {
            probe_id: 0,
            body: BodyId(0),
            signed_distance_m: f64::INFINITY,
            gradient_in_control_world: Vec3::zeros(),
            gradient_norm: 0.0,
            normal_in_control_world: Vec3::x(),
            point_robot_in_control_world: Vec3::zeros(),
            point_world_in_control_world: Vec3::zeros(),
            relative_normal_velocity_mps: 0.0,
            relative_normal_acceleration_bias_mps2: 0.0,
            jacobian_row: RowDVector::zeros(generalized_dof),
            field_source: SdfSampleSource::TrilinearGrid,
            proxy_quality: DistanceQuality::ExactSphere,
        }
    }
}

#[derive(Clone, Debug)]
pub struct WorldCollisionEvaluationScratch {
    point_jacobian: DMatrix<f64>,
}

impl WorldCollisionEvaluationScratch {
    pub fn new(generalized_dof: usize) -> Self {
        Self {
            point_jacobian: DMatrix::zeros(3, generalized_dof),
        }
    }
}

#[derive(Clone, Debug)]
pub struct WorldCollisionBarrierScratch {
    sample: WorldDistanceSample,
    evaluation: WorldCollisionEvaluationScratch,
    active_rows: DMatrix<f64>,
    active_probe_ids: Vec<u32>,
    active_bodies: Vec<BodyId>,
    active_field_sources: Vec<SdfSampleSource>,
    active_proxy_qualities: Vec<DistanceQuality>,
    active_gradient_norms: Vec<f64>,
    active_relative_normal_velocities: Vec<f64>,
    active_bias_accelerations: Vec<f64>,
    active_required_normal_accelerations: Vec<f64>,
    active_len: usize,
}

impl WorldCollisionBarrierScratch {
    pub fn new(generalized_dof: usize, maximum_probes: usize) -> Self {
        Self {
            sample: WorldDistanceSample::workspace(generalized_dof),
            evaluation: WorldCollisionEvaluationScratch::new(generalized_dof),
            active_rows: DMatrix::zeros(maximum_probes, generalized_dof),
            active_probe_ids: vec![0; maximum_probes],
            active_bodies: vec![BodyId(0); maximum_probes],
            active_field_sources: vec![SdfSampleSource::TrilinearGrid; maximum_probes],
            active_proxy_qualities: vec![DistanceQuality::ExactSphere; maximum_probes],
            active_gradient_norms: vec![0.0; maximum_probes],
            active_relative_normal_velocities: vec![0.0; maximum_probes],
            active_bias_accelerations: vec![0.0; maximum_probes],
            active_required_normal_accelerations: vec![0.0; maximum_probes],
            active_len: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.active_rows.nrows()
    }
}

/// World collision evidence stays independent of robot self-collision and
/// commanded-segment admission. The closest probe and limiting active row are
/// separate for the same reason as the self-collision evidence.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldCollisionBarrierEvidence {
    pub enabled: bool,
    pub represented_probe_count: usize,
    pub unsupported_shape_count: usize,
    pub outside_policy: SdfOutsidePolicy,
    pub active_probe_count: usize,
    pub minimum_signed_distance_m: f64,
    pub minimum_margin_m: f64,
    pub closest_probe_id: Option<u32>,
    pub closest_body: Option<BodyId>,
    pub closest_field_source: Option<SdfSampleSource>,
    pub closest_proxy_quality: Option<DistanceQuality>,
    pub closest_gradient_norm: f64,
    pub limiting_active_probe_id: Option<u32>,
    pub limiting_active_body: Option<BodyId>,
    pub limiting_field_source: Option<SdfSampleSource>,
    pub limiting_proxy_quality: Option<DistanceQuality>,
    pub limiting_gradient_norm: f64,
    pub limiting_relative_normal_velocity_mps: f64,
    pub limiting_normal_bias_acceleration_mps2: f64,
    pub limiting_required_normal_acceleration_mps2: f64,
    pub limiting_achieved_normal_acceleration_mps2: f64,
    pub minimum_barrier_residual_mps2: f64,
}

impl WorldCollisionBarrierEvidence {
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            represented_probe_count: 0,
            unsupported_shape_count: 0,
            outside_policy: SdfOutsidePolicy::Reject,
            active_probe_count: 0,
            minimum_signed_distance_m: f64::INFINITY,
            minimum_margin_m: f64::INFINITY,
            closest_probe_id: None,
            closest_body: None,
            closest_field_source: None,
            closest_proxy_quality: None,
            closest_gradient_norm: 0.0,
            limiting_active_probe_id: None,
            limiting_active_body: None,
            limiting_field_source: None,
            limiting_proxy_quality: None,
            limiting_gradient_norm: 0.0,
            limiting_relative_normal_velocity_mps: 0.0,
            limiting_normal_bias_acceleration_mps2: 0.0,
            limiting_required_normal_acceleration_mps2: 0.0,
            limiting_achieved_normal_acceleration_mps2: 0.0,
            minimum_barrier_residual_mps2: f64::INFINITY,
        }
    }
}

impl Default for WorldCollisionBarrierEvidence {
    fn default() -> Self {
        Self::disabled()
    }
}

#[derive(Debug, Error)]
pub enum WorldCollisionError {
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error(transparent)]
    Trajectory(#[from] TrajectoryError),
    #[error("dense SDF grid dimensions, spacing, transform, or samples are invalid")]
    InvalidGrid,
    #[error("world SDF query point is invalid")]
    InvalidQuery,
    #[error("world collision probe layout is invalid")]
    InvalidProbe,
    #[error("world scene epoch/source/validity stamp is invalid")]
    InvalidSceneStamp,
    #[error("world SDF query left the known field")]
    OutsideKnownField,
    #[error("world SDF query for probe {probe_id} left the known field")]
    ProbeOutsideKnownField { probe_id: u32 },
    #[error("world SDF gradient is degenerate inside the active band for probe {probe_id}")]
    DegenerateGradient { probe_id: u32 },
    #[error("world collision barrier configuration is invalid")]
    InvalidConfig,
    #[error("world collision clearance is invalid")]
    InvalidClearance,
    #[error("world collision barrier scratch or constraint capacity is too small")]
    ConstraintCapacity,
    #[error("world collision query dimension mismatch")]
    Dimension,
}

impl WorldCollisionError {
    fn with_probe(self, probe_id: u32) -> Self {
        match self {
            Self::OutsideKnownField => Self::ProbeOutsideKnownField { probe_id },
            other => other,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DVector, Translation3, UnitQuaternion};
    use std::path::PathBuf;

    use super::*;
    use crate::{
        model::RobotState,
        trajectory::{QuinticSegment, RootPredictionErrorGrowth},
        urdf::load_urdf_file,
    };

    fn assert_close(left: f64, right: f64, epsilon: f64) {
        assert!(
            (left - right).abs() <= epsilon,
            "expected {left:.16e} ~= {right:.16e} within {epsilon:.3e}"
        );
    }

    #[test]
    fn scene_stamp_distinguishes_epoch_time_age_and_horizon_failures() {
        let stamp = WorldSceneStamp {
            scene_epoch: 7,
            source_time_ns: 100,
            valid_from_ns: 100,
            valid_until_ns: 200,
        };
        let valid = stamp.evidence(120, 50, Some(7), Some(30), true);
        assert_eq!(valid.validity, WorldSceneValidity::Valid);
        assert_eq!(valid.age_ns, Some(20));
        assert_eq!(valid.horizon_end_ns, 170);
        assert_eq!(
            stamp.evidence(120, 50, Some(8), Some(30), true).validity,
            WorldSceneValidity::EpochMismatch
        );
        assert_eq!(
            stamp.evidence(90, 0, Some(7), Some(30), true).validity,
            WorldSceneValidity::SourceFromFuture
        );
        assert_eq!(
            stamp.evidence(120, 90, Some(7), Some(30), true).validity,
            WorldSceneValidity::HorizonExpired
        );
        assert_eq!(
            stamp.evidence(160, 0, Some(7), Some(30), true).validity,
            WorldSceneValidity::TooOld
        );
        assert_eq!(
            stamp.evidence(201, 0, Some(7), None, true).validity,
            WorldSceneValidity::ExpiredAtTick
        );
        assert_eq!(
            WorldSceneStamp {
                valid_from_ns: 2,
                valid_until_ns: 1,
                ..stamp
            }
            .evidence(1, 0, None, None, false)
            .validity,
            WorldSceneValidity::InvalidStamp
        );
    }

    fn plane_grid(outside_policy: SdfOutsidePolicy) -> DenseSdfGrid {
        let dimensions = [5, 5, 5];
        let spacing = Vec3::repeat(0.5);
        let control_world_from_grid = Transform3::from_parts(
            Translation3::new(-1.0, -1.0, -1.0),
            UnitQuaternion::identity(),
        );
        let mut samples = Vec::new();
        for z in 0..dimensions[2] {
            for _y in 0..dimensions[1] {
                for _x in 0..dimensions[0] {
                    samples.push(-1.0 + z as f64 * spacing.z);
                }
            }
        }
        DenseSdfGrid::new(
            control_world_from_grid,
            dimensions,
            spacing,
            samples,
            outside_policy,
        )
        .unwrap()
    }

    fn slider_model() -> CompiledModel {
        load_urdf_file(
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/tight_avoidance_toy.urdf"),
        )
        .unwrap()
    }

    fn wall_field(maximum_y: f64, outside_policy: SdfOutsidePolicy) -> DenseSdfGrid {
        let spacing = Vec3::repeat(0.1);
        let dimensions = [5, (maximum_y / spacing.y).round() as usize + 1, 5];
        let origin = Vec3::new(-0.2, 0.0, -0.2);
        let mut samples = Vec::new();
        for _z in 0..dimensions[2] {
            for y in 0..dimensions[1] {
                let world_y = origin.y + y as f64 * spacing.y;
                for _x in 0..dimensions[0] {
                    samples.push(0.55 - world_y);
                }
            }
        }
        DenseSdfGrid::new(
            Transform3::from_parts(Translation3::from(origin), UnitQuaternion::identity()),
            dimensions,
            spacing,
            samples,
            outside_policy,
        )
        .unwrap()
    }

    fn slider_world(
        model: &CompiledModel,
        maximum_y: f64,
        outside_policy: SdfOutsidePolicy,
    ) -> CompiledWorldCollisionModel {
        CompiledWorldCollisionModel::new(
            model,
            wall_field(maximum_y, outside_policy),
            vec![WorldSphereProbe {
                stable_id: 7,
                body: model.body_id("slider").unwrap(),
                body_from_sphere: Transform3::identity(),
                radius_m: 0.1,
                proxy_quality: DistanceQuality::ExactSphere,
            }],
            0,
        )
        .unwrap()
    }

    fn slider_segment(end: f64) -> QuinticSegment {
        let mut segment = QuinticSegment {
            start_time_ns: 0,
            duration_ns: 0,
            coefficients: Vec::with_capacity(1),
        };
        segment
            .set_boundary_conditions(
                0,
                20_000_000,
                &DVector::from_element(1, 0.0),
                &DVector::zeros(1),
                &DVector::zeros(1),
                &DVector::from_element(1, end),
                &DVector::zeros(1),
                &DVector::zeros(1),
            )
            .unwrap();
        segment
    }

    #[test]
    fn swept_world_query_consumes_floating_root_prediction() {
        let model = slider_model();
        let world = slider_world(&model, 1.0, SdfOutsidePolicy::Reject);
        let segment = slider_segment(0.0);
        let mut state = RobotState::zeros(&model);
        let mut acceleration = DVector::zeros(model.dof);
        let mut cache = ModelCache::new(&model);
        let mut scratch = WorldCollisionSweepScratch::with_grid_capacity(model.dof, 1, 21);
        let fixed = world
            .validate_segment_on_grid_buffered(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                -1.0,
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
            )
            .unwrap();

        let mut root_prediction = RootPosePredictionSegment::stationary();
        root_prediction
            .set_boundary_conditions(
                0,
                20_000_000,
                Transform3::identity(),
                Motion6::default(),
                SpatialAcceleration6::default(),
                Motion6(nalgebra::SVector::<f64, 6>::new(
                    0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
                )),
                Motion6::default(),
                SpatialAcceleration6::default(),
            )
            .unwrap();
        let moving = world
            .validate_segment_on_grid_with_root_prediction_buffered(
                &model,
                &segment,
                &root_prediction,
                1_000_000,
                -1.0,
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
            )
            .unwrap();
        assert_close(
            moving.minimum_signed_distance_m,
            fixed.minimum_signed_distance_m - 0.2,
            1e-12,
        );
        assert_eq!(moving.minimum_time_ns, Some(20_000_000));

        let mut maximum_root_twist = Motion6::default();
        root_prediction
            .maximum_abs_twist_between_into(0, 20_000_000, &mut maximum_root_twist)
            .unwrap();
        let continuity = world
            .continuous_clearance_bound_with_root_prediction(
                &scratch,
                &[0.0],
                &root_prediction,
                &maximum_root_twist,
                0.0005,
            )
            .unwrap();
        assert!(continuity.limiting_distance_rate_bound_mps > 0.0);
        assert!(continuity.minimum_clearance_lower_bound_m < moving.minimum_signed_distance_m);

        root_prediction
            .set_error_growth(RootPredictionErrorGrowth {
                initial_translation_radius_m: 0.001,
                translation_velocity_error_bound_mps: 0.05,
                ..RootPredictionErrorGrowth::default()
            })
            .unwrap();
        let uncertain = world
            .validate_segment_on_grid_with_root_prediction_buffered(
                &model,
                &segment,
                &root_prediction,
                1_000_000,
                -1.0,
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
            )
            .unwrap();
        assert_close(
            uncertain.maximum_prediction_clearance_erosion_m,
            0.002,
            1e-12,
        );
        assert_close(
            uncertain.minimum_signed_distance_m,
            moving.minimum_signed_distance_m - 0.002,
            1e-12,
        );
    }

    #[test]
    fn trilinear_plane_distance_and_gradient_are_exact() {
        let grid = plane_grid(SdfOutsidePolicy::Reject);
        let sample = grid.query(Vec3::new(0.17, -0.31, 0.37)).unwrap();
        assert_close(sample.signed_distance_m, 0.37, 1e-12);
        assert_close(sample.gradient_in_control_world.x, 0.0, 1e-12);
        assert_close(sample.gradient_in_control_world.y, 0.0, 1e-12);
        assert_close(sample.gradient_in_control_world.z, 1.0, 1e-12);
        assert_eq!(sample.source, SdfSampleSource::TrilinearGrid);
    }

    #[test]
    fn rotated_affine_field_gradient_matches_world_finite_difference() {
        let dimensions = [4, 4, 4];
        let spacing = Vec3::new(0.4, 0.5, 0.6);
        let rotation = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
        let transform = Transform3::from_parts(Translation3::new(-0.5, 0.2, -0.4), rotation);
        let local_gradient = Vec3::new(0.3, -0.4, 0.5);
        let mut samples = Vec::new();
        for z in 0..dimensions[2] {
            for y in 0..dimensions[1] {
                for x in 0..dimensions[0] {
                    samples.push(local_gradient.dot(&Vec3::new(
                        x as f64 * spacing.x,
                        y as f64 * spacing.y,
                        z as f64 * spacing.z,
                    )));
                }
            }
        }
        let grid = DenseSdfGrid::new(
            transform,
            dimensions,
            spacing,
            samples,
            SdfOutsidePolicy::Reject,
        )
        .unwrap();
        let point = transform
            .transform_point(&Point3::new(0.55, 0.62, 0.71))
            .coords;
        let sample = grid.query(point).unwrap();
        let step = 1e-6;
        for axis in 0..3 {
            let mut plus = point;
            let mut minus = point;
            plus[axis] += step;
            minus[axis] -= step;
            let finite_difference = (grid.query(plus).unwrap().signed_distance_m
                - grid.query(minus).unwrap().signed_distance_m)
                / (2.0 * step);
            assert_close(
                finite_difference,
                sample.gradient_in_control_world[axis],
                2e-9,
            );
        }
    }

    #[test]
    fn outside_policy_rejects_or_returns_occupied_boundary() {
        let rejected = plane_grid(SdfOutsidePolicy::Reject);
        assert!(matches!(
            rejected.query(Vec3::new(1.1, 0.0, 0.0)),
            Err(WorldCollisionError::OutsideKnownField)
        ));

        let occupied = plane_grid(SdfOutsidePolicy::OccupiedBoundary);
        let outside = occupied.query(Vec3::new(1.2, 0.0, 0.0)).unwrap();
        assert_close(outside.signed_distance_m, -0.2, 1e-12);
        assert_close(outside.gradient_in_control_world.x, -1.0, 1e-12);
        assert_eq!(outside.source, SdfSampleSource::OccupiedBoundary);

        let near_inside = occupied.query(Vec3::new(0.95, 0.0, 0.5)).unwrap();
        assert_close(near_inside.signed_distance_m, 0.05, 1e-12);
        assert_eq!(near_inside.source, SdfSampleSource::OccupiedBoundary);
    }

    #[test]
    fn invalid_or_nonfinite_grids_are_rejected() {
        assert!(matches!(
            DenseSdfGrid::new(
                Transform3::identity(),
                [1, 2, 2],
                Vec3::repeat(1.0),
                vec![0.0; 4],
                SdfOutsidePolicy::Reject,
            ),
            Err(WorldCollisionError::InvalidGrid)
        ));
        assert!(matches!(
            DenseSdfGrid::new(
                Transform3::identity(),
                [2, 2, 2],
                Vec3::repeat(1.0),
                vec![f64::NAN; 8],
                SdfOutsidePolicy::Reject,
            ),
            Err(WorldCollisionError::InvalidGrid)
        ));
    }

    #[test]
    fn segment_grid_retains_world_probe_collision_and_continuity_provenance() {
        let model = slider_model();
        let world = slider_world(&model, 1.0, SdfOutsidePolicy::Reject);
        assert_close(world.field_lipschitz_bound, 1.0, 1e-12);
        let segment = slider_segment(0.2);
        let mut state = RobotState::zeros(&model);
        let mut acceleration = DVector::zeros(model.dof);
        let mut cache = ModelCache::new(&model);
        let mut scratch =
            WorldCollisionSweepScratch::with_grid_capacity(model.dof, world.probes.len(), 21);
        let mut sweep = world
            .validate_segment_on_grid_buffered(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.02,
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
            )
            .unwrap();
        assert_eq!(sweep.samples_evaluated, 21);
        assert_eq!(sweep.probe_samples_evaluated, 21);
        assert_eq!(sweep.minimum_probe_id, Some(7));
        assert_eq!(sweep.minimum_body, Some(model.body_id("slider").unwrap()));
        assert_close(sweep.minimum_signed_distance_m, -0.05, 1e-12);
        assert!(sweep.has_sampled_violation());
        assert!(sweep.is_known());
        let continuity = world
            .adaptive_continuous_clearance_bound(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.02,
                3,
                &[20.0],
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
                &mut sweep,
            )
            .unwrap();
        assert_eq!(continuity.limiting_probe_id, Some(7));
        assert_eq!(
            continuity.limiting_body,
            Some(model.body_id("slider").unwrap())
        );
    }

    #[test]
    fn reject_field_turns_segment_exit_into_typed_unknown_evidence() {
        let model = slider_model();
        let world = slider_world(&model, 0.4, SdfOutsidePolicy::Reject);
        let segment = slider_segment(0.2);
        let mut state = RobotState::zeros(&model);
        let mut acceleration = DVector::zeros(model.dof);
        let mut cache = ModelCache::new(&model);
        let mut scratch =
            WorldCollisionSweepScratch::with_grid_capacity(model.dof, world.probes.len(), 21);
        let sweep = world
            .validate_segment_on_grid_buffered(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.02,
                &mut state,
                &mut acceleration,
                &mut cache,
                &mut scratch,
            )
            .unwrap();
        assert!(!sweep.is_known());
        assert_eq!(sweep.first_unknown_probe_id, Some(7));
        assert_eq!(
            sweep.first_unknown_body,
            Some(model.body_id("slider").unwrap())
        );
        assert!(sweep.first_unknown_time_ns.is_some());
        assert!(!sweep.is_clear());
    }
}
