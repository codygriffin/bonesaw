use nalgebra::{DMatrix, Point3, RowDVector};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    math::{ControlTime, Motion6, Transform3, Vec3},
    model::{
        BodyId, CollisionShape, CompiledModel, DynamicsCache, ModelCache, ModelError, RobotState,
    },
    solver::{ConstraintBuffer, LinearConstraint, Priority, Task, TaskBuffer, TaskKind},
    trajectory::{QuinticSegment, TrajectoryError},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SphereProxy {
    pub stable_id: u32,
    pub body: BodyId,
    #[serde(with = "crate::math::serde_transform3")]
    pub body_from_sphere: Transform3,
    pub radius: f64,
    pub quality: DistanceQuality,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrimitiveProxy {
    pub stable_id: u32,
    pub body: BodyId,
    #[serde(with = "crate::math::serde_transform3")]
    pub body_from_shape: Transform3,
    pub geometry: PrimitiveGeometry,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum PrimitiveGeometry {
    Sphere {
        radius: f64,
    },
    /// A conservative capsule around a URDF cylinder, or the exact authored
    /// capsule. This preserves the entire cylindrical volume without the
    /// axial overreach of a chain of cell spheres.
    Capsule {
        radius: f64,
        half_length: f64,
    },
    Box {
        half_extents: Vec3,
    },
}

impl PrimitiveGeometry {
    fn bounding_radius(&self) -> f64 {
        match self {
            Self::Sphere { radius } => *radius,
            Self::Capsule {
                radius,
                half_length,
            } => radius + half_length,
            Self::Box { half_extents } => half_extents.norm(),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct CollisionPair {
    pub stable_id: u32,
    pub a: usize,
    pub b: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum DistanceQuality {
    ExactSphere,
    /// Analytic closest-feature witness on the tight authored primitive proxy.
    /// This covers sphere, capsule, and box combinations whose local feature
    /// derivative is represented directly by the returned point Jacobians.
    AnalyticPrimitive,
    /// Conservative tight-primitive witness. Box-box currently uses the
    /// maximum separating-axis gap, which is a valid lower bound rather than
    /// exact Euclidean distance.
    ConservativePrimitive,
    ConservativeBoundingSphere,
    /// One cell in a deterministic union-of-spheres cover. Each cell sphere
    /// contains its assigned primitive volume, while avoiding the extreme
    /// overreach of one sphere around an entire long limb or broad box.
    ConservativeSphereCover,
}

#[derive(Clone, Debug)]
pub struct DistanceSample {
    pub pair_id: u32,
    pub signed_distance: f64,
    pub normal_in_control_world: Vec3,
    pub point_a_in_control_world: Vec3,
    pub point_b_in_control_world: Vec3,
    pub relative_normal_velocity: f64,
    /// `Jdot * v` contribution to relative normal acceleration at the locally
    /// selected rigid-body features. Curvature from changing closest features
    /// and normal direction is intentionally excluded, making the floating
    /// barrier a named local linearization rather than an exact CCD claim.
    pub relative_normal_acceleration_bias: f64,
    pub jacobian_row: RowDVector<f64>,
    pub quality: DistanceQuality,
}

#[derive(Clone, Copy, Debug)]
struct PrimitiveDistanceWitness {
    signed_distance: f64,
    /// Oriented so `distance_dot = normal · (velocity_b - velocity_a)` for
    /// the locally selected features.
    normal: Vec3,
    feature_a: Vec3,
    feature_b: Vec3,
    surface_a: Vec3,
    surface_b: Vec3,
    quality: DistanceQuality,
}

impl PrimitiveDistanceWitness {
    fn swapped(self) -> Self {
        Self {
            signed_distance: self.signed_distance,
            normal: -self.normal,
            feature_a: self.feature_b,
            feature_b: self.feature_a,
            surface_a: self.surface_b,
            surface_b: self.surface_a,
            quality: self.quality,
        }
    }
}

impl DistanceSample {
    pub fn workspace(dof: usize) -> Self {
        Self {
            pair_id: 0,
            signed_distance: f64::INFINITY,
            normal_in_control_world: Vec3::x(),
            point_a_in_control_world: Vec3::zeros(),
            point_b_in_control_world: Vec3::zeros(),
            relative_normal_velocity: 0.0,
            relative_normal_acceleration_bias: 0.0,
            jacobian_row: RowDVector::zeros(dof),
            quality: DistanceQuality::ConservativeBoundingSphere,
        }
    }
}

/// Reusable storage for collision distance Jacobians.
#[derive(Clone, Debug)]
pub struct CollisionEvaluationScratch {
    jacobian_a: DMatrix<f64>,
    jacobian_b: DMatrix<f64>,
    primitive_world_from_shape: Vec<Transform3>,
    primitive_pair_minimum_distances: Vec<f64>,
    primitive_pair_grid_distances: Vec<f64>,
    interval_max_abs_velocity: Vec<f64>,
}

impl CollisionEvaluationScratch {
    pub fn new(dof: usize, primitive_count: usize, primitive_pair_count: usize) -> Self {
        Self::with_grid_capacity(dof, primitive_count, primitive_pair_count, 0)
    }

    pub fn with_grid_capacity(
        dof: usize,
        primitive_count: usize,
        primitive_pair_count: usize,
        grid_sample_capacity: usize,
    ) -> Self {
        Self {
            jacobian_a: DMatrix::zeros(3, dof),
            jacobian_b: DMatrix::zeros(3, dof),
            primitive_world_from_shape: vec![Transform3::identity(); primitive_count],
            primitive_pair_minimum_distances: vec![f64::INFINITY; primitive_pair_count],
            primitive_pair_grid_distances: vec![
                f64::INFINITY;
                primitive_pair_count
                    .saturating_mul(grid_sample_capacity)
            ],
            interval_max_abs_velocity: vec![0.0; dof],
        }
    }
}

/// Fixed-capacity scratch for floating acceleration barriers. Active rows and
/// their scalar witnesses are retained until the solve completes so the
/// limiting post-solve residual can be reported without re-running geometry.
#[derive(Clone, Debug)]
pub struct CollisionAccelerationBarrierScratch {
    sample: DistanceSample,
    evaluation: CollisionEvaluationScratch,
    active_rows: DMatrix<f64>,
    active_pair_ids: Vec<u32>,
    active_qualities: Vec<DistanceQuality>,
    active_relative_normal_velocities: Vec<f64>,
    active_bias_accelerations: Vec<f64>,
    active_required_normal_accelerations: Vec<f64>,
    active_len: usize,
}

impl CollisionAccelerationBarrierScratch {
    pub fn new(generalized_dof: usize, maximum_pairs: usize) -> Self {
        Self {
            sample: DistanceSample::workspace(generalized_dof),
            evaluation: CollisionEvaluationScratch::new(generalized_dof, 0, 0),
            active_rows: DMatrix::zeros(maximum_pairs, generalized_dof),
            active_pair_ids: vec![0; maximum_pairs],
            active_qualities: vec![DistanceQuality::ExactSphere; maximum_pairs],
            active_relative_normal_velocities: vec![0.0; maximum_pairs],
            active_bias_accelerations: vec![0.0; maximum_pairs],
            active_required_normal_accelerations: vec![0.0; maximum_pairs],
            active_len: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.active_rows.nrows()
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct CollisionSweepReport {
    pub samples_evaluated: usize,
    pub minimum_signed_distance: f64,
    pub minimum_pair_id: Option<u32>,
    pub minimum_time_ns: Option<ControlTime>,
    pub first_violation_pair_id: Option<u32>,
    pub first_violation_time_ns: Option<ControlTime>,
}

/// A pair-consistent lower bound on represented clearance between trajectory
/// samples. The sampled minimum and relative-speed bound always refer to the
/// same primitive pair, avoiding a pessimistic cross-product of unrelated
/// global extrema.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct CollisionContinuityReport {
    pub minimum_clearance_lower_bound: f64,
    pub limiting_pair_id: Option<u32>,
    pub limiting_relative_speed_bound: f64,
    pub leaf_interval_count: usize,
    pub refinement_pair_samples_evaluated: usize,
    pub unresolved_interval_count: usize,
    pub maximum_subdivision_depth_reached: u8,
}

impl CollisionSweepReport {
    pub fn is_clear(&self) -> bool {
        self.first_violation_pair_id.is_none()
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct CollisionAvoidanceConfig {
    /// Signed distance at which the hard barrier permits no approach velocity.
    pub hard_margin: f64,
    /// Distance below which the pair emits hard and soft rows.
    pub influence_margin: f64,
    /// First-order barrier/repeller gain in s⁻¹.
    pub separation_gain: f64,
    /// Clamp on requested separating velocity.
    pub maximum_separation_speed: f64,
    pub soft_weight: f64,
    pub soft_priority: Priority,
    pub hard_stable_id_base: u32,
    pub soft_stable_id_base: u32,
}

/// Locally linearized relative-degree-two collision barrier for the floating
/// acceleration solve.
///
/// For clearance `h = distance - hard_margin`, the emitted inequality is
///
/// `J_distance qdd + bias + 2 zeta omega hdot + omega^2 h >= 0`.
///
/// `bias` is the relative rigid-feature `Jdot * v` projected on the current
/// closest-feature normal. Closest-feature switching and normal curvature are
/// excluded and remain the responsibility of sampled/continuous command
/// admission.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollisionAccelerationBarrierConfig {
    pub hard_margin: f64,
    pub influence_margin: f64,
    pub natural_frequency_rad_s: f64,
    pub damping_ratio: f64,
    pub hard_stable_id_base: u32,
}

impl Default for CollisionAccelerationBarrierConfig {
    fn default() -> Self {
        Self {
            hard_margin: 0.02,
            influence_margin: 0.10,
            natural_frequency_rad_s: 12.0,
            damping_ratio: 1.0,
            hard_stable_id_base: 0x6000_0000,
        }
    }
}

impl CollisionAccelerationBarrierConfig {
    pub fn validate(self) -> bool {
        self.hard_margin.is_finite()
            && self.influence_margin.is_finite()
            && self.influence_margin >= self.hard_margin
            && self.natural_frequency_rad_s.is_finite()
            && self.natural_frequency_rad_s > 0.0
            && self.damping_ratio.is_finite()
            && self.damping_ratio >= 0.0
    }
}

/// Typed collision-viability evidence from one floating WBC query. The
/// closest represented pair and the most active emitted barrier remain
/// separate so an inactive safe pair is never presented as a hard row.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CollisionAccelerationBarrierEvidence {
    pub enabled: bool,
    pub represented_pair_count: usize,
    pub unsupported_shape_count: usize,
    pub active_pair_count: usize,
    pub minimum_signed_distance_m: f64,
    pub minimum_margin_m: f64,
    pub closest_pair_id: Option<u32>,
    pub closest_quality: Option<DistanceQuality>,
    pub limiting_active_pair_id: Option<u32>,
    pub limiting_active_quality: Option<DistanceQuality>,
    pub limiting_relative_normal_velocity_mps: f64,
    pub limiting_normal_bias_acceleration_mps2: f64,
    pub limiting_required_normal_acceleration_mps2: f64,
    pub limiting_achieved_normal_acceleration_mps2: f64,
    pub minimum_barrier_residual_mps2: f64,
}

impl CollisionAccelerationBarrierEvidence {
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            represented_pair_count: 0,
            unsupported_shape_count: 0,
            active_pair_count: 0,
            minimum_signed_distance_m: f64::INFINITY,
            minimum_margin_m: f64::INFINITY,
            closest_pair_id: None,
            closest_quality: None,
            limiting_active_pair_id: None,
            limiting_active_quality: None,
            limiting_relative_normal_velocity_mps: 0.0,
            limiting_normal_bias_acceleration_mps2: 0.0,
            limiting_required_normal_acceleration_mps2: 0.0,
            limiting_achieved_normal_acceleration_mps2: 0.0,
            minimum_barrier_residual_mps2: f64::INFINITY,
        }
    }
}

impl Default for CollisionAccelerationBarrierEvidence {
    fn default() -> Self {
        Self::disabled()
    }
}

#[derive(Debug, Error)]
pub enum CollisionError {
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error(transparent)]
    Trajectory(#[from] TrajectoryError),
    #[error("collision avoidance configuration is invalid")]
    InvalidConfig,
    #[error("collision clearance must be finite")]
    InvalidClearance,
    #[error("compiled task scratch capacity is too small for collision repellers")]
    TaskCapacity,
    #[error("compiled constraint scratch capacity is too small for collision barriers")]
    ConstraintCapacity,
}

impl Default for CollisionAvoidanceConfig {
    fn default() -> Self {
        Self {
            hard_margin: 0.02,
            influence_margin: 0.10,
            separation_gain: 8.0,
            maximum_separation_speed: 1.0,
            soft_weight: 0.25,
            soft_priority: Priority::Viability,
            hard_stable_id_base: 0x4000_0000,
            soft_stable_id_base: 0x5000_0000,
        }
    }
}

impl CollisionAvoidanceConfig {
    pub fn validate(self) -> bool {
        self.hard_margin.is_finite()
            && self.influence_margin.is_finite()
            && self.influence_margin >= self.hard_margin
            && self.separation_gain.is_finite()
            && self.separation_gain >= 0.0
            && self.maximum_separation_speed.is_finite()
            && self.maximum_separation_speed >= 0.0
            && self.soft_weight.is_finite()
            && self.soft_weight >= 0.0
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CompiledCollisionModel {
    pub spheres: Vec<SphereProxy>,
    pub self_pairs: Vec<CollisionPair>,
    /// One tight conservative primitive per supported authored collision
    /// shape. Command admission and canonical fixed-root avoidance share this
    /// table; sphere covers remain an explicitly named fallback.
    #[serde(default)]
    pub validation_primitives: Vec<PrimitiveProxy>,
    #[serde(default)]
    pub validation_pairs: Vec<CollisionPair>,
    /// Potential primitive pairs removed because their bodies are connected
    /// only through collisionless kinematic carrier links. URDFs commonly use
    /// such links to stack intersecting joint axes; treating the carrier as a
    /// physical separation creates false self-collisions between the actual
    /// neighboring shapes.
    #[serde(default)]
    pub virtual_bridge_exclusion_count: usize,
    /// Authored collision shapes that the high-rate conservative proxy cannot
    /// represent. Callers must apply an explicit policy before admission.
    #[serde(default)]
    pub unsupported_shape_count: usize,
    /// Flat `[sphere][coordinate]` conservative center-speed coefficients:
    /// `|v_sphere| <= sum(coeff * |qdot|)` for a fixed root pose.
    #[serde(default)]
    pub sphere_speed_coefficients: Vec<f64>,
    #[serde(default)]
    pub primitive_speed_coefficients: Vec<f64>,
    #[serde(default)]
    pub speed_coordinate_count: usize,
}

impl CompiledCollisionModel {
    pub fn pair_bodies(&self, stable_id: u32) -> Option<(BodyId, BodyId)> {
        let pair = self.validation_pairs.get(stable_id as usize)?;
        (pair.stable_id == stable_id).then(|| {
            (
                self.validation_primitives[pair.a].body,
                self.validation_primitives[pair.b].body,
            )
        })
    }

    /// Compile a deterministic conservative sphere proxy for every primitive
    /// collision shape. Physically adjacent bodies are excluded from
    /// self-pairs, including adjacency bridged by collisionless kinematic
    /// carrier links used to stack joint axes.
    pub fn compile(model: &CompiledModel) -> Self {
        let mut spheres = Vec::new();
        let mut validation_primitives = Vec::new();
        let mut unsupported_shape_count = 0;
        for body in &model.bodies {
            for shape in &body.collisions {
                let mut push =
                    |body_from_sphere: Transform3, radius: f64, quality: DistanceQuality| {
                        if radius.is_finite() && radius > 0.0 {
                            spheres.push(SphereProxy {
                                stable_id: spheres.len() as u32,
                                body: body.id,
                                body_from_sphere,
                                radius,
                                quality,
                            });
                        }
                    };
                match shape {
                    CollisionShape::Sphere {
                        radius,
                        body_from_shape,
                    } => {
                        validation_primitives.push(PrimitiveProxy {
                            stable_id: validation_primitives.len() as u32,
                            body: body.id,
                            body_from_shape: *body_from_shape,
                            geometry: PrimitiveGeometry::Sphere { radius: *radius },
                        });
                        push(*body_from_shape, *radius, DistanceQuality::ExactSphere);
                    }
                    CollisionShape::Cylinder {
                        radius,
                        half_length,
                        body_from_shape,
                    } => {
                        validation_primitives.push(PrimitiveProxy {
                            stable_id: validation_primitives.len() as u32,
                            body: body.id,
                            body_from_shape: *body_from_shape,
                            geometry: PrimitiveGeometry::Capsule {
                                radius: *radius,
                                half_length: *half_length,
                            },
                        });
                        append_cylinder_sphere_cover(
                            &mut push,
                            *body_from_shape,
                            *radius,
                            *half_length,
                        );
                    }
                    CollisionShape::Capsule {
                        radius,
                        half_length,
                        body_from_shape,
                    } => {
                        validation_primitives.push(PrimitiveProxy {
                            stable_id: validation_primitives.len() as u32,
                            body: body.id,
                            body_from_shape: *body_from_shape,
                            geometry: PrimitiveGeometry::Capsule {
                                radius: *radius,
                                half_length: *half_length,
                            },
                        });
                        append_cylinder_sphere_cover(
                            &mut push,
                            *body_from_shape,
                            *radius,
                            *half_length,
                        );
                        for z in [-*half_length, *half_length] {
                            push(
                                *body_from_shape * Transform3::translation(0.0, 0.0, z),
                                *radius,
                                DistanceQuality::ConservativeSphereCover,
                            );
                        }
                    }
                    CollisionShape::Box {
                        half_extents,
                        body_from_shape,
                    } => {
                        validation_primitives.push(PrimitiveProxy {
                            stable_id: validation_primitives.len() as u32,
                            body: body.id,
                            body_from_shape: *body_from_shape,
                            geometry: PrimitiveGeometry::Box {
                                half_extents: *half_extents,
                            },
                        });
                        append_box_sphere_cover(&mut push, *body_from_shape, *half_extents);
                    }
                    CollisionShape::Mesh { .. } => {
                        unsupported_shape_count += 1;
                    }
                }
            }
        }

        let mut self_pairs = Vec::new();
        for a in 0..spheres.len() {
            for b in a + 1..spheres.len() {
                let body_a = spheres[a].body;
                let body_b = spheres[b].body;
                if body_a == body_b {
                    continue;
                }
                if collision_adjacent(model, body_a, body_b) {
                    continue;
                }
                self_pairs.push(CollisionPair {
                    stable_id: self_pairs.len() as u32,
                    a,
                    b,
                });
            }
        }
        let mut validation_pairs = Vec::new();
        let mut virtual_bridge_exclusion_count = 0;
        for a in 0..validation_primitives.len() {
            for b in a + 1..validation_primitives.len() {
                let body_a = validation_primitives[a].body;
                let body_b = validation_primitives[b].body;
                if body_a == body_b {
                    continue;
                }
                if collision_adjacent(model, body_a, body_b) {
                    if !adjacent(model, body_a, body_b) {
                        virtual_bridge_exclusion_count += 1;
                    }
                    continue;
                }
                validation_pairs.push(CollisionPair {
                    stable_id: validation_pairs.len() as u32,
                    a,
                    b,
                });
            }
        }
        let sphere_speed_coefficients = spheres
            .iter()
            .flat_map(|sphere| sphere_speed_coefficients(model, sphere))
            .collect();
        let primitive_speed_coefficients = validation_primitives
            .iter()
            .flat_map(|primitive| primitive_speed_coefficients(model, primitive))
            .collect();
        Self {
            spheres,
            self_pairs,
            validation_primitives,
            validation_pairs,
            virtual_bridge_exclusion_count,
            unsupported_shape_count,
            sphere_speed_coefficients,
            primitive_speed_coefficients,
            speed_coordinate_count: model.dof,
        }
    }

    /// Conservative fixed-root relative center-speed bound across every
    /// compiled self-pair. Returns `None` for a mismatched or legacy layout.
    pub fn maximum_relative_speed_bound(&self, maximum_joint_velocity: &[f64]) -> Option<f64> {
        if maximum_joint_velocity.len() != self.speed_coordinate_count
            || self.primitive_speed_coefficients.len()
                != self
                    .validation_primitives
                    .len()
                    .saturating_mul(self.speed_coordinate_count)
        {
            return None;
        }
        let mut maximum = 0.0_f64;
        for pair in &self.validation_pairs {
            let a = pair.a * self.speed_coordinate_count;
            let b = pair.b * self.speed_coordinate_count;
            let mut bound = 0.0;
            for coordinate in 0..self.speed_coordinate_count {
                bound += (self.primitive_speed_coefficients[a + coordinate]
                    + self.primitive_speed_coefficients[b + coordinate])
                    * maximum_joint_velocity[coordinate];
            }
            maximum = maximum.max(bound);
        }
        Some(maximum)
    }

    /// Pair-tight fixed-root continuous-clearance certificate for the most
    /// recent buffered segment scan. Returns `None` for a mismatched or legacy
    /// scratch/coefficient layout.
    pub fn pairwise_continuous_clearance_bound(
        &self,
        evaluation: &CollisionEvaluationScratch,
        maximum_joint_velocity: &[f64],
        half_interval_seconds: f64,
    ) -> Option<CollisionContinuityReport> {
        if maximum_joint_velocity.len() != self.speed_coordinate_count
            || !half_interval_seconds.is_finite()
            || half_interval_seconds < 0.0
            || self.primitive_speed_coefficients.len()
                != self
                    .validation_primitives
                    .len()
                    .saturating_mul(self.speed_coordinate_count)
            || evaluation.primitive_pair_minimum_distances.len() != self.validation_pairs.len()
        {
            return None;
        }

        let mut report = CollisionContinuityReport {
            minimum_clearance_lower_bound: f64::INFINITY,
            limiting_pair_id: None,
            limiting_relative_speed_bound: 0.0,
            leaf_interval_count: 0,
            refinement_pair_samples_evaluated: 0,
            unresolved_interval_count: 0,
            maximum_subdivision_depth_reached: 0,
        };
        for (pair_index, pair) in self.validation_pairs.iter().enumerate() {
            let a = pair.a * self.speed_coordinate_count;
            let b = pair.b * self.speed_coordinate_count;
            let mut relative_speed_bound = 0.0;
            for coordinate in 0..self.speed_coordinate_count {
                relative_speed_bound += (self.primitive_speed_coefficients[a + coordinate]
                    + self.primitive_speed_coefficients[b + coordinate])
                    * maximum_joint_velocity[coordinate];
            }
            let clearance_lower_bound = evaluation.primitive_pair_minimum_distances[pair_index]
                - relative_speed_bound * half_interval_seconds;
            if clearance_lower_bound < report.minimum_clearance_lower_bound {
                report.minimum_clearance_lower_bound = clearance_lower_bound;
                report.limiting_pair_id = Some(pair.stable_id);
                report.limiting_relative_speed_bound = relative_speed_bound;
            }
        }
        Some(report)
    }

    /// Refine only pair/interval leaves whose local Lipschitz certificate does
    /// not yet establish the required clearance. Midpoints are real geometry
    /// samples: any discovered violation is added to `sweep`, while an
    /// unresolved leaf remains distinct continuous-certification evidence.
    #[allow(clippy::too_many_arguments)]
    pub fn adaptive_pairwise_continuous_clearance_bound(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        control_world_from_root: Transform3,
        sample_period_ns: i64,
        required_clearance: f64,
        maximum_subdivision_depth: u8,
        maximum_joint_velocity: &[f64],
        state_scratch: &mut RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        evaluation: &mut CollisionEvaluationScratch,
        sweep: &mut CollisionSweepReport,
    ) -> Result<CollisionContinuityReport, CollisionError> {
        let intervals = (segment.duration_ns / sample_period_ns) as usize;
        let pair_count = self.validation_pairs.len();
        let required_grid_values = (intervals + 1).saturating_mul(pair_count);
        if !required_clearance.is_finite()
            || sample_period_ns <= 0
            || segment.duration_ns % sample_period_ns != 0
            || maximum_subdivision_depth > 12
            || maximum_joint_velocity.len() != model.dof
            || evaluation.primitive_pair_grid_distances.len() < required_grid_values
            || evaluation.interval_max_abs_velocity.len() != model.dof
        {
            return Err(TrajectoryError::Dimension.into());
        }

        state_scratch.control_world_from_root = control_world_from_root;
        let mut continuity = CollisionContinuityReport {
            minimum_clearance_lower_bound: f64::INFINITY,
            limiting_pair_id: None,
            limiting_relative_speed_bound: 0.0,
            leaf_interval_count: 0,
            refinement_pair_samples_evaluated: 0,
            unresolved_interval_count: 0,
            maximum_subdivision_depth_reached: 0,
        };
        let base_half_interval_seconds = 0.5 * sample_period_ns as f64 * 1e-9;
        for (pair_index, pair) in self.validation_pairs.iter().enumerate() {
            let broad_phase_speed = self
                .relative_speed_bound_for_pair(*pair, maximum_joint_velocity)
                .ok_or(TrajectoryError::Dimension)?;
            let broad_phase_lower = evaluation.primitive_pair_minimum_distances[pair_index]
                - broad_phase_speed * base_half_interval_seconds;
            if broad_phase_lower >= required_clearance {
                continuity.leaf_interval_count += intervals;
                if broad_phase_lower < continuity.minimum_clearance_lower_bound {
                    continuity.minimum_clearance_lower_bound = broad_phase_lower;
                    continuity.limiting_pair_id = Some(pair.stable_id);
                    continuity.limiting_relative_speed_bound = broad_phase_speed;
                }
                continue;
            }
            for interval in 0..intervals {
                let left_time_ns = segment.start_time_ns + interval as i64 * sample_period_ns;
                let right_time_ns = left_time_ns + sample_period_ns;
                let left_distance =
                    evaluation.primitive_pair_grid_distances[interval * pair_count + pair_index];
                let right_distance = evaluation.primitive_pair_grid_distances
                    [(interval + 1) * pair_count + pair_index];
                self.refine_pair_interval(
                    model,
                    segment,
                    required_clearance,
                    maximum_subdivision_depth,
                    pair_index,
                    *pair,
                    left_time_ns,
                    right_time_ns,
                    left_distance,
                    right_distance,
                    0,
                    state_scratch,
                    acceleration_scratch,
                    model_scratch,
                    evaluation,
                    sweep,
                    &mut continuity,
                )?;
                if !sweep.is_clear() {
                    return Ok(continuity);
                }
            }
        }
        Ok(continuity)
    }

    #[allow(clippy::too_many_arguments)]
    fn refine_pair_interval(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        required_clearance: f64,
        maximum_subdivision_depth: u8,
        pair_index: usize,
        pair: CollisionPair,
        left_time_ns: ControlTime,
        right_time_ns: ControlTime,
        left_distance: f64,
        right_distance: f64,
        depth: u8,
        state_scratch: &mut RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        evaluation: &mut CollisionEvaluationScratch,
        sweep: &mut CollisionSweepReport,
        continuity: &mut CollisionContinuityReport,
    ) -> Result<(), CollisionError> {
        segment.maximum_abs_velocity_between_into(
            left_time_ns,
            right_time_ns,
            &mut evaluation.interval_max_abs_velocity,
        )?;
        let relative_speed_bound = self
            .relative_speed_bound_for_pair(pair, &evaluation.interval_max_abs_velocity)
            .ok_or(TrajectoryError::Dimension)?;
        let half_interval_seconds = 0.5 * (right_time_ns - left_time_ns) as f64 * 1e-9;
        let lower_bound =
            left_distance.min(right_distance) - relative_speed_bound * half_interval_seconds;
        let midpoint_time_ns = left_time_ns + (right_time_ns - left_time_ns) / 2;
        let may_refine = lower_bound < required_clearance
            && depth < maximum_subdivision_depth
            && midpoint_time_ns > left_time_ns
            && midpoint_time_ns < right_time_ns;
        if !may_refine {
            continuity.leaf_interval_count += 1;
            continuity.maximum_subdivision_depth_reached =
                continuity.maximum_subdivision_depth_reached.max(depth);
            if lower_bound < required_clearance {
                continuity.unresolved_interval_count += 1;
            }
            if lower_bound < continuity.minimum_clearance_lower_bound {
                continuity.minimum_clearance_lower_bound = lower_bound;
                continuity.limiting_pair_id = Some(pair.stable_id);
                continuity.limiting_relative_speed_bound = relative_speed_bound;
            }
            return Ok(());
        }

        let midpoint_distance = self.primitive_pair_distance_at_time(
            model,
            segment,
            pair_index,
            midpoint_time_ns,
            state_scratch,
            acceleration_scratch,
            model_scratch,
        )?;
        continuity.refinement_pair_samples_evaluated += 1;
        record_sweep_distance(
            sweep,
            pair.stable_id,
            midpoint_time_ns,
            midpoint_distance,
            required_clearance,
        );
        if midpoint_distance < required_clearance {
            continuity.leaf_interval_count += 1;
            continuity.unresolved_interval_count += 1;
            continuity.maximum_subdivision_depth_reached =
                continuity.maximum_subdivision_depth_reached.max(depth + 1);
            if lower_bound < continuity.minimum_clearance_lower_bound {
                continuity.minimum_clearance_lower_bound = lower_bound;
                continuity.limiting_pair_id = Some(pair.stable_id);
                continuity.limiting_relative_speed_bound = relative_speed_bound;
            }
            return Ok(());
        }
        self.refine_pair_interval(
            model,
            segment,
            required_clearance,
            maximum_subdivision_depth,
            pair_index,
            pair,
            left_time_ns,
            midpoint_time_ns,
            left_distance,
            midpoint_distance,
            depth + 1,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            evaluation,
            sweep,
            continuity,
        )?;
        self.refine_pair_interval(
            model,
            segment,
            required_clearance,
            maximum_subdivision_depth,
            pair_index,
            pair,
            midpoint_time_ns,
            right_time_ns,
            midpoint_distance,
            right_distance,
            depth + 1,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            evaluation,
            sweep,
            continuity,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn primitive_pair_distance_at_time(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        pair_index: usize,
        time_ns: ControlTime,
        state_scratch: &mut RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
    ) -> Result<f64, CollisionError> {
        let pair = self.validation_pairs[pair_index];
        segment.evaluate_into(
            time_ns,
            state_scratch.q.as_mut_slice(),
            state_scratch.v.as_mut_slice(),
            acceleration_scratch.as_mut_slice(),
        )?;
        model.forward_kinematics(state_scratch, model_scratch)?;
        let primitive_a = &self.validation_primitives[pair.a];
        let primitive_b = &self.validation_primitives[pair.b];
        let world_from_a =
            model_scratch.world_from_body[primitive_a.body.0] * primitive_a.body_from_shape;
        let world_from_b =
            model_scratch.world_from_body[primitive_b.body.0] * primitive_b.body_from_shape;
        Ok(primitive_signed_distance(
            &primitive_a.geometry,
            world_from_a,
            &primitive_b.geometry,
            world_from_b,
        ))
    }

    fn relative_speed_bound_for_pair(
        &self,
        pair: CollisionPair,
        maximum_joint_velocity: &[f64],
    ) -> Option<f64> {
        if maximum_joint_velocity.len() != self.speed_coordinate_count
            || self.primitive_speed_coefficients.len()
                != self
                    .validation_primitives
                    .len()
                    .saturating_mul(self.speed_coordinate_count)
        {
            return None;
        }
        let a = pair.a * self.speed_coordinate_count;
        let b = pair.b * self.speed_coordinate_count;
        let mut bound = 0.0;
        for coordinate in 0..self.speed_coordinate_count {
            bound += (self.primitive_speed_coefficients[a + coordinate]
                + self.primitive_speed_coefficients[b + coordinate])
                * maximum_joint_velocity[coordinate];
        }
        Some(bound)
    }

    pub fn evaluate_pair(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        pair: CollisionPair,
    ) -> Result<DistanceSample, ModelError> {
        let mut sample = DistanceSample::workspace(model.dof);
        let mut scratch = CollisionEvaluationScratch::new(model.dof, 0, 0);
        self.evaluate_pair_into(
            model,
            cache,
            generalized_velocity,
            pair,
            &mut sample,
            &mut scratch,
        )?;
        Ok(sample)
    }

    pub fn evaluate_pair_into(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        pair: CollisionPair,
        sample: &mut DistanceSample,
        scratch: &mut CollisionEvaluationScratch,
    ) -> Result<(), ModelError> {
        let a = &self.spheres[pair.a];
        let b = &self.spheres[pair.b];
        let world_from_a = cache.world_from_body[a.body.0] * a.body_from_sphere;
        let world_from_b = cache.world_from_body[b.body.0] * b.body_from_sphere;
        let center_a = world_from_a.translation.vector;
        let center_b = world_from_b.translation.vector;
        let delta = center_b - center_a;
        let center_distance = delta.norm();
        let normal = if center_distance > 1e-12 {
            delta / center_distance
        } else {
            // Stable deterministic normal for coincident centers.
            Vec3::x()
        };
        let point_a = center_a + a.radius * normal;
        let point_b = center_b - b.radius * normal;
        let offset_a_body = cache.world_from_body[a.body.0]
            .inverse()
            .transform_point(&Point3::from(center_a))
            .coords;
        let offset_b_body = cache.world_from_body[b.body.0]
            .inverse()
            .transform_point(&Point3::from(center_b))
            .coords;
        model.point_jacobian_into(
            cache,
            model.bodies[a.body.0].body_frame,
            offset_a_body,
            &mut scratch.jacobian_a,
        )?;
        model.point_jacobian_into(
            cache,
            model.bodies[b.body.0].body_frame,
            offset_b_body,
            &mut scratch.jacobian_b,
        )?;
        for column in 0..model.dof {
            sample.jacobian_row[column] = (0..3)
                .map(|axis| {
                    normal[axis]
                        * (scratch.jacobian_b[(axis, column)] - scratch.jacobian_a[(axis, column)])
                })
                .sum();
        }
        let relative_normal_velocity = sample
            .jacobian_row
            .iter()
            .zip(generalized_velocity.iter())
            .map(|(jacobian, velocity)| jacobian * velocity)
            .sum();
        sample.pair_id = pair.stable_id;
        sample.signed_distance = center_distance - a.radius - b.radius;
        sample.normal_in_control_world = normal;
        sample.point_a_in_control_world = point_a;
        sample.point_b_in_control_world = point_b;
        sample.relative_normal_velocity = relative_normal_velocity;
        sample.relative_normal_acceleration_bias = 0.0;
        sample.quality = match (a.quality, b.quality) {
            (DistanceQuality::ExactSphere, DistanceQuality::ExactSphere) => {
                DistanceQuality::ExactSphere
            }
            (DistanceQuality::ConservativeBoundingSphere, _)
            | (_, DistanceQuality::ConservativeBoundingSphere) => {
                DistanceQuality::ConservativeBoundingSphere
            }
            _ => DistanceQuality::ConservativeSphereCover,
        };
        Ok(())
    }

    /// Evaluate one pair from the tight primitive table used by command
    /// admission. The returned Jacobian is built at the same locally selected
    /// closest features that produced the signed distance, so avoidance and
    /// admission share pair IDs, geometry, and distance semantics.
    pub fn evaluate_validation_pair(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        pair: CollisionPair,
    ) -> Result<DistanceSample, ModelError> {
        let mut sample = DistanceSample::workspace(model.dof);
        let mut scratch = CollisionEvaluationScratch::new(model.dof, 0, 0);
        self.evaluate_validation_pair_into(
            model,
            cache,
            generalized_velocity,
            pair,
            &mut sample,
            &mut scratch,
        )?;
        Ok(sample)
    }

    pub fn evaluate_validation_pair_into(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        pair: CollisionPair,
        sample: &mut DistanceSample,
        scratch: &mut CollisionEvaluationScratch,
    ) -> Result<(), ModelError> {
        let a = &self.validation_primitives[pair.a];
        let b = &self.validation_primitives[pair.b];
        let world_from_a = cache.world_from_body[a.body.0] * a.body_from_shape;
        let world_from_b = cache.world_from_body[b.body.0] * b.body_from_shape;
        let witness =
            primitive_distance_witness(&a.geometry, world_from_a, &b.geometry, world_from_b);
        let offset_a_body = cache.world_from_body[a.body.0]
            .inverse()
            .transform_point(&Point3::from(witness.feature_a))
            .coords;
        let offset_b_body = cache.world_from_body[b.body.0]
            .inverse()
            .transform_point(&Point3::from(witness.feature_b))
            .coords;
        model.point_jacobian_into(
            cache,
            model.bodies[a.body.0].body_frame,
            offset_a_body,
            &mut scratch.jacobian_a,
        )?;
        model.point_jacobian_into(
            cache,
            model.bodies[b.body.0].body_frame,
            offset_b_body,
            &mut scratch.jacobian_b,
        )?;
        for column in 0..model.dof {
            sample.jacobian_row[column] = (0..3)
                .map(|axis| {
                    witness.normal[axis]
                        * (scratch.jacobian_b[(axis, column)] - scratch.jacobian_a[(axis, column)])
                })
                .sum();
        }
        sample.pair_id = pair.stable_id;
        sample.signed_distance = witness.signed_distance;
        sample.normal_in_control_world = witness.normal;
        sample.point_a_in_control_world = witness.surface_a;
        sample.point_b_in_control_world = witness.surface_b;
        sample.relative_normal_velocity = sample
            .jacobian_row
            .iter()
            .zip(generalized_velocity.iter())
            .map(|(jacobian, velocity)| jacobian * velocity)
            .sum();
        sample.relative_normal_acceleration_bias = 0.0;
        sample.quality = witness.quality;
        Ok(())
    }

    /// Floating-root form of the shared tight-primitive witness. The returned
    /// row is ordered `[root angular; root linear; joints]`; its velocity and
    /// rigid-feature bias terms use the same world-expressed tangent convention
    /// as [`crate::dynamic_wbc::FloatingDynamicWbc`].
    #[allow(clippy::too_many_arguments)]
    pub fn evaluate_validation_pair_floating_into(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        dynamics: &DynamicsCache,
        root_twist_world: Motion6,
        joint_velocity: &nalgebra::DVector<f64>,
        pair: CollisionPair,
        sample: &mut DistanceSample,
        scratch: &mut CollisionEvaluationScratch,
    ) -> Result<(), ModelError> {
        let generalized_dof = model.dof + 6;
        if sample.jacobian_row.len() != generalized_dof
            || joint_velocity.len() != model.dof
            || scratch.jacobian_a.ncols() != generalized_dof
            || scratch.jacobian_b.ncols() != generalized_dof
        {
            return Err(ModelError::AccelerationDimension {
                expected: generalized_dof,
                actual: sample.jacobian_row.len(),
            });
        }
        let a = &self.validation_primitives[pair.a];
        let b = &self.validation_primitives[pair.b];
        let world_from_a = cache.world_from_body[a.body.0] * a.body_from_shape;
        let world_from_b = cache.world_from_body[b.body.0] * b.body_from_shape;
        let witness =
            primitive_distance_witness(&a.geometry, world_from_a, &b.geometry, world_from_b);
        let offset_a_body = cache.world_from_body[a.body.0]
            .inverse()
            .transform_point(&Point3::from(witness.feature_a))
            .coords;
        let offset_b_body = cache.world_from_body[b.body.0]
            .inverse()
            .transform_point(&Point3::from(witness.feature_b))
            .coords;
        model.floating_point_jacobian_into(
            cache,
            model.bodies[a.body.0].body_frame,
            offset_a_body,
            &mut scratch.jacobian_a,
        )?;
        model.floating_point_jacobian_into(
            cache,
            model.bodies[b.body.0].body_frame,
            offset_b_body,
            &mut scratch.jacobian_b,
        )?;
        for column in 0..generalized_dof {
            sample.jacobian_row[column] = (0..3)
                .map(|axis| {
                    witness.normal[axis]
                        * (scratch.jacobian_b[(axis, column)] - scratch.jacobian_a[(axis, column)])
                })
                .sum();
        }
        sample.pair_id = pair.stable_id;
        sample.signed_distance = witness.signed_distance;
        sample.normal_in_control_world = witness.normal;
        sample.point_a_in_control_world = witness.surface_a;
        sample.point_b_in_control_world = witness.surface_b;
        sample.relative_normal_velocity = (0..6)
            .map(|coordinate| sample.jacobian_row[coordinate] * root_twist_world.0[coordinate])
            .sum::<f64>()
            + (0..model.dof)
                .map(|coordinate| sample.jacobian_row[6 + coordinate] * joint_velocity[coordinate])
                .sum::<f64>();
        let bias_a = model.point_bias_acceleration_world(
            model.bodies[a.body.0].body_frame,
            offset_a_body,
            cache,
            dynamics,
        )?;
        let bias_b = model.point_bias_acceleration_world(
            model.bodies[b.body.0].body_frame,
            offset_b_body,
            cache,
            dynamics,
        )?;
        sample.relative_normal_acceleration_bias = witness.normal.dot(&(bias_b - bias_a));
        sample.quality = witness.quality;
        Ok(())
    }

    /// Emit locally linearized acceleration-level collision barriers into the
    /// floating WBC hard-row buffer while retaining typed pair evidence.
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
        scratch: &mut CollisionAccelerationBarrierScratch,
        evidence: &mut CollisionAccelerationBarrierEvidence,
    ) -> Result<(), CollisionError> {
        if !config.validate() {
            return Err(CollisionError::InvalidConfig);
        }
        if scratch.capacity() < self.validation_pairs.len()
            || scratch.active_rows.ncols() != model.dof + 6
        {
            return Err(CollisionError::ConstraintCapacity);
        }
        *evidence = CollisionAccelerationBarrierEvidence {
            enabled: true,
            represented_pair_count: self.validation_pairs.len(),
            unsupported_shape_count: self.unsupported_shape_count,
            ..CollisionAccelerationBarrierEvidence::disabled()
        };
        scratch.active_len = 0;
        let omega = config.natural_frequency_rad_s;
        let damping = 2.0 * config.damping_ratio * omega;
        for pair in &self.validation_pairs {
            self.evaluate_validation_pair_floating_into(
                model,
                cache,
                dynamics,
                root_twist_world,
                joint_velocity,
                *pair,
                &mut scratch.sample,
                &mut scratch.evaluation,
            )?;
            let margin = scratch.sample.signed_distance - config.hard_margin;
            if scratch.sample.signed_distance < evidence.minimum_signed_distance_m {
                evidence.minimum_signed_distance_m = scratch.sample.signed_distance;
                evidence.minimum_margin_m = margin;
                evidence.closest_pair_id = Some(pair.stable_id);
                evidence.closest_quality = Some(scratch.sample.quality);
            }
            if scratch.sample.signed_distance >= config.influence_margin {
                continue;
            }
            let required_normal_acceleration =
                -damping * scratch.sample.relative_normal_velocity - omega * omega * margin;
            let controllable_lower =
                required_normal_acceleration - scratch.sample.relative_normal_acceleration_bias;
            let row = hard_rows.push().ok_or(CollisionError::ConstraintCapacity)?;
            row.stable_id = config.hard_stable_id_base.saturating_add(pair.stable_id);
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
            scratch.active_pair_ids[active] = pair.stable_id;
            scratch.active_qualities[active] = scratch.sample.quality;
            scratch.active_relative_normal_velocities[active] =
                scratch.sample.relative_normal_velocity;
            scratch.active_bias_accelerations[active] =
                scratch.sample.relative_normal_acceleration_bias;
            scratch.active_required_normal_accelerations[active] = required_normal_acceleration;
            scratch.active_len += 1;
        }
        evidence.active_pair_count = scratch.active_len;
        Ok(())
    }

    /// Finish barrier telemetry using the solved generalized acceleration. The
    /// limiting active pair is the row with minimum post-solve HOCBF residual,
    /// not necessarily the geometrically closest pair.
    pub fn finalize_floating_acceleration_barrier_evidence(
        &self,
        generalized_acceleration: &nalgebra::DVector<f64>,
        scratch: &CollisionAccelerationBarrierScratch,
        evidence: &mut CollisionAccelerationBarrierEvidence,
    ) -> Result<(), CollisionError> {
        if generalized_acceleration.len() != scratch.active_rows.ncols() {
            return Err(CollisionError::ConstraintCapacity);
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
                evidence.limiting_active_pair_id = Some(scratch.active_pair_ids[active]);
                evidence.limiting_active_quality = Some(scratch.active_qualities[active]);
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

    /// Append control-barrier inequalities and soft repeller rows for every
    /// pair inside the influence margin. The barrier is
    ///
    /// `J_distance qdot >= -gain * (distance - hard_margin)`.
    ///
    /// It allows bounded approach outside the hard margin and commands
    /// separation after penetration. Stable pair IDs make row ordering and
    /// replay independent of discovery order.
    pub fn emit_sphere_cover_avoidance_rows(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        config: CollisionAvoidanceConfig,
        hard_rows: &mut Vec<LinearConstraint>,
        soft_tasks: &mut Vec<Task>,
    ) -> Result<(), CollisionError> {
        if !config.validate() {
            return Err(CollisionError::InvalidConfig);
        }
        for pair in &self.self_pairs {
            let sample = self.evaluate_pair(model, cache, generalized_velocity, *pair)?;
            if sample.signed_distance >= config.influence_margin {
                continue;
            }
            let lower = (-config.separation_gain * (sample.signed_distance - config.hard_margin))
                .clamp(
                    -config.maximum_separation_speed,
                    config.maximum_separation_speed,
                );
            hard_rows.push(LinearConstraint {
                stable_id: config.hard_stable_id_base.saturating_add(pair.stable_id),
                coefficients: sample.jacobian_row.clone(),
                lower,
                upper: f64::INFINITY,
            });

            if config.soft_weight > 0.0 {
                let target_velocity = (config.separation_gain
                    * (config.influence_margin - sample.signed_distance))
                    .clamp(0.0, config.maximum_separation_speed);
                soft_tasks.push(Task {
                    stable_id: config.soft_stable_id_base.saturating_add(pair.stable_id),
                    kind: TaskKind::Repeller,
                    priority: config.soft_priority,
                    jacobian: DMatrix::from_row_slice(1, model.dof, sample.jacobian_row.as_slice()),
                    target_velocity: nalgebra::DVector::from_element(1, target_velocity),
                    weight: config.soft_weight,
                });
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn emit_sphere_cover_avoidance_rows_buffered(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        config: CollisionAvoidanceConfig,
        hard_rows: &mut ConstraintBuffer,
        soft_tasks: &mut TaskBuffer,
        sample: &mut DistanceSample,
        scratch: &mut CollisionEvaluationScratch,
    ) -> Result<(), CollisionError> {
        if !config.validate() {
            return Err(CollisionError::InvalidConfig);
        }
        for pair in &self.self_pairs {
            self.evaluate_pair_into(model, cache, generalized_velocity, *pair, sample, scratch)?;
            if sample.signed_distance >= config.influence_margin {
                continue;
            }
            let lower = (-config.separation_gain * (sample.signed_distance - config.hard_margin))
                .clamp(
                    -config.maximum_separation_speed,
                    config.maximum_separation_speed,
                );
            let row = hard_rows.push().ok_or(CollisionError::ConstraintCapacity)?;
            row.stable_id = config.hard_stable_id_base.saturating_add(pair.stable_id);
            row.coefficients.copy_from(&sample.jacobian_row);
            row.lower = lower;
            row.upper = f64::INFINITY;
            if config.soft_weight > 0.0 {
                let target_velocity = (config.separation_gain
                    * (config.influence_margin - sample.signed_distance))
                    .clamp(0.0, config.maximum_separation_speed);
                let task = soft_tasks.push_one().ok_or(CollisionError::TaskCapacity)?;
                task.stable_id = config.soft_stable_id_base.saturating_add(pair.stable_id);
                task.kind = TaskKind::Repeller;
                task.priority = config.soft_priority;
                task.jacobian.row_mut(0).copy_from(&sample.jacobian_row);
                task.target_velocity[0] = target_velocity;
                task.weight = config.soft_weight;
            }
        }
        Ok(())
    }

    /// Emit avoidance rows from the same tight primitive table and stable pair
    /// IDs used by command admission. The sphere-cover variant remains public
    /// as an explicit conservative fallback for callers that have not yet
    /// admitted analytic primitive Jacobians.
    pub fn emit_avoidance_rows(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        config: CollisionAvoidanceConfig,
        hard_rows: &mut Vec<LinearConstraint>,
        soft_tasks: &mut Vec<Task>,
    ) -> Result<(), CollisionError> {
        if !config.validate() {
            return Err(CollisionError::InvalidConfig);
        }
        for pair in &self.validation_pairs {
            let sample =
                self.evaluate_validation_pair(model, cache, generalized_velocity, *pair)?;
            if sample.signed_distance >= config.influence_margin {
                continue;
            }
            let lower = (-config.separation_gain * (sample.signed_distance - config.hard_margin))
                .clamp(
                    -config.maximum_separation_speed,
                    config.maximum_separation_speed,
                );
            hard_rows.push(LinearConstraint {
                stable_id: config.hard_stable_id_base.saturating_add(pair.stable_id),
                coefficients: sample.jacobian_row.clone(),
                lower,
                upper: f64::INFINITY,
            });
            if config.soft_weight > 0.0 {
                let target_velocity = (config.separation_gain
                    * (config.influence_margin - sample.signed_distance))
                    .clamp(0.0, config.maximum_separation_speed);
                soft_tasks.push(Task {
                    stable_id: config.soft_stable_id_base.saturating_add(pair.stable_id),
                    kind: TaskKind::Repeller,
                    priority: config.soft_priority,
                    jacobian: DMatrix::from_row_slice(1, model.dof, sample.jacobian_row.as_slice()),
                    target_velocity: nalgebra::DVector::from_element(1, target_velocity),
                    weight: config.soft_weight,
                });
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn emit_avoidance_rows_buffered(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
        config: CollisionAvoidanceConfig,
        hard_rows: &mut ConstraintBuffer,
        soft_tasks: &mut TaskBuffer,
        sample: &mut DistanceSample,
        scratch: &mut CollisionEvaluationScratch,
    ) -> Result<(), CollisionError> {
        if !config.validate() {
            return Err(CollisionError::InvalidConfig);
        }
        for pair in &self.validation_pairs {
            self.evaluate_validation_pair_into(
                model,
                cache,
                generalized_velocity,
                *pair,
                sample,
                scratch,
            )?;
            if sample.signed_distance >= config.influence_margin {
                continue;
            }
            let lower = (-config.separation_gain * (sample.signed_distance - config.hard_margin))
                .clamp(
                    -config.maximum_separation_speed,
                    config.maximum_separation_speed,
                );
            let row = hard_rows.push().ok_or(CollisionError::ConstraintCapacity)?;
            row.stable_id = config.hard_stable_id_base.saturating_add(pair.stable_id);
            row.coefficients.copy_from(&sample.jacobian_row);
            row.lower = lower;
            row.upper = f64::INFINITY;
            if config.soft_weight > 0.0 {
                let target_velocity = (config.separation_gain
                    * (config.influence_margin - sample.signed_distance))
                    .clamp(0.0, config.maximum_separation_speed);
                let task = soft_tasks.push_one().ok_or(CollisionError::TaskCapacity)?;
                task.stable_id = config.soft_stable_id_base.saturating_add(pair.stable_id);
                task.kind = TaskKind::Repeller;
                task.priority = config.soft_priority;
                task.jacobian.row_mut(0).copy_from(&sample.jacobian_row);
                task.target_velocity[0] = target_velocity;
                task.weight = config.soft_weight;
            }
        }
        Ok(())
    }

    /// Validate every self-pair on the segment's dense servo grid, including
    /// both endpoints. Callers provide robot/model/acceleration scratch so the
    /// scan itself adds no trajectory or FK buffer allocation.
    #[allow(clippy::too_many_arguments)]
    pub fn validate_segment_on_grid(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        control_world_from_root: Transform3,
        sample_period_ns: i64,
        required_clearance: f64,
        state_scratch: &mut RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
    ) -> Result<CollisionSweepReport, CollisionError> {
        let mut sample = DistanceSample::workspace(model.dof);
        let grid_sample_count = if sample_period_ns > 0 {
            (segment.duration_ns / sample_period_ns) as usize + 1
        } else {
            0
        };
        let mut evaluation = CollisionEvaluationScratch::with_grid_capacity(
            model.dof,
            self.validation_primitives.len(),
            self.validation_pairs.len(),
            grid_sample_count,
        );
        self.validate_segment_on_grid_buffered(
            model,
            segment,
            control_world_from_root,
            sample_period_ns,
            required_clearance,
            state_scratch,
            acceleration_scratch,
            model_scratch,
            &mut sample,
            &mut evaluation,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn validate_segment_on_grid_buffered(
        &self,
        model: &CompiledModel,
        segment: &QuinticSegment,
        control_world_from_root: Transform3,
        sample_period_ns: i64,
        required_clearance: f64,
        state_scratch: &mut RobotState,
        acceleration_scratch: &mut nalgebra::DVector<f64>,
        model_scratch: &mut ModelCache,
        sample: &mut DistanceSample,
        evaluation: &mut CollisionEvaluationScratch,
    ) -> Result<CollisionSweepReport, CollisionError> {
        if !required_clearance.is_finite() {
            return Err(CollisionError::InvalidClearance);
        }
        if sample_period_ns <= 0 || segment.duration_ns % sample_period_ns != 0 {
            return Err(TrajectoryError::SamplePeriod.into());
        }
        if state_scratch.q.len() != model.dof
            || state_scratch.v.len() != model.dof
            || acceleration_scratch.len() != model.dof
            || segment.coefficients.len() != model.dof
        {
            return Err(TrajectoryError::Dimension.into());
        }

        state_scratch.control_world_from_root = control_world_from_root;
        let intervals = (segment.duration_ns / sample_period_ns) as usize;
        let mut report = CollisionSweepReport {
            samples_evaluated: intervals + 1,
            minimum_signed_distance: f64::INFINITY,
            minimum_pair_id: None,
            minimum_time_ns: None,
            first_violation_pair_id: None,
            first_violation_time_ns: None,
        };
        if evaluation.primitive_pair_minimum_distances.len() != self.validation_pairs.len() {
            return Err(TrajectoryError::Dimension.into());
        }
        evaluation
            .primitive_pair_minimum_distances
            .fill(f64::INFINITY);
        let pair_count = self.validation_pairs.len();
        let grid_value_count = (intervals + 1).saturating_mul(pair_count);
        let retain_grid = evaluation.primitive_pair_grid_distances.len() >= grid_value_count;
        for sample_index in 0..=intervals {
            let time_ns = segment.start_time_ns + sample_index as i64 * sample_period_ns;
            segment.evaluate_into(
                time_ns,
                state_scratch.q.as_mut_slice(),
                state_scratch.v.as_mut_slice(),
                acceleration_scratch.as_mut_slice(),
            )?;
            model.forward_kinematics(state_scratch, model_scratch)?;
            if evaluation.primitive_world_from_shape.len() != self.validation_primitives.len() {
                return Err(TrajectoryError::Dimension.into());
            }
            for (primitive, world_from_shape) in self
                .validation_primitives
                .iter()
                .zip(evaluation.primitive_world_from_shape.iter_mut())
            {
                *world_from_shape =
                    model_scratch.world_from_body[primitive.body.0] * primitive.body_from_shape;
            }
            for (pair_index, pair) in self.validation_pairs.iter().enumerate() {
                // Admission needs only represented geometry. Avoid constructing
                // point Jacobians for every pair at every trajectory sample;
                // the controller-side barrier path still evaluates the full
                // signed-distance Jacobian when it emits a row.
                let signed_distance = primitive_signed_distance(
                    &self.validation_primitives[pair.a].geometry,
                    evaluation.primitive_world_from_shape[pair.a],
                    &self.validation_primitives[pair.b].geometry,
                    evaluation.primitive_world_from_shape[pair.b],
                );
                evaluation.primitive_pair_minimum_distances[pair_index] =
                    evaluation.primitive_pair_minimum_distances[pair_index].min(signed_distance);
                if retain_grid {
                    evaluation.primitive_pair_grid_distances
                        [sample_index * pair_count + pair_index] = signed_distance;
                }
                record_sweep_distance(
                    &mut report,
                    pair.stable_id,
                    time_ns,
                    signed_distance,
                    required_clearance,
                );
            }
        }
        let _ = sample;
        Ok(report)
    }

    pub fn minimum_distance(
        &self,
        model: &CompiledModel,
        cache: &ModelCache,
        generalized_velocity: &nalgebra::DVector<f64>,
    ) -> Result<Option<DistanceSample>, ModelError> {
        let mut minimum: Option<DistanceSample> = None;
        for pair in &self.self_pairs {
            let sample = self.evaluate_pair(model, cache, generalized_velocity, *pair)?;
            if minimum
                .as_ref()
                .is_none_or(|current| sample.signed_distance < current.signed_distance)
            {
                minimum = Some(sample);
            }
        }
        Ok(minimum)
    }
}

fn record_sweep_distance(
    report: &mut CollisionSweepReport,
    pair_id: u32,
    time_ns: ControlTime,
    signed_distance: f64,
    required_clearance: f64,
) {
    if signed_distance < report.minimum_signed_distance
        || (signed_distance == report.minimum_signed_distance
            && (time_ns, pair_id)
                < (
                    report.minimum_time_ns.unwrap_or(ControlTime::MAX),
                    report.minimum_pair_id.unwrap_or(u32::MAX),
                ))
    {
        report.minimum_signed_distance = signed_distance;
        report.minimum_pair_id = Some(pair_id);
        report.minimum_time_ns = Some(time_ns);
    }
    if signed_distance < required_clearance
        && report.first_violation_time_ns.is_none_or(|first_time| {
            time_ns < first_time
                || (time_ns == first_time
                    && pair_id < report.first_violation_pair_id.unwrap_or(u32::MAX))
        })
    {
        report.first_violation_pair_id = Some(pair_id);
        report.first_violation_time_ns = Some(time_ns);
    }
}

fn primitive_signed_distance(
    a: &PrimitiveGeometry,
    world_from_a: Transform3,
    b: &PrimitiveGeometry,
    world_from_b: Transform3,
) -> f64 {
    primitive_distance_witness(a, world_from_a, b, world_from_b).signed_distance
}

fn primitive_distance_witness(
    a: &PrimitiveGeometry,
    world_from_a: Transform3,
    b: &PrimitiveGeometry,
    world_from_b: Transform3,
) -> PrimitiveDistanceWitness {
    match (a, b) {
        (PrimitiveGeometry::Sphere { radius: ra }, PrimitiveGeometry::Sphere { radius: rb }) => {
            let center_a = world_from_a.translation.vector;
            let center_b = world_from_b.translation.vector;
            let delta = center_b - center_a;
            let distance = delta.norm();
            let normal = stable_direction(delta);
            PrimitiveDistanceWitness {
                signed_distance: distance - ra - rb,
                normal,
                feature_a: center_a,
                feature_b: center_b,
                surface_a: center_a + *ra * normal,
                surface_b: center_b - *rb * normal,
                quality: DistanceQuality::ExactSphere,
            }
        }
        (
            PrimitiveGeometry::Sphere { radius: sphere },
            PrimitiveGeometry::Capsule {
                radius: capsule,
                half_length,
            },
        ) => {
            let center = world_from_a.translation.vector;
            let (start, end) = capsule_segment(world_from_b, *half_length);
            let closest = closest_point_on_segment(center, start, end);
            let delta = closest - center;
            let distance = delta.norm();
            let normal = stable_direction(delta);
            PrimitiveDistanceWitness {
                signed_distance: distance - sphere - capsule,
                normal,
                feature_a: center,
                feature_b: closest,
                surface_a: center + *sphere * normal,
                surface_b: closest - *capsule * normal,
                quality: DistanceQuality::AnalyticPrimitive,
            }
        }
        (PrimitiveGeometry::Capsule { .. }, PrimitiveGeometry::Sphere { .. }) => {
            primitive_distance_witness(b, world_from_b, a, world_from_a).swapped()
        }
        (
            PrimitiveGeometry::Capsule {
                radius: ra,
                half_length: ha,
            },
            PrimitiveGeometry::Capsule {
                radius: rb,
                half_length: hb,
            },
        ) => {
            let (a0, a1) = capsule_segment(world_from_a, *ha);
            let (b0, b1) = capsule_segment(world_from_b, *hb);
            let (closest_a, closest_b) = closest_points_on_segments(a0, a1, b0, b1);
            let delta = closest_b - closest_a;
            let distance = delta.norm();
            let normal = stable_direction(delta);
            PrimitiveDistanceWitness {
                signed_distance: distance - ra - rb,
                normal,
                feature_a: closest_a,
                feature_b: closest_b,
                surface_a: closest_a + *ra * normal,
                surface_b: closest_b - *rb * normal,
                quality: DistanceQuality::AnalyticPrimitive,
            }
        }
        (PrimitiveGeometry::Sphere { radius }, PrimitiveGeometry::Box { half_extents }) => {
            let box_from_world = world_from_b.inverse();
            let center = world_from_a.translation.vector;
            let point = box_from_world
                .transform_point(&Point3::from(world_from_a.translation.vector))
                .coords;
            let (point_distance, point_on_box, normal_point_to_box) =
                point_box_distance_witness(point, *half_extents);
            let normal = world_from_b.rotation.transform_vector(&normal_point_to_box);
            let surface_a = if point_distance > 0.0 {
                center + *radius * normal
            } else {
                center - *radius * normal
            };
            PrimitiveDistanceWitness {
                signed_distance: point_distance - radius,
                normal,
                feature_a: center,
                feature_b: world_from_b
                    .transform_point(&Point3::from(point_on_box))
                    .coords,
                surface_a,
                surface_b: world_from_b
                    .transform_point(&Point3::from(point_on_box))
                    .coords,
                quality: DistanceQuality::AnalyticPrimitive,
            }
        }
        (PrimitiveGeometry::Box { .. }, PrimitiveGeometry::Sphere { .. }) => {
            primitive_distance_witness(b, world_from_b, a, world_from_a).swapped()
        }
        (
            PrimitiveGeometry::Capsule {
                radius,
                half_length,
            },
            PrimitiveGeometry::Box { half_extents },
        ) => {
            let (start_world, end_world) = capsule_segment(world_from_a, *half_length);
            let box_from_world = world_from_b.inverse();
            let start = box_from_world
                .transform_point(&Point3::from(start_world))
                .coords;
            let end = box_from_world
                .transform_point(&Point3::from(end_world))
                .coords;
            let (centerline_distance, point_on_segment, point_on_box, normal_segment_to_box) =
                segment_box_distance_witness(start, end, *half_extents);
            let normal = world_from_b
                .rotation
                .transform_vector(&normal_segment_to_box);
            let feature_a = world_from_b
                .transform_point(&Point3::from(point_on_segment))
                .coords;
            let feature_b = world_from_b
                .transform_point(&Point3::from(point_on_box))
                .coords;
            PrimitiveDistanceWitness {
                signed_distance: centerline_distance - radius,
                normal,
                feature_a,
                feature_b,
                surface_a: if centerline_distance > 0.0 {
                    feature_a + *radius * normal
                } else {
                    feature_a - *radius * normal
                },
                surface_b: feature_b,
                quality: DistanceQuality::AnalyticPrimitive,
            }
        }
        (PrimitiveGeometry::Box { .. }, PrimitiveGeometry::Capsule { .. }) => {
            primitive_distance_witness(b, world_from_b, a, world_from_a).swapped()
        }
        (
            PrimitiveGeometry::Box { half_extents: ea },
            PrimitiveGeometry::Box { half_extents: eb },
        ) => box_box_sat_witness(world_from_a, *ea, world_from_b, *eb),
    }
}

fn stable_direction(delta: Vec3) -> Vec3 {
    let norm = delta.norm();
    if norm > 1e-12 {
        delta / norm
    } else {
        Vec3::x()
    }
}

fn capsule_segment(world_from_shape: Transform3, half_length: f64) -> (Vec3, Vec3) {
    (
        world_from_shape
            .transform_point(&Point3::new(0.0, 0.0, -half_length))
            .coords,
        world_from_shape
            .transform_point(&Point3::new(0.0, 0.0, half_length))
            .coords,
    )
}

fn closest_point_on_segment(point: Vec3, start: Vec3, end: Vec3) -> Vec3 {
    let delta = end - start;
    let denominator = delta.norm_squared();
    if denominator <= 1e-24 {
        return start;
    }
    let t = ((point - start).dot(&delta) / denominator).clamp(0.0, 1.0);
    start + t * delta
}

fn closest_points_on_segments(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3) -> (Vec3, Vec3) {
    let u = a1 - a0;
    let v = b1 - b0;
    let w = a0 - b0;
    let aa = u.dot(&u);
    let bb = u.dot(&v);
    let cc = v.dot(&v);
    let dd = u.dot(&w);
    let ee = v.dot(&w);
    let denominator = aa * cc - bb * bb;
    let mut s_numerator;
    let mut s_denominator = denominator;
    let mut t_numerator;
    let mut t_denominator = denominator;
    if denominator < 1e-24 {
        s_numerator = 0.0;
        s_denominator = 1.0;
        t_numerator = ee;
        t_denominator = cc;
    } else {
        s_numerator = bb * ee - cc * dd;
        t_numerator = aa * ee - bb * dd;
        if s_numerator < 0.0 {
            s_numerator = 0.0;
            t_numerator = ee;
            t_denominator = cc;
        } else if s_numerator > s_denominator {
            s_numerator = s_denominator;
            t_numerator = ee + bb;
            t_denominator = cc;
        }
    }
    if t_numerator < 0.0 {
        t_numerator = 0.0;
        if -dd < 0.0 {
            s_numerator = 0.0;
        } else if -dd > aa {
            s_numerator = s_denominator;
        } else {
            s_numerator = -dd;
            s_denominator = aa;
        }
    } else if t_numerator > t_denominator {
        t_numerator = t_denominator;
        if -dd + bb < 0.0 {
            s_numerator = 0.0;
        } else if -dd + bb > aa {
            s_numerator = s_denominator;
        } else {
            s_numerator = -dd + bb;
            s_denominator = aa;
        }
    }
    let s = if s_numerator.abs() < 1e-24 {
        0.0
    } else {
        s_numerator / s_denominator
    };
    let t = if t_numerator.abs() < 1e-24 {
        0.0
    } else {
        t_numerator / t_denominator
    };
    (a0 + s * u, b0 + t * v)
}

fn point_box_distance_witness(point: Vec3, half_extents: Vec3) -> (f64, Vec3, Vec3) {
    let q = point.map(f64::abs) - half_extents;
    let outside = q.map(|component| component.max(0.0));
    let outside_distance = outside.norm();
    if outside_distance > 1e-12 {
        let point_on_box =
            point.zip_map(&half_extents, |value, extent| value.clamp(-extent, extent));
        let normal = (point_on_box - point) / outside_distance;
        return (outside_distance, point_on_box, normal);
    }

    let mut axis = 0;
    let mut margin = half_extents[0] - point[0].abs();
    for candidate in 1..3 {
        let candidate_margin = half_extents[candidate] - point[candidate].abs();
        if candidate_margin < margin {
            axis = candidate;
            margin = candidate_margin;
        }
    }
    let sign = if point[axis] < 0.0 { -1.0 } else { 1.0 };
    let mut point_on_box = point;
    point_on_box[axis] = sign * half_extents[axis];
    let mut outward = Vec3::zeros();
    outward[axis] = sign;
    (-margin.max(0.0), point_on_box, -outward)
}

fn segment_box_distance_witness(
    start: Vec3,
    end: Vec3,
    half_extents: Vec3,
) -> (f64, Vec3, Vec3, Vec3) {
    let delta = end - start;
    let mut breaks = [0.0_f64; 8];
    let mut count = 2;
    breaks[0] = 0.0;
    breaks[1] = 1.0;
    for axis in 0..3 {
        if delta[axis].abs() <= 1e-15 {
            continue;
        }
        for boundary in [-half_extents[axis], half_extents[axis]] {
            let t = (boundary - start[axis]) / delta[axis];
            if t > 0.0 && t < 1.0 {
                breaks[count] = t;
                count += 1;
            }
        }
    }
    breaks[..count].sort_by(f64::total_cmp);
    let squared_at = |t: f64| {
        let point = start + t * delta;
        let q = point.map(f64::abs) - half_extents;
        q.map(|component| component.max(0.0).powi(2)).sum()
    };
    let mut minimum_squared = f64::INFINITY;
    let mut minimum_t = 0.0;
    let mut consider = |t: f64| {
        let squared = squared_at(t);
        if squared < minimum_squared || (squared == minimum_squared && t < minimum_t) {
            minimum_squared = squared;
            minimum_t = t;
        }
    };
    for index in 0..count - 1 {
        let lower = breaks[index];
        let upper = breaks[index + 1];
        consider(lower);
        consider(upper);
        if upper - lower <= 1e-15 {
            continue;
        }
        let middle = 0.5 * (lower + upper);
        let point = start + middle * delta;
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for axis in 0..3 {
            let boundary = if point[axis] < -half_extents[axis] {
                Some(-half_extents[axis])
            } else if point[axis] > half_extents[axis] {
                Some(half_extents[axis])
            } else {
                None
            };
            if let Some(boundary) = boundary {
                numerator += delta[axis] * (start[axis] - boundary);
                denominator += delta[axis] * delta[axis];
            }
        }
        if denominator > 1e-24 {
            let stationary = (-numerator / denominator).clamp(lower, upper);
            consider(stationary);
        }
    }
    if minimum_squared > 1e-20 {
        let point_on_segment = start + minimum_t * delta;
        let (_, point_on_box, normal) = point_box_distance_witness(point_on_segment, half_extents);
        return (
            minimum_squared.sqrt(),
            point_on_segment,
            point_on_box,
            normal,
        );
    }

    // A centerline that intersects or lies inside the box has zero unsigned
    // distance, matching command admission's conservative capsule-box proxy.
    // Select the deterministic in-box point nearest a face to obtain a useful
    // local separating direction without changing that scalar distance.
    let mut inside_t = minimum_t;
    let mut inside_signed = f64::NEG_INFINITY;
    let mut consider_inside = |t: f64| {
        let point = start + t * delta;
        let (signed, _, _) = point_box_distance_witness(point, half_extents);
        if signed <= 1e-10 && (signed > inside_signed || (signed == inside_signed && t < inside_t))
        {
            inside_signed = signed;
            inside_t = t;
        }
    };
    for index in 0..count {
        consider_inside(breaks[index]);
    }
    for index in 0..count - 1 {
        consider_inside(0.5 * (breaks[index] + breaks[index + 1]));
    }
    let point_on_segment = start + inside_t * delta;
    let (_, point_on_box, normal) = point_box_distance_witness(point_on_segment, half_extents);
    (0.0, point_on_segment, point_on_box, normal)
}

fn box_box_sat_witness(
    world_from_a: Transform3,
    half_a: Vec3,
    world_from_b: Transform3,
    half_b: Vec3,
) -> PrimitiveDistanceWitness {
    let rotation_a = world_from_a.rotation.to_rotation_matrix();
    let rotation_b = world_from_b.rotation.to_rotation_matrix();
    let axes_a = [
        rotation_a.matrix().column(0).into_owned(),
        rotation_a.matrix().column(1).into_owned(),
        rotation_a.matrix().column(2).into_owned(),
    ];
    let axes_b = [
        rotation_b.matrix().column(0).into_owned(),
        rotation_b.matrix().column(1).into_owned(),
        rotation_b.matrix().column(2).into_owned(),
    ];
    let delta = world_from_b.translation.vector - world_from_a.translation.vector;
    let projected_radius = |axes: &[Vec3; 3], extents: Vec3, axis: Vec3| {
        extents.x * axes[0].dot(&axis).abs()
            + extents.y * axes[1].dot(&axis).abs()
            + extents.z * axes[2].dot(&axis).abs()
    };
    let mut maximum_gap = f64::NEG_INFINITY;
    let mut limiting_axis = Vec3::x();
    let mut test_axis = |axis: Vec3| {
        let norm = axis.norm();
        if norm <= 1e-12 {
            return;
        }
        let mut axis = axis / norm;
        if delta.dot(&axis) < 0.0 {
            axis = -axis;
        }
        let gap = delta.dot(&axis)
            - projected_radius(&axes_a, half_a, axis)
            - projected_radius(&axes_b, half_b, axis);
        if gap > maximum_gap {
            maximum_gap = gap;
            limiting_axis = axis;
        }
    };
    for axis in axes_a.iter().chain(axes_b.iter()) {
        test_axis(*axis);
    }
    for axis_a in &axes_a {
        for axis_b in &axes_b {
            test_axis(axis_a.cross(axis_b));
        }
    }
    let support = |center: Vec3, axes: &[Vec3; 3], extents: Vec3, direction: Vec3| {
        let mut point = center;
        for axis in 0..3 {
            point += axes[axis]
                * if axes[axis].dot(&direction) < 0.0 {
                    -extents[axis]
                } else {
                    extents[axis]
                };
        }
        point
    };
    let feature_a = support(
        world_from_a.translation.vector,
        &axes_a,
        half_a,
        limiting_axis,
    );
    let feature_b = support(
        world_from_b.translation.vector,
        &axes_b,
        half_b,
        -limiting_axis,
    );
    PrimitiveDistanceWitness {
        signed_distance: maximum_gap,
        normal: limiting_axis,
        feature_a,
        feature_b,
        surface_a: feature_a,
        surface_b: feature_b,
        quality: DistanceQuality::ConservativePrimitive,
    }
}

fn append_cylinder_sphere_cover(
    push: &mut impl FnMut(Transform3, f64, DistanceQuality),
    body_from_shape: Transform3,
    radius: f64,
    half_length: f64,
) {
    if !radius.is_finite() || radius <= 0.0 || !half_length.is_finite() || half_length <= 0.0 {
        return;
    }
    // At most five axial cells keeps the compiled pair table bounded while a
    // 45 mm target half-cell sharply reduces overreach on humanoid limbs.
    let cells = ((half_length / 0.045).ceil() as usize).clamp(1, 5);
    let cell_half = half_length / cells as f64;
    let cover_radius = radius.hypot(cell_half);
    for cell in 0..cells {
        let z = -half_length + cell_half * (2 * cell + 1) as f64;
        push(
            body_from_shape * Transform3::translation(0.0, 0.0, z),
            cover_radius,
            DistanceQuality::ConservativeSphereCover,
        );
    }
}

fn append_box_sphere_cover(
    push: &mut impl FnMut(Transform3, f64, DistanceQuality),
    body_from_shape: Transform3,
    half_extents: Vec3,
) {
    if !half_extents
        .iter()
        .all(|extent| extent.is_finite() && *extent > 0.0)
    {
        return;
    }
    let cells = |extent: f64| ((extent / 0.06).ceil() as usize).clamp(1, 3);
    let nx = cells(half_extents.x);
    let ny = cells(half_extents.y);
    let nz = cells(half_extents.z);
    let cell_half = Vec3::new(
        half_extents.x / nx as f64,
        half_extents.y / ny as f64,
        half_extents.z / nz as f64,
    );
    let cover_radius = cell_half.norm();
    for x in 0..nx {
        for y in 0..ny {
            for z in 0..nz {
                let center = Vec3::new(
                    -half_extents.x + cell_half.x * (2 * x + 1) as f64,
                    -half_extents.y + cell_half.y * (2 * y + 1) as f64,
                    -half_extents.z + cell_half.z * (2 * z + 1) as f64,
                );
                push(
                    body_from_shape * Transform3::translation(center.x, center.y, center.z),
                    cover_radius,
                    DistanceQuality::ConservativeSphereCover,
                );
            }
        }
    }
}

fn sphere_speed_coefficients(model: &CompiledModel, sphere: &SphereProxy) -> Vec<f64> {
    body_point_speed_coefficients(
        model,
        sphere.body,
        sphere.body_from_sphere.translation.vector.norm(),
    )
}

fn primitive_speed_coefficients(model: &CompiledModel, primitive: &PrimitiveProxy) -> Vec<f64> {
    body_point_speed_coefficients(
        model,
        primitive.body,
        primitive.body_from_shape.translation.vector.norm() + primitive.geometry.bounding_radius(),
    )
}

fn body_point_speed_coefficients(
    model: &CompiledModel,
    mut body: BodyId,
    mut reach: f64,
) -> Vec<f64> {
    let mut coefficients = vec![0.0; model.dof];
    while let Some(joint_id) = model.bodies[body.0].parent_joint {
        let joint = &model.joints[joint_id.0];
        if let Some(coordinate) = joint.coordinate {
            match joint.kind {
                crate::model::JointKind::Revolute | crate::model::JointKind::Continuous => {
                    coefficients[coordinate] = reach;
                }
                crate::model::JointKind::Prismatic => {
                    coefficients[coordinate] = 1.0;
                }
                crate::model::JointKind::Fixed => {}
            }
        }
        reach += joint.parent_from_joint.translation.vector.norm();
        if joint.kind == crate::model::JointKind::Prismatic {
            let travel = joint.limit.lower.abs().max(joint.limit.upper.abs());
            reach += travel;
        }
        body = joint.parent;
    }
    coefficients
}

fn adjacent(model: &CompiledModel, a: BodyId, b: BodyId) -> bool {
    model.bodies[a.0]
        .parent_joint
        .is_some_and(|joint| model.joints[joint.0].parent == b)
        || model.bodies[b.0]
            .parent_joint
            .is_some_and(|joint| model.joints[joint.0].parent == a)
}

/// Whether two shape-bearing bodies are physical neighbors once purely
/// kinematic carrier links are collapsed. Traversal may pass through any
/// number of bodies without collision geometry, but never through another
/// shape-bearing body. This is a topology-derived exclusion rather than a
/// pose-dependent heuristic, so pair ordering and replay remain stable.
fn collision_adjacent(model: &CompiledModel, a: BodyId, b: BodyId) -> bool {
    if adjacent(model, a, b) {
        return true;
    }

    let mut visited = vec![false; model.bodies.len()];
    let mut stack = vec![a];
    visited[a.0] = true;
    while let Some(body) = stack.pop() {
        for joint in &model.joints {
            let neighbor = if joint.parent == body {
                Some(joint.child)
            } else if joint.child == body {
                Some(joint.parent)
            } else {
                None
            };
            let Some(neighbor) = neighbor else {
                continue;
            };
            // A prismatic carrier creates real pose-dependent separation and
            // must remain collision-tested. Revolute/continuous/fixed carrier
            // chains model coincident stacked axes or rigid shims.
            if joint.kind == crate::model::JointKind::Prismatic {
                continue;
            }
            if neighbor == b {
                return true;
            }
            if !visited[neighbor.0] && model.bodies[neighbor.0].collisions.is_empty() {
                visited[neighbor.0] = true;
                stack.push(neighbor);
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use nalgebra::DVector;

    use crate::{model::ModelCache, urdf::load_urdf};

    use super::*;

    #[test]
    fn sphere_distance_and_gradient_have_consistent_sign() {
        let source = r#"
        <robot name="distance">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".2"/></geometry></collision></link>
          <link name="slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".3"/></geometry></collision></link>
          <link name="second_slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/><limit lower="-.4" upper=".4" velocity="2" effort="5"/></joint>
          <joint name="second_slide" type="prismatic"><parent link="slider"/><child link="second_slider"/><origin xyz="0 1 0"/><axis xyz="0 1 0"/><limit lower="-.4" upper=".4" velocity="2" effort="5"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let state = crate::model::RobotState::zeros(&model);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let collision = CompiledCollisionModel::compile(&model);
        assert_eq!(collision.self_pairs.len(), 0, "adjacent links are filtered");

        // Re-enable the single analytic pair for this focused primitive test.
        let pair = CollisionPair {
            stable_id: 0,
            a: 0,
            b: 1,
        };
        let sample = collision
            .evaluate_pair(&model, &cache, &DVector::from_vec(vec![1.0, 0.0]), pair)
            .unwrap();
        assert!((sample.signed_distance - 0.5).abs() < 1e-12);
        assert!((sample.relative_normal_velocity - 1.0).abs() < 1e-12);
        assert!((sample.jacobian_row[0] - 1.0).abs() < 1e-12);
        assert_eq!(sample.quality, DistanceQuality::ExactSphere);
    }

    #[test]
    fn collisionless_joint_carriers_collapse_into_physical_adjacency() {
        let source = r#"
        <robot name="carrier-adjacency">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".2"/></geometry></collision></link>
          <link name="axis_carrier"><inertial><mass value=".01"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <link name="arm"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".1"/></geometry></collision></link>
          <link name="wrist_carrier"><inertial><mass value=".01"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link>
          <link name="hand"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".08"/></geometry></collision></link>
          <joint name="base_axis" type="revolute"><parent link="base"/><child link="axis_carrier"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
          <joint name="axis_arm" type="revolute"><parent link="axis_carrier"/><child link="arm"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
          <joint name="arm_wrist" type="fixed"><parent link="arm"/><child link="wrist_carrier"/></joint>
          <joint name="wrist_hand" type="fixed"><parent link="wrist_carrier"/><child link="hand"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let collision = CompiledCollisionModel::compile(&model);

        assert_eq!(collision.spheres.len(), 3);
        assert_eq!(collision.virtual_bridge_exclusion_count, 2);
        assert_eq!(collision.self_pairs.len(), 1);
        assert_eq!(collision.self_pairs[0].a, 0);
        assert_eq!(collision.self_pairs[0].b, 2);
    }

    #[test]
    fn primitive_admission_distances_cover_sphere_capsule_and_box_pairs() {
        let sphere = PrimitiveGeometry::Sphere { radius: 0.05 };
        let capsule = PrimitiveGeometry::Capsule {
            radius: 0.05,
            half_length: 0.2,
        };
        let box_geometry = PrimitiveGeometry::Box {
            half_extents: Vec3::new(0.1, 0.1, 0.1),
        };
        let identity = Transform3::identity();
        let offset_x = Transform3::translation(0.3, 0.0, 0.0);
        let offset_y = Transform3::translation(0.0, 0.3, 0.0);

        assert!(
            (primitive_signed_distance(&sphere, offset_x, &box_geometry, identity) - 0.15).abs()
                < 1e-12
        );
        assert!(
            (primitive_signed_distance(&capsule, offset_x, &box_geometry, identity) - 0.15).abs()
                < 1e-12
        );
        assert!(
            (primitive_signed_distance(&capsule, identity, &capsule, offset_y) - 0.2).abs() < 1e-12
        );
        assert!(
            (primitive_signed_distance(&box_geometry, identity, &box_geometry, offset_x) - 0.1)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn tight_primitive_witnesses_are_symmetric_and_match_translation_derivatives() {
        let sphere = PrimitiveGeometry::Sphere { radius: 0.1 };
        let capsule = PrimitiveGeometry::Capsule {
            radius: 0.08,
            half_length: 0.25,
        };
        let box_geometry = PrimitiveGeometry::Box {
            half_extents: Vec3::new(0.2, 0.1, 0.15),
        };
        let identity = Transform3::identity();
        let rotated = Transform3::from_parts(
            nalgebra::Translation3::new(0.45, 0.34, 0.21),
            nalgebra::UnitQuaternion::from_euler_angles(0.18, -0.27, 0.31),
        );
        let cases = [
            (
                &sphere,
                identity,
                &sphere,
                Transform3::translation(0.6, 0.2, 0.1),
            ),
            (
                &sphere,
                identity,
                &capsule,
                Transform3::translation(0.5, 0.3, 0.2),
            ),
            (
                &capsule,
                identity,
                &capsule,
                Transform3::translation(0.5, 0.3, 0.1),
            ),
            (
                &sphere,
                identity,
                &box_geometry,
                Transform3::translation(0.5, 0.3, 0.2),
            ),
            (
                &capsule,
                identity,
                &box_geometry,
                Transform3::translation(0.5, 0.3, 0.2),
            ),
            (&box_geometry, identity, &box_geometry, rotated),
        ];
        let epsilon = 1e-7;
        for (a, world_from_a, b, world_from_b) in cases {
            let witness = primitive_distance_witness(a, world_from_a, b, world_from_b);
            let swapped = primitive_distance_witness(b, world_from_b, a, world_from_a);
            assert!(witness.signed_distance.is_finite());
            assert!((witness.normal.norm() - 1.0).abs() < 1e-12);
            assert!((witness.signed_distance - swapped.signed_distance).abs() < 1e-12);
            assert!((witness.normal + swapped.normal).norm() < 1e-12);
            assert!((witness.feature_a - swapped.feature_b).norm() < 1e-12);
            assert!((witness.feature_b - swapped.feature_a).norm() < 1e-12);
            assert!(witness.surface_a.iter().all(|value| value.is_finite()));
            assert!(witness.surface_b.iter().all(|value| value.is_finite()));

            let mut shifted_b = world_from_b;
            shifted_b.translation.vector += epsilon * witness.normal;
            let shifted = primitive_signed_distance(a, world_from_a, b, shifted_b);
            let derivative = (shifted - witness.signed_distance) / epsilon;
            assert!(
                (derivative - 1.0).abs() < 2e-6,
                "translation derivative {derivative} for {a:?} / {b:?}"
            );
        }
    }

    #[test]
    fn continuous_clearance_bound_keeps_distance_and_speed_on_the_same_pair() {
        let primitive = |stable_id, body| PrimitiveProxy {
            stable_id,
            body: BodyId(body),
            body_from_shape: Transform3::identity(),
            geometry: PrimitiveGeometry::Sphere { radius: 0.01 },
        };
        let collision = CompiledCollisionModel {
            validation_primitives: vec![
                primitive(0, 0),
                primitive(1, 1),
                primitive(2, 2),
                primitive(3, 3),
            ],
            validation_pairs: vec![
                CollisionPair {
                    stable_id: 10,
                    a: 0,
                    b: 1,
                },
                CollisionPair {
                    stable_id: 11,
                    a: 2,
                    b: 3,
                },
            ],
            // One coordinate: pair 10 is slow (0.1 m/s), pair 11 is fast
            // (4 m/s). Their sampled minima deliberately have the opposite
            // ordering.
            primitive_speed_coefficients: vec![0.05, 0.05, 2.0, 2.0],
            speed_coordinate_count: 1,
            ..CompiledCollisionModel::default()
        };
        let mut scratch = CollisionEvaluationScratch::new(1, 4, 2);
        scratch.primitive_pair_minimum_distances[0] = 0.021;
        scratch.primitive_pair_minimum_distances[1] = 0.030;

        let report = collision
            .pairwise_continuous_clearance_bound(&scratch, &[1.0], 0.005)
            .unwrap();
        assert_eq!(report.limiting_pair_id, Some(11));
        assert!((report.limiting_relative_speed_bound - 4.0).abs() < 1e-12);
        assert!((report.minimum_clearance_lower_bound - 0.010).abs() < 1e-12);

        // The old cross-pair construction would have mixed pair 10's 21 mm
        // sample with pair 11's 4 m/s speed and reported only 1 mm.
        let cross_pair_bound = 0.021 - 4.0 * 0.005;
        assert!(report.minimum_clearance_lower_bound > cross_pair_bound + 0.008);
    }

    #[test]
    fn adaptive_pair_interval_refinement_turns_midpoint_penetration_into_sampled_evidence() {
        let source = r#"
        <robot name="adaptive-clearance">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".2"/></geometry></collision></link>
          <link name="slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".3"/></geometry></collision></link>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz=".53 0 0"/><axis xyz="1 0 0"/><limit lower="-.1" upper=".1" velocity="100" effort="5"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let mut collision = CompiledCollisionModel::compile(&model);
        collision.validation_pairs.push(CollisionPair {
            stable_id: 7,
            a: 0,
            b: 1,
        });
        let q = DVector::zeros(1);
        let v0 = DVector::from_element(1, -40.0);
        let v1 = DVector::from_element(1, 40.0);
        let acceleration = DVector::zeros(1);
        let segment =
            QuinticSegment::new(0, 1_000_000, &q, &v0, &acceleration, &q, &v1, &acceleration)
                .unwrap();
        let mut state = RobotState::zeros(&model);
        let mut acceleration_scratch = DVector::zeros(model.dof);
        let mut cache = ModelCache::new(&model);
        let mut sample = DistanceSample::workspace(model.dof);
        let mut evaluation = CollisionEvaluationScratch::with_grid_capacity(1, 2, 1, 2);
        let mut sweep = collision
            .validate_segment_on_grid_buffered(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.02,
                &mut state,
                &mut acceleration_scratch,
                &mut cache,
                &mut sample,
                &mut evaluation,
            )
            .unwrap();
        assert!(sweep.is_clear());
        assert!((sweep.minimum_signed_distance - 0.03).abs() < 1e-12);

        let continuity = collision
            .adaptive_pairwise_continuous_clearance_bound(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.02,
                1,
                &[40.0],
                &mut state,
                &mut acceleration_scratch,
                &mut cache,
                &mut evaluation,
                &mut sweep,
            )
            .unwrap();
        assert_eq!(continuity.refinement_pair_samples_evaluated, 1);
        assert_eq!(continuity.maximum_subdivision_depth_reached, 1);
        assert!(!sweep.is_clear());
        assert_eq!(sweep.first_violation_pair_id, Some(7));
        assert_eq!(sweep.first_violation_time_ns, Some(500_000));
        assert!(sweep.minimum_signed_distance < 0.02);
    }

    #[test]
    fn tight_primitive_avoidance_matches_admission_and_removes_cover_pressure() {
        let source = r#"
        <robot name="tight-avoidance">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><box size="1 .1 .1"/></geometry></collision></link>
          <link name="slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".1"/></geometry></collision></link>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz="0 .3 0"/><axis xyz="0 1 0"/><limit lower="-.2" upper=".2" velocity="2" effort="5"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let mut collision = CompiledCollisionModel::compile(&model);
        assert_eq!(collision.spheres.len(), 4);
        assert_eq!(collision.validation_primitives.len(), 2);

        // Adjacent shapes are excluded in production; re-enable the focused
        // pair in both representations to compare their local authority rows.
        let tight_pair = CollisionPair {
            stable_id: 7,
            a: 0,
            b: 1,
        };
        collision.validation_pairs.push(tight_pair);
        collision.self_pairs.push(CollisionPair {
            stable_id: 7,
            a: 1,
            b: 3,
        });
        let velocity = DVector::from_element(model.dof, 0.4);
        let tight = collision
            .evaluate_validation_pair(&model, &cache, &velocity, tight_pair)
            .unwrap();
        assert!((tight.signed_distance - 0.15).abs() < 1e-12);
        assert!((tight.jacobian_row[0] - 1.0).abs() < 1e-12);
        assert!((tight.relative_normal_velocity - 0.4).abs() < 1e-12);
        assert_eq!(tight.quality, DistanceQuality::AnalyticPrimitive);
        assert!((tight.normal_in_control_world - Vec3::y()).norm() < 1e-12);
        assert!((tight.point_a_in_control_world.y - 0.05).abs() < 1e-12);
        assert!((tight.point_b_in_control_world.y - 0.20).abs() < 1e-12);

        let world_from_a =
            cache.world_from_body[0] * collision.validation_primitives[0].body_from_shape;
        let world_from_b =
            cache.world_from_body[1] * collision.validation_primitives[1].body_from_shape;
        assert!(
            (primitive_signed_distance(
                &collision.validation_primitives[0].geometry,
                world_from_a,
                &collision.validation_primitives[1].geometry,
                world_from_b,
            ) - tight.signed_distance)
                .abs()
                < 1e-12
        );

        let epsilon = 1e-7;
        let mut shifted_state = state.clone();
        shifted_state.q[0] = epsilon;
        let mut shifted_cache = ModelCache::new(&model);
        model
            .forward_kinematics(&shifted_state, &mut shifted_cache)
            .unwrap();
        let shifted = collision
            .evaluate_validation_pair(
                &model,
                &shifted_cache,
                &DVector::zeros(model.dof),
                tight_pair,
            )
            .unwrap();
        let finite_difference = (shifted.signed_distance - tight.signed_distance) / epsilon;
        assert!((finite_difference - tight.jacobian_row[0]).abs() < 1e-8);

        let config = CollisionAvoidanceConfig {
            hard_margin: 0.02,
            influence_margin: 0.10,
            separation_gain: 2.0,
            maximum_separation_speed: 1.0,
            soft_weight: 0.0,
            ..CollisionAvoidanceConfig::default()
        };
        let mut tight_rows = Vec::new();
        let mut tight_tasks = Vec::new();
        collision
            .emit_avoidance_rows(
                &model,
                &cache,
                &DVector::zeros(model.dof),
                config,
                &mut tight_rows,
                &mut tight_tasks,
            )
            .unwrap();
        assert!(tight_rows.is_empty());

        let mut cover_rows = Vec::new();
        let mut cover_tasks = Vec::new();
        collision
            .emit_sphere_cover_avoidance_rows(
                &model,
                &cache,
                &DVector::zeros(model.dof),
                config,
                &mut cover_rows,
                &mut cover_tasks,
            )
            .unwrap();
        assert_eq!(cover_rows.len(), 1);
        let cover = collision
            .evaluate_pair(
                &model,
                &cache,
                &DVector::zeros(model.dof),
                collision.self_pairs[0],
            )
            .unwrap();
        assert!(cover.signed_distance < config.influence_margin);
        assert!(cover.signed_distance < tight.signed_distance - 0.1);
    }

    #[test]
    fn avoidance_rows_form_a_stable_control_barrier_and_repeller() {
        let source = r#"
        <robot name="avoidance">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".2"/></geometry></collision></link>
          <link name="slider"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><sphere radius=".3"/></geometry></collision></link>
          <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/><limit lower="-.4" upper=".4" velocity="2" effort="5"/></joint>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let state = crate::model::RobotState::zeros(&model);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let mut collision = CompiledCollisionModel::compile(&model);
        collision.self_pairs.push(CollisionPair {
            stable_id: 7,
            a: 0,
            b: 1,
        });
        collision.validation_pairs.push(CollisionPair {
            stable_id: 7,
            a: 0,
            b: 1,
        });
        let config = CollisionAvoidanceConfig {
            hard_margin: 0.4,
            influence_margin: 0.6,
            separation_gain: 2.0,
            maximum_separation_speed: 10.0,
            soft_weight: 0.5,
            ..CollisionAvoidanceConfig::default()
        };
        let mut hard = Vec::new();
        let mut soft = Vec::new();
        collision
            .emit_avoidance_rows(
                &model,
                &cache,
                &DVector::zeros(model.dof),
                config,
                &mut hard,
                &mut soft,
            )
            .unwrap();

        assert_eq!(hard.len(), 1);
        assert_eq!(hard[0].stable_id, config.hard_stable_id_base + 7);
        assert!((hard[0].coefficients[0] - 1.0).abs() < 1e-12);
        assert!((hard[0].lower + 0.2).abs() < 1e-12);
        assert!(hard[0].upper.is_infinite());
        assert_eq!(soft.len(), 1);
        assert_eq!(soft[0].kind, TaskKind::Repeller);
        assert_eq!(soft[0].stable_id, config.soft_stable_id_base + 7);
        assert!((soft[0].target_velocity[0] - 0.2).abs() < 1e-12);
        assert!((soft[0].weight - 0.5).abs() < 1e-12);

        let zero = DVector::zeros(model.dof);
        let end = DVector::from_element(model.dof, -0.4);
        let segment =
            QuinticSegment::new(10, 20_000_000, &zero, &zero, &zero, &end, &zero, &zero).unwrap();
        let mut state_scratch = RobotState::zeros(&model);
        let mut acceleration_scratch = DVector::zeros(model.dof);
        let report = collision
            .validate_segment_on_grid(
                &model,
                &segment,
                Transform3::identity(),
                1_000_000,
                0.2,
                &mut state_scratch,
                &mut acceleration_scratch,
                &mut cache,
            )
            .unwrap();
        assert_eq!(report.samples_evaluated, 21);
        assert_eq!(report.minimum_pair_id, Some(7));
        assert_eq!(report.minimum_time_ns, Some(20_000_010));
        assert!((report.minimum_signed_distance - 0.1).abs() < 1e-12);
        assert_eq!(report.first_violation_pair_id, Some(7));
        assert!(!report.is_clear());
    }
}
