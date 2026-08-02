use nalgebra::{DMatrix, DVector, Point3, UnitQuaternion};
use thiserror::Error;

use crate::{
    math::{Motion6, Vec3},
    model::{CompiledModel, DynamicsCache, FrameId, ModelCache, ModelError, RobotState},
    solver::{
        HierarchicalSolver, Priority, SolveResult, SolveStatus, SolverWorkspace, TaskBuffer,
        TaskKind, VelocityBounds,
    },
};

/// A two-coordinate point target solved in a selected world plane.
///
/// Targets are intentionally explicit and allocation-free. Multiple targets
/// may be supplied when their coordinate pairs are disjoint, as with the two
/// legs of a planar wheeled biped.
#[derive(Clone, Copy, Debug)]
pub struct PlanarPointIkTarget {
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub target_world: Vec3,
    pub coordinates: [usize; 2],
    /// World-vector component indices, for example `[0, 2]` for X/Z.
    pub axes: [usize; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct PlanarIkOptions {
    pub maximum_iterations: usize,
    pub damping: f64,
    pub maximum_step_rad: f64,
    pub tolerance_m: f64,
}

impl Default for PlanarIkOptions {
    fn default() -> Self {
        Self {
            maximum_iterations: 12,
            damping: 1e-8,
            maximum_step_rad: 0.2,
            tolerance_m: 1e-8,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlanarIkReport {
    pub converged: bool,
    pub iterations: usize,
    pub maximum_planar_error_m: f64,
    pub singular_updates: usize,
}

#[derive(Clone, Debug)]
pub struct PlanarIkScratch {
    pub model: ModelCache,
    point_jacobian: DMatrix<f64>,
    lower: DVector<f64>,
    upper: DVector<f64>,
}

/// One Cartesian frame-origin target in the offline whole-body witness solve.
///
/// The solve is deliberately kinematic and stateless with respect to the WBC:
/// it certifies that a morphology can realize authored root, CoM, and effector
/// geometry before any policy, integration, or physics rollout is introduced.
#[derive(Clone, Copy, Debug)]
pub struct WholeBodyPointIkTarget {
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub target_world: Vec3,
    pub weight: f64,
    pub target_orientation_world: UnitQuaternion<f64>,
    pub orientation_weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct WholeBodyPointJetTarget {
    pub frame: FrameId,
    pub point_in_frame: Vec3,
    pub target_velocity_world: Vec3,
    pub target_acceleration_world: Vec3,
    pub weight: f64,
    pub target_angular_velocity_world: Vec3,
    pub target_angular_acceleration_world: Vec3,
    pub angular_weight: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct WholeBodyIkOptions {
    pub maximum_iterations: usize,
    /// Minimum Gauss-Newton updates before an already acceptable witness may
    /// stop. Sequential offline traces use this to avoid tolerance-sized
    /// staircase motion while retaining a wider physical acceptance envelope.
    pub minimum_iterations: usize,
    pub damping: f64,
    pub posture_weight: f64,
    pub center_of_mass_weight: f64,
    pub orientation_weight: f64,
    pub maximum_step_rad: f64,
    pub point_tolerance_m: f64,
    pub center_of_mass_tolerance_m: f64,
    pub orientation_tolerance_rad: f64,
}

impl Default for WholeBodyIkOptions {
    fn default() -> Self {
        Self {
            maximum_iterations: 32,
            minimum_iterations: 0,
            damping: 1e-6,
            posture_weight: 1e-4,
            center_of_mass_weight: 0.25,
            orientation_weight: 1.0,
            maximum_step_rad: 0.12,
            point_tolerance_m: 1e-4,
            center_of_mass_tolerance_m: 5e-3,
            orientation_tolerance_rad: 1e-3,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WholeBodyIkReport {
    pub converged: bool,
    pub iterations: usize,
    pub maximum_point_error_m: f64,
    pub center_of_mass_error_m: f64,
    pub maximum_orientation_error_rad: f64,
    pub limit_clamped_coordinates: usize,
    pub factorization_failures: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct WholeBodyJetOptions {
    pub velocity_damping: f64,
    pub acceleration_damping: f64,
    pub center_of_mass_weight: f64,
    /// Preserve point and angular effector jets lexicographically before CoM
    /// and coordinate regularization. This is the contact-consistent offline
    /// witness mode; the weighted normal-equation path remains available for
    /// deliberately approximate morphology projections.
    pub strict_effector_targets: bool,
}

impl Default for WholeBodyJetOptions {
    fn default() -> Self {
        Self {
            velocity_damping: 1e-6,
            acceleration_damping: 1e-4,
            center_of_mass_weight: 0.25,
            strict_effector_targets: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WholeBodyJetReport {
    pub maximum_point_velocity_residual_mps: f64,
    pub maximum_point_acceleration_residual_mps2: f64,
    pub maximum_angular_velocity_residual_rad_s: f64,
    pub maximum_angular_acceleration_residual_rad_s2: f64,
    pub center_of_mass_velocity_residual_mps: f64,
    pub center_of_mass_acceleration_residual_mps2: f64,
    pub factorization_failures: usize,
}

/// Caller-owned storage for the dense, damped whole-body IK normal equations.
///
/// Construction allocates according to the immutable morphology. Repeated
/// solves only overwrite these buffers.
#[derive(Clone, Debug)]
pub struct WholeBodyIkScratch {
    pub model: ModelCache,
    dynamics: DynamicsCache,
    point_jacobian: DMatrix<f64>,
    angular_jacobian: DMatrix<f64>,
    center_of_mass_jacobian: DMatrix<f64>,
    floating_point_jacobian: DMatrix<f64>,
    floating_angular_jacobian: DMatrix<f64>,
    floating_center_of_mass_jacobian: DMatrix<f64>,
    floating_bias: DVector<f64>,
    normal: DMatrix<f64>,
    rhs: DVector<f64>,
    step: DVector<f64>,
    best_q: DVector<f64>,
    jet_solver: HierarchicalSolver,
    jet_tasks: TaskBuffer,
    jet_bounds: VelocityBounds,
    jet_result: SolveResult,
    jet_workspace: SolverWorkspace,
    lower: DVector<f64>,
    upper: DVector<f64>,
}

impl WholeBodyIkScratch {
    pub fn new(model: &CompiledModel) -> Self {
        let jet_three_row_capacity = model.bodies.len().saturating_mul(2).saturating_add(1);
        let jet_one_row_capacity = model.dof;
        let jet_task_capacity = jet_three_row_capacity
            .saturating_add(jet_one_row_capacity)
            .saturating_add(1);
        let jet_row_capacity = jet_three_row_capacity
            .saturating_mul(3)
            .saturating_add(jet_one_row_capacity);
        let mut lower = DVector::from_element(model.dof, f64::NEG_INFINITY);
        let mut upper = DVector::from_element(model.dof, f64::INFINITY);
        for joint in &model.joints {
            let Some(coordinate) = joint.coordinate else {
                continue;
            };
            lower[coordinate] = joint.limit.lower;
            upper[coordinate] = joint.limit.upper;
        }
        Self {
            model: ModelCache::new(model),
            dynamics: DynamicsCache::new(model),
            point_jacobian: DMatrix::zeros(3, model.dof),
            angular_jacobian: DMatrix::zeros(3, model.dof),
            center_of_mass_jacobian: DMatrix::zeros(3, model.dof),
            floating_point_jacobian: DMatrix::zeros(3, model.dof + 6),
            floating_angular_jacobian: DMatrix::zeros(3, model.dof + 6),
            floating_center_of_mass_jacobian: DMatrix::zeros(3, model.dof + 6),
            floating_bias: DVector::zeros(model.dof + 6),
            normal: DMatrix::zeros(model.dof, model.dof),
            rhs: DVector::zeros(model.dof),
            step: DVector::zeros(model.dof),
            best_q: DVector::zeros(model.dof),
            jet_solver: HierarchicalSolver::default(),
            jet_tasks: TaskBuffer::new(model.dof, jet_three_row_capacity, jet_one_row_capacity),
            jet_bounds: VelocityBounds::unbounded(model.dof),
            jet_result: SolveResult::workspace(model.dof, 0),
            jet_workspace: SolverWorkspace::new(model.dof, jet_row_capacity, jet_task_capacity, 0),
            lower,
            upper,
        }
    }
}

impl PlanarIkScratch {
    pub fn new(model: &CompiledModel) -> Self {
        let mut lower = DVector::from_element(model.dof, f64::NEG_INFINITY);
        let mut upper = DVector::from_element(model.dof, f64::INFINITY);
        for joint in &model.joints {
            let Some(coordinate) = joint.coordinate else {
                continue;
            };
            lower[coordinate] = joint.limit.lower;
            upper[coordinate] = joint.limit.upper;
        }
        Self {
            model: ModelCache::new(model),
            point_jacobian: DMatrix::zeros(3, model.dof),
            lower,
            upper,
        }
    }
}

#[derive(Debug, Error)]
pub enum PlanarIkError {
    #[error("invalid planar IK input")]
    InvalidInput,
    #[error(transparent)]
    Model(#[from] ModelError),
}

#[derive(Debug, Error)]
pub enum WholeBodyIkError {
    #[error("invalid whole-body IK input")]
    InvalidInput,
    #[error(transparent)]
    Model(#[from] ModelError),
}

/// Solve joint velocity and acceleration jets at an already-certified posture.
///
/// The root tangent is treated as known. The velocity solve enforces `J v`,
/// then the acceleration solve uses the model's analytic `Jdot v` bias. This
/// avoids manufacturing discontinuous jets by finite-differencing independent
/// position-only IK solves.
#[allow(clippy::too_many_arguments)]
pub fn solve_whole_body_kinematic_jets_into(
    model: &CompiledModel,
    state: &mut RobotState,
    root_twist_world: Motion6,
    root_angular_acceleration_world: Vec3,
    root_linear_acceleration_world: Vec3,
    point_targets: &[WholeBodyPointJetTarget],
    center_of_mass_target_velocity_world: Option<Vec3>,
    center_of_mass_target_acceleration_world: Option<Vec3>,
    coordinate_regularization: Option<&DVector<f64>>,
    joint_acceleration: &mut DVector<f64>,
    options: WholeBodyJetOptions,
    scratch: &mut WholeBodyIkScratch,
) -> Result<WholeBodyJetReport, WholeBodyIkError> {
    state.validate(model)?;
    let dof = model.dof;
    let generalized_dof = dof + 6;
    if joint_acceleration.len() != dof
        || coordinate_regularization.is_some_and(|regularization| {
            regularization.len() != dof
                || regularization
                    .iter()
                    .any(|value| !value.is_finite() || *value <= 0.0)
        })
        || scratch.point_jacobian.ncols() != dof
        || scratch.center_of_mass_jacobian.ncols() != dof
        || scratch.floating_point_jacobian.ncols() != generalized_dof
        || scratch.floating_center_of_mass_jacobian.ncols() != generalized_dof
        || scratch.floating_bias.len() != generalized_dof
        || !root_twist_world.0.iter().all(|value| value.is_finite())
        || !root_angular_acceleration_world
            .iter()
            .chain(root_linear_acceleration_world.iter())
            .all(|value| value.is_finite())
        || !options.velocity_damping.is_finite()
        || options.velocity_damping <= 0.0
        || !options.acceleration_damping.is_finite()
        || options.acceleration_damping <= 0.0
        || !options.center_of_mass_weight.is_finite()
        || options.center_of_mass_weight < 0.0
        || center_of_mass_target_velocity_world
            .is_some_and(|target| !target.iter().all(|value| value.is_finite()))
        || center_of_mass_target_acceleration_world
            .is_some_and(|target| !target.iter().all(|value| value.is_finite()))
        || point_targets.iter().any(|target| {
            target.frame.0 >= model.bodies.len()
                || !target.point_in_frame.iter().all(|value| value.is_finite())
                || !target
                    .target_velocity_world
                    .iter()
                    .chain(target.target_acceleration_world.iter())
                    .all(|value| value.is_finite())
                || !target.weight.is_finite()
                || target.weight < 0.0
                || !target
                    .target_angular_velocity_world
                    .iter()
                    .chain(target.target_angular_acceleration_world.iter())
                    .all(|value| value.is_finite())
                || !target.angular_weight.is_finite()
                || target.angular_weight < 0.0
        })
    {
        return Err(WholeBodyIkError::InvalidInput);
    }
    model.forward_kinematics(state, &mut scratch.model)?;

    let mut report = WholeBodyJetReport::default();
    if options.strict_effector_targets {
        scratch.jet_tasks.begin();
        for (index, target) in point_targets.iter().enumerate() {
            if target.weight == 0.0 {
                continue;
            }
            model.floating_point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.floating_point_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_point_jacobian,
                &mut scratch.point_jacobian,
            );
            let known_root_velocity = multiply_root_columns(
                &scratch.floating_point_jacobian,
                root_twist_world.0.as_slice(),
            );
            push_strict_three_row_jet_task(
                &mut scratch.jet_tasks,
                100 + index as u32 * 2,
                TaskKind::Point,
                Priority::Invariant,
                target.weight,
                &scratch.point_jacobian,
                target.target_velocity_world - known_root_velocity,
            )?;
            if target.angular_weight > 0.0 {
                model.floating_angular_jacobian_into(
                    &scratch.model,
                    target.frame,
                    &mut scratch.floating_angular_jacobian,
                )?;
                copy_joint_columns(
                    &scratch.floating_angular_jacobian,
                    &mut scratch.angular_jacobian,
                );
                let known_root_velocity = multiply_root_columns(
                    &scratch.floating_angular_jacobian,
                    root_twist_world.0.as_slice(),
                );
                push_strict_three_row_jet_task(
                    &mut scratch.jet_tasks,
                    101 + index as u32 * 2,
                    TaskKind::Orientation,
                    Priority::Invariant,
                    target.angular_weight,
                    &scratch.angular_jacobian,
                    target.target_angular_velocity_world - known_root_velocity,
                )?;
            }
        }
        if let Some(target) = center_of_mass_target_velocity_world
            && options.center_of_mass_weight > 0.0
        {
            model.floating_com_jacobian_into(
                &scratch.model,
                &mut scratch.dynamics,
                &mut scratch.floating_center_of_mass_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_center_of_mass_jacobian,
                &mut scratch.center_of_mass_jacobian,
            );
            let known_root_velocity = multiply_root_columns(
                &scratch.floating_center_of_mass_jacobian,
                root_twist_world.0.as_slice(),
            );
            push_strict_three_row_jet_task(
                &mut scratch.jet_tasks,
                1_000,
                TaskKind::CenterOfMass,
                Priority::Preference,
                options.center_of_mass_weight,
                &scratch.center_of_mass_jacobian,
                target - known_root_velocity,
            )?;
        }
        push_strict_coordinate_regularization(
            &mut scratch.jet_tasks,
            dof,
            options.velocity_damping,
            coordinate_regularization,
        )?;
        if !solve_strict_jet_tasks_into(scratch) {
            report.factorization_failures += 1;
            return Ok(report);
        }
        state.v.copy_from(&scratch.jet_result.velocity);
    } else {
        begin_jet_normal_equations(
            options.velocity_damping,
            coordinate_regularization,
            &mut scratch.normal,
            &mut scratch.rhs,
        );
        for target in point_targets {
            if target.weight == 0.0 {
                continue;
            }
            model.floating_point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.floating_point_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_point_jacobian,
                &mut scratch.point_jacobian,
            );
            let known_root_velocity = multiply_root_columns(
                &scratch.floating_point_jacobian,
                root_twist_world.0.as_slice(),
            );
            accumulate_normal_equations(
                &scratch.point_jacobian,
                target.target_velocity_world - known_root_velocity,
                target.weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
            if target.angular_weight > 0.0 {
                model.floating_angular_jacobian_into(
                    &scratch.model,
                    target.frame,
                    &mut scratch.floating_angular_jacobian,
                )?;
                copy_joint_columns(
                    &scratch.floating_angular_jacobian,
                    &mut scratch.angular_jacobian,
                );
                let known_root_velocity = multiply_root_columns(
                    &scratch.floating_angular_jacobian,
                    root_twist_world.0.as_slice(),
                );
                accumulate_normal_equations(
                    &scratch.angular_jacobian,
                    target.target_angular_velocity_world - known_root_velocity,
                    target.angular_weight,
                    &mut scratch.normal,
                    &mut scratch.rhs,
                );
            }
        }
        if let Some(target) = center_of_mass_target_velocity_world
            && options.center_of_mass_weight > 0.0
        {
            model.floating_com_jacobian_into(
                &scratch.model,
                &mut scratch.dynamics,
                &mut scratch.floating_center_of_mass_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_center_of_mass_jacobian,
                &mut scratch.center_of_mass_jacobian,
            );
            let known_root_velocity = multiply_root_columns(
                &scratch.floating_center_of_mass_jacobian,
                root_twist_world.0.as_slice(),
            );
            accumulate_normal_equations(
                &scratch.center_of_mass_jacobian,
                target - known_root_velocity,
                options.center_of_mass_weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
        }
        if !cholesky_solve_into(&mut scratch.normal, &scratch.rhs, &mut scratch.step) {
            report.factorization_failures += 1;
            return Ok(report);
        }
        state.v.copy_from(&scratch.step);
    }

    model.floating_bias_forces_into(
        state,
        root_twist_world,
        Vec3::zeros(),
        &scratch.model,
        &mut scratch.dynamics,
        &mut scratch.floating_bias,
    )?;
    let root_acceleration = [
        root_angular_acceleration_world.x,
        root_angular_acceleration_world.y,
        root_angular_acceleration_world.z,
        root_linear_acceleration_world.x,
        root_linear_acceleration_world.y,
        root_linear_acceleration_world.z,
    ];
    let center_of_mass_bias =
        model.center_of_mass_bias_acceleration_world(&scratch.model, &scratch.dynamics)?;
    if options.strict_effector_targets {
        scratch.jet_tasks.begin();
        for (index, target) in point_targets.iter().enumerate() {
            if target.weight == 0.0 {
                continue;
            }
            model.floating_point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.floating_point_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_point_jacobian,
                &mut scratch.point_jacobian,
            );
            let root_contribution =
                multiply_root_columns(&scratch.floating_point_jacobian, &root_acceleration);
            let bias = model.point_bias_acceleration_world(
                target.frame,
                target.point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )?;
            push_strict_three_row_jet_task(
                &mut scratch.jet_tasks,
                100 + index as u32 * 2,
                TaskKind::Point,
                Priority::Invariant,
                target.weight,
                &scratch.point_jacobian,
                target.target_acceleration_world - root_contribution - bias,
            )?;
            if target.angular_weight > 0.0 {
                model.floating_angular_jacobian_into(
                    &scratch.model,
                    target.frame,
                    &mut scratch.floating_angular_jacobian,
                )?;
                copy_joint_columns(
                    &scratch.floating_angular_jacobian,
                    &mut scratch.angular_jacobian,
                );
                let root_contribution =
                    multiply_root_columns(&scratch.floating_angular_jacobian, &root_acceleration);
                let bias =
                    model.angular_bias_acceleration_world(target.frame, &scratch.dynamics)?;
                push_strict_three_row_jet_task(
                    &mut scratch.jet_tasks,
                    101 + index as u32 * 2,
                    TaskKind::Orientation,
                    Priority::Invariant,
                    target.angular_weight,
                    &scratch.angular_jacobian,
                    target.target_angular_acceleration_world - root_contribution - bias,
                )?;
            }
        }
        if let Some(target) = center_of_mass_target_acceleration_world
            && options.center_of_mass_weight > 0.0
        {
            model.floating_com_jacobian_into(
                &scratch.model,
                &mut scratch.dynamics,
                &mut scratch.floating_center_of_mass_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_center_of_mass_jacobian,
                &mut scratch.center_of_mass_jacobian,
            );
            let root_contribution = multiply_root_columns(
                &scratch.floating_center_of_mass_jacobian,
                &root_acceleration,
            );
            push_strict_three_row_jet_task(
                &mut scratch.jet_tasks,
                1_000,
                TaskKind::CenterOfMass,
                Priority::Preference,
                options.center_of_mass_weight,
                &scratch.center_of_mass_jacobian,
                target - root_contribution - center_of_mass_bias,
            )?;
        }
        push_strict_coordinate_regularization(
            &mut scratch.jet_tasks,
            dof,
            options.acceleration_damping,
            coordinate_regularization,
        )?;
        if !solve_strict_jet_tasks_into(scratch) {
            report.factorization_failures += 1;
            return Ok(report);
        }
        joint_acceleration.copy_from(&scratch.jet_result.velocity);
    } else {
        begin_jet_normal_equations(
            options.acceleration_damping,
            coordinate_regularization,
            &mut scratch.normal,
            &mut scratch.rhs,
        );
        for target in point_targets {
            if target.weight == 0.0 {
                continue;
            }
            model.floating_point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.floating_point_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_point_jacobian,
                &mut scratch.point_jacobian,
            );
            let root_contribution =
                multiply_root_columns(&scratch.floating_point_jacobian, &root_acceleration);
            let bias = model.point_bias_acceleration_world(
                target.frame,
                target.point_in_frame,
                &scratch.model,
                &scratch.dynamics,
            )?;
            accumulate_normal_equations(
                &scratch.point_jacobian,
                target.target_acceleration_world - root_contribution - bias,
                target.weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
            if target.angular_weight > 0.0 {
                model.floating_angular_jacobian_into(
                    &scratch.model,
                    target.frame,
                    &mut scratch.floating_angular_jacobian,
                )?;
                copy_joint_columns(
                    &scratch.floating_angular_jacobian,
                    &mut scratch.angular_jacobian,
                );
                let root_contribution =
                    multiply_root_columns(&scratch.floating_angular_jacobian, &root_acceleration);
                let bias =
                    model.angular_bias_acceleration_world(target.frame, &scratch.dynamics)?;
                accumulate_normal_equations(
                    &scratch.angular_jacobian,
                    target.target_angular_acceleration_world - root_contribution - bias,
                    target.angular_weight,
                    &mut scratch.normal,
                    &mut scratch.rhs,
                );
            }
        }
        if let Some(target) = center_of_mass_target_acceleration_world
            && options.center_of_mass_weight > 0.0
        {
            model.floating_com_jacobian_into(
                &scratch.model,
                &mut scratch.dynamics,
                &mut scratch.floating_center_of_mass_jacobian,
            )?;
            copy_joint_columns(
                &scratch.floating_center_of_mass_jacobian,
                &mut scratch.center_of_mass_jacobian,
            );
            let root_contribution = multiply_root_columns(
                &scratch.floating_center_of_mass_jacobian,
                &root_acceleration,
            );
            accumulate_normal_equations(
                &scratch.center_of_mass_jacobian,
                target - root_contribution - center_of_mass_bias,
                options.center_of_mass_weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
        }
        if !cholesky_solve_into(&mut scratch.normal, &scratch.rhs, &mut scratch.step) {
            report.factorization_failures += 1;
            return Ok(report);
        }
        joint_acceleration.copy_from(&scratch.step);
    }

    for target in point_targets {
        if target.weight == 0.0 {
            continue;
        }
        model.floating_point_jacobian_into(
            &scratch.model,
            target.frame,
            target.point_in_frame,
            &mut scratch.floating_point_jacobian,
        )?;
        let velocity = multiply_generalized(
            &scratch.floating_point_jacobian,
            root_twist_world.0.as_slice(),
            state.v.as_slice(),
        );
        let bias = model.point_bias_acceleration_world(
            target.frame,
            target.point_in_frame,
            &scratch.model,
            &scratch.dynamics,
        )?;
        let acceleration = multiply_generalized(
            &scratch.floating_point_jacobian,
            &root_acceleration,
            joint_acceleration.as_slice(),
        ) + bias;
        report.maximum_point_velocity_residual_mps = report
            .maximum_point_velocity_residual_mps
            .max((target.target_velocity_world - velocity).norm());
        report.maximum_point_acceleration_residual_mps2 = report
            .maximum_point_acceleration_residual_mps2
            .max((target.target_acceleration_world - acceleration).norm());
        if target.angular_weight > 0.0 {
            model.floating_angular_jacobian_into(
                &scratch.model,
                target.frame,
                &mut scratch.floating_angular_jacobian,
            )?;
            let angular_velocity = multiply_generalized(
                &scratch.floating_angular_jacobian,
                root_twist_world.0.as_slice(),
                state.v.as_slice(),
            );
            let angular_bias =
                model.angular_bias_acceleration_world(target.frame, &scratch.dynamics)?;
            let angular_acceleration = multiply_generalized(
                &scratch.floating_angular_jacobian,
                &root_acceleration,
                joint_acceleration.as_slice(),
            ) + angular_bias;
            report.maximum_angular_velocity_residual_rad_s = report
                .maximum_angular_velocity_residual_rad_s
                .max((target.target_angular_velocity_world - angular_velocity).norm());
            report.maximum_angular_acceleration_residual_rad_s2 = report
                .maximum_angular_acceleration_residual_rad_s2
                .max((target.target_angular_acceleration_world - angular_acceleration).norm());
        }
    }
    if let Some(target) = center_of_mass_target_velocity_world {
        let velocity = multiply_generalized(
            &scratch.floating_center_of_mass_jacobian,
            root_twist_world.0.as_slice(),
            state.v.as_slice(),
        );
        report.center_of_mass_velocity_residual_mps = (target - velocity).norm();
    }
    if let Some(target) = center_of_mass_target_acceleration_world {
        let acceleration = multiply_generalized(
            &scratch.floating_center_of_mass_jacobian,
            &root_acceleration,
            joint_acceleration.as_slice(),
        ) + center_of_mass_bias;
        report.center_of_mass_acceleration_residual_mps2 = (target - acceleration).norm();
    }
    Ok(report)
}

fn begin_jet_normal_equations(
    damping: f64,
    coordinate_regularization: Option<&DVector<f64>>,
    normal: &mut DMatrix<f64>,
    rhs: &mut DVector<f64>,
) {
    normal.fill(0.0);
    rhs.fill(0.0);
    for coordinate in 0..normal.nrows() {
        normal[(coordinate, coordinate)] =
            damping * coordinate_regularization.map_or(1.0, |weights| weights[coordinate]);
    }
}

fn push_strict_three_row_jet_task(
    tasks: &mut TaskBuffer,
    stable_id: u32,
    kind: TaskKind,
    priority: Priority,
    weight: f64,
    jacobian: &DMatrix<f64>,
    target: Vec3,
) -> Result<(), WholeBodyIkError> {
    let task = tasks.push_three().ok_or(WholeBodyIkError::InvalidInput)?;
    task.stable_id = stable_id;
    task.kind = kind;
    task.priority = priority;
    task.weight = weight;
    task.jacobian.copy_from(jacobian);
    task.target_velocity.copy_from_slice(target.as_slice());
    Ok(())
}

fn push_strict_coordinate_regularization(
    tasks: &mut TaskBuffer,
    dof: usize,
    damping: f64,
    coordinate_regularization: Option<&DVector<f64>>,
) -> Result<(), WholeBodyIkError> {
    for coordinate in 0..dof {
        let task = tasks.push_one().ok_or(WholeBodyIkError::InvalidInput)?;
        task.stable_id = 10_000 + coordinate as u32;
        task.kind = TaskKind::Posture;
        task.priority = Priority::Preference;
        task.weight =
            damping * coordinate_regularization.map_or(1.0, |weights| weights[coordinate]);
        task.jacobian[(0, coordinate)] = 1.0;
        task.target_velocity[0] = 0.0;
    }
    Ok(())
}

fn solve_strict_jet_tasks_into(scratch: &mut WholeBodyIkScratch) -> bool {
    scratch.jet_solver.solve_task_buffer_into(
        scratch.jet_bounds.lower.len(),
        &scratch.jet_tasks,
        &scratch.jet_bounds,
        &[],
        &mut scratch.jet_result,
        &mut scratch.jet_workspace,
    );
    matches!(
        scratch.jet_result.diagnostics.status,
        SolveStatus::Solved | SolveStatus::SolvedWithSlack
    )
}

fn copy_joint_columns(floating: &DMatrix<f64>, joints: &mut DMatrix<f64>) {
    for row in 0..3 {
        for column in 0..joints.ncols() {
            joints[(row, column)] = floating[(row, 6 + column)];
        }
    }
}

fn multiply_root_columns(jacobian: &DMatrix<f64>, root: &[f64]) -> Vec3 {
    Vec3::from_fn(|row, _| {
        (0..6)
            .map(|column| jacobian[(row, column)] * root[column])
            .sum()
    })
}

fn multiply_generalized(jacobian: &DMatrix<f64>, root: &[f64], joints: &[f64]) -> Vec3 {
    Vec3::from_fn(|row, _| {
        (0..6)
            .map(|column| jacobian[(row, column)] * root[column])
            .chain(
                joints
                    .iter()
                    .enumerate()
                    .map(|(column, value)| jacobian[(row, 6 + column)] * value),
            )
            .sum()
    })
}

/// Solve a morphology-consistent posture for fixed root/effector/CoM targets.
///
/// This is an offline reference-certificate primitive, not a feedback policy.
/// It forms damped normal equations in caller-owned storage and uses an
/// allocation-free in-place Cholesky factorization. Joint limits are applied
/// on every update. Sequential callers may warm-start from the previous tick.
pub fn solve_whole_body_ik_into(
    model: &CompiledModel,
    state: &mut RobotState,
    point_targets: &[WholeBodyPointIkTarget],
    center_of_mass_target_world: Option<Vec3>,
    nominal_posture: &DVector<f64>,
    options: WholeBodyIkOptions,
    scratch: &mut WholeBodyIkScratch,
) -> Result<WholeBodyIkReport, WholeBodyIkError> {
    state.validate(model)?;
    let dof = model.dof;
    if nominal_posture.len() != dof
        || scratch.point_jacobian.ncols() != dof
        || scratch.center_of_mass_jacobian.ncols() != dof
        || scratch.normal.nrows() != dof
        || scratch.normal.ncols() != dof
        || scratch.rhs.len() != dof
        || scratch.step.len() != dof
        || scratch.best_q.len() != dof
        || scratch.lower.len() != dof
        || scratch.upper.len() != dof
        || options.maximum_iterations == 0
        || options.minimum_iterations > options.maximum_iterations
        || !options.damping.is_finite()
        || options.damping <= 0.0
        || !options.posture_weight.is_finite()
        || options.posture_weight < 0.0
        || !options.center_of_mass_weight.is_finite()
        || options.center_of_mass_weight < 0.0
        || !options.orientation_weight.is_finite()
        || options.orientation_weight < 0.0
        || !options.maximum_step_rad.is_finite()
        || options.maximum_step_rad <= 0.0
        || !options.point_tolerance_m.is_finite()
        || options.point_tolerance_m < 0.0
        || !options.center_of_mass_tolerance_m.is_finite()
        || options.center_of_mass_tolerance_m < 0.0
        || !options.orientation_tolerance_rad.is_finite()
        || options.orientation_tolerance_rad < 0.0
        || !nominal_posture.iter().all(|value| value.is_finite())
        || center_of_mass_target_world
            .is_some_and(|target| !target.iter().all(|value| value.is_finite()))
        || point_targets.iter().any(|target| {
            target.frame.0 >= model.bodies.len()
                || !target.point_in_frame.iter().all(|value| value.is_finite())
                || !target.target_world.iter().all(|value| value.is_finite())
                || !target.weight.is_finite()
                || target.weight < 0.0
                || !target.orientation_weight.is_finite()
                || target.orientation_weight < 0.0
        })
    {
        return Err(WholeBodyIkError::InvalidInput);
    }

    let mut report = WholeBodyIkReport::default();
    update_whole_body_report(
        model,
        state,
        point_targets,
        center_of_mass_target_world,
        options,
        scratch,
        &mut report,
    )?;
    if report.converged && options.minimum_iterations == 0 {
        return Ok(report);
    }
    scratch.best_q.copy_from(&state.q);
    let mut best_report = report;
    let mut best_merit =
        whole_body_report_merit(report, center_of_mass_target_world.is_some(), options);
    for iteration in 0..options.maximum_iterations {
        model.forward_kinematics(state, &mut scratch.model)?;
        scratch.normal.fill(0.0);
        scratch.rhs.fill(0.0);
        for coordinate in 0..dof {
            scratch.normal[(coordinate, coordinate)] = options.damping + options.posture_weight;
            scratch.rhs[coordinate] =
                options.posture_weight * (nominal_posture[coordinate] - state.q[coordinate]);
        }

        for target in point_targets {
            if target.weight == 0.0 {
                continue;
            }
            model.point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.point_jacobian,
            )?;
            let point_world = scratch.model.world_from_body[target.frame.0]
                .transform_point(&Point3::from(target.point_in_frame))
                .coords;
            let error = target.target_world - point_world;
            accumulate_normal_equations(
                &scratch.point_jacobian,
                error,
                target.weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
            if target.orientation_weight > 0.0 && options.orientation_weight > 0.0 {
                model.angular_jacobian_into(
                    &scratch.model,
                    target.frame,
                    &mut scratch.angular_jacobian,
                )?;
                let rotation = scratch.model.world_from_body[target.frame.0].rotation;
                let local_error = rotation
                    .rotation_to(&target.target_orientation_world)
                    .scaled_axis();
                let error_world = rotation.transform_vector(&local_error);
                accumulate_normal_equations(
                    &scratch.angular_jacobian,
                    error_world,
                    target.orientation_weight * options.orientation_weight,
                    &mut scratch.normal,
                    &mut scratch.rhs,
                );
            }
        }

        if let Some(target) = center_of_mass_target_world
            && options.center_of_mass_weight > 0.0
        {
            model.com_jacobian_into(&scratch.model, &mut scratch.center_of_mass_jacobian)?;
            accumulate_normal_equations(
                &scratch.center_of_mass_jacobian,
                target - scratch.model.center_of_mass_world,
                options.center_of_mass_weight,
                &mut scratch.normal,
                &mut scratch.rhs,
            );
        }

        if !cholesky_solve_into(&mut scratch.normal, &scratch.rhs, &mut scratch.step) {
            report.factorization_failures += 1;
            break;
        }
        for coordinate in 0..dof {
            let update =
                scratch.step[coordinate].clamp(-options.maximum_step_rad, options.maximum_step_rad);
            let unconstrained = state.q[coordinate] + update;
            let constrained =
                unconstrained.clamp(scratch.lower[coordinate], scratch.upper[coordinate]);
            if constrained != unconstrained {
                report.limit_clamped_coordinates += 1;
            }
            state.q[coordinate] = constrained;
        }
        report.iterations = iteration + 1;
        update_whole_body_report(
            model,
            state,
            point_targets,
            center_of_mass_target_world,
            options,
            scratch,
            &mut report,
        )?;
        let merit = whole_body_report_merit(report, center_of_mass_target_world.is_some(), options);
        if merit < best_merit {
            best_merit = merit;
            best_report = report;
            scratch.best_q.copy_from(&state.q);
        }
        if report.converged && iteration + 1 >= options.minimum_iterations {
            break;
        }
    }
    if !report.converged {
        let total_iterations = report.iterations;
        let total_limit_clamps = report.limit_clamped_coordinates;
        let total_factorization_failures = report.factorization_failures;
        state.q.copy_from(&scratch.best_q);
        report = best_report;
        report.iterations = total_iterations;
        report.limit_clamped_coordinates = total_limit_clamps;
        report.factorization_failures = total_factorization_failures;
        // Keep the model cache coherent with the posture returned to callers.
        update_whole_body_report(
            model,
            state,
            point_targets,
            center_of_mass_target_world,
            options,
            scratch,
            &mut report,
        )?;
    }
    Ok(report)
}

fn whole_body_report_merit(
    report: WholeBodyIkReport,
    has_center_of_mass_target: bool,
    options: WholeBodyIkOptions,
) -> f64 {
    let point_scale = options.point_tolerance_m.max(1.0e-12);
    let orientation_scale = options.orientation_tolerance_rad.max(1.0e-12);
    let mut merit = (report.maximum_point_error_m / point_scale)
        .max(report.maximum_orientation_error_rad / orientation_scale);
    if has_center_of_mass_target && options.center_of_mass_weight > 0.0 {
        merit = merit
            .max(report.center_of_mass_error_m / options.center_of_mass_tolerance_m.max(1.0e-12));
    }
    merit
}

fn accumulate_normal_equations(
    jacobian: &DMatrix<f64>,
    error: Vec3,
    weight: f64,
    normal: &mut DMatrix<f64>,
    rhs: &mut DVector<f64>,
) {
    let dof = jacobian.ncols();
    for column in 0..dof {
        let mut projected_error = 0.0;
        for axis in 0..3 {
            projected_error += jacobian[(axis, column)] * error[axis];
        }
        rhs[column] += weight * projected_error;
        for other in 0..=column {
            let mut product = 0.0;
            for axis in 0..3 {
                product += jacobian[(axis, column)] * jacobian[(axis, other)];
            }
            let value = weight * product;
            normal[(column, other)] += value;
            if column != other {
                normal[(other, column)] += value;
            }
        }
    }
}

fn cholesky_solve_into(
    normal: &mut DMatrix<f64>,
    rhs: &DVector<f64>,
    solution: &mut DVector<f64>,
) -> bool {
    let size = normal.nrows();
    for row in 0..size {
        for column in 0..=row {
            let mut value = normal[(row, column)];
            for inner in 0..column {
                value -= normal[(row, inner)] * normal[(column, inner)];
            }
            if row == column {
                if !value.is_finite() || value <= 1e-18 {
                    return false;
                }
                normal[(row, column)] = value.sqrt();
            } else {
                normal[(row, column)] = value / normal[(column, column)];
            }
        }
    }
    for row in 0..size {
        let mut value = rhs[row];
        for column in 0..row {
            value -= normal[(row, column)] * solution[column];
        }
        solution[row] = value / normal[(row, row)];
    }
    for row in (0..size).rev() {
        let mut value = solution[row];
        for column in row + 1..size {
            value -= normal[(column, row)] * solution[column];
        }
        solution[row] = value / normal[(row, row)];
    }
    solution.iter().all(|value| value.is_finite())
}

fn update_whole_body_report(
    model: &CompiledModel,
    state: &RobotState,
    point_targets: &[WholeBodyPointIkTarget],
    center_of_mass_target_world: Option<Vec3>,
    options: WholeBodyIkOptions,
    scratch: &mut WholeBodyIkScratch,
    report: &mut WholeBodyIkReport,
) -> Result<(), ModelError> {
    model.forward_kinematics(state, &mut scratch.model)?;
    report.maximum_point_error_m = 0.0;
    for target in point_targets {
        if target.weight == 0.0 {
            continue;
        }
        let point_world = scratch.model.world_from_body[target.frame.0]
            .transform_point(&Point3::from(target.point_in_frame))
            .coords;
        report.maximum_point_error_m = report
            .maximum_point_error_m
            .max((target.target_world - point_world).norm());
    }
    report.center_of_mass_error_m = center_of_mass_target_world.map_or(0.0, |target| {
        (target - scratch.model.center_of_mass_world).norm()
    });
    report.maximum_orientation_error_rad = 0.0;
    for target in point_targets {
        if target.orientation_weight == 0.0 || options.orientation_weight == 0.0 {
            continue;
        }
        let rotation = scratch.model.world_from_body[target.frame.0].rotation;
        report.maximum_orientation_error_rad = report.maximum_orientation_error_rad.max(
            rotation
                .rotation_to(&target.target_orientation_world)
                .angle(),
        );
    }
    report.converged = report.maximum_point_error_m <= options.point_tolerance_m
        && (center_of_mass_target_world.is_none()
            || options.center_of_mass_weight == 0.0
            || report.center_of_mass_error_m <= options.center_of_mass_tolerance_m)
        && report.maximum_orientation_error_rad <= options.orientation_tolerance_rad;
    Ok(())
}

/// Solve independent two-coordinate point targets with damped Gauss-Newton.
///
/// The caller supplies state and reusable scratch. Once constructed, the solve
/// performs no heap allocation and applies authored joint-position limits on
/// every update.
pub fn solve_planar_point_ik_into(
    model: &CompiledModel,
    state: &mut RobotState,
    targets: &[PlanarPointIkTarget],
    options: PlanarIkOptions,
    scratch: &mut PlanarIkScratch,
) -> Result<PlanarIkReport, PlanarIkError> {
    state.validate(model)?;
    if scratch.point_jacobian.ncols() != model.dof
        || scratch.lower.len() != model.dof
        || scratch.upper.len() != model.dof
        || options.maximum_iterations == 0
        || !options.damping.is_finite()
        || options.damping < 0.0
        || !options.maximum_step_rad.is_finite()
        || options.maximum_step_rad <= 0.0
        || !options.tolerance_m.is_finite()
        || options.tolerance_m < 0.0
        || targets.iter().any(|target| {
            target.frame.0 >= model.bodies.len()
                || target.coordinates[0] >= model.dof
                || target.coordinates[1] >= model.dof
                || target.coordinates[0] == target.coordinates[1]
                || target.axes[0] >= 3
                || target.axes[1] >= 3
                || target.axes[0] == target.axes[1]
                || !target.point_in_frame.iter().all(|value| value.is_finite())
                || !target.target_world.iter().all(|value| value.is_finite())
        })
    {
        return Err(PlanarIkError::InvalidInput);
    }

    let mut report = PlanarIkReport::default();
    for iteration in 0..options.maximum_iterations {
        let mut maximum_error: f64 = 0.0;
        for target in targets {
            model.forward_kinematics(state, &mut scratch.model)?;
            model.point_jacobian_into(
                &scratch.model,
                target.frame,
                target.point_in_frame,
                &mut scratch.point_jacobian,
            )?;
            let point_world: Vec3 = scratch.model.world_from_body[target.frame.0]
                .transform_point(&Point3::from(target.point_in_frame))
                .coords;
            let error = target.target_world - point_world;
            let e0 = error[target.axes[0]];
            let e1 = error[target.axes[1]];
            maximum_error = maximum_error.max(e0.hypot(e1));

            let c0 = target.coordinates[0];
            let c1 = target.coordinates[1];
            let j00 = scratch.point_jacobian[(target.axes[0], c0)];
            let j01 = scratch.point_jacobian[(target.axes[0], c1)];
            let j10 = scratch.point_jacobian[(target.axes[1], c0)];
            let j11 = scratch.point_jacobian[(target.axes[1], c1)];
            let a = j00 * j00 + j10 * j10 + options.damping;
            let b = j00 * j01 + j10 * j11;
            let d = j01 * j01 + j11 * j11 + options.damping;
            let rhs0 = j00 * e0 + j10 * e1;
            let rhs1 = j01 * e0 + j11 * e1;
            let determinant = a * d - b * b;
            if determinant.abs() <= 1e-18 {
                report.singular_updates += 1;
                continue;
            }
            let delta0 = ((d * rhs0 - b * rhs1) / determinant)
                .clamp(-options.maximum_step_rad, options.maximum_step_rad);
            let delta1 = ((a * rhs1 - b * rhs0) / determinant)
                .clamp(-options.maximum_step_rad, options.maximum_step_rad);
            state.q[c0] = (state.q[c0] + delta0).clamp(scratch.lower[c0], scratch.upper[c0]);
            state.q[c1] = (state.q[c1] + delta1).clamp(scratch.lower[c1], scratch.upper[c1]);
        }
        report.iterations = iteration + 1;
        report.maximum_planar_error_m = maximum_error;
        if maximum_error <= options.tolerance_m {
            report.converged = true;
            break;
        }
    }

    model.forward_kinematics(state, &mut scratch.model)?;
    report.maximum_planar_error_m = 0.0;
    for target in targets {
        let point_world: Vec3 = scratch.model.world_from_body[target.frame.0]
            .transform_point(&Point3::from(target.point_in_frame))
            .coords;
        let error = target.target_world - point_world;
        report.maximum_planar_error_m = report
            .maximum_planar_error_m
            .max(error[target.axes[0]].hypot(error[target.axes[1]]));
    }
    report.converged = report.maximum_planar_error_m <= options.tolerance_m;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::load_urdf;

    #[test]
    fn upkie_planar_squat_preserves_both_material_contacts_without_allocation() {
        let model = load_urdf(include_str!("../../../models/upkie/upkie.urdf")).unwrap();
        let mut state = RobotState::zeros(&model);
        for (name, value) in [
            ("left_hip", 0.4),
            ("left_knee", -0.625),
            ("right_hip", -0.4),
            ("right_knee", 0.625),
        ] {
            let coordinate = model
                .joint_id(name)
                .and_then(|joint| model.joints[joint.0].coordinate)
                .unwrap();
            state.q[coordinate] = value;
        }
        let mut scratch = PlanarIkScratch::new(&model);
        model
            .forward_kinematics(&state, &mut scratch.model)
            .unwrap();
        let left_frame = model.frame_id("left_contact").unwrap();
        let right_frame = model.frame_id("right_contact").unwrap();
        let targets = [
            PlanarPointIkTarget {
                frame: left_frame,
                point_in_frame: Vec3::zeros(),
                target_world: scratch.model.world_from_body[left_frame.0]
                    .translation
                    .vector,
                coordinates: [0, 1],
                axes: [0, 2],
            },
            PlanarPointIkTarget {
                frame: right_frame,
                point_in_frame: Vec3::zeros(),
                target_world: scratch.model.world_from_body[right_frame.0]
                    .translation
                    .vector,
                coordinates: [3, 4],
                axes: [0, 2],
            },
        ];
        state.control_world_from_root.translation.vector.z = -0.12;
        let report = solve_planar_point_ik_into(
            &model,
            &mut state,
            &targets,
            PlanarIkOptions::default(),
            &mut scratch,
        )
        .unwrap();
        assert!(report.converged, "{report:?}");
        assert!(report.maximum_planar_error_m < 1e-7, "{report:?}");
        assert!(state.q[0] > 0.4);
        assert!(state.q[1] < -0.625);
        assert!(state.q[3] < -0.4);
        assert!(state.q[4] > 0.625);
    }

    #[test]
    fn whole_body_position_and_jet_witness_reproduce_a_stationary_pose() {
        let model = load_urdf(include_str!("../../../models/upkie/upkie.urdf")).unwrap();
        let mut state = RobotState::zeros(&model);
        let mut scratch = WholeBodyIkScratch::new(&model);
        model
            .forward_kinematics(&state, &mut scratch.model)
            .unwrap();
        let frame = model.frame_id("left_contact").unwrap();
        let position_target = WholeBodyPointIkTarget {
            frame,
            point_in_frame: Vec3::zeros(),
            target_world: scratch.model.world_from_body[frame.0].translation.vector,
            weight: 1.0,
            target_orientation_world: scratch.model.world_from_body[frame.0].rotation,
            orientation_weight: 1.0,
        };
        let center_of_mass_target = scratch.model.center_of_mass_world;
        let nominal_posture = state.q.clone();
        let position = solve_whole_body_ik_into(
            &model,
            &mut state,
            &[position_target],
            Some(center_of_mass_target),
            &nominal_posture,
            WholeBodyIkOptions::default(),
            &mut scratch,
        )
        .unwrap();
        assert!(position.converged, "{position:?}");
        assert_eq!(position.iterations, 0);

        let jet_target = WholeBodyPointJetTarget {
            frame,
            point_in_frame: Vec3::zeros(),
            target_velocity_world: Vec3::zeros(),
            target_acceleration_world: Vec3::zeros(),
            weight: 1.0,
            target_angular_velocity_world: Vec3::zeros(),
            target_angular_acceleration_world: Vec3::zeros(),
            angular_weight: 1.0,
        };
        let mut joint_acceleration = DVector::zeros(model.dof);
        let jets = solve_whole_body_kinematic_jets_into(
            &model,
            &mut state,
            Motion6::default(),
            Vec3::zeros(),
            Vec3::zeros(),
            &[jet_target],
            Some(Vec3::zeros()),
            Some(Vec3::zeros()),
            None,
            &mut joint_acceleration,
            WholeBodyJetOptions::default(),
            &mut scratch,
        )
        .unwrap();
        assert_eq!(jets.factorization_failures, 0);
        assert!(state.v.norm() < 1e-12, "{}", state.v.norm());
        assert!(
            joint_acceleration.norm() < 1e-12,
            "{}",
            joint_acceleration.norm()
        );
        assert!(jets.maximum_point_velocity_residual_mps < 1e-12);
        assert!(jets.maximum_point_acceleration_residual_mps2 < 1e-12);
        assert!(jets.center_of_mass_velocity_residual_mps < 1e-12);
        assert!(jets.center_of_mass_acceleration_residual_mps2 < 1e-12);

        let strict_jets = solve_whole_body_kinematic_jets_into(
            &model,
            &mut state,
            Motion6::default(),
            Vec3::zeros(),
            Vec3::zeros(),
            &[jet_target],
            Some(Vec3::zeros()),
            Some(Vec3::zeros()),
            None,
            &mut joint_acceleration,
            WholeBodyJetOptions {
                strict_effector_targets: true,
                ..WholeBodyJetOptions::default()
            },
            &mut scratch,
        )
        .unwrap();
        assert_eq!(strict_jets.factorization_failures, 0);
        assert!(state.v.norm() < 1e-12, "{}", state.v.norm());
        assert!(
            joint_acceleration.norm() < 1e-12,
            "{}",
            joint_acceleration.norm()
        );
        assert!(strict_jets.maximum_point_velocity_residual_mps < 1e-12);
        assert!(strict_jets.maximum_point_acceleration_residual_mps2 < 1e-12);
    }
}
