//! Allocation-free contact-transition impulse and velocity-jump bounds.
//!
//! This module does not infer contact, friction, compliance, or effective mass.
//! The caller supplies conservative pre-impact witnesses and a generalized
//! velocity response `M^-1 J^T`. The result is a componentwise outer bound,
//! not a complementarity solve, impact-law identification, or safety claim.

use nalgebra::{DMatrix, DVector, Point3};

use crate::model::{
    CompiledModel, DynamicsCache, FloatingRobotState, FrameId, ModelCache, ModelError, RobotState,
};

/// Contact-witness columns in the flat contact-major input.
pub const CONTACT_TRANSITION_WITNESS_WIDTH: usize = 4;
/// Local contact-impulse axes: tangent X, tangent Y, then normal.
pub const CONTACT_TRANSITION_IMPULSE_WIDTH: usize = 3;
/// Spatial-impulse axes: moment X/Y/Z, then force X/Y/Z.
pub const SPATIAL_IMPULSE_WIDTH: usize = 6;
/// Directional witness columns: speed[3], effective mass[3], sustained
/// force[3], then friction coefficient.
pub const DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH: usize = 10;
/// Spatial patch witness columns: speed XYZ, effective mass XYZ, sustained
/// force XYZ, friction, half-length X, half-width Y, and torsion radius.
/// Force and moment capacities share one nonnegative normal impulse.
pub const SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH: usize = 13;

#[derive(Clone, Copy, Debug)]
pub struct ContactTransitionInput<'a> {
    /// Earliest and latest possible impact time measured from the current
    /// state. Both bounds are finite, nonnegative, and ordered.
    pub transition_time_lower_s: f64,
    pub transition_time_upper_s: f64,
    /// Upper coefficient of restitution shared by the declared contacts.
    pub restitution_upper: f64,
    /// Contact-major rows `[closing_speed_upper_m_s,
    /// effective_normal_mass_upper_kg, sustained_normal_force_upper_n,
    /// friction_coefficient_upper]`.
    pub contact_witnesses: &'a [f64],
    /// Constant pre-impact generalized acceleration for the candidate.
    pub generalized_acceleration: &'a [f64],
    /// Row-major `[dof, contact, 3]` map from local impulse to generalized
    /// velocity jump. This is the caller's `M^-1 J^T` witness.
    pub impulse_velocity_response: &'a [f64],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactTransitionError {
    InvalidConfig,
    Dimension,
    InvalidWitness,
    InvalidResponse,
}

/// One point-contact response query expressed in the control-world frame.
///
/// The three basis vectors are ordered tangent X, tangent Y, then normal. They
/// must form a right-handed orthonormal basis. Keeping the point in world
/// coordinates lets a plant or collision layer pass its exact prospective
/// point while Rust remains the sole owner of the model Jacobian and inertia.
#[derive(Clone, Copy, Debug)]
pub struct PointImpulseResponseSpec {
    pub frame: FrameId,
    pub point_world: crate::math::Vec3,
    pub basis_world: [crate::math::Vec3; CONTACT_TRANSITION_IMPULSE_WIDTH],
}

/// One spatial-impulse response query expressed in a declared orthonormal basis.
///
/// The first three response columns are moment impulse about
/// `reference_point_world`; the last three are force impulse through that
/// point. The reference point is rigidly attached to `frame` for the query.
/// This representation preserves both resultant force and resultant moment,
/// unlike replacing a distributed contact patch with one force-only point.
#[derive(Clone, Copy, Debug)]
pub struct SpatialImpulseResponseSpec {
    pub frame: FrameId,
    pub reference_point_world: crate::math::Vec3,
    pub basis_world: [crate::math::Vec3; CONTACT_TRANSITION_IMPULSE_WIDTH],
}

/// Caller-owned workspace for model-derived `M^-1 J^T` contact responses.
///
/// Construction allocates once. `write_point_impulse_velocity_response` then
/// performs FK, floating mass assembly, one Cholesky factorization, and all
/// point-axis solves without growing storage.
#[derive(Clone, Debug)]
pub struct ContactTransitionResponseScratch {
    pub model: ModelCache,
    dynamics: DynamicsCache,
    mass_factor: DMatrix<f64>,
    inverse_mass: DMatrix<f64>,
    partition_factor: DMatrix<f64>,
    point_jacobian: DMatrix<f64>,
    angular_jacobian: DMatrix<f64>,
    rhs: DVector<f64>,
    solution: DVector<f64>,
    partition_rhs: DVector<f64>,
    partition_solution: DVector<f64>,
    partition_radius: DVector<f64>,
}

impl ContactTransitionResponseScratch {
    pub fn new(model: &CompiledModel) -> Self {
        let generalized_dof = model.dof + 6;
        Self {
            model: ModelCache::new(model),
            dynamics: DynamicsCache::new(model),
            mass_factor: DMatrix::zeros(generalized_dof, generalized_dof),
            inverse_mass: DMatrix::zeros(generalized_dof, generalized_dof),
            partition_factor: DMatrix::zeros(generalized_dof, generalized_dof),
            point_jacobian: DMatrix::zeros(3, generalized_dof),
            angular_jacobian: DMatrix::zeros(3, generalized_dof),
            rhs: DVector::zeros(generalized_dof),
            solution: DVector::zeros(generalized_dof),
            partition_rhs: DVector::zeros(generalized_dof),
            partition_solution: DVector::zeros(generalized_dof),
            partition_radius: DVector::zeros(generalized_dof),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactTransitionResponseError {
    Dimension,
    InvalidContact,
    SingularMassMatrix,
    Model,
}

fn factor_spd_lower(matrix: &mut DMatrix<f64>) -> bool {
    let size = matrix.nrows();
    for row in 0..size {
        for column in 0..=row {
            let mut value = matrix[(row, column)];
            for inner in 0..column {
                value -= matrix[(row, inner)] * matrix[(column, inner)];
            }
            if row == column {
                if !value.is_finite() || value <= 1.0e-18 {
                    return false;
                }
                matrix[(row, column)] = value.sqrt();
            } else {
                matrix[(row, column)] = value / matrix[(column, column)];
            }
        }
    }
    true
}

fn solve_spd_factor_into(
    factor: &DMatrix<f64>,
    rhs: &DVector<f64>,
    solution: &mut DVector<f64>,
) -> bool {
    let size = factor.nrows();
    for row in 0..size {
        let mut value = rhs[row];
        for column in 0..row {
            value -= factor[(row, column)] * solution[column];
        }
        solution[row] = value / factor[(row, row)];
    }
    for row in (0..size).rev() {
        let mut value = solution[row];
        for column in row + 1..size {
            value -= factor[(column, row)] * solution[column];
        }
        solution[row] = value / factor[(row, row)];
    }
    solution.iter().all(|value| value.is_finite())
}

fn factor_spd_lower_prefix(matrix: &mut DMatrix<f64>, size: usize) -> bool {
    for row in 0..size {
        for column in 0..=row {
            let mut value = matrix[(row, column)];
            for inner in 0..column {
                value -= matrix[(row, inner)] * matrix[(column, inner)];
            }
            if row == column {
                if !value.is_finite() || value <= 1.0e-18 {
                    return false;
                }
                matrix[(row, column)] = value.sqrt();
            } else {
                matrix[(row, column)] = value / matrix[(column, column)];
            }
        }
    }
    true
}

fn solve_spd_factor_prefix_into(
    factor: &DMatrix<f64>,
    rhs: &DVector<f64>,
    solution: &mut DVector<f64>,
    size: usize,
) -> bool {
    for row in 0..size {
        let mut value = rhs[row];
        for column in 0..row {
            value -= factor[(row, column)] * solution[column];
        }
        solution[row] = value / factor[(row, row)];
    }
    for row in (0..size).rev() {
        let mut value = solution[row];
        for column in row + 1..size {
            value -= factor[(column, row)] * solution[column];
        }
        solution[row] = value / factor[(row, row)];
    }
    solution.iter().take(size).all(|value| value.is_finite())
}

/// Derive point-contact impulse response and directional effective mass.
///
/// `response_out` is row-major `[generalized_dof, contact, 3]` and
/// `effective_mass_out` is contact-major `[contact, 3]`. The generalized
/// tangent is `[root angular; root linear; joints]`. All inputs are validated
/// before either output is modified. Runtime work is allocation-free.
pub fn write_point_impulse_velocity_response(
    model: &CompiledModel,
    state: &RobotState,
    contacts: &[PointImpulseResponseSpec],
    scratch: &mut ContactTransitionResponseScratch,
    response_out: &mut [f64],
    effective_mass_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    let contact_axes = contacts
        .len()
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ContactTransitionResponseError::Dimension)?;
    if contacts.is_empty()
        || response_out.len()
            != generalized_dof
                .checked_mul(contact_axes)
                .ok_or(ContactTransitionResponseError::Dimension)?
        || effective_mass_out.len() != contact_axes
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.point_jacobian.shape() != (3, generalized_dof)
        || scratch.rhs.len() != generalized_dof
        || scratch.solution.len() != generalized_dof
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    let valid_contact = |contact: &PointImpulseResponseSpec| {
        if contact.frame.0 >= model.bodies.len()
            || contact.point_world.iter().any(|value| !value.is_finite())
            || contact
                .basis_world
                .iter()
                .flat_map(|axis| axis.iter())
                .any(|value| !value.is_finite())
        {
            return false;
        }
        for axis in 0..3 {
            if (contact.basis_world[axis].norm_squared() - 1.0).abs() > 1.0e-9 {
                return false;
            }
            for other in 0..axis {
                if contact.basis_world[axis]
                    .dot(&contact.basis_world[other])
                    .abs()
                    > 1.0e-9
                {
                    return false;
                }
            }
        }
        contact.basis_world[0]
            .cross(&contact.basis_world[1])
            .dot(&contact.basis_world[2])
            > 1.0 - 1.0e-9
    };
    if !contacts.iter().all(valid_contact) {
        return Err(ContactTransitionResponseError::InvalidContact);
    }

    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    if !factor_spd_lower(&mut scratch.mass_factor) {
        return Err(ContactTransitionResponseError::SingularMassMatrix);
    }

    for (contact_index, contact) in contacts.iter().enumerate() {
        let pose = scratch.model.world_from_body[contact.frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(contact.point_world))
            .coords;
        model
            .floating_point_jacobian_into(
                &scratch.model,
                contact.frame,
                point_in_frame,
                &mut scratch.point_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        for axis in 0..CONTACT_TRANSITION_IMPULSE_WIDTH {
            for coordinate in 0..generalized_dof {
                scratch.rhs[coordinate] = (0..3)
                    .map(|row| {
                        scratch.point_jacobian[(row, coordinate)] * contact.basis_world[axis][row]
                    })
                    .sum();
            }
            if !solve_spd_factor_into(&scratch.mass_factor, &scratch.rhs, &mut scratch.solution) {
                return Err(ContactTransitionResponseError::SingularMassMatrix);
            }
            let inverse_effective_mass = scratch.rhs.dot(&scratch.solution);
            if !inverse_effective_mass.is_finite() || inverse_effective_mass <= 0.0 {
                return Err(ContactTransitionResponseError::SingularMassMatrix);
            }
            effective_mass_out[contact_index * 3 + axis] = 1.0 / inverse_effective_mass;
            for coordinate in 0..generalized_dof {
                response_out[coordinate * contact_axes + contact_index * 3 + axis] =
                    scratch.solution[coordinate];
            }
        }
    }
    Ok(())
}

/// Derive point-contact response together with the coupled Delassus operator.
///
/// `delassus_out` is row-major `[contact * 3, contact * 3]` in the same
/// tangent-X, tangent-Y, normal ordering as `response_out`. It contains
/// `J M^-1 J^T`, including cross-contact and cross-axis terms that disappear
/// from independent directional effective masses. Runtime work is
/// allocation-free; the point Jacobians are recomputed into existing scratch
/// after the shared mass solve.
pub fn write_point_impulse_velocity_response_with_delassus(
    model: &CompiledModel,
    state: &RobotState,
    contacts: &[PointImpulseResponseSpec],
    scratch: &mut ContactTransitionResponseScratch,
    response_out: &mut [f64],
    effective_mass_out: &mut [f64],
    delassus_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    let contact_axes = contacts
        .len()
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ContactTransitionResponseError::Dimension)?;
    if delassus_out.len()
        != contact_axes
            .checked_mul(contact_axes)
            .ok_or(ContactTransitionResponseError::Dimension)?
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    write_point_impulse_velocity_response(
        model,
        state,
        contacts,
        scratch,
        response_out,
        effective_mass_out,
    )?;

    for (contact_index, contact) in contacts.iter().enumerate() {
        let pose = scratch.model.world_from_body[contact.frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(contact.point_world))
            .coords;
        model
            .floating_point_jacobian_into(
                &scratch.model,
                contact.frame,
                point_in_frame,
                &mut scratch.point_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        for axis in 0..CONTACT_TRANSITION_IMPULSE_WIDTH {
            let row = contact_index * CONTACT_TRANSITION_IMPULSE_WIDTH + axis;
            for column in 0..=row {
                let value = (0..generalized_dof)
                    .map(|coordinate| {
                        let axis_jacobian = (0..3)
                            .map(|world_axis| {
                                contact.basis_world[axis][world_axis]
                                    * scratch.point_jacobian[(world_axis, coordinate)]
                            })
                            .sum::<f64>();
                        axis_jacobian * response_out[coordinate * contact_axes + column]
                    })
                    .sum::<f64>();
                if !value.is_finite() {
                    return Err(ContactTransitionResponseError::Model);
                }
                delassus_out[row * contact_axes + column] = value;
                delassus_out[column * contact_axes + row] = value;
            }
        }
    }
    Ok(())
}

/// Derive a spatial-wrench impulse response and its coupled Delassus operator.
///
/// `response_out` is row-major `[generalized_dof, wrench, 6]` and
/// `delassus_out` is `[wrench * 6, wrench * 6]`. Each six-axis block is ordered
/// `[moment XYZ about the declared reference point; force XYZ through it]` in
/// the declared basis. Construction owns all storage; this query performs no
/// runtime allocation.
pub fn write_spatial_impulse_velocity_response(
    model: &CompiledModel,
    state: &RobotState,
    wrenches: &[SpatialImpulseResponseSpec],
    scratch: &mut ContactTransitionResponseScratch,
    response_out: &mut [f64],
    delassus_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    let wrench_axes = wrenches
        .len()
        .checked_mul(SPATIAL_IMPULSE_WIDTH)
        .ok_or(ContactTransitionResponseError::Dimension)?;
    if wrenches.is_empty()
        || response_out.len()
            != generalized_dof
                .checked_mul(wrench_axes)
                .ok_or(ContactTransitionResponseError::Dimension)?
        || delassus_out.len()
            != wrench_axes
                .checked_mul(wrench_axes)
                .ok_or(ContactTransitionResponseError::Dimension)?
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.point_jacobian.shape() != (3, generalized_dof)
        || scratch.angular_jacobian.shape() != (3, generalized_dof)
        || scratch.rhs.len() != generalized_dof
        || scratch.solution.len() != generalized_dof
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    let valid_wrench = |wrench: &SpatialImpulseResponseSpec| {
        if wrench.frame.0 >= model.bodies.len()
            || wrench
                .reference_point_world
                .iter()
                .any(|value| !value.is_finite())
            || wrench
                .basis_world
                .iter()
                .flat_map(|axis| axis.iter())
                .any(|value| !value.is_finite())
        {
            return false;
        }
        for axis in 0..3 {
            if (wrench.basis_world[axis].norm_squared() - 1.0).abs() > 1.0e-9 {
                return false;
            }
            for other in 0..axis {
                if wrench.basis_world[axis]
                    .dot(&wrench.basis_world[other])
                    .abs()
                    > 1.0e-9
                {
                    return false;
                }
            }
        }
        wrench.basis_world[0]
            .cross(&wrench.basis_world[1])
            .dot(&wrench.basis_world[2])
            > 1.0 - 1.0e-9
    };
    if !wrenches.iter().all(valid_wrench) {
        return Err(ContactTransitionResponseError::InvalidContact);
    }

    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    if !factor_spd_lower(&mut scratch.mass_factor) {
        return Err(ContactTransitionResponseError::SingularMassMatrix);
    }

    for (wrench_index, wrench) in wrenches.iter().enumerate() {
        let pose = scratch.model.world_from_body[wrench.frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(wrench.reference_point_world))
            .coords;
        model
            .floating_point_jacobian_into(
                &scratch.model,
                wrench.frame,
                point_in_frame,
                &mut scratch.point_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        model
            .floating_angular_jacobian_into(
                &scratch.model,
                wrench.frame,
                &mut scratch.angular_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        for spatial_axis in 0..SPATIAL_IMPULSE_WIDTH {
            let (jacobian, basis_axis) = if spatial_axis < 3 {
                (&scratch.angular_jacobian, spatial_axis)
            } else {
                (&scratch.point_jacobian, spatial_axis - 3)
            };
            for coordinate in 0..generalized_dof {
                scratch.rhs[coordinate] = (0..3)
                    .map(|row| jacobian[(row, coordinate)] * wrench.basis_world[basis_axis][row])
                    .sum();
            }
            if !solve_spd_factor_into(&scratch.mass_factor, &scratch.rhs, &mut scratch.solution) {
                return Err(ContactTransitionResponseError::SingularMassMatrix);
            }
            let column = wrench_index * SPATIAL_IMPULSE_WIDTH + spatial_axis;
            for coordinate in 0..generalized_dof {
                response_out[coordinate * wrench_axes + column] = scratch.solution[coordinate];
            }
        }
    }

    for (wrench_index, wrench) in wrenches.iter().enumerate() {
        let pose = scratch.model.world_from_body[wrench.frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(wrench.reference_point_world))
            .coords;
        model
            .floating_point_jacobian_into(
                &scratch.model,
                wrench.frame,
                point_in_frame,
                &mut scratch.point_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        model
            .floating_angular_jacobian_into(
                &scratch.model,
                wrench.frame,
                &mut scratch.angular_jacobian,
            )
            .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
        for spatial_axis in 0..SPATIAL_IMPULSE_WIDTH {
            let row = wrench_index * SPATIAL_IMPULSE_WIDTH + spatial_axis;
            let (jacobian, basis_axis) = if spatial_axis < 3 {
                (&scratch.angular_jacobian, spatial_axis)
            } else {
                (&scratch.point_jacobian, spatial_axis - 3)
            };
            for column in 0..=row {
                let value = (0..generalized_dof)
                    .map(|coordinate| {
                        let projected = (0..3)
                            .map(|world_axis| {
                                wrench.basis_world[basis_axis][world_axis]
                                    * jacobian[(world_axis, coordinate)]
                            })
                            .sum::<f64>();
                        projected * response_out[coordinate * wrench_axes + column]
                    })
                    .sum::<f64>();
                if !value.is_finite() {
                    return Err(ContactTransitionResponseError::Model);
                }
                delassus_out[row * wrench_axes + column] = value;
                delassus_out[column * wrench_axes + row] = value;
            }
        }
    }
    Ok(())
}

/// Write generalized momentum-impulse residuals for candidate velocity jumps.
///
/// Each row is `M(q) * (observed_delta_velocity - predicted_delta_velocity)`
/// in the floating tangent `[root angular; root linear; joints]`. This is an
/// independent diagnostic covector: it does not widen a contact response or
/// grant authority. `predicted_delta_velocity` and `residual_out` are
/// row-major `[candidate, generalized_dof]`. Runtime work is allocation-free.
pub fn write_generalized_momentum_impulse_residuals(
    model: &CompiledModel,
    state: &RobotState,
    observed_delta_velocity: &[f64],
    predicted_delta_velocity: &[f64],
    scratch: &mut ContactTransitionResponseScratch,
    residual_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    if observed_delta_velocity.len() != generalized_dof
        || predicted_delta_velocity.is_empty()
        || !predicted_delta_velocity
            .len()
            .is_multiple_of(generalized_dof)
        || residual_out.len() != predicted_delta_velocity.len()
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || observed_delta_velocity
            .iter()
            .chain(predicted_delta_velocity)
            .any(|value| !value.is_finite())
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    let candidates = predicted_delta_velocity.len() / generalized_dof;
    for candidate in 0..candidates {
        let offset = candidate * generalized_dof;
        for row in 0..generalized_dof {
            residual_out[offset + row] = (0..generalized_dof)
                .map(|column| {
                    scratch.mass_factor[(row, column)]
                        * (observed_delta_velocity[column]
                            - predicted_delta_velocity[offset + column])
                })
                .sum();
        }
    }
    if residual_out.iter().any(|value| !value.is_finite()) {
        return Err(ContactTransitionResponseError::Model);
    }
    Ok(())
}

/// Map a componentwise generalized-momentum impulse box through `M(q)^-1`.
///
/// Momentum is a covector in `[root moment; root impulse; joint impulse]` and
/// output is the matching generalized-velocity tangent. The full inverse mass
/// coupling is retained: this is not a coordinatewise division by inertia.
/// The caller owns bounds and output storage; runtime work is allocation-free.
pub fn write_generalized_velocity_interval_from_momentum_box(
    model: &CompiledModel,
    state: &RobotState,
    momentum_lower: &[f64],
    momentum_upper: &[f64],
    scratch: &mut ContactTransitionResponseScratch,
    velocity_lower_out: &mut [f64],
    velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    if momentum_lower.len() != generalized_dof
        || momentum_upper.len() != generalized_dof
        || velocity_lower_out.len() != generalized_dof
        || velocity_upper_out.len() != generalized_dof
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.rhs.len() != generalized_dof
        || scratch.solution.len() != generalized_dof
        || momentum_lower
            .iter()
            .chain(momentum_upper)
            .any(|value| !value.is_finite())
        || momentum_lower
            .iter()
            .zip(momentum_upper)
            .any(|(lower, upper)| lower > upper)
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    if !factor_spd_lower(&mut scratch.mass_factor) {
        return Err(ContactTransitionResponseError::SingularMassMatrix);
    }

    velocity_lower_out.fill(0.0);
    velocity_upper_out.fill(0.0);
    for momentum_coordinate in 0..generalized_dof {
        scratch.rhs.fill(0.0);
        scratch.rhs[momentum_coordinate] = 1.0;
        if !solve_spd_factor_into(&scratch.mass_factor, &scratch.rhs, &mut scratch.solution) {
            return Err(ContactTransitionResponseError::SingularMassMatrix);
        }
        for velocity_coordinate in 0..generalized_dof {
            let coefficient = scratch.solution[velocity_coordinate];
            let endpoint_a = coefficient * momentum_lower[momentum_coordinate];
            let endpoint_b = coefficient * momentum_upper[momentum_coordinate];
            velocity_lower_out[velocity_coordinate] += endpoint_a.min(endpoint_b);
            velocity_upper_out[velocity_coordinate] += endpoint_a.max(endpoint_b);
        }
    }
    if velocity_lower_out
        .iter()
        .chain(velocity_upper_out.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ContactTransitionResponseError::Model);
    }
    Ok(())
}

/// Bound each generalized-velocity component induced by a zero-centered
/// generalized-impulse ellipsoid in the exact kinetic metric.
///
/// The declared set is `p^T M(q)^-1 p <= twice_kinetic_energy_upper_j`.
/// Its exact support in velocity coordinate `i`, after `delta_v = M^-1 p`, is
/// `sqrt(twice_kinetic_energy_upper_j * (M^-1)_ii)`. Unlike a coordinate box,
/// this representation does not add the absolute value of every coupled
/// inverse-mass column. Outputs are symmetric, caller-owned, and runtime work
/// is allocation-free.
pub fn write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
    model: &CompiledModel,
    state: &RobotState,
    twice_kinetic_energy_upper_j: f64,
    scratch: &mut ContactTransitionResponseScratch,
    velocity_lower_out: &mut [f64],
    velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    if !twice_kinetic_energy_upper_j.is_finite()
        || twice_kinetic_energy_upper_j < 0.0
        || velocity_lower_out.len() != generalized_dof
        || velocity_upper_out.len() != generalized_dof
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.rhs.len() != generalized_dof
        || scratch.solution.len() != generalized_dof
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    if !factor_spd_lower(&mut scratch.mass_factor) {
        return Err(ContactTransitionResponseError::SingularMassMatrix);
    }
    for coordinate in 0..generalized_dof {
        scratch.rhs.fill(0.0);
        scratch.rhs[coordinate] = 1.0;
        if !solve_spd_factor_into(&scratch.mass_factor, &scratch.rhs, &mut scratch.solution) {
            return Err(ContactTransitionResponseError::SingularMassMatrix);
        }
        let inverse_mass_diagonal = scratch.solution[coordinate];
        if !inverse_mass_diagonal.is_finite() || inverse_mass_diagonal <= 0.0 {
            return Err(ContactTransitionResponseError::SingularMassMatrix);
        }
        let radius = (twice_kinetic_energy_upper_j * inverse_mass_diagonal).sqrt();
        velocity_lower_out[coordinate] = -radius;
        velocity_upper_out[coordinate] = radius;
    }
    Ok(())
}

/// Bound the Minkowski sum of independently budgeted root and articulated
/// generalized-impulse ellipsoids in the exact kinetic metric.
///
/// Momentum coordinates are partitioned as root `[0, 6)` and articulated
/// `[6, n)`. For partition `P`, the declared set is
/// `p_P^T (M^-1)_PP p_P <= twice_energy_P`, with every other momentum
/// coordinate zero. The exact support of velocity coordinate `i` is
/// `sqrt(E_P * A_iP A_PP^-1 A_Pi)`, where `A=M^-1`. Independent partition
/// supports add. This keeps root disturbance and articulated residual budgets
/// separately visible without replacing either with a coordinate box.
/// Caller-owned outputs and construction-owned scratch make runtime work
/// allocation-free.
pub fn write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
    model: &CompiledModel,
    state: &RobotState,
    root_twice_kinetic_energy_upper_j: f64,
    articulated_twice_kinetic_energy_upper_j: f64,
    scratch: &mut ContactTransitionResponseScratch,
    velocity_lower_out: &mut [f64],
    velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionResponseError> {
    let generalized_dof = model.dof + 6;
    if !root_twice_kinetic_energy_upper_j.is_finite()
        || root_twice_kinetic_energy_upper_j < 0.0
        || !articulated_twice_kinetic_energy_upper_j.is_finite()
        || articulated_twice_kinetic_energy_upper_j < 0.0
        || velocity_lower_out.len() != generalized_dof
        || velocity_upper_out.len() != generalized_dof
        || scratch.mass_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.inverse_mass.shape() != (generalized_dof, generalized_dof)
        || scratch.partition_factor.shape() != (generalized_dof, generalized_dof)
        || scratch.rhs.len() != generalized_dof
        || scratch.solution.len() != generalized_dof
        || scratch.partition_rhs.len() != generalized_dof
        || scratch.partition_solution.len() != generalized_dof
        || scratch.partition_radius.len() != generalized_dof
    {
        return Err(ContactTransitionResponseError::Dimension);
    }
    model
        .forward_kinematics(state, &mut scratch.model)
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    model
        .floating_mass_matrix_into(
            &scratch.model,
            &mut scratch.dynamics,
            &mut scratch.mass_factor,
        )
        .map_err(|_: ModelError| ContactTransitionResponseError::Model)?;
    if !factor_spd_lower(&mut scratch.mass_factor) {
        return Err(ContactTransitionResponseError::SingularMassMatrix);
    }
    for column in 0..generalized_dof {
        scratch.rhs.fill(0.0);
        scratch.rhs[column] = 1.0;
        if !solve_spd_factor_into(&scratch.mass_factor, &scratch.rhs, &mut scratch.solution) {
            return Err(ContactTransitionResponseError::SingularMassMatrix);
        }
        for row in 0..generalized_dof {
            scratch.inverse_mass[(row, column)] = scratch.solution[row];
        }
    }
    if scratch.inverse_mass.iter().any(|value| !value.is_finite()) {
        return Err(ContactTransitionResponseError::Model);
    }

    scratch.partition_radius.fill(0.0);
    for (offset, size, twice_energy) in [
        (0, 6, root_twice_kinetic_energy_upper_j),
        (6, model.dof, articulated_twice_kinetic_energy_upper_j),
    ] {
        if size == 0 || twice_energy == 0.0 {
            continue;
        }
        for row in 0..size {
            for column in 0..size {
                scratch.partition_factor[(row, column)] =
                    scratch.inverse_mass[(offset + row, offset + column)];
            }
        }
        if !factor_spd_lower_prefix(&mut scratch.partition_factor, size) {
            return Err(ContactTransitionResponseError::SingularMassMatrix);
        }
        for coordinate in 0..generalized_dof {
            for partition_coordinate in 0..size {
                scratch.partition_rhs[partition_coordinate] =
                    scratch.inverse_mass[(coordinate, offset + partition_coordinate)];
            }
            if !solve_spd_factor_prefix_into(
                &scratch.partition_factor,
                &scratch.partition_rhs,
                &mut scratch.partition_solution,
                size,
            ) {
                return Err(ContactTransitionResponseError::SingularMassMatrix);
            }
            let support_squared_per_joule = (0..size)
                .map(|partition_coordinate| {
                    scratch.partition_rhs[partition_coordinate]
                        * scratch.partition_solution[partition_coordinate]
                })
                .sum::<f64>();
            if !support_squared_per_joule.is_finite() || support_squared_per_joule < -1.0e-12 {
                return Err(ContactTransitionResponseError::Model);
            }
            let radius = (twice_energy * support_squared_per_joule.max(0.0)).sqrt();
            if !radius.is_finite() {
                return Err(ContactTransitionResponseError::Model);
            }
            scratch.partition_radius[coordinate] += radius;
        }
    }
    for coordinate in 0..generalized_dof {
        velocity_lower_out[coordinate] = -scratch.partition_radius[coordinate];
        velocity_upper_out[coordinate] = scratch.partition_radius[coordinate];
    }
    Ok(())
}

/// Fixed-work coupled rigid-contact impulse input.
#[derive(Clone, Copy, Debug)]
pub struct CoupledContactImpulseInput<'a> {
    /// Signed pre-impulse contact velocity, contact-major tangent-X,
    /// tangent-Y, normal. Negative normal velocity is closing.
    pub contact_velocity: &'a [f64],
    /// Row-major `J M^-1 J^T` in the same axis ordering.
    pub delassus: &'a [f64],
    /// Per-axis absolute impulse caps. Normal caps must be nonnegative.
    pub impulse_upper: &'a [f64],
    /// Per-contact friction coefficients.
    pub friction: &'a [f64],
    /// Shared coefficient of restitution in `[0, 1]`.
    pub restitution: f64,
    /// Dimensionless diagonal compliance ratio. The projected update uses
    /// `W_ii * (1 + ratio)` while the returned contact velocity is evaluated
    /// through the unmodified physical Delassus operator.
    pub diagonal_regularization_ratio: f64,
    /// Number of deterministic forward-and-reverse projected sweeps.
    pub sweeps: usize,
}

/// A finite, explicitly enumerated contact-estimator uncertainty set.
///
/// Hypotheses are packed hypothesis-major. Each one owns contact velocity,
/// impulse caps, friction, restitution, and regularization while sharing the
/// state-local Delassus operator and generalized impulse response. This type
/// makes the estimator boundary explicit: the function below envelopes only
/// the declared finite set and does not silently claim continuous uncertainty
/// between hypotheses.
#[derive(Clone, Copy, Debug)]
pub struct CoupledContactHypothesisEnvelopeInput<'a> {
    pub hypothesis_count: usize,
    pub contact_velocity_hypotheses: &'a [f64],
    pub delassus: &'a [f64],
    pub impulse_upper_hypotheses: &'a [f64],
    pub friction_hypotheses: &'a [f64],
    pub restitution_hypotheses: &'a [f64],
    pub diagonal_regularization_ratio_hypotheses: &'a [f64],
    pub impulse_velocity_response: &'a [f64],
    pub sweeps: usize,
}

/// Fixed-substep compliant contact evolution input.
///
/// Gap is positive while separated. Normal stiffness and damping are explicit
/// per-contact model parameters; this function does not infer them from a
/// simulator name or material label.
#[derive(Clone, Copy, Debug)]
pub struct CompliantContactImpulseInput<'a> {
    pub contact_gap: &'a [f64],
    pub contact_velocity: &'a [f64],
    pub delassus: &'a [f64],
    pub impulse_upper: &'a [f64],
    pub friction: &'a [f64],
    pub normal_stiffness: &'a [f64],
    pub normal_damping: &'a [f64],
    pub time_step_s: f64,
    pub substeps: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompliantFrictionCone {
    /// Euclidean tangent disk, matching an elliptic two-axis section.
    Circular,
    /// L1 tangent diamond, matching a pyramidal two-axis section.
    Pyramidal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompliantStepIntegrator {
    /// Explicit tangent decay and pre-impulse gap update.
    ExplicitEuler,
    /// Implicit tangent decay and post-impulse gap update.
    ImplicitEuler,
    /// Exact scalar tangent decay and trapezoidal gap update.
    ExponentialTrapezoidal,
    /// Four-stage generalized Runge--Kutta state/contact evolution.
    ///
    /// This integrator is only valid for the model-coupled solver. The
    /// scalar/contact-space solvers retain [`Self::ExponentialTrapezoidal`]
    /// as their exact tangent-decay/trapezoidal-gap scheme and reject this
    /// model-owned variant.
    GeneralizedRk4,
}

/// Positive time-constant/damping-ratio compliant-contact law.
///
/// The law follows the documented constraint reference parameterization
/// `b = 2 / (d_width τ)`, `k = d(r) / (d_width² τ² ζ²)`, and
/// `a_c + d(r) (b v + k r) = (1 - d(r)) a_free`. Therefore the acceleration
/// contributed by the constraint is `d(r) (-b v - k r - a_free)`; in
/// particular, effective spring stiffness carries two impedance factors.
/// `minimum_time_constant_s` is an explicit caller-owned integration-safety
/// clamp rather than a hidden simulator assumption. Friction uses the
/// zero-residual impedance `d_min` and the selected cone section.
#[derive(Clone, Copy, Debug)]
pub struct PositiveReferenceCompliantContactImpulseInput<'a> {
    pub contact_gap: &'a [f64],
    pub contact_velocity: &'a [f64],
    /// Free (non-contact) point acceleration in the same contact bases.
    pub contact_free_acceleration: &'a [f64],
    pub delassus: &'a [f64],
    pub impulse_upper: &'a [f64],
    pub friction: &'a [f64],
    pub effective_normal_mass: &'a [f64],
    pub time_constant_s: &'a [f64],
    pub damping_ratio: &'a [f64],
    pub impedance_min: &'a [f64],
    pub impedance_max: &'a [f64],
    pub impedance_width_m: &'a [f64],
    pub impedance_midpoint: &'a [f64],
    pub impedance_power: &'a [f64],
    pub minimum_time_constant_s: f64,
    pub time_step_s: f64,
    pub substeps: usize,
    pub friction_cone: CompliantFrictionCone,
    pub integrator: CompliantStepIntegrator,
}

/// Positive-reference contact law distributed through the complete Delassus
/// operator at every microstep.
///
/// Unlike [`PositiveReferenceCompliantContactImpulseInput`], this input does
/// not accept independently estimated effective masses. The full state-local
/// `J M^-1 J^T` operator owns both diagonal response and cross-contact load
/// distribution. Fixed forward/reverse projected sweeps solve each compliant
/// velocity increment over the declared friction section and cumulative
/// impulse caps.
#[derive(Clone, Copy, Debug)]
pub struct CoupledPositiveReferenceCompliantContactImpulseInput<'a> {
    pub contact_gap: &'a [f64],
    pub contact_velocity: &'a [f64],
    pub contact_free_acceleration: &'a [f64],
    pub delassus: &'a [f64],
    pub impulse_upper: &'a [f64],
    pub friction: &'a [f64],
    pub time_constant_s: &'a [f64],
    pub damping_ratio: &'a [f64],
    pub impedance_min: &'a [f64],
    pub impedance_max: &'a [f64],
    pub impedance_width_m: &'a [f64],
    pub impedance_midpoint: &'a [f64],
    pub impedance_power: &'a [f64],
    pub minimum_time_constant_s: f64,
    pub time_step_s: f64,
    pub substeps: usize,
    pub projection_sweeps: usize,
    pub friction_cone: CompliantFrictionCone,
    pub integrator: CompliantStepIntegrator,
}

/// Model-owned, state-refreshing positive-reference contact evolution.
///
/// `contacts` describe rigid material points at the initial state. The solver
/// converts them to frame-local coordinates once, advances the generalized
/// state over `state_steps`, and refreshes point geometry, velocity, Jacobian,
/// inverse mass, and the complete Delassus operator before every later event
/// test. `compliance_substeps` and `projection_sweeps` are bounded inner work;
/// neither changes the independently declared state/event clock.
#[derive(Clone, Copy, Debug)]
pub struct ModelCoupledPositiveReferenceCompliantContactImpulseInput<'a> {
    pub initial_state: &'a FloatingRobotState,
    pub contacts: &'a [PointImpulseResponseSpec],
    /// Radius of a spherical support surface at each point. Zero preserves a
    /// rigid material point. A positive radius keeps the plane support point
    /// at `center - normal * radius` as the body rotates.
    pub contact_surface_radius_m: &'a [f64],
    pub plane_normal_world: crate::math::Vec3,
    pub plane_offset_m: f64,
    pub generalized_free_acceleration: &'a [f64],
    /// Initial free point acceleration in contact-major local axes. The
    /// difference from the model-computed `J qdd + Jdot v` is retained as a
    /// causal cross-model correction while state-local kinematics are
    /// refreshed.
    pub initial_contact_free_acceleration: &'a [f64],
    pub impulse_upper: &'a [f64],
    pub friction: &'a [f64],
    pub time_constant_s: &'a [f64],
    pub damping_ratio: &'a [f64],
    pub impedance_min: &'a [f64],
    pub impedance_max: &'a [f64],
    pub impedance_width_m: &'a [f64],
    pub impedance_midpoint: &'a [f64],
    pub impedance_power: &'a [f64],
    pub minimum_time_constant_s: f64,
    pub time_step_s: f64,
    pub state_steps: usize,
    pub compliance_substeps: usize,
    pub projection_sweeps: usize,
    pub friction_cone: CompliantFrictionCone,
    /// `ExplicitEuler`, `ImplicitEuler`, and `ExponentialTrapezoidal` keep
    /// their legacy single-event behavior; `GeneralizedRk4` enables the
    /// four-stage state/contact evolution below.
    pub integrator: CompliantStepIntegrator,
}

/// Preallocated workspace for model-owned contact event/state evolution.
#[derive(Clone, Debug)]
pub struct ModelCoupledPositiveReferenceContactScratch {
    state: FloatingRobotState,
    rk4_base_state: FloatingRobotState,
    response_scratch: ContactTransitionResponseScratch,
    local_points: Vec<crate::math::Vec3>,
    contact_surface_radius_m: Vec<f64>,
    live_contacts: Vec<PointImpulseResponseSpec>,
    response: Vec<f64>,
    effective_mass: Vec<f64>,
    delassus: Vec<f64>,
    contact_gap: Vec<f64>,
    contact_velocity: Vec<f64>,
    contact_free_acceleration: Vec<f64>,
    contact_acceleration_bias: Vec<f64>,
    remaining_impulse_upper: Vec<f64>,
    desired_velocity_delta: Vec<f64>,
    step_impulse: Vec<f64>,
    outer_impulse: Vec<f64>,
    step_velocity_after: Vec<f64>,
    step_gap_after: Vec<f64>,
    total_impulse: Vec<f64>,
    generalized_velocity_delta: DVector<f64>,
    generalized_free_acceleration: DVector<f64>,
    held_generalized_force: DVector<f64>,
    zero_generalized_acceleration: DVector<f64>,
    rk4_stage_velocity: [DVector<f64>; 4],
    rk4_stage_acceleration: [DVector<f64>; 4],
    rk4_stage_impulse: [DVector<f64>; 4],
}

impl ModelCoupledPositiveReferenceContactScratch {
    pub fn new(model: &CompiledModel, contact_count: usize) -> Self {
        let generalized_dof = model.dof + 6;
        let axes = contact_count * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let identity_basis = [
            crate::math::Vec3::x(),
            crate::math::Vec3::y(),
            crate::math::Vec3::z(),
        ];
        Self {
            state: FloatingRobotState::zeros(model),
            rk4_base_state: FloatingRobotState::zeros(model),
            response_scratch: ContactTransitionResponseScratch::new(model),
            local_points: vec![crate::math::Vec3::zeros(); contact_count],
            contact_surface_radius_m: vec![0.0; contact_count],
            live_contacts: vec![
                PointImpulseResponseSpec {
                    frame: FrameId(0),
                    point_world: crate::math::Vec3::zeros(),
                    basis_world: identity_basis,
                };
                contact_count
            ],
            response: vec![0.0; generalized_dof * axes],
            effective_mass: vec![0.0; axes],
            delassus: vec![0.0; axes * axes],
            contact_gap: vec![0.0; contact_count],
            contact_velocity: vec![0.0; axes],
            contact_free_acceleration: vec![0.0; axes],
            contact_acceleration_bias: vec![0.0; axes],
            remaining_impulse_upper: vec![0.0; axes],
            desired_velocity_delta: vec![0.0; axes],
            step_impulse: vec![0.0; axes],
            outer_impulse: vec![0.0; axes],
            step_velocity_after: vec![0.0; axes],
            step_gap_after: vec![0.0; contact_count],
            total_impulse: vec![0.0; axes],
            generalized_velocity_delta: DVector::zeros(generalized_dof),
            generalized_free_acceleration: DVector::zeros(generalized_dof),
            held_generalized_force: DVector::zeros(generalized_dof),
            zero_generalized_acceleration: DVector::zeros(generalized_dof),
            rk4_stage_velocity: std::array::from_fn(|_| DVector::zeros(generalized_dof)),
            rk4_stage_acceleration: std::array::from_fn(|_| DVector::zeros(generalized_dof)),
            rk4_stage_impulse: std::array::from_fn(|_| DVector::zeros(axes)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelCoupledPositiveReferenceContactError {
    Dimension,
    InvalidConfig,
    InvalidWitness,
    Response(ContactTransitionResponseError),
    Contact(CoupledContactImpulseError),
    Model,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoupledContactImpulseError {
    InvalidConfig,
    Dimension,
    InvalidWitness,
    InvalidDelassus,
}

fn validate_coupled_contact_impulse(
    input: CoupledContactImpulseInput<'_>,
    impulse_len: usize,
    after_len: usize,
) -> Result<(), CoupledContactImpulseError> {
    let axes = input.contact_velocity.len();
    if axes == 0 || !axes.is_multiple_of(CONTACT_TRANSITION_IMPULSE_WIDTH) {
        return Err(CoupledContactImpulseError::Dimension);
    }
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;
    if input.impulse_upper.len() != axes
        || input.friction.len() != contacts
        || impulse_len != axes
        || after_len != axes
        || input.delassus.len() != axes.saturating_mul(axes)
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    if !input.restitution.is_finite()
        || !(0.0..=1.0).contains(&input.restitution)
        || !input.diagonal_regularization_ratio.is_finite()
        || !(0.0..=1.0e6).contains(&input.diagonal_regularization_ratio)
        || input.sweeps == 0
        || input.sweeps > 256
    {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    if input
        .contact_velocity
        .iter()
        .any(|value| !value.is_finite())
        || input
            .impulse_upper
            .iter()
            .chain(input.friction)
            .any(|value| !nonnegative_finite(*value))
    {
        return Err(CoupledContactImpulseError::InvalidWitness);
    }
    for row in 0..axes {
        let diagonal = input.delassus[row * axes + row];
        if !diagonal.is_finite() || diagonal <= 1.0e-18 {
            return Err(CoupledContactImpulseError::InvalidDelassus);
        }
        for column in 0..axes {
            let value = input.delassus[row * axes + column];
            let transpose = input.delassus[column * axes + row];
            let scale = 1.0_f64.max(value.abs()).max(transpose.abs());
            if !value.is_finite() || (value - transpose).abs() > 1.0e-9 * scale {
                return Err(CoupledContactImpulseError::InvalidDelassus);
            }
        }
    }
    Ok(())
}

fn coupled_contact_velocity(
    contact_velocity: &[f64],
    delassus: &[f64],
    impulse: &[f64],
    row: usize,
) -> f64 {
    let axes = impulse.len();
    contact_velocity[row]
        + (0..axes)
            .map(|column| delassus[row * axes + column] * impulse[column])
            .sum::<f64>()
}

fn update_coupled_contact(
    input: CoupledContactImpulseInput<'_>,
    contact: usize,
    impulse_out: &mut [f64],
) {
    let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
    let normal_target = if input.contact_velocity[normal] < 0.0 {
        -input.restitution * input.contact_velocity[normal]
    } else {
        0.0
    };
    let normal_diagonal = input.delassus[normal * impulse_out.len() + normal];
    let normal_velocity =
        coupled_contact_velocity(input.contact_velocity, input.delassus, impulse_out, normal)
            + input.diagonal_regularization_ratio * normal_diagonal * impulse_out[normal];
    impulse_out[normal] = (impulse_out[normal]
        + (normal_target - normal_velocity)
            / (normal_diagonal * (1.0 + input.diagonal_regularization_ratio)))
        .clamp(0.0, input.impulse_upper[normal]);

    let friction_radius = input.friction[contact] * impulse_out[normal];
    for tangent in 0..2 {
        let axis = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + tangent;
        let diagonal = input.delassus[axis * impulse_out.len() + axis];
        let velocity =
            coupled_contact_velocity(input.contact_velocity, input.delassus, impulse_out, axis)
                + input.diagonal_regularization_ratio * diagonal * impulse_out[axis];
        let cap = input.impulse_upper[axis].min(friction_radius);
        impulse_out[axis] = (impulse_out[axis]
            - velocity / (diagonal * (1.0 + input.diagonal_regularization_ratio)))
            .clamp(-cap, cap);
    }
    let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
    let tangent_y = tangent_x + 1;
    let tangent_norm = impulse_out[tangent_x].hypot(impulse_out[tangent_y]);
    if tangent_norm > friction_radius && tangent_norm > 0.0 {
        let scale = friction_radius / tangent_norm;
        impulse_out[tangent_x] *= scale;
        impulse_out[tangent_y] *= scale;
    }
}

/// Solve one coupled passive impulse with deterministic projected sweeps.
///
/// This is a model witness, not an outer bound or complementarity
/// certificate. The full Delassus matrix couples both contacts; projection
/// enforces nonnegative bounded normal impulse, per-axis passive caps, and a
/// circular Coulomb section. All inputs are validated before outputs change.
/// Runtime work is `O(sweeps * contacts²)` and allocation-free.
pub fn solve_coupled_contact_impulse(
    input: CoupledContactImpulseInput<'_>,
    impulse_out: &mut [f64],
    contact_velocity_after_out: &mut [f64],
) -> Result<(), CoupledContactImpulseError> {
    let axes = input.contact_velocity.len();
    validate_coupled_contact_impulse(input, impulse_out.len(), contact_velocity_after_out.len())?;
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;

    impulse_out.fill(0.0);
    for _ in 0..input.sweeps {
        for contact in 0..contacts {
            update_coupled_contact(input, contact, impulse_out);
        }
        for contact in (0..contacts).rev() {
            update_coupled_contact(input, contact, impulse_out);
        }
    }
    for row in 0..axes {
        contact_velocity_after_out[row] =
            coupled_contact_velocity(input.contact_velocity, input.delassus, impulse_out, row);
    }
    Ok(())
}

/// Envelope generalized velocity jumps over a declared finite contact set.
///
/// `impulse_scratch`, `contact_velocity_after_scratch`, and
/// `generalized_delta_scratch` are caller-owned. Every hypothesis is validated
/// before either output changes, so a malformed late hypothesis is atomic.
/// Runtime work is fixed by the declared hypothesis count and sweep count and
/// performs no allocation.
pub fn write_coupled_contact_hypothesis_velocity_envelope(
    input: CoupledContactHypothesisEnvelopeInput<'_>,
    impulse_scratch: &mut [f64],
    contact_velocity_after_scratch: &mut [f64],
    generalized_delta_scratch: &mut [f64],
    generalized_velocity_lower_out: &mut [f64],
    generalized_velocity_upper_out: &mut [f64],
) -> Result<(), CoupledContactImpulseError> {
    let hypotheses = input.hypothesis_count;
    let axes = impulse_scratch.len();
    if hypotheses == 0
        || axes == 0
        || !axes.is_multiple_of(CONTACT_TRANSITION_IMPULSE_WIDTH)
        || contact_velocity_after_scratch.len() != axes
        || generalized_delta_scratch.is_empty()
        || generalized_velocity_lower_out.len() != generalized_delta_scratch.len()
        || generalized_velocity_upper_out.len() != generalized_delta_scratch.len()
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;
    let generalized_dof = generalized_delta_scratch.len();
    if input.contact_velocity_hypotheses.len() != hypotheses.saturating_mul(axes)
        || input.impulse_upper_hypotheses.len() != hypotheses.saturating_mul(axes)
        || input.friction_hypotheses.len() != hypotheses.saturating_mul(contacts)
        || input.restitution_hypotheses.len() != hypotheses
        || input.diagonal_regularization_ratio_hypotheses.len() != hypotheses
        || input.impulse_velocity_response.len() != generalized_dof.saturating_mul(axes)
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    if input
        .impulse_velocity_response
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(CoupledContactImpulseError::InvalidWitness);
    }

    for hypothesis in 0..hypotheses {
        let axis_start = hypothesis * axes;
        let contact_start = hypothesis * contacts;
        validate_coupled_contact_impulse(
            CoupledContactImpulseInput {
                contact_velocity: &input.contact_velocity_hypotheses[axis_start..axis_start + axes],
                delassus: input.delassus,
                impulse_upper: &input.impulse_upper_hypotheses[axis_start..axis_start + axes],
                friction: &input.friction_hypotheses[contact_start..contact_start + contacts],
                restitution: input.restitution_hypotheses[hypothesis],
                diagonal_regularization_ratio: input.diagonal_regularization_ratio_hypotheses
                    [hypothesis],
                sweeps: input.sweeps,
            },
            axes,
            axes,
        )?;
    }

    generalized_velocity_lower_out.fill(f64::INFINITY);
    generalized_velocity_upper_out.fill(f64::NEG_INFINITY);
    for hypothesis in 0..hypotheses {
        let axis_start = hypothesis * axes;
        let contact_start = hypothesis * contacts;
        solve_coupled_contact_impulse(
            CoupledContactImpulseInput {
                contact_velocity: &input.contact_velocity_hypotheses[axis_start..axis_start + axes],
                delassus: input.delassus,
                impulse_upper: &input.impulse_upper_hypotheses[axis_start..axis_start + axes],
                friction: &input.friction_hypotheses[contact_start..contact_start + contacts],
                restitution: input.restitution_hypotheses[hypothesis],
                diagonal_regularization_ratio: input.diagonal_regularization_ratio_hypotheses
                    [hypothesis],
                sweeps: input.sweeps,
            },
            impulse_scratch,
            contact_velocity_after_scratch,
        )?;
        for coordinate in 0..generalized_dof {
            let response =
                &input.impulse_velocity_response[coordinate * axes..(coordinate + 1) * axes];
            let delta = response
                .iter()
                .zip(impulse_scratch.iter())
                .map(|(coefficient, impulse)| coefficient * impulse)
                .sum::<f64>();
            generalized_delta_scratch[coordinate] = delta;
            generalized_velocity_lower_out[coordinate] =
                generalized_velocity_lower_out[coordinate].min(delta);
            generalized_velocity_upper_out[coordinate] =
                generalized_velocity_upper_out[coordinate].max(delta);
        }
    }
    Ok(())
}

/// Integrate one explicit compliant contact model over fixed substeps.
///
/// Each substep predicts penetration from the current signed gap and normal
/// velocity, applies a Kelvin–Voigt normal impulse, projects the tangent impulse
/// to the current circular Coulomb disk, then updates the fully coupled contact
/// velocity through `J M^-1 J^T`. Gap advances with the post-impulse normal
/// velocity. This is a deterministic model witness, not a complementarity or
/// continuous-time enclosure. All storage is caller-owned.
pub fn solve_substepped_compliant_contact_impulse(
    input: CompliantContactImpulseInput<'_>,
    step_impulse_scratch: &mut [f64],
    impulse_out: &mut [f64],
    contact_velocity_after_out: &mut [f64],
    contact_gap_after_out: &mut [f64],
) -> Result<(), CoupledContactImpulseError> {
    let axes = input.contact_velocity.len();
    if axes == 0 || !axes.is_multiple_of(CONTACT_TRANSITION_IMPULSE_WIDTH) {
        return Err(CoupledContactImpulseError::Dimension);
    }
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;
    if input.contact_gap.len() != contacts
        || input.normal_stiffness.len() != contacts
        || input.normal_damping.len() != contacts
        || contact_gap_after_out.len() != contacts
        || step_impulse_scratch.len() != axes
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    validate_coupled_contact_impulse(
        CoupledContactImpulseInput {
            contact_velocity: input.contact_velocity,
            delassus: input.delassus,
            impulse_upper: input.impulse_upper,
            friction: input.friction,
            restitution: 0.0,
            diagonal_regularization_ratio: 0.0,
            sweeps: 1,
        },
        impulse_out.len(),
        contact_velocity_after_out.len(),
    )?;
    if !input.time_step_s.is_finite()
        || input.time_step_s <= 0.0
        || input.substeps == 0
        || input.substeps > 256
    {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    if input.contact_gap.iter().any(|value| !value.is_finite())
        || input
            .normal_stiffness
            .iter()
            .chain(input.normal_damping)
            .any(|value| !nonnegative_finite(*value))
    {
        return Err(CoupledContactImpulseError::InvalidWitness);
    }

    impulse_out.fill(0.0);
    contact_velocity_after_out.copy_from_slice(input.contact_velocity);
    contact_gap_after_out.copy_from_slice(input.contact_gap);
    let substep_s = input.time_step_s / input.substeps as f64;
    for _ in 0..input.substeps {
        step_impulse_scratch.fill(0.0);
        for contact in 0..contacts {
            let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
            let tangent_y = tangent_x + 1;
            let normal = tangent_x + 2;
            let predicted_gap =
                contact_gap_after_out[contact] + substep_s * contact_velocity_after_out[normal];
            if predicted_gap >= 0.0 {
                continue;
            }
            let penetration = -predicted_gap;
            let closing_speed = (-contact_velocity_after_out[normal]).max(0.0);
            let normal_force = input.normal_stiffness[contact] * penetration
                + input.normal_damping[contact] * closing_speed;
            let normal_remaining = (input.impulse_upper[normal] - impulse_out[normal]).max(0.0);
            let normal_impulse = (normal_force * substep_s).clamp(0.0, normal_remaining);
            step_impulse_scratch[normal] = normal_impulse;

            let friction_radius = input.friction[contact] * normal_impulse;
            let diagonal_x = input.delassus[tangent_x * axes + tangent_x];
            let diagonal_y = input.delassus[tangent_y * axes + tangent_y];
            let desired_total_x = (impulse_out[tangent_x]
                - contact_velocity_after_out[tangent_x] / diagonal_x)
                .clamp(
                    -input.impulse_upper[tangent_x],
                    input.impulse_upper[tangent_x],
                );
            let desired_total_y = (impulse_out[tangent_y]
                - contact_velocity_after_out[tangent_y] / diagonal_y)
                .clamp(
                    -input.impulse_upper[tangent_y],
                    input.impulse_upper[tangent_y],
                );
            let mut tangent_impulse_x = desired_total_x - impulse_out[tangent_x];
            let mut tangent_impulse_y = desired_total_y - impulse_out[tangent_y];
            let tangent_norm = tangent_impulse_x.hypot(tangent_impulse_y);
            if tangent_norm > friction_radius && tangent_norm > 0.0 {
                let scale = friction_radius / tangent_norm;
                tangent_impulse_x *= scale;
                tangent_impulse_y *= scale;
            }
            step_impulse_scratch[tangent_x] = tangent_impulse_x;
            step_impulse_scratch[tangent_y] = tangent_impulse_y;
        }
        for row in 0..axes {
            let velocity_delta = (0..axes)
                .map(|column| input.delassus[row * axes + column] * step_impulse_scratch[column])
                .sum::<f64>();
            contact_velocity_after_out[row] += velocity_delta;
        }
        for axis in 0..axes {
            impulse_out[axis] += step_impulse_scratch[axis];
        }
        for contact in 0..contacts {
            let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
            contact_gap_after_out[contact] += substep_s * contact_velocity_after_out[normal];
        }
    }
    Ok(())
}

fn positive_reference_impedance(
    position: f64,
    minimum: f64,
    maximum: f64,
    width: f64,
    midpoint: f64,
    power: f64,
) -> f64 {
    let normalized = position.abs() / width;
    if normalized >= 1.0 {
        return maximum;
    }
    let shape = if normalized < midpoint {
        normalized.powf(power) / midpoint.powf(power - 1.0)
    } else {
        1.0 - (1.0 - normalized).powf(power) / (1.0 - midpoint).powf(power - 1.0)
    };
    (minimum + shape * (maximum - minimum)).clamp(minimum, maximum)
}

fn project_tangent_impulse(
    tangent_x: f64,
    tangent_y: f64,
    radius: f64,
    cone: CompliantFrictionCone,
) -> (f64, f64) {
    match cone {
        CompliantFrictionCone::Circular => {
            let norm = tangent_x.hypot(tangent_y);
            if norm > radius && norm > 0.0 {
                let scale = radius / norm;
                (tangent_x * scale, tangent_y * scale)
            } else {
                (tangent_x, tangent_y)
            }
        }
        CompliantFrictionCone::Pyramidal => {
            let absolute_x = tangent_x.abs();
            let absolute_y = tangent_y.abs();
            if absolute_x + absolute_y <= radius {
                return (tangent_x, tangent_y);
            }
            if absolute_x - absolute_y >= radius {
                return (tangent_x.signum() * radius, 0.0);
            }
            if absolute_y - absolute_x >= radius {
                return (0.0, tangent_y.signum() * radius);
            }
            let threshold = 0.5 * (absolute_x + absolute_y - radius);
            (
                tangent_x.signum() * (absolute_x - threshold).max(0.0),
                tangent_y.signum() * (absolute_y - threshold).max(0.0),
            )
        }
    }
}

/// Integrate the documented positive-reference compliant law over fixed
/// microsteps.
///
/// This is still a reduced point-contact witness: it does not reproduce a
/// simulator's global nonlinear constraint optimization. It does, however,
/// retain the declared impedance spline, integration-safety time-constant
/// clamp, tangent decay, and cone geometry instead of collapsing them into one
/// fitted constant. Every input is validated before output mutation and all
/// work uses caller-owned storage.
pub fn solve_positive_reference_compliant_contact_impulse(
    input: PositiveReferenceCompliantContactImpulseInput<'_>,
    step_impulse_scratch: &mut [f64],
    impulse_out: &mut [f64],
    contact_velocity_after_out: &mut [f64],
    contact_gap_after_out: &mut [f64],
) -> Result<(), CoupledContactImpulseError> {
    if input.integrator == CompliantStepIntegrator::GeneralizedRk4 {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    let axes = input.contact_velocity.len();
    if axes == 0 || !axes.is_multiple_of(CONTACT_TRANSITION_IMPULSE_WIDTH) {
        return Err(CoupledContactImpulseError::Dimension);
    }
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;
    let contact_lengths = [
        input.contact_gap.len(),
        input.friction.len(),
        input.effective_normal_mass.len(),
        input.time_constant_s.len(),
        input.damping_ratio.len(),
        input.impedance_min.len(),
        input.impedance_max.len(),
        input.impedance_width_m.len(),
        input.impedance_midpoint.len(),
        input.impedance_power.len(),
        contact_gap_after_out.len(),
    ];
    if contact_lengths.iter().any(|length| *length != contacts)
        || input.contact_free_acceleration.len() != axes
        || step_impulse_scratch.len() != axes
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    validate_coupled_contact_impulse(
        CoupledContactImpulseInput {
            contact_velocity: input.contact_velocity,
            delassus: input.delassus,
            impulse_upper: input.impulse_upper,
            friction: input.friction,
            restitution: 0.0,
            diagonal_regularization_ratio: 0.0,
            sweeps: 1,
        },
        impulse_out.len(),
        contact_velocity_after_out.len(),
    )?;
    if !input.minimum_time_constant_s.is_finite()
        || input.minimum_time_constant_s <= 0.0
        || !input.time_step_s.is_finite()
        || input.time_step_s <= 0.0
        || input.substeps == 0
        || input.substeps > 256
    {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    for contact in 0..contacts {
        let finite_positive = |value: f64| value.is_finite() && value > 0.0;
        if !input.contact_gap[contact].is_finite()
            || !finite_positive(input.effective_normal_mass[contact])
            || !finite_positive(input.time_constant_s[contact])
            || !finite_positive(input.damping_ratio[contact])
            || !input.impedance_min[contact].is_finite()
            || !input.impedance_max[contact].is_finite()
            || input.impedance_min[contact] <= 0.0
            || input.impedance_min[contact] > input.impedance_max[contact]
            || input.impedance_max[contact] >= 1.0
            || !finite_positive(input.impedance_width_m[contact])
            || !input.impedance_midpoint[contact].is_finite()
            || !(0.0..1.0).contains(&input.impedance_midpoint[contact])
            || !input.impedance_power[contact].is_finite()
            || input.impedance_power[contact] < 1.0
        {
            return Err(CoupledContactImpulseError::InvalidWitness);
        }
    }
    if input
        .contact_free_acceleration
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(CoupledContactImpulseError::InvalidWitness);
    }

    impulse_out.fill(0.0);
    contact_velocity_after_out.copy_from_slice(input.contact_velocity);
    contact_gap_after_out.copy_from_slice(input.contact_gap);
    let substep_s = input.time_step_s / input.substeps as f64;
    for _ in 0..input.substeps {
        step_impulse_scratch.fill(0.0);
        for (contact, gap_after) in contact_gap_after_out.iter().copied().enumerate() {
            let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
            let tangent_y = tangent_x + 1;
            let normal = tangent_x + 2;
            let predicted_gap = gap_after
                + substep_s * contact_velocity_after_out[normal]
                + 0.5 * substep_s * substep_s * input.contact_free_acceleration[normal];
            if predicted_gap >= 0.0 {
                continue;
            }
            let impedance = positive_reference_impedance(
                predicted_gap,
                input.impedance_min[contact],
                input.impedance_max[contact],
                input.impedance_width_m[contact],
                input.impedance_midpoint[contact],
                input.impedance_power[contact],
            );
            let time_constant = input.time_constant_s[contact].max(input.minimum_time_constant_s);
            let damping_ratio = input.damping_ratio[contact];
            let impedance_width = input.impedance_max[contact];
            let mass = input.effective_normal_mass[contact];
            let reference_damping = 2.0 / (impedance_width * time_constant);
            let reference_stiffness = impedance
                / (impedance_width
                    * impedance_width
                    * time_constant
                    * time_constant
                    * damping_ratio
                    * damping_ratio);
            let reference_acceleration = -reference_damping * contact_velocity_after_out[normal]
                - reference_stiffness * predicted_gap;
            let contact_acceleration =
                impedance * (reference_acceleration - input.contact_free_acceleration[normal]);
            let normal_force = (mass * contact_acceleration).max(0.0);
            let normal_remaining = (input.impulse_upper[normal] - impulse_out[normal]).max(0.0);
            let normal_impulse = (normal_force * substep_s).clamp(0.0, normal_remaining);
            step_impulse_scratch[normal] = normal_impulse;

            let tangent_impedance = input.impedance_min[contact];
            let decay_rate = tangent_impedance * reference_damping;
            let diagonal_x = input.delassus[tangent_x * axes + tangent_x];
            let diagonal_y = input.delassus[tangent_y * axes + tangent_y];
            let contact_velocity_delta = |axis: usize| match input.integrator {
                CompliantStepIntegrator::ExplicitEuler => {
                    tangent_impedance
                        * (-reference_damping * contact_velocity_after_out[axis]
                            - input.contact_free_acceleration[axis])
                        * substep_s
                }
                CompliantStepIntegrator::ImplicitEuler => {
                    tangent_impedance
                        * (-reference_damping * contact_velocity_after_out[axis]
                            - input.contact_free_acceleration[axis])
                        * substep_s
                        / (1.0 + decay_rate * substep_s)
                }
                CompliantStepIntegrator::ExponentialTrapezoidal => {
                    let decay = (-decay_rate * substep_s).exp();
                    (decay - 1.0) * contact_velocity_after_out[axis]
                        + (1.0 - tangent_impedance)
                            * input.contact_free_acceleration[axis]
                            * (1.0 - decay)
                            / decay_rate
                        - input.contact_free_acceleration[axis] * substep_s
                }
                CompliantStepIntegrator::GeneralizedRk4 => {
                    unreachable!("generalized RK4 is model-coupled only")
                }
            };
            let desired_total_x =
                (impulse_out[tangent_x] + contact_velocity_delta(tangent_x) / diagonal_x).clamp(
                    -input.impulse_upper[tangent_x],
                    input.impulse_upper[tangent_x],
                );
            let desired_total_y =
                (impulse_out[tangent_y] + contact_velocity_delta(tangent_y) / diagonal_y).clamp(
                    -input.impulse_upper[tangent_y],
                    input.impulse_upper[tangent_y],
                );
            let (tangent_impulse_x, tangent_impulse_y) = project_tangent_impulse(
                desired_total_x - impulse_out[tangent_x],
                desired_total_y - impulse_out[tangent_y],
                input.friction[contact] * normal_impulse,
                input.friction_cone,
            );
            step_impulse_scratch[tangent_x] = tangent_impulse_x;
            step_impulse_scratch[tangent_y] = tangent_impulse_y;
        }

        if matches!(
            input.integrator,
            CompliantStepIntegrator::ExplicitEuler
                | CompliantStepIntegrator::ExponentialTrapezoidal
        ) {
            let scale = if input.integrator == CompliantStepIntegrator::ExplicitEuler {
                1.0
            } else {
                0.5
            };
            for (contact, gap_after) in contact_gap_after_out.iter_mut().enumerate() {
                let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
                *gap_after += scale * substep_s * contact_velocity_after_out[normal];
                if input.integrator == CompliantStepIntegrator::ExplicitEuler {
                    *gap_after +=
                        0.5 * substep_s * substep_s * input.contact_free_acceleration[normal];
                }
            }
        }
        for (row, velocity_after) in contact_velocity_after_out.iter_mut().enumerate() {
            let velocity_delta = (0..axes)
                .map(|column| input.delassus[row * axes + column] * step_impulse_scratch[column])
                .sum::<f64>();
            *velocity_after += substep_s * input.contact_free_acceleration[row] + velocity_delta;
        }
        for axis in 0..axes {
            impulse_out[axis] += step_impulse_scratch[axis];
        }
        if matches!(
            input.integrator,
            CompliantStepIntegrator::ImplicitEuler
                | CompliantStepIntegrator::ExponentialTrapezoidal
        ) {
            let scale = if input.integrator == CompliantStepIntegrator::ImplicitEuler {
                1.0
            } else {
                0.5
            };
            for (contact, gap_after) in contact_gap_after_out.iter_mut().enumerate() {
                let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
                *gap_after += scale * substep_s * contact_velocity_after_out[normal];
            }
        }
    }
    Ok(())
}

fn validate_coupled_positive_reference_compliant_contact(
    input: CoupledPositiveReferenceCompliantContactImpulseInput<'_>,
    desired_velocity_delta_scratch_len: usize,
    step_impulse_scratch_len: usize,
    impulse_out_len: usize,
    contact_velocity_after_out_len: usize,
    contact_gap_after_out_len: usize,
) -> Result<usize, CoupledContactImpulseError> {
    if input.integrator == CompliantStepIntegrator::GeneralizedRk4 {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    let axes = input.contact_velocity.len();
    if axes == 0 || !axes.is_multiple_of(CONTACT_TRANSITION_IMPULSE_WIDTH) {
        return Err(CoupledContactImpulseError::Dimension);
    }
    let contacts = axes / CONTACT_TRANSITION_IMPULSE_WIDTH;
    let contact_lengths = [
        input.contact_gap.len(),
        input.friction.len(),
        input.time_constant_s.len(),
        input.damping_ratio.len(),
        input.impedance_min.len(),
        input.impedance_max.len(),
        input.impedance_width_m.len(),
        input.impedance_midpoint.len(),
        input.impedance_power.len(),
        contact_gap_after_out_len,
    ];
    if contact_lengths.iter().any(|length| *length != contacts)
        || input.contact_free_acceleration.len() != axes
        || desired_velocity_delta_scratch_len != axes
        || step_impulse_scratch_len != axes
    {
        return Err(CoupledContactImpulseError::Dimension);
    }
    validate_coupled_contact_impulse(
        CoupledContactImpulseInput {
            contact_velocity: input.contact_velocity,
            delassus: input.delassus,
            impulse_upper: input.impulse_upper,
            friction: input.friction,
            restitution: 0.0,
            diagonal_regularization_ratio: 0.0,
            sweeps: 1,
        },
        impulse_out_len,
        contact_velocity_after_out_len,
    )?;
    if !input.minimum_time_constant_s.is_finite()
        || input.minimum_time_constant_s <= 0.0
        || !input.time_step_s.is_finite()
        || input.time_step_s <= 0.0
        || input.substeps == 0
        || input.substeps > 256
        || input.projection_sweeps == 0
        || input.projection_sweeps > 256
    {
        return Err(CoupledContactImpulseError::InvalidConfig);
    }
    if input
        .contact_free_acceleration
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(CoupledContactImpulseError::InvalidWitness);
    }
    for contact in 0..contacts {
        let finite_positive = |value: f64| value.is_finite() && value > 0.0;
        if !input.contact_gap[contact].is_finite()
            || !finite_positive(input.time_constant_s[contact])
            || !finite_positive(input.damping_ratio[contact])
            || !input.impedance_min[contact].is_finite()
            || !input.impedance_max[contact].is_finite()
            || input.impedance_min[contact] <= 0.0
            || input.impedance_min[contact] > input.impedance_max[contact]
            || input.impedance_max[contact] >= 1.0
            || !finite_positive(input.impedance_width_m[contact])
            || !input.impedance_midpoint[contact].is_finite()
            || !(0.0..1.0).contains(&input.impedance_midpoint[contact])
            || !input.impedance_power[contact].is_finite()
            || input.impedance_power[contact] < 1.0
        {
            return Err(CoupledContactImpulseError::InvalidWitness);
        }
    }
    Ok(contacts)
}

fn update_soft_contact_distribution(
    input: CoupledPositiveReferenceCompliantContactImpulseInput<'_>,
    desired_velocity_delta: &[f64],
    impulse: &[f64],
    step_impulse: &mut [f64],
    contact: usize,
) {
    let axes = step_impulse.len();
    let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
    let tangent_y = tangent_x + 1;
    let normal = tangent_x + 2;
    if desired_velocity_delta[normal] <= 0.0 {
        step_impulse[tangent_x] = 0.0;
        step_impulse[tangent_y] = 0.0;
        step_impulse[normal] = 0.0;
        return;
    }

    let normal_response = (0..axes)
        .map(|column| input.delassus[normal * axes + column] * step_impulse[column])
        .sum::<f64>();
    let normal_diagonal = input.delassus[normal * axes + normal];
    let normal_remaining = (input.impulse_upper[normal] - impulse[normal]).max(0.0);
    step_impulse[normal] = (step_impulse[normal]
        + (desired_velocity_delta[normal] - normal_response) / normal_diagonal)
        .clamp(0.0, normal_remaining);

    for tangent in [tangent_x, tangent_y] {
        let response = (0..axes)
            .map(|column| input.delassus[tangent * axes + column] * step_impulse[column])
            .sum::<f64>();
        let diagonal = input.delassus[tangent * axes + tangent];
        let candidate_step =
            step_impulse[tangent] + (desired_velocity_delta[tangent] - response) / diagonal;
        let total = (impulse[tangent] + candidate_step)
            .clamp(-input.impulse_upper[tangent], input.impulse_upper[tangent]);
        step_impulse[tangent] = total - impulse[tangent];
    }
    let normal_total = impulse[normal] + step_impulse[normal];
    let (projected_x, projected_y) = project_tangent_impulse(
        impulse[tangent_x] + step_impulse[tangent_x],
        impulse[tangent_y] + step_impulse[tangent_y],
        input.friction[contact] * normal_total,
        input.friction_cone,
    );
    step_impulse[tangent_x] = projected_x - impulse[tangent_x];
    step_impulse[tangent_y] = projected_y - impulse[tangent_y];
}

/// Integrate the positive-reference law while solving every microstep's soft
/// velocity increment through the complete contact-space Delassus operator.
///
/// The projected distribution minimizes the coupled quadratic contact-space
/// residual for a fixed number of deterministic forward/reverse sweeps. It is
/// a bounded-work reduced soft-contact solve, not MuJoCo's generalized
/// nonlinear constraint optimizer or an outer bound. Every input is validated
/// before output mutation and all workspaces are caller-owned.
pub fn solve_coupled_positive_reference_compliant_contact_impulse(
    input: CoupledPositiveReferenceCompliantContactImpulseInput<'_>,
    desired_velocity_delta_scratch: &mut [f64],
    step_impulse_scratch: &mut [f64],
    impulse_out: &mut [f64],
    contact_velocity_after_out: &mut [f64],
    contact_gap_after_out: &mut [f64],
) -> Result<(), CoupledContactImpulseError> {
    let contacts = validate_coupled_positive_reference_compliant_contact(
        input,
        desired_velocity_delta_scratch.len(),
        step_impulse_scratch.len(),
        impulse_out.len(),
        contact_velocity_after_out.len(),
        contact_gap_after_out.len(),
    )?;
    let axes = input.contact_velocity.len();
    impulse_out.fill(0.0);
    contact_velocity_after_out.copy_from_slice(input.contact_velocity);
    contact_gap_after_out.copy_from_slice(input.contact_gap);
    let substep_s = input.time_step_s / input.substeps as f64;

    for _ in 0..input.substeps {
        desired_velocity_delta_scratch.fill(0.0);
        step_impulse_scratch.fill(0.0);
        for (contact, gap_after) in contact_gap_after_out.iter().copied().enumerate() {
            let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
            let tangent_y = tangent_x + 1;
            let normal = tangent_x + 2;
            let predicted_gap = gap_after
                + substep_s * contact_velocity_after_out[normal]
                + 0.5 * substep_s * substep_s * input.contact_free_acceleration[normal];
            if predicted_gap >= 0.0 {
                continue;
            }
            let impedance = positive_reference_impedance(
                predicted_gap,
                input.impedance_min[contact],
                input.impedance_max[contact],
                input.impedance_width_m[contact],
                input.impedance_midpoint[contact],
                input.impedance_power[contact],
            );
            let time_constant = input.time_constant_s[contact].max(input.minimum_time_constant_s);
            let damping_ratio = input.damping_ratio[contact];
            let impedance_width = input.impedance_max[contact];
            let reference_damping = 2.0 / (impedance_width * time_constant);
            let reference_stiffness = impedance
                / (impedance_width
                    * impedance_width
                    * time_constant
                    * time_constant
                    * damping_ratio
                    * damping_ratio);
            let reference_acceleration = -reference_damping * contact_velocity_after_out[normal]
                - reference_stiffness * predicted_gap;
            desired_velocity_delta_scratch[normal] = (impedance
                * (reference_acceleration - input.contact_free_acceleration[normal])
                * substep_s)
                .max(0.0);
            if desired_velocity_delta_scratch[normal] <= 0.0 {
                continue;
            }

            let tangent_impedance = input.impedance_min[contact];
            let decay_rate = tangent_impedance * reference_damping;
            let tangent_delta = |axis: usize| match input.integrator {
                CompliantStepIntegrator::ExplicitEuler => {
                    tangent_impedance
                        * (-reference_damping * contact_velocity_after_out[axis]
                            - input.contact_free_acceleration[axis])
                        * substep_s
                }
                CompliantStepIntegrator::ImplicitEuler => {
                    tangent_impedance
                        * (-reference_damping * contact_velocity_after_out[axis]
                            - input.contact_free_acceleration[axis])
                        * substep_s
                        / (1.0 + decay_rate * substep_s)
                }
                CompliantStepIntegrator::ExponentialTrapezoidal => {
                    let decay = (-decay_rate * substep_s).exp();
                    (decay - 1.0) * contact_velocity_after_out[axis]
                        + (1.0 - tangent_impedance)
                            * input.contact_free_acceleration[axis]
                            * (1.0 - decay)
                            / decay_rate
                        - input.contact_free_acceleration[axis] * substep_s
                }
                CompliantStepIntegrator::GeneralizedRk4 => {
                    unreachable!("generalized RK4 is model-coupled only")
                }
            };
            desired_velocity_delta_scratch[tangent_x] = tangent_delta(tangent_x);
            desired_velocity_delta_scratch[tangent_y] = tangent_delta(tangent_y);
        }

        for _ in 0..input.projection_sweeps {
            for contact in 0..contacts {
                update_soft_contact_distribution(
                    input,
                    desired_velocity_delta_scratch,
                    impulse_out,
                    step_impulse_scratch,
                    contact,
                );
            }
            for contact in (0..contacts).rev() {
                update_soft_contact_distribution(
                    input,
                    desired_velocity_delta_scratch,
                    impulse_out,
                    step_impulse_scratch,
                    contact,
                );
            }
        }

        if matches!(
            input.integrator,
            CompliantStepIntegrator::ExplicitEuler
                | CompliantStepIntegrator::ExponentialTrapezoidal
        ) {
            let scale = if input.integrator == CompliantStepIntegrator::ExplicitEuler {
                1.0
            } else {
                0.5
            };
            for (contact, gap_after) in contact_gap_after_out.iter_mut().enumerate() {
                let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
                *gap_after += scale * substep_s * contact_velocity_after_out[normal];
                if input.integrator == CompliantStepIntegrator::ExplicitEuler {
                    *gap_after +=
                        0.5 * substep_s * substep_s * input.contact_free_acceleration[normal];
                }
            }
        }
        for (row, velocity_after) in contact_velocity_after_out.iter_mut().enumerate() {
            let velocity_delta = (0..axes)
                .map(|column| input.delassus[row * axes + column] * step_impulse_scratch[column])
                .sum::<f64>();
            *velocity_after += substep_s * input.contact_free_acceleration[row] + velocity_delta;
        }
        for axis in 0..axes {
            impulse_out[axis] += step_impulse_scratch[axis];
        }
        if matches!(
            input.integrator,
            CompliantStepIntegrator::ImplicitEuler
                | CompliantStepIntegrator::ExponentialTrapezoidal
        ) {
            let scale = if input.integrator == CompliantStepIntegrator::ImplicitEuler {
                1.0
            } else {
                0.5
            };
            for (contact, gap_after) in contact_gap_after_out.iter_mut().enumerate() {
                let normal = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + 2;
                *gap_after += scale * substep_s * contact_velocity_after_out[normal];
            }
        }
    }
    Ok(())
}

fn refresh_model_contact_snapshot(
    model: &CompiledModel,
    plane_normal_world: crate::math::Vec3,
    plane_offset_m: f64,
    apply_acceleration_bias: bool,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    refresh_model_contact_geometry_and_response(
        model,
        plane_normal_world,
        plane_offset_m,
        scratch,
    )?;
    write_model_contact_motion(apply_acceleration_bias, model, scratch)
}

fn refresh_model_contact_geometry_and_response(
    model: &CompiledModel,
    plane_normal_world: crate::math::Vec3,
    plane_offset_m: f64,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    let generalized_dof = model.dof + 6;
    let contacts = scratch.live_contacts.len();
    let axes = contacts * CONTACT_TRANSITION_IMPULSE_WIDTH;
    model
        .forward_kinematics(&scratch.state.robot, &mut scratch.response_scratch.model)
        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
    for contact in 0..contacts {
        let frame = scratch.live_contacts[contact].frame;
        let center_world = scratch.response_scratch.model.world_from_body[frame.0]
            .transform_point(&Point3::from(scratch.local_points[contact]))
            .coords;
        let point_world =
            center_world - plane_normal_world * scratch.contact_surface_radius_m[contact];
        scratch.live_contacts[contact].point_world = point_world;
        scratch.contact_gap[contact] = plane_normal_world.dot(&point_world) - plane_offset_m;
    }
    write_point_impulse_velocity_response_with_delassus(
        model,
        &scratch.state.robot,
        &scratch.live_contacts,
        &mut scratch.response_scratch,
        &mut scratch.response,
        &mut scratch.effective_mass,
        &mut scratch.delassus,
    )
    .map_err(ModelCoupledPositiveReferenceContactError::Response)?;
    if scratch.response.len() != generalized_dof * axes {
        return Err(ModelCoupledPositiveReferenceContactError::Dimension);
    }
    Ok(())
}

fn write_model_contact_motion(
    apply_acceleration_bias: bool,
    model: &CompiledModel,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    let generalized_dof = model.dof + 6;
    let contacts = scratch.live_contacts.len();
    model
        .floating_inverse_dynamics_into(
            &scratch.state.robot,
            scratch.state.root_twist_world,
            &scratch.generalized_free_acceleration,
            crate::math::Vec3::zeros(),
            &scratch.response_scratch.model,
            &mut scratch.response_scratch.dynamics,
            &mut scratch.response_scratch.rhs,
        )
        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;

    for contact in 0..contacts {
        let spec = scratch.live_contacts[contact];
        model
            .floating_point_jacobian_into(
                &scratch.response_scratch.model,
                spec.frame,
                scratch.local_points[contact],
                &mut scratch.response_scratch.point_jacobian,
            )
            .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
        let point_acceleration_world = model
            .point_bias_acceleration_world(
                spec.frame,
                scratch.local_points[contact],
                &scratch.response_scratch.model,
                &scratch.response_scratch.dynamics,
            )
            .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
        for axis in 0..CONTACT_TRANSITION_IMPULSE_WIDTH {
            let output = contact * CONTACT_TRANSITION_IMPULSE_WIDTH + axis;
            let mut velocity = 0.0;
            for row in 0..3 {
                let basis = spec.basis_world[axis][row];
                let mut world_velocity = 0.0;
                for coordinate in 0..generalized_dof {
                    let generalized_velocity = if coordinate < 6 {
                        scratch.state.root_twist_world.0[coordinate]
                    } else {
                        scratch.state.robot.v[coordinate - 6]
                    };
                    let jacobian = scratch.response_scratch.point_jacobian[(row, coordinate)];
                    world_velocity += jacobian * generalized_velocity;
                }
                velocity += basis * world_velocity;
            }
            scratch.contact_velocity[output] = velocity;
            scratch.contact_free_acceleration[output] = spec.basis_world[axis]
                .dot(&point_acceleration_world)
                + if apply_acceleration_bias {
                    scratch.contact_acceleration_bias[output]
                } else {
                    0.0
                };
        }
    }
    Ok(())
}

/// Reconstruct stage-local free acceleration under the generalized force
/// identified at the authored initial state. The response writer has already
/// factored the current floating mass matrix, so this is one allocation-free
/// triangular solve followed by inverse dynamics to populate point motion.
fn refresh_model_free_acceleration_from_held_force(
    model: &CompiledModel,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    model
        .floating_bias_forces_into(
            &scratch.state.robot,
            scratch.state.root_twist_world,
            crate::math::Vec3::zeros(),
            &scratch.response_scratch.model,
            &mut scratch.response_scratch.dynamics,
            &mut scratch.response_scratch.rhs,
        )
        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
    for coordinate in 0..scratch.held_generalized_force.len() {
        scratch.response_scratch.partition_rhs[coordinate] =
            scratch.held_generalized_force[coordinate] - scratch.response_scratch.rhs[coordinate];
    }
    if !solve_spd_factor_into(
        &scratch.response_scratch.mass_factor,
        &scratch.response_scratch.partition_rhs,
        &mut scratch.generalized_free_acceleration,
    ) {
        return Err(ModelCoupledPositiveReferenceContactError::Response(
            ContactTransitionResponseError::SingularMassMatrix,
        ));
    }
    write_model_contact_motion(true, model, scratch)
}

fn write_floating_velocity(state: &FloatingRobotState, velocity: &mut DVector<f64>) {
    velocity.as_mut_slice()[..6].copy_from_slice(state.root_twist_world.0.as_slice());
    velocity.as_mut_slice()[6..].copy_from_slice(state.robot.v.as_slice());
}

fn overwrite_floating_velocity(state: &mut FloatingRobotState, velocity: &DVector<f64>) {
    state
        .root_twist_world
        .0
        .as_mut_slice()
        .copy_from_slice(&velocity.as_slice()[..6]);
    state
        .robot
        .v
        .as_mut_slice()
        .copy_from_slice(&velocity.as_slice()[6..]);
}

fn solve_model_contact_stage(
    input: ModelCoupledPositiveReferenceCompliantContactImpulseInput<'_>,
    state_step_s: f64,
    local_integrator: CompliantStepIntegrator,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    for contact in 0..scratch.live_contacts.len() {
        let tangent_x = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
        let tangent_y = tangent_x + 1;
        let normal = tangent_x + 2;
        if scratch.contact_gap[contact] > 0.0 {
            scratch.remaining_impulse_upper[tangent_x] = 0.0;
            scratch.remaining_impulse_upper[tangent_y] = 0.0;
            scratch.remaining_impulse_upper[normal] = 0.0;
            continue;
        }
        scratch.remaining_impulse_upper[tangent_x] =
            (input.impulse_upper[tangent_x] - scratch.total_impulse[tangent_x].abs()).max(0.0);
        scratch.remaining_impulse_upper[tangent_y] =
            (input.impulse_upper[tangent_y] - scratch.total_impulse[tangent_y].abs()).max(0.0);
        scratch.remaining_impulse_upper[normal] =
            (input.impulse_upper[normal] - scratch.total_impulse[normal]).max(0.0);
    }
    solve_coupled_positive_reference_compliant_contact_impulse(
        CoupledPositiveReferenceCompliantContactImpulseInput {
            contact_gap: &scratch.contact_gap,
            contact_velocity: &scratch.contact_velocity,
            contact_free_acceleration: &scratch.contact_free_acceleration,
            delassus: &scratch.delassus,
            impulse_upper: &scratch.remaining_impulse_upper,
            friction: input.friction,
            time_constant_s: input.time_constant_s,
            damping_ratio: input.damping_ratio,
            impedance_min: input.impedance_min,
            impedance_max: input.impedance_max,
            impedance_width_m: input.impedance_width_m,
            impedance_midpoint: input.impedance_midpoint,
            impedance_power: input.impedance_power,
            minimum_time_constant_s: input.minimum_time_constant_s,
            time_step_s: state_step_s,
            substeps: input.compliance_substeps,
            projection_sweeps: input.projection_sweeps,
            friction_cone: input.friction_cone,
            integrator: local_integrator,
        },
        &mut scratch.desired_velocity_delta,
        &mut scratch.step_impulse,
        &mut scratch.outer_impulse,
        &mut scratch.step_velocity_after,
        &mut scratch.step_gap_after,
    )
    .map_err(ModelCoupledPositiveReferenceContactError::Contact)
}

/// Advance a floating model through a bounded sequence of causal contact
/// events while refreshing state-local geometry and dynamics.
///
/// Outputs are committed only after the complete evolution succeeds. Scratch
/// mutation is intentionally not transactional; it is caller-owned workspace.
/// Construction of the scratch allocates, while this function does not grow
/// any storage.
pub fn solve_model_coupled_positive_reference_compliant_contact_impulse(
    model: &CompiledModel,
    input: ModelCoupledPositiveReferenceCompliantContactImpulseInput<'_>,
    scratch: &mut ModelCoupledPositiveReferenceContactScratch,
    impulse_out: &mut [f64],
    contact_velocity_after_out: &mut [f64],
    contact_gap_after_out: &mut [f64],
    state_after_out: &mut FloatingRobotState,
) -> Result<(), ModelCoupledPositiveReferenceContactError> {
    let contacts = input.contacts.len();
    let axes = contacts
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ModelCoupledPositiveReferenceContactError::Dimension)?;
    let generalized_dof = model.dof + 6;
    let contact_lengths = [
        input.friction.len(),
        input.time_constant_s.len(),
        input.damping_ratio.len(),
        input.impedance_min.len(),
        input.impedance_max.len(),
        input.impedance_width_m.len(),
        input.impedance_midpoint.len(),
        input.impedance_power.len(),
        input.contact_surface_radius_m.len(),
        contact_gap_after_out.len(),
        scratch.local_points.len(),
        scratch.contact_surface_radius_m.len(),
        scratch.live_contacts.len(),
    ];
    if contacts == 0
        || contact_lengths.iter().any(|length| *length != contacts)
        || input.generalized_free_acceleration.len() != generalized_dof
        || input.initial_contact_free_acceleration.len() != axes
        || input.impulse_upper.len() != axes
        || impulse_out.len() != axes
        || contact_velocity_after_out.len() != axes
        || scratch.response.len() != generalized_dof * axes
        || scratch.effective_mass.len() != axes
        || scratch.delassus.len() != axes * axes
        || scratch.contact_velocity.len() != axes
        || scratch.contact_free_acceleration.len() != axes
        || scratch.contact_acceleration_bias.len() != axes
        || scratch.remaining_impulse_upper.len() != axes
        || scratch.outer_impulse.len() != axes
        || scratch.total_impulse.len() != axes
        || scratch.generalized_velocity_delta.len() != generalized_dof
        || scratch.generalized_free_acceleration.len() != generalized_dof
        || scratch.held_generalized_force.len() != generalized_dof
        || scratch.zero_generalized_acceleration.len() != generalized_dof
        || scratch
            .rk4_stage_velocity
            .iter()
            .any(|value| value.len() != generalized_dof)
        || scratch
            .rk4_stage_acceleration
            .iter()
            .any(|value| value.len() != generalized_dof)
        || scratch
            .rk4_stage_impulse
            .iter()
            .any(|value| value.len() != axes)
        || state_after_out.robot.q.len() != model.dof
        || state_after_out.robot.v.len() != model.dof
    {
        return Err(ModelCoupledPositiveReferenceContactError::Dimension);
    }
    input
        .initial_state
        .validate(model)
        .map_err(|_| ModelCoupledPositiveReferenceContactError::InvalidWitness)?;
    if input.state_steps == 0
        || input.state_steps > 64
        || input.compliance_substeps == 0
        || input.compliance_substeps > 256
        || input.projection_sweeps == 0
        || input.projection_sweeps > 256
        || !input.time_step_s.is_finite()
        || input.time_step_s <= 0.0
        || !input.minimum_time_constant_s.is_finite()
        || input.minimum_time_constant_s <= 0.0
        || !input.plane_offset_m.is_finite()
        || (input.plane_normal_world.norm_squared() - 1.0).abs() > 1.0e-9
    {
        return Err(ModelCoupledPositiveReferenceContactError::InvalidConfig);
    }
    if input
        .generalized_free_acceleration
        .iter()
        .chain(input.initial_contact_free_acceleration)
        .chain(input.impulse_upper)
        .chain(input.contact_surface_radius_m)
        .any(|value| !value.is_finite())
        || input.impulse_upper.iter().any(|value| *value < 0.0)
        || input
            .contact_surface_radius_m
            .iter()
            .any(|value| *value < 0.0)
        || input
            .contacts
            .iter()
            .any(|contact| contact.basis_world[2].dot(&input.plane_normal_world) < 1.0 - 1.0e-9)
    {
        return Err(ModelCoupledPositiveReferenceContactError::InvalidWitness);
    }

    scratch.state.clone_from(input.initial_state);
    scratch
        .generalized_free_acceleration
        .as_mut_slice()
        .copy_from_slice(input.generalized_free_acceleration);
    scratch.live_contacts.copy_from_slice(input.contacts);
    scratch
        .contact_surface_radius_m
        .copy_from_slice(input.contact_surface_radius_m);
    model
        .forward_kinematics(&scratch.state.robot, &mut scratch.response_scratch.model)
        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
    for (contact, spec) in input.contacts.iter().enumerate() {
        if spec.frame.0 >= model.bodies.len() {
            return Err(ModelCoupledPositiveReferenceContactError::InvalidWitness);
        }
        let pose = scratch.response_scratch.model.world_from_body[spec.frame.0];
        let center_world =
            spec.point_world + input.plane_normal_world * input.contact_surface_radius_m[contact];
        scratch.local_points[contact] = pose
            .inverse_transform_point(&Point3::from(center_world))
            .coords;
    }
    refresh_model_contact_snapshot(
        model,
        input.plane_normal_world,
        input.plane_offset_m,
        false,
        scratch,
    )?;
    for axis in 0..axes {
        scratch.contact_acceleration_bias[axis] =
            input.initial_contact_free_acceleration[axis] - scratch.contact_free_acceleration[axis];
        scratch.contact_free_acceleration[axis] = input.initial_contact_free_acceleration[axis];
    }
    scratch
        .held_generalized_force
        .copy_from(&scratch.response_scratch.rhs);
    scratch.total_impulse.fill(0.0);
    let state_step_s = input.time_step_s / input.state_steps as f64;
    if input.integrator == CompliantStepIntegrator::GeneralizedRk4 {
        let stage_scale = [0.0, 0.5, 0.5, 1.0];
        let stage_source = [0, 0, 1, 2];
        let rk4_weight = [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0];
        for _ in 0..input.state_steps {
            scratch.rk4_base_state.clone_from(&scratch.state);
            for stage in 0..4 {
                if stage > 0 {
                    let source = stage_source[stage];
                    let stage_dt = stage_scale[stage] * state_step_s;
                    scratch.state.clone_from(&scratch.rk4_base_state);
                    overwrite_floating_velocity(
                        &mut scratch.state,
                        &scratch.rk4_stage_velocity[source],
                    );
                    model
                        .integrate_floating(
                            &mut scratch.state,
                            &scratch.zero_generalized_acceleration,
                            stage_dt,
                        )
                        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
                    for coordinate in 0..generalized_dof {
                        scratch.generalized_velocity_delta[coordinate] = scratch.rk4_stage_velocity
                            [0][coordinate]
                            + stage_dt * scratch.rk4_stage_acceleration[source][coordinate];
                    }
                    overwrite_floating_velocity(
                        &mut scratch.state,
                        &scratch.generalized_velocity_delta,
                    );
                    model
                        .integrate_floating(
                            &mut scratch.state,
                            &scratch.zero_generalized_acceleration,
                            0.0,
                        )
                        .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
                }
                refresh_model_contact_geometry_and_response(
                    model,
                    input.plane_normal_world,
                    input.plane_offset_m,
                    scratch,
                )?;
                refresh_model_free_acceleration_from_held_force(model, scratch)?;
                write_floating_velocity(&scratch.state, &mut scratch.rk4_stage_velocity[stage]);
                // The local explicit solve evaluates the compliant-contact
                // right-hand side at this RK stage. The four generalized
                // state evaluations, rather than the former point-only
                // exponential/trapezoidal update, now own the global clock.
                solve_model_contact_stage(
                    input,
                    state_step_s,
                    CompliantStepIntegrator::ExplicitEuler,
                    scratch,
                )?;
                scratch.rk4_stage_impulse[stage]
                    .as_mut_slice()
                    .copy_from_slice(&scratch.outer_impulse);
                for coordinate in 0..generalized_dof {
                    let contact_acceleration = (0..axes)
                        .map(|axis| {
                            scratch.response[coordinate * axes + axis] * scratch.outer_impulse[axis]
                                / state_step_s
                        })
                        .sum::<f64>();
                    scratch.rk4_stage_acceleration[stage][coordinate] =
                        scratch.generalized_free_acceleration[coordinate] + contact_acceleration;
                }
            }

            for axis in 0..axes {
                scratch.outer_impulse[axis] = (0..4)
                    .map(|stage| rk4_weight[stage] * scratch.rk4_stage_impulse[stage][axis])
                    .sum();
                scratch.total_impulse[axis] += scratch.outer_impulse[axis];
            }
            for coordinate in 0..generalized_dof {
                scratch.generalized_velocity_delta[coordinate] = (0..4)
                    .map(|stage| rk4_weight[stage] * scratch.rk4_stage_velocity[stage][coordinate])
                    .sum();
            }
            scratch.state.clone_from(&scratch.rk4_base_state);
            overwrite_floating_velocity(&mut scratch.state, &scratch.generalized_velocity_delta);
            model
                .integrate_floating(
                    &mut scratch.state,
                    &scratch.zero_generalized_acceleration,
                    state_step_s,
                )
                .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
            for coordinate in 0..generalized_dof {
                scratch.generalized_velocity_delta[coordinate] = scratch.rk4_stage_velocity[0]
                    [coordinate]
                    + state_step_s
                        * (0..4)
                            .map(|stage| {
                                rk4_weight[stage]
                                    * scratch.rk4_stage_acceleration[stage][coordinate]
                            })
                            .sum::<f64>();
            }
            overwrite_floating_velocity(&mut scratch.state, &scratch.generalized_velocity_delta);
            model
                .integrate_floating(
                    &mut scratch.state,
                    &scratch.zero_generalized_acceleration,
                    0.0,
                )
                .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
            refresh_model_contact_geometry_and_response(
                model,
                input.plane_normal_world,
                input.plane_offset_m,
                scratch,
            )?;
            refresh_model_free_acceleration_from_held_force(model, scratch)?;
        }
    } else {
        for _ in 0..input.state_steps {
            // Euler collision membership is sampled once at the authored
            // state-step boundary. RK4 instead samples each declared stage.
            solve_model_contact_stage(input, state_step_s, input.integrator, scratch)?;
            for axis in 0..axes {
                scratch.total_impulse[axis] += scratch.outer_impulse[axis];
            }
            for coordinate in 0..generalized_dof {
                scratch.generalized_velocity_delta[coordinate] = (0..axes)
                    .map(|axis| {
                        scratch.response[coordinate * axes + axis] * scratch.outer_impulse[axis]
                    })
                    .sum();
            }
            for axis in 0..6 {
                scratch.state.root_twist_world.0[axis] += scratch.generalized_velocity_delta[axis];
            }
            for joint in 0..model.dof {
                scratch.state.robot.v[joint] += scratch.generalized_velocity_delta[6 + joint];
            }
            // The authored Euler families are velocity-first: apply free
            // acceleration to the tangent, then advance configuration with
            // that updated tangent. Do not replace this with midpoint
            // kinematics; doing so changes which contacts exist at the next
            // outer collision-detection tick.
            for axis in 0..6 {
                scratch.state.root_twist_world.0[axis] +=
                    scratch.generalized_free_acceleration[axis] * state_step_s;
            }
            for joint in 0..model.dof {
                scratch.state.robot.v[joint] +=
                    scratch.generalized_free_acceleration[6 + joint] * state_step_s;
            }
            scratch.generalized_velocity_delta.fill(0.0);
            model
                .integrate_floating(
                    &mut scratch.state,
                    &scratch.generalized_velocity_delta,
                    state_step_s,
                )
                .map_err(|_| ModelCoupledPositiveReferenceContactError::Model)?;
            refresh_model_contact_snapshot(
                model,
                input.plane_normal_world,
                input.plane_offset_m,
                true,
                scratch,
            )?;
        }
    }

    impulse_out.copy_from_slice(&scratch.total_impulse);
    contact_velocity_after_out.copy_from_slice(&scratch.contact_velocity);
    contact_gap_after_out.copy_from_slice(&scratch.contact_gap);
    state_after_out.clone_from(&scratch.state);
    Ok(())
}

fn nonnegative_finite(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Write componentwise contact-impulse and generalized velocity-jump bounds.
///
/// Each normal impulse is bounded by
///
/// `m_eff_max * (1 + restitution_max) * closing_speed_max
///  + normal_force_max * transition_time_max`.
///
/// Each tangential axis is independently box-bounded by `mu_max * normal_max`.
/// The box is intentionally an outer approximation of a friction cone. The
/// returned generalized interval adds the uncertain constant-acceleration
/// prefix and projects every impulse interval through the supplied response.
/// Work is `O(dof * contacts)` and no allocation is performed.
pub fn write_contact_transition_bounds(
    input: ContactTransitionInput<'_>,
    impulse_upper_out: &mut [f64],
    delta_velocity_lower_out: &mut [f64],
    delta_velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionError> {
    if !nonnegative_finite(input.transition_time_lower_s)
        || !nonnegative_finite(input.transition_time_upper_s)
        || input.transition_time_lower_s > input.transition_time_upper_s
        || !nonnegative_finite(input.restitution_upper)
        || input.restitution_upper > 1.0
    {
        return Err(ContactTransitionError::InvalidConfig);
    }
    let dof = input.generalized_acceleration.len();
    if dof == 0
        || input.contact_witnesses.is_empty()
        || !input
            .contact_witnesses
            .len()
            .is_multiple_of(CONTACT_TRANSITION_WITNESS_WIDTH)
    {
        return Err(ContactTransitionError::Dimension);
    }
    let contacts = input.contact_witnesses.len() / CONTACT_TRANSITION_WITNESS_WIDTH;
    let impulse_values = contacts
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ContactTransitionError::Dimension)?;
    let response_values = dof
        .checked_mul(impulse_values)
        .ok_or(ContactTransitionError::Dimension)?;
    if input.impulse_velocity_response.len() != response_values
        || impulse_upper_out.len() != impulse_values
        || delta_velocity_lower_out.len() != dof
        || delta_velocity_upper_out.len() != dof
    {
        return Err(ContactTransitionError::Dimension);
    }
    if input
        .contact_witnesses
        .iter()
        .any(|value| !nonnegative_finite(*value))
    {
        return Err(ContactTransitionError::InvalidWitness);
    }
    if input
        .generalized_acceleration
        .iter()
        .chain(input.impulse_velocity_response.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ContactTransitionError::InvalidResponse);
    }

    for contact in 0..contacts {
        let witness = &input.contact_witnesses[contact * CONTACT_TRANSITION_WITNESS_WIDTH
            ..(contact + 1) * CONTACT_TRANSITION_WITNESS_WIDTH];
        let closing_speed_upper_m_s = witness[0];
        let effective_normal_mass_upper_kg = witness[1];
        let sustained_normal_force_upper_n = witness[2];
        let friction_coefficient_upper = witness[3];
        let normal_upper_ns = effective_normal_mass_upper_kg
            * (1.0 + input.restitution_upper)
            * closing_speed_upper_m_s
            + sustained_normal_force_upper_n * input.transition_time_upper_s;
        let tangent_upper_ns = friction_coefficient_upper * normal_upper_ns;
        if !normal_upper_ns.is_finite() || !tangent_upper_ns.is_finite() {
            return Err(ContactTransitionError::InvalidWitness);
        }
    }

    for contact in 0..contacts {
        let witness = &input.contact_witnesses[contact * CONTACT_TRANSITION_WITNESS_WIDTH
            ..(contact + 1) * CONTACT_TRANSITION_WITNESS_WIDTH];
        let normal_upper_ns = witness[1] * (1.0 + input.restitution_upper) * witness[0]
            + witness[2] * input.transition_time_upper_s;
        let tangent_upper_ns = witness[3] * normal_upper_ns;
        let output = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
        impulse_upper_out[output] = tangent_upper_ns;
        impulse_upper_out[output + 1] = tangent_upper_ns;
        impulse_upper_out[output + 2] = normal_upper_ns;
    }

    for coordinate in 0..dof {
        let acceleration = input.generalized_acceleration[coordinate];
        let (mut lower, mut upper) = if acceleration >= 0.0 {
            (
                acceleration * input.transition_time_lower_s,
                acceleration * input.transition_time_upper_s,
            )
        } else {
            (
                acceleration * input.transition_time_upper_s,
                acceleration * input.transition_time_lower_s,
            )
        };
        let response_row = &input.impulse_velocity_response
            [coordinate * impulse_values..(coordinate + 1) * impulse_values];
        for contact in 0..contacts {
            let impulse = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
            for tangent_axis in 0..2 {
                let radius = response_row[impulse + tangent_axis].abs()
                    * impulse_upper_out[impulse + tangent_axis];
                lower -= radius;
                upper += radius;
            }
            let normal_delta = response_row[impulse + 2] * impulse_upper_out[impulse + 2];
            lower += normal_delta.min(0.0);
            upper += normal_delta.max(0.0);
        }
        delta_velocity_lower_out[coordinate] = lower;
        delta_velocity_upper_out[coordinate] = upper;
    }
    Ok(())
}

/// Contact-transition input with a componentwise continuous-acceleration tube.
///
/// The interval is independent of the impact impulse box. This distinction is
/// important when contact impulse already covers but declared external load,
/// actuator realization, or smooth model mismatch remains between support
/// hypotheses and the following plant interval.
#[derive(Clone, Copy, Debug)]
pub struct ContactTransitionAccelerationIntervalInput<'a> {
    pub transition_time_lower_s: f64,
    pub transition_time_upper_s: f64,
    pub restitution_upper: f64,
    pub contact_witnesses: &'a [f64],
    pub generalized_acceleration_lower: &'a [f64],
    pub generalized_acceleration_upper: &'a [f64],
    pub impulse_velocity_response: &'a [f64],
}

/// Write the physical impulse bound plus a componentwise acceleration tube.
///
/// For every coordinate, all four products of acceleration and nonnegative
/// transition-time endpoints are considered before the impulse response is
/// added. Inputs are fully validated before outputs change; work remains
/// `O(dof * contacts)` and allocation-free.
pub fn write_contact_transition_acceleration_interval_bounds(
    input: ContactTransitionAccelerationIntervalInput<'_>,
    impulse_upper_out: &mut [f64],
    delta_velocity_lower_out: &mut [f64],
    delta_velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionError> {
    if !nonnegative_finite(input.transition_time_lower_s)
        || !nonnegative_finite(input.transition_time_upper_s)
        || input.transition_time_lower_s > input.transition_time_upper_s
        || !nonnegative_finite(input.restitution_upper)
        || input.restitution_upper > 1.0
    {
        return Err(ContactTransitionError::InvalidConfig);
    }
    let dof = input.generalized_acceleration_lower.len();
    if dof == 0
        || input.generalized_acceleration_upper.len() != dof
        || input.contact_witnesses.is_empty()
        || !input
            .contact_witnesses
            .len()
            .is_multiple_of(CONTACT_TRANSITION_WITNESS_WIDTH)
    {
        return Err(ContactTransitionError::Dimension);
    }
    let contacts = input.contact_witnesses.len() / CONTACT_TRANSITION_WITNESS_WIDTH;
    let impulse_values = contacts
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ContactTransitionError::Dimension)?;
    let response_values = dof
        .checked_mul(impulse_values)
        .ok_or(ContactTransitionError::Dimension)?;
    if input.impulse_velocity_response.len() != response_values
        || impulse_upper_out.len() != impulse_values
        || delta_velocity_lower_out.len() != dof
        || delta_velocity_upper_out.len() != dof
    {
        return Err(ContactTransitionError::Dimension);
    }
    if input
        .contact_witnesses
        .iter()
        .any(|value| !nonnegative_finite(*value))
    {
        return Err(ContactTransitionError::InvalidWitness);
    }
    if input
        .generalized_acceleration_lower
        .iter()
        .chain(input.generalized_acceleration_upper.iter())
        .chain(input.impulse_velocity_response.iter())
        .any(|value| !value.is_finite())
        || input
            .generalized_acceleration_lower
            .iter()
            .zip(input.generalized_acceleration_upper)
            .any(|(lower, upper)| lower > upper)
    {
        return Err(ContactTransitionError::InvalidResponse);
    }
    for contact in 0..contacts {
        let witness = &input.contact_witnesses[contact * CONTACT_TRANSITION_WITNESS_WIDTH
            ..(contact + 1) * CONTACT_TRANSITION_WITNESS_WIDTH];
        let normal_upper_ns = witness[1] * (1.0 + input.restitution_upper) * witness[0]
            + witness[2] * input.transition_time_upper_s;
        let tangent_upper_ns = witness[3] * normal_upper_ns;
        if !normal_upper_ns.is_finite() || !tangent_upper_ns.is_finite() {
            return Err(ContactTransitionError::InvalidWitness);
        }
    }

    for contact in 0..contacts {
        let witness = &input.contact_witnesses[contact * CONTACT_TRANSITION_WITNESS_WIDTH
            ..(contact + 1) * CONTACT_TRANSITION_WITNESS_WIDTH];
        let normal_upper_ns = witness[1] * (1.0 + input.restitution_upper) * witness[0]
            + witness[2] * input.transition_time_upper_s;
        let tangent_upper_ns = witness[3] * normal_upper_ns;
        let output = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
        impulse_upper_out[output] = tangent_upper_ns;
        impulse_upper_out[output + 1] = tangent_upper_ns;
        impulse_upper_out[output + 2] = normal_upper_ns;
    }

    for coordinate in 0..dof {
        let acceleration_lower = input.generalized_acceleration_lower[coordinate];
        let acceleration_upper = input.generalized_acceleration_upper[coordinate];
        let products = [
            acceleration_lower * input.transition_time_lower_s,
            acceleration_lower * input.transition_time_upper_s,
            acceleration_upper * input.transition_time_lower_s,
            acceleration_upper * input.transition_time_upper_s,
        ];
        let mut lower = products.into_iter().fold(f64::INFINITY, f64::min);
        let mut upper = products.into_iter().fold(f64::NEG_INFINITY, f64::max);
        let response_row = &input.impulse_velocity_response
            [coordinate * impulse_values..(coordinate + 1) * impulse_values];
        for contact in 0..contacts {
            let impulse = contact * CONTACT_TRANSITION_IMPULSE_WIDTH;
            for tangent_axis in 0..2 {
                let radius = response_row[impulse + tangent_axis].abs()
                    * impulse_upper_out[impulse + tangent_axis];
                lower -= radius;
                upper += radius;
            }
            let normal_delta = response_row[impulse + 2] * impulse_upper_out[impulse + 2];
            lower += normal_delta.min(0.0);
            upper += normal_delta.max(0.0);
        }
        delta_velocity_lower_out[coordinate] = lower;
        delta_velocity_upper_out[coordinate] = upper;
    }
    Ok(())
}

/// Contact transition through a finite patch spatial-wrench outer set.
///
/// Each patch uses one resultant normal impulse. Tangential force, CoP moment,
/// and torsional moment capacities are all proportional to that same normal
/// impulse, avoiding the independent-point Cartesian product.
#[derive(Clone, Copy, Debug)]
pub struct SpatialPatchTransitionInput<'a> {
    pub transition_time_lower_s: f64,
    pub transition_time_upper_s: f64,
    pub restitution_upper: f64,
    /// Patch-major rows `[speed_xyz, effective_mass_xyz,
    /// sustained_force_xyz, friction, half_length_x, half_width_y,
    /// torsion_radius]`.
    pub patch_witnesses: &'a [f64],
    pub generalized_acceleration_lower: &'a [f64],
    pub generalized_acceleration_upper: &'a [f64],
    /// Row-major `[dof, patch, 6]`, moment XYZ then force XYZ.
    pub spatial_impulse_velocity_response: &'a [f64],
}

#[inline]
fn spatial_patch_support(
    normal_impulse_upper: f64,
    friction: f64,
    tangent_x_free_upper: f64,
    tangent_y_free_upper: f64,
    signed_normal_response: f64,
    moment_radius_per_normal: f64,
    tangent_x_response_abs: f64,
    tangent_y_response_abs: f64,
) -> f64 {
    let objective = |normal_impulse: f64| {
        (signed_normal_response + moment_radius_per_normal) * normal_impulse
            + tangent_x_response_abs * (friction * normal_impulse).min(tangent_x_free_upper)
            + tangent_y_response_abs * (friction * normal_impulse).min(tangent_y_free_upper)
    };
    let mut support = 0.0_f64.max(objective(normal_impulse_upper));
    if friction > 0.0 {
        support = support.max(objective(
            (tangent_x_free_upper / friction).min(normal_impulse_upper),
        ));
        support = support.max(objective(
            (tangent_y_free_upper / friction).min(normal_impulse_upper),
        ));
    }
    support
}

/// Write a componentwise generalized velocity bound for finite contact patches.
///
/// For normal impulse `j` in `[0, j_max]`, the patch set is
/// `|fx|,|fy| <= mu*j`, `|mx| <= half_width*j`,
/// `|my| <= half_length*j`, and `|mz| <= torsion_radius*j`.
/// The exact support of this box-pyramid for each response row is evaluated
/// without enumerating vertices. Inputs validate atomically and work is
/// `O(dof * patches)` with no allocation.
pub fn write_spatial_patch_contact_transition_bounds(
    input: SpatialPatchTransitionInput<'_>,
    normal_impulse_upper_out: &mut [f64],
    delta_velocity_lower_out: &mut [f64],
    delta_velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionError> {
    if !nonnegative_finite(input.transition_time_lower_s)
        || !nonnegative_finite(input.transition_time_upper_s)
        || input.transition_time_lower_s > input.transition_time_upper_s
        || !nonnegative_finite(input.restitution_upper)
        || input.restitution_upper > 1.0
    {
        return Err(ContactTransitionError::InvalidConfig);
    }
    let dof = input.generalized_acceleration_lower.len();
    if dof == 0
        || input.generalized_acceleration_upper.len() != dof
        || input.patch_witnesses.is_empty()
        || !input
            .patch_witnesses
            .len()
            .is_multiple_of(SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH)
    {
        return Err(ContactTransitionError::Dimension);
    }
    let patches = input.patch_witnesses.len() / SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH;
    if normal_impulse_upper_out.len() != patches
        || delta_velocity_lower_out.len() != dof
        || delta_velocity_upper_out.len() != dof
        || input.spatial_impulse_velocity_response.len()
            != dof
                .saturating_mul(patches)
                .saturating_mul(SPATIAL_IMPULSE_WIDTH)
    {
        return Err(ContactTransitionError::Dimension);
    }
    if input
        .patch_witnesses
        .iter()
        .any(|value| !nonnegative_finite(*value))
    {
        return Err(ContactTransitionError::InvalidWitness);
    }
    if input
        .generalized_acceleration_lower
        .iter()
        .chain(input.generalized_acceleration_upper)
        .chain(input.spatial_impulse_velocity_response)
        .any(|value| !value.is_finite())
        || input
            .generalized_acceleration_lower
            .iter()
            .zip(input.generalized_acceleration_upper)
            .any(|(lower, upper)| lower > upper)
    {
        return Err(ContactTransitionError::InvalidResponse);
    }
    for patch in 0..patches {
        let witness = &input.patch_witnesses[patch * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH
            ..(patch + 1) * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH];
        let normal_upper = witness[5] * (1.0 + input.restitution_upper) * witness[2]
            + witness[8] * input.transition_time_upper_s;
        let tangent_x_upper = (witness[9] * normal_upper)
            .min(witness[3] * witness[0] + witness[6] * input.transition_time_upper_s);
        let tangent_y_upper = (witness[9] * normal_upper)
            .min(witness[4] * witness[1] + witness[7] * input.transition_time_upper_s);
        if !normal_upper.is_finite()
            || !tangent_x_upper.is_finite()
            || !tangent_y_upper.is_finite()
            || [witness[10], witness[11], witness[12]]
                .into_iter()
                .any(|scale| !(scale * normal_upper).is_finite())
        {
            return Err(ContactTransitionError::InvalidWitness);
        }
    }
    let patch_response_width = patches * SPATIAL_IMPULSE_WIDTH;
    for coordinate in 0..dof {
        let response_row = &input.spatial_impulse_velocity_response
            [coordinate * patch_response_width..(coordinate + 1) * patch_response_width];
        for patch in 0..patches {
            let witness = &input.patch_witnesses[patch * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH
                ..(patch + 1) * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH];
            let response =
                &response_row[patch * SPATIAL_IMPULSE_WIDTH..(patch + 1) * SPATIAL_IMPULSE_WIDTH];
            let normal_upper = witness[5] * (1.0 + input.restitution_upper) * witness[2]
                + witness[8] * input.transition_time_upper_s;
            let tangent_x_free =
                witness[3] * witness[0] + witness[6] * input.transition_time_upper_s;
            let tangent_y_free =
                witness[4] * witness[1] + witness[7] * input.transition_time_upper_s;
            let moment_radius_per_normal = response[0].abs() * witness[11]
                + response[1].abs() * witness[10]
                + response[2].abs() * witness[12];
            let upper_support = spatial_patch_support(
                normal_upper,
                witness[9],
                tangent_x_free,
                tangent_y_free,
                response[5],
                moment_radius_per_normal,
                response[3].abs(),
                response[4].abs(),
            );
            let lower_support = spatial_patch_support(
                normal_upper,
                witness[9],
                tangent_x_free,
                tangent_y_free,
                -response[5],
                moment_radius_per_normal,
                response[3].abs(),
                response[4].abs(),
            );
            if !upper_support.is_finite() || !lower_support.is_finite() {
                return Err(ContactTransitionError::InvalidResponse);
            }
        }
    }

    for patch in 0..patches {
        let witness = &input.patch_witnesses[patch * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH
            ..(patch + 1) * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH];
        normal_impulse_upper_out[patch] = witness[5] * (1.0 + input.restitution_upper) * witness[2]
            + witness[8] * input.transition_time_upper_s;
    }
    for coordinate in 0..dof {
        let acceleration_lower = input.generalized_acceleration_lower[coordinate];
        let acceleration_upper = input.generalized_acceleration_upper[coordinate];
        let products = [
            acceleration_lower * input.transition_time_lower_s,
            acceleration_lower * input.transition_time_upper_s,
            acceleration_upper * input.transition_time_lower_s,
            acceleration_upper * input.transition_time_upper_s,
        ];
        let mut lower = products.into_iter().fold(f64::INFINITY, f64::min);
        let mut upper = products.into_iter().fold(f64::NEG_INFINITY, f64::max);
        let response_row = &input.spatial_impulse_velocity_response
            [coordinate * patch_response_width..(coordinate + 1) * patch_response_width];
        for patch in 0..patches {
            let witness = &input.patch_witnesses[patch * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH
                ..(patch + 1) * SPATIAL_PATCH_TRANSITION_WITNESS_WIDTH];
            let response =
                &response_row[patch * SPATIAL_IMPULSE_WIDTH..(patch + 1) * SPATIAL_IMPULSE_WIDTH];
            let normal_upper = normal_impulse_upper_out[patch];
            let tangent_x_free =
                witness[3] * witness[0] + witness[6] * input.transition_time_upper_s;
            let tangent_y_free =
                witness[4] * witness[1] + witness[7] * input.transition_time_upper_s;
            let moment_radius_per_normal = response[0].abs() * witness[11]
                + response[1].abs() * witness[10]
                + response[2].abs() * witness[12];
            upper += spatial_patch_support(
                normal_upper,
                witness[9],
                tangent_x_free,
                tangent_y_free,
                response[5],
                moment_radius_per_normal,
                response[3].abs(),
                response[4].abs(),
            );
            lower -= spatial_patch_support(
                normal_upper,
                witness[9],
                tangent_x_free,
                tangent_y_free,
                -response[5],
                moment_radius_per_normal,
                response[3].abs(),
                response[4].abs(),
            );
        }
        delta_velocity_lower_out[coordinate] = lower;
        delta_velocity_upper_out[coordinate] = upper;
    }
    Ok(())
}

/// Contact transition with directional slip/mass/load witnesses and a
/// componentwise continuous-acceleration interval.
#[derive(Clone, Copy, Debug)]
pub struct DirectionalContactTransitionInput<'a> {
    pub transition_time_lower_s: f64,
    pub transition_time_upper_s: f64,
    pub restitution_upper: f64,
    /// Contact-major rows `[speed_upper_xyz, effective_mass_upper_xyz,
    /// sustained_force_upper_xyz, friction_upper]` in tangent-X,
    /// tangent-Y, normal order.
    pub contact_witnesses: &'a [f64],
    pub generalized_acceleration_lower: &'a [f64],
    pub generalized_acceleration_upper: &'a [f64],
    pub impulse_velocity_response: &'a [f64],
}

/// Write a passive directional impulse tube.
///
/// The normal axis keeps the R201 restitution bound. Each tangential axis is
/// bounded by the smaller of Coulomb capacity and the impulse required by its
/// declared slip/effective-mass witness plus sustained tangential load:
/// `J_t <= min(mu J_n, m_eff,t v_slip,t + F_t dt_max)`.
/// Tangential signs remain symmetric, so this is still an outer box rather
/// than a contact-law or complementarity solution.
pub fn write_directional_contact_transition_bounds(
    input: DirectionalContactTransitionInput<'_>,
    impulse_upper_out: &mut [f64],
    delta_velocity_lower_out: &mut [f64],
    delta_velocity_upper_out: &mut [f64],
) -> Result<(), ContactTransitionError> {
    if !nonnegative_finite(input.transition_time_lower_s)
        || !nonnegative_finite(input.transition_time_upper_s)
        || input.transition_time_lower_s > input.transition_time_upper_s
        || !nonnegative_finite(input.restitution_upper)
        || input.restitution_upper > 1.0
    {
        return Err(ContactTransitionError::InvalidConfig);
    }
    let dof = input.generalized_acceleration_lower.len();
    if dof == 0
        || input.generalized_acceleration_upper.len() != dof
        || input.contact_witnesses.is_empty()
        || !input
            .contact_witnesses
            .len()
            .is_multiple_of(DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH)
    {
        return Err(ContactTransitionError::Dimension);
    }
    let contacts = input.contact_witnesses.len() / DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH;
    let impulse_values = contacts
        .checked_mul(CONTACT_TRANSITION_IMPULSE_WIDTH)
        .ok_or(ContactTransitionError::Dimension)?;
    if input.impulse_velocity_response.len() != dof.saturating_mul(impulse_values)
        || impulse_upper_out.len() != impulse_values
        || delta_velocity_lower_out.len() != dof
        || delta_velocity_upper_out.len() != dof
    {
        return Err(ContactTransitionError::Dimension);
    }
    if input
        .contact_witnesses
        .iter()
        .any(|value| !nonnegative_finite(*value))
    {
        return Err(ContactTransitionError::InvalidWitness);
    }
    if input
        .generalized_acceleration_lower
        .iter()
        .chain(input.generalized_acceleration_upper)
        .chain(input.impulse_velocity_response)
        .any(|value| !value.is_finite())
        || input
            .generalized_acceleration_lower
            .iter()
            .zip(input.generalized_acceleration_upper)
            .any(|(lower, upper)| lower > upper)
    {
        return Err(ContactTransitionError::InvalidResponse);
    }
    for contact in 0..contacts {
        let start = contact * DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH;
        let witness =
            &input.contact_witnesses[start..start + DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH];
        let normal = witness[5] * (1.0 + input.restitution_upper) * witness[2]
            + witness[8] * input.transition_time_upper_s;
        let friction = witness[9] * normal;
        let tangent_x =
            (witness[3] * witness[0] + witness[6] * input.transition_time_upper_s).min(friction);
        let tangent_y =
            (witness[4] * witness[1] + witness[7] * input.transition_time_upper_s).min(friction);
        if !normal.is_finite() || !tangent_x.is_finite() || !tangent_y.is_finite() {
            return Err(ContactTransitionError::InvalidWitness);
        }
    }

    for contact in 0..contacts {
        let start = contact * DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH;
        let witness =
            &input.contact_witnesses[start..start + DIRECTIONAL_CONTACT_TRANSITION_WITNESS_WIDTH];
        let normal = witness[5] * (1.0 + input.restitution_upper) * witness[2]
            + witness[8] * input.transition_time_upper_s;
        let friction = witness[9] * normal;
        let output = contact * 3;
        impulse_upper_out[output] =
            (witness[3] * witness[0] + witness[6] * input.transition_time_upper_s).min(friction);
        impulse_upper_out[output + 1] =
            (witness[4] * witness[1] + witness[7] * input.transition_time_upper_s).min(friction);
        impulse_upper_out[output + 2] = normal;
    }

    for coordinate in 0..dof {
        let products = [
            input.generalized_acceleration_lower[coordinate] * input.transition_time_lower_s,
            input.generalized_acceleration_lower[coordinate] * input.transition_time_upper_s,
            input.generalized_acceleration_upper[coordinate] * input.transition_time_lower_s,
            input.generalized_acceleration_upper[coordinate] * input.transition_time_upper_s,
        ];
        let mut lower = products.into_iter().fold(f64::INFINITY, f64::min);
        let mut upper = products.into_iter().fold(f64::NEG_INFINITY, f64::max);
        let response_row = &input.impulse_velocity_response
            [coordinate * impulse_values..(coordinate + 1) * impulse_values];
        for impulse in 0..impulse_values {
            if impulse % 3 < 2 {
                let radius = response_row[impulse].abs() * impulse_upper_out[impulse];
                lower -= radius;
                upper += radius;
            } else {
                let delta = response_row[impulse] * impulse_upper_out[impulse];
                lower += delta.min(0.0);
                upper += delta.max(0.0);
            }
        }
        delta_velocity_lower_out[coordinate] = lower;
        delta_velocity_upper_out[coordinate] = upper;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input<'a>(
        contacts: &'a [f64],
        acceleration: &'a [f64],
        response: &'a [f64],
    ) -> ContactTransitionInput<'a> {
        ContactTransitionInput {
            transition_time_lower_s: 0.01,
            transition_time_upper_s: 0.02,
            restitution_upper: 0.5,
            contact_witnesses: contacts,
            generalized_acceleration: acceleration,
            impulse_velocity_response: response,
        }
    }

    #[test]
    fn combines_uncertain_acceleration_time_and_impulse_box() {
        let mut impulse = [0.0; 3];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_contact_transition_bounds(
            input(&[2.0, 3.0, 10.0, 0.5], &[4.0], &[2.0, -1.0, 0.5]),
            &mut impulse,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert_eq!(impulse, [4.6, 4.6, 9.2]);
        assert!((lower[0] - -13.76).abs() < 1e-12);
        assert!((upper[0] - 18.48).abs() < 1e-12);
    }

    #[test]
    fn normal_response_sign_and_negative_acceleration_are_outer_bounded() {
        let mut impulse = [0.0; 3];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_contact_transition_bounds(
            input(&[1.0, 2.0, 0.0, 0.0], &[-3.0], &[0.0, 0.0, -2.0]),
            &mut impulse,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert_eq!(impulse, [0.0, 0.0, 3.0]);
        assert!((lower[0] - -6.06).abs() < 1e-12);
        assert!((upper[0] - -0.03).abs() < 1e-12);
    }

    #[test]
    fn contact_order_does_not_change_projected_interval() {
        let acceleration = [0.0, 1.0];
        let contacts_ab = [1.0, 2.0, 3.0, 0.4, 0.5, 4.0, 5.0, 0.2];
        let contacts_ba = [0.5, 4.0, 5.0, 0.2, 1.0, 2.0, 3.0, 0.4];
        let response_ab = [1.0, 2.0, 3.0, -1.0, 0.5, 2.0, 0.0, 1.0, -2.0, 3.0, 0.0, 1.0];
        let response_ba = [-1.0, 0.5, 2.0, 1.0, 2.0, 3.0, 3.0, 0.0, 1.0, 0.0, 1.0, -2.0];
        let mut impulse_ab = [0.0; 6];
        let mut impulse_ba = [0.0; 6];
        let mut lower_ab = [0.0; 2];
        let mut upper_ab = [0.0; 2];
        let mut lower_ba = [0.0; 2];
        let mut upper_ba = [0.0; 2];
        write_contact_transition_bounds(
            input(&contacts_ab, &acceleration, &response_ab),
            &mut impulse_ab,
            &mut lower_ab,
            &mut upper_ab,
        )
        .unwrap();
        write_contact_transition_bounds(
            input(&contacts_ba, &acceleration, &response_ba),
            &mut impulse_ba,
            &mut lower_ba,
            &mut upper_ba,
        )
        .unwrap();
        for coordinate in 0..2 {
            assert!((lower_ab[coordinate] - lower_ba[coordinate]).abs() < 1e-12);
            assert!((upper_ab[coordinate] - upper_ba[coordinate]).abs() < 1e-12);
        }
        assert_eq!(&impulse_ab[..3], &impulse_ba[3..]);
        assert_eq!(&impulse_ab[3..], &impulse_ba[..3]);
    }

    #[test]
    fn invalid_input_does_not_modify_outputs() {
        let mut impulse = [7.0; 3];
        let mut lower = [8.0; 1];
        let mut upper = [9.0; 1];
        let mut invalid = input(&[1.0, 2.0, 3.0, -0.1], &[0.0], &[0.0; 3]);
        invalid.transition_time_upper_s = 0.005;
        assert_eq!(
            write_contact_transition_bounds(invalid, &mut impulse, &mut lower, &mut upper),
            Err(ContactTransitionError::InvalidConfig)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(lower, [8.0]);
        assert_eq!(upper, [9.0]);
    }

    #[test]
    fn realized_values_inside_authored_boxes_are_covered() {
        let contacts = [1.0, 2.0, 0.0, 0.5];
        let response = [1.0, -2.0, 0.25];
        let mut impulse = [0.0; 3];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_contact_transition_bounds(
            input(&contacts, &[2.0], &response),
            &mut impulse,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        let actual_impulse = [0.5 * impulse[0], -0.25 * impulse[1], 0.8 * impulse[2]];
        let actual = 2.0 * 0.015
            + response[0] * actual_impulse[0]
            + response[1] * actual_impulse[1]
            + response[2] * actual_impulse[2];
        assert!(lower[0] <= actual && actual <= upper[0]);
    }

    #[test]
    fn model_owned_spatial_response_solves_all_wrench_columns() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.03 * (0.47 * coordinate as f64).cos();
        }
        let frame = FrameId(model.root.0);
        let reference_point_world = crate::math::Vec3::new(0.11, -0.07, 0.19);
        let wrench = SpatialImpulseResponseSpec {
            frame,
            reference_point_world,
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        };
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut response = vec![0.0; generalized_dof * SPATIAL_IMPULSE_WIDTH];
        let mut delassus = [0.0; SPATIAL_IMPULSE_WIDTH * SPATIAL_IMPULSE_WIDTH];
        write_spatial_impulse_velocity_response(
            &model,
            &state,
            &[wrench],
            &mut scratch,
            &mut response,
            &mut delassus,
        )
        .unwrap();

        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        let pose = scratch.model.world_from_body[frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(reference_point_world))
            .coords;
        let mut point = DMatrix::zeros(3, generalized_dof);
        let mut angular = DMatrix::zeros(3, generalized_dof);
        model
            .floating_point_jacobian_into(&scratch.model, frame, point_in_frame, &mut point)
            .unwrap();
        model
            .floating_angular_jacobian_into(&scratch.model, frame, &mut angular)
            .unwrap();
        for spatial_axis in 0..SPATIAL_IMPULSE_WIDTH {
            let solution = DVector::from_iterator(
                generalized_dof,
                (0..generalized_dof)
                    .map(|coordinate| response[coordinate * SPATIAL_IMPULSE_WIDTH + spatial_axis]),
            );
            let expected = if spatial_axis < 3 {
                angular.row(spatial_axis).transpose()
            } else {
                point.row(spatial_axis - 3).transpose()
            };
            assert!((&mass * &solution - &expected).norm() < 1.0e-9);
        }
        for row in 0..SPATIAL_IMPULSE_WIDTH {
            assert!(delassus[row * SPATIAL_IMPULSE_WIDTH + row] > 0.0);
            for column in 0..SPATIAL_IMPULSE_WIDTH {
                assert_eq!(
                    delassus[row * SPATIAL_IMPULSE_WIDTH + column],
                    delassus[column * SPATIAL_IMPULSE_WIDTH + row]
                );
            }
        }
    }

    #[test]
    fn spatial_wrench_translation_matches_force_at_contact_point() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let frame = FrameId(model.root.0);
        let reference = crate::math::Vec3::new(-0.04, 0.02, 0.15);
        let contact_point = crate::math::Vec3::new(0.09, -0.08, -0.03);
        let force = crate::math::Vec3::new(0.7, -0.4, 1.2);
        let moment_about_reference = (contact_point - reference).cross(&force);
        let basis = [
            crate::math::Vec3::x(),
            crate::math::Vec3::y(),
            crate::math::Vec3::z(),
        ];
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut point_response = vec![0.0; generalized_dof * 3];
        let mut effective_mass = [0.0; 3];
        write_point_impulse_velocity_response(
            &model,
            &state,
            &[PointImpulseResponseSpec {
                frame,
                point_world: contact_point,
                basis_world: basis,
            }],
            &mut scratch,
            &mut point_response,
            &mut effective_mass,
        )
        .unwrap();
        let mut spatial_response = vec![0.0; generalized_dof * SPATIAL_IMPULSE_WIDTH];
        let mut delassus = [0.0; SPATIAL_IMPULSE_WIDTH * SPATIAL_IMPULSE_WIDTH];
        write_spatial_impulse_velocity_response(
            &model,
            &state,
            &[SpatialImpulseResponseSpec {
                frame,
                reference_point_world: reference,
                basis_world: basis,
            }],
            &mut scratch,
            &mut spatial_response,
            &mut delassus,
        )
        .unwrap();
        for coordinate in 0..generalized_dof {
            let point_delta = (0..3)
                .map(|axis| point_response[coordinate * 3 + axis] * force[axis])
                .sum::<f64>();
            let spatial_delta = (0..3)
                .map(|axis| {
                    spatial_response[coordinate * SPATIAL_IMPULSE_WIDTH + axis]
                        * moment_about_reference[axis]
                        + spatial_response[coordinate * SPATIAL_IMPULSE_WIDTH + 3 + axis]
                            * force[axis]
                })
                .sum::<f64>();
            assert!((point_delta - spatial_delta).abs() < 1.0e-10);
        }
    }

    #[test]
    fn generalized_momentum_residual_matches_floating_mass_covector() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.02 * (0.29 * coordinate as f64).sin();
        }
        let generalized_dof = model.dof + 6;
        let observed = DVector::from_iterator(
            generalized_dof,
            (0..generalized_dof).map(|index| 0.01 * (index + 1) as f64),
        );
        let mut predicted = vec![0.0; 2 * generalized_dof];
        for coordinate in 0..generalized_dof {
            predicted[coordinate] = -0.003 * coordinate as f64;
            predicted[generalized_dof + coordinate] = 0.004 * coordinate as f64;
        }
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut residual = vec![0.0; predicted.len()];
        write_generalized_momentum_impulse_residuals(
            &model,
            &state,
            observed.as_slice(),
            &predicted,
            &mut scratch,
            &mut residual,
        )
        .unwrap();
        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        for candidate in 0..2 {
            let predicted_candidate = DVector::from_column_slice(
                &predicted[candidate * generalized_dof..(candidate + 1) * generalized_dof],
            );
            let expected = &mass * (&observed - predicted_candidate);
            let actual = DVector::from_column_slice(
                &residual[candidate * generalized_dof..(candidate + 1) * generalized_dof],
            );
            assert!((actual - expected).norm() < 1.0e-10);
        }
    }

    #[test]
    fn generalized_momentum_residual_rejects_shape_atomically() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut residual = vec![7.0; generalized_dof];
        assert_eq!(
            write_generalized_momentum_impulse_residuals(
                &model,
                &state,
                &vec![0.0; generalized_dof],
                &vec![0.0; generalized_dof - 1],
                &mut scratch,
                &mut residual,
            ),
            Err(ContactTransitionResponseError::Dimension)
        );
        assert!(residual.iter().all(|value| *value == 7.0));
    }

    #[test]
    fn momentum_box_maps_through_full_inverse_mass() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.03 * (0.37 * coordinate as f64).sin();
        }
        let generalized_dof = model.dof + 6;
        let momentum = DVector::from_iterator(
            generalized_dof,
            (0..generalized_dof).map(|index| 0.002 * (index as f64 - 4.0)),
        );
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![0.0; generalized_dof];
        let mut upper = vec![0.0; generalized_dof];
        write_generalized_velocity_interval_from_momentum_box(
            &model,
            &state,
            momentum.as_slice(),
            momentum.as_slice(),
            &mut scratch,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert_eq!(lower, upper);
        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        let velocity = DVector::from_column_slice(&lower);
        assert!((&mass * velocity - momentum).norm() < 1.0e-9);
    }

    #[test]
    fn momentum_box_rejects_inversion_before_outputs_change() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let mut momentum_lower = vec![0.0; generalized_dof];
        let mut momentum_upper = vec![0.0; generalized_dof];
        momentum_lower[3] = 1.0;
        momentum_upper[3] = -1.0;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![7.0; generalized_dof];
        let mut upper = vec![8.0; generalized_dof];
        assert_eq!(
            write_generalized_velocity_interval_from_momentum_box(
                &model,
                &state,
                &momentum_lower,
                &momentum_upper,
                &mut scratch,
                &mut lower,
                &mut upper,
            ),
            Err(ContactTransitionResponseError::Dimension)
        );
        assert!(lower.iter().all(|value| *value == 7.0));
        assert!(upper.iter().all(|value| *value == 8.0));
    }

    #[test]
    fn kinetic_impulse_ellipsoid_uses_exact_inverse_mass_diagonal() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.02 * (0.43 * coordinate as f64).sin();
        }
        let generalized_dof = model.dof + 6;
        let twice_energy = 0.037;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![0.0; generalized_dof];
        let mut upper = vec![0.0; generalized_dof];
        write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
            &model,
            &state,
            twice_energy,
            &mut scratch,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        let inverse = mass.try_inverse().unwrap();
        for coordinate in 0..generalized_dof {
            let expected = (twice_energy * inverse[(coordinate, coordinate)]).sqrt();
            assert!((upper[coordinate] - expected).abs() < 1.0e-10);
            assert_eq!(lower[coordinate], -upper[coordinate]);
        }
    }

    #[test]
    fn kinetic_impulse_ellipsoid_rejects_negative_energy_atomically() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![7.0; generalized_dof];
        let mut upper = vec![8.0; generalized_dof];
        assert_eq!(
            write_generalized_velocity_bounds_from_kinetic_impulse_ellipsoid(
                &model,
                &state,
                -1.0,
                &mut scratch,
                &mut lower,
                &mut upper,
            ),
            Err(ContactTransitionResponseError::Dimension)
        );
        assert!(lower.iter().all(|value| *value == 7.0));
        assert!(upper.iter().all(|value| *value == 8.0));
    }

    #[test]
    fn split_kinetic_impulse_ellipsoids_match_independent_block_oracle() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.025 * (0.29 * coordinate as f64).sin();
        }
        let generalized_dof = model.dof + 6;
        let root_energy = 0.021;
        let articulated_energy = 0.006;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![0.0; generalized_dof];
        let mut upper = vec![0.0; generalized_dof];
        write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
            &model,
            &state,
            root_energy,
            articulated_energy,
            &mut scratch,
            &mut lower,
            &mut upper,
        )
        .unwrap();

        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        let inverse = mass.try_inverse().unwrap();
        let root_inverse = DMatrix::from_fn(6, 6, |row, column| inverse[(row, column)])
            .try_inverse()
            .unwrap();
        let articulated_inverse = DMatrix::from_fn(model.dof, model.dof, |row, column| {
            inverse[(6 + row, 6 + column)]
        })
        .try_inverse()
        .unwrap();
        for coordinate in 0..generalized_dof {
            let root_column = DVector::from_fn(6, |row, _| inverse[(coordinate, row)]);
            let articulated_column =
                DVector::from_fn(model.dof, |row, _| inverse[(coordinate, 6 + row)]);
            let root_support = root_column.dot(&(&root_inverse * &root_column));
            let articulated_support =
                articulated_column.dot(&(&articulated_inverse * &articulated_column));
            let expected = (root_energy * root_support).sqrt()
                + (articulated_energy * articulated_support).sqrt();
            assert!((upper[coordinate] - expected).abs() < 1.0e-9);
            assert_eq!(lower[coordinate], -upper[coordinate]);
        }
    }

    #[test]
    fn split_kinetic_impulse_ellipsoids_reject_bad_budget_atomically() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut lower = vec![7.0; generalized_dof];
        let mut upper = vec![8.0; generalized_dof];
        assert_eq!(
            write_generalized_velocity_bounds_from_split_kinetic_impulse_ellipsoids(
                &model,
                &state,
                0.1,
                -0.1,
                &mut scratch,
                &mut lower,
                &mut upper,
            ),
            Err(ContactTransitionResponseError::Dimension)
        );
        assert!(lower.iter().all(|value| *value == 7.0));
        assert!(upper.iter().all(|value| *value == 8.0));
    }

    #[test]
    fn model_owned_point_response_solves_floating_inertia_columns() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut state = RobotState::zeros(&model);
        for coordinate in 0..model.dof {
            state.q[coordinate] = 0.04 * (0.31 * coordinate as f64).sin();
        }
        let frame = FrameId(model.root.0);
        let contact = PointImpulseResponseSpec {
            frame,
            point_world: crate::math::Vec3::new(0.08, -0.03, 0.12),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        };
        let generalized_dof = model.dof + 6;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut response = vec![0.0; generalized_dof * 3];
        let mut effective_mass = [0.0; 3];
        write_point_impulse_velocity_response(
            &model,
            &state,
            &[contact],
            &mut scratch,
            &mut response,
            &mut effective_mass,
        )
        .unwrap();

        let mut dynamics = DynamicsCache::new(&model);
        let mut mass = DMatrix::zeros(generalized_dof, generalized_dof);
        model
            .floating_mass_matrix_into(&scratch.model, &mut dynamics, &mut mass)
            .unwrap();
        let pose = scratch.model.world_from_body[frame.0];
        let point_in_frame = pose
            .inverse_transform_point(&Point3::from(contact.point_world))
            .coords;
        let mut jacobian = DMatrix::zeros(3, generalized_dof);
        model
            .floating_point_jacobian_into(&scratch.model, frame, point_in_frame, &mut jacobian)
            .unwrap();
        for axis in 0..3 {
            let solution = DVector::from_iterator(
                generalized_dof,
                (0..generalized_dof).map(|coordinate| response[coordinate * 3 + axis]),
            );
            let expected = jacobian.row(axis).transpose();
            assert!((&mass * &solution - &expected).norm() < 1.0e-9);
            let inverse_mass = expected.dot(&solution);
            assert!((effective_mass[axis] - 1.0 / inverse_mass).abs() < 1.0e-9);
        }
    }

    #[test]
    fn model_owned_delassus_is_symmetric_and_matches_directional_mass() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let contacts = [
            PointImpulseResponseSpec {
                frame: FrameId(model.root.0),
                point_world: crate::math::Vec3::new(0.05, 0.10, 0.0),
                basis_world: [
                    crate::math::Vec3::x(),
                    crate::math::Vec3::y(),
                    crate::math::Vec3::z(),
                ],
            },
            PointImpulseResponseSpec {
                frame: FrameId(model.root.0),
                point_world: crate::math::Vec3::new(-0.03, -0.09, 0.02),
                basis_world: [
                    crate::math::Vec3::x(),
                    crate::math::Vec3::y(),
                    crate::math::Vec3::z(),
                ],
            },
        ];
        let axes = contacts.len() * 3;
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut response = vec![0.0; (model.dof + 6) * axes];
        let mut effective_mass = vec![0.0; axes];
        let mut delassus = vec![0.0; axes * axes];
        write_point_impulse_velocity_response_with_delassus(
            &model,
            &state,
            &contacts,
            &mut scratch,
            &mut response,
            &mut effective_mass,
            &mut delassus,
        )
        .unwrap();
        for row in 0..axes {
            assert!((delassus[row * axes + row] - 1.0 / effective_mass[row]).abs() < 1.0e-9);
            for column in 0..axes {
                assert_eq!(delassus[row * axes + column], delassus[column * axes + row]);
            }
        }
        assert!(delassus[2 * axes + 5].abs() > 1.0e-9);
    }

    #[test]
    fn model_coupled_contact_refreshes_state_without_growing_storage() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut initial = FloatingRobotState::zeros(&model);
        initial.robot.control_world_from_root.translation.vector.z = 0.0015;
        initial.root_twist_world.0[5] = -1.0;
        let contacts = [PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::new(0.0, 0.0, 0.0015),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        }];
        let generalized_dof = model.dof + 6;
        let acceleration = vec![0.0; generalized_dof];
        let input = ModelCoupledPositiveReferenceCompliantContactImpulseInput {
            initial_state: &initial,
            contacts: &contacts,
            contact_surface_radius_m: &[0.0],
            plane_normal_world: crate::math::Vec3::z(),
            plane_offset_m: 0.0,
            generalized_free_acceleration: &acceleration,
            initial_contact_free_acceleration: &[0.0; 3],
            impulse_upper: &[10.0; 3],
            friction: &[0.5],
            time_constant_s: &[0.02],
            damping_ratio: &[1.0],
            impedance_min: &[0.8],
            impedance_max: &[0.9],
            impedance_width_m: &[0.001],
            impedance_midpoint: &[0.5],
            impedance_power: &[2.0],
            minimum_time_constant_s: 0.002,
            time_step_s: 0.005,
            state_steps: 5,
            compliance_substeps: 8,
            projection_sweeps: 8,
            friction_cone: CompliantFrictionCone::Circular,
            integrator: CompliantStepIntegrator::ImplicitEuler,
        };
        let mut scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut impulse = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut gap = [0.0; 1];
        let mut after = FloatingRobotState::zeros(&model);
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            input,
            &mut scratch,
            &mut impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert!(impulse[2] > 0.0);
        assert!(after.root_twist_world.0[5] > initial.root_twist_world.0[5]);
        let first = (impulse, velocity, gap, after.clone());
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            input,
            &mut scratch,
            &mut impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert_eq!(impulse, first.0);
        assert_eq!(velocity, first.1);
        assert_eq!(gap, first.2);
        assert_eq!(
            after.root_twist_world.0.as_slice(),
            first.3.root_twist_world.0.as_slice()
        );
    }

    #[test]
    fn model_coupled_rk4_integrates_generalized_constant_force() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut initial = FloatingRobotState::zeros(&model);
        initial.robot.control_world_from_root.translation.vector.z = 1.0;
        let contacts = [PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::new(0.0, 0.0, 1.0),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        }];
        let mut acceleration = vec![0.0; model.dof + 6];
        acceleration[5] = 2.0;
        let mut scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut impulse = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut gap = [0.0];
        let mut after = FloatingRobotState::zeros(&model);
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            ModelCoupledPositiveReferenceCompliantContactImpulseInput {
                initial_state: &initial,
                contacts: &contacts,
                contact_surface_radius_m: &[0.0],
                plane_normal_world: crate::math::Vec3::z(),
                plane_offset_m: 0.0,
                generalized_free_acceleration: &acceleration,
                initial_contact_free_acceleration: &[0.0, 0.0, 2.0],
                impulse_upper: &[0.0; 3],
                friction: &[0.5],
                time_constant_s: &[0.02],
                damping_ratio: &[1.0],
                impedance_min: &[0.8],
                impedance_max: &[0.9],
                impedance_width_m: &[0.001],
                impedance_midpoint: &[0.5],
                impedance_power: &[2.0],
                minimum_time_constant_s: 0.002,
                time_step_s: 0.01,
                state_steps: 1,
                compliance_substeps: 1,
                projection_sweeps: 1,
                friction_cone: CompliantFrictionCone::Circular,
                integrator: CompliantStepIntegrator::GeneralizedRk4,
            },
            &mut scratch,
            &mut impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert_eq!(impulse, [0.0; 3]);
        assert!((after.root_twist_world.0[5] - 0.02).abs() < 1.0e-12);
        assert!(
            (after.robot.control_world_from_root.translation.vector.z - 1.0001).abs() < 1.0e-12
        );
    }

    #[test]
    fn model_coupled_rk4_detects_contact_at_intermediate_stage() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut initial = FloatingRobotState::zeros(&model);
        initial.robot.control_world_from_root.translation.vector.z = 0.0004;
        initial.root_twist_world.0[5] = -1.0;
        let contacts = [PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::new(0.0, 0.0, 0.0004),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        }];
        let acceleration = vec![0.0; model.dof + 6];
        let base_input = ModelCoupledPositiveReferenceCompliantContactImpulseInput {
            initial_state: &initial,
            contacts: &contacts,
            contact_surface_radius_m: &[0.0],
            plane_normal_world: crate::math::Vec3::z(),
            plane_offset_m: 0.0,
            generalized_free_acceleration: &acceleration,
            initial_contact_free_acceleration: &[0.0; 3],
            impulse_upper: &[0.0, 0.0, 10.0],
            friction: &[0.5],
            time_constant_s: &[0.02],
            damping_ratio: &[1.0],
            impedance_min: &[0.8],
            impedance_max: &[0.9],
            impedance_width_m: &[0.001],
            impedance_midpoint: &[0.5],
            impedance_power: &[2.0],
            minimum_time_constant_s: 0.002,
            time_step_s: 0.001,
            state_steps: 1,
            compliance_substeps: 1,
            projection_sweeps: 1,
            friction_cone: CompliantFrictionCone::Circular,
            integrator: CompliantStepIntegrator::ExplicitEuler,
        };
        let mut explicit_scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut explicit_impulse = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut gap = [0.0];
        let mut after = FloatingRobotState::zeros(&model);
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            base_input,
            &mut explicit_scratch,
            &mut explicit_impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert_eq!(explicit_impulse, [0.0; 3]);

        // Exponential-trapezoidal remains the legacy single-event model
        // integrator. It samples the initial separated witness only, so this
        // crossing is intentionally not promoted to a contact event.
        let mut exponential_scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut exponential_impulse = [0.0; 3];
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            ModelCoupledPositiveReferenceCompliantContactImpulseInput {
                integrator: CompliantStepIntegrator::ExponentialTrapezoidal,
                ..base_input
            },
            &mut exponential_scratch,
            &mut exponential_impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert_eq!(exponential_impulse, [0.0; 3]);

        let mut rk4_scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut rk4_impulse = [0.0; 3];
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            ModelCoupledPositiveReferenceCompliantContactImpulseInput {
                integrator: CompliantStepIntegrator::GeneralizedRk4,
                ..base_input
            },
            &mut rk4_scratch,
            &mut rk4_impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert!(rk4_impulse[2] > 0.0);
        assert!(after.root_twist_world.0[5] > initial.root_twist_world.0[5]);
    }

    #[test]
    fn model_coupled_contact_refreshes_convective_point_acceleration() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let mut initial = FloatingRobotState::zeros(&model);
        initial.robot.control_world_from_root.translation.vector.z = 1.0;
        initial.root_twist_world.0[2] = 10.0;
        let contacts = [PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::new(0.1, 0.0, 1.0),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        }];
        let acceleration = vec![0.0; model.dof + 6];
        let mut scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut impulse = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut gap = [0.0; 1];
        let mut after = FloatingRobotState::zeros(&model);
        solve_model_coupled_positive_reference_compliant_contact_impulse(
            &model,
            ModelCoupledPositiveReferenceCompliantContactImpulseInput {
                initial_state: &initial,
                contacts: &contacts,
                contact_surface_radius_m: &[0.0],
                plane_normal_world: crate::math::Vec3::z(),
                plane_offset_m: 0.0,
                generalized_free_acceleration: &acceleration,
                initial_contact_free_acceleration: &[-10.0, 0.0, 0.0],
                impulse_upper: &[0.0; 3],
                friction: &[0.5],
                time_constant_s: &[0.02],
                damping_ratio: &[1.0],
                impedance_min: &[0.8],
                impedance_max: &[0.9],
                impedance_width_m: &[0.001],
                impedance_midpoint: &[0.5],
                impedance_power: &[2.0],
                minimum_time_constant_s: 0.002,
                time_step_s: 0.005,
                state_steps: 5,
                compliance_substeps: 1,
                projection_sweeps: 1,
                friction_cone: CompliantFrictionCone::Circular,
                integrator: CompliantStepIntegrator::ImplicitEuler,
            },
            &mut scratch,
            &mut impulse,
            &mut velocity,
            &mut gap,
            &mut after,
        )
        .unwrap();
        assert_eq!(impulse, [0.0; 3]);
        assert!(scratch.contact_free_acceleration[1] < -0.1);
        assert!(scratch.contact_free_acceleration[0] > -10.0);
    }

    #[test]
    fn model_coupled_contact_rejects_bad_clock_atomically() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let initial = FloatingRobotState::zeros(&model);
        let contacts = [PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::zeros(),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::y(),
                crate::math::Vec3::z(),
            ],
        }];
        let acceleration = vec![0.0; model.dof + 6];
        let mut scratch = ModelCoupledPositiveReferenceContactScratch::new(&model, 1);
        let mut impulse = [7.0; 3];
        let mut velocity = [8.0; 3];
        let mut gap = [9.0];
        let mut after = FloatingRobotState::zeros(&model);
        assert_eq!(
            solve_model_coupled_positive_reference_compliant_contact_impulse(
                &model,
                ModelCoupledPositiveReferenceCompliantContactImpulseInput {
                    initial_state: &initial,
                    contacts: &contacts,
                    contact_surface_radius_m: &[0.0],
                    plane_normal_world: crate::math::Vec3::z(),
                    plane_offset_m: 0.0,
                    generalized_free_acceleration: &acceleration,
                    initial_contact_free_acceleration: &[0.0; 3],
                    impulse_upper: &[1.0; 3],
                    friction: &[0.5],
                    time_constant_s: &[0.02],
                    damping_ratio: &[1.0],
                    impedance_min: &[0.8],
                    impedance_max: &[0.9],
                    impedance_width_m: &[0.001],
                    impedance_midpoint: &[0.5],
                    impedance_power: &[2.0],
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.005,
                    state_steps: 0,
                    compliance_substeps: 8,
                    projection_sweeps: 8,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::ImplicitEuler,
                },
                &mut scratch,
                &mut impulse,
                &mut velocity,
                &mut gap,
                &mut after,
            ),
            Err(ModelCoupledPositiveReferenceContactError::InvalidConfig)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(velocity, [8.0; 3]);
        assert_eq!(gap, [9.0]);
    }

    #[test]
    fn coupled_solver_preserves_cross_contact_normal_response() {
        let axes = 6;
        let mut delassus = [0.0; 36];
        for axis in 0..axes {
            delassus[axis * axes + axis] = 1.0;
        }
        delassus[2 * axes + 5] = 0.5;
        delassus[5 * axes + 2] = 0.5;
        let velocity = [0.0, 0.0, -1.0, 0.0, 0.0, -1.0];
        let upper = [0.0, 0.0, 10.0, 0.0, 0.0, 10.0];
        let mut impulse = [0.0; 6];
        let mut after = [0.0; 6];
        solve_coupled_contact_impulse(
            CoupledContactImpulseInput {
                contact_velocity: &velocity,
                delassus: &delassus,
                impulse_upper: &upper,
                friction: &[0.0, 0.0],
                restitution: 0.0,
                diagonal_regularization_ratio: 0.0,
                sweeps: 16,
            },
            &mut impulse,
            &mut after,
        )
        .unwrap();
        assert!((impulse[2] - 2.0 / 3.0).abs() < 1.0e-9);
        assert!((impulse[5] - 2.0 / 3.0).abs() < 1.0e-9);
        assert!(after[2].abs() < 1.0e-9);
        assert!(after[5].abs() < 1.0e-9);
    }

    #[test]
    fn coupled_solver_projects_tangent_to_current_coulomb_disk() {
        let axes = 3;
        let mut delassus = [0.0; 9];
        for axis in 0..axes {
            delassus[axis * axes + axis] = 1.0;
        }
        let mut impulse = [0.0; 3];
        let mut after = [0.0; 3];
        solve_coupled_contact_impulse(
            CoupledContactImpulseInput {
                contact_velocity: &[-2.0, -2.0, -1.0],
                delassus: &delassus,
                impulse_upper: &[10.0, 10.0, 10.0],
                friction: &[0.5],
                restitution: 0.0,
                diagonal_regularization_ratio: 0.0,
                sweeps: 2,
            },
            &mut impulse,
            &mut after,
        )
        .unwrap();
        assert!((impulse[2] - 1.0).abs() < 1.0e-12);
        assert!((impulse[0].hypot(impulse[1]) - 0.5).abs() < 1.0e-12);
        assert!(after.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn coupled_solver_rejects_asymmetry_before_outputs_change() {
        let mut impulse = [7.0; 3];
        let mut after = [8.0; 3];
        assert_eq!(
            solve_coupled_contact_impulse(
                CoupledContactImpulseInput {
                    contact_velocity: &[0.0; 3],
                    delassus: &[1.0, 0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    impulse_upper: &[1.0; 3],
                    friction: &[0.5],
                    restitution: 0.0,
                    diagonal_regularization_ratio: 0.0,
                    sweeps: 1,
                },
                &mut impulse,
                &mut after,
            ),
            Err(CoupledContactImpulseError::InvalidDelassus)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(after, [8.0; 3]);
    }

    #[test]
    fn coupled_solver_diagonal_regularization_is_explicit_compliance() {
        let mut impulse = [0.0; 3];
        let mut after = [0.0; 3];
        solve_coupled_contact_impulse(
            CoupledContactImpulseInput {
                contact_velocity: &[0.0, 0.0, -1.0],
                delassus: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                impulse_upper: &[0.0, 0.0, 10.0],
                friction: &[0.0],
                restitution: 0.0,
                diagonal_regularization_ratio: 1.0,
                sweeps: 1,
            },
            &mut impulse,
            &mut after,
        )
        .unwrap();
        assert_eq!(impulse, [0.0, 0.0, 0.5]);
        assert_eq!(after, [0.0, 0.0, -0.5]);
    }

    #[test]
    fn coupled_hypothesis_envelope_matches_independent_solutions() {
        let delassus = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let velocities = [0.0, 0.0, -1.0, 0.0, 0.0, -2.0];
        let uppers = [0.0, 0.0, 10.0, 0.0, 0.0, 10.0];
        let response = [
            0.0, 0.0, 1.0, // coordinate 0
            0.0, 0.0, -2.0, // coordinate 1
        ];
        let mut impulse = [0.0; 3];
        let mut after = [0.0; 3];
        let mut delta = [0.0; 2];
        let mut lower = [7.0; 2];
        let mut upper = [8.0; 2];
        write_coupled_contact_hypothesis_velocity_envelope(
            CoupledContactHypothesisEnvelopeInput {
                hypothesis_count: 2,
                contact_velocity_hypotheses: &velocities,
                delassus: &delassus,
                impulse_upper_hypotheses: &uppers,
                friction_hypotheses: &[0.0, 0.0],
                restitution_hypotheses: &[0.0, 0.0],
                diagonal_regularization_ratio_hypotheses: &[0.0, 0.0],
                impulse_velocity_response: &response,
                sweeps: 2,
            },
            &mut impulse,
            &mut after,
            &mut delta,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert_eq!(lower, [1.0, -4.0]);
        assert_eq!(upper, [2.0, -2.0]);
    }

    #[test]
    fn coupled_hypothesis_envelope_rejects_late_invalid_hypothesis_atomically() {
        let mut impulse = [0.0; 3];
        let mut after = [0.0; 3];
        let mut delta = [0.0; 1];
        let mut lower = [7.0];
        let mut upper = [8.0];
        assert_eq!(
            write_coupled_contact_hypothesis_velocity_envelope(
                CoupledContactHypothesisEnvelopeInput {
                    hypothesis_count: 2,
                    contact_velocity_hypotheses: &[0.0, 0.0, -1.0, 0.0, 0.0, f64::NAN],
                    delassus: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    impulse_upper_hypotheses: &[10.0; 6],
                    friction_hypotheses: &[0.5, 0.5],
                    restitution_hypotheses: &[0.0, 0.0],
                    diagonal_regularization_ratio_hypotheses: &[0.0, 0.0],
                    impulse_velocity_response: &[0.0, 0.0, 1.0],
                    sweeps: 1,
                },
                &mut impulse,
                &mut after,
                &mut delta,
                &mut lower,
                &mut upper,
            ),
            Err(CoupledContactImpulseError::InvalidWitness)
        );
        assert_eq!(lower, [7.0]);
        assert_eq!(upper, [8.0]);
    }

    #[test]
    fn positive_reference_impedance_and_cone_sections_match_declared_geometry() {
        assert_eq!(
            positive_reference_impedance(0.0, 0.8, 0.96, 0.001, 0.5, 2.0),
            0.8
        );
        assert_eq!(
            positive_reference_impedance(-0.001, 0.8, 0.96, 0.001, 0.5, 2.0),
            0.96
        );
        assert!(
            (positive_reference_impedance(-0.0005, 0.8, 0.96, 0.001, 0.5, 2.0) - 0.88).abs()
                < 1.0e-12
        );
        let circular = project_tangent_impulse(1.0, 1.0, 1.0, CompliantFrictionCone::Circular);
        let pyramidal = project_tangent_impulse(1.0, 1.0, 1.0, CompliantFrictionCone::Pyramidal);
        assert!((circular.0 - 2.0_f64.sqrt().recip()).abs() < 1.0e-12);
        assert_eq!(pyramidal, (0.5, 0.5));
        assert!((pyramidal.0.abs() + pyramidal.1.abs() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn positive_reference_contact_is_position_dependent_and_atomic() {
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let velocity = [0.3, -0.2, 0.0];
        let acceleration = [0.0; 3];
        let upper = [10.0; 3];
        let friction = [0.5];
        let mass = [2.0];
        let time_constant = [0.02];
        let damping_ratio = [1.0];
        let impedance_min = [0.8];
        let impedance_max = [0.96];
        let impedance_width = [0.001];
        let midpoint = [0.5];
        let power = [2.0];
        let run = |gap_value: f64, impulse: &mut [f64; 3]| {
            let gap = [gap_value];
            let mut step = [0.0; 3];
            let mut after = [0.0; 3];
            let mut gap_after = [0.0];
            solve_positive_reference_compliant_contact_impulse(
                PositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &gap,
                    contact_velocity: &velocity,
                    contact_free_acceleration: &acceleration,
                    delassus: &identity,
                    impulse_upper: &upper,
                    friction: &friction,
                    effective_normal_mass: &mass,
                    time_constant_s: &time_constant,
                    damping_ratio: &damping_ratio,
                    impedance_min: &impedance_min,
                    impedance_max: &impedance_max,
                    impedance_width_m: &impedance_width,
                    impedance_midpoint: &midpoint,
                    impedance_power: &power,
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.005,
                    substeps: 8,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::ImplicitEuler,
                },
                &mut step,
                impulse,
                &mut after,
                &mut gap_after,
            )
            .unwrap();
        };
        let mut shallow = [0.0; 3];
        let mut deep = [0.0; 3];
        run(-1.0e-6, &mut shallow);
        run(-0.001, &mut deep);
        assert!(deep[2] > shallow[2]);
        assert!(deep[0].hypot(deep[1]) <= 0.5 * deep[2] + 1.0e-12);

        let mut step = [0.0; 3];
        let mut impulse = [7.0; 3];
        let mut after = [8.0; 3];
        let mut gap_after = [9.0];
        let invalid_max = [1.0];
        assert_eq!(
            solve_positive_reference_compliant_contact_impulse(
                PositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &[-0.001],
                    contact_velocity: &velocity,
                    contact_free_acceleration: &acceleration,
                    delassus: &identity,
                    impulse_upper: &upper,
                    friction: &friction,
                    effective_normal_mass: &mass,
                    time_constant_s: &time_constant,
                    damping_ratio: &damping_ratio,
                    impedance_min: &impedance_min,
                    impedance_max: &invalid_max,
                    impedance_width_m: &impedance_width,
                    impedance_midpoint: &midpoint,
                    impedance_power: &power,
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.005,
                    substeps: 8,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::ImplicitEuler,
                },
                &mut step,
                &mut impulse,
                &mut after,
                &mut gap_after,
            ),
            Err(CoupledContactImpulseError::InvalidWitness)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(after, [8.0; 3]);
        assert_eq!(gap_after, [9.0]);
    }

    #[test]
    fn scalar_exponential_trapezoidal_is_distinct_from_model_rk4() {
        let delassus = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let gap = [-0.001];
        let velocity = [0.1, -0.2, -0.3];
        let acceleration = [0.0, 0.0, -0.4];
        let upper = [10.0; 3];
        let friction = [0.5];
        let effective_mass = [1.0];
        let time_constant = [0.02];
        let damping_ratio = [1.0];
        let impedance_min = [0.8];
        let impedance_max = [0.96];
        let impedance_width = [0.001];
        let impedance_midpoint = [0.5];
        let impedance_power = [2.0];
        let run = |integrator| {
            let mut scratch = [0.0; 3];
            let mut impulse = [0.0; 3];
            let mut after = [0.0; 3];
            let mut gap_after = [0.0];
            solve_positive_reference_compliant_contact_impulse(
                PositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &gap,
                    contact_velocity: &velocity,
                    contact_free_acceleration: &acceleration,
                    delassus: &delassus,
                    impulse_upper: &upper,
                    friction: &friction,
                    effective_normal_mass: &effective_mass,
                    time_constant_s: &time_constant,
                    damping_ratio: &damping_ratio,
                    impedance_min: &impedance_min,
                    impedance_max: &impedance_max,
                    impedance_width_m: &impedance_width,
                    impedance_midpoint: &impedance_midpoint,
                    impedance_power: &impedance_power,
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.001,
                    substeps: 4,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator,
                },
                &mut scratch,
                &mut impulse,
                &mut after,
                &mut gap_after,
            )
            .map(|()| (impulse, after, gap_after))
        };

        let first = run(CompliantStepIntegrator::ExponentialTrapezoidal).unwrap();
        let repeat = run(CompliantStepIntegrator::ExponentialTrapezoidal).unwrap();
        assert_eq!(repeat, first);

        let mut rejected_impulse = [7.0; 3];
        let mut rejected_after = [8.0; 3];
        let mut rejected_gap_after = [9.0];
        let mut rejected_scratch = [0.0; 3];
        assert_eq!(
            solve_positive_reference_compliant_contact_impulse(
                PositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &gap,
                    contact_velocity: &velocity,
                    contact_free_acceleration: &acceleration,
                    delassus: &delassus,
                    impulse_upper: &upper,
                    friction: &friction,
                    effective_normal_mass: &effective_mass,
                    time_constant_s: &time_constant,
                    damping_ratio: &damping_ratio,
                    impedance_min: &impedance_min,
                    impedance_max: &impedance_max,
                    impedance_width_m: &impedance_width,
                    impedance_midpoint: &impedance_midpoint,
                    impedance_power: &impedance_power,
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.001,
                    substeps: 4,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::GeneralizedRk4,
                },
                &mut rejected_scratch,
                &mut rejected_impulse,
                &mut rejected_after,
                &mut rejected_gap_after,
            ),
            Err(CoupledContactImpulseError::InvalidConfig)
        );
        assert_eq!(rejected_impulse, [7.0; 3]);
        assert_eq!(rejected_after, [8.0; 3]);
        assert_eq!(rejected_gap_after, [9.0]);
    }

    #[test]
    fn positive_reference_contact_matches_documented_free_acceleration_equation() {
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let upper = [10.0; 3];
        let friction = [0.5];
        let mass = [1.0];
        let time_constant = [0.02];
        let damping_ratio = [1.0];
        let impedance = [0.8];
        let width = [0.001];
        let midpoint = [0.5];
        let power = [2.0];
        let run = |gap: f64,
                   velocity: [f64; 3],
                   acceleration: [f64; 3],
                   time_step_s: f64|
         -> ([f64; 3], [f64; 3]) {
            let gap = [gap];
            let mut step = [0.0; 3];
            let mut impulse = [0.0; 3];
            let mut after = [0.0; 3];
            let mut gap_after = [0.0];
            solve_positive_reference_compliant_contact_impulse(
                PositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &gap,
                    contact_velocity: &velocity,
                    contact_free_acceleration: &acceleration,
                    delassus: &identity,
                    impulse_upper: &upper,
                    friction: &friction,
                    effective_normal_mass: &mass,
                    time_constant_s: &time_constant,
                    damping_ratio: &damping_ratio,
                    impedance_min: &impedance,
                    impedance_max: &impedance,
                    impedance_width_m: &width,
                    impedance_midpoint: &midpoint,
                    impedance_power: &power,
                    minimum_time_constant_s: 1.0e-8,
                    time_step_s,
                    substeps: 1,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::ExplicitEuler,
                },
                &mut step,
                &mut impulse,
                &mut after,
                &mut gap_after,
            )
            .unwrap();
            (impulse, after)
        };

        // MuJoCo's documented constant-impedance equilibrium is
        // r = a_free (1-d) tau² zeta². Compensate the predictor's half-step
        // free-acceleration position term so it evaluates that exact r.
        let time_step_s = 1.0e-6;
        let free_acceleration = -9.81;
        let equilibrium_gap = free_acceleration * (1.0 - 0.8) * 0.02_f64.powi(2);
        let initial_gap = equilibrium_gap - 0.5 * time_step_s * time_step_s * free_acceleration;
        let (equilibrium_impulse, equilibrium_after) = run(
            initial_gap,
            [0.0; 3],
            [0.0, 0.0, free_acceleration],
            time_step_s,
        );
        assert!((equilibrium_impulse[2] - 9.81 * time_step_s).abs() < 1.0e-12);
        assert!(equilibrium_after[2].abs() < 1.0e-12);

        // Damping is signed: sufficiently fast separation releases a shallow
        // contact instead of retaining a spring-only attractive force.
        let (separating_impulse, _) = run(-1.0e-6, [0.0, 0.0, 1.0], [0.0; 3], 1.0e-6);
        assert_eq!(separating_impulse[2], 0.0);

        // At zero friction velocity, impedance interpolates free tangent
        // acceleration instead of letting the full unconstrained step leak
        // through the contact solve.
        let (_, tangent_after) = run(-0.001, [0.0; 3], [1.0, 0.0, 0.0], 1.0e-4);
        assert!((tangent_after[0] - 2.0e-5).abs() < 1.0e-12);
    }

    #[test]
    fn substepped_compliance_evolves_gap_velocity_and_bounded_impulse() {
        let input = CompliantContactImpulseInput {
            contact_gap: &[-0.01],
            contact_velocity: &[0.0, 0.0, -1.0],
            delassus: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            impulse_upper: &[1.0, 1.0, 1.0],
            friction: &[0.5],
            normal_stiffness: &[100.0],
            normal_damping: &[0.0],
            time_step_s: 0.01,
            substeps: 1,
        };
        let mut step = [0.0; 3];
        let mut impulse = [0.0; 3];
        let mut velocity = [0.0; 3];
        let mut gap = [0.0];
        solve_substepped_compliant_contact_impulse(
            input,
            &mut step,
            &mut impulse,
            &mut velocity,
            &mut gap,
        )
        .unwrap();
        assert!((impulse[2] - 0.02).abs() < 1.0e-12);
        assert!((velocity[2] + 0.98).abs() < 1.0e-12);
        assert!((gap[0] + 0.0198).abs() < 1.0e-12);
        assert_eq!(impulse[0].hypot(impulse[1]), 0.0);

        let mut repeat_impulse = [0.0; 3];
        let mut repeat_velocity = [0.0; 3];
        let mut repeat_gap = [0.0];
        solve_substepped_compliant_contact_impulse(
            input,
            &mut step,
            &mut repeat_impulse,
            &mut repeat_velocity,
            &mut repeat_gap,
        )
        .unwrap();
        assert_eq!(repeat_impulse, impulse);
        assert_eq!(repeat_velocity, velocity);
        assert_eq!(repeat_gap, gap);
    }

    #[test]
    fn substepped_compliance_rejects_invalid_model_atomically() {
        let mut step = [0.0; 3];
        let mut impulse = [7.0; 3];
        let mut velocity = [8.0; 3];
        let mut gap = [9.0];
        assert_eq!(
            solve_substepped_compliant_contact_impulse(
                CompliantContactImpulseInput {
                    contact_gap: &[0.0],
                    contact_velocity: &[0.0, 0.0, -1.0],
                    delassus: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    impulse_upper: &[1.0; 3],
                    friction: &[0.5],
                    normal_stiffness: &[f64::NAN],
                    normal_damping: &[1.0],
                    time_step_s: 0.01,
                    substeps: 4,
                },
                &mut step,
                &mut impulse,
                &mut velocity,
                &mut gap,
            ),
            Err(CoupledContactImpulseError::InvalidWitness)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(velocity, [8.0; 3]);
        assert_eq!(gap, [9.0]);
    }

    #[test]
    fn response_rejects_non_orthonormal_basis_before_outputs_change() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let model = crate::urdf::load_urdf(source).unwrap();
        let state = RobotState::zeros(&model);
        let contact = PointImpulseResponseSpec {
            frame: FrameId(model.root.0),
            point_world: crate::math::Vec3::zeros(),
            basis_world: [
                crate::math::Vec3::x(),
                crate::math::Vec3::x(),
                crate::math::Vec3::z(),
            ],
        };
        let mut scratch = ContactTransitionResponseScratch::new(&model);
        let mut response = vec![7.0; (model.dof + 6) * 3];
        let mut effective_mass = [8.0; 3];
        assert_eq!(
            write_point_impulse_velocity_response(
                &model,
                &state,
                &[contact],
                &mut scratch,
                &mut response,
                &mut effective_mass,
            ),
            Err(ContactTransitionResponseError::InvalidContact)
        );
        assert!(response.iter().all(|value| *value == 7.0));
        assert_eq!(effective_mass, [8.0; 3]);
    }

    #[test]
    fn acceleration_interval_checks_all_time_endpoint_products() {
        let mut impulse = [0.0; 3];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_contact_transition_acceleration_interval_bounds(
            ContactTransitionAccelerationIntervalInput {
                transition_time_lower_s: 0.01,
                transition_time_upper_s: 0.02,
                restitution_upper: 0.0,
                contact_witnesses: &[0.0, 1.0, 0.0, 0.0],
                generalized_acceleration_lower: &[-4.0],
                generalized_acceleration_upper: &[6.0],
                impulse_velocity_response: &[0.0; 3],
            },
            &mut impulse,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert_eq!(impulse, [0.0; 3]);
        assert!((lower[0] - -0.08).abs() < 1.0e-12);
        assert!((upper[0] - 0.12).abs() < 1.0e-12);
    }

    #[test]
    fn acceleration_interval_rejects_inversion_before_output_changes() {
        let mut impulse = [7.0; 3];
        let mut lower = [8.0; 1];
        let mut upper = [9.0; 1];
        assert_eq!(
            write_contact_transition_acceleration_interval_bounds(
                ContactTransitionAccelerationIntervalInput {
                    transition_time_lower_s: 0.0,
                    transition_time_upper_s: 0.01,
                    restitution_upper: 1.0,
                    contact_witnesses: &[1.0, 1.0, 0.0, 0.0],
                    generalized_acceleration_lower: &[2.0],
                    generalized_acceleration_upper: &[1.0],
                    impulse_velocity_response: &[0.0; 3],
                },
                &mut impulse,
                &mut lower,
                &mut upper,
            ),
            Err(ContactTransitionError::InvalidResponse)
        );
        assert_eq!(impulse, [7.0; 3]);
        assert_eq!(lower, [8.0]);
        assert_eq!(upper, [9.0]);
    }

    #[test]
    fn directional_tangent_uses_tighter_slip_or_friction_witness() {
        let mut impulse = [0.0; 3];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_directional_contact_transition_bounds(
            DirectionalContactTransitionInput {
                transition_time_lower_s: 0.01,
                transition_time_upper_s: 0.02,
                restitution_upper: 0.5,
                contact_witnesses: &[
                    0.1, 3.0, 1.0, // speed t_x, t_y, n
                    1.0, 5.0, 6.0, // effective mass t_x, t_y, n
                    0.0, 20.0, 30.0, // sustained force t_x, t_y, n
                    0.5,
                ],
                generalized_acceleration_lower: &[0.0],
                generalized_acceleration_upper: &[0.0],
                impulse_velocity_response: &[1.0, 1.0, 0.0],
            },
            &mut impulse,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        // Normal = 6 * 1.5 * 1 + 30 * .02 = 9.6, hence Coulomb = 4.8.
        // X is slip-limited at 0.1; Y is Coulomb-limited at 4.8.
        assert!((impulse[0] - 0.1).abs() < 1.0e-12);
        assert!((impulse[1] - 4.8).abs() < 1.0e-12);
        assert!((impulse[2] - 9.6).abs() < 1.0e-12);
        assert!((lower[0] - -4.9).abs() < 1.0e-12);
        assert!((upper[0] - 4.9).abs() < 1.0e-12);
    }

    #[test]
    fn spatial_patch_ties_force_and_moment_capacity_to_one_normal_impulse() {
        let mut normal = [0.0; 1];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_spatial_patch_contact_transition_bounds(
            SpatialPatchTransitionInput {
                transition_time_lower_s: 0.005,
                transition_time_upper_s: 0.010,
                restitution_upper: 0.5,
                patch_witnesses: &[
                    10.0, 10.0, 1.0, // speed XYZ
                    1.0, 1.0, 2.0, // effective mass XYZ
                    0.0, 0.0, 10.0, // sustained force XYZ
                    0.4, 0.2, 0.1, 0.05, // friction, half X/Y, torsion radius
                ],
                generalized_acceleration_lower: &[-2.0],
                generalized_acceleration_upper: &[4.0],
                spatial_impulse_velocity_response: &[1.0, -2.0, 0.5, 3.0, -4.0, 0.25],
            },
            &mut normal,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert!((normal[0] - 3.1).abs() < 1.0e-12);
        assert!((lower[0] - -9.5525).abs() < 1.0e-12);
        assert!((upper[0] - 11.1225).abs() < 1.0e-12);
    }

    #[test]
    fn spatial_patch_checks_the_slip_saturation_kink() {
        let mut normal = [0.0; 1];
        let mut lower = [0.0; 1];
        let mut upper = [0.0; 1];
        write_spatial_patch_contact_transition_bounds(
            SpatialPatchTransitionInput {
                transition_time_lower_s: 0.0,
                transition_time_upper_s: 0.005,
                restitution_upper: 1.0,
                patch_witnesses: &[
                    1.0, 0.0, 5.0, // speed x, y, normal
                    1.0, 1.0, 1.0, // effective mass x, y, normal
                    0.0, 0.0, 0.0, // sustained force x, y, normal
                    1.0, 0.0, 0.0, 0.0, // friction and moment radii
                ],
                generalized_acceleration_lower: &[0.0],
                generalized_acceleration_upper: &[0.0],
                // f(j) = 0.5 j +/- min(j, 1). The lower endpoint occurs
                // at the slip-cap kink j=1, not at j=0 or j=10.
                spatial_impulse_velocity_response: &[0.0, 0.0, 0.0, 1.0, 0.0, 0.5],
            },
            &mut normal,
            &mut lower,
            &mut upper,
        )
        .unwrap();
        assert!((normal[0] - 10.0).abs() < 1.0e-12);
        assert!((lower[0] - -0.5).abs() < 1.0e-12);
        assert!((upper[0] - 6.0).abs() < 1.0e-12);
    }

    #[test]
    fn spatial_patch_rejects_inverted_acceleration_atomically() {
        let mut normal = [7.0; 1];
        let mut lower = [8.0; 1];
        let mut upper = [9.0; 1];
        assert_eq!(
            write_spatial_patch_contact_transition_bounds(
                SpatialPatchTransitionInput {
                    transition_time_lower_s: 0.0,
                    transition_time_upper_s: 0.005,
                    restitution_upper: 1.0,
                    patch_witnesses: &[
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.5, 0.1, 0.1, 0.02,
                    ],
                    generalized_acceleration_lower: &[2.0],
                    generalized_acceleration_upper: &[1.0],
                    spatial_impulse_velocity_response: &[0.0; 6],
                },
                &mut normal,
                &mut lower,
                &mut upper,
            ),
            Err(ContactTransitionError::InvalidResponse)
        );
        assert_eq!(normal, [7.0]);
        assert_eq!(lower, [8.0]);
        assert_eq!(upper, [9.0]);
    }

    #[test]
    fn coupled_positive_reference_distributes_soft_response_through_delassus() {
        let axes = 6;
        let mut delassus = [0.0; 36];
        for axis in 0..axes {
            delassus[axis * axes + axis] = 1.0;
        }
        delassus[2 * axes + 5] = 0.5;
        delassus[5 * axes + 2] = 0.5;
        let gap = [-0.001; 2];
        let velocity = [0.0; 6];
        let acceleration = [0.0; 6];
        let upper = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let friction = [0.0; 2];
        let time_constant = [0.02; 2];
        let damping_ratio = [1.0; 2];
        let impedance = [0.8; 2];
        let width = [0.001; 2];
        let midpoint = [0.5; 2];
        let power = [2.0; 2];
        let mut desired = [0.0; 6];
        let mut step = [0.0; 6];
        let mut coupled_impulse = [0.0; 6];
        let mut coupled_after = [0.0; 6];
        let mut coupled_gap = [0.0; 2];
        solve_coupled_positive_reference_compliant_contact_impulse(
            CoupledPositiveReferenceCompliantContactImpulseInput {
                contact_gap: &gap,
                contact_velocity: &velocity,
                contact_free_acceleration: &acceleration,
                delassus: &delassus,
                impulse_upper: &upper,
                friction: &friction,
                time_constant_s: &time_constant,
                damping_ratio: &damping_ratio,
                impedance_min: &impedance,
                impedance_max: &impedance,
                impedance_width_m: &width,
                impedance_midpoint: &midpoint,
                impedance_power: &power,
                minimum_time_constant_s: 0.002,
                time_step_s: 0.001,
                substeps: 1,
                projection_sweeps: 32,
                friction_cone: CompliantFrictionCone::Circular,
                integrator: CompliantStepIntegrator::ExplicitEuler,
            },
            &mut desired,
            &mut step,
            &mut coupled_impulse,
            &mut coupled_after,
            &mut coupled_gap,
        )
        .unwrap();

        let mut independent_step = [0.0; 6];
        let mut independent_impulse = [0.0; 6];
        let mut independent_after = [0.0; 6];
        let mut independent_gap = [0.0; 2];
        solve_positive_reference_compliant_contact_impulse(
            PositiveReferenceCompliantContactImpulseInput {
                contact_gap: &gap,
                contact_velocity: &velocity,
                contact_free_acceleration: &acceleration,
                delassus: &delassus,
                impulse_upper: &upper,
                friction: &friction,
                effective_normal_mass: &[1.0; 2],
                time_constant_s: &time_constant,
                damping_ratio: &damping_ratio,
                impedance_min: &impedance,
                impedance_max: &impedance,
                impedance_width_m: &width,
                impedance_midpoint: &midpoint,
                impedance_power: &power,
                minimum_time_constant_s: 0.002,
                time_step_s: 0.001,
                substeps: 1,
                friction_cone: CompliantFrictionCone::Circular,
                integrator: CompliantStepIntegrator::ExplicitEuler,
            },
            &mut independent_step,
            &mut independent_impulse,
            &mut independent_after,
            &mut independent_gap,
        )
        .unwrap();

        let target_delta = desired[2];
        let coupled_error =
            (coupled_after[2] - target_delta).abs() + (coupled_after[5] - target_delta).abs();
        let independent_error = (independent_after[2] - target_delta).abs()
            + (independent_after[5] - target_delta).abs();
        assert!(coupled_error < 1.0e-12);
        assert!(coupled_error < independent_error);
        assert!((coupled_impulse[2] - 2.0 * target_delta / 3.0).abs() < 1.0e-12);
        assert!((coupled_impulse[5] - 2.0 * target_delta / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn coupled_positive_reference_rejects_bad_sweeps_atomically() {
        let mut desired = [6.0; 3];
        let mut step = [7.0; 3];
        let mut impulse = [8.0; 3];
        let mut after = [9.0; 3];
        let mut gap_after = [10.0];
        assert_eq!(
            solve_coupled_positive_reference_compliant_contact_impulse(
                CoupledPositiveReferenceCompliantContactImpulseInput {
                    contact_gap: &[-0.001],
                    contact_velocity: &[0.0; 3],
                    contact_free_acceleration: &[0.0; 3],
                    delassus: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    impulse_upper: &[1.0; 3],
                    friction: &[0.5],
                    time_constant_s: &[0.02],
                    damping_ratio: &[1.0],
                    impedance_min: &[0.8],
                    impedance_max: &[0.9],
                    impedance_width_m: &[0.001],
                    impedance_midpoint: &[0.5],
                    impedance_power: &[2.0],
                    minimum_time_constant_s: 0.002,
                    time_step_s: 0.001,
                    substeps: 1,
                    projection_sweeps: 0,
                    friction_cone: CompliantFrictionCone::Circular,
                    integrator: CompliantStepIntegrator::ExplicitEuler,
                },
                &mut desired,
                &mut step,
                &mut impulse,
                &mut after,
                &mut gap_after,
            ),
            Err(CoupledContactImpulseError::InvalidConfig)
        );
        assert_eq!(desired, [6.0; 3]);
        assert_eq!(step, [7.0; 3]);
        assert_eq!(impulse, [8.0; 3]);
        assert_eq!(after, [9.0; 3]);
        assert_eq!(gap_after, [10.0]);
    }
}
