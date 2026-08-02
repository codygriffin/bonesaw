//! Allocation-free contact-transition impulse and velocity-jump bounds.
//!
//! This module does not infer contact, friction, compliance, or effective mass.
//! The caller supplies conservative pre-impact witnesses and a generalized
//! velocity response `M^-1 J^T`. The result is a componentwise outer bound,
//! not a complementarity solve, impact-law identification, or safety claim.

use nalgebra::{DMatrix, DVector, Point3};

use crate::model::{CompiledModel, DynamicsCache, FrameId, ModelCache, ModelError, RobotState};

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
}
