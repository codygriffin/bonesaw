use nalgebra::{DMatrix, DVector, RowDVector};
use serde::{Deserialize, Serialize};

const HARD_CONSTRAINT_TOLERANCE: f64 = 1e-8;
const FEASIBILITY_SEED_TOLERANCE: f64 = HARD_CONSTRAINT_TOLERANCE;
const FEASIBILITY_POLISH_TRIGGER: f64 = 1e-3;
const FEASIBILITY_ACTIVE_BAND: f64 = 5e-3;
const CACHE_FEASIBILITY_ROW_NORMS: bool = true;
const SKIP_ZERO_FEASIBILITY_UPDATES: bool = true;
const USE_EUCLIDEAN_ACTIVE_SET_FEASIBILITY: bool = false;
const DYKSTRA_SWEEP_LIMIT_BEFORE_ACTIVE_SET: Option<usize> = Some(8);
const ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT: usize = 64;
const CACHE_JACOBI_COLUMN_ENERGIES: bool = true;
const SKIP_DISCARDED_JACOBI_COUPLINGS: bool = cfg!(feature = "jacobi-discarded-pair-experiment");
const FUSE_INITIAL_JACOBI_ENERGY_SCAN: bool = cfg!(feature = "jacobi-energy-scan-experiment");
const USE_JACOBI_COLUMN_SLICES: bool = !cfg!(feature = "jacobi-column-slice-control")
    || cfg!(feature = "jacobi-column-slice-experiment");
const USE_JACOBI_COLUMN_ITERATORS: bool = cfg!(feature = "jacobi-column-iterator-experiment");
const USE_DENSE_MULTIPLY_ROW_SLICES: bool = !cfg!(feature = "dense-multiply-row-slice-control")
    || cfg!(feature = "dense-multiply-row-slice-experiment");

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[repr(u8)]
pub enum Priority {
    Invariant = 0,
    Viability = 1,
    Intent = 2,
    Preference = 3,
    Style = 4,
}

impl Priority {
    pub const ALL: [Self; 5] = [
        Self::Invariant,
        Self::Viability,
        Self::Intent,
        Self::Preference,
        Self::Style,
    ];
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum TaskKind {
    Point,
    Orientation,
    Gaze,
    Repeller,
    CenterOfMass,
    CentroidalMomentum,
    Posture,
    Velocity,
    Acceleration,
    ActuatorTorque,
    ContactWrench,
}

#[derive(Clone, Debug)]
pub struct Task {
    pub stable_id: u32,
    pub kind: TaskKind,
    pub priority: Priority,
    pub jacobian: DMatrix<f64>,
    pub target_velocity: DVector<f64>,
    pub weight: f64,
}

/// Compiler-sized reusable task slots grouped by their row shape.
///
/// Point/orientation/CoM tasks use three-row slots, collision repellers use
/// one-row slots, and posture has one `dof × dof` slot. Active task indices
/// preserve emission order without moving or dropping matrix allocations.
#[derive(Clone, Debug)]
pub struct TaskBuffer {
    tasks: Vec<Task>,
    active_indices: Vec<usize>,
    three_row_start: usize,
    three_row_capacity: usize,
    three_row_used: usize,
    one_row_start: usize,
    one_row_capacity: usize,
    one_row_used: usize,
    posture_index: usize,
    posture_used: bool,
}

impl TaskBuffer {
    pub fn new(dof: usize, three_row_capacity: usize, one_row_capacity: usize) -> Self {
        let total = three_row_capacity
            .saturating_add(one_row_capacity)
            .saturating_add(1);
        let mut tasks = Vec::with_capacity(total);
        for _ in 0..three_row_capacity {
            tasks.push(empty_task(3, dof));
        }
        let one_row_start = tasks.len();
        for _ in 0..one_row_capacity {
            tasks.push(empty_task(1, dof));
        }
        let posture_index = tasks.len();
        tasks.push(empty_task(dof, dof));
        Self {
            tasks,
            active_indices: Vec::with_capacity(total),
            three_row_start: 0,
            three_row_capacity,
            three_row_used: 0,
            one_row_start,
            one_row_capacity,
            one_row_used: 0,
            posture_index,
            posture_used: false,
        }
    }

    pub fn begin(&mut self) {
        self.active_indices.clear();
        self.three_row_used = 0;
        self.one_row_used = 0;
        self.posture_used = false;
    }

    pub fn push_three(&mut self) -> Option<&mut Task> {
        if self.three_row_used >= self.three_row_capacity {
            return None;
        }
        let index = self.three_row_start + self.three_row_used;
        self.three_row_used += 1;
        self.activate(index)
    }

    pub fn push_one(&mut self) -> Option<&mut Task> {
        if self.one_row_used >= self.one_row_capacity {
            return None;
        }
        let index = self.one_row_start + self.one_row_used;
        self.one_row_used += 1;
        self.activate(index)
    }

    pub fn push_posture(&mut self) -> Option<&mut Task> {
        if self.posture_used {
            return None;
        }
        self.posture_used = true;
        let index = self.posture_index;
        self.activate(index)
    }

    pub fn tasks(&self) -> &[Task] {
        &self.tasks
    }

    pub fn active_indices(&self) -> &[usize] {
        &self.active_indices
    }

    pub fn active_len(&self) -> usize {
        self.active_indices.len()
    }

    pub fn three_row_capacity(&self) -> usize {
        self.three_row_capacity
    }

    fn activate(&mut self, index: usize) -> Option<&mut Task> {
        self.active_indices.push(index);
        let task = self.tasks.get_mut(index)?;
        task.jacobian.fill(0.0);
        task.target_velocity.fill(0.0);
        task.weight = 0.0;
        Some(task)
    }
}

fn empty_task(rows: usize, dof: usize) -> Task {
    Task {
        stable_id: 0,
        kind: TaskKind::Velocity,
        priority: Priority::Style,
        jacobian: DMatrix::zeros(rows, dof),
        target_velocity: DVector::zeros(rows),
        weight: 0.0,
    }
}

#[derive(Clone, Debug)]
pub struct VelocityBounds {
    pub lower: DVector<f64>,
    pub upper: DVector<f64>,
}

#[derive(Clone, Debug)]
pub struct LinearConstraint {
    pub stable_id: u32,
    pub coefficients: RowDVector<f64>,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug)]
pub struct ConstraintBuffer {
    constraints: Vec<LinearConstraint>,
    active_len: usize,
}

impl ConstraintBuffer {
    pub fn new(dof: usize, capacity: usize) -> Self {
        let mut constraints = Vec::with_capacity(capacity);
        for _ in 0..capacity {
            constraints.push(LinearConstraint {
                stable_id: 0,
                coefficients: RowDVector::zeros(dof),
                lower: f64::NEG_INFINITY,
                upper: f64::INFINITY,
            });
        }
        Self {
            constraints,
            active_len: 0,
        }
    }

    pub fn begin(&mut self) {
        self.active_len = 0;
    }

    pub fn push(&mut self) -> Option<&mut LinearConstraint> {
        let index = self.active_len;
        let constraint = self.constraints.get_mut(index)?;
        self.active_len += 1;
        constraint.coefficients.fill(0.0);
        constraint.lower = f64::NEG_INFINITY;
        constraint.upper = f64::INFINITY;
        Some(constraint)
    }

    pub fn active(&self) -> &[LinearConstraint] {
        &self.constraints[..self.active_len]
    }

    pub fn active_mut(&mut self) -> &mut [LinearConstraint] {
        &mut self.constraints[..self.active_len]
    }

    pub fn active_len(&self) -> usize {
        self.active_len
    }

    pub fn capacity(&self) -> usize {
        self.constraints.len()
    }
}

impl VelocityBounds {
    pub fn unbounded(dof: usize) -> Self {
        Self {
            lower: DVector::from_element(dof, f64::NEG_INFINITY),
            upper: DVector::from_element(dof, f64::INFINITY),
        }
    }

    pub fn validate(&self, dof: usize) -> bool {
        self.lower.len() == dof
            && self.upper.len() == dof
            && self
                .lower
                .iter()
                .zip(self.upper.iter())
                .all(|(lower, upper)| !lower.is_nan() && !upper.is_nan() && lower <= upper)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum SolveStatus {
    Solved,
    SolvedWithSlack,
    /// The bounded feasibility/numerical work budget ended without either an
    /// accepted feasible point or a mathematical infeasibility certificate.
    MaxIterations,
    PrimalInfeasible,
    NumericalFailure,
    InvalidProblem,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LevelResidual {
    pub priority: Priority,
    pub rows: usize,
    pub l2: f64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SolveDiagnostics {
    pub status: SolveStatus,
    pub level_residuals: Vec<LevelResidual>,
    pub minimum_bound_margin: f64,
    /// Largest coordinate-bound violation in the terminal hard-feasibility
    /// witness. Zero on a feasible solve; infinity means no valid witness was
    /// available (for example, an invalid problem declaration).
    pub maximum_bound_violation: f64,
    /// Coordinate owning `maximum_bound_violation`. Ties are resolved by the
    /// smallest coordinate so the witness remains deterministic.
    pub limiting_bound_coordinate: Option<usize>,
    /// True when the limiting coordinate exceeded its upper bound; false for
    /// a lower-bound violation. Meaningful only with a limiting coordinate.
    pub limiting_bound_is_upper: bool,
    pub clipped_levels: Vec<Priority>,
    pub rank_by_level: Vec<usize>,
    pub active_constraints: Vec<u32>,
    /// Largest named linear-row violation in the terminal hard-feasibility
    /// witness, separately from anonymous coordinate bounds.
    pub maximum_linear_constraint_violation: f64,
    /// Stable ID owning `maximum_linear_constraint_violation`.
    pub limiting_linear_constraint: Option<u32>,
    /// True when the limiting named row exceeded its upper bound.
    pub limiting_linear_constraint_is_upper: bool,
    pub maximum_constraint_violation: f64,
    /// Number of task-level dense pseudoinverse evaluations this tick.
    pub task_pseudoinverse_calls: usize,
    /// Task-level dense pseudoinverse evaluations attributed by priority.
    pub task_pseudoinverse_calls_by_level: [usize; Priority::ALL.len()],
    /// Cyclic Jacobi sweeps spent in task-level pseudoinverses this tick.
    pub task_jacobi_sweeps: usize,
    /// Task-level Jacobi sweeps attributed by priority.
    pub task_jacobi_sweeps_by_level: [usize; Priority::ALL.len()],
    /// Number of bound/inequality truncations accepted while solving tasks.
    pub clipped_steps: usize,
    /// Bound/inequality truncations attributed by priority.
    pub clipped_steps_by_level: [usize; Priority::ALL.len()],
    /// The equality nullspace reused the exact seed pseudoinverse this tick.
    pub equality_pseudoinverse_reused: bool,
    /// Full cyclic projection sweeps used to find a hard-feasible seed.
    pub feasibility_projection_sweeps: usize,
    /// Individual bound/constraint halfspace projections attempted by the seed.
    pub feasibility_halfspace_projections: usize,
    /// Active-set correction iterations used after a near-feasible seed.
    pub feasibility_polish_iterations: usize,
    /// Dense pseudoinverses used by active-set feasibility polishing.
    pub feasibility_polish_pseudoinverse_calls: usize,
    /// Jacobi sweeps spent in active-set feasibility polishing.
    pub feasibility_polish_jacobi_sweeps: usize,
    /// A terminal hard result or resumable exact Dykstra prefix came from a
    /// revalidated hard-problem cache. Logical feasibility counters still
    /// report the complete cold-equivalent work represented by the witness.
    pub feasibility_seed_reused: bool,
    /// A bounded cached Dykstra prefix was resumed to the destination's
    /// uncapped terminal result rather than accepted as a terminal seed.
    pub feasibility_prefix_resumed: bool,
}

impl Clone for SolveDiagnostics {
    fn clone(&self) -> Self {
        Self {
            status: self.status,
            level_residuals: self.level_residuals.clone(),
            minimum_bound_margin: self.minimum_bound_margin,
            maximum_bound_violation: self.maximum_bound_violation,
            limiting_bound_coordinate: self.limiting_bound_coordinate,
            limiting_bound_is_upper: self.limiting_bound_is_upper,
            clipped_levels: self.clipped_levels.clone(),
            rank_by_level: self.rank_by_level.clone(),
            active_constraints: self.active_constraints.clone(),
            maximum_linear_constraint_violation: self.maximum_linear_constraint_violation,
            limiting_linear_constraint: self.limiting_linear_constraint,
            limiting_linear_constraint_is_upper: self.limiting_linear_constraint_is_upper,
            maximum_constraint_violation: self.maximum_constraint_violation,
            task_pseudoinverse_calls: self.task_pseudoinverse_calls,
            task_pseudoinverse_calls_by_level: self.task_pseudoinverse_calls_by_level,
            task_jacobi_sweeps: self.task_jacobi_sweeps,
            task_jacobi_sweeps_by_level: self.task_jacobi_sweeps_by_level,
            clipped_steps: self.clipped_steps,
            clipped_steps_by_level: self.clipped_steps_by_level,
            equality_pseudoinverse_reused: self.equality_pseudoinverse_reused,
            feasibility_projection_sweeps: self.feasibility_projection_sweeps,
            feasibility_halfspace_projections: self.feasibility_halfspace_projections,
            feasibility_polish_iterations: self.feasibility_polish_iterations,
            feasibility_polish_pseudoinverse_calls: self.feasibility_polish_pseudoinverse_calls,
            feasibility_polish_jacobi_sweeps: self.feasibility_polish_jacobi_sweeps,
            feasibility_seed_reused: self.feasibility_seed_reused,
            feasibility_prefix_resumed: self.feasibility_prefix_resumed,
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.status = source.status;
        self.level_residuals.clone_from(&source.level_residuals);
        self.minimum_bound_margin = source.minimum_bound_margin;
        self.maximum_bound_violation = source.maximum_bound_violation;
        self.limiting_bound_coordinate = source.limiting_bound_coordinate;
        self.limiting_bound_is_upper = source.limiting_bound_is_upper;
        self.clipped_levels.clone_from(&source.clipped_levels);
        self.rank_by_level.clone_from(&source.rank_by_level);
        self.active_constraints
            .clone_from(&source.active_constraints);
        self.maximum_linear_constraint_violation = source.maximum_linear_constraint_violation;
        self.limiting_linear_constraint = source.limiting_linear_constraint;
        self.limiting_linear_constraint_is_upper = source.limiting_linear_constraint_is_upper;
        self.maximum_constraint_violation = source.maximum_constraint_violation;
        self.task_pseudoinverse_calls = source.task_pseudoinverse_calls;
        self.task_pseudoinverse_calls_by_level = source.task_pseudoinverse_calls_by_level;
        self.task_jacobi_sweeps = source.task_jacobi_sweeps;
        self.task_jacobi_sweeps_by_level = source.task_jacobi_sweeps_by_level;
        self.clipped_steps = source.clipped_steps;
        self.clipped_steps_by_level = source.clipped_steps_by_level;
        self.equality_pseudoinverse_reused = source.equality_pseudoinverse_reused;
        self.feasibility_projection_sweeps = source.feasibility_projection_sweeps;
        self.feasibility_halfspace_projections = source.feasibility_halfspace_projections;
        self.feasibility_polish_iterations = source.feasibility_polish_iterations;
        self.feasibility_polish_pseudoinverse_calls = source.feasibility_polish_pseudoinverse_calls;
        self.feasibility_polish_jacobi_sweeps = source.feasibility_polish_jacobi_sweeps;
        self.feasibility_seed_reused = source.feasibility_seed_reused;
        self.feasibility_prefix_resumed = source.feasibility_prefix_resumed;
    }
}

#[derive(Clone, Debug)]
pub struct SolveResult {
    pub velocity: DVector<f64>,
    pub diagnostics: SolveDiagnostics,
}

impl SolveResult {
    pub fn workspace(dof: usize, maximum_constraints: usize) -> Self {
        Self {
            velocity: DVector::zeros(dof),
            diagnostics: SolveDiagnostics {
                status: SolveStatus::InvalidProblem,
                level_residuals: Vec::with_capacity(Priority::ALL.len()),
                minimum_bound_margin: f64::NEG_INFINITY,
                maximum_bound_violation: f64::INFINITY,
                limiting_bound_coordinate: None,
                limiting_bound_is_upper: false,
                clipped_levels: Vec::with_capacity(Priority::ALL.len()),
                rank_by_level: Vec::with_capacity(Priority::ALL.len()),
                active_constraints: Vec::with_capacity(maximum_constraints),
                maximum_linear_constraint_violation: f64::INFINITY,
                limiting_linear_constraint: None,
                limiting_linear_constraint_is_upper: false,
                maximum_constraint_violation: f64::INFINITY,
                task_pseudoinverse_calls: 0,
                task_pseudoinverse_calls_by_level: [0; Priority::ALL.len()],
                task_jacobi_sweeps: 0,
                task_jacobi_sweeps_by_level: [0; Priority::ALL.len()],
                clipped_steps: 0,
                clipped_steps_by_level: [0; Priority::ALL.len()],
                equality_pseudoinverse_reused: false,
                feasibility_projection_sweeps: 0,
                feasibility_halfspace_projections: 0,
                feasibility_polish_iterations: 0,
                feasibility_polish_pseudoinverse_calls: 0,
                feasibility_polish_jacobi_sweeps: 0,
                feasibility_seed_reused: false,
                feasibility_prefix_resumed: false,
            },
        }
    }
}

/// Fixed-capacity dense workspace for one hierarchical solve.
///
/// Matrices use row-major flat storage because every logical shape changes
/// between priority levels. The backing vectors retain their compiler-sized
/// allocations while only the active row prefixes change.
#[derive(Clone, Debug)]
pub struct SolverWorkspace {
    dof: usize,
    maximum_task_rows: usize,
    ordered_tasks: Vec<usize>,
    ordered_constraints: Vec<usize>,
    halfspaces: Vec<Halfspace>,
    feasibility_nonzero_indices: Vec<usize>,
    feasibility_nonzero_values: Vec<f64>,
    feasibility_problem_bits: Vec<u64>,
    cached_feasibility_problem_bits: Vec<u64>,
    cached_feasibility_solution: Vec<f64>,
    cached_feasibility_multipliers: Vec<f64>,
    cached_feasibility_valid: bool,
    cached_feasibility_projection_sweeps: usize,
    cached_feasibility_halfspace_projections: usize,
    cached_feasibility_polish_iterations: usize,
    cached_feasibility_polish_pseudoinverse_calls: usize,
    cached_feasibility_polish_jacobi_sweeps: usize,
    cached_equality_pseudoinverse_valid: bool,
    cached_feasibility_exhausted: bool,
    cached_feasibility_budget_independent: bool,
    cached_feasibility_prefix_continuable: bool,
    cached_feasibility_maximum_violation: f64,
    multipliers: Vec<f64>,
    solution: Vec<f64>,
    nullspace: Vec<f64>,
    equality_nullspace: Vec<f64>,
    level_matrix: Vec<f64>,
    target: Vec<f64>,
    projected: Vec<f64>,
    pseudo_inverse: Vec<f64>,
    orthogonal_columns: Vec<f64>,
    right_vectors: Vec<f64>,
    singular_values: Vec<f64>,
    rhs: Vec<f64>,
    reduced_step: Vec<f64>,
    correction: Vec<f64>,
    bound_row: Vec<f64>,
    rank_one_column: Vec<f64>,
    nullspace_times_pseudo_inverse: Vec<f64>,
    square_product: Vec<f64>,
}

impl SolverWorkspace {
    pub fn new(
        dof: usize,
        maximum_task_rows: usize,
        maximum_tasks: usize,
        maximum_constraints: usize,
    ) -> Self {
        let task_elements = maximum_task_rows.saturating_mul(dof);
        let square_elements = dof.saturating_mul(dof);
        let feasibility_key_elements = feasibility_problem_key_capacity(dof, maximum_constraints);
        Self {
            dof,
            maximum_task_rows,
            ordered_tasks: Vec::with_capacity(maximum_tasks),
            ordered_constraints: Vec::with_capacity(maximum_constraints),
            halfspaces: Vec::with_capacity(
                dof.saturating_mul(2)
                    .saturating_add(maximum_constraints.saturating_mul(2)),
            ),
            feasibility_nonzero_indices: Vec::with_capacity(
                maximum_constraints.saturating_mul(dof),
            ),
            feasibility_nonzero_values: Vec::with_capacity(maximum_constraints.saturating_mul(dof)),
            feasibility_problem_bits: Vec::with_capacity(feasibility_key_elements),
            cached_feasibility_problem_bits: Vec::with_capacity(feasibility_key_elements),
            cached_feasibility_solution: vec![0.0; dof],
            cached_feasibility_multipliers: Vec::with_capacity(
                dof.saturating_mul(2)
                    .saturating_add(maximum_constraints.saturating_mul(2)),
            ),
            cached_feasibility_valid: false,
            cached_feasibility_projection_sweeps: 0,
            cached_feasibility_halfspace_projections: 0,
            cached_feasibility_polish_iterations: 0,
            cached_feasibility_polish_pseudoinverse_calls: 0,
            cached_feasibility_polish_jacobi_sweeps: 0,
            cached_equality_pseudoinverse_valid: false,
            cached_feasibility_exhausted: false,
            cached_feasibility_budget_independent: false,
            cached_feasibility_prefix_continuable: false,
            cached_feasibility_maximum_violation: f64::INFINITY,
            multipliers: Vec::with_capacity(
                dof.saturating_mul(2)
                    .saturating_add(maximum_constraints.saturating_mul(2)),
            ),
            solution: vec![0.0; dof],
            nullspace: vec![0.0; square_elements],
            equality_nullspace: vec![0.0; square_elements],
            level_matrix: vec![0.0; task_elements],
            target: vec![0.0; maximum_task_rows],
            projected: vec![0.0; task_elements],
            pseudo_inverse: vec![0.0; task_elements],
            orthogonal_columns: vec![0.0; task_elements],
            right_vectors: vec![0.0; square_elements],
            singular_values: vec![0.0; dof],
            rhs: vec![0.0; maximum_task_rows],
            reduced_step: vec![0.0; dof],
            correction: vec![0.0; dof],
            bound_row: vec![0.0; dof],
            rank_one_column: vec![0.0; dof],
            nullspace_times_pseudo_inverse: vec![0.0; task_elements],
            square_product: vec![0.0; square_elements],
        }
    }

    /// Copy one immutable terminal hard-feasibility witness into this
    /// workspace without sharing any mutable solver storage.
    ///
    /// The next solve still rebuilds and bit-compares its complete hard
    /// problem key before reuse. Equality-factorization storage is deliberately
    /// not transferred: the destination constructs its own factor before the
    /// key check and may reuse that local factor under the copied validity bit.
    /// `false` invalidates the destination witness and means
    /// the source had no valid witness or the fixed capacities differ.
    pub fn import_hard_feasibility_witness_from(&mut self, source: &Self) -> bool {
        let compatible = source.cached_feasibility_valid
            && self.dof == source.dof
            && self.cached_feasibility_solution.len() == source.cached_feasibility_solution.len()
            && self.cached_feasibility_problem_bits.capacity()
                >= source.cached_feasibility_problem_bits.len()
            && self.cached_feasibility_multipliers.capacity()
                >= source.cached_feasibility_multipliers.len();
        if !compatible {
            self.cached_feasibility_valid = false;
            self.cached_feasibility_prefix_continuable = false;
            self.cached_equality_pseudoinverse_valid = false;
            return false;
        }
        self.cached_feasibility_problem_bits.clear();
        self.cached_feasibility_problem_bits
            .extend_from_slice(&source.cached_feasibility_problem_bits);
        self.cached_feasibility_solution
            .copy_from_slice(&source.cached_feasibility_solution);
        self.cached_feasibility_multipliers.clear();
        self.cached_feasibility_multipliers
            .extend_from_slice(&source.cached_feasibility_multipliers);
        self.cached_feasibility_projection_sweeps = source.cached_feasibility_projection_sweeps;
        self.cached_feasibility_halfspace_projections =
            source.cached_feasibility_halfspace_projections;
        self.cached_feasibility_polish_iterations = source.cached_feasibility_polish_iterations;
        self.cached_feasibility_polish_pseudoinverse_calls =
            source.cached_feasibility_polish_pseudoinverse_calls;
        self.cached_feasibility_polish_jacobi_sweeps =
            source.cached_feasibility_polish_jacobi_sweeps;
        self.cached_feasibility_exhausted = source.cached_feasibility_exhausted;
        self.cached_feasibility_budget_independent = source.cached_feasibility_budget_independent;
        self.cached_feasibility_prefix_continuable = source.cached_feasibility_prefix_continuable;
        self.cached_feasibility_maximum_violation = source.cached_feasibility_maximum_violation;
        self.cached_equality_pseudoinverse_valid = source.cached_equality_pseudoinverse_valid;
        self.cached_feasibility_valid = true;
        true
    }

    fn ensure_capacity(&mut self, dof: usize, task_rows: usize, tasks: usize, constraints: usize) {
        if self.dof != dof {
            *self = Self::new(dof, task_rows, tasks, constraints);
            return;
        }
        if task_rows > self.maximum_task_rows {
            self.maximum_task_rows = task_rows;
            let task_elements = task_rows.saturating_mul(dof);
            self.level_matrix.resize(task_elements, 0.0);
            self.target.resize(task_rows, 0.0);
            self.projected.resize(task_elements, 0.0);
            self.pseudo_inverse.resize(task_elements, 0.0);
            self.orthogonal_columns.resize(task_elements, 0.0);
            self.rhs.resize(task_rows, 0.0);
            self.nullspace_times_pseudo_inverse
                .resize(task_elements, 0.0);
        }
        self.ordered_tasks
            .reserve(tasks.saturating_sub(self.ordered_tasks.len()));
        self.ordered_constraints
            .reserve(constraints.saturating_sub(self.ordered_constraints.len()));
        let halfspaces = dof
            .saturating_mul(2)
            .saturating_add(constraints.saturating_mul(2));
        self.halfspaces
            .reserve(halfspaces.saturating_sub(self.halfspaces.len()));
        self.multipliers
            .reserve(halfspaces.saturating_sub(self.multipliers.len()));
        self.cached_feasibility_multipliers
            .reserve(halfspaces.saturating_sub(self.cached_feasibility_multipliers.len()));
        let nonzero_indices = constraints.saturating_mul(dof);
        self.feasibility_nonzero_indices
            .reserve(nonzero_indices.saturating_sub(self.feasibility_nonzero_indices.len()));
        self.feasibility_nonzero_values
            .reserve(nonzero_indices.saturating_sub(self.feasibility_nonzero_values.len()));
        let feasibility_key_elements = feasibility_problem_key_capacity(dof, constraints);
        reserve_total_capacity(&mut self.feasibility_problem_bits, feasibility_key_elements);
        reserve_total_capacity(
            &mut self.cached_feasibility_problem_bits,
            feasibility_key_elements,
        );
    }
}

#[derive(Clone, Debug)]
pub struct HierarchicalSolver {
    pub singular_value_tolerance: f64,
    /// Tikhonov damping applied only to soft task-level pseudoinverses. Hard
    /// feasibility and equality-nullspace construction remain undamped.
    pub task_singular_value_damping: f64,
    /// Maximum primal/dual active-set iterations used to seed hard
    /// feasibility. A smaller planner-only budget may return `MaxIterations`;
    /// it never turns an unfinished candidate into an executable command.
    pub maximum_feasibility_iterations: usize,
    /// Optional total Dykstra sweep ceiling for hard-feasibility seeding.
    /// `None` preserves the exact row-count-scaled fallback. A finite real-time
    /// profile fails closed with `MaxIterations` when the ceiling is exhausted.
    pub maximum_feasibility_projection_sweeps: Option<usize>,
    /// Optional normalized primal-violation threshold for continuing an
    /// exhausted finite projection prefix with the established row-scaled
    /// budget. This preserves exact Dykstra state for promising anytime
    /// queries while distant candidates still fail closed at the prefix.
    pub feasibility_projection_continuation_violation_threshold: Option<f64>,
    /// Re-solve displaced hard equalities before inspecting inequality-only
    /// active-set violations. Disabled by default to preserve the established
    /// exact fallback until a controller profile admits the repair.
    pub repair_feasibility_equalities_before_inequalities: bool,
    /// Restrict Dykstra linear-row dot products and transpose updates to each
    /// row's first-to-last nonzero coefficient span. This leaves the ordered
    /// projections, cached norms, sweep budget, and all non-Dykstra kernels
    /// unchanged while avoiding known-zero prefix/suffix work.
    pub use_feasibility_row_spans: bool,
    /// Reuse a terminal exact or exhausted hard-feasibility result when the
    /// complete ordered hard problem and feasibility profile are bit-identical
    /// to a prior solve in this workspace. Exhausted results remain fail-closed
    /// and never enter the soft hierarchy. Disabled by default and never
    /// shared across sessions.
    pub reuse_identical_hard_feasibility_seed: bool,
    /// Continue an exhausted, bit-identical bounded feasibility problem from
    /// its cached Dykstra point and multipliers. Each call receives a fresh
    /// ceiling-sized slice; diagnostics report cumulative work so callers can
    /// observe the continuation. Disabled by default.
    pub continue_identical_exhausted_feasibility_prefix: bool,
}

impl Default for HierarchicalSolver {
    fn default() -> Self {
        Self {
            singular_value_tolerance: 1e-9,
            task_singular_value_damping: 0.0,
            maximum_feasibility_iterations: ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT,
            maximum_feasibility_projection_sweeps: None,
            feasibility_projection_continuation_violation_threshold: None,
            repair_feasibility_equalities_before_inequalities: false,
            use_feasibility_row_spans: false,
            reuse_identical_hard_feasibility_seed: false,
            continue_identical_exhausted_feasibility_prefix: false,
        }
    }
}

impl HierarchicalSolver {
    /// Solve prioritized least-squares tasks in a recursively maintained null
    /// space. Bounds may truncate a level's step, but truncation is itself in
    /// the previous levels' null space, preserving their achieved optimum.
    pub fn solve(&self, dof: usize, tasks: &[Task], bounds: &VelocityBounds) -> SolveResult {
        self.solve_constrained(dof, tasks, bounds, &[])
    }

    pub fn solve_constrained(
        &self,
        dof: usize,
        tasks: &[Task],
        bounds: &VelocityBounds,
        constraints: &[LinearConstraint],
    ) -> SolveResult {
        let rows = tasks.iter().map(|task| task.jacobian.nrows()).sum();
        let mut workspace = SolverWorkspace::new(dof, rows, tasks.len(), constraints.len());
        let mut result = SolveResult::workspace(dof, constraints.len());
        self.solve_constrained_into(dof, tasks, bounds, constraints, &mut result, &mut workspace);
        result
    }

    /// Solve into caller-owned result and dense work buffers.
    ///
    /// When `result` and `workspace` were constructed for the compiled maximum
    /// row counts, this function performs no heap allocation.
    pub fn solve_constrained_into(
        &self,
        dof: usize,
        tasks: &[Task],
        bounds: &VelocityBounds,
        constraints: &[LinearConstraint],
        result: &mut SolveResult,
        workspace: &mut SolverWorkspace,
    ) {
        self.solve_active_into(dof, tasks, None, bounds, constraints, result, workspace);
    }

    pub fn solve_task_buffer_into(
        &self,
        dof: usize,
        tasks: &TaskBuffer,
        bounds: &VelocityBounds,
        constraints: &[LinearConstraint],
        result: &mut SolveResult,
        workspace: &mut SolverWorkspace,
    ) {
        self.solve_active_into(
            dof,
            tasks.tasks(),
            Some(tasks.active_indices()),
            bounds,
            constraints,
            result,
            workspace,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_active_into(
        &self,
        dof: usize,
        tasks: &[Task],
        active_task_indices: Option<&[usize]>,
        bounds: &VelocityBounds,
        constraints: &[LinearConstraint],
        result: &mut SolveResult,
        workspace: &mut SolverWorkspace,
    ) {
        let active_task_count = active_task_indices.map_or(tasks.len(), <[usize]>::len);
        let invalid_task = |task: &Task| {
            task.jacobian.ncols() != dof
                || task.jacobian.nrows() != task.target_velocity.len()
                || !task.weight.is_finite()
                || task.weight < 0.0
                || !task.jacobian.iter().all(|x| x.is_finite())
                || !task.target_velocity.iter().all(|x| x.is_finite())
        };
        let invalid_tasks = if let Some(indices) = active_task_indices {
            indices.iter().any(|index| invalid_task(&tasks[*index]))
        } else {
            tasks.iter().any(invalid_task)
        };
        let invalid = !self.singular_value_tolerance.is_finite()
            || self.singular_value_tolerance <= 0.0
            || !self.task_singular_value_damping.is_finite()
            || self.task_singular_value_damping < 0.0
            || self.maximum_feasibility_iterations == 0
            || self.maximum_feasibility_iterations > ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT
            || self
                .maximum_feasibility_projection_sweeps
                .is_some_and(|sweeps| sweeps == 0)
            || self
                .feasibility_projection_continuation_violation_threshold
                .is_some_and(|threshold| !threshold.is_finite() || threshold < 0.0)
            || !bounds.validate(dof)
            || invalid_tasks
            || constraints.iter().any(|constraint| {
                constraint.coefficients.len() != dof
                    || !constraint
                        .coefficients
                        .iter()
                        .all(|value| value.is_finite())
                    || constraint.lower.is_nan()
                    || constraint.upper.is_nan()
                    || constraint.lower > constraint.upper
            });
        let total_rows: usize = active_task_indices.map_or_else(
            || tasks.iter().map(|task| task.jacobian.nrows()).sum(),
            |indices| {
                indices
                    .iter()
                    .map(|index| tasks[*index].jacobian.nrows())
                    .sum()
            },
        );
        let equality_rows = constraints
            .iter()
            .filter(|constraint| {
                constraint.lower.is_finite() && constraint.lower == constraint.upper
            })
            .count();
        let maximum_feasibility_rows = equality_rows
            .saturating_add(dof.saturating_mul(2))
            .saturating_add(constraints.len().saturating_mul(2));
        workspace.ensure_capacity(
            dof,
            total_rows.max(maximum_feasibility_rows),
            active_task_count,
            constraints.len(),
        );
        prepare_result(result, dof);
        if invalid {
            fail_into(result, SolveStatus::InvalidProblem);
            return;
        }

        workspace.ordered_tasks.clear();
        if let Some(indices) = active_task_indices {
            workspace.ordered_tasks.extend_from_slice(indices);
        } else {
            workspace.ordered_tasks.extend(0..tasks.len());
        }
        stable_insertion_sort_by_key(&mut workspace.ordered_tasks, |index| {
            (tasks[index].priority, tasks[index].stable_id)
        });
        workspace.ordered_constraints.clear();
        workspace.ordered_constraints.extend(0..constraints.len());
        stable_insertion_sort_by_key(&mut workspace.ordered_constraints, |index| {
            constraints[index].stable_id
        });

        let mut equality_pseudoinverse_valid = false;
        if workspace.ordered_constraints.is_empty() {
            for index in 0..dof {
                workspace.solution[index] = 0.0_f64.clamp(bounds.lower[index], bounds.upper[index]);
            }
        } else {
            seed_linear_equalities_into(
                dof,
                constraints,
                &workspace.ordered_constraints,
                equality_rows,
                self.singular_value_tolerance.min(1e-12),
                &mut workspace.solution,
                &mut workspace.level_matrix,
                &mut workspace.target,
                &mut workspace.pseudo_inverse,
                &mut workspace.orthogonal_columns,
                &mut workspace.right_vectors,
                &mut workspace.singular_values,
            );
            equality_pseudoinverse_valid = equality_rows > 0;
            let reuse_cached_feasibility_seed = if self.reuse_identical_hard_feasibility_seed {
                build_feasibility_problem_key(
                    dof,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                    self.singular_value_tolerance,
                    self.maximum_feasibility_iterations,
                    self.maximum_feasibility_projection_sweeps,
                    self.feasibility_projection_continuation_violation_threshold,
                    self.repair_feasibility_equalities_before_inequalities,
                    self.use_feasibility_row_spans,
                    &mut workspace.feasibility_problem_bits,
                );
                workspace.cached_feasibility_valid
                    && feasibility_problem_keys_match(
                        &workspace.feasibility_problem_bits,
                        &workspace.cached_feasibility_problem_bits,
                        workspace.cached_feasibility_budget_independent,
                        workspace.cached_feasibility_prefix_continuable,
                    )
            } else {
                false
            };
            let resume_cached_feasibility_prefix = reuse_cached_feasibility_seed
                && workspace.cached_feasibility_exhausted
                && workspace.cached_feasibility_prefix_continuable
                && (workspace.feasibility_problem_bits
                    != workspace.cached_feasibility_problem_bits
                    || self.continue_identical_exhausted_feasibility_prefix);
            let feasibility_seed = if resume_cached_feasibility_prefix {
                workspace.solution[..dof]
                    .copy_from_slice(&workspace.cached_feasibility_solution[..dof]);
                equality_pseudoinverse_valid = workspace.cached_equality_pseudoinverse_valid;
                result.diagnostics.feasibility_polish_iterations =
                    workspace.cached_feasibility_polish_iterations;
                result.diagnostics.feasibility_polish_pseudoinverse_calls =
                    workspace.cached_feasibility_polish_pseudoinverse_calls;
                result.diagnostics.feasibility_polish_jacobi_sweeps =
                    workspace.cached_feasibility_polish_jacobi_sweeps;
                result.diagnostics.feasibility_seed_reused = true;
                result.diagnostics.feasibility_prefix_resumed = true;
                let rebuilt = build_halfspaces_into(
                    dof,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                    &mut workspace.halfspaces,
                    &mut workspace.feasibility_nonzero_indices,
                    &mut workspace.feasibility_nonzero_values,
                    self.use_feasibility_row_spans,
                );
                if let Err(maximum_violation) = rebuilt {
                    FeasibilitySeedOutcome {
                        seed: FeasibilitySeed::StructurallyInfeasible(maximum_violation),
                        projection_sweeps: workspace.cached_feasibility_projection_sweeps,
                        halfspace_projections: workspace.cached_feasibility_halfspace_projections,
                    }
                } else {
                    debug_assert_eq!(
                        workspace.cached_feasibility_multipliers.len(),
                        workspace.halfspaces.len()
                    );
                    workspace.multipliers.clear();
                    workspace
                        .multipliers
                        .extend_from_slice(&workspace.cached_feasibility_multipliers);
                    let maximum_projection_sweeps = self
                        .maximum_feasibility_projection_sweeps
                        .unwrap_or_else(|| maximum_dykstra_sweeps(workspace.halfspaces.len()));
                    let identical_bounded_continuation = workspace.feasibility_problem_bits
                        == workspace.cached_feasibility_problem_bits;
                    let additional_projection_sweeps = if identical_bounded_continuation {
                        maximum_projection_sweeps
                    } else {
                        maximum_projection_sweeps
                            .saturating_sub(workspace.cached_feasibility_projection_sweeps)
                    };
                    let continued = continue_feasible_point_into(
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                        &mut workspace.solution,
                        &workspace.halfspaces,
                        &workspace.feasibility_nonzero_indices,
                        &workspace.feasibility_nonzero_values,
                        &mut workspace.multipliers,
                        additional_projection_sweeps,
                        self.use_feasibility_row_spans,
                    );
                    FeasibilitySeedOutcome {
                        seed: continued.seed,
                        projection_sweeps: workspace
                            .cached_feasibility_projection_sweeps
                            .saturating_add(continued.projection_sweeps),
                        halfspace_projections: workspace
                            .cached_feasibility_halfspace_projections
                            .saturating_add(continued.halfspace_projections),
                    }
                }
            } else if reuse_cached_feasibility_seed {
                workspace.solution[..dof]
                    .copy_from_slice(&workspace.cached_feasibility_solution[..dof]);
                equality_pseudoinverse_valid = workspace.cached_equality_pseudoinverse_valid;
                result.diagnostics.feasibility_polish_iterations =
                    workspace.cached_feasibility_polish_iterations;
                result.diagnostics.feasibility_polish_pseudoinverse_calls =
                    workspace.cached_feasibility_polish_pseudoinverse_calls;
                result.diagnostics.feasibility_polish_jacobi_sweeps =
                    workspace.cached_feasibility_polish_jacobi_sweeps;
                result.diagnostics.feasibility_seed_reused = true;
                FeasibilitySeedOutcome {
                    seed: if workspace.cached_feasibility_exhausted {
                        FeasibilitySeed::Exhausted(workspace.cached_feasibility_maximum_violation)
                    } else {
                        FeasibilitySeed::Exact
                    },
                    projection_sweeps: workspace.cached_feasibility_projection_sweeps,
                    halfspace_projections: workspace.cached_feasibility_halfspace_projections,
                }
            } else if USE_EUCLIDEAN_ACTIVE_SET_FEASIBILITY {
                workspace.reduced_step[..dof].copy_from_slice(&workspace.solution[..dof]);
                let halfspace_result = build_halfspaces_into(
                    dof,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                    &mut workspace.halfspaces,
                    &mut workspace.feasibility_nonzero_indices,
                    &mut workspace.feasibility_nonzero_values,
                    false,
                );
                if let Err(maximum_violation) = halfspace_result {
                    FeasibilitySeedOutcome {
                        seed: FeasibilitySeed::StructurallyInfeasible(maximum_violation),
                        projection_sweeps: 0,
                        halfspace_projections: 0,
                    }
                } else {
                    workspace.multipliers.clear();
                    workspace
                        .multipliers
                        .resize(workspace.halfspaces.len(), 0.0);
                    let projected = project_feasible_point_active_set_into(
                        dof,
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                        equality_rows,
                        &workspace.halfspaces,
                        &mut workspace.multipliers,
                        &workspace.reduced_step,
                        &mut workspace.solution,
                        &mut workspace.level_matrix,
                        &mut workspace.target,
                        &mut workspace.pseudo_inverse,
                        &mut workspace.orthogonal_columns,
                        &mut workspace.right_vectors,
                        &mut workspace.singular_values,
                        &mut workspace.correction,
                        self.repair_feasibility_equalities_before_inequalities,
                        FEASIBILITY_SEED_TOLERANCE,
                        self.maximum_feasibility_iterations,
                    );
                    result.diagnostics.feasibility_polish_iterations = projected.iterations;
                    result.diagnostics.feasibility_polish_pseudoinverse_calls =
                        projected.pseudoinverse_calls;
                    result.diagnostics.feasibility_polish_jacobi_sweeps = projected.jacobi_sweeps;
                    if projected.pseudoinverse_calls > 0 {
                        equality_pseudoinverse_valid = false;
                    }
                    if projected.accepted {
                        FeasibilitySeedOutcome {
                            seed: FeasibilitySeed::Exact,
                            projection_sweeps: 0,
                            halfspace_projections: 0,
                        }
                    } else {
                        FeasibilitySeedOutcome {
                            seed: FeasibilitySeed::Exhausted(
                                maximum_bound_violation_slice(&workspace.solution, bounds).max(
                                    maximum_constraint_violation_flat(
                                        &workspace.solution,
                                        constraints,
                                        &workspace.ordered_constraints,
                                    ),
                                ),
                            ),
                            projection_sweeps: 0,
                            halfspace_projections: 0,
                        }
                    }
                }
            } else {
                let mut seed = find_feasible_point_into(
                    dof,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                    &mut workspace.solution,
                    &mut workspace.halfspaces,
                    &mut workspace.feasibility_nonzero_indices,
                    &mut workspace.feasibility_nonzero_values,
                    &mut workspace.multipliers,
                    self.maximum_feasibility_projection_sweeps,
                    self.use_feasibility_row_spans,
                );
                if matches!(seed.seed, FeasibilitySeed::Exhausted(_))
                    && DYKSTRA_SWEEP_LIMIT_BEFORE_ACTIVE_SET.is_some()
                    && !workspace.halfspaces.is_empty()
                    && workspace.multipliers.len() == workspace.halfspaces.len()
                {
                    workspace.reduced_step[..dof].copy_from_slice(&workspace.solution[..dof]);
                    workspace.rhs[..workspace.multipliers.len()]
                        .copy_from_slice(&workspace.multipliers);
                    workspace.multipliers.fill(0.0);
                    let projected = project_feasible_point_active_set_into(
                        dof,
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                        equality_rows,
                        &workspace.halfspaces,
                        &mut workspace.multipliers,
                        &workspace.reduced_step,
                        &mut workspace.solution,
                        &mut workspace.level_matrix,
                        &mut workspace.target,
                        &mut workspace.pseudo_inverse,
                        &mut workspace.orthogonal_columns,
                        &mut workspace.right_vectors,
                        &mut workspace.singular_values,
                        &mut workspace.correction,
                        self.repair_feasibility_equalities_before_inequalities,
                        FEASIBILITY_SEED_TOLERANCE,
                        self.maximum_feasibility_iterations,
                    );
                    result.diagnostics.feasibility_polish_iterations = projected.iterations;
                    result.diagnostics.feasibility_polish_pseudoinverse_calls =
                        projected.pseudoinverse_calls;
                    result.diagnostics.feasibility_polish_jacobi_sweeps = projected.jacobi_sweeps;
                    if projected.pseudoinverse_calls > 0 {
                        equality_pseudoinverse_valid = false;
                    }
                    if projected.accepted {
                        seed.seed = FeasibilitySeed::Exact;
                    } else {
                        // The active set is a bounded accelerator, not a new
                        // infeasibility oracle. Restore the exact Dykstra
                        // state and finish its original bounded budget when
                        // an add/remove cycle reaches the active-set cap.
                        workspace.solution[..dof].copy_from_slice(&workspace.reduced_step[..dof]);
                        let halfspace_count = workspace.multipliers.len();
                        workspace
                            .multipliers
                            .copy_from_slice(&workspace.rhs[..halfspace_count]);
                        let established_projection_sweeps =
                            maximum_dykstra_sweeps(workspace.halfspaces.len());
                        let maximum_projection_sweeps = self
                            .maximum_feasibility_projection_sweeps
                            .unwrap_or(established_projection_sweeps);
                        let remaining_sweeps =
                            maximum_projection_sweeps.saturating_sub(seed.projection_sweeps);
                        let continued = continue_feasible_point_into(
                            bounds,
                            constraints,
                            &workspace.ordered_constraints,
                            &mut workspace.solution,
                            &workspace.halfspaces,
                            &workspace.feasibility_nonzero_indices,
                            &workspace.feasibility_nonzero_values,
                            &mut workspace.multipliers,
                            remaining_sweeps,
                            self.use_feasibility_row_spans,
                        );
                        seed.seed = continued.seed;
                        seed.projection_sweeps += continued.projection_sweeps;
                        seed.halfspace_projections += continued.halfspace_projections;
                        let continue_anytime_projection = matches!(
                            seed.seed,
                            FeasibilitySeed::Exhausted(violation)
                                if maximum_projection_sweeps < established_projection_sweeps
                                    && self
                                        .feasibility_projection_continuation_violation_threshold
                                        .is_some_and(|threshold| violation <= threshold)
                        );
                        if continue_anytime_projection {
                            let continued = continue_feasible_point_into(
                                bounds,
                                constraints,
                                &workspace.ordered_constraints,
                                &mut workspace.solution,
                                &workspace.halfspaces,
                                &workspace.feasibility_nonzero_indices,
                                &workspace.feasibility_nonzero_values,
                                &mut workspace.multipliers,
                                established_projection_sweeps
                                    .saturating_sub(seed.projection_sweeps),
                                self.use_feasibility_row_spans,
                            );
                            seed.seed = continued.seed;
                            seed.projection_sweeps += continued.projection_sweeps;
                            seed.halfspace_projections += continued.halfspace_projections;
                        }
                    }
                }
                seed
            };
            if self.reuse_identical_hard_feasibility_seed
                && (!reuse_cached_feasibility_seed || resume_cached_feasibility_prefix)
                && matches!(
                    feasibility_seed.seed,
                    FeasibilitySeed::Exact | FeasibilitySeed::Exhausted(_)
                )
            {
                workspace
                    .cached_feasibility_problem_bits
                    .clone_from(&workspace.feasibility_problem_bits);
                workspace.cached_feasibility_solution[..dof]
                    .copy_from_slice(&workspace.solution[..dof]);
                workspace.cached_feasibility_multipliers.clear();
                workspace
                    .cached_feasibility_multipliers
                    .extend_from_slice(&workspace.multipliers);
                workspace.cached_feasibility_projection_sweeps = feasibility_seed.projection_sweeps;
                workspace.cached_feasibility_halfspace_projections =
                    feasibility_seed.halfspace_projections;
                workspace.cached_feasibility_polish_iterations =
                    result.diagnostics.feasibility_polish_iterations;
                workspace.cached_feasibility_polish_pseudoinverse_calls =
                    result.diagnostics.feasibility_polish_pseudoinverse_calls;
                workspace.cached_feasibility_polish_jacobi_sweeps =
                    result.diagnostics.feasibility_polish_jacobi_sweeps;
                workspace.cached_equality_pseudoinverse_valid = equality_pseudoinverse_valid;
                workspace.cached_feasibility_exhausted =
                    matches!(feasibility_seed.seed, FeasibilitySeed::Exhausted(_));
                workspace.cached_feasibility_budget_independent =
                    matches!(feasibility_seed.seed, FeasibilitySeed::Exact)
                        && exact_seed_is_projection_budget_independent(
                            self.maximum_feasibility_projection_sweeps,
                            result.diagnostics.feasibility_polish_pseudoinverse_calls,
                        );
                workspace.cached_feasibility_prefix_continuable =
                    matches!(feasibility_seed.seed, FeasibilitySeed::Exhausted(_))
                        && (self.continue_identical_exhausted_feasibility_prefix
                            || exhausted_prefix_is_projection_continuable(
                                self.maximum_feasibility_projection_sweeps,
                            ));
                workspace.cached_feasibility_maximum_violation = match feasibility_seed.seed {
                    FeasibilitySeed::Exhausted(violation) => violation,
                    _ => 0.0,
                };
                workspace.cached_feasibility_valid = true;
            }
            result.diagnostics.feasibility_projection_sweeps = feasibility_seed.projection_sweeps;
            result.diagnostics.feasibility_halfspace_projections =
                feasibility_seed.halfspace_projections;
            let feasibility_seed_was_near = matches!(feasibility_seed.seed, FeasibilitySeed::Near);
            match feasibility_seed.seed {
                FeasibilitySeed::Exact => {}
                FeasibilitySeed::Near => {
                    equality_pseudoinverse_valid = false;
                    let polished = polish_feasible_point_into(
                        dof,
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                        equality_rows,
                        &workspace.halfspaces,
                        &mut workspace.multipliers,
                        &mut workspace.solution,
                        &mut workspace.level_matrix,
                        &mut workspace.target,
                        &mut workspace.pseudo_inverse,
                        &mut workspace.orthogonal_columns,
                        &mut workspace.right_vectors,
                        &mut workspace.singular_values,
                        &mut workspace.correction,
                        FEASIBILITY_SEED_TOLERANCE,
                    );
                    result.diagnostics.feasibility_polish_iterations = polished.iterations;
                    result.diagnostics.feasibility_polish_pseudoinverse_calls =
                        polished.pseudoinverse_calls;
                    result.diagnostics.feasibility_polish_jacobi_sweeps = polished.jacobi_sweeps;
                    if !polished.accepted {
                        record_hard_violation_witness(
                            &mut result.diagnostics,
                            &workspace.solution,
                            bounds,
                            constraints,
                            &workspace.ordered_constraints,
                        );
                        fail_into(result, SolveStatus::MaxIterations);
                        return;
                    }
                }
                FeasibilitySeed::StructurallyInfeasible(maximum_violation) => {
                    record_hard_violation_witness(
                        &mut result.diagnostics,
                        &workspace.solution,
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                    );
                    result.diagnostics.maximum_constraint_violation = result
                        .diagnostics
                        .maximum_constraint_violation
                        .max(maximum_violation);
                    fail_into(result, SolveStatus::PrimalInfeasible);
                    return;
                }
                FeasibilitySeed::Exhausted(maximum_violation) => {
                    record_hard_violation_witness(
                        &mut result.diagnostics,
                        &workspace.solution,
                        bounds,
                        constraints,
                        &workspace.ordered_constraints,
                    );
                    result.diagnostics.maximum_constraint_violation = result
                        .diagnostics
                        .maximum_constraint_violation
                        .max(maximum_violation);
                    fail_into(result, SolveStatus::MaxIterations);
                    return;
                }
            }
            if self.reuse_identical_hard_feasibility_seed
                && !reuse_cached_feasibility_seed
                && feasibility_seed_was_near
            {
                workspace
                    .cached_feasibility_problem_bits
                    .clone_from(&workspace.feasibility_problem_bits);
                workspace.cached_feasibility_solution[..dof]
                    .copy_from_slice(&workspace.solution[..dof]);
                workspace.cached_feasibility_multipliers.clear();
                workspace
                    .cached_feasibility_multipliers
                    .extend_from_slice(&workspace.multipliers);
                workspace.cached_feasibility_projection_sweeps = feasibility_seed.projection_sweeps;
                workspace.cached_feasibility_halfspace_projections =
                    feasibility_seed.halfspace_projections;
                workspace.cached_feasibility_polish_iterations =
                    result.diagnostics.feasibility_polish_iterations;
                workspace.cached_feasibility_polish_pseudoinverse_calls =
                    result.diagnostics.feasibility_polish_pseudoinverse_calls;
                workspace.cached_feasibility_polish_jacobi_sweeps =
                    result.diagnostics.feasibility_polish_jacobi_sweeps;
                workspace.cached_equality_pseudoinverse_valid = equality_pseudoinverse_valid;
                workspace.cached_feasibility_exhausted = false;
                workspace.cached_feasibility_budget_independent =
                    exact_seed_is_projection_budget_independent(
                        self.maximum_feasibility_projection_sweeps,
                        result.diagnostics.feasibility_polish_pseudoinverse_calls,
                    );
                workspace.cached_feasibility_prefix_continuable = false;
                workspace.cached_feasibility_maximum_violation = 0.0;
                workspace.cached_feasibility_valid = true;
            }
        }
        fill_identity(&mut workspace.equality_nullspace, dof);
        if equality_rows > 0 {
            if !equality_pseudoinverse_valid {
                let mut row = 0;
                for constraint_index in workspace.ordered_constraints.iter().copied() {
                    let constraint = &constraints[constraint_index];
                    if !constraint.lower.is_finite() || constraint.lower != constraint.upper {
                        continue;
                    }
                    let norm = constraint.coefficients.norm().max(1e-300);
                    for coordinate in 0..dof {
                        workspace.level_matrix[row * dof + coordinate] =
                            constraint.coefficients[coordinate] / norm;
                    }
                    row += 1;
                }
                debug_assert_eq!(row, equality_rows);
                pseudo_inverse_flat_into(
                    &workspace.level_matrix,
                    equality_rows,
                    dof,
                    self.singular_value_tolerance.min(1e-12),
                    &mut workspace.pseudo_inverse,
                    &mut workspace.orthogonal_columns,
                    &mut workspace.right_vectors,
                    &mut workspace.singular_values,
                );
            } else {
                result.diagnostics.equality_pseudoinverse_reused = true;
            }
            multiply(
                &workspace.pseudo_inverse,
                dof,
                equality_rows,
                &workspace.level_matrix,
                dof,
                &mut workspace.square_product,
            );
            for index in 0..dof * dof {
                workspace.equality_nullspace[index] -= workspace.square_product[index];
            }
        }
        workspace.nullspace[..dof * dof]
            .copy_from_slice(&workspace.equality_nullspace[..dof * dof]);

        for priority in Priority::ALL {
            let declared_rows: usize = workspace
                .ordered_tasks
                .iter()
                .map(|index| &tasks[*index])
                .filter(|task| task.priority == priority && task.weight > 0.0)
                .map(|task| task.jacobian.nrows())
                .sum();
            let rows = if task_row_compaction_enabled() {
                workspace
                    .ordered_tasks
                    .iter()
                    .map(|index| &tasks[*index])
                    .filter(|task| task.priority == priority && task.weight > 0.0)
                    .map(|task| {
                        (0..task.jacobian.nrows())
                            .filter(|&local_row| {
                                !task_row_is_constant_and_satisfied(
                                    task, local_row, priority, bounds,
                                )
                            })
                            .count()
                    })
                    .sum()
            } else {
                declared_rows
            };
            if rows == 0 {
                if declared_rows > 0 {
                    result.diagnostics.rank_by_level.push(0);
                    result.diagnostics.level_residuals.push(LevelResidual {
                        priority,
                        rows: declared_rows,
                        l2: 0.0,
                    });
                }
                continue;
            }
            let mut row = 0;
            for task_index in workspace.ordered_tasks.iter().copied() {
                let task = &tasks[task_index];
                if task.priority != priority || task.weight <= 0.0 {
                    continue;
                }
                let scale = task.weight.sqrt();
                for local_row in 0..task.jacobian.nrows() {
                    if task_row_compaction_enabled()
                        && task_row_is_constant_and_satisfied(task, local_row, priority, bounds)
                    {
                        continue;
                    }
                    let norm = task.jacobian.row(local_row).norm();
                    let normalization = if norm > 1e-12 { scale / norm } else { scale };
                    for coordinate in 0..dof {
                        workspace.level_matrix[row * dof + coordinate] =
                            task.jacobian[(local_row, coordinate)] * normalization;
                    }
                    workspace.target[row] = task.target_velocity[local_row] * normalization;
                    row += 1;
                }
            }
            debug_assert_eq!(row, rows);

            let mut level_was_clipped = false;
            let mut level_rank = 0;
            let mut projected_matches_nullspace = false;
            let mut carry_task_correction = false;
            let mut task_projector_valid = false;
            let mut task_projector_ready = false;
            for _ in 0..=dof.saturating_add(workspace.ordered_constraints.len()) {
                if !carry_task_correction {
                    multiply(
                        &workspace.level_matrix,
                        rows,
                        dof,
                        &workspace.nullspace,
                        dof,
                        &mut workspace.projected,
                    );
                    let pseudoinverse = task_pseudo_inverse_flat_into_profiled(
                        &workspace.projected,
                        rows,
                        dof,
                        self.singular_value_tolerance,
                        self.task_singular_value_damping,
                        &mut workspace.pseudo_inverse,
                        &mut workspace.orthogonal_columns,
                        &mut workspace.right_vectors,
                        &mut workspace.singular_values,
                    );
                    result.diagnostics.task_pseudoinverse_calls += 1;
                    result.diagnostics.task_pseudoinverse_calls_by_level[priority as usize] += 1;
                    result.diagnostics.task_jacobi_sweeps += pseudoinverse.jacobi_sweeps;
                    result.diagnostics.task_jacobi_sweeps_by_level[priority as usize] +=
                        pseudoinverse.jacobi_sweeps;
                    projected_matches_nullspace = true;
                    level_rank = level_rank.max(pseudoinverse.rank);
                    matrix_vector_residual(
                        &workspace.level_matrix,
                        rows,
                        dof,
                        &workspace.solution,
                        &workspace.target,
                        &mut workspace.rhs,
                    );
                    multiply_matrix_vector(
                        &workspace.pseudo_inverse,
                        dof,
                        rows,
                        &workspace.rhs,
                        &mut workspace.reduced_step,
                    );
                    multiply_matrix_vector(
                        &workspace.nullspace,
                        dof,
                        dof,
                        &workspace.reduced_step,
                        &mut workspace.correction,
                    );
                    multiply_matrix_vector(
                        &workspace.equality_nullspace,
                        dof,
                        dof,
                        &workspace.correction,
                        &mut workspace.reduced_step,
                    );
                    workspace.correction[..dof].copy_from_slice(&workspace.reduced_step[..dof]);
                    task_projector_valid =
                        cfg!(feature = "clipped-task-nullspace-repair-experiment")
                            && priority != Priority::Style
                            && self.task_singular_value_damping == 0.0;
                    if task_projector_valid {
                        build_task_nullspace_projector(
                            dof,
                            rows,
                            &workspace.nullspace,
                            &workspace.pseudo_inverse,
                            &workspace.projected,
                            &mut workspace.nullspace_times_pseudo_inverse,
                            &mut workspace.square_product,
                        );
                    }
                }
                let (alpha, hit) = maximum_feasible_step_flat(
                    &workspace.solution,
                    &workspace.correction,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                );
                let correction_norm_squared = squared_norm(&workspace.correction[..dof]);
                for index in 0..dof {
                    workspace.solution[index] += workspace.correction[index] * alpha;
                }
                if alpha >= 1.0 - 1e-12 || correction_norm_squared < 1e-24 {
                    if task_projector_valid {
                        workspace.nullspace[..dof * dof]
                            .copy_from_slice(&workspace.square_product[..dof * dof]);
                        task_projector_ready = true;
                    }
                    break;
                }

                level_was_clipped = true;
                result.diagnostics.clipped_steps += 1;
                result.diagnostics.clipped_steps_by_level[priority as usize] += 1;
                let Some(hit) = hit else {
                    break;
                };
                for coordinate in 0..dof {
                    workspace.correction[coordinate] *= 1.0 - alpha;
                }
                let froze_primary = match hit {
                    ConstraintHit::Coordinate(index) => {
                        workspace.solution[index] = if workspace.correction[index] > 0.0 {
                            bounds.upper[index]
                        } else {
                            bounds.lower[index]
                        };
                        freeze_coordinate_in_nullspace(
                            &mut workspace.nullspace,
                            dof,
                            index,
                            &mut workspace.bound_row,
                            &mut workspace.rank_one_column,
                        )
                    }
                    ConstraintHit::Linear(index) => {
                        let constraint = &constraints[workspace.ordered_constraints[index]];
                        if !result
                            .diagnostics
                            .active_constraints
                            .contains(&constraint.stable_id)
                        {
                            result
                                .diagnostics
                                .active_constraints
                                .push(constraint.stable_id);
                        }
                        freeze_linear_in_nullspace(
                            &mut workspace.nullspace,
                            dof,
                            &constraint.coefficients,
                            &mut workspace.bound_row,
                            &mut workspace.rank_one_column,
                        )
                    }
                };
                if !froze_primary {
                    break;
                }
                let repaired_task_optimum = task_projector_valid
                    && repair_correction_through_task_nullspace(
                        hit,
                        dof,
                        constraints,
                        &workspace.ordered_constraints,
                        &mut workspace.square_product,
                        &mut workspace.correction,
                        &mut workspace.bound_row,
                        &mut workspace.rank_one_column,
                    );
                freeze_coincident_step_limits(
                    &mut workspace.nullspace,
                    dof,
                    hit,
                    alpha,
                    &workspace.solution,
                    &workspace.correction,
                    bounds,
                    constraints,
                    &workspace.ordered_constraints,
                    &mut result.diagnostics.active_constraints,
                    &mut workspace.bound_row,
                    &mut workspace.rank_one_column,
                );
                carry_task_correction = repaired_task_optimum;
                task_projector_valid = repaired_task_optimum;
                projected_matches_nullspace = false;
            }
            result.diagnostics.rank_by_level.push(level_rank);
            if level_was_clipped {
                result.diagnostics.clipped_levels.push(priority);
            }

            // Lower levels may only move in the null space of this level in
            // addition to all previously accumulated null spaces. Style is
            // terminal, so constructing its projector cannot affect output.
            if priority != Priority::Style && !task_projector_ready {
                if !projected_matches_nullspace {
                    multiply(
                        &workspace.level_matrix,
                        rows,
                        dof,
                        &workspace.nullspace,
                        dof,
                        &mut workspace.projected,
                    );
                    let pseudoinverse = task_pseudo_inverse_flat_into_profiled(
                        &workspace.projected,
                        rows,
                        dof,
                        self.singular_value_tolerance,
                        self.task_singular_value_damping,
                        &mut workspace.pseudo_inverse,
                        &mut workspace.orthogonal_columns,
                        &mut workspace.right_vectors,
                        &mut workspace.singular_values,
                    );
                    result.diagnostics.task_pseudoinverse_calls += 1;
                    result.diagnostics.task_pseudoinverse_calls_by_level[priority as usize] += 1;
                    result.diagnostics.task_jacobi_sweeps += pseudoinverse.jacobi_sweeps;
                    result.diagnostics.task_jacobi_sweeps_by_level[priority as usize] +=
                        pseudoinverse.jacobi_sweeps;
                }
                multiply(
                    &workspace.nullspace,
                    dof,
                    dof,
                    &workspace.pseudo_inverse,
                    rows,
                    &mut workspace.nullspace_times_pseudo_inverse,
                );
                multiply(
                    &workspace.nullspace_times_pseudo_inverse,
                    dof,
                    rows,
                    &workspace.projected,
                    dof,
                    &mut workspace.square_product,
                );
                for index in 0..dof * dof {
                    workspace.nullspace[index] -= workspace.square_product[index];
                }
            }

            matrix_vector_residual(
                &workspace.level_matrix,
                rows,
                dof,
                &workspace.solution,
                &workspace.target,
                &mut workspace.rhs,
            );
            result.diagnostics.level_residuals.push(LevelResidual {
                priority,
                rows: declared_rows,
                l2: squared_norm(&workspace.rhs[..rows]).sqrt(),
            });
        }

        for index in 0..dof {
            workspace.solution[index] =
                workspace.solution[index].clamp(bounds.lower[index], bounds.upper[index]);
            result.velocity[index] = workspace.solution[index];
        }
        let maximum_constraint_violation = maximum_constraint_violation_flat(
            &workspace.solution,
            constraints,
            &workspace.ordered_constraints,
        );
        let minimum_bound_margin = (0..dof)
            .map(|index| {
                (workspace.solution[index] - bounds.lower[index])
                    .min(bounds.upper[index] - workspace.solution[index])
            })
            .fold(f64::INFINITY, f64::min);
        let status = if maximum_constraint_violation > HARD_CONSTRAINT_TOLERANCE {
            SolveStatus::NumericalFailure
        } else if result.diagnostics.clipped_levels.is_empty() {
            SolveStatus::Solved
        } else {
            SolveStatus::SolvedWithSlack
        };
        result.diagnostics.status = status;
        result.diagnostics.minimum_bound_margin = minimum_bound_margin;
        record_hard_violation_witness(
            &mut result.diagnostics,
            &workspace.solution,
            bounds,
            constraints,
            &workspace.ordered_constraints,
        );
        debug_assert_eq!(
            result.diagnostics.maximum_constraint_violation.to_bits(),
            maximum_constraint_violation.to_bits()
        );
    }
}

/// Return true only when a soft row is already a constant zero residual over
/// the feasible set. Exact-zero rows are inert at every priority. A row backed
/// entirely by fixed coordinates is compacted only at terminal Style, where it
/// cannot alter lower-level clipping provenance or nullspace construction.
fn task_row_is_constant_and_satisfied(
    task: &Task,
    row: usize,
    priority: Priority,
    bounds: &VelocityBounds,
) -> bool {
    let mut value = 0.0;
    let mut has_nonzero_coefficient = false;
    for coordinate in 0..task.jacobian.ncols() {
        let coefficient = task.jacobian[(row, coordinate)];
        if coefficient == 0.0 {
            continue;
        }
        has_nonzero_coefficient = true;
        if !fixed_style_task_row_compaction_enabled()
            || priority != Priority::Style
            || !bounds.lower[coordinate].is_finite()
            || bounds.lower[coordinate] != bounds.upper[coordinate]
        {
            return false;
        }
        value += coefficient * bounds.lower[coordinate];
    }
    if !has_nonzero_coefficient {
        return zero_task_row_compaction_enabled() && task.target_velocity[row] == 0.0;
    }
    value == task.target_velocity[row]
}

#[inline]
const fn task_row_compaction_enabled() -> bool {
    zero_task_row_compaction_enabled() || fixed_style_task_row_compaction_enabled()
}

#[inline]
const fn zero_task_row_compaction_enabled() -> bool {
    !cfg!(feature = "resolved-task-row-compaction-control")
        || cfg!(feature = "resolved-task-row-compaction-experiment")
        || cfg!(feature = "zero-task-row-compaction-experiment")
}

#[inline]
const fn fixed_style_task_row_compaction_enabled() -> bool {
    !cfg!(feature = "resolved-task-row-compaction-control")
        || cfg!(feature = "resolved-task-row-compaction-experiment")
        || cfg!(feature = "fixed-style-task-row-compaction-experiment")
}

#[allow(clippy::too_many_arguments)]
fn seed_linear_equalities_into(
    dof: usize,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    equality_rows: usize,
    tolerance: f64,
    solution: &mut [f64],
    matrix: &mut [f64],
    target: &mut [f64],
    pseudo_inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
) {
    solution[..dof].fill(0.0);
    if equality_rows == 0 {
        return;
    }
    let mut row = 0;
    for constraint_index in ordered_constraints.iter().copied() {
        let constraint = &constraints[constraint_index];
        if !constraint.lower.is_finite() || constraint.lower != constraint.upper {
            continue;
        }
        let norm = constraint.coefficients.norm().max(1e-300);
        for coordinate in 0..dof {
            matrix[row * dof + coordinate] = constraint.coefficients[coordinate] / norm;
        }
        target[row] = constraint.lower / norm;
        row += 1;
    }
    debug_assert_eq!(row, equality_rows);
    pseudo_inverse_flat_into(
        matrix,
        equality_rows,
        dof,
        tolerance,
        pseudo_inverse,
        orthogonal_columns,
        right_vectors,
        singular_values,
    );
    multiply_matrix_vector(pseudo_inverse, dof, equality_rows, target, solution);
}

fn prepare_result(result: &mut SolveResult, dof: usize) {
    if result.velocity.len() != dof {
        result.velocity = DVector::zeros(dof);
    } else {
        result.velocity.fill(0.0);
    }
    result.diagnostics.status = SolveStatus::InvalidProblem;
    result.diagnostics.level_residuals.clear();
    result.diagnostics.minimum_bound_margin = f64::NEG_INFINITY;
    result.diagnostics.maximum_bound_violation = f64::INFINITY;
    result.diagnostics.limiting_bound_coordinate = None;
    result.diagnostics.limiting_bound_is_upper = false;
    result.diagnostics.clipped_levels.clear();
    result.diagnostics.rank_by_level.clear();
    result.diagnostics.active_constraints.clear();
    result.diagnostics.maximum_linear_constraint_violation = f64::INFINITY;
    result.diagnostics.limiting_linear_constraint = None;
    result.diagnostics.limiting_linear_constraint_is_upper = false;
    result.diagnostics.maximum_constraint_violation = f64::INFINITY;
    result.diagnostics.task_pseudoinverse_calls = 0;
    result.diagnostics.task_pseudoinverse_calls_by_level.fill(0);
    result.diagnostics.task_jacobi_sweeps = 0;
    result.diagnostics.task_jacobi_sweeps_by_level.fill(0);
    result.diagnostics.clipped_steps = 0;
    result.diagnostics.clipped_steps_by_level.fill(0);
    result.diagnostics.equality_pseudoinverse_reused = false;
    result.diagnostics.feasibility_projection_sweeps = 0;
    result.diagnostics.feasibility_halfspace_projections = 0;
    result.diagnostics.feasibility_polish_iterations = 0;
    result.diagnostics.feasibility_polish_pseudoinverse_calls = 0;
    result.diagnostics.feasibility_polish_jacobi_sweeps = 0;
    result.diagnostics.feasibility_seed_reused = false;
    result.diagnostics.feasibility_prefix_resumed = false;
}

fn feasibility_problem_key_capacity(dof: usize, constraints: usize) -> usize {
    16_usize
        .saturating_add(dof.saturating_mul(2))
        .saturating_add(constraints.saturating_mul(dof.saturating_add(4)))
}

fn reserve_total_capacity(values: &mut Vec<u64>, required: usize) {
    if values.capacity() < required {
        values.reserve(required.saturating_sub(values.len()));
    }
}

#[allow(clippy::too_many_arguments)]
fn build_feasibility_problem_key(
    dof: usize,
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    singular_value_tolerance: f64,
    maximum_feasibility_iterations: usize,
    maximum_feasibility_projection_sweeps: Option<usize>,
    feasibility_projection_continuation_violation_threshold: Option<f64>,
    repair_feasibility_equalities_before_inequalities: bool,
    use_feasibility_row_spans: bool,
    out: &mut Vec<u64>,
) {
    out.clear();
    out.push(dof as u64);
    out.push(constraints.len() as u64);
    out.push(singular_value_tolerance.to_bits());
    out.push(maximum_feasibility_iterations as u64);
    out.push(maximum_feasibility_projection_sweeps.unwrap_or(usize::MAX) as u64);
    out.push(
        feasibility_projection_continuation_violation_threshold.map_or(u64::MAX, f64::to_bits),
    );
    out.push(repair_feasibility_equalities_before_inequalities as u64);
    out.push(use_feasibility_row_spans as u64);
    for index in 0..dof {
        out.push(bounds.lower[index].to_bits());
        out.push(bounds.upper[index].to_bits());
    }
    for constraint_index in ordered_constraints.iter().copied() {
        let constraint = &constraints[constraint_index];
        out.push(constraint.stable_id as u64);
        out.push(constraint.lower.to_bits());
        out.push(constraint.upper.to_bits());
        out.push(constraint.coefficients.len() as u64);
        out.extend(constraint.coefficients.iter().map(|value| value.to_bits()));
    }
}

/// Exact keys include the local projection ceiling and continuation threshold
/// at slots four and five. A terminal witness that reached hard feasibility
/// without depending on those stopping controls may cross that profile
/// boundary. An exhausted bounded prefix may also cross only so an uncapped
/// destination can resume the identical Dykstra state. Every structural,
/// arithmetic, bound, and ordered-row bit must still match.
fn feasibility_problem_keys_match(
    current: &[u64],
    cached: &[u64],
    budget_independent: bool,
    prefix_continuable: bool,
) -> bool {
    current == cached
        || ((budget_independent || prefix_continuable)
            && current.len() == cached.len()
            && current.len() >= 8
            // Compatibility is deliberately one-way: an uncapped authority
            // may consume a terminal exact prefix from a bounded planner. A
            // bounded destination must retain its own fail-closed stopping
            // semantics rather than inheriting an uncapped result.
            && current[4] == usize::MAX as u64
            && current[..4] == cached[..4]
            && current[6..] == cached[6..])
}

fn exact_seed_is_projection_budget_independent(
    maximum_projection_sweeps: Option<usize>,
    active_set_pseudoinverse_calls: usize,
) -> bool {
    maximum_projection_sweeps.is_none_or(|ceiling| {
        ceiling >= DYKSTRA_SWEEP_LIMIT_BEFORE_ACTIVE_SET.unwrap_or(usize::MAX)
            || active_set_pseudoinverse_calls == 0
    })
}

fn exhausted_prefix_is_projection_continuable(maximum_projection_sweeps: Option<usize>) -> bool {
    maximum_projection_sweeps.is_none_or(|ceiling| {
        ceiling >= DYKSTRA_SWEEP_LIMIT_BEFORE_ACTIVE_SET.unwrap_or(usize::MAX)
    })
}

fn fail_into(result: &mut SolveResult, status: SolveStatus) {
    result.velocity.fill(0.0);
    result.diagnostics.status = status;
    result.diagnostics.minimum_bound_margin = f64::NEG_INFINITY;
    if !result.diagnostics.maximum_constraint_violation.is_finite() {
        result.diagnostics.maximum_constraint_violation = f64::INFINITY;
    }
}

fn stable_insertion_sort_by_key<T: Copy, K: Ord>(values: &mut [T], mut key: impl FnMut(T) -> K) {
    for index in 1..values.len() {
        let value = values[index];
        let value_key = key(value);
        let mut insertion = index;
        while insertion > 0 && key(values[insertion - 1]) > value_key {
            values[insertion] = values[insertion - 1];
            insertion -= 1;
        }
        values[insertion] = value;
    }
}

fn fill_identity(matrix: &mut [f64], dimension: usize) {
    matrix[..dimension * dimension].fill(0.0);
    for index in 0..dimension {
        matrix[index * dimension + index] = 1.0;
    }
}

fn multiply(
    left: &[f64],
    left_rows: usize,
    inner: usize,
    right: &[f64],
    right_columns: usize,
    output: &mut [f64],
) {
    if USE_DENSE_MULTIPLY_ROW_SLICES {
        multiply_row_slices(left, left_rows, inner, right, right_columns, output);
    } else {
        multiply_flat_indices(left, left_rows, inner, right, right_columns, output);
    }
}

fn multiply_flat_indices(
    left: &[f64],
    left_rows: usize,
    inner: usize,
    right: &[f64],
    right_columns: usize,
    output: &mut [f64],
) {
    output[..left_rows * right_columns].fill(0.0);
    for row in 0..left_rows {
        for shared in 0..inner {
            let left_value = left[row * inner + shared];
            if left_value == 0.0 {
                continue;
            }
            for column in 0..right_columns {
                output[row * right_columns + column] +=
                    left_value * right[shared * right_columns + column];
            }
        }
    }
}

fn multiply_row_slices(
    left: &[f64],
    left_rows: usize,
    inner: usize,
    right: &[f64],
    right_columns: usize,
    output: &mut [f64],
) {
    output[..left_rows * right_columns].fill(0.0);
    for row in 0..left_rows {
        let left_row = &left[row * inner..(row + 1) * inner];
        let output_row = &mut output[row * right_columns..(row + 1) * right_columns];
        for shared in 0..inner {
            let left_value = left_row[shared];
            if left_value == 0.0 {
                continue;
            }
            let right_row = &right[shared * right_columns..(shared + 1) * right_columns];
            for column in 0..right_columns {
                output_row[column] += left_value * right_row[column];
            }
        }
    }
}

fn multiply_matrix_vector(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    vector: &[f64],
    output: &mut [f64],
) {
    for row in 0..rows {
        let mut value = 0.0;
        for column in 0..columns {
            value += matrix[row * columns + column] * vector[column];
        }
        output[row] = value;
    }
}

fn matrix_vector_residual(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    vector: &[f64],
    target: &[f64],
    output: &mut [f64],
) {
    for row in 0..rows {
        let mut value = -target[row];
        for column in 0..columns {
            value += matrix[row * columns + column] * vector[column];
        }
        output[row] = -value;
    }
}

fn squared_norm(vector: &[f64]) -> f64 {
    vector.iter().map(|value| value * value).sum()
}

fn initialize_jacobi_column_energies(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    column_energies: &mut [f64],
    fuse_frobenius_scan: bool,
) -> f64 {
    if fuse_frobenius_scan {
        let mut frobenius_squared = 0.0;
        for column in 0..columns {
            let mut norm_squared = 0.0;
            for row in 0..rows {
                let value = matrix[column * rows + row];
                let square = value * value;
                frobenius_squared += square;
                norm_squared += square;
            }
            column_energies[column] = norm_squared;
        }
        frobenius_squared
    } else {
        let frobenius_squared = squared_norm(&matrix[..rows * columns]);
        for column in 0..columns {
            let mut norm_squared = 0.0;
            for row in 0..rows {
                let value = matrix[column * rows + row];
                norm_squared += value * value;
            }
            column_energies[column] = norm_squared;
        }
        frobenius_squared
    }
}

#[derive(Clone, Copy, Debug)]
struct PseudoinverseOutcome {
    rank: usize,
    jacobi_sweeps: usize,
}

/// Pseudoinverse of a row-major `rows × columns` matrix into a row-major
/// `columns × rows` output, using preallocated one-sided Jacobi storage.
#[allow(clippy::too_many_arguments)]
fn pseudo_inverse_flat_into(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    tolerance: f64,
    inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
) -> usize {
    pseudo_inverse_flat_into_profiled(
        matrix,
        rows,
        columns,
        tolerance,
        0.0,
        inverse,
        orthogonal_columns,
        right_vectors,
        singular_values,
    )
    .rank
}

/// Pseudoinverse plus work attribution for task-level diagnostics.
#[allow(clippy::too_many_arguments)]
fn pseudo_inverse_flat_into_profiled(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    tolerance: f64,
    damping: f64,
    inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
) -> PseudoinverseOutcome {
    pseudo_inverse_flat_into_profiled_with_energy_cache(
        matrix,
        rows,
        columns,
        tolerance,
        damping,
        inverse,
        orthogonal_columns,
        right_vectors,
        singular_values,
        CACHE_JACOBI_COLUMN_ENERGIES,
        SKIP_DISCARDED_JACOBI_COUPLINGS,
    )
}

/// Task-only dispatch for the guarded row-Gram experiment.
///
/// For a wide, full-row-rank task matrix `A`, the damped inverse is
/// `A^T (A A^T + lambda^2 I)^-1`. Factoring the small row Gram matrix avoids
/// rotating `rows * (rows - 1) / 2` column pairs across every generalized
/// coordinate on each Jacobi sweep. Forming `A A^T` squares the condition
/// number, however, so this path is deliberately restricted to matrices whose
/// unregularized Cholesky pivots remain far above both the SVD truncation scale
/// and a conservative floating-point cancellation floor. Everything else uses
/// the established one-sided Jacobi kernel.
#[allow(clippy::too_many_arguments)]
fn task_pseudo_inverse_flat_into_profiled(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    tolerance: f64,
    damping: f64,
    inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
) -> PseudoinverseOutcome {
    if cfg!(feature = "rank-one-task-pseudoinverse-experiment") && rows == 1 {
        return pseudo_inverse_single_row_into(
            matrix,
            columns,
            tolerance,
            damping,
            inverse,
            singular_values,
        );
    }
    if cfg!(feature = "row-gram-task-pseudoinverse-experiment")
        && rows <= columns
        && let Some(outcome) = pseudo_inverse_wide_row_gram_into(
            matrix,
            rows,
            columns,
            tolerance,
            damping,
            inverse,
            orthogonal_columns,
            singular_values,
        )
    {
        return outcome;
    }
    pseudo_inverse_flat_into_profiled(
        matrix,
        rows,
        columns,
        tolerance,
        damping,
        inverse,
        orthogonal_columns,
        right_vectors,
        singular_values,
    )
}

/// Arithmetic-equivalent specialization of the established wide-matrix
/// branch when the projected task contains one row. There are no column pairs
/// to rotate. The ordinary kernel's first Frobenius scan only derives a Jacobi
/// discard floor that cannot be consumed in this shape, so the retained norm
/// scan directly supplies the same singular value and inverse factor.
fn pseudo_inverse_single_row_into(
    matrix: &[f64],
    columns: usize,
    tolerance: f64,
    damping: f64,
    inverse: &mut [f64],
    singular_values: &mut [f64],
) -> PseudoinverseOutcome {
    debug_assert!(matrix.len() >= columns);
    debug_assert!(inverse.len() >= columns);
    debug_assert!(!singular_values.is_empty());

    let mut norm_squared = 0.0;
    for value in matrix.iter().take(columns) {
        norm_squared += value * value;
    }
    let singular_value = norm_squared.max(0.0).sqrt();
    singular_values[0] = singular_value;
    let threshold = tolerance * singular_value.max(1.0);
    if singular_value <= threshold {
        inverse[..columns].fill(0.0);
        return PseudoinverseOutcome {
            rank: 0,
            jacobi_sweeps: 0,
        };
    }

    let inverse_squared = 1.0 / (singular_value * singular_value + damping * damping);
    for coordinate in 0..columns {
        inverse[coordinate] = 0.0 + inverse_squared * 1.0 * matrix[coordinate];
    }
    PseudoinverseOutcome {
        rank: 1,
        jacobi_sweeps: 0,
    }
}

/// Guarded pseudoinverse for a wide `rows x columns` matrix. Returns `None`
/// before writing `inverse` when the row Gram matrix is not safely positive
/// definite, allowing a bit-for-bit ordinary-kernel fallback from the caller.
fn pseudo_inverse_wide_row_gram_into(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    tolerance: f64,
    damping: f64,
    inverse: &mut [f64],
    gram: &mut [f64],
    solve: &mut [f64],
) -> Option<PseudoinverseOutcome> {
    if rows == 0 || columns == 0 || rows > columns {
        return None;
    }
    debug_assert!(matrix.len() >= rows * columns);
    debug_assert!(inverse.len() >= columns * rows);
    debug_assert!(gram.len() >= rows * rows);
    debug_assert!(solve.len() >= rows);

    let mut frobenius_squared = 0.0;
    for row in 0..rows {
        for column in 0..columns {
            let value = matrix[row * columns + column];
            frobenius_squared += value * value;
        }
    }
    let singular_scale = frobenius_squared.sqrt().max(1.0);
    let truncation_guard = (16.0 * tolerance * singular_scale).powi(2);
    let cancellation_guard =
        4096.0 * f64::EPSILON * frobenius_squared.max(1.0) * rows.max(1) as f64;
    let pivot_guard = truncation_guard.max(cancellation_guard);

    build_row_gram(matrix, rows, columns, 0.0, gram);
    if !cholesky_factor_lower(gram, rows, pivot_guard) {
        return None;
    }

    if damping > 0.0 {
        build_row_gram(matrix, rows, columns, damping * damping, gram);
        if !cholesky_factor_lower(gram, rows, 0.0) {
            return None;
        }
    }

    for coordinate in 0..columns {
        for row in 0..rows {
            let mut value = matrix[row * columns + coordinate];
            for inner in 0..row {
                value -= gram[row * rows + inner] * solve[inner];
            }
            solve[row] = value / gram[row * rows + row];
        }
        for row in (0..rows).rev() {
            let mut value = solve[row];
            for inner in row + 1..rows {
                value -= gram[inner * rows + row] * solve[inner];
            }
            solve[row] = value / gram[row * rows + row];
        }
        inverse[coordinate * rows..(coordinate + 1) * rows].copy_from_slice(&solve[..rows]);
    }

    Some(PseudoinverseOutcome {
        rank: rows,
        jacobi_sweeps: 0,
    })
}

fn build_row_gram(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    diagonal_shift: f64,
    gram: &mut [f64],
) {
    for row in 0..rows {
        for column in 0..=row {
            let mut value = 0.0;
            for coordinate in 0..columns {
                value += matrix[row * columns + coordinate] * matrix[column * columns + coordinate];
            }
            if row == column {
                value += diagonal_shift;
            }
            gram[row * rows + column] = value;
            gram[column * rows + row] = value;
        }
    }
}

/// In-place lower Cholesky factorization. The guard is applied to each Schur
/// pivot before its square root and is intentionally stricter than positivity.
fn cholesky_factor_lower(matrix: &mut [f64], size: usize, pivot_guard: f64) -> bool {
    for row in 0..size {
        for column in 0..=row {
            let mut value = matrix[row * size + column];
            for inner in 0..column {
                value -= matrix[row * size + inner] * matrix[column * size + inner];
            }
            if row == column {
                if !value.is_finite() || value <= pivot_guard {
                    return false;
                }
                matrix[row * size + row] = value.sqrt();
            } else {
                matrix[row * size + column] = value / matrix[column * size + column];
            }
        }
    }
    true
}

#[allow(clippy::too_many_arguments)]
fn pseudo_inverse_flat_into_profiled_with_energy_cache(
    matrix: &[f64],
    rows: usize,
    columns: usize,
    tolerance: f64,
    damping: f64,
    inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
    cache_column_energies: bool,
    skip_discarded_couplings: bool,
) -> PseudoinverseOutcome {
    inverse[..columns * rows].fill(0.0);
    if rows == 0 || columns == 0 {
        return PseudoinverseOutcome {
            rank: 0,
            jacobi_sweeps: 0,
        };
    }

    let transposed = columns > rows;
    let (tall_rows, tall_columns) = if transposed {
        (columns, rows)
    } else {
        (rows, columns)
    };
    if transposed {
        // B = Aᵀ in column-major storage.
        for row in 0..columns {
            for column in 0..rows {
                orthogonal_columns[column * tall_rows + row] = matrix[column * columns + row];
            }
        }
    } else {
        for row in 0..rows {
            for column in 0..columns {
                orthogonal_columns[column * tall_rows + row] = matrix[row * columns + column];
            }
        }
    }
    fill_identity(right_vectors, tall_columns);
    let frobenius_norm = initialize_jacobi_column_energies(
        orthogonal_columns,
        tall_rows,
        tall_columns,
        singular_values,
        FUSE_INITIAL_JACOBI_ENERGY_SCAN,
    )
    .sqrt();
    // Stop rotating numerically null columns well below the actual singular
    // truncation threshold. The 1e-3 guard band preserves near-threshold
    // singular directions while bounding work on accumulated null spaces.
    let discarded_energy_floor = (tolerance * 1e-3 * frobenius_norm.max(1.0)).powi(2);
    let jacobi_sweeps = one_sided_jacobi_flat(
        orthogonal_columns,
        tall_rows,
        tall_columns,
        right_vectors,
        singular_values,
        cache_column_energies,
        skip_discarded_couplings,
        discarded_energy_floor,
        USE_JACOBI_COLUMN_SLICES,
        USE_JACOBI_COLUMN_ITERATORS,
    );
    if !cache_column_energies {
        for column in 0..tall_columns {
            let mut norm_squared = 0.0;
            for row in 0..tall_rows {
                let value = orthogonal_columns[column * tall_rows + row];
                norm_squared += value * value;
            }
            singular_values[column] = norm_squared;
        }
    }
    let mut maximum_singular_value: f64 = 0.0;
    for singular_value in singular_values.iter_mut().take(tall_columns) {
        *singular_value = singular_value.max(0.0).sqrt();
        maximum_singular_value = maximum_singular_value.max(*singular_value);
    }
    let threshold = tolerance * maximum_singular_value.max(1.0);
    let mut rank = 0;
    for singular_vector in 0..tall_columns {
        let singular_value = singular_values[singular_vector];
        if singular_value <= threshold {
            continue;
        }
        rank += 1;
        let inverse_squared = 1.0 / (singular_value * singular_value + damping * damping);
        if transposed {
            // B = Aᵀ and A⁺ = (B⁺)ᵀ.
            for output_row in 0..columns {
                for output_column in 0..rows {
                    inverse[output_row * rows + output_column] += inverse_squared
                        * right_vectors[singular_vector * tall_columns + output_column]
                        * orthogonal_columns[singular_vector * tall_rows + output_row];
                }
            }
        } else {
            for output_row in 0..columns {
                for output_column in 0..rows {
                    inverse[output_row * rows + output_column] += inverse_squared
                        * right_vectors[singular_vector * tall_columns + output_row]
                        * orthogonal_columns[singular_vector * tall_rows + output_column];
                }
            }
        }
    }
    PseudoinverseOutcome {
        rank,
        jacobi_sweeps,
    }
}

fn one_sided_jacobi_flat(
    matrix: &mut [f64],
    rows: usize,
    columns: usize,
    right_vectors: &mut [f64],
    column_energies: &mut [f64],
    cache_column_energies: bool,
    skip_discarded_couplings: bool,
    discarded_energy_floor: f64,
    use_column_slices: bool,
    use_column_iterators: bool,
) -> usize {
    if columns < 2 {
        return 0;
    }
    let mut sweeps = 0;
    for _ in 0..256 {
        sweeps += 1;
        let mut rotated = false;
        for p in 0..columns - 1 {
            for q in p + 1..columns {
                let mut alpha = column_energies[p];
                let mut beta = column_energies[q];
                // Cached energies already prove that this pair will be
                // rejected below. Avoid the full coupling dot without
                // changing any value subsequently consumed by the solve.
                if cache_column_energies
                    && skip_discarded_couplings
                    && (alpha <= discarded_energy_floor || beta <= discarded_energy_floor)
                {
                    continue;
                }
                let mut coupling = 0.0;
                if use_column_slices {
                    let p_column = &matrix[p * rows..(p + 1) * rows];
                    let q_column = &matrix[q * rows..(q + 1) * rows];
                    if use_column_iterators {
                        for (p_value, q_value) in p_column.iter().zip(q_column) {
                            coupling += p_value * q_value;
                        }
                    } else {
                        for row in 0..rows {
                            coupling += p_column[row] * q_column[row];
                        }
                    }
                } else {
                    for row in 0..rows {
                        let p_value = matrix[p * rows + row];
                        let q_value = matrix[q * rows + row];
                        coupling += p_value * q_value;
                    }
                }
                if !cache_column_energies {
                    alpha = 0.0;
                    beta = 0.0;
                    if use_column_slices {
                        let p_column = &matrix[p * rows..(p + 1) * rows];
                        let q_column = &matrix[q * rows..(q + 1) * rows];
                        if use_column_iterators {
                            for (p_value, q_value) in p_column.iter().zip(q_column) {
                                alpha += p_value * p_value;
                                beta += q_value * q_value;
                            }
                        } else {
                            for row in 0..rows {
                                let p_value = p_column[row];
                                let q_value = q_column[row];
                                alpha += p_value * p_value;
                                beta += q_value * q_value;
                            }
                        }
                    } else {
                        for row in 0..rows {
                            let p_value = matrix[p * rows + row];
                            let q_value = matrix[q * rows + row];
                            alpha += p_value * p_value;
                            beta += q_value * q_value;
                        }
                    }
                }
                let scale = (alpha * beta).max(0.0).sqrt();
                if alpha <= discarded_energy_floor
                    || beta <= discarded_energy_floor
                    || scale == 0.0
                    || coupling.abs() <= 64.0 * f64::EPSILON * rows.max(1) as f64 * scale
                {
                    continue;
                }
                rotated = true;
                let tau = (beta - alpha) / (2.0 * coupling);
                let tangent = if tau >= 0.0 {
                    1.0 / (tau + (1.0 + tau * tau).sqrt())
                } else {
                    -1.0 / (-tau + (1.0 + tau * tau).sqrt())
                };
                let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
                let sine = tangent * cosine;
                if use_column_slices {
                    let (before_q, q_and_after) = matrix.split_at_mut(q * rows);
                    let p_column = &mut before_q[p * rows..(p + 1) * rows];
                    let q_column = &mut q_and_after[..rows];
                    if use_column_iterators {
                        for (p_value, q_value) in p_column.iter_mut().zip(q_column) {
                            let previous_p = *p_value;
                            let previous_q = *q_value;
                            *p_value = cosine * previous_p - sine * previous_q;
                            *q_value = sine * previous_p + cosine * previous_q;
                        }
                    } else {
                        for row in 0..rows {
                            let p_value = p_column[row];
                            let q_value = q_column[row];
                            p_column[row] = cosine * p_value - sine * q_value;
                            q_column[row] = sine * p_value + cosine * q_value;
                        }
                    }
                } else {
                    for row in 0..rows {
                        let p_value = matrix[p * rows + row];
                        let q_value = matrix[q * rows + row];
                        matrix[p * rows + row] = cosine * p_value - sine * q_value;
                        matrix[q * rows + row] = sine * p_value + cosine * q_value;
                    }
                }
                if use_column_slices {
                    let (before_q, q_and_after) = right_vectors.split_at_mut(q * columns);
                    let p_column = &mut before_q[p * columns..(p + 1) * columns];
                    let q_column = &mut q_and_after[..columns];
                    if use_column_iterators {
                        for (p_value, q_value) in p_column.iter_mut().zip(q_column) {
                            let previous_p = *p_value;
                            let previous_q = *q_value;
                            *p_value = cosine * previous_p - sine * previous_q;
                            *q_value = sine * previous_p + cosine * previous_q;
                        }
                    } else {
                        for row in 0..columns {
                            let p_value = p_column[row];
                            let q_value = q_column[row];
                            p_column[row] = cosine * p_value - sine * q_value;
                            q_column[row] = sine * p_value + cosine * q_value;
                        }
                    }
                } else {
                    for row in 0..columns {
                        let p_value = right_vectors[p * columns + row];
                        let q_value = right_vectors[q * columns + row];
                        right_vectors[p * columns + row] = cosine * p_value - sine * q_value;
                        right_vectors[q * columns + row] = sine * p_value + cosine * q_value;
                    }
                }
                if cache_column_energies {
                    let twice_cross = 2.0 * sine * cosine * coupling;
                    let cosine_squared = cosine * cosine;
                    let sine_squared = sine * sine;
                    column_energies[p] =
                        (cosine_squared * alpha - twice_cross + sine_squared * beta).max(0.0);
                    column_energies[q] =
                        (sine_squared * alpha + twice_cross + cosine_squared * beta).max(0.0);
                }
            }
        }
        if cache_column_energies && rotated {
            // Re-anchor the recursively updated energies at each sweep
            // boundary. This retains the exact column-wise summation order
            // while avoiding two full dot products for every Jacobi pair.
            for column in 0..columns {
                let mut norm_squared = 0.0;
                if use_column_slices {
                    let values = &matrix[column * rows..(column + 1) * rows];
                    for value in values {
                        norm_squared += value * value;
                    }
                } else {
                    for row in 0..rows {
                        let value = matrix[column * rows + row];
                        norm_squared += value * value;
                    }
                }
                column_energies[column] = norm_squared;
            }
        }
        if !rotated {
            break;
        }
    }
    sweeps
}

fn maximum_feasible_step_flat(
    current: &[f64],
    delta: &[f64],
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
) -> (f64, Option<ConstraintHit>) {
    let mut alpha: f64 = 1.0;
    let mut hit = None;
    for index in 0..current.len() {
        let candidate = if delta[index] > 0.0 && bounds.upper[index].is_finite() {
            Some((bounds.upper[index] - current[index]) / delta[index])
        } else if delta[index] < 0.0 && bounds.lower[index].is_finite() {
            Some((bounds.lower[index] - current[index]) / delta[index])
        } else {
            None
        };
        if let Some(candidate) = candidate
            && candidate < alpha - 1e-14
        {
            alpha = candidate;
            hit = Some(ConstraintHit::Coordinate(index));
        }
    }
    for (ordered_index, constraint_index) in ordered_constraints.iter().copied().enumerate() {
        let constraint = &constraints[constraint_index];
        // Corrections are constructed in the null space of finite equality
        // rows. Re-treating those rows as one-sided step limits makes
        // round-off at the seeded solution clip every semantic task.
        if constraint.lower.is_finite() && constraint.lower == constraint.upper {
            continue;
        }
        let value = row_dot_slice(&constraint.coefficients, current);
        let rate = row_dot_slice(&constraint.coefficients, delta);
        let candidate = if rate > 1e-14 && constraint.upper.is_finite() {
            Some((constraint.upper - value) / rate)
        } else if rate < -1e-14 && constraint.lower.is_finite() {
            Some((constraint.lower - value) / rate)
        } else {
            None
        };
        if let Some(candidate) = candidate
            && candidate < alpha - 1e-14
        {
            alpha = candidate;
            hit = Some(ConstraintHit::Linear(ordered_index));
        }
    }
    (alpha.clamp(0.0, 1.0), hit)
}

fn freeze_coordinate_in_nullspace(
    nullspace: &mut [f64],
    dof: usize,
    index: usize,
    bound_row: &mut [f64],
    rank_one_column: &mut [f64],
) -> bool {
    bound_row[..dof].copy_from_slice(&nullspace[index * dof..(index + 1) * dof]);
    freeze_nullspace_row(nullspace, dof, bound_row, rank_one_column)
}

fn build_task_nullspace_projector(
    dof: usize,
    task_rows: usize,
    nullspace: &[f64],
    task_pseudoinverse: &[f64],
    projected_task: &[f64],
    nullspace_times_pseudoinverse: &mut [f64],
    task_nullspace: &mut [f64],
) {
    multiply(
        nullspace,
        dof,
        dof,
        task_pseudoinverse,
        task_rows,
        nullspace_times_pseudoinverse,
    );
    multiply(
        nullspace_times_pseudoinverse,
        dof,
        task_rows,
        projected_task,
        dof,
        task_nullspace,
    );
    for index in 0..dof * dof {
        task_nullspace[index] = nullspace[index] - task_nullspace[index];
    }
}

/// Preserve the current task optimum after one new hard-limit hit by moving
/// the remaining correction through the task nullspace. Returns false when the
/// bound has no usable direction in that nullspace, which requires the ordinary
/// projected SVD to compute the newly clipped optimum.
#[allow(clippy::too_many_arguments)]
fn repair_correction_through_task_nullspace(
    hit: ConstraintHit,
    dof: usize,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    task_nullspace: &mut [f64],
    correction: &mut [f64],
    bound_row: &mut [f64],
    rank_one_column: &mut [f64],
) -> bool {
    let rate = match hit {
        ConstraintHit::Coordinate(index) => {
            bound_row[..dof].copy_from_slice(&task_nullspace[index * dof..(index + 1) * dof]);
            correction[index]
        }
        ConstraintHit::Linear(index) => {
            let coefficients = &constraints[ordered_constraints[index]].coefficients;
            for column in 0..dof {
                let mut value = 0.0;
                for inner in 0..dof {
                    value += coefficients[inner] * task_nullspace[inner * dof + column];
                }
                bound_row[column] = value;
            }
            row_dot_slice(coefficients, &correction[..dof])
        }
    };
    let denominator = squared_norm(&bound_row[..dof]);
    if denominator <= 1e-20 {
        return false;
    }
    let scale = rate / denominator;
    for coordinate in 0..dof {
        correction[coordinate] -= bound_row[coordinate] * scale;
    }
    freeze_nullspace_row(task_nullspace, dof, bound_row, rank_one_column)
}

fn freeze_linear_in_nullspace(
    nullspace: &mut [f64],
    dof: usize,
    coefficients: &RowDVector<f64>,
    bound_row: &mut [f64],
    rank_one_column: &mut [f64],
) -> bool {
    for column in 0..dof {
        let mut value = 0.0;
        for inner in 0..dof {
            value += coefficients[inner] * nullspace[inner * dof + column];
        }
        bound_row[column] = value;
    }
    freeze_nullspace_row(nullspace, dof, bound_row, rank_one_column)
}

/// Freeze one newly active row while retaining all previously accumulated
/// null-space restrictions. Returns false when the row was already frozen.
fn freeze_nullspace_row(
    nullspace: &mut [f64],
    dof: usize,
    bound_row: &[f64],
    rank_one_column: &mut [f64],
) -> bool {
    let norm_squared = squared_norm(&bound_row[..dof]);
    if norm_squared <= 1e-24 {
        return false;
    }
    for row in 0..dof {
        let mut value = 0.0;
        for column in 0..dof {
            value += nullspace[row * dof + column] * bound_row[column];
        }
        rank_one_column[row] = value / norm_squared;
    }
    for row in 0..dof {
        for column in 0..dof {
            nullspace[row * dof + column] -= rank_one_column[row] * bound_row[column];
        }
    }
    true
}

#[allow(clippy::too_many_arguments)]
fn freeze_coincident_step_limits(
    nullspace: &mut [f64],
    dof: usize,
    primary_hit: ConstraintHit,
    alpha: f64,
    solution: &[f64],
    correction: &[f64],
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    active_constraints: &mut Vec<u32>,
    bound_row: &mut [f64],
    rank_one_column: &mut [f64],
) {
    if !cfg!(feature = "coincident-step-limit-freeze-experiment") {
        return;
    }

    for index in 0..dof {
        if matches!(primary_hit, ConstraintHit::Coordinate(primary) if primary == index) {
            continue;
        }
        let (rate, limit) = if correction[index] > 1e-14 && bounds.upper[index].is_finite() {
            (correction[index], bounds.upper[index])
        } else if correction[index] < -1e-14 && bounds.lower[index].is_finite() {
            (correction[index], bounds.lower[index])
        } else {
            continue;
        };
        let previous = solution[index] - alpha * rate;
        let candidate = (limit - previous) / rate;
        if candidate.to_bits() == alpha.to_bits() {
            freeze_coordinate_in_nullspace(nullspace, dof, index, bound_row, rank_one_column);
        }
    }

    for (ordered_index, constraint_index) in ordered_constraints.iter().copied().enumerate() {
        if matches!(primary_hit, ConstraintHit::Linear(primary) if primary == ordered_index) {
            continue;
        }
        let constraint = &constraints[constraint_index];
        if constraint.lower.is_finite() && constraint.lower == constraint.upper {
            continue;
        }
        let value = row_dot_slice(&constraint.coefficients, solution);
        let rate = row_dot_slice(&constraint.coefficients, correction);
        let limit = if rate > 1e-14 && constraint.upper.is_finite() {
            constraint.upper
        } else if rate < -1e-14 && constraint.lower.is_finite() {
            constraint.lower
        } else {
            continue;
        };
        let previous = value - alpha * rate;
        let candidate = (limit - previous) / rate;
        if candidate.to_bits() == alpha.to_bits()
            && freeze_linear_in_nullspace(
                nullspace,
                dof,
                &constraint.coefficients,
                bound_row,
                rank_one_column,
            )
            && !active_constraints.contains(&constraint.stable_id)
        {
            active_constraints.push(constraint.stable_id);
        }
    }
}

fn find_feasible_point_into(
    dof: usize,
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    solution: &mut [f64],
    halfspaces: &mut Vec<Halfspace>,
    nonzero_indices: &mut Vec<usize>,
    nonzero_values: &mut Vec<f64>,
    multipliers: &mut Vec<f64>,
    maximum_projection_sweeps: Option<usize>,
    use_row_spans: bool,
) -> FeasibilitySeedOutcome {
    if let Err(maximum_violation) = build_halfspaces_into(
        dof,
        bounds,
        constraints,
        ordered_constraints,
        halfspaces,
        nonzero_indices,
        nonzero_values,
        use_row_spans,
    ) {
        // A structurally contradictory zero-norm row has no projection
        // representation. Do not expose a partial halfspace list to the
        // hybrid active-set fallback.
        halfspaces.clear();
        multipliers.clear();
        return FeasibilitySeedOutcome {
            seed: FeasibilitySeed::StructurallyInfeasible(maximum_violation),
            projection_sweeps: 0,
            halfspace_projections: 0,
        };
    }
    multipliers.clear();
    multipliers.resize(halfspaces.len(), 0.0);
    let maximum_sweeps = DYKSTRA_SWEEP_LIMIT_BEFORE_ACTIVE_SET
        .unwrap_or_else(|| maximum_dykstra_sweeps(halfspaces.len()))
        .min(maximum_projection_sweeps.unwrap_or_else(|| maximum_dykstra_sweeps(halfspaces.len())));
    continue_feasible_point_into(
        bounds,
        constraints,
        ordered_constraints,
        solution,
        halfspaces,
        nonzero_indices,
        nonzero_values,
        multipliers,
        maximum_sweeps,
        use_row_spans,
    )
}

fn maximum_dykstra_sweeps(halfspace_count: usize) -> usize {
    512_usize.max(32 * halfspace_count)
}

#[allow(clippy::too_many_arguments)]
fn continue_feasible_point_into(
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    solution: &mut [f64],
    halfspaces: &[Halfspace],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    multipliers: &mut [f64],
    maximum_sweeps: usize,
    use_row_spans: bool,
) -> FeasibilitySeedOutcome {
    if use_row_spans {
        return continue_feasible_point_sparse_into(
            bounds,
            constraints,
            ordered_constraints,
            solution,
            halfspaces,
            nonzero_indices,
            nonzero_values,
            multipliers,
            maximum_sweeps,
        );
    }
    let mut projection_sweeps = 0;
    let mut halfspace_projections = 0;
    let mut negative_zeros = NegativeZeroTracker::new(solution, use_row_spans);
    for _ in 0..maximum_sweeps {
        for (halfspace_index, halfspace) in halfspaces.iter().copied().enumerate() {
            let (value, limit, norm_squared) = halfspace_value_limit_norm_flat_with_spans(
                halfspace,
                solution,
                bounds,
                constraints,
                ordered_constraints,
                nonzero_indices,
                nonzero_values,
                use_row_spans,
            );
            let previous = multipliers[halfspace_index];
            let excess = value - limit;
            let normalized_excess = if norm_squared == 1.0 {
                excess
            } else {
                excess / norm_squared
            };
            let next = (previous + normalized_excess).max(0.0);
            let delta = next - previous;
            multipliers[halfspace_index] = next;
            if !SKIP_ZERO_FEASIBILITY_UPDATES || delta != 0.0 {
                apply_halfspace_transpose_flat(
                    halfspace,
                    -delta,
                    solution,
                    constraints,
                    ordered_constraints,
                    nonzero_indices,
                    nonzero_values,
                    use_row_spans,
                    &mut negative_zeros,
                );
            }
        }
        projection_sweeps += 1;
        halfspace_projections += halfspaces.len();
        let maximum_violation = maximum_bound_violation_slice(solution, bounds).max(
            maximum_constraint_violation_for_dykstra(
                solution,
                constraints,
                ordered_constraints,
                halfspaces,
                nonzero_indices,
                nonzero_values,
                use_row_spans,
            ),
        );
        if maximum_violation <= HARD_CONSTRAINT_TOLERANCE {
            return FeasibilitySeedOutcome {
                seed: FeasibilitySeed::Exact,
                projection_sweeps,
                halfspace_projections,
            };
        }
        if maximum_violation <= FEASIBILITY_POLISH_TRIGGER {
            return FeasibilitySeedOutcome {
                seed: FeasibilitySeed::Near,
                projection_sweeps,
                halfspace_projections,
            };
        }
    }
    let maximum_violation = maximum_bound_violation_slice(solution, bounds).max(
        maximum_constraint_violation_for_dykstra(
            solution,
            constraints,
            ordered_constraints,
            halfspaces,
            nonzero_indices,
            nonzero_values,
            use_row_spans,
        ),
    );
    FeasibilitySeedOutcome {
        seed: FeasibilitySeed::Exhausted(maximum_violation),
        projection_sweeps,
        halfspace_projections,
    }
}

#[allow(clippy::too_many_arguments)]
fn continue_feasible_point_sparse_into(
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    solution: &mut [f64],
    halfspaces: &[Halfspace],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    multipliers: &mut [f64],
    maximum_sweeps: usize,
) -> FeasibilitySeedOutcome {
    let mut projection_sweeps = 0;
    let mut halfspace_projections = 0;
    let mut negative_zeros = NegativeZeroTracker::new(solution, true);
    for _ in 0..maximum_sweeps {
        for (halfspace_index, halfspace) in halfspaces.iter().copied().enumerate() {
            let (value, limit, norm_squared) = halfspace_value_limit_norm_sparse(
                halfspace,
                solution,
                nonzero_indices,
                nonzero_values,
            );
            let previous = multipliers[halfspace_index];
            let excess = value - limit;
            let normalized_excess = if norm_squared == 1.0 {
                excess
            } else {
                excess / norm_squared
            };
            let next = (previous + normalized_excess).max(0.0);
            let delta = next - previous;
            multipliers[halfspace_index] = next;
            if !SKIP_ZERO_FEASIBILITY_UPDATES || delta != 0.0 {
                apply_halfspace_transpose_sparse(
                    halfspace,
                    -delta,
                    solution,
                    constraints,
                    ordered_constraints,
                    nonzero_indices,
                    nonzero_values,
                    &mut negative_zeros,
                );
            }
        }
        projection_sweeps += 1;
        halfspace_projections += halfspaces.len();
        let maximum_violation = maximum_bound_violation_slice(solution, bounds).max(
            maximum_constraint_violation_for_dykstra_sparse(
                solution,
                constraints,
                ordered_constraints,
                halfspaces,
                nonzero_indices,
                nonzero_values,
            ),
        );
        if maximum_violation <= HARD_CONSTRAINT_TOLERANCE {
            return FeasibilitySeedOutcome {
                seed: FeasibilitySeed::Exact,
                projection_sweeps,
                halfspace_projections,
            };
        }
        if maximum_violation <= FEASIBILITY_POLISH_TRIGGER {
            return FeasibilitySeedOutcome {
                seed: FeasibilitySeed::Near,
                projection_sweeps,
                halfspace_projections,
            };
        }
    }
    let maximum_violation = maximum_bound_violation_slice(solution, bounds).max(
        maximum_constraint_violation_for_dykstra_sparse(
            solution,
            constraints,
            ordered_constraints,
            halfspaces,
            nonzero_indices,
            nonzero_values,
        ),
    );
    FeasibilitySeedOutcome {
        seed: FeasibilitySeed::Exhausted(maximum_violation),
        projection_sweeps,
        halfspace_projections,
    }
}

fn build_halfspaces_into(
    dof: usize,
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    halfspaces: &mut Vec<Halfspace>,
    nonzero_indices: &mut Vec<usize>,
    nonzero_values: &mut Vec<f64>,
    compile_nonzero_indices: bool,
) -> Result<(), f64> {
    halfspaces.clear();
    nonzero_indices.clear();
    nonzero_values.clear();
    for index in 0..dof {
        if bounds.lower[index].is_finite() {
            halfspaces.push(Halfspace::CoordinateLower(index, -bounds.lower[index]));
        }
        if bounds.upper[index].is_finite() {
            halfspaces.push(Halfspace::CoordinateUpper(index, bounds.upper[index]));
        }
    }
    for (ordered_index, constraint_index) in ordered_constraints.iter().copied().enumerate() {
        let constraint = &constraints[constraint_index];
        let norm_squared = constraint.coefficients.norm_squared();
        if norm_squared <= 1e-24 {
            if 0.0 < constraint.lower || 0.0 > constraint.upper {
                return Err(f64::INFINITY);
            }
            continue;
        }
        let nonzero_start = nonzero_indices.len();
        if compile_nonzero_indices {
            for (index, coefficient) in constraint.coefficients.iter().copied().enumerate() {
                if coefficient != 0.0 {
                    nonzero_indices.push(index);
                    nonzero_values.push(coefficient);
                }
            }
        }
        let nonzero_end = nonzero_indices.len();
        if constraint.lower.is_finite() {
            halfspaces.push(Halfspace::LinearLower(
                ordered_index,
                norm_squared,
                nonzero_start,
                nonzero_end,
                -constraint.lower,
            ));
        }
        if constraint.upper.is_finite() {
            halfspaces.push(Halfspace::LinearUpper(
                ordered_index,
                norm_squared,
                nonzero_start,
                nonzero_end,
                constraint.upper,
            ));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn polish_feasible_point_into(
    dof: usize,
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    equality_rows: usize,
    halfspaces: &[Halfspace],
    active: &mut [f64],
    solution: &mut [f64],
    matrix: &mut [f64],
    target: &mut [f64],
    pseudo_inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
    correction: &mut [f64],
    acceptance_tolerance: f64,
) -> FeasibilityPolishOutcome {
    active.fill(0.0);
    for (index, halfspace) in halfspaces.iter().copied().enumerate() {
        if halfspace_is_equality(halfspace, constraints, ordered_constraints) {
            continue;
        }
        let (value, limit, norm_squared) = halfspace_value_limit_norm_flat(
            halfspace,
            solution,
            bounds,
            constraints,
            ordered_constraints,
        );
        let normalized_slack = (limit - value) / norm_squared.sqrt().max(1e-300);
        if normalized_slack <= FEASIBILITY_ACTIVE_BAND {
            active[index] = 1.0;
        }
    }

    let mut iterations = 0;
    let mut pseudoinverse_calls = 0;
    let mut jacobi_sweeps = 0;
    for _ in 0..=dof.saturating_add(constraints.len()) {
        iterations += 1;
        let mut row = 0;
        for constraint_index in ordered_constraints.iter().copied() {
            let constraint = &constraints[constraint_index];
            if !constraint.lower.is_finite() || constraint.lower != constraint.upper {
                continue;
            }
            let norm = constraint.coefficients.norm().max(1e-300);
            for coordinate in 0..dof {
                matrix[row * dof + coordinate] = constraint.coefficients[coordinate] / norm;
            }
            target[row] =
                (constraint.lower - row_dot_slice(&constraint.coefficients, solution)) / norm;
            row += 1;
        }
        debug_assert_eq!(row, equality_rows);
        for (index, halfspace) in halfspaces.iter().copied().enumerate() {
            if active[index] == 0.0
                || halfspace_is_equality(halfspace, constraints, ordered_constraints)
            {
                continue;
            }
            let (value, limit, norm_squared) = halfspace_value_limit_norm_flat(
                halfspace,
                solution,
                bounds,
                constraints,
                ordered_constraints,
            );
            let norm = norm_squared.sqrt().max(1e-300);
            write_halfspace_row(
                halfspace,
                dof,
                1.0 / norm,
                constraints,
                ordered_constraints,
                &mut matrix[row * dof..(row + 1) * dof],
            );
            target[row] = (limit - value) / norm;
            row += 1;
        }
        let pseudoinverse = pseudo_inverse_flat_into_profiled(
            matrix,
            row,
            dof,
            1e-12,
            0.0,
            pseudo_inverse,
            orthogonal_columns,
            right_vectors,
            singular_values,
        );
        pseudoinverse_calls += 1;
        jacobi_sweeps += pseudoinverse.jacobi_sweeps;
        multiply_matrix_vector(pseudo_inverse, dof, row, target, correction);
        for coordinate in 0..dof {
            solution[coordinate] += correction[coordinate];
        }

        let mut best_new_active = None;
        let mut best_new_violation = HARD_CONSTRAINT_TOLERANCE;
        let maximum_violation =
            maximum_constraint_violation_flat(solution, constraints, ordered_constraints)
                .max(maximum_bound_violation_slice(solution, bounds));
        if maximum_violation <= acceptance_tolerance {
            return FeasibilityPolishOutcome {
                accepted: true,
                iterations,
                pseudoinverse_calls,
                jacobi_sweeps,
            };
        }
        for (index, halfspace) in halfspaces.iter().copied().enumerate() {
            if active[index] != 0.0
                || halfspace_is_equality(halfspace, constraints, ordered_constraints)
            {
                continue;
            }
            let (value, limit, norm_squared) = halfspace_value_limit_norm_flat(
                halfspace,
                solution,
                bounds,
                constraints,
                ordered_constraints,
            );
            let violation = (value - limit) / norm_squared.sqrt().max(1e-300);
            if violation > best_new_violation {
                best_new_active = Some(index);
                best_new_violation = violation;
            }
        }
        if let Some(index) = best_new_active {
            active[index] = 1.0;
        }
        // If every violated row is already active, repeat the same
        // pseudoinverse correction. Dense redundant contact rows can leave a
        // residual only a few ulps above the hard tolerance on the first
        // pass; treating that as primal infeasibility creates a false contact
        // transition even though the active system is consistent.
    }
    FeasibilityPolishOutcome {
        accepted: false,
        iterations,
        pseudoinverse_calls,
        jacobi_sweeps,
    }
}

/// Project `origin` onto the complete hard-feasible polytope with a primal/dual
/// active set. Every affine solve is measured from the immutable origin, so a
/// converged result is the unique Euclidean projection rather than a sequence
/// of locally projected corrections.
#[allow(clippy::too_many_arguments)]
fn project_feasible_point_active_set_into(
    dof: usize,
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    equality_rows: usize,
    halfspaces: &[Halfspace],
    active: &mut [f64],
    origin: &[f64],
    solution: &mut [f64],
    matrix: &mut [f64],
    target: &mut [f64],
    pseudo_inverse: &mut [f64],
    orthogonal_columns: &mut [f64],
    right_vectors: &mut [f64],
    singular_values: &mut [f64],
    correction: &mut [f64],
    repair_equalities_before_inequalities: bool,
    acceptance_tolerance: f64,
    configured_maximum_iterations: usize,
) -> FeasibilityPolishOutcome {
    active.fill(0.0);
    solution[..dof].copy_from_slice(&origin[..dof]);
    let maximum_iterations = configured_maximum_iterations
        .min(ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT)
        .min(4_usize.saturating_mul(dof.saturating_add(halfspaces.len()).saturating_add(1)));
    let mut iterations = 0;
    let mut pseudoinverse_calls = 0;
    let mut jacobi_sweeps = 0;
    // The bounded Dykstra prefix may leave equality rows displaced by later
    // inequality projections. Re-solve the equality block before inspecting
    // only non-equality violations; otherwise an equality-only residual makes
    // the accelerator fail after one iteration without ever doing its solve.
    let mut active_set_needs_solve = repair_equalities_before_inequalities && equality_rows > 0;
    let mut seen_active_sets = [0_u64; ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT];
    let mut seen_active_set_count = 0;

    while iterations < maximum_iterations {
        let mut active_set_hash = 0xcbf2_9ce4_8422_2325_u64;
        for (index, flag) in active.iter().copied().enumerate() {
            if flag != 0.0 {
                active_set_hash ^= index as u64;
                active_set_hash = active_set_hash.wrapping_mul(0x100_0000_01b3);
            }
        }
        if seen_active_sets[..seen_active_set_count].contains(&active_set_hash) {
            return FeasibilityPolishOutcome {
                accepted: false,
                iterations,
                pseudoinverse_calls,
                jacobi_sweeps,
            };
        }
        seen_active_sets[seen_active_set_count] = active_set_hash;
        seen_active_set_count += 1;
        iterations += 1;
        if active_set_needs_solve {
            let mut row = 0;
            for constraint_index in ordered_constraints.iter().copied() {
                let constraint = &constraints[constraint_index];
                if !constraint.lower.is_finite() || constraint.lower != constraint.upper {
                    continue;
                }
                let norm = constraint.coefficients.norm().max(1e-300);
                for coordinate in 0..dof {
                    matrix[row * dof + coordinate] = constraint.coefficients[coordinate] / norm;
                }
                target[row] =
                    (constraint.lower - row_dot_slice(&constraint.coefficients, origin)) / norm;
                row += 1;
            }
            debug_assert_eq!(row, equality_rows);
            for (index, halfspace) in halfspaces.iter().copied().enumerate() {
                if active[index] == 0.0
                    || halfspace_is_equality(halfspace, constraints, ordered_constraints)
                {
                    continue;
                }
                let (value, limit, norm_squared) = halfspace_value_limit_norm_flat(
                    halfspace,
                    origin,
                    bounds,
                    constraints,
                    ordered_constraints,
                );
                let norm = norm_squared.sqrt().max(1e-300);
                write_halfspace_row(
                    halfspace,
                    dof,
                    1.0 / norm,
                    constraints,
                    ordered_constraints,
                    &mut matrix[row * dof..(row + 1) * dof],
                );
                target[row] = (limit - value) / norm;
                row += 1;
            }
            let pseudoinverse = pseudo_inverse_flat_into_profiled(
                matrix,
                row,
                dof,
                1e-12,
                0.0,
                pseudo_inverse,
                orthogonal_columns,
                right_vectors,
                singular_values,
            );
            pseudoinverse_calls += 1;
            jacobi_sweeps += pseudoinverse.jacobi_sweeps;
            multiply_matrix_vector(pseudo_inverse, dof, row, target, correction);
            for coordinate in 0..dof {
                solution[coordinate] = origin[coordinate] + correction[coordinate];
            }

            let mut most_negative = None;
            let mut most_negative_multiplier = -1e-10;
            let mut active_row = equality_rows;
            for (index, halfspace) in halfspaces.iter().copied().enumerate() {
                if active[index] == 0.0
                    || halfspace_is_equality(halfspace, constraints, ordered_constraints)
                {
                    continue;
                }
                let mut multiplier = 0.0;
                for coordinate in 0..dof {
                    multiplier -=
                        pseudo_inverse[coordinate * row + active_row] * correction[coordinate];
                }
                if multiplier < most_negative_multiplier {
                    most_negative = Some(index);
                    most_negative_multiplier = multiplier;
                }
                active_row += 1;
            }
            debug_assert_eq!(active_row, row);
            if let Some(index) = most_negative {
                active[index] = 0.0;
                continue;
            }
        }

        let mut most_violated = None;
        let mut maximum_normalized_violation = acceptance_tolerance;
        for (index, halfspace) in halfspaces.iter().copied().enumerate() {
            if active[index] != 0.0
                || halfspace_is_equality(halfspace, constraints, ordered_constraints)
            {
                continue;
            }
            let (value, limit, norm_squared) = halfspace_value_limit_norm_flat(
                halfspace,
                solution,
                bounds,
                constraints,
                ordered_constraints,
            );
            let violation = (value - limit) / norm_squared.sqrt().max(1e-300);
            if violation > maximum_normalized_violation {
                most_violated = Some(index);
                maximum_normalized_violation = violation;
            }
        }
        if let Some(index) = most_violated {
            active[index] = 1.0;
            active_set_needs_solve = true;
            continue;
        }

        let maximum_violation = maximum_bound_violation_slice(solution, bounds).max(
            maximum_constraint_violation_flat(solution, constraints, ordered_constraints),
        );
        return FeasibilityPolishOutcome {
            accepted: maximum_violation <= acceptance_tolerance,
            iterations,
            pseudoinverse_calls,
            jacobi_sweeps,
        };
    }

    FeasibilityPolishOutcome {
        accepted: false,
        iterations,
        pseudoinverse_calls,
        jacobi_sweeps,
    }
}

fn halfspace_value_limit_norm_flat(
    halfspace: Halfspace,
    solution: &[f64],
    _bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
) -> (f64, f64, f64) {
    match halfspace {
        Halfspace::CoordinateUpper(index, limit) => (solution[index], limit, 1.0),
        Halfspace::CoordinateLower(index, limit) => (-solution[index], limit, 1.0),
        Halfspace::LinearUpper(index, norm_squared, _, _, limit) => {
            let constraint = &constraints[ordered_constraints[index]];
            let norm_squared = if CACHE_FEASIBILITY_ROW_NORMS {
                norm_squared
            } else {
                constraint.coefficients.norm_squared()
            };
            (
                row_dot_slice(&constraint.coefficients, solution),
                limit,
                norm_squared,
            )
        }
        Halfspace::LinearLower(index, norm_squared, _, _, limit) => {
            let constraint = &constraints[ordered_constraints[index]];
            let norm_squared = if CACHE_FEASIBILITY_ROW_NORMS {
                norm_squared
            } else {
                constraint.coefficients.norm_squared()
            };
            (
                -row_dot_slice(&constraint.coefficients, solution),
                limit,
                norm_squared,
            )
        }
    }
}

fn halfspace_value_limit_norm_flat_with_spans(
    halfspace: Halfspace,
    solution: &[f64],
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    use_row_spans: bool,
) -> (f64, f64, f64) {
    if !use_row_spans {
        return halfspace_value_limit_norm_flat(
            halfspace,
            solution,
            bounds,
            constraints,
            ordered_constraints,
        );
    }
    match halfspace {
        Halfspace::CoordinateUpper(index, limit) => (solution[index], limit, 1.0),
        Halfspace::CoordinateLower(index, limit) => (-solution[index], limit, 1.0),
        Halfspace::LinearUpper(_index, norm_squared, start, end, limit) => (
            row_dot_sparse_slice(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
            ),
            limit,
            norm_squared,
        ),
        Halfspace::LinearLower(_index, norm_squared, start, end, limit) => (
            -row_dot_sparse_slice(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
            ),
            limit,
            norm_squared,
        ),
    }
}

fn halfspace_value_limit_norm_sparse(
    halfspace: Halfspace,
    solution: &[f64],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
) -> (f64, f64, f64) {
    match halfspace {
        Halfspace::CoordinateUpper(index, limit) => (solution[index], limit, 1.0),
        Halfspace::CoordinateLower(index, limit) => (-solution[index], limit, 1.0),
        Halfspace::LinearUpper(_, norm_squared, start, end, limit) => (
            row_dot_sparse_slice(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
            ),
            limit,
            norm_squared,
        ),
        Halfspace::LinearLower(_, norm_squared, start, end, limit) => (
            -row_dot_sparse_slice(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
            ),
            limit,
            norm_squared,
        ),
    }
}

fn halfspace_is_equality(
    halfspace: Halfspace,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
) -> bool {
    match halfspace {
        Halfspace::CoordinateUpper(_, _) | Halfspace::CoordinateLower(_, _) => false,
        Halfspace::LinearUpper(index, _, _, _, _) | Halfspace::LinearLower(index, _, _, _, _) => {
            let constraint = &constraints[ordered_constraints[index]];
            constraint.lower.is_finite() && constraint.lower == constraint.upper
        }
    }
}

fn write_halfspace_row(
    halfspace: Halfspace,
    dof: usize,
    scale: f64,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    output: &mut [f64],
) {
    output[..dof].fill(0.0);
    match halfspace {
        Halfspace::CoordinateUpper(index, _) => output[index] = scale,
        Halfspace::CoordinateLower(index, _) => output[index] = -scale,
        Halfspace::LinearUpper(index, _, _, _, _) => {
            let row = &constraints[ordered_constraints[index]].coefficients;
            for coordinate in 0..dof {
                output[coordinate] = scale * row[coordinate];
            }
        }
        Halfspace::LinearLower(index, _, _, _, _) => {
            let row = &constraints[ordered_constraints[index]].coefficients;
            for coordinate in 0..dof {
                output[coordinate] = -scale * row[coordinate];
            }
        }
    }
}

fn apply_halfspace_transpose_flat(
    halfspace: Halfspace,
    scale: f64,
    solution: &mut [f64],
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    use_row_spans: bool,
    negative_zeros: &mut NegativeZeroTracker,
) {
    match halfspace {
        Halfspace::CoordinateUpper(index, _) => {
            if use_row_spans {
                negative_zeros.add(index, &mut solution[index], scale);
            } else {
                solution[index] += scale;
            }
        }
        Halfspace::CoordinateLower(index, _) => {
            if use_row_spans {
                negative_zeros.add(index, &mut solution[index], -scale);
            } else {
                solution[index] -= scale;
            }
        }
        Halfspace::LinearUpper(index, _, start, end, _) => {
            if use_row_spans {
                apply_sparse_row_transpose(
                    solution,
                    &nonzero_indices[start..end],
                    &nonzero_values[start..end],
                    scale,
                    negative_zeros,
                );
                if negative_zeros.any() {
                    preserve_dense_zero_update_signs(
                        &constraints[ordered_constraints[index]].coefficients,
                        scale,
                        solution,
                        negative_zeros,
                    );
                }
            } else {
                let row = &constraints[ordered_constraints[index]].coefficients;
                for coordinate in 0..solution.len() {
                    solution[coordinate] += scale * row[coordinate];
                }
            }
        }
        Halfspace::LinearLower(index, _, start, end, _) => {
            if use_row_spans {
                apply_sparse_row_transpose(
                    solution,
                    &nonzero_indices[start..end],
                    &nonzero_values[start..end],
                    -scale,
                    negative_zeros,
                );
                if negative_zeros.any() {
                    preserve_dense_zero_update_signs(
                        &constraints[ordered_constraints[index]].coefficients,
                        -scale,
                        solution,
                        negative_zeros,
                    );
                }
            } else {
                let row = &constraints[ordered_constraints[index]].coefficients;
                for coordinate in 0..solution.len() {
                    solution[coordinate] -= scale * row[coordinate];
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_halfspace_transpose_sparse(
    halfspace: Halfspace,
    scale: f64,
    solution: &mut [f64],
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    negative_zeros: &mut NegativeZeroTracker,
) {
    match halfspace {
        Halfspace::CoordinateUpper(index, _) => {
            negative_zeros.add(index, &mut solution[index], scale);
        }
        Halfspace::CoordinateLower(index, _) => {
            negative_zeros.add(index, &mut solution[index], -scale);
        }
        Halfspace::LinearUpper(index, _, start, end, _) => {
            apply_sparse_row_transpose(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
                scale,
                negative_zeros,
            );
            if negative_zeros.any() {
                preserve_dense_zero_update_signs(
                    &constraints[ordered_constraints[index]].coefficients,
                    scale,
                    solution,
                    negative_zeros,
                );
            }
        }
        Halfspace::LinearLower(index, _, start, end, _) => {
            apply_sparse_row_transpose(
                solution,
                &nonzero_indices[start..end],
                &nonzero_values[start..end],
                -scale,
                negative_zeros,
            );
            if negative_zeros.any() {
                preserve_dense_zero_update_signs(
                    &constraints[ordered_constraints[index]].coefficients,
                    -scale,
                    solution,
                    negative_zeros,
                );
            }
        }
    }
}

const NEGATIVE_ZERO_BITS: u64 = (-0.0_f64).to_bits();

#[derive(Clone, Copy, Debug, Default)]
struct NegativeZeroTracker {
    mask: u128,
    overflow: bool,
}

impl NegativeZeroTracker {
    fn new(solution: &[f64], enabled: bool) -> Self {
        let mut tracker = Self::default();
        if enabled {
            tracker.recompute(solution);
        }
        tracker
    }

    #[inline(always)]
    fn any(self) -> bool {
        self.mask != 0 || self.overflow
    }

    #[inline(always)]
    fn add(&mut self, index: usize, value: &mut f64, delta: f64) {
        if !self.any() {
            *value += delta;
            return;
        }
        *value += delta;
        if index < u128::BITS as usize {
            let bit = 1_u128 << index;
            if delta != 0.0 {
                // In round-to-nearest mode a genuinely nonzero add cannot
                // produce -0: exact cancellation produces +0. It also clears
                // any previous signed zero at this coordinate.
                self.mask &= !bit;
            } else if value.to_bits() == NEGATIVE_ZERO_BITS {
                self.mask |= bit;
            } else {
                self.mask &= !bit;
            }
        } else if value.to_bits() == NEGATIVE_ZERO_BITS {
            self.overflow = true;
        }
    }

    fn recompute(&mut self, solution: &[f64]) {
        self.mask = 0;
        self.overflow = false;
        for (index, value) in solution.iter().enumerate() {
            if value.to_bits() != NEGATIVE_ZERO_BITS {
                continue;
            }
            if index < u128::BITS as usize {
                self.mask |= 1_u128 << index;
            } else {
                self.overflow = true;
            }
        }
    }
}

/// Reproduce the only observable effect of the dense `x += scale * ±0` path.
///
/// Adding a signed zero cannot change a finite nonzero value. It can, however,
/// turn `-0` into `+0`. That bit matters to downstream exact tie/freeze checks,
/// so sparse traversal must retain it even though it skips the zero multiply.
fn preserve_dense_zero_update_signs(
    row: &RowDVector<f64>,
    scale: f64,
    solution: &mut [f64],
    negative_zeros: &mut NegativeZeroTracker,
) {
    if negative_zeros.overflow {
        for (coefficient, value) in row.iter().zip(solution.iter_mut()) {
            if *coefficient == 0.0 && value.to_bits() == NEGATIVE_ZERO_BITS {
                *value += scale * *coefficient;
            }
        }
        negative_zeros.recompute(solution);
        return;
    }

    let mut mask = negative_zeros.mask;
    while mask != 0 {
        let coordinate = mask.trailing_zeros() as usize;
        mask &= mask - 1;
        let coefficient = row[coordinate];
        if coefficient == 0.0 {
            negative_zeros.add(coordinate, &mut solution[coordinate], scale * coefficient);
        }
    }
}

fn row_dot_slice(row: &RowDVector<f64>, vector: &[f64]) -> f64 {
    row.iter()
        .zip(vector.iter())
        .map(|(coefficient, value)| coefficient * value)
        .sum()
}

fn row_dot_sparse_slice(vector: &[f64], nonzero_indices: &[usize], nonzero_values: &[f64]) -> f64 {
    debug_assert_eq!(nonzero_indices.len(), nonzero_values.len());
    debug_assert!(nonzero_indices.iter().all(|index| *index < vector.len()));
    let mut sum = 0.0;
    for offset in 0..nonzero_indices.len() {
        // SAFETY: both sparse arrays are compiled together from a validated
        // dense row, and each retained coordinate is strictly below the DOF.
        unsafe {
            let index = *nonzero_indices.get_unchecked(offset);
            sum += nonzero_values.get_unchecked(offset) * vector.get_unchecked(index);
        }
    }
    sum
}

fn apply_sparse_row_transpose(
    solution: &mut [f64],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    scale: f64,
    negative_zeros: &mut NegativeZeroTracker,
) {
    debug_assert_eq!(nonzero_indices.len(), nonzero_values.len());
    debug_assert!(nonzero_indices.iter().all(|index| *index < solution.len()));
    if !negative_zeros.any() {
        for offset in 0..nonzero_indices.len() {
            // SAFETY: the sparse arrays are compiled together from a
            // validated dense row and retain only in-range coordinates.
            unsafe {
                let index = *nonzero_indices.get_unchecked(offset);
                *solution.get_unchecked_mut(index) += scale * nonzero_values.get_unchecked(offset);
            }
        }
        return;
    }
    for offset in 0..nonzero_indices.len() {
        // SAFETY: the sparse metadata construction invariants are restated by
        // the debug assertions above. The vectors are immutable during solve.
        unsafe {
            let index = *nonzero_indices.get_unchecked(offset);
            let value = solution.get_unchecked_mut(index);
            negative_zeros.add(index, value, scale * nonzero_values.get_unchecked(offset));
        }
    }
}

fn maximum_bound_violation_slice(solution: &[f64], bounds: &VelocityBounds) -> f64 {
    (0..solution.len()).fold(0.0_f64, |maximum, index| {
        maximum
            .max((bounds.lower[index] - solution[index]).max(0.0))
            .max((solution[index] - bounds.upper[index]).max(0.0))
    })
}

fn record_hard_violation_witness(
    diagnostics: &mut SolveDiagnostics,
    solution: &[f64],
    bounds: &VelocityBounds,
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
) {
    let mut maximum_bound_violation = 0.0_f64;
    let mut limiting_bound_coordinate = None;
    let mut limiting_bound_is_upper = false;
    for index in 0..solution.len() {
        let lower_violation = (bounds.lower[index] - solution[index]).max(0.0);
        if lower_violation > maximum_bound_violation {
            maximum_bound_violation = lower_violation;
            limiting_bound_coordinate = Some(index);
            limiting_bound_is_upper = false;
        }
        let upper_violation = (solution[index] - bounds.upper[index]).max(0.0);
        if upper_violation > maximum_bound_violation {
            maximum_bound_violation = upper_violation;
            limiting_bound_coordinate = Some(index);
            limiting_bound_is_upper = true;
        }
    }

    let mut maximum_linear_constraint_violation = 0.0_f64;
    let mut limiting_linear_constraint = None;
    let mut limiting_linear_constraint_is_upper = false;
    for constraint_index in ordered_constraints.iter().copied() {
        let constraint = &constraints[constraint_index];
        let value = row_dot_slice(&constraint.coefficients, solution);
        let lower_violation = (constraint.lower - value).max(0.0);
        if lower_violation > maximum_linear_constraint_violation {
            maximum_linear_constraint_violation = lower_violation;
            limiting_linear_constraint = Some(constraint.stable_id);
            limiting_linear_constraint_is_upper = false;
        }
        let upper_violation = (value - constraint.upper).max(0.0);
        if upper_violation > maximum_linear_constraint_violation {
            maximum_linear_constraint_violation = upper_violation;
            limiting_linear_constraint = Some(constraint.stable_id);
            limiting_linear_constraint_is_upper = true;
        }
    }

    diagnostics.maximum_bound_violation = maximum_bound_violation;
    diagnostics.limiting_bound_coordinate = limiting_bound_coordinate;
    diagnostics.limiting_bound_is_upper = limiting_bound_is_upper;
    diagnostics.maximum_linear_constraint_violation = maximum_linear_constraint_violation;
    diagnostics.limiting_linear_constraint = limiting_linear_constraint;
    diagnostics.limiting_linear_constraint_is_upper = limiting_linear_constraint_is_upper;
    diagnostics.maximum_constraint_violation =
        maximum_bound_violation.max(maximum_linear_constraint_violation);
}

fn maximum_constraint_violation_flat(
    solution: &[f64],
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
) -> f64 {
    ordered_constraints
        .iter()
        .copied()
        .fold(0.0_f64, |maximum, index| {
            let constraint = &constraints[index];
            let value = row_dot_slice(&constraint.coefficients, solution);
            maximum
                .max((constraint.lower - value).max(0.0))
                .max((value - constraint.upper).max(0.0))
        })
}

fn maximum_constraint_violation_for_dykstra(
    solution: &[f64],
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    halfspaces: &[Halfspace],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
    use_row_spans: bool,
) -> f64 {
    if !use_row_spans {
        return maximum_constraint_violation_flat(solution, constraints, ordered_constraints);
    }
    let mut maximum = 0.0_f64;
    let mut previous_ordered_index = usize::MAX;
    for halfspace in halfspaces.iter().copied() {
        let (ordered_index, start, end) = match halfspace {
            Halfspace::CoordinateUpper(_, _) | Halfspace::CoordinateLower(_, _) => continue,
            Halfspace::LinearUpper(index, _, start, end, _)
            | Halfspace::LinearLower(index, _, start, end, _) => (index, start, end),
        };
        if ordered_index == previous_ordered_index {
            continue;
        }
        previous_ordered_index = ordered_index;
        let constraint = &constraints[ordered_constraints[ordered_index]];
        let value = row_dot_sparse_slice(
            solution,
            &nonzero_indices[start..end],
            &nonzero_values[start..end],
        );
        maximum = maximum
            .max((constraint.lower - value).max(0.0))
            .max((value - constraint.upper).max(0.0));
    }
    maximum
}

fn maximum_constraint_violation_for_dykstra_sparse(
    solution: &[f64],
    constraints: &[LinearConstraint],
    ordered_constraints: &[usize],
    halfspaces: &[Halfspace],
    nonzero_indices: &[usize],
    nonzero_values: &[f64],
) -> f64 {
    let mut maximum = 0.0_f64;
    let mut previous_ordered_index = usize::MAX;
    for halfspace in halfspaces.iter().copied() {
        let (ordered_index, start, end) = match halfspace {
            Halfspace::CoordinateUpper(_, _) | Halfspace::CoordinateLower(_, _) => continue,
            Halfspace::LinearUpper(index, _, start, end, _)
            | Halfspace::LinearLower(index, _, start, end, _) => (index, start, end),
        };
        if ordered_index == previous_ordered_index {
            continue;
        }
        previous_ordered_index = ordered_index;
        let constraint = &constraints[ordered_constraints[ordered_index]];
        let value = row_dot_sparse_slice(
            solution,
            &nonzero_indices[start..end],
            &nonzero_values[start..end],
        );
        maximum = maximum
            .max((constraint.lower - value).max(0.0))
            .max((value - constraint.upper).max(0.0));
    }
    maximum
}

#[cfg(test)]
fn pseudo_inverse(matrix: &DMatrix<f64>, tolerance: f64) -> (DMatrix<f64>, usize) {
    if matrix.nrows() == 0 || matrix.ncols() == 0 {
        return (DMatrix::zeros(matrix.ncols(), matrix.nrows()), 0);
    }
    // Work on the orientation with fewer columns because one-sided Jacobi
    // rotates column pairs. This is a large win for the usual WBC shape
    // (few task rows, many generalized coordinates).
    if matrix.ncols() > matrix.nrows() {
        let transpose = matrix.transpose();
        let (transpose_inverse, rank) = pseudo_inverse_tall(&transpose, tolerance);
        return (transpose_inverse.transpose(), rank);
    }
    pseudo_inverse_tall(matrix, tolerance)
}

#[cfg(test)]
fn pseudo_inverse_tall(matrix: &DMatrix<f64>, tolerance: f64) -> (DMatrix<f64>, usize) {
    // One-sided Jacobi SVD orthogonalizes A's columns directly, avoiding the
    // condition-number squaring of an AᵀA eigensolve. If B = A V = U Σ, then
    // Aᐩ = V diag(1/σ²) Bᵀ.
    let mut orthogonal_columns = matrix.clone();
    let mut right_vectors = DMatrix::identity(matrix.ncols(), matrix.ncols());
    one_sided_jacobi(&mut orthogonal_columns, &mut right_vectors);
    let mut singular_values = DVector::zeros(matrix.ncols());
    for column in 0..matrix.ncols() {
        singular_values[column] = orthogonal_columns.column(column).norm();
    }
    let maximum_singular_value = singular_values.iter().copied().fold(0.0_f64, f64::max);
    let singular_threshold = tolerance * maximum_singular_value.max(1.0);
    let mut inverse = DMatrix::<f64>::zeros(matrix.ncols(), matrix.nrows());
    let mut rank = 0;
    for singular_vector in 0..matrix.ncols() {
        let singular_value = singular_values[singular_vector];
        if singular_value <= singular_threshold {
            continue;
        }
        rank += 1;
        let inverse_squared = 1.0 / (singular_value * singular_value);
        for output_row in 0..matrix.ncols() {
            for output_column in 0..matrix.nrows() {
                inverse[(output_row, output_column)] += inverse_squared
                    * right_vectors[(output_row, singular_vector)]
                    * orthogonal_columns[(output_column, singular_vector)];
            }
        }
    }
    (inverse, rank)
}

#[cfg(test)]
fn one_sided_jacobi(matrix: &mut DMatrix<f64>, right_vectors: &mut DMatrix<f64>) {
    let columns = matrix.ncols();
    if columns < 2 {
        return;
    }
    for _ in 0..256 {
        let mut rotated = false;
        for p in 0..columns - 1 {
            for q in p + 1..columns {
                let mut alpha = 0.0;
                let mut beta = 0.0;
                let mut coupling = 0.0;
                for row in 0..matrix.nrows() {
                    let p_value = matrix[(row, p)];
                    let q_value = matrix[(row, q)];
                    alpha += p_value * p_value;
                    beta += q_value * q_value;
                    coupling += p_value * q_value;
                }
                let scale = (alpha * beta).max(0.0).sqrt();
                if scale == 0.0
                    || coupling.abs() <= 64.0 * f64::EPSILON * matrix.nrows().max(1) as f64 * scale
                {
                    continue;
                }
                rotated = true;
                let tau = (beta - alpha) / (2.0 * coupling);
                let tangent = if tau >= 0.0 {
                    1.0 / (tau + (1.0 + tau * tau).sqrt())
                } else {
                    -1.0 / (-tau + (1.0 + tau * tau).sqrt())
                };
                let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
                let sine = tangent * cosine;
                for row in 0..matrix.nrows() {
                    let p_value = matrix[(row, p)];
                    let q_value = matrix[(row, q)];
                    matrix[(row, p)] = cosine * p_value - sine * q_value;
                    matrix[(row, q)] = sine * p_value + cosine * q_value;
                }
                for row in 0..columns {
                    let p_value = right_vectors[(row, p)];
                    let q_value = right_vectors[(row, q)];
                    right_vectors[(row, p)] = cosine * p_value - sine * q_value;
                    right_vectors[(row, q)] = sine * p_value + cosine * q_value;
                }
            }
        }
        if !rotated {
            break;
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum ConstraintHit {
    Coordinate(usize),
    Linear(usize),
}

#[derive(Clone, Copy, Debug)]
enum Halfspace {
    CoordinateUpper(usize, f64),
    CoordinateLower(usize, f64),
    LinearUpper(usize, f64, usize, usize, f64),
    LinearLower(usize, f64, usize, usize, f64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum FeasibilitySeed {
    Exact,
    Near,
    StructurallyInfeasible(f64),
    Exhausted(f64),
}

#[derive(Clone, Copy, Debug)]
struct FeasibilitySeedOutcome {
    seed: FeasibilitySeed,
    projection_sweeps: usize,
    halfspace_projections: usize,
}

#[derive(Clone, Copy, Debug)]
struct FeasibilityPolishOutcome {
    accepted: bool,
    iterations: usize,
    pseudoinverse_calls: usize,
    jacobi_sweeps: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dykstra_sparse_indices_are_bit_exact_for_linear_rows() {
        let dof = 8;
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![
                f64::NEG_INFINITY,
                -0.25,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                -0.5,
                f64::NEG_INFINITY,
            ]),
            upper: DVector::from_vec(vec![
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                0.75,
            ]),
        };
        let constraints = [
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
                lower: 1.25,
                upper: 1.25,
            },
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[
                    0.0, -0.5, 0.0, 1.5, 0.0, -0.25, 0.0, 0.0,
                ]),
                lower: -0.2,
                upper: 0.4,
            },
        ];
        let ordered_constraints = [1, 0];
        let mut halfspaces = Vec::new();
        let mut nonzero_indices = Vec::new();
        let mut nonzero_values = Vec::new();
        build_halfspaces_into(
            dof,
            &bounds,
            &constraints,
            &ordered_constraints,
            &mut halfspaces,
            &mut nonzero_indices,
            &mut nonzero_values,
            true,
        )
        .unwrap();
        assert_eq!(nonzero_indices, [1, 3, 5, 2, 4]);
        assert_eq!(nonzero_values, [-0.5, 1.5, -0.25, 1.0, 2.0]);
        assert!(
            halfspaces
                .iter()
                .any(|halfspace| matches!(halfspace, Halfspace::LinearUpper(_, _, 0, 3, _)))
        );
        assert!(
            halfspaces
                .iter()
                .any(|halfspace| matches!(halfspace, Halfspace::LinearLower(_, _, 3, 5, _)))
        );

        let origin = [0.2, -0.75, -0.4, 0.5, 0.1, 0.25, -0.75, 1.0];
        let mut reference_solution = origin;
        let mut candidate_solution = origin;
        let mut reference_multipliers = vec![0.0; halfspaces.len()];
        let mut candidate_multipliers = vec![0.0; halfspaces.len()];
        let reference = continue_feasible_point_into(
            &bounds,
            &constraints,
            &ordered_constraints,
            &mut reference_solution,
            &halfspaces,
            &nonzero_indices,
            &nonzero_values,
            &mut reference_multipliers,
            64,
            false,
        );
        let candidate = continue_feasible_point_into(
            &bounds,
            &constraints,
            &ordered_constraints,
            &mut candidate_solution,
            &halfspaces,
            &nonzero_indices,
            &nonzero_values,
            &mut candidate_multipliers,
            64,
            true,
        );

        assert_eq!(reference.seed, candidate.seed);
        assert_eq!(reference.projection_sweeps, candidate.projection_sweeps);
        assert_eq!(
            reference.halfspace_projections,
            candidate.halfspace_projections
        );
        assert_eq!(
            reference_solution.map(f64::to_bits),
            candidate_solution.map(f64::to_bits)
        );
        assert_eq!(
            reference_multipliers
                .iter()
                .copied()
                .map(f64::to_bits)
                .collect::<Vec<_>>(),
            candidate_multipliers
                .iter()
                .copied()
                .map(f64::to_bits)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn sparse_transpose_preserves_dense_signed_zero_updates() {
        let row = RowDVector::from_row_slice(&[0.0, -0.0, 1.0]);
        let mut positive_scale = [-0.0, -0.0, 2.0];
        let mut positive_tracker = NegativeZeroTracker::new(&positive_scale, true);
        preserve_dense_zero_update_signs(&row, 3.0, &mut positive_scale, &mut positive_tracker);
        assert_eq!(positive_tracker.mask, 1 << 1);
        assert_eq!(positive_scale[0].to_bits(), 0.0_f64.to_bits());
        assert_eq!(positive_scale[1].to_bits(), (-0.0_f64).to_bits());
        assert_eq!(positive_scale[2].to_bits(), 2.0_f64.to_bits());

        let mut negative_scale = [-0.0, -0.0, 2.0];
        let mut negative_tracker = NegativeZeroTracker::new(&negative_scale, true);
        preserve_dense_zero_update_signs(&row, -3.0, &mut negative_scale, &mut negative_tracker);
        assert_eq!(negative_tracker.mask, 1 << 0);
        assert_eq!(negative_scale[0].to_bits(), (-0.0_f64).to_bits());
        assert_eq!(negative_scale[1].to_bits(), 0.0_f64.to_bits());
        assert_eq!(negative_scale[2].to_bits(), 2.0_f64.to_bits());

        let mut cancellation = [-1.0];
        let mut cancellation_tracker = NegativeZeroTracker::new(&cancellation, true);
        cancellation_tracker.add(0, &mut cancellation[0], 1.0);
        assert_eq!(cancellation[0].to_bits(), 0.0_f64.to_bits());
        assert!(!cancellation_tracker.any());
    }

    #[test]
    fn active_set_projection_matches_unique_euclidean_boundary_solution() {
        let dof = 2;
        let bounds = VelocityBounds {
            lower: DVector::from_element(dof, f64::NEG_INFINITY),
            upper: DVector::from_vec(vec![0.75, f64::INFINITY]),
        };
        let constraints = [LinearConstraint {
            stable_id: 1,
            coefficients: RowDVector::from_row_slice(&[1.0, 1.0]),
            lower: 2.0,
            upper: 2.0,
        }];
        let ordered_constraints = [0];
        let halfspaces = [
            Halfspace::CoordinateUpper(0, 0.75),
            Halfspace::LinearLower(0, 2.0, 0, 2, -2.0),
            Halfspace::LinearUpper(0, 2.0, 0, 2, 2.0),
        ];
        let origin = [1.0, 1.0];
        let mut solution = origin;
        let mut active = vec![0.0; halfspaces.len()];
        let mut workspace = SolverWorkspace::new(dof, 8, 0, constraints.len());
        let outcome = project_feasible_point_active_set_into(
            dof,
            &bounds,
            &constraints,
            &ordered_constraints,
            1,
            &halfspaces,
            &mut active,
            &origin,
            &mut solution,
            &mut workspace.level_matrix,
            &mut workspace.target,
            &mut workspace.pseudo_inverse,
            &mut workspace.orthogonal_columns,
            &mut workspace.right_vectors,
            &mut workspace.singular_values,
            &mut workspace.correction,
            true,
            HARD_CONSTRAINT_TOLERANCE,
            ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT,
        );
        assert!(outcome.accepted);
        assert_eq!(outcome.pseudoinverse_calls, 2);
        assert!((solution[0] - 0.75).abs() < 1e-12);
        assert!((solution[1] - 1.25).abs() < 1e-12);
        assert!((solution[0] + solution[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn equality_first_repair_handles_a_displaced_dykstra_origin() {
        let dof = 2;
        let bounds = VelocityBounds {
            lower: DVector::from_element(dof, f64::NEG_INFINITY),
            upper: DVector::from_vec(vec![0.75, f64::INFINITY]),
        };
        let constraints = [LinearConstraint {
            stable_id: 1,
            coefficients: RowDVector::from_row_slice(&[1.0, 1.0]),
            lower: 2.0,
            upper: 2.0,
        }];
        let ordered_constraints = [0];
        let halfspaces = [
            Halfspace::CoordinateUpper(0, 0.75),
            Halfspace::LinearLower(0, 2.0, 0, 2, -2.0),
            Halfspace::LinearUpper(0, 2.0, 0, 2, 2.0),
        ];
        let origin = [0.75, 1.0];
        let mut established_solution = origin;
        let mut established_active = vec![0.0; halfspaces.len()];
        let mut established_workspace = SolverWorkspace::new(dof, 8, 0, constraints.len());
        let established = project_feasible_point_active_set_into(
            dof,
            &bounds,
            &constraints,
            &ordered_constraints,
            1,
            &halfspaces,
            &mut established_active,
            &origin,
            &mut established_solution,
            &mut established_workspace.level_matrix,
            &mut established_workspace.target,
            &mut established_workspace.pseudo_inverse,
            &mut established_workspace.orthogonal_columns,
            &mut established_workspace.right_vectors,
            &mut established_workspace.singular_values,
            &mut established_workspace.correction,
            false,
            HARD_CONSTRAINT_TOLERANCE,
            ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT,
        );
        assert!(!established.accepted);
        assert_eq!(established.iterations, 1);
        assert_eq!(established.pseudoinverse_calls, 0);

        let mut repaired_solution = origin;
        let mut repaired_active = vec![0.0; halfspaces.len()];
        let mut repaired_workspace = SolverWorkspace::new(dof, 8, 0, constraints.len());
        let repaired = project_feasible_point_active_set_into(
            dof,
            &bounds,
            &constraints,
            &ordered_constraints,
            1,
            &halfspaces,
            &mut repaired_active,
            &origin,
            &mut repaired_solution,
            &mut repaired_workspace.level_matrix,
            &mut repaired_workspace.target,
            &mut repaired_workspace.pseudo_inverse,
            &mut repaired_workspace.orthogonal_columns,
            &mut repaired_workspace.right_vectors,
            &mut repaired_workspace.singular_values,
            &mut repaired_workspace.correction,
            true,
            HARD_CONSTRAINT_TOLERANCE,
            ACTIVE_SET_FEASIBILITY_ITERATION_LIMIT,
        );
        assert!(repaired.accepted);
        assert!((repaired_solution[0] - 0.75).abs() < 1e-12);
        assert!((repaired_solution[1] - 1.25).abs() < 1e-12);
    }

    #[test]
    fn active_set_projection_rejects_an_inconsistent_polytope() {
        let dof = 1;
        // The public solver rejects these malformed bounds before projection;
        // this direct kernel test uses explicit contradictory halfspaces to
        // exercise active-set non-convergence instead.
        let valid_bounds = VelocityBounds::unbounded(dof);
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let ordered_constraints = [0, 1];
        let halfspaces = [
            Halfspace::LinearLower(0, 1.0, 0, 1, -1.0),
            Halfspace::LinearUpper(1, 1.0, 0, 1, 0.0),
        ];
        let origin = [0.0];
        let mut solution = origin;
        let mut active = vec![0.0; halfspaces.len()];
        let mut workspace = SolverWorkspace::new(dof, 4, 0, constraints.len());
        let outcome = project_feasible_point_active_set_into(
            dof,
            &valid_bounds,
            &constraints,
            &ordered_constraints,
            0,
            &halfspaces,
            &mut active,
            &origin,
            &mut solution,
            &mut workspace.level_matrix,
            &mut workspace.target,
            &mut workspace.pseudo_inverse,
            &mut workspace.orthogonal_columns,
            &mut workspace.right_vectors,
            &mut workspace.singular_values,
            &mut workspace.correction,
            true,
            HARD_CONSTRAINT_TOLERANCE,
            1,
        );
        assert!(!outcome.accepted);
        assert_eq!(outcome.iterations, 1);
    }

    #[test]
    fn near_feasible_seed_is_polished_on_equality_and_active_bound() {
        let dof = 2;
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![0.0, 0.0]),
            upper: DVector::from_element(dof, f64::INFINITY),
        };
        let constraints = [LinearConstraint {
            stable_id: 1,
            coefficients: RowDVector::from_row_slice(&[1.0, 1.0]),
            lower: 1.0,
            upper: 1.0,
        }];
        let ordered_constraints = [0];
        let halfspaces = [
            Halfspace::CoordinateLower(0, -0.0),
            Halfspace::CoordinateLower(1, -0.0),
            Halfspace::LinearLower(0, 2.0, 0, 2, -1.0),
            Halfspace::LinearUpper(0, 2.0, 0, 2, 1.0),
        ];
        let mut active = vec![0.0; halfspaces.len()];
        let mut solution = vec![-5e-4, 1.0005];
        let mut workspace = SolverWorkspace::new(dof, 8, 0, constraints.len());
        let polished = polish_feasible_point_into(
            dof,
            &bounds,
            &constraints,
            &ordered_constraints,
            1,
            &halfspaces,
            &mut active,
            &mut solution,
            &mut workspace.level_matrix,
            &mut workspace.target,
            &mut workspace.pseudo_inverse,
            &mut workspace.orthogonal_columns,
            &mut workspace.right_vectors,
            &mut workspace.singular_values,
            &mut workspace.correction,
            HARD_CONSTRAINT_TOLERANCE,
        );
        assert!(polished.accepted);
        assert!(polished.iterations >= 1);
        assert_eq!(polished.iterations, polished.pseudoinverse_calls);
        assert!(solution[0].abs() < 1e-10);
        assert!((solution[1] - 1.0).abs() < 1e-10);
        assert!(maximum_bound_violation_slice(&solution, &bounds) < 1e-10);
        assert!(
            maximum_constraint_violation_flat(&solution, &constraints, &ordered_constraints)
                < 1e-10
        );
    }

    #[test]
    fn jacobi_pseudoinverse_satisfies_penrose_identities() {
        for (rows, columns) in [(1, 3), (3, 1), (3, 4), (6, 4), (4, 6)] {
            let mut matrix = DMatrix::from_fn(rows, columns, |row, column| {
                ((row * 17 + column * 11 + rows * 3) as f64 * 0.37).sin()
            });
            if rows > 2 {
                // Exercise exact rank deficiency.
                let first_row = matrix.row(0).clone_owned();
                matrix.row_mut(rows - 1).copy_from(&first_row);
            }
            let (inverse, _) = pseudo_inverse(&matrix, 1e-9);
            let first_error = (&matrix * &inverse * &matrix - &matrix).norm();
            let second_error = (&inverse * &matrix * &inverse - &inverse).norm();
            assert!(
                first_error < 1e-7,
                "A A+ A failed for {rows}x{columns}: {first_error:e}"
            );
            assert!(
                second_error < 1e-7,
                "A+ A A+ failed for {rows}x{columns}: {second_error:e}"
            );
        }
    }

    #[test]
    fn task_damping_bounds_a_near_singular_inverse_without_changing_rank() {
        let matrix = [1.0, 0.0, 0.0, 1e-10];
        let mut exact = [0.0; 4];
        let mut damped = [0.0; 4];
        let mut orthogonal_columns = [0.0; 4];
        let mut right_vectors = [0.0; 4];
        let mut singular_values = [0.0; 2];
        let exact_outcome = pseudo_inverse_flat_into_profiled(
            &matrix,
            2,
            2,
            1e-12,
            0.0,
            &mut exact,
            &mut orthogonal_columns,
            &mut right_vectors,
            &mut singular_values,
        );
        let damped_outcome = pseudo_inverse_flat_into_profiled(
            &matrix,
            2,
            2,
            1e-12,
            1e-8,
            &mut damped,
            &mut orthogonal_columns,
            &mut right_vectors,
            &mut singular_values,
        );

        assert_eq!(exact_outcome.rank, 2);
        assert_eq!(damped_outcome.rank, 2);
        assert!(exact[3] > 9.9e9);
        assert!(damped[3] > 9.9e5 && damped[3] < 1.01e6);
        assert!(damped[3] < exact[3] * 1e-3);
        assert!((damped[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn guarded_row_gram_matches_jacobi_for_well_conditioned_wide_tasks() {
        let rows = 3;
        let columns = 8;
        let matrix = [
            1.0, 0.2, -0.1, 0.4, 0.0, 0.3, -0.2, 0.1, -0.3, 1.1, 0.2, -0.2, 0.5, 0.0, 0.1, -0.4,
            0.2, -0.1, 0.9, 0.3, -0.2, 0.4, 0.6, 0.0,
        ];
        for damping in [0.0, 1e-8, 1e-3] {
            let mut expected = vec![0.0; rows * columns];
            let mut expected_orthogonal = vec![0.0; rows * columns];
            let mut expected_right = vec![0.0; rows * rows];
            let mut expected_singular = vec![0.0; rows];
            let expected_outcome = pseudo_inverse_flat_into_profiled(
                &matrix,
                rows,
                columns,
                1e-12,
                damping,
                &mut expected,
                &mut expected_orthogonal,
                &mut expected_right,
                &mut expected_singular,
            );

            let mut actual = vec![f64::NAN; rows * columns];
            let mut gram = vec![0.0; rows * columns];
            let mut solve = vec![0.0; rows];
            let actual_outcome = pseudo_inverse_wide_row_gram_into(
                &matrix,
                rows,
                columns,
                1e-12,
                damping,
                &mut actual,
                &mut gram,
                &mut solve,
            )
            .expect("well-conditioned fixture should take the row-Gram path");

            assert_eq!(actual_outcome.rank, expected_outcome.rank);
            assert_eq!(actual_outcome.jacobi_sweeps, 0);
            let absolute_error = actual
                .iter()
                .zip(&expected)
                .map(|(left, right)| (left - right).powi(2))
                .sum::<f64>()
                .sqrt();
            let expected_norm = expected
                .iter()
                .map(|value| value * value)
                .sum::<f64>()
                .sqrt();
            assert!(
                absolute_error / expected_norm.max(1.0) < 1e-12,
                "row-Gram/Jacobi relative error {:#e} at damping {damping:#e}",
                absolute_error / expected_norm.max(1.0)
            );
        }
    }

    #[test]
    fn guarded_row_gram_rejects_rank_deficiency_without_touching_output() {
        let rows = 2;
        let columns = 4;
        let matrix = [1.0, -2.0, 3.0, 0.5, 2.0, -4.0, 6.0, 1.0];
        let mut inverse = vec![7.0; rows * columns];
        let mut gram = vec![0.0; rows * columns];
        let mut solve = vec![0.0; rows];

        assert!(
            pseudo_inverse_wide_row_gram_into(
                &matrix,
                rows,
                columns,
                1e-12,
                1e-8,
                &mut inverse,
                &mut gram,
                &mut solve,
            )
            .is_none()
        );
        assert_eq!(inverse, vec![7.0; rows * columns]);
    }

    #[test]
    fn single_row_specialization_is_bit_exact_to_established_kernel() {
        let mut random_state = 0xf093_271d_b424_6e19_u64;
        for columns in 1..=96 {
            for damping in [0.0, 1e-8, 1e-3] {
                let matrix = (0..columns)
                    .map(|_| {
                        random_state ^= random_state << 13;
                        random_state ^= random_state >> 7;
                        random_state ^= random_state << 17;
                        random_state as i64 as f64 / i64::MAX as f64
                    })
                    .collect::<Vec<_>>();
                let mut expected = vec![0.0; columns];
                let mut orthogonal = vec![0.0; columns];
                let mut right = [0.0; 1];
                let mut expected_singular = [0.0; 1];
                let expected_outcome = pseudo_inverse_flat_into_profiled(
                    &matrix,
                    1,
                    columns,
                    1e-12,
                    damping,
                    &mut expected,
                    &mut orthogonal,
                    &mut right,
                    &mut expected_singular,
                );
                let mut actual = vec![f64::NAN; columns];
                let mut actual_singular = [f64::NAN; 1];
                let actual_outcome = pseudo_inverse_single_row_into(
                    &matrix,
                    columns,
                    1e-12,
                    damping,
                    &mut actual,
                    &mut actual_singular,
                );

                assert_eq!(actual_outcome.rank, expected_outcome.rank);
                assert_eq!(actual_outcome.jacobi_sweeps, expected_outcome.jacobi_sweeps);
                assert_eq!(actual_singular[0].to_bits(), expected_singular[0].to_bits());
                assert!(
                    actual
                        .iter()
                        .zip(&expected)
                        .all(|(left, right)| left.to_bits() == right.to_bits())
                );
            }
        }

        let zero = [0.0; 8];
        let mut expected = [1.0; 8];
        let mut actual = [2.0; 8];
        let mut orthogonal = [0.0; 8];
        let mut right = [0.0; 1];
        let mut expected_singular = [0.0; 1];
        let mut actual_singular = [0.0; 1];
        let expected_outcome = pseudo_inverse_flat_into_profiled(
            &zero,
            1,
            8,
            1e-12,
            1e-8,
            &mut expected,
            &mut orthogonal,
            &mut right,
            &mut expected_singular,
        );
        let actual_outcome = pseudo_inverse_single_row_into(
            &zero,
            8,
            1e-12,
            1e-8,
            &mut actual,
            &mut actual_singular,
        );
        assert_eq!(actual_outcome.rank, expected_outcome.rank);
        assert_eq!(actual, expected);
    }

    #[test]
    fn jacobi_pseudoinverse_matches_nalgebra_svd_oracle() {
        for (rows, columns) in [(1, 3), (3, 1), (3, 4), (6, 4), (4, 6), (8, 8)] {
            let mut matrix = DMatrix::from_fn(rows, columns, |row, column| {
                ((row * 29 + column * 13 + rows * 7) as f64 * 0.23).cos()
            });
            if rows > 2 {
                let first_row = matrix.row(0).clone_owned();
                matrix.row_mut(rows - 1).copy_from(&first_row);
            }
            let (actual, actual_rank) = pseudo_inverse(&matrix, 1e-9);
            let svd = matrix.clone().svd(true, true);
            let maximum = svd.singular_values.iter().copied().fold(0.0_f64, f64::max);
            let threshold = 1e-9 * maximum.max(1.0);
            let expected_rank = svd
                .singular_values
                .iter()
                .filter(|value| **value > threshold)
                .count();
            let expected = svd.pseudo_inverse(threshold).unwrap();
            let difference = (&actual - &expected).norm();
            assert_eq!(actual_rank, expected_rank, "rank for {rows}x{columns}");
            assert!(
                difference < 1e-6,
                "pseudoinverse differs from SVD for {rows}x{columns}: {:e}",
                difference
            );
        }

        let near_threshold = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1e-8, 1e-10]));
        let (actual, actual_rank) = pseudo_inverse(&near_threshold, 1e-9);
        let expected = near_threshold
            .clone()
            .svd(true, true)
            .pseudo_inverse(1e-9)
            .unwrap();
        assert_eq!(actual_rank, 2);
        assert!((&actual - expected).norm() < 1e-6);

        let mut random_state = 0x8c3c_010c_b475_4c9d_u64;
        for rows in 1..=10 {
            for columns in 1..=12 {
                for sample in 0..8 {
                    let mut matrix = DMatrix::from_fn(rows, columns, |_, _| {
                        random_state ^= random_state << 13;
                        random_state ^= random_state >> 7;
                        random_state ^= random_state << 17;
                        (random_state as i64 as f64) / (i64::MAX as f64)
                    });
                    if rows > 1 && sample % 2 == 0 {
                        let first_row = matrix.row(0).clone_owned();
                        matrix.row_mut(rows - 1).copy_from(&first_row);
                    }
                    let (actual, actual_rank) = pseudo_inverse(&matrix, 1e-9);
                    let svd = matrix.clone().svd(true, true);
                    let maximum = svd.singular_values.iter().copied().fold(0.0_f64, f64::max);
                    let threshold = 1e-9 * maximum.max(1.0);
                    let expected_rank = svd
                        .singular_values
                        .iter()
                        .filter(|value| **value > threshold)
                        .count();
                    let expected = svd.pseudo_inverse(threshold).unwrap();
                    let difference = (&actual - &expected).norm();
                    let relative_difference = difference / expected.norm().max(1.0);
                    assert_eq!(
                        actual_rank, expected_rank,
                        "fuzz rank for {rows}x{columns}, sample {sample}"
                    );
                    if sample % 2 == 0 && rows > 1 {
                        let left_projector = &matrix * &actual;
                        let right_projector = &actual * &matrix;
                        assert!(
                            (&left_projector * &matrix - &matrix).norm() < 1e-10,
                            "rank-deficient A A+ A for {rows}x{columns}, sample {sample}"
                        );
                        assert!(
                            (&right_projector * &actual - &actual).norm() < 1e-10,
                            "rank-deficient A+ A A+ for {rows}x{columns}, sample {sample}"
                        );
                        assert!(
                            (&left_projector - left_projector.transpose()).norm() < 1e-10,
                            "rank-deficient A A+ symmetry for {rows}x{columns}, sample {sample}"
                        );
                        assert!(
                            (&right_projector - right_projector.transpose()).norm() < 1e-10,
                            "rank-deficient A+ A symmetry for {rows}x{columns}, sample {sample}"
                        );
                    } else {
                        assert!(
                            relative_difference < 1e-7,
                            "fuzz pseudoinverse for {rows}x{columns}, sample {sample}: absolute {difference:e}, relative {relative_difference:e}"
                        );
                    }
                }
            }
        }

        // This deterministically generated matrix has a third singular value
        // around 4e-15. Some SVD implementations return noticeably non-Penrose
        // results after truncation, so judge the difficult case by all four
        // defining identities rather than using one implementation as truth.
        let rank_deficient = DMatrix::from_column_slice(
            3,
            3,
            &[
                -0.5643075887018155,
                -0.9568070030962322,
                -0.009627589979188752,
                0.9969118739931341,
                0.3739092052982786,
                -0.7726245873628075,
                -0.6866079313485899,
                0.48762929521735127,
                0.979109559968583,
            ],
        );
        let (inverse, rank) = pseudo_inverse(&rank_deficient, 1e-9);
        assert_eq!(rank, 2);
        let left_projector = &rank_deficient * &inverse;
        let right_projector = &inverse * &rank_deficient;
        assert!((&left_projector * &rank_deficient - &rank_deficient).norm() < 1e-12);
        assert!((&right_projector * &inverse - &inverse).norm() < 1e-12);
        assert!((&left_projector - left_projector.transpose()).norm() < 1e-12);
        assert!((&right_projector - right_projector.transpose()).norm() < 1e-12);
    }

    #[test]
    fn discarded_jacobi_pair_skip_is_bitwise_identical_to_dot_control() {
        let rows = 6;
        let columns = 4;
        let matrix = [
            1.0, 1e-14, 0.3, 0.0, -0.2, -2e-14, 0.7, 0.0, 0.4, 1e-14, -0.1, 0.0, 0.8, 3e-14, 0.2,
            0.0, -0.6, -1e-14, 0.5, 0.0, 0.1, 2e-14, -0.9, 0.0,
        ];
        let solve = |skip_discarded_couplings| {
            let mut inverse = vec![0.0; rows * columns];
            let mut orthogonal = vec![0.0; rows * columns];
            let mut right = vec![0.0; columns * columns];
            let mut singular = vec![0.0; columns];
            let outcome = pseudo_inverse_flat_into_profiled_with_energy_cache(
                &matrix,
                rows,
                columns,
                1e-9,
                0.0,
                &mut inverse,
                &mut orthogonal,
                &mut right,
                &mut singular,
                true,
                skip_discarded_couplings,
            );
            (outcome, inverse, orthogonal, right, singular)
        };

        let control = solve(false);
        let optimized = solve(true);

        assert_eq!(optimized.0.rank, control.0.rank);
        assert_eq!(optimized.0.jacobi_sweeps, control.0.jacobi_sweeps);
        for (optimized_values, control_values) in [
            (&optimized.1, &control.1),
            (&optimized.2, &control.2),
            (&optimized.3, &control.3),
            (&optimized.4, &control.4),
        ] {
            assert!(
                optimized_values
                    .iter()
                    .zip(control_values)
                    .all(|(left, right)| left.to_bits() == right.to_bits())
            );
        }
    }

    #[test]
    fn jacobi_column_slices_preserve_every_rotation_bit() {
        let mut random_state = 0x27bd_e140_6c8a_f395_u64;
        for (rows, columns) in [(8, 2), (29, 6), (58, 14)] {
            for sample in 0..4 {
                let mut source = Vec::with_capacity(rows * columns);
                for column in 0..columns {
                    for row in 0..rows {
                        random_state ^= random_state << 13;
                        random_state ^= random_state >> 7;
                        random_state ^= random_state << 17;
                        let mut value = random_state as i64 as f64 / i64::MAX as f64;
                        if sample == 0 && column + 1 == columns {
                            value = 0.0;
                        } else if sample == 1 && column + 1 == columns {
                            value = source[row];
                        }
                        source.push(value);
                    }
                }
                let solve = |use_column_slices| {
                    let mut matrix = source.clone();
                    let mut right = vec![0.0; columns * columns];
                    fill_identity(&mut right, columns);
                    let mut energies = vec![0.0; columns];
                    let frobenius = initialize_jacobi_column_energies(
                        &matrix,
                        rows,
                        columns,
                        &mut energies,
                        false,
                    )
                    .sqrt();
                    let floor = (1e-12 * 1e-3 * frobenius.max(1.0)).powi(2);
                    let sweeps = one_sided_jacobi_flat(
                        &mut matrix,
                        rows,
                        columns,
                        &mut right,
                        &mut energies,
                        true,
                        false,
                        floor,
                        use_column_slices,
                        false,
                    );
                    (sweeps, matrix, right, energies)
                };

                let control = solve(false);
                let candidate = solve(true);
                assert_eq!(candidate.0, control.0);
                for (candidate_values, control_values) in [
                    (&candidate.1, &control.1),
                    (&candidate.2, &control.2),
                    (&candidate.3, &control.3),
                ] {
                    assert!(
                        candidate_values
                            .iter()
                            .zip(control_values)
                            .all(|(left, right)| left.to_bits() == right.to_bits())
                    );
                }
            }
        }
    }

    #[test]
    fn jacobi_column_iterators_preserve_every_rotation_bit() {
        let mut random_state = 0x5ec1_81d9_76a2_b04f_u64;
        for (rows, columns) in [(8, 2), (29, 6), (58, 14)] {
            for sample in 0..4 {
                let mut source = Vec::with_capacity(rows * columns);
                for column in 0..columns {
                    for row in 0..rows {
                        random_state ^= random_state << 13;
                        random_state ^= random_state >> 7;
                        random_state ^= random_state << 17;
                        let mut value = random_state as i64 as f64 / i64::MAX as f64;
                        if sample == 0 && column + 1 == columns {
                            value = 0.0;
                        } else if sample == 1 && column + 1 == columns {
                            value = source[row];
                        }
                        source.push(value);
                    }
                }
                let solve = |use_column_iterators| {
                    let mut matrix = source.clone();
                    let mut right = vec![0.0; columns * columns];
                    fill_identity(&mut right, columns);
                    let mut energies = vec![0.0; columns];
                    let frobenius = initialize_jacobi_column_energies(
                        &matrix,
                        rows,
                        columns,
                        &mut energies,
                        false,
                    )
                    .sqrt();
                    let floor = (1e-12 * 1e-3 * frobenius.max(1.0)).powi(2);
                    let sweeps = one_sided_jacobi_flat(
                        &mut matrix,
                        rows,
                        columns,
                        &mut right,
                        &mut energies,
                        true,
                        false,
                        floor,
                        true,
                        use_column_iterators,
                    );
                    (sweeps, matrix, right, energies)
                };

                let control = solve(false);
                let candidate = solve(true);
                assert_eq!(candidate.0, control.0);
                for (candidate_values, control_values) in [
                    (&candidate.1, &control.1),
                    (&candidate.2, &control.2),
                    (&candidate.3, &control.3),
                ] {
                    assert!(
                        candidate_values
                            .iter()
                            .zip(control_values)
                            .all(|(left, right)| left.to_bits() == right.to_bits())
                    );
                }
            }
        }
    }

    #[test]
    fn dense_multiply_row_slices_preserve_every_product_bit() {
        let mut random_state = 0x8742_1c6d_901e_ab35_u64;
        for (left_rows, inner, right_columns) in [(1, 1, 1), (3, 4, 2), (29, 58, 14), (58, 58, 58)]
        {
            for sparse in [false, true] {
                let mut sample = || {
                    random_state ^= random_state << 13;
                    random_state ^= random_state >> 7;
                    random_state ^= random_state << 17;
                    random_state as i64 as f64 / i64::MAX as f64
                };
                let mut left: Vec<_> = (0..left_rows * inner).map(|_| sample()).collect();
                let mut right: Vec<_> = (0..inner * right_columns).map(|_| sample()).collect();
                if sparse {
                    for (index, value) in left.iter_mut().enumerate() {
                        if index % 3 == 0 {
                            *value = 0.0;
                        }
                    }
                    for (index, value) in right.iter_mut().enumerate() {
                        if index % 5 == 0 {
                            *value = -0.0;
                        }
                    }
                }
                let mut control = vec![f64::NAN; left_rows * right_columns];
                let mut candidate = control.clone();
                multiply_flat_indices(&left, left_rows, inner, &right, right_columns, &mut control);
                multiply_row_slices(
                    &left,
                    left_rows,
                    inner,
                    &right,
                    right_columns,
                    &mut candidate,
                );
                assert!(
                    candidate
                        .iter()
                        .zip(control)
                        .all(|(left, right)| left.to_bits() == right.to_bits())
                );
            }
        }
    }

    #[test]
    fn fused_initial_jacobi_energy_scan_is_bitwise_identical_to_control() {
        let rows = 6;
        let columns = 4;
        let matrix = [
            1.0, 1e-14, 0.3, 0.0, -0.2, -2e-14, 0.7, 0.0, 0.4, 1e-14, -0.1, 0.0, 0.8, 3e-14, 0.2,
            0.0, -0.6, -1e-14, 0.5, 0.0, 0.1, 2e-14, -0.9, 0.0,
        ];
        let mut control_energies = [0.0; 4];
        let mut fused_energies = [0.0; 4];

        let control =
            initialize_jacobi_column_energies(&matrix, rows, columns, &mut control_energies, false);
        let fused =
            initialize_jacobi_column_energies(&matrix, rows, columns, &mut fused_energies, true);

        assert_eq!(fused.to_bits(), control.to_bits());
        assert!(
            fused_energies
                .iter()
                .zip(control_energies)
                .all(|(left, right)| left.to_bits() == right.to_bits())
        );
    }

    #[test]
    #[cfg(any(
        feature = "resolved-task-row-compaction-experiment",
        feature = "zero-task-row-compaction-experiment",
        feature = "fixed-style-task-row-compaction-experiment"
    ))]
    fn resolved_task_row_compaction_is_conservative_by_priority() {
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![2.0, -1.0]),
            upper: DVector::from_vec(vec![2.0, 1.0]),
        };
        let task = Task {
            stable_id: 1,
            kind: TaskKind::ContactWrench,
            priority: Priority::Style,
            jacobian: DMatrix::from_row_slice(
                4,
                2,
                &[
                    0.0, 0.0, // exact zero row
                    1.0, 0.0, // fixed and satisfied
                    1.0, 0.0, // fixed but mismatched
                    0.0, 1.0, // free coordinate
                ],
            ),
            target_velocity: DVector::from_vec(vec![0.0, 2.0, 3.0, 0.0]),
            weight: 1.0,
        };

        assert_eq!(
            task_row_is_constant_and_satisfied(&task, 0, Priority::Viability, &bounds),
            zero_task_row_compaction_enabled()
        );
        assert_eq!(
            task_row_is_constant_and_satisfied(&task, 1, Priority::Style, &bounds),
            fixed_style_task_row_compaction_enabled()
        );
        assert!(!task_row_is_constant_and_satisfied(
            &task,
            1,
            Priority::Preference,
            &bounds
        ));
        assert!(!task_row_is_constant_and_satisfied(
            &task,
            2,
            Priority::Style,
            &bounds
        ));
        assert!(!task_row_is_constant_and_satisfied(
            &task,
            3,
            Priority::Style,
            &bounds
        ));
    }

    #[test]
    fn flat_pseudoinverse_matches_established_jacobi_kernel() {
        let mut random_state = 0x76a9_13e4_b8c2_55d1_u64;
        for rows in 1..=24 {
            for columns in 1..=24 {
                for sample in 0..4 {
                    let mut matrix = DMatrix::from_fn(rows, columns, |_, _| {
                        random_state ^= random_state << 13;
                        random_state ^= random_state >> 7;
                        random_state ^= random_state << 17;
                        (random_state as i64 as f64) / i64::MAX as f64
                    });
                    if rows > 1 && sample % 2 == 0 {
                        let first = matrix.row(0).clone_owned();
                        matrix.row_mut(rows - 1).copy_from(&first);
                    }
                    let (expected, expected_rank) = pseudo_inverse(&matrix, 1e-9);
                    let mut inverse = vec![0.0; rows * columns];
                    let mut orthogonal = vec![0.0; rows * columns];
                    let mut right = vec![0.0; rows.min(columns).pow(2)];
                    let mut singular = vec![0.0; rows.min(columns)];
                    let mut row_major = Vec::with_capacity(rows * columns);
                    for row in 0..rows {
                        for column in 0..columns {
                            row_major.push(matrix[(row, column)]);
                        }
                    }
                    let actual_rank = pseudo_inverse_flat_into(
                        &row_major,
                        rows,
                        columns,
                        1e-9,
                        &mut inverse,
                        &mut orthogonal,
                        &mut right,
                        &mut singular,
                    );
                    let actual =
                        DMatrix::from_fn(columns, rows, |row, column| inverse[row * rows + column]);
                    let relative_error = (&actual - &expected).norm() / expected.norm().max(1.0);
                    assert_eq!(
                        actual_rank, expected_rank,
                        "rank mismatch for {rows}x{columns} sample {sample}"
                    );
                    assert!(
                        relative_error < 1e-10,
                        "inverse mismatch for {rows}x{columns} sample {sample}: {relative_error:e}"
                    );
                }
            }
        }
    }

    #[test]
    fn lower_priority_preserves_higher_optimum() {
        let tasks = vec![
            Task {
                stable_id: 1,
                kind: TaskKind::Velocity,
                priority: Priority::Intent,
                jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 0.0]),
                target_velocity: DVector::from_vec(vec![1.0]),
                weight: 1.0,
            },
            Task {
                stable_id: 2,
                kind: TaskKind::Posture,
                priority: Priority::Preference,
                jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
                target_velocity: DVector::from_vec(vec![0.0]),
                weight: 1.0,
            },
        ];
        let result = HierarchicalSolver::default().solve(2, &tasks, &VelocityBounds::unbounded(2));
        assert!((result.velocity[0] - 1.0).abs() < 1e-9);
        assert!((result.velocity[1] + 1.0).abs() < 1e-9);
    }

    #[test]
    fn projected_inverse_is_reused_until_the_nullspace_changes() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::identity(1, 1),
            target_velocity: DVector::from_vec(vec![0.5]),
            weight: 1.0,
        };
        let solver = HierarchicalSolver::default();
        let unconstrained = solver.solve(
            1,
            std::slice::from_ref(&task),
            &VelocityBounds::unbounded(1),
        );
        assert_eq!(unconstrained.diagnostics.task_pseudoinverse_calls, 1);
        assert_eq!(unconstrained.diagnostics.clipped_steps, 0);

        let clipped = solver.solve(
            1,
            &[task],
            &VelocityBounds {
                lower: DVector::from_vec(vec![-1.0]),
                upper: DVector::from_vec(vec![0.25]),
            },
        );
        assert_eq!(clipped.diagnostics.task_pseudoinverse_calls, 2);
        assert_eq!(clipped.diagnostics.clipped_steps, 1);
        assert_eq!(clipped.diagnostics.status, SolveStatus::SolvedWithSlack);
    }

    #[cfg(feature = "coincident-step-limit-freeze-experiment")]
    #[test]
    fn coincident_step_limits_share_one_projected_inverse_rebuild() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Viability,
            jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
            target_velocity: DVector::from_vec(vec![2.0]),
            weight: 1.0,
        };
        let result = HierarchicalSolver::default().solve(
            2,
            &[task],
            &VelocityBounds {
                lower: DVector::from_vec(vec![-1.0, -1.0]),
                upper: DVector::from_vec(vec![0.5, 0.5]),
            },
        );
        assert_eq!(result.velocity.as_slice(), &[0.5, 0.5]);
        assert_eq!(result.diagnostics.clipped_steps, 1);
        assert_eq!(result.diagnostics.task_pseudoinverse_calls, 2);
        assert_eq!(result.diagnostics.status, SolveStatus::SolvedWithSlack);
    }

    #[test]
    fn equality_seed_inverse_is_reused_for_the_nullspace() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
            target_velocity: DVector::from_vec(vec![1.0]),
            weight: 1.0,
        };
        let equality = LinearConstraint {
            stable_id: 7,
            coefficients: RowDVector::from_row_slice(&[1.0, 0.0]),
            lower: 0.25,
            upper: 0.25,
        };
        let result = HierarchicalSolver::default().solve_constrained(
            2,
            &[task],
            &VelocityBounds::unbounded(2),
            &[equality],
        );
        assert!(result.diagnostics.equality_pseudoinverse_reused);
        assert!((result.velocity[0] - 0.25).abs() < 1e-10);
        assert!((result.velocity[1] - 0.75).abs() < 1e-10);
    }

    #[test]
    fn bounds_truncate_without_violating() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::identity(1, 1),
            target_velocity: DVector::from_vec(vec![10.0]),
            weight: 1.0,
        };
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![-1.0]),
            upper: DVector::from_vec(vec![1.0]),
        };
        let result = HierarchicalSolver::default().solve(1, &[task], &bounds);
        assert_eq!(result.velocity[0], 1.0);
        assert_eq!(result.diagnostics.status, SolveStatus::SolvedWithSlack);
    }

    #[test]
    fn active_bound_reoptimizes_same_priority_in_remaining_space() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
            target_velocity: DVector::from_vec(vec![2.0]),
            weight: 1.0,
        };
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![-10.0, -10.0]),
            upper: DVector::from_vec(vec![0.5, 10.0]),
        };
        let result = HierarchicalSolver::default().solve(2, &[task], &bounds);
        assert!(
            (result.velocity[0] - 0.5).abs() < 1e-10,
            "velocity {:?}, diagnostics {:?}",
            result.velocity,
            result.diagnostics
        );
        assert!((result.velocity[1] - 1.5).abs() < 1e-10);
        assert!(result.diagnostics.level_residuals[0].l2 < 1e-10);
        #[cfg(feature = "clipped-task-nullspace-repair-experiment")]
        assert_eq!(result.diagnostics.task_pseudoinverse_calls, 1);
    }

    #[test]
    fn linear_constraint_activates_and_reoptimizes_remaining_space() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
            target_velocity: DVector::from_vec(vec![2.0]),
            weight: 1.0,
        };
        let constraint = LinearConstraint {
            stable_id: 42,
            coefficients: RowDVector::from_row_slice(&[1.0, 0.0]),
            lower: f64::NEG_INFINITY,
            upper: 0.5,
        };
        let result = HierarchicalSolver::default().solve_constrained(
            2,
            &[task],
            &VelocityBounds::unbounded(2),
            &[constraint],
        );
        assert!((result.velocity[0] - 0.5).abs() < 1e-10);
        assert!((result.velocity[1] - 1.5).abs() < 1e-10);
        assert!(result.diagnostics.level_residuals[0].l2 < 1e-10);
        assert_eq!(result.diagnostics.active_constraints, vec![42]);
        assert!(result.diagnostics.maximum_constraint_violation < 1e-10);
    }

    #[test]
    fn feasible_boundary_does_not_prevent_motion_into_interior() {
        let task = Task {
            stable_id: 1,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::identity(1, 1),
            target_velocity: DVector::from_vec(vec![2.0]),
            weight: 1.0,
        };
        let constraint = LinearConstraint {
            stable_id: 1,
            coefficients: RowDVector::from_row_slice(&[1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        };
        let result = HierarchicalSolver::default().solve_constrained(
            1,
            &[task],
            &VelocityBounds::unbounded(1),
            &[constraint],
        );
        assert!((result.velocity[0] - 2.0).abs() < 1e-10);
        assert_eq!(result.diagnostics.status, SolveStatus::Solved);
    }

    #[test]
    fn contradictory_nonzero_rows_exhaust_without_a_false_infeasibility_certificate() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let result = HierarchicalSolver::default().solve_constrained(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
        );
        assert_eq!(result.diagnostics.status, SolveStatus::MaxIterations);
        assert!(result.diagnostics.feasibility_projection_sweeps > 0);
        assert_eq!(result.diagnostics.maximum_bound_violation, 0.0);
        assert_eq!(result.diagnostics.limiting_bound_coordinate, None);
        assert_eq!(result.diagnostics.limiting_linear_constraint, Some(1));
        assert!(!result.diagnostics.limiting_linear_constraint_is_upper);
        assert!(result.diagnostics.maximum_linear_constraint_violation > 0.0);
    }

    #[test]
    fn exhausted_hard_witness_names_the_coordinate_bound_separately() {
        let constraint = LinearConstraint {
            stable_id: 41,
            coefficients: RowDVector::from_row_slice(&[1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        };
        let result = HierarchicalSolver::default().solve_constrained(
            1,
            &[],
            &VelocityBounds {
                lower: DVector::from_vec(vec![f64::NEG_INFINITY]),
                upper: DVector::from_vec(vec![0.0]),
            },
            &[constraint],
        );
        assert_eq!(result.diagnostics.status, SolveStatus::MaxIterations);
        assert_eq!(result.diagnostics.limiting_bound_coordinate, Some(0));
        assert!(result.diagnostics.limiting_bound_is_upper);
        assert!(result.diagnostics.maximum_bound_violation > 0.0);
        assert_eq!(result.diagnostics.maximum_linear_constraint_violation, 0.0);
        assert_eq!(result.diagnostics.limiting_linear_constraint, None);
    }

    #[test]
    fn projection_sweep_ceiling_fails_closed_without_exact_fallback_work() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            ..HierarchicalSolver::default()
        };
        let result = solver.solve_constrained(1, &[], &VelocityBounds::unbounded(1), &constraints);
        assert_eq!(result.diagnostics.status, SolveStatus::MaxIterations);
        assert_eq!(result.diagnostics.feasibility_projection_sweeps, 3);
        assert_eq!(result.diagnostics.feasibility_halfspace_projections, 6);
    }

    #[test]
    fn near_exhausted_projection_can_continue_from_exact_prefix_state() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            feasibility_projection_continuation_violation_threshold: Some(2.0),
            ..HierarchicalSolver::default()
        };
        let result = solver.solve_constrained(1, &[], &VelocityBounds::unbounded(1), &constraints);
        assert_eq!(result.diagnostics.status, SolveStatus::MaxIterations);
        assert_eq!(result.diagnostics.feasibility_projection_sweeps, 512);
        assert_eq!(result.diagnostics.feasibility_halfspace_projections, 1024);
    }

    #[test]
    fn bit_identical_hard_problem_reuses_exact_seed_across_soft_tasks() {
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![-2.0, -2.0]),
            upper: DVector::from_vec(vec![2.0, 2.0]),
        };
        let constraints = [LinearConstraint {
            stable_id: 7,
            coefficients: RowDVector::from_row_slice(&[1.0, 1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        }];
        let task = |stable_id, target: [f64; 2]| Task {
            stable_id,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::identity(2, 2),
            target_velocity: DVector::from_vec(target.to_vec()),
            weight: 1.0,
        };
        let first_task = task(1, [0.25, 0.75]);
        let second_task = task(2, [1.5, -0.25]);
        let solver = HierarchicalSolver {
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };

        let mut cold_workspace = SolverWorkspace::new(2, 2, 1, 1);
        let mut cold = SolveResult::workspace(2, 1);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&second_task),
            &bounds,
            &constraints,
            &mut cold,
            &mut cold_workspace,
        );
        assert!(!cold.diagnostics.feasibility_seed_reused);

        let mut warm_workspace = SolverWorkspace::new(2, 2, 1, 1);
        let mut warm = SolveResult::workspace(2, 1);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&first_task),
            &bounds,
            &constraints,
            &mut warm,
            &mut warm_workspace,
        );
        assert!(!warm.diagnostics.feasibility_seed_reused);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&second_task),
            &bounds,
            &constraints,
            &mut warm,
            &mut warm_workspace,
        );

        assert!(warm.diagnostics.feasibility_seed_reused);
        assert_eq!(warm.diagnostics.status, cold.diagnostics.status);
        assert_eq!(
            warm.diagnostics.feasibility_projection_sweeps,
            cold.diagnostics.feasibility_projection_sweeps
        );
        assert_eq!(
            warm.diagnostics.feasibility_halfspace_projections,
            cold.diagnostics.feasibility_halfspace_projections
        );
        assert_eq!(
            warm.diagnostics.feasibility_polish_iterations,
            cold.diagnostics.feasibility_polish_iterations
        );
        assert_eq!(
            warm.diagnostics.equality_pseudoinverse_reused,
            cold.diagnostics.equality_pseudoinverse_reused
        );
        assert!(
            warm.velocity
                .iter()
                .zip(cold.velocity.iter())
                .all(|(left, right)| left.to_bits() == right.to_bits())
        );

        let changed_constraints = [LinearConstraint {
            lower: 0.9,
            ..constraints[0].clone()
        }];
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&second_task),
            &bounds,
            &changed_constraints,
            &mut warm,
            &mut warm_workspace,
        );
        assert!(!warm.diagnostics.feasibility_seed_reused);
    }

    #[test]
    fn copied_hard_feasibility_witness_is_revalidated_in_an_independent_workspace() {
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![-2.0, -2.0]),
            upper: DVector::from_vec(vec![2.0, 2.0]),
        };
        let constraints = [LinearConstraint {
            stable_id: 7,
            coefficients: RowDVector::from_row_slice(&[1.0, 1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        }];
        let task = |stable_id, target: [f64; 2]| Task {
            stable_id,
            kind: TaskKind::Velocity,
            priority: Priority::Intent,
            jacobian: DMatrix::identity(2, 2),
            target_velocity: DVector::from_vec(target.to_vec()),
            weight: 1.0,
        };
        let source_task = task(1, [0.25, 0.75]);
        let destination_task = task(2, [1.5, -0.25]);
        let solver = HierarchicalSolver {
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };

        let mut source_workspace = SolverWorkspace::new(2, 2, 1, 1);
        let mut source = SolveResult::workspace(2, 1);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&source_task),
            &bounds,
            &constraints,
            &mut source,
            &mut source_workspace,
        );
        assert!(!source.diagnostics.feasibility_seed_reused);

        let mut cold_workspace = SolverWorkspace::new(2, 2, 1, 1);
        let mut cold = SolveResult::workspace(2, 1);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&destination_task),
            &bounds,
            &constraints,
            &mut cold,
            &mut cold_workspace,
        );

        let mut imported_workspace = SolverWorkspace::new(2, 2, 1, 1);
        assert!(imported_workspace.import_hard_feasibility_witness_from(&source_workspace));
        let mut imported = SolveResult::workspace(2, 1);
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&destination_task),
            &bounds,
            &constraints,
            &mut imported,
            &mut imported_workspace,
        );

        assert!(imported.diagnostics.feasibility_seed_reused);
        assert_eq!(imported.diagnostics.status, cold.diagnostics.status);
        assert_eq!(
            imported.diagnostics.feasibility_projection_sweeps,
            cold.diagnostics.feasibility_projection_sweeps
        );
        assert_eq!(
            imported.diagnostics.feasibility_halfspace_projections,
            cold.diagnostics.feasibility_halfspace_projections
        );
        assert_eq!(
            imported.diagnostics.equality_pseudoinverse_reused,
            cold.diagnostics.equality_pseudoinverse_reused
        );
        assert!(
            imported
                .velocity
                .iter()
                .zip(cold.velocity.iter())
                .all(|(left, right)| left.to_bits() == right.to_bits())
        );

        let changed_constraints = [LinearConstraint {
            lower: 0.9,
            ..constraints[0].clone()
        }];
        solver.solve_constrained_into(
            2,
            std::slice::from_ref(&destination_task),
            &bounds,
            &changed_constraints,
            &mut imported,
            &mut imported_workspace,
        );
        assert!(!imported.diagnostics.feasibility_seed_reused);

        let mut profile_workspace = SolverWorkspace::new(2, 2, 1, 1);
        assert!(profile_workspace.import_hard_feasibility_witness_from(&source_workspace));
        let changed_profile = HierarchicalSolver {
            maximum_feasibility_iterations: 63,
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        changed_profile.solve_constrained_into(
            2,
            std::slice::from_ref(&destination_task),
            &bounds,
            &constraints,
            &mut imported,
            &mut profile_workspace,
        );
        assert!(!imported.diagnostics.feasibility_seed_reused);
    }

    #[test]
    fn budget_independent_exact_witness_crosses_only_projection_stopping_controls() {
        let bounds = VelocityBounds {
            lower: DVector::from_vec(vec![-2.0]),
            upper: DVector::from_vec(vec![2.0]),
        };
        let constraints = [LinearConstraint {
            stable_id: 4,
            coefficients: RowDVector::from_row_slice(&[1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        }];
        let source_solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            feasibility_projection_continuation_violation_threshold: Some(0.1),
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let destination_solver = HierarchicalSolver {
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let mut source_workspace = SolverWorkspace::new(1, 0, 0, 1);
        let mut source = SolveResult::workspace(1, 1);
        source_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut source,
            &mut source_workspace,
        );
        assert_eq!(source.diagnostics.status, SolveStatus::Solved);

        let mut destination_workspace = SolverWorkspace::new(1, 0, 0, 1);
        assert!(destination_workspace.import_hard_feasibility_witness_from(&source_workspace));
        let mut destination = SolveResult::workspace(1, 1);
        destination_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut destination,
            &mut destination_workspace,
        );
        assert!(destination.diagnostics.feasibility_seed_reused);
        assert_eq!(
            destination.velocity[0].to_bits(),
            source.velocity[0].to_bits()
        );

        let changed_arithmetic_solver = HierarchicalSolver {
            use_feasibility_row_spans: true,
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        changed_arithmetic_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut destination,
            &mut destination_workspace,
        );
        assert!(!destination.diagnostics.feasibility_seed_reused);

        let mut uncapped_workspace = SolverWorkspace::new(1, 0, 0, 1);
        let mut uncapped = SolveResult::workspace(1, 1);
        destination_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut uncapped,
            &mut uncapped_workspace,
        );
        let mut bounded_destination_workspace = SolverWorkspace::new(1, 0, 0, 1);
        assert!(
            bounded_destination_workspace.import_hard_feasibility_witness_from(&uncapped_workspace)
        );
        source_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut destination,
            &mut bounded_destination_workspace,
        );
        assert!(!destination.diagnostics.feasibility_seed_reused);
    }

    #[test]
    fn exhausted_witness_cannot_cross_projection_budget_profiles() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let source_solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let destination_solver = HierarchicalSolver {
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let mut source_workspace = SolverWorkspace::new(1, 0, 0, 2);
        let mut source = SolveResult::workspace(1, 2);
        source_solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut source,
            &mut source_workspace,
        );
        assert_eq!(source.diagnostics.status, SolveStatus::MaxIterations);

        let mut destination_workspace = SolverWorkspace::new(1, 0, 0, 2);
        assert!(destination_workspace.import_hard_feasibility_witness_from(&source_workspace));
        let mut destination = SolveResult::workspace(1, 2);
        destination_solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut destination,
            &mut destination_workspace,
        );
        assert_eq!(destination.diagnostics.status, SolveStatus::MaxIterations);
        assert!(!destination.diagnostics.feasibility_seed_reused);
        assert!(destination.diagnostics.feasibility_projection_sweeps > 3);
    }

    #[test]
    fn bounded_dykstra_prefix_resumes_to_cold_equivalent_uncapped_result() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let bounded_solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(8),
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let uncapped_solver = HierarchicalSolver {
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let bounds = VelocityBounds::unbounded(1);

        let mut source_workspace = SolverWorkspace::new(1, 0, 0, 2);
        let mut source = SolveResult::workspace(1, 2);
        bounded_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut source,
            &mut source_workspace,
        );
        assert_eq!(source.diagnostics.status, SolveStatus::MaxIterations);
        assert_eq!(source.diagnostics.feasibility_projection_sweeps, 8);

        let mut cold_workspace = SolverWorkspace::new(1, 0, 0, 2);
        let mut cold = SolveResult::workspace(1, 2);
        uncapped_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut cold,
            &mut cold_workspace,
        );

        let mut resumed_workspace = SolverWorkspace::new(1, 0, 0, 2);
        assert!(resumed_workspace.import_hard_feasibility_witness_from(&source_workspace));
        let mut resumed = SolveResult::workspace(1, 2);
        uncapped_solver.solve_constrained_into(
            1,
            &[],
            &bounds,
            &constraints,
            &mut resumed,
            &mut resumed_workspace,
        );

        assert!(resumed.diagnostics.feasibility_seed_reused);
        assert!(resumed.diagnostics.feasibility_prefix_resumed);
        assert_eq!(resumed.diagnostics.status, cold.diagnostics.status);
        assert_eq!(
            resumed.diagnostics.feasibility_projection_sweeps,
            cold.diagnostics.feasibility_projection_sweeps
        );
        assert_eq!(
            resumed.diagnostics.feasibility_halfspace_projections,
            cold.diagnostics.feasibility_halfspace_projections
        );
        assert_eq!(
            resumed.diagnostics.maximum_constraint_violation.to_bits(),
            cold.diagnostics.maximum_constraint_violation.to_bits()
        );
        assert!(resumed.velocity.iter().all(|value| *value == 0.0));
    }

    #[test]
    fn bit_identical_exhausted_hard_problem_reuses_fail_closed_result() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            reuse_identical_hard_feasibility_seed: true,
            ..HierarchicalSolver::default()
        };
        let mut workspace = SolverWorkspace::new(1, 0, 0, 2);
        let mut first = SolveResult::workspace(1, 2);
        solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut first,
            &mut workspace,
        );
        assert_eq!(first.diagnostics.status, SolveStatus::MaxIterations);
        assert!(!first.diagnostics.feasibility_seed_reused);

        let mut second = SolveResult::workspace(1, 2);
        solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut second,
            &mut workspace,
        );
        assert_eq!(second.diagnostics.status, SolveStatus::MaxIterations);
        assert!(second.diagnostics.feasibility_seed_reused);
        assert_eq!(
            second.diagnostics.maximum_constraint_violation.to_bits(),
            first.diagnostics.maximum_constraint_violation.to_bits()
        );
        assert_eq!(
            second.diagnostics.feasibility_projection_sweeps,
            first.diagnostics.feasibility_projection_sweeps
        );
        assert_eq!(
            second.diagnostics.feasibility_halfspace_projections,
            first.diagnostics.feasibility_halfspace_projections
        );

        let mut imported_workspace = SolverWorkspace::new(1, 0, 0, 2);
        assert!(imported_workspace.import_hard_feasibility_witness_from(&workspace));
        let mut imported = SolveResult::workspace(1, 2);
        solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut imported,
            &mut imported_workspace,
        );
        assert_eq!(imported.diagnostics.status, SolveStatus::MaxIterations);
        assert!(imported.diagnostics.feasibility_seed_reused);
        assert!(imported.velocity.iter().all(|value| *value == 0.0));
        assert_eq!(
            imported.diagnostics.maximum_constraint_violation.to_bits(),
            first.diagnostics.maximum_constraint_violation.to_bits()
        );
    }

    #[test]
    fn identical_exhausted_problem_can_spend_a_second_bounded_slice() {
        let constraints = [
            LinearConstraint {
                stable_id: 1,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: 1.0,
                upper: f64::INFINITY,
            },
            LinearConstraint {
                stable_id: 2,
                coefficients: RowDVector::from_row_slice(&[1.0]),
                lower: f64::NEG_INFINITY,
                upper: 0.0,
            },
        ];
        let solver = HierarchicalSolver {
            maximum_feasibility_projection_sweeps: Some(3),
            reuse_identical_hard_feasibility_seed: true,
            continue_identical_exhausted_feasibility_prefix: true,
            ..HierarchicalSolver::default()
        };
        let mut workspace = SolverWorkspace::new(1, 0, 0, 2);
        let mut first = SolveResult::workspace(1, 2);
        solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut first,
            &mut workspace,
        );
        let mut second = SolveResult::workspace(1, 2);
        solver.solve_constrained_into(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &constraints,
            &mut second,
            &mut workspace,
        );
        assert_eq!(first.diagnostics.status, SolveStatus::MaxIterations);
        assert_eq!(second.diagnostics.status, SolveStatus::MaxIterations);
        assert!(second.diagnostics.feasibility_seed_reused);
        assert!(second.diagnostics.feasibility_prefix_resumed);
        assert_eq!(first.diagnostics.feasibility_projection_sweeps, 3);
        assert_eq!(second.diagnostics.feasibility_projection_sweeps, 6);
        assert_eq!(
            second.diagnostics.feasibility_halfspace_projections,
            2 * first.diagnostics.feasibility_halfspace_projections
        );
    }

    #[test]
    fn contradictory_zero_norm_row_never_enters_projection_fallback() {
        let constraint = LinearConstraint {
            stable_id: 9,
            coefficients: RowDVector::from_row_slice(&[0.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        };
        let result = HierarchicalSolver::default().solve_constrained(
            1,
            &[],
            &VelocityBounds::unbounded(1),
            &[constraint],
        );
        assert_eq!(result.diagnostics.status, SolveStatus::PrimalInfeasible);
        assert!(
            result
                .diagnostics
                .maximum_constraint_violation
                .is_infinite()
        );
        assert_eq!(result.diagnostics.feasibility_projection_sweeps, 0);
        assert_eq!(result.diagnostics.feasibility_polish_iterations, 0);
    }
}
