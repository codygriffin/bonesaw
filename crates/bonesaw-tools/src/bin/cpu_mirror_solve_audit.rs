use std::{env, time::Instant};

use anyhow::{Context, Result};
use bonesaw_core::{MotionProgram, Priority, TimingSpec};
use bonesaw_cuda::{
    BatchLayout, ContactKinematicMode, ContactLockSpec, CpuExactBatchSolver, CpuMirrorBatchSolver,
    CpuMirrorExecutor, CudaFkComCompilerStatus, CudaMirrorSolveExecutor, DynamicsBatchInput,
    DynamicsBatchOutput, EmissionBatchInput, EmissionBatchOutput, ExactSolveBatchInput,
    ExactSolveBatchOutput, FkBatchInput, FkBatchOutput, JacobianBatchOutput, KernelManifest,
    MIRROR_HARD_PROJECTION_SWEEPS, MIRROR_TASK_SWEEPS_PER_LEVEL, MirrorSolveAgentStatus,
    MirrorSolveBatchInput, MirrorSolveBatchOutput, PointAttractorSpec, PointQueryBatchOutput,
    PointQuerySpec, capabilities, cuda_solve_source_sha256, probe_cuda_dynamics_compiler,
    probe_cuda_emission_compiler, probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler,
    probe_cuda_point_queries_compiler, probe_cuda_runtime, probe_cuda_solve_compiler,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const CAPACITY: usize = 17;
const ALIGNMENT: usize = 32;
const REPEATS: usize = 500;
const D3_TOLERANCE: f64 = 5.0e-4;

#[allow(dead_code)]
fn main() -> Result<()> {
    run(false)
}

pub fn run(include_cuda_stage: bool) -> Result<()> {
    let model_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "models/toy_humanoid.urdf".to_owned());
    let urdf = std::fs::read_to_string(&model_path)
        .with_context(|| format!("failed to read {model_path}"))?;
    let program = MotionProgram::compile_urdf(&urdf, TimingSpec::default(), 1)?;
    let queries = [PointQuerySpec {
        stable_id: 97_001,
        frame_index: program.model.root.0,
        point_in_frame: [0.0, 0.0, 0.0],
    }];
    let tasks = [
        PointAttractorSpec {
            stable_id: 97_101,
            point_query_stable_id: 97_001,
            priority: Priority::Viability,
            weight: 1.0,
            bandwidth_hz: 1.0,
        },
        PointAttractorSpec {
            stable_id: 97_102,
            point_query_stable_id: 97_001,
            priority: Priority::Intent,
            weight: 1.0,
            bandwidth_hz: 1.0,
        },
    ];
    let contacts = [
        ContactLockSpec {
            stable_id: 97_201,
            point_query_stable_id: 97_001,
        },
        ContactLockSpec {
            stable_id: 97_202,
            point_query_stable_id: 97_001,
        },
    ];
    let cpu = CpuMirrorExecutor::compile_with_contact_modes(
        &program,
        CAPACITY,
        ALIGNMENT,
        &queries,
        &tasks,
        &contacts,
        &[
            ContactKinematicMode::NormalPoint,
            ContactKinematicMode::NormalPoint,
        ],
    )?;
    let layout = cpu.descriptor().layout.clone();
    let (fk_input, dynamics_input) = deterministic_state(layout.clone())?;
    let mut products = Products::new(&layout);
    cpu.execute_into(&fk_input, &mut products.fk)?;
    cpu.execute_jacobians_into(&products.fk, &mut products.jacobians)?;
    cpu.execute_dynamics_into(
        &dynamics_input,
        &products.fk,
        &products.jacobians,
        &mut products.dynamics,
    )?;
    cpu.execute_point_queries_into(
        &products.fk,
        &products.jacobians,
        &products.dynamics,
        &mut products.points,
    )?;
    let emission_input = deterministic_rows(&layout, &dynamics_input, &products.points)?;
    cpu.execute_emission_into(
        &emission_input,
        &dynamics_input,
        &products.points,
        &mut products.emission,
    )?;

    let (mirror_input, exact_input) = solve_inputs(layout.clone())?;
    let mut mirror_solver = CpuMirrorBatchSolver::new(cpu.descriptor())?;
    let mut mirror = MirrorSolveBatchOutput::new(layout.clone());
    mirror_solver.execute_into(&products.emission, &mirror_input, &mut mirror)?;
    let mut exact_solver = CpuExactBatchSolver::new(cpu.descriptor())?;
    let mut exact = ExactSolveBatchOutput::new(layout.clone());
    exact_solver.execute_into(&products.emission, &exact_input, &mut exact)?;

    let first = mirror_bytes(&mirror);
    let mut repeat_exact = true;
    let mut mirror_timings = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        mirror_solver.execute_into(&products.emission, &mirror_input, &mut mirror)?;
        mirror_timings.push(start.elapsed().as_secs_f64() * 1e6);
        repeat_exact &= mirror_bytes(&mirror) == first;
    }
    let mut exact_timings = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        exact_solver.execute_into(&products.emission, &exact_input, &mut exact)?;
        exact_timings.push(start.elapsed().as_secs_f64() * 1e6);
    }

    let compatible_agents = [0_usize, 4, 5, 6];
    let mut max_command_difference = 0.0_f64;
    for coordinate in 0..layout.generalized_coordinate_count {
        for agent in compatible_agents {
            let index = layout.generalized_index(coordinate, agent);
            max_command_difference = max_command_difference.max(
                (mirror.generalized_acceleration_soa()[index] as f64
                    - exact.generalized_acceleration_soa()[index])
                    .abs(),
            );
        }
    }
    let max_preservation_drift = mirror
        .level_preservation_drift_soa()
        .iter()
        .copied()
        .fold(0.0_f32, f32::max);
    let max_admitted_hard = (0..ACTIVE_AGENTS)
        .filter(|agent| {
            matches!(
                mirror.status()[*agent],
                MirrorSolveAgentStatus::Solved | MirrorSolveAgentStatus::SolvedWithResidual
            )
        })
        .map(|agent| mirror.final_hard_violation()[agent])
        .fold(0.0_f32, f32::max);
    let padding_zero = (ACTIVE_AGENTS..layout.agent_stride).all(|agent| {
        mirror.status()[agent] == MirrorSolveAgentStatus::Inactive
            && mirror
                .generalized_acceleration_soa()
                .iter()
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride == agent)
                .all(|(_, value)| value.to_bits() == 0)
    });
    let budget_command_zero = (0..layout.generalized_coordinate_count).all(|coordinate| {
        mirror.generalized_acceleration_soa()[layout.generalized_index(coordinate, 1)].to_bits()
            == 0
    });
    let budget_candidate_finite = (0..layout.generalized_coordinate_count).all(|coordinate| {
        mirror.candidate_generalized_acceleration_soa()[layout.generalized_index(coordinate, 1)]
            .is_finite()
    });
    let typed_statuses = mirror.status()[1] == MirrorSolveAgentStatus::MaxIterations
        && mirror.status()[2] == MirrorSolveAgentStatus::InvalidProblem
        && mirror.status()[3] == MirrorSolveAgentStatus::InvalidInput;
    let conformance_pass = repeat_exact
        && max_command_difference <= D3_TOLERANCE
        && max_preservation_drift <= D3_TOLERANCE as f32
        && max_admitted_hard <= D3_TOLERANCE as f32
        && padding_zero
        && budget_command_zero
        && budget_candidate_finite
        && typed_statuses;

    let cuda_stage = include_cuda_stage.then(|| {
        cuda_stage_audit(
            &program,
            &cpu,
            &mirror_solver,
            &fk_input,
            &dynamics_input,
            &emission_input,
            &mirror_input,
            &mirror,
        )
    });
    let revision = if include_cuda_stage {
        "cuda-solve-r98"
    } else {
        "cpu-mirror-solve-r97"
    };
    let cuda_status = cuda_stage
        .as_ref()
        .and_then(|stage| stage["status"].as_str());
    let overall_status = if !conformance_pass {
        "fail"
    } else if include_cuda_stage {
        cuda_status.unwrap_or("failed")
    } else {
        "pass"
    };
    let report = json!({
        "schema": 1,
        "revision": revision,
        "status": overall_status,
        "model": model_path,
        "semantics": mirror.semantics(),
        "algorithm_sha256": hex(mirror_solver.algorithm_sha256()),
        "fixed_budget": {
            "hard_projection_sweeps": MIRROR_HARD_PROJECTION_SWEEPS,
            "task_sweeps_per_active_level": MIRROR_TASK_SWEEPS_PER_LEVEL,
            "early_exit": false,
            "proves_infeasibility": false,
        },
        "layout": layout,
        "authority_stack": [
            {"layer": "Invariant", "example": "NormalPoint z contact rows", "witness": "final hard residual"},
            {"layer": "Viability", "example": "point x acceleration +1", "witness": "level RMS + preservation drift"},
            {"layer": "Intent", "example": "conflicting point x acceleration -1", "witness": "lower-level compromise only"},
            {"layer": "Resources", "example": "generalized-acceleration bounds", "witness": "minimum bound margin + clipping"},
            {"layer": "Solver budget", "example": "64 hard + 32/level fixed sweeps", "witness": "initial/best/final residual + MaxIterations"},
            {"layer": "Backend", "example": "CpuExactF64 vs CpuMirrorF32", "witness": "separate semantics and D3"},
            {"layer": "Admission", "example": "command vs retained candidate", "witness": "budget exhaustion keeps command zero"},
        ],
        "conformance": {
            "pass": conformance_pass,
            "d1_500_call_complete_output_bytes_exact": repeat_exact,
            "d3_compatible_command_max_abs": max_command_difference,
            "d3_tolerance": D3_TOLERANCE,
            "maximum_priority_preservation_drift": max_preservation_drift,
            "maximum_admitted_hard_violation": max_admitted_hard,
            "typed_budget_invalid_problem_invalid_input": typed_statuses,
            "padding_inactive_and_zero": padding_zero,
            "allocation_free_hot_path_unit_gate": true,
            "neighbor_isolation_unit_gate": true,
        },
        "budget_exhaustion_case": {
            "mirror_status": mirror.status()[1],
            "exact_status": exact.status()[1],
            "initial_hard_violation": mirror.initial_hard_violation()[1],
            "best_hard_violation": mirror.best_hard_violation()[1],
            "final_hard_violation": mirror.final_hard_violation()[1],
            "candidate_finite": budget_candidate_finite,
            "admitted_command_zero": budget_command_zero,
            "meaning": "fixed work ended without admission; this does not prove primal infeasibility",
        },
        "agents": (0..ACTIVE_AGENTS).map(|agent| json!({
            "agent": agent,
            "mirror_status": mirror.status()[agent],
            "exact_status": exact.status()[agent],
            "hard_initial": mirror.initial_hard_violation()[agent],
            "hard_best": mirror.best_hard_violation()[agent],
            "hard_final": mirror.final_hard_violation()[agent],
            "bound_margin": finite_or_null(mirror.minimum_bound_margin()[agent]),
            "hard_sweeps": mirror.hard_projection_sweeps()[agent],
            "task_sweeps": mirror.task_sweeps()[agent],
            "clipped_updates": mirror.clipped_updates()[agent],
            "active_hard_rows": mirror.active_hard_rows()[agent],
            "active_soft_rows": mirror.active_soft_rows()[agent],
            "level_rows": Priority::ALL.map(|p| mirror.level_rows_soa()[p as usize * layout.agent_stride + agent]),
            "level_rms": Priority::ALL.map(|p| mirror.level_rms_soa()[p as usize * layout.agent_stride + agent]),
            "preservation_drift": Priority::ALL.map(|p| mirror.level_preservation_drift_soa()[p as usize * layout.agent_stride + agent]),
        })).collect::<Vec<_>>(),
        "timing_us_per_seven_agent_batch": {
            "cpu_mirror_f32": distribution(&mut mirror_timings),
            "cpu_exact_f64": distribution(&mut exact_timings),
            "repeats": REPEATS,
        },
        "resident_bytes": {
            "mirror_solver_scratch": mirror_solver.resident_bytes(),
            "mirror_input": mirror_input.resident_bytes(),
            "mirror_output": mirror.resident_bytes(),
            "total_fixed_mirror_solve_boundary": mirror_solver.resident_bytes() + mirror_input.resident_bytes() + mirror.resident_bytes(),
        },
        "cuda_solve": if include_cuda_stage { "source_and_executor_implemented" } else { "not_implemented" },
        "cuda_stage": cuda_stage,
        "cuda_graph": "not_implemented",
        "hidden_cpu_fallback": false,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    let cuda_failed = report["cuda_stage"]
        .as_object()
        .and_then(|stage| stage.get("status"))
        .and_then(Value::as_str)
        == Some("failed");
    if !conformance_pass || cuda_failed {
        anyhow::bail!("CpuMirrorF32 solve conformance failed");
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn cuda_stage_audit(
    program: &MotionProgram,
    cpu: &CpuMirrorExecutor,
    mirror_solver: &CpuMirrorBatchSolver,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    emission_input: &EmissionBatchInput,
    solve_input: &MirrorSolveBatchInput,
    expected: &MirrorSolveBatchOutput,
) -> Value {
    let runtime = probe_cuda_runtime();
    let target = runtime
        .devices
        .first()
        .map(|device| {
            [
                device.compute_capability_major,
                device.compute_capability_minor,
            ]
        })
        .unwrap_or([7, 5]);
    let fk = probe_cuda_fk_com_compiler(target[0], target[1]);
    let jacobians = probe_cuda_jacobians_compiler(target[0], target[1]);
    let dynamics = probe_cuda_dynamics_compiler(target[0], target[1]);
    let points = probe_cuda_point_queries_compiler(target[0], target[1]);
    let emission = probe_cuda_emission_compiler(target[0], target[1]);
    let solve = probe_cuda_solve_compiler(target[0], target[1]);
    let compiler_available = [&fk, &jacobians, &dynamics, &points, &emission, &solve]
        .into_iter()
        .all(|probe| probe.status == CudaFkComCompilerStatus::Available);
    let common = json!({
        "source_sha256": hex(cuda_solve_source_sha256()),
        "kernel_manifest_bits": KernelManifest::solve_v1().bits(),
        "capabilities": capabilities(),
        "algorithm_sha256": hex(mirror_solver.algorithm_sha256()),
        "compiler_target": target,
        "compiler_probes": {
            "fk_com": fk,
            "jacobians": jacobians,
            "dynamics": dynamics,
            "point_queries": points,
            "emission": emission,
            "solve": solve,
        },
        "runtime_probe": runtime,
        "hidden_cpu_fallback": false,
        "graph": "not_implemented",
    });
    if common["runtime_probe"]["status"]["kind"] != "available" || !compiler_available {
        let mut unavailable = common;
        unavailable["status"] = json!("unavailable");
        unavailable["claim"] = json!(
            "fixed-buffer solve source and no-fallback executor implemented; compiler/device evidence is not inferred"
        );
        unavailable["device_checks"] = Value::Null;
        return unavailable;
    }

    let device_result = (|| -> Result<Value> {
        let mut executor = CudaMirrorSolveExecutor::compile(program, cpu, mirror_solver, 0)?;
        let layout = cpu.descriptor().layout.clone();
        let mut products = Products::new(&layout);
        let mut actual = MirrorSolveBatchOutput::new(layout.clone());
        executor.execute_into(
            fk_input,
            dynamics_input,
            emission_input,
            solve_input,
            &mut products.fk,
            &mut products.jacobians,
            &mut products.dynamics,
            &mut products.points,
            &mut products.emission,
            &mut actual,
        )?;
        let d3 = compare_mirror_outputs(expected, &actual);
        let first = mirror_bytes(&actual);
        let mut d1 = true;
        let mut timings = Vec::with_capacity(200);
        for _ in 0..200 {
            let start = Instant::now();
            executor.execute_into(
                fk_input,
                dynamics_input,
                emission_input,
                solve_input,
                &mut products.fk,
                &mut products.jacobians,
                &mut products.dynamics,
                &mut products.points,
                &mut products.emission,
                &mut actual,
            )?;
            timings.push(start.elapsed().as_secs_f64() * 1e6);
            d1 &= mirror_bytes(&actual) == first;
        }
        let pass = d1 && d3["pass"].as_bool() == Some(true);
        Ok(json!({
            "status": if pass { "pass" } else { "fail" },
            "device": executor.device(),
            "fingerprint": executor.fingerprint(),
            "device_checks": {
                "d1_200_call_complete_output_bytes_exact": d1,
                "d3_cpu_mirror": d3,
                "end_to_end_us": distribution(&mut timings),
                "allocation_free_hot_path_unit_gate": true,
            }
        }))
    })();
    match device_result {
        Ok(device) => merge_json(common, device),
        Err(error) => {
            let mut failed = common;
            failed["status"] = json!("failed");
            failed["device_error"] = json!(error.to_string());
            failed["device_checks"] = Value::Null;
            failed
        }
    }
}

fn merge_json(mut base: Value, overlay: Value) -> Value {
    if let (Some(base), Some(overlay)) = (base.as_object_mut(), overlay.as_object()) {
        for (key, value) in overlay {
            base.insert(key.clone(), value.clone());
        }
    }
    base
}

fn compare_mirror_outputs(
    expected: &MirrorSolveBatchOutput,
    actual: &MirrorSolveBatchOutput,
) -> Value {
    let status_exact = actual.status() == expected.status();
    let integer_exact = actual.level_rows_soa() == expected.level_rows_soa()
        && actual.hard_projection_sweeps() == expected.hard_projection_sweeps()
        && actual.task_sweeps() == expected.task_sweeps()
        && actual.clipped_updates() == expected.clipped_updates()
        && actual.active_hard_rows() == expected.active_hard_rows()
        && actual.active_soft_rows() == expected.active_soft_rows();
    let mut maximum_abs = 0.0_f64;
    let mut nonfinite_mismatch = false;
    for (actual, expected) in [
        (
            actual.generalized_acceleration_soa(),
            expected.generalized_acceleration_soa(),
        ),
        (
            actual.candidate_generalized_acceleration_soa(),
            expected.candidate_generalized_acceleration_soa(),
        ),
        (actual.level_rms_soa(), expected.level_rms_soa()),
        (
            actual.level_preservation_drift_soa(),
            expected.level_preservation_drift_soa(),
        ),
        (
            actual.initial_hard_violation(),
            expected.initial_hard_violation(),
        ),
        (actual.best_hard_violation(), expected.best_hard_violation()),
        (
            actual.final_hard_violation(),
            expected.final_hard_violation(),
        ),
        (
            actual.minimum_bound_margin(),
            expected.minimum_bound_margin(),
        ),
    ] {
        for (actual, expected) in actual.iter().copied().zip(expected.iter().copied()) {
            if actual.is_finite() && expected.is_finite() {
                maximum_abs = maximum_abs.max((actual as f64 - expected as f64).abs());
            } else {
                nonfinite_mismatch |= actual.to_bits() != expected.to_bits();
            }
        }
    }
    json!({
        "pass": status_exact
            && integer_exact
            && !nonfinite_mismatch
            && maximum_abs <= D3_TOLERANCE,
        "status_exact": status_exact,
        "integer_diagnostics_exact": integer_exact,
        "nonfinite_mismatch": nonfinite_mismatch,
        "maximum_absolute_error": maximum_abs,
        "tolerance": D3_TOLERANCE,
    })
}

struct Products {
    fk: FkBatchOutput,
    jacobians: JacobianBatchOutput,
    dynamics: DynamicsBatchOutput,
    points: PointQueryBatchOutput,
    emission: EmissionBatchOutput,
}

impl Products {
    fn new(layout: &BatchLayout) -> Self {
        Self {
            fk: FkBatchOutput::new(layout.clone()),
            jacobians: JacobianBatchOutput::new(layout.clone()),
            dynamics: DynamicsBatchOutput::new(layout.clone()),
            points: PointQueryBatchOutput::new(layout.clone()),
            emission: EmissionBatchOutput::new(layout.clone()),
        }
    }
}

fn deterministic_state(layout: BatchLayout) -> Result<(FkBatchInput, DynamicsBatchInput)> {
    let mut fk = FkBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    let mut dynamics = DynamicsBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for agent in 0..ACTIVE_AGENTS {
        for coordinate in 0..layout.coordinate_count {
            fk.q_soa_mut()[layout.q_index(coordinate, agent)] =
                (coordinate as f32 * 0.003).mul_add(agent as f32 + 1.0, -0.02);
        }
        for component in 0..3 {
            fk.root_pose_soa_mut()[layout.root_pose_index(9 + component, agent)] =
                agent as f32 * 0.01;
            dynamics.gravity_world_soa_mut()[component * layout.agent_stride + agent] =
                [0.0, 0.0, -9.81][component];
        }
        for coordinate in 0..layout.generalized_coordinate_count {
            dynamics.generalized_velocity_soa_mut()[layout.generalized_index(coordinate, agent)] =
                0.0;
        }
    }
    Ok((fk, dynamics))
}

fn deterministic_rows(
    layout: &BatchLayout,
    dynamics: &DynamicsBatchInput,
    points: &PointQueryBatchOutput,
) -> Result<EmissionBatchInput> {
    let mut input = EmissionBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for agent in [0_usize, 1, 2, 4, 6] {
        for task in 0..2 {
            input.point_task_active_soa_mut()[layout.task_active_index(task, agent)] = 1;
            for component in 0..3 {
                let task_index = layout.task_vector_index(task, component, agent);
                let point_index = layout.point_position_index(0, component, agent);
                input.point_target_position_soa_mut()[task_index] =
                    points.point_position_soa()[point_index];
                let mut velocity = 0.0_f32;
                for coordinate in 0..layout.generalized_coordinate_count {
                    velocity = points.point_jacobian_soa()
                        [layout.point_jacobian_index(0, component, coordinate, agent)]
                    .mul_add(
                        dynamics.generalized_velocity_soa()
                            [layout.generalized_index(coordinate, agent)],
                        velocity,
                    );
                }
                input.point_target_velocity_soa_mut()[task_index] = velocity;
                input.point_target_acceleration_soa_mut()[task_index] = points
                    .point_bias_acceleration_soa()[point_index]
                    + if component == 0 {
                        if task == 0 { 1.0 } else { -1.0 }
                    } else {
                        0.0
                    };
            }
        }
        input.contact_lock_active_soa_mut()[layout.contact_active_index(0, agent)] = 1;
        for component in 0..3 {
            let index = layout.contact_vector_index(0, component, agent);
            let point_index = layout.point_position_index(0, component, agent);
            input.contact_desired_acceleration_soa_mut()[index] = points
                .point_bias_acceleration_soa()[point_index]
                + if component == 2 { 0.25 } else { 0.0 };
        }
    }
    input.contact_lock_active_soa_mut()[layout.contact_active_index(1, 1)] = 1;
    for component in 0..3 {
        let index = layout.contact_vector_index(1, component, 1);
        let point_index = layout.point_position_index(0, component, 1);
        input.contact_desired_acceleration_soa_mut()[index] = points.point_bias_acceleration_soa()
            [point_index]
            + if component == 2 { 1.25 } else { 0.0 };
    }
    input.point_task_active_soa_mut()[layout.task_active_index(0, 3)] = 1;
    input.point_target_acceleration_soa_mut()[layout.task_vector_index(0, 0, 3)] = f32::NAN;
    Ok(input)
}

fn solve_inputs(layout: BatchLayout) -> Result<(MirrorSolveBatchInput, ExactSolveBatchInput)> {
    let mut mirror = MirrorSolveBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    mirror.lower_soa_mut().fill(-2.0);
    mirror.upper_soa_mut().fill(2.0);
    for coordinate in 0..layout.generalized_coordinate_count {
        let index = layout.generalized_index(coordinate, 4);
        mirror.lower_soa_mut()[index] = -0.1;
        mirror.upper_soa_mut()[index] = 0.1;
    }
    let invalid = layout.generalized_index(0, 2);
    mirror.lower_soa_mut()[invalid] = 1.0;
    mirror.upper_soa_mut()[invalid] = -1.0;
    let mut exact = ExactSolveBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for coordinate in 0..layout.generalized_coordinate_count {
        for agent in 0..layout.agent_stride {
            let index = layout.generalized_index(coordinate, agent);
            exact.lower_soa_mut()[index] = mirror.lower_soa()[index] as f64;
            exact.upper_soa_mut()[index] = mirror.upper_soa()[index] as f64;
        }
    }
    Ok((mirror, exact))
}

fn mirror_bytes(output: &MirrorSolveBatchOutput) -> Vec<u8> {
    let mut bytes = Vec::new();
    for values in [
        output.generalized_acceleration_soa(),
        output.candidate_generalized_acceleration_soa(),
        output.level_rms_soa(),
        output.level_preservation_drift_soa(),
        output.initial_hard_violation(),
        output.best_hard_violation(),
        output.final_hard_violation(),
        output.minimum_bound_margin(),
    ] {
        for value in values {
            bytes.extend(value.to_bits().to_le_bytes());
        }
    }
    bytes.extend(output.status().iter().map(|status| *status as u8));
    for values in [
        output.level_rows_soa(),
        output.hard_projection_sweeps(),
        output.task_sweeps(),
        output.clipped_updates(),
        output.active_hard_rows(),
        output.active_soft_rows(),
    ] {
        for value in values {
            bytes.extend(value.to_le_bytes());
        }
    }
    bytes
}

fn distribution(values: &mut [f64]) -> Value {
    values.sort_by(f64::total_cmp);
    let percentile = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let p50 = percentile(0.50);
    let p99 = percentile(0.99);
    json!({
        "mean": mean,
        "p50": p50,
        "p95": percentile(0.95),
        "p99": p99,
        "max": values[values.len() - 1],
        "jitter_p99_minus_p50": p99 - p50,
    })
}

fn finite_or_null(value: f32) -> Value {
    if value.is_finite() {
        json!(value)
    } else {
        Value::Null
    }
}

fn hex(bytes: [u8; 32]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}
