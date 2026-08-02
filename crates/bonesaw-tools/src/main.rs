use std::{
    alloc::{GlobalAlloc, Layout, System},
    env,
    hint::black_box,
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use anyhow::{Context, Result, bail};
use bonesaw_core::{
    CollisionAvoidanceConfig, CompiledCollisionModel, CompiledFrameAtlas, CompiledSignalProgram,
    CompiledTaskProgram, ContactMode, ContactSpec, Controller, ControllerConfig, ControllerInput,
    ControllerOutputBuffer, ControllerScratch, ControllerState, DynamicWbc, DynamicWbcConfig,
    DynamicWbcInput, DynamicWbcOutput, DynamicWbcScratch, DynamicsCache, ExternalFrameHistories,
    ExternalFrameInputs, ExternalFrameSample, FloatingDynamicWbc, FloatingDynamicWbcInput,
    FloatingDynamicWbcOutput, FloatingDynamicWbcScratch, FloatingJointAccelerationTask,
    FloatingPointAccelerationTask, FloatingRobotState, FloatingTaskCommand, FloatingTaskPriorities,
    FloatingTaskState, FrameAtlasSnapshot, FrameId, FrameQuery, FrameTarget, HierarchicalSolver,
    HistoricalFrameQuery, HistoryQueryPolicy, LinearConstraint, ModelCache, Motion6, MotionProgram,
    PlanarIkOptions, PlanarIkScratch, PlanarPointIkTarget, Priority, RobotHistory, RobotState,
    RotationJet, ScalarJet, SignalInputFrame, SignalJet, SignalMemory, SignalOp,
    SignalOutputBuffer, SignalOutputSpec, SignalScratch, SolveStatus, StepStatus, Task, TaskKind,
    TaskSpec, TimedRobotState, TimingSpec, Transform3, Vec3, VectorJet, VelocityBounds,
    solve_planar_point_ik_into,
};
use bonesaw_tools::{UpkieWheelBalancer, UpkieWheelBalancerState, compile_floating_balance_policy};
use nalgebra::{DMatrix, DVector, Point3, Quaternion, RowDVector, UnitQuaternion};
use serde::Serialize;

struct CountingAllocator;

static ALLOCATION_CALLS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
        unsafe { System.realloc(pointer, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Clone, Debug)]
struct Options {
    model: PathBuf,
    ticks: usize,
    json: bool,
    inspect: bool,
    squat_only: bool,
    floating_wbc_only: bool,
}

#[derive(Clone, Debug, Serialize)]
struct ScenarioReport {
    name: &'static str,
    ticks: usize,
    rms_position_error_m: f64,
    max_position_error_m: f64,
    max_joint_limit_violation: f64,
    degraded_ticks: usize,
    contingency_ticks: usize,
    mean_tick_us: f64,
    p50_tick_us: f64,
    p99_tick_us: f64,
    max_tick_us: f64,
    allocations_per_tick: f64,
    allocated_bytes_per_tick: f64,
}

#[derive(Clone, Debug, Serialize)]
struct DeterminismReport {
    ticks: usize,
    bitwise_equal: bool,
    first_mismatch_tick: Option<usize>,
}

#[derive(Clone, Debug, Serialize)]
struct QueryReport {
    queries: usize,
    elapsed_ms: f64,
    queries_per_second: f64,
}

#[derive(Clone, Debug, Serialize)]
struct DynamicsReport {
    states: usize,
    maximum_mass_symmetry_error: f64,
    minimum_mass_eigenvalue: f64,
    maximum_inverse_forward_error: f64,
    mean_round_trip_us: f64,
    p99_round_trip_us: f64,
}

#[derive(Clone, Debug, Serialize)]
struct CollisionReport {
    sphere_proxies: usize,
    self_pairs: usize,
    virtual_bridge_exclusions: usize,
    distance_samples: usize,
    minimum_signed_distance_m: f64,
    minimum_pair_id: Option<u32>,
    minimum_pair_bodies: Option<[String; 2]>,
    maximum_jacobian_finite_difference_error: f64,
    p99_full_scan_us: f64,
    controller_allocations_per_tick: f64,
    controller_allocated_bytes_per_tick: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ConstraintReport {
    maximum_feasible_violation: f64,
    active_reoptimized_residual: f64,
    contradictory_problem_detected: bool,
    p99_solve_us: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SignalGraphReport {
    ticks: usize,
    nodes: usize,
    outputs: usize,
    memory_slots: usize,
    bitwise_repeat: bool,
    maximum_output_acceleration_abs: f64,
    mean_step_us: f64,
    p50_step_us: f64,
    p99_step_us: f64,
    max_step_us: f64,
    allocations_per_step: f64,
    allocated_bytes_per_step: f64,
}

#[derive(Clone, Debug, Serialize)]
struct CompiledRigReport {
    ticks: usize,
    signal_nodes: usize,
    task_slots: usize,
    tracking_rms_m: f64,
    orientation_tracking_rms_rad: f64,
    bitwise_repeat: bool,
    mean_tick_us: f64,
    p50_tick_us: f64,
    p99_tick_us: f64,
    max_tick_us: f64,
    allocations_per_tick: f64,
    allocated_bytes_per_tick: f64,
}

#[derive(Clone, Debug, Serialize)]
struct DynamicWbcReport {
    ticks: usize,
    contacts: usize,
    decision_variables: usize,
    maximum_dynamics_residual: f64,
    maximum_contact_acceleration_residual: f64,
    minimum_friction_margin: f64,
    minimum_torque_margin: f64,
    acceleration_tracking_rms: f64,
    maximum_acceleration_abs: f64,
    maximum_torque_abs: f64,
    maximum_contact_force_abs: f64,
    friction_active_ticks: usize,
    infeasible_ticks: usize,
    bitwise_repeat: bool,
    mean_tick_us: f64,
    p50_tick_us: f64,
    p99_tick_us: f64,
    max_tick_us: f64,
    allocations_per_tick: f64,
    allocated_bytes_per_tick: f64,
}

#[derive(Clone, Debug, Serialize)]
struct NativeWbcSentinelReport {
    model: String,
    program_fingerprint: String,
    scope: &'static str,
    floating_dynamic_wbc: DynamicWbcReport,
}

#[derive(Clone, Debug, Serialize)]
struct FloatingSquatReport {
    ticks: usize,
    compiled_signal_nodes: usize,
    compiled_task_slots: usize,
    maximum_policy_oracle_acceleration_delta: f64,
    target_lowering_m: f64,
    achieved_lowering_m: f64,
    final_root_height_error_m: f64,
    root_height_tracking_rms_m: f64,
    root_horizontal_tracking_rms_m: f64,
    final_root_horizontal_error_m: f64,
    center_of_mass_tracking_rms_m: f64,
    maximum_center_of_mass_tracking_error_m: f64,
    maximum_contact_position_delta_m: f64,
    maximum_constrained_contact_drift_m: f64,
    maximum_permitted_rolling_travel_m: f64,
    maximum_root_rotation_rad: f64,
    maximum_generalized_acceleration_abs: f64,
    maximum_actuator_torque_abs: f64,
    maximum_contact_force_abs: f64,
    maximum_dynamics_residual: f64,
    maximum_contact_acceleration_residual: f64,
    minimum_friction_margin: f64,
    minimum_torque_margin: f64,
    degraded_ticks: usize,
    infeasible_ticks: usize,
    mean_step_us: f64,
    p50_step_us: f64,
    p99_step_us: f64,
    max_step_us: f64,
    allocations_per_step: f64,
    allocated_bytes_per_step: f64,
}

#[derive(Clone, Debug, Serialize)]
struct EvaluationReport {
    model: String,
    program_fingerprint: String,
    bodies: usize,
    dof: usize,
    total_mass_kg: f64,
    scenarios: Vec<ScenarioReport>,
    determinism: DeterminismReport,
    frame_queries: QueryReport,
    historical_frame_queries: QueryReport,
    dynamics: DynamicsReport,
    collision: CollisionReport,
    constraints: ConstraintReport,
    signal_graph: SignalGraphReport,
    compiled_rig: CompiledRigReport,
    dynamic_wbc: DynamicWbcReport,
    floating_dynamic_wbc: DynamicWbcReport,
    floating_squat: Option<FloatingSquatReport>,
    floating_unprojected_squat: Option<FloatingSquatReport>,
}

#[derive(Clone, Copy)]
enum Scenario {
    Reach,
    BimanualConflict,
    WalkRetarget,
}

fn main() -> Result<()> {
    let options = parse_options()?;
    let program = MotionProgram::compile_urdf_file(&options.model, TimingSpec::default(), 1)
        .with_context(|| format!("compiling {}", options.model.display()))?;
    let model = &program.model;
    let fingerprint = fingerprint_hex(&program.header.fingerprint_sha256);
    if options.inspect {
        let inspection = ModelInspection {
            model: model.name.clone(),
            program_fingerprint: fingerprint,
            schema: program.header.schema_version,
            bodies: model.bodies.len(),
            dof: model.dof,
            actuators: program.actuation.actuators.len(),
            actuator_resource_models: program
                .actuation
                .actuators
                .iter()
                .filter(|actuator| actuator.resource_model.is_some())
                .count(),
            total_mass_kg: model.bodies.iter().map(|body| body.mass).sum(),
            collision_shapes: model.bodies.iter().map(|body| body.collisions.len()).sum(),
            coordinates: model
                .coordinate_names()
                .into_iter()
                .map(str::to_owned)
                .collect(),
        };
        if options.json {
            println!("{}", serde_json::to_string_pretty(&inspection)?);
        } else {
            println!(
                "{}: {} bodies, {} DOF, {} actuators, schema {}, {:.3} kg, {} collision shapes, {} resource profiles\nfingerprint: {}\ncoordinates: {}",
                inspection.model,
                inspection.bodies,
                inspection.dof,
                inspection.actuators,
                inspection.schema,
                inspection.total_mass_kg,
                inspection.collision_shapes,
                inspection.actuator_resource_models,
                inspection.program_fingerprint,
                inspection.coordinates.join(", ")
            );
        }
        return Ok(());
    }
    if options.squat_only {
        if model.name != "upkie" {
            bail!("--squat-only requires the Upkie model");
        }
        let squat = floating_squat_eval(model, options.ticks.max(100), false)?;
        if options.json {
            println!("{}", serde_json::to_string_pretty(&squat)?);
        } else {
            println!("{squat:#?}");
        }
        return Ok(());
    }
    if options.floating_wbc_only {
        let sentinel = NativeWbcSentinelReport {
            model: model.name.clone(),
            program_fingerprint: fingerprint,
            scope: "native process with one fixed-shape floating WBC solve per measured tick; model loading, setup, timing collection, semantic aggregation, and serialization remain outside or around the solve loop",
            floating_dynamic_wbc: floating_dynamic_wbc_eval(model, options.ticks)?,
        };
        if options.json {
            println!("{}", serde_json::to_string_pretty(&sentinel)?);
        } else {
            println!("{sentinel:#?}");
        }
        return Ok(());
    }
    let report = EvaluationReport {
        model: model.name.clone(),
        program_fingerprint: fingerprint,
        bodies: model.bodies.len(),
        dof: model.dof,
        total_mass_kg: model.bodies.iter().map(|body| body.mass).sum(),
        scenarios: vec![
            run_scenario(model, options.ticks, Scenario::Reach)?,
            run_scenario(model, options.ticks, Scenario::BimanualConflict)?,
            run_scenario(model, options.ticks, Scenario::WalkRetarget)?,
        ],
        determinism: determinism_eval(model, options.ticks.min(500))?,
        frame_queries: query_benchmark(model, options.ticks.max(100) * 100)?,
        historical_frame_queries: historical_query_benchmark(model, options.ticks.max(100) * 10)?,
        dynamics: dynamics_eval(model, options.ticks.clamp(20, 200))?,
        collision: collision_eval(model, options.ticks.clamp(20, 100))?,
        constraints: constraint_eval(options.ticks.max(100))?,
        signal_graph: signal_graph_eval(options.ticks.max(100))?,
        compiled_rig: compiled_rig_eval(&program, options.ticks.max(100))?,
        dynamic_wbc: dynamic_wbc_eval(model, options.ticks)?,
        floating_dynamic_wbc: floating_dynamic_wbc_eval(model, options.ticks)?,
        floating_squat: if model.name == "upkie" {
            Some(floating_squat_eval(model, options.ticks.max(100), true)?)
        } else {
            None
        },
        floating_unprojected_squat: if model.name == "upkie" {
            Some(floating_squat_eval(model, options.ticks.max(100), false)?)
        } else {
            None
        },
    };

    if options.json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print_human(&report);
    }
    Ok(())
}

fn parse_options() -> Result<Options> {
    let mut options = Options {
        model: PathBuf::from("models/toy_humanoid.urdf"),
        ticks: 500,
        json: false,
        inspect: false,
        squat_only: false,
        floating_wbc_only: false,
    };
    let mut arguments = env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--model" => {
                options.model = arguments
                    .next()
                    .map(PathBuf::from)
                    .context("--model requires a path")?;
            }
            "--ticks" => {
                options.ticks = arguments
                    .next()
                    .context("--ticks requires a positive integer")?
                    .parse()
                    .context("invalid --ticks value")?;
                if options.ticks == 0 {
                    bail!("--ticks must be positive");
                }
            }
            "--json" => options.json = true,
            "--inspect" => options.inspect = true,
            "--squat-only" => options.squat_only = true,
            "--floating-wbc-only" => options.floating_wbc_only = true,
            "--help" | "-h" => {
                println!(
                    "Usage: bonesaw-eval [--model PATH] [--ticks N] [--json] [--inspect] [--squat-only] [--floating-wbc-only]\n\
                     Runs reach, conflicting-priority, walking-retarget, replay-determinism,\n\
                     and fast frame-query evaluations. --inspect only validates a model;\n\
                     --squat-only runs the focused raw Upkie rolling-balance sentinel;\n\
                     --floating-wbc-only runs the native fixed-input CPU counter sentinel."
                );
                std::process::exit(0);
            }
            unknown => bail!("unknown argument {unknown}; try --help"),
        }
    }
    Ok(options)
}

#[derive(Clone, Debug, Serialize)]
struct ModelInspection {
    model: String,
    program_fingerprint: String,
    schema: u32,
    bodies: usize,
    dof: usize,
    actuators: usize,
    actuator_resource_models: usize,
    total_mass_kg: f64,
    collision_shapes: usize,
    coordinates: Vec<String>,
}

fn fingerprint_hex(fingerprint: &[u8; 32]) -> String {
    fingerprint
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn run_scenario(
    model: &bonesaw_core::CompiledModel,
    ticks: usize,
    scenario: Scenario,
) -> Result<ScenarioReport> {
    let controller = Controller::new(model.clone(), ControllerConfig::default())?;
    let mut state = RobotState::zeros(model);
    seed_standing_pose(model, &mut state);
    let mut controller_state = ControllerState::new(state.clone(), 0);
    let mut controller_state_next = controller_state.clone();
    let mut scratch = ControllerScratch::new(model, 12);
    let config = ControllerConfig::default();
    let mut output_buffer = ControllerOutputBuffer::with_layout(
        model,
        config.control_horizon_ns,
        config.sample_period_ns,
    );
    let origins = frame_origins(model, &state)?;
    let mut squared_error = 0.0;
    let mut max_error: f64 = 0.0;
    let mut measured_targets = 0usize;
    let mut timings = Vec::with_capacity(ticks);
    let mut degraded = 0;
    let mut contingency = 0;
    let dt = 0.02;
    let mut controller_allocations = 0_u64;
    let mut controller_allocated_bytes = 0_u64;
    let mut controller_input = ControllerInput {
        tick_time_ns: 0,
        state: state.clone(),
        signal_inputs: Default::default(),
        frame_targets: Vec::new(),
        com_target_world: None,
        posture_target: Some(standing_posture(model)),
        hard_constraints: Vec::new(),
    };

    for tick in 0..ticks {
        let time = tick as f64 * dt;
        let (targets, com_target) = scenario_targets(model, &origins, scenario, time)?;
        let target_positions: Vec<_> = targets
            .iter()
            .map(|target| (target.frame, target.target_position_world))
            .collect();
        controller_input.tick_time_ns = tick as i64 * 20_000_000;
        controller_input.state.clone_from(&state);
        controller_input.frame_targets = targets;
        controller_input.com_target_world = com_target;
        let allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let start = Instant::now();
        controller.advance_into_reusable(
            &controller_input,
            &controller_state,
            &mut controller_state_next,
            &mut output_buffer,
            &mut scratch,
        )?;
        let output = output_buffer
            .value
            .as_ref()
            .context("controller accepted tick without producing output")?;
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        controller_allocations += ALLOCATION_CALLS.load(Ordering::Relaxed) - allocations_before;
        controller_allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        match output.status {
            StepStatus::Degraded => degraded += 1,
            StepStatus::Contingency => contingency += 1,
            _ => {}
        }
        state.clone_from(&output.next_state);
        std::mem::swap(&mut controller_state, &mut controller_state_next);
        let current = frame_origins(model, &state)?;
        for (frame, target) in target_positions {
            let error = (current[frame.0] - target).norm();
            squared_error += error * error;
            max_error = max_error.max(error);
            measured_targets += 1;
        }
    }

    let mut max_violation: f64 = 0.0;
    for joint in &model.joints {
        if let Some(index) = joint.coordinate {
            max_violation = max_violation
                .max((joint.limit.lower - state.q[index]).max(0.0))
                .max((state.q[index] - joint.limit.upper).max(0.0));
        }
    }
    timings.sort_by(f64::total_cmp);
    Ok(ScenarioReport {
        name: match scenario {
            Scenario::Reach => "end_effector_reach",
            Scenario::BimanualConflict => "bimanual_priority_conflict",
            Scenario::WalkRetarget => "walking_motion_retarget",
        },
        ticks,
        rms_position_error_m: (squared_error / measured_targets.max(1) as f64).sqrt(),
        max_position_error_m: max_error,
        max_joint_limit_violation: max_violation,
        degraded_ticks: degraded,
        contingency_ticks: contingency,
        mean_tick_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_tick_us: percentile(&timings, 0.50),
        p99_tick_us: percentile(&timings, 0.99),
        max_tick_us: *timings.last().unwrap_or(&0.0),
        allocations_per_tick: controller_allocations as f64 / ticks as f64,
        allocated_bytes_per_tick: controller_allocated_bytes as f64 / ticks as f64,
    })
}

fn scenario_targets(
    model: &bonesaw_core::CompiledModel,
    origins: &[Vec3],
    scenario: Scenario,
    time: f64,
) -> Result<(Vec<FrameTarget>, Option<Vec3>)> {
    let left_hand = first_named_frame(model, &["left_hand", "left_contact", "left_wheel_center"])?;
    let right_hand = first_named_frame(
        model,
        &["right_hand", "right_contact", "right_wheel_center"],
    )?;
    let left_foot = first_named_frame(model, &["left_foot", "left_contact", "left_wheel_center"])?;
    let right_foot = first_named_frame(
        model,
        &["right_foot", "right_contact", "right_wheel_center"],
    )?;
    let has_distinct_arms =
        model.frame_id("left_hand").is_some() && model.frame_id("right_hand").is_some();
    let mut targets = Vec::new();

    match scenario {
        Scenario::Reach => {
            let phase = 2.0 * std::f64::consts::PI * 0.25 * time;
            for (stable_id, frame, side) in [(10, left_hand, 1.0), (20, right_hand, -1.0)] {
                targets.push(point_target(
                    stable_id,
                    frame,
                    origins[frame.0]
                        + Vec3::new(
                            0.10 + 0.04 * phase.cos(),
                            side * (0.12 + 0.03 * phase.sin()),
                            0.12 + 0.03 * phase.sin(),
                        ),
                    Priority::Intent,
                    1.0,
                ));
            }
            Ok((targets, None))
        }
        Scenario::BimanualConflict => {
            // Both hands request the same point while CoM remains a higher-level
            // viability target. This exposes hierarchy leakage and singularity.
            let midpoint = 0.5 * (origins[left_hand.0] + origins[right_hand.0]);
            let shared = midpoint + Vec3::new(0.18, 0.0, 0.12);
            targets.push(point_target(10, left_hand, shared, Priority::Intent, 1.0));
            targets.push(point_target(20, right_hand, shared, Priority::Intent, 1.0));
            let com = model_com(model, &RobotState::zeros(model))?;
            Ok((targets, Some(com)))
        }
        Scenario::WalkRetarget => {
            let phase = 2.0 * std::f64::consts::PI * time;
            for (stable_id, frame, offset) in
                [(30, left_foot, 0.0), (40, right_foot, std::f64::consts::PI)]
            {
                let local = phase + offset;
                let lift = 0.055 * local.sin().max(0.0);
                let stride = 0.075 * local.sin();
                targets.push(point_target(
                    stable_id,
                    frame,
                    origins[frame.0] + Vec3::new(stride, 0.0, lift),
                    Priority::Intent,
                    1.0,
                ));
            }
            if has_distinct_arms {
                // Arm swing is retargeted at lower weight than the feet when
                // the model actually has distinct arm end effectors.
                targets.push(point_target(
                    50,
                    left_hand,
                    origins[left_hand.0] + Vec3::new(-0.06 * phase.sin(), 0.0, 0.0),
                    Priority::Preference,
                    0.35,
                ));
                targets.push(point_target(
                    60,
                    right_hand,
                    origins[right_hand.0] + Vec3::new(0.06 * phase.sin(), 0.0, 0.0),
                    Priority::Preference,
                    0.35,
                ));
            }
            Ok((targets, None))
        }
    }
}

fn point_target(
    stable_id: u32,
    frame: FrameId,
    target_position_world: Vec3,
    priority: Priority,
    weight: f64,
) -> FrameTarget {
    FrameTarget {
        stable_id,
        frame,
        point_in_frame: Vec3::zeros(),
        target_position_world,
        target_orientation_world: None,
        feedforward_linear_velocity_world: Vec3::zeros(),
        priority,
        weight,
    }
}

fn determinism_eval(
    model: &bonesaw_core::CompiledModel,
    ticks: usize,
) -> Result<DeterminismReport> {
    let first = replay_velocities(model, ticks)?;
    let second = replay_velocities(model, ticks)?;
    let mismatch = first.iter().zip(second.iter()).position(|(a, b)| a != b);
    Ok(DeterminismReport {
        ticks,
        bitwise_equal: mismatch.is_none() && first.len() == second.len(),
        first_mismatch_tick: mismatch,
    })
}

fn replay_velocities(model: &bonesaw_core::CompiledModel, ticks: usize) -> Result<Vec<Vec<u64>>> {
    let controller = Controller::new(model.clone(), ControllerConfig::default())?;
    let mut state = RobotState::zeros(model);
    seed_standing_pose(model, &mut state);
    let mut controller_state = ControllerState::new(state.clone(), 0);
    let mut controller_state_next = controller_state.clone();
    let mut scratch = ControllerScratch::new(model, 12);
    let mut output_buffer = ControllerOutputBuffer::default();
    let origins = frame_origins(model, &state)?;
    let mut result = Vec::with_capacity(ticks);
    for tick in 0..ticks {
        let (targets, com) =
            scenario_targets(model, &origins, Scenario::WalkRetarget, tick as f64 * 0.02)?;
        controller.advance_into(
            ControllerInput {
                tick_time_ns: tick as i64 * 20_000_000,
                state,
                signal_inputs: Default::default(),
                frame_targets: targets,
                com_target_world: com,
                posture_target: Some(standing_posture(model)),
                hard_constraints: vec![],
            },
            &controller_state,
            &mut controller_state_next,
            &mut output_buffer,
            &mut scratch,
        )?;
        let output = output_buffer
            .take()
            .context("controller accepted replay tick without output")?;
        result.push(
            output
                .commanded_velocity
                .iter()
                .map(|value| value.to_bits())
                .collect(),
        );
        state = output.next_state;
        std::mem::swap(&mut controller_state, &mut controller_state_next);
    }
    Ok(result)
}

fn query_benchmark(model: &bonesaw_core::CompiledModel, queries: usize) -> Result<QueryReport> {
    let mut state = RobotState::zeros(model);
    seed_standing_pose(model, &mut state);
    let mut cache = ModelCache::new(model);
    model.forward_kinematics(&state, &mut cache)?;
    let root = FrameId(model.root.0);
    let start = Instant::now();
    for index in 0..queries {
        let to = FrameId(index % model.bodies.len());
        black_box(model.query_frame(&state, &cache, FrameQuery { from: root, to })?);
    }
    let elapsed = start.elapsed().as_secs_f64();
    Ok(QueryReport {
        queries,
        elapsed_ms: elapsed * 1e3,
        queries_per_second: queries as f64 / elapsed,
    })
}

fn historical_query_benchmark(
    model: &bonesaw_core::CompiledModel,
    queries: usize,
) -> Result<QueryReport> {
    const SAMPLES: usize = 64;
    const SAMPLE_PERIOD_NS: i64 = 20_000_000;

    let atlas = CompiledFrameAtlas::standard(model);
    let mut robot_history = RobotHistory::new(SAMPLES);
    let mut external_histories =
        ExternalFrameHistories::new((0..atlas.external_slot_count).map(|_| SAMPLES));
    for sample in 0..SAMPLES {
        let time_ns = sample as i64 * SAMPLE_PERIOD_NS;
        let mut state = RobotState::zeros(model);
        seed_standing_pose(model, &mut state);
        state.control_world_from_root = Transform3::translation(sample as f64 * 0.001, 0.0, 0.0);
        robot_history.push(TimedRobotState {
            time_ns,
            sequence: 1,
            state,
        });
        for slot in 0..atlas.external_slot_count {
            external_histories
                .slot_mut(slot)
                .expect("atlas and history slot counts match")
                .push(ExternalFrameSample {
                    time_ns,
                    anchor_from_frame: Transform3::identity(),
                    twist: None,
                    acceleration: None,
                    covariance: None,
                    sequence: 1,
                });
        }
    }

    let mut model_cache = ModelCache::new(model);
    let mut external_inputs = ExternalFrameInputs::new(&atlas);
    let mut snapshot = FrameAtlasSnapshot::new(&atlas);
    let to = *atlas
        .body_frames
        .last()
        .context("frame atlas has no robot bodies")?;
    let start = Instant::now();
    for index in 0..queries {
        let interval = index % (SAMPLES - 1);
        let time_ns = interval as i64 * SAMPLE_PERIOD_NS + SAMPLE_PERIOD_NS / 2;
        black_box(atlas.query_history(
            model,
            &robot_history,
            &external_histories,
            HistoricalFrameQuery {
                from: atlas.map,
                to,
                time_ns,
                policy: HistoryQueryPolicy::default(),
            },
            &mut model_cache,
            &mut external_inputs,
            &mut snapshot,
        )?);
    }
    let elapsed = start.elapsed().as_secs_f64();
    Ok(QueryReport {
        queries,
        elapsed_ms: elapsed * 1e3,
        queries_per_second: queries as f64 / elapsed,
    })
}

fn dynamics_eval(model: &bonesaw_core::CompiledModel, states: usize) -> Result<DynamicsReport> {
    let mut maximum_symmetry: f64 = 0.0;
    let mut minimum_eigenvalue = f64::INFINITY;
    let mut maximum_round_trip: f64 = 0.0;
    let mut timings = Vec::with_capacity(states);
    let gravity = Vec3::new(0.0, 0.0, -9.81);
    let mut model_cache = ModelCache::new(model);
    let mut dynamics_cache = DynamicsCache::new(model);

    for sample in 0..states {
        let mut state = RobotState::zeros(model);
        for joint in &model.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let center = if joint.limit.lower.is_finite() && joint.limit.upper.is_finite() {
                0.5 * (joint.limit.lower + joint.limit.upper)
            } else {
                0.0
            };
            let range = if joint.limit.lower.is_finite() && joint.limit.upper.is_finite() {
                0.2 * (joint.limit.upper - joint.limit.lower)
            } else {
                0.2
            };
            state.q[index] = center + range * (sample as f64 * 0.37 + index as f64 * 0.61).sin();
            state.v[index] = 0.3 * (sample as f64 * 0.19 + index as f64 * 0.43).cos();
        }
        let acceleration = DVector::from_iterator(
            model.dof,
            (0..model.dof).map(|index| 0.4 * (sample as f64 * 0.23 + index as f64 * 0.31).sin()),
        );
        model.forward_kinematics(&state, &mut model_cache)?;
        let start = Instant::now();
        let mass = model.mass_matrix(&model_cache)?;
        let torque = model.inverse_dynamics(
            &state,
            &acceleration,
            gravity,
            &model_cache,
            &mut dynamics_cache,
        )?;
        let recovered =
            model.forward_dynamics(&state, &torque, gravity, &model_cache, &mut dynamics_cache)?;
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        maximum_symmetry = maximum_symmetry.max((&mass - mass.transpose()).norm());
        let eigenvalues = mass.symmetric_eigen().eigenvalues;
        minimum_eigenvalue =
            minimum_eigenvalue.min(eigenvalues.iter().copied().fold(f64::INFINITY, f64::min));
        maximum_round_trip = maximum_round_trip.max((&recovered - acceleration).norm());
    }
    timings.sort_by(f64::total_cmp);
    Ok(DynamicsReport {
        states,
        maximum_mass_symmetry_error: maximum_symmetry,
        minimum_mass_eigenvalue: minimum_eigenvalue,
        maximum_inverse_forward_error: maximum_round_trip,
        mean_round_trip_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p99_round_trip_us: percentile(&timings, 0.99),
    })
}

fn collision_eval(model: &bonesaw_core::CompiledModel, states: usize) -> Result<CollisionReport> {
    let collision = CompiledCollisionModel::compile(model);
    let mut minimum_signed_distance = f64::INFINITY;
    let mut minimum_pair = None;
    let mut maximum_jacobian_error: f64 = 0.0;
    let mut timings = Vec::with_capacity(states);
    let mut distance_samples = 0;
    let mut cache = ModelCache::new(model);
    let mut plus_cache = ModelCache::new(model);
    let mut minus_cache = ModelCache::new(model);
    let zero_velocity = DVector::zeros(model.dof);
    let epsilon = 1e-7;

    for sample_index in 0..states {
        let mut state = RobotState::zeros(model);
        seed_standing_pose(model, &mut state);
        for joint in &model.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let perturbation = 0.05 * (sample_index as f64 * 0.31 + index as f64 * 0.47).sin();
            state.q[index] =
                (state.q[index] + perturbation).clamp(joint.limit.lower, joint.limit.upper);
        }
        model.forward_kinematics(&state, &mut cache)?;
        let start = Instant::now();
        let mut nearest = None;
        for pair in &collision.self_pairs {
            let distance = collision.evaluate_pair(model, &cache, &zero_velocity, *pair)?;
            if distance.signed_distance < minimum_signed_distance {
                minimum_signed_distance = distance.signed_distance;
                minimum_pair = Some(*pair);
            }
            distance_samples += 1;
            if nearest
                .as_ref()
                .is_none_or(|current: &bonesaw_core::DistanceSample| {
                    distance.signed_distance < current.signed_distance
                })
            {
                nearest = Some(distance);
            }
        }
        timings.push(start.elapsed().as_secs_f64() * 1e6);

        if let Some(nearest) = nearest {
            let pair = collision.self_pairs[nearest.pair_id as usize];
            for coordinate in 0..model.dof {
                let mut plus = state.clone();
                let mut minus = state.clone();
                plus.q[coordinate] += epsilon;
                minus.q[coordinate] -= epsilon;
                model.forward_kinematics(&plus, &mut plus_cache)?;
                model.forward_kinematics(&minus, &mut minus_cache)?;
                let plus_distance = collision
                    .evaluate_pair(model, &plus_cache, &zero_velocity, pair)?
                    .signed_distance;
                let minus_distance = collision
                    .evaluate_pair(model, &minus_cache, &zero_velocity, pair)?
                    .signed_distance;
                let finite_difference = (plus_distance - minus_distance) / (2.0 * epsilon);
                maximum_jacobian_error = maximum_jacobian_error
                    .max((finite_difference - nearest.jacobian_row[coordinate]).abs());
            }
        }
    }
    timings.sort_by(f64::total_cmp);
    let (controller_allocations, controller_allocated_bytes) =
        collision_controller_allocation_eval(model, states.max(20))?;
    Ok(CollisionReport {
        sphere_proxies: collision.spheres.len(),
        self_pairs: collision.self_pairs.len(),
        virtual_bridge_exclusions: collision.virtual_bridge_exclusion_count,
        distance_samples,
        minimum_signed_distance_m: if distance_samples == 0 {
            f64::NAN
        } else {
            minimum_signed_distance
        },
        minimum_pair_id: minimum_pair.map(|pair| pair.stable_id),
        minimum_pair_bodies: minimum_pair.map(|pair| {
            [
                model.bodies[collision.spheres[pair.a].body.0].name.clone(),
                model.bodies[collision.spheres[pair.b].body.0].name.clone(),
            ]
        }),
        maximum_jacobian_finite_difference_error: maximum_jacobian_error,
        p99_full_scan_us: percentile(&timings, 0.99),
        controller_allocations_per_tick: controller_allocations,
        controller_allocated_bytes_per_tick: controller_allocated_bytes,
    })
}

fn collision_controller_allocation_eval(
    model: &bonesaw_core::CompiledModel,
    ticks: usize,
) -> Result<(f64, f64)> {
    let config = ControllerConfig {
        collision_avoidance: Some(CollisionAvoidanceConfig::default()),
        ..ControllerConfig::default()
    };
    let controller = Controller::new(model.clone(), config.clone())?;
    let mut state = RobotState::zeros(model);
    seed_standing_pose(model, &mut state);
    let mut controller_state = ControllerState::new(state.clone(), 0);
    let mut controller_state_next = controller_state.clone();
    let mut scratch = ControllerScratch::new(model, 1);
    let mut output_buffer = ControllerOutputBuffer::with_layout(
        model,
        config.control_horizon_ns,
        config.sample_period_ns,
    );
    let mut input = ControllerInput {
        tick_time_ns: 0,
        state: state.clone(),
        signal_inputs: Default::default(),
        frame_targets: Vec::new(),
        com_target_world: None,
        posture_target: None,
        hard_constraints: Vec::new(),
    };
    let mut calls = 0_u64;
    let mut bytes = 0_u64;
    for tick in 0..ticks {
        input.tick_time_ns = tick as i64 * config.control_horizon_ns;
        input.state.clone_from(&state);
        let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        controller.advance_into_reusable(
            &input,
            &controller_state,
            &mut controller_state_next,
            &mut output_buffer,
            &mut scratch,
        )?;
        calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
        bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        let output = output_buffer
            .value
            .as_ref()
            .context("collision controller produced no output")?;
        state.clone_from(&output.next_state);
        std::mem::swap(&mut controller_state, &mut controller_state_next);
    }
    Ok((calls as f64 / ticks as f64, bytes as f64 / ticks as f64))
}

fn constraint_eval(iterations: usize) -> Result<ConstraintReport> {
    let solver = HierarchicalSolver::default();
    let task = Task {
        stable_id: 1,
        kind: TaskKind::Velocity,
        priority: Priority::Intent,
        jacobian: DMatrix::from_row_slice(1, 2, &[1.0, 1.0]),
        target_velocity: DVector::from_vec(vec![2.0]),
        weight: 1.0,
    };
    let active = LinearConstraint {
        stable_id: 10,
        coefficients: RowDVector::from_row_slice(&[1.0, 0.0]),
        lower: f64::NEG_INFINITY,
        upper: 0.5,
    };
    let bounds = VelocityBounds::unbounded(2);
    let mut timings = Vec::with_capacity(iterations);
    let mut maximum_violation: f64 = 0.0;
    let mut maximum_residual: f64 = 0.0;
    for _ in 0..iterations {
        let start = Instant::now();
        let result = solver.solve_constrained(
            2,
            std::slice::from_ref(&task),
            &bounds,
            std::slice::from_ref(&active),
        );
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        maximum_violation = maximum_violation.max(result.diagnostics.maximum_constraint_violation);
        maximum_residual = maximum_residual.max(
            result
                .diagnostics
                .level_residuals
                .first()
                .map_or(f64::INFINITY, |level| level.l2),
        );
    }

    let contradictory = [
        LinearConstraint {
            stable_id: 20,
            coefficients: RowDVector::from_row_slice(&[1.0]),
            lower: 1.0,
            upper: f64::INFINITY,
        },
        LinearConstraint {
            stable_id: 21,
            coefficients: RowDVector::from_row_slice(&[1.0]),
            lower: f64::NEG_INFINITY,
            upper: 0.0,
        },
    ];
    let contradictory_problem_detected = solver
        .solve_constrained(1, &[], &VelocityBounds::unbounded(1), &contradictory)
        .diagnostics
        .status
        == SolveStatus::PrimalInfeasible;
    timings.sort_by(f64::total_cmp);
    Ok(ConstraintReport {
        maximum_feasible_violation: maximum_violation,
        active_reoptimized_residual: maximum_residual,
        contradictory_problem_detected,
        p99_solve_us: percentile(&timings, 0.99),
    })
}

fn signal_graph_eval(ticks: usize) -> Result<SignalGraphReport> {
    let graph = CompiledSignalProgram::compile(
        vec![
            SignalOp::InputVector {
                stable_id: 1,
                input: 0,
            },
            SignalOp::ConstantVector {
                stable_id: 2,
                jet: VectorJet {
                    value: Vec3::new(0.15, -0.05, 0.2),
                    velocity: Vec3::zeros(),
                    acceleration: Vec3::zeros(),
                },
            },
            SignalOp::Add {
                stable_id: 3,
                left: 0,
                right: 1,
            },
            SignalOp::InputScalar {
                stable_id: 4,
                input: 0,
            },
            SignalOp::ConstantVector {
                stable_id: 5,
                jet: VectorJet {
                    value: Vec3::new(-0.2, 0.1, 0.35),
                    velocity: Vec3::zeros(),
                    acceleration: Vec3::zeros(),
                },
            },
            SignalOp::Blend {
                stable_id: 6,
                left: 2,
                right: 4,
                weight: 3,
            },
            SignalOp::Scale {
                stable_id: 7,
                source: 5,
                scale: 0.8,
            },
            SignalOp::LowPass {
                stable_id: 8,
                source: 6,
                bandwidth_hz: 6.0,
            },
            SignalOp::CriticallyDampedSpring {
                stable_id: 9,
                source: 7,
                bandwidth_hz: 3.0,
            },
            SignalOp::InputScalar {
                stable_id: 10,
                input: 1,
            },
            SignalOp::DeadbandScalar {
                stable_id: 11,
                source: 9,
                radius: 0.025,
            },
            SignalOp::ClampScalar {
                stable_id: 12,
                source: 10,
                lower: -0.5,
                upper: 0.5,
            },
            SignalOp::CriticallyDampedSpring {
                stable_id: 13,
                source: 11,
                bandwidth_hz: 4.0,
            },
        ],
        vec![
            SignalOutputSpec {
                stable_id: 100,
                node: 8,
            },
            SignalOutputSpec {
                stable_id: 101,
                node: 12,
            },
        ],
    )?;
    let mut input =
        SignalInputFrame::with_layout(graph.scalar_input_count(), graph.vector_input_count());
    let mut memory_a = SignalMemory::new(&graph);
    let mut memory_a_next = SignalMemory::new(&graph);
    let mut memory_b = SignalMemory::new(&graph);
    let mut memory_b_next = SignalMemory::new(&graph);
    let mut scratch_a = SignalScratch::new(&graph);
    let mut scratch_b = SignalScratch::new(&graph);
    let mut output_a = SignalOutputBuffer::new(&graph);
    let mut output_b = SignalOutputBuffer::new(&graph);
    let mut timings = Vec::with_capacity(ticks);
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    let mut bitwise_repeat = true;
    let mut maximum_output_acceleration_abs: f64 = 0.0;

    for tick in 0..ticks {
        let time = tick as f64 * 0.02;
        input.vectors[0] = VectorJet {
            value: Vec3::new(
                (0.7 * time).sin(),
                (0.4 * time).cos(),
                0.5 * (1.1 * time).sin(),
            ),
            velocity: Vec3::new(
                0.7 * (0.7 * time).cos(),
                -0.4 * (0.4 * time).sin(),
                0.55 * (1.1 * time).cos(),
            ),
            acceleration: Vec3::new(
                -0.49 * (0.7 * time).sin(),
                -0.16 * (0.4 * time).cos(),
                -0.605 * (1.1 * time).sin(),
            ),
        };
        input.scalars[0] = ScalarJet {
            value: 0.5 + 0.4 * (0.3 * time).sin(),
            velocity: 0.12 * (0.3 * time).cos(),
            acceleration: -0.036 * (0.3 * time).sin(),
        };
        input.scalars[1] = ScalarJet {
            value: 0.65 * (0.5 * time).sin(),
            velocity: 0.325 * (0.5 * time).cos(),
            acceleration: -0.1625 * (0.5 * time).sin(),
        };

        let allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let started = Instant::now();
        graph.evaluate_into(
            0.02,
            &input,
            &memory_a,
            &mut memory_a_next,
            &mut output_a,
            &mut scratch_a,
        )?;
        timings.push(started.elapsed().as_secs_f64() * 1e6);
        graph.evaluate_into(
            0.02,
            &input,
            &memory_b,
            &mut memory_b_next,
            &mut output_b,
            &mut scratch_b,
        )?;
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - allocations_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;

        bitwise_repeat &= output_a
            .values
            .iter()
            .zip(&output_b.values)
            .all(|(left, right)| signal_jet_bitwise_equal(*left, *right));
        bitwise_repeat &= memory_a_next.bitwise_eq(&memory_b_next);
        for output in &output_a.values {
            maximum_output_acceleration_abs = maximum_output_acceleration_abs.max(match output {
                SignalJet::Scalar(jet) => jet.acceleration.abs(),
                SignalJet::Vector(jet) => jet.acceleration.amax(),
                SignalJet::Rotation(jet) => jet.angular_acceleration_world.amax(),
            });
        }
        std::mem::swap(&mut memory_a, &mut memory_a_next);
        std::mem::swap(&mut memory_b, &mut memory_b_next);
    }

    timings.sort_by(f64::total_cmp);
    Ok(SignalGraphReport {
        ticks,
        nodes: graph.node_count(),
        outputs: graph.output_count(),
        memory_slots: graph.memory_slot_count(),
        bitwise_repeat,
        maximum_output_acceleration_abs,
        mean_step_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_step_us: percentile(&timings, 0.50),
        p99_step_us: percentile(&timings, 0.99),
        max_step_us: *timings.last().unwrap_or(&0.0),
        allocations_per_step: allocation_calls as f64 / (2 * ticks) as f64,
        allocated_bytes_per_step: allocated_bytes as f64 / (2 * ticks) as f64,
    })
}

fn compiled_rig_eval(base: &MotionProgram, ticks: usize) -> Result<CompiledRigReport> {
    let model = &base.model;
    let point_frame =
        first_named_frame(model, &["left_hand", "left_contact", "left_wheel", "torso"])?;
    let orientation_frame = first_named_frame(
        model,
        &["right_hand", "right_contact", "right_wheel", "torso"],
    )?;
    let signals = CompiledSignalProgram::compile(
        vec![
            SignalOp::InputVector {
                stable_id: 20_001,
                input: 0,
            },
            SignalOp::LowPass {
                stable_id: 20_002,
                source: 0,
                bandwidth_hz: 8.0,
            },
            SignalOp::InputVector {
                stable_id: 20_003,
                input: 1,
            },
            SignalOp::CriticallyDampedSpring {
                stable_id: 20_004,
                source: 2,
                bandwidth_hz: 6.0,
            },
            SignalOp::InputRotation {
                stable_id: 20_005,
                input: 0,
            },
            SignalOp::CriticallyDampedSpring {
                stable_id: 20_006,
                source: 4,
                bandwidth_hz: 5.0,
            },
        ],
        vec![
            SignalOutputSpec {
                stable_id: 21_001,
                node: 1,
            },
            SignalOutputSpec {
                stable_id: 21_002,
                node: 3,
            },
            SignalOutputSpec {
                stable_id: 21_003,
                node: 5,
            },
        ],
    )?;
    let with_signals = base.clone().with_signals(signals)?;
    let tasks = CompiledTaskProgram::compile(
        &with_signals.model,
        &with_signals.signals,
        vec![
            TaskSpec::Point {
                stable_id: 22_001,
                frame: point_frame,
                point_in_frame: Vec3::zeros(),
                target_signal: 21_001,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 2.5,
            },
            TaskSpec::CenterOfMass {
                stable_id: 22_002,
                target_signal: 21_002,
                priority: Priority::Viability,
                weight: 1.0,
                bandwidth_hz: 1.5,
            },
            TaskSpec::Orientation {
                stable_id: 22_003,
                frame: orientation_frame,
                target_signal: 21_003,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 2.0,
            },
        ],
    )?;
    let program = with_signals.with_tasks(tasks)?;
    let controller = Controller::from_program(&program)?;
    let mut initial = RobotState::zeros(model);
    seed_standing_pose(model, &mut initial);
    let origins = frame_origins(model, &initial)?;
    let point_origin = origins[point_frame.0];
    let com_origin = model_com(model, &initial)?;
    let mut initial_cache = ModelCache::new(model);
    model.forward_kinematics(&initial, &mut initial_cache)?;
    let orientation_origin = initial_cache.world_from_body[orientation_frame.0].rotation;
    let standing = initial.q.clone();
    let mut state_a = ControllerState::for_program(initial.clone(), &program);
    let mut state_a_next = state_a.clone();
    let mut state_b = state_a.clone();
    let mut state_b_next = state_a.clone();
    let mut scratch_a = ControllerScratch::for_program(&program, 0);
    let mut scratch_b = ControllerScratch::for_program(&program, 0);
    let mut output_a = ControllerOutputBuffer::with_layout(
        model,
        program.timing.control_horizon_ns,
        program.timing.sample_period_ns,
    );
    let mut output_b = ControllerOutputBuffer::with_layout(
        model,
        program.timing.control_horizon_ns,
        program.timing.sample_period_ns,
    );
    let mut input = ControllerInput {
        tick_time_ns: 0,
        state: initial,
        signal_inputs: SignalInputFrame::with_full_layout(0, 2, 1),
        frame_targets: Vec::new(),
        com_target_world: None,
        posture_target: Some(standing),
        hard_constraints: Vec::new(),
    };
    let mut timings = Vec::with_capacity(ticks);
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    let mut squared_tracking_error = 0.0;
    let mut squared_orientation_error = 0.0;
    let mut bitwise_repeat = true;

    for tick in 0..ticks {
        let time = tick as f64 * 0.02;
        input.tick_time_ns = tick as i64 * 20_000_000;
        input.signal_inputs.vectors[0] = VectorJet {
            value: point_origin
                + Vec3::new(
                    0.025 * (0.7 * time).sin(),
                    0.020 * (0.5 * time).sin(),
                    0.015 * (0.9 * time).sin(),
                ),
            velocity: Vec3::new(
                0.0175 * (0.7 * time).cos(),
                0.0100 * (0.5 * time).cos(),
                0.0135 * (0.9 * time).cos(),
            ),
            acceleration: Vec3::new(
                -0.01225 * (0.7 * time).sin(),
                -0.00500 * (0.5 * time).sin(),
                -0.01215 * (0.9 * time).sin(),
            ),
        };
        input.signal_inputs.vectors[1] = VectorJet {
            value: com_origin
                + Vec3::new(
                    0.010 * (0.4 * time).sin(),
                    0.005 * (0.6 * time).sin(),
                    0.006 * (0.3 * time).sin(),
                ),
            velocity: Vec3::new(
                0.004 * (0.4 * time).cos(),
                0.003 * (0.6 * time).cos(),
                0.0018 * (0.3 * time).cos(),
            ),
            acceleration: Vec3::new(
                -0.0016 * (0.4 * time).sin(),
                -0.0018 * (0.6 * time).sin(),
                -0.00054 * (0.3 * time).sin(),
            ),
        };
        let orientation_angle = 0.06 * (0.35 * time).sin();
        input.signal_inputs.rotations[0] = RotationJet {
            value: UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.0, orientation_angle))
                * orientation_origin,
            angular_velocity_world: Vec3::new(0.0, 0.0, 0.021 * (0.35 * time).cos()),
            angular_acceleration_world: Vec3::new(0.0, 0.0, -0.00735 * (0.35 * time).sin()),
        };

        let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let started = Instant::now();
        let status_a = controller.advance_into_reusable(
            &input,
            &state_a,
            &mut state_a_next,
            &mut output_a,
            &mut scratch_a,
        )?;
        timings.push(started.elapsed().as_secs_f64() * 1e6);
        let status_b = controller.advance_into_reusable(
            &input,
            &state_b,
            &mut state_b_next,
            &mut output_b,
            &mut scratch_b,
        )?;
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;

        let result_a = output_a
            .value
            .as_ref()
            .context("missing compiled-rig output")?;
        let result_b = output_b
            .value
            .as_ref()
            .context("missing repeated compiled-rig output")?;
        bitwise_repeat &= status_a == status_b
            && result_a
                .commanded_velocity
                .iter()
                .zip(result_b.commanded_velocity.iter())
                .all(|(left, right)| left.to_bits() == right.to_bits())
            && state_a_next
                .signal_memory
                .bitwise_eq(&state_b_next.signal_memory);
        let actual = Vec3::from_row_slice(&result_a.frames[point_frame.0].translation);
        squared_tracking_error += (actual - input.signal_inputs.vectors[0].value).norm_squared();
        let rotation_xyzw = result_a.frames[orientation_frame.0].rotation_xyzw;
        let actual_rotation = UnitQuaternion::new_normalize(Quaternion::new(
            rotation_xyzw[3],
            rotation_xyzw[0],
            rotation_xyzw[1],
            rotation_xyzw[2],
        ));
        squared_orientation_error += actual_rotation
            .rotation_to(&input.signal_inputs.rotations[0].value)
            .angle()
            .powi(2);
        input.state.clone_from(&result_a.next_state);
        std::mem::swap(&mut state_a, &mut state_a_next);
        std::mem::swap(&mut state_b, &mut state_b_next);
    }

    timings.sort_by(f64::total_cmp);
    Ok(CompiledRigReport {
        ticks,
        signal_nodes: program.signals.node_count(),
        task_slots: program.tasks.three_row_slots(),
        tracking_rms_m: (squared_tracking_error / ticks as f64).sqrt(),
        orientation_tracking_rms_rad: (squared_orientation_error / ticks as f64).sqrt(),
        bitwise_repeat,
        mean_tick_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_tick_us: percentile(&timings, 0.50),
        p99_tick_us: percentile(&timings, 0.99),
        max_tick_us: *timings.last().unwrap_or(&0.0),
        allocations_per_tick: allocation_calls as f64 / (2 * ticks) as f64,
        allocated_bytes_per_tick: allocated_bytes as f64 / (2 * ticks) as f64,
    })
}

fn signal_jet_bitwise_equal(left: SignalJet, right: SignalJet) -> bool {
    match (left, right) {
        (SignalJet::Scalar(left), SignalJet::Scalar(right)) => {
            left.value.to_bits() == right.value.to_bits()
                && left.velocity.to_bits() == right.velocity.to_bits()
                && left.acceleration.to_bits() == right.acceleration.to_bits()
        }
        (SignalJet::Vector(left), SignalJet::Vector(right)) => (0..3).all(|axis| {
            left.value[axis].to_bits() == right.value[axis].to_bits()
                && left.velocity[axis].to_bits() == right.velocity[axis].to_bits()
                && left.acceleration[axis].to_bits() == right.acceleration[axis].to_bits()
        }),
        (SignalJet::Rotation(left), SignalJet::Rotation(right)) => {
            (0..4).all(|axis| {
                left.value.quaternion().coords[axis].to_bits()
                    == right.value.quaternion().coords[axis].to_bits()
            }) && (0..3).all(|axis| {
                left.angular_velocity_world[axis].to_bits()
                    == right.angular_velocity_world[axis].to_bits()
                    && left.angular_acceleration_world[axis].to_bits()
                        == right.angular_acceleration_world[axis].to_bits()
            })
        }
        _ => false,
    }
}

fn dynamic_wbc_eval(model: &bonesaw_core::CompiledModel, ticks: usize) -> Result<DynamicWbcReport> {
    let controller = DynamicWbc::new(model.clone(), DynamicWbcConfig::default())?;
    let mut state = RobotState::zeros(model);
    seed_standing_pose(model, &mut state);
    let total_weight = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
    let contact_names =
        if model.frame_id("left_foot").is_some() && model.frame_id("right_foot").is_some() {
            [
                ("left_foot", ContactMode::LockedPoint),
                ("right_foot", ContactMode::LockedPoint),
            ]
        } else if model.frame_id("left_ankle_roll_link").is_some()
            && model.frame_id("right_ankle_roll_link").is_some()
        {
            [
                ("left_ankle_roll_link", ContactMode::LockedPoint),
                ("right_ankle_roll_link", ContactMode::LockedPoint),
            ]
        } else {
            [
                ("left_wheel_center", ContactMode::RollingPoint),
                ("right_wheel_center", ContactMode::RollingPoint),
            ]
        };
    let mut contacts = [
        ContactSpec::horizontal(
            1,
            named_frame(model, contact_names[0].0)?,
            Vec3::zeros(),
            contact_names[0].1,
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
        ContactSpec::horizontal(
            2,
            named_frame(model, contact_names[1].0)?,
            Vec3::zeros(),
            contact_names[1].1,
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
    ];
    let acceleration_bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -50.0),
        upper: DVector::from_element(model.dof, 50.0),
    };
    let torque_bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -10_000.0),
        upper: DVector::from_element(model.dof, 10_000.0),
    };
    let mut desired_acceleration = DVector::zeros(model.dof);
    let maximum_constraints = model.dof + contacts.len() * 7;
    let mut scratch = DynamicWbcScratch::new(model, contacts.len());
    let mut output = DynamicWbcOutput::workspace(model.dof, contacts.len(), maximum_constraints);
    let mut timings = Vec::with_capacity(ticks);
    let mut maximum_dynamics_residual: f64 = 0.0;
    let mut maximum_contact_residual: f64 = 0.0;
    let mut minimum_friction_margin = f64::INFINITY;
    let mut minimum_torque_margin = f64::INFINITY;
    let mut acceleration_tracking_squared = 0.0;
    let mut acceleration_tracking_samples = 0_usize;
    let mut maximum_acceleration_abs: f64 = 0.0;
    let mut maximum_torque_abs: f64 = 0.0;
    let mut maximum_contact_force_abs: f64 = 0.0;
    let mut friction_active_ticks = 0;
    let mut infeasible_ticks = 0;
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    for tick in 0..ticks {
        for coordinate in 0..model.dof {
            desired_acceleration[coordinate] =
                0.25 * (tick as f64 * 0.019 + coordinate as f64 * 0.37).sin();
        }
        contacts[0].nominal_tangent_x_force = 1.5
            * contacts[0].friction_coefficient
            * contacts[0].nominal_normal_force
            * (tick as f64 * 0.031).sin();
        let input = DynamicWbcInput {
            state: &state,
            desired_acceleration: &desired_acceleration,
            acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
        };
        let allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let start = Instant::now();
        controller.solve_into(input, &mut output, &mut scratch)?;
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - allocations_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        maximum_dynamics_residual = maximum_dynamics_residual.max(output.dynamics_residual_linf);
        maximum_contact_residual =
            maximum_contact_residual.max(output.contact_acceleration_residual_linf);
        minimum_friction_margin = minimum_friction_margin.min(output.minimum_friction_margin);
        minimum_torque_margin = minimum_torque_margin.min(output.minimum_torque_margin);
        for coordinate in 0..model.dof {
            let error =
                output.generalized_acceleration[coordinate] - desired_acceleration[coordinate];
            acceleration_tracking_squared += error * error;
            acceleration_tracking_samples += 1;
            maximum_acceleration_abs =
                maximum_acceleration_abs.max(output.generalized_acceleration[coordinate].abs());
            maximum_torque_abs = maximum_torque_abs.max(output.actuator_torque[coordinate].abs());
        }
        maximum_contact_force_abs =
            maximum_contact_force_abs.max(output.contact_force_basis.amax());
        if output
            .solve
            .active_constraints
            .iter()
            .any(|stable_id| stable_id & 0xf000_0000 == 0x3000_0000)
        {
            friction_active_ticks += 1;
        }
        if !matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ) {
            infeasible_ticks += 1;
        }
    }

    controller.solve_into(
        DynamicWbcInput {
            state: &state,
            desired_acceleration: &desired_acceleration,
            acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
        },
        &mut output,
        &mut scratch,
    )?;
    let first_acceleration = output.generalized_acceleration.clone();
    let first_torque = output.actuator_torque.clone();
    let first_forces = output.contact_force_basis.clone();
    controller.solve_into(
        DynamicWbcInput {
            state: &state,
            desired_acceleration: &desired_acceleration,
            acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
        },
        &mut output,
        &mut scratch,
    )?;
    let bitwise_repeat = first_acceleration
        .iter()
        .zip(output.generalized_acceleration.iter())
        .all(|(left, right)| left.to_bits() == right.to_bits())
        && first_torque
            .iter()
            .zip(output.actuator_torque.iter())
            .all(|(left, right)| left.to_bits() == right.to_bits())
        && first_forces
            .iter()
            .zip(output.contact_force_basis.iter())
            .all(|(left, right)| left.to_bits() == right.to_bits());

    timings.sort_by(f64::total_cmp);
    Ok(DynamicWbcReport {
        ticks,
        contacts: contacts.len(),
        decision_variables: model.dof * 2 + contacts.len() * 3,
        maximum_dynamics_residual,
        maximum_contact_acceleration_residual: maximum_contact_residual,
        minimum_friction_margin,
        minimum_torque_margin,
        acceleration_tracking_rms: (acceleration_tracking_squared
            / acceleration_tracking_samples.max(1) as f64)
            .sqrt(),
        maximum_acceleration_abs,
        maximum_torque_abs,
        maximum_contact_force_abs,
        friction_active_ticks,
        infeasible_ticks,
        bitwise_repeat,
        mean_tick_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_tick_us: percentile(&timings, 0.50),
        p99_tick_us: percentile(&timings, 0.99),
        max_tick_us: *timings.last().unwrap_or(&0.0),
        allocations_per_tick: allocation_calls as f64 / ticks as f64,
        allocated_bytes_per_tick: allocated_bytes as f64 / ticks as f64,
    })
}

fn floating_dynamic_wbc_eval(
    model: &bonesaw_core::CompiledModel,
    ticks: usize,
) -> Result<DynamicWbcReport> {
    let controller = FloatingDynamicWbc::new(model.clone(), DynamicWbcConfig::default())?;
    // Use the authored neutral posture for the point-contact sentinel. The
    // bent kinematic demo posture can place the toy humanoid's CoM outside the
    // pitch-support line; representing a finite foot patch moment requires the
    // future six-dimensional contact-wrench profile.
    let state = RobotState::zeros(model);
    let generalized_dof = model.dof + 6;
    let total_weight = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
    let contact_names =
        if model.frame_id("left_foot").is_some() && model.frame_id("right_foot").is_some() {
            [
                ("left_foot", ContactMode::LockedPoint),
                ("right_foot", ContactMode::LockedPoint),
            ]
        } else if model.frame_id("left_ankle_roll_link").is_some()
            && model.frame_id("right_ankle_roll_link").is_some()
        {
            [
                ("left_ankle_roll_link", ContactMode::LockedPoint),
                ("right_ankle_roll_link", ContactMode::LockedPoint),
            ]
        } else {
            [
                ("left_wheel_center", ContactMode::RollingPoint),
                ("right_wheel_center", ContactMode::RollingPoint),
            ]
        };
    let mut contacts = [
        ContactSpec::horizontal(
            1,
            named_frame(model, contact_names[0].0)?,
            Vec3::zeros(),
            contact_names[0].1,
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
        ContactSpec::horizontal(
            2,
            named_frame(model, contact_names[1].0)?,
            Vec3::zeros(),
            contact_names[1].1,
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
    ];
    // Keep one Cartesian task active in the allocation sentinel. A zero target
    // on a locked/rolling contact is intentionally redundant: it exercises
    // point Jacobian, bias-acceleration, task assembly, and diagnostics without
    // changing the nominal static-balance feasibility problem.
    let point_tasks = [FloatingPointAccelerationTask {
        stable_id: 9,
        frame: contacts[0].frame,
        point_in_frame: Vec3::zeros(),
        desired_acceleration_world: Vec3::zeros(),
        priority: Priority::Preference,
        weight: 0.25,
    }];
    let acceleration_bounds = VelocityBounds {
        lower: DVector::from_element(generalized_dof, -100.0),
        upper: DVector::from_element(generalized_dof, 100.0),
    };
    let torque_bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -10_000.0),
        upper: DVector::from_element(model.dof, 10_000.0),
    };
    let mut desired_acceleration = DVector::zeros(generalized_dof);
    let maximum_constraints = generalized_dof + contacts.len() * 8;
    let mut scratch = FloatingDynamicWbcScratch::new(model, contacts.len());
    let mut output =
        FloatingDynamicWbcOutput::workspace(model.dof, contacts.len(), maximum_constraints);
    let mut timings = Vec::with_capacity(ticks);
    let mut maximum_dynamics_residual: f64 = 0.0;
    let mut maximum_contact_residual: f64 = 0.0;
    let mut minimum_friction_margin = f64::INFINITY;
    let mut minimum_torque_margin = f64::INFINITY;
    let mut acceleration_tracking_squared = 0.0;
    let mut acceleration_tracking_samples = 0_usize;
    let mut maximum_acceleration_abs: f64 = 0.0;
    let mut maximum_torque_abs: f64 = 0.0;
    let mut maximum_contact_force_abs: f64 = 0.0;
    let mut friction_active_ticks = 0;
    let mut infeasible_ticks = 0;
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    for tick in 0..ticks {
        desired_acceleration.fill(0.0);
        contacts[0].nominal_tangent_x_force = 1.5
            * contacts[0].friction_coefficient
            * contacts[0].nominal_normal_force
            * (tick as f64 * 0.031).sin();
        let input = FloatingDynamicWbcInput {
            state: &state,
            root_twist_world: Motion6::default(),
            desired_generalized_acceleration: &desired_acceleration,
            task_priorities: FloatingTaskPriorities::default(),
            task_weights: Default::default(),
            joint_posture_weight: 1.0,
            joint_acceleration_task: None,
            center_of_mass_task: None,
            centroidal_angular_momentum_task: None,
            frame_angular_acceleration_tasks: &[],
            point_acceleration_tasks: &point_tasks,
            generalized_acceleration_bounds: &acceleration_bounds,
            torque_bounds: &torque_bounds,
            actuator_effort: None,
            contacts: &contacts,
            support_patches: &[],
        };
        let allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let start = Instant::now();
        controller.solve_into(input, &mut output, &mut scratch)?;
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - allocations_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        maximum_dynamics_residual = maximum_dynamics_residual.max(output.dynamics_residual_linf);
        maximum_contact_residual =
            maximum_contact_residual.max(output.contact_acceleration_residual_linf);
        minimum_friction_margin = minimum_friction_margin.min(output.minimum_friction_margin);
        minimum_torque_margin = minimum_torque_margin.min(output.minimum_torque_margin);
        for coordinate in 0..generalized_dof {
            let error =
                output.generalized_acceleration[coordinate] - desired_acceleration[coordinate];
            acceleration_tracking_squared += error * error;
            acceleration_tracking_samples += 1;
            maximum_acceleration_abs =
                maximum_acceleration_abs.max(output.generalized_acceleration[coordinate].abs());
        }
        for coordinate in 0..model.dof {
            maximum_torque_abs = maximum_torque_abs.max(output.actuator_torque[coordinate].abs());
        }
        maximum_contact_force_abs =
            maximum_contact_force_abs.max(output.contact_force_basis.amax());
        if output
            .solve
            .active_constraints
            .iter()
            .any(|stable_id| stable_id & 0xf000_0000 == 0x3000_0000)
        {
            friction_active_ticks += 1;
        }
        if !matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ) {
            infeasible_ticks += 1;
        }
    }

    let repeat_input = FloatingDynamicWbcInput {
        state: &state,
        root_twist_world: Motion6::default(),
        desired_generalized_acceleration: &desired_acceleration,
        task_priorities: FloatingTaskPriorities::default(),
        task_weights: Default::default(),
        joint_posture_weight: 1.0,
        joint_acceleration_task: None,
        center_of_mass_task: None,
        centroidal_angular_momentum_task: None,
        frame_angular_acceleration_tasks: &[],
        point_acceleration_tasks: &point_tasks,
        generalized_acceleration_bounds: &acceleration_bounds,
        torque_bounds: &torque_bounds,
        actuator_effort: None,
        contacts: &contacts,
        support_patches: &[],
    };
    controller.solve_into(repeat_input, &mut output, &mut scratch)?;
    let first_acceleration = output.generalized_acceleration.clone();
    let first_torque = output.actuator_torque.clone();
    let first_forces = output.contact_force_basis.clone();
    controller.solve_into(repeat_input, &mut output, &mut scratch)?;
    let bitwise_repeat = first_acceleration
        .iter()
        .zip(output.generalized_acceleration.iter())
        .all(|(left, right)| left.to_bits() == right.to_bits())
        && first_torque
            .iter()
            .zip(output.actuator_torque.iter())
            .all(|(left, right)| left.to_bits() == right.to_bits())
        && first_forces
            .iter()
            .zip(output.contact_force_basis.iter())
            .all(|(left, right)| left.to_bits() == right.to_bits());

    timings.sort_by(f64::total_cmp);
    Ok(DynamicWbcReport {
        ticks,
        contacts: contacts.len(),
        decision_variables: generalized_dof + model.dof + contacts.len() * 3,
        maximum_dynamics_residual,
        maximum_contact_acceleration_residual: maximum_contact_residual,
        minimum_friction_margin,
        minimum_torque_margin,
        acceleration_tracking_rms: (acceleration_tracking_squared
            / acceleration_tracking_samples.max(1) as f64)
            .sqrt(),
        maximum_acceleration_abs,
        maximum_torque_abs,
        maximum_contact_force_abs,
        friction_active_ticks,
        infeasible_ticks,
        bitwise_repeat,
        mean_tick_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_tick_us: percentile(&timings, 0.50),
        p99_tick_us: percentile(&timings, 0.99),
        max_tick_us: *timings.last().unwrap_or(&0.0),
        allocations_per_tick: allocation_calls as f64 / ticks as f64,
        allocated_bytes_per_tick: allocated_bytes as f64 / ticks as f64,
    })
}

fn floating_squat_eval(
    model: &bonesaw_core::CompiledModel,
    ticks: usize,
    project_state: bool,
) -> Result<FloatingSquatReport> {
    const DT: f64 = 0.005;
    const LOWERING_DURATION: f64 = 2.0;
    const LOWERING: f64 = 0.12;
    let controller = FloatingDynamicWbc::new(
        model.clone(),
        DynamicWbcConfig {
            acceleration_weight: 8.0,
            contact_force_weight: 0.0,
            actuator_torque_weight: 0.0,
            ..DynamicWbcConfig::default()
        },
    )?;
    let mut state = FloatingRobotState::zeros(model);
    state.robot.q = standing_posture(model);
    let standing = state.robot.q.clone();
    let standing_root_z = state.robot.control_world_from_root.translation.vector.z;
    let support_frames = [
        ("left_wheel_center", "right_wheel_center"),
        ("left_contact", "right_contact"),
        ("left_foot", "right_foot"),
    ]
    .into_iter()
    .find_map(|(left, right)| Some((model.frame_id(left)?, model.frame_id(right)?)))
    .context("floating squat requires a left/right support-frame pair")?;
    let mut cache = ModelCache::new(model);
    model.forward_kinematics(&state.robot, &mut cache)?;
    let support_targets = [
        cache.world_from_body[support_frames.0.0]
            .transform_point(&Point3::origin())
            .coords,
        cache.world_from_body[support_frames.1.0]
            .transform_point(&Point3::origin())
            .coords,
    ];
    let planar_ik_targets = [
        PlanarPointIkTarget {
            frame: support_frames.0,
            point_in_frame: Vec3::zeros(),
            target_world: support_targets[0],
            coordinates: [
                model
                    .joint_id("left_hip")
                    .and_then(|joint| model.joints[joint.0].coordinate)
                    .context("Upkie left hip coordinate is required")?,
                model
                    .joint_id("left_knee")
                    .and_then(|joint| model.joints[joint.0].coordinate)
                    .context("Upkie left knee coordinate is required")?,
            ],
            axes: [0, 2],
        },
        PlanarPointIkTarget {
            frame: support_frames.1,
            point_in_frame: Vec3::zeros(),
            target_world: support_targets[1],
            coordinates: [
                model
                    .joint_id("right_hip")
                    .and_then(|joint| model.joints[joint.0].coordinate)
                    .context("Upkie right hip coordinate is required")?,
                model
                    .joint_id("right_knee")
                    .and_then(|joint| model.joints[joint.0].coordinate)
                    .context("Upkie right knee coordinate is required")?,
            ],
            axes: [0, 2],
        },
    ];
    let mut planar_ik_scratch = PlanarIkScratch::new(model);
    let mut squat_target = state.robot.clone();
    if !project_state {
        let support_center_x = 0.5 * (support_targets[0].x + support_targets[1].x);
        squat_target.control_world_from_root.rotation = Default::default();
        squat_target.control_world_from_root.translation.vector.x = 0.0;
        squat_target.control_world_from_root.translation.vector.y = 0.0;
        squat_target.control_world_from_root.translation.vector.z = standing_root_z;
        squat_target.q.copy_from(&standing);
        solve_planar_point_ik_into(
            model,
            &mut squat_target,
            &planar_ik_targets,
            PlanarIkOptions::default(),
            &mut planar_ik_scratch,
        )?;
        for _ in 0..8 {
            model.forward_kinematics(&squat_target, &mut cache)?;
            let balance_error = cache.center_of_mass_world.x - support_center_x;
            if balance_error.abs() <= 1e-9 {
                break;
            }
            squat_target.control_world_from_root.translation.vector.x -= balance_error;
            solve_planar_point_ik_into(
                model,
                &mut squat_target,
                &planar_ik_targets,
                PlanarIkOptions::default(),
                &mut planar_ik_scratch,
            )?;
        }
        state.robot.clone_from(&squat_target);
    }
    let wheel_balancer = UpkieWheelBalancer::compile(model, &state.robot)?;
    if !project_state
        && let Some(velocity) = env::var("BONESAW_SQUAT_PERTURB")
            .ok()
            .and_then(|value| value.parse().ok())
    {
        state.root_twist_world.0[3] = velocity;
        wheel_balancer.set_admissible_ground_velocity(velocity, state.robot.v.as_mut_slice());
    }
    let mut wheel_balancer_state = UpkieWheelBalancerState::default();
    let mut desired_wheel_accelerations = [0.0; 2];
    let target_ground_position = 0.5 * (support_targets[0].x + support_targets[1].x);
    let total_weight = model.bodies.iter().map(|body| body.mass).sum::<f64>() * 9.81;
    let mut contacts = [
        ContactSpec::horizontal(
            1,
            support_frames.0,
            Vec3::zeros(),
            wheel_balancer.left_contact_mode(),
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
        ContactSpec::horizontal(
            2,
            support_frames.1,
            Vec3::zeros(),
            wheel_balancer.right_contact_mode(),
            0.8,
            2.0 * total_weight,
            0.5 * total_weight,
        ),
    ];
    let generalized_dof = model.dof + 6;
    let acceleration_bounds = VelocityBounds {
        lower: DVector::from_element(generalized_dof, -200.0),
        upper: DVector::from_element(generalized_dof, 200.0),
    };
    let mut torque_bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -1_000.0),
        upper: DVector::from_element(model.dof, 1_000.0),
    };
    for joint in &model.joints {
        let Some(index) = joint.coordinate else {
            continue;
        };
        if joint.limit.effort.is_finite() {
            torque_bounds.lower[index] = -joint.limit.effort.abs();
            torque_bounds.upper[index] = joint.limit.effort.abs();
        }
    }
    let mut desired = DVector::zeros(generalized_dof);
    let mut generalized_velocity = DVector::zeros(generalized_dof);
    let mut jacobian = DMatrix::zeros(3, generalized_dof);
    let mut target_cache = ModelCache::new(model);
    let mut center_of_mass_dynamics = DynamicsCache::new(model);
    let mut center_of_mass_jacobian = DMatrix::zeros(3, generalized_dof);
    let (floating_signals, floating_tasks) =
        compile_floating_balance_policy(model, !project_state)?;
    let mut floating_signal_inputs = SignalInputFrame::with_full_layout(0, 2, 1);
    let mut floating_signal_memory = SignalMemory::new(&floating_signals);
    let mut floating_signal_memory_next = SignalMemory::new(&floating_signals);
    let mut floating_signal_scratch = SignalScratch::new(&floating_signals);
    let mut floating_signal_output = SignalOutputBuffer::new(&floating_signals);
    let mut floating_task_command = FloatingTaskCommand::default();
    let mut scratch = FloatingDynamicWbcScratch::new(model, contacts.len());
    let mut output =
        FloatingDynamicWbcOutput::workspace(model.dof, contacts.len(), generalized_dof + 16);
    let mut timings = Vec::with_capacity(ticks);
    let mut height_error_squared = 0.0;
    let mut horizontal_error_squared = 0.0;
    let mut center_of_mass_error_squared = 0.0;
    let mut maximum_center_of_mass_error: f64 = 0.0;
    let mut maximum_policy_oracle_acceleration_delta: f64 = 0.0;
    let mut maximum_contact_position_delta: f64 = 0.0;
    let mut maximum_constrained_contact_drift: f64 = 0.0;
    let mut maximum_permitted_rolling_travel: f64 = 0.0;
    let mut maximum_root_rotation: f64 = 0.0;
    let mut maximum_generalized_acceleration_abs: f64 = 0.0;
    let mut maximum_actuator_torque_abs: f64 = 0.0;
    let mut maximum_contact_force_abs: f64 = 0.0;
    let mut maximum_dynamics_residual: f64 = 0.0;
    let mut maximum_contact_acceleration_residual: f64 = 0.0;
    let mut minimum_friction_margin = f64::INFINITY;
    let mut minimum_torque_margin = f64::INFINITY;
    let mut degraded_ticks = 0;
    let mut infeasible_ticks = 0;
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    let trace_squat = env::var_os("BONESAW_TRACE_SQUAT").is_some();

    for tick in 0..ticks {
        let ramp = ((tick as f64 * DT) / LOWERING_DURATION).clamp(0.0, 1.0);
        let desired_root_z = standing_root_z - LOWERING * ramp;
        let allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let started = Instant::now();
        squat_target.clone_from(&state.robot);
        squat_target.control_world_from_root.rotation = Default::default();
        squat_target.control_world_from_root.translation.vector.x = 0.0;
        squat_target.control_world_from_root.translation.vector.y = 0.0;
        squat_target.control_world_from_root.translation.vector.z = desired_root_z;
        squat_target.q.copy_from(&standing);
        let mut ik_report = solve_planar_point_ik_into(
            model,
            &mut squat_target,
            &planar_ik_targets,
            PlanarIkOptions::default(),
            &mut planar_ik_scratch,
        )?;
        if !project_state {
            let support_center_x = 0.5 * (support_targets[0].x + support_targets[1].x);
            for _ in 0..6 {
                model.forward_kinematics(&squat_target, &mut target_cache)?;
                let balance_error = target_cache.center_of_mass_world.x - support_center_x;
                if balance_error.abs() <= 1e-9 {
                    break;
                }
                squat_target.control_world_from_root.translation.vector.x -= balance_error;
                ik_report = solve_planar_point_ik_into(
                    model,
                    &mut squat_target,
                    &planar_ik_targets,
                    PlanarIkOptions::default(),
                    &mut planar_ik_scratch,
                )?;
            }
        }
        if !ik_report.converged {
            bail!(
                "Upkie squat contact IK failed at tick {tick}: {:.3} mm",
                ik_report.maximum_planar_error_m * 1000.0
            );
        }
        model.forward_kinematics(&squat_target, &mut target_cache)?;
        for index in 0..model.dof {
            squat_target.v[index] = (squat_target.q[index] - state.robot.q[index]) / DT;
        }
        let mut squat_target_root_twist = Motion6::default();
        squat_target_root_twist.0[5] =
            (desired_root_z - state.robot.control_world_from_root.translation.vector.z) / DT;
        desired.fill(0.0);
        for index in 0..model.dof {
            desired[6 + index] = (24.0 * (squat_target.q[index] - state.robot.q[index])
                - 7.0 * state.robot.v[index])
                .clamp(-12.0, 12.0);
        }
        model.forward_kinematics(&state.robot, &mut cache)?;
        let center_of_mass_error = target_cache.center_of_mass_world - cache.center_of_mass_world;
        center_of_mass_error_squared += center_of_mass_error.norm_squared();
        maximum_center_of_mass_error =
            maximum_center_of_mass_error.max(center_of_mass_error.norm());
        for axis in 0..6 {
            generalized_velocity[axis] = state.root_twist_world.0[axis];
        }
        generalized_velocity
            .rows_mut(6, model.dof)
            .copy_from(&state.robot.v);
        model.floating_com_jacobian_into(
            &cache,
            &mut center_of_mass_dynamics,
            &mut center_of_mass_jacobian,
        )?;
        let mut center_of_mass_velocity = Vec3::zeros();
        for row in 0..3 {
            for column in 0..generalized_dof {
                center_of_mass_velocity[row] +=
                    center_of_mass_jacobian[(row, column)] * generalized_velocity[column];
            }
        }
        let target_pitch = 0.0;
        floating_signal_inputs.rotations[0] = RotationJet {
            value: UnitQuaternion::from_scaled_axis(Vec3::new(0.0, target_pitch, 0.0)),
            ..Default::default()
        };
        floating_signal_inputs.vectors[0] = VectorJet {
            value: squat_target.control_world_from_root.translation.vector,
            ..Default::default()
        };
        floating_signal_inputs.vectors[1] = VectorJet {
            value: target_cache.center_of_mass_world,
            ..Default::default()
        };
        floating_signals.evaluate_into(
            DT,
            &floating_signal_inputs,
            &floating_signal_memory,
            &mut floating_signal_memory_next,
            &mut floating_signal_output,
            &mut floating_signal_scratch,
        )?;
        floating_tasks.emit_floating_command_into(
            &floating_signal_output,
            FloatingTaskState {
                control_world_from_root: state.robot.control_world_from_root,
                root_twist_world: state.root_twist_world,
                center_of_mass_world: cache.center_of_mass_world,
                center_of_mass_velocity_world: center_of_mass_velocity,
            },
            &mut floating_task_command,
        )?;
        desired.as_mut_slice()[..6]
            .copy_from_slice(floating_task_command.root_acceleration_world.0.as_slice());
        let SignalJet::Rotation(filtered_root_rotation) = floating_signal_output.values[0] else {
            unreachable!("floating policy root-orientation output is a rotation")
        };
        let SignalJet::Vector(filtered_root_translation) = floating_signal_output.values[1] else {
            unreachable!("floating policy root-translation output is a vector")
        };
        let SignalJet::Vector(filtered_center_of_mass) = floating_signal_output.values[2] else {
            unreachable!("floating policy center-of-mass output is a vector")
        };
        let rotation_error = state
            .robot
            .control_world_from_root
            .rotation
            .transform_vector(
                &state
                    .robot
                    .control_world_from_root
                    .rotation
                    .rotation_to(&filtered_root_rotation.value)
                    .scaled_axis(),
            );
        let mut oracle_root_acceleration = Motion6::default();
        for axis in 0..3 {
            oracle_root_acceleration.0[axis] = (filtered_root_rotation.angular_acceleration_world
                [axis]
                + 20.0 * rotation_error[axis]
                + 8.0
                    * (filtered_root_rotation.angular_velocity_world[axis]
                        - state.root_twist_world.0[axis]))
                .clamp(-8.0, 8.0);
        }
        for axis in 0..2 {
            oracle_root_acceleration.0[3 + axis] = (filtered_root_translation.acceleration[axis]
                + 6.0
                    * (filtered_root_translation.value[axis]
                        - state.robot.control_world_from_root.translation.vector[axis])
                + 5.0
                    * (filtered_root_translation.velocity[axis]
                        - state.root_twist_world.0[3 + axis]))
                .clamp(-4.0, 4.0);
        }
        oracle_root_acceleration.0[5] = (filtered_root_translation.acceleration.z
            + 18.0
                * (filtered_root_translation.value.z
                    - state.robot.control_world_from_root.translation.vector.z)
            + 9.0 * (filtered_root_translation.velocity.z - state.root_twist_world.0[5]))
            .clamp(-4.0, 4.0);
        for axis in 0..6 {
            maximum_policy_oracle_acceleration_delta = maximum_policy_oracle_acceleration_delta
                .max(
                    (floating_task_command.root_acceleration_world.0[axis]
                        - oracle_root_acceleration.0[axis])
                        .abs(),
                );
        }
        if !project_state {
            let oracle_center_of_mass_acceleration = (filtered_center_of_mass.acceleration
                + 30.0 * (filtered_center_of_mass.value - cache.center_of_mass_world)
                + 10.0 * (filtered_center_of_mass.velocity - center_of_mass_velocity))
                .map(|value| value.clamp(-8.0, 8.0));
            let compiled_center_of_mass_acceleration = floating_task_command
                .center_of_mass_task
                .expect("wheeled policy compiles a CoM slot")
                .desired_acceleration_world;
            for axis in 0..3 {
                maximum_policy_oracle_acceleration_delta = maximum_policy_oracle_acceleration_delta
                    .max(
                        (compiled_center_of_mass_acceleration[axis]
                            - oracle_center_of_mass_acceleration[axis])
                            .abs(),
                    );
            }
        }
        for (contact, target) in contacts.iter_mut().zip(&support_targets) {
            model.floating_point_jacobian_into(
                &cache,
                contact.frame,
                contact.point_in_frame,
                &mut jacobian,
            )?;
            let point = cache.world_from_body[contact.frame.0]
                .transform_point(&Point3::from(contact.point_in_frame))
                .coords;
            let velocity = Vec3::from_fn(|row, _| {
                (0..generalized_dof)
                    .map(|column| jacobian[(row, column)] * generalized_velocity[column])
                    .sum()
            });
            contact.desired_point_acceleration_world =
                (40.0 * (*target - point) - 14.0 * velocity).map(|value| value.clamp(-8.0, 8.0));
            if matches!(contact.mode, ContactMode::RollingWheel { .. }) {
                let rolling_axis = contact.tangent_x_world;
                contact.desired_point_acceleration_world -=
                    rolling_axis * rolling_axis.dot(&contact.desired_point_acceleration_world);
            }
            let contact_delta = *target - point;
            maximum_contact_position_delta =
                maximum_contact_position_delta.max(contact_delta.norm());
            let (constrained_drift, permitted_travel) = match contact.mode {
                ContactMode::LockedPoint => (contact_delta.norm(), 0.0),
                ContactMode::NormalPoint => (
                    contact_delta.z.abs(),
                    Vec3::new(contact_delta.x, contact_delta.y, 0.0).norm(),
                ),
                ContactMode::RollingPoint | ContactMode::RollingWheel { .. } => (
                    contact_delta
                        .dot(&contact.tangent_y_world)
                        .hypot(contact_delta.dot(&contact.normal_world)),
                    contact_delta.dot(&contact.tangent_x_world).abs(),
                ),
            };
            maximum_constrained_contact_drift =
                maximum_constrained_contact_drift.max(constrained_drift);
            maximum_permitted_rolling_travel =
                maximum_permitted_rolling_travel.max(permitted_travel);
        }
        let ground_position = contacts
            .iter()
            .map(|contact| {
                cache.world_from_body[contact.frame.0]
                    .transform_point(&Point3::from(contact.point_in_frame))
                    .x
            })
            .sum::<f64>()
            / contacts.len() as f64;
        let ground_height = contacts
            .iter()
            .map(|contact| {
                cache.world_from_body[contact.frame.0]
                    .transform_point(&Point3::from(contact.point_in_frame))
                    .z
            })
            .sum::<f64>()
            / contacts.len() as f64;
        let virtual_pitch = (cache.center_of_mass_world.x - ground_position)
            .atan2((cache.center_of_mass_world.z - ground_height).max(0.05));
        wheel_balancer.emit_accelerations(
            DT,
            target_ground_position,
            ground_position,
            virtual_pitch,
            state.robot.v.as_slice(),
            &mut wheel_balancer_state,
            &mut desired_wheel_accelerations,
        );
        for coordinate in 0..model.dof {
            desired[6 + coordinate] = (30.0
                * (squat_target.q[coordinate] - state.robot.q[coordinate])
                - 10.0 * state.robot.v[coordinate])
                .clamp(-50.0, 50.0);
        }
        for row in 0..wheel_balancer.coordinates.len() {
            desired[6 + wheel_balancer.coordinates[row]] = desired_wheel_accelerations[row];
        }
        let mut task_priorities = floating_task_command.task_priorities;
        let mut task_weights = floating_task_command.task_weights;
        if !project_state {
            task_priorities.joint_posture = Priority::Intent;
            task_weights.joint_posture = 4.0;
        }
        controller.solve_into(
            FloatingDynamicWbcInput {
                state: &state.robot,
                root_twist_world: state.root_twist_world,
                desired_generalized_acceleration: &desired,
                task_priorities,
                task_weights,
                joint_posture_weight: 1.0,
                joint_acceleration_task: (!project_state).then_some(
                    FloatingJointAccelerationTask {
                        coordinates: &wheel_balancer.coordinates,
                        desired_accelerations: &desired_wheel_accelerations,
                        priority: Priority::Viability,
                        weight: 1.0,
                    },
                ),
                center_of_mass_task: floating_task_command.center_of_mass_task,
                centroidal_angular_momentum_task: None,
                frame_angular_acceleration_tasks: &[],
                point_acceleration_tasks: &[],
                generalized_acceleration_bounds: &acceleration_bounds,
                torque_bounds: &torque_bounds,
                actuator_effort: None,
                contacts: &contacts,
                support_patches: &[],
            },
            &mut output,
            &mut scratch,
        )?;
        std::mem::swap(
            &mut floating_signal_memory,
            &mut floating_signal_memory_next,
        );
        if trace_squat && (tick % 10 == 0 || output.status == SolveStatus::PrimalInfeasible) {
            eprintln!(
                "squat tick={tick} projected={project_state} status={:?} root_x={:.6} \
                 target_x={:.6} com_x={:.6} target_com_x={:.6} root_z={:.6} \
                 target_z={desired_root_z:.6} com_z={:.6} target_com_z={:.6} \
                 root_vz={:.6} root_az={:.6} contact_res={:.3e} dynamics_res={:.3e} \
                 friction_margin={:.6} torque_margin={:.6} rotation={:.6} \
                 pitch={:.6} ground_x={ground_position:.6} wheel_qdd={:?} \
                 q={:?} violation={:.3e}",
                output.status,
                state.robot.control_world_from_root.translation.vector.x,
                squat_target.control_world_from_root.translation.vector.x,
                cache.center_of_mass_world.x,
                target_cache.center_of_mass_world.x,
                state.robot.control_world_from_root.translation.vector.z,
                cache.center_of_mass_world.z,
                target_cache.center_of_mass_world.z,
                state.root_twist_world.0[5],
                output.generalized_acceleration[5],
                output.contact_acceleration_residual_linf,
                output.dynamics_residual_linf,
                output.minimum_friction_margin,
                output.minimum_torque_margin,
                state
                    .robot
                    .control_world_from_root
                    .rotation
                    .scaled_axis()
                    .norm(),
                state.robot.control_world_from_root.rotation.scaled_axis().y,
                desired_wheel_accelerations,
                state.robot.q.as_slice(),
                output.solve.maximum_constraint_violation,
            );
        }
        if matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ) {
            model.integrate_floating(&mut state, &output.generalized_acceleration, DT)?;
        }
        if project_state {
            state.robot.clone_from(&squat_target);
            state.root_twist_world = squat_target_root_twist;
        }
        timings.push(started.elapsed().as_secs_f64() * 1e6);
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - allocations_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        maximum_generalized_acceleration_abs =
            maximum_generalized_acceleration_abs.max(output.generalized_acceleration.amax());
        maximum_actuator_torque_abs =
            maximum_actuator_torque_abs.max(output.actuator_torque.amax());
        maximum_contact_force_abs =
            maximum_contact_force_abs.max(output.contact_force_world.amax());
        maximum_dynamics_residual = maximum_dynamics_residual.max(output.dynamics_residual_linf);
        maximum_contact_acceleration_residual =
            maximum_contact_acceleration_residual.max(output.contact_acceleration_residual_linf);
        minimum_friction_margin = minimum_friction_margin.min(output.minimum_friction_margin);
        minimum_torque_margin = minimum_torque_margin.min(output.minimum_torque_margin);
        match output.status {
            SolveStatus::SolvedWithSlack => degraded_ticks += 1,
            SolveStatus::Solved => {}
            _ => {
                infeasible_ticks += 1;
            }
        }
        let height_error =
            desired_root_z - state.robot.control_world_from_root.translation.vector.z;
        height_error_squared += height_error * height_error;
        let horizontal_error = squat_target.control_world_from_root.translation.vector.xy()
            - state.robot.control_world_from_root.translation.vector.xy();
        horizontal_error_squared += horizontal_error.norm_squared();
        maximum_root_rotation =
            maximum_root_rotation.max(state.robot.control_world_from_root.rotation.angle());
    }

    timings.sort_by(f64::total_cmp);
    let final_height_error = squat_target.control_world_from_root.translation.vector.z
        - state.robot.control_world_from_root.translation.vector.z;
    let final_horizontal_error = (squat_target.control_world_from_root.translation.vector.xy()
        - state.robot.control_world_from_root.translation.vector.xy())
    .norm();
    Ok(FloatingSquatReport {
        ticks,
        compiled_signal_nodes: floating_signals.node_count(),
        compiled_task_slots: floating_tasks.floating_task_slots(),
        maximum_policy_oracle_acceleration_delta,
        target_lowering_m: LOWERING,
        achieved_lowering_m: standing_root_z
            - state.robot.control_world_from_root.translation.vector.z,
        final_root_height_error_m: final_height_error,
        root_height_tracking_rms_m: (height_error_squared / ticks as f64).sqrt(),
        root_horizontal_tracking_rms_m: (horizontal_error_squared / ticks as f64).sqrt(),
        final_root_horizontal_error_m: final_horizontal_error,
        center_of_mass_tracking_rms_m: (center_of_mass_error_squared / ticks as f64).sqrt(),
        maximum_center_of_mass_tracking_error_m: maximum_center_of_mass_error,
        maximum_contact_position_delta_m: maximum_contact_position_delta,
        maximum_constrained_contact_drift_m: maximum_constrained_contact_drift,
        maximum_permitted_rolling_travel_m: maximum_permitted_rolling_travel,
        maximum_root_rotation_rad: maximum_root_rotation,
        maximum_generalized_acceleration_abs,
        maximum_actuator_torque_abs,
        maximum_contact_force_abs,
        maximum_dynamics_residual,
        maximum_contact_acceleration_residual,
        minimum_friction_margin,
        minimum_torque_margin,
        degraded_ticks,
        infeasible_ticks,
        mean_step_us: timings.iter().sum::<f64>() / timings.len() as f64,
        p50_step_us: percentile(&timings, 0.50),
        p99_step_us: percentile(&timings, 0.99),
        max_step_us: *timings.last().unwrap_or(&0.0),
        allocations_per_step: allocation_calls as f64 / ticks as f64,
        allocated_bytes_per_step: allocated_bytes as f64 / ticks as f64,
    })
}

fn frame_origins(model: &bonesaw_core::CompiledModel, state: &RobotState) -> Result<Vec<Vec3>> {
    let mut cache = ModelCache::new(model);
    model.forward_kinematics(state, &mut cache)?;
    Ok(cache
        .world_from_body
        .iter()
        .map(|pose| pose.translation.vector)
        .collect())
}

fn model_com(model: &bonesaw_core::CompiledModel, state: &RobotState) -> Result<Vec3> {
    let mut cache = ModelCache::new(model);
    model.forward_kinematics(state, &mut cache)?;
    Ok(cache.center_of_mass_world)
}

fn named_frame(model: &bonesaw_core::CompiledModel, name: &str) -> Result<FrameId> {
    model
        .frame_id(name)
        .with_context(|| format!("model is missing required evaluation frame {name}"))
}

fn first_named_frame(model: &bonesaw_core::CompiledModel, names: &[&str]) -> Result<FrameId> {
    names
        .iter()
        .find_map(|name| model.frame_id(name))
        .with_context(|| {
            format!(
                "model is missing every required evaluation frame alias: {}",
                names.join(", ")
            )
        })
}

fn standing_posture(model: &bonesaw_core::CompiledModel) -> DVector<f64> {
    let mut q = DVector::zeros(model.dof);
    let upkie = [
        ("left_hip", 0.4),
        ("right_hip", -0.4),
        ("left_knee", -0.625),
        ("right_knee", 0.625),
    ];
    let generic = [
        ("left_hip_pitch", -0.15),
        ("right_hip_pitch", -0.15),
        ("left_knee", 0.3),
        ("right_knee", 0.3),
        ("left_ankle", -0.15),
        ("right_ankle", -0.15),
        ("left_shoulder_roll", 0.25),
        ("right_shoulder_roll", -0.25),
        ("left_elbow", -0.25),
        ("right_elbow", 0.25),
    ];
    let entries: &[(&str, f64)] = if model.joint_id("left_hip").is_some() {
        &upkie
    } else {
        &generic
    };
    for &(name, value) in entries {
        if let Some(joint) = model.joint_id(name)
            && let Some(index) = model.joints[joint.0].coordinate
        {
            q[index] = value;
        }
    }
    q
}

fn seed_standing_pose(model: &bonesaw_core::CompiledModel, state: &mut RobotState) {
    state.q = standing_posture(model);
}

fn percentile(sorted: &[f64], fraction: f64) -> f64 {
    let index = ((sorted.len().saturating_sub(1)) as f64 * fraction).round() as usize;
    sorted.get(index).copied().unwrap_or(0.0)
}

fn print_human(report: &EvaluationReport) {
    println!(
        "Bonesaw CPU evaluation — {} ({} bodies, {} DOF, {:.2} kg)",
        report.model, report.bodies, report.dof, report.total_mass_kg
    );
    println!("Program {}", report.program_fingerprint);
    println!();
    println!(
        "{:<29} {:>9} {:>9} {:>9} {:>9} {:>8}",
        "scenario", "RMS cm", "max cm", "p50 us", "p99 us", "fallback"
    );
    for scenario in &report.scenarios {
        println!(
            "{:<29} {:>9.3} {:>9.3} {:>9.1} {:>9.1} {:>8}",
            scenario.name,
            scenario.rms_position_error_m * 100.0,
            scenario.max_position_error_m * 100.0,
            scenario.p50_tick_us,
            scenario.p99_tick_us,
            scenario.contingency_ticks,
        );
    }
    println!();
    println!(
        "Replay determinism: {} ({} ticks)",
        if report.determinism.bitwise_equal {
            "bitwise stable"
        } else {
            "MISMATCH"
        },
        report.determinism.ticks
    );
    println!(
        "Frame queries: {:.0}/s ({} queries in {:.2} ms)",
        report.frame_queries.queries_per_second,
        report.frame_queries.queries,
        report.frame_queries.elapsed_ms
    );
    println!(
        "Historical atlas queries: {:.0}/s ({} queries in {:.2} ms)",
        report.historical_frame_queries.queries_per_second,
        report.historical_frame_queries.queries,
        report.historical_frame_queries.elapsed_ms
    );
    println!(
        "Dynamics: {} states, inverse/forward max error {:.2e}, min mass eigenvalue {:.2e}, p99 {:.1} µs",
        report.dynamics.states,
        report.dynamics.maximum_inverse_forward_error,
        report.dynamics.minimum_mass_eigenvalue,
        report.dynamics.p99_round_trip_us
    );
    println!(
        "Collision: {} proxies / {} pairs, min {:.3} m, Jacobian FD max {:.2e}, full-scan p99 {:.1} µs, controller alloc {:.1} calls / {:.1} bytes per tick",
        report.collision.sphere_proxies,
        report.collision.self_pairs,
        report.collision.minimum_signed_distance_m,
        report.collision.maximum_jacobian_finite_difference_error,
        report.collision.p99_full_scan_us,
        report.collision.controller_allocations_per_tick,
        report.collision.controller_allocated_bytes_per_tick,
    );
    println!(
        "Constraints: violation {:.2e}, active residual {:.2e}, contradiction {}, p99 {:.1} µs",
        report.constraints.maximum_feasible_violation,
        report.constraints.active_reoptimized_residual,
        if report.constraints.contradictory_problem_detected {
            "typed infeasible"
        } else {
            "MISSED"
        },
        report.constraints.p99_solve_us
    );
    println!(
        "Compiled rig: {} signal nodes / {} task slots, point RMS {:.3} cm, orientation RMS {:.3} deg, p99 {:.1} µs, {}, alloc {:.1} calls / {:.1} bytes per tick",
        report.compiled_rig.signal_nodes,
        report.compiled_rig.task_slots,
        report.compiled_rig.tracking_rms_m * 100.0,
        report
            .compiled_rig
            .orientation_tracking_rms_rad
            .to_degrees(),
        report.compiled_rig.p99_tick_us,
        if report.compiled_rig.bitwise_repeat {
            "bitwise repeat"
        } else {
            "MISMATCH"
        },
        report.compiled_rig.allocations_per_tick,
        report.compiled_rig.allocated_bytes_per_tick,
    );
    println!(
        "Dynamic WBC: {} variables / {} contacts, dynamics {:.2e}, contact {:.2e}, friction margin {:.2e}, p99 {:.1} µs, alloc {:.1} calls / {:.1} bytes per tick",
        report.dynamic_wbc.decision_variables,
        report.dynamic_wbc.contacts,
        report.dynamic_wbc.maximum_dynamics_residual,
        report.dynamic_wbc.maximum_contact_acceleration_residual,
        report.dynamic_wbc.minimum_friction_margin,
        report.dynamic_wbc.p99_tick_us,
        report.dynamic_wbc.allocations_per_tick,
        report.dynamic_wbc.allocated_bytes_per_tick,
    );
    println!(
        "Floating dynamic WBC: {} variables / {} contacts, dynamics {:.2e}, contact {:.2e}, friction margin {:.2e}, p99 {:.1} µs, alloc {:.1} calls / {:.1} bytes per tick",
        report.floating_dynamic_wbc.decision_variables,
        report.floating_dynamic_wbc.contacts,
        report.floating_dynamic_wbc.maximum_dynamics_residual,
        report
            .floating_dynamic_wbc
            .maximum_contact_acceleration_residual,
        report.floating_dynamic_wbc.minimum_friction_margin,
        report.floating_dynamic_wbc.p99_tick_us,
        report.floating_dynamic_wbc.allocations_per_tick,
        report.floating_dynamic_wbc.allocated_bytes_per_tick,
    );
    println!(
        "Control-path allocation sentinel: {:.1}, {:.1}, {:.1} calls/tick",
        report.scenarios[0].allocations_per_tick,
        report.scenarios[1].allocations_per_tick,
        report.scenarios[2].allocations_per_tick
    );
}

#[allow(dead_code)]
fn model_filename(path: &Path) -> &str {
    path.file_name()
        .and_then(|value| value.to_str())
        .unwrap_or("model")
}
