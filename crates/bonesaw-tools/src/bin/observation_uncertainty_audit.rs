use std::{
    alloc::{GlobalAlloc, Layout, System},
    env,
    hint::black_box,
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use anyhow::{Context, Result, ensure};
use bonesaw_core::{
    CollisionAccelerationBarrierConfig, DynamicWbcConfig, FloatingDynamicWbc,
    FloatingDynamicWbcInput, FloatingDynamicWbcOutput, FloatingDynamicWbcScratch,
    FloatingTaskPriorities, FloatingTaskWeights, Motion6, ReconstructionProvenance,
    RobotObservationErrorBound, RobotObservationErrorGrowth,
    RobotObservationReconstructionEvidence, RobotObservationStamp, RobotState, SolveStatus, Vec3,
    VelocityBounds, joint_acceleration_interval_with_observation_error, load_urdf_file,
};
use nalgebra::DVector;
use serde_json::{Value, json};

const EXPOSURE_NS: [i64; 7] = [
    0, 500_000, 1_000_000, 2_500_000, 5_000_000, 7_500_000, 10_000_000,
];
const ERROR_REPEATS: usize = 200_000;

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

fn main() -> Result<()> {
    let model_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "models/tight_avoidance_toy.urdf".to_owned());
    let repeats = env::args()
        .nth(2)
        .map(|value| value.parse::<usize>())
        .transpose()
        .context("solve repeats must be an unsigned integer")?
        .unwrap_or(500);
    ensure!(repeats >= 20, "solve repeats must be at least 20");

    let model = load_urdf_file(&model_path)
        .with_context(|| format!("failed to load uncertainty fixture {model_path}"))?;
    ensure!(
        model.dof == 1,
        "uncertainty fixture must have one coordinate"
    );
    let barrier = CollisionAccelerationBarrierConfig {
        hard_margin: 0.02,
        influence_margin: 0.10,
        natural_frequency_rad_s: 10.0,
        damping_ratio: 1.0,
        ..CollisionAccelerationBarrierConfig::default()
    };
    let controller = FloatingDynamicWbc::new(
        model.clone(),
        DynamicWbcConfig {
            gravity_world: Vec3::zeros(),
            floating_collision_barrier: Some(barrier),
            ..DynamicWbcConfig::default()
        },
    )?;
    ensure!(
        controller.collision_pair_count() == 1,
        "fixture must have one collision pair"
    );

    let mut state = RobotState::zeros(&model);
    state.q[0] = -0.12;
    state.v[0] = -0.2;
    let generalized_dof = model.dof + 6;
    let mut desired = DVector::zeros(generalized_dof);
    desired[6] = -50.0;
    let acceleration_bounds = VelocityBounds {
        lower: DVector::from_element(generalized_dof, -100.0),
        upper: DVector::from_element(generalized_dof, 100.0),
    };
    let torque_bounds = VelocityBounds {
        lower: DVector::from_element(model.dof, -100.0),
        upper: DVector::from_element(model.dof, 100.0),
    };
    let input = FloatingDynamicWbcInput {
        state: &state,
        root_twist_world: Motion6::default(),
        desired_generalized_acceleration: &desired,
        task_priorities: FloatingTaskPriorities::default(),
        task_weights: FloatingTaskWeights::default(),
        joint_posture_weight: 1.0,
        joint_acceleration_task: None,
        center_of_mass_task: None,
        centroidal_angular_momentum_task: None,
        frame_angular_acceleration_tasks: &[],
        point_acceleration_tasks: &[],
        generalized_acceleration_bounds: &acceleration_bounds,
        torque_bounds: &torque_bounds,
        actuator_effort: None,
        contacts: &[],
        support_patches: &[],
    };
    let mut scratch = FloatingDynamicWbcScratch::new(&model, 0);
    let mut output = FloatingDynamicWbcOutput::workspace(
        model.dof,
        0,
        controller.maximum_constraint_count(0, 0),
    );

    for _ in 0..20 {
        controller.solve_into(input, &mut output, &mut scratch)?;
    }
    let nominal = benchmark_nominal(&controller, input, &mut output, &mut scratch, repeats)?;

    let growth = live_growth();
    let nominal_joint_upper = joint_acceleration_interval_with_observation_error(
        0.23, 3.5, 0.0, 0.0, -0.2618, 0.2618, 8.0, 200.0, 0.005,
    )
    .context("nominal joint-stopping interval is empty")?
    .1;
    let mut sweep = Vec::with_capacity(EXPOSURE_NS.len());
    let mut previous_point_error = -1.0_f64;
    let mut previous_robust_margin = f64::INFINITY;
    let mut previous_joint_upper = f64::INFINITY;
    for exposure_ns in EXPOSURE_NS {
        let evidence = prediction_evidence(exposure_ns);
        let bound = evidence.conservative_error_bound(growth)?;
        ensure!(bound.represented_point_position_error_m >= previous_point_error);
        let joint_interval = joint_acceleration_interval_with_observation_error(
            0.23,
            3.5,
            bound.joint_position_error_rad,
            bound.joint_velocity_error_rad_s,
            -0.2618,
            0.2618,
            8.0,
            200.0,
            0.005,
        )
        .context("robust joint-stopping interval became empty")?;
        ensure!(joint_interval.1 <= previous_joint_upper + 1e-12);

        let error_cost = benchmark_error_bound(evidence, growth);
        for _ in 0..20 {
            controller.solve_into_with_observation_error(
                input,
                bound,
                &mut output,
                &mut scratch,
            )?;
        }
        let solve = benchmark_pair(
            &controller,
            input,
            bound,
            &mut output,
            &mut scratch,
            repeats,
        )?;
        controller.solve_into_with_observation_error(input, bound, &mut output, &mut scratch)?;
        let robust_margin = output.collision_barrier.minimum_margin_m;
        let raw_margin = robust_margin + 2.0 * bound.represented_point_position_error_m;
        ensure!(robust_margin <= previous_robust_margin + 1e-12);
        ensure!((raw_margin - nominal["collision_margin_m"].as_f64().unwrap()).abs() < 1e-12);
        ensure!(matches!(
            output.status,
            SolveStatus::Solved | SolveStatus::SolvedWithSlack
        ));

        sweep.push(json!({
            "exposure_ns": exposure_ns,
            "inside_live_prediction_horizon": evidence.hard_constraint_eligible,
            "error_bound": bound_json(bound),
            "error_bound_ns_per_call": error_cost["ns_per_call"],
            "error_bound_allocation_calls": error_cost["allocation_calls"],
            "error_bound_allocated_bytes": error_cost["allocated_bytes"],
            "joint_stopping_interval_rad_s2": [joint_interval.0, joint_interval.1],
            "joint_stopping_upper_erosion_rad_s2": nominal_joint_upper - joint_interval.1,
            "self_collision_raw_margin_m": raw_margin,
            "self_collision_robust_margin_m": robust_margin,
            "self_collision_margin_erosion_m": raw_margin - robust_margin,
            "required_normal_acceleration_mps2": output.collision_barrier.limiting_required_normal_acceleration_mps2,
            "achieved_normal_acceleration_mps2": output.collision_barrier.limiting_achieved_normal_acceleration_mps2,
            "generalized_acceleration_slide_rad_s2": output.generalized_acceleration[6],
            "solve": solve,
        }));
        previous_point_error = bound.represented_point_position_error_m;
        previous_robust_margin = robust_margin;
        previous_joint_upper = joint_interval.1;
    }

    let status = sweep.iter().all(|row| {
        row["error_bound_allocation_calls"] == 0
            && row["error_bound_allocated_bytes"] == 0
            && row["solve"]["allocation_calls"] == 0
            && row["solve"]["allocated_bytes"] == 0
            && row["solve"]["bitwise_repeat"] == true
    }) && nominal["allocation_calls"] == 0
        && nominal["allocated_bytes"] == 0
        && nominal["bitwise_repeat"] == true;
    let report = json!({
        "schema": 1,
        "revision": "observation-uncertainty-exposure-r120",
        "status": if status { "pass" } else { "fail" },
        "model": model_path,
        "execution": "fixed_state_without_policy_physics_or_integration",
        "solve_repeats": repeats,
        "error_bound_repeats": ERROR_REPEATS,
        "fixed_value_bytes": {
            "reconstruction_evidence": std::mem::size_of::<RobotObservationReconstructionEvidence>(),
            "error_growth": std::mem::size_of::<RobotObservationErrorGrowth>(),
            "error_bound": std::mem::size_of::<RobotObservationErrorBound>(),
        },
        "nominal": nominal,
        "sweep": sweep,
        "claims": {
            "monotone_error_growth": true,
            "monotone_self_collision_margin_consumption": true,
            "monotone_joint_stopping_consumption": true,
            "hot_path_allocation_free": status,
            "calibrated_estimator_or_probability": false,
            "plant_response": false,
        },
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    ensure!(status, "uncertainty exposure audit failed");
    Ok(())
}

fn live_growth() -> RobotObservationErrorGrowth {
    RobotObservationErrorGrowth {
        joint_velocity_error_bound_rad_s: 0.08,
        joint_acceleration_error_bound_rad_s2: 6.0,
        root_linear_velocity_error_bound_m_s: 0.03,
        root_linear_acceleration_error_bound_m_s2: 0.8,
        root_angular_velocity_error_bound_rad_s: 0.04,
        root_angular_acceleration_error_bound_rad_s2: 1.0,
        represented_point_velocity_error_bound_m_s: 0.05,
        represented_point_acceleration_error_bound_m_s2: 1.0,
        center_of_mass_velocity_error_bound_m_s: 0.03,
        center_of_mass_acceleration_error_bound_m_s2: 0.5,
        ..RobotObservationErrorGrowth::default()
    }
}

fn prediction_evidence(exposure_ns: i64) -> RobotObservationReconstructionEvidence {
    let stamp = RobotObservationStamp {
        source_time_ns: 0,
        mapped_time_ns: 0,
        source_sequence: 1,
        source_id: 0xB015,
        synchronization_uncertainty_ns: 0,
    };
    RobotObservationReconstructionEvidence {
        program_epoch: 120,
        provenance: if exposure_ns == 0 {
            ReconstructionProvenance::ExactSample
        } else {
            ReconstructionProvenance::PredictedConstantVelocity
        },
        source_interval_ns: (0, 0),
        lower_stamp: stamp,
        upper_stamp: stamp,
        source_age_ns: exposure_ns,
        source_age_headroom_ns: 10_000_000_i64.saturating_sub(exposure_ns),
        maximum_synchronization_uncertainty_ns: 0,
        synchronization_headroom_ns: 2_000_000,
        hard_constraint_eligible: exposure_ns <= 5_000_000,
    }
}

fn bound_json(bound: RobotObservationErrorBound) -> Value {
    json!({
        "joint_position_error_rad": bound.joint_position_error_rad,
        "joint_velocity_error_rad_s": bound.joint_velocity_error_rad_s,
        "root_translation_error_m": bound.root_translation_error_m,
        "root_rotation_error_rad": bound.root_rotation_error_rad,
        "represented_point_position_error_m": bound.represented_point_position_error_m,
        "center_of_mass_position_error_m": bound.center_of_mass_position_error_m,
    })
}

fn benchmark_error_bound(
    evidence: RobotObservationReconstructionEvidence,
    growth: RobotObservationErrorGrowth,
) -> Value {
    for _ in 0..1_000 {
        black_box(evidence)
            .conservative_error_bound(black_box(growth))
            .unwrap();
    }
    let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
    let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
    let start = Instant::now();
    let mut witness = 0.0;
    for _ in 0..ERROR_REPEATS {
        witness += black_box(evidence)
            .conservative_error_bound(black_box(growth))
            .unwrap()
            .joint_position_error_rad;
    }
    black_box(witness);
    let elapsed = start.elapsed().as_secs_f64();
    let allocation_calls = ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
    let allocated_bytes = ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
    json!({
        "ns_per_call": elapsed * 1e9 / ERROR_REPEATS as f64,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
    })
}

fn benchmark_nominal(
    controller: &FloatingDynamicWbc,
    input: FloatingDynamicWbcInput<'_>,
    output: &mut FloatingDynamicWbcOutput,
    scratch: &mut FloatingDynamicWbcScratch,
    repeats: usize,
) -> Result<Value> {
    controller.solve_into(input, output, scratch)?;
    let reference = output.generalized_acceleration.clone();
    let mut timings = Vec::with_capacity(repeats);
    let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
    let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
    let mut bitwise_repeat = true;
    for _ in 0..repeats {
        let start = Instant::now();
        controller.solve_into(input, output, scratch)?;
        timings.push(start.elapsed().as_secs_f64() * 1e6);
        bitwise_repeat &= same_bits(&reference, &output.generalized_acceleration);
    }
    let allocation_calls = ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
    let allocated_bytes = ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
    Ok(json!({
        "collision_margin_m": output.collision_barrier.minimum_margin_m,
        "required_normal_acceleration_mps2": output.collision_barrier.limiting_required_normal_acceleration_mps2,
        "timing_us": timing_summary(timings),
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
        "bitwise_repeat": bitwise_repeat,
    }))
}

fn benchmark_pair(
    controller: &FloatingDynamicWbc,
    input: FloatingDynamicWbcInput<'_>,
    bound: RobotObservationErrorBound,
    output: &mut FloatingDynamicWbcOutput,
    scratch: &mut FloatingDynamicWbcScratch,
    repeats: usize,
) -> Result<Value> {
    controller.solve_into(input, output, scratch)?;
    let nominal_reference = output.generalized_acceleration.clone();
    controller.solve_into_with_observation_error(input, bound, output, scratch)?;
    let robust_reference = output.generalized_acceleration.clone();
    let mut nominal_timings = Vec::with_capacity(repeats);
    let mut robust_timings = Vec::with_capacity(repeats);
    let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
    let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
    let mut bitwise_repeat = true;
    for repeat in 0..repeats {
        if repeat % 2 == 0 {
            let start = Instant::now();
            controller.solve_into(input, output, scratch)?;
            nominal_timings.push(start.elapsed().as_secs_f64() * 1e6);
            bitwise_repeat &= same_bits(&nominal_reference, &output.generalized_acceleration);
            let start = Instant::now();
            controller.solve_into_with_observation_error(input, bound, output, scratch)?;
            robust_timings.push(start.elapsed().as_secs_f64() * 1e6);
            bitwise_repeat &= same_bits(&robust_reference, &output.generalized_acceleration);
        } else {
            let start = Instant::now();
            controller.solve_into_with_observation_error(input, bound, output, scratch)?;
            robust_timings.push(start.elapsed().as_secs_f64() * 1e6);
            bitwise_repeat &= same_bits(&robust_reference, &output.generalized_acceleration);
            let start = Instant::now();
            controller.solve_into(input, output, scratch)?;
            nominal_timings.push(start.elapsed().as_secs_f64() * 1e6);
            bitwise_repeat &= same_bits(&nominal_reference, &output.generalized_acceleration);
        }
    }
    let allocation_calls = ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
    let allocated_bytes = ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
    let nominal_timing = timing_summary(nominal_timings);
    let robust_timing = timing_summary(robust_timings);
    let p50_delta_us =
        robust_timing["p50"].as_f64().unwrap() - nominal_timing["p50"].as_f64().unwrap();
    let p99_delta_us =
        robust_timing["p99"].as_f64().unwrap() - nominal_timing["p99"].as_f64().unwrap();
    Ok(json!({
        "nominal_timing_us": nominal_timing,
        "robust_timing_us": robust_timing,
        "p50_delta_us": p50_delta_us,
        "p99_delta_us": p99_delta_us,
        "allocation_calls": allocation_calls,
        "allocated_bytes": allocated_bytes,
        "bitwise_repeat": bitwise_repeat,
    }))
}

fn same_bits(left: &DVector<f64>, right: &DVector<f64>) -> bool {
    left.iter()
        .zip(right.iter())
        .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn timing_summary(mut timings: Vec<f64>) -> Value {
    timings.sort_by(f64::total_cmp);
    let percentile = |quantile: f64| {
        let index = ((timings.len() - 1) as f64 * quantile).ceil() as usize;
        timings[index]
    };
    let p50 = percentile(0.50);
    let p99 = percentile(0.99);
    json!({
        "mean": timings.iter().sum::<f64>() / timings.len() as f64,
        "p50": p50,
        "p95": percentile(0.95),
        "p99": p99,
        "max": *timings.last().unwrap(),
        "jitter_p99_minus_p50": p99 - p50,
    })
}
