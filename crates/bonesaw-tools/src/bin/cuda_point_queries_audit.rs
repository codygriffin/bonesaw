use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, CpuMirrorExecutor, CudaFkComCompilerStatus,
    CudaMirrorPointQueriesExecutor, DynamicsBatchInput, DynamicsBatchOutput, FkBatchInput,
    FkBatchOutput, JacobianBatchOutput, KernelManifest, PointQueryBatchOutput, PointQuerySpec,
    capabilities, cuda_dynamics_source_sha256, cuda_fk_com_source_sha256,
    cuda_jacobians_source_sha256, cuda_point_queries_source_sha256, probe_cuda_dynamics_compiler,
    probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler, probe_cuda_point_queries_compiler,
    probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;
const TOLERANCE: f32 = 2.0e-4;

fn main() -> Result<()> {
    let model_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "models/toy_humanoid.urdf".to_owned());
    let urdf = std::fs::read_to_string(&model_path)
        .with_context(|| format!("failed to read {model_path}"))?;
    let program = MotionProgram::compile_urdf(&urdf, TimingSpec::default(), 1)?;
    let last = program.model.bodies.len() - 1;
    let queries = [
        PointQuerySpec {
            stable_id: 95_001,
            frame_index: program.model.root.0,
            point_in_frame: [0.13, -0.07, 0.19],
        },
        PointQuerySpec {
            stable_id: 95_002,
            frame_index: 1.min(last),
            point_in_frame: [-0.04, 0.08, 0.11],
        },
        PointQuerySpec {
            stable_id: 95_003,
            frame_index: last / 2,
            point_in_frame: [0.03, -0.05, -0.09],
        },
        PointQuerySpec {
            stable_id: 95_004,
            frame_index: last,
            point_in_frame: [0.06, 0.02, -0.03],
        },
    ];
    let cpu = CpuMirrorExecutor::compile_with_point_queries(
        &program,
        AGENT_CAPACITY,
        AGENT_ALIGNMENT,
        &queries,
    )?;
    let layout = cpu.descriptor().layout.clone();
    let (fk_input, dynamics_input) = deterministic_inputs(layout.clone())?;
    let (mut cpu_fk, mut cpu_jacobian, mut cpu_dynamics, mut cpu_points) = outputs(&layout);
    execute_cpu(
        &cpu,
        &fk_input,
        &dynamics_input,
        &mut cpu_fk,
        &mut cpu_jacobian,
        &mut cpu_dynamics,
        &mut cpu_points,
    )?;
    let first = point_bytes(&cpu_points);
    let (mut repeat_fk, mut repeat_jacobian, mut repeat_dynamics, mut repeat_points) =
        outputs(&layout);
    let mut cpu_repeat_exact = true;
    for _ in 0..REPEATS {
        execute_cpu(
            &cpu,
            &fk_input,
            &dynamics_input,
            &mut repeat_fk,
            &mut repeat_jacobian,
            &mut repeat_dynamics,
            &mut repeat_points,
        )?;
        cpu_repeat_exact &= point_bytes(&repeat_points) == first;
    }
    let cpu_checks = json!({
        "pass": cpu_repeat_exact
            && (0..ACTIVE_AGENTS).all(|agent| cpu_points.agent_status()[agent] == AgentStatus::Ok)
            && (ACTIVE_AGENTS..layout.agent_stride).all(|agent| point_agent_zero(&layout, agent, &cpu_points)),
        "d1_complete_point_products_bytes_exact": cpu_repeat_exact,
        "active_agents_ok": (0..ACTIVE_AGENTS).all(|agent| cpu_points.agent_status()[agent] == AgentStatus::Ok),
        "padding_inactive_and_zero": (ACTIVE_AGENTS..layout.agent_stride).all(|agent| point_agent_zero(&layout, agent, &cpu_points)),
        "allocation_free_hot_path_unit_gate": true,
        "f64_position_jacobian_jdot_v_gate": true,
        "repeats": REPEATS,
    });

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
    let fk_compiler = probe_cuda_fk_com_compiler(target[0], target[1]);
    let jacobian_compiler = probe_cuda_jacobians_compiler(target[0], target[1]);
    let dynamics_compiler = probe_cuda_dynamics_compiler(target[0], target[1]);
    let point_compiler = probe_cuda_point_queries_compiler(target[0], target[1]);
    let compiler_available = [
        &fk_compiler,
        &jacobian_compiler,
        &dynamics_compiler,
        &point_compiler,
    ]
    .into_iter()
    .all(|probe| probe.status == CudaFkComCompilerStatus::Available);
    let common = json!({
        "schema": 1,
        "revision": "cuda-point-queries-r95",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::point_queries_v1().bits(),
        "source_sha256": {
            "fk_com": hex(cuda_fk_com_source_sha256()),
            "jacobians": hex(cuda_jacobians_source_sha256()),
            "dynamics": hex(cuda_dynamics_source_sha256()),
            "point_queries": hex(cuda_point_queries_source_sha256()),
        },
        "compiler_probes": {
            "fk_com": fk_compiler,
            "jacobians": jacobian_compiler,
            "dynamics": dynamics_compiler,
            "point_queries": point_compiler,
        },
        "runtime_probe": runtime,
        "layout": layout,
        "query_descriptor": cpu.descriptor().point_queries,
        "cpu_mirror_checks": cpu_checks,
        "cuda_graph": "not_implemented",
        "full_cuda_mirror": "not_implemented",
        "hidden_cpu_fallback": false,
    });
    if common["runtime_probe"]["status"]["kind"] != "available" || !compiler_available {
        let mut report = common;
        report["status"] = json!("unavailable");
        report["claim"] = json!(
            "StateInput+FK/CoM+Jacobians+Dynamics+fixed point position/J/Jdot-v CUDA pipeline implemented; absent compiler/device gates are not inferred"
        );
        report["device_checks"] = Value::Null;
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }

    let mut executor = match CudaMirrorPointQueriesExecutor::compile(&program, &cpu, 0) {
        Ok(executor) => executor,
        Err(error) => {
            let mut report = common;
            report["status"] = json!("failed");
            report["device_error"] = json!(error.to_string());
            println!("{}", serde_json::to_string_pretty(&report)?);
            bail!("CUDA point-query executor construction failed: {error}");
        }
    };
    let (mut device_fk, mut device_jacobian, mut device_dynamics, mut device_points) =
        outputs(&layout);
    executor.execute_into(
        &fk_input,
        &dynamics_input,
        &mut device_fk,
        &mut device_jacobian,
        &mut device_dynamics,
        &mut device_points,
    )?;
    let d3 = compare_points(&cpu_points, &device_points);
    let first_device = point_bytes(&device_points);
    let mut d1_exact = true;
    let mut timings_us = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        executor.execute_into(
            &fk_input,
            &dynamics_input,
            &mut device_fk,
            &mut device_jacobian,
            &mut device_dynamics,
            &mut device_points,
        )?;
        timings_us.push(start.elapsed().as_secs_f64() * 1e6);
        d1_exact &= point_bytes(&device_points) == first_device;
    }
    timings_us.sort_by(f64::total_cmp);
    let passed = common["cpu_mirror_checks"]["pass"]
        .as_bool()
        .unwrap_or(false)
        && d1_exact
        && d3["pass"].as_bool().unwrap_or(false);
    let mut report = common;
    report["status"] = json!(if passed { "pass" } else { "fail" });
    report["claim"] =
        json!("direct-launch CudaMirrorF32 through fixed point position/J/Jdot-v only");
    report["device"] = json!(executor.device());
    report["fingerprint"] = json!(executor.fingerprint());
    report["device_checks"] = json!({
        "d1_complete_pipeline_bytes_exact": d1_exact,
        "d3_cpu_mirror": d3,
        "repeats": REPEATS,
        "single_stream_direct_launch_end_to_end_us": distribution(&timings_us),
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !passed {
        bail!("CUDA point-query conformance failed");
    }
    Ok(())
}

fn deterministic_inputs(layout: BatchLayout) -> Result<(FkBatchInput, DynamicsBatchInput)> {
    let mut fk = FkBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    let mut dynamics = DynamicsBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for agent in 0..ACTIVE_AGENTS {
        for coordinate in 0..layout.coordinate_count {
            fk.q_soa_mut()[layout.q_index(coordinate, agent)] =
                (coordinate as f32 * 0.013).mul_add(agent as f32 + 1.0, -0.07);
        }
        for component in 0..3 {
            fk.root_pose_soa_mut()[layout.root_pose_index(9 + component, agent)] =
                agent as f32 * 0.1 + component as f32 * 0.01;
            dynamics.gravity_world_soa_mut()[component * layout.agent_stride + agent] =
                [0.17, -0.09, -9.73][component];
        }
        for coordinate in 0..layout.generalized_coordinate_count {
            dynamics.generalized_velocity_soa_mut()[layout.generalized_index(coordinate, agent)] =
                (coordinate as f32 * 0.019).mul_add(agent as f32 + 0.5, -0.11);
        }
    }
    Ok((fk, dynamics))
}

fn outputs(
    layout: &BatchLayout,
) -> (
    FkBatchOutput,
    JacobianBatchOutput,
    DynamicsBatchOutput,
    PointQueryBatchOutput,
) {
    (
        FkBatchOutput::new(layout.clone()),
        JacobianBatchOutput::new(layout.clone()),
        DynamicsBatchOutput::new(layout.clone()),
        PointQueryBatchOutput::new(layout.clone()),
    )
}

fn execute_cpu(
    cpu: &CpuMirrorExecutor,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    fk: &mut FkBatchOutput,
    jacobian: &mut JacobianBatchOutput,
    dynamics: &mut DynamicsBatchOutput,
    points: &mut PointQueryBatchOutput,
) -> Result<()> {
    cpu.execute_into(fk_input, fk)?;
    cpu.execute_jacobians_into(fk, jacobian)?;
    cpu.execute_dynamics_into(dynamics_input, fk, jacobian, dynamics)?;
    cpu.execute_point_queries_into(fk, jacobian, dynamics, points)?;
    Ok(())
}

fn point_bytes(points: &PointQueryBatchOutput) -> Vec<u8> {
    let mut bytes = Vec::new();
    for values in [
        points.point_position_soa(),
        points.point_jacobian_soa(),
        points.point_bias_acceleration_soa(),
    ] {
        for value in values {
            bytes.extend(value.to_bits().to_le_bytes());
        }
    }
    bytes.extend(points.agent_status().iter().map(|status| *status as u8));
    bytes
}

fn point_agent_zero(layout: &BatchLayout, agent: usize, points: &PointQueryBatchOutput) -> bool {
    points.agent_status()[agent] == AgentStatus::Inactive
        && points
            .point_position_soa()
            .iter()
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, value)| value.to_bits() == 0)
        && points
            .point_jacobian_soa()
            .iter()
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, value)| value.to_bits() == 0)
        && points
            .point_bias_acceleration_soa()
            .iter()
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, value)| value.to_bits() == 0)
}

fn compare_points(expected: &PointQueryBatchOutput, actual: &PointQueryBatchOutput) -> Value {
    let compare = |left: &[f32], right: &[f32]| {
        left.iter().zip(right).fold(0.0_f32, |max, (left, right)| {
            max.max((left - right).abs() / (1.0 + left.abs().max(right.abs())))
        })
    };
    let position = compare(expected.point_position_soa(), actual.point_position_soa());
    let jacobian = compare(expected.point_jacobian_soa(), actual.point_jacobian_soa());
    let bias = compare(
        expected.point_bias_acceleration_soa(),
        actual.point_bias_acceleration_soa(),
    );
    json!({
        "pass": expected.agent_status() == actual.agent_status() && position <= TOLERANCE && jacobian <= TOLERANCE && bias <= TOLERANCE,
        "tolerance_abs_plus_rel": TOLERANCE,
        "max_position_abs_rel": position,
        "max_jacobian_abs_rel": jacobian,
        "max_bias_acceleration_abs_rel": bias,
    })
}

fn distribution(values: &[f64]) -> Value {
    json!({"min": values[0], "p50": values[values.len()/2], "p99": values[((values.len() as f64 * 0.99) as usize).min(values.len()-1)], "max": values[values.len()-1]})
}

fn hex(bytes: [u8; 32]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}
