use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, CpuMirrorExecutor, CudaFkComCompilerStatus, CudaKinematicsModel,
    CudaMirrorFkComExecutor, CudaRuntimeStatus, FkBatchInput, FkBatchOutput, KernelManifest,
    capabilities, cuda_fk_com_source_sha256, probe_cuda_fk_com_compiler, probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;
const D2_TOLERANCE: f32 = 3.0e-5;

fn main() -> Result<()> {
    let model_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "models/toy_humanoid.urdf".to_owned());
    let urdf = std::fs::read_to_string(&model_path)
        .with_context(|| format!("failed to read {model_path}"))?;
    let program = MotionProgram::compile_urdf(&urdf, TimingSpec::default(), 1)?;
    let cpu = CpuMirrorExecutor::compile(&program, AGENT_CAPACITY, AGENT_ALIGNMENT)?;
    let layout = cpu.descriptor().layout.clone();
    let model = CudaKinematicsModel::compile(&program, &layout)?;
    let input = deterministic_input(layout.clone())?;
    let mut cpu_output = FkBatchOutput::new(layout.clone());
    cpu.execute_into(&input, &mut cpu_output)?;
    let cpu_checks = cpu_checks(&cpu, &layout, &input, &cpu_output)?;
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
    let compiler = probe_cuda_fk_com_compiler(target[0], target[1]);
    let source_sha256 = hex(cuda_fk_com_source_sha256());
    let model_sha256 = hex(model.hash());

    if runtime.status != CudaRuntimeStatus::Available
        || compiler.status != CudaFkComCompilerStatus::Available
    {
        println!(
            "{}",
            serde_json::to_string_pretty(&json!({
                "schema": 1,
                "revision": "cuda-fk-com-r92",
                "status": "unavailable",
                "claim": "generic fixed-buffer CUDA FK/CoM executor implemented; absent compiler/device gates are not inferred",
                "model": model_path,
                "capabilities": capabilities(),
                "kernel_manifest_bits": KernelManifest::fk_center_of_mass_v1().bits(),
                "source_sha256": source_sha256,
                "model_pack_sha256": model_sha256,
                "compiler_probe": compiler,
                "runtime_probe": runtime,
                "layout": layout,
                "cpu_mirror_checks": cpu_checks,
                "device_checks": Value::Null,
                "cuda_graph": "not_implemented",
                "full_cuda_mirror": "not_implemented",
                "hidden_cpu_fallback": false,
            }))?
        );
        return Ok(());
    }

    let mut executor = match CudaMirrorFkComExecutor::compile(&program, &cpu, 0) {
        Ok(executor) => executor,
        Err(error) => {
            println!(
                "{}",
                serde_json::to_string_pretty(&json!({
                    "schema": 1,
                    "revision": "cuda-fk-com-r92",
                    "status": "failed",
                    "claim": "compiler and runtime available but FK/CoM executor construction failed",
                    "model": model_path,
                    "capabilities": capabilities(),
                    "kernel_manifest_bits": KernelManifest::fk_center_of_mass_v1().bits(),
                    "source_sha256": source_sha256,
                    "model_pack_sha256": model_sha256,
                    "compiler_probe": compiler,
                    "runtime_probe": runtime,
                    "layout": layout,
                    "cpu_mirror_checks": cpu_checks,
                    "device_error": error.to_string(),
                    "hidden_cpu_fallback": false,
                }))?
            );
            bail!("CUDA FK/CoM executor construction failed: {error}");
        }
    };
    let mut device_output = FkBatchOutput::new(layout.clone());
    executor.execute_into(&input, &mut device_output)?;
    let d2 = compare_outputs(&layout, &cpu_output, &device_output);
    let first_bytes = output_bytes(&device_output);
    let mut d1_exact = true;
    let mut timings_us = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        executor.execute_into(&input, &mut device_output)?;
        timings_us.push(start.elapsed().as_secs_f64() * 1e6);
        d1_exact &= output_bytes(&device_output) == first_bytes;
    }
    timings_us.sort_by(f64::total_cmp);

    let mut poisoned = input.clone();
    let poison_agent = 2;
    poisoned.q_soa_mut()[layout.q_index(0, poison_agent)] = f32::NAN;
    let mut poisoned_output = FkBatchOutput::new(layout.clone());
    executor.execute_into(&poisoned, &mut poisoned_output)?;
    let invalid_typed = poisoned_output.agent_status()[poison_agent] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != poison_agent)
        .all(|agent| agent_equal(&layout, agent, &device_output, &poisoned_output));
    let padding_deterministic = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| agent_zero(&layout, agent, &poisoned_output));
    let d2_pass = d2["pass"].as_bool().unwrap_or(false);
    let passed = cpu_checks["pass"].as_bool().unwrap_or(false)
        && d1_exact
        && d2_pass
        && invalid_typed
        && neighbor_isolation
        && padding_deterministic;
    let report = json!({
        "schema": 1,
        "revision": "cuda-fk-com-r92",
        "status": if passed { "pass" } else { "fail" },
        "claim": "direct-launch CudaMirrorF32 StateInput + generic tree FK + center of mass stages only",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::fk_center_of_mass_v1().bits(),
        "source_sha256": source_sha256,
        "model_pack_sha256": model_sha256,
        "compiler_probe": compiler,
        "runtime_probe": runtime,
        "layout": layout,
        "cpu_mirror_checks": cpu_checks,
        "device": executor.device(),
        "fingerprint": executor.fingerprint(),
        "nvrtc_log": executor.nvrtc_log(),
        "device_checks": {
            "d1_repeat_bytes_exact": d1_exact,
            "d2_cpu_mirror": d2,
            "invalid_agent_typed": invalid_typed,
            "neighbor_isolation_exact": neighbor_isolation,
            "padding_inactive_and_zero": padding_deterministic,
            "repeats": REPEATS,
            "direct_launch_end_to_end_us": distribution(&timings_us),
        },
        "cuda_graph": "not_implemented",
        "full_cuda_mirror": "not_implemented",
        "hidden_cpu_fallback": false,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !passed {
        bail!("CUDA FK/CoM conformance failed");
    }
    Ok(())
}

fn deterministic_input(layout: BatchLayout) -> Result<FkBatchInput> {
    let mut input = FkBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for agent in 0..ACTIVE_AGENTS {
        for coordinate in 0..layout.coordinate_count {
            input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                (coordinate as f32 * 0.013).mul_add(agent as f32 + 1.0, -0.07);
        }
        for component in 0..3 {
            input.root_pose_soa_mut()[layout.root_pose_index(9 + component, agent)] =
                (agent as f32 * 0.1) + component as f32 * 0.01;
        }
    }
    Ok(input)
}

fn cpu_checks(
    cpu: &CpuMirrorExecutor,
    layout: &BatchLayout,
    input: &FkBatchInput,
    expected: &FkBatchOutput,
) -> Result<Value> {
    let first = output_bytes(expected);
    let mut repeated = FkBatchOutput::new(layout.clone());
    let mut repeat_exact = true;
    for _ in 0..REPEATS {
        cpu.execute_into(input, &mut repeated)?;
        repeat_exact &= output_bytes(&repeated) == first;
    }
    let active_ok =
        (0..ACTIVE_AGENTS).all(|agent| expected.agent_status()[agent] == AgentStatus::Ok);
    let padding_zero =
        (ACTIVE_AGENTS..layout.agent_stride).all(|agent| agent_zero(layout, agent, expected));

    let mut poisoned = input.clone();
    poisoned.q_soa_mut()[layout.q_index(0, 2)] = f32::NAN;
    let mut poisoned_output = FkBatchOutput::new(layout.clone());
    cpu.execute_into(&poisoned, &mut poisoned_output)?;
    let invalid_typed = poisoned_output.agent_status()[2] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != 2)
        .all(|agent| agent_equal(layout, agent, expected, &poisoned_output));
    Ok(json!({
        "pass": repeat_exact && active_ok && padding_zero && invalid_typed && neighbor_isolation,
        "d1_repeat_bytes_exact": repeat_exact,
        "active_agents_ok": active_ok,
        "padding_inactive_and_zero": padding_zero,
        "invalid_agent_typed": invalid_typed,
        "neighbor_isolation_exact": neighbor_isolation,
        "repeats": REPEATS,
        "allocation_free_hot_path_unit_gate": true,
    }))
}

fn compare_outputs(
    layout: &BatchLayout,
    expected: &FkBatchOutput,
    actual: &FkBatchOutput,
) -> Value {
    let status_exact = expected.agent_status() == actual.agent_status();
    let max_pose = max_abs_diff(expected.body_pose_soa(), actual.body_pose_soa());
    let max_com = max_abs_diff(expected.center_of_mass_soa(), actual.center_of_mass_soa());
    let max_mass = max_abs_diff(expected.total_mass(), actual.total_mass());
    let inactive_zero = (0..layout.agent_stride)
        .filter(|agent| expected.agent_status()[*agent] != AgentStatus::Ok)
        .all(|agent| agent_zero(layout, agent, actual));
    json!({
        "pass": status_exact && max_pose <= D2_TOLERANCE && max_com <= D2_TOLERANCE
            && max_mass <= D2_TOLERANCE && inactive_zero,
        "tolerance": D2_TOLERANCE,
        "status_exact": status_exact,
        "max_abs_body_pose": max_pose,
        "max_abs_center_of_mass": max_com,
        "max_abs_total_mass": max_mass,
        "inactive_and_invalid_zero": inactive_zero,
    })
}

fn max_abs_diff(left: &[f32], right: &[f32]) -> f32 {
    left.iter()
        .zip(right)
        .map(|(left, right)| (left - right).abs())
        .fold(0.0, f32::max)
}

fn output_bytes(output: &FkBatchOutput) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(
        (output.body_pose_soa().len()
            + output.center_of_mass_soa().len()
            + output.total_mass().len())
            * size_of::<f32>()
            + output.agent_status().len(),
    );
    for value in output
        .body_pose_soa()
        .iter()
        .chain(output.center_of_mass_soa())
        .chain(output.total_mass())
    {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    bytes.extend(output.agent_status().iter().map(|status| *status as u8));
    bytes
}

fn agent_equal(
    layout: &BatchLayout,
    agent: usize,
    left: &FkBatchOutput,
    right: &FkBatchOutput,
) -> bool {
    left.agent_status()[agent] == right.agent_status()[agent]
        && (0..layout.body_count).all(|body| {
            left.body_pose(agent, body)
                .unwrap()
                .iter()
                .zip(right.body_pose(agent, body).unwrap())
                .all(|(left, right)| left.to_bits() == right.to_bits())
        })
        && left
            .center_of_mass(agent)
            .unwrap()
            .iter()
            .zip(right.center_of_mass(agent).unwrap())
            .all(|(left, right)| left.to_bits() == right.to_bits())
        && left.total_mass()[agent].to_bits() == right.total_mass()[agent].to_bits()
}

fn agent_zero(layout: &BatchLayout, agent: usize, output: &FkBatchOutput) -> bool {
    output.agent_status()[agent] != AgentStatus::Ok
        && (0..layout.body_count).all(|body| {
            (0..layout.pose_components).all(|component| {
                output.body_pose_soa()[layout.body_pose_index(body, component, agent)].to_bits()
                    == 0
            })
        })
        && (0..3).all(|component| {
            output.center_of_mass_soa()[layout.com_index(component, agent)].to_bits() == 0
        })
        && output.total_mass()[agent].to_bits() == 0
}

fn distribution(sorted: &[f64]) -> Value {
    let percentile = |fraction: f64| {
        let index = ((sorted.len() - 1) as f64 * fraction).round() as usize;
        sorted[index]
    };
    json!({
        "minimum": sorted[0],
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "maximum": sorted[sorted.len() - 1],
    })
}

fn hex(value: [u8; 32]) -> String {
    value.iter().map(|byte| format!("{byte:02x}")).collect()
}
