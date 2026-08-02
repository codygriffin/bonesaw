use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, CpuMirrorExecutor, CpuMirrorStateInputExecutor,
    CudaMirrorStateInputExecutor, CudaRuntimeStatus, CudaStateInputOutput, FkBatchInput,
    KernelManifest, capabilities, cuda_state_input_kernel_sha256, probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;

fn main() -> Result<()> {
    let model_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "models/toy_humanoid.urdf".to_owned());
    let urdf = std::fs::read_to_string(&model_path)
        .with_context(|| format!("failed to read {model_path}"))?;
    let program = MotionProgram::compile_urdf(&urdf, TimingSpec::default(), 1)?;
    let cpu_mirror = CpuMirrorExecutor::compile(&program, AGENT_CAPACITY, AGENT_ALIGNMENT)?;
    let layout = cpu_mirror.descriptor().layout.clone();
    let cpu_state = CpuMirrorStateInputExecutor::compile(&program, layout.clone())?;
    let input = deterministic_input(layout.clone())?;
    let mut cpu_output = CudaStateInputOutput::new(layout.clone());
    cpu_state.execute_into(&input, &mut cpu_output)?;
    let cpu_checks = cpu_checks(&layout, &cpu_output);
    let probe = probe_cuda_runtime();
    let kernel_sha256 = cuda_state_input_kernel_sha256()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();

    if probe.status != CudaRuntimeStatus::Available {
        println!(
            "{}",
            serde_json::to_string_pretty(&json!({
                "schema": 1,
                "revision": "cuda-state-input-r91",
                "status": "unavailable",
                "claim": "real direct-launch state-input kernel compiled; no device execution claim",
                "model": model_path,
                "capabilities": capabilities(),
                "kernel_manifest_bits": KernelManifest::state_input_v1().bits(),
                "kernel_sha256": kernel_sha256,
                "runtime_probe": probe,
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

    let mut executor = match CudaMirrorStateInputExecutor::compile(&cpu_mirror, 0) {
        Ok(executor) => executor,
        Err(error) => {
            println!(
                "{}",
                serde_json::to_string_pretty(&json!({
                    "schema": 1,
                    "revision": "cuda-state-input-r91",
                    "status": "failed",
                    "claim": "runtime available but direct-launch executor construction failed",
                    "model": model_path,
                    "capabilities": capabilities(),
                    "kernel_manifest_bits": KernelManifest::state_input_v1().bits(),
                    "kernel_sha256": kernel_sha256,
                    "runtime_probe": probe,
                    "layout": layout,
                    "cpu_mirror_checks": cpu_checks,
                    "device_error": error.to_string(),
                    "hidden_cpu_fallback": false,
                }))?
            );
            bail!("CUDA state-input executor construction failed: {error}");
        }
    };
    let mut device_output = CudaStateInputOutput::new(layout.clone());
    executor.execute_into(&input, &mut device_output)?;
    let d2_exact = outputs_equal(&cpu_output, &device_output);
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
    let mut poisoned_output = CudaStateInputOutput::new(layout.clone());
    executor.execute_into(&poisoned, &mut poisoned_output)?;
    let invalid_typed = poisoned_output.status(poison_agent) == Some(AgentStatus::InvalidInput);
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != poison_agent)
        .all(|agent| agent_equal(&layout, agent, &device_output, &poisoned_output));
    let padding_deterministic = (ACTIVE_AGENTS..layout.agent_stride).all(|agent| {
        poisoned_output.status(agent) == Some(AgentStatus::Inactive)
            && agent_zero(&layout, agent, &poisoned_output)
    });
    let passed = cpu_checks["pass"].as_bool().unwrap_or(false)
        && d1_exact
        && d2_exact
        && invalid_typed
        && neighbor_isolation
        && padding_deterministic;
    let report = json!({
        "schema": 1,
        "revision": "cuda-state-input-r91",
        "status": if passed { "pass" } else { "fail" },
        "claim": "direct-launch CudaMirrorF32 state-input stage only",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::state_input_v1().bits(),
        "kernel_sha256": kernel_sha256,
        "runtime_probe": probe,
        "layout": layout,
        "cpu_mirror_checks": cpu_checks,
        "device": executor.device(),
        "fingerprint": executor.fingerprint(),
        "device_checks": {
            "d1_repeat_bytes_exact": d1_exact,
            "d2_cpu_mirror_bytes_exact": d2_exact,
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
        bail!("CUDA state-input conformance failed");
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

fn cpu_checks(layout: &BatchLayout, output: &CudaStateInputOutput) -> Value {
    let active_ok = (0..ACTIVE_AGENTS).all(|agent| output.status(agent) == Some(AgentStatus::Ok));
    let padding_zero = (ACTIVE_AGENTS..layout.agent_stride).all(|agent| {
        output.status(agent) == Some(AgentStatus::Inactive) && agent_zero(layout, agent, output)
    });
    json!({
        "pass": active_ok && padding_zero,
        "active_agents_ok": active_ok,
        "padding_inactive_and_zero": padding_zero,
    })
}

fn outputs_equal(left: &CudaStateInputOutput, right: &CudaStateInputOutput) -> bool {
    left.q_soa() == right.q_soa()
        && left.root_pose_soa() == right.root_pose_soa()
        && left.status_bytes() == right.status_bytes()
}

fn output_bytes(output: &CudaStateInputOutput) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(
        (output.q_soa().len() + output.root_pose_soa().len()) * size_of::<f32>()
            + output.status_bytes().len(),
    );
    for value in output.q_soa().iter().chain(output.root_pose_soa()) {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    bytes.extend_from_slice(output.status_bytes());
    bytes
}

fn agent_equal(
    layout: &BatchLayout,
    agent: usize,
    left: &CudaStateInputOutput,
    right: &CudaStateInputOutput,
) -> bool {
    left.status(agent) == right.status(agent)
        && (0..layout.coordinate_count).all(|coordinate| {
            left.q_soa()[layout.q_index(coordinate, agent)].to_bits()
                == right.q_soa()[layout.q_index(coordinate, agent)].to_bits()
        })
        && (0..layout.pose_components).all(|component| {
            left.root_pose_soa()[layout.root_pose_index(component, agent)].to_bits()
                == right.root_pose_soa()[layout.root_pose_index(component, agent)].to_bits()
        })
}

fn agent_zero(layout: &BatchLayout, agent: usize, output: &CudaStateInputOutput) -> bool {
    (0..layout.coordinate_count)
        .all(|coordinate| output.q_soa()[layout.q_index(coordinate, agent)].to_bits() == 0)
        && (0..layout.pose_components).all(|component| {
            output.root_pose_soa()[layout.root_pose_index(component, agent)].to_bits() == 0
        })
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
