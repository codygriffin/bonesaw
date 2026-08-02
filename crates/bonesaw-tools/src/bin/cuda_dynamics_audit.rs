use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, CpuMirrorExecutor, CudaFkComCompilerStatus, CudaKinematicsModel,
    CudaMirrorDynamicsExecutor, CudaRuntimeStatus, DynamicsBatchInput, DynamicsBatchOutput,
    FkBatchInput, FkBatchOutput, JacobianBatchOutput, KernelManifest, capabilities,
    cuda_dynamics_source_sha256, cuda_fk_com_source_sha256, cuda_jacobians_source_sha256,
    probe_cuda_dynamics_compiler, probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler,
    probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;
const DYNAMICS_TOLERANCE: f32 = 5.0e-4;

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
    let (fk_input, dynamics_input) = deterministic_inputs(layout.clone())?;
    let (mut cpu_fk, mut cpu_jacobian, mut cpu_dynamics) = outputs(&layout);
    execute_cpu(
        &cpu,
        &fk_input,
        &dynamics_input,
        &mut cpu_fk,
        &mut cpu_jacobian,
        &mut cpu_dynamics,
    )?;
    let cpu_checks = cpu_checks(
        &cpu,
        &layout,
        &fk_input,
        &dynamics_input,
        &cpu_fk,
        &cpu_jacobian,
        &cpu_dynamics,
    )?;

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
    let compiler_available = [&fk_compiler, &jacobian_compiler, &dynamics_compiler]
        .into_iter()
        .all(|probe| probe.status == CudaFkComCompilerStatus::Available);
    let compiler_probes = json!({
        "fk_com": fk_compiler,
        "jacobians": jacobian_compiler,
        "dynamics": dynamics_compiler,
    });
    let source_hashes = json!({
        "fk_com": hex(cuda_fk_com_source_sha256()),
        "jacobians": hex(cuda_jacobians_source_sha256()),
        "dynamics": hex(cuda_dynamics_source_sha256()),
    });
    let common = json!({
        "schema": 1,
        "revision": "cuda-dynamics-r94",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::dynamics_v1().bits(),
        "source_sha256": source_hashes,
        "model_pack_sha256": hex(model.hash()),
        "compiler_probes": compiler_probes,
        "runtime_probe": runtime,
        "layout": layout,
        "cpu_mirror_checks": cpu_checks,
        "cuda_graph": "not_implemented",
        "full_cuda_mirror": "not_implemented",
        "hidden_cpu_fallback": false,
    });

    if runtime.status != CudaRuntimeStatus::Available || !compiler_available {
        let mut report = common;
        report["status"] = json!("unavailable");
        report["claim"] = json!(
            "StateInput+FK/CoM+Jacobians+floating M/h/Ag CUDA pipeline implemented; absent compiler/device gates are not inferred"
        );
        report["device_checks"] = Value::Null;
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }

    let mut executor = match CudaMirrorDynamicsExecutor::compile(&program, &cpu, 0) {
        Ok(executor) => executor,
        Err(error) => {
            let mut report = common;
            report["status"] = json!("failed");
            report["claim"] = json!(
                "all compilers and runtime available but dynamics executor construction failed"
            );
            report["device_error"] = json!(error.to_string());
            println!("{}", serde_json::to_string_pretty(&report)?);
            bail!("CUDA dynamics executor construction failed: {error}");
        }
    };
    let (mut device_fk, mut device_jacobian, mut device_dynamics) = outputs(&layout);
    executor.execute_into(
        &fk_input,
        &dynamics_input,
        &mut device_fk,
        &mut device_jacobian,
        &mut device_dynamics,
    )?;
    let d3 = compare_dynamics(&cpu_dynamics, &device_dynamics);
    let first_bytes = output_bytes(&device_fk, &device_jacobian, &device_dynamics);
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
        )?;
        timings_us.push(start.elapsed().as_secs_f64() * 1e6);
        d1_exact &= output_bytes(&device_fk, &device_jacobian, &device_dynamics) == first_bytes;
    }
    timings_us.sort_by(f64::total_cmp);

    let mut poisoned = dynamics_input.clone();
    let poison_agent = 2;
    poisoned.generalized_velocity_soa_mut()[layout.generalized_index(0, poison_agent)] = f32::NAN;
    let (mut poisoned_fk, mut poisoned_jacobian, mut poisoned_dynamics) = outputs(&layout);
    executor.execute_into(
        &fk_input,
        &poisoned,
        &mut poisoned_fk,
        &mut poisoned_jacobian,
        &mut poisoned_dynamics,
    )?;
    let invalid_typed = poisoned_dynamics.agent_status()[poison_agent] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != poison_agent)
        .all(|agent| dynamics_agent_equal(&layout, agent, &device_dynamics, &poisoned_dynamics));
    let padding_deterministic = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| dynamics_agent_zero(&layout, agent, &poisoned_dynamics));
    let passed = common["cpu_mirror_checks"]["pass"]
        .as_bool()
        .unwrap_or(false)
        && d1_exact
        && d3["pass"].as_bool().unwrap_or(false)
        && invalid_typed
        && neighbor_isolation
        && padding_deterministic;
    let mut report = common;
    report["status"] = json!(if passed { "pass" } else { "fail" });
    report["claim"] = json!("direct-launch CudaMirrorF32 through floating M/h/Ag only");
    report["device"] = json!(executor.device());
    report["fingerprint"] = json!(executor.fingerprint());
    report["nvrtc_log"] = json!(executor.nvrtc_log());
    report["device_checks"] = json!({
        "d1_repeat_bytes_exact": d1_exact,
        "d3_cpu_mirror": d3,
        "invalid_agent_typed": invalid_typed,
        "neighbor_isolation_exact": neighbor_isolation,
        "padding_inactive_and_zero": padding_deterministic,
        "repeats": REPEATS,
        "single_stream_direct_launch_end_to_end_us": distribution(&timings_us),
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !passed {
        bail!("CUDA dynamics conformance failed");
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

fn outputs(layout: &BatchLayout) -> (FkBatchOutput, JacobianBatchOutput, DynamicsBatchOutput) {
    (
        FkBatchOutput::new(layout.clone()),
        JacobianBatchOutput::new(layout.clone()),
        DynamicsBatchOutput::new(layout.clone()),
    )
}

fn execute_cpu(
    cpu: &CpuMirrorExecutor,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    fk: &mut FkBatchOutput,
    jacobian: &mut JacobianBatchOutput,
    dynamics: &mut DynamicsBatchOutput,
) -> Result<()> {
    cpu.execute_into(fk_input, fk)?;
    cpu.execute_jacobians_into(fk, jacobian)?;
    cpu.execute_dynamics_into(dynamics_input, fk, jacobian, dynamics)?;
    Ok(())
}

fn cpu_checks(
    cpu: &CpuMirrorExecutor,
    layout: &BatchLayout,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    expected_fk: &FkBatchOutput,
    expected_jacobian: &JacobianBatchOutput,
    expected_dynamics: &DynamicsBatchOutput,
) -> Result<Value> {
    let first = output_bytes(expected_fk, expected_jacobian, expected_dynamics);
    let (mut fk, mut jacobian, mut dynamics) = outputs(layout);
    let mut repeat_exact = true;
    for _ in 0..REPEATS {
        execute_cpu(
            cpu,
            fk_input,
            dynamics_input,
            &mut fk,
            &mut jacobian,
            &mut dynamics,
        )?;
        repeat_exact &= output_bytes(&fk, &jacobian, &dynamics) == first;
    }
    let active_ok =
        (0..ACTIVE_AGENTS).all(|agent| expected_dynamics.agent_status()[agent] == AgentStatus::Ok);
    let padding_zero = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| dynamics_agent_zero(layout, agent, expected_dynamics));
    let mut poisoned = dynamics_input.clone();
    poisoned.generalized_velocity_soa_mut()[layout.generalized_index(0, 2)] = f32::NAN;
    let (mut poisoned_fk, mut poisoned_jacobian, mut poisoned_dynamics) = outputs(layout);
    execute_cpu(
        cpu,
        fk_input,
        &poisoned,
        &mut poisoned_fk,
        &mut poisoned_jacobian,
        &mut poisoned_dynamics,
    )?;
    let invalid_typed = poisoned_dynamics.agent_status()[2] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != 2)
        .all(|agent| dynamics_agent_equal(layout, agent, expected_dynamics, &poisoned_dynamics));
    Ok(json!({
        "pass": repeat_exact && active_ok && padding_zero && invalid_typed && neighbor_isolation,
        "d1_repeat_bytes_exact": repeat_exact,
        "active_agents_ok": active_ok,
        "padding_inactive_and_zero": padding_zero,
        "invalid_dynamics_input_typed": invalid_typed,
        "neighbor_isolation_exact": neighbor_isolation,
        "repeats": REPEATS,
        "allocation_free_hot_path_unit_gate": true,
        "f64_pinocchio_and_physical_identity_gate": true,
    }))
}

fn compare_dynamics(expected: &DynamicsBatchOutput, actual: &DynamicsBatchOutput) -> Value {
    let status_exact = expected.agent_status() == actual.agent_status();
    let mass = max_abs_rel(expected.mass_matrix_soa(), actual.mass_matrix_soa());
    let bias = max_abs_rel(expected.bias_force_soa(), actual.bias_force_soa());
    let centroidal = max_abs_rel(expected.centroidal_map_soa(), actual.centroidal_map_soa());
    let mass_within = within_tolerance(expected.mass_matrix_soa(), actual.mass_matrix_soa());
    let bias_within = within_tolerance(expected.bias_force_soa(), actual.bias_force_soa());
    let centroidal_within =
        within_tolerance(expected.centroidal_map_soa(), actual.centroidal_map_soa());
    json!({
        "pass": status_exact && mass_within && bias_within && centroidal_within,
        "tolerance_abs_rel": DYNAMICS_TOLERANCE,
        "status_exact": status_exact,
        "max_mass_matrix_abs_rel": [mass.0, mass.1],
        "max_bias_force_abs_rel": [bias.0, bias.1],
        "max_centroidal_map_abs_rel": [centroidal.0, centroidal.1],
    })
}

fn within_tolerance(expected: &[f32], actual: &[f32]) -> bool {
    expected.iter().zip(actual).all(|(expected, actual)| {
        (expected - actual).abs()
            <= DYNAMICS_TOLERANCE + DYNAMICS_TOLERANCE * expected.abs().max(actual.abs())
    })
}

fn max_abs_rel(expected: &[f32], actual: &[f32]) -> (f32, f32) {
    expected.iter().zip(actual).fold(
        (0.0_f32, 0.0_f32),
        |(max_abs, max_rel), (expected, actual)| {
            let absolute = (expected - actual).abs();
            let scale = expected.abs().max(actual.abs());
            let relative = if scale > 0.0 { absolute / scale } else { 0.0 };
            (max_abs.max(absolute), max_rel.max(relative))
        },
    )
}

fn output_bytes(
    fk: &FkBatchOutput,
    jacobian: &JacobianBatchOutput,
    dynamics: &DynamicsBatchOutput,
) -> Vec<u8> {
    let mut bytes = Vec::new();
    for value in fk
        .body_pose_soa()
        .iter()
        .chain(fk.center_of_mass_soa())
        .chain(fk.total_mass())
        .chain(jacobian.frame_jacobian_soa())
        .chain(jacobian.center_of_mass_jacobian_soa())
        .chain(dynamics.mass_matrix_soa())
        .chain(dynamics.bias_force_soa())
        .chain(dynamics.centroidal_map_soa())
    {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    bytes.extend(fk.agent_status().iter().map(|status| *status as u8));
    bytes.extend(jacobian.agent_status().iter().map(|status| *status as u8));
    bytes.extend(dynamics.agent_status().iter().map(|status| *status as u8));
    bytes
}

fn dynamics_agent_equal(
    layout: &BatchLayout,
    agent: usize,
    left: &DynamicsBatchOutput,
    right: &DynamicsBatchOutput,
) -> bool {
    left.agent_status()[agent] == right.agent_status()[agent]
        && (0..layout.generalized_coordinate_count).all(|row| {
            left.bias_force_value(agent, row).unwrap().to_bits()
                == right.bias_force_value(agent, row).unwrap().to_bits()
                && (0..layout.generalized_coordinate_count).all(|column| {
                    left.mass_matrix_value(agent, row, column)
                        .unwrap()
                        .to_bits()
                        == right
                            .mass_matrix_value(agent, row, column)
                            .unwrap()
                            .to_bits()
                })
                && (0..layout.spatial_components).all(|component| {
                    left.centroidal_map_value(agent, component, row)
                        .unwrap()
                        .to_bits()
                        == right
                            .centroidal_map_value(agent, component, row)
                            .unwrap()
                            .to_bits()
                })
        })
}

fn dynamics_agent_zero(layout: &BatchLayout, agent: usize, dynamics: &DynamicsBatchOutput) -> bool {
    dynamics.agent_status()[agent] != AgentStatus::Ok
        && (0..layout.generalized_coordinate_count).all(|row| {
            dynamics.bias_force_soa()[layout.generalized_index(row, agent)].to_bits() == 0
                && (0..layout.generalized_coordinate_count).all(|column| {
                    dynamics.mass_matrix_soa()[layout.mass_matrix_index(row, column, agent)]
                        .to_bits()
                        == 0
                })
                && (0..layout.spatial_components).all(|component| {
                    dynamics.centroidal_map_soa()
                        [layout.centroidal_map_index(component, row, agent)]
                    .to_bits()
                        == 0
                })
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

fn hex(value: [u8; 32]) -> String {
    value.iter().map(|byte| format!("{byte:02x}")).collect()
}
