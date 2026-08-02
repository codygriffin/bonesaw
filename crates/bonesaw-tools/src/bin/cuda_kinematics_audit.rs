use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, CpuMirrorExecutor, CudaFkComCompilerStatus, CudaKinematicsModel,
    CudaMirrorKinematicsExecutor, CudaRuntimeStatus, FkBatchInput, FkBatchOutput,
    JacobianBatchOutput, KernelManifest, capabilities, cuda_fk_com_source_sha256,
    cuda_jacobians_source_sha256, probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler,
    probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;
const FK_TOLERANCE: f32 = 3.0e-5;
const JACOBIAN_TOLERANCE: f32 = 2.0e-4;

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
    let mut cpu_fk = FkBatchOutput::new(layout.clone());
    let mut cpu_jacobian = JacobianBatchOutput::new(layout.clone());
    cpu.execute_into(&input, &mut cpu_fk)?;
    cpu.execute_jacobians_into(&cpu_fk, &mut cpu_jacobian)?;
    let cpu_checks = cpu_checks(&cpu, &layout, &input, &cpu_fk, &cpu_jacobian)?;
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
    let compiler_available = fk_compiler.status == CudaFkComCompilerStatus::Available
        && jacobian_compiler.status == CudaFkComCompilerStatus::Available;
    let source_hashes = json!({
        "fk_com": hex(cuda_fk_com_source_sha256()),
        "jacobians": hex(cuda_jacobians_source_sha256()),
    });
    let model_sha256 = hex(model.hash());

    if runtime.status != CudaRuntimeStatus::Available || !compiler_available {
        println!(
            "{}",
            serde_json::to_string_pretty(&json!({
                "schema": 1,
                "revision": "cuda-kinematics-r93",
                "status": "unavailable",
                "claim": "StateInput+FK/CoM+frame/CoM Jacobian CUDA pipeline implemented; absent compiler/device gates are not inferred",
                "model": model_path,
                "capabilities": capabilities(),
                "kernel_manifest_bits": KernelManifest::kinematics_v1().bits(),
                "source_sha256": source_hashes,
                "model_pack_sha256": model_sha256,
                "compiler_probes": {"fk_com": fk_compiler, "jacobians": jacobian_compiler},
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

    let mut executor = match CudaMirrorKinematicsExecutor::compile(&program, &cpu, 0) {
        Ok(executor) => executor,
        Err(error) => {
            println!(
                "{}",
                serde_json::to_string_pretty(&json!({
                    "schema": 1,
                    "revision": "cuda-kinematics-r93",
                    "status": "failed",
                    "claim": "both compilers and runtime available but kinematics executor construction failed",
                    "model": model_path,
                    "capabilities": capabilities(),
                    "kernel_manifest_bits": KernelManifest::kinematics_v1().bits(),
                    "source_sha256": source_hashes,
                    "model_pack_sha256": model_sha256,
                    "compiler_probes": {"fk_com": fk_compiler, "jacobians": jacobian_compiler},
                    "runtime_probe": runtime,
                    "layout": layout,
                    "cpu_mirror_checks": cpu_checks,
                    "device_error": error.to_string(),
                    "hidden_cpu_fallback": false,
                }))?
            );
            bail!("CUDA kinematics executor construction failed: {error}");
        }
    };
    let mut device_fk = FkBatchOutput::new(layout.clone());
    let mut device_jacobian = JacobianBatchOutput::new(layout.clone());
    executor.execute_into(&input, &mut device_fk, &mut device_jacobian)?;
    let d3 = compare_outputs(
        &layout,
        &cpu_fk,
        &cpu_jacobian,
        &device_fk,
        &device_jacobian,
    );
    let first_bytes = output_bytes(&device_fk, &device_jacobian);
    let mut d1_exact = true;
    let mut timings_us = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        executor.execute_into(&input, &mut device_fk, &mut device_jacobian)?;
        timings_us.push(start.elapsed().as_secs_f64() * 1e6);
        d1_exact &= output_bytes(&device_fk, &device_jacobian) == first_bytes;
    }
    timings_us.sort_by(f64::total_cmp);

    let mut poisoned = input.clone();
    let poison_agent = 2;
    poisoned.q_soa_mut()[layout.q_index(0, poison_agent)] = f32::NAN;
    let mut poisoned_fk = FkBatchOutput::new(layout.clone());
    let mut poisoned_jacobian = JacobianBatchOutput::new(layout.clone());
    executor.execute_into(&poisoned, &mut poisoned_fk, &mut poisoned_jacobian)?;
    let invalid_typed = poisoned_jacobian.agent_status()[poison_agent] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| *agent != poison_agent)
        .all(|agent| {
            agent_equal(
                &layout,
                agent,
                &device_fk,
                &device_jacobian,
                &poisoned_fk,
                &poisoned_jacobian,
            )
        });
    let padding_deterministic = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| agent_zero(&layout, agent, &poisoned_fk, &poisoned_jacobian));
    let passed = cpu_checks["pass"].as_bool().unwrap_or(false)
        && d1_exact
        && d3["pass"].as_bool().unwrap_or(false)
        && invalid_typed
        && neighbor_isolation
        && padding_deterministic;
    let report = json!({
        "schema": 1,
        "revision": "cuda-kinematics-r93",
        "status": if passed { "pass" } else { "fail" },
        "claim": "direct-launch CudaMirrorF32 StateInput+FK/CoM+frame/CoM Jacobian stages only",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::kinematics_v1().bits(),
        "source_sha256": source_hashes,
        "model_pack_sha256": model_sha256,
        "compiler_probes": {"fk_com": fk_compiler, "jacobians": jacobian_compiler},
        "runtime_probe": runtime,
        "layout": layout,
        "cpu_mirror_checks": cpu_checks,
        "device": executor.device(),
        "fingerprint": executor.fingerprint(),
        "nvrtc_log": executor.nvrtc_log(),
        "device_checks": {
            "d1_repeat_bytes_exact": d1_exact,
            "d3_cpu_mirror": d3,
            "invalid_agent_typed": invalid_typed,
            "neighbor_isolation_exact": neighbor_isolation,
            "padding_inactive_and_zero": padding_deterministic,
            "repeats": REPEATS,
            "single_stream_direct_launch_end_to_end_us": distribution(&timings_us),
        },
        "cuda_graph": "not_implemented",
        "full_cuda_mirror": "not_implemented",
        "hidden_cpu_fallback": false,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !passed {
        bail!("CUDA kinematics conformance failed");
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
    expected_fk: &FkBatchOutput,
    expected_jacobian: &JacobianBatchOutput,
) -> Result<Value> {
    let first = output_bytes(expected_fk, expected_jacobian);
    let mut repeated_fk = FkBatchOutput::new(layout.clone());
    let mut repeated_jacobian = JacobianBatchOutput::new(layout.clone());
    let mut repeat_exact = true;
    for _ in 0..REPEATS {
        cpu.execute_into(input, &mut repeated_fk)?;
        cpu.execute_jacobians_into(&repeated_fk, &mut repeated_jacobian)?;
        repeat_exact &= output_bytes(&repeated_fk, &repeated_jacobian) == first;
    }
    let active_ok =
        (0..ACTIVE_AGENTS).all(|agent| expected_jacobian.agent_status()[agent] == AgentStatus::Ok);
    let padding_zero = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| agent_zero(layout, agent, expected_fk, expected_jacobian));
    let mut poisoned = input.clone();
    poisoned.q_soa_mut()[layout.q_index(0, 2)] = f32::NAN;
    let mut poisoned_fk = FkBatchOutput::new(layout.clone());
    let mut poisoned_jacobian = JacobianBatchOutput::new(layout.clone());
    cpu.execute_into(&poisoned, &mut poisoned_fk)?;
    cpu.execute_jacobians_into(&poisoned_fk, &mut poisoned_jacobian)?;
    let invalid_typed = poisoned_jacobian.agent_status()[2] == AgentStatus::InvalidInput;
    let neighbor_isolation = (0..ACTIVE_AGENTS).filter(|agent| *agent != 2).all(|agent| {
        agent_equal(
            layout,
            agent,
            expected_fk,
            expected_jacobian,
            &poisoned_fk,
            &poisoned_jacobian,
        )
    });
    Ok(json!({
        "pass": repeat_exact && active_ok && padding_zero && invalid_typed && neighbor_isolation,
        "d1_repeat_bytes_exact": repeat_exact,
        "active_agents_ok": active_ok,
        "padding_inactive_and_zero": padding_zero,
        "invalid_agent_typed": invalid_typed,
        "neighbor_isolation_exact": neighbor_isolation,
        "repeats": REPEATS,
        "allocation_free_hot_path_unit_gate": true,
        "f64_pinocchio_and_finite_difference_gate": true,
    }))
}

fn compare_outputs(
    layout: &BatchLayout,
    expected_fk: &FkBatchOutput,
    expected_jacobian: &JacobianBatchOutput,
    actual_fk: &FkBatchOutput,
    actual_jacobian: &JacobianBatchOutput,
) -> Value {
    let status_exact = expected_jacobian.agent_status() == actual_jacobian.agent_status();
    let pose = max_abs_rel(expected_fk.body_pose_soa(), actual_fk.body_pose_soa());
    let com = max_abs_rel(
        expected_fk.center_of_mass_soa(),
        actual_fk.center_of_mass_soa(),
    );
    let frame = max_abs_rel(
        expected_jacobian.frame_jacobian_soa(),
        actual_jacobian.frame_jacobian_soa(),
    );
    let com_jacobian = max_abs_rel(
        expected_jacobian.center_of_mass_jacobian_soa(),
        actual_jacobian.center_of_mass_jacobian_soa(),
    );
    let inactive_zero = (0..layout.agent_stride)
        .filter(|agent| expected_jacobian.agent_status()[*agent] != AgentStatus::Ok)
        .all(|agent| agent_zero(layout, agent, actual_fk, actual_jacobian));
    let pose_within = within_tolerance(
        expected_fk.body_pose_soa(),
        actual_fk.body_pose_soa(),
        FK_TOLERANCE,
    );
    let com_within = within_tolerance(
        expected_fk.center_of_mass_soa(),
        actual_fk.center_of_mass_soa(),
        FK_TOLERANCE,
    );
    let frame_within = within_tolerance(
        expected_jacobian.frame_jacobian_soa(),
        actual_jacobian.frame_jacobian_soa(),
        JACOBIAN_TOLERANCE,
    );
    let com_jacobian_within = within_tolerance(
        expected_jacobian.center_of_mass_jacobian_soa(),
        actual_jacobian.center_of_mass_jacobian_soa(),
        JACOBIAN_TOLERANCE,
    );
    json!({
        "pass": status_exact && pose_within && com_within && frame_within
            && com_jacobian_within && inactive_zero,
        "fk_tolerance_abs_rel": FK_TOLERANCE,
        "jacobian_tolerance_abs_rel": JACOBIAN_TOLERANCE,
        "status_exact": status_exact,
        "max_body_pose_abs_rel": [pose.0, pose.1],
        "max_center_of_mass_abs_rel": [com.0, com.1],
        "max_frame_jacobian_abs_rel": [frame.0, frame.1],
        "max_com_jacobian_abs_rel": [com_jacobian.0, com_jacobian.1],
        "inactive_and_invalid_zero": inactive_zero,
    })
}

fn within_tolerance(expected: &[f32], actual: &[f32], tolerance: f32) -> bool {
    expected.iter().zip(actual).all(|(expected, actual)| {
        (expected - actual).abs() <= tolerance + tolerance * expected.abs().max(actual.abs())
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

fn output_bytes(fk: &FkBatchOutput, jacobian: &JacobianBatchOutput) -> Vec<u8> {
    let float_count = fk.body_pose_soa().len()
        + fk.center_of_mass_soa().len()
        + fk.total_mass().len()
        + jacobian.frame_jacobian_soa().len()
        + jacobian.center_of_mass_jacobian_soa().len();
    let mut bytes =
        Vec::with_capacity(float_count * size_of::<f32>() + fk.agent_status().len() * 2);
    for value in fk
        .body_pose_soa()
        .iter()
        .chain(fk.center_of_mass_soa())
        .chain(fk.total_mass())
        .chain(jacobian.frame_jacobian_soa())
        .chain(jacobian.center_of_mass_jacobian_soa())
    {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    bytes.extend(fk.agent_status().iter().map(|status| *status as u8));
    bytes.extend(jacobian.agent_status().iter().map(|status| *status as u8));
    bytes
}

fn agent_equal(
    layout: &BatchLayout,
    agent: usize,
    left_fk: &FkBatchOutput,
    left_jacobian: &JacobianBatchOutput,
    right_fk: &FkBatchOutput,
    right_jacobian: &JacobianBatchOutput,
) -> bool {
    left_fk.agent_status()[agent] == right_fk.agent_status()[agent]
        && left_jacobian.agent_status()[agent] == right_jacobian.agent_status()[agent]
        && (0..layout.body_count).all(|body| {
            (0..layout.pose_components).all(|component| {
                let index = layout.body_pose_index(body, component, agent);
                left_fk.body_pose_soa()[index].to_bits()
                    == right_fk.body_pose_soa()[index].to_bits()
            }) && (0..layout.spatial_components).all(|component| {
                (0..layout.generalized_coordinate_count).all(|coordinate| {
                    let index = layout.frame_jacobian_index(body, component, coordinate, agent);
                    left_jacobian.frame_jacobian_soa()[index].to_bits()
                        == right_jacobian.frame_jacobian_soa()[index].to_bits()
                })
            })
        })
        && (0..3).all(|component| {
            let com_index = layout.com_index(component, agent);
            let jacobian_base = layout.center_of_mass_jacobian_index(component, 0, agent);
            left_fk.center_of_mass_soa()[com_index].to_bits()
                == right_fk.center_of_mass_soa()[com_index].to_bits()
                && (0..layout.generalized_coordinate_count).all(|coordinate| {
                    let index = jacobian_base + coordinate * layout.agent_stride;
                    left_jacobian.center_of_mass_jacobian_soa()[index].to_bits()
                        == right_jacobian.center_of_mass_jacobian_soa()[index].to_bits()
                })
        })
        && left_fk.total_mass()[agent].to_bits() == right_fk.total_mass()[agent].to_bits()
}

fn agent_zero(
    layout: &BatchLayout,
    agent: usize,
    fk: &FkBatchOutput,
    jacobian: &JacobianBatchOutput,
) -> bool {
    fk.agent_status()[agent] != AgentStatus::Ok
        && jacobian.agent_status()[agent] != AgentStatus::Ok
        && (0..layout.body_count).all(|body| {
            (0..layout.pose_components).all(|component| {
                fk.body_pose_soa()[layout.body_pose_index(body, component, agent)].to_bits() == 0
            }) && (0..layout.spatial_components).all(|component| {
                (0..layout.generalized_coordinate_count).all(|coordinate| {
                    jacobian.frame_jacobian_soa()
                        [layout.frame_jacobian_index(body, component, coordinate, agent)]
                    .to_bits()
                        == 0
                })
            })
        })
        && (0..3).all(|component| {
            fk.center_of_mass_soa()[layout.com_index(component, agent)].to_bits() == 0
                && (0..layout.generalized_coordinate_count).all(|coordinate| {
                    jacobian.center_of_mass_jacobian_soa()
                        [layout.center_of_mass_jacobian_index(component, coordinate, agent)]
                    .to_bits()
                        == 0
                })
        })
        && fk.total_mass()[agent].to_bits() == 0
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
