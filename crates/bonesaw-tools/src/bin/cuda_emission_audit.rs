use std::{env, time::Instant};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, Priority, TimingSpec};
use bonesaw_cuda::{
    AgentStatus, BatchLayout, ContactKinematicMode, ContactLockSpec, CpuMirrorExecutor,
    CudaFkComCompilerStatus, CudaMirrorEmissionExecutor, DynamicsBatchInput, DynamicsBatchOutput,
    EmissionBatchInput, EmissionBatchOutput, FkBatchInput, FkBatchOutput, JacobianBatchOutput,
    KernelManifest, PointAttractorSpec, PointQueryBatchOutput, PointQuerySpec, capabilities,
    cuda_dynamics_source_sha256, cuda_emission_source_sha256, cuda_fk_com_source_sha256,
    cuda_jacobians_source_sha256, cuda_point_queries_source_sha256, probe_cuda_dynamics_compiler,
    probe_cuda_emission_compiler, probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler,
    probe_cuda_point_queries_compiler, probe_cuda_runtime,
};
use serde_json::{Value, json};

const ACTIVE_AGENTS: usize = 7;
const AGENT_CAPACITY: usize = 17;
const AGENT_ALIGNMENT: usize = 32;
const REPEATS: usize = 200;
const TOLERANCE: f32 = 5.0e-4;

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
            stable_id: 96_001,
            frame_index: program.model.root.0,
            point_in_frame: [0.13, -0.07, 0.19],
        },
        PointQuerySpec {
            stable_id: 96_002,
            frame_index: 1.min(last),
            point_in_frame: [-0.04, 0.08, 0.11],
        },
        PointQuerySpec {
            stable_id: 96_003,
            frame_index: last / 2,
            point_in_frame: [0.03, -0.05, -0.09],
        },
        PointQuerySpec {
            stable_id: 96_004,
            frame_index: last,
            point_in_frame: [0.06, 0.02, -0.03],
        },
    ];
    let tasks = [
        PointAttractorSpec {
            stable_id: 96_101,
            point_query_stable_id: 96_001,
            priority: Priority::Viability,
            weight: 1.5,
            bandwidth_hz: 1.5,
        },
        PointAttractorSpec {
            stable_id: 96_102,
            point_query_stable_id: 96_004,
            priority: Priority::Intent,
            weight: 0.75,
            bandwidth_hz: 2.25,
        },
    ];
    let contacts = [
        ContactLockSpec {
            stable_id: 96_201,
            point_query_stable_id: 96_001,
        },
        ContactLockSpec {
            stable_id: 96_202,
            point_query_stable_id: 96_002,
        },
        ContactLockSpec {
            stable_id: 96_203,
            point_query_stable_id: 96_003,
        },
        ContactLockSpec {
            stable_id: 96_204,
            point_query_stable_id: 96_004,
        },
    ];
    let modes = [
        ContactKinematicMode::LockedPoint,
        ContactKinematicMode::NormalPoint,
        ContactKinematicMode::RollingPoint,
        ContactKinematicMode::Disabled,
    ];
    let cpu = CpuMirrorExecutor::compile_with_contact_modes(
        &program,
        AGENT_CAPACITY,
        AGENT_ALIGNMENT,
        &queries,
        &tasks,
        &contacts,
        &modes,
    )?;
    let layout = cpu.descriptor().layout.clone();
    let (fk_input, dynamics_input) = deterministic_state_inputs(layout.clone())?;
    let emission_input = deterministic_emission_input(layout.clone())?;
    let mut cpu_outputs = Outputs::new(&layout);
    execute_cpu(
        &cpu,
        &fk_input,
        &dynamics_input,
        &emission_input,
        &mut cpu_outputs,
    )?;
    let oracle = cpu_row_oracle(&cpu, &dynamics_input, &emission_input, &cpu_outputs);
    let first = emission_bytes(&cpu_outputs.emission);
    let mut repeated = Outputs::new(&layout);
    let mut repeat_exact = true;
    for _ in 0..REPEATS {
        execute_cpu(
            &cpu,
            &fk_input,
            &dynamics_input,
            &emission_input,
            &mut repeated,
        )?;
        repeat_exact &= emission_bytes(&repeated.emission) == first;
    }

    let mut poisoned_input = emission_input.clone();
    poisoned_input.point_task_active_soa_mut()[layout.task_active_index(0, 2)] = 2;
    poisoned_input.point_target_position_soa_mut()[layout.task_vector_index(0, 0, 3)] = f32::NAN;
    let mut poisoned = Outputs::new(&layout);
    execute_cpu(
        &cpu,
        &fk_input,
        &dynamics_input,
        &poisoned_input,
        &mut poisoned,
    )?;
    let invalid_typed = [2, 3].into_iter().all(|agent| {
        poisoned.emission.agent_status()[agent] == AgentStatus::InvalidInput
            && emission_agent_zero(&layout, agent, &poisoned.emission)
    });
    let neighbor_isolation = (0..ACTIVE_AGENTS)
        .filter(|agent| ![2, 3].contains(agent))
        .all(|agent| {
            emission_agent_equal(&layout, agent, &cpu_outputs.emission, &poisoned.emission)
        });
    let padding_zero = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| emission_agent_zero(&layout, agent, &cpu_outputs.emission));
    let inactive_rows_zero =
        inactive_and_mode_filtered_rows_zero(&cpu, &emission_input, &cpu_outputs.emission);
    let cpu_pass = repeat_exact
        && oracle["pass"].as_bool().unwrap_or(false)
        && invalid_typed
        && neighbor_isolation
        && padding_zero
        && inactive_rows_zero;

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
    let emission_compiler = probe_cuda_emission_compiler(target[0], target[1]);
    let compiler_available = [
        &fk_compiler,
        &jacobian_compiler,
        &dynamics_compiler,
        &point_compiler,
        &emission_compiler,
    ]
    .into_iter()
    .all(|probe| probe.status == CudaFkComCompilerStatus::Available);
    let common = json!({
        "schema": 1,
        "revision": "cuda-emission-r96",
        "model": model_path,
        "capabilities": capabilities(),
        "kernel_manifest_bits": KernelManifest::emission_v1().bits(),
        "source_sha256": {
            "fk_com": hex(cuda_fk_com_source_sha256()),
            "jacobians": hex(cuda_jacobians_source_sha256()),
            "dynamics": hex(cuda_dynamics_source_sha256()),
            "point_queries": hex(cuda_point_queries_source_sha256()),
            "emission": hex(cuda_emission_source_sha256()),
        },
        "compiler_probes": {
            "fk_com": fk_compiler, "jacobians": jacobian_compiler,
            "dynamics": dynamics_compiler, "point_queries": point_compiler,
            "emission": emission_compiler,
        },
        "runtime_probe": runtime,
        "layout": layout,
        "point_queries": cpu.descriptor().point_queries,
        "point_tasks": cpu.descriptor().point_tasks,
        "contact_locks": cpu.descriptor().contact_locks,
        "cpu_mirror_checks": {
            "pass": cpu_pass,
            "d1_emission_bytes_exact": repeat_exact,
            "independent_row_oracle": oracle,
            "malformed_mask_and_nan_typed": invalid_typed,
            "invalid_neighbor_isolation_exact": neighbor_isolation,
            "inactive_and_mode_filtered_rows_zero": inactive_rows_zero,
            "padding_inactive_and_zero": padding_zero,
            "allocation_free_hot_path_unit_gate": true,
            "repeats": REPEATS,
        },
        "cuda_graph": "not_implemented",
        "hierarchical_solve": "not_implemented",
        "full_cuda_mirror": "not_implemented",
        "hidden_cpu_fallback": false,
    });
    if common["runtime_probe"]["status"]["kind"] != "available" || !compiler_available {
        let mut report = common;
        report["status"] = json!("unavailable");
        report["claim"] = json!(
            "StateInput through fixed point-task/contact row emission CUDA pipeline implemented; absent compiler/device gates are not inferred"
        );
        report["device_checks"] = Value::Null;
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }

    let mut executor = match CudaMirrorEmissionExecutor::compile(&program, &cpu, 0) {
        Ok(executor) => executor,
        Err(error) => {
            let mut report = common;
            report["status"] = json!("failed");
            report["device_error"] = json!(error.to_string());
            println!("{}", serde_json::to_string_pretty(&report)?);
            bail!("CUDA emission executor construction failed: {error}");
        }
    };
    let mut device_outputs = Outputs::new(&layout);
    execute_device(
        &mut executor,
        &fk_input,
        &dynamics_input,
        &emission_input,
        &mut device_outputs,
    )?;
    let d3 = compare_emission(&cpu_outputs.emission, &device_outputs.emission);
    let first_device = emission_bytes(&device_outputs.emission);
    let mut d1_exact = true;
    let mut timings_us = Vec::with_capacity(REPEATS);
    for _ in 0..REPEATS {
        let start = Instant::now();
        execute_device(
            &mut executor,
            &fk_input,
            &dynamics_input,
            &emission_input,
            &mut device_outputs,
        )?;
        timings_us.push(start.elapsed().as_secs_f64() * 1e6);
        d1_exact &= emission_bytes(&device_outputs.emission) == first_device;
    }
    timings_us.sort_by(f64::total_cmp);
    let mut device_poisoned = Outputs::new(&layout);
    execute_device(
        &mut executor,
        &fk_input,
        &dynamics_input,
        &poisoned_input,
        &mut device_poisoned,
    )?;
    let device_invalid = [2, 3].into_iter().all(|agent| {
        device_poisoned.emission.agent_status()[agent] == AgentStatus::InvalidInput
            && emission_agent_zero(&layout, agent, &device_poisoned.emission)
    });
    let device_neighbors = (0..ACTIVE_AGENTS)
        .filter(|agent| ![2, 3].contains(agent))
        .all(|agent| {
            emission_agent_equal(
                &layout,
                agent,
                &device_outputs.emission,
                &device_poisoned.emission,
            )
        });
    let device_padding = (ACTIVE_AGENTS..layout.agent_stride)
        .all(|agent| emission_agent_zero(&layout, agent, &device_outputs.emission));
    let passed = cpu_pass
        && d1_exact
        && d3["pass"].as_bool().unwrap_or(false)
        && device_invalid
        && device_neighbors
        && device_padding;
    let mut report = common;
    report["status"] = json!(if passed { "pass" } else { "fail" });
    report["claim"] =
        json!("direct-launch CudaMirrorF32 through fixed point-task/contact emission only");
    report["device"] = json!(executor.device());
    report["fingerprint"] = json!(executor.fingerprint());
    report["nvrtc_log"] = json!(executor.nvrtc_log());
    report["device_checks"] = json!({
        "d1_emission_bytes_exact": d1_exact,
        "d3_cpu_mirror": d3,
        "malformed_mask_and_nan_typed": device_invalid,
        "invalid_neighbor_isolation_exact": device_neighbors,
        "padding_inactive_and_zero": device_padding,
        "repeats": REPEATS,
        "single_stream_direct_launch_end_to_end_us": distribution(&timings_us),
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !passed {
        bail!("CUDA emission conformance failed");
    }
    Ok(())
}

struct Outputs {
    fk: FkBatchOutput,
    jacobian: JacobianBatchOutput,
    dynamics: DynamicsBatchOutput,
    points: PointQueryBatchOutput,
    emission: EmissionBatchOutput,
}

impl Outputs {
    fn new(layout: &BatchLayout) -> Self {
        Self {
            fk: FkBatchOutput::new(layout.clone()),
            jacobian: JacobianBatchOutput::new(layout.clone()),
            dynamics: DynamicsBatchOutput::new(layout.clone()),
            points: PointQueryBatchOutput::new(layout.clone()),
            emission: EmissionBatchOutput::new(layout.clone()),
        }
    }
}

fn deterministic_state_inputs(layout: BatchLayout) -> Result<(FkBatchInput, DynamicsBatchInput)> {
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

fn deterministic_emission_input(layout: BatchLayout) -> Result<EmissionBatchInput> {
    let mut input = EmissionBatchInput::new(layout.clone(), ACTIVE_AGENTS)?;
    for agent in 0..ACTIVE_AGENTS {
        for slot in 0..layout.point_task_count {
            let active = slot == 0 || agent % 2 == 0;
            input.point_task_active_soa_mut()[layout.task_active_index(slot, agent)] =
                u8::from(active);
            for component in 0..3 {
                let index = layout.task_vector_index(slot, component, agent);
                input.point_target_position_soa_mut()[index] =
                    0.12 + 0.001 * agent as f32 + 0.03 * slot as f32 + 0.01 * component as f32;
                input.point_target_velocity_soa_mut()[index] =
                    -0.04 + 0.002 * agent as f32 - 0.01 * slot as f32 + 0.005 * component as f32;
                input.point_target_acceleration_soa_mut()[index] =
                    0.03 - 0.001 * agent as f32 + 0.02 * slot as f32 - 0.004 * component as f32;
            }
        }
        for slot in 0..layout.contact_lock_count {
            let active = match slot {
                0 | 3 => true,
                1 => agent % 2 == 0,
                _ => agent % 3 == 0,
            };
            input.contact_lock_active_soa_mut()[layout.contact_active_index(slot, agent)] =
                u8::from(active);
            for component in 0..3 {
                input.contact_desired_acceleration_soa_mut()
                    [layout.contact_vector_index(slot, component, agent)] =
                    0.005 * agent as f32 + 0.01 * slot as f32 - 0.002 * component as f32;
            }
        }
    }
    Ok(input)
}

fn execute_cpu(
    cpu: &CpuMirrorExecutor,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    emission_input: &EmissionBatchInput,
    outputs: &mut Outputs,
) -> Result<()> {
    cpu.execute_into(fk_input, &mut outputs.fk)?;
    cpu.execute_jacobians_into(&outputs.fk, &mut outputs.jacobian)?;
    cpu.execute_dynamics_into(
        dynamics_input,
        &outputs.fk,
        &outputs.jacobian,
        &mut outputs.dynamics,
    )?;
    cpu.execute_point_queries_into(
        &outputs.fk,
        &outputs.jacobian,
        &outputs.dynamics,
        &mut outputs.points,
    )?;
    cpu.execute_emission_into(
        emission_input,
        dynamics_input,
        &outputs.points,
        &mut outputs.emission,
    )?;
    Ok(())
}

fn execute_device(
    executor: &mut CudaMirrorEmissionExecutor,
    fk_input: &FkBatchInput,
    dynamics_input: &DynamicsBatchInput,
    emission_input: &EmissionBatchInput,
    outputs: &mut Outputs,
) -> Result<()> {
    executor.execute_into(
        fk_input,
        dynamics_input,
        emission_input,
        &mut outputs.fk,
        &mut outputs.jacobian,
        &mut outputs.dynamics,
        &mut outputs.points,
        &mut outputs.emission,
    )?;
    Ok(())
}

fn cpu_row_oracle(
    cpu: &CpuMirrorExecutor,
    dynamics: &DynamicsBatchInput,
    input: &EmissionBatchInput,
    outputs: &Outputs,
) -> Value {
    let layout = &cpu.descriptor().layout;
    let mut maximum = 0.0_f32;
    for agent in 0..ACTIVE_AGENTS {
        for (slot, task) in cpu.descriptor().point_tasks.iter().enumerate() {
            if input.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0 {
                continue;
            }
            let position = outputs
                .points
                .point_position(agent, task.point_query_slot)
                .unwrap();
            let bias = outputs
                .points
                .point_bias_acceleration(agent, task.point_query_slot)
                .unwrap();
            let omega = std::f32::consts::TAU * task.bandwidth_hz();
            for component in 0..3 {
                let mut velocity = 0.0_f32;
                for coordinate in 0..layout.generalized_coordinate_count {
                    let jacobian = outputs
                        .points
                        .point_jacobian_value(agent, task.point_query_slot, component, coordinate)
                        .unwrap();
                    velocity = jacobian.mul_add(
                        dynamics.generalized_velocity_soa()
                            [layout.generalized_index(coordinate, agent)],
                        velocity,
                    );
                    maximum = maximum.max(
                        (outputs.emission.point_task_jacobian_soa()
                            [layout.task_jacobian_index(slot, component, coordinate, agent)]
                            - jacobian)
                            .abs(),
                    );
                }
                let target_position =
                    0.12 + 0.001 * agent as f32 + 0.03 * slot as f32 + 0.01 * component as f32;
                let target_velocity =
                    -0.04 + 0.002 * agent as f32 - 0.01 * slot as f32 + 0.005 * component as f32;
                let target_acceleration =
                    0.03 - 0.001 * agent as f32 + 0.02 * slot as f32 - 0.004 * component as f32;
                let position_error = target_position - position[component];
                let velocity_error = target_velocity - velocity;
                let desired = (omega * omega).mul_add(
                    position_error,
                    (2.0 * omega).mul_add(velocity_error, target_acceleration),
                );
                let index = layout.task_vector_index(slot, component, agent);
                maximum = maximum.max(
                    (outputs.emission.point_task_position_error_soa()[index] - position_error)
                        .abs(),
                );
                maximum = maximum.max(
                    (outputs.emission.point_task_velocity_error_soa()[index] - velocity_error)
                        .abs(),
                );
                maximum = maximum.max(
                    (outputs.emission.point_task_desired_acceleration_soa()[index] - desired).abs(),
                );
                maximum = maximum.max(
                    (outputs.emission.point_task_rhs_soa()[index] - (desired - bias[component]))
                        .abs(),
                );
            }
        }
        for (slot, contact) in cpu.descriptor().contact_locks.iter().enumerate() {
            if input.contact_lock_active_soa()[layout.contact_active_index(slot, agent)] == 0 {
                continue;
            }
            let bias = outputs
                .points
                .point_bias_acceleration(agent, contact.point_query_slot)
                .unwrap();
            for component in 0..3 {
                let enabled = contact.kinematic_mode.axis_enabled(component);
                let desired = 0.005 * agent as f32 + 0.01 * slot as f32 - 0.002 * component as f32;
                let rhs = if enabled {
                    desired - bias[component]
                } else {
                    0.0
                };
                maximum = maximum.max(
                    (outputs.emission.contact_lock_rhs_soa()
                        [layout.contact_vector_index(slot, component, agent)]
                        - rhs)
                        .abs(),
                );
                for coordinate in 0..layout.generalized_coordinate_count {
                    let expected = if enabled {
                        outputs
                            .points
                            .point_jacobian_value(
                                agent,
                                contact.point_query_slot,
                                component,
                                coordinate,
                            )
                            .unwrap()
                    } else {
                        0.0
                    };
                    maximum = maximum.max(
                        (outputs.emission.contact_lock_jacobian_soa()
                            [layout.contact_jacobian_index(slot, component, coordinate, agent)]
                            - expected)
                            .abs(),
                    );
                }
            }
        }
    }
    json!({"pass": maximum <= 1.0e-5, "maximum_absolute_error": maximum, "tolerance": 1.0e-5})
}

fn emission_bytes(output: &EmissionBatchOutput) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend(output.point_task_active_soa());
    for values in [
        output.point_task_position_error_soa(),
        output.point_task_velocity_error_soa(),
        output.point_task_desired_acceleration_soa(),
        output.point_task_jacobian_soa(),
        output.point_task_rhs_soa(),
        output.contact_lock_jacobian_soa(),
        output.contact_lock_rhs_soa(),
    ] {
        for value in values {
            bytes.extend(value.to_bits().to_le_bytes());
        }
    }
    bytes.extend(output.contact_lock_active_soa());
    bytes.extend(output.agent_status().iter().map(|status| *status as u8));
    bytes
}

fn emission_agent_zero(layout: &BatchLayout, agent: usize, output: &EmissionBatchOutput) -> bool {
    output.agent_status()[agent] != AgentStatus::Ok
        && output
            .point_task_active_soa()
            .iter()
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, value)| *value == 0)
        && output
            .contact_lock_active_soa()
            .iter()
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, value)| *value == 0)
        && [
            output.point_task_position_error_soa(),
            output.point_task_velocity_error_soa(),
            output.point_task_desired_acceleration_soa(),
            output.point_task_jacobian_soa(),
            output.point_task_rhs_soa(),
            output.contact_lock_jacobian_soa(),
            output.contact_lock_rhs_soa(),
        ]
        .into_iter()
        .all(|values| {
            values
                .iter()
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride == agent)
                .all(|(_, value)| value.to_bits() == 0)
        })
}

fn emission_agent_equal(
    layout: &BatchLayout,
    agent: usize,
    left: &EmissionBatchOutput,
    right: &EmissionBatchOutput,
) -> bool {
    left.agent_status()[agent] == right.agent_status()[agent]
        && left
            .point_task_active_soa()
            .iter()
            .zip(right.point_task_active_soa())
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, (left, right))| left == right)
        && left
            .contact_lock_active_soa()
            .iter()
            .zip(right.contact_lock_active_soa())
            .enumerate()
            .filter(|(index, _)| index % layout.agent_stride == agent)
            .all(|(_, (left, right))| left == right)
        && [
            (
                left.point_task_position_error_soa(),
                right.point_task_position_error_soa(),
            ),
            (
                left.point_task_velocity_error_soa(),
                right.point_task_velocity_error_soa(),
            ),
            (
                left.point_task_desired_acceleration_soa(),
                right.point_task_desired_acceleration_soa(),
            ),
            (
                left.point_task_jacobian_soa(),
                right.point_task_jacobian_soa(),
            ),
            (left.point_task_rhs_soa(), right.point_task_rhs_soa()),
            (
                left.contact_lock_jacobian_soa(),
                right.contact_lock_jacobian_soa(),
            ),
            (left.contact_lock_rhs_soa(), right.contact_lock_rhs_soa()),
        ]
        .into_iter()
        .all(|(left, right)| {
            left.iter()
                .zip(right)
                .enumerate()
                .filter(|(index, _)| index % layout.agent_stride == agent)
                .all(|(_, (left, right))| left.to_bits() == right.to_bits())
        })
}

fn inactive_and_mode_filtered_rows_zero(
    cpu: &CpuMirrorExecutor,
    input: &EmissionBatchInput,
    output: &EmissionBatchOutput,
) -> bool {
    let layout = &cpu.descriptor().layout;
    for agent in 0..ACTIVE_AGENTS {
        for slot in 0..layout.point_task_count {
            if input.point_task_active_soa()[layout.task_active_index(slot, agent)] == 0 {
                if output.point_task_active_soa()[layout.task_active_index(slot, agent)] != 0 {
                    return false;
                }
                for component in 0..3 {
                    let vector_index = layout.task_vector_index(slot, component, agent);
                    if [
                        output.point_task_position_error_soa()[vector_index],
                        output.point_task_velocity_error_soa()[vector_index],
                        output.point_task_desired_acceleration_soa()[vector_index],
                        output.point_task_rhs_soa()[vector_index],
                    ]
                    .into_iter()
                    .any(|value| value.to_bits() != 0)
                    {
                        return false;
                    }
                    for coordinate in 0..layout.generalized_coordinate_count {
                        if output.point_task_jacobian_soa()
                            [layout.task_jacobian_index(slot, component, coordinate, agent)]
                        .to_bits()
                            != 0
                        {
                            return false;
                        }
                    }
                }
            }
        }
        for (slot, contact) in cpu.descriptor().contact_locks.iter().enumerate() {
            for component in 0..3 {
                if !contact.kinematic_mode.axis_enabled(component) {
                    if output.contact_lock_rhs_soa()
                        [layout.contact_vector_index(slot, component, agent)]
                    .to_bits()
                        != 0
                    {
                        return false;
                    }
                    for coordinate in 0..layout.generalized_coordinate_count {
                        if output.contact_lock_jacobian_soa()
                            [layout.contact_jacobian_index(slot, component, coordinate, agent)]
                        .to_bits()
                            != 0
                        {
                            return false;
                        }
                    }
                }
            }
        }
    }
    true
}

fn compare_emission(expected: &EmissionBatchOutput, actual: &EmissionBatchOutput) -> Value {
    let compare = |left: &[f32], right: &[f32]| {
        left.iter().zip(right).fold(0.0_f32, |max, (left, right)| {
            max.max((left - right).abs() / (1.0 + left.abs().max(right.abs())))
        })
    };
    let fields = [
        compare(
            expected.point_task_position_error_soa(),
            actual.point_task_position_error_soa(),
        ),
        compare(
            expected.point_task_velocity_error_soa(),
            actual.point_task_velocity_error_soa(),
        ),
        compare(
            expected.point_task_desired_acceleration_soa(),
            actual.point_task_desired_acceleration_soa(),
        ),
        compare(
            expected.point_task_jacobian_soa(),
            actual.point_task_jacobian_soa(),
        ),
        compare(expected.point_task_rhs_soa(), actual.point_task_rhs_soa()),
        compare(
            expected.contact_lock_jacobian_soa(),
            actual.contact_lock_jacobian_soa(),
        ),
        compare(
            expected.contact_lock_rhs_soa(),
            actual.contact_lock_rhs_soa(),
        ),
    ];
    let maximum = fields.into_iter().fold(0.0_f32, f32::max);
    json!({"pass": expected.agent_status() == actual.agent_status() && expected.point_task_active_soa() == actual.point_task_active_soa() && expected.contact_lock_active_soa() == actual.contact_lock_active_soa() && maximum <= TOLERANCE, "maximum_abs_rel": maximum, "tolerance_abs_plus_rel": TOLERANCE})
}

fn distribution(values: &[f64]) -> Value {
    json!({"min": values[0], "p50": values[values.len()/2], "p99": values[((values.len() as f64 * 0.99) as usize).min(values.len()-1)], "max": values[values.len()-1]})
}
fn hex(bytes: [u8; 32]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}
