//! Direct CUDA state-input + forward-kinematics + center-of-mass executor.
//!
//! The model tree is packed and uploaded once. Each launch assigns one full
//! agent to one CUDA thread, preserving the CPU mirror's per-agent arithmetic
//! order and avoiding inter-agent reductions, atomics, and runtime allocation.
//! Construction or execution failure is returned directly; there is no CPU
//! fallback behind this API.

use bonesaw_core::MotionProgram;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{
    AgentStatus, BackendFingerprint, BackendProfile, BatchLayout, CpuMirrorBatchSolver,
    CpuMirrorExecutor, CudaDeviceInfo, CudaKinematicsModel, CudaKinematicsModelError,
    DynamicsBatchInput, DynamicsBatchOutput, EmissionBatchInput, EmissionBatchOutput, FkBatchInput,
    FkBatchOutput, JacobianBatchOutput, MirrorSolveAgentStatus, MirrorSolveBatchInput,
    MirrorSolveBatchOutput, NvrtcError, PointQueryBatchOutput, ScalarFormat, Version,
};

#[cfg(not(target_os = "linux"))]
use crate::cuda_driver::DriverError;
#[cfg(target_os = "linux")]
use crate::{
    CUDA_DYNAMICS_SOURCE, CUDA_EMISSION_SOURCE, CUDA_FK_COM_SOURCE, CUDA_JACOBIANS_SOURCE,
    CUDA_POINT_QUERIES_SOURCE, CUDA_SOLVE_SOURCE,
    cuda_driver::{CuContext, CuDevicePtr, CuFunction, CuModule, Driver, DriverError},
    cuda_nvrtc::NvrtcCompiler,
    cuda_state_input::{
        BLOCK_SIZE, STATE_INPUT_FUNCTION, STATE_INPUT_PTX, query_device_info, version_from_driver,
    },
};

const FK_COM_FUNCTION: &str = "bonesaw_fk_com_v1";
const JACOBIANS_FUNCTION: &str = "bonesaw_jacobians_v1";
const DYNAMICS_FUNCTION: &str = "bonesaw_dynamics_v1";
const POINT_QUERIES_FUNCTION: &str = "bonesaw_point_queries_v1";
const EMISSION_FUNCTION: &str = "bonesaw_emission_v1";
const SOLVE_FUNCTION: &str = "bonesaw_solve_v1";

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CudaFkComCompilerStatus {
    Available,
    UnsupportedPlatform,
    LibraryUnavailable,
    SymbolUnavailable,
    VersionQueryFailed,
    CompileFailed,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct CudaFkComCompilerProbe {
    pub status: CudaFkComCompilerStatus,
    pub detail: Option<String>,
    pub toolkit_version: Option<Version>,
    pub target_compute_capability: [i32; 2],
    pub ptx_sha256: Option<[u8; 32]>,
    pub ptx_bytes: Option<usize>,
    pub compiler_log: Option<String>,
}

/// Compile the generic FK/CoM source without requiring a CUDA device.
///
/// This separates host compiler evidence from runtime-device evidence. The
/// caller chooses the virtual compute target; no device availability is
/// inferred from a successful NVRTC compilation.
pub fn probe_cuda_fk_com_compiler(
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_FK_COM_SOURCE,
        "bonesaw_fk_com_v1.cu",
        compute_major,
        compute_minor,
    )
}

/// Compile the generic frame/CoM Jacobian source without requiring a device.
pub fn probe_cuda_jacobians_compiler(
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_JACOBIANS_SOURCE,
        "bonesaw_jacobians_v1.cu",
        compute_major,
        compute_minor,
    )
}

/// Compile the generic floating M/h/Ag source without requiring a device.
pub fn probe_cuda_dynamics_compiler(
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_DYNAMICS_SOURCE,
        "bonesaw_dynamics_v1.cu",
        compute_major,
        compute_minor,
    )
}

/// Compile fixed point position/J/Jdot-v source without requiring a device.
pub fn probe_cuda_point_queries_compiler(
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_POINT_QUERIES_SOURCE,
        "bonesaw_point_queries_v1.cu",
        compute_major,
        compute_minor,
    )
}

/// Compile fixed point-task/contact row emission without requiring a device.
pub fn probe_cuda_emission_compiler(
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_EMISSION_SOURCE,
        "bonesaw_emission_v1.cu",
        compute_major,
        compute_minor,
    )
}

/// Compile the fixed-level hierarchical solve source without requiring a
/// device. Compiler success is not device conformance.
pub fn probe_cuda_solve_compiler(compute_major: i32, compute_minor: i32) -> CudaFkComCompilerProbe {
    probe_cuda_source_compiler(
        CUDA_SOLVE_SOURCE,
        "bonesaw_solve_v1.cu",
        compute_major,
        compute_minor,
    )
}

fn probe_cuda_source_compiler(
    source: &str,
    source_name: &str,
    compute_major: i32,
    compute_minor: i32,
) -> CudaFkComCompilerProbe {
    let target_compute_capability = [compute_major, compute_minor];
    #[cfg(target_os = "linux")]
    {
        let compiler = match NvrtcCompiler::load() {
            Ok(compiler) => compiler,
            Err(error) => {
                return compiler_probe_error(error, None, target_compute_capability, false);
            }
        };
        let (major, minor) = match compiler.version() {
            Ok(version) => version,
            Err(error) => {
                return compiler_probe_error(error, None, target_compute_capability, false);
            }
        };
        let toolkit_version = Version {
            major: major.max(0) as u32,
            minor: minor.max(0) as u32,
            patch: 0,
        };
        match compiler.compile(source, source_name, compute_major, compute_minor) {
            Ok((ptx, log)) => CudaFkComCompilerProbe {
                status: CudaFkComCompilerStatus::Available,
                detail: None,
                toolkit_version: Some(toolkit_version),
                target_compute_capability,
                ptx_sha256: Some(Sha256::digest(ptx.as_bytes()).into()),
                ptx_bytes: Some(ptx.len()),
                compiler_log: (!log.is_empty()).then_some(log),
            },
            Err(error) => compiler_probe_error(
                error,
                Some(toolkit_version),
                target_compute_capability,
                true,
            ),
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (source, source_name);
        CudaFkComCompilerProbe {
            status: CudaFkComCompilerStatus::UnsupportedPlatform,
            detail: Some("dynamic NVRTC loading is currently supported only on Linux".to_owned()),
            toolkit_version: None,
            target_compute_capability,
            ptx_sha256: None,
            ptx_bytes: None,
            compiler_log: None,
        }
    }
}

#[cfg(target_os = "linux")]
fn compiler_probe_error(
    error: NvrtcError,
    toolkit_version: Option<Version>,
    target_compute_capability: [i32; 2],
    compiling: bool,
) -> CudaFkComCompilerProbe {
    let status = match &error {
        NvrtcError::LibraryUnavailable => CudaFkComCompilerStatus::LibraryUnavailable,
        NvrtcError::MissingSymbol(_) => CudaFkComCompilerStatus::SymbolUnavailable,
        _ if compiling => CudaFkComCompilerStatus::CompileFailed,
        _ => CudaFkComCompilerStatus::VersionQueryFailed,
    };
    CudaFkComCompilerProbe {
        status,
        detail: Some(error.to_string()),
        toolkit_version,
        target_compute_capability,
        ptx_sha256: None,
        ptx_bytes: None,
        compiler_log: match error {
            NvrtcError::Call { log, .. } if !log.is_empty() => Some(log),
            _ => None,
        },
    }
}

#[derive(Debug, Error)]
pub enum CudaFkComError {
    #[error(transparent)]
    Driver(#[from] DriverError),
    #[error(transparent)]
    Nvrtc(#[from] NvrtcError),
    #[error(transparent)]
    Model(#[from] CudaKinematicsModelError),
    #[error("program does not match the compiled CPU mirror")]
    ProgramMismatch,
    #[error("input or output layout does not match the CUDA executor")]
    LayoutMismatch,
    #[error("CUDA buffer size overflows usize")]
    SizeOverflow,
    #[error("CUDA dimensions exceed the u32 kernel ABI")]
    DimensionOverflow,
    #[error("CUDA FK/CoM kernel returned unknown agent status byte {0}")]
    InvalidStatusByte(u8),
    #[error("CUDA Jacobian stage was not compiled for this executor")]
    JacobiansUnavailable,
    #[error("CUDA dynamics stage was not compiled for this executor")]
    DynamicsUnavailable,
    #[error("CUDA point-query stage was not compiled for this executor")]
    PointQueriesUnavailable,
    #[error("CUDA emission stage was not compiled for this executor")]
    EmissionUnavailable,
    #[error("CUDA hierarchical solve stage was not compiled for this executor")]
    SolveUnavailable,
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct DeviceModel {
    joint_parent: CuDevicePtr,
    joint_child: CuDevicePtr,
    joint_coordinate: CuDevicePtr,
    joint_kind: CuDevicePtr,
    joint_parent_from_joint: CuDevicePtr,
    joint_axis: CuDevicePtr,
    body_parent_joint: CuDevicePtr,
    body_mass: CuDevicePtr,
    body_com: CuDevicePtr,
    body_inertia: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl DeviceModel {
    const ZERO: Self = Self {
        joint_parent: 0,
        joint_child: 0,
        joint_coordinate: 0,
        joint_kind: 0,
        joint_parent_from_joint: 0,
        joint_axis: 0,
        body_parent_joint: 0,
        body_mass: 0,
        body_com: 0,
        body_inertia: 0,
    };

    fn pointers(&self) -> [CuDevicePtr; 10] {
        [
            self.joint_parent,
            self.joint_child,
            self.joint_coordinate,
            self.joint_kind,
            self.joint_parent_from_joint,
            self.joint_axis,
            self.body_parent_joint,
            self.body_mass,
            self.body_com,
            self.body_inertia,
        ]
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct DynamicsStage {
    module: CuModule,
    function: CuFunction,
    generalized_velocity: CuDevicePtr,
    gravity_world: CuDevicePtr,
    angular_velocity: CuDevicePtr,
    angular_acceleration: CuDevicePtr,
    linear_velocity_origin: CuDevicePtr,
    linear_acceleration_origin: CuDevicePtr,
    mass_matrix: CuDevicePtr,
    bias_force: CuDevicePtr,
    centroidal_map: CuDevicePtr,
    output_status: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl DynamicsStage {
    const ZERO: Self = Self {
        module: core::ptr::null_mut(),
        function: core::ptr::null_mut(),
        generalized_velocity: 0,
        gravity_world: 0,
        angular_velocity: 0,
        angular_acceleration: 0,
        linear_velocity_origin: 0,
        linear_acceleration_origin: 0,
        mass_matrix: 0,
        bias_force: 0,
        centroidal_map: 0,
        output_status: 0,
    };

    fn destroy(&mut self, driver: &Driver) {
        driver.free(self.output_status);
        driver.free(self.centroidal_map);
        driver.free(self.bias_force);
        driver.free(self.mass_matrix);
        driver.free(self.linear_acceleration_origin);
        driver.free(self.linear_velocity_origin);
        driver.free(self.angular_acceleration);
        driver.free(self.angular_velocity);
        driver.free(self.gravity_world);
        driver.free(self.generalized_velocity);
        driver.unload_module(self.module);
        *self = Self::ZERO;
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct PointQueryStage {
    module: CuModule,
    function: CuFunction,
    query_frame: CuDevicePtr,
    query_point_in_frame: CuDevicePtr,
    point_position: CuDevicePtr,
    point_jacobian: CuDevicePtr,
    point_bias_acceleration: CuDevicePtr,
    output_status: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl PointQueryStage {
    const ZERO: Self = Self {
        module: core::ptr::null_mut(),
        function: core::ptr::null_mut(),
        query_frame: 0,
        query_point_in_frame: 0,
        point_position: 0,
        point_jacobian: 0,
        point_bias_acceleration: 0,
        output_status: 0,
    };

    fn destroy(&mut self, driver: &Driver) {
        driver.free(self.output_status);
        driver.free(self.point_bias_acceleration);
        driver.free(self.point_jacobian);
        driver.free(self.point_position);
        driver.free(self.query_point_in_frame);
        driver.free(self.query_frame);
        driver.unload_module(self.module);
        *self = Self::ZERO;
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct EmissionStage {
    module: CuModule,
    function: CuFunction,
    task_query_slot: CuDevicePtr,
    task_bandwidth_hz: CuDevicePtr,
    contact_query_slot: CuDevicePtr,
    contact_mode: CuDevicePtr,
    task_active_input: CuDevicePtr,
    target_position: CuDevicePtr,
    target_velocity: CuDevicePtr,
    target_acceleration: CuDevicePtr,
    contact_active_input: CuDevicePtr,
    contact_desired_acceleration: CuDevicePtr,
    task_active_output: CuDevicePtr,
    task_position_error: CuDevicePtr,
    task_velocity_error: CuDevicePtr,
    task_desired_acceleration: CuDevicePtr,
    task_jacobian: CuDevicePtr,
    task_rhs: CuDevicePtr,
    contact_active_output: CuDevicePtr,
    contact_jacobian: CuDevicePtr,
    contact_rhs: CuDevicePtr,
    output_status: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl EmissionStage {
    const ZERO: Self = Self {
        module: core::ptr::null_mut(),
        function: core::ptr::null_mut(),
        task_query_slot: 0,
        task_bandwidth_hz: 0,
        contact_query_slot: 0,
        contact_mode: 0,
        task_active_input: 0,
        target_position: 0,
        target_velocity: 0,
        target_acceleration: 0,
        contact_active_input: 0,
        contact_desired_acceleration: 0,
        task_active_output: 0,
        task_position_error: 0,
        task_velocity_error: 0,
        task_desired_acceleration: 0,
        task_jacobian: 0,
        task_rhs: 0,
        contact_active_output: 0,
        contact_jacobian: 0,
        contact_rhs: 0,
        output_status: 0,
    };
    fn destroy(&mut self, driver: &Driver) {
        for pointer in [
            self.output_status,
            self.contact_rhs,
            self.contact_jacobian,
            self.contact_active_output,
            self.task_rhs,
            self.task_jacobian,
            self.task_desired_acceleration,
            self.task_velocity_error,
            self.task_position_error,
            self.task_active_output,
            self.contact_desired_acceleration,
            self.contact_active_input,
            self.target_acceleration,
            self.target_velocity,
            self.target_position,
            self.task_active_input,
            self.contact_mode,
            self.contact_query_slot,
            self.task_bandwidth_hz,
            self.task_query_slot,
        ] {
            driver.free(pointer);
        }
        driver.unload_module(self.module);
        *self = Self::ZERO;
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct SolveStage {
    module: CuModule,
    function: CuFunction,
    task_priority: CuDevicePtr,
    task_weight: CuDevicePtr,
    contact_mode: CuDevicePtr,
    lower: CuDevicePtr,
    upper: CuDevicePtr,
    command: CuDevicePtr,
    candidate: CuDevicePtr,
    status: CuDevicePtr,
    level_rows: CuDevicePtr,
    level_rms: CuDevicePtr,
    level_preservation_drift: CuDevicePtr,
    initial_hard_violation: CuDevicePtr,
    best_hard_violation: CuDevicePtr,
    final_hard_violation: CuDevicePtr,
    minimum_bound_margin: CuDevicePtr,
    hard_projection_sweeps: CuDevicePtr,
    task_sweeps: CuDevicePtr,
    clipped_updates: CuDevicePtr,
    active_hard_rows: CuDevicePtr,
    active_soft_rows: CuDevicePtr,
    x: CuDevicePtr,
    best: CuDevicePtr,
    locked_rows: CuDevicePtr,
    locked_rhs: CuDevicePtr,
    locked_reference_rhs: CuDevicePtr,
    locked_priority: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl SolveStage {
    const ZERO: Self = Self {
        module: core::ptr::null_mut(),
        function: core::ptr::null_mut(),
        task_priority: 0,
        task_weight: 0,
        contact_mode: 0,
        lower: 0,
        upper: 0,
        command: 0,
        candidate: 0,
        status: 0,
        level_rows: 0,
        level_rms: 0,
        level_preservation_drift: 0,
        initial_hard_violation: 0,
        best_hard_violation: 0,
        final_hard_violation: 0,
        minimum_bound_margin: 0,
        hard_projection_sweeps: 0,
        task_sweeps: 0,
        clipped_updates: 0,
        active_hard_rows: 0,
        active_soft_rows: 0,
        x: 0,
        best: 0,
        locked_rows: 0,
        locked_rhs: 0,
        locked_reference_rhs: 0,
        locked_priority: 0,
    };

    fn destroy(&mut self, driver: &Driver) {
        for pointer in [
            self.locked_priority,
            self.locked_reference_rhs,
            self.locked_rhs,
            self.locked_rows,
            self.best,
            self.x,
            self.active_soft_rows,
            self.active_hard_rows,
            self.clipped_updates,
            self.task_sweeps,
            self.hard_projection_sweeps,
            self.minimum_bound_margin,
            self.final_hard_violation,
            self.best_hard_violation,
            self.initial_hard_violation,
            self.level_preservation_drift,
            self.level_rms,
            self.level_rows,
            self.status,
            self.candidate,
            self.command,
            self.upper,
            self.lower,
            self.contact_mode,
            self.task_weight,
            self.task_priority,
        ] {
            driver.free(pointer);
        }
        driver.unload_module(self.module);
        *self = Self::ZERO;
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct JacobianStage {
    module: CuModule,
    function: CuFunction,
    frame_jacobian: CuDevicePtr,
    center_of_mass_jacobian: CuDevicePtr,
    output_status: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl JacobianStage {
    const ZERO: Self = Self {
        module: core::ptr::null_mut(),
        function: core::ptr::null_mut(),
        frame_jacobian: 0,
        center_of_mass_jacobian: 0,
        output_status: 0,
    };

    fn destroy(&mut self, driver: &Driver) {
        driver.free(self.output_status);
        driver.free(self.center_of_mass_jacobian);
        driver.free(self.frame_jacobian);
        driver.unload_module(self.module);
        *self = Self::ZERO;
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct LinuxExecutor {
    driver: Driver,
    context: CuContext,
    state_module: CuModule,
    state_function: CuFunction,
    fk_module: CuModule,
    fk_function: CuFunction,
    q_input: CuDevicePtr,
    root_input: CuDevicePtr,
    q_state: CuDevicePtr,
    root_state: CuDevicePtr,
    state_status: CuDevicePtr,
    model: DeviceModel,
    body_pose: CuDevicePtr,
    center_of_mass: CuDevicePtr,
    total_mass: CuDevicePtr,
    output_status: CuDevicePtr,
    jacobians: Option<JacobianStage>,
    dynamics: Option<DynamicsStage>,
    point_queries: Option<PointQueryStage>,
    emission: Option<EmissionStage>,
    solve: Option<SolveStage>,
}

#[cfg(target_os = "linux")]
impl Drop for LinuxExecutor {
    fn drop(&mut self) {
        let _ = self.driver.set_current(self.context);
        if let Some(mut emission) = self.emission.take() {
            emission.destroy(&self.driver);
        }
        if let Some(mut solve) = self.solve.take() {
            solve.destroy(&self.driver);
        }
        if let Some(mut point_queries) = self.point_queries.take() {
            point_queries.destroy(&self.driver);
        }
        if let Some(mut dynamics) = self.dynamics.take() {
            dynamics.destroy(&self.driver);
        }
        if let Some(mut jacobians) = self.jacobians.take() {
            jacobians.destroy(&self.driver);
        }
        self.driver.free(self.output_status);
        self.driver.free(self.total_mass);
        self.driver.free(self.center_of_mass);
        self.driver.free(self.body_pose);
        for pointer in self.model.pointers().into_iter().rev() {
            self.driver.free(pointer);
        }
        self.driver.free(self.state_status);
        self.driver.free(self.root_state);
        self.driver.free(self.q_state);
        self.driver.free(self.root_input);
        self.driver.free(self.q_input);
        self.driver.unload_module(self.fk_module);
        self.driver.unload_module(self.state_module);
        self.driver.destroy_context(self.context);
    }
}

/// Fixed-buffer CUDA executor for state validation, tree FK, and system CoM.
#[derive(Debug)]
pub struct CudaMirrorFkComExecutor {
    layout: BatchLayout,
    fingerprint: BackendFingerprint,
    device: CudaDeviceInfo,
    model_hash: [u8; 32],
    root_body: u32,
    nvrtc_log: String,
    status_staging: Vec<u8>,
    #[cfg(target_os = "linux")]
    inner: LinuxExecutor,
}

/// StateInput + FK/CoM + floating frame/CoM Jacobian pipeline.
#[derive(Debug)]
pub struct CudaMirrorKinematicsExecutor {
    inner: CudaMirrorFkComExecutor,
}

/// StateInput + FK/CoM + Jacobians + floating M/h/Ag pipeline.
#[derive(Debug)]
pub struct CudaMirrorDynamicsExecutor {
    inner: CudaMirrorFkComExecutor,
}

/// StateInput + FK/CoM + Jacobians + Dynamics + fixed point products.
#[derive(Debug)]
pub struct CudaMirrorPointQueriesExecutor {
    inner: CudaMirrorFkComExecutor,
}

/// StateInput through fixed point-task/contact row emission.
#[derive(Debug)]
pub struct CudaMirrorEmissionExecutor {
    inner: CudaMirrorFkComExecutor,
}

/// StateInput through fixed-level hierarchical solve. This executor has no
/// CPU fallback and does not promote actuation, integration, or Graph.
#[derive(Debug)]
pub struct CudaMirrorSolveExecutor {
    inner: CudaMirrorFkComExecutor,
}

impl CudaMirrorFkComExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu_mirror: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let layout = cpu_mirror.descriptor().layout.clone();
        if layout.program_fingerprint != program.header.fingerprint_sha256 {
            return Err(CudaFkComError::ProgramMismatch);
        }
        let model = CudaKinematicsModel::compile(program, &layout)?;

        let driver = Driver::load()?;
        driver.initialize()?;
        let raw_device = driver.device(device_ordinal)?;
        let device = query_device_info(&driver, raw_device, device_ordinal)?;
        let driver_version = driver.driver_version()?;

        let compiler = NvrtcCompiler::load()?;
        let (toolkit_major, toolkit_minor) = compiler.version()?;
        let (fk_ptx, nvrtc_log) = compiler.compile(
            CUDA_FK_COM_SOURCE,
            "bonesaw_fk_com_v1.cu",
            device.compute_capability_major,
            device.compute_capability_minor,
        )?;

        let context = driver.create_context(raw_device)?;
        let mut inner = LinuxExecutor {
            driver,
            context,
            state_module: core::ptr::null_mut(),
            state_function: core::ptr::null_mut(),
            fk_module: core::ptr::null_mut(),
            fk_function: core::ptr::null_mut(),
            q_input: 0,
            root_input: 0,
            q_state: 0,
            root_state: 0,
            state_status: 0,
            model: DeviceModel::ZERO,
            body_pose: 0,
            center_of_mass: 0,
            total_mass: 0,
            output_status: 0,
            jacobians: None,
            dynamics: None,
            point_queries: None,
            emission: None,
            solve: None,
        };
        inner.state_module = inner.driver.load_module(STATE_INPUT_PTX)?;
        inner.state_function = inner
            .driver
            .function(inner.state_module, STATE_INPUT_FUNCTION)?;
        inner.fk_module = inner.driver.load_module(&fk_ptx)?;
        inner.fk_function = inner.driver.function(inner.fk_module, FK_COM_FUNCTION)?;

        let q_elements = layout.q_len().map_err(|_| CudaFkComError::SizeOverflow)?;
        let root_elements = layout
            .root_pose_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        inner.q_input = allocate_elements::<f32>(&inner.driver, q_elements)?;
        inner.root_input = allocate_elements::<f32>(&inner.driver, root_elements)?;
        inner.q_state = allocate_elements::<f32>(&inner.driver, q_elements)?;
        inner.root_state = allocate_elements::<f32>(&inner.driver, root_elements)?;
        inner.state_status = allocate_elements::<u8>(&inner.driver, layout.agent_stride)?;

        inner.model.joint_parent = upload(&inner.driver, model.joint_parent())?;
        inner.model.joint_child = upload(&inner.driver, model.joint_child())?;
        inner.model.joint_coordinate = upload(&inner.driver, model.joint_coordinate())?;
        inner.model.joint_kind = upload(&inner.driver, model.joint_kind())?;
        inner.model.joint_parent_from_joint =
            upload(&inner.driver, model.joint_parent_from_joint())?;
        inner.model.joint_axis = upload(&inner.driver, model.joint_axis())?;
        inner.model.body_parent_joint = upload(&inner.driver, model.body_parent_joint())?;
        inner.model.body_mass = upload(&inner.driver, model.body_mass())?;
        inner.model.body_com = upload(&inner.driver, model.body_com())?;
        inner.model.body_inertia = upload(&inner.driver, model.body_inertia())?;

        inner.body_pose = allocate_elements::<f32>(
            &inner.driver,
            layout
                .body_pose_len()
                .map_err(|_| CudaFkComError::SizeOverflow)?,
        )?;
        inner.center_of_mass = allocate_elements::<f32>(&inner.driver, 3 * layout.agent_stride)?;
        inner.total_mass = allocate_elements::<f32>(&inner.driver, layout.agent_stride)?;
        inner.output_status = allocate_elements::<u8>(&inner.driver, layout.agent_stride)?;

        let kernel_hash = combined_kernel_hash(&fk_ptx, model.hash());
        let mut fingerprint = cpu_mirror.fingerprint().clone();
        fingerprint.backend_profile = BackendProfile::CudaMirrorF32;
        fingerprint.scalar_format = ScalarFormat::F32;
        fingerprint.cpu_isa = None;
        fingerprint.cuda_toolkit = Some(Version {
            major: toolkit_major.max(0) as u32,
            minor: toolkit_minor.max(0) as u32,
            patch: 0,
        });
        fingerprint.cuda_driver = Some(version_from_driver(driver_version));
        fingerprint.gpu_architecture = Some(format!(
            "sm_{}{}",
            device.compute_capability_major, device.compute_capability_minor
        ));
        fingerprint.gpu_sm_count = Some(device.multiprocessor_count as u32);
        fingerprint.kernel_hash = kernel_hash;
        Ok(Self {
            status_staging: vec![AgentStatus::Inactive as u8; layout.agent_stride],
            layout,
            fingerprint,
            device,
            model_hash: model.hash(),
            root_body: model.root_body(),
            nvrtc_log,
            inner,
        })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu_mirror: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn fingerprint(&self) -> &BackendFingerprint {
        &self.fingerprint
    }

    pub fn device(&self) -> &CudaDeviceInfo {
        &self.device
    }

    pub fn model_hash(&self) -> [u8; 32] {
        self.model_hash
    }

    pub fn nvrtc_log(&self) -> &str {
        &self.nvrtc_log
    }

    #[cfg(target_os = "linux")]
    fn enable_jacobians(&mut self) -> Result<(), CudaFkComError> {
        self.inner.driver.set_current(self.inner.context)?;
        let compiler = NvrtcCompiler::load()?;
        let (jacobian_ptx, log) = compiler.compile(
            CUDA_JACOBIANS_SOURCE,
            "bonesaw_jacobians_v1.cu",
            self.device.compute_capability_major,
            self.device.compute_capability_minor,
        )?;
        let mut stage = JacobianStage::ZERO;
        let install = (|| -> Result<(), CudaFkComError> {
            stage.module = self.inner.driver.load_module(&jacobian_ptx)?;
            stage.function = self
                .inner
                .driver
                .function(stage.module, JACOBIANS_FUNCTION)?;
            stage.frame_jacobian = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .frame_jacobian_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.center_of_mass_jacobian = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .center_of_mass_jacobian_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.output_status =
                allocate_elements::<u8>(&self.inner.driver, self.layout.agent_stride)?;
            Ok(())
        })();
        if let Err(error) = install {
            stage.destroy(&self.inner.driver);
            return Err(error);
        }
        self.fingerprint.kernel_hash =
            combined_jacobian_kernel_hash(self.fingerprint.kernel_hash, &jacobian_ptx);
        if !log.is_empty() {
            if !self.nvrtc_log.is_empty() {
                self.nvrtc_log.push('\n');
            }
            self.nvrtc_log.push_str(&log);
        }
        self.inner.jacobians = Some(stage);
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn enable_dynamics(&mut self) -> Result<(), CudaFkComError> {
        if self.inner.jacobians.is_none() {
            return Err(CudaFkComError::JacobiansUnavailable);
        }
        self.inner.driver.set_current(self.inner.context)?;
        let compiler = NvrtcCompiler::load()?;
        let (dynamics_ptx, log) = compiler.compile(
            CUDA_DYNAMICS_SOURCE,
            "bonesaw_dynamics_v1.cu",
            self.device.compute_capability_major,
            self.device.compute_capability_minor,
        )?;
        let mut stage = DynamicsStage::ZERO;
        let install = (|| -> Result<(), CudaFkComError> {
            stage.module = self.inner.driver.load_module(&dynamics_ptx)?;
            stage.function = self
                .inner
                .driver
                .function(stage.module, DYNAMICS_FUNCTION)?;
            stage.generalized_velocity = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .generalized_velocity_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.gravity_world = allocate_elements::<f32>(
                &self.inner.driver,
                3_usize
                    .checked_mul(self.layout.agent_stride)
                    .ok_or(CudaFkComError::SizeOverflow)?,
            )?;
            let body_vector_len = self
                .layout
                .body_count
                .checked_mul(3)
                .and_then(|count| count.checked_mul(self.layout.agent_stride))
                .ok_or(CudaFkComError::SizeOverflow)?;
            stage.angular_velocity = allocate_elements::<f32>(&self.inner.driver, body_vector_len)?;
            stage.angular_acceleration =
                allocate_elements::<f32>(&self.inner.driver, body_vector_len)?;
            stage.linear_velocity_origin =
                allocate_elements::<f32>(&self.inner.driver, body_vector_len)?;
            stage.linear_acceleration_origin =
                allocate_elements::<f32>(&self.inner.driver, body_vector_len)?;
            stage.mass_matrix = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .mass_matrix_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.bias_force = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .generalized_velocity_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.centroidal_map = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .centroidal_map_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.output_status =
                allocate_elements::<u8>(&self.inner.driver, self.layout.agent_stride)?;
            Ok(())
        })();
        if let Err(error) = install {
            stage.destroy(&self.inner.driver);
            return Err(error);
        }
        self.fingerprint.kernel_hash =
            combined_dynamics_kernel_hash(self.fingerprint.kernel_hash, &dynamics_ptx);
        if !log.is_empty() {
            if !self.nvrtc_log.is_empty() {
                self.nvrtc_log.push('\n');
            }
            self.nvrtc_log.push_str(&log);
        }
        self.inner.dynamics = Some(stage);
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn enable_point_queries(
        &mut self,
        cpu_mirror: &CpuMirrorExecutor,
    ) -> Result<(), CudaFkComError> {
        if self.inner.dynamics.is_none() {
            return Err(CudaFkComError::DynamicsUnavailable);
        }
        if cpu_mirror.descriptor().layout != self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.inner.driver.set_current(self.inner.context)?;
        let compiler = NvrtcCompiler::load()?;
        let (point_ptx, log) = compiler.compile(
            CUDA_POINT_QUERIES_SOURCE,
            "bonesaw_point_queries_v1.cu",
            self.device.compute_capability_major,
            self.device.compute_capability_minor,
        )?;
        let mut query_frames = Vec::with_capacity(self.layout.point_query_count);
        let mut query_points = Vec::with_capacity(self.layout.point_query_count * 3);
        for query in &cpu_mirror.descriptor().point_queries {
            query_frames.push(
                u32::try_from(query.frame_index).map_err(|_| CudaFkComError::DimensionOverflow)?,
            );
            query_points.extend(query.point_in_frame());
        }
        let mut stage = PointQueryStage::ZERO;
        let install = (|| -> Result<(), CudaFkComError> {
            stage.module = self.inner.driver.load_module(&point_ptx)?;
            stage.function = self
                .inner
                .driver
                .function(stage.module, POINT_QUERIES_FUNCTION)?;
            stage.query_frame = upload(&self.inner.driver, &query_frames)?;
            stage.query_point_in_frame = upload(&self.inner.driver, &query_points)?;
            stage.point_position = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .point_position_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.point_jacobian = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .point_jacobian_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.point_bias_acceleration = allocate_elements::<f32>(
                &self.inner.driver,
                self.layout
                    .point_bias_acceleration_len()
                    .map_err(|_| CudaFkComError::SizeOverflow)?,
            )?;
            stage.output_status =
                allocate_elements::<u8>(&self.inner.driver, self.layout.agent_stride)?;
            Ok(())
        })();
        if let Err(error) = install {
            stage.destroy(&self.inner.driver);
            return Err(error);
        }
        self.fingerprint.kernel_hash =
            combined_point_query_kernel_hash(self.fingerprint.kernel_hash, &point_ptx, cpu_mirror);
        if !log.is_empty() {
            if !self.nvrtc_log.is_empty() {
                self.nvrtc_log.push('\n');
            }
            self.nvrtc_log.push_str(&log);
        }
        self.inner.point_queries = Some(stage);
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn enable_emission(&mut self, cpu: &CpuMirrorExecutor) -> Result<(), CudaFkComError> {
        if self.inner.point_queries.is_none() {
            return Err(CudaFkComError::PointQueriesUnavailable);
        }
        if cpu.descriptor().layout != self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.inner.driver.set_current(self.inner.context)?;
        let compiler = NvrtcCompiler::load()?;
        let (ptx, log) = compiler.compile(
            CUDA_EMISSION_SOURCE,
            "bonesaw_emission_v1.cu",
            self.device.compute_capability_major,
            self.device.compute_capability_minor,
        )?;
        let task_query = cpu
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| abi_u32(task.point_query_slot))
            .collect::<Result<Vec<_>, _>>()?;
        let task_bandwidth = cpu
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.bandwidth_hz())
            .collect::<Vec<_>>();
        let contact_query = cpu
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| abi_u32(contact.point_query_slot))
            .collect::<Result<Vec<_>, _>>()?;
        let contact_mode = cpu
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| contact.kinematic_mode as u32)
            .collect::<Vec<_>>();
        let task_active_len = self
            .layout
            .point_task_active_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let task_vector_len = self
            .layout
            .point_task_vector_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let task_jacobian_len = self
            .layout
            .point_task_jacobian_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let contact_active_len = self
            .layout
            .contact_lock_active_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let contact_vector_len = self
            .layout
            .contact_lock_vector_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let contact_jacobian_len = self
            .layout
            .contact_lock_jacobian_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let mut stage = EmissionStage::ZERO;
        let install = (|| -> Result<(), CudaFkComError> {
            stage.module = self.inner.driver.load_module(&ptx)?;
            stage.function = self
                .inner
                .driver
                .function(stage.module, EMISSION_FUNCTION)?;
            stage.task_query_slot = upload(&self.inner.driver, &task_query)?;
            stage.task_bandwidth_hz = upload(&self.inner.driver, &task_bandwidth)?;
            stage.contact_query_slot = upload(&self.inner.driver, &contact_query)?;
            stage.contact_mode = upload(&self.inner.driver, &contact_mode)?;
            stage.task_active_input = allocate_elements::<u8>(&self.inner.driver, task_active_len)?;
            stage.target_position = allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.target_velocity = allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.target_acceleration =
                allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.contact_active_input =
                allocate_elements::<u8>(&self.inner.driver, contact_active_len)?;
            stage.contact_desired_acceleration =
                allocate_elements::<f32>(&self.inner.driver, contact_vector_len)?;
            stage.task_active_output =
                allocate_elements::<u8>(&self.inner.driver, task_active_len)?;
            stage.task_position_error =
                allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.task_velocity_error =
                allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.task_desired_acceleration =
                allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.task_jacobian = allocate_elements::<f32>(&self.inner.driver, task_jacobian_len)?;
            stage.task_rhs = allocate_elements::<f32>(&self.inner.driver, task_vector_len)?;
            stage.contact_active_output =
                allocate_elements::<u8>(&self.inner.driver, contact_active_len)?;
            stage.contact_jacobian =
                allocate_elements::<f32>(&self.inner.driver, contact_jacobian_len)?;
            stage.contact_rhs = allocate_elements::<f32>(&self.inner.driver, contact_vector_len)?;
            stage.output_status =
                allocate_elements::<u8>(&self.inner.driver, self.layout.agent_stride)?;
            Ok(())
        })();
        if let Err(error) = install {
            stage.destroy(&self.inner.driver);
            return Err(error);
        }
        self.fingerprint.kernel_hash =
            combined_emission_kernel_hash(self.fingerprint.kernel_hash, &ptx, cpu);
        if !log.is_empty() {
            if !self.nvrtc_log.is_empty() {
                self.nvrtc_log.push('\n');
            }
            self.nvrtc_log.push_str(&log);
        }
        self.inner.emission = Some(stage);
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn enable_solve(
        &mut self,
        cpu: &CpuMirrorExecutor,
        mirror_solver: &CpuMirrorBatchSolver,
    ) -> Result<(), CudaFkComError> {
        if self.inner.emission.is_none() {
            return Err(CudaFkComError::EmissionUnavailable);
        }
        if cpu.descriptor().layout != self.layout || mirror_solver.descriptor() != cpu.descriptor()
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.inner.driver.set_current(self.inner.context)?;
        let compiler = NvrtcCompiler::load()?;
        let (ptx, log) = compiler.compile(
            CUDA_SOLVE_SOURCE,
            "bonesaw_solve_v1.cu",
            self.device.compute_capability_major,
            self.device.compute_capability_minor,
        )?;
        let task_priority = cpu
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.priority as u8)
            .collect::<Vec<_>>();
        let task_weight = cpu
            .descriptor()
            .point_tasks
            .iter()
            .map(|task| task.weight())
            .collect::<Vec<_>>();
        let contact_mode = cpu
            .descriptor()
            .contact_locks
            .iter()
            .map(|contact| contact.kinematic_mode as u32)
            .collect::<Vec<_>>();
        let vector_len = self
            .layout
            .generalized_velocity_len()
            .map_err(|_| CudaFkComError::SizeOverflow)?;
        let level_len = 5_usize
            .checked_mul(self.layout.agent_stride)
            .ok_or(CudaFkComError::SizeOverflow)?;
        let maximum_soft_rows = self
            .layout
            .point_task_count
            .checked_mul(3)
            .ok_or(CudaFkComError::SizeOverflow)?;
        let locked_vector_len = maximum_soft_rows
            .checked_mul(self.layout.agent_stride)
            .ok_or(CudaFkComError::SizeOverflow)?;
        let locked_matrix_len = maximum_soft_rows
            .checked_mul(self.layout.generalized_coordinate_count)
            .and_then(|value| value.checked_mul(self.layout.agent_stride))
            .ok_or(CudaFkComError::SizeOverflow)?;
        let stride = self.layout.agent_stride;
        let mut stage = SolveStage::ZERO;
        let install = (|| -> Result<(), CudaFkComError> {
            stage.module = self.inner.driver.load_module(&ptx)?;
            stage.function = self.inner.driver.function(stage.module, SOLVE_FUNCTION)?;
            stage.task_priority = upload(&self.inner.driver, &task_priority)?;
            stage.task_weight = upload(&self.inner.driver, &task_weight)?;
            stage.contact_mode = upload(&self.inner.driver, &contact_mode)?;
            stage.lower = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.upper = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.command = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.candidate = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.status = allocate_elements::<u8>(&self.inner.driver, stride)?;
            stage.level_rows = allocate_elements::<u32>(&self.inner.driver, level_len)?;
            stage.level_rms = allocate_elements::<f32>(&self.inner.driver, level_len)?;
            stage.level_preservation_drift =
                allocate_elements::<f32>(&self.inner.driver, level_len)?;
            stage.initial_hard_violation = allocate_elements::<f32>(&self.inner.driver, stride)?;
            stage.best_hard_violation = allocate_elements::<f32>(&self.inner.driver, stride)?;
            stage.final_hard_violation = allocate_elements::<f32>(&self.inner.driver, stride)?;
            stage.minimum_bound_margin = allocate_elements::<f32>(&self.inner.driver, stride)?;
            stage.hard_projection_sweeps = allocate_elements::<u32>(&self.inner.driver, stride)?;
            stage.task_sweeps = allocate_elements::<u32>(&self.inner.driver, stride)?;
            stage.clipped_updates = allocate_elements::<u32>(&self.inner.driver, stride)?;
            stage.active_hard_rows = allocate_elements::<u32>(&self.inner.driver, stride)?;
            stage.active_soft_rows = allocate_elements::<u32>(&self.inner.driver, stride)?;
            stage.x = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.best = allocate_elements::<f32>(&self.inner.driver, vector_len)?;
            stage.locked_rows = allocate_elements::<f32>(&self.inner.driver, locked_matrix_len)?;
            stage.locked_rhs = allocate_elements::<f32>(&self.inner.driver, locked_vector_len)?;
            stage.locked_reference_rhs =
                allocate_elements::<f32>(&self.inner.driver, locked_vector_len)?;
            stage.locked_priority = allocate_elements::<u8>(&self.inner.driver, locked_vector_len)?;
            Ok(())
        })();
        if let Err(error) = install {
            stage.destroy(&self.inner.driver);
            return Err(error);
        }
        self.fingerprint.kernel_hash = combined_solve_kernel_hash(
            self.fingerprint.kernel_hash,
            &ptx,
            mirror_solver.algorithm_sha256(),
        );
        if !log.is_empty() {
            if !self.nvrtc_log.is_empty() {
                self.nvrtc_log.push('\n');
            }
            self.nvrtc_log.push_str(&log);
        }
        self.inner.solve = Some(stage);
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn launch_fk(&mut self, input: &FkBatchInput) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.inner.driver.set_current(self.inner.context)?;
        self.inner
            .driver
            .copy_to_device(self.inner.q_input, input.q_soa())?;
        self.inner
            .driver
            .copy_to_device(self.inner.root_input, input.root_pose_soa())?;

        let mut q_input = self.inner.q_input;
        let mut root_input = self.inner.root_input;
        let mut active_agents = abi_u32(input.active_agents())?;
        let mut coordinate_count = abi_u32(self.layout.coordinate_count)?;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut q_state = self.inner.q_state;
        let mut root_state = self.inner.root_state;
        let mut state_status = self.inner.state_status;
        let mut state_parameters = [
            (&mut q_input as *mut CuDevicePtr).cast(),
            (&mut root_input as *mut CuDevicePtr).cast(),
            (&mut active_agents as *mut u32).cast(),
            (&mut coordinate_count as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut q_state as *mut CuDevicePtr).cast(),
            (&mut root_state as *mut CuDevicePtr).cast(),
            (&mut state_status as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner.driver.launch(
            self.inner.state_function,
            grid,
            BLOCK_SIZE,
            &mut state_parameters,
        )?;

        let mut root_body = self.root_body;
        let mut joint_count = abi_u32(self.layout.joint_count)?;
        let mut body_count = abi_u32(self.layout.body_count)?;
        let mut joint_parent = self.inner.model.joint_parent;
        let mut joint_child = self.inner.model.joint_child;
        let mut joint_coordinate = self.inner.model.joint_coordinate;
        let mut joint_kind = self.inner.model.joint_kind;
        let mut joint_parent_from_joint = self.inner.model.joint_parent_from_joint;
        let mut joint_axis = self.inner.model.joint_axis;
        let mut body_mass = self.inner.model.body_mass;
        let mut body_com = self.inner.model.body_com;
        let mut body_pose = self.inner.body_pose;
        let mut center_of_mass = self.inner.center_of_mass;
        let mut total_mass = self.inner.total_mass;
        let mut output_status = self.inner.output_status;
        let mut fk_parameters = [
            (&mut q_state as *mut CuDevicePtr).cast(),
            (&mut root_state as *mut CuDevicePtr).cast(),
            (&mut state_status as *mut CuDevicePtr).cast(),
            (&mut active_agents as *mut u32).cast(),
            (&mut coordinate_count as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut root_body as *mut u32).cast(),
            (&mut joint_count as *mut u32).cast(),
            (&mut body_count as *mut u32).cast(),
            (&mut joint_parent as *mut CuDevicePtr).cast(),
            (&mut joint_child as *mut CuDevicePtr).cast(),
            (&mut joint_coordinate as *mut CuDevicePtr).cast(),
            (&mut joint_kind as *mut CuDevicePtr).cast(),
            (&mut joint_parent_from_joint as *mut CuDevicePtr).cast(),
            (&mut joint_axis as *mut CuDevicePtr).cast(),
            (&mut body_mass as *mut CuDevicePtr).cast(),
            (&mut body_com as *mut CuDevicePtr).cast(),
            (&mut body_pose as *mut CuDevicePtr).cast(),
            (&mut center_of_mass as *mut CuDevicePtr).cast(),
            (&mut total_mass as *mut CuDevicePtr).cast(),
            (&mut output_status as *mut CuDevicePtr).cast(),
        ];
        self.inner
            .driver
            .launch(self.inner.fk_function, grid, BLOCK_SIZE, &mut fk_parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_fk_output(&mut self, output: &mut FkBatchOutput) -> Result<(), CudaFkComError> {
        self.inner
            .driver
            .copy_from_device(output.body_pose_soa_mut(), self.inner.body_pose)?;
        self.inner
            .driver
            .copy_from_device(output.center_of_mass_soa_mut(), self.inner.center_of_mass)?;
        self.inner
            .driver
            .copy_from_device(output.total_mass_mut(), self.inner.total_mass)?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, self.inner.output_status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                value if value == AgentStatus::Inactive as u8 => AgentStatus::Inactive,
                value if value == AgentStatus::Ok as u8 => AgentStatus::Ok,
                value if value == AgentStatus::InvalidInput as u8 => AgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        input: &FkBatchInput,
        output: &mut FkBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout || output.layout() != &self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(input)?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(output)
    }

    #[cfg(target_os = "linux")]
    fn launch_jacobians(&mut self) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .jacobians
            .as_mut()
            .ok_or(CudaFkComError::JacobiansUnavailable)?;
        let mut body_pose = self.inner.body_pose;
        let mut total_mass = self.inner.total_mass;
        let mut input_status = self.inner.output_status;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut root_body = self.root_body;
        let mut body_count = abi_u32(self.layout.body_count)?;
        let mut generalized_coordinate_count = abi_u32(self.layout.generalized_coordinate_count)?;
        let mut joint_parent = self.inner.model.joint_parent;
        let mut joint_coordinate = self.inner.model.joint_coordinate;
        let mut joint_kind = self.inner.model.joint_kind;
        let mut joint_parent_from_joint = self.inner.model.joint_parent_from_joint;
        let mut joint_axis = self.inner.model.joint_axis;
        let mut body_parent_joint = self.inner.model.body_parent_joint;
        let mut body_mass = self.inner.model.body_mass;
        let mut body_com = self.inner.model.body_com;
        let mut frame_jacobian = stage.frame_jacobian;
        let mut center_of_mass_jacobian = stage.center_of_mass_jacobian;
        let mut output_status = stage.output_status;
        let mut parameters = [
            (&mut body_pose as *mut CuDevicePtr).cast(),
            (&mut total_mass as *mut CuDevicePtr).cast(),
            (&mut input_status as *mut CuDevicePtr).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut root_body as *mut u32).cast(),
            (&mut body_count as *mut u32).cast(),
            (&mut generalized_coordinate_count as *mut u32).cast(),
            (&mut joint_parent as *mut CuDevicePtr).cast(),
            (&mut joint_coordinate as *mut CuDevicePtr).cast(),
            (&mut joint_kind as *mut CuDevicePtr).cast(),
            (&mut joint_parent_from_joint as *mut CuDevicePtr).cast(),
            (&mut joint_axis as *mut CuDevicePtr).cast(),
            (&mut body_parent_joint as *mut CuDevicePtr).cast(),
            (&mut body_mass as *mut CuDevicePtr).cast(),
            (&mut body_com as *mut CuDevicePtr).cast(),
            (&mut frame_jacobian as *mut CuDevicePtr).cast(),
            (&mut center_of_mass_jacobian as *mut CuDevicePtr).cast(),
            (&mut output_status as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(stage.function, grid, BLOCK_SIZE, &mut parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_jacobian_output(
        &mut self,
        output: &mut JacobianBatchOutput,
    ) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .jacobians
            .as_ref()
            .ok_or(CudaFkComError::JacobiansUnavailable)?;
        self.inner
            .driver
            .copy_from_device(output.frame_jacobian_soa_mut(), stage.frame_jacobian)?;
        self.inner.driver.copy_from_device(
            output.center_of_mass_jacobian_soa_mut(),
            stage.center_of_mass_jacobian,
        )?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, stage.output_status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                value if value == AgentStatus::Inactive as u8 => AgentStatus::Inactive,
                value if value == AgentStatus::Ok as u8 => AgentStatus::Ok,
                value if value == AgentStatus::InvalidInput as u8 => AgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn execute_jacobians_into(
        &mut self,
        input: &FkBatchInput,
        fk_output: &mut FkBatchOutput,
        output: &mut JacobianBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout
            || fk_output.layout() != &self.layout
            || output.layout() != &self.layout
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(input)?;
        self.launch_jacobians()?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(fk_output)?;
        self.copy_jacobian_output(output)
    }

    #[cfg(target_os = "linux")]
    fn launch_dynamics(&mut self, input: &DynamicsBatchInput) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        let jacobians = self
            .inner
            .jacobians
            .as_ref()
            .ok_or(CudaFkComError::JacobiansUnavailable)?;
        let frame_jacobian_pointer = jacobians.frame_jacobian;
        let jacobian_status_pointer = jacobians.output_status;
        let stage = self
            .inner
            .dynamics
            .as_mut()
            .ok_or(CudaFkComError::DynamicsUnavailable)?;
        self.inner
            .driver
            .copy_to_device(stage.generalized_velocity, input.generalized_velocity_soa())?;
        self.inner
            .driver
            .copy_to_device(stage.gravity_world, input.gravity_world_soa())?;

        let mut generalized_velocity = stage.generalized_velocity;
        let mut gravity_world = stage.gravity_world;
        let mut body_pose = self.inner.body_pose;
        let mut center_of_mass = self.inner.center_of_mass;
        let mut input_status = jacobian_status_pointer;
        let mut active_agents = abi_u32(input.active_agents())?;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut root_body = self.root_body;
        let mut joint_count = abi_u32(self.layout.joint_count)?;
        let mut body_count = abi_u32(self.layout.body_count)?;
        let mut generalized_coordinate_count = abi_u32(self.layout.generalized_coordinate_count)?;
        let mut joint_parent = self.inner.model.joint_parent;
        let mut joint_child = self.inner.model.joint_child;
        let mut joint_coordinate = self.inner.model.joint_coordinate;
        let mut joint_kind = self.inner.model.joint_kind;
        let mut joint_parent_from_joint = self.inner.model.joint_parent_from_joint;
        let mut joint_axis = self.inner.model.joint_axis;
        let mut body_mass = self.inner.model.body_mass;
        let mut body_com = self.inner.model.body_com;
        let mut body_inertia = self.inner.model.body_inertia;
        let mut frame_jacobian = frame_jacobian_pointer;
        let mut angular_velocity = stage.angular_velocity;
        let mut angular_acceleration = stage.angular_acceleration;
        let mut linear_velocity_origin = stage.linear_velocity_origin;
        let mut linear_acceleration_origin = stage.linear_acceleration_origin;
        let mut mass_matrix = stage.mass_matrix;
        let mut bias_force = stage.bias_force;
        let mut centroidal_map = stage.centroidal_map;
        let mut output_status = stage.output_status;
        let mut parameters = [
            (&mut generalized_velocity as *mut CuDevicePtr).cast(),
            (&mut gravity_world as *mut CuDevicePtr).cast(),
            (&mut body_pose as *mut CuDevicePtr).cast(),
            (&mut center_of_mass as *mut CuDevicePtr).cast(),
            (&mut input_status as *mut CuDevicePtr).cast(),
            (&mut active_agents as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut root_body as *mut u32).cast(),
            (&mut joint_count as *mut u32).cast(),
            (&mut body_count as *mut u32).cast(),
            (&mut generalized_coordinate_count as *mut u32).cast(),
            (&mut joint_parent as *mut CuDevicePtr).cast(),
            (&mut joint_child as *mut CuDevicePtr).cast(),
            (&mut joint_coordinate as *mut CuDevicePtr).cast(),
            (&mut joint_kind as *mut CuDevicePtr).cast(),
            (&mut joint_parent_from_joint as *mut CuDevicePtr).cast(),
            (&mut joint_axis as *mut CuDevicePtr).cast(),
            (&mut body_mass as *mut CuDevicePtr).cast(),
            (&mut body_com as *mut CuDevicePtr).cast(),
            (&mut body_inertia as *mut CuDevicePtr).cast(),
            (&mut frame_jacobian as *mut CuDevicePtr).cast(),
            (&mut angular_velocity as *mut CuDevicePtr).cast(),
            (&mut angular_acceleration as *mut CuDevicePtr).cast(),
            (&mut linear_velocity_origin as *mut CuDevicePtr).cast(),
            (&mut linear_acceleration_origin as *mut CuDevicePtr).cast(),
            (&mut mass_matrix as *mut CuDevicePtr).cast(),
            (&mut bias_force as *mut CuDevicePtr).cast(),
            (&mut centroidal_map as *mut CuDevicePtr).cast(),
            (&mut output_status as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(stage.function, grid, BLOCK_SIZE, &mut parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_dynamics_output(
        &mut self,
        output: &mut DynamicsBatchOutput,
    ) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .dynamics
            .as_ref()
            .ok_or(CudaFkComError::DynamicsUnavailable)?;
        self.inner
            .driver
            .copy_from_device(output.mass_matrix_soa_mut(), stage.mass_matrix)?;
        self.inner
            .driver
            .copy_from_device(output.bias_force_soa_mut(), stage.bias_force)?;
        self.inner
            .driver
            .copy_from_device(output.centroidal_map_soa_mut(), stage.centroidal_map)?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, stage.output_status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                value if value == AgentStatus::Inactive as u8 => AgentStatus::Inactive,
                value if value == AgentStatus::Ok as u8 => AgentStatus::Ok,
                value if value == AgentStatus::InvalidInput as u8 => AgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn execute_dynamics_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if fk_input.layout() != &self.layout
            || dynamics_input.layout() != &self.layout
            || fk_output.layout() != &self.layout
            || jacobian_output.layout() != &self.layout
            || dynamics_output.layout() != &self.layout
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(fk_input)?;
        self.launch_jacobians()?;
        self.launch_dynamics(dynamics_input)?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(fk_output)?;
        self.copy_jacobian_output(jacobian_output)?;
        self.copy_dynamics_output(dynamics_output)
    }

    #[cfg(target_os = "linux")]
    fn launch_point_queries(&mut self) -> Result<(), CudaFkComError> {
        let jacobians = self
            .inner
            .jacobians
            .as_ref()
            .ok_or(CudaFkComError::JacobiansUnavailable)?;
        let dynamics = self
            .inner
            .dynamics
            .as_ref()
            .ok_or(CudaFkComError::DynamicsUnavailable)?;
        let stage = self
            .inner
            .point_queries
            .as_mut()
            .ok_or(CudaFkComError::PointQueriesUnavailable)?;
        let mut body_pose = self.inner.body_pose;
        let mut frame_jacobian = jacobians.frame_jacobian;
        let mut input_status = dynamics.output_status;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut point_query_count = abi_u32(self.layout.point_query_count)?;
        let mut generalized_coordinate_count = abi_u32(self.layout.generalized_coordinate_count)?;
        let mut query_frame = stage.query_frame;
        let mut query_point_in_frame = stage.query_point_in_frame;
        let mut angular_velocity = dynamics.angular_velocity;
        let mut angular_acceleration = dynamics.angular_acceleration;
        let mut linear_acceleration_origin = dynamics.linear_acceleration_origin;
        let mut point_position = stage.point_position;
        let mut point_jacobian = stage.point_jacobian;
        let mut point_bias_acceleration = stage.point_bias_acceleration;
        let mut output_status = stage.output_status;
        let mut parameters = [
            (&mut body_pose as *mut CuDevicePtr).cast(),
            (&mut frame_jacobian as *mut CuDevicePtr).cast(),
            (&mut input_status as *mut CuDevicePtr).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut point_query_count as *mut u32).cast(),
            (&mut generalized_coordinate_count as *mut u32).cast(),
            (&mut query_frame as *mut CuDevicePtr).cast(),
            (&mut query_point_in_frame as *mut CuDevicePtr).cast(),
            (&mut angular_velocity as *mut CuDevicePtr).cast(),
            (&mut angular_acceleration as *mut CuDevicePtr).cast(),
            (&mut linear_acceleration_origin as *mut CuDevicePtr).cast(),
            (&mut point_position as *mut CuDevicePtr).cast(),
            (&mut point_jacobian as *mut CuDevicePtr).cast(),
            (&mut point_bias_acceleration as *mut CuDevicePtr).cast(),
            (&mut output_status as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(stage.function, grid, BLOCK_SIZE, &mut parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_point_query_output(
        &mut self,
        output: &mut PointQueryBatchOutput,
    ) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .point_queries
            .as_ref()
            .ok_or(CudaFkComError::PointQueriesUnavailable)?;
        self.inner
            .driver
            .copy_from_device(output.point_position_soa_mut(), stage.point_position)?;
        self.inner
            .driver
            .copy_from_device(output.point_jacobian_soa_mut(), stage.point_jacobian)?;
        self.inner.driver.copy_from_device(
            output.point_bias_acceleration_soa_mut(),
            stage.point_bias_acceleration,
        )?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, stage.output_status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                value if value == AgentStatus::Inactive as u8 => AgentStatus::Inactive,
                value if value == AgentStatus::Ok as u8 => AgentStatus::Ok,
                value if value == AgentStatus::InvalidInput as u8 => AgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn execute_point_queries_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if fk_input.layout() != &self.layout
            || dynamics_input.layout() != &self.layout
            || fk_output.layout() != &self.layout
            || jacobian_output.layout() != &self.layout
            || dynamics_output.layout() != &self.layout
            || point_output.layout() != &self.layout
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(fk_input)?;
        self.launch_jacobians()?;
        self.launch_dynamics(dynamics_input)?;
        self.launch_point_queries()?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(fk_output)?;
        self.copy_jacobian_output(jacobian_output)?;
        self.copy_dynamics_output(dynamics_output)?;
        self.copy_point_query_output(point_output)
    }

    #[cfg(target_os = "linux")]
    fn launch_emission(
        &mut self,
        input: &EmissionBatchInput,
        dynamics_input: &DynamicsBatchInput,
    ) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout || dynamics_input.layout() != &self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        let points = self
            .inner
            .point_queries
            .as_ref()
            .ok_or(CudaFkComError::PointQueriesUnavailable)?;
        let point_position_pointer = points.point_position;
        let point_jacobian_pointer = points.point_jacobian;
        let point_bias_pointer = points.point_bias_acceleration;
        let point_status_pointer = points.output_status;
        let dynamics = self
            .inner
            .dynamics
            .as_ref()
            .ok_or(CudaFkComError::DynamicsUnavailable)?;
        let generalized_velocity_pointer = dynamics.generalized_velocity;
        let stage = self
            .inner
            .emission
            .as_mut()
            .ok_or(CudaFkComError::EmissionUnavailable)?;
        self.inner
            .driver
            .copy_to_device(stage.task_active_input, input.point_task_active_soa())?;
        self.inner
            .driver
            .copy_to_device(stage.target_position, input.point_target_position_soa())?;
        self.inner
            .driver
            .copy_to_device(stage.target_velocity, input.point_target_velocity_soa())?;
        self.inner.driver.copy_to_device(
            stage.target_acceleration,
            input.point_target_acceleration_soa(),
        )?;
        self.inner
            .driver
            .copy_to_device(stage.contact_active_input, input.contact_lock_active_soa())?;
        self.inner.driver.copy_to_device(
            stage.contact_desired_acceleration,
            input.contact_desired_acceleration_soa(),
        )?;
        let mut generalized_velocity = generalized_velocity_pointer;
        let mut point_position = point_position_pointer;
        let mut point_jacobian = point_jacobian_pointer;
        let mut point_bias = point_bias_pointer;
        let mut input_status = point_status_pointer;
        let mut emission_active = abi_u32(input.active_agents())?;
        let mut dynamics_active = abi_u32(dynamics_input.active_agents())?;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut task_count = abi_u32(self.layout.point_task_count)?;
        let mut contact_count = abi_u32(self.layout.contact_lock_count)?;
        let mut generalized_count = abi_u32(self.layout.generalized_coordinate_count)?;
        let mut task_query = stage.task_query_slot;
        let mut task_bandwidth = stage.task_bandwidth_hz;
        let mut contact_query = stage.contact_query_slot;
        let mut contact_mode = stage.contact_mode;
        let mut task_active_input = stage.task_active_input;
        let mut target_position = stage.target_position;
        let mut target_velocity = stage.target_velocity;
        let mut target_acceleration = stage.target_acceleration;
        let mut contact_active_input = stage.contact_active_input;
        let mut contact_desired = stage.contact_desired_acceleration;
        let mut task_active_output = stage.task_active_output;
        let mut task_position_error = stage.task_position_error;
        let mut task_velocity_error = stage.task_velocity_error;
        let mut task_desired = stage.task_desired_acceleration;
        let mut task_jacobian = stage.task_jacobian;
        let mut task_rhs = stage.task_rhs;
        let mut contact_active_output = stage.contact_active_output;
        let mut contact_jacobian = stage.contact_jacobian;
        let mut contact_rhs = stage.contact_rhs;
        let mut output_status = stage.output_status;
        let mut parameters = [
            (&mut generalized_velocity as *mut CuDevicePtr).cast(),
            (&mut point_position as *mut CuDevicePtr).cast(),
            (&mut point_jacobian as *mut CuDevicePtr).cast(),
            (&mut point_bias as *mut CuDevicePtr).cast(),
            (&mut input_status as *mut CuDevicePtr).cast(),
            (&mut emission_active as *mut u32).cast(),
            (&mut dynamics_active as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut task_count as *mut u32).cast(),
            (&mut contact_count as *mut u32).cast(),
            (&mut generalized_count as *mut u32).cast(),
            (&mut task_query as *mut CuDevicePtr).cast(),
            (&mut task_bandwidth as *mut CuDevicePtr).cast(),
            (&mut contact_query as *mut CuDevicePtr).cast(),
            (&mut contact_mode as *mut CuDevicePtr).cast(),
            (&mut task_active_input as *mut CuDevicePtr).cast(),
            (&mut target_position as *mut CuDevicePtr).cast(),
            (&mut target_velocity as *mut CuDevicePtr).cast(),
            (&mut target_acceleration as *mut CuDevicePtr).cast(),
            (&mut contact_active_input as *mut CuDevicePtr).cast(),
            (&mut contact_desired as *mut CuDevicePtr).cast(),
            (&mut task_active_output as *mut CuDevicePtr).cast(),
            (&mut task_position_error as *mut CuDevicePtr).cast(),
            (&mut task_velocity_error as *mut CuDevicePtr).cast(),
            (&mut task_desired as *mut CuDevicePtr).cast(),
            (&mut task_jacobian as *mut CuDevicePtr).cast(),
            (&mut task_rhs as *mut CuDevicePtr).cast(),
            (&mut contact_active_output as *mut CuDevicePtr).cast(),
            (&mut contact_jacobian as *mut CuDevicePtr).cast(),
            (&mut contact_rhs as *mut CuDevicePtr).cast(),
            (&mut output_status as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(stage.function, grid, BLOCK_SIZE, &mut parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_emission_output(
        &mut self,
        output: &mut EmissionBatchOutput,
    ) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .emission
            .as_ref()
            .ok_or(CudaFkComError::EmissionUnavailable)?;
        self.inner
            .driver
            .copy_from_device(output.point_task_active_soa_mut(), stage.task_active_output)?;
        self.inner.driver.copy_from_device(
            output.point_task_position_error_soa_mut(),
            stage.task_position_error,
        )?;
        self.inner.driver.copy_from_device(
            output.point_task_velocity_error_soa_mut(),
            stage.task_velocity_error,
        )?;
        self.inner.driver.copy_from_device(
            output.point_task_desired_acceleration_soa_mut(),
            stage.task_desired_acceleration,
        )?;
        self.inner
            .driver
            .copy_from_device(output.point_task_jacobian_soa_mut(), stage.task_jacobian)?;
        self.inner
            .driver
            .copy_from_device(output.point_task_rhs_soa_mut(), stage.task_rhs)?;
        self.inner.driver.copy_from_device(
            output.contact_lock_active_soa_mut(),
            stage.contact_active_output,
        )?;
        self.inner.driver.copy_from_device(
            output.contact_lock_jacobian_soa_mut(),
            stage.contact_jacobian,
        )?;
        self.inner
            .driver
            .copy_from_device(output.contact_lock_rhs_soa_mut(), stage.contact_rhs)?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, stage.output_status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                0 => AgentStatus::Inactive,
                1 => AgentStatus::Ok,
                2 => AgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn execute_emission_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
        emission_output: &mut EmissionBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if [
            fk_input.layout(),
            dynamics_input.layout(),
            emission_input.layout(),
            fk_output.layout(),
            jacobian_output.layout(),
            dynamics_output.layout(),
            point_output.layout(),
            emission_output.layout(),
        ]
        .into_iter()
        .any(|layout| layout != &self.layout)
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(fk_input)?;
        self.launch_jacobians()?;
        self.launch_dynamics(dynamics_input)?;
        self.launch_point_queries()?;
        self.launch_emission(emission_input, dynamics_input)?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(fk_output)?;
        self.copy_jacobian_output(jacobian_output)?;
        self.copy_dynamics_output(dynamics_output)?;
        self.copy_point_query_output(point_output)?;
        self.copy_emission_output(emission_output)
    }

    #[cfg(target_os = "linux")]
    fn launch_solve(&mut self, input: &MirrorSolveBatchInput) -> Result<(), CudaFkComError> {
        if input.layout() != &self.layout {
            return Err(CudaFkComError::LayoutMismatch);
        }
        let emission = self
            .inner
            .emission
            .as_ref()
            .ok_or(CudaFkComError::EmissionUnavailable)?;
        let emission_status_pointer = emission.output_status;
        let task_active_pointer = emission.task_active_output;
        let task_jacobian_pointer = emission.task_jacobian;
        let task_rhs_pointer = emission.task_rhs;
        let contact_active_pointer = emission.contact_active_output;
        let contact_jacobian_pointer = emission.contact_jacobian;
        let contact_rhs_pointer = emission.contact_rhs;
        let stage = self
            .inner
            .solve
            .as_mut()
            .ok_or(CudaFkComError::SolveUnavailable)?;
        self.inner
            .driver
            .copy_to_device(stage.lower, input.lower_soa())?;
        self.inner
            .driver
            .copy_to_device(stage.upper, input.upper_soa())?;
        let mut emission_status = emission_status_pointer;
        let mut active_agents = abi_u32(input.active_agents())?;
        let mut agent_capacity = abi_u32(self.layout.agent_capacity)?;
        let mut agent_stride = abi_u32(self.layout.agent_stride)?;
        let mut task_count = abi_u32(self.layout.point_task_count)?;
        let mut contact_count = abi_u32(self.layout.contact_lock_count)?;
        let mut dof = abi_u32(self.layout.generalized_coordinate_count)?;
        let mut task_priority = stage.task_priority;
        let mut task_weight = stage.task_weight;
        let mut contact_mode = stage.contact_mode;
        let mut task_active = task_active_pointer;
        let mut task_jacobian = task_jacobian_pointer;
        let mut task_rhs = task_rhs_pointer;
        let mut contact_active = contact_active_pointer;
        let mut contact_jacobian = contact_jacobian_pointer;
        let mut contact_rhs = contact_rhs_pointer;
        let mut lower = stage.lower;
        let mut upper = stage.upper;
        let mut command = stage.command;
        let mut candidate = stage.candidate;
        let mut status = stage.status;
        let mut level_rows = stage.level_rows;
        let mut level_rms = stage.level_rms;
        let mut level_preservation_drift = stage.level_preservation_drift;
        let mut initial_hard_violation = stage.initial_hard_violation;
        let mut best_hard_violation = stage.best_hard_violation;
        let mut final_hard_violation = stage.final_hard_violation;
        let mut minimum_bound_margin = stage.minimum_bound_margin;
        let mut hard_projection_sweeps = stage.hard_projection_sweeps;
        let mut task_sweeps = stage.task_sweeps;
        let mut clipped_updates = stage.clipped_updates;
        let mut active_hard_rows = stage.active_hard_rows;
        let mut active_soft_rows = stage.active_soft_rows;
        let mut x = stage.x;
        let mut best = stage.best;
        let mut locked_rows = stage.locked_rows;
        let mut locked_rhs = stage.locked_rhs;
        let mut locked_reference_rhs = stage.locked_reference_rhs;
        let mut locked_priority = stage.locked_priority;
        let mut parameters = [
            (&mut emission_status as *mut CuDevicePtr).cast(),
            (&mut active_agents as *mut u32).cast(),
            (&mut agent_capacity as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut task_count as *mut u32).cast(),
            (&mut contact_count as *mut u32).cast(),
            (&mut dof as *mut u32).cast(),
            (&mut task_priority as *mut CuDevicePtr).cast(),
            (&mut task_weight as *mut CuDevicePtr).cast(),
            (&mut contact_mode as *mut CuDevicePtr).cast(),
            (&mut task_active as *mut CuDevicePtr).cast(),
            (&mut task_jacobian as *mut CuDevicePtr).cast(),
            (&mut task_rhs as *mut CuDevicePtr).cast(),
            (&mut contact_active as *mut CuDevicePtr).cast(),
            (&mut contact_jacobian as *mut CuDevicePtr).cast(),
            (&mut contact_rhs as *mut CuDevicePtr).cast(),
            (&mut lower as *mut CuDevicePtr).cast(),
            (&mut upper as *mut CuDevicePtr).cast(),
            (&mut command as *mut CuDevicePtr).cast(),
            (&mut candidate as *mut CuDevicePtr).cast(),
            (&mut status as *mut CuDevicePtr).cast(),
            (&mut level_rows as *mut CuDevicePtr).cast(),
            (&mut level_rms as *mut CuDevicePtr).cast(),
            (&mut level_preservation_drift as *mut CuDevicePtr).cast(),
            (&mut initial_hard_violation as *mut CuDevicePtr).cast(),
            (&mut best_hard_violation as *mut CuDevicePtr).cast(),
            (&mut final_hard_violation as *mut CuDevicePtr).cast(),
            (&mut minimum_bound_margin as *mut CuDevicePtr).cast(),
            (&mut hard_projection_sweeps as *mut CuDevicePtr).cast(),
            (&mut task_sweeps as *mut CuDevicePtr).cast(),
            (&mut clipped_updates as *mut CuDevicePtr).cast(),
            (&mut active_hard_rows as *mut CuDevicePtr).cast(),
            (&mut active_soft_rows as *mut CuDevicePtr).cast(),
            (&mut x as *mut CuDevicePtr).cast(),
            (&mut best as *mut CuDevicePtr).cast(),
            (&mut locked_rows as *mut CuDevicePtr).cast(),
            (&mut locked_rhs as *mut CuDevicePtr).cast(),
            (&mut locked_reference_rhs as *mut CuDevicePtr).cast(),
            (&mut locked_priority as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(stage.function, grid, BLOCK_SIZE, &mut parameters)?;
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn copy_solve_output(
        &mut self,
        output: &mut MirrorSolveBatchOutput,
    ) -> Result<(), CudaFkComError> {
        let stage = self
            .inner
            .solve
            .as_ref()
            .ok_or(CudaFkComError::SolveUnavailable)?;
        output.clear();
        self.inner
            .driver
            .copy_from_device(output.generalized_acceleration_soa_mut(), stage.command)?;
        self.inner.driver.copy_from_device(
            output.candidate_generalized_acceleration_soa_mut(),
            stage.candidate,
        )?;
        self.inner
            .driver
            .copy_from_device(output.level_rows_soa_mut(), stage.level_rows)?;
        self.inner
            .driver
            .copy_from_device(output.level_rms_soa_mut(), stage.level_rms)?;
        self.inner.driver.copy_from_device(
            output.level_preservation_drift_soa_mut(),
            stage.level_preservation_drift,
        )?;
        self.inner.driver.copy_from_device(
            output.initial_hard_violation_mut(),
            stage.initial_hard_violation,
        )?;
        self.inner
            .driver
            .copy_from_device(output.best_hard_violation_mut(), stage.best_hard_violation)?;
        self.inner.driver.copy_from_device(
            output.final_hard_violation_mut(),
            stage.final_hard_violation,
        )?;
        self.inner.driver.copy_from_device(
            output.minimum_bound_margin_mut(),
            stage.minimum_bound_margin,
        )?;
        self.inner.driver.copy_from_device(
            output.hard_projection_sweeps_mut(),
            stage.hard_projection_sweeps,
        )?;
        self.inner
            .driver
            .copy_from_device(output.task_sweeps_mut(), stage.task_sweeps)?;
        self.inner
            .driver
            .copy_from_device(output.clipped_updates_mut(), stage.clipped_updates)?;
        self.inner
            .driver
            .copy_from_device(output.active_hard_rows_mut(), stage.active_hard_rows)?;
        self.inner
            .driver
            .copy_from_device(output.active_soft_rows_mut(), stage.active_soft_rows)?;
        self.inner
            .driver
            .copy_from_device(&mut self.status_staging, stage.status)?;
        for agent in 0..self.layout.agent_stride {
            let status = match self.status_staging[agent] {
                0 => MirrorSolveAgentStatus::Inactive,
                1 => MirrorSolveAgentStatus::Solved,
                2 => MirrorSolveAgentStatus::SolvedWithResidual,
                3 => MirrorSolveAgentStatus::MaxIterations,
                4 => MirrorSolveAgentStatus::InvalidProblem,
                5 => MirrorSolveAgentStatus::InvalidInput,
                value => return Err(CudaFkComError::InvalidStatusByte(value)),
            };
            output.set_status(agent, status);
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    #[allow(clippy::too_many_arguments)]
    fn execute_solve_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        solve_input: &MirrorSolveBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
        emission_output: &mut EmissionBatchOutput,
        solve_output: &mut MirrorSolveBatchOutput,
    ) -> Result<(), CudaFkComError> {
        if [
            fk_input.layout(),
            dynamics_input.layout(),
            emission_input.layout(),
            solve_input.layout(),
            fk_output.layout(),
            jacobian_output.layout(),
            dynamics_output.layout(),
            point_output.layout(),
            emission_output.layout(),
            solve_output.layout(),
        ]
        .into_iter()
        .any(|layout| layout != &self.layout)
        {
            return Err(CudaFkComError::LayoutMismatch);
        }
        self.launch_fk(fk_input)?;
        self.launch_jacobians()?;
        self.launch_dynamics(dynamics_input)?;
        self.launch_point_queries()?;
        self.launch_emission(emission_input, dynamics_input)?;
        self.launch_solve(solve_input)?;
        self.inner.driver.synchronize()?;
        self.copy_fk_output(fk_output)?;
        self.copy_jacobian_output(jacobian_output)?;
        self.copy_dynamics_output(dynamics_output)?;
        self.copy_point_query_output(point_output)?;
        self.copy_emission_output(emission_output)?;
        self.copy_solve_output(solve_output)
    }

    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _input: &FkBatchInput,
        _output: &mut FkBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

impl CudaMirrorKinematicsExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu_mirror: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let mut inner = CudaMirrorFkComExecutor::compile(program, cpu_mirror, device_ordinal)?;
        inner.enable_jacobians()?;
        Ok(Self { inner })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu_mirror: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }

    pub fn layout(&self) -> &BatchLayout {
        self.inner.layout()
    }

    pub fn fingerprint(&self) -> &BackendFingerprint {
        self.inner.fingerprint()
    }

    pub fn device(&self) -> &CudaDeviceInfo {
        self.inner.device()
    }

    pub fn model_hash(&self) -> [u8; 32] {
        self.inner.model_hash()
    }

    pub fn nvrtc_log(&self) -> &str {
        self.inner.nvrtc_log()
    }
    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        input: &FkBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
    ) -> Result<(), CudaFkComError> {
        self.inner
            .execute_jacobians_into(input, fk_output, jacobian_output)
    }

    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _input: &FkBatchInput,
        _fk_output: &mut FkBatchOutput,
        _jacobian_output: &mut JacobianBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

impl CudaMirrorDynamicsExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu_mirror: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let mut inner = CudaMirrorFkComExecutor::compile(program, cpu_mirror, device_ordinal)?;
        inner.enable_jacobians()?;
        inner.enable_dynamics()?;
        Ok(Self { inner })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu_mirror: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }

    pub fn layout(&self) -> &BatchLayout {
        self.inner.layout()
    }

    pub fn fingerprint(&self) -> &BackendFingerprint {
        self.inner.fingerprint()
    }

    pub fn device(&self) -> &CudaDeviceInfo {
        self.inner.device()
    }

    pub fn model_hash(&self) -> [u8; 32] {
        self.inner.model_hash()
    }

    pub fn nvrtc_log(&self) -> &str {
        self.inner.nvrtc_log()
    }

    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
    ) -> Result<(), CudaFkComError> {
        self.inner.execute_dynamics_into(
            fk_input,
            dynamics_input,
            fk_output,
            jacobian_output,
            dynamics_output,
        )
    }

    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _fk_input: &FkBatchInput,
        _dynamics_input: &DynamicsBatchInput,
        _fk_output: &mut FkBatchOutput,
        _jacobian_output: &mut JacobianBatchOutput,
        _dynamics_output: &mut DynamicsBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

impl CudaMirrorPointQueriesExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu_mirror: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let mut inner = CudaMirrorFkComExecutor::compile(program, cpu_mirror, device_ordinal)?;
        inner.enable_jacobians()?;
        inner.enable_dynamics()?;
        inner.enable_point_queries(cpu_mirror)?;
        Ok(Self { inner })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu_mirror: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }

    pub fn layout(&self) -> &BatchLayout {
        self.inner.layout()
    }

    pub fn fingerprint(&self) -> &BackendFingerprint {
        self.inner.fingerprint()
    }

    pub fn device(&self) -> &CudaDeviceInfo {
        self.inner.device()
    }

    pub fn model_hash(&self) -> [u8; 32] {
        self.inner.model_hash()
    }

    pub fn nvrtc_log(&self) -> &str {
        self.inner.nvrtc_log()
    }

    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
    ) -> Result<(), CudaFkComError> {
        self.inner.execute_point_queries_into(
            fk_input,
            dynamics_input,
            fk_output,
            jacobian_output,
            dynamics_output,
            point_output,
        )
    }

    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _fk_input: &FkBatchInput,
        _dynamics_input: &DynamicsBatchInput,
        _fk_output: &mut FkBatchOutput,
        _jacobian_output: &mut JacobianBatchOutput,
        _dynamics_output: &mut DynamicsBatchOutput,
        _point_output: &mut PointQueryBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

impl CudaMirrorEmissionExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let mut inner = CudaMirrorFkComExecutor::compile(program, cpu, device_ordinal)?;
        inner.enable_jacobians()?;
        inner.enable_dynamics()?;
        inner.enable_point_queries(cpu)?;
        inner.enable_emission(cpu)?;
        Ok(Self { inner })
    }
    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
    pub fn layout(&self) -> &BatchLayout {
        self.inner.layout()
    }
    pub fn fingerprint(&self) -> &BackendFingerprint {
        self.inner.fingerprint()
    }
    pub fn device(&self) -> &CudaDeviceInfo {
        self.inner.device()
    }
    pub fn model_hash(&self) -> [u8; 32] {
        self.inner.model_hash()
    }
    pub fn nvrtc_log(&self) -> &str {
        self.inner.nvrtc_log()
    }
    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
        emission_output: &mut EmissionBatchOutput,
    ) -> Result<(), CudaFkComError> {
        self.inner.execute_emission_into(
            fk_input,
            dynamics_input,
            emission_input,
            fk_output,
            jacobian_output,
            dynamics_output,
            point_output,
            emission_output,
        )
    }
    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _fk_input: &FkBatchInput,
        _dynamics_input: &DynamicsBatchInput,
        _emission_input: &EmissionBatchInput,
        _fk_output: &mut FkBatchOutput,
        _jacobian_output: &mut JacobianBatchOutput,
        _dynamics_output: &mut DynamicsBatchOutput,
        _point_output: &mut PointQueryBatchOutput,
        _emission_output: &mut EmissionBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

impl CudaMirrorSolveExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        program: &MotionProgram,
        cpu: &CpuMirrorExecutor,
        mirror_solver: &CpuMirrorBatchSolver,
        device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        let mut inner = CudaMirrorFkComExecutor::compile(program, cpu, device_ordinal)?;
        inner.enable_jacobians()?;
        inner.enable_dynamics()?;
        inner.enable_point_queries(cpu)?;
        inner.enable_emission(cpu)?;
        inner.enable_solve(cpu, mirror_solver)?;
        Ok(Self { inner })
    }
    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _program: &MotionProgram,
        _cpu: &CpuMirrorExecutor,
        _mirror_solver: &CpuMirrorBatchSolver,
        _device_ordinal: i32,
    ) -> Result<Self, CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
    pub fn layout(&self) -> &BatchLayout {
        self.inner.layout()
    }
    pub fn fingerprint(&self) -> &BackendFingerprint {
        self.inner.fingerprint()
    }
    pub fn device(&self) -> &CudaDeviceInfo {
        self.inner.device()
    }
    pub fn model_hash(&self) -> [u8; 32] {
        self.inner.model_hash()
    }
    pub fn nvrtc_log(&self) -> &str {
        self.inner.nvrtc_log()
    }
    pub const fn manifest(&self) -> crate::KernelManifest {
        crate::KernelManifest::solve_v1()
    }
    #[cfg(target_os = "linux")]
    #[allow(clippy::too_many_arguments)]
    pub fn execute_into(
        &mut self,
        fk_input: &FkBatchInput,
        dynamics_input: &DynamicsBatchInput,
        emission_input: &EmissionBatchInput,
        solve_input: &MirrorSolveBatchInput,
        fk_output: &mut FkBatchOutput,
        jacobian_output: &mut JacobianBatchOutput,
        dynamics_output: &mut DynamicsBatchOutput,
        point_output: &mut PointQueryBatchOutput,
        emission_output: &mut EmissionBatchOutput,
        solve_output: &mut MirrorSolveBatchOutput,
    ) -> Result<(), CudaFkComError> {
        self.inner.execute_solve_into(
            fk_input,
            dynamics_input,
            emission_input,
            solve_input,
            fk_output,
            jacobian_output,
            dynamics_output,
            point_output,
            emission_output,
            solve_output,
        )
    }
    #[cfg(not(target_os = "linux"))]
    #[allow(clippy::too_many_arguments)]
    pub fn execute_into(
        &mut self,
        _fk_input: &FkBatchInput,
        _dynamics_input: &DynamicsBatchInput,
        _emission_input: &EmissionBatchInput,
        _solve_input: &MirrorSolveBatchInput,
        _fk_output: &mut FkBatchOutput,
        _jacobian_output: &mut JacobianBatchOutput,
        _dynamics_output: &mut DynamicsBatchOutput,
        _point_output: &mut PointQueryBatchOutput,
        _emission_output: &mut EmissionBatchOutput,
        _solve_output: &mut MirrorSolveBatchOutput,
    ) -> Result<(), CudaFkComError> {
        Err(CudaFkComError::Driver(DriverError::UnsupportedPlatform))
    }
}

#[cfg(target_os = "linux")]
fn upload<T>(driver: &Driver, values: &[T]) -> Result<CuDevicePtr, CudaFkComError> {
    let pointer = allocate_elements::<T>(driver, values.len())?;
    if !values.is_empty() {
        driver.copy_to_device(pointer, values)?;
    }
    Ok(pointer)
}

#[cfg(target_os = "linux")]
fn allocate_elements<T>(driver: &Driver, elements: usize) -> Result<CuDevicePtr, CudaFkComError> {
    let bytes = elements
        .checked_mul(core::mem::size_of::<T>())
        .ok_or(CudaFkComError::SizeOverflow)?
        .max(1);
    Ok(driver.allocate(bytes)?)
}

#[cfg(target_os = "linux")]
fn abi_u32(value: usize) -> Result<u32, CudaFkComError> {
    u32::try_from(value).map_err(|_| CudaFkComError::DimensionOverflow)
}

#[cfg(target_os = "linux")]
fn combined_kernel_hash(fk_ptx: &str, model_hash: [u8; 32]) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-state-fk-com-executor-v1");
    digest.update(STATE_INPUT_PTX.as_bytes());
    digest.update(fk_ptx.as_bytes());
    digest.update(model_hash);
    digest.finalize().into()
}

#[cfg(target_os = "linux")]
fn combined_jacobian_kernel_hash(previous: [u8; 32], jacobian_ptx: &str) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-state-fk-com-jacobians-executor-v1");
    digest.update(previous);
    digest.update(jacobian_ptx.as_bytes());
    digest.finalize().into()
}

#[cfg(target_os = "linux")]
fn combined_dynamics_kernel_hash(previous: [u8; 32], dynamics_ptx: &str) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-state-kinematics-dynamics-executor-v1");
    digest.update(previous);
    digest.update(dynamics_ptx.as_bytes());
    digest.finalize().into()
}

#[cfg(target_os = "linux")]
fn combined_point_query_kernel_hash(
    previous: [u8; 32],
    point_ptx: &str,
    cpu_mirror: &CpuMirrorExecutor,
) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-state-products-point-queries-executor-v1");
    digest.update(previous);
    digest.update(point_ptx.as_bytes());
    for query in &cpu_mirror.descriptor().point_queries {
        digest.update(query.stable_id.to_le_bytes());
        digest.update((query.frame_index as u64).to_le_bytes());
        for bits in query.point_in_frame_bits {
            digest.update(bits.to_le_bytes());
        }
    }
    digest.finalize().into()
}

#[cfg(target_os = "linux")]
fn combined_emission_kernel_hash(
    previous: [u8; 32],
    ptx: &str,
    cpu: &CpuMirrorExecutor,
) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-state-products-emission-executor-v1");
    digest.update(previous);
    digest.update(ptx.as_bytes());
    for task in &cpu.descriptor().point_tasks {
        digest.update(task.stable_id.to_le_bytes());
        digest.update((task.point_query_slot as u64).to_le_bytes());
        digest.update((task.priority as u8).to_le_bytes());
        digest.update(task.weight_bits.to_le_bytes());
        digest.update(task.bandwidth_hz_bits.to_le_bytes());
    }
    for contact in &cpu.descriptor().contact_locks {
        digest.update(contact.stable_id.to_le_bytes());
        digest.update((contact.point_query_slot as u64).to_le_bytes());
        digest.update((contact.kinematic_mode as u8).to_le_bytes());
    }
    digest.finalize().into()
}

#[cfg(target_os = "linux")]
fn combined_solve_kernel_hash(
    previous: [u8; 32],
    ptx: &str,
    mirror_algorithm_hash: [u8; 32],
) -> [u8; 32] {
    let mut digest = Sha256::new();
    digest.update(b"bonesaw-cuda-fixed-level-solve-executor-v1");
    digest.update(previous);
    digest.update(ptx.as_bytes());
    digest.update(mirror_algorithm_hash);
    digest.finalize().into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AgentStatus, CudaRuntimeStatus, FkBatchInput, FkBatchOutput, allocation_sentinel,
        probe_cuda_runtime,
    };
    use bonesaw_core::{Priority, TimingSpec};

    #[test]
    fn compiler_probe_never_claims_ptx_without_success() {
        for probe in [
            probe_cuda_fk_com_compiler(7, 5),
            probe_cuda_jacobians_compiler(7, 5),
            probe_cuda_dynamics_compiler(7, 5),
            probe_cuda_point_queries_compiler(7, 5),
            probe_cuda_emission_compiler(7, 5),
            probe_cuda_solve_compiler(7, 5),
        ] {
            if probe.status == CudaFkComCompilerStatus::Available {
                assert!(probe.toolkit_version.is_some());
                assert!(probe.ptx_sha256.is_some());
                assert!(probe.ptx_bytes.is_some_and(|bytes| bytes > 0));
                assert!(probe.detail.is_none());
            } else {
                assert!(probe.ptx_sha256.is_none());
                assert!(probe.ptx_bytes.is_none());
                assert!(probe.detail.is_some());
            }
        }
    }

    #[test]
    fn device_fk_com_is_gated_then_matches_cpu_and_repeats_without_rust_allocation() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let cpu = CpuMirrorExecutor::compile(&program, 5, 32).unwrap();
        let layout = cpu.descriptor().layout.clone();
        let mut input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.coordinate_count {
            for agent in 0..4 {
                input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                    (coordinate as f32 * 0.03125) - (agent as f32 * 0.0625);
            }
        }
        for agent in 0..4 {
            input.root_pose_soa_mut()[layout.root_pose_index(9, agent)] = agent as f32 * 0.1;
        }
        input.q_soa_mut()[layout.q_index(0, 2)] = f32::NAN;
        let mut expected = FkBatchOutput::new(layout.clone());
        cpu.execute_into(&input, &mut expected).unwrap();

        let runtime = probe_cuda_runtime();
        let compiler = runtime.devices.first().map(|device| {
            probe_cuda_fk_com_compiler(
                device.compute_capability_major,
                device.compute_capability_minor,
            )
        });
        let executor = CudaMirrorFkComExecutor::compile(&program, &cpu, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compiler
                .as_ref()
                .is_none_or(|probe| probe.status != CudaFkComCompilerStatus::Available)
        {
            assert!(executor.is_err());
            return;
        }
        let mut executor = executor.expect("available driver, device, and compiler must construct");
        let mut actual = FkBatchOutput::new(layout.clone());
        executor.execute_into(&input, &mut actual).unwrap();
        assert_eq!(actual.agent_status(), expected.agent_status());
        for agent in 0..layout.agent_stride {
            if actual.agent_status()[agent] == AgentStatus::Ok {
                for body in 0..layout.body_count {
                    for (left, right) in actual
                        .body_pose(agent, body)
                        .unwrap()
                        .into_iter()
                        .zip(expected.body_pose(agent, body).unwrap())
                    {
                        assert!((left - right).abs() <= 3.0e-5, "{left} != {right}");
                    }
                }
                for (left, right) in actual
                    .center_of_mass(agent)
                    .unwrap()
                    .into_iter()
                    .zip(expected.center_of_mass(agent).unwrap())
                {
                    assert!((left - right).abs() <= 3.0e-5, "{left} != {right}");
                }
            } else {
                assert!(
                    actual
                        .body_pose_soa()
                        .iter()
                        .enumerate()
                        .filter(|(index, _)| index % layout.agent_stride == agent)
                        .all(|(_, value)| value.to_bits() == 0)
                );
            }
        }

        let mut repeated = FkBatchOutput::new(layout);
        executor.execute_into(&input, &mut repeated).unwrap();
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..10 {
                executor.execute_into(&input, &mut repeated).unwrap();
            }
        });
        assert_eq!(calls, 0);
        assert_eq!(bytes, 0);
        assert_eq!(repeated.body_pose_soa(), actual.body_pose_soa());
        assert_eq!(repeated.center_of_mass_soa(), actual.center_of_mass_soa());
        assert_eq!(repeated.total_mass(), actual.total_mass());
        assert_eq!(repeated.agent_status(), actual.agent_status());
    }

    #[test]
    fn device_jacobians_are_gated_then_match_cpu_and_repeat_without_rust_allocation() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let cpu = CpuMirrorExecutor::compile(&program, 5, 32).unwrap();
        let layout = cpu.descriptor().layout.clone();
        let mut input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.coordinate_count {
            for agent in 0..4 {
                input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                    (coordinate as f32 * 0.021) - (agent as f32 * 0.043);
            }
        }
        input.q_soa_mut()[layout.q_index(1, 2)] = f32::NAN;
        let mut expected_fk = FkBatchOutput::new(layout.clone());
        let mut expected_jacobian = JacobianBatchOutput::new(layout.clone());
        cpu.execute_into(&input, &mut expected_fk).unwrap();
        cpu.execute_jacobians_into(&expected_fk, &mut expected_jacobian)
            .unwrap();

        let runtime = probe_cuda_runtime();
        let compilers = runtime.devices.first().map(|device| {
            [
                probe_cuda_fk_com_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_jacobians_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
            ]
        });
        let executor = CudaMirrorKinematicsExecutor::compile(&program, &cpu, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compilers.as_ref().is_none_or(|probes| {
                probes
                    .iter()
                    .any(|probe| probe.status != CudaFkComCompilerStatus::Available)
            })
        {
            assert!(executor.is_err());
            return;
        }
        let mut executor = executor.expect("available device and both compilers must construct");
        let mut actual_fk = FkBatchOutput::new(layout.clone());
        let mut actual_jacobian = JacobianBatchOutput::new(layout.clone());
        executor
            .execute_into(&input, &mut actual_fk, &mut actual_jacobian)
            .unwrap();
        assert_eq!(
            actual_jacobian.agent_status(),
            expected_jacobian.agent_status()
        );
        assert_close(
            actual_jacobian.frame_jacobian_soa(),
            expected_jacobian.frame_jacobian_soa(),
            2.0e-4,
        );
        assert_close(
            actual_jacobian.center_of_mass_jacobian_soa(),
            expected_jacobian.center_of_mass_jacobian_soa(),
            2.0e-4,
        );

        let mut repeated_fk = FkBatchOutput::new(layout.clone());
        let mut repeated_jacobian = JacobianBatchOutput::new(layout);
        executor
            .execute_into(&input, &mut repeated_fk, &mut repeated_jacobian)
            .unwrap();
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..10 {
                executor
                    .execute_into(&input, &mut repeated_fk, &mut repeated_jacobian)
                    .unwrap();
            }
        });
        assert_eq!(calls, 0);
        assert_eq!(bytes, 0);
        assert_eq!(
            repeated_jacobian.frame_jacobian_soa(),
            actual_jacobian.frame_jacobian_soa()
        );
        assert_eq!(
            repeated_jacobian.center_of_mass_jacobian_soa(),
            actual_jacobian.center_of_mass_jacobian_soa()
        );
        assert_eq!(
            repeated_jacobian.agent_status(),
            actual_jacobian.agent_status()
        );
    }

    #[test]
    fn device_dynamics_are_gated_then_match_cpu_and_repeat_without_rust_allocation() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let cpu = CpuMirrorExecutor::compile(&program, 5, 32).unwrap();
        let layout = cpu.descriptor().layout.clone();
        let mut fk_input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.coordinate_count {
            for agent in 0..4 {
                fk_input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                    coordinate as f32 * 0.017 - agent as f32 * 0.029;
            }
        }
        for agent in 0..4 {
            fk_input.root_pose_soa_mut()[layout.root_pose_index(9, agent)] = agent as f32 * 0.04;
            fk_input.root_pose_soa_mut()[layout.root_pose_index(10, agent)] = agent as f32 * -0.03;
        }
        fk_input.q_soa_mut()[layout.q_index(1, 2)] = f32::NAN;

        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.generalized_coordinate_count {
            for agent in 0..4 {
                dynamics_input.generalized_velocity_soa_mut()
                    [layout.generalized_index(coordinate, agent)] =
                    coordinate as f32 * 0.023 - agent as f32 * 0.011;
            }
        }
        for agent in 0..4 {
            dynamics_input.gravity_world_soa_mut()[agent] = 0.17;
            dynamics_input.gravity_world_soa_mut()[layout.agent_stride + agent] = -0.09;
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.73;
        }
        dynamics_input.generalized_velocity_soa_mut()[layout.generalized_index(3, 1)] = f32::NAN;

        let mut expected_fk = FkBatchOutput::new(layout.clone());
        let mut expected_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut expected_dynamics = DynamicsBatchOutput::new(layout.clone());
        cpu.execute_into(&fk_input, &mut expected_fk).unwrap();
        cpu.execute_jacobians_into(&expected_fk, &mut expected_jacobian)
            .unwrap();
        cpu.execute_dynamics_into(
            &dynamics_input,
            &expected_fk,
            &expected_jacobian,
            &mut expected_dynamics,
        )
        .unwrap();

        let runtime = probe_cuda_runtime();
        let compilers = runtime.devices.first().map(|device| {
            [
                probe_cuda_fk_com_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_jacobians_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_dynamics_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
            ]
        });
        let executor = CudaMirrorDynamicsExecutor::compile(&program, &cpu, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compilers.as_ref().is_none_or(|probes| {
                probes
                    .iter()
                    .any(|probe| probe.status != CudaFkComCompilerStatus::Available)
            })
        {
            assert!(executor.is_err());
            return;
        }

        let mut executor = executor.expect("available device and three compilers must construct");
        let mut actual_fk = FkBatchOutput::new(layout.clone());
        let mut actual_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut actual_dynamics = DynamicsBatchOutput::new(layout.clone());
        executor
            .execute_into(
                &fk_input,
                &dynamics_input,
                &mut actual_fk,
                &mut actual_jacobian,
                &mut actual_dynamics,
            )
            .unwrap();
        assert_eq!(
            actual_dynamics.agent_status(),
            expected_dynamics.agent_status()
        );
        assert_close(
            actual_dynamics.mass_matrix_soa(),
            expected_dynamics.mass_matrix_soa(),
            5.0e-4,
        );
        assert_close(
            actual_dynamics.bias_force_soa(),
            expected_dynamics.bias_force_soa(),
            5.0e-4,
        );
        assert_close(
            actual_dynamics.centroidal_map_soa(),
            expected_dynamics.centroidal_map_soa(),
            5.0e-4,
        );

        let mut repeated_fk = FkBatchOutput::new(layout.clone());
        let mut repeated_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut repeated_dynamics = DynamicsBatchOutput::new(layout);
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..10 {
                executor
                    .execute_into(
                        &fk_input,
                        &dynamics_input,
                        &mut repeated_fk,
                        &mut repeated_jacobian,
                        &mut repeated_dynamics,
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        assert_eq!(
            repeated_dynamics.mass_matrix_soa(),
            actual_dynamics.mass_matrix_soa()
        );
        assert_eq!(
            repeated_dynamics.bias_force_soa(),
            actual_dynamics.bias_force_soa()
        );
        assert_eq!(
            repeated_dynamics.centroidal_map_soa(),
            actual_dynamics.centroidal_map_soa()
        );
        assert_eq!(
            repeated_dynamics.agent_status(),
            actual_dynamics.agent_status()
        );
    }

    #[test]
    fn device_point_queries_are_gated_then_match_cpu_and_repeat_without_rust_allocation() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let body_count = program.model.bodies.len();
        let query_specs = [
            crate::PointQuerySpec {
                stable_id: 91_001,
                frame_index: program.model.root.0,
                point_in_frame: [0.13, -0.07, 0.19],
            },
            crate::PointQuerySpec {
                stable_id: 91_002,
                frame_index: 1.min(body_count - 1),
                point_in_frame: [-0.04, 0.08, 0.11],
            },
            crate::PointQuerySpec {
                stable_id: 91_003,
                frame_index: body_count / 2,
                point_in_frame: [0.03, -0.05, -0.09],
            },
            crate::PointQuerySpec {
                stable_id: 91_004,
                frame_index: body_count - 1,
                point_in_frame: [0.06, 0.02, -0.03],
            },
        ];
        let cpu =
            CpuMirrorExecutor::compile_with_point_queries(&program, 5, 32, &query_specs).unwrap();
        let layout = cpu.descriptor().layout.clone();
        let mut fk_input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.coordinate_count {
            for agent in 0..4 {
                fk_input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                    coordinate as f32 * 0.017 - agent as f32 * 0.029;
            }
        }
        for agent in 0..4 {
            fk_input.root_pose_soa_mut()[layout.root_pose_index(9, agent)] = agent as f32 * 0.04;
            fk_input.root_pose_soa_mut()[layout.root_pose_index(10, agent)] = agent as f32 * -0.03;
        }
        fk_input.q_soa_mut()[layout.q_index(1, 2)] = f32::NAN;

        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.generalized_coordinate_count {
            for agent in 0..4 {
                dynamics_input.generalized_velocity_soa_mut()
                    [layout.generalized_index(coordinate, agent)] =
                    coordinate as f32 * 0.023 - agent as f32 * 0.011;
            }
        }
        for agent in 0..4 {
            dynamics_input.gravity_world_soa_mut()[agent] = 0.17;
            dynamics_input.gravity_world_soa_mut()[layout.agent_stride + agent] = -0.09;
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.73;
        }
        dynamics_input.generalized_velocity_soa_mut()[layout.generalized_index(3, 1)] = f32::NAN;

        let mut expected_fk = FkBatchOutput::new(layout.clone());
        let mut expected_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut expected_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut expected_points = PointQueryBatchOutput::new(layout.clone());
        cpu.execute_into(&fk_input, &mut expected_fk).unwrap();
        cpu.execute_jacobians_into(&expected_fk, &mut expected_jacobian)
            .unwrap();
        cpu.execute_dynamics_into(
            &dynamics_input,
            &expected_fk,
            &expected_jacobian,
            &mut expected_dynamics,
        )
        .unwrap();
        cpu.execute_point_queries_into(
            &expected_fk,
            &expected_jacobian,
            &expected_dynamics,
            &mut expected_points,
        )
        .unwrap();

        let runtime = probe_cuda_runtime();
        let compilers = runtime.devices.first().map(|device| {
            [
                probe_cuda_fk_com_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_jacobians_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_dynamics_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_point_queries_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
            ]
        });
        let executor = CudaMirrorPointQueriesExecutor::compile(&program, &cpu, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compilers.as_ref().is_none_or(|probes| {
                probes
                    .iter()
                    .any(|probe| probe.status != CudaFkComCompilerStatus::Available)
            })
        {
            assert!(executor.is_err());
            return;
        }

        let mut executor = executor.expect("available device and four compilers must construct");
        let mut actual_fk = FkBatchOutput::new(layout.clone());
        let mut actual_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut actual_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut actual_points = PointQueryBatchOutput::new(layout.clone());
        executor
            .execute_into(
                &fk_input,
                &dynamics_input,
                &mut actual_fk,
                &mut actual_jacobian,
                &mut actual_dynamics,
                &mut actual_points,
            )
            .unwrap();
        assert_eq!(actual_points.agent_status(), expected_points.agent_status());
        assert_close(
            actual_points.point_position_soa(),
            expected_points.point_position_soa(),
            2.0e-4,
        );
        assert_close(
            actual_points.point_jacobian_soa(),
            expected_points.point_jacobian_soa(),
            2.0e-4,
        );
        assert_close(
            actual_points.point_bias_acceleration_soa(),
            expected_points.point_bias_acceleration_soa(),
            2.0e-4,
        );

        let mut repeated_fk = FkBatchOutput::new(layout.clone());
        let mut repeated_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut repeated_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut repeated_points = PointQueryBatchOutput::new(layout);
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..10 {
                executor
                    .execute_into(
                        &fk_input,
                        &dynamics_input,
                        &mut repeated_fk,
                        &mut repeated_jacobian,
                        &mut repeated_dynamics,
                        &mut repeated_points,
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        assert_eq!(
            repeated_points.point_position_soa(),
            actual_points.point_position_soa()
        );
        assert_eq!(
            repeated_points.point_jacobian_soa(),
            actual_points.point_jacobian_soa()
        );
        assert_eq!(
            repeated_points.point_bias_acceleration_soa(),
            actual_points.point_bias_acceleration_soa()
        );
        assert_eq!(repeated_points.agent_status(), actual_points.agent_status());
    }

    #[test]
    fn device_emission_is_gated_then_matches_cpu_rows() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let last = program.model.bodies.len() - 1;
        let queries = [
            crate::PointQuerySpec {
                stable_id: 96_001,
                frame_index: program.model.root.0,
                point_in_frame: [0.11, -0.04, 0.17],
            },
            crate::PointQuerySpec {
                stable_id: 96_002,
                frame_index: last,
                point_in_frame: [0.02, 0.0, -0.01],
            },
        ];
        let tasks = [crate::PointAttractorSpec {
            stable_id: 96_101,
            point_query_stable_id: 96_001,
            priority: Priority::Intent,
            weight: 1.5,
            bandwidth_hz: 2.25,
        }];
        let contacts = [crate::ContactLockSpec {
            stable_id: 96_201,
            point_query_stable_id: 96_002,
        }];
        let cpu = CpuMirrorExecutor::compile_with_emission_plan(
            &program, 3, 32, &queries, &tasks, &contacts,
        )
        .unwrap();
        let layout = cpu.descriptor().layout.clone();
        let mut fk_input = FkBatchInput::new(layout.clone(), 2).unwrap();
        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            for coordinate in 0..layout.coordinate_count {
                fk_input.q_soa_mut()[layout.q_index(coordinate, agent)] =
                    coordinate as f32 * 0.013 - agent as f32 * 0.021;
            }
            for coordinate in 0..layout.generalized_coordinate_count {
                dynamics_input.generalized_velocity_soa_mut()
                    [layout.generalized_index(coordinate, agent)] =
                    coordinate as f32 * 0.019 - agent as f32 * 0.017;
            }
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.81;
        }
        let mut emission_input = EmissionBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            emission_input.point_task_active_soa_mut()[layout.task_active_index(0, agent)] = 1;
            emission_input.contact_lock_active_soa_mut()[layout.contact_active_index(0, agent)] = 1;
            for component in 0..3 {
                let task_index = layout.task_vector_index(0, component, agent);
                emission_input.point_target_position_soa_mut()[task_index] =
                    0.1 + agent as f32 * 0.02 + component as f32 * 0.01;
                emission_input.point_target_velocity_soa_mut()[task_index] =
                    -0.03 + component as f32 * 0.005;
                emission_input.point_target_acceleration_soa_mut()[task_index] =
                    0.02 - agent as f32 * 0.004;
                let contact_index = layout.contact_vector_index(0, component, agent);
                emission_input.contact_desired_acceleration_soa_mut()[contact_index] =
                    component as f32 * 0.003;
            }
        }
        let mut expected_fk = FkBatchOutput::new(layout.clone());
        let mut expected_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut expected_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut expected_points = PointQueryBatchOutput::new(layout.clone());
        let mut expected_emission = EmissionBatchOutput::new(layout.clone());
        cpu.execute_into(&fk_input, &mut expected_fk).unwrap();
        cpu.execute_jacobians_into(&expected_fk, &mut expected_jacobian)
            .unwrap();
        cpu.execute_dynamics_into(
            &dynamics_input,
            &expected_fk,
            &expected_jacobian,
            &mut expected_dynamics,
        )
        .unwrap();
        cpu.execute_point_queries_into(
            &expected_fk,
            &expected_jacobian,
            &expected_dynamics,
            &mut expected_points,
        )
        .unwrap();
        cpu.execute_emission_into(
            &emission_input,
            &dynamics_input,
            &expected_points,
            &mut expected_emission,
        )
        .unwrap();

        let runtime = probe_cuda_runtime();
        let compilers = runtime.devices.first().map(|device| {
            [
                probe_cuda_fk_com_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_jacobians_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_dynamics_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_point_queries_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_emission_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
            ]
        });
        let executor = CudaMirrorEmissionExecutor::compile(&program, &cpu, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compilers.as_ref().is_none_or(|probes| {
                probes
                    .iter()
                    .any(|probe| probe.status != CudaFkComCompilerStatus::Available)
            })
        {
            assert!(executor.is_err());
            return;
        }
        let mut executor = executor.unwrap();
        let mut actual_fk = FkBatchOutput::new(layout.clone());
        let mut actual_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut actual_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut actual_points = PointQueryBatchOutput::new(layout.clone());
        let mut actual_emission = EmissionBatchOutput::new(layout);
        executor
            .execute_into(
                &fk_input,
                &dynamics_input,
                &emission_input,
                &mut actual_fk,
                &mut actual_jacobian,
                &mut actual_dynamics,
                &mut actual_points,
                &mut actual_emission,
            )
            .unwrap();
        assert_eq!(
            actual_emission.agent_status(),
            expected_emission.agent_status()
        );
        assert_eq!(
            actual_emission.point_task_active_soa(),
            expected_emission.point_task_active_soa()
        );
        assert_eq!(
            actual_emission.contact_lock_active_soa(),
            expected_emission.contact_lock_active_soa()
        );
        for (actual, expected) in [
            (
                actual_emission.point_task_position_error_soa(),
                expected_emission.point_task_position_error_soa(),
            ),
            (
                actual_emission.point_task_velocity_error_soa(),
                expected_emission.point_task_velocity_error_soa(),
            ),
            (
                actual_emission.point_task_desired_acceleration_soa(),
                expected_emission.point_task_desired_acceleration_soa(),
            ),
            (
                actual_emission.point_task_jacobian_soa(),
                expected_emission.point_task_jacobian_soa(),
            ),
            (
                actual_emission.point_task_rhs_soa(),
                expected_emission.point_task_rhs_soa(),
            ),
            (
                actual_emission.contact_lock_jacobian_soa(),
                expected_emission.contact_lock_jacobian_soa(),
            ),
            (
                actual_emission.contact_lock_rhs_soa(),
                expected_emission.contact_lock_rhs_soa(),
            ),
        ] {
            assert_close(actual, expected, 5.0e-4);
        }
    }

    #[test]
    fn device_solve_is_gated_then_matches_fixed_level_cpu_mirror() {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let queries = [crate::PointQuerySpec {
            stable_id: 98_001,
            frame_index: program.model.root.0,
            point_in_frame: [0.0, 0.0, 0.0],
        }];
        let tasks = [
            crate::PointAttractorSpec {
                stable_id: 98_101,
                point_query_stable_id: 98_001,
                priority: Priority::Viability,
                weight: 1.0,
                bandwidth_hz: 1.0,
            },
            crate::PointAttractorSpec {
                stable_id: 98_102,
                point_query_stable_id: 98_001,
                priority: Priority::Intent,
                weight: 1.0,
                bandwidth_hz: 1.0,
            },
        ];
        let contacts = [crate::ContactLockSpec {
            stable_id: 98_201,
            point_query_stable_id: 98_001,
        }];
        let cpu = CpuMirrorExecutor::compile_with_contact_modes(
            &program,
            3,
            32,
            &queries,
            &tasks,
            &contacts,
            &[crate::ContactKinematicMode::NormalPoint],
        )
        .unwrap();
        let layout = cpu.descriptor().layout.clone();
        let fk_input = FkBatchInput::new(layout.clone(), 2).unwrap();
        let mut dynamics_input = DynamicsBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            dynamics_input.gravity_world_soa_mut()[2 * layout.agent_stride + agent] = -9.81;
        }
        let mut emission_input = EmissionBatchInput::new(layout.clone(), 2).unwrap();
        for agent in 0..2 {
            for task in 0..2 {
                emission_input.point_task_active_soa_mut()[layout.task_active_index(task, agent)] =
                    1;
                emission_input.point_target_acceleration_soa_mut()
                    [layout.task_vector_index(task, 0, agent)] = if task == 0 { 1.0 } else { -1.0 };
            }
            emission_input.contact_lock_active_soa_mut()[layout.contact_active_index(0, agent)] = 1;
            emission_input.contact_desired_acceleration_soa_mut()
                [layout.contact_vector_index(0, 2, agent)] = 0.25;
        }
        let mut solve_input = MirrorSolveBatchInput::new(layout.clone(), 2).unwrap();
        solve_input.lower_soa_mut().fill(-2.0);
        solve_input.upper_soa_mut().fill(2.0);
        let invalid = layout.generalized_index(0, 1);
        solve_input.lower_soa_mut()[invalid] = 1.0;
        solve_input.upper_soa_mut()[invalid] = -1.0;

        let mut expected_fk = FkBatchOutput::new(layout.clone());
        let mut expected_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut expected_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut expected_points = PointQueryBatchOutput::new(layout.clone());
        let mut expected_emission = EmissionBatchOutput::new(layout.clone());
        cpu.execute_into(&fk_input, &mut expected_fk).unwrap();
        cpu.execute_jacobians_into(&expected_fk, &mut expected_jacobian)
            .unwrap();
        cpu.execute_dynamics_into(
            &dynamics_input,
            &expected_fk,
            &expected_jacobian,
            &mut expected_dynamics,
        )
        .unwrap();
        cpu.execute_point_queries_into(
            &expected_fk,
            &expected_jacobian,
            &expected_dynamics,
            &mut expected_points,
        )
        .unwrap();
        cpu.execute_emission_into(
            &emission_input,
            &dynamics_input,
            &expected_points,
            &mut expected_emission,
        )
        .unwrap();
        let mut mirror_solver = CpuMirrorBatchSolver::new(cpu.descriptor()).unwrap();
        let mut expected_solve = MirrorSolveBatchOutput::new(layout.clone());
        mirror_solver
            .execute_into(&expected_emission, &solve_input, &mut expected_solve)
            .unwrap();

        let runtime = probe_cuda_runtime();
        let compilers = runtime.devices.first().map(|device| {
            [
                probe_cuda_fk_com_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_jacobians_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_dynamics_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_point_queries_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_emission_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
                probe_cuda_solve_compiler(
                    device.compute_capability_major,
                    device.compute_capability_minor,
                ),
            ]
        });
        let executor = CudaMirrorSolveExecutor::compile(&program, &cpu, &mirror_solver, 0);
        if runtime.status != CudaRuntimeStatus::Available
            || compilers.as_ref().is_none_or(|probes| {
                probes
                    .iter()
                    .any(|probe| probe.status != CudaFkComCompilerStatus::Available)
            })
        {
            assert!(executor.is_err());
            return;
        }
        let mut executor = executor.unwrap();
        let mut actual_fk = FkBatchOutput::new(layout.clone());
        let mut actual_jacobian = JacobianBatchOutput::new(layout.clone());
        let mut actual_dynamics = DynamicsBatchOutput::new(layout.clone());
        let mut actual_points = PointQueryBatchOutput::new(layout.clone());
        let mut actual_emission = EmissionBatchOutput::new(layout.clone());
        let mut actual_solve = MirrorSolveBatchOutput::new(layout.clone());
        executor
            .execute_into(
                &fk_input,
                &dynamics_input,
                &emission_input,
                &solve_input,
                &mut actual_fk,
                &mut actual_jacobian,
                &mut actual_dynamics,
                &mut actual_points,
                &mut actual_emission,
                &mut actual_solve,
            )
            .unwrap();
        assert_eq!(actual_solve.status(), expected_solve.status());
        assert_eq!(
            actual_solve.level_rows_soa(),
            expected_solve.level_rows_soa()
        );
        assert_eq!(
            actual_solve.hard_projection_sweeps(),
            expected_solve.hard_projection_sweeps()
        );
        assert_eq!(actual_solve.task_sweeps(), expected_solve.task_sweeps());
        for (actual, expected) in [
            (
                actual_solve.generalized_acceleration_soa(),
                expected_solve.generalized_acceleration_soa(),
            ),
            (
                actual_solve.candidate_generalized_acceleration_soa(),
                expected_solve.candidate_generalized_acceleration_soa(),
            ),
            (actual_solve.level_rms_soa(), expected_solve.level_rms_soa()),
            (
                actual_solve.level_preservation_drift_soa(),
                expected_solve.level_preservation_drift_soa(),
            ),
            (
                actual_solve.initial_hard_violation(),
                expected_solve.initial_hard_violation(),
            ),
            (
                actual_solve.best_hard_violation(),
                expected_solve.best_hard_violation(),
            ),
            (
                actual_solve.final_hard_violation(),
                expected_solve.final_hard_violation(),
            ),
        ] {
            assert_close(actual, expected, 5.0e-4);
        }

        let mut repeated_solve = MirrorSolveBatchOutput::new(layout.clone());
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..10 {
                executor
                    .execute_into(
                        &fk_input,
                        &dynamics_input,
                        &emission_input,
                        &solve_input,
                        &mut actual_fk,
                        &mut actual_jacobian,
                        &mut actual_dynamics,
                        &mut actual_points,
                        &mut actual_emission,
                        &mut repeated_solve,
                    )
                    .unwrap();
            }
        });
        assert_eq!((calls, bytes), (0, 0));
        assert_eq!(
            repeated_solve.generalized_acceleration_soa(),
            actual_solve.generalized_acceleration_soa()
        );
        assert_eq!(
            repeated_solve.candidate_generalized_acceleration_soa(),
            actual_solve.candidate_generalized_acceleration_soa()
        );
        assert_eq!(repeated_solve.status(), actual_solve.status());
    }

    fn assert_close(actual: &[f32], expected: &[f32], tolerance: f32) {
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.iter().copied().zip(expected.iter().copied()) {
            let bound = tolerance + tolerance * actual.abs().max(expected.abs());
            assert!(
                (actual - expected).abs() <= bound,
                "{actual} != {expected} (bound {bound})"
            );
        }
    }
}
