//! First real `CudaMirrorF32` stage: fixed-layout state ingest.
//!
//! This stage is intentionally small and independently admissible. It proves
//! the driver boundary, device memory ownership, device-side finite checking,
//! deterministic padding, per-agent isolation, and runtime fingerprinting
//! before any FK or solver claim. It never falls back to CPU execution.

use bonesaw_core::MotionProgram;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{
    AgentStatus, BackendFingerprint, BackendProfile, BatchLayout, CpuMirrorExecutor, FkBatchInput,
    ScalarFormat, Version,
};

#[cfg(target_os = "linux")]
use crate::cuda_driver::{
    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR,
    CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, CuContext, CuDevicePtr, CuFunction, CuModule, Driver,
};
use crate::cuda_driver::{CUDA_ERROR_NO_DEVICE, DriverError};

pub(crate) const STATE_INPUT_PTX: &str = include_str!("kernels/state_input_v1.ptx");
pub(crate) const STATE_INPUT_FUNCTION: &str = "bonesaw_state_input_v1";
pub(crate) const BLOCK_SIZE: u32 = 128;

pub fn cuda_state_input_kernel_sha256() -> [u8; 32] {
    Sha256::digest(STATE_INPUT_PTX.as_bytes()).into()
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CudaRuntimeStatus {
    Available,
    UnsupportedPlatform,
    DriverLibraryUnavailable,
    DriverSymbolUnavailable { symbol: String },
    DriverInitializationFailed { code: i32 },
    NoDevice,
    DriverQueryFailed { operation: String, code: i32 },
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct CudaDeviceInfo {
    pub ordinal: i32,
    pub name: String,
    pub compute_capability_major: i32,
    pub compute_capability_minor: i32,
    pub multiprocessor_count: i32,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct CudaRuntimeProbe {
    pub status: CudaRuntimeStatus,
    pub driver_version: Option<Version>,
    pub devices: Vec<CudaDeviceInfo>,
}

#[derive(Debug, Error)]
pub enum CudaStateInputError {
    #[error(transparent)]
    Driver(#[from] DriverError),
    #[error("input or output layout does not match the CUDA executor")]
    LayoutMismatch,
    #[error("CUDA buffer size overflows usize")]
    SizeOverflow,
    #[error("CUDA dimensions exceed the u32 kernel ABI")]
    DimensionOverflow,
    #[error("program does not match the state-input batch layout")]
    ProgramMismatch,
}

#[derive(Clone, Debug)]
pub struct CudaStateInputOutput {
    layout: BatchLayout,
    q_soa: Vec<f32>,
    root_pose_soa: Vec<f32>,
    status: Vec<u8>,
}

impl CudaStateInputOutput {
    pub fn new(layout: BatchLayout) -> Self {
        Self {
            q_soa: vec![0.0; layout.q_len().expect("validated batch layout")],
            root_pose_soa: vec![0.0; layout.root_pose_len().expect("validated batch layout")],
            status: vec![AgentStatus::Inactive as u8; layout.agent_stride],
            layout,
        }
    }

    pub fn layout(&self) -> &BatchLayout {
        &self.layout
    }

    pub fn q_soa(&self) -> &[f32] {
        &self.q_soa
    }

    pub fn root_pose_soa(&self) -> &[f32] {
        &self.root_pose_soa
    }

    pub fn status_bytes(&self) -> &[u8] {
        &self.status
    }

    pub fn status(&self, agent: usize) -> Option<AgentStatus> {
        match *self.status.get(agent)? {
            value if value == AgentStatus::Inactive as u8 => Some(AgentStatus::Inactive),
            value if value == AgentStatus::Ok as u8 => Some(AgentStatus::Ok),
            value if value == AgentStatus::InvalidInput as u8 => Some(AgentStatus::InvalidInput),
            _ => None,
        }
    }
}

/// Arithmetic and masking reference for the device state-input kernel.
#[derive(Clone, Debug)]
pub struct CpuMirrorStateInputExecutor {
    layout: BatchLayout,
}

impl CpuMirrorStateInputExecutor {
    pub fn compile(
        program: &MotionProgram,
        layout: BatchLayout,
    ) -> Result<Self, CudaStateInputError> {
        if layout.program_fingerprint != program.header.fingerprint_sha256
            || layout.coordinate_count != program.model.dof
        {
            return Err(CudaStateInputError::ProgramMismatch);
        }
        Ok(Self { layout })
    }

    pub fn execute_into(
        &self,
        input: &FkBatchInput,
        output: &mut CudaStateInputOutput,
    ) -> Result<(), CudaStateInputError> {
        if input.layout() != &self.layout || output.layout() != &self.layout {
            return Err(CudaStateInputError::LayoutMismatch);
        }
        let stride = self.layout.agent_stride;
        for agent in 0..stride {
            let active = agent < input.active_agents();
            let mut finite = active;
            if active {
                for coordinate in 0..self.layout.coordinate_count {
                    finite &= input.q_soa()[self.layout.q_index(coordinate, agent)].is_finite();
                }
                for component in 0..self.layout.pose_components {
                    finite &= input.root_pose_soa()[self.layout.root_pose_index(component, agent)]
                        .is_finite();
                }
            }
            output.status[agent] = if !active {
                AgentStatus::Inactive as u8
            } else if finite {
                AgentStatus::Ok as u8
            } else {
                AgentStatus::InvalidInput as u8
            };
            for coordinate in 0..self.layout.coordinate_count {
                let index = self.layout.q_index(coordinate, agent);
                output.q_soa[index] = if finite { input.q_soa()[index] } else { 0.0 };
            }
            for component in 0..self.layout.pose_components {
                let index = self.layout.root_pose_index(component, agent);
                output.root_pose_soa[index] = if finite {
                    input.root_pose_soa()[index]
                } else {
                    0.0
                };
            }
        }
        Ok(())
    }
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct LinuxExecutor {
    driver: Driver,
    context: CuContext,
    module: CuModule,
    function: CuFunction,
    q_input: CuDevicePtr,
    root_input: CuDevicePtr,
    q_output: CuDevicePtr,
    root_output: CuDevicePtr,
    status_output: CuDevicePtr,
}

#[cfg(target_os = "linux")]
impl Drop for LinuxExecutor {
    fn drop(&mut self) {
        let _ = self.driver.set_current(self.context);
        self.driver.free(self.status_output);
        self.driver.free(self.root_output);
        self.driver.free(self.q_output);
        self.driver.free(self.root_input);
        self.driver.free(self.q_input);
        self.driver.unload_module(self.module);
        self.driver.destroy_context(self.context);
    }
}

/// Direct-launch device executor for the first certified CUDA stage.
///
/// Construction owns a CUDA context, JIT-loaded PTX module, and fixed device
/// buffers. `execute_into` performs no host allocation and has no fallback.
#[derive(Debug)]
pub struct CudaMirrorStateInputExecutor {
    layout: BatchLayout,
    fingerprint: BackendFingerprint,
    device: CudaDeviceInfo,
    #[cfg(target_os = "linux")]
    inner: LinuxExecutor,
}

impl CudaMirrorStateInputExecutor {
    #[cfg(target_os = "linux")]
    pub fn compile(
        cpu_mirror: &CpuMirrorExecutor,
        device_ordinal: i32,
    ) -> Result<Self, CudaStateInputError> {
        let layout = cpu_mirror.descriptor().layout.clone();
        let driver = Driver::load()?;
        driver.initialize()?;
        let raw_device = driver.device(device_ordinal)?;
        let device = query_device_info(&driver, raw_device, device_ordinal)?;
        let driver_version = driver.driver_version()?;
        let context = driver.create_context(raw_device)?;
        let mut inner = LinuxExecutor {
            driver,
            context,
            module: core::ptr::null_mut(),
            function: core::ptr::null_mut(),
            q_input: 0,
            root_input: 0,
            q_output: 0,
            root_output: 0,
            status_output: 0,
        };
        inner.module = inner.driver.load_module(STATE_INPUT_PTX)?;
        inner.function = inner.driver.function(inner.module, STATE_INPUT_FUNCTION)?;
        let q_bytes = byte_count::<f32>(
            layout
                .q_len()
                .map_err(|_| CudaStateInputError::SizeOverflow)?,
        )?;
        let root_bytes = byte_count::<f32>(
            layout
                .root_pose_len()
                .map_err(|_| CudaStateInputError::SizeOverflow)?,
        )?;
        inner.q_input = inner.driver.allocate(q_bytes)?;
        inner.root_input = inner.driver.allocate(root_bytes)?;
        inner.q_output = inner.driver.allocate(q_bytes)?;
        inner.root_output = inner.driver.allocate(root_bytes)?;
        inner.status_output = inner.driver.allocate(layout.agent_stride)?;

        let mut fingerprint = cpu_mirror.fingerprint().clone();
        fingerprint.backend_profile = BackendProfile::CudaMirrorF32;
        fingerprint.scalar_format = ScalarFormat::F32;
        fingerprint.cpu_isa = None;
        fingerprint.cuda_driver = Some(version_from_driver(driver_version));
        fingerprint.gpu_architecture = Some(format!(
            "sm_{}{}",
            device.compute_capability_major, device.compute_capability_minor
        ));
        fingerprint.gpu_sm_count = Some(device.multiprocessor_count as u32);
        fingerprint.kernel_hash = cuda_state_input_kernel_sha256();
        Ok(Self {
            layout,
            fingerprint,
            device,
            inner,
        })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn compile(
        _cpu_mirror: &CpuMirrorExecutor,
        _device_ordinal: i32,
    ) -> Result<Self, CudaStateInputError> {
        Err(CudaStateInputError::Driver(
            DriverError::UnsupportedPlatform,
        ))
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

    #[cfg(target_os = "linux")]
    pub fn execute_into(
        &mut self,
        input: &FkBatchInput,
        output: &mut CudaStateInputOutput,
    ) -> Result<(), CudaStateInputError> {
        if input.layout() != &self.layout || output.layout() != &self.layout {
            return Err(CudaStateInputError::LayoutMismatch);
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
        let mut active_agents = u32::try_from(input.active_agents())
            .map_err(|_| CudaStateInputError::DimensionOverflow)?;
        let mut coordinate_count = u32::try_from(self.layout.coordinate_count)
            .map_err(|_| CudaStateInputError::DimensionOverflow)?;
        let mut agent_stride = u32::try_from(self.layout.agent_stride)
            .map_err(|_| CudaStateInputError::DimensionOverflow)?;
        let mut q_output = self.inner.q_output;
        let mut root_output = self.inner.root_output;
        let mut status_output = self.inner.status_output;
        let mut parameters = [
            (&mut q_input as *mut CuDevicePtr).cast(),
            (&mut root_input as *mut CuDevicePtr).cast(),
            (&mut active_agents as *mut u32).cast(),
            (&mut coordinate_count as *mut u32).cast(),
            (&mut agent_stride as *mut u32).cast(),
            (&mut q_output as *mut CuDevicePtr).cast(),
            (&mut root_output as *mut CuDevicePtr).cast(),
            (&mut status_output as *mut CuDevicePtr).cast(),
        ];
        let grid = agent_stride.div_ceil(BLOCK_SIZE);
        self.inner
            .driver
            .launch(self.inner.function, grid, BLOCK_SIZE, &mut parameters)?;
        self.inner.driver.synchronize()?;
        self.inner
            .driver
            .copy_from_device(&mut output.q_soa, self.inner.q_output)?;
        self.inner
            .driver
            .copy_from_device(&mut output.root_pose_soa, self.inner.root_output)?;
        self.inner
            .driver
            .copy_from_device(&mut output.status, self.inner.status_output)?;
        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    pub fn execute_into(
        &mut self,
        _input: &FkBatchInput,
        _output: &mut CudaStateInputOutput,
    ) -> Result<(), CudaStateInputError> {
        Err(CudaStateInputError::Driver(
            DriverError::UnsupportedPlatform,
        ))
    }
}

pub fn probe_cuda_runtime() -> CudaRuntimeProbe {
    #[cfg(target_os = "linux")]
    {
        let driver = match Driver::load() {
            Ok(driver) => driver,
            Err(error) => return probe_from_error(error),
        };
        if let Err(error) = driver.initialize() {
            return probe_from_error(error);
        }
        let driver_version = driver.driver_version().ok().map(version_from_driver);
        let count = match driver.device_count() {
            Ok(0) => {
                return CudaRuntimeProbe {
                    status: CudaRuntimeStatus::NoDevice,
                    driver_version,
                    devices: Vec::new(),
                };
            }
            Ok(count) => count,
            Err(error) => return probe_from_error(error),
        };
        let mut devices = Vec::with_capacity(count as usize);
        for ordinal in 0..count {
            let result = driver
                .device(ordinal)
                .and_then(|device| query_device_info(&driver, device, ordinal));
            match result {
                Ok(info) => devices.push(info),
                Err(error) => return probe_from_error(error),
            }
        }
        CudaRuntimeProbe {
            status: CudaRuntimeStatus::Available,
            driver_version,
            devices,
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        CudaRuntimeProbe {
            status: CudaRuntimeStatus::UnsupportedPlatform,
            driver_version: None,
            devices: Vec::new(),
        }
    }
}

#[cfg(target_os = "linux")]
pub(crate) fn query_device_info(
    driver: &Driver,
    device: crate::cuda_driver::CuDevice,
    ordinal: i32,
) -> Result<CudaDeviceInfo, DriverError> {
    Ok(CudaDeviceInfo {
        ordinal,
        name: driver.device_name(device)?,
        compute_capability_major: driver
            .device_attribute(device, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR)?,
        compute_capability_minor: driver
            .device_attribute(device, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR)?,
        multiprocessor_count: driver
            .device_attribute(device, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT)?,
    })
}

pub(crate) fn version_from_driver(value: i32) -> Version {
    Version {
        major: (value / 1_000).max(0) as u32,
        minor: ((value % 1_000) / 10).max(0) as u32,
        patch: 0,
    }
}

fn probe_from_error(error: DriverError) -> CudaRuntimeProbe {
    let status = match error {
        #[cfg(target_os = "linux")]
        DriverError::LibraryUnavailable => CudaRuntimeStatus::DriverLibraryUnavailable,
        #[cfg(target_os = "linux")]
        DriverError::MissingSymbol(symbol) => CudaRuntimeStatus::DriverSymbolUnavailable {
            symbol: symbol.to_owned(),
        },
        #[cfg(target_os = "linux")]
        DriverError::Call {
            operation: "cuInit",
            code,
        } if code == CUDA_ERROR_NO_DEVICE => CudaRuntimeStatus::NoDevice,
        #[cfg(target_os = "linux")]
        DriverError::Call {
            operation: "cuInit",
            code,
        } => CudaRuntimeStatus::DriverInitializationFailed { code },
        #[cfg(target_os = "linux")]
        DriverError::Call { operation, code } => CudaRuntimeStatus::DriverQueryFailed {
            operation: operation.to_owned(),
            code,
        },
        #[cfg(target_os = "linux")]
        DriverError::DeviceOrdinal { .. } | DriverError::KernelNul => {
            CudaRuntimeStatus::DriverQueryFailed {
                operation: error.to_string(),
                code: -1,
            }
        }
        #[cfg(not(target_os = "linux"))]
        DriverError::UnsupportedPlatform => CudaRuntimeStatus::UnsupportedPlatform,
    };
    CudaRuntimeProbe {
        status,
        driver_version: None,
        devices: Vec::new(),
    }
}

fn byte_count<T>(elements: usize) -> Result<usize, CudaStateInputError> {
    elements
        .checked_mul(core::mem::size_of::<T>())
        .ok_or(CudaStateInputError::SizeOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BatchLayout, allocation_sentinel};
    use bonesaw_core::TimingSpec;

    fn fixture() -> (MotionProgram, BatchLayout, FkBatchInput) {
        let program = MotionProgram::compile_urdf(
            include_str!("../../../models/toy_humanoid.urdf"),
            TimingSpec::default(),
            1,
        )
        .unwrap();
        let layout = BatchLayout::compile(&program, 5, 32).unwrap();
        let mut input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for coordinate in 0..layout.coordinate_count {
            for agent in 0..4 {
                let index = layout.q_index(coordinate, agent);
                input.q_soa_mut()[index] = coordinate as f32 * 0.01 + agent as f32;
            }
        }
        (program, layout, input)
    }

    #[test]
    fn cpu_state_input_is_bitwise_repeatable_isolated_and_allocation_free() {
        let (program, layout, mut input) = fixture();
        input.q_soa_mut()[layout.q_index(2, 2)] = f32::NAN;
        let executor = CpuMirrorStateInputExecutor::compile(&program, layout.clone()).unwrap();
        let mut first = CudaStateInputOutput::new(layout.clone());
        executor.execute_into(&input, &mut first).unwrap();
        assert_eq!(first.status(0), Some(AgentStatus::Ok));
        assert_eq!(first.status(2), Some(AgentStatus::InvalidInput));
        assert_eq!(first.status(4), Some(AgentStatus::Inactive));
        for coordinate in 0..layout.coordinate_count {
            assert_eq!(first.q_soa()[layout.q_index(coordinate, 2)].to_bits(), 0);
        }
        let mut second = CudaStateInputOutput::new(layout.clone());
        let (_, calls, bytes) = allocation_sentinel::measure(|| {
            for _ in 0..100 {
                executor.execute_into(&input, &mut second).unwrap();
            }
        });
        assert_eq!(calls, 0);
        assert_eq!(bytes, 0);
        assert_eq!(first.q_soa(), second.q_soa());
        assert_eq!(first.root_pose_soa(), second.root_pose_soa());
        assert_eq!(first.status_bytes(), second.status_bytes());

        let mut permuted_input = FkBatchInput::new(layout.clone(), 4).unwrap();
        for destination in 0..4 {
            let source = 3 - destination;
            for coordinate in 0..layout.coordinate_count {
                permuted_input.q_soa_mut()[layout.q_index(coordinate, destination)] =
                    input.q_soa()[layout.q_index(coordinate, source)];
            }
            for component in 0..layout.pose_components {
                permuted_input.root_pose_soa_mut()
                    [layout.root_pose_index(component, destination)] =
                    input.root_pose_soa()[layout.root_pose_index(component, source)];
            }
        }
        let mut permuted_output = CudaStateInputOutput::new(layout.clone());
        executor
            .execute_into(&permuted_input, &mut permuted_output)
            .unwrap();
        for destination in 0..4 {
            let source = 3 - destination;
            assert_eq!(permuted_output.status(destination), first.status(source));
            for coordinate in 0..layout.coordinate_count {
                assert_eq!(
                    permuted_output.q_soa()[layout.q_index(coordinate, destination)].to_bits(),
                    first.q_soa()[layout.q_index(coordinate, source)].to_bits()
                );
            }
        }
    }

    #[test]
    fn runtime_probe_is_typed_and_never_claims_a_device_without_metadata() {
        let probe = probe_cuda_runtime();
        let (program, layout, input) = fixture();
        let cpu_mirror = CpuMirrorExecutor::compile(&program, 5, 32).unwrap();
        if probe.status == CudaRuntimeStatus::Available {
            assert!(!probe.devices.is_empty());
            assert!(probe.driver_version.is_some());
            assert!(
                probe
                    .devices
                    .iter()
                    .all(|device| device.compute_capability_major > 0)
            );
            let cpu_state = CpuMirrorStateInputExecutor::compile(&program, layout.clone()).unwrap();
            let mut expected = CudaStateInputOutput::new(layout.clone());
            cpu_state.execute_into(&input, &mut expected).unwrap();
            let mut device = CudaMirrorStateInputExecutor::compile(&cpu_mirror, 0).unwrap();
            let mut actual = CudaStateInputOutput::new(layout);
            device.execute_into(&input, &mut actual).unwrap();
            assert_eq!(actual.q_soa(), expected.q_soa());
            assert_eq!(actual.root_pose_soa(), expected.root_pose_soa());
            assert_eq!(actual.status_bytes(), expected.status_bytes());
        } else {
            assert!(probe.devices.is_empty());
            assert!(CudaMirrorStateInputExecutor::compile(&cpu_mirror, 0).is_err());
        }
    }

    #[test]
    fn embedded_kernel_has_a_stable_stage_name_and_no_atomic_instruction() {
        assert!(STATE_INPUT_PTX.contains(".entry bonesaw_state_input_v1"));
        assert!(!STATE_INPUT_PTX.contains("atom."));
        assert!(!STATE_INPUT_PTX.contains("fastmath"));
        assert_eq!(Sha256::digest(STATE_INPUT_PTX.as_bytes()).len(), 32);
    }
}
