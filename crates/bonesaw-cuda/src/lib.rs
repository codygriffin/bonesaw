//! Deterministic batch mirror and CUDA kernel boundary.
//!
//! Gate 6 begins with fixed-layout `CpuMirrorF32` references. R91 adds the
//! first real device implementation for StateInput; R92 adds a generic packed
//! tree FK/CoM executor; R93 extends the same fixed-buffer stream through
//! floating frame-origin and center-of-mass Jacobians; R94 adds floating
//! M/h/Ag products; R95 adds fixed point position/J/Jdot-v products; R96 adds
//! fixed point-task/contact row emission. R97 freezes a preallocated,
//! fixed-iteration `CpuMirrorF32` hierarchical solve reference. R98 adds the
//! matching no-fallback CUDA solve source and fixed-buffer executor boundary;
//! actuation, integration, Graph capture, and device solve certification remain
//! unavailable. No device success is inferred when a compiler, driver, device,
//! or conformance run is absent.

mod cpu_exact_dynamic;
mod cpu_exact_solve;
mod cpu_mirror;
mod cpu_mirror_solve;
mod cuda_driver;
mod cuda_fk_com;
mod cuda_kinematics;
mod cuda_nvrtc;
mod cuda_state_input;
mod joint_envelope;
mod kernel_api;
mod layouts;

pub use cpu_exact_dynamic::{
    CpuExactDynamicBatchSolver, ExactDynamicBatchInput, ExactDynamicBatchOutput, ExactDynamicError,
    ExactDynamicSupportPatchSpec,
};
pub use cpu_exact_solve::{
    CpuExactBatchSolver, ExactSolveAgentStatus, ExactSolveBatchInput, ExactSolveBatchOutput,
    ExactSolveError, PRIORITY_LEVEL_COUNT,
};
pub use cpu_mirror::{
    BatchExecutionError, ContactLockSpec, CpuMirrorExecutor, PointAttractorSpec, PointQuerySpec,
    RigidPatchBasisDiagnostic, RigidPatchBasisPlan, RigidPatchBasisSpec,
    derive_rigid_patch_contact_modes,
};
pub use cpu_mirror_solve::{
    CpuMirrorBatchSolver, MIRROR_HARD_PROJECTION_SWEEPS, MIRROR_TASK_SWEEPS_PER_LEVEL,
    MirrorSolveAgentStatus, MirrorSolveBatchInput, MirrorSolveBatchOutput, MirrorSolveError,
    MirrorSolveSemantics,
};
pub use cuda_fk_com::{
    CudaFkComCompilerProbe, CudaFkComCompilerStatus, CudaFkComError, CudaMirrorDynamicsExecutor,
    CudaMirrorEmissionExecutor, CudaMirrorFkComExecutor, CudaMirrorKinematicsExecutor,
    CudaMirrorPointQueriesExecutor, CudaMirrorSolveExecutor, probe_cuda_dynamics_compiler,
    probe_cuda_emission_compiler, probe_cuda_fk_com_compiler, probe_cuda_jacobians_compiler,
    probe_cuda_point_queries_compiler, probe_cuda_solve_compiler,
};
pub use cuda_kinematics::{
    CUDA_DYNAMICS_SOURCE, CUDA_EMISSION_SOURCE, CUDA_FK_COM_SOURCE, CUDA_JACOBIANS_SOURCE,
    CUDA_POINT_QUERIES_SOURCE, CUDA_SOLVE_SOURCE, CudaJointKind, CudaKinematicsModel,
    CudaKinematicsModelError, cuda_dynamics_source_sha256, cuda_emission_source_sha256,
    cuda_fk_com_source_sha256, cuda_jacobians_source_sha256, cuda_point_queries_source_sha256,
    cuda_solve_source_sha256,
};
pub use cuda_nvrtc::NvrtcError;
pub use cuda_state_input::{
    CpuMirrorStateInputExecutor, CudaDeviceInfo, CudaMirrorStateInputExecutor, CudaRuntimeProbe,
    CudaRuntimeStatus, CudaStateInputError, CudaStateInputOutput, cuda_state_input_kernel_sha256,
    probe_cuda_runtime,
};
pub use joint_envelope::{
    CpuJointEnvelopeBatchExecutor, JointEnvelopeAgentStatus, JointEnvelopeBatchInput,
    JointEnvelopeBatchOutput, JointEnvelopeError,
};
pub use kernel_api::{
    BackendFingerprint, BackendProfile, BatchProgramDescriptor, ContactKinematicMode,
    ContactLockDescriptor, CudaBackendCapabilities, CudaBackendStatus, KERNEL_ABI_VERSION,
    KernelManifest, KernelStage, MathFlags, MirrorAlgorithm, PointQueryDescriptor,
    PointTaskDescriptor, ScalarFormat, Version, capabilities, status,
};
pub use layouts::{
    AgentStatus, BatchInputError, BatchLayout, BatchLayoutError, DynamicsBatchInput,
    DynamicsBatchOutput, EmissionBatchInput, EmissionBatchOutput, FkBatchInput, FkBatchOutput,
    JacobianBatchOutput, POSE_COMPONENTS, PointQueryBatchOutput, ROOT_TANGENT_COMPONENTS,
    SPATIAL_COMPONENTS,
};

#[cfg(test)]
mod allocation_sentinel {
    use std::{
        alloc::{GlobalAlloc, Layout, System},
        cell::Cell,
    };

    pub struct ThreadCountingAllocator;

    thread_local! {
        static ENABLED: Cell<bool> = const { Cell::new(false) };
        static CALLS: Cell<u64> = const { Cell::new(0) };
        static BYTES: Cell<u64> = const { Cell::new(0) };
    }

    unsafe impl GlobalAlloc for ThreadCountingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            ENABLED.with(|enabled| {
                if enabled.get() {
                    CALLS.set(CALLS.get() + 1);
                    BYTES.set(BYTES.get() + layout.size() as u64);
                }
            });
            unsafe { System.alloc(layout) }
        }

        unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
            unsafe { System.dealloc(ptr, layout) }
        }

        unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
            ENABLED.with(|enabled| {
                if enabled.get() {
                    CALLS.set(CALLS.get() + 1);
                    BYTES.set(BYTES.get() + new_size as u64);
                }
            });
            unsafe { System.realloc(ptr, layout, new_size) }
        }
    }

    pub fn measure<T>(operation: impl FnOnce() -> T) -> (T, u64, u64) {
        CALLS.with(|calls| calls.set(0));
        BYTES.with(|bytes| bytes.set(0));
        ENABLED.with(|enabled| enabled.set(true));
        let result = operation();
        ENABLED.with(|enabled| enabled.set(false));
        let calls = CALLS.with(Cell::get);
        let bytes = BYTES.with(Cell::get);
        (result, calls, bytes)
    }
}

#[cfg(test)]
#[global_allocator]
static TEST_ALLOCATOR: allocation_sentinel::ThreadCountingAllocator =
    allocation_sentinel::ThreadCountingAllocator;
