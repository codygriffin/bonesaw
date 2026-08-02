use bonesaw_core::Priority;
use serde::{Deserialize, Serialize};

use crate::BatchLayout;

pub const KERNEL_ABI_VERSION: u32 = 2;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum BackendProfile {
    CpuExactF64,
    CpuMirrorF32,
    CudaMirrorF32,
    CudaThroughputF32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum ScalarFormat {
    F32,
    F64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum MirrorAlgorithm {
    FixedTreeF32V1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct MathFlags {
    pub explicit_fma: bool,
    pub fast_math: bool,
    pub flush_denormals_to_zero: bool,
}

impl MathFlags {
    pub const MIRROR_F32: Self = Self {
        explicit_fma: true,
        fast_math: false,
        flush_denormals_to_zero: false,
    };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum KernelStage {
    StateInput = 0,
    ForwardKinematics = 1,
    CenterOfMass = 2,
    FrameAtlas = 3,
    Jacobians = 4,
    Distance = 5,
    TaskEmission = 6,
    ConstraintEmission = 7,
    HierarchicalSolve = 8,
    Actuation = 9,
    Integration = 10,
    RigidBodyDynamics = 11,
    PointQueries = 12,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct PointQueryDescriptor {
    pub stable_id: u32,
    pub frame_index: usize,
    /// Canonical f32 point coordinates stored as bits so the descriptor is
    /// equality-comparable and hashes the exact kernel constants.
    pub point_in_frame_bits: [u32; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct PointTaskDescriptor {
    pub stable_id: u32,
    pub point_query_slot: usize,
    pub priority: Priority,
    pub weight_bits: u32,
    pub bandwidth_hz_bits: u32,
}

impl PointTaskDescriptor {
    pub fn weight(self) -> f32 {
        f32::from_bits(self.weight_bits)
    }

    pub fn bandwidth_hz(self) -> f32 {
        f32::from_bits(self.bandwidth_hz_bits)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ContactLockDescriptor {
    pub stable_id: u32,
    pub point_query_slot: usize,
    pub kinematic_mode: ContactKinematicMode,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ContactKinematicMode {
    Disabled = 0,
    LockedPoint = 1,
    NormalPoint = 2,
    RollingPoint = 3,
}

impl ContactKinematicMode {
    pub const fn axis_enabled(self, component: usize) -> bool {
        match self {
            Self::Disabled => false,
            Self::LockedPoint => component < 3,
            Self::NormalPoint => component == 2,
            Self::RollingPoint => component == 1 || component == 2,
        }
    }
}

impl PointQueryDescriptor {
    pub fn point_in_frame(self) -> [f32; 3] {
        self.point_in_frame_bits.map(f32::from_bits)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct KernelManifest {
    supported: u64,
}

impl KernelManifest {
    pub const fn state_input_v1() -> Self {
        Self {
            supported: 1 << KernelStage::StateInput as u8,
        }
    }

    pub const fn kinematics_v1() -> Self {
        Self {
            supported: (1 << KernelStage::StateInput as u8)
                | (1 << KernelStage::ForwardKinematics as u8)
                | (1 << KernelStage::CenterOfMass as u8)
                | (1 << KernelStage::Jacobians as u8),
        }
    }

    pub const fn dynamics_v1() -> Self {
        Self {
            supported: Self::kinematics_v1().supported
                | (1 << KernelStage::RigidBodyDynamics as u8),
        }
    }

    pub const fn point_queries_v1() -> Self {
        Self {
            supported: Self::dynamics_v1().supported | (1 << KernelStage::PointQueries as u8),
        }
    }

    pub const fn emission_v1() -> Self {
        Self {
            supported: Self::point_queries_v1().supported
                | (1 << KernelStage::TaskEmission as u8)
                | (1 << KernelStage::ConstraintEmission as u8),
        }
    }

    pub const fn solve_v1() -> Self {
        Self {
            supported: Self::emission_v1().supported | (1 << KernelStage::HierarchicalSolve as u8),
        }
    }

    pub const fn fk_center_of_mass_v1() -> Self {
        Self {
            supported: (1 << KernelStage::StateInput as u8)
                | (1 << KernelStage::ForwardKinematics as u8)
                | (1 << KernelStage::CenterOfMass as u8),
        }
    }

    pub const fn supports(self, stage: KernelStage) -> bool {
        self.supported & (1 << stage as u8) != 0
    }

    pub const fn bits(self) -> u64 {
        self.supported
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct BatchProgramDescriptor {
    pub kernel_abi_version: u32,
    pub algorithm: MirrorAlgorithm,
    pub layout: BatchLayout,
    pub manifest: KernelManifest,
    pub math_flags: MathFlags,
    pub point_queries: Vec<PointQueryDescriptor>,
    pub point_tasks: Vec<PointTaskDescriptor>,
    pub contact_locks: Vec<ContactLockDescriptor>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct Version {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct BackendFingerprint {
    pub program_hash: [u8; 32],
    pub core_build_hash: [u8; 32],
    pub backend_profile: BackendProfile,
    pub scalar_format: ScalarFormat,
    pub cpu_isa: Option<String>,
    pub cuda_toolkit: Option<Version>,
    pub cuda_driver: Option<Version>,
    pub gpu_architecture: Option<String>,
    pub gpu_sm_count: Option<u32>,
    pub kernel_hash: [u8; 32],
    pub math_flags: MathFlags,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum CudaBackendStatus {
    CudaMirrorFkComCompiledRuntimeProbeRequired,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct CudaBackendCapabilities {
    pub cpu_mirror_f32: bool,
    /// A real direct-launch CUDA state-input kernel is compiled into the
    /// crate. Runtime device availability and certification are separate.
    pub cuda_mirror_state_input_f32: bool,
    /// The generic FK + center-of-mass CUDA source, packed model ABI, and
    /// direct-launch executor are compiled. Runtime compilation and device
    /// conformance remain separately gated.
    pub cuda_mirror_fk_com_f32: bool,
    /// The generic floating frame-origin and center-of-mass Jacobian stage is
    /// implemented behind the same compiler/runtime/device admission gates.
    pub cuda_mirror_jacobians_f32: bool,
    /// Floating M(q), h(q,v,g), and Ag(q) device code is implemented behind
    /// independent compiler/runtime/device conformance gates.
    pub cuda_mirror_dynamics_f32: bool,
    /// Compiler-resolved point position, Jacobian, and Jdot-v device products
    /// are implemented behind independent conformance gates.
    pub cuda_mirror_point_queries_f32: bool,
    /// Fixed point-task and contact-lock row emission is implemented behind
    /// independent compiler/runtime/device conformance gates.
    pub cuda_mirror_emission_f32: bool,
    /// Fixed-level hierarchical solve source and direct-launch fixed-buffer
    /// executor are implemented. Device certification remains independent.
    pub cuda_mirror_solve_f32: bool,
    pub cuda_mirror_f32: bool,
    pub cuda_throughput_f32: bool,
    pub cuda_graph_executor: bool,
}

pub const fn status() -> CudaBackendStatus {
    CudaBackendStatus::CudaMirrorFkComCompiledRuntimeProbeRequired
}

pub const fn capabilities() -> CudaBackendCapabilities {
    CudaBackendCapabilities {
        cpu_mirror_f32: true,
        cuda_mirror_state_input_f32: true,
        cuda_mirror_fk_com_f32: true,
        cuda_mirror_jacobians_f32: true,
        cuda_mirror_dynamics_f32: true,
        cuda_mirror_point_queries_f32: true,
        cuda_mirror_emission_f32: true,
        cuda_mirror_solve_f32: true,
        cuda_mirror_f32: false,
        cuda_throughput_f32: false,
        cuda_graph_executor: false,
    }
}
