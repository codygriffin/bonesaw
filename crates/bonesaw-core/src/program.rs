use std::{fs, path::Path};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{
    actuation::{ActuationError, CompiledActuation},
    collision::CollisionAvoidanceConfig,
    frames::CompiledFrameAtlas,
    model::CompiledModel,
    rig::{CompiledTaskProgram, TaskCompileError},
    signal::{CompiledSignalProgram, SignalCompileError},
    urdf::UrdfError,
};

const ARCHIVE_MAGIC: &[u8; 8] = b"BNSW0006";
const ARCHIVE_SCHEMA_VERSION: u32 = 6;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TimingSpec {
    pub control_horizon_ns: i64,
    pub sample_period_ns: i64,
}

impl Default for TimingSpec {
    fn default() -> Self {
        Self {
            control_horizon_ns: 20_000_000,
            sample_period_ns: 1_000_000,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProgramHeader {
    pub schema_version: u32,
    pub program_epoch: u64,
    pub fingerprint_sha256: [u8; 32],
}

/// Immutable result of authoring-time model compilation.
///
/// Canonical model, frame atlas, typed signal graph, resolved task slots, and
/// timing layout are frozen here rather than assembled by a runtime adapter.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionProgram {
    pub header: ProgramHeader,
    pub model: CompiledModel,
    pub actuation: CompiledActuation,
    pub frames: CompiledFrameAtlas,
    pub signals: CompiledSignalProgram,
    pub tasks: CompiledTaskProgram,
    pub timing: TimingSpec,
    #[serde(default)]
    pub collision_avoidance: Option<CollisionAvoidanceConfig>,
}

#[derive(Debug, Error)]
pub enum ProgramError {
    #[error(transparent)]
    Urdf(#[from] UrdfError),
    #[error("control horizon and sample period must be positive and divide exactly")]
    Timing,
    #[error("motion-program archive has an invalid magic or truncated header")]
    ArchiveHeader,
    #[error("motion-program archive payload length is invalid")]
    ArchiveLength,
    #[error("motion-program archive checksum does not match its payload")]
    ArchiveChecksum,
    #[error("motion-program schema {actual} is unsupported; expected {expected}")]
    ArchiveSchema { expected: u32, actual: u32 },
    #[error("motion-program fingerprint does not match its canonical content")]
    ArchiveFingerprint,
    #[error("motion-program archive invariant failed: {0}")]
    ArchiveInvariant(String),
    #[error("collision avoidance configuration is invalid")]
    CollisionConfig,
    #[error(transparent)]
    Actuation(#[from] ActuationError),
    #[error(transparent)]
    Signal(#[from] SignalCompileError),
    #[error(transparent)]
    Task(#[from] TaskCompileError),
    #[error(transparent)]
    Serialization(#[from] serde_json::Error),
}

#[derive(Serialize)]
struct FingerprintContent<'a> {
    schema_version: u32,
    model: &'a CompiledModel,
    actuation: &'a CompiledActuation,
    frames: &'a CompiledFrameAtlas,
    signals: &'a CompiledSignalProgram,
    tasks: &'a CompiledTaskProgram,
    timing: TimingSpec,
    collision_avoidance: &'a Option<CollisionAvoidanceConfig>,
}

impl MotionProgram {
    pub fn compile_urdf_file(
        path: impl AsRef<Path>,
        timing: TimingSpec,
        program_epoch: u64,
    ) -> Result<Self, ProgramError> {
        let source = fs::read_to_string(path).map_err(UrdfError::from)?;
        Self::compile_urdf(&source, timing, program_epoch)
    }

    pub fn compile_urdf(
        source: &str,
        timing: TimingSpec,
        program_epoch: u64,
    ) -> Result<Self, ProgramError> {
        if timing.control_horizon_ns <= 0
            || timing.sample_period_ns <= 0
            || timing.control_horizon_ns % timing.sample_period_ns != 0
        {
            return Err(ProgramError::Timing);
        }
        let model = crate::urdf::load_urdf(source)?;
        let actuation = CompiledActuation::identity_from_model(&model);
        actuation.validate(&model)?;
        let frames = CompiledFrameAtlas::standard(&model);
        let signals = CompiledSignalProgram::default();
        let tasks = CompiledTaskProgram::default();
        let collision_avoidance = None;
        let fingerprint_sha256 = fingerprint(
            &model,
            &actuation,
            &frames,
            &signals,
            &tasks,
            timing,
            &collision_avoidance,
        )?;
        Ok(Self {
            header: ProgramHeader {
                schema_version: ARCHIVE_SCHEMA_VERSION,
                program_epoch,
                fingerprint_sha256,
            },
            model,
            actuation,
            frames,
            signals,
            tasks,
            timing,
            collision_avoidance,
        })
    }

    /// Authoring-time builder that makes the collision policy part of the
    /// canonical archive and program fingerprint.
    pub fn with_collision_avoidance(
        mut self,
        collision_avoidance: CollisionAvoidanceConfig,
    ) -> Result<Self, ProgramError> {
        if !collision_avoidance.validate() {
            return Err(ProgramError::CollisionConfig);
        }
        self.collision_avoidance = Some(collision_avoidance);
        self.header.fingerprint_sha256 = fingerprint(
            &self.model,
            &self.actuation,
            &self.frames,
            &self.signals,
            &self.tasks,
            self.timing,
            &self.collision_avoidance,
        )?;
        Ok(self)
    }

    /// Authoring-time builder that freezes the typed signal topology and its
    /// fixed state/output layout into the program epoch and fingerprint.
    pub fn with_signals(mut self, signals: CompiledSignalProgram) -> Result<Self, ProgramError> {
        signals.validate()?;
        self.tasks.validate(&self.model, &signals)?;
        self.signals = signals;
        self.header.fingerprint_sha256 = fingerprint(
            &self.model,
            &self.actuation,
            &self.frames,
            &self.signals,
            &self.tasks,
            self.timing,
            &self.collision_avoidance,
        )?;
        Ok(self)
    }

    /// Authoring-time builder that freezes resolved task slots and response
    /// profiles into the program epoch and fingerprint.
    pub fn with_tasks(mut self, tasks: CompiledTaskProgram) -> Result<Self, ProgramError> {
        tasks.validate(&self.model, &self.signals)?;
        self.tasks = tasks;
        self.header.fingerprint_sha256 = fingerprint(
            &self.model,
            &self.actuation,
            &self.frames,
            &self.signals,
            &self.tasks,
            self.timing,
            &self.collision_avoidance,
        )?;
        Ok(self)
    }

    /// Authoring-time replacement for the URDF identity fallback. Coupled,
    /// differential, or passive mechanisms become immutable program data.
    pub fn with_actuation(mut self, actuation: CompiledActuation) -> Result<Self, ProgramError> {
        actuation.validate(&self.model)?;
        self.actuation = actuation;
        self.header.fingerprint_sha256 = fingerprint(
            &self.model,
            &self.actuation,
            &self.frames,
            &self.signals,
            &self.tasks,
            self.timing,
            &self.collision_avoidance,
        )?;
        Ok(self)
    }

    /// Canonical portable archive:
    /// `[8-byte magic][u64 JSON length][JSON payload][SHA-256 payload]`.
    ///
    /// Struct field order and vector order are stable; runtime name indexes are
    /// skipped and rebuilt on load.
    pub fn to_archive_bytes(&self) -> Result<Vec<u8>, ProgramError> {
        self.validate_loaded()?;
        if self.header.fingerprint_sha256
            != fingerprint(
                &self.model,
                &self.actuation,
                &self.frames,
                &self.signals,
                &self.tasks,
                self.timing,
                &self.collision_avoidance,
            )?
        {
            return Err(ProgramError::ArchiveFingerprint);
        }
        let payload = serde_json::to_vec(self)?;
        let checksum: [u8; 32] = Sha256::digest(&payload).into();
        let mut archive = Vec::with_capacity(16 + payload.len() + checksum.len());
        archive.extend_from_slice(ARCHIVE_MAGIC);
        archive.extend_from_slice(&(payload.len() as u64).to_le_bytes());
        archive.extend_from_slice(&payload);
        archive.extend_from_slice(&checksum);
        Ok(archive)
    }

    pub fn from_archive_bytes(archive: &[u8]) -> Result<Self, ProgramError> {
        if archive.len() < 16 + 32 || archive.get(..8) != Some(ARCHIVE_MAGIC) {
            return Err(ProgramError::ArchiveHeader);
        }
        let payload_len = u64::from_le_bytes(
            archive[8..16]
                .try_into()
                .map_err(|_| ProgramError::ArchiveHeader)?,
        ) as usize;
        let payload_end = 16_usize
            .checked_add(payload_len)
            .ok_or(ProgramError::ArchiveLength)?;
        let checksum_end = payload_end
            .checked_add(32)
            .ok_or(ProgramError::ArchiveLength)?;
        if checksum_end != archive.len() {
            return Err(ProgramError::ArchiveLength);
        }
        let payload = &archive[16..payload_end];
        let expected: [u8; 32] = Sha256::digest(payload).into();
        if archive[payload_end..checksum_end] != expected {
            return Err(ProgramError::ArchiveChecksum);
        }
        let mut program: Self = serde_json::from_slice(payload)?;
        if program.header.schema_version != ARCHIVE_SCHEMA_VERSION {
            return Err(ProgramError::ArchiveSchema {
                expected: ARCHIVE_SCHEMA_VERSION,
                actual: program.header.schema_version,
            });
        }
        program.validate_loaded()?;
        let fingerprint = fingerprint(
            &program.model,
            &program.actuation,
            &program.frames,
            &program.signals,
            &program.tasks,
            program.timing,
            &program.collision_avoidance,
        )?;
        if fingerprint != program.header.fingerprint_sha256 {
            return Err(ProgramError::ArchiveFingerprint);
        }
        program.model.rebuild_indexes();
        program.frames.rebuild_indexes();
        Ok(program)
    }

    pub fn write_archive(&self, path: impl AsRef<Path>) -> Result<(), ProgramError> {
        fs::write(path, self.to_archive_bytes()?).map_err(UrdfError::from)?;
        Ok(())
    }

    pub fn read_archive(path: impl AsRef<Path>) -> Result<Self, ProgramError> {
        let archive = fs::read(path).map_err(UrdfError::from)?;
        Self::from_archive_bytes(&archive)
    }

    fn validate_loaded(&self) -> Result<(), ProgramError> {
        if self.timing.control_horizon_ns <= 0
            || self.timing.sample_period_ns <= 0
            || self.timing.control_horizon_ns % self.timing.sample_period_ns != 0
        {
            return Err(ProgramError::Timing);
        }
        if self
            .collision_avoidance
            .is_some_and(|collision| !collision.validate())
        {
            return Err(ProgramError::CollisionConfig);
        }
        self.signals.validate()?;
        self.tasks.validate(&self.model, &self.signals)?;
        self.actuation.validate(&self.model)?;
        if self.model.bodies.is_empty()
            || self
                .model
                .bodies
                .iter()
                .enumerate()
                .any(|(index, body)| body.id.0 != index)
            || self.model.joints.iter().enumerate().any(|(index, joint)| {
                joint.id.0 != index
                    || joint.parent.0 >= self.model.bodies.len()
                    || joint.child.0 >= self.model.bodies.len()
            })
        {
            return Err(ProgramError::ArchiveInvariant(
                "canonical model IDs or references are inconsistent".into(),
            ));
        }
        if self.frames.entries.is_empty()
            || self
                .frames
                .entries
                .iter()
                .enumerate()
                .any(|(index, entry)| entry.id.0 != index)
        {
            return Err(ProgramError::ArchiveInvariant(
                "frame atlas IDs are inconsistent".into(),
            ));
        }
        Ok(())
    }
}

fn fingerprint(
    model: &CompiledModel,
    actuation: &CompiledActuation,
    frames: &CompiledFrameAtlas,
    signals: &CompiledSignalProgram,
    tasks: &CompiledTaskProgram,
    timing: TimingSpec,
    collision_avoidance: &Option<CollisionAvoidanceConfig>,
) -> Result<[u8; 32], ProgramError> {
    let content = FingerprintContent {
        schema_version: ARCHIVE_SCHEMA_VERSION,
        model,
        actuation,
        frames,
        signals,
        tasks,
        timing,
        collision_avoidance,
    };
    let canonical = serde_json::to_vec(&content)?;
    let mut hash = Sha256::new();
    hash.update(b"bonesaw-motion-program-v6\0");
    hash.update(canonical);
    Ok(hash.finalize().into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        actuation::ActuatorResourceModel,
        rig::TaskSpec,
        signal::{ScalarJet, SignalOp, SignalOutputSpec, VectorJet},
        solver::Priority,
    };

    #[test]
    fn identical_source_compiles_to_identical_fingerprint() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let a = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        let b = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        assert_eq!(a.header.fingerprint_sha256, b.header.fingerprint_sha256);
        assert_eq!(a.model.dof, 18);
    }

    #[test]
    fn actuator_mapping_and_resource_profile_change_fingerprint_and_round_trip() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let base = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        let mut actuation = base.actuation.clone();
        actuation.actuators[0].resource_model = Some(ActuatorResourceModel {
            torque_constant_nm_per_amp: 0.8,
            winding_resistance_ohm: 0.12,
            thermal_resistance_c_per_w: 0.45,
            thermal_time_constant_s: 35.0,
            ambient_temperature_c: 25.0,
            derating_start_temperature_c: 70.0,
            shutdown_temperature_c: 90.0,
            minimum_effort_fraction: 0.2,
        });
        let profiled = base.clone().with_actuation(actuation).unwrap();
        assert_ne!(
            base.header.fingerprint_sha256,
            profiled.header.fingerprint_sha256
        );
        let loaded =
            MotionProgram::from_archive_bytes(&profiled.to_archive_bytes().unwrap()).unwrap();
        assert_eq!(loaded.actuation, profiled.actuation);
        assert_eq!(
            loaded.header.fingerprint_sha256,
            profiled.header.fingerprint_sha256
        );
    }

    #[test]
    fn official_g1_archive_round_trip_when_reference_is_cached() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf");
        if !path.exists() {
            return;
        }
        let program = MotionProgram::compile_urdf_file(path, TimingSpec::default(), 17).unwrap();
        let archive = program.to_archive_bytes().unwrap();
        let loaded = MotionProgram::from_archive_bytes(&archive).unwrap();
        assert_eq!(
            loaded.header.fingerprint_sha256,
            program.header.fingerprint_sha256
        );
        assert_eq!(loaded.model.bodies.len(), program.model.bodies.len());
        assert_eq!(loaded.model.dof, 23);
    }

    #[test]
    fn canonical_archive_round_trip_rebuilds_indexes() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let collision = CollisionAvoidanceConfig {
            hard_margin: 0.01,
            influence_margin: 0.08,
            ..CollisionAvoidanceConfig::default()
        };
        let program = MotionProgram::compile_urdf(source, TimingSpec::default(), 7)
            .unwrap()
            .with_collision_avoidance(collision)
            .unwrap();
        let a = program.to_archive_bytes().unwrap();
        let b = program.to_archive_bytes().unwrap();
        assert_eq!(a, b);

        let loaded = MotionProgram::from_archive_bytes(&a).unwrap();
        assert_eq!(
            loaded.header.fingerprint_sha256,
            program.header.fingerprint_sha256
        );
        assert_eq!(
            loaded.model.frame_id("left_hand"),
            program.model.frame_id("left_hand")
        );
        assert_eq!(
            loaded.frames.frame_id("map"),
            program.frames.frame_id("map")
        );
        assert_eq!(
            loaded
                .collision_avoidance
                .expect("collision policy survives archive")
                .hard_margin,
            collision.hard_margin
        );
    }

    #[test]
    fn signal_topology_changes_fingerprint_and_survives_archive() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let base = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputScalar {
                    stable_id: 10,
                    input: 0,
                },
                SignalOp::DeadbandScalar {
                    stable_id: 11,
                    source: 0,
                    radius: 0.02,
                },
                SignalOp::CriticallyDampedSpring {
                    stable_id: 12,
                    source: 1,
                    bandwidth_hz: 3.0,
                },
                SignalOp::ConstantScalar {
                    stable_id: 13,
                    jet: ScalarJet {
                        value: 0.25,
                        velocity: 0.0,
                        acceleration: 0.0,
                    },
                },
                SignalOp::Add {
                    stable_id: 14,
                    left: 2,
                    right: 3,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 100,
                node: 4,
            }],
        )
        .unwrap();
        let program = base.clone().with_signals(signals.clone()).unwrap();
        assert_ne!(
            base.header.fingerprint_sha256,
            program.header.fingerprint_sha256
        );
        let loaded =
            MotionProgram::from_archive_bytes(&program.to_archive_bytes().unwrap()).unwrap();
        assert_eq!(loaded.signals, signals);
        assert_eq!(
            loaded.header.fingerprint_sha256,
            program.header.fingerprint_sha256
        );
    }

    #[test]
    fn resolved_task_plan_changes_fingerprint_and_survives_archive() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let base = MotionProgram::compile_urdf(source, TimingSpec::default(), 9).unwrap();
        let signals = CompiledSignalProgram::compile(
            vec![
                SignalOp::ConstantVector {
                    stable_id: 10,
                    jet: VectorJet {
                        value: crate::Vec3::new(0.2, 0.1, 1.0),
                        velocity: crate::Vec3::zeros(),
                        acceleration: crate::Vec3::zeros(),
                    },
                },
                SignalOp::ConstantRotation {
                    stable_id: 11,
                    jet: Default::default(),
                },
            ],
            vec![
                SignalOutputSpec {
                    stable_id: 100,
                    node: 0,
                },
                SignalOutputSpec {
                    stable_id: 101,
                    node: 1,
                },
            ],
        )
        .unwrap();
        let with_signals = base.with_signals(signals).unwrap();
        let tasks = CompiledTaskProgram::compile(
            &with_signals.model,
            &with_signals.signals,
            vec![
                TaskSpec::Point {
                    stable_id: 200,
                    frame: with_signals.model.frame_id("left_hand").unwrap(),
                    point_in_frame: crate::Vec3::zeros(),
                    target_signal: 100,
                    priority: Priority::Intent,
                    weight: 1.0,
                    bandwidth_hz: 2.5,
                },
                TaskSpec::FloatingRootOrientation {
                    stable_id: 201,
                    target_signal: 101,
                    priority: Priority::Viability,
                    weight: 1.0,
                    bandwidth_hz: 1.0,
                    damping_ratio: 1.0,
                    maximum_acceleration: 8.0,
                },
            ],
        )
        .unwrap();
        let without_tasks_fingerprint = with_signals.header.fingerprint_sha256;
        let program = with_signals.with_tasks(tasks.clone()).unwrap();

        assert_ne!(without_tasks_fingerprint, program.header.fingerprint_sha256);
        let loaded =
            MotionProgram::from_archive_bytes(&program.to_archive_bytes().unwrap()).unwrap();
        assert_eq!(loaded.tasks, tasks);
        assert_eq!(loaded.header.schema_version, 6);
    }

    #[test]
    fn archive_detects_payload_corruption() {
        let source = include_str!("../../../models/toy_humanoid.urdf");
        let program = MotionProgram::compile_urdf(source, TimingSpec::default(), 7).unwrap();
        let mut archive = program.to_archive_bytes().unwrap();
        archive[20] ^= 0x01;
        assert!(matches!(
            MotionProgram::from_archive_bytes(&archive),
            Err(ProgramError::ArchiveChecksum)
        ));
    }
}
