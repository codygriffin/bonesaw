use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    history::{
        ExternalFrameHistories, HistoryQueryError, HistoryQueryPolicy, ReconstructionProvenance,
        RobotHistory,
    },
    math::Transform3,
    model::{BodyId, CompiledModel, ModelCache, ModelError},
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct AtlasFrameId(pub usize);

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ExternalFrameSlotId(pub usize);

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FrameProvider {
    ControlWorld,
    BodyFrame {
        body: BodyId,
    },
    FixedSite {
        parent: AtlasFrameId,
        #[serde(with = "crate::math::serde_transform3")]
        parent_from_site: Transform3,
    },
    ExternalSlot {
        slot: ExternalFrameSlotId,
        anchor: AtlasFrameId,
    },
    Derived {
        op: DerivedFrameOp,
    },
}

/// Bounded, compile-time frame operations. Dependencies must precede the
/// derived entry in the atlas, so runtime evaluation is one flat pass.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum DerivedFrameOp {
    Midpoint {
        first: AtlasFrameId,
        second: AtlasFrameId,
    },
    OriginFromOrientation {
        origin: AtlasFrameId,
        orientation: AtlasFrameId,
    },
    GroundProjected {
        source: AtlasFrameId,
        ground_z: f64,
    },
}

impl DerivedFrameOp {
    fn dependencies(&self) -> [Option<AtlasFrameId>; 2] {
        match *self {
            Self::Midpoint { first, second } => [Some(first), Some(second)],
            Self::OriginFromOrientation {
                origin,
                orientation,
            } => [Some(origin), Some(orientation)],
            Self::GroundProjected { source, .. } => [Some(source), None],
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrameAtlasEntry {
    pub id: AtlasFrameId,
    pub name: String,
    pub provider: FrameProvider,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompiledFrameAtlas {
    pub entries: Vec<FrameAtlasEntry>,
    pub control_world: AtlasFrameId,
    pub odom: AtlasFrameId,
    pub map: AtlasFrameId,
    pub body_frames: Vec<AtlasFrameId>,
    pub external_slot_count: usize,
    #[serde(skip)]
    names: BTreeMap<String, AtlasFrameId>,
}

#[derive(Clone, Debug)]
pub struct ExternalFrameInputs {
    pub anchor_from_frame: Vec<Option<Transform3>>,
}

impl ExternalFrameInputs {
    pub fn new(atlas: &CompiledFrameAtlas) -> Self {
        Self {
            anchor_from_frame: vec![None; atlas.external_slot_count],
        }
    }

    pub fn set(&mut self, slot: ExternalFrameSlotId, anchor_from_frame: Transform3) {
        if let Some(value) = self.anchor_from_frame.get_mut(slot.0) {
            *value = Some(anchor_from_frame);
        }
    }
}

/// Convenient shell input for the standard rooted navigation-frame layout.
///
/// `map_from_odom` may jump; `control_world_from_odom` should remain smooth.
#[derive(Clone, Copy, Debug)]
pub struct RootedFrameInput {
    pub control_world_from_odom: Transform3,
    pub map_from_odom: Transform3,
}

impl Default for RootedFrameInput {
    fn default() -> Self {
        Self {
            control_world_from_odom: Transform3::identity(),
            map_from_odom: Transform3::identity(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AtlasFrameProvenance {
    ControlRoot,
    RobotModel,
    ExternalObservation,
    FixedDerived,
}

#[derive(Clone, Debug)]
pub struct FrameAtlasSnapshot {
    pub control_world_from_frame: Vec<Transform3>,
    pub provenance: Vec<AtlasFrameProvenance>,
}

impl FrameAtlasSnapshot {
    pub fn new(atlas: &CompiledFrameAtlas) -> Self {
        Self {
            control_world_from_frame: vec![Transform3::identity(); atlas.entries.len()],
            provenance: vec![AtlasFrameProvenance::FixedDerived; atlas.entries.len()],
        }
    }
}

#[derive(Clone, Debug)]
pub struct AtlasFrameEstimate {
    pub from_to: Transform3,
    pub from: AtlasFrameId,
    pub to: AtlasFrameId,
}

#[derive(Clone, Copy, Debug)]
pub struct HistoricalFrameQuery {
    pub from: AtlasFrameId,
    pub to: AtlasFrameId,
    pub time_ns: i64,
    pub policy: HistoryQueryPolicy,
}

#[derive(Clone, Debug)]
pub struct HistoricalAtlasEstimate {
    pub estimate: AtlasFrameEstimate,
    pub robot_provenance: ReconstructionProvenance,
    pub external_provenance: Vec<ReconstructionProvenance>,
    pub support_interval_ns: (i64, i64),
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum FrameAtlasError {
    #[error("frame atlas external slot {0} is unavailable")]
    ExternalUnavailable(usize),
    #[error("frame atlas id {0} is out of range")]
    FrameOutOfRange(usize),
    #[error("derived frame dependency {dependency} must precede frame {frame}")]
    DerivedDependencyOrder { frame: usize, dependency: usize },
    #[error("frame atlas already contains the name {0}")]
    DuplicateName(String),
    #[error("derived frame parameter is not finite")]
    NonFiniteDerivedParameter,
}

#[derive(Debug, Error)]
pub enum HistoricalFrameQueryError {
    #[error("robot history reconstruction failed: {0}")]
    RobotHistory(#[source] HistoryQueryError),
    #[error("external-frame slot {slot} reconstruction failed: {source}")]
    ExternalHistory {
        slot: usize,
        #[source]
        source: HistoryQueryError,
    },
    #[error("external-frame history has {actual} slots but the atlas requires {required}")]
    ExternalSlotCount { required: usize, actual: usize },
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error(transparent)]
    Atlas(#[from] FrameAtlasError),
}

impl CompiledFrameAtlas {
    /// Standard rooted-motion atlas:
    ///
    /// `control_world` is the smooth WBC root; `odom` is a smooth external
    /// navigation frame; `map` is the possibly discontinuous global reporting
    /// frame. Robot frames always derive from the canonical model state.
    pub fn standard(model: &CompiledModel) -> Self {
        let mut entries = vec![
            FrameAtlasEntry {
                id: AtlasFrameId(0),
                name: "control_world".into(),
                provider: FrameProvider::ControlWorld,
            },
            FrameAtlasEntry {
                id: AtlasFrameId(1),
                name: "odom".into(),
                provider: FrameProvider::ExternalSlot {
                    slot: ExternalFrameSlotId(0),
                    anchor: AtlasFrameId(0),
                },
            },
            FrameAtlasEntry {
                id: AtlasFrameId(2),
                name: "map".into(),
                provider: FrameProvider::ExternalSlot {
                    slot: ExternalFrameSlotId(1),
                    anchor: AtlasFrameId(0),
                },
            },
        ];
        let mut body_frames = Vec::with_capacity(model.bodies.len());
        for body in &model.bodies {
            let id = AtlasFrameId(entries.len());
            body_frames.push(id);
            entries.push(FrameAtlasEntry {
                id,
                name: body.name.clone(),
                provider: FrameProvider::BodyFrame { body: body.id },
            });
        }
        let names = entries
            .iter()
            .map(|entry| (entry.name.clone(), entry.id))
            .collect();
        Self {
            entries,
            control_world: AtlasFrameId(0),
            odom: AtlasFrameId(1),
            map: AtlasFrameId(2),
            body_frames,
            external_slot_count: 2,
            names,
        }
    }

    pub fn rebuild_indexes(&mut self) {
        self.names = self
            .entries
            .iter()
            .map(|entry| (entry.name.clone(), entry.id))
            .collect();
    }

    /// Append a topologically valid derived frame. Restricting dependencies to
    /// existing entries rejects cycles by construction.
    pub fn add_derived_frame(
        &mut self,
        name: impl Into<String>,
        op: DerivedFrameOp,
    ) -> Result<AtlasFrameId, FrameAtlasError> {
        let name = name.into();
        if self.names.contains_key(&name) {
            return Err(FrameAtlasError::DuplicateName(name));
        }
        let id = AtlasFrameId(self.entries.len());
        for dependency in op.dependencies().into_iter().flatten() {
            if dependency.0 >= id.0 {
                return Err(FrameAtlasError::DerivedDependencyOrder {
                    frame: id.0,
                    dependency: dependency.0,
                });
            }
        }
        if matches!(
            op,
            DerivedFrameOp::GroundProjected { ground_z, .. } if !ground_z.is_finite()
        ) {
            return Err(FrameAtlasError::NonFiniteDerivedParameter);
        }
        self.entries.push(FrameAtlasEntry {
            id,
            name: name.clone(),
            provider: FrameProvider::Derived { op },
        });
        self.names.insert(name, id);
        Ok(id)
    }

    pub fn frame_id(&self, name: &str) -> Option<AtlasFrameId> {
        self.names.get(name).copied()
    }

    pub fn rooted_inputs(&self, input: RootedFrameInput) -> ExternalFrameInputs {
        let mut external = ExternalFrameInputs::new(self);
        external.set(ExternalFrameSlotId(0), input.control_world_from_odom);
        // T_C_M = T_C_O * inverse(T_M_O)
        external.set(
            ExternalFrameSlotId(1),
            input.control_world_from_odom * input.map_from_odom.inverse(),
        );
        external
    }

    pub fn evaluate(
        &self,
        model_cache: &ModelCache,
        external: &ExternalFrameInputs,
    ) -> Result<FrameAtlasSnapshot, FrameAtlasError> {
        let mut snapshot = FrameAtlasSnapshot::new(self);
        self.evaluate_into(model_cache, external, &mut snapshot)?;
        Ok(snapshot)
    }

    pub fn evaluate_into(
        &self,
        model_cache: &ModelCache,
        external: &ExternalFrameInputs,
        snapshot: &mut FrameAtlasSnapshot,
    ) -> Result<(), FrameAtlasError> {
        if snapshot.control_world_from_frame.len() != self.entries.len() {
            snapshot
                .control_world_from_frame
                .resize(self.entries.len(), Transform3::identity());
        }
        if snapshot.provenance.len() != self.entries.len() {
            snapshot
                .provenance
                .resize(self.entries.len(), AtlasFrameProvenance::FixedDerived);
        }
        let poses = &mut snapshot.control_world_from_frame;
        let provenance = &mut snapshot.provenance;
        for entry in &self.entries {
            let (pose, source) = match &entry.provider {
                FrameProvider::ControlWorld => {
                    (Transform3::identity(), AtlasFrameProvenance::ControlRoot)
                }
                FrameProvider::BodyFrame { body } => (
                    model_cache.world_from_body[body.0],
                    AtlasFrameProvenance::RobotModel,
                ),
                FrameProvider::FixedSite {
                    parent,
                    parent_from_site,
                } => (
                    poses[parent.0] * *parent_from_site,
                    AtlasFrameProvenance::FixedDerived,
                ),
                FrameProvider::ExternalSlot { slot, anchor } => {
                    let anchor_from_frame = external
                        .anchor_from_frame
                        .get(slot.0)
                        .and_then(|value| *value)
                        .ok_or(FrameAtlasError::ExternalUnavailable(slot.0))?;
                    (
                        poses[anchor.0] * anchor_from_frame,
                        AtlasFrameProvenance::ExternalObservation,
                    )
                }
                FrameProvider::Derived { op } => {
                    let pose = match *op {
                        DerivedFrameOp::Midpoint { first, second } => {
                            let a = poses[first.0];
                            let b = poses[second.0];
                            Transform3::from_parts(
                                nalgebra::Translation3::from(
                                    0.5 * (a.translation.vector + b.translation.vector),
                                ),
                                a.rotation.slerp(&b.rotation, 0.5),
                            )
                        }
                        DerivedFrameOp::OriginFromOrientation {
                            origin,
                            orientation,
                        } => Transform3::from_parts(
                            poses[origin.0].translation,
                            poses[orientation.0].rotation,
                        ),
                        DerivedFrameOp::GroundProjected { source, ground_z } => {
                            let source = poses[source.0];
                            let (_, _, yaw) = source.rotation.euler_angles();
                            Transform3::from_parts(
                                nalgebra::Translation3::new(
                                    source.translation.x,
                                    source.translation.y,
                                    ground_z,
                                ),
                                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
                            )
                        }
                    };
                    (pose, AtlasFrameProvenance::FixedDerived)
                }
            };
            poses[entry.id.0] = pose;
            provenance[entry.id.0] = source;
        }
        Ok(())
    }

    pub fn query(
        &self,
        snapshot: &FrameAtlasSnapshot,
        from: AtlasFrameId,
        to: AtlasFrameId,
    ) -> Result<AtlasFrameEstimate, FrameAtlasError> {
        let control_world_from_from = snapshot
            .control_world_from_frame
            .get(from.0)
            .ok_or(FrameAtlasError::FrameOutOfRange(from.0))?;
        let control_world_from_to = snapshot
            .control_world_from_frame
            .get(to.0)
            .ok_or(FrameAtlasError::FrameOutOfRange(to.0))?;
        Ok(AtlasFrameEstimate {
            from_to: control_world_from_from.inverse() * control_world_from_to,
            from,
            to,
        })
    }

    /// Reconstruct only authoritative robot/external state at the requested
    /// time, evaluate the atlas once, then answer a mutually consistent
    /// pairwise query. Caller-owned caches avoid topology discovery.
    #[allow(clippy::too_many_arguments)]
    pub fn query_history(
        &self,
        model: &CompiledModel,
        robot_history: &RobotHistory,
        external_histories: &ExternalFrameHistories,
        query: HistoricalFrameQuery,
        model_cache: &mut ModelCache,
        external_inputs: &mut ExternalFrameInputs,
        snapshot: &mut FrameAtlasSnapshot,
    ) -> Result<HistoricalAtlasEstimate, HistoricalFrameQueryError> {
        if external_histories.len() < self.external_slot_count {
            return Err(HistoricalFrameQueryError::ExternalSlotCount {
                required: self.external_slot_count,
                actual: external_histories.len(),
            });
        }
        let robot = robot_history
            .reconstruct(model, query.time_ns, query.policy)
            .map_err(HistoricalFrameQueryError::RobotHistory)?;
        model.forward_kinematics(&robot.state, model_cache)?;

        if external_inputs.anchor_from_frame.len() != self.external_slot_count {
            external_inputs
                .anchor_from_frame
                .resize(self.external_slot_count, None);
        }
        let mut external_provenance = Vec::with_capacity(self.external_slot_count);
        let mut support_start = robot.source_interval_ns.0;
        let mut support_end = robot.source_interval_ns.1;
        for slot in 0..self.external_slot_count {
            let reconstructed = external_histories
                .slot(slot)
                .expect("slot count validated")
                .reconstruct(query.time_ns, query.policy)
                .map_err(|source| HistoricalFrameQueryError::ExternalHistory { slot, source })?;
            external_inputs.anchor_from_frame[slot] = Some(reconstructed.sample.anchor_from_frame);
            external_provenance.push(reconstructed.provenance);
            support_start = support_start.min(reconstructed.source_interval_ns.0);
            support_end = support_end.max(reconstructed.source_interval_ns.1);
        }
        self.evaluate_into(model_cache, external_inputs, snapshot)?;
        Ok(HistoricalAtlasEstimate {
            estimate: self.query(snapshot, query.from, query.to)?,
            robot_provenance: robot.provenance,
            external_provenance,
            support_interval_ns: (support_start, support_end),
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        history::{ExternalFrameHistory, ExternalFrameSample, TimedRobotState},
        model::RobotState,
        urdf::load_urdf,
    };

    use super::*;

    #[test]
    fn map_jump_changes_reporting_not_control_world_robot_pose() {
        let source = r#"
        <robot name="rooted">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let atlas = CompiledFrameAtlas::standard(&model);
        let mut state = RobotState::zeros(&model);
        state.control_world_from_root = Transform3::translation(1.0, 2.0, 0.0);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();

        let before_external = atlas.rooted_inputs(RootedFrameInput::default());
        let before = atlas.evaluate(&cache, &before_external).unwrap();
        let base = atlas.frame_id("base").unwrap();
        let control_before = atlas.query(&before, atlas.control_world, base).unwrap();
        let map_before = atlas.query(&before, atlas.map, base).unwrap();

        let after_external = atlas.rooted_inputs(RootedFrameInput {
            control_world_from_odom: Transform3::identity(),
            map_from_odom: Transform3::translation(10.0, 0.0, 0.0),
        });
        let after = atlas.evaluate(&cache, &after_external).unwrap();
        let control_after = atlas.query(&after, atlas.control_world, base).unwrap();
        let map_after = atlas.query(&after, atlas.map, base).unwrap();

        assert!(
            (control_before.from_to.translation.vector - control_after.from_to.translation.vector)
                .norm()
                < 1e-12
        );
        assert!(
            (map_after.from_to.translation.vector - map_before.from_to.translation.vector).norm()
                > 9.0
        );
    }

    #[test]
    fn derived_frames_evaluate_in_compiled_order() {
        let source = r#"
        <robot name="derived">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let mut atlas = CompiledFrameAtlas::standard(&model);
        let base = atlas.frame_id("base").unwrap();
        let midpoint = atlas
            .add_derived_frame(
                "odom_base_midpoint",
                DerivedFrameOp::Midpoint {
                    first: atlas.odom,
                    second: base,
                },
            )
            .unwrap();
        let grounded = atlas
            .add_derived_frame(
                "base_ground",
                DerivedFrameOp::GroundProjected {
                    source: base,
                    ground_z: 0.0,
                },
            )
            .unwrap();

        let mut state = RobotState::zeros(&model);
        state.control_world_from_root = Transform3::translation(2.0, 0.0, 1.0);
        let mut cache = ModelCache::new(&model);
        model.forward_kinematics(&state, &mut cache).unwrap();
        let external = atlas.rooted_inputs(RootedFrameInput::default());
        let snapshot = atlas.evaluate(&cache, &external).unwrap();

        assert!(
            (snapshot.control_world_from_frame[midpoint.0]
                .translation
                .vector
                - crate::math::Vec3::new(1.0, 0.0, 0.5))
            .norm()
                < 1e-12
        );
        assert_eq!(
            snapshot.control_world_from_frame[grounded.0].translation.z,
            0.0
        );
    }

    #[test]
    fn historical_query_reconstructs_minimal_state_and_external_slots() {
        let source = r#"
        <robot name="history">
          <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
        </robot>"#;
        let model = load_urdf(source).unwrap();
        let atlas = CompiledFrameAtlas::standard(&model);
        let mut robot_history = RobotHistory::new(4);
        for (time_ns, x) in [(0, 0.0), (20_000_000, 2.0)] {
            let mut state = RobotState::zeros(&model);
            state.control_world_from_root = Transform3::translation(x, 0.0, 0.0);
            robot_history.push(TimedRobotState {
                time_ns,
                sequence: 1,
                state,
            });
        }
        let mut external_histories = ExternalFrameHistories::new([4, 4]);
        for slot in 0..2 {
            let history: &mut ExternalFrameHistory = external_histories.slot_mut(slot).unwrap();
            for time_ns in [0, 20_000_000] {
                history.push(ExternalFrameSample {
                    time_ns,
                    anchor_from_frame: Transform3::identity(),
                    twist: None,
                    acceleration: None,
                    covariance: None,
                    sequence: 1,
                });
            }
        }
        let mut model_cache = ModelCache::new(&model);
        let mut external_inputs = ExternalFrameInputs::new(&atlas);
        let mut snapshot = FrameAtlasSnapshot::new(&atlas);
        let estimate = atlas
            .query_history(
                &model,
                &robot_history,
                &external_histories,
                HistoricalFrameQuery {
                    from: atlas.map,
                    to: atlas.frame_id("base").unwrap(),
                    time_ns: 10_000_000,
                    policy: HistoryQueryPolicy::default(),
                },
                &mut model_cache,
                &mut external_inputs,
                &mut snapshot,
            )
            .unwrap();

        assert_eq!(
            estimate.robot_provenance,
            ReconstructionProvenance::Interpolated
        );
        assert!((estimate.estimate.from_to.translation.x - 1.0).abs() < 1e-12);
        assert_eq!(estimate.support_interval_ns, (0, 20_000_000));
    }
}
