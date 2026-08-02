use std::f64::consts::TAU;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::math::Vec3;
use nalgebra::UnitQuaternion;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScalarJet {
    pub value: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VectorJet {
    pub value: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RotationJet {
    pub value: UnitQuaternion<f64>,
    pub angular_velocity_world: Vec3,
    pub angular_acceleration_world: Vec3,
}

impl Default for RotationJet {
    fn default() -> Self {
        Self {
            value: UnitQuaternion::identity(),
            angular_velocity_world: Vec3::zeros(),
            angular_acceleration_world: Vec3::zeros(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SignalJet {
    Scalar(ScalarJet),
    Vector(VectorJet),
    Rotation(RotationJet),
}

impl SignalJet {
    pub fn kind(self) -> SignalKind {
        match self {
            Self::Scalar(_) => SignalKind::Scalar,
            Self::Vector(_) => SignalKind::Vector,
            Self::Rotation(_) => SignalKind::Rotation,
        }
    }

    fn is_finite(self) -> bool {
        match self {
            Self::Scalar(jet) => {
                jet.value.is_finite() && jet.velocity.is_finite() && jet.acceleration.is_finite()
            }
            Self::Vector(jet) => {
                jet.value.iter().all(|value| value.is_finite())
                    && jet.velocity.iter().all(|value| value.is_finite())
                    && jet.acceleration.iter().all(|value| value.is_finite())
            }
            Self::Rotation(jet) => {
                jet.value
                    .quaternion()
                    .coords
                    .iter()
                    .all(|value| value.is_finite())
                    && jet
                        .angular_velocity_world
                        .iter()
                        .all(|value| value.is_finite())
                    && jet
                        .angular_acceleration_world
                        .iter()
                        .all(|value| value.is_finite())
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum SignalKind {
    Scalar,
    Vector,
    Rotation,
}

#[derive(Clone, Debug, Default)]
pub struct SignalInputFrame {
    pub scalars: Vec<ScalarJet>,
    pub vectors: Vec<VectorJet>,
    pub rotations: Vec<RotationJet>,
}

impl SignalInputFrame {
    pub fn with_layout(scalar_inputs: usize, vector_inputs: usize) -> Self {
        Self {
            scalars: vec![ScalarJet::default(); scalar_inputs],
            vectors: vec![VectorJet::default(); vector_inputs],
            rotations: Vec::new(),
        }
    }

    pub fn with_full_layout(
        scalar_inputs: usize,
        vector_inputs: usize,
        rotation_inputs: usize,
    ) -> Self {
        Self {
            scalars: vec![ScalarJet::default(); scalar_inputs],
            vectors: vec![VectorJet::default(); vector_inputs],
            rotations: vec![RotationJet::default(); rotation_inputs],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SignalOp {
    InputScalar {
        stable_id: u32,
        input: usize,
    },
    InputVector {
        stable_id: u32,
        input: usize,
    },
    InputRotation {
        stable_id: u32,
        input: usize,
    },
    ConstantScalar {
        stable_id: u32,
        jet: ScalarJet,
    },
    ConstantVector {
        stable_id: u32,
        jet: VectorJet,
    },
    ConstantRotation {
        stable_id: u32,
        jet: RotationJet,
    },
    Add {
        stable_id: u32,
        left: usize,
        right: usize,
    },
    Scale {
        stable_id: u32,
        source: usize,
        scale: f64,
    },
    Blend {
        stable_id: u32,
        left: usize,
        right: usize,
        weight: usize,
    },
    ClampScalar {
        stable_id: u32,
        source: usize,
        lower: f64,
        upper: f64,
    },
    DeadbandScalar {
        stable_id: u32,
        source: usize,
        radius: f64,
    },
    LowPass {
        stable_id: u32,
        source: usize,
        bandwidth_hz: f64,
    },
    CriticallyDampedSpring {
        stable_id: u32,
        source: usize,
        bandwidth_hz: f64,
    },
}

impl SignalOp {
    pub fn stable_id(self) -> u32 {
        match self {
            Self::InputScalar { stable_id, .. }
            | Self::InputVector { stable_id, .. }
            | Self::InputRotation { stable_id, .. }
            | Self::ConstantScalar { stable_id, .. }
            | Self::ConstantVector { stable_id, .. }
            | Self::ConstantRotation { stable_id, .. }
            | Self::Add { stable_id, .. }
            | Self::Scale { stable_id, .. }
            | Self::Blend { stable_id, .. }
            | Self::ClampScalar { stable_id, .. }
            | Self::DeadbandScalar { stable_id, .. }
            | Self::LowPass { stable_id, .. }
            | Self::CriticallyDampedSpring { stable_id, .. } => stable_id,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct SignalOutputSpec {
    pub stable_id: u32,
    pub node: usize,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "SerializedSignalProgram")]
pub struct CompiledSignalProgram {
    nodes: Vec<SignalOp>,
    node_kinds: Vec<SignalKind>,
    memory_slot_by_node: Vec<Option<usize>>,
    outputs: Vec<SignalOutputSpec>,
    scalar_input_count: usize,
    vector_input_count: usize,
    rotation_input_count: usize,
    memory_slots: usize,
}

#[derive(Deserialize)]
struct SerializedSignalProgram {
    nodes: Vec<SignalOp>,
    node_kinds: Vec<SignalKind>,
    memory_slot_by_node: Vec<Option<usize>>,
    outputs: Vec<SignalOutputSpec>,
    scalar_input_count: usize,
    vector_input_count: usize,
    rotation_input_count: usize,
    memory_slots: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SignalMemoryCell {
    Scalar {
        value: f64,
        velocity: f64,
        initialized: bool,
    },
    Vector {
        value: Vec3,
        velocity: Vec3,
        initialized: bool,
    },
    Rotation {
        value: UnitQuaternion<f64>,
        angular_velocity_world: Vec3,
        initialized: bool,
    },
}

#[derive(Debug, Default, PartialEq)]
pub struct SignalMemory {
    cells: Vec<SignalMemoryCell>,
}

impl Clone for SignalMemory {
    fn clone(&self) -> Self {
        Self {
            cells: self.cells.clone(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.cells.clone_from(&source.cells);
    }
}

impl SignalMemory {
    pub fn new(program: &CompiledSignalProgram) -> Self {
        let mut cells = Vec::with_capacity(program.memory_slots);
        for (node, slot) in program.memory_slot_by_node.iter().enumerate() {
            if slot.is_none() {
                continue;
            }
            cells.push(match program.node_kinds[node] {
                SignalKind::Scalar => SignalMemoryCell::Scalar {
                    value: 0.0,
                    velocity: 0.0,
                    initialized: false,
                },
                SignalKind::Vector => SignalMemoryCell::Vector {
                    value: Vec3::zeros(),
                    velocity: Vec3::zeros(),
                    initialized: false,
                },
                SignalKind::Rotation => SignalMemoryCell::Rotation {
                    value: UnitQuaternion::identity(),
                    angular_velocity_world: Vec3::zeros(),
                    initialized: false,
                },
            });
        }
        Self { cells }
    }

    pub fn bitwise_eq(&self, other: &Self) -> bool {
        self.cells.len() == other.cells.len()
            && self
                .cells
                .iter()
                .zip(&other.cells)
                .all(|(left, right)| memory_cell_bitwise_eq(*left, *right))
    }
}

#[derive(Clone, Debug, Default)]
pub struct SignalScratch {
    values: Vec<SignalJet>,
}

impl SignalScratch {
    pub fn new(program: &CompiledSignalProgram) -> Self {
        Self {
            values: program.node_kinds.iter().copied().map(zero_jet).collect(),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct SignalOutputBuffer {
    pub values: Vec<SignalJet>,
}

impl SignalOutputBuffer {
    pub fn new(program: &CompiledSignalProgram) -> Self {
        Self {
            values: program
                .outputs
                .iter()
                .map(|output| zero_jet(program.node_kinds[output.node]))
                .collect(),
        }
    }
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum SignalCompileError {
    #[error("signal stable ID {0} is duplicated")]
    DuplicateStableId(u32),
    #[error("signal output stable ID {0} is duplicated")]
    DuplicateOutputStableId(u32),
    #[error("signal node {node} depends on non-prior node {dependency}")]
    DependencyOrder { node: usize, dependency: usize },
    #[error("signal node {node} expects {expected:?} but dependency has {actual:?}")]
    TypeMismatch {
        node: usize,
        expected: SignalKind,
        actual: SignalKind,
    },
    #[error("signal node {node} does not support {kind:?} values")]
    UnsupportedKind { node: usize, kind: SignalKind },
    #[error("signal node {0} contains an invalid numeric parameter")]
    InvalidParameter(usize),
    #[error("signal output {0} refers to a missing node")]
    OutputNode(usize),
    #[error("compiled signal layout does not match its canonical node list")]
    LayoutMismatch,
}

#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum SignalEvaluationError {
    #[error("signal input layout does not match the compiled program")]
    InputLayout,
    #[error("signal state or scratch layout does not match the compiled program")]
    WorkspaceLayout,
    #[error("signal input contains NaN or infinity")]
    NonFiniteInput,
    #[error("signal timestep is invalid")]
    InvalidTimestep,
}

impl CompiledSignalProgram {
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn output_count(&self) -> usize {
        self.outputs.len()
    }

    pub fn scalar_input_count(&self) -> usize {
        self.scalar_input_count
    }

    pub fn vector_input_count(&self) -> usize {
        self.vector_input_count
    }

    pub fn rotation_input_count(&self) -> usize {
        self.rotation_input_count
    }

    pub fn memory_slot_count(&self) -> usize {
        self.memory_slots
    }

    pub fn output_index(&self, stable_id: u32) -> Option<usize> {
        self.outputs
            .iter()
            .position(|output| output.stable_id == stable_id)
    }

    pub fn output_kind(&self, output: usize) -> Option<SignalKind> {
        self.outputs
            .get(output)
            .and_then(|output| self.node_kinds.get(output.node))
            .copied()
    }

    pub fn compile(
        nodes: Vec<SignalOp>,
        outputs: Vec<SignalOutputSpec>,
    ) -> Result<Self, SignalCompileError> {
        let mut node_kinds = Vec::with_capacity(nodes.len());
        let mut memory_slot_by_node = Vec::with_capacity(nodes.len());
        let mut stable_ids = Vec::with_capacity(nodes.len());
        let mut scalar_input_count = 0;
        let mut vector_input_count = 0;
        let mut rotation_input_count = 0;
        let mut memory_slots = 0;

        for (node_index, op) in nodes.iter().copied().enumerate() {
            let stable_id = op.stable_id();
            if stable_ids.contains(&stable_id) {
                return Err(SignalCompileError::DuplicateStableId(stable_id));
            }
            stable_ids.push(stable_id);
            let kind = match op {
                SignalOp::InputScalar { input, .. } => {
                    scalar_input_count = scalar_input_count.max(input.saturating_add(1));
                    SignalKind::Scalar
                }
                SignalOp::InputVector { input, .. } => {
                    vector_input_count = vector_input_count.max(input.saturating_add(1));
                    SignalKind::Vector
                }
                SignalOp::InputRotation { input, .. } => {
                    rotation_input_count = rotation_input_count.max(input.saturating_add(1));
                    SignalKind::Rotation
                }
                SignalOp::ConstantScalar { jet, .. } => {
                    if !SignalJet::Scalar(jet).is_finite() {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    SignalKind::Scalar
                }
                SignalOp::ConstantVector { jet, .. } => {
                    if !SignalJet::Vector(jet).is_finite() {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    SignalKind::Vector
                }
                SignalOp::ConstantRotation { jet, .. } => {
                    if !SignalJet::Rotation(jet).is_finite() {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    SignalKind::Rotation
                }
                SignalOp::Add { left, right, .. } => {
                    let left_kind = prior_kind(&node_kinds, node_index, left)?;
                    let right_kind = prior_kind(&node_kinds, node_index, right)?;
                    require_kind(node_index, left_kind, right_kind)?;
                    require_euclidean_kind(node_index, left_kind)?;
                    left_kind
                }
                SignalOp::Scale { source, scale, .. } => {
                    if !scale.is_finite() {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    let source_kind = prior_kind(&node_kinds, node_index, source)?;
                    require_euclidean_kind(node_index, source_kind)?;
                    source_kind
                }
                SignalOp::Blend {
                    left,
                    right,
                    weight,
                    ..
                } => {
                    let left_kind = prior_kind(&node_kinds, node_index, left)?;
                    let right_kind = prior_kind(&node_kinds, node_index, right)?;
                    require_kind(node_index, left_kind, right_kind)?;
                    let weight_kind = prior_kind(&node_kinds, node_index, weight)?;
                    require_kind(node_index, SignalKind::Scalar, weight_kind)?;
                    require_euclidean_kind(node_index, left_kind)?;
                    left_kind
                }
                SignalOp::ClampScalar {
                    source,
                    lower,
                    upper,
                    ..
                } => {
                    if !lower.is_finite() || !upper.is_finite() || lower > upper {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    let source_kind = prior_kind(&node_kinds, node_index, source)?;
                    require_kind(node_index, SignalKind::Scalar, source_kind)?;
                    SignalKind::Scalar
                }
                SignalOp::DeadbandScalar { source, radius, .. } => {
                    if !radius.is_finite() || radius < 0.0 {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    let source_kind = prior_kind(&node_kinds, node_index, source)?;
                    require_kind(node_index, SignalKind::Scalar, source_kind)?;
                    SignalKind::Scalar
                }
                SignalOp::LowPass {
                    source,
                    bandwidth_hz,
                    ..
                }
                | SignalOp::CriticallyDampedSpring {
                    source,
                    bandwidth_hz,
                    ..
                } => {
                    if !bandwidth_hz.is_finite() || bandwidth_hz <= 0.0 {
                        return Err(SignalCompileError::InvalidParameter(node_index));
                    }
                    prior_kind(&node_kinds, node_index, source)?
                }
            };
            let memory_slot = matches!(
                op,
                SignalOp::LowPass { .. } | SignalOp::CriticallyDampedSpring { .. }
            )
            .then(|| {
                let slot = memory_slots;
                memory_slots += 1;
                slot
            });
            node_kinds.push(kind);
            memory_slot_by_node.push(memory_slot);
        }

        let mut output_ids = Vec::with_capacity(outputs.len());
        for output in &outputs {
            if output.node >= nodes.len() {
                return Err(SignalCompileError::OutputNode(output.node));
            }
            if output_ids.contains(&output.stable_id) {
                return Err(SignalCompileError::DuplicateOutputStableId(
                    output.stable_id,
                ));
            }
            output_ids.push(output.stable_id);
        }

        Ok(Self {
            nodes,
            node_kinds,
            memory_slot_by_node,
            outputs,
            scalar_input_count,
            vector_input_count,
            rotation_input_count,
            memory_slots,
        })
    }

    pub fn validate(&self) -> Result<(), SignalCompileError> {
        let rebuilt = Self::compile(self.nodes.clone(), self.outputs.clone())?;
        if rebuilt == *self {
            Ok(())
        } else {
            Err(SignalCompileError::LayoutMismatch)
        }
    }

    pub fn evaluate_into(
        &self,
        timestep_seconds: f64,
        input: &SignalInputFrame,
        memory_in: &SignalMemory,
        memory_out: &mut SignalMemory,
        output: &mut SignalOutputBuffer,
        scratch: &mut SignalScratch,
    ) -> Result<(), SignalEvaluationError> {
        if !timestep_seconds.is_finite() || timestep_seconds < 0.0 {
            return Err(SignalEvaluationError::InvalidTimestep);
        }
        if input.scalars.len() != self.scalar_input_count
            || input.vectors.len() != self.vector_input_count
            || input.rotations.len() != self.rotation_input_count
        {
            return Err(SignalEvaluationError::InputLayout);
        }
        if input
            .scalars
            .iter()
            .copied()
            .any(|jet| !SignalJet::Scalar(jet).is_finite())
            || input
                .vectors
                .iter()
                .copied()
                .any(|jet| !SignalJet::Vector(jet).is_finite())
            || input
                .rotations
                .iter()
                .copied()
                .any(|jet| !SignalJet::Rotation(jet).is_finite())
        {
            return Err(SignalEvaluationError::NonFiniteInput);
        }
        if memory_in.cells.len() != self.memory_slots
            || memory_out.cells.len() != self.memory_slots
            || scratch.values.len() != self.nodes.len()
            || output.values.len() != self.outputs.len()
            || !self.memory_layout_matches(memory_in)
            || !self.memory_layout_matches(memory_out)
        {
            return Err(SignalEvaluationError::WorkspaceLayout);
        }
        memory_out.cells.copy_from_slice(&memory_in.cells);

        for (node_index, op) in self.nodes.iter().copied().enumerate() {
            let value = match op {
                SignalOp::InputScalar { input: slot, .. } => SignalJet::Scalar(input.scalars[slot]),
                SignalOp::InputVector { input: slot, .. } => SignalJet::Vector(input.vectors[slot]),
                SignalOp::InputRotation { input: slot, .. } => {
                    SignalJet::Rotation(input.rotations[slot])
                }
                SignalOp::ConstantScalar { jet, .. } => SignalJet::Scalar(jet),
                SignalOp::ConstantVector { jet, .. } => SignalJet::Vector(jet),
                SignalOp::ConstantRotation { jet, .. } => SignalJet::Rotation(jet),
                SignalOp::Add { left, right, .. } => {
                    add_jets(scratch.values[left], scratch.values[right])
                }
                SignalOp::Scale { source, scale, .. } => scale_jet(scratch.values[source], scale),
                SignalOp::Blend {
                    left,
                    right,
                    weight,
                    ..
                } => blend_jets(
                    scratch.values[left],
                    scratch.values[right],
                    scalar(scratch.values[weight]),
                ),
                SignalOp::ClampScalar {
                    source,
                    lower,
                    upper,
                    ..
                } => SignalJet::Scalar(clamp_scalar(scalar(scratch.values[source]), lower, upper)),
                SignalOp::DeadbandScalar { source, radius, .. } => {
                    SignalJet::Scalar(deadband_scalar(scalar(scratch.values[source]), radius))
                }
                SignalOp::LowPass {
                    source,
                    bandwidth_hz,
                    ..
                } => {
                    let slot = self.memory_slot_by_node[node_index]
                        .expect("validated stateful signal has a memory slot");
                    low_pass(
                        scratch.values[source],
                        bandwidth_hz,
                        timestep_seconds,
                        &mut memory_out.cells[slot],
                    )
                }
                SignalOp::CriticallyDampedSpring {
                    source,
                    bandwidth_hz,
                    ..
                } => {
                    let slot = self.memory_slot_by_node[node_index]
                        .expect("validated stateful signal has a memory slot");
                    critically_damped_spring(
                        scratch.values[source],
                        bandwidth_hz,
                        timestep_seconds,
                        &mut memory_out.cells[slot],
                    )
                }
            };
            scratch.values[node_index] = value;
        }
        for (slot, spec) in self.outputs.iter().enumerate() {
            output.values[slot] = scratch.values[spec.node];
        }
        Ok(())
    }

    fn memory_layout_matches(&self, memory: &SignalMemory) -> bool {
        self.memory_slot_by_node
            .iter()
            .enumerate()
            .filter_map(|(node, slot)| slot.map(|slot| (node, slot)))
            .all(|(node, slot)| {
                memory
                    .cells
                    .get(slot)
                    .is_some_and(|cell| cell.kind() == self.node_kinds[node])
            })
    }
}

impl TryFrom<SerializedSignalProgram> for CompiledSignalProgram {
    type Error = SignalCompileError;

    fn try_from(serialized: SerializedSignalProgram) -> Result<Self, Self::Error> {
        let canonical = Self::compile(serialized.nodes, serialized.outputs)?;
        if canonical.node_kinds == serialized.node_kinds
            && canonical.memory_slot_by_node == serialized.memory_slot_by_node
            && canonical.scalar_input_count == serialized.scalar_input_count
            && canonical.vector_input_count == serialized.vector_input_count
            && canonical.rotation_input_count == serialized.rotation_input_count
            && canonical.memory_slots == serialized.memory_slots
        {
            Ok(canonical)
        } else {
            Err(SignalCompileError::LayoutMismatch)
        }
    }
}

impl SignalMemoryCell {
    fn kind(self) -> SignalKind {
        match self {
            Self::Scalar { .. } => SignalKind::Scalar,
            Self::Vector { .. } => SignalKind::Vector,
            Self::Rotation { .. } => SignalKind::Rotation,
        }
    }
}

fn memory_cell_bitwise_eq(left: SignalMemoryCell, right: SignalMemoryCell) -> bool {
    match (left, right) {
        (
            SignalMemoryCell::Scalar {
                value: left_value,
                velocity: left_velocity,
                initialized: left_initialized,
            },
            SignalMemoryCell::Scalar {
                value: right_value,
                velocity: right_velocity,
                initialized: right_initialized,
            },
        ) => {
            left_initialized == right_initialized
                && left_value.to_bits() == right_value.to_bits()
                && left_velocity.to_bits() == right_velocity.to_bits()
        }
        (
            SignalMemoryCell::Vector {
                value: left_value,
                velocity: left_velocity,
                initialized: left_initialized,
            },
            SignalMemoryCell::Vector {
                value: right_value,
                velocity: right_velocity,
                initialized: right_initialized,
            },
        ) => {
            left_initialized == right_initialized
                && (0..3).all(|axis| {
                    left_value[axis].to_bits() == right_value[axis].to_bits()
                        && left_velocity[axis].to_bits() == right_velocity[axis].to_bits()
                })
        }
        (
            SignalMemoryCell::Rotation {
                value: left_value,
                angular_velocity_world: left_velocity,
                initialized: left_initialized,
            },
            SignalMemoryCell::Rotation {
                value: right_value,
                angular_velocity_world: right_velocity,
                initialized: right_initialized,
            },
        ) => {
            left_initialized == right_initialized
                && (0..4).all(|axis| {
                    left_value.quaternion().coords[axis].to_bits()
                        == right_value.quaternion().coords[axis].to_bits()
                })
                && (0..3)
                    .all(|axis| left_velocity[axis].to_bits() == right_velocity[axis].to_bits())
        }
        _ => false,
    }
}

fn prior_kind(
    kinds: &[SignalKind],
    node: usize,
    dependency: usize,
) -> Result<SignalKind, SignalCompileError> {
    kinds
        .get(dependency)
        .copied()
        .ok_or(SignalCompileError::DependencyOrder { node, dependency })
}

fn require_kind(
    node: usize,
    expected: SignalKind,
    actual: SignalKind,
) -> Result<(), SignalCompileError> {
    if expected == actual {
        Ok(())
    } else {
        Err(SignalCompileError::TypeMismatch {
            node,
            expected,
            actual,
        })
    }
}

fn require_euclidean_kind(node: usize, kind: SignalKind) -> Result<(), SignalCompileError> {
    if matches!(kind, SignalKind::Scalar | SignalKind::Vector) {
        Ok(())
    } else {
        Err(SignalCompileError::UnsupportedKind { node, kind })
    }
}

fn zero_jet(kind: SignalKind) -> SignalJet {
    match kind {
        SignalKind::Scalar => SignalJet::Scalar(ScalarJet::default()),
        SignalKind::Vector => SignalJet::Vector(VectorJet::default()),
        SignalKind::Rotation => SignalJet::Rotation(RotationJet::default()),
    }
}

fn scalar(jet: SignalJet) -> ScalarJet {
    match jet {
        SignalJet::Scalar(value) => value,
        SignalJet::Vector(_) => unreachable!("compiled signal type mismatch"),
        SignalJet::Rotation(_) => unreachable!("compiled signal type mismatch"),
    }
}

fn add_jets(left: SignalJet, right: SignalJet) -> SignalJet {
    match (left, right) {
        (SignalJet::Scalar(left), SignalJet::Scalar(right)) => SignalJet::Scalar(ScalarJet {
            value: left.value + right.value,
            velocity: left.velocity + right.velocity,
            acceleration: left.acceleration + right.acceleration,
        }),
        (SignalJet::Vector(left), SignalJet::Vector(right)) => SignalJet::Vector(VectorJet {
            value: left.value + right.value,
            velocity: left.velocity + right.velocity,
            acceleration: left.acceleration + right.acceleration,
        }),
        (SignalJet::Rotation(_), SignalJet::Rotation(_)) => {
            unreachable!("rotation addition is rejected by compilation")
        }
        _ => unreachable!("compiled signal type mismatch"),
    }
}

fn scale_jet(source: SignalJet, scale: f64) -> SignalJet {
    match source {
        SignalJet::Scalar(source) => SignalJet::Scalar(ScalarJet {
            value: source.value * scale,
            velocity: source.velocity * scale,
            acceleration: source.acceleration * scale,
        }),
        SignalJet::Vector(source) => SignalJet::Vector(VectorJet {
            value: source.value * scale,
            velocity: source.velocity * scale,
            acceleration: source.acceleration * scale,
        }),
        SignalJet::Rotation(_) => unreachable!("rotation scaling is rejected by compilation"),
    }
}

fn blend_jets(left: SignalJet, right: SignalJet, weight: ScalarJet) -> SignalJet {
    match (left, right) {
        (SignalJet::Scalar(left), SignalJet::Scalar(right)) => {
            let delta_value = right.value - left.value;
            let delta_velocity = right.velocity - left.velocity;
            let delta_acceleration = right.acceleration - left.acceleration;
            SignalJet::Scalar(ScalarJet {
                value: left.value + delta_value * weight.value,
                velocity: left.velocity
                    + delta_velocity * weight.value
                    + delta_value * weight.velocity,
                acceleration: left.acceleration
                    + delta_acceleration * weight.value
                    + 2.0 * delta_velocity * weight.velocity
                    + delta_value * weight.acceleration,
            })
        }
        (SignalJet::Vector(left), SignalJet::Vector(right)) => {
            let delta_value = right.value - left.value;
            let delta_velocity = right.velocity - left.velocity;
            let delta_acceleration = right.acceleration - left.acceleration;
            SignalJet::Vector(VectorJet {
                value: left.value + delta_value * weight.value,
                velocity: left.velocity
                    + delta_velocity * weight.value
                    + delta_value * weight.velocity,
                acceleration: left.acceleration
                    + delta_acceleration * weight.value
                    + 2.0 * delta_velocity * weight.velocity
                    + delta_value * weight.acceleration,
            })
        }
        (SignalJet::Rotation(_), SignalJet::Rotation(_)) => {
            unreachable!("rotation blending is rejected by compilation")
        }
        _ => unreachable!("compiled signal type mismatch"),
    }
}

fn clamp_scalar(source: ScalarJet, lower: f64, upper: f64) -> ScalarJet {
    if source.value < lower {
        ScalarJet {
            value: lower,
            ..ScalarJet::default()
        }
    } else if source.value > upper {
        ScalarJet {
            value: upper,
            ..ScalarJet::default()
        }
    } else {
        source
    }
}

fn deadband_scalar(source: ScalarJet, radius: f64) -> ScalarJet {
    if source.value.abs() <= radius {
        ScalarJet::default()
    } else {
        ScalarJet {
            value: source.value.signum() * (source.value.abs() - radius),
            ..source
        }
    }
}

fn low_pass(
    source: SignalJet,
    bandwidth_hz: f64,
    timestep_seconds: f64,
    memory: &mut SignalMemoryCell,
) -> SignalJet {
    let omega = TAU * bandwidth_hz;
    match (source, memory) {
        (
            SignalJet::Scalar(source),
            SignalMemoryCell::Scalar {
                value,
                velocity,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *velocity = source.velocity;
                *initialized = true;
                return SignalJet::Scalar(source);
            }
            let alpha = 1.0 - (-omega * timestep_seconds).exp();
            *value += alpha * (source.value - *value);
            *velocity = omega * (source.value - *value);
            SignalJet::Scalar(ScalarJet {
                value: *value,
                velocity: *velocity,
                acceleration: omega * (source.velocity - *velocity),
            })
        }
        (
            SignalJet::Vector(source),
            SignalMemoryCell::Vector {
                value,
                velocity,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *velocity = source.velocity;
                *initialized = true;
                return SignalJet::Vector(source);
            }
            let alpha = 1.0 - (-omega * timestep_seconds).exp();
            *value += alpha * (source.value - *value);
            *velocity = omega * (source.value - *value);
            SignalJet::Vector(VectorJet {
                value: *value,
                velocity: *velocity,
                acceleration: omega * (source.velocity - *velocity),
            })
        }
        (
            SignalJet::Rotation(source),
            SignalMemoryCell::Rotation {
                value,
                angular_velocity_world,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *angular_velocity_world = source.angular_velocity_world;
                *initialized = true;
                return SignalJet::Rotation(source);
            }
            let alpha = 1.0 - (-omega * timestep_seconds).exp();
            let error_world = rotation_error_world(value, &source.value);
            *value = UnitQuaternion::from_scaled_axis(alpha * error_world) * *value;
            let remaining_error_world = rotation_error_world(value, &source.value);
            *angular_velocity_world = omega * remaining_error_world;
            SignalJet::Rotation(RotationJet {
                value: *value,
                angular_velocity_world: *angular_velocity_world,
                angular_acceleration_world: omega
                    * (source.angular_velocity_world - *angular_velocity_world),
            })
        }
        _ => unreachable!("compiled signal memory type mismatch"),
    }
}

fn critically_damped_spring(
    source: SignalJet,
    bandwidth_hz: f64,
    timestep_seconds: f64,
    memory: &mut SignalMemoryCell,
) -> SignalJet {
    let omega = TAU * bandwidth_hz;
    let decay = (-omega * timestep_seconds).exp();
    match (source, memory) {
        (
            SignalJet::Scalar(source),
            SignalMemoryCell::Scalar {
                value,
                velocity,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *velocity = source.velocity;
                *initialized = true;
                return SignalJet::Scalar(source);
            }
            let error = *value - source.value;
            let error_velocity = *velocity - source.velocity;
            let c = error_velocity + omega * error;
            let next_error = (error + c * timestep_seconds) * decay;
            let next_error_velocity = (error_velocity - omega * c * timestep_seconds) * decay;
            *value = source.value + next_error;
            *velocity = source.velocity + next_error_velocity;
            SignalJet::Scalar(ScalarJet {
                value: *value,
                velocity: *velocity,
                acceleration: source.acceleration
                    - 2.0 * omega * next_error_velocity
                    - omega * omega * next_error,
            })
        }
        (
            SignalJet::Vector(source),
            SignalMemoryCell::Vector {
                value,
                velocity,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *velocity = source.velocity;
                *initialized = true;
                return SignalJet::Vector(source);
            }
            let error = *value - source.value;
            let error_velocity = *velocity - source.velocity;
            let c = error_velocity + error * omega;
            let next_error = (error + c * timestep_seconds) * decay;
            let next_error_velocity = (error_velocity - c * (omega * timestep_seconds)) * decay;
            *value = source.value + next_error;
            *velocity = source.velocity + next_error_velocity;
            SignalJet::Vector(VectorJet {
                value: *value,
                velocity: *velocity,
                acceleration: source.acceleration
                    - next_error_velocity * (2.0 * omega)
                    - next_error * (omega * omega),
            })
        }
        (
            SignalJet::Rotation(source),
            SignalMemoryCell::Rotation {
                value,
                angular_velocity_world,
                initialized,
            },
        ) => {
            if !*initialized {
                *value = source.value;
                *angular_velocity_world = source.angular_velocity_world;
                *initialized = true;
                return SignalJet::Rotation(source);
            }
            let error = rotation_error_world(&source.value, value);
            let error_velocity = *angular_velocity_world - source.angular_velocity_world;
            let c = error_velocity + omega * error;
            let next_error = (error + c * timestep_seconds) * decay;
            let next_error_velocity = (error_velocity - omega * c * timestep_seconds) * decay;
            *value = UnitQuaternion::from_scaled_axis(next_error) * source.value;
            *angular_velocity_world = source.angular_velocity_world + next_error_velocity;
            SignalJet::Rotation(RotationJet {
                value: *value,
                angular_velocity_world: *angular_velocity_world,
                angular_acceleration_world: source.angular_acceleration_world
                    - 2.0 * omega * next_error_velocity
                    - omega * omega * next_error,
            })
        }
        _ => unreachable!("compiled signal memory type mismatch"),
    }
}

fn rotation_error_world(from: &UnitQuaternion<f64>, to: &UnitQuaternion<f64>) -> Vec3 {
    from.transform_vector(&from.rotation_to(to).scaled_axis())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn graph() -> CompiledSignalProgram {
        CompiledSignalProgram::compile(
            vec![
                SignalOp::InputVector {
                    stable_id: 1,
                    input: 0,
                },
                SignalOp::ConstantVector {
                    stable_id: 2,
                    jet: VectorJet {
                        value: Vec3::new(2.0, -1.0, 0.5),
                        velocity: Vec3::new(0.1, 0.2, 0.3),
                        acceleration: Vec3::new(-0.2, 0.4, 0.1),
                    },
                },
                SignalOp::InputScalar {
                    stable_id: 3,
                    input: 0,
                },
                SignalOp::Blend {
                    stable_id: 4,
                    left: 0,
                    right: 1,
                    weight: 2,
                },
                SignalOp::CriticallyDampedSpring {
                    stable_id: 5,
                    source: 3,
                    bandwidth_hz: 2.0,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 100,
                node: 4,
            }],
        )
        .unwrap()
    }

    #[test]
    fn compiled_graph_propagates_vector_jet_derivatives() {
        let graph = graph();
        let mut input =
            SignalInputFrame::with_layout(graph.scalar_input_count, graph.vector_input_count);
        input.vectors[0] = VectorJet {
            value: Vec3::new(0.5, 1.0, -2.0),
            velocity: Vec3::new(0.2, -0.1, 0.4),
            acceleration: Vec3::new(0.3, -0.2, 0.1),
        };
        input.scalars[0] = ScalarJet {
            value: 0.25,
            velocity: 0.5,
            acceleration: -0.75,
        };
        let memory = SignalMemory::new(&graph);
        let mut next_memory = memory.clone();
        let mut scratch = SignalScratch::new(&graph);
        let mut output = SignalOutputBuffer::new(&graph);
        graph
            .evaluate_into(
                0.02,
                &input,
                &memory,
                &mut next_memory,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let SignalJet::Vector(actual) = output.values[0] else {
            panic!("vector output expected");
        };
        let a = input.vectors[0];
        let b = match graph.nodes[1] {
            SignalOp::ConstantVector { jet, .. } => jet,
            _ => unreachable!(),
        };
        let w = input.scalars[0];
        let expected = match blend_jets(SignalJet::Vector(a), SignalJet::Vector(b), w) {
            SignalJet::Vector(value) => value,
            _ => unreachable!(),
        };
        assert!((actual.value - expected.value).norm() < 1e-12);
        assert!((actual.velocity - expected.velocity).norm() < 1e-12);
        assert!((actual.acceleration - expected.acceleration).norm() < 1e-12);
    }

    #[test]
    fn stateful_evaluation_is_explicit_and_scratch_independent() {
        let graph = graph();
        let mut input =
            SignalInputFrame::with_layout(graph.scalar_input_count, graph.vector_input_count);
        input.vectors[0].value = Vec3::new(1.0, 2.0, 3.0);
        input.scalars[0].value = 0.5;
        let initial = SignalMemory::new(&graph);
        let mut memory_a = initial.clone();
        let mut memory_b = initial.clone();
        let mut scratch_a = SignalScratch::new(&graph);
        let mut scratch_b = SignalScratch::new(&graph);
        let mut output_a = SignalOutputBuffer::new(&graph);
        let mut output_b = SignalOutputBuffer::new(&graph);

        graph
            .evaluate_into(
                0.02,
                &input,
                &initial,
                &mut memory_a,
                &mut output_a,
                &mut scratch_a,
            )
            .unwrap();
        graph
            .evaluate_into(
                0.02,
                &input,
                &initial,
                &mut memory_b,
                &mut output_b,
                &mut scratch_b,
            )
            .unwrap();
        assert_eq!(memory_a, memory_b);
        assert_eq!(output_a.values, output_b.values);

        input.vectors[0].value += Vec3::new(0.25, -0.5, 1.0);
        input.scalars[0].value = 0.75;
        let mut memory_a_next = initial.clone();
        let mut memory_b_next = initial;
        graph
            .evaluate_into(
                0.02,
                &input,
                &memory_a,
                &mut memory_a_next,
                &mut output_a,
                &mut scratch_a,
            )
            .unwrap();
        graph
            .evaluate_into(
                0.02,
                &input,
                &memory_b,
                &mut memory_b_next,
                &mut output_b,
                &mut scratch_b,
            )
            .unwrap();
        assert_eq!(memory_a_next, memory_b_next);
        assert_eq!(output_a.values, output_b.values);
    }

    #[test]
    fn compiler_rejects_forward_edges_and_type_mismatches() {
        let forward = CompiledSignalProgram::compile(
            vec![SignalOp::Scale {
                stable_id: 1,
                source: 1,
                scale: 1.0,
            }],
            vec![],
        );
        assert_eq!(
            forward,
            Err(SignalCompileError::DependencyOrder {
                node: 0,
                dependency: 1
            })
        );

        let mismatch = CompiledSignalProgram::compile(
            vec![
                SignalOp::ConstantScalar {
                    stable_id: 1,
                    jet: ScalarJet::default(),
                },
                SignalOp::ConstantVector {
                    stable_id: 2,
                    jet: VectorJet::default(),
                },
                SignalOp::Add {
                    stable_id: 3,
                    left: 0,
                    right: 1,
                },
            ],
            vec![],
        );
        assert_eq!(
            mismatch,
            Err(SignalCompileError::TypeMismatch {
                node: 2,
                expected: SignalKind::Scalar,
                actual: SignalKind::Vector,
            })
        );
    }

    #[test]
    fn deserialization_rejects_forged_compiled_layout() {
        let graph = CompiledSignalProgram::compile(
            vec![SignalOp::InputScalar {
                stable_id: 1,
                input: 0,
            }],
            vec![SignalOutputSpec {
                stable_id: 2,
                node: 0,
            }],
        )
        .unwrap();
        let mut value = serde_json::to_value(graph).unwrap();
        value["scalar_input_count"] = serde_json::json!(2);

        assert!(serde_json::from_value::<CompiledSignalProgram>(value).is_err());
    }

    #[test]
    fn evaluation_rejects_same_size_memory_with_wrong_kind() {
        let scalar_graph = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputScalar {
                    stable_id: 1,
                    input: 0,
                },
                SignalOp::LowPass {
                    stable_id: 2,
                    source: 0,
                    bandwidth_hz: 2.0,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 3,
                node: 1,
            }],
        )
        .unwrap();
        let vector_graph = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputVector {
                    stable_id: 1,
                    input: 0,
                },
                SignalOp::LowPass {
                    stable_id: 2,
                    source: 0,
                    bandwidth_hz: 2.0,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 3,
                node: 1,
            }],
        )
        .unwrap();
        let input = SignalInputFrame::with_layout(1, 0);
        let wrong_memory = SignalMemory::new(&vector_graph);
        let mut wrong_memory_out = SignalMemory::new(&vector_graph);
        let mut output = SignalOutputBuffer::new(&scalar_graph);
        let mut scratch = SignalScratch::new(&scalar_graph);

        assert_eq!(
            scalar_graph.evaluate_into(
                0.01,
                &input,
                &wrong_memory,
                &mut wrong_memory_out,
                &mut output,
                &mut scratch,
            ),
            Err(SignalEvaluationError::WorkspaceLayout)
        );
    }

    #[test]
    fn rotation_spring_stays_on_so3_and_exposes_world_angular_jet() {
        let graph = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputRotation {
                    stable_id: 1,
                    input: 0,
                },
                SignalOp::CriticallyDampedSpring {
                    stable_id: 2,
                    source: 0,
                    bandwidth_hz: 3.0,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 3,
                node: 1,
            }],
        )
        .unwrap();
        let mut input = SignalInputFrame::with_full_layout(0, 0, 1);
        let memory_initial = SignalMemory::new(&graph);
        let mut memory_at_identity = SignalMemory::new(&graph);
        let mut memory_next = SignalMemory::new(&graph);
        let mut scratch = SignalScratch::new(&graph);
        let mut output = SignalOutputBuffer::new(&graph);
        graph
            .evaluate_into(
                0.02,
                &input,
                &memory_initial,
                &mut memory_at_identity,
                &mut output,
                &mut scratch,
            )
            .unwrap();

        input.rotations[0] = RotationJet {
            value: UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.0, 0.3)),
            angular_velocity_world: Vec3::new(0.0, 0.0, 0.1),
            angular_acceleration_world: Vec3::new(0.0, 0.0, -0.02),
        };
        graph
            .evaluate_into(
                0.02,
                &input,
                &memory_at_identity,
                &mut memory_next,
                &mut output,
                &mut scratch,
            )
            .unwrap();
        let SignalJet::Rotation(actual) = output.values[0] else {
            panic!("rotation output expected");
        };
        let angle = actual.value.angle();
        assert!(angle > 0.0 && angle < 0.3);
        assert!((actual.value.quaternion().norm() - 1.0).abs() < 1e-12);
        assert!(
            actual
                .angular_velocity_world
                .iter()
                .chain(actual.angular_acceleration_world.iter())
                .all(|value| value.is_finite())
        );
    }

    #[test]
    fn rotation_spring_derivatives_match_small_step_finite_differences() {
        let graph = CompiledSignalProgram::compile(
            vec![
                SignalOp::InputRotation {
                    stable_id: 1,
                    input: 0,
                },
                SignalOp::CriticallyDampedSpring {
                    stable_id: 2,
                    source: 0,
                    bandwidth_hz: 3.0,
                },
            ],
            vec![SignalOutputSpec {
                stable_id: 3,
                node: 1,
            }],
        )
        .unwrap();
        let mut input = SignalInputFrame::with_full_layout(0, 0, 1);
        let memory_initial = SignalMemory::new(&graph);
        let mut memory_at_identity = SignalMemory::new(&graph);
        let mut memory_a = SignalMemory::new(&graph);
        let mut memory_b = SignalMemory::new(&graph);
        let mut scratch = SignalScratch::new(&graph);
        let mut output_a = SignalOutputBuffer::new(&graph);
        let mut output_b = SignalOutputBuffer::new(&graph);
        graph
            .evaluate_into(
                1.0e-5,
                &input,
                &memory_initial,
                &mut memory_at_identity,
                &mut output_a,
                &mut scratch,
            )
            .unwrap();

        input.rotations[0] = RotationJet {
            value: UnitQuaternion::from_scaled_axis(Vec3::new(0.04, -0.03, 0.2)),
            angular_velocity_world: Vec3::zeros(),
            angular_acceleration_world: Vec3::zeros(),
        };
        graph
            .evaluate_into(
                1.0e-5,
                &input,
                &memory_at_identity,
                &mut memory_a,
                &mut output_a,
                &mut scratch,
            )
            .unwrap();
        graph
            .evaluate_into(
                1.0e-5,
                &input,
                &memory_a,
                &mut memory_b,
                &mut output_b,
                &mut scratch,
            )
            .unwrap();

        let SignalJet::Rotation(a) = output_a.values[0] else {
            panic!("rotation output expected");
        };
        let SignalJet::Rotation(b) = output_b.values[0] else {
            panic!("rotation output expected");
        };
        let dt = 1.0e-5;
        let finite_velocity = rotation_error_world(&a.value, &b.value) / dt;
        let midpoint_velocity = (a.angular_velocity_world + b.angular_velocity_world) * 0.5;
        let finite_acceleration = (b.angular_velocity_world - a.angular_velocity_world) / dt;
        let midpoint_acceleration =
            (a.angular_acceleration_world + b.angular_acceleration_world) * 0.5;

        assert!((finite_velocity - midpoint_velocity).norm() < 1.0e-6);
        assert!((finite_acceleration - midpoint_acceleration).norm() < 1.0e-3);
    }

    #[test]
    fn compiler_rejects_euclidean_add_on_rotations() {
        let result = CompiledSignalProgram::compile(
            vec![
                SignalOp::ConstantRotation {
                    stable_id: 1,
                    jet: RotationJet::default(),
                },
                SignalOp::ConstantRotation {
                    stable_id: 2,
                    jet: RotationJet::default(),
                },
                SignalOp::Add {
                    stable_id: 3,
                    left: 0,
                    right: 1,
                },
            ],
            vec![],
        );
        assert_eq!(
            result,
            Err(SignalCompileError::UnsupportedKind {
                node: 2,
                kind: SignalKind::Rotation,
            })
        );
    }
}
