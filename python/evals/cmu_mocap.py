"""Small, deterministic ASF/AMC reader for the pinned CMU walking oracle.

The evaluation keeps motion-data handling in Python. It extracts only named
landmark positions; no parser or source-mocap object crosses the Rust control
boundary.
"""

from __future__ import annotations

import dataclasses
import pathlib
from collections.abc import Iterable

import numpy as np


@dataclasses.dataclass(frozen=True)
class Bone:
    name: str
    direction: np.ndarray
    length_m: float
    axis_degrees: np.ndarray
    axis_order: str
    dofs: tuple[str, ...]
    parent: str


@dataclasses.dataclass(frozen=True)
class Skeleton:
    bones: dict[str, Bone]
    children: dict[str, tuple[str, ...]]
    root_order: tuple[str, ...]
    root_axis_order: str
    length_to_meters: float


@dataclasses.dataclass(frozen=True)
class Motion:
    frame_numbers: np.ndarray
    channels: tuple[dict[str, np.ndarray], ...]


@dataclasses.dataclass(frozen=True)
class RetargetedWalk:
    targets: np.ndarray
    stance: np.ndarray
    cadence_scale: np.ndarray
    source_phase_frames: np.ndarray
    metadata: dict[str, object]


@dataclasses.dataclass(frozen=True)
class FloatingRetargetedWalk:
    root_targets: np.ndarray
    targets: np.ndarray
    stance: np.ndarray
    cadence_scale: np.ndarray
    source_phase_frames: np.ndarray
    metadata: dict[str, object]


def _clean_lines(path: pathlib.Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="ascii").replace("\r", "").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def _rotation(axis: str, radians: float) -> np.ndarray:
    cosine = float(np.cos(radians))
    sine = float(np.sin(radians))
    if axis == "X":
        return np.array(
            [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
            dtype=np.float64,
        )
    if axis == "Y":
        return np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
            dtype=np.float64,
        )
    if axis == "Z":
        return np.array(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    raise ValueError(f"unsupported rotation axis {axis!r}")


def _euler_degrees(values: Iterable[float], order: Iterable[str]) -> np.ndarray:
    result = np.eye(3, dtype=np.float64)
    for value, axis in zip(values, order, strict=True):
        result = result @ _rotation(axis.upper(), np.deg2rad(float(value)))
    return result


def load_skeleton(path: pathlib.Path) -> Skeleton:
    lines = _clean_lines(path)
    length_units = None
    root_order: tuple[str, ...] | None = None
    root_axis_order = "XYZ"
    raw_bones: dict[str, dict[str, object]] = {}
    children: dict[str, tuple[str, ...]] = {}
    section = ""
    index = 0
    while index < len(lines):
        line = lines[index]
        if line.startswith(":"):
            section = line[1:].lower()
            index += 1
            continue
        if section == "units" and line.startswith("length "):
            length_units = float(line.split()[1])
        elif section == "root":
            fields = line.split()
            if fields[0] == "order":
                root_order = tuple(fields[1:])
            elif fields[0] == "axis":
                root_axis_order = fields[1].upper()
        elif section == "bonedata" and line == "begin":
            record: dict[str, object] = {"dofs": ()}
            index += 1
            while lines[index] != "end":
                fields = lines[index].split()
                if fields[0] == "name":
                    record["name"] = fields[1]
                elif fields[0] == "direction":
                    record["direction"] = np.asarray(fields[1:4], dtype=np.float64)
                elif fields[0] == "length":
                    record["length"] = float(fields[1])
                elif fields[0] == "axis":
                    record["axis"] = np.asarray(fields[1:4], dtype=np.float64)
                    record["axis_order"] = fields[4].upper()
                elif fields[0] == "dof":
                    record["dofs"] = tuple(fields[1:])
                index += 1
            name = str(record["name"])
            raw_bones[name] = record
        elif section == "hierarchy" and line not in {"begin", "end"}:
            fields = line.split()
            children[fields[0]] = tuple(fields[1:])
        index += 1

    if length_units is None or root_order is None:
        raise ValueError("ASF is missing root order or length units")
    # CMU's FAQ specifies this exact conversion for its ASF/AMC corpus.
    length_to_meters = (1.0 / length_units) * 2.54 / 100.0
    parent_by_name: dict[str, str] = {}
    for parent, descendants in children.items():
        for descendant in descendants:
            if descendant in parent_by_name:
                raise ValueError(f"bone {descendant!r} has multiple parents")
            parent_by_name[descendant] = parent
    if set(raw_bones) != set(parent_by_name):
        missing = sorted(set(raw_bones).symmetric_difference(parent_by_name))
        raise ValueError(f"hierarchy does not cover all bones: {missing}")
    bones = {
        name: Bone(
            name=name,
            direction=np.asarray(record["direction"], dtype=np.float64),
            length_m=float(record["length"]) * length_to_meters,
            axis_degrees=np.asarray(record["axis"], dtype=np.float64),
            axis_order=str(record["axis_order"]),
            dofs=tuple(record["dofs"]),
            parent=parent_by_name[name],
        )
        for name, record in raw_bones.items()
    }
    return Skeleton(
        bones=bones,
        children={name: tuple(value) for name, value in children.items()},
        root_order=root_order,
        root_axis_order=root_axis_order,
        length_to_meters=length_to_meters,
    )


def load_motion(path: pathlib.Path) -> Motion:
    lines = _clean_lines(path)
    channels: list[dict[str, np.ndarray]] = []
    frame_numbers: list[int] = []
    current: dict[str, np.ndarray] | None = None
    for line in lines:
        if line.startswith(":") or line.startswith("#!"):
            continue
        fields = line.split()
        if len(fields) == 1 and fields[0].isdigit():
            frame_numbers.append(int(fields[0]))
            current = {}
            channels.append(current)
            continue
        if current is None:
            raise ValueError("AMC channel row appears before its first frame")
        current[fields[0]] = np.asarray(fields[1:], dtype=np.float64)
    numbers = np.asarray(frame_numbers, dtype=np.int64)
    if len(numbers) == 0 or not np.array_equal(numbers, np.arange(1, len(numbers) + 1)):
        raise ValueError("AMC frames must be contiguous and one-indexed")
    return Motion(numbers, tuple(channels))


def landmark_positions(
    skeleton: Skeleton,
    motion: Motion,
    landmark_names: tuple[str, ...],
) -> np.ndarray:
    unknown = sorted(set(landmark_names).difference(set(skeleton.bones) | {"root"}))
    if unknown:
        raise ValueError(f"unknown landmark bones: {unknown}")
    positions = np.empty((len(motion.channels), len(landmark_names), 3), dtype=np.float64)
    landmark_index = {name: index for index, name in enumerate(landmark_names)}
    for frame_index, channels in enumerate(motion.channels):
        root_values = channels.get("root")
        if root_values is None or len(root_values) != len(skeleton.root_order):
            raise ValueError(f"AMC frame {frame_index + 1} has an invalid root row")
        translation = np.zeros(3, dtype=np.float64)
        rotation_values: list[float] = []
        rotation_order: list[str] = []
        for channel, value in zip(skeleton.root_order, root_values, strict=True):
            if channel.startswith("T"):
                translation["XYZ".index(channel[1])] = value * skeleton.length_to_meters
            elif channel.startswith("R"):
                rotation_values.append(float(value))
                rotation_order.append(channel[1])
            else:
                raise ValueError(f"unsupported root channel {channel!r}")
        root_rotation = _euler_degrees(rotation_values, rotation_order)
        endpoint_by_name = {"root": translation}
        rotation_by_name = {"root": root_rotation}
        if "root" in landmark_index:
            positions[frame_index, landmark_index["root"]] = translation
        stack = list(reversed(skeleton.children.get("root", ())))
        while stack:
            name = stack.pop()
            bone = skeleton.bones[name]
            values = channels.get(name, np.zeros(len(bone.dofs), dtype=np.float64))
            if len(values) != len(bone.dofs):
                raise ValueError(
                    f"AMC frame {frame_index + 1} has {len(values)} channels for "
                    f"{name!r}, expected {len(bone.dofs)}"
                )
            coordinate = _euler_degrees(bone.axis_degrees, bone.axis_order)
            animated = _euler_degrees(
                values,
                (degree[1].upper() for degree in bone.dofs),
            )
            local_rotation = coordinate @ animated @ coordinate.T
            parent_rotation = rotation_by_name[bone.parent]
            rotation_by_name[name] = parent_rotation @ local_rotation
            endpoint_by_name[name] = endpoint_by_name[bone.parent] + (
                rotation_by_name[name] @ bone.direction
            ) * bone.length_m
            if name in landmark_index:
                positions[frame_index, landmark_index[name]] = endpoint_by_name[name]
            stack.extend(reversed(skeleton.children.get(name, ())))
    if not np.isfinite(positions).all():
        raise ValueError("non-finite landmark position in AMC evaluation")
    return positions


def _periodic_catmull_rom(samples: np.ndarray, phase: np.ndarray) -> np.ndarray:
    count = samples.shape[0]
    if count < 4:
        raise ValueError("periodic interpolation needs at least four samples")
    base = np.floor(phase).astype(np.int64)
    alpha = (phase - base).reshape((-1,) + (1,) * (samples.ndim - 1))
    p0 = samples[(base - 1) % count]
    p1 = samples[base % count]
    p2 = samples[(base + 1) % count]
    p3 = samples[(base + 2) % count]
    return 0.5 * (
        2.0 * p1
        + (-p0 + p2) * alpha
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * alpha**2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * alpha**3
    )


def retarget_subject_37_walk(
    skeleton_path: pathlib.Path,
    motion_path: pathlib.Path,
    target_origins: np.ndarray,
    target_pelvis: np.ndarray,
    ticks: int,
    timestep_seconds: float,
) -> RetargetedWalk:
    """Retarget one pinned gait cycle into four target-effector trajectories.

    CMU uses Y-up with Z as the principal walking direction. Bonesaw uses
    X-forward, Y-left, Z-up, so source coordinates are reordered to `[Z, X, Y]`.
    The fixed-base benchmark removes source root translation and scales legs
    and arms independently from the target morphology.
    """

    if target_origins.shape != (4, 3) or target_pelvis.shape != (3,):
        raise ValueError("walking target origins must be [4, 3] plus one pelvis")
    if ticks <= 0 or not np.isfinite(timestep_seconds) or timestep_seconds <= 0.0:
        raise ValueError("walking trace duration must be positive")
    skeleton = load_skeleton(skeleton_path)
    motion = load_motion(motion_path)
    landmarks = landmark_positions(
        skeleton,
        motion,
        ("root", "lfoot", "rfoot", "lhand", "rhand"),
    )[..., [2, 0, 1]]
    root_relative = landmarks[:, 1:] - landmarks[:, :1]

    # Subject 37 trial 1 contains a stable same-phase interval at these
    # one-indexed source frames. The endpoint landmark closure is below 1 cm
    # RMS before morphology scaling.
    cycle_start = 134
    cycle_stop = 291
    cycle = root_relative[cycle_start:cycle_stop].copy()
    if cycle.shape[0] != cycle_stop - cycle_start:
        raise ValueError("pinned CMU motion is shorter than its gait-cycle window")

    source_leg_length = float(np.mean(np.linalg.norm(cycle[:, :2], axis=2)))
    source_arm_length = float(np.mean(np.linalg.norm(cycle[:, 2:], axis=2)))
    target_leg_length = float(
        np.mean(np.linalg.norm(target_origins[:2] - target_pelvis, axis=1))
    )
    target_arm_length = float(
        np.mean(np.linalg.norm(target_origins[2:] - target_pelvis, axis=1))
    )
    leg_scale = target_leg_length / source_leg_length
    arm_scale = target_arm_length / source_arm_length
    morphology_scale = np.asarray(
        [leg_scale, leg_scale, arm_scale, arm_scale],
        dtype=np.float64,
    )

    source_delta = cycle - cycle[0]
    # Preserve the measured swing shape but place the lowest point of each
    # source foot exactly on its target standing-height plane.
    source_delta[:, :2, 2] = cycle[:, :2, 2] - np.min(cycle[:, :2, 2], axis=0)
    # The source window endpoints are the same gait phase but not bit-identical
    # samples. A deterministic six-harmonic periodic reconstruction removes
    # the loop seam and high-frequency marker noise without inventing a
    # procedural gait.
    maximum_harmonic = 6
    spectrum = np.fft.rfft(source_delta, axis=0)
    spectrum[maximum_harmonic + 1 :] = 0.0
    source_delta = np.fft.irfft(spectrum, n=len(source_delta), axis=0)
    source_delta -= source_delta[0]
    source_delta[:, :2, 2] -= np.min(source_delta[:, :2, 2], axis=0)
    # The benchmark pelvis is fixed. Remove the source pelvis-bob component
    # from both foot heights by grounding the lower foot at every source phase;
    # otherwise a normal slow walk is mislabeled as a flight phase.
    source_delta[:, :2, 2] -= np.min(source_delta[:, :2, 2], axis=1)[:, None]

    cadence_pattern = np.asarray([0.75, 1.0, 1.25, 1.0], dtype=np.float64)
    cadence_block_ticks = max(1, int(round(8.0 / timestep_seconds)))
    cadence_scale = cadence_pattern[
        (np.arange(ticks, dtype=np.int64) // cadence_block_ticks) % len(cadence_pattern)
    ]
    phase = np.zeros(ticks, dtype=np.float64)
    if ticks > 1:
        phase[1:] = np.cumsum(
            120.0 * timestep_seconds * cadence_scale[:-1],
            dtype=np.float64,
        )
    sampled_delta = _periodic_catmull_rom(source_delta, phase)
    targets = target_origins[None, :, :] + sampled_delta * morphology_scale[None, :, None]
    foot_lift = targets[:, :2, 2] - target_origins[None, :2, 2]
    stance = foot_lift <= 0.025
    flight = ~np.any(stance, axis=1)
    double_support = np.all(stance, axis=1)
    contact_transitions = int(np.count_nonzero(stance[1:] != stance[:-1]))
    if np.any(flight):
        raise ValueError("fixed-pelvis walking reference contains a flight phase")
    if not np.any(double_support):
        raise ValueError("fixed-pelvis walking reference contains no double support")
    if not np.all(np.any(stance, axis=0)) or not np.all(np.any(~stance, axis=0)):
        raise ValueError("each walking-reference foot must contain stance and swing phases")

    closure_position_rms = float(np.sqrt(np.mean((cycle[-1] - cycle[0]) ** 2)))
    target_step = np.diff(targets, axis=0)
    target_velocity = target_step / timestep_seconds
    target_acceleration = np.diff(target_velocity, axis=0) / timestep_seconds
    metadata: dict[str, object] = {
        "dataset": "CMU Graphics Lab Motion Capture Database",
        "subject": 37,
        "trial": 1,
        "description": "slow walk",
        "source_rate_hz": 120,
        "source_frames": len(motion.channels),
        "cycle_start_frame_zero_based": cycle_start,
        "cycle_stop_frame_exclusive_zero_based": cycle_stop,
        "cycle_duration_seconds": (cycle_stop - cycle_start) / 120.0,
        "closure_position_rms_m": closure_position_rms,
        "periodic_reconstruction_maximum_harmonic": maximum_harmonic,
        "fixed_pelvis_grounding": "lower bilateral foot at zero height each phase",
        "reference_flight_fraction": float(np.mean(flight)),
        "reference_double_support_fraction": float(np.mean(double_support)),
        "reference_contact_transition_count": contact_transitions,
        "leg_morphology_scale": leg_scale,
        "arm_morphology_scale": arm_scale,
        "cadence_scale_pattern": cadence_pattern.tolist(),
        "cadence_block_seconds": cadence_block_ticks * timestep_seconds,
        "maximum_target_speed_m_s": float(np.max(np.linalg.norm(target_velocity, axis=2))),
        "maximum_target_acceleration_m_s2": float(
            np.max(np.linalg.norm(target_acceleration, axis=2))
        ),
    }
    if not np.isfinite(targets).all():
        raise ValueError("non-finite target in retargeted walking corpus")
    return RetargetedWalk(
        targets=targets,
        stance=stance.astype(np.uint8),
        cadence_scale=cadence_scale,
        source_phase_frames=np.mod(phase, len(cycle)),
        metadata=metadata,
    )


def retarget_subject_37_walk_floating(
    skeleton_path: pathlib.Path,
    motion_path: pathlib.Path,
    target_origins: np.ndarray,
    target_root: np.ndarray,
    ticks: int,
    timestep_seconds: float,
) -> FloatingRetargetedWalk:
    """Retarget the pinned gait with continuous root progression.

    The cyclic body motion is separated from the source root's horizontal
    stride. Periodic residuals are reconstructed with the same six-harmonic
    basis as the fixed-root corpus, while unbounded phase advances the root and
    all end effectors by the measured per-cycle displacement.
    """

    fixed = retarget_subject_37_walk(
        skeleton_path,
        motion_path,
        target_origins,
        target_root,
        ticks,
        timestep_seconds,
    )
    skeleton = load_skeleton(skeleton_path)
    motion = load_motion(motion_path)
    landmarks = landmark_positions(
        skeleton,
        motion,
        ("root", "lfoot", "rfoot", "lhand", "rhand"),
    )[..., [2, 0, 1]]
    cycle_start = int(fixed.metadata["cycle_start_frame_zero_based"])
    cycle_stop = int(fixed.metadata["cycle_stop_frame_exclusive_zero_based"])
    cycle = landmarks[cycle_start:cycle_stop].copy()
    count = len(cycle)
    sample_fraction = np.arange(count, dtype=np.float64) / count
    stride = (cycle[-1, 0] - cycle[0, 0]) * (count / (count - 1))
    # Vertical and lateral cycle drift are capture/reference-frame noise rather
    # than unbounded locomotion. Only forward progression remains absolute.
    stride[1:] = 0.0
    linear_progress = sample_fraction[:, None] * stride[None, :]

    root_residual = cycle[:, 0] - cycle[0, 0] - linear_progress
    effector_residual = (
        cycle[:, 1:] - cycle[0:1, 1:] - linear_progress[:, None, :]
    )
    maximum_harmonic = int(fixed.metadata["periodic_reconstruction_maximum_harmonic"])
    for residual in (root_residual, effector_residual):
        spectrum = np.fft.rfft(residual, axis=0)
        spectrum[maximum_harmonic + 1 :] = 0.0
        residual[:] = np.fft.irfft(spectrum, n=count, axis=0)
        residual -= residual[0]
    # Re-establish a physical floor after harmonic reconstruction.  The
    # floating corpus uses absolute foot motion, unlike the fixed-pelvis
    # corpus's per-phase grounding, so each reconstructed foot must have one
    # repeatable zero-height plane before contact can be classified.
    effector_residual[:, :2, 2] -= np.min(
        effector_residual[:, :2, 2], axis=0
    )

    phase = np.asarray(fixed.source_phase_frames, dtype=np.float64)
    completed_phase = np.zeros(ticks, dtype=np.float64)
    if ticks > 1:
        completed_phase[1:] = np.cumsum(
            120.0 * timestep_seconds * fixed.cadence_scale[:-1],
            dtype=np.float64,
        )
    progression = completed_phase[:, None] / count * stride[None, :]
    sampled_root_residual = _periodic_catmull_rom(root_residual, phase)
    sampled_effector_residual = _periodic_catmull_rom(effector_residual, phase)

    leg_scale = float(fixed.metadata["leg_morphology_scale"])
    arm_scale = float(fixed.metadata["arm_morphology_scale"])
    morphology_scale = np.asarray(
        [leg_scale, leg_scale, arm_scale, arm_scale],
        dtype=np.float64,
    )
    root_targets = target_root[None, :] + (
        progression + sampled_root_residual
    ) * leg_scale
    targets = (
        target_origins[None, :, :]
        + progression[:, None, :] * leg_scale
        + sampled_effector_residual * morphology_scale[None, :, None]
    )
    root_step = np.diff(root_targets, axis=0)
    root_velocity = root_step / timestep_seconds
    edge_order = 2 if ticks >= 3 else 1
    target_velocity = np.gradient(
        targets, timestep_seconds, axis=0, edge_order=edge_order
    )
    foot_lift = targets[:, :2, 2] - target_origins[None, :2, 2]
    # CMU subject 37 has endpoint kinematics but no synchronized force-plate
    # labels. A rigid target foot cannot reproduce the toe/heel marker's
    # absolute stance height, and the old speed threshold could leave a foot
    # classified as swing through its next lift. Use bilateral height order:
    # the lower endpoint is always support and a narrow overlap band provides
    # deterministic double support around a crossing.
    relative_support_margin = 0.005
    stance = foot_lift <= foot_lift[:, ::-1] + relative_support_margin
    canonical_root_y = target_root[1] + root_residual[:, 1] * leg_scale
    canonical_foot_lift = effector_residual[:, :2, 2] * leg_scale
    canonical_stance = (
        canonical_foot_lift
        <= canonical_foot_lift[:, ::-1] + relative_support_margin
    )
    canonical_left_support = canonical_stance[:, 0] & ~canonical_stance[:, 1]
    canonical_right_support = canonical_stance[:, 1] & ~canonical_stance[:, 0]
    if not np.any(canonical_left_support) or not np.any(canonical_right_support):
        raise ValueError("floating walking reference lacks bilateral single support")
    # Register lateral root excursion from the pinned canonical cycle, never
    # from the caller's requested output duration. Otherwise a truncated final
    # cycle changes every earlier root target and makes controller traces
    # depend on the evaluation buffer length.
    source_left_root_y = float(np.mean(canonical_root_y[canonical_left_support]))
    source_right_root_y = float(np.mean(canonical_root_y[canonical_right_support]))
    source_support_span = source_left_root_y - source_right_root_y
    target_left_y = float(target_origins[0, 1])
    target_right_y = float(target_origins[1, 1])
    target_support_span = target_left_y - target_right_y
    if abs(source_support_span) <= 1e-9 or abs(target_support_span) <= 1e-9:
        raise ValueError("floating walking reference has degenerate lateral support geometry")
    lateral_support_scale = target_support_span / source_support_span
    source_support_midpoint = 0.5 * (source_left_root_y + source_right_root_y)
    target_support_midpoint = 0.5 * (target_left_y + target_right_y)
    root_targets[:, 1] = target_support_midpoint + lateral_support_scale * (
        root_targets[:, 1] - source_support_midpoint
    )
    root_step = np.diff(root_targets, axis=0)
    root_velocity = root_step / timestep_seconds
    flight = ~np.any(stance, axis=1)
    double_support = np.all(stance, axis=1)
    contact_transitions = int(np.count_nonzero(stance[1:] != stance[:-1]))
    if np.any(flight):
        raise ValueError("floating walking reference contains a flight phase")
    if not np.any(double_support):
        raise ValueError("floating walking reference contains no double support")
    if not np.all(np.any(stance, axis=0)) or not np.all(np.any(~stance, axis=0)):
        raise ValueError(
            "each floating walking-reference foot must contain stance and swing phases"
        )
    metadata = {
        **fixed.metadata,
        "profile": "floating_root_progression",
        "source_stride_displacement_m": stride.tolist(),
        "target_stride_displacement_m": (stride * leg_scale).tolist(),
        "target_mean_forward_speed_m_s": float(
            stride[0] * leg_scale / fixed.metadata["cycle_duration_seconds"]
        ),
        "maximum_root_speed_m_s": float(
            np.max(np.linalg.norm(root_velocity, axis=1))
        ),
        "floating_contact_classifier": (
            "bilateral endpoint-height order with a 0.005 m overlap band; "
            "the lower foot is always support"
        ),
        "floating_relative_support_margin_m": relative_support_margin,
        "floating_lateral_support_retarget": {
            "method": (
                "affine map of pinned canonical-cycle single-support root means "
                "to target foot centers"
            ),
            "calibration_sample_count": count,
            "source_left_root_y_m": source_left_root_y,
            "source_right_root_y_m": source_right_root_y,
            "target_left_support_y_m": target_left_y,
            "target_right_support_y_m": target_right_y,
            "scale": lateral_support_scale,
        },
        "floating_reference_flight_fraction": float(np.mean(flight)),
        "floating_reference_double_support_fraction": float(
            np.mean(double_support)
        ),
        "floating_reference_contact_transition_count": contact_transitions,
        "maximum_floating_target_speed_m_s": float(
            np.max(np.linalg.norm(target_velocity, axis=2))
        ),
    }
    if not np.isfinite(root_targets).all() or not np.isfinite(targets).all():
        raise ValueError("non-finite target in floating walking corpus")
    return FloatingRetargetedWalk(
        root_targets=root_targets,
        targets=targets,
        stance=stance.astype(np.uint8),
        cadence_scale=fixed.cadence_scale,
        source_phase_frames=fixed.source_phase_frames,
        metadata=metadata,
    )
