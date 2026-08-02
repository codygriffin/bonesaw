# G1 oracle-state WBC admission

**PASS · 43/43 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable reference root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

This run uses the program's independent actuator bounds; no synthetic coupled transmission is enabled.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 2317/2317 converged | foot 4.04 mm · CoM 29.20 mm | 18.0 µs | 24.4 µs | 71.2 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.62% · a 0.75% · CoM v 34.68% · a 35.21% | analytic Jv / Jq̈+J̇v | 28.2 µs | 34.3 µs | 46.2 µs | 0 calls / 0 B |
| Stateless floating WBC | 2317/2317 solved | dynamics 1.71e-09 · contact 6.66e-11 · CoP margin 0.500 cm | 3419.4 µs | 5703.0 µs | 7791.3 µs | 0 calls / 0 B |

The maximum viability-task RMS is `0.734981` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `1.432 rad/s` / `8.932 rad/s²`. The full generalized-acceleration witness deviation is `25.693` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

### Tracking residuals over execution time

Peak residual remains the admission rule, while p99, time-RMS, integrated residual, violating ticks, and longest contiguous run describe whether unavailable authority is isolated or persistent. Angular and linear task units are kept separate.

| task | units | p99 | max | time RMS | integral | above contract | longest run |
|---|---:|---:|---:|---:|---:|---:|---:|
| root_angular | rad/s^2 | 0.0005 | 0.0029 | 0.0001 | 0.0007 | 0 / 2317 | 0 ticks |
| root_horizontal | m/s^2 | 0.5228 | 0.6529 | 0.1537 | 0.8003 | 0 / 2317 | 0 ticks |
| root_height | m/s^2 | 0.0004 | 0.0008 | 0.0001 | 0.0004 | 0 / 2317 | 0 ticks |
| center_of_mass | m/s^2 | 0.5683 | 0.7350 | 0.1560 | 0.7855 | 0 / 2317 | 0 ticks |
| frame_angular_0 | rad/s^2 | 0.0007 | 0.0038 | 0.0002 | 0.0008 | 0 / 2317 | 0 ticks |
| point_0 | m/s^2 | 0.0009 | 0.0021 | 0.0002 | 0.0008 | 0 / 2317 | 0 ticks |
| point_1 | m/s^2 | 0.0006 | 0.0011 | 0.0001 | 0.0006 | 0 / 2317 | 0 ticks |

## Contact-transition windows

The authored schedule contains `4` liftoffs and `4` touchdowns (`8` edges total). A ±8-tick window around every edge covers `136` unique ticks. Liftoff feet are `[0, 1, 0, 1]` and strictly alternate: `True`.

Across contact edges, the maximum authored position / velocity / acceleration deltas are `0.000 mm` / `0.000163 m/s` / `0.064606 m/s²`. Every edge window solves without infeasible/failed status; maximum dynamics/contact residual is `1.10e-09` / `4.16e-11`, minimum CoP margin is `5.000 mm`, peak actuator utilization is `38.35%`, and maximum WBC time is `7.287 ms`.

| tick | edge | foot | WBC solved | hard dynamics / contact | CoP margin | actuator use | max time | clipped ticks |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 300 | liftoff | 0 | 17/17 | 5.52e-10 / 3.63e-11 | 5.14 mm | 35.89% | 7.287 ms | 8 |
| 529 | touchdown | 0 | 17/17 | 4.14e-10 / 1.46e-11 | 5.00 mm | 38.35% | 5.690 ms | 12 |
| 879 | liftoff | 1 | 17/17 | 8.87e-10 / 3.37e-11 | 5.00 mm | 32.65% | 4.139 ms | 8 |
| 1108 | touchdown | 1 | 17/17 | 7.71e-10 / 4.16e-11 | 5.00 mm | 33.49% | 4.224 ms | 9 |
| 1458 | liftoff | 0 | 17/17 | 4.95e-10 / 1.39e-11 | 5.00 mm | 28.59% | 3.647 ms | 14 |
| 1687 | touchdown | 0 | 17/17 | 5.99e-10 / 1.67e-11 | 5.00 mm | 29.06% | 3.872 ms | 13 |
| 2037 | liftoff | 1 | 17/17 | 1.10e-09 / 3.16e-11 | 5.00 mm | 23.43% | 3.826 ms | 13 |
| 2266 | touchdown | 1 | 17/17 | 3.82e-10 / 2.89e-11 | 6.23 mm | 24.19% | 3.873 ms | 9 |

### Per-step execution

Each row begins after the preceding touchdown (or tick zero) and ends at the current touchdown, so transfer and swing work are both charged to the step that consumes them.

| step | foot | ticks | solved / slack | clipped | root angular / linear / height | CoM / swing point | hard dynamics / contact | CoP | actuator | p50 / p99 / max |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 530 | 249 / 281 | 281 | 0.003 / 0.195 / 0.001 | 0.176 / 0.002 | 1.46e-09 / 5.37e-11 | 5.00 mm | 43.6% | 3.451 / 6.310 / 7.791 ms |
| 1 | 1 | 579 | 256 / 323 | 323 | 0.000 / 0.652 / 0.000 | 0.677 / 0.001 | 1.50e-09 / 6.66e-11 | 5.00 mm | 39.6% | 3.410 / 5.911 / 7.248 ms |
| 2 | 0 | 579 | 164 / 415 | 415 | 0.002 / 0.653 / 0.000 | 0.735 / 0.002 | 1.71e-09 / 6.08e-11 | 5.00 mm | 36.6% | 3.433 / 5.147 / 5.683 ms |
| 3 | 1 | 579 | 201 / 378 | 378 | 0.000 / 0.633 / 0.001 | 0.721 / 0.001 | 1.30e-09 / 4.56e-11 | 5.00 mm | 30.7% | 3.373 / 5.627 / 7.085 ms |

## Measured authority stack

This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.

| authority signal | observed envelope | provisional warning / critical | longest warning run |
|---|---:|---:|---:|
| Invariant · maximum hard-row residual | p99 1.14e-09 · max 1.71e-09 | 1e-09 / 1e-08 | 4 ticks |
| Viability · finite-support margin | p01 5.00 mm · min 5.00 mm | 7.5 / 5.0 mm | 263 ticks |
| Physical · nearest joint-position limit | p01 21.59° · min 21.59° | 5.0 / 0.0° | 0 ticks |
| Physical · peak actuator effort | p99 43.44% · max 43.60% | 80% / 100% | 0 ticks |
| Compute · Rust WBC wall time | p99 5703.0 µs · max 7791.3 µs | 4000 / 5000 µs | 20 tick |
| Persistent thermal / reliability | UNMODELED | no calibrated plant contract | N/A |

### Capability curves

Each row asks how many of the 2317 independent oracle ticks would be adverse if that absolute threshold were chosen. These sweeps expose sensitivity without tuning a threshold to this corpus.

| hard residual ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 1e-10 | 1819 | 78.51% |
| 3e-10 | 778 | 33.58% |
| 1e-09 | 40 | 1.73% |
| 3e-09 | 0 | 0.00% |
| 1e-08 | 0 | 0.00% |

| minimum support margin | adverse ticks | fraction |
|---:|---:|---:|
| 0.0 mm | 0 | 0.00% |
| 2.5 mm | 0 | 0.00% |
| 5.0 mm | 0 | 0.00% |
| 7.5 mm | 1430 | 61.72% |
| 10.0 mm | 1836 | 79.24% |
| 15.0 mm | 2038 | 87.96% |
| 20.0 mm | 2091 | 90.25% |

| minimum joint headroom | adverse ticks | fraction |
|---:|---:|---:|
| 0.0° | 0 | 0.00% |
| 1.0° | 0 | 0.00% |
| 2.0° | 0 | 0.00% |
| 5.0° | 0 | 0.00% |
| 10.0° | 0 | 0.00% |
| 15.0° | 0 | 0.00% |

| maximum actuator utilization | adverse ticks | fraction |
|---:|---:|---:|
| 25% | 1624 | 70.09% |
| 50% | 0 | 0.00% |
| 60% | 0 | 0.00% |
| 70% | 0 | 0.00% |
| 80% | 0 | 0.00% |
| 90% | 0 | 0.00% |
| 100% | 0 | 0.00% |

| WBC wall-time ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 2.0 ms | 1784 | 77.00% |
| 3.0 ms | 1431 | 61.76% |
| 4.0 ms | 167 | 7.21% |
| 5.0 ms | 56 | 2.42% |
| 10.0 ms | 0 | 0.00% |
| 20.0 ms | 0 | 0.00% |

### Actuator and nullspace provenance

The physical bound used by Rust is `min(global 2000 Nm cap, URDF effort limit)` for each coordinate. The most highly utilized actuators are:

| actuator | authored limit | maximum effort | p99 use | peak use |
|---|---:|---:|---:|---:|
| `right_ankle_pitch_joint` | 35.0 Nm | 15.26 Nm | 43.44% | 43.60% |
| `left_ankle_pitch_joint` | 35.0 Nm | 13.85 Nm | 38.26% | 39.58% |
| `left_shoulder_pitch_joint` | 25.0 Nm | 7.52 Nm | 15.96% | 30.08% |
| `left_shoulder_roll_joint` | 25.0 Nm | 7.28 Nm | 10.14% | 29.12% |
| `right_shoulder_roll_joint` | 25.0 Nm | 6.91 Nm | 13.02% | 27.65% |
| `right_shoulder_pitch_joint` | 25.0 Nm | 6.03 Nm | 14.43% | 24.10% |

The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.

| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |
|---|---:|---:|---:|---:|
| Invariant | 0 | 1.00 / 1 | 0.00 / 0 | 0.00 / 0 |
| Viability | 1211 | 3.87 / 15 | 0.00 / 0 | 3.15 / 14 |
| Intent | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Preference | 1393 | 1.35 / 7 | 17.68 / 98 | 0.86 / 7 |
| Style | 1264 | 1.03 / 3 | 14.31 / 54 | 0.56 / 3 |

Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.

### Tight physical bounds versus loose control

The retained loose-bound control is `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-multistep-oracle-r54/no-loose-control.npz`. It is available: `False`. All 0 compared physical/status arrays are bit-for-bit identical: `False`. The URDF effort limits certify the existing solution rather than changing it; this does **not** imply that effort, speed, electrical power, temperature, or reliability are interchangeable.

## Predeclared gates

- PASS `reference_inputs_remain_bitwise_immutable`
- PASS `all_kinematic_witness_ticks_converge`
- PASS `kinematic_point_error_le_1cm`
- PASS `kinematic_com_error_le_3cm`
- PASS `kinematic_joint_velocity_le_8rad_s`
- PASS `kinematic_joint_acceleration_le_200rad_s2`
- PASS `kinematic_hot_loop_has_zero_allocations`
- PASS `kinematic_jet_hot_loop_has_zero_allocations`
- PASS `kinematic_point_velocity_residual_le_10pct`
- PASS `kinematic_point_acceleration_residual_le_25pct`
- PASS `kinematic_com_velocity_residual_le_40pct`
- PASS `kinematic_com_acceleration_residual_le_40pct`
- PASS `kinematic_orientation_error_le_1deg`
- PASS `kinematic_angular_velocity_residual_le_1mrad_s`
- PASS `kinematic_angular_acceleration_residual_le_1e_2rad_s2`
- PASS `no_wbc_infeasible_or_failed_ticks`
- PASS `wbc_dynamics_residual_le_1e_8`
- PASS `wbc_contact_residual_le_1e_8`
- PASS `wbc_friction_margin_nonnegative`
- PASS `wbc_cop_respects_declared_eroded_support`
- PASS `wbc_torque_margin_nonnegative`
- PASS `authority_joint_positions_within_authored_limits`
- PASS `authority_actuator_effort_within_authored_limits`
- PASS `authority_original_constraint_violation_le_1e_8`
- PASS `wbc_root_angular_task_rms_le_1rad_s2`
- PASS `wbc_root_linear_and_com_task_rms_le_1m_s2`
- PASS `wbc_effector_linear_task_rms_le_0_1m_s2`
- PASS `wbc_effector_angular_task_rms_le_1rad_s2`
- PASS `wbc_p99_latency_le_20ms`
- PASS `wbc_hot_loop_has_zero_allocations`
- PASS `physical_outputs_are_bitwise_repeatable`
- PASS `reference_contact_transitions_meet_declared_minimum`
- PASS `reference_liftoffs_meet_declared_minimum`
- PASS `reference_liftoffs_strictly_alternate`
- PASS `reference_has_no_flight_ticks`
- PASS `contact_edge_position_delta_le_1mm`
- PASS `contact_edge_velocity_delta_le_0_02mps`
- PASS `contact_edge_acceleration_delta_le_0_5mps2`
- PASS `wbc_transition_windows_all_solve`
- PASS `wbc_transition_dynamics_residual_le_1e_8`
- PASS `wbc_transition_contact_residual_le_1e_8`
- PASS `wbc_transition_cop_respects_eroded_support`
- PASS `wbc_transition_latency_le_20ms`

## Interpretation

A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red hard-admission gate means the solver cannot preserve dynamics/contact/actuation invariants. A tracking-red result with hard admission green means the reference remains physically feasible only by continuously relaxing declared acceleration tasks; the solved-with-slack count, per-task residuals, and per-step rows quantify exactly where and how much. Neither result can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
