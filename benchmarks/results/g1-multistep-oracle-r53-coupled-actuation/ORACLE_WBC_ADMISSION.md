# G1 oracle-state WBC admission

**HARD ADMISSION PASS · TRACKING RED · 43/45 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable reference root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

This run additionally enables a **synthetic coupled-actuation fixture** on `left_ankle_pitch_joint` / `right_ankle_pitch_joint` with the differential block `[[0.5, 0.5], [-1, 1]]`. These are deliberately not claimed as G1 transmission parameters. Rust enforces both actuator outputs as exact inequality rows at ±14.000 Nm; the pair reaches at least 99% utilization for 119 ticks and generalized/actuator mechanical power agrees within 4.44e-15 W.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 2317/2317 converged | foot 4.04 mm · CoM 29.20 mm | 18.4 µs | 24.8 µs | 83.1 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.62% · a 0.75% · CoM v 34.68% · a 35.21% | analytic Jv / Jq̈+J̇v | 28.6 µs | 35.8 µs | 43.5 µs | 0 calls / 0 B |
| Stateless floating WBC | 2317/2317 solved | dynamics 1.71e-09 · contact 6.66e-11 · CoP margin 0.500 cm | 3507.5 µs | 5938.1 µs | 9534.3 µs | 0 calls / 0 B |

The maximum viability-task RMS is `2.298717` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `1.432 rad/s` / `8.932 rad/s²`. The full generalized-acceleration witness deviation is `57.093` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

### Tracking residuals over execution time

Peak residual remains the admission rule, while p99, time-RMS, integrated residual, violating ticks, and longest contiguous run describe whether unavailable authority is isolated or persistent. Angular and linear task units are kept separate.

| task | units | p99 | max | time RMS | integral | above contract | longest run |
|---|---:|---:|---:|---:|---:|---:|---:|
| root_angular | rad/s^2 | 1.5351 | 2.2987 | 0.3937 | 1.9625 | 107 / 2317 | 21 ticks |
| root_horizontal | m/s^2 | 0.4974 | 0.7138 | 0.1417 | 0.7192 | 0 / 2317 | 0 ticks |
| root_height | m/s^2 | 0.9289 | 1.2967 | 0.2649 | 1.3334 | 12 / 2317 | 3 ticks |
| center_of_mass | m/s^2 | 0.6124 | 0.8920 | 0.1595 | 0.7768 | 0 / 2317 | 0 ticks |
| frame_angular_0 | rad/s^2 | 0.0006 | 0.0029 | 0.0002 | 0.0007 | 0 / 2317 | 0 ticks |
| point_0 | m/s^2 | 0.0008 | 0.0020 | 0.0002 | 0.0006 | 0 / 2317 | 0 ticks |
| point_1 | m/s^2 | 0.0006 | 0.0011 | 0.0001 | 0.0006 | 0 / 2317 | 0 ticks |

## Contact-transition windows

The authored schedule contains `4` liftoffs and `4` touchdowns (`8` edges total). A ±8-tick window around every edge covers `136` unique ticks. Liftoff feet are `[0, 1, 0, 1]` and strictly alternate: `True`.

Across contact edges, the maximum authored position / velocity / acceleration deltas are `0.000 mm` / `0.000163 m/s` / `0.064606 m/s²`. Every edge window solves without infeasible/failed status; maximum dynamics/contact residual is `1.10e-09` / `4.16e-11`, minimum CoP margin is `5.000 mm`, peak actuator utilization is `96.42%`, and maximum WBC time is `7.943 ms`.

| tick | edge | foot | WBC solved | hard dynamics / contact | CoP margin | actuator use | max time | clipped ticks |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 300 | liftoff | 0 | 17/17 | 5.52e-10 / 3.63e-11 | 5.14 mm | 90.27% | 7.943 ms | 8 |
| 529 | touchdown | 0 | 17/17 | 4.14e-10 / 1.46e-11 | 5.00 mm | 96.42% | 5.817 ms | 12 |
| 879 | liftoff | 1 | 17/17 | 8.87e-10 / 3.37e-11 | 5.00 mm | 41.21% | 4.170 ms | 8 |
| 1108 | touchdown | 1 | 17/17 | 7.71e-10 / 4.16e-11 | 5.00 mm | 42.94% | 4.487 ms | 9 |
| 1458 | liftoff | 0 | 17/17 | 4.95e-10 / 1.39e-11 | 5.00 mm | 71.55% | 4.650 ms | 14 |
| 1687 | touchdown | 0 | 17/17 | 5.99e-10 / 1.67e-11 | 5.00 mm | 73.18% | 4.554 ms | 13 |
| 2037 | liftoff | 1 | 17/17 | 1.10e-09 / 3.16e-11 | 5.00 mm | 31.31% | 4.785 ms | 13 |
| 2266 | touchdown | 1 | 17/17 | 3.82e-10 / 2.89e-11 | 6.23 mm | 31.32% | 3.475 ms | 9 |

### Per-step execution

Each row begins after the preceding touchdown (or tick zero) and ends at the current touchdown, so transfer and swing work are both charged to the step that consumes them.

| step | foot | ticks | solved / slack | clipped | root angular / linear / height | CoM / swing point | hard dynamics / contact | CoP | actuator | p50 / p99 / max |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 530 | 136 / 394 | 394 | 0.980 / 0.231 / 0.491 | 0.241 / 0.002 | 1.46e-09 / 5.37e-11 | 5.00 mm | 100.0% | 3.512 / 7.040 / 7.943 ms |
| 1 | 1 | 579 | 256 / 323 | 323 | 2.299 / 0.714 / 1.263 | 0.892 / 0.001 | 1.50e-09 / 6.66e-11 | 5.00 mm | 92.8% | 3.455 / 6.129 / 9.534 ms |
| 2 | 0 | 579 | 164 / 415 | 415 | 1.661 / 0.607 / 1.156 | 0.796 / 0.002 | 1.71e-09 / 6.08e-11 | 5.00 mm | 100.0% | 3.624 / 5.165 / 6.110 ms |
| 3 | 1 | 579 | 201 / 378 | 378 | 1.409 / 0.650 / 1.297 | 0.787 / 0.001 | 1.30e-09 / 4.56e-11 | 5.00 mm | 67.9% | 3.487 / 5.402 / 6.177 ms |

## Measured authority stack

This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.

| authority signal | observed envelope | provisional warning / critical | longest warning run |
|---|---:|---:|---:|
| Invariant · maximum hard-row residual | p99 1.14e-09 · max 1.71e-09 | 1e-09 / 1e-08 | 4 ticks |
| Viability · finite-support margin | p01 5.00 mm · min 5.00 mm | 7.5 / 5.0 mm | 250 ticks |
| Physical · nearest joint-position limit | p01 21.59° · min 21.59° | 5.0 / 0.0° | 0 ticks |
| Physical · peak actuator effort | p99 100.00% · max 100.00% | 80% / 100% | 197 ticks |
| Compute · Rust WBC wall time | p99 5938.1 µs · max 9534.3 µs | 4000 / 5000 µs | 64 tick |
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
| 7.5 mm | 1386 | 59.82% |
| 10.0 mm | 1726 | 74.49% |
| 15.0 mm | 2011 | 86.79% |
| 20.0 mm | 2067 | 89.21% |

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
| 25% | 2266 | 97.80% |
| 50% | 1328 | 57.32% |
| 60% | 1060 | 45.75% |
| 70% | 872 | 37.63% |
| 80% | 517 | 22.31% |
| 90% | 165 | 7.12% |
| 100% | 0 | 0.00% |

| WBC wall-time ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 2.0 ms | 2317 | 100.00% |
| 3.0 ms | 1551 | 66.94% |
| 4.0 ms | 360 | 15.54% |
| 5.0 ms | 105 | 4.53% |
| 10.0 ms | 0 | 0.00% |
| 20.0 ms | 0 | 0.00% |

### Actuator and nullspace provenance

Rust enforces the authored actuator-space box through exact rows of `Gᵀ τ_generalized`; the synthetic pair uses ±14.000 Nm and all other rows retain their program limits. The most highly utilized actuators are:

| actuator | authored limit | maximum effort | p99 use | peak use |
|---|---:|---:|---:|---:|
| `left_ankle_pitch_joint` | 14.0 Nm | 14.00 Nm | 100.00% | 100.00% |
| `right_ankle_pitch_joint` | 14.0 Nm | 13.88 Nm | 99.12% | 99.13% |
| `left_shoulder_roll_joint` | 25.0 Nm | 16.77 Nm | 10.54% | 67.07% |
| `right_shoulder_roll_joint` | 25.0 Nm | 11.24 Nm | 18.25% | 44.98% |
| `left_shoulder_pitch_joint` | 25.0 Nm | 10.23 Nm | 16.67% | 40.93% |
| `right_shoulder_pitch_joint` | 25.0 Nm | 8.93 Nm | 14.66% | 35.71% |

The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.

| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |
|---|---:|---:|---:|---:|
| Invariant | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Viability | 1211 | 4.03 / 16 | 20.85 / 80 | 3.30 / 15 |
| Intent | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Preference | 1514 | 1.39 / 7 | 18.75 / 106 | 0.94 / 7 |
| Style | 1320 | 1.04 / 5 | 14.20 / 65 | 0.59 / 4 |

Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.

### Tight physical bounds versus loose control

The retained loose-bound control is `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-multistep-oracle-r53/oracle-wbc-admission-raw.npz`. It is available: `True`. All 9 compared physical/status arrays are bit-for-bit identical: `False`. The coupled fixture is expected to differ from the independent loose-bound control when its actuator-space rows become active; comparison deltas are evidence of that intervention, not a parity regression.

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
- FAIL `wbc_root_angular_task_rms_le_1rad_s2`
- FAIL `wbc_root_linear_and_com_task_rms_le_1m_s2`
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
- PASS `coupled_actuator_power_duality_le_1e_10w`
- PASS `coupled_actuator_polytope_is_materially_active`

## Interpretation

A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red hard-admission gate means the solver cannot preserve dynamics/contact/actuation invariants. A tracking-red result with hard admission green means the reference remains physically feasible only by continuously relaxing declared acceleration tasks; the solved-with-slack count, per-task residuals, and per-step rows quantify exactly where and how much. Neither result can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
