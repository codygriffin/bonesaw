# G1 oracle-state WBC admission

**HARD ADMISSION PASS · TRACKING RED · 39/42 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable reference root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

This run uses the program's independent actuator bounds; no synthetic coupled transmission is enabled.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 2317/2317 converged | foot 4.04 mm · CoM 29.20 mm | 18.3 µs | 26.3 µs | 77.9 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.79% · a 0.81% · CoM v 34.22% · a 34.80% | analytic Jv / Jq̈+J̇v | 28.5 µs | 40.5 µs | 50.3 µs | 0 calls / 0 B |
| Stateless floating WBC | 2317/2317 solved | dynamics 1.73e-09 · contact 6.58e-11 · CoP margin 0.500 cm | 2582.7 µs | 4270.5 µs | 5417.9 µs | 0 calls / 0 B |

The maximum viability-task RMS is `3.224011` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `1.432 rad/s` / `8.932 rad/s²`. The full generalized-acceleration witness deviation is `81.545` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

## Contact-transition windows

The authored schedule contains `4` liftoffs and `4` touchdowns (`8` edges total). A ±8-tick window around every edge covers `136` unique ticks. Liftoff feet are `[0, 1, 0, 1]` and strictly alternate: `True`.

Across contact edges, the maximum authored position / velocity / acceleration deltas are `0.000 mm` / `0.000163 m/s` / `0.064606 m/s²`. Every edge window solves without infeasible/failed status; maximum dynamics/contact residual is `9.81e-10` / `4.16e-11`, minimum CoP margin is `5.000 mm`, peak actuator utilization is `38.37%`, and maximum WBC time is `5.207 ms`.

| tick | edge | foot | WBC solved | hard dynamics / contact | CoP margin | actuator use | max time | clipped ticks |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 300 | liftoff | 0 | 17/17 | 9.81e-10 / 3.16e-11 | 5.00 mm | 35.90% | 3.876 ms | 8 |
| 529 | touchdown | 0 | 17/17 | 4.14e-10 / 1.45e-11 | 5.00 mm | 38.37% | 5.207 ms | 12 |
| 879 | liftoff | 1 | 17/17 | 8.65e-10 / 2.31e-11 | 5.00 mm | 33.95% | 3.002 ms | 8 |
| 1108 | touchdown | 1 | 17/17 | 5.68e-10 / 4.16e-11 | 5.00 mm | 33.48% | 3.017 ms | 9 |
| 1458 | liftoff | 0 | 17/17 | 6.31e-10 / 1.31e-11 | 5.00 mm | 29.93% | 4.099 ms | 14 |
| 1687 | touchdown | 0 | 17/17 | 5.86e-10 / 1.43e-11 | 5.00 mm | 29.05% | 3.377 ms | 13 |
| 2037 | liftoff | 1 | 17/17 | 6.58e-10 / 2.78e-11 | 5.00 mm | 27.19% | 4.669 ms | 13 |
| 2266 | touchdown | 1 | 17/17 | 3.81e-10 / 2.89e-11 | 6.16 mm | 24.19% | 2.599 ms | 9 |

### Per-step execution

Each row begins after the preceding touchdown (or tick zero) and ends at the current touchdown, so transfer and swing work are both charged to the step that consumes them.

| step | foot | ticks | solved / slack | clipped | root angular / linear / height | CoM / swing point | hard dynamics / contact | CoP | actuator | p50 / p99 / max |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 530 | 156 / 374 | 374 | 3.224 / 0.675 / 1.585 | 1.237 / 2.204 | 1.06e-09 / 6.58e-11 | 5.00 mm | 43.6% | 2.578 / 4.027 / 5.317 ms |
| 1 | 1 | 579 | 237 / 342 | 342 | 2.820 / 0.784 / 1.565 | 1.224 / 0.000 | 1.73e-09 / 5.50e-11 | 5.00 mm | 55.7% | 2.583 / 4.415 / 5.207 ms |
| 2 | 0 | 579 | 144 / 435 | 435 | 2.056 / 0.697 / 1.471 | 1.092 / 0.440 | 1.33e-09 / 5.47e-11 | 5.00 mm | 71.9% | 2.630 / 4.155 / 5.418 ms |
| 3 | 1 | 579 | 183 / 396 | 396 | 1.424 / 0.667 / 1.453 | 1.060 / 0.000 | 1.50e-09 / 5.80e-11 | 5.00 mm | 45.6% | 2.575 / 4.294 / 4.669 ms |

## Measured authority stack

This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.

| authority signal | observed envelope | provisional warning / critical | longest warning run |
|---|---:|---:|---:|
| Invariant · maximum hard-row residual | p99 1.03e-09 · max 1.73e-09 | 1e-09 / 1e-08 | 5 ticks |
| Viability · finite-support margin | p01 5.00 mm · min 5.00 mm | 7.5 / 5.0 mm | 297 ticks |
| Physical · nearest joint-position limit | p01 21.59° · min 21.59° | 5.0 / 0.0° | 0 ticks |
| Physical · peak actuator effort | p99 43.54% · max 71.91% | 80% / 100% | 0 ticks |
| Compute · Rust WBC wall time | p99 4270.5 µs · max 5417.9 µs | 4000 / 5000 µs | 6 tick |
| Persistent thermal / reliability | UNMODELED | no calibrated plant contract | N/A |

### Capability curves

Each row asks how many of the 2317 independent oracle ticks would be adverse if that absolute threshold were chosen. These sweeps expose sensitivity without tuning a threshold to this corpus.

| hard residual ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 1e-10 | 1840 | 79.41% |
| 3e-10 | 757 | 32.67% |
| 1e-09 | 30 | 1.29% |
| 3e-09 | 0 | 0.00% |
| 1e-08 | 0 | 0.00% |

| minimum support margin | adverse ticks | fraction |
|---:|---:|---:|
| 0.0 mm | 0 | 0.00% |
| 2.5 mm | 0 | 0.00% |
| 5.0 mm | 0 | 0.00% |
| 7.5 mm | 1450 | 62.58% |
| 10.0 mm | 1804 | 77.86% |
| 15.0 mm | 2022 | 87.27% |
| 20.0 mm | 2107 | 90.94% |

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
| 25% | 1895 | 81.79% |
| 50% | 4 | 0.17% |
| 60% | 1 | 0.04% |
| 70% | 1 | 0.04% |
| 80% | 0 | 0.00% |
| 90% | 0 | 0.00% |
| 100% | 0 | 0.00% |

| WBC wall-time ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 2.0 ms | 1561 | 67.37% |
| 3.0 ms | 333 | 14.37% |
| 4.0 ms | 43 | 1.86% |
| 5.0 ms | 5 | 0.22% |
| 10.0 ms | 0 | 0.00% |
| 20.0 ms | 0 | 0.00% |

### Actuator and nullspace provenance

The physical bound used by Rust is `min(global 2000 Nm cap, URDF effort limit)` for each coordinate. The most highly utilized actuators are:

| actuator | authored limit | maximum effort | p99 use | peak use |
|---|---:|---:|---:|---:|
| `right_shoulder_roll_joint` | 25.0 Nm | 17.98 Nm | 14.05% | 71.91% |
| `left_shoulder_pitch_joint` | 25.0 Nm | 15.22 Nm | 15.78% | 60.88% |
| `right_shoulder_pitch_joint` | 25.0 Nm | 14.12 Nm | 18.66% | 56.50% |
| `left_shoulder_roll_joint` | 25.0 Nm | 14.10 Nm | 12.28% | 56.40% |
| `right_ankle_pitch_joint` | 35.0 Nm | 19.49 Nm | 43.49% | 55.70% |
| `left_ankle_pitch_joint` | 35.0 Nm | 16.06 Nm | 38.38% | 45.90% |

The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.

| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |
|---|---:|---:|---:|---:|
| Invariant | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Viability | 1366 | 4.58 / 18 | 29.07 / 107 | 3.97 / 18 |
| Intent | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Preference | 1287 | 1.69 / 7 | 13.48 / 60 | 1.04 / 6 |
| Style | 632 | 1.14 / 5 | 10.33 / 43 | 0.33 / 5 |

Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.

### Tight physical bounds versus loose control

The retained loose-bound control is `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-multistep-oracle-r52/no-loose-control.npz`. It is available: `False`. All 0 compared physical/status arrays are bit-for-bit identical: `False`. The URDF effort limits certify the existing solution rather than changing it; this does **not** imply that effort, speed, electrical power, temperature, or reliability are interchangeable.

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
- FAIL `wbc_effector_task_rms_le_0_1`
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
