# G1 oracle-state WBC admission

**RED · 41/44 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable R44 root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

This run additionally enables a **synthetic coupled-actuation fixture** on `left_ankle_pitch_joint` / `right_ankle_pitch_joint` with the differential block `[[0.5, 0.5], [-1, 1]]`. These are deliberately not claimed as G1 transmission parameters. Rust enforces both actuator outputs as exact inequality rows at ±14.000 Nm; the pair reaches at least 99% utilization for 32 ticks and generalized/actuator mechanical power agrees within 1.07e-14 W.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 600/600 converged | foot 4.99 mm · CoM 27.28 mm | 1.3 µs | 18.4 µs | 35.7 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.17% · a 12.96% · CoM v 7.69% · a 25.54% | analytic Jv / Jq̈+J̇v | 29.0 µs | 35.7 µs | 45.8 µs | 0 calls / 0 B |
| Stateless floating WBC | 600/600 solved | dynamics 1.22e-09 · contact 4.88e-11 · CoP margin 0.500 cm | 2594.5 µs | 4567.2 µs | 5233.3 µs | 0 calls / 0 B |

The maximum viability-task RMS is `11.921308` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `4.429 rad/s` / `123.305 rad/s²`. The full generalized-acceleration witness deviation is `36.716` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

## Contact-transition windows

The authored schedule contains `1` liftoffs and `1` touchdowns (`2` edges total). A ±8-tick window around every edge covers `34` unique ticks. Liftoff feet are `[0]` and strictly alternate: `True`.

Across contact edges, the maximum authored position / velocity / acceleration deltas are `0.000 mm` / `0.000173 m/s` / `0.068844 m/s²`. Every edge window solves without infeasible/failed status; maximum dynamics/contact residual is `5.52e-10` / `1.54e-11`, minimum CoP margin is `5.000 mm`, peak actuator utilization is `100.00%`, and maximum WBC time is `3.525 ms`.

| tick | edge | foot | WBC solved | hard dynamics / contact | CoP margin | actuator use | max time | clipped ticks |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 199 | liftoff | 0 | 17/17 | 9.48e-11 / 3.47e-12 | 5.00 mm | 100.00% | 2.758 ms | 17 |
| 428 | touchdown | 0 | 17/17 | 5.52e-10 / 1.54e-11 | 8.61 mm | 100.00% | 3.525 ms | 15 |

## Measured authority stack

This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.

| authority signal | observed envelope | provisional warning / critical | longest warning run |
|---|---:|---:|---:|
| Invariant · maximum hard-row residual | p99 1.22e-09 · max 1.22e-09 | 1e-09 / 1e-08 | 17 ticks |
| Viability · finite-support margin | p01 5.00 mm · min 5.00 mm | 7.5 / 5.0 mm | 89 ticks |
| Physical · nearest joint-position limit | p01 11.48° · min 11.48° | 5.0 / 0.0° | 0 ticks |
| Physical · peak actuator effort | p99 100.00% · max 100.00% | 80% / 100% | 22 ticks |
| Compute · Rust WBC wall time | p99 4567.2 µs · max 5233.3 µs | 4000 / 5000 µs | 3 tick |
| Persistent thermal / reliability | UNMODELED | no calibrated plant contract | N/A |

### Capability curves

Each row asks how many of the 600 independent oracle ticks would be adverse if that absolute threshold were chosen. These sweeps expose sensitivity without tuning a threshold to this corpus.

| hard residual ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 1e-10 | 458 | 76.33% |
| 3e-10 | 148 | 24.67% |
| 1e-09 | 21 | 3.50% |
| 3e-09 | 0 | 0.00% |
| 1e-08 | 0 | 0.00% |

| minimum support margin | adverse ticks | fraction |
|---:|---:|---:|
| 0.0 mm | 0 | 0.00% |
| 2.5 mm | 0 | 0.00% |
| 5.0 mm | 0 | 0.00% |
| 7.5 mm | 185 | 30.83% |
| 10.0 mm | 415 | 69.17% |
| 15.0 mm | 518 | 86.33% |
| 20.0 mm | 548 | 91.33% |

| minimum joint headroom | adverse ticks | fraction |
|---:|---:|---:|
| 0.0° | 0 | 0.00% |
| 1.0° | 0 | 0.00% |
| 2.0° | 0 | 0.00% |
| 5.0° | 0 | 0.00% |
| 10.0° | 0 | 0.00% |
| 15.0° | 489 | 81.50% |

| maximum actuator utilization | adverse ticks | fraction |
|---:|---:|---:|
| 25% | 599 | 99.83% |
| 50% | 398 | 66.33% |
| 60% | 335 | 55.83% |
| 70% | 236 | 39.33% |
| 80% | 90 | 15.00% |
| 90% | 42 | 7.00% |
| 100% | 0 | 0.00% |

| WBC wall-time ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 2.0 ms | 445 | 74.17% |
| 3.0 ms | 123 | 20.50% |
| 4.0 ms | 13 | 2.17% |
| 5.0 ms | 1 | 0.17% |
| 10.0 ms | 0 | 0.00% |
| 20.0 ms | 0 | 0.00% |

### Actuator and nullspace provenance

Rust enforces the authored actuator-space box through exact rows of `Gᵀ τ_generalized`; the synthetic pair uses ±14.000 Nm and all other rows retain their program limits. The most highly utilized actuators are:

| actuator | authored limit | maximum effort | p99 use | peak use |
|---|---:|---:|---:|---:|
| `left_ankle_pitch_joint` | 14.0 Nm | 14.00 Nm | 100.00% | 100.00% |
| `right_ankle_pitch_joint` | 14.0 Nm | 13.85 Nm | 98.88% | 98.90% |
| `right_shoulder_roll_joint` | 25.0 Nm | 12.91 Nm | 43.21% | 51.63% |
| `left_shoulder_pitch_joint` | 25.0 Nm | 6.93 Nm | 20.82% | 27.70% |
| `right_shoulder_pitch_joint` | 25.0 Nm | 6.92 Nm | 25.77% | 27.67% |
| `left_shoulder_roll_joint` | 25.0 Nm | 6.25 Nm | 16.95% | 25.01% |

The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.

| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |
|---|---:|---:|---:|---:|
| Invariant | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Viability | 399 | 3.62 / 13 | 21.24 / 70 | 2.80 / 13 |
| Intent | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Preference | 142 | 1.23 / 5 | 10.62 / 47 | 0.34 / 5 |
| Style | 81 | 1.11 / 4 | 10.51 / 40 | 0.16 / 4 |

Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.

### Tight physical bounds versus loose control

The retained loose-bound control is `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-oracle-wbc-admission-r50-actuation/oracle-wbc-admission-raw.npz`. It is available: `True`. All 9 compared physical/status arrays are bit-for-bit identical: `False`. The coupled fixture is expected to differ from the independent loose-bound control when its actuator-space rows become active; comparison deltas are evidence of that intervention, not a parity regression.

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
- PASS `coupled_actuator_power_duality_le_1e_10w`
- PASS `coupled_actuator_polytope_is_materially_active`

## Interpretation

A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red algebraic WBC gate means the solver cannot admit that projected witness under the declared dynamics/contact/actuation envelope. Neither failure can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
