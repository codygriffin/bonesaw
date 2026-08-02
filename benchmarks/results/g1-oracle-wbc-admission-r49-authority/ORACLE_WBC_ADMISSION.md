# G1 oracle-state WBC admission

**PASS · 30/30 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable R44 root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 600/600 converged | foot 4.99 mm · CoM 27.28 mm | 1.3 µs | 19.6 µs | 36.9 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.17% · a 12.96% · CoM v 7.69% · a 25.54% | analytic Jv / Jq̈+J̇v | 28.8 µs | 33.9 µs | 37.3 µs | 0 calls / 0 B |
| Stateless floating WBC | 600/600 solved | dynamics 1.22e-09 · contact 4.88e-11 · CoP margin 0.500 cm | 2483.5 µs | 4021.5 µs | 4601.7 µs | 0 calls / 0 B |

The maximum viability-task RMS is `0.571231` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `4.429 rad/s` / `123.305 rad/s²`. The full generalized-acceleration witness deviation is `37.237` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

## Measured authority stack

This is a stack of independent, typed evidence—not an aggregate score. Warning and critical values are explicit provisional evaluation contracts. Corpus percentiles are shown as observations, never substituted for physical limits.

| authority signal | observed envelope | provisional warning / critical | longest warning run |
|---|---:|---:|---:|
| Invariant · maximum hard-row residual | p99 1.22e-09 · max 1.22e-09 | 1e-09 / 1e-08 | 17 ticks |
| Viability · finite-support margin | p01 5.00 mm · min 5.00 mm | 7.5 / 5.0 mm | 160 ticks |
| Physical · nearest joint-position limit | p01 11.48° · min 11.48° | 5.0 / 0.0° | 0 ticks |
| Physical · peak actuator effort | p99 42.84% · max 57.47% | 80% / 100% | 0 ticks |
| Compute · Rust WBC wall time | p99 4021.5 µs · max 4601.7 µs | 4000 / 5000 µs | 2 tick |
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
| 7.5 mm | 174 | 29.00% |
| 10.0 mm | 394 | 65.67% |
| 15.0 mm | 522 | 87.00% |
| 20.0 mm | 567 | 94.50% |

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
| 25% | 420 | 70.00% |
| 50% | 4 | 0.67% |
| 60% | 0 | 0.00% |
| 70% | 0 | 0.00% |
| 80% | 0 | 0.00% |
| 90% | 0 | 0.00% |
| 100% | 0 | 0.00% |

| WBC wall-time ceiling | adverse ticks | fraction |
|---:|---:|---:|
| 2.0 ms | 389 | 64.83% |
| 3.0 ms | 101 | 16.83% |
| 4.0 ms | 8 | 1.33% |
| 5.0 ms | 0 | 0.00% |
| 10.0 ms | 0 | 0.00% |
| 20.0 ms | 0 | 0.00% |

### Actuator and nullspace provenance

The physical bound used by Rust is `min(global 2000 Nm cap, URDF effort limit)` for each coordinate. The most highly utilized actuators are:

| actuator | URDF limit | maximum torque | p99 use | peak use |
|---|---:|---:|---:|---:|
| `left_ankle_pitch_joint` | 35.0 Nm | 20.12 Nm | 42.11% | 57.47% |
| `right_ankle_pitch_joint` | 35.0 Nm | 14.99 Nm | 41.94% | 42.83% |
| `left_shoulder_pitch_joint` | 25.0 Nm | 7.41 Nm | 16.27% | 29.65% |
| `right_shoulder_pitch_joint` | 25.0 Nm | 5.92 Nm | 16.75% | 23.67% |
| `right_hip_roll_joint` | 139.0 Nm | 32.69 Nm | 22.73% | 23.52% |
| `right_hip_pitch_joint` | 139.0 Nm | 30.46 Nm | 7.41% | 21.92% |

The lexicographic work counters show where the solver spends or clips authority. Invariant and Intent rows require no pseudoinverse work in this trace because their relevant conditions are represented by the hard feasibility system and inactive task slots, respectively.

| priority | ticks with clipping | pseudoinverse calls mean / max | Jacobi sweeps mean / max | clipped steps mean / max |
|---|---:|---:|---:|---:|
| Invariant | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Viability | 350 | 2.84 / 13 | 16.90 / 66 | 1.93 / 13 |
| Intent | 0 | 0.00 / 0 | 0.00 / 0 | 0.00 / 0 |
| Preference | 102 | 1.18 / 6 | 10.07 / 49 | 0.26 / 5 |
| Style | 103 | 1.15 / 3 | 11.53 / 33 | 0.18 / 3 |

Raw task RMS values are retained in JSON/NPZ but intentionally not colored together: the task slots have different physical units and require task-specific normalizers. Clipping remains a unit-safe provenance signal.

### Tight physical bounds versus loose control

The retained loose-bound control is `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-oracle-wbc-admission-r45-margin5mm/oracle-wbc-admission-raw.npz`. It is available: `True`. All 9 compared physical/status arrays are bit-for-bit identical: `True`. This means the URDF effort limits certify the existing solution rather than changing it; it does **not** imply that effort, speed, electrical power, temperature, or reliability are interchangeable.

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
- PASS `authority_actuator_effort_within_urdf_limits`
- PASS `authority_original_constraint_violation_le_1e_8`
- PASS `wbc_root_angular_task_rms_le_1rad_s2`
- PASS `wbc_root_linear_and_com_task_rms_le_1m_s2`
- PASS `wbc_effector_task_rms_le_0_1`
- PASS `wbc_p99_latency_le_20ms`
- PASS `wbc_hot_loop_has_zero_allocations`
- PASS `physical_outputs_are_bitwise_repeatable`

## Interpretation

A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red algebraic WBC gate means the solver cannot admit that projected witness under the declared dynamics/contact/actuation envelope. Neither failure can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
