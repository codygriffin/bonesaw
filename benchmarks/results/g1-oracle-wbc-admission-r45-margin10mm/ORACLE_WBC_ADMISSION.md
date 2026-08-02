# G1 oracle-state WBC admission

**RED · 26/27 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first projects the immutable R44 root/CoM/foot geometry into a morphology-consistent joint position/velocity/acceleration witness, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. The WBC receives the acceleration tasks induced by that projected generalized jet; authored-to-projected tracking residuals are gated separately. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 600/600 converged | foot 4.99 mm · CoM 27.28 mm | 1.3 µs | 19.2 µs | 36.2 µs | 0 calls / 0 B |
| Rust analytic jet projection | authored residual: foot v 0.17% · a 12.96% · CoM v 7.69% · a 25.54% | analytic Jv / Jq̈+J̇v | 27.7 µs | 34.1 µs | 37.3 µs | 0 calls / 0 B |
| Stateless floating WBC | 600/600 solved | dynamics 1.22e-09 · contact 4.88e-11 · CoP margin 1.000 cm | 2462.1 µs | 3909.0 µs | 4057.5 µs | 0 calls / 0 B |

The maximum viability-task RMS is `0.571231` in the task's physical acceleration units; the maximum joint-witness speed/acceleration is `4.429 rad/s` / `123.305 rad/s²`. The full generalized-acceleration witness deviation is `37.237` in mixed tangent units, dominated by lower-priority posture/style directions rather than the separately gated physical tasks. Physical WBC outputs are bitwise repeatable: `True`.

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
- PASS `wbc_root_angular_task_rms_le_1rad_s2`
- PASS `wbc_root_linear_and_com_task_rms_le_1m_s2`
- FAIL `wbc_effector_task_rms_le_0_1`
- PASS `wbc_p99_latency_le_20ms`
- PASS `wbc_hot_loop_has_zero_allocations`
- PASS `physical_outputs_are_bitwise_repeatable`

## Interpretation

A red morphology gate means the authored reference cannot be projected within the declared position/jet error envelope. A red algebraic WBC gate means the solver cannot admit that projected witness under the declared dynamics/contact/actuation envelope. Neither failure can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
