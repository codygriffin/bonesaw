# G1 oracle-state WBC admission

**RED · 19/22 gates**

This benchmark contains no feedback policy, state integration, or physics simulator. Rust first generates a morphology-consistent joint witness for the immutable R43 root/CoM/foot geometry, then evaluates every floating inverse-dynamics WBC tick independently at that oracle state. Rigid-body mass, inertia, Jacobian, bias-force, contact, friction, and torque equations remain active because they are the WBC being evaluated.

## Results

| stage | status | geometry / residual | p50 | p99 | max | allocations |
|---|---:|---:|---:|---:|---:|---:|
| Rust whole-body IK witness | 600/600 converged | foot 4.99 mm · CoM 27.28 mm | 1.3 µs | 19.5 µs | 36.8 µs | 0 calls / 0 B |
| Stateless floating WBC | 600/600 solved | dynamics 1.21e-09 · contact 4.65e-11 | 2508.5 µs | 6371.3 µs | 6860.1 µs | 0 calls / 0 B |

The maximum viability-task RMS is `1.689832` in physical acceleration/momentum-rate units; the maximum joint-witness speed/acceleration is `3.899 rad/s` / `79.890 rad/s²`. Physical WBC outputs are bitwise repeatable: `True`.

## Predeclared gates

- PASS `reference_inputs_remain_bitwise_immutable`
- PASS `all_kinematic_witness_ticks_converge`
- PASS `kinematic_point_error_le_1cm`
- PASS `kinematic_com_error_le_3cm`
- PASS `kinematic_joint_velocity_le_8rad_s`
- PASS `kinematic_joint_acceleration_le_200rad_s2`
- PASS `kinematic_hot_loop_has_zero_allocations`
- PASS `kinematic_jet_hot_loop_has_zero_allocations`
- FAIL `kinematic_point_velocity_residual_le_1mm_s`
- FAIL `kinematic_point_acceleration_residual_le_1cm_s2`
- PASS `kinematic_orientation_error_le_1deg`
- PASS `kinematic_angular_velocity_residual_le_1mrad_s`
- PASS `kinematic_angular_acceleration_residual_le_1e_2rad_s2`
- PASS `no_wbc_infeasible_or_failed_ticks`
- PASS `wbc_dynamics_residual_le_1e_8`
- PASS `wbc_contact_residual_le_1e_8`
- PASS `wbc_friction_margin_nonnegative`
- PASS `wbc_torque_margin_nonnegative`
- FAIL `wbc_viability_task_rms_le_0_1`
- PASS `wbc_p99_latency_le_20ms`
- PASS `wbc_hot_loop_has_zero_allocations`
- PASS `physical_outputs_are_bitwise_repeatable`

## Interpretation

A red kinematic gate means the authored reference has no demonstrated morphology witness. A red algebraic WBC gate means the solver cannot admit the witness under the declared dynamics/contact/actuation envelope. Neither failure can be blamed on rollout tuning, disturbance policy, simulator contact, or accumulated state error.

The integrated floating corpus remains a separate closed-loop robustness test; it is not used to decide this stateless admission result.
