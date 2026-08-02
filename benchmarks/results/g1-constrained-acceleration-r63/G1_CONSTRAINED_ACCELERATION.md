# Bonesaw G1 constrained-acceleration realization · r63

## Result

At every immutable r54 oracle state, a compact Rust solve imposes r62 realized actuator effort, floating dynamics, and active locked-contact acceleration as exact equalities, then minimizes departure from admitted r54 acceleration and contact force. Acceleration bounds, unilateral normal force, friction, and the 5 mm finite-patch support margin are scored continuously afterward rather than converted into an active-set timeout. This exposes instantaneous consequence without a policy, state integration, contact simulation, or rigid-body rollout. Every profile covers all 2,317 states; deterministic replay uses a declared 128-tick witness prefix per profile.

> This is a state-local counterfactual, not forward simulation and not evidence of closed-loop stability. Synthetic actuator profiles remain sensitivity envelopes, not Unitree G1 calibration. Each signal stays separate; there is no aggregate health score.

## Capability matrix

| case | bandwidth / rate / availability | accel RMS | root lin / joint RMS | p99 / max bound pressure | longest >1% | edge recovery max | A/N/F/S violation ticks | p99 Rust solve |
|---|---|---|---|---|---|---|---|---|
| ideal_control | infinite Hz / infinite Nm/s / 100% | 0.0000 | 0.0000 / 0.0000 | 0.000% / 0.000% | 0 ms | 0 ms | 0/0/0/0 | 1776.8 µs |
| fast_synthetic | 20.0 Hz / 5000.0 Nm/s / 100% | 4.5785 | 0.0338 / 5.1410 | 15.591% / 381.582% | 165 ms | 75 ms | 3/386/1085/427 | 1520.8 µs |
| medium_synthetic | 10.0 Hz / 1000.0 Nm/s / 100% | 7.3042 | 0.0562 / 8.2012 | 30.600% / 491.513% | 770 ms | 105 ms | 8/377/1079/413 | 1911.7 µs |
| slow_synthetic | 5.0 Hz / 250.0 Nm/s / 100% | 10.7278 | 0.1042 / 12.0413 | 63.094% / 548.727% | 1170 ms | 510 ms | 15/373/1021/347 | 1896.8 µs |
| half_available | 10.0 Hz / 1000.0 Nm/s / 50% | 7.3046 | 0.0560 / 8.2016 | 30.600% / 491.513% | 770 ms | 105 ms | 8/377/1079/413 | 1894.4 µs |
| quarter_available_slow | 5.0 Hz / 250.0 Nm/s / 25% | 11.0553 | 0.2640 / 12.4016 | 65.613% / 550.588% | 5655 ms | 120 ms | 15/222/960/110 | 2119.4 µs |

## Most exposed coordinates

| case | coordinate | p99 absolute acceleration error | maximum | RMS |
|---|---|---|---|---|
| ideal_control | left_hip_yaw_joint | 0.00000 | 0.00000 | 0.00000 |
| ideal_control | right_hip_yaw_joint | 0.00000 | 0.00000 | 0.00000 |
| ideal_control | left_ankle_roll_joint | 0.00000 | 0.00000 | 0.00000 |
| fast_synthetic | right_shoulder_roll_joint | 15.23081 | 70.80756 | 3.57291 |
| fast_synthetic | left_shoulder_roll_joint | 15.17397 | 66.10235 | 3.60535 |
| fast_synthetic | left_elbow_joint | 13.54461 | 55.90142 | 3.82032 |
| medium_synthetic | left_elbow_joint | 17.18214 | 71.41081 | 5.02326 |
| medium_synthetic | left_shoulder_roll_joint | 16.09899 | 89.33798 | 4.65512 |
| medium_synthetic | right_shoulder_roll_joint | 15.63908 | 99.12340 | 4.65354 |
| slow_synthetic | left_ankle_roll_joint | 54.28929 | 1097.45466 | 43.85266 |
| slow_synthetic | left_ankle_pitch_joint | 34.01175 | 753.35519 | 30.24731 |
| slow_synthetic | right_ankle_roll_joint | 31.58873 | 338.78507 | 14.12695 |
| half_available | left_elbow_joint | 17.18214 | 71.27093 | 5.00211 |
| half_available | left_shoulder_roll_joint | 16.09899 | 96.06934 | 4.69654 |
| half_available | right_shoulder_roll_joint | 15.63908 | 98.63790 | 4.65069 |
| quarter_available_slow | left_ankle_roll_joint | 55.84452 | 1101.17507 | 44.18407 |
| quarter_available_slow | left_ankle_pitch_joint | 31.72820 | 750.55835 | 30.13900 |
| quarter_available_slow | right_ankle_roll_joint | 30.90245 | 338.78507 | 14.11055 |

## Interpretation

- r54 remains the admission certificate for commanded WBC acceleration and effort. r62 remains the causal command-response certificate. r63 only maps r62 effort into a state-local constrained acceleration consequence.
- Bound pressure is the largest coordinate acceleration error divided by the declared 200-unit generalized acceleration bound. Root angular, root linear, and joint RMS values remain separate because their physical units differ.
- Fixed effort, floating dynamics, and locked-contact acceleration are hard equalities. Acceleration bounds, unilateral force, friction, and support are independent continuous signals; crossing one is measured loss of authority, not hidden solver slack.
- The compact equality solve deliberately removes the generic torque decision block and inequality projector. This makes the query continuous and allocation-free, while r54 remains the separate full hard-inequality admission certificate.
- Dwell and edge recovery distinguish a brief mismatch from persistent loss of authority. An unrecovered edge means the 1%-of-bound threshold did not hold for five ticks before the next contact transition.
- Hardware deployment still requires measured actuator bandwidth/rate/derating profiles, estimator error, delay, compliance, and an integrated closed-loop stability campaign.

## Mechanism gates

- PASS `source_artifacts_remain_immutable`
- PASS `r54_full_contact_force_replay_is_byte_exact_on_31_non_timing_fields`
- PASS `ideal_fixed_effort_reconstructs_admitted_acceleration_within_1e-5`
- PASS `all_scenarios_repeat_128_tick_witness_bit_exactly_except_timing`
- PASS `all_admitted_fixed_effort_equalities_hold_within_1e-6_nm`
- PASS `all_scenarios_avoid_numerical_failure`
- PASS `all_admitted_ticks_keep_hard_constraint_violations_below_1e-7`
- PASS `all_rust_steps_are_allocation_free`

## Artifacts

`constrained-acceleration-raw.npz` retains every scenario's realized generalized acceleration, acceleration error, fixed effort, contact force, task residual, equality residual, timing, allocation trace, and all four scored-inequality traces. The JSON retains the full capability curve, dwell, edge recovery, worst coordinates, exact-equality evidence, separate inequality pressure, evaluation boundary, and source checksums. The CSV is the compact matrix.
