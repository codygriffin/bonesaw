# Bonesaw executed-command forecast-versus-realized contract · r159

> Measurement mechanism **PASS** · unconditional frozen holdout **FAIL** · force-quiescent frozen holdout **FAIL** · live authority promotion **NO**.

## Result

The final admitted WBC emitted **11,034** allocation-free eight-knot path witnesses. After exact-support provenance, transition revocation, and terminal censoring, **63,308** forecast/realization pairs remain. The unconditional independent holdout covered **32,207/32,787 (98.2310%)** pairs jointly across all eight state components. This failure is retained: future external wrench is absent from the forecast state.

R157 first measured the 26 shadow-confirmed proposal paths and exposed hybrid-support error. R158 adds and causally tests a support-dwell/direction/load guard. R159 broadens the realization question to every final fresh exact-WBC command and freezes a separate calibration/holdout corpus. It does not let either envelope command the robot. A typed Rust certificate still has to consume an admitted bound, expire on sequence/evidence/support changes, and pass a second causal plant A/B before authority can move.

## Error over execution time

| horizon ms | cal n | cal norm p99 | cal norm max | holdout n | holdout norm p99 | holdout norm max | joint coverage | worst bound ratio |
|---|---|---|---|---|---|---|---|---|
| 30 | 4339 | 0.385 | 1.693 | 4881 | 0.287 | 0.877 | 100.000% | 0.981 |
| 60 | 4098 | 0.469 | 1.299 | 4526 | 0.477 | 0.961 | 99.426% | 2.256 |
| 90 | 3938 | 0.355 | 1.235 | 4284 | 0.598 | 1.108 | 99.743% | 2.249 |
| 120 | 3816 | 0.300 | 0.848 | 4101 | 0.677 | 1.376 | 98.781% | 3.210 |
| 150 | 3715 | 0.269 | 0.848 | 3941 | 0.680 | 1.489 | 97.437% | 11.874 |
| 180 | 3625 | 0.273 | 0.848 | 3805 | 0.677 | 1.492 | 97.188% | 11.461 |
| 210 | 3537 | 0.281 | 0.849 | 3683 | 0.637 | 1.427 | 96.443% | 13.343 |
| 240 | 3453 | 0.291 | 0.849 | 3566 | 0.599 | 1.260 | 95.681% | 14.275 |

Normalized error is the maximum of absolute roll/pitch/yaw error divided by 45°, angular-rate errors divided by 4 rad/s, lateral-position error divided by 0.10 m, and lateral-velocity error divided by 1 m/s. Absolute per-state distributions and all 64 calibrated bounds are retained in JSON.

## Conditional force-quiescent envelope

When no unmodeled external force acts anywhere inside the forecast interval, **58,209** pairs remain and the independent holdout covers **29,486/29,765 (99.0627%)** jointly. The **5,099** excluded pairs remain in the unconditional table above; they are not relabeled as model success.

| horizon ms | cal n | cal norm p99 | cal norm max | holdout n | holdout norm p99 | holdout norm max | joint coverage | worst bound ratio |
|---|---|---|---|---|---|---|---|---|
| 30 | 4156 | 0.349 | 1.693 | 4614 | 0.268 | 0.877 | 100.000% | 0.981 |
| 60 | 3891 | 0.409 | 1.299 | 4218 | 0.414 | 0.961 | 99.384% | 2.722 |
| 90 | 3710 | 0.087 | 1.235 | 3946 | 0.430 | 1.108 | 99.721% | 2.519 |
| 120 | 3565 | 0.095 | 0.825 | 3732 | 0.541 | 1.376 | 98.660% | 3.416 |
| 150 | 3440 | 0.129 | 0.165 | 3542 | 0.523 | 1.489 | 98.108% | 21.053 |
| 180 | 3332 | 0.164 | 0.214 | 3382 | 0.430 | 1.492 | 98.788% | 12.951 |
| 210 | 3226 | 0.197 | 0.257 | 3236 | 0.213 | 1.427 | 98.795% | 13.343 |
| 240 | 3124 | 0.223 | 0.293 | 3095 | 0.231 | 1.260 | 98.546% | 14.275 |

## Cases and censoring

| case | partition | origins | 240 ms pairs | transition censored | forced pairs | support mismatch | terminal censored | forecast p99 µs | replay |
|---|---|---|---|---|---|---|---|---|---|
| nominal | calibration | 681 | 513 | 823 | 0 | 66 | 212 | 0.48 | YES |
| forward_2n | calibration | 735 | 560 | 783 | 168 | 39 | 216 | 0.53 | YES |
| forward_4n_reference | holdout | 570 | 197 | 2163 | 320 | 128 | 216 | 0.66 | YES |
| forward_6n_overload | calibration | 246 | 166 | 235 | 120 | 21 | 216 | 0.46 | YES |
| backward_2n | calibration | 682 | 492 | 858 | 168 | 69 | 214 | 0.45 | YES |
| backward_4n | holdout | 545 | 222 | 1710 | 248 | 46 | 216 | 0.60 | YES |
| left_1n | calibration | 451 | 246 | 1080 | 368 | 113 | 198 | 0.50 | YES |
| left_2n | holdout | 523 | 333 | 926 | 368 | 109 | 216 | 0.40 | YES |
| left_4n | calibration | 369 | 203 | 695 | 332 | 22 | 216 | 0.54 | YES |
| right_2n | calibration | 402 | 262 | 632 | 368 | 49 | 216 | 0.93 | YES |
| right_4n | holdout | 439 | 275 | 799 | 368 | 80 | 216 | 0.55 | YES |
| diagonal_4n | calibration | 496 | 248 | 1193 | 121 | 83 | 216 | 0.49 | YES |
| up_4n | calibration | 752 | 604 | 686 | 368 | 100 | 215 | 0.77 | YES |
| down_4n | holdout | 684 | 561 | 518 | 368 | 72 | 188 | 0.51 | YES |
| handle_forward_4n | holdout | 409 | 157 | 1326 | 76 | 92 | 216 | 0.47 | YES |
| short_8n_50ms | calibration | 357 | 159 | 1110 | 64 | 55 | 216 | 0.42 | YES |
| long_2n_200ms | holdout | 627 | 287 | 1752 | 176 | 18 | 184 | 0.67 | YES |
| forward_2n_three_pulses | holdout | 723 | 534 | 761 | 778 | 27 | 201 | 0.49 | YES |
| forward_4n_friction_0p1 | holdout | 1200 | 986 | 857 | 320 | 16 | 216 | 0.40 | YES |
| forward_4n_friction_0p03 | holdout | 143 | 14 | 547 | 0 | 85 | 216 | 0.47 | YES |

A raw support change is a certificate revocation, not a large model residual. An origin whose exact raw support differs from the WBC's admitted support is also censored. This keeps contact-authority errors visible rather than laundering them into a permissive dynamics bound.

## Mechanism gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| frozen_calibration_holdout_split_complete | PASS |
| candidate_replay_exact | PASS |
| eight_fixed_knots | PASS |
| knot_times_are_30_to_240_ms | PASS |
| calibration_has_every_knot | PASS |
| holdout_has_every_knot | PASS |
| quiescent_calibration_has_every_knot | PASS |
| quiescent_holdout_has_every_knot | PASS |
| finite_realization_error | PASS |
| zero_rust_allocation | PASS |

## Dataflow and allocation

Python owns the immutable case split, MuJoCo sequencing, future-state joins, censoring, statistics, and report generation. Rust owns the final exact WBC, fixed-size path integration, output validation, timing, and allocation witness. The hot path writes a caller-owned `[8, 9]` array; no path list, dictionary, or per-knot Python object is created in the control loop.

Forecast emission timing across valid origins: p50 **0.311 µs**, p99 **0.511 µs**, max **8.416 µs**. Rust allocation calls and bytes are zero by gate.

## Certificate boundary

The eventual certificate may erode a forecast by an admitted per-knot, per-state error bound only while input validation, exact observation provenance, monotonically increasing sequence, and unchanged support all hold. It must fail closed on expiry or any evidence discontinuity. R159 deliberately contains no path from `certificate_eligible_for_codification` to executable request authority. The frozen envelope misses holdout outliers, so it is not yet admissible.
