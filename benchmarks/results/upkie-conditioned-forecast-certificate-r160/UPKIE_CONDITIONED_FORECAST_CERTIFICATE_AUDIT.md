# Bonesaw conditioned forecast certificate discovery · r160

> Discovery gates **FAIL** · live authority promotion **NO**.

## Result

The retained 20-case calibration contributes **63,308** support-stable joint samples and **1,212** populated component cells; **564** meet the predeclared eight-sample minimum. The 13 genuinely new cases contribute **57,999** samples. The certificate issues for **57,434/57,999 (99.026%)**, misses **19** issued samples, and covers **99.96692%** of issued samples. Its largest issued component bound is **2.599×** that state's declared scale.

Cells are keyed only by forecast knot, exact origin support, state component, absolute normalized origin state, absolute normalized final-WBC achieved acceleration, and whether that acceleration opens the component's short capture coordinate. Calibration uses a 2× observed-maximum reserve. Future wrench, future support, fall outcome, and realized error are unavailable to issuance. Sparse or unseen cells refuse issuance.

## New holdout by horizon

| horizon ms | samples | issued | issuance | misses | issued coverage | worst error/bound | largest norm bound |
|---|---|---|---|---|---|---|---|
| 30 | 7960 | 7802 | 98.02% | 11 | 99.8590% | 3.889 | 1.419 |
| 60 | 7657 | 7570 | 98.86% | 5 | 99.9339% | 3.058 | 2.599 |
| 90 | 7452 | 7392 | 99.19% | 1 | 99.9865% | 1.369 | 1.695 |
| 120 | 7274 | 7246 | 99.62% | 0 | 100.0000% | 0.759 | 1.696 |
| 150 | 7115 | 7092 | 99.68% | 0 | 100.0000% | 0.687 | 1.696 |
| 180 | 6977 | 6933 | 99.37% | 0 | 100.0000% | 0.674 | 1.697 |
| 210 | 6845 | 6761 | 98.77% | 2 | 99.9704% | 1.148 | 1.698 |
| 240 | 6719 | 6638 | 98.79% | 0 | 100.0000% | 0.766 | 1.698 |

## New holdout by case

| case | samples | issued | issuance | misses | issued coverage | worst error/bound | replay |
|---|---|---|---|---|---|---|---|
| r160_backward_3n | 9368 | 9273 | 98.99% | 2 | 99.9784% | 1.148 | YES |
| r160_diagonal_opposite_3n | 2021 | 1937 | 95.84% | 4 | 99.7935% | 3.889 | YES |
| r160_down_2n | 4585 | 4577 | 99.83% | 0 | 100.0000% | 0.491 | YES |
| r160_forward_1n | 4596 | 4585 | 99.76% | 0 | 100.0000% | 0.496 | YES |
| r160_forward_1p5n_250ms | 2209 | 2202 | 99.68% | 8 | 99.6367% | 3.058 | YES |
| r160_forward_3n | 8836 | 8798 | 99.57% | 0 | 100.0000% | 0.766 | YES |
| r160_forward_3n_friction_0p2 | 8836 | 8798 | 99.57% | 0 | 100.0000% | 0.766 | YES |
| r160_forward_4n_75ms | 4104 | 4030 | 98.20% | 0 | 100.0000% | 0.653 | YES |
| r160_handle_backward_3n | 1640 | 1632 | 99.51% | 2 | 99.8775% | 2.581 | YES |
| r160_left_1p5n_two_pulses | 2548 | 2495 | 97.92% | 3 | 99.8798% | 1.154 | YES |
| r160_left_3n | 2721 | 2675 | 98.31% | 0 | 100.0000% | 0.674 | YES |
| r160_right_3n | 1910 | 1871 | 97.96% | 0 | 100.0000% | 0.650 | YES |
| r160_up_2n | 4625 | 4561 | 98.62% | 0 | 100.0000% | 0.321 | YES |

## Admission gates

| gate | result |
|---|---|
| retained_calibration_complete | PASS |
| new_holdout_complete | PASS |
| new_holdout_names_disjoint | PASS |
| new_holdout_replay_exact | PASS |
| calibration_samples_exercised | PASS |
| holdout_samples_exercised | PASS |
| origin_only_features | PASS |
| minimum_issuance_fraction | PASS |
| zero_issued_holdout_misses | FAIL |
| useful_maximum_bound | FAIL |

## Authority boundary

This is model discovery, not a safety guarantee and not an executable request. Passing would only make the fixed cell layout eligible for a typed Rust implementation with sequence, evidence, support, age, and calibration-fingerprint revocation. That implementation would still require an additional untouched holdout and the complete causal plant A/B. A failed gate is retained as evidence and cannot be repaired by silently widening or dropping the offending new cases.
