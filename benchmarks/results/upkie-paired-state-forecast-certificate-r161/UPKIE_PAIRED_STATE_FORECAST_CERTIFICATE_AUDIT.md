# Bonesaw paired-state forecast certificate · r161

> Offline certificate discovery **FAIL** · live authority promotion **NO**.

## Result

R160's 19 misses were all pitch angle/rate and exposed a missing conjugate-state feature. R161 freezes that result, adds the paired position/rate bin, refuses any cell with fewer than eight calibration samples or a component bound above one declared state scale, expands calibration to the 33 now-observed r133/r160 cases, and evaluates 13 second-holdout cases that were not used to select the layout.

The second holdout contains **50,387** support-stable comparisons. The candidate issues **8,089 (16.054%)**, records **8** issued misses, covers **99.901100%** of issued samples, and retains a largest issued bound of **0.962×** its component scale. Sparse/unseen refusals occur on **305** comparisons and over-wide-bound refusals on **42,039**; these counts can overlap.

## Second holdout by horizon

| ms | samples | issued | issuance | misses | coverage | worst error/bound | largest norm bound |
|---|---|---|---|---|---|---|---|
| 30 | 6756 | 986 | 14.59% | 3 | 99.69574% | 3.699 | 0.944 |
| 60 | 6549 | 954 | 14.57% | 2 | 99.79036% | 1.908 | 0.962 |
| 90 | 6423 | 959 | 14.93% | 3 | 99.68717% | 1.739 | 0.948 |
| 120 | 6315 | 940 | 14.89% | 0 | 100.00000% | 0.710 | 0.770 |
| 150 | 6215 | 1078 | 17.35% | 0 | 100.00000% | 0.825 | 0.740 |
| 180 | 6127 | 1065 | 17.38% | 0 | 100.00000% | 0.567 | 0.722 |
| 210 | 6043 | 1058 | 17.51% | 0 | 100.00000% | 0.651 | 0.787 |
| 240 | 5959 | 1049 | 17.60% | 0 | 100.00000% | 0.680 | 0.803 |

## Second holdout by case

| case | samples | issued | issuance | misses | coverage | worst error/bound | replay |
|---|---|---|---|---|---|---|---|
| r161_backward_1p5n | 4955 | 1176 | 23.73% | 0 | 100.00000% | 0.680 | YES |
| r161_backward_3n_friction_0p5 | 4307 | 1343 | 31.18% | 0 | 100.00000% | 0.608 | YES |
| r161_backward_5n | 1496 | 117 | 7.82% | 0 | 100.00000% | 0.451 | YES |
| r161_diagonal_opposite_4p5n | 1475 | 114 | 7.73% | 0 | 100.00000% | 0.744 | YES |
| r161_down_3n | 4473 | 343 | 7.67% | 0 | 100.00000% | 0.143 | YES |
| r161_forward_0p5n | 4929 | 343 | 6.96% | 0 | 100.00000% | 0.367 | YES |
| r161_forward_2p5n | 6918 | 1676 | 24.23% | 0 | 100.00000% | 0.585 | YES |
| r161_forward_3n_150ms | 1792 | 254 | 14.17% | 7 | 97.24409% | 3.699 | YES |
| r161_forward_5n | 1473 | 114 | 7.74% | 0 | 100.00000% | 0.502 | YES |
| r161_handle_forward_2n | 9144 | 1831 | 20.02% | 1 | 99.94539% | 1.133 | YES |
| r161_left_2p5n | 2080 | 219 | 10.53% | 0 | 100.00000% | 0.476 | YES |
| r161_right_2p5n | 2880 | 223 | 7.74% | 0 | 100.00000% | 0.710 | YES |
| r161_up_3n | 4465 | 336 | 7.53% | 0 | 100.00000% | 0.126 | YES |

## Gates

| gate | result |
|---|---|
| calibration_complete | PASS |
| second_holdout_complete | PASS |
| second_holdout_disjoint | PASS |
| second_holdout_replay_exact | PASS |
| origin_only_features | PASS |
| minimum_issuance_fraction | FAIL |
| zero_issued_holdout_misses | FAIL |
| all_issued_bounds_within_one_state_scale | PASS |

## Authority boundary

Only origin-time state, achieved acceleration, and exact support enter a cell. A refusal is non-executable; future force, support, and outcome remain unavailable. Even a full offline pass would authorize only implementation of a fingerprinted fixed-capacity Rust certificate with sequence/evidence/support/age revocation, followed by a third untouched holdout and the complete causal plant gate. R161 itself cannot affect request or torque authority.
