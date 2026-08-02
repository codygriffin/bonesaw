# Bonesaw G1 center-response + split-residual replay · r216

> Construction replay **PASS** · recommended causal row **closing_weighted_load_1** · profile **NOT PROMOTED** · authority **NOT ADMITTED** · physics/controller/policy steps **0**.

## Contract

- Inputs are the checksum-pinned immutable R215 replay. Completed impulses and raw model error are labels only; no simulator is stepped and no state is carried between rows.
- Twelve causal rows combine four declared center rules with 0, 1/2, or one supported-weight impulse. The thirteenth row uses completed impulse to place and drive the center and is explicitly oracle-only.
- Rust reconstructs point response, generalized momentum residual, and the exact Minkowski sum of independent root and articulated kinetic-impulse ellipsoids. The displayed 5% calibration reserve is label-informed construction width, not a frozen profile.

## Construction comparison

| candidate | causal | root E₂ J | joint E₂ J | worst cross-law membership | root ω width p95 | root v width p95 | joint width p95 | split bound p99 µs |
|---|---|---|---|---|---|---|---|---|
| geometric_load_0 | yes | 21.48360 | 22.94205 | 72.9% | 71.168 | 4.397 | 705.890 | 154.007 |
| geometric_load_0.5 | yes | 20.80070 | 23.43038 | 79.2% | 70.952 | 4.377 | 712.891 | 154.196 |
| geometric_load_1 | yes | 20.16470 | 24.35433 | 87.5% | 71.085 | 4.377 | 726.201 | 111.191 |
| closing_weighted_load_0 | yes | 21.30558 | 22.80558 | 72.9% | 70.914 | 4.381 | 703.766 | 169.167 |
| closing_weighted_load_0.5 | yes | 20.63231 | 23.29698 | 79.2% | 70.707 | 4.362 | 710.837 | 109.315 |
| closing_weighted_load_1 | yes | 20.00585 | 24.20652 | 87.5% | 70.837 | 4.361 | 723.978 | 119.003 |
| earliest_load_0 | yes | 23.40995 | 24.19410 | 66.7% | 73.702 | 4.558 | 725.196 | 105.925 |
| earliest_load_0.5 | yes | 22.48520 | 26.38352 | 72.9% | 74.522 | 4.592 | 756.110 | 115.865 |
| earliest_load_1 | yes | 21.60674 | 32.24944 | 72.9% | 77.719 | 4.756 | 833.674 | 128.610 |
| lowest_load_0 | yes | 23.40995 | 24.19410 | 66.7% | 73.702 | 4.558 | 725.196 | 131.877 |
| lowest_load_0.5 | yes | 22.48520 | 26.38352 | 72.9% | 74.522 | 4.592 | 756.110 | 138.449 |
| lowest_load_1 | yes | 21.60674 | 32.24944 | 72.9% | 77.719 | 4.756 | 833.674 | 111.699 |
| completed_impulse_center_oracle | ORACLE | 0.10782 | 0.86846 | 93.8% | 9.339 | 0.546 | 135.145 | 103.922 |

## Decision

R216 recommends `closing_weighted_load_1` only as the construction to freeze next. Its energy maxima were learned from R215 and therefore cannot validate themselves. R217 must round and freeze independent root/articulated budgets before new laws and state offsets are opened, and must keep strict coverage and useful width conjunctive.
