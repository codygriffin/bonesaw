# Upkie root-wrench force/moment scale probe — R318

Status: **EXPLORATORY EVIDENCE; NO AUTHORITY PROMOTION**.

The optional Rust-owned six-axis scale vector changes the measured consequence without changing public authority. This small screen is not a robustness qualification: the full R311 schedule/lever holdout, contiguous scale neighborhood, and hardware calibration remain required before any promotion.

| profile | case | terminal tick | nonadmitted | upright tail | max tilt | max torque |
|:--|:--|--:|:--|:--|--:|--:|
| scalar_0.70 | centered_plus8 | 351 | [338] | no | 0.924 | 0.334 |
| scalar_0.70 | upper_plus8 | — | — | yes | 0.118 | 0.164 |
| scalar_0.70 | upper_minus6 | — | — | yes | 0.126 | 0.163 |
| moment_0.70_force_0.52 | centered_plus8 | — | — | yes | 0.085 | 0.153 |
| moment_0.70_force_0.52 | upper_plus8 | 73 | [42] | no | 0.823 | 0.300 |
| moment_0.70_force_0.52 | upper_minus6 | — | — | yes | 0.144 | 0.153 |
| moment_0.70_force_0.68 | centered_plus8 | — | — | yes | 0.088 | 0.232 |
| moment_0.70_force_0.68 | upper_plus8 | 107 | — | no | 0.884 | 0.553 |
| moment_0.70_force_0.68 | upper_minus6 | — | — | yes | 0.120 | 0.153 |
| moment_0.68_force_0.70 | centered_plus8 | 357 | [338] | no | 1.003 | 0.336 |
| moment_0.68_force_0.70 | upper_plus8 | — | — | yes | 0.046 | 0.182 |
| moment_0.68_force_0.70 | upper_minus6 | — | — | yes | 0.124 | 0.153 |

## Gates

- PASS `frozen_mujoco_version_matches`
- PASS `public_controller_remains_default_off`
- PASS `vector_path_changes_at_least_one_observed_consequence`
- OPEN `all_vector_rows_meet_zero_fall_holdout`
- OPEN `all_vector_rows_meet_zero_nonadmission_holdout`

R310 remains public; no vector profile is executable authority.
