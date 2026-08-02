# Bonesaw G1 fixed-effort Pinocchio reference · r64

## Result

An independent Pinocchio 4.0.0 model reconstructs floating mass, bias, eight sole-point Jacobians, and contact acceleration bias at 48 selected immutable r54 states. A separate NumPy SVD implementation rebuilds the four-level r63 lexicographic equality solve for four effort profiles. No Bonesaw model product or solver workspace is consumed by the reference.

> This remains a state-local differential oracle: no policy, integration, contact simulation, or rigid-body rollout. It validates r63 equations and optimizer semantics, not calibrated actuator response or stability.

## Differential matrix

| scenario | acceleration RMS / max | contact-force RMS / max | Pin dynamics max | Pin contact max | NumPy p99 |
|---|---|---|---|---|---|
| ideal_control | 9.786e-11 / 1.258e-09 | 5.072e-09 / 1.301e-07 | 1.021e-09 | 4.360e-10 | 19547.3 µs |
| fast_synthetic | 9.875e-11 / 1.242e-09 | 1.169e-08 / 1.860e-07 | 1.672e-09 | 4.373e-10 | 11808.8 µs |
| slow_synthetic | 1.030e-10 / 1.235e-09 | 2.429e-08 / 3.046e-07 | 6.292e-09 | 4.377e-10 | 10704.9 µs |
| quarter_available_slow | 1.028e-10 / 1.232e-09 | 2.561e-08 / 3.047e-07 | 6.244e-09 | 4.372e-10 | 14881.2 µs |

## Interpretation

- Pinocchio independently owns inertial products and point Jacobians; NumPy independently owns equality seeding, damped task pseudoinverses, and nullspace freezing.
- The sample set includes every contact edge neighborhood and additional evenly distributed/adverse-pressure states. It is a differential sentinel, not a replacement for r63's complete 2,317-state curve.
- Dynamics/contact residuals evaluate the Rust solution directly under Pinocchio products. Solution deltas then test the independent lexicographic optimizer, so product and optimization disagreement cannot hide in one number.
- Acceleration and contact-force comparisons remain separate because their units and operational meaning differ.

## Gates

- PASS `source_artifacts_remain_immutable`
- PASS `pinocchio_products_validate_every_rust_dynamics_row_below_2e-8`
- PASS `pinocchio_products_validate_every_rust_contact_row_below_2e-8`
- PASS `numpy_lexicographic_acceleration_matches_rust_within_2e-5`
- PASS `numpy_lexicographic_contact_force_matches_rust_within_2e-4_n`
- PASS `numpy_reference_repeats_bit_exactly_on_two_states_per_scenario`

## Artifacts

The raw NPZ retains every selected-state acceleration/contact-force/complete-solution delta, direct Pinocchio hard residual, and reference product/solve timing. The JSON retains selected ticks, per-scenario distributions, evaluation boundaries, and source checksums. Markdown and public HTML are human-readable mirrors.
