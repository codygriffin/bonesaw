# Bonesaw G1 positive-reference compliance audit · r226

> Generic mechanisms **PASS** · viable solver/profile constructions **6** · profile/authority **NOT PROMOTED** · physics/integration/policy/controller steps **0 / 0 / 0 / 0**.

## Contract

- Rust implements the documented positive time-constant/damping-ratio law from [MuJoCo solver parameters](https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters): `K=1/(dmax² τ² ζ²)`, `B=2/(dmax τ)`, the complete position-dependent impedance spline, the explicit `τ≥2·physics_dt` refsafe clamp, first-order tangent decay, and circular/pyramidal sections. These are reference equations, not fitted R221 coefficients.
- The new input types causal free point acceleration separately from contact velocity. This audit obtains that witness through 96 prestate `mj_forward` plus `J qacc_smooth + Jdot qvel` reference queries. It takes no reference step, policy, controller step, or state integration; online authority would have to obtain the same witness from the admitted WBC/model path.
- The construction grid compares independent effective-normal-mass response with complete Delassus distribution using 32 fixed forward/reverse projected sweeps. It tests 5, 8, 16, 32, 64, and 128 fixed microsteps both with and without the free-acceleration witness. The authored global integrator selects an explicit-character reduced scheme, but neither model reproduces MuJoCo's generalized RK4 or implicitfast step. R220's 16× cap and frozen residual box are retained only as comparators because R221 labels are no longer untouched. A row could advance only if both laws have strict frozen-box coverage, fitted 2.0/0.5/10.0 widths, repeat, allocation, and deadline together; no result is authority.
- Every timed query repeats bitwise and allocates no Rust heap memory. A complete retained rerun reproduced all 192 non-timing NPZ arrays exactly; timing is reported but excluded from semantic equality.

## Construction audit

| solver | profile | law | coverage in R220 box | residual p95 ω/v/joint | fitted width ω/v/joint | query p99 µs | gate |
|---|---|---|---|---|---|---|---|
| diagonal_effective_mass | reference_steps5_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.055 / 0.016 / 3.514 | 0.158 / 0.053 / 9.377 | 4.001 | candidate |
| diagonal_effective_mass | reference_steps5_zero_accel | hard_pyramidal_rk4 | 66.667% | 1.137 / 0.191 / 35.006 | 2.709 / 0.491 / 256.618 | 4.842 | reject |
| diagonal_effective_mass | reference_steps5_free_accel | mid_elliptic_implicitfast | 100.000% | 0.050 / 0.014 / 3.650 | 0.136 / 0.048 / 8.712 | 3.729 | candidate |
| diagonal_effective_mass | reference_steps5_free_accel | hard_pyramidal_rk4 | 62.500% | 1.115 / 0.184 / 37.708 | 2.676 / 0.486 / 270.265 | 3.886 | reject |
| diagonal_effective_mass | reference_steps8_zero_accel | mid_elliptic_implicitfast | 97.917% | 0.058 / 0.016 / 3.173 | 0.133 / 0.054 / 10.254 | 5.707 | reject |
| diagonal_effective_mass | reference_steps8_zero_accel | hard_pyramidal_rk4 | 72.917% | 1.234 / 0.211 / 11.274 | 2.690 / 0.491 / 57.748 | 6.753 | reject |
| diagonal_effective_mass | reference_steps8_free_accel | mid_elliptic_implicitfast | 100.000% | 0.053 / 0.014 / 3.065 | 0.130 / 0.050 / 9.662 | 5.566 | candidate |
| diagonal_effective_mass | reference_steps8_free_accel | hard_pyramidal_rk4 | 68.750% | 1.212 / 0.208 / 18.811 | 2.649 / 0.477 / 61.675 | 6.796 | reject |
| diagonal_effective_mass | reference_steps16_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.059 / 0.017 / 3.462 | 0.138 / 0.056 / 9.415 | 9.390 | candidate |
| diagonal_effective_mass | reference_steps16_zero_accel | hard_pyramidal_rk4 | 70.833% | 1.270 / 0.214 / 7.123 | 2.803 / 0.506 / 16.682 | 15.106 | reject |
| diagonal_effective_mass | reference_steps16_free_accel | mid_elliptic_implicitfast | 100.000% | 0.055 / 0.015 / 2.963 | 0.135 / 0.052 / 8.663 | 11.115 | candidate |
| diagonal_effective_mass | reference_steps16_free_accel | hard_pyramidal_rk4 | 75.000% | 1.251 / 0.211 / 7.499 | 2.776 / 0.494 / 17.644 | 25.852 | reject |
| diagonal_effective_mass | reference_steps32_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.059 / 0.017 / 3.621 | 0.135 / 0.058 / 9.001 | 17.083 | candidate |
| diagonal_effective_mass | reference_steps32_zero_accel | hard_pyramidal_rk4 | 70.833% | 1.282 / 0.215 / 7.085 | 2.839 / 0.511 / 17.182 | 25.445 | reject |
| diagonal_effective_mass | reference_steps32_free_accel | mid_elliptic_implicitfast | 100.000% | 0.055 / 0.015 / 3.120 | 0.136 / 0.053 / 9.099 | 17.044 | candidate |
| diagonal_effective_mass | reference_steps32_free_accel | hard_pyramidal_rk4 | 75.000% | 1.264 / 0.212 / 7.009 | 2.805 / 0.499 / 15.810 | 19.606 | reject |
| diagonal_effective_mass | reference_steps64_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.060 / 0.017 / 3.708 | 0.136 / 0.058 / 9.042 | 45.346 | candidate |
| diagonal_effective_mass | reference_steps64_zero_accel | hard_pyramidal_rk4 | 70.833% | 1.291 / 0.216 / 7.449 | 2.860 / 0.517 / 17.742 | 38.906 | reject |
| diagonal_effective_mass | reference_steps64_free_accel | mid_elliptic_implicitfast | 100.000% | 0.056 / 0.016 / 3.171 | 0.138 / 0.054 / 8.671 | 30.734 | candidate |
| diagonal_effective_mass | reference_steps64_free_accel | hard_pyramidal_rk4 | 75.000% | 1.267 / 0.212 / 6.294 | 2.813 / 0.501 / 16.071 | 39.420 | reject |
| diagonal_effective_mass | reference_steps128_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.060 / 0.018 / 3.796 | 0.136 / 0.059 / 8.943 | 64.180 | candidate |
| diagonal_effective_mass | reference_steps128_zero_accel | hard_pyramidal_rk4 | 70.833% | 1.293 / 0.216 / 7.676 | 2.868 / 0.518 / 18.066 | 83.676 | reject |
| diagonal_effective_mass | reference_steps128_free_accel | mid_elliptic_implicitfast | 100.000% | 0.056 / 0.016 / 3.193 | 0.137 / 0.054 / 8.686 | 63.549 | candidate |
| diagonal_effective_mass | reference_steps128_free_accel | hard_pyramidal_rk4 | 72.917% | 1.272 / 0.213 / 6.449 | 2.818 / 0.503 / 16.369 | 73.838 | reject |
| coupled_delassus | reference_steps5_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.036 / 0.009 / 2.829 | 0.149 / 0.032 / 6.204 | 80.266 | candidate |
| coupled_delassus | reference_steps5_zero_accel | hard_pyramidal_rk4 | 72.917% | 0.192 / 0.044 / 20.576 | 1.702 / 0.177 / 62.997 | 57.660 | reject |
| coupled_delassus | reference_steps5_free_accel | mid_elliptic_implicitfast | 100.000% | 0.044 / 0.016 / 3.247 | 0.100 / 0.040 / 7.968 | 79.924 | candidate |
| coupled_delassus | reference_steps5_free_accel | hard_pyramidal_rk4 | 70.833% | 0.200 / 0.042 / 26.896 | 1.988 / 0.213 / 68.919 | 57.474 | reject |
| coupled_delassus | reference_steps8_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.026 / 0.006 / 1.805 | 0.091 / 0.033 / 6.687 | 124.664 | candidate |
| coupled_delassus | reference_steps8_zero_accel | hard_pyramidal_rk4 | 87.500% | 0.208 / 0.042 / 7.194 | 1.542 / 0.214 / 36.604 | 142.401 | reject |
| coupled_delassus | reference_steps8_free_accel | mid_elliptic_implicitfast | 100.000% | 0.033 / 0.013 / 2.368 | 0.085 / 0.042 / 6.671 | 125.880 | candidate |
| coupled_delassus | reference_steps8_free_accel | hard_pyramidal_rk4 | 81.250% | 0.399 / 0.076 / 9.032 | 1.811 / 0.270 / 26.133 | 164.023 | reject |
| coupled_delassus | reference_steps16_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.004 / 1.863 | 0.101 / 0.023 / 6.421 | 237.429 | candidate |
| coupled_delassus | reference_steps16_zero_accel | hard_pyramidal_rk4 | 97.917% | 0.100 / 0.014 / 3.840 | 0.730 / 0.103 / 17.555 | 289.436 | reject |
| coupled_delassus | reference_steps16_free_accel | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.100 | 0.094 / 0.035 / 6.289 | 239.262 | candidate |
| coupled_delassus | reference_steps16_free_accel | hard_pyramidal_rk4 | 93.750% | 0.176 / 0.036 / 4.967 | 0.912 / 0.129 / 16.053 | 318.276 | reject |
| coupled_delassus | reference_steps32_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.004 / 1.909 | 0.096 / 0.019 / 6.326 | 451.779 | candidate |
| coupled_delassus | reference_steps32_zero_accel | hard_pyramidal_rk4 | 100.000% | 0.081 / 0.011 / 3.702 | 0.543 / 0.104 / 8.556 | 603.565 | candidate |
| coupled_delassus | reference_steps32_free_accel | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.008 / 1.729 | 0.095 / 0.030 / 6.131 | 462.837 | candidate |
| coupled_delassus | reference_steps32_free_accel | hard_pyramidal_rk4 | 100.000% | 0.069 / 0.013 / 3.538 | 0.449 / 0.065 / 9.320 | 634.373 | candidate |
| coupled_delassus | reference_steps64_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.004 / 2.091 | 0.099 / 0.018 / 6.373 | 901.986 | candidate |
| coupled_delassus | reference_steps64_zero_accel | hard_pyramidal_rk4 | 100.000% | 0.138 / 0.016 / 3.996 | 0.724 / 0.104 / 9.047 | 1208.206 | candidate |
| coupled_delassus | reference_steps64_free_accel | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.682 | 0.104 / 0.027 / 5.757 | 900.592 | candidate |
| coupled_delassus | reference_steps64_free_accel | hard_pyramidal_rk4 | 100.000% | 0.093 / 0.011 / 2.913 | 0.437 / 0.062 / 8.669 | 1255.009 | candidate |
| coupled_delassus | reference_steps128_zero_accel | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.004 / 2.129 | 0.098 / 0.017 / 6.347 | 1779.182 | candidate |
| coupled_delassus | reference_steps128_zero_accel | hard_pyramidal_rk4 | 100.000% | 0.152 / 0.019 / 4.440 | 0.835 / 0.120 / 9.564 | 2417.022 | candidate |
| coupled_delassus | reference_steps128_free_accel | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.619 | 0.101 / 0.027 / 5.776 | 1830.967 | candidate |
| coupled_delassus | reference_steps128_free_accel | hard_pyramidal_rk4 | 100.000% | 0.084 / 0.013 / 3.084 | 0.416 / 0.049 / 8.650 | 2535.492 | candidate |

## Decision

Retain the typed positive-reference law, causal free-acceleration boundary, and complete-Delassus projected distribution. The audit removes the known constant-impedance/reference-scaling error and directly tests cross-contact coupling, but no solver/microstep/free-acceleration row passes both laws and useful fitted width. Therefore freeze nothing and do not spend a fresh holdout. The remaining gap is fidelity to the reference optimizer's soft-constraint regularization and contact formulation, not another scalar stiffness, microstep, or residual-width sweep.
