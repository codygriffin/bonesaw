# Bonesaw G1 coupled positive-reference compliance audit · r227

> Generic coupled mechanism **PASS** · viable construction profiles **12** · profile/authority **NOT PROMOTED** · physics/integration/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0 / 0**.

## Contract

- Rust converts the documented positive-reference acceleration into a desired contact-space velocity increment, then distributes it through the complete state-local Delassus operator with fixed forward/reverse projected sweeps. Effective mass is no longer an independent per-contact input.
- Every microstep enforces cumulative per-axis caps, nonnegative normal impulse, and the declared circular/pyramidal friction section. The solver is bounded-work and allocation-free, but is not MuJoCo's generalized nonlinear constraint optimizer or an outer bound.
- Impulse caps are label-free momentum limits: total model mass times maximum causal closing speed plus one control interval of gravity, with tangent capacity from the authored friction coefficient. The audit reuses rejected immutable R221 labels and R220's residual box only as a construction comparator. It makes 96 prestate forward-dynamics queries and takes no physics integration, policy, controller, selector, or plant-action step. No row can enter authority from this audit.
- Every query repeats bitwise and allocates no Rust heap memory. A complete retained rerun reproduced all 192 non-timing NPZ arrays exactly; timing is reported but excluded from semantic equality.

## Construction grid

| profile | law | coverage in R220 box | residual p95 ω/v/joint | fitted width ω/v/joint | query p99 µs | gate |
|---|---|---|---|---|---|---|
| coupled_steps16_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.032 / 0.009 / 2.919 | 0.126 / 0.023 / 8.283 | 17.114 | candidate |
| coupled_steps16_sweeps1 | hard_pyramidal_rk4 | 91.667% | 0.949 / 0.159 / 4.867 | 2.341 / 0.412 / 11.437 | 20.183 | reject |
| coupled_steps16_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.027 / 0.004 / 1.912 | 0.114 / 0.023 / 5.542 | 24.189 | candidate |
| coupled_steps16_sweeps2 | hard_pyramidal_rk4 | 91.667% | 0.624 / 0.092 / 4.915 | 1.768 / 0.307 / 16.504 | 30.129 | reject |
| coupled_steps16_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.008 / 2.011 | 0.105 / 0.032 / 5.884 | 37.732 | candidate |
| coupled_steps16_sweeps4 | hard_pyramidal_rk4 | 93.750% | 0.341 / 0.037 / 4.962 | 1.034 / 0.171 / 16.136 | 49.525 | reject |
| coupled_steps16_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.010 / 2.092 | 0.098 / 0.034 / 6.254 | 68.455 | candidate |
| coupled_steps16_sweeps8 | hard_pyramidal_rk4 | 93.750% | 0.130 / 0.015 / 5.085 | 0.649 / 0.083 / 16.176 | 86.567 | reject |
| coupled_steps16_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.010 / 2.097 | 0.095 / 0.034 / 6.281 | 124.248 | candidate |
| coupled_steps16_sweeps16 | hard_pyramidal_rk4 | 93.750% | 0.130 / 0.022 / 4.997 | 0.835 / 0.115 / 16.128 | 162.448 | reject |
| coupled_steps16_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.100 | 0.094 / 0.035 / 6.289 | 238.009 | candidate |
| coupled_steps16_sweeps32 | hard_pyramidal_rk4 | 93.750% | 0.176 / 0.036 / 4.967 | 0.912 / 0.129 / 16.053 | 322.910 | reject |
| coupled_steps16_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.102 | 0.094 / 0.035 / 6.297 | 460.111 | candidate |
| coupled_steps16_sweeps64 | hard_pyramidal_rk4 | 93.750% | 0.180 / 0.038 / 4.944 | 0.918 / 0.130 / 15.972 | 616.231 | reject |
| coupled_steps16_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.103 | 0.093 / 0.035 / 6.301 | 906.586 | candidate |
| coupled_steps16_sweeps128 | hard_pyramidal_rk4 | 93.750% | 0.180 / 0.038 / 4.944 | 0.918 / 0.130 / 15.926 | 1237.898 | reject |
| coupled_steps32_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.034 / 0.009 / 3.039 | 0.128 / 0.026 / 8.517 | 29.065 | candidate |
| coupled_steps32_sweeps1 | hard_pyramidal_rk4 | 91.667% | 0.984 / 0.164 / 5.291 | 2.394 / 0.422 / 12.713 | 39.654 | reject |
| coupled_steps32_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.025 / 0.004 / 2.026 | 0.115 / 0.019 / 6.166 | 43.617 | candidate |
| coupled_steps32_sweeps2 | hard_pyramidal_rk4 | 95.833% | 0.647 / 0.097 / 4.510 | 1.838 / 0.320 / 10.131 | 59.237 | reject |
| coupled_steps32_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.025 / 0.006 / 1.737 | 0.107 / 0.027 / 6.084 | 71.805 | candidate |
| coupled_steps32_sweeps4 | hard_pyramidal_rk4 | 100.000% | 0.333 / 0.045 / 3.158 | 1.165 / 0.191 / 9.529 | 97.609 | candidate |
| coupled_steps32_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.008 / 1.729 | 0.099 / 0.030 / 6.077 | 131.621 | candidate |
| coupled_steps32_sweeps8 | hard_pyramidal_rk4 | 100.000% | 0.133 / 0.020 / 3.414 | 0.609 / 0.088 / 9.514 | 172.211 | candidate |
| coupled_steps32_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.008 / 1.731 | 0.096 / 0.030 / 6.097 | 239.094 | candidate |
| coupled_steps32_sweeps16 | hard_pyramidal_rk4 | 100.000% | 0.064 / 0.010 / 2.698 | 0.201 / 0.024 / 9.433 | 322.908 | candidate |
| coupled_steps32_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.008 / 1.729 | 0.095 / 0.030 / 6.131 | 455.333 | candidate |
| coupled_steps32_sweeps32 | hard_pyramidal_rk4 | 100.000% | 0.069 / 0.013 / 3.538 | 0.449 / 0.065 / 9.320 | 637.006 | candidate |
| coupled_steps32_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.008 / 1.720 | 0.094 / 0.030 / 6.181 | 892.559 | candidate |
| coupled_steps32_sweeps64 | hard_pyramidal_rk4 | 100.000% | 0.072 / 0.013 / 3.579 | 0.462 / 0.068 / 9.207 | 1241.996 | candidate |
| coupled_steps32_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.008 / 1.714 | 0.094 / 0.030 / 6.242 | 1761.661 | candidate |
| coupled_steps32_sweeps128 | hard_pyramidal_rk4 | 100.000% | 0.072 / 0.013 / 3.575 | 0.461 / 0.068 / 9.145 | 2442.644 | candidate |
| coupled_steps64_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.034 / 0.009 / 3.089 | 0.130 / 0.027 / 8.841 | 56.016 | candidate |
| coupled_steps64_sweeps1 | hard_pyramidal_rk4 | 91.667% | 1.002 / 0.166 / 5.487 | 2.416 / 0.425 / 14.366 | 75.885 | reject |
| coupled_steps64_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.004 / 2.005 | 0.121 / 0.018 / 6.010 | 86.041 | candidate |
| coupled_steps64_sweeps2 | hard_pyramidal_rk4 | 95.833% | 0.666 / 0.100 / 4.415 | 1.891 / 0.328 / 12.242 | 114.325 | reject |
| coupled_steps64_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.005 / 1.691 | 0.115 / 0.025 / 5.723 | 140.070 | candidate |
| coupled_steps64_sweeps4 | hard_pyramidal_rk4 | 100.000% | 0.348 / 0.048 / 3.792 | 1.341 / 0.212 / 9.620 | 190.396 | candidate |
| coupled_steps64_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.681 | 0.108 / 0.027 / 5.708 | 253.569 | candidate |
| coupled_steps64_sweeps8 | hard_pyramidal_rk4 | 100.000% | 0.141 / 0.021 / 3.135 | 0.762 / 0.111 / 8.313 | 341.078 | candidate |
| coupled_steps64_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.007 / 1.683 | 0.105 / 0.027 / 5.727 | 474.353 | candidate |
| coupled_steps64_sweeps16 | hard_pyramidal_rk4 | 100.000% | 0.076 / 0.013 / 2.935 | 0.385 / 0.051 / 8.560 | 637.234 | candidate |
| coupled_steps64_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.682 | 0.104 / 0.027 / 5.757 | 895.313 | candidate |
| coupled_steps64_sweeps32 | hard_pyramidal_rk4 | 100.000% | 0.093 / 0.011 / 2.913 | 0.437 / 0.062 / 8.669 | 1247.821 | candidate |
| coupled_steps64_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.020 / 0.007 / 1.673 | 0.103 / 0.027 / 5.803 | 1741.536 | candidate |
| coupled_steps64_sweeps64 | hard_pyramidal_rk4 | 100.000% | 0.095 / 0.012 / 2.890 | 0.447 / 0.065 / 8.674 | 2449.426 | candidate |
| coupled_steps64_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.020 / 0.007 / 1.667 | 0.103 / 0.027 / 5.863 | 3453.143 | candidate |
| coupled_steps64_sweeps128 | hard_pyramidal_rk4 | 100.000% | 0.096 / 0.012 / 2.878 | 0.446 / 0.065 / 8.645 | 4799.120 | candidate |

## Decision

The label-diagnostic row `coupled_steps64_sweeps8` reaches 100.000% mid-law and 100.000% hard-law sample coverage, but it is not used for selection. The causal convergence rule selects `coupled_steps32_sweeps32` by comparing only predicted generalized velocity: doubling sweeps changes at most 1.200% of the corresponding useful-width gate, doubling substeps changes at most 16.858%, and retained p99 remains below 5 ms. That independently selected row also covers 100.000%/100.000% at useful width on rejected labels. Freeze this construction profile before generating a new law/state holdout; authority remains absent.
