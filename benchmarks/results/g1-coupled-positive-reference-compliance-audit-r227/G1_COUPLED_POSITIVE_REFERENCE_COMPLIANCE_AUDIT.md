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
| coupled_steps16_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.032 / 0.009 / 2.919 | 0.126 / 0.023 / 8.283 | 19.617 | candidate |
| coupled_steps16_sweeps1 | hard_pyramidal_rk4 | 91.667% | 0.949 / 0.159 / 4.867 | 2.341 / 0.412 / 11.437 | 21.052 | reject |
| coupled_steps16_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.027 / 0.004 / 1.912 | 0.114 / 0.023 / 5.542 | 22.910 | candidate |
| coupled_steps16_sweeps2 | hard_pyramidal_rk4 | 91.667% | 0.624 / 0.092 / 4.915 | 1.768 / 0.307 / 16.504 | 30.410 | reject |
| coupled_steps16_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.008 / 2.011 | 0.105 / 0.032 / 5.884 | 49.081 | candidate |
| coupled_steps16_sweeps4 | hard_pyramidal_rk4 | 93.750% | 0.341 / 0.037 / 4.962 | 1.034 / 0.171 / 16.136 | 49.680 | reject |
| coupled_steps16_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.010 / 2.092 | 0.098 / 0.034 / 6.254 | 68.371 | candidate |
| coupled_steps16_sweeps8 | hard_pyramidal_rk4 | 93.750% | 0.130 / 0.015 / 5.085 | 0.649 / 0.083 / 16.176 | 86.973 | reject |
| coupled_steps16_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.030 / 0.010 / 2.097 | 0.095 / 0.034 / 6.281 | 128.981 | candidate |
| coupled_steps16_sweeps16 | hard_pyramidal_rk4 | 93.750% | 0.130 / 0.022 / 4.997 | 0.835 / 0.115 / 16.128 | 170.777 | reject |
| coupled_steps16_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.100 | 0.094 / 0.035 / 6.289 | 242.854 | candidate |
| coupled_steps16_sweeps32 | hard_pyramidal_rk4 | 93.750% | 0.176 / 0.036 / 4.967 | 0.912 / 0.129 / 16.053 | 319.797 | reject |
| coupled_steps16_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.102 | 0.094 / 0.035 / 6.297 | 464.831 | candidate |
| coupled_steps16_sweeps64 | hard_pyramidal_rk4 | 93.750% | 0.180 / 0.038 / 4.944 | 0.918 / 0.130 / 15.972 | 623.567 | reject |
| coupled_steps16_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.029 / 0.010 / 2.103 | 0.093 / 0.035 / 6.301 | 909.021 | candidate |
| coupled_steps16_sweeps128 | hard_pyramidal_rk4 | 93.750% | 0.180 / 0.038 / 4.944 | 0.918 / 0.130 / 15.926 | 1218.512 | reject |
| coupled_steps32_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.034 / 0.009 / 3.039 | 0.128 / 0.026 / 8.517 | 32.595 | candidate |
| coupled_steps32_sweeps1 | hard_pyramidal_rk4 | 91.667% | 0.984 / 0.164 / 5.291 | 2.394 / 0.422 / 12.713 | 38.706 | reject |
| coupled_steps32_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.025 / 0.004 / 2.026 | 0.115 / 0.019 / 6.166 | 42.527 | candidate |
| coupled_steps32_sweeps2 | hard_pyramidal_rk4 | 95.833% | 0.647 / 0.097 / 4.510 | 1.838 / 0.320 / 10.131 | 58.448 | reject |
| coupled_steps32_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.025 / 0.006 / 1.737 | 0.107 / 0.027 / 6.084 | 72.177 | candidate |
| coupled_steps32_sweeps4 | hard_pyramidal_rk4 | 100.000% | 0.333 / 0.045 / 3.158 | 1.165 / 0.191 / 9.529 | 95.281 | candidate |
| coupled_steps32_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.008 / 1.729 | 0.099 / 0.030 / 6.077 | 129.669 | candidate |
| coupled_steps32_sweeps8 | hard_pyramidal_rk4 | 100.000% | 0.133 / 0.020 / 3.414 | 0.609 / 0.088 / 9.514 | 172.420 | candidate |
| coupled_steps32_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.008 / 1.731 | 0.096 / 0.030 / 6.097 | 238.384 | candidate |
| coupled_steps32_sweeps16 | hard_pyramidal_rk4 | 100.000% | 0.064 / 0.010 / 2.698 | 0.201 / 0.024 / 9.433 | 322.415 | candidate |
| coupled_steps32_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.008 / 1.729 | 0.095 / 0.030 / 6.131 | 460.285 | candidate |
| coupled_steps32_sweeps32 | hard_pyramidal_rk4 | 100.000% | 0.069 / 0.013 / 3.538 | 0.449 / 0.065 / 9.320 | 630.887 | candidate |
| coupled_steps32_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.008 / 1.720 | 0.094 / 0.030 / 6.181 | 896.701 | candidate |
| coupled_steps32_sweeps64 | hard_pyramidal_rk4 | 100.000% | 0.072 / 0.013 / 3.579 | 0.462 / 0.068 / 9.207 | 1240.481 | candidate |
| coupled_steps32_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.008 / 1.714 | 0.094 / 0.030 / 6.242 | 1801.865 | candidate |
| coupled_steps32_sweeps128 | hard_pyramidal_rk4 | 100.000% | 0.072 / 0.013 / 3.575 | 0.461 / 0.068 / 9.145 | 2434.676 | candidate |
| coupled_steps64_sweeps1 | mid_elliptic_implicitfast | 100.000% | 0.034 / 0.009 / 3.089 | 0.130 / 0.027 / 8.841 | 58.475 | candidate |
| coupled_steps64_sweeps1 | hard_pyramidal_rk4 | 91.667% | 1.002 / 0.166 / 5.487 | 2.416 / 0.425 / 14.366 | 75.326 | reject |
| coupled_steps64_sweeps2 | mid_elliptic_implicitfast | 100.000% | 0.024 / 0.004 / 2.005 | 0.121 / 0.018 / 6.010 | 85.796 | candidate |
| coupled_steps64_sweeps2 | hard_pyramidal_rk4 | 95.833% | 0.666 / 0.100 / 4.415 | 1.891 / 0.328 / 12.242 | 115.053 | reject |
| coupled_steps64_sweeps4 | mid_elliptic_implicitfast | 100.000% | 0.023 / 0.005 / 1.691 | 0.115 / 0.025 / 5.723 | 139.943 | candidate |
| coupled_steps64_sweeps4 | hard_pyramidal_rk4 | 100.000% | 0.348 / 0.048 / 3.792 | 1.341 / 0.212 / 9.620 | 190.869 | candidate |
| coupled_steps64_sweeps8 | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.681 | 0.108 / 0.027 / 5.708 | 259.523 | candidate |
| coupled_steps64_sweeps8 | hard_pyramidal_rk4 | 100.000% | 0.141 / 0.021 / 3.135 | 0.762 / 0.111 / 8.313 | 337.488 | candidate |
| coupled_steps64_sweeps16 | mid_elliptic_implicitfast | 100.000% | 0.022 / 0.007 / 1.683 | 0.105 / 0.027 / 5.727 | 471.874 | candidate |
| coupled_steps64_sweeps16 | hard_pyramidal_rk4 | 100.000% | 0.076 / 0.013 / 2.935 | 0.385 / 0.051 / 8.560 | 636.929 | candidate |
| coupled_steps64_sweeps32 | mid_elliptic_implicitfast | 100.000% | 0.021 / 0.007 / 1.682 | 0.104 / 0.027 / 5.757 | 895.507 | candidate |
| coupled_steps64_sweeps32 | hard_pyramidal_rk4 | 100.000% | 0.093 / 0.011 / 2.913 | 0.437 / 0.062 / 8.669 | 1252.844 | candidate |
| coupled_steps64_sweeps64 | mid_elliptic_implicitfast | 100.000% | 0.020 / 0.007 / 1.673 | 0.103 / 0.027 / 5.803 | 1748.207 | candidate |
| coupled_steps64_sweeps64 | hard_pyramidal_rk4 | 100.000% | 0.095 / 0.012 / 2.890 | 0.447 / 0.065 / 8.674 | 2512.657 | candidate |
| coupled_steps64_sweeps128 | mid_elliptic_implicitfast | 100.000% | 0.020 / 0.007 / 1.667 | 0.103 / 0.027 / 5.863 | 3418.656 | candidate |
| coupled_steps64_sweeps128 | hard_pyramidal_rk4 | 100.000% | 0.096 / 0.012 / 2.878 | 0.446 / 0.065 / 8.645 | 4806.029 | candidate |

## Decision

The label-diagnostic row `coupled_steps64_sweeps8` reaches 100.000% mid-law and 100.000% hard-law sample coverage, but it is not used for selection. The causal convergence rule selects `coupled_steps32_sweeps32` by comparing only predicted generalized velocity: doubling sweeps changes at most 1.200% of the corresponding useful-width gate, doubling substeps changes at most 16.858%, and retained p99 remains below 5 ms. That independently selected row also covers 100.000%/100.000% at useful width on rejected labels. Freeze this construction profile before generating a new law/state holdout; authority remains absent.
