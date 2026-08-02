# Bonesaw Upkie contact-transition replay · r124

## Outcome

> **Admission: PASS.** 20,000 immutable Upkie states exercise every point-contact mode, single/double/no support, mixed touchdown rows, and bounded rolling slip. There is no policy, contact estimator, state integration, simulator, or physics rollout.

## Boundary and corpus

| item | value |
|---|---|
| states | 20,000 |
| seed | 0xb0e5a7 |
| model SHA-256 | d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483 |
| mode codes | {0: 'LockedPoint', 1: 'NormalPoint', 2: 'RollingPoint', 3: 'RollingWheel'} |
| contact transitions | 288 |
| queries touching ±250 qdd cap | 10,712 / 20,000 (53.56%) |
| input fingerprint preserved | True |
| policy / estimator / integration / physics | none / none / none / none |

`contact_mode_trace[tick,target]` selects LockedPoint, NormalPoint, RollingPoint, or RollingWheel before the preallocated Rust tick. Contact activity remains a separate mask. Descriptors are validated for the entire call before any output is written.

## Independent Pinocchio hard equations

| witness | worst observed | gate |
|---|---|---|
| floating dynamics L∞ | 3.016e-11 | ≤ 2e-6 |
| mode-specific contact L∞ | 6.133e-12 | ≤ 2e-6 |
| rolling velocity witness L∞ | 3.331e-16 m/s | ≤ 2e-10 |
| minimum normal force | 22.015021 N | ≥ -1e-8 |
| minimum friction margin | -1.066e-14 N | ≥ -2e-7 |
| minimum effort margin | 1.612e+00 Nm | ≥ -2e-7 |
| inactive packed-force tail | 0.000e+00 | exact zero |

For each active target the oracle selects only the rows declared by its mode: XYZ for locked, Z for normal, YZ for free rolling, and wheel-coupled X plus YZ for RollingWheel. Contact forces are independently packed in active-target order before reconstructing M·qdd+h−Sᵀτ−Jᵀf.

## Behavior by contact regime

| regime | queries | statuses | tracking RMS | dyn L∞ | contact L∞ | p50 / p99 / max µs | qdd cap |
|---|---|---|---|---|---|---|---|
| double_rolling | 1942 | {'Solved': 113, 'SolvedWithSlack': 1829} | 88.459 | 2.32e-11 | 6.13e-12 | 128.1 / 196.9 / 214.3 | 920 |
| left_rolling | 1960 | {'SolvedWithSlack': 1960} | 104.890 | 2.12e-11 | 2.12e-13 | 93.8 / 136.7 / 170.3 | 1776 |
| right_rolling | 2006 | {'SolvedWithSlack': 2006} | 103.717 | 2.06e-11 | 1.95e-13 | 93.1 / 139.1 / 172.7 | 1803 |
| double_normal | 2123 | {'Solved': 1053, 'SolvedWithSlack': 1070} | 50.169 | 3.02e-11 | 3.21e-12 | 99.7 / 140.4 / 180.8 | 317 |
| double_free_rolling | 2101 | {'Solved': 897, 'SolvedWithSlack': 1204} | 53.015 | 3.01e-11 | 2.86e-12 | 112.9 / 159.5 / 201.2 | 356 |
| double_locked | 1962 | {'SolvedWithSlack': 1962} | 101.326 | 2.15e-11 | 5.62e-12 | 124.7 / 181.7 / 220.4 | 1881 |
| airborne | 2009 | {'SolvedWithSlack': 2009} | 135.551 | 4.95e-12 | 0.00e+00 | 67.7 / 120.8 / 207.3 | 1812 |
| left_roll_right_normal | 1883 | {'Solved': 406, 'SolvedWithSlack': 1477} | 56.950 | 2.39e-11 | 2.93e-12 | 112.0 / 154.1 / 196.4 | 471 |
| left_normal_right_roll | 2025 | {'Solved': 454, 'SolvedWithSlack': 1571} | 57.662 | 2.48e-11 | 2.85e-12 | 110.0 / 150.6 / 191.9 | 533 |
| double_rolling_slip | 1989 | {'Solved': 86, 'SolvedWithSlack': 1903} | 90.460 | 2.27e-11 | 6.09e-12 | 127.9 / 175.9 / 213.2 | 843 |

Tracking remains a continuous lower-layer witness. Exact hard equations do not claim that the requested posture or acceleration is realizable.

## Transition-edge windows

| check | result |
|---|---|
| transition_count | 288 |
| queries_in_plus_minus_two_window | 1440 |
| status_counts | {'Solved': 224, 'SolvedWithSlack': 1216} |
| dynamics_linf | 2.916600294611271e-11 |
| contact_linf | 5.222378085534274e-12 |
| maximum_constraint_violation | 2.916600294611271e-11 |

Each ±2-row edge window is checked independently; there is intentionally no contact state carried across the edge.

## Long-run CPU, memory, jitter, and work

| measurement | result |
|---|---|
| latency mean / std / MAD | 109.9 / 21.7 / 14.3 µs |
| latency p50 / p95 / p99 / p99.9 / max | 110.4 / 140.7 / 172.9 / 202.6 / 220.4 µs |
| adjacent jitter p50 / p95 / p99 / max | 3.7 / 15.0 / 32.2 / 115.1 µs |
| deadline misses >0.5 / 1 / 2 / 5 ms | 0 / 0 / 0 / 0 |
| whole call wall / CPU | 2.199 / 2.199 s |
| throughput | 9094.7 queries/s |
| RSS before / after / delta | 106.08 / 106.08 / 0.00 MiB |
| hot-loop allocation sentinel | 0 calls / 0 bytes |
| GC collections | 0 |

## Replay invariance

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| reverse_order_exact | True |
| four_chunk_exact | True |
| allocation_calls | 0 |

## Invalid-input fault probes

| fault | typed ValueError | all outputs unchanged |
|---|---|---|
| nonfinite_state | True | True |
| invalid_contact_mode | True | True |
| invalid_contact_activity | True | True |
| zero_rolling_coefficient | True | True |

Nonfinite state, invalid mode/activity, and degenerate rolling coefficients are rejected atomically before the first measured tick. Runtime infeasibility remains a different typed solver status; schema faults are not mislabeled as physics.

## Remaining boundary

This report admits row composition and bounded state-local execution across contact regimes. It does not choose those regimes, estimate touchdown, model impact impulses, or prove hybrid closed-loop stability. The next CPU tranche is a long adversarial resource/observation replay over this same mode boundary, followed by a separately versioned contact estimator or external simulator. CUDA batching remains downstream of this exact CPU reference.
