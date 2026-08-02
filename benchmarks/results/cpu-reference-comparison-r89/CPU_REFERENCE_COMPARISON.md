# Bonesaw CPU reference comparison · r89

## Executive result

The CPU concept is demonstrated at three deliberately separate boundaries. The current G1 stateless oracle passes all 43/43 uncoupled and 45/45 synthetic coupled-actuation gates over 2,317 ticks, four alternating steps, and eight contact edges. Pinocchio independently validates the rigid-body products to floating-point roundoff. The official-aligned Rust rolling law is bitwise identical to the pinned upstream Upkie C++ implementation over 100,000 sequential samples. PlaCo remains an independent fixed-base task-level comparator on identical targets, not a falsely equivalent floating inverse-dynamics solver.

> Timing is host- and run-specific. The G1 r54 admission is retained and checksum-addressed; PlaCo, Pinocchio, and Upkie were freshly rerun together at 2026-08-01T10:00:09Z on the recorded host. Same-boundary ratios are publishable, but the report still makes no cross-boundary speedup claim.

## What is actually comparable

| Reference | Shared boundary | Strongest claim | Must not be inferred |
|---|---|---|---|
| Pinocchio 4.0 | same URDF states and convention adapters | FK/Jacobian/CoM/mass/bias/RNEA/centroidal products agree at roundoff | controller or task-policy parity |
| Upkie C++ WheelBalancer | same sequential inputs, default gains, 5 ms update order | 0/100,000 canonical command mismatches | whole-body WBC parity; adapter costs differ |
| PlaCo 0.9.23 | same toy model, targets, frames, initial state, and 50 Hz corpus | independent task tracking/latency/resource behavior | identical QP semantics; PlaCo is soft weighted and position-only |
| G1 r54 oracle | same immutable projected state/jet at every 5 ms tick | floating WBC feasibility and task admission without policy or physics | closed-loop stability or disturbance recovery |

## Current G1 CPU WBC

Boundary: policy=False, physics=False, integration=False, oracle-state-per-tick=True. This isolates WBC admission from a learned policy, simulator, or rollout drift.

| Profile | gates | solved / slack / fail | p50 / p99 / max µs | >5 / >20 ms | alloc calls / bytes |
|---|---|---|---|---|---|
| G1 authored transmission | 43/43 PASS | 870 / 1447 / 0 | 3510.5 / 5876.3 / 7831.8 | 76 / 0 | 0 / 0 |
| synthetic ankle differential | 45/45 PASS | 757 / 1560 / 0 | 3498.3 / 5845.2 / 8121.5 | — / 0 | 0 / 0 |

### CPU, memory, jitter, and deadlines

| Metric | Value |
|---|---|
| wall / process CPU | 7.724879 s / 7.723322 s |
| CPU / wall | 0.999799 |
| throughput | 299.9 WBC ticks/s |
| RSS before / after / growth | 45.39 / 46.89 / 1.50 MiB |
| Python tracemalloc peak | 1306 B (native Rust excluded) |
| hot-loop allocation sentinel | 0 calls / 0 bytes |
| latency mean / std / MAD | 3333.8 / 829.1 / 424.5 µs |
| latency p50 / p95 / p99 / p99.9 / max | 3510.5 / 4598.6 / 5876.3 / 7524.0 / 7831.8 µs |
| absolute inter-tick jitter p50 / p95 / p99 / max | 95.8 / 1452.2 / 2120.2 / 5197.3 µs |
| deadline misses >1 / >2 / >5 / >10 / >20 ms | 2317 / 2317 / 76 / 0 / 0 |

### Tracking and physical feasibility

| Signal | time RMS | p50 | p95 | p99 | max |
|---|---|---|---|---|---|
| point position m | 0.000085 | 0.000012 | 0.000021 | 0.000023 | 0.004042 |
| orientation rad | 0.000264 | 0.000001 | 0.000002 | 0.000002 | 0.012707 |
| center of mass m | 0.021825 | 0.023575 | 0.027160 | 0.029071 | 0.029205 |

These are morphology-projection pose errors. WBC task residuals below are physical acceleration residuals against the same generalized-acceleration witness; they are not silently substituted for pose error.

| Hard/resource quantity | Observed worst case |
|---|---|
| dynamics residual | 1.712e-09 |
| contact acceleration residual | 6.656e-11 |
| minimum friction margin | -8.165e-12 |
| minimum eroded support margin | 5.000 mm |
| maximum actuator utilization | 53.128% |
| minimum joint margin | 21.593° |
| bitwise repeat | PASS for every compared physical output |

### Task/nullspace residual stack

| Task | units | clipped ticks | time RMS | p99 | max | integral |
|---|---|---|---|---|---|---|
| actuator_torque_style | task-specific acceleration units | 1267 | 6.800122 | 8.470223 | 9.761569 | 76.852063 |
| center_of_mass | m/s^2 | 1211 | 0.155056 | 0.581311 | 0.811208 | 0.769045 |
| centroidal_angular_momentum_rate | task-specific acceleration units | 1211 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| contact_force_style | task-specific acceleration units | 1267 | 21.099750 | 28.517871 | 34.050969 | 237.739733 |
| frame_angular_0 | rad/s^2 | 0 | 0.000145 | 0.000600 | 0.002052 | 0.000728 |
| joint_posture | task-specific acceleration units | 1394 | 6.129988 | 15.809632 | 52.153629 | 50.242801 |
| point_0 | m/s^2 | 1211 | 0.000194 | 0.000901 | 0.001421 | 0.000744 |
| point_1 | m/s^2 | 1211 | 0.000142 | 0.000612 | 0.001144 | 0.000577 |
| point_2 | m/s^2 | 0 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| point_3 | m/s^2 | 0 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| protected_joint_acceleration | task-specific acceleration units | 1211 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| root_angular | rad/s^2 | 0 | 0.000112 | 0.000425 | 0.001849 | 0.000587 |
| root_height | m/s^2 | 0 | 0.000086 | 0.000357 | 0.000775 | 0.000424 |
| root_horizontal | m/s^2 | 1211 | 0.152221 | 0.524801 | 0.697476 | 0.782358 |

A clipped tick means a lower-priority objective was relaxed to preserve higher authority. It is not a hidden solver failure: 1,447 authored-transmission ticks report typed slack, while Invariant root attitude/height, dynamics, contacts, limits, and all declared tracking thresholds remain admitted.

### Solver work and latency attribution

| Work counter | mean | p95 | p99 | max | correlation with latency |
|---|---|---|---|---|---|
| pseudoinverse calls | 7.25 | 16.00 | 16.00 | 21.00 | 0.4864 |
| jacobi sweeps | 52.61 | 82.00 | 108.00 | 152.00 | 0.6757 |
| halfspace projections | 235.67 | 242.00 | 242.00 | 242.00 | 0.7076 |
| clipped steps | 4.58 | 14.00 | 15.00 | 18.00 | 0.5135 |

### Execution over time (1 s windows)

| ticks | seconds | p50 / p99 / max µs | point / CoM RMS mm | dyn / contact max | effort max | slack | pinv / sweeps mean |
|---|---|---|---|---|---|---|---|
| 0–199 | 0.0–1.0 | 3627 / 5444 / 5740 | 0.286 / 14.333 | 1.5e-09 / 5.4e-11 | 36.1% | 98 | 6.55 / 47.62 |
| 200–399 | 1.0–2.0 | 3530 / 7576 / 7832 | 0.017 / 24.725 | 1.2e-09 / 5.4e-11 | 42.6% | 140 | 5.62 / 50.26 |
| 400–599 | 2.0–3.0 | 3065 / 6093 / 6226 | 0.021 / 27.049 | 1.1e-09 / 1.5e-11 | 43.6% | 113 | 5.41 / 50.77 |
| 600–799 | 3.0–4.0 | 3738 / 5163 / 7239 | 0.014 / 17.180 | 1.2e-09 / 5.5e-11 | 53.1% | 173 | 10.61 / 64.61 |
| 800–999 | 4.0–5.0 | 2372 / 3878 / 4383 | 0.011 / 20.740 | 1.5e-09 / 6.7e-11 | 37.9% | 79 | 7.07 / 47.53 |
| 1000–1199 | 5.0–6.0 | 2352 / 5688 / 5758 | 0.015 / 22.053 | 1.1e-09 / 4.2e-11 | 38.4% | 92 | 5.70 / 44.96 |
| 1200–1399 | 6.0–7.0 | 3671 / 5777 / 7033 | 0.010 / 14.457 | 1.6e-09 / 6.1e-11 | 47.0% | 171 | 10.56 / 63.45 |
| 1400–1599 | 7.0–8.0 | 3109 / 5395 / 6852 | 0.013 / 25.550 | 1.7e-09 / 4.1e-11 | 34.0% | 114 | 7.20 / 57.37 |
| 1600–1799 | 8.0–9.0 | 3588 / 5548 / 6129 | 0.013 / 24.503 | 1.3e-09 / 4.4e-11 | 34.0% | 151 | 6.46 / 54.23 |
| 1800–1999 | 9.0–10.0 | 3603 / 4989 / 6357 | 0.009 / 16.764 | 1.3e-09 / 4.6e-11 | 51.4% | 173 | 10.07 / 61.47 |
| 2000–2199 | 10.0–11.0 | 2351 / 3922 / 4923 | 0.013 / 26.145 | 1.1e-09 / 3.5e-11 | 29.3% | 82 | 5.66 / 43.27 |
| 2200–2316 | 11.0–11.6 | 2814 / 3680 / 3808 | 0.010 / 23.815 | 8.4e-10 / 3.1e-11 | 28.5% | 61 | 5.39 / 41.02 |

## PlaCo fixed-base task comparison (fresh same-session r89)

| scenario | impl | RMS / steady cm | IAE / ISE | p50 / p99 / max µs | jitter p99 µs | steps/s | peak / Δ RSS MiB | GC |
|---|---|---|---|---|---|---|---|---|
| end_effector_reach | Bonesaw | 0.630 / 0.129 | 0.3042 / 0.0079 | 35.5 / 40.5 / 57.2 | 5.5 | 27640 | 43.82 / 1.37 | 0 |
| end_effector_reach | PlaCo | 0.809 / 0.737 | 1.4943 / 0.0131 | 97.5 / 108.1 / 242.2 | 12.2 | 9547 | 95.94 / 0.26 | 0 |
| bimanual_priority_conflict | Bonesaw | 0.475 / 0.000 | 0.0431 / 0.0045 | 71.9 / 83.4 / 156.9 | 11.2 | 13717 | 47.94 / 1.37 | 0 |
| bimanual_priority_conflict | PlaCo | 0.365 / 0.252 | 0.5146 / 0.0027 | 108.6 / 118.4 / 320.9 | 11.6 | 8628 | 99.71 / 0.26 | 0 |
| walking_motion_retarget | Bonesaw | 4.048 / 4.062 | 11.0005 / 0.6555 | 55.6 / 67.1 / 83.8 | 9.5 | 17802 | 50.36 / 1.37 | 0 |
| walking_motion_retarget | PlaCo | 2.634 / 2.645 | 5.2549 / 0.2776 | 131.1 / 143.0 / 271.1 | 12.8 | 7226 | 102.34 / 0.50 | 0 |

PlaCo tracks the walking endpoints better on this fixed-pelvis toy corpus; Bonesaw is faster and reaches lower steady error on the reach/conflict cases. Both walking gates in the shared run fail a predeclared sub-gate (Bonesaw foot RMS; PlaCo clearance RMS). The current G1 result above is a different, floating inverse-dynamics admission boundary and is not used to erase that historical failure.

## Pinocchio rigid-body product oracle (fresh same-session r89)

| model | states / frame samples | frame position | Jacobian | CoM | mass | bias | inverse dynamics | centroidal map | gate |
|---|---|---|---|---|---|---|---|---|---|
| upkie | 50 / 2050 | 2.22e-16 | 8.98e-16 | 1.11e-16 | 8.88e-16 | 2.13e-14 | 2.13e-14 | 8.88e-16 | PASS |
| g1_23dof_mode_10 | 50 / 1500 | 3.99e-16 | 9.99e-16 | 6.79e-17 | 7.11e-15 | 1.71e-13 | 1.71e-13 | 7.11e-15 | PASS |

## Upkie upstream controller oracle (fresh same-session r89)

| quantity | official Upkie C++ | Bonesaw Rust, official gains | interpretation |
|---|---|---|---|
| command mismatches | reference | 0 | canonical bitwise PASS |
| max / RMS command error | reference | 0.000e+00 / 0.000e+00 | exact over 100,000 sequential samples |
| p50 / p99 / max latency µs | 1.393 / 2.385 / 17.563 | 0.030 / 0.031 / 5.029 | C++ includes Dictionary I/O; Rust is typed call |
| jitter p99 µs | 0.250 | 0.011 | boundary cost differs |
| peak RSS MiB | 8.64 | 8.68 | isolated workers |
| hot-loop allocations | not instrumented | 0 calls / 0 bytes | native sentinel |

Pinned upstream commit: `1abdf373bbe8ee7d9f081293024f961ae9a8e2c1`. The live-tuned Bonesaw profile is intentionally different (RMS command delta 0.1177 m/s) and is not counted as parity.

## Remaining risks and next gates

- This is an oracle-state WBC admission test, not a policy, simulation, or hardware stability claim.
- Real G1 coupled transmissions are not authored; the 45/45 differential is a synthetic semantics fixture.
- Thermal/reliability authority remains unavailable until calibrated actuator parameters and telemetry exist.
- Fresh same-session PlaCo timing supports within-boundary latency, jitter, CPU, and memory ratios; its position-only weighted QP still does not define a universal WBC speedup.
- The next CPU gate should add deterministic closed-loop plant surrogates separately from this policy/physics-free certificate, then hardware-in-the-loop timing and disturbance recovery.
- CUDA batching remains deferred until these CPU semantics, artifacts, and per-row divergence checks are frozen.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-metrics.json | 0706030dc9d2f116d4b291efff7c3a041576fa4fba030f99150d7518c9229e18 |
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz | f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295 |
| benchmarks/results/g1-multistep-oracle-r54-coupled-actuation/oracle-wbc-admission-metrics.json | d02aee27af2445371b936725250af1fff08baa4efb14436944c0189aa8dd41ae |
| benchmarks/results/g1-multistep-oracle-r54-coupled-actuation/oracle-wbc-admission-raw.npz | 5c5d93a9b92e96de20585725624de2b203a3f6cd97bd0e177936f684404d73fa |
| benchmarks/results/reference-r89/reference-metrics.json | 8816ff8103915721fcabbd14938ab31938df899002b8e766355ad371de888cc5 |
| benchmarks/results/reference-r89/bonesaw-raw.npz | c5f008de98638b68aac3aad3e78eb8aff1ca96dd1c98770dc7b2aea26d270859 |
| benchmarks/results/reference-r89/placo-raw.npz | 513ad8f2535107ee7e4a8010cc57badc3374e431c7ef4e2e7cf179ffaca87b58 |
| benchmarks/results/reference-r89/upkie-controller-raw.npz | 8d73cb324fa48dd4b27dbc18a557114fba4aa04baaf987b841a1f050f9034e68 |

Machine-readable companions: `comparison-metrics.json`, `g1-execution-windows.csv`, `g1-task-residuals.csv`, and `fixed-base-comparison.csv`. Raw per-tick NPZ files remain in their checksum-addressed source directories to avoid lossy duplication.
