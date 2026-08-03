# Upkie floating reference comparison · R317

## Result

This is the first isolated full-body Upkie comparison against PlaCo's floating
inverse-dynamics implementation. Both workers consume the same 256 frozen
states and acceleration targets; policy, integration, simulator, and physics
rollout are all absent. Bonesaw retains its Rust nonholonomic rolling-contact
model. PlaCo uses its independently implemented unilateral material-point
contacts at the two wheel centers.

**Reference availability: PASS. Exact controller parity: NOT CLAIMED.** The
contact laws are intentionally visible and materially different, so output
divergence is evidence rather than a failure hidden by tuning.

## CPU, memory, jitter, and success

| implementation | successful states | solve p50 / p99 / max µs | jitter p99 µs | full query mean µs | queries/s | peak RSS MiB | GC |
|---|---|---|---|---|---|---|---|
| Bonesaw | 256/256 | 118.8 / 131.2 / 147.4 | 15.0 | 119.3 | 8384 | 88.71 | 0 |
| Placo | 256/256 | 54.7 / 65.2 / 103.2 | 11.4 | 125.6 | 7962 | 406.19 | 0 |

Bonesaw's measured solve loop reports
`0` Rust allocation
calls and `0` bytes.
Timing and RSS are isolated-process observations on this host; they are not a
universal speedup claim. Bonesaw's full-query mean amortizes one fixed-shape
PyO3 batch boundary over 256 rows; PlaCo's includes Python state/task updates
for every row. Solve-only and full-query columns are therefore both retained.

| implementation | wall s | CPU s | CPU/wall | RSS before MiB | RSS after MiB | RSS delta MiB |
|---|---|---|---|---|---|---|
| Bonesaw | 0.244287 | 0.244262 | 0.999899 | 88.92 | 88.92 | 0.0000 |
| Placo | 0.257224 | 0.257103 | 0.999531 | 406.42 | 406.42 | 0.0000 |

## Shared-target tracking

| acceleration target | Bonesaw RMS | PlaCo RMS |
|---|---|---|
| root_angular_rad_s2 | 0.000126 | 0.000000 |
| root_horizontal_m_s2 | 0.886564 | 0.990486 |
| root_height_m_s2 | 0.000120 | 0.000000 |
| joint_rad_s2 | 141.783789 | 39.283979 |

## Output divergence

| quantity | RMS delta | maximum absolute delta |
|---|---|---|
| generalized_acceleration | 101.978721 | 272.061369 |
| actuator_torque | 0.351065 | 0.692298 |
| contact_normal_force | 2.913330 | 5.721716 |

The normal-force comparison is the closest contact quantity shared by both
models. Tangential force is not declared equivalent: Bonesaw's force belongs
to a rolling basis, while PlaCo anchors a material wheel-center point.

## Execution over corpus order

| ticks | implementation | solve p50 / p99 µs | root XY RMS | joint RMS | max |τ| Nm | mean Fz N |
|---|---|---|---|---|---|---|
| 0–31 | Bonesaw | 121.0 / 143.8 | 1.9121 | 157.48 | 0.838 | 26.17 |
| 0–31 | Placo | 54.8 / 89.5 | 1.3505 | 26.90 | 1.158 | 23.08 |
| 32–63 | Bonesaw | 119.4 / 125.2 | 1.1746 | 147.05 | 1.191 | 25.82 |
| 32–63 | Placo | 55.1 / 63.0 | 0.6646 | 40.49 | 1.523 | 22.39 |
| 64–95 | Bonesaw | 118.7 / 125.8 | 0.3095 | 126.84 | 1.109 | 27.63 |
| 64–95 | Placo | 53.9 / 64.1 | 1.4714 | 32.96 | 1.594 | 25.46 |
| 96–127 | Bonesaw | 120.3 / 134.9 | 0.7231 | 158.53 | 0.984 | 28.24 |
| 96–127 | Placo | 53.6 / 59.7 | 0.9091 | 32.72 | 1.236 | 27.23 |
| 128–159 | Bonesaw | 117.1 / 122.6 | 0.1690 | 119.24 | 1.211 | 27.62 |
| 128–159 | Placo | 53.6 / 58.5 | 0.9740 | 58.29 | 0.729 | 30.97 |
| 160–191 | Bonesaw | 116.4 / 125.3 | 0.0041 | 126.24 | 1.270 | 25.22 |
| 160–191 | Placo | 53.5 / 59.5 | 1.1380 | 32.82 | 1.002 | 29.18 |
| 192–223 | Bonesaw | 117.5 / 128.2 | 0.0042 | 134.06 | 1.223 | 25.07 |
| 192–223 | Placo | 53.9 / 59.9 | 0.4541 | 34.27 | 1.489 | 24.79 |
| 224–255 | Bonesaw | 119.5 / 128.3 | 0.7779 | 158.27 | 1.152 | 25.14 |
| 224–255 | Placo | 55.0 / 79.8 | 0.3763 | 46.66 | 1.466 | 23.20 |

Each row is a lossless 32-state partition of the frozen corpus. The window
table keeps phase-local latency, tracking, torque, and normal-load changes
visible instead of reducing the run to one aggregate score.

## Boundary and next gate

- Corpus: `benchmarks/results/upkie-state-local-wbc-r123/upkie-state-local-wbc-raw.npz`; 256 independent rows.
- Bonesaw: exact rolling rows, hierarchical task stack, preallocated Rust hot loop.
- PlaCo: independent weighted QP, floating dynamics, unilateral frictional point contacts.
- No learned policy, physics engine, integration, state propagation, or reset behavior is in this comparison.
- The next parity gate is an independent solver with the same nonholonomic
  wheel row and contact-force basis, followed separately by a shared closed-loop
  plant corpus. Hardware timing and thermal/reliability authority remain open.

## Provenance

| artifact | version or SHA-256 |
|---|---|
| Bonesaw | 0.1.0 |
| PlaCo | 0.9.23 |
| model | d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483 |
| input corpus | f31c69dd9b1f8bab7f58a8870b53e390b45bae848c348668009440e24c1e1e3e |
| evaluator | d638209f10b7002b3808fa946b489c9b2a2dfa48a7d0d2750a18b0969ab121c7 |
| Bonesaw raw | 3574846fab8ac26935cd722aa497b89bd8d4f259841d9cb14db35fa2e89232d5 |
| PlaCo raw | 7a3cdf0f7ccc2f61ffa7caa8f0d234f26d2cc95ab0462eabce5ba859311e07d8 |
