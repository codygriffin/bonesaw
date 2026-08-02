# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 460 | 2.300 s | 13.763 cm | 0.687 cm | 37.686 cm | 37.011 cm | 55.180° | 8.000 rad/s | 41758.4 µs |

Nominal hard residual maxima: dynamics `4.241e-09`, contact acceleration `1.813e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 43.956 cm |
| authored reference vs measured CoM RMS / p95 | 38.013 / 85.387 cm |
| stance foot RMS | 17.276 cm |
| swing foot RMS | 53.269 cm |
| hand RMS | 68.872 cm |
| maximum root rotation | 177.974° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.571e-09 |
| contact acceleration residual | 2.709e-10 |
| raw max dynamics residual, including rejected ticks | 6.571e-09 |
| raw max contact residual, including rejected ticks | 2.709e-10 |
| active normal force range | 0.000–874.335 N |
| centroidal momentum-rate residual RMS / max | 66.151 / 412.913 N·m |
| point-task acceleration RMS max | 117.509 m/s² |
| frame-angular acceleration RMS max | 186.177 rad/s² |
| longest pre-contact / touchdown transition | 192 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 132 / 132 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `19.601` / `44.345 cm`.
- Virtual ZMP clipped on `54.17%` of ticks; clip-distance RMS / max `34.067` / `96.414 cm`.
- Measured-height natural frequency min / p50 / max: `3.643` / `3.758` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.222 m`; height-floor ticks: `123`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-63.078` / `-57.773 cm`; inside on `45.50%` of ticks.

## Capture-aware landing

- Active target-ticks: `332`; policy updates `200`, frozen `132`.
- Maximum applied offset / root reach: `0.0800 / 0.8394 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 88`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `132 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0129 m / 1.6528 / 0.0094 m/s`.
- Individually viable position / tangential / normal target-ticks: `1 / 0 / 3`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `331`, multi-support `239`.
- Joint-velocity envelope active on `448` ticks; maximum active coordinates `9`; mean target/applied scale `0.683` / `0.767`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2663.4 µs | 107244.4 µs | 207169.7 µs | 224115.2 µs | 129 | 139 | 192 | 0 | 108 | 32 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 13967.0 | 39988.9 | 442.0 | 14867.5 | 223950.4 | 224098.8 | 155629.6 | 600 | 128 | 42 | 71.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2553.7 | 2680.6 | 2739.7 | 2765.1 |
| solved_with_slack | 139 | 2722.5 | 4444.7 | 5226.5 | 5435.7 |
| normal_contact_contingency | 108 | 6926.2 | 18124.1 | 86106.9 | 204329.9 |
| contact_release_contingency | 32 | 180684.9 | 223743.6 | 224029.9 | 224115.2 |
| precontact_transition | 192 | 2490.8 | 10847.7 | 51395.3 | 55275.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.03 | 13.0 | 14.0 | 22 | 5.77 | 13.0 | 18 | 0.0321 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 54.23/8.0/2694.6/5446 | 11896.61/1776.0/598207.9/1176336 | 1.85/15.0/18 | 1.60/14.0/17 | 13.39/126.0/145 | 0.1402 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 139 | 8.08/14.6/22 | 5.29/12.0/18 |
| normal_contact_contingency | 108 | 10.15/14.9/15 | 9.45/14.0/15 |
| contact_release_contingency | 32 | 7.62/10.0/10 | 5.84/8.7/9 |
| precontact_transition | 192 | 8.89/13.0/13 | 7.90/12.1/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.05/6.0/10 | 9.10/30.0/41 | 1.51/6.0/10 | 286 |
| viability | 1.88/9.0/10 | 11.06/43.0/60 | 1.44/8.0/9 | 362 |
| intent | 1.55/5.0/6 | 8.27/28.0/36 | 1.30/5.0/6 | 455 |
| preference | 1.42/6.0/12 | 12.71/60.0/120 | 1.05/6.0/11 | 399 |
| style | 1.12/4.0/5 | 9.53/26.0/36 | 0.47/3.0/5 | 234 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 332 | 0 | 239 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 460 | 140 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `332` ticks, planned normal touchdown `0` ticks, normal fallback `143` ticks.
Precontact sole-center tangential speed: p50 `3.2931 m/s`, p95 `7.2546 m/s`, max `8.2136 m/s` over 332 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.380 | 8.367 | 8.367 | 0.998 | 0.998 | 48.102 | 48.531 | 0.430 | 49.227 | 0.001 | 0 | 93 | 0 | 0 | 1,096 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2543.0 | 2695.9 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2631.1 | 2834.5 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2607.6 | 3277.9 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 3286.2 | 5389.0 | 9.83 | 59.58 | 7.22 | 1.00 | 233.80 | 6.433 | 0.001 | 9.40e-10 | 7.60e-11 | 0 |
| 240–299 | 2090.8 | 3008.1 | 7.60 | 47.47 | 5.75 | 1.00 | 222.00 | 6.431 | 10.536 | 2.57e-09 | 1.02e-10 | 32 |
| 300–359 | 2342.8 | 3724.3 | 8.37 | 52.87 | 7.02 | 1.00 | 222.00 | 8.187 | 37.278 | 1.18e-09 | 4.10e-11 | 60 |
| 360–419 | 2322.2 | 23060.5 | 8.92 | 55.72 | 8.10 | 51.17 | 11359.00 | 17.615 | 17.415 | 8.94e-10 | 1.48e-11 | 60 |
| 420–479 | 10027.9 | 134996.1 | 10.38 | 67.45 | 9.73 | 465.75 | 102082.90 | 46.883 | 35.904 | 4.24e-09 | 1.81e-10 | 60 |
| 480–539 | 5699.0 | 217855.3 | 8.52 | 50.90 | 7.45 | 7.30 | 1568.20 | 89.858 | 63.566 | 4.45e-09 | 2.71e-10 | 60 |
| 540–599 | 14190.5 | 223849.5 | 10.17 | 62.22 | 9.37 | 12.10 | 2576.20 | 92.665 | 58.352 | 6.57e-09 | 6.12e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 43.956 | 32.463 | 68.872 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
