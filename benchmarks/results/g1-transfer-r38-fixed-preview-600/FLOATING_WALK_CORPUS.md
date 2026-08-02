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
| 416 | 2.080 s | 14.209 cm | 0.963 cm | 20.005 cm | 31.411 cm | 53.746° | 8.000 rad/s | 16517.1 µs |

Nominal hard residual maxima: dynamics `7.727e-09`, contact acceleration `3.152e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 60.843 cm |
| authored reference vs measured CoM RMS / p95 | 53.964 / 128.005 cm |
| stance foot RMS | 36.054 cm |
| swing foot RMS | 56.772 cm |
| hand RMS | 82.000 cm |
| maximum root rotation | 175.533° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.040e-09 |
| contact acceleration residual | 3.525e-10 |
| raw max dynamics residual, including rejected ticks | 9.040e-09 |
| raw max contact residual, including rejected ticks | 3.525e-10 |
| active normal force range | 0.000–586.466 N |
| centroidal momentum-rate residual RMS / max | 65.776 / 318.219 N·m |
| point-task acceleration RMS max | 144.443 m/s² |
| frame-angular acceleration RMS max | 356.340 rad/s² |
| longest pre-contact / touchdown transition | 188 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `24.596` / `49.123 cm`.
- Virtual ZMP clipped on `66.83%` of ticks; clip-distance RMS / max `49.225` / `113.127 cm`.
- Measured-height natural frequency min / p50 / max: `3.691` / `3.786` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.625 m`; height-floor ticks: `150`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-75.093` / `-71.251 cm`; inside on `33.17%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8796 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 46`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.3639 m / 0.2905 / 0.0760 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 4`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `316` ticks; maximum active coordinates `10`; mean target/applied scale `0.629` / `0.720`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2555.2 µs | 102545.3 µs | 209139.9 µs | 236233.2 µs | 159 | 69 | 188 | 0 | 148 | 36 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15555.4 | 40856.6 | 569.5 | 18035.8 | 228420.7 | 235452.0 | 97993.3 | 600 | 119 | 59 | 64.3 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 159 | 2481.4 | 3681.6 | 3944.5 | 3982.5 |
| solved_with_slack | 69 | 2514.6 | 3807.7 | 4571.0 | 4814.7 |
| normal_contact_contingency | 148 | 4502.2 | 91972.9 | 97147.5 | 190016.5 |
| contact_release_contingency | 36 | 120988.4 | 221589.8 | 231668.3 | 236233.2 |
| precontact_transition | 188 | 2208.7 | 10621.4 | 53175.7 | 67759.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.67 | 12.0 | 13.0 | 15 | 5.21 | 13.0 | 13 | 0.1031 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 187.45/8.0/6120.2/6369 | 40607.91/1776.0/1321958.9/1375704 | 1.47/15.0/23 | 1.22/14.0/22 | 10.04/121.0/193 | 0.3150 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 159 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 69 | 7.52/13.0/13 | 4.38/10.0/10 |
| normal_contact_contingency | 148 | 9.10/13.0/15 | 8.34/13.0/13 |
| contact_release_contingency | 36 | 7.78/10.6/11 | 6.50/9.6/10 |
| precontact_transition | 188 | 8.84/13.0/13 | 7.20/13.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.98/6.0/8 | 8.46/28.0/35 | 1.47/6.0/8 | 291 |
| viability | 1.67/7.0/9 | 9.25/42.0/48 | 1.20/7.0/9 | 346 |
| intent | 1.63/6.0/8 | 8.06/30.0/42 | 1.35/6.0/8 | 439 |
| preference | 1.27/6.0/8 | 12.07/57.0/89 | 0.77/5.0/7 | 310 |
| style | 1.13/4.0/6 | 9.31/28.1/53 | 0.42/4.0/6 | 193 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 416 | 184 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `187` ticks.
Precontact sole-center tangential speed: p50 `1.8554 m/s`, p95 `6.6730 m/s`, max `9.2807 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9.334 | 9.331 | 9.331 | 1.000 | 1.000 | 48.262 | 48.516 | 0.254 | 49.277 | 0.001 | 0 | 44 | 0 | 0 | 104 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2482.1 | 3949.6 | 5.00 | 32.48 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.86e-11 | 0 |
| 60–119 | 2500.7 | 3752.9 | 5.00 | 31.78 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 120–179 | 2524.4 | 4502.0 | 6.50 | 36.30 | 2.37 | 1.00 | 234.00 | 0.227 | 0.000 | 1.25e-09 | 5.52e-11 | 0 |
| 180–239 | 2025.4 | 3896.3 | 7.03 | 40.20 | 3.58 | 1.00 | 225.80 | 2.411 | 0.072 | 1.94e-09 | 6.47e-11 | 12 |
| 240–299 | 2046.2 | 4803.3 | 8.38 | 50.90 | 5.52 | 1.00 | 222.00 | 4.008 | 2.983 | 8.30e-10 | 1.71e-11 | 60 |
| 300–359 | 2383.6 | 4516.7 | 9.13 | 54.95 | 8.23 | 1.70 | 377.40 | 12.193 | 11.832 | 1.26e-09 | 1.33e-11 | 60 |
| 360–419 | 2183.0 | 117884.9 | 9.07 | 56.82 | 8.35 | 186.42 | 41383.40 | 37.559 | 25.329 | 7.73e-09 | 3.15e-10 | 60 |
| 420–479 | 15670.2 | 228538.1 | 8.80 | 54.18 | 7.90 | 100.78 | 21753.00 | 84.517 | 52.507 | 9.04e-09 | 3.52e-10 | 60 |
| 480–539 | 6814.7 | 104333.5 | 9.08 | 58.83 | 8.22 | 308.37 | 66605.90 | 98.172 | 82.905 | 7.11e-10 | 1.43e-11 | 60 |
| 540–599 | 3961.2 | 96920.0 | 8.72 | 55.15 | 7.92 | 1272.27 | 274809.60 | 136.589 | 94.556 | 4.72e-09 | 2.36e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 60.843 | 44.002 | 82.000 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
