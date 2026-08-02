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
- Joint-velocity envelope: `viability` priority with weight `0.300`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 453 | 2.265 s | 8.950 cm | 10.612 cm | 31.955 cm | 30.917 cm | 48.066° | 8.000 rad/s | 92723.4 µs |

Nominal hard residual maxima: dynamics `1.552e-09`, contact acceleration `5.501e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 65.304 cm |
| authored reference vs measured CoM RMS / p95 | 60.608 / 162.023 cm |
| stance foot RMS | 49.883 cm |
| swing foot RMS | 43.794 cm |
| hand RMS | 86.135 cm |
| maximum root rotation | 163.382° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 5.641e-09 |
| contact acceleration residual | 1.215e-10 |
| raw max dynamics residual, including rejected ticks | 5.641e-09 |
| raw max contact residual, including rejected ticks | 1.215e-10 |
| active normal force range | 0.000–944.875 N |
| centroidal momentum-rate residual RMS / max | 61.056 / 400.538 N·m |
| point-task acceleration RMS max | 111.342 m/s² |
| frame-angular acceleration RMS max | 321.235 rad/s² |
| longest pre-contact / touchdown transition | 225 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `24.207` / `54.914 cm`.
- Virtual ZMP clipped on `53.67%` of ticks; clip-distance RMS / max `53.497` / `118.482 cm`.
- Measured-height natural frequency min / p50 / max: `3.700` / `3.770` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.881 m`; height-floor ticks: `115`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-84.584` / `-80.841 cm`; inside on `47.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8247 m`.
- Authored-offset / reach / slew limited ticks: `186 / 0 / 60`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.3867 m / 1.2624 / 0.0094 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 4`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `433` ticks; maximum active coordinates `8`; mean target/applied scale `0.637` / `0.723`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2589.8 µs | 38027.0 µs | 104164.5 µs | 177120.6 µs | 130 | 98 | 225 | 0 | 141 | 6 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7405.0 | 20459.4 | 277.6 | 4132.3 | 169363.6 | 176344.9 | 12969.1 | 600 | 49 | 34 | 135.0 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 130 | 2535.1 | 2826.3 | 3021.8 | 3898.3 |
| solved_with_slack | 98 | 2591.8 | 2861.9 | 3886.0 | 4980.5 |
| normal_contact_contingency | 141 | 2805.3 | 15554.1 | 44017.8 | 116282.1 |
| contact_release_contingency | 6 | 103439.7 | 173883.1 | 176473.1 | 177120.6 |
| precontact_transition | 225 | 2660.0 | 80013.5 | 105354.1 | 115539.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.65 | 11.0 | 13.0 | 16 | 5.36 | 12.0 | 15 | 0.1582 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 204.44/120.7/5297.1/6942 | 45251.10/26082.0/1175954.0/1541124 | 0.54/8.0/13 | 0.38/7.0/12 | 3.15/64.1/99 | 0.7525 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 130 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 98 | 6.98/13.0/14 | 3.92/10.0/11 |
| normal_contact_contingency | 141 | 8.88/12.6/15 | 7.65/12.6/14 |
| contact_release_contingency | 6 | 7.33/8.0/8 | 6.17/7.0/7 |
| precontact_transition | 225 | 8.72/13.0/16 | 7.64/12.0/15 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.01/6.0/6 | 8.70/25.0/30 | 1.51/6.0/6 | 316 |
| viability | 1.76/6.0/7 | 11.27/42.0/54 | 1.39/6.0/7 | 385 |
| intent | 1.54/5.0/10 | 8.26/26.0/49 | 1.31/5.0/10 | 470 |
| preference | 1.30/6.0/8 | 10.94/48.1/72 | 0.80/5.0/8 | 338 |
| style | 1.05/3.0/4 | 8.85/20.0/28 | 0.35/2.0/4 | 193 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 453 | 147 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `150` ticks.
Precontact sole-center tangential speed: p50 `2.2944 m/s`, p95 `6.3461 m/s`, max `7.3786 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 4.443 | 4.440 | 4.440 | 0.999 | 0.999 | 47.973 | 48.398 | 0.426 | 49.094 | 0.001 | 0 | 93 | 0 | 0 | 248 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2523.9 | 2865.0 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2582.6 | 3011.6 | 5.38 | 36.58 | 0.93 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.50e-11 | 0 |
| 120–179 | 2594.7 | 4342.0 | 6.30 | 42.32 | 2.27 | 1.00 | 234.00 | 1.285 | 0.000 | 1.16e-09 | 4.78e-11 | 0 |
| 180–239 | 2522.8 | 4255.4 | 7.35 | 46.20 | 4.70 | 1.00 | 225.80 | 6.326 | 1.252 | 1.00e-09 | 3.29e-11 | 12 |
| 240–299 | 2813.4 | 4105.7 | 9.07 | 56.38 | 7.67 | 2.52 | 558.70 | 6.110 | 22.970 | 1.22e-09 | 1.42e-11 | 60 |
| 300–359 | 2595.4 | 4517.4 | 8.60 | 51.70 | 7.65 | 3.10 | 688.20 | 7.931 | 24.848 | 1.55e-09 | 2.23e-11 | 60 |
| 360–419 | 2283.5 | 4126.0 | 8.25 | 48.70 | 7.30 | 1.58 | 351.50 | 11.506 | 29.074 | 8.49e-10 | 1.17e-11 | 60 |
| 420–479 | 6082.2 | 115843.9 | 9.53 | 58.93 | 8.43 | 2009.80 | 444920.20 | 35.691 | 46.022 | 5.61e-10 | 5.63e-12 | 60 |
| 480–539 | 2332.2 | 169480.2 | 8.67 | 53.73 | 7.15 | 16.97 | 3664.20 | 109.568 | 72.846 | 5.64e-09 | 5.23e-11 | 60 |
| 540–599 | 2885.8 | 10004.7 | 8.38 | 52.10 | 7.53 | 6.48 | 1400.40 | 170.567 | 116.533 | 4.86e-09 | 1.21e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 65.304 | 47.954 | 86.135 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
